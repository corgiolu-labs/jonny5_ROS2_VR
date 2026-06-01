/*
 * hal_spi_slave.c — HAL SPI Slave con DMA circular
 *
 * SPI1 in slave mode always-armed: il DMA circular è avviato una sola volta
 * e rimane attivo indefinitamente. Gli eventi half/full arrivano via callback
 * DMA RX e vengono processati in un thread dedicato (non in ISR).
 *
 * Double-buffer DMA (128 byte = 2 × 64 byte):
 *   spi_tx_buf: [frame0 TX | frame1 TX]  — half0 = indice 0, half1 = indice 1
 *   spi_rx_buf: [frame0 RX | frame1 RX]
 *
 * Quando half N è pronto in RX, il thread:
 *   1. Commit RX half N al boundary layer.
 *   2. Costruisce la risposta TX nel half opposto (1-N) già trasmesso.
 *
 * Perché NON usiamo i callback HAL SPI complete:
 *   HAL_SPI_TxRxCpltCallback/HalfCpltCallback vengono overridati con stub
 *   vuoti per evitare che la HAL disabiliti RXDMAEN/TXDMAEN, rompendo il
 *   circular mode.
 *
 * NOTE [Refactor-Phase1]:
 *   - Le funzioni sul critical path (ISR DMA, spi_service_thread_entry,
 *     process_spi_command_build_tx) NON devono essere modificate nei refactor.
 *   - Le funzioni marcate nei report come SUPERFLUA_LEGACY o TEST_ONLY
 *     (diagnostica NSS, thread GPIO di debug, getter legacy) sono mantenute
 *     per compatibilità e potranno essere raggruppate in blocchi commentati
 *     "LEGACY" senza cambiare il comportamento runtime.
 */

#include "spi/hal_spi_slave.h"
#include "spi/boundary_buffers.h"
#include "spi/j5_protocol.h"
#include "spi/assist_v2_raw.h"

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_dma.h>
#include <string.h>

/* =========================================================
 * Costanti
 * ========================================================= */

#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE) || IS_ENABLED(CONFIG_J5_CANONICAL_PADDED_128_MODE)
#define SPI_FRAME_LEN          J5_ASSIST_V2_FRAME_SIZE /* 128 — WIRE REGISTER v1 */
#else
#define SPI_FRAME_LEN          J5_PROTOCOL_FRAME_SIZE  /* 64 */
#endif
#define SPI_FRAMES_IN_CIRCULAR 2
#define SPI_BUF_LEN            (SPI_FRAME_LEN * SPI_FRAMES_IN_CIRCULAR)

#define SPI_SERVICE_STACK_SIZE 1024
/* Priorita' complessiva: RT(5) > IMU(6) > SPI service(7).
 * Con la versione precedente (SPI service=6 > IMU=7) le wake-up del DMA SPI
 * preempravano il thread IMU durante il path I2C1 dell'IMU, producendo in campo
 * wedge ricorrenti del bus (SDA held low) con probabilita' dipendente dal rate
 * SPI. Ora IMU non e' piu' preempribile da spi_service; SPI service resta
 * preempribile dal loop RT (5), preservando la disciplina RT originaria. */
#define SPI_SERVICE_PRIO       7

/** Se 1 il DMA circular rimane sempre armato (no polling NSS). */
#define SPI_DMA_ALWAYS_ARMED   1

/** Watchdog: se nessun half-DMA arriva entro questo tempo (ms), forza restart DMA.
 *  Protegge da OVR/MODF silenti che fermano il DMA senza invocare la error callback. */
#define SPI_WATCHDOG_MS        3000

/**
 * Abilitare a 1 solo in debug: i printk per-frame rallentano il path
 * critico e possono aprire finestre in cui il master clocka mentre lo
 * slave è sotto carico → errori OVR sporadici.
 */
/* Per-frame diagnostic logging removed — all RT path printk converted to
 * LOG_DBG (compiled out at LOG_DEFAULT_LEVEL=0) or removed entirely. */

/* =========================================================
 * HAL handles e DMA buffers
 * ========================================================= */

static SPI_HandleTypeDef  hspi1;
static DMA_HandleTypeDef  hdma_spi1_rx;   /* DMA2 Stream0 Channel3 */
static DMA_HandleTypeDef  hdma_spi1_tx;   /* DMA2 Stream3 Channel3 */

/* Double-buffer DMA: 2 × SPI_FRAME_LEN (64 or 128) */
static uint8_t spi_tx_buf[SPI_BUF_LEN];
static uint8_t spi_rx_buf[SPI_BUF_LEN];

static bool spi_initialized = false;

/* Timestamp (ms) dell'ultimo frame SPI valido ricevuto dal Pi.
 * Aggiornato atomicamente a ogni boundary_hal_rx_commit().
 * Letto dal RT loop per il watchdog timeout. */
static atomic_t _last_frame_rx_ms = ATOMIC_INIT(0);

uint32_t hal_spi_last_frame_age_ms(void)
{
    uint32_t last = (uint32_t)atomic_get(&_last_frame_rx_ms);
    if (last == 0) { return UINT32_MAX; }
    return k_uptime_get_32() - last;
}

/* =========================================================
 * Engine thread + eventi ISR → thread
 * ========================================================= */

static struct k_thread spi_service_thread;
static K_THREAD_STACK_DEFINE(spi_service_stack, SPI_SERVICE_STACK_SIZE);
static struct k_sem  spi_evt_sem;
static atomic_t      pending_mask;    /* bit0=half0 pronto, bit1=half1 pronto */
static atomic_t      spi_error_flag;  /* 1 = errore DMA, da riavviare in thread context */

/* Ingest cross-boundary: ultimi (SPI_FRAME_LEN-1) byte dell'half precedente + half corrente.
 * Copre frame J5 fino a 128 B che iniziano in coda a un half DMA e continuano nel successivo.
 * spi_rx_window: tail_len + SPI_FRAME_LEN <= 2*SPI_FRAME_LEN - 1 (entra in SPI_FRAME_LEN*2). */
#define SPI_INGEST_TAIL_LEN ((size_t)(SPI_FRAME_LEN - 1U))
static uint8_t spi_ingest_tail[SPI_FRAME_LEN - 1U];
static bool    spi_ingest_tail_valid = false;
static uint8_t spi_rx_window[SPI_FRAME_LEN * 2U];

/* Anti re-parse: stesso frame J5VR/J5IK (header seq + tipo) visto due volte nella finestra sovrapposta. */
static uint16_t spi_last_j5vr_ik_seq = 0U;
static uint8_t  spi_last_j5vr_ik_ft = 0U;
static bool     spi_last_j5vr_ik_valid = false;

/* Watchdog: timestamp dell'ultimo half-DMA processato, per rilevare stop silenti del DMA. */
static int64_t spi_last_half_ms = 0;

/* =========================================================
 * Forward declarations
 * ========================================================= */

static void process_spi_command_build_tx(const uint8_t *rx, size_t len,
                                         uint8_t out_tx[SPI_FRAME_LEN]);
static bool spi_is_valid_j5_frame_at(const uint8_t *buf, size_t len, size_t off, uint8_t *ft, uint8_t *d6);
static const uint8_t *spi_pick_normalized_frame(const uint8_t *rx, size_t len, size_t *off_out);
static uint32_t spi_fp32(const uint8_t *p, size_t n);
static bool seq_is_newer_u16(uint16_t a, uint16_t b);
static void spi_dma_rx_half_cb(DMA_HandleTypeDef *hdma);
static void spi_dma_rx_full_cb(DMA_HandleTypeDef *hdma);
static void spi_dma_error_cb(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef spi_start_dma_circular(void);

/* =========================================================
 * DMA flag helpers
 * ========================================================= */

/**
 * dma_clear_spi1_flags — azzera i pending flag DMA per Stream0 (RX) e Stream3 (TX).
 * Chiamata sia in spi_start_dma_circular che in restart da errore.
 */
static inline void dma_clear_spi1_flags(void)
{
    /* Stream0 (RX): LIFCR bit[5:0] */
    DMA2->LIFCR  = (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5);
    /* Stream3 (TX): LIFCR bit[27:22] */
    DMA2->LIFCR |= (1u << 22) | (1u << 24) | (1u << 25) | (1u << 26) | (1u << 27);
}

/* =========================================================
 * Service thread
 * ========================================================= */

static void spi_service_thread_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    spi_last_half_ms = k_uptime_get();

    while (1)
    {
        int rc = k_sem_take(&spi_evt_sem, K_MSEC(SPI_WATCHDOG_MS));

        if (!spi_initialized) { continue; }

        /* Watchdog: se il semaforo è scaduto (rc=-EAGAIN) e non ci sono half pendenti,
         * il DMA si è fermato senza errore esplicito (OVR/MODF silente). Forza restart. */
        if (rc == -EAGAIN)
        {
            uint32_t mask_now = (uint32_t)atomic_get(&pending_mask);
            uint32_t elapsed  = (uint32_t)(k_uptime_get() - spi_last_half_ms);
            if (mask_now == 0U && elapsed >= (uint32_t)SPI_WATCHDOG_MS)
            {
                printk("[SPI] WATCHDOG: no half in %u ms (SR=0x%08x) forcing restart\n",
                       elapsed, (unsigned)SPI1->SR);
                (void)HAL_SPI_DMAStop(&hspi1);
                dma_clear_spi1_flags();
                spi_ingest_tail_valid = false;
                spi_last_j5vr_ik_valid = false;
                (void)spi_start_dma_circular();
                spi_last_half_ms = k_uptime_get();
            }
            continue;
        }

        /* Errore DMA latched: restart in thread context */
        if (atomic_cas(&spi_error_flag, 1, 0))
        {
            printk("[SPI] error restart: SR=0x%08x state=%u\n",
                   (unsigned)SPI1->SR, (unsigned)hspi1.State);
            (void)HAL_SPI_DMAStop(&hspi1);
            dma_clear_spi1_flags();
            spi_ingest_tail_valid = false;
            spi_last_j5vr_ik_valid = false;
            (void)spi_start_dma_circular();
            spi_last_half_ms = k_uptime_get();
        }

        /* Processa tutti gli half pendenti (priorità a half0) */
        while (1)
        {
            uint32_t mask = (uint32_t)atomic_get(&pending_mask);
            if (mask == 0U) { break; }

            const int      half    = (mask & 0x1U) ? 0 : 1;
            const uint32_t bit     = (half == 0) ? 0x1U : 0x2U;
            if (!atomic_cas(&pending_mask, (atomic_val_t)mask, (atomic_val_t)(mask & ~bit)))
            {
                continue;
            }

            const uint8_t *rx_ptr      = &spi_rx_buf[half * SPI_FRAME_LEN];
            uint8_t       *tx_ptr_next = &spi_tx_buf[(1 - half) * SPI_FRAME_LEN];

            /* Commit RX al boundary layer (NO copy in ISR) */
            uint8_t *rx_write_ptr = boundary_hal_rx_write_ptr();
            if (rx_write_ptr != NULL)
            {
                memcpy(rx_write_ptr, rx_ptr, SPI_FRAME_LEN);
                boundary_hal_rx_commit();
                atomic_set(&_last_frame_rx_ms, (atomic_val_t)k_uptime_get_32());
            }

            /* Finestra estesa tail + current_half per frame che attraversano il boundary DMA. */
            size_t window_len;
            if (spi_ingest_tail_valid) {
                memcpy(spi_rx_window, spi_ingest_tail, SPI_INGEST_TAIL_LEN);
                memcpy(spi_rx_window + SPI_INGEST_TAIL_LEN, rx_ptr, SPI_FRAME_LEN);
                window_len = SPI_INGEST_TAIL_LEN + SPI_FRAME_LEN;
            } else {
                memcpy(spi_rx_window, rx_ptr, SPI_FRAME_LEN);
                window_len = SPI_FRAME_LEN;
            }
            process_spi_command_build_tx(spi_rx_window, window_len, tx_ptr_next);
            memcpy(spi_ingest_tail, rx_ptr + (SPI_FRAME_LEN - SPI_INGEST_TAIL_LEN), SPI_INGEST_TAIL_LEN);
            spi_ingest_tail_valid = true;
            spi_last_half_ms = k_uptime_get();
        }
    }
}

/* =========================================================
 * DMA RX callbacks (ISR context — solo latch flag + semaforo)
 * ========================================================= */

static void spi_dma_rx_half_cb(DMA_HandleTypeDef *hdma)
{
    ARG_UNUSED(hdma);
    atomic_or(&pending_mask, 0x1);
    k_sem_give(&spi_evt_sem);
}

static void spi_dma_rx_full_cb(DMA_HandleTypeDef *hdma)
{
    ARG_UNUSED(hdma);
    atomic_or(&pending_mask, 0x2);
    k_sem_give(&spi_evt_sem);
}

static void spi_dma_error_cb(DMA_HandleTypeDef *hdma)
{
    ARG_UNUSED(hdma);
    /* Log in ISR con printk (Zephyr immediate mode, safe in ISR) */
    printk("[SPI] DMA error cb: SR=0x%08x EC=0x%08x state=%u\n",
           (unsigned)SPI1->SR,
           (unsigned)HAL_SPI_GetError(&hspi1),
           (unsigned)hspi1.State);
    atomic_set(&spi_error_flag, 1);
    k_sem_give(&spi_evt_sem);
}

/* =========================================================
 * DMA circular start / restart
 * ========================================================= */

/**
 * spi_clear_peripheral_errors — azzera i flag OVR/MODF nel registro SR di SPI1
 * e ri-abilita SPE se disabilitato da MODF.  Chiamare PRIMA di spi_start_dma_circular
 * nei percorsi di restart per evitare che un OVR residuo scateni immediatamente
 * una nuova error callback non appena il DMA riparte.
 *
 * Sequenza da RM0390 §28.4.8 (Overrun) e §28.4.9 (Mode fault):
 *   OVR:  1) leggi SPI_DR, 2) leggi SPI_SR.
 *   MODF: 1) leggi SPI_SR (già fatto per OVR), 2) scrivi CR1.
 */
static void spi_clear_peripheral_errors(void)
{
    /* Leggi SR per campionare OVR/MODF */
    volatile uint32_t sr = SPI1->SR;

    if (sr & SPI_SR_OVR)
    {
        /* Clear OVR: lettura DR poi SR */
        volatile uint8_t dummy = (volatile uint8_t)(SPI1->DR);
        (void)dummy;
        (void)(SPI1->SR);
        printk("[SPI] OVR cleared in restart\n");
    }

    if (sr & SPI_SR_MODF)
    {
        /* Clear MODF: SR già letto, ora scrivi CR1 (qualsiasi valore) */
        SPI1->CR1 = SPI1->CR1;
        printk("[SPI] MODF cleared in restart\n");
    }

    /* Assicura SPE=1: MODF lo azzera automaticamente */
    if (!(SPI1->CR1 & SPI_CR1_SPE))
    {
        SPI1->CR1 |= SPI_CR1_SPE;
        printk("[SPI] SPE re-enabled in restart\n");
    }
}

static HAL_StatusTypeDef spi_start_dma_circular(void)
{
    dma_clear_spi1_flags();
    spi_clear_peripheral_errors();

    /* Forza lo state HAL a READY per permettere la start dopo un errore */
    hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
    hspi1.State     = HAL_SPI_STATE_READY;

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_BUF_LEN);
    if (st != HAL_OK)
    {
        atomic_set(&spi_error_flag, 1);
        k_sem_give(&spi_evt_sem);
        return st;
    }

    /* Riduce IRQ load: eventi half/full solo dal RX stream; TX stream silenzioso */
    __HAL_DMA_DISABLE_IT(&hdma_spi1_tx, DMA_IT_HT | DMA_IT_TC);

    /* Override callback DMA RX: evita il "complete" interno della HAL SPI
     * che potrebbe disabilitare RXDMAEN/TXDMAEN in alcuni path. */
    hdma_spi1_rx.XferHalfCpltCallback = spi_dma_rx_half_cb;
    hdma_spi1_rx.XferCpltCallback     = spi_dma_rx_full_cb;
    hdma_spi1_rx.XferErrorCallback    = spi_dma_error_cb;
    hdma_spi1_tx.XferErrorCallback    = spi_dma_error_cb;

    return HAL_OK;
}

/* =========================================================
 * ISR stubs (Zephyr IRQ_CONNECT)
 * ========================================================= */

static void spi1_isr(const void *arg)
{
    ARG_UNUSED(arg);
    HAL_SPI_IRQHandler(&hspi1);
}

static void dma2_stream0_isr(const void *arg)
{
    ARG_UNUSED(arg);
    HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

static void dma2_stream3_isr(const void *arg)
{
    ARG_UNUSED(arg);
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/* =========================================================
 * GPIO init
 * ========================================================= */

static void hal_spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* PA4=NSS, PA5=SCK, PA6=MISO, PA7=MOSI — tutti AF5 (SPI1) */
    GPIO_InitStruct.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* =========================================================
 * SPI init
 * ========================================================= */

void hal_spi_slave_init(void)
{
    if (spi_initialized) { return; }

    hal_spi_gpio_init();

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_SLAVE;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_HARD_INPUT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; /* ignorato in slave */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        printk("[SPI] ERROR: HAL_SPI_Init failed\n");
        spi_initialized = false;
        return;
    }

    /* === NSS hardware deterministico ===
     * CR1: SSM=0, SSI=0, MSTR=0 (slave), SPE=1
     * CR2: NSSP=0
     * Forziamo MODE 0 (CPOL=0, CPHA=0) a livello registro per sicurezza.
     * HAL_SPI_Init() può non azzerare SSM/SSI in tutti i path. */
    SPI1->CR1 &= ~(1u << 9);  /* SSM  = 0 */
    SPI1->CR1 &= ~(1u << 8);  /* SSI  = 0 */
    SPI1->CR1 &= ~(1u << 2);  /* MSTR = 0 */
    SPI1->CR1 |=  (1u << 6);  /* SPE  = 1 */
    SPI1->CR1 &= ~(1u << 1);  /* CPOL = 0 */
    SPI1->CR1 &= ~(1u << 0);  /* CPHA = 0 */
    SPI1->CR2 &= ~(1u << 3);  /* NSSP = 0 */

    /* PA4 AF5 (SPI1_NSS) con pull-up */
    GPIOA->MODER  &= ~(3u << (4 * 2));
    GPIOA->MODER  |=  (2u << (4 * 2));          /* AF mode */
    GPIOA->AFR[0] &= ~(0xFu << (4 * 4));
    GPIOA->AFR[0] |=  (5u << (4 * 4));          /* AF5 = SPI1_NSS */
    GPIOA->PUPDR  &= ~(3u << (4 * 2));
    GPIOA->PUPDR  |=  (1u << (4 * 2));          /* Pull-up */

    printk("[SPI] NSS hardware forced: SSM=0, SSI=0, PA4=AF5\n");

    /* ISR SPI1 (IRQn=35) per errori */
    IRQ_CONNECT(35, 0, spi1_isr, NULL, 0);
    irq_enable(35);

    /* TX/RX iniziale: entrambi gli half con frame STATUS valido */
    memset(spi_tx_buf, 0x00, SPI_BUF_LEN);
    memset(spi_rx_buf, 0x00, SPI_BUF_LEN);
    for (int i = 0; i < SPI_FRAMES_IN_CIRCULAR; i++)
    {
        j5_frame_t init_frame;
        uint8_t *slot = &spi_tx_buf[i * SPI_FRAME_LEN];

        j5_build_frame(&init_frame, J5_FRAME_TYPE_STATUS, (uint16_t)i);
        memcpy(slot, &init_frame, sizeof(j5_frame_t));
#if SPI_FRAME_LEN > J5_PROTOCOL_FRAME_SIZE
        memset(slot + sizeof(j5_frame_t), 0,
               SPI_FRAME_LEN - sizeof(j5_frame_t));
#endif
    }

    k_sem_init(&spi_evt_sem, 0, 1);
    atomic_set(&pending_mask,   0);
    atomic_set(&spi_error_flag, 0);

    spi_initialized = true;

    {
        HAL_StatusTypeDef st = spi_start_dma_circular();
        if (st != HAL_OK)
        {
            printk("[SPI] ERROR: start DMA failed st=0x%x state=%u err=0x%08x\n",
                   (unsigned)st, (unsigned)hspi1.State,
                   (unsigned)HAL_SPI_GetError(&hspi1));
        }
    }

    k_thread_create(&spi_service_thread,
                    spi_service_stack,
                    SPI_SERVICE_STACK_SIZE,
                    spi_service_thread_entry,
                    NULL, NULL, NULL,
                    K_PRIO_PREEMPT(SPI_SERVICE_PRIO),
                    0, K_NO_WAIT);
    k_thread_name_set(&spi_service_thread, "spi_service");

    printk("[SPI] Initialized — DMA circular, frame_len=%d, ALWAYS_ARMED=%d\n",
           SPI_FRAME_LEN, SPI_DMA_ALWAYS_ARMED);
}

/* =========================================================
 * HAL MSP Init (DMA)
 * ========================================================= */

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) { return; }

    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA2 Stream0 Channel3 — SPI1 RX (PERIPH → MEMORY, circular) */
    hdma_spi1_rx.Instance                 = DMA2_Stream0;
    hdma_spi1_rx.Init.Channel             = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_spi1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_spi1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
        printk("[SPI] ERROR: HAL_DMA_Init RX failed\n");
        return;
    }
    __HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);

    /* DMA2 Stream3 Channel3 — SPI1 TX (MEMORY → PERIPH, circular) */
    hdma_spi1_tx.Instance                 = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel             = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_spi1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
        printk("[SPI] ERROR: HAL_DMA_Init TX failed\n");
        return;
    }
    __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);

    IRQ_CONNECT(56, 0, dma2_stream0_isr, NULL, 0);
    irq_enable(56);
    IRQ_CONNECT(59, 0, dma2_stream3_isr, NULL, 0);
    irq_enable(59);

    printk("[SPI] MSP Init: DMA configured (RX=Stream0/Ch3, TX=Stream3/Ch3)\n");
}

/* =========================================================
 * Command dispatch + TX builder
 * ========================================================= */

/** Ritorna il sequence counter big-endian estratto dal frame RX. */
static uint16_t get_rx_sequence(const uint8_t *rx)
{
    if (rx == NULL) { return 0; }
    return (uint16_t)((rx[4] << 8) | rx[5]);
}

static bool spi_is_dup_j5vr_ik_apply(const uint8_t *rx_norm, uint8_t ft_expect)
{
    if (!spi_last_j5vr_ik_valid || rx_norm == NULL) {
        return false;
    }
    if (rx_norm[3] != ft_expect) {
        return false;
    }
    return get_rx_sequence(rx_norm) == spi_last_j5vr_ik_seq && spi_last_j5vr_ik_ft == ft_expect;
}

static void spi_mark_j5vr_ik_applied(const uint8_t *rx_norm)
{
    if (rx_norm == NULL) {
        return;
    }
    const uint8_t ft = rx_norm[3];
    if (ft != (uint8_t)J5_FRAME_TYPE_J5VR && ft != (uint8_t)J5_FRAME_TYPE_J5IK) {
        return;
    }
    spi_last_j5vr_ik_seq = get_rx_sequence(rx_norm);
    spi_last_j5vr_ik_ft = ft;
    spi_last_j5vr_ik_valid = true;
}

/** Risposta TEST_ECHO: frame_type=0x02, payload 0xAA. */
static void handle_test_echo(const uint8_t *rx, j5_frame_t *tx)
{
    j5_build_frame(tx, J5_FRAME_TYPE_TEST_ECHO, get_rx_sequence(rx));
    memset(tx->payload, 0xAA, 54);
}

/** Risposta STATUS OK: payload[0-1]=0x00, poi telemetria diagnostica. */
static void handle_status_ok(const uint8_t *rx, j5_frame_t *tx)
{
    j5_build_frame(tx, J5_FRAME_TYPE_STATUS, get_rx_sequence(rx));
    tx->payload[0] = 0x00;
    tx->payload[1] = 0x00;
    j5vr_fill_tx_telemetry(tx->payload);
}

/** Risposta STATUS errore: payload[0]=error_code, payload[1]=tipo frame RX non valido. */
static void handle_status_error(const uint8_t *rx, j5_frame_t *tx,
                                uint8_t error_code, uint8_t bad_type)
{
    j5_build_frame(tx, J5_FRAME_TYPE_STATUS, get_rx_sequence(rx));
    tx->payload[0] = error_code;
    tx->payload[1] = bad_type;
}

#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)
/**
 * SPI guard (128B RAW): mismatch dichiarato / tipo illegale su frame "esteso".
 * Non invoca j5vr_parse né tocca attuazione; STATUS 0xED osservabile lato Pi.
 * payload[1]=subcode: 0x01=len RX insufficiente, 0x02=byte[6] non 64/128,
 *                    0x03=byte[6]==128 ma frame_type non 0x06/0x07
 * payload[2]=dettaglio (len, byte6, o frame_type)
 */
static volatile uint32_t s_assist_spi_guard_events;

static void assist_spi_emit_guard_tx(const uint8_t *rx, uint8_t *out_tx, uint8_t subcode,
				     uint8_t detail)
{
	s_assist_spi_guard_events++;

	j5_frame_t tx_frame;

	j5_build_frame(&tx_frame, J5_FRAME_TYPE_STATUS, get_rx_sequence(rx));
	tx_frame.payload[0] = 0xED;
	tx_frame.payload[1] = subcode;
	tx_frame.payload[2] = detail;
	if (sizeof(tx_frame.payload) > 3) {
		(void)memset(tx_frame.payload + 3, 0, sizeof(tx_frame.payload) - 3);
	}

	memcpy(out_tx, &tx_frame, sizeof(tx_frame));
#if SPI_FRAME_LEN > J5_PROTOCOL_FRAME_SIZE
	memset(out_tx + sizeof(tx_frame), 0, SPI_FRAME_LEN - sizeof(tx_frame));
#endif

	if ((s_assist_spi_guard_events & 0x3FU) == 1U) {
		printk("[ASSIST_RAW][SPI_GUARD] sub=0x%02X detail=0x%02X count=%u\n",
		       (unsigned)subcode, (unsigned)detail,
		       (unsigned)s_assist_spi_guard_events);
	}
}
#endif /* CONFIG_ASSIST_V2_RAW_MODE */

/**
 * process_spi_command_build_tx — parser frame RX e costruzione risposta TX.
 *
 * Frame valido (64B, header 'J''5'):
 *   TELEMETRY → risposta TELEMETRY con snapshot IMU+servo
 *   TEST_ECHO → echo con payload 0xAA
 *   STATUS    → STATUS con diagnostica VR
 *   J5VR      → parse payload VR, risposta TELEMETRY
 *   default   → STATUS con codice errore 0xEE
 *
 * Frame non valido (header errato o lunghezza insufficiente):
 *   → STATUS con codice errore 0xEE
 */
static void process_spi_command_build_tx(const uint8_t *rx, size_t len,
                                         uint8_t out_tx[SPI_FRAME_LEN])
{
    j5_frame_t tx_frame;
    size_t norm_off = 0U;
    const uint8_t *rx_norm = spi_pick_normalized_frame(rx, len, &norm_off);
    if (rx_norm == NULL) {
        j5_build_frame(&tx_frame, J5_FRAME_TYPE_STATUS, 0);
        tx_frame.payload[0] = 0xEE;
        tx_frame.payload[1] = (len > 0) ? rx[0] : 0;
        memcpy(out_tx, &tx_frame, sizeof(j5_frame_t));
#if SPI_FRAME_LEN > J5_PROTOCOL_FRAME_SIZE
        memset(out_tx + sizeof(j5_frame_t), 0, SPI_FRAME_LEN - sizeof(j5_frame_t));
#endif
        return;
    }
    const j5_frame_t *rx_frame = (const j5_frame_t *)rx_norm;
    {
        static uint32_t n = 0U, last_fp = 0U;
        static uint16_t last_seq = 0U;
        static uint8_t last_ft = 0U, last_d6 = 0U;
        static size_t last_off = (size_t)-1;
        const uint32_t fp = spi_fp32(rx, len);
        const uint16_t seq = get_rx_sequence(rx_norm);
        const uint8_t ft = rx_norm[3];
        const uint8_t d6 = rx_norm[6];
        const bool changed = (fp != last_fp) || (seq != last_seq) || (ft != last_ft) || (d6 != last_d6) || (norm_off != last_off);
        if (changed) {
            last_fp = fp; last_seq = seq; last_ft = ft; last_d6 = d6; last_off = norm_off;
        }
        n++;
    }
    (void)0; /* SPI_RX_NORM diagnostic removed — use LOG_DBG if needed */

#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)
    /* RAW 128B: niente interpretazione J5VR su frame incompleti o incoerenti */
    if ((len < J5_ASSIST_V2_FRAME_SIZE) && (len < J5_PROTOCOL_FRAME_SIZE)) {
	assist_spi_emit_guard_tx(rx_norm, out_tx, 0x01, (uint8_t)(len & 0xFF));
	return;
    }

    if (rx_norm[0] == 'J' && rx_norm[1] == '5') {
	const uint8_t d6 = rx_norm[6];

	if (d6 != J5_PROTOCOL_FRAME_SIZE && d6 != J5_ASSIST_V2_FRAME_SIZE) {
	    assist_spi_emit_guard_tx(rx_norm, out_tx, 0x02, d6);
	    return;
	}

	if (d6 == J5_ASSIST_V2_FRAME_SIZE) {
	    const uint8_t ft = rx_norm[3];
	    static uint32_t raw_ft_log = 0;
	    if ((raw_ft_log++ % 100U) == 0U) {
		printk("[SPI_RX_RAW128] ft=0x%02X d6=%u seq=%u\n",
		       (unsigned)ft, (unsigned)d6, (unsigned)get_rx_sequence(rx_norm));
	    }

	    if (ft == (uint8_t)J5_FRAME_TYPE_ASSIST_V2_CONTROL) {
		assist_v2_raw_handle_control_and_build_telemetry(rx_norm, out_tx);
		return;
	    }
	    if (ft == (uint8_t)J5_FRAME_TYPE_ASSIST_V2_TELEMETRY) {
		assist_v2_raw_build_telemetry_only(rx_norm, out_tx);
		return;
	    }
	    /* Compat: alcuni bridge inviano frame J5VR/J5IK legacy (64B) padded a 128B.
	     * In RAW mode non devono essere scartati: parse payload legacy e rispondi telemetria legacy. */
	    if (ft == (uint8_t)J5_FRAME_TYPE_J5VR) {
		g_j5vr_last_rx_seq = get_rx_sequence(rx_norm);
		if (!spi_is_dup_j5vr_ik_apply(rx_norm, (uint8_t)J5_FRAME_TYPE_J5VR)) {
			j5vr_parse_payload(rx_frame->payload);
			spi_mark_j5vr_ik_applied(rx_norm);
		}
		j5_build_frame(&tx_frame, J5_FRAME_TYPE_TELEMETRY, get_rx_sequence(rx_norm));
		memcpy(out_tx, &tx_frame, sizeof(j5_frame_t));
		memset(out_tx + sizeof(j5_frame_t), 0, SPI_FRAME_LEN - sizeof(j5_frame_t));
		return;
	    }
	    if (ft == (uint8_t)J5_FRAME_TYPE_J5IK) {
		if (!spi_is_dup_j5vr_ik_apply(rx_norm, (uint8_t)J5_FRAME_TYPE_J5IK)) {
			j5ik_parse_payload(rx_frame->payload);
			spi_mark_j5vr_ik_applied(rx_norm);
		}
		j5_build_frame(&tx_frame, J5_FRAME_TYPE_TELEMETRY, get_rx_sequence(rx_norm));
		memcpy(out_tx, &tx_frame, sizeof(j5_frame_t));
		memset(out_tx + sizeof(j5_frame_t), 0, SPI_FRAME_LEN - sizeof(j5_frame_t));
		return;
	    }
	    /* byte[6]==128 ma tipo non v2: non passare al parser legacy su 128 byte */
	    assist_spi_emit_guard_tx(rx, out_tx, 0x03, ft);
	    return;
	}
	/* d6==64: frame legacy nel prefisso 64B; prosegui sotto */
	/* Legacy frame diagnostic removed — zero-cost on RT path */
	/* J5VR frame diagnostic parsing removed — protocol dispatch below */
    }
#endif

    if (rx_norm[0] == 'J' && rx_norm[1] == '5')
    {
        const uint8_t frame_type = rx_frame->frame_type;

        switch (frame_type)
        {
            case J5_FRAME_TYPE_TELEMETRY:
                j5_build_frame(&tx_frame, J5_FRAME_TYPE_TELEMETRY, get_rx_sequence(rx_norm));
                break;

            case J5_FRAME_TYPE_TEST_ECHO:
                handle_test_echo(rx_norm, &tx_frame);
                break;

            case J5_FRAME_TYPE_STATUS:
                handle_status_ok(rx_norm, &tx_frame);
                break;

            case J5_FRAME_TYPE_J5VR:
                g_j5vr_last_rx_seq = get_rx_sequence(rx_norm);
                if (!spi_is_dup_j5vr_ik_apply(rx_norm, (uint8_t)J5_FRAME_TYPE_J5VR)) {
                    j5vr_parse_payload(rx_frame->payload);
                    spi_mark_j5vr_ik_applied(rx_norm);
                }
                j5_build_frame(&tx_frame, J5_FRAME_TYPE_TELEMETRY, get_rx_sequence(rx_norm));
                break;

            case J5_FRAME_TYPE_J5IK:
                if (!spi_is_dup_j5vr_ik_apply(rx_norm, (uint8_t)J5_FRAME_TYPE_J5IK)) {
                    j5ik_parse_payload(rx_frame->payload);
                    spi_mark_j5vr_ik_applied(rx_norm);
                }
                j5_build_frame(&tx_frame, J5_FRAME_TYPE_TELEMETRY, get_rx_sequence(rx_norm));
                break;

            default:
                handle_status_error(rx_norm, &tx_frame, 0xEE, frame_type);
                break;
        }
    }
    else
    {
        /* Header 'J''5' mancante o frame troppo corto */
        j5_build_frame(&tx_frame, J5_FRAME_TYPE_STATUS, 0);
        tx_frame.payload[0] = 0xEE;
        tx_frame.payload[1] = (len > 0) ? rx[0] : 0;
    }

    memcpy(out_tx, &tx_frame, sizeof(j5_frame_t));
#if SPI_FRAME_LEN > J5_PROTOCOL_FRAME_SIZE
    memset(out_tx + sizeof(j5_frame_t), 0, SPI_FRAME_LEN - sizeof(j5_frame_t));
#endif
}

static bool spi_is_valid_j5_frame_at(const uint8_t *buf, size_t len, size_t off, uint8_t *ft, uint8_t *d6)
{
    if (buf == NULL || off + J5_PROTOCOL_FRAME_SIZE > len) { return false; }
    if (buf[off] != 'J' || buf[off + 1] != '5') { return false; }
    if (buf[off + 2] != 0x01) { return false; } /* protocol_version */
    const uint8_t l = buf[off + 6];
#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)
    if (l != J5_PROTOCOL_FRAME_SIZE && l != J5_ASSIST_V2_FRAME_SIZE) { return false; }
#else
    if (l != J5_PROTOCOL_FRAME_SIZE) { return false; }
#endif
    if (off + (size_t)l > len) { return false; }
    if (buf[off + 7] != 0x00) { return false; } /* flags attesi 0 */
    const uint8_t t = buf[off + 3];
    switch (t) {
        case J5_FRAME_TYPE_TELEMETRY:
        case J5_FRAME_TYPE_TEST_ECHO:
        case J5_FRAME_TYPE_STATUS:
        case J5_FRAME_TYPE_J5VR:
        case J5_FRAME_TYPE_J5IK:
            break;
#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)
        case J5_FRAME_TYPE_ASSIST_V2_CONTROL:
        case J5_FRAME_TYPE_ASSIST_V2_TELEMETRY:
            break;
#endif
        default:
            return false;
    }
    if (ft != NULL) { *ft = t; }
    if (d6 != NULL) { *d6 = l; }
    return true;
}

static const uint8_t *spi_pick_normalized_frame(const uint8_t *rx, size_t len, size_t *off_out)
{
    if (rx == NULL || len < J5_PROTOCOL_FRAME_SIZE) { return NULL; }
    size_t best_off = (size_t)-1;
    uint16_t best_seq = 0U;
    int best_score = -1;
    bool ambiguous = false;
    for (size_t off = 0; off + J5_PROTOCOL_FRAME_SIZE <= len; off++) {
        uint8_t ft = 0U, d6 = 0U;
        if (!spi_is_valid_j5_frame_at(rx, len, off, &ft, &d6)) { continue; }
        const uint16_t seq = (uint16_t)(((uint16_t)rx[off + 4] << 8) | (uint16_t)rx[off + 5]);
        int score = 0;
#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)
        if (d6 == J5_ASSIST_V2_FRAME_SIZE) {
            score += (off == 0U) ? 100 : -100;
        } else {
            /* Legacy 64B in slot 128: prefer offsets canonici 0/64, ma consenti recovery da disallineamento. */
            if (off == 0U || off == 64U) score += 50;
            else score += 10;
        }
        if (ft == (uint8_t)J5_FRAME_TYPE_J5VR || ft == (uint8_t)J5_FRAME_TYPE_ASSIST_V2_CONTROL) score += 20;
#else
        /* Canonical 128 padded: la sola semantica attiva resta quella 64B nel prefisso.
         * Il tail 64..127 e ignorato e non deve competere come fonte semantica. */
        if (off == 0U || off == 64U) score += 50;
        else score += 10;
        if (ft == (uint8_t)J5_FRAME_TYPE_J5VR) score += 20;
#endif

        if (best_off == (size_t)-1) {
            best_off = off;
            best_seq = seq;
            best_score = score;
            ambiguous = false;
            continue;
        }

        if (score > best_score) {
            best_off = off;
            best_seq = seq;
            best_score = score;
            ambiguous = false;
            continue;
        }
        if (score == best_score) {
            if (seq_is_newer_u16(seq, best_seq)) {
                best_off = off;
                best_seq = seq;
                ambiguous = false;
            } else if (seq == best_seq && off != best_off) {
                ambiguous = true;
            }
        }
    }
    if (best_off == (size_t)-1) {
        static uint32_t n0 = 0U, last_fp0 = 0U;
        const uint32_t fp0 = spi_fp32(rx, len);
        if (fp0 != last_fp0) {
            printk("[SPI_RX_TIMELINE] t=%u n=%u slot=%u valid=0 fp=%08X raw8=%02X%02X%02X%02X%02X%02X%02X%02X\n",
                   (unsigned)k_uptime_get_32(), (unsigned)n0, (unsigned)len, (unsigned)fp0,
                   (unsigned)(len > 0 ? rx[0] : 0U), (unsigned)(len > 1 ? rx[1] : 0U),
                   (unsigned)(len > 2 ? rx[2] : 0U), (unsigned)(len > 3 ? rx[3] : 0U),
                   (unsigned)(len > 4 ? rx[4] : 0U), (unsigned)(len > 5 ? rx[5] : 0U),
                   (unsigned)(len > 6 ? rx[6] : 0U), (unsigned)(len > 7 ? rx[7] : 0U));
            last_fp0 = fp0;
        }
        n0++;
        return NULL;
    }
    if (ambiguous) {
        printk("[SPI_RX_TIMELINE] t=%u n=0 slot=%u valid=0 amb=1 fp=%08X\n",
               (unsigned)k_uptime_get_32(), (unsigned)len, (unsigned)spi_fp32(rx, len));
        return NULL;
    }
    if (off_out != NULL) { *off_out = best_off; }
    return rx + best_off;
}

static uint32_t spi_fp32(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    if (p == NULL) { return h; }
    for (size_t i = 0; i < n; i++) {
        h ^= (uint32_t)p[i];
        h *= 16777619u;
    }
    return h;
}

static bool seq_is_newer_u16(uint16_t a, uint16_t b)
{
    return (int16_t)(a - b) > 0;
}

/* =========================================================
 * HAL SPI callback overrides (stub intenzionalmente vuoti)
 *
 * La HAL STM32 in HAL_SPI_TxRxCpltCallback e HalfCpltCallback chiama
 * __HAL_SPI_DISABLE_IT() che disabilita RXDMAEN/TXDMAEN, rompendo il
 * circular mode. Questi stub prevengono quel comportamento.
 * Gli eventi sono gestiti esclusivamente dai callback DMA RX.
 * ========================================================= */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) { return; }
    /* Vuoto intenzionale */
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) { return; }
    /* Vuoto intenzionale */
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) { return; }

    printk("[SPI] HAL_SPI_ErrorCallback EC=0x%08x SR=0x%08x\n",
           (unsigned)hspi->ErrorCode, (unsigned)SPI1->SR);

    /* Clear OVR (STM32F4: leggere DR poi SR) */
    if (hspi->ErrorCode & HAL_SPI_ERROR_OVR)
    {
        volatile uint32_t tmp = hspi->Instance->DR;
        tmp = hspi->Instance->SR;
        (void)tmp;
    }

    /* Clear MODF (STM32F4: SR read + CR1 write; manteniamo SPE=1) */
    if (hspi->ErrorCode & HAL_SPI_ERROR_MODF)
    {
        volatile uint32_t tmp = hspi->Instance->SR;
        (void)tmp;
        hspi->Instance->CR1 |= (1u << 6); /* SPE */
    }

    /* Clear CRCERR se impostato (SR bit4) */
    if (hspi->Instance->SR & (1u << 4))
    {
        hspi->Instance->SR &= ~(1u << 4);
    }

    atomic_set(&spi_error_flag, 1);
    k_sem_give(&spi_evt_sem);
}

/* =========================================================
 * API pubblica
 * ========================================================= */

bool hal_spi_slave_is_ready(void)
{
    return spi_initialized;
}

static int hal_spi_slave_sys_init(void)
{
    hal_spi_slave_init();
    return 0;
}

SYS_INIT(hal_spi_slave_sys_init, POST_KERNEL, 50);
