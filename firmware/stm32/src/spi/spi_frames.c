/*
 * spi_frames.c — logica dei frame J5 sul link SPI (nessun accesso all'hardware).
 *
 * Spostata verbatim da hal_spi_slave.c: per ogni half DMA ricevuto costruisce
 * la finestra (coda dell'half precedente + half corrente), sceglie il frame J5
 * valido, lo applica (J5VR) e scrive la risposta nel buffer TX del
 * prossimo half. Separata dall'HAL perché la stessa logica gira identica nel
 * simulatore su PC (tools/fw_sim).
 */

#include "spi/spi_frames.h"
#include "spi/j5_protocol.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

/* Timestamp (ms) dell'ultimo frame SPI valido ricevuto dal Pi.
 * Aggiornato quando il parser trova un frame J5 valido (header/versione/tipo):
 * rumore o frame corrotti non tengono vivo il watchdog.
 * Letto dal RT loop per il watchdog timeout. */
static atomic_t _last_frame_rx_ms = ATOMIC_INIT(0);

uint32_t hal_spi_last_frame_age_ms(void)
{
    uint32_t last = (uint32_t)atomic_get(&_last_frame_rx_ms);
    if (last == 0) { return UINT32_MAX; }
    return k_uptime_get_32() - last;
}


/* Ingest cross-boundary: ultimi (SPI_FRAME_LEN-1) byte dell'half precedente + half corrente.
 * Copre frame J5 fino a 128 B che iniziano in coda a un half DMA e continuano nel successivo.
 * spi_rx_window: tail_len + SPI_FRAME_LEN <= 2*SPI_FRAME_LEN - 1 (entra in SPI_FRAME_LEN*2). */
#define SPI_INGEST_TAIL_LEN ((size_t)(SPI_FRAME_LEN - 1U))
static uint8_t spi_ingest_tail[SPI_FRAME_LEN - 1U];
static bool    spi_ingest_tail_valid = false;
static uint8_t spi_rx_window[SPI_FRAME_LEN * 2U];

/* Anti re-parse: stesso frame J5VR (header seq) visto due volte nella finestra sovrapposta. */
static uint16_t spi_last_j5vr_seq = 0U;
static bool     spi_last_j5vr_valid = false;


/* =========================================================
 * Forward declarations
 * ========================================================= */

static void process_spi_command_build_tx(const uint8_t *rx, size_t len,
                                         uint8_t out_tx[SPI_FRAME_LEN]);
static bool spi_is_valid_j5_frame_at(const uint8_t *buf, size_t len, size_t off, uint8_t *ft, uint8_t *d6);
static const uint8_t *spi_pick_normalized_frame(const uint8_t *rx, size_t len, size_t *off_out);
static bool seq_is_newer_u16(uint16_t a, uint16_t b);

/* =========================================================
 * Command dispatch + TX builder
 * ========================================================= */

/** Ritorna il sequence counter big-endian estratto dal frame RX. */
static uint16_t get_rx_sequence(const uint8_t *rx)
{
    if (rx == NULL) { return 0; }
    return (uint16_t)((rx[4] << 8) | rx[5]);
}

static bool spi_is_dup_j5vr_apply(const uint8_t *rx_norm)
{
    return spi_last_j5vr_valid && get_rx_sequence(rx_norm) == spi_last_j5vr_seq;
}

static void spi_mark_j5vr_applied(const uint8_t *rx_norm)
{
    spi_last_j5vr_seq = get_rx_sequence(rx_norm);
    spi_last_j5vr_valid = true;
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
    atomic_set(&_last_frame_rx_ms, (atomic_val_t)k_uptime_get_32());


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
                if (!spi_is_dup_j5vr_apply(rx_norm)) {
                    j5vr_parse_payload(rx_frame->payload);
                    spi_mark_j5vr_applied(rx_norm);
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
    if (buf[off + 2] != (uint8_t)J5_PROTOCOL_VERSION_V1) { return false; } /* protocol_version */
    const uint8_t l = buf[off + 6];
    if (l != J5_PROTOCOL_FRAME_SIZE) { return false; }
    if (off + (size_t)l > len) { return false; }
    if (buf[off + 7] != 0x00) { return false; } /* flags attesi 0 */
    const uint8_t t = buf[off + 3];
    switch (t) {
        case J5_FRAME_TYPE_TELEMETRY:
        case J5_FRAME_TYPE_TEST_ECHO:
        case J5_FRAME_TYPE_STATUS:
        case J5_FRAME_TYPE_J5VR:
            break;
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
        /* Canonical 128 padded: la sola semantica attiva resta quella 64B nel prefisso.
         * Il tail 64..127 e ignorato e non deve competere come fonte semantica. */
        if (off == 0U || off == 64U) score += 50;
        else score += 10;
        if (ft == (uint8_t)J5_FRAME_TYPE_J5VR) score += 20;

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
    if (best_off == (size_t)-1 || ambiguous) {
        return NULL;   /* no frame, or two frames with the same seq: answer STATUS 0xEE */
    }
    if (off_out != NULL) { *off_out = best_off; }
    return rx + best_off;
}


static bool seq_is_newer_u16(uint16_t a, uint16_t b)
{
    return (int16_t)(a - b) > 0;
}


/* =========================================================
 * Ingest di un half DMA (chiamato dal thread SPI service in hal_spi_slave.c)
 * ========================================================= */

void spi_frames_ingest_half(const uint8_t *rx_ptr, uint8_t *tx_ptr_next)
{
    /* Finestra estesa tail + current_half per frame che attraversano il confine tra le due half DMA. */
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
}

void spi_frames_reset(void)
{
    spi_ingest_tail_valid = false;
    spi_last_j5vr_valid = false;
}
