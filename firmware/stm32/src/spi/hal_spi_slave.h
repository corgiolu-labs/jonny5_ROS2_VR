/*
 * HAL SPI Slave - Header
 * 
 * SPI1 slave mode deterministica con DMA circular (always-armed).
 * TX buffer sempre precaricato, RX acquisito durante transazione.
 * 
 * Architettura JONNY5 v1.0 - SPI Deterministico
 */

#ifndef HAL_SPI_SLAVE_H
#define HAL_SPI_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

/* Inizializzazione HAL SPI slave */
void hal_spi_slave_init(void);

/* Verifica se SPI è pronto */
bool hal_spi_slave_is_ready(void);

/** Tempo in ms dall'ultimo frame SPI valido ricevuto dal Pi.
 *  Ritorna UINT32_MAX se nessun frame è mai arrivato.
 *  Usato dal watchdog in rt_loop.c. */
uint32_t hal_spi_last_frame_age_ms(void);

/** Se nessun frame SPI valido arriva entro questo tempo, il RT loop
 *  porta il sistema in STATE_SAFE. */
#define SPI_FRAME_TIMEOUT_MS 500U

#endif /* HAL_SPI_SLAVE_H */
