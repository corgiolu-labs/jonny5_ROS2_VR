/*
 * Boundary Buffers - Header
 * 
 * Boundary layer blindato HAL ↔ Zephyr con operazioni atomiche.
 * Latest sample wins, no race condition, no mutex in HAL.
 * 
 * Architettura JONNY5 v1.0 - Sezione 8.3 - FASE 2
 */

#ifndef BOUNDARY_BUFFERS_H
#define BOUNDARY_BUFFERS_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>

/* RX slot size: 64 legacy-only, 128 when ASSIST_V2_RAW_MODE (must match SPI_FRAME_LEN). */
#if IS_ENABLED(CONFIG_ASSIST_V2_RAW_MODE)
#define J5_FRAME_SIZE 128
#else
#define J5_FRAME_SIZE 64
#endif
#define RX_SLOTS 2

/* Inizializzazione boundary buffers */
void boundary_init(void);

/* HAL-side API */
/* CHI: HAL scrive nel buffer RX active */
uint8_t* boundary_hal_rx_write_ptr(void);

/* CHI: HAL committa frame completo (toggle active_idx, latest-sample-wins) */
void boundary_hal_rx_commit(void);

#endif /* BOUNDARY_BUFFERS_H */
