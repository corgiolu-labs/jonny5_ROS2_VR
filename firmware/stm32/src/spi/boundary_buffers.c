/*
 * Boundary Buffers - Implementation
 * 
 * Boundary layer blindato HAL ↔ Zephyr con operazioni atomiche.
 * Latest sample wins, no race condition, no mutex in HAL.
 * 
 * Architettura JONNY5 v1.0 - Sezione 8.3 - FASE 2
 */

#include "spi/boundary_buffers.h"
#include <zephyr/sys/atomic.h>
#include <string.h>

/* Struttura interna: doppio buffer RX (no tearing) */
static uint8_t rx_buf[RX_SLOTS][J5_FRAME_SIZE];
static atomic_t rx_active_idx;     /* 0/1: slot in scrittura HAL */

/* Inizializzazione boundary buffers */
void boundary_init(void)
{
    /* Azzera buffer RX */
    memset(rx_buf[0], 0, J5_FRAME_SIZE);
    memset(rx_buf[1], 0, J5_FRAME_SIZE);

    /* Reset indice attivo */
    atomic_set(&rx_active_idx, 0);
}

/* HAL-side: ottiene puntatore al buffer RX active per scrittura diretta */
uint8_t* boundary_hal_rx_write_ptr(void)
{
    uint32_t idx = atomic_get(&rx_active_idx);
    return rx_buf[idx];
}

/* HAL-side: committa frame RX completo (latest sample wins) */
void boundary_hal_rx_commit(void)
{
    /* Toggle active_idx: passa allo slot alternativo (latest-sample-wins) */
    uint32_t old_idx = atomic_get(&rx_active_idx);
    uint32_t new_idx = 1 - old_idx;
    atomic_set(&rx_active_idx, new_idx);
}

