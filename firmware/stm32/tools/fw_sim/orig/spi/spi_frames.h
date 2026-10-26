/*
 * spi_frames.h — logica dei frame J5 sul link SPI, indipendente dall'HAL.
 */
#ifndef SPI_FRAMES_H
#define SPI_FRAMES_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/sys/util.h>
#include "spi/j5_protocol.h"

/* Lunghezza di un trasferimento SPI = metà del buffer DMA circolare.
 * Master e slave devono usare la stessa lunghezza. */
#if IS_ENABLED(CONFIG_J5_CANONICAL_PADDED_128_MODE)
#define SPI_FRAME_LEN          128U                    /* 64 B canonici + 64 B a zero */
#else
#define SPI_FRAME_LEN          J5_PROTOCOL_FRAME_SIZE  /* 64 */
#endif

/** Processa un half DMA appena ricevuto (SPI_FRAME_LEN byte) e scrive la
 *  risposta (SPI_FRAME_LEN byte) che uscirà nel trasferimento successivo. */
void spi_frames_ingest_half(const uint8_t *rx_half, uint8_t *tx_next_half);

/** Dimentica coda e anti-duplicato (dopo un restart del DMA). */
void spi_frames_reset(void);

#endif /* SPI_FRAMES_H */
