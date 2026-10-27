/*
 * uart_control.h — UART Control Plane API
 *
 * Control plane UART ASCII per comandi di teleoperazione e configurazione.
 * Comandi supportati: STOP, SAFE, ENABLE, STATUS?,
 *   HOME, PARK, TELEOPPOSE, SETPOSE, SETPOSE_T_HR, SETPOSE_INIT, SET_OFFSETS,
 *   SET_JOINT_LIMITS, SET_VR_PARAMS, VR?
 *
 * Formato richiesta: [#<seq>] <CMD> [argomenti]\n
 * Formato risposta:  [#<seq>] OK <CMD> | ERR <REASON>\n
 *
 * Connessione fisica: USART3 (PB10 TX, PB11 RX) → Raspberry Pi.
 */

#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include <stdbool.h>

/** Inizializza UART1 e il ring buffer RX. Da chiamare una volta all'avvio. */
void uart_control_init(void);

/** Processa i byte disponibili nel ring buffer e dispatcha i comandi completi.
 *  Da chiamare periodicamente (es. dal RT loop o da un thread dedicato). */
void uart_control_process(void);

/**
 * uart_send_unsolicited — invia un messaggio non sollecitato (senza seq number).
 * Usato da j5vr_setpose per inviare SETPOSE_DONE a fine traiettoria.
 * Usa uart_poll_out (busy-wait): chiamare solo a eventi rari, non ad ogni tick.
 */
void uart_send_unsolicited(const char *msg);


#endif /* UART_CONTROL_H */
