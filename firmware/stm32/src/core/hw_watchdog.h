/*
 * hw_watchdog.h — watchdog hardware (IWDG) alimentato dal RT loop a 1 kHz.
 *
 * Opt-in: CONFIG_WATCHDOG=y + CONFIG_J5_HW_WATCHDOG=y (prj.conf) e alias
 * devicetree "watchdog0" abilitato (nucleo_g474re / SHIELD rev3 lo ha).
 * Se il RT loop si blocca (fatal halt, deadlock, overrun infinito) l'IWDG non
 * viene piu' alimentato e resetta l'MCU: al boot lo stato e' SAFE e i PWM
 * ripartono spenti, invece di restare sull'ultimo impulso.
 * Senza CONFIG_J5_HW_WATCHDOG le funzioni sono no-op.
 */
#ifndef HW_WATCHDOG_H
#define HW_WATCHDOG_H

/** Avvia l'IWDG. Ritorna 0 se attivo, <0 in errore, 1 se disabilitato da config. */
int hw_watchdog_init(void);

/** Alimenta l'IWDG (chiamata a ogni tick del RT loop). */
void hw_watchdog_feed(void);

#endif /* HW_WATCHDOG_H */
