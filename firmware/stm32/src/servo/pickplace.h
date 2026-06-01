/*
 * pickplace.h — Pick & Place actuator driver (PWM su 2 IRFZ44N MOSFET gate).
 *
 * Hardware:
 *   PP1 → PA0 (TIM2_CH1) → gate IRFZ44N → elettrovalvola.
 *   PP2 → PA1 (TIM2_CH2) → gate IRFZ44N → coppia di vacuum motors.
 *
 * Frequenza PWM: 20 kHz (sopra soglia udibile), comune ai 2 canali (stesso TIM2).
 * Polarità NORMAL: duty 0% = MOSFET OFF, duty 100% = MOSFET completamente ON.
 *
 * Sicurezza:
 *   - pickplace_init() lascia entrambi i canali a duty=0 (MOSFET OFF) al boot.
 *   - pickplace_safe_off() viene chiamata da uart_control.c sui comandi
 *     STOP/SAFE/RESET per garantire shutdown immediato.
 */

#ifndef PICKPLACE_H
#define PICKPLACE_H

#include <stdint.h>
#include <stdbool.h>

#define PICKPLACE_CH_COUNT  2U
#define PICKPLACE_CH_PP1    0U  /* PA0 — elettrovalvola */
#define PICKPLACE_CH_PP2    1U  /* PA1 — vacuum motors */

#define PICKPLACE_DUTY_MIN  0U
#define PICKPLACE_DUTY_MAX  100U

/**
 * pickplace_init — verifica readiness dei due PWM device-tree spec e
 * applica duty=0 a entrambi i canali (MOSFET OFF al boot).
 * Return: true se entrambi i canali pronti, false se almeno uno non lo è.
 */
bool pickplace_init(void);

/**
 * pickplace_set_duty — imposta il duty cycle [0..100] sul canale.
 * channel: PICKPLACE_CH_PP1 o PICKPLACE_CH_PP2.
 * duty_0_100: clampato in [0,100]; 0 = MOSFET OFF.
 * Return: true on success.
 */
bool pickplace_set_duty(uint8_t channel, uint8_t duty_0_100);

/**
 * pickplace_get_duty — ultimo duty applicato al canale (0 se non inizializzato).
 */
uint8_t pickplace_get_duty(uint8_t channel);

/**
 * pickplace_safe_off — azzera entrambi i canali (duty=0).
 * Chiamato da uart_control.c su STOP/SAFE/RESET.
 */
void pickplace_safe_off(void);

#endif /* PICKPLACE_H */
