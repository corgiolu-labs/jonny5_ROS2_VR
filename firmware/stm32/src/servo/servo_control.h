/*
 * Servo Control - Header
 * 
 * Controllo servo PWM per 6 DOF robot.
 * Mappatura: BASE, SPALLA, GOMITO, YAW, PITCH, ROLL (indici 0-5)
 * 
 * Architettura JONNY5 v1.0 - Sezione 5.2.1
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* Enum giunti robot */
typedef enum {
    SERVO_BASE = 0,    /* J3  - PB7 - TIM4_CH2  (gruppo A) */
    SERVO_SPALLA,     /* J4  - PB9 - TIM17_CH1 (gruppo A) */
    SERVO_GOMITO,     /* J6  - PB5 - TIM3_CH2  (gruppo A) */
    SERVO_YAW,        /* J7  - PB6 - TIM4_CH1  (gruppo A) — rotazione dx/sx (ZYX) */
    SERVO_PITCH,      /* J14 - PA3 - TIM5_CH4  (gruppo B, 330 Hz) */
    SERVO_ROLL,       /* J16 - PA2 - TIM5_CH3  (gruppo B, 330 Hz) — tilt laterale (ZYX) */
    SERVO_COUNT       /* Numero totale servo */
} servo_joint_t;

/* Global physical safety limits — no servo can go outside this range */
#define SERVO_SAFETY_MIN_DEG  10.0f
#define SERVO_SAFETY_MAX_DEG 170.0f

/* Inizializzazione controllo servo */
bool servo_control_init(void);

/**
 * Frequenza PWM (Hz) per il giunto: gruppo A (B/S/G/Y) 50 Hz, gruppo B (PITCH/ROLL) 330 Hz.
 */
uint32_t servo_get_period_hz(servo_joint_t joint);

/**
 * Versione float per movimento sub-degree.
 * Usata dal trajectory tick (j5vr_setpose_tick) ad alta freq (1 kHz)
 * per evitare la quantizzazione a 1° del PWM nelle interpolazioni
 * smooth dei profili RTR3/RTR5/BCB/BB. Calcolo PWM in float.
 */
bool servo_set_angle_f(servo_joint_t joint, float angle_deg);

/** Angolo logico corrente 0–180 per tutti i giunti. */
uint8_t servo_get_angle(servo_joint_t joint);

/** Angolo logico corrente con la frazione (ultimo valore comandato, 0–180). */
float servo_get_angle_f(servo_joint_t joint);

/**
 * Dichiara dove si trova il servo SENZA scrivere il PWM (SETPOSE_INIT dal Pi all'avvio: la posa in cui il
 * braccio e' stato spento, PARK). Da qui partono il limitatore di velocita' e la prima traiettoria, cosi' il
 * primo impulso non fa saltare il servo alla posa HOME.
 */
void servo_set_angle_init(servo_joint_t joint, float angle_deg);

/* Ultimo pulse width applicato (microsecondi); 0 se mai applicato / disabilitato */
uint16_t servo_get_last_pulse_us(servo_joint_t joint);

/* Disabilita tutti i servo (safe state) */
void servo_disable_all(void);

/**
 * Rilassa i servo digitali (GIMOD 2065) togliendo il segnale PWM.
 * A differenza di servo_disable_all(), non tocca i servo analogici (LDX-218)
 * e non cambia lo stato logico degli angoli.
 * Da chiamare quando il robot è fermo da >200ms per eliminare il buzz.
 * Al prossimo servo_set_angle_f() il PWM viene automaticamente ripristinato.
 */
void servo_relax_digital(void);


/*
 * Accesso agli offset meccanici e alla posa teleop.
 * Usare i getter invece di accedere direttamente ai static interni.
 */
const uint8_t *servo_get_offset_deg(void);  /* {104,107,77,88,95,104} di default */
const uint8_t *servo_get_teleop_deg(void);  /* {104,107,124,88,126,104} di default */

/* Aggiorna offset meccanici a runtime (da uart_control / SET_OFFSETS). */
void servo_offset_set(const uint8_t values[SERVO_COUNT]);

/**
 * Aggiorna configurazione PWM a runtime per entrambi i gruppi timer.
 * Tutti i valori vengono clampati ai limiti hardware assoluti.
 *
 * tim8_*: BASE/SPALLA/GOMITO/YAW
 * tim1_*: PITCH/ROLL
 */
void servo_set_pwm_config(uint32_t tim8_hz, uint32_t tim8_min_us, uint32_t tim8_max_us,
			  uint32_t tim8_max_deg, uint32_t tim1_hz, uint32_t tim1_min_us,
			  uint32_t tim1_max_us, uint32_t tim1_max_deg);

/*
 * Macro di compatibilità per i moduli che leggono servo_offset_deg[i] / servo_teleop_deg[i].
 * Sostituire con servo_get_offset_deg()[i] per accesso diretto.
 */
#define servo_offset_deg  (servo_get_offset_deg())
#define servo_teleop_deg  (servo_get_teleop_deg())

#endif /* SERVO_CONTROL_H */
