/*
 * Servo Control - Implementation
 *
 * Controllo servo PWM per 6 DOF robot.
 *
 * Architettura JONNY5 v1.0 - Sezione 5.2.1
 */

#include "servo/servo_control.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <errno.h>

LOG_MODULE_REGISTER(servo_control, LOG_LEVEL_INF);

/* Device tree: pwm_dt_spec. Ordine 0..5 = BASE, SPALLA, GOMITO, YAW, PITCH, ROLL. */
static const struct pwm_dt_spec servo_pwms[SERVO_COUNT] = {
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo0)), /* BASE   = TIM8_CH1 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo1)), /* SPALLA = TIM8_CH2 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo2)), /* GOMITO = TIM8_CH3 */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo3)), /* YAW    = TIM8_CH4 (PC9) */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo4)), /* PITCH  = TIM1_CH1 (PA8) */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo5)), /* ROLL   = TIM1_CH4 (PA11) */
};

/* Limiti assoluti di sicurezza hardware (mai superabili a runtime). */
#define SERVO_MIN_US_HARD 400U
#define SERVO_MAX_US_HARD 2600U
#define SERVO_HZ_MIN      10U
#define SERVO_HZ_MAX      400U
#define SERVO_DEG_MIN     60U
#define SERVO_DEG_MAX     360U

#define SERVO_TIMER_MAX 0xFFFFU

/* Configurazione PWM runtime per gruppo TIM8 (BASE/SPALLA/GOMITO/YAW). */
static uint32_t _pwm_hz_tim8     = 50U;
static uint32_t _min_us_tim8     = 500U;
static uint32_t _max_us_tim8     = 2500U;
static uint32_t _max_deg_tim8    = 180U;

/* Configurazione PWM runtime per gruppo TIM1 (PITCH/ROLL). */
static uint32_t _pwm_hz_tim1     = 50U;
static uint32_t _min_us_tim1     = 500U;
static uint32_t _max_us_tim1     = 2500U;
static uint32_t _max_deg_tim1    = 180U;

/* Stato corrente angoli servo in gradi logici 0..180 per tutti i giunti. */
static uint8_t servo_angles[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
static uint16_t servo_last_pulse_us[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

static uint8_t servo_last_sent_angle[SERVO_COUNT] = {255, 255, 255, 255, 255, 255};

#define DIGITAL_SERVO_MASK ((1U << SERVO_PITCH) | (1U << SERVO_ROLL))

/*
 * Offset meccanici e pose predefinite - modificabili a runtime via SET_OFFSETS UART.
 */
static uint8_t _servo_offset_deg[SERVO_COUNT] = {104, 107, 77, 88, 95, 104};
static uint8_t _servo_teleop_deg[SERVO_COUNT] = {104, 107, 124, 88, 126, 104};

static bool servo_ready[SERVO_COUNT];

uint32_t servo_get_period_hz(servo_joint_t joint)
{
	if (joint >= SERVO_COUNT) {
		return _pwm_hz_tim8;
	}
	return (joint >= SERVO_PITCH) ? _pwm_hz_tim1 : _pwm_hz_tim8;
}

void servo_set_pwm_config(uint32_t tim8_hz, uint32_t tim8_min_us, uint32_t tim8_max_us,
			  uint32_t tim8_max_deg, uint32_t tim1_hz, uint32_t tim1_min_us,
			  uint32_t tim1_max_us, uint32_t tim1_max_deg)
{
	/* Clamp dentro i limiti hardware assoluti. */
	_pwm_hz_tim8  = CLAMP(tim8_hz,      SERVO_HZ_MIN,   SERVO_HZ_MAX);
	_min_us_tim8  = CLAMP(tim8_min_us,  SERVO_MIN_US_HARD, SERVO_MAX_US_HARD);
	_max_us_tim8  = CLAMP(tim8_max_us,  SERVO_MIN_US_HARD, SERVO_MAX_US_HARD);
	_max_deg_tim8 = CLAMP(tim8_max_deg, SERVO_DEG_MIN,  SERVO_DEG_MAX);

	_pwm_hz_tim1  = CLAMP(tim1_hz,      SERVO_HZ_MIN,   SERVO_HZ_MAX);
	_min_us_tim1  = CLAMP(tim1_min_us,  SERVO_MIN_US_HARD, SERVO_MAX_US_HARD);
	_max_us_tim1  = CLAMP(tim1_max_us,  SERVO_MIN_US_HARD, SERVO_MAX_US_HARD);
	_max_deg_tim1 = CLAMP(tim1_max_deg, SERVO_DEG_MIN,  SERVO_DEG_MAX);

	/* min < max (scambia se invertiti). */
	if (_min_us_tim8 >= _max_us_tim8) { _min_us_tim8 = _max_us_tim8 - 1U; }
	if (_min_us_tim1 >= _max_us_tim1) { _min_us_tim1 = _max_us_tim1 - 1U; }

	printk("[SERVO_CFG] TIM8: %u Hz  %u..%u us  0..%u deg\n",
	       (unsigned)_pwm_hz_tim8, (unsigned)_min_us_tim8,
	       (unsigned)_max_us_tim8, (unsigned)_max_deg_tim8);
	printk("[SERVO_CFG] TIM1: %u Hz  %u..%u us  0..%u deg\n",
	       (unsigned)_pwm_hz_tim1, (unsigned)_min_us_tim1,
	       (unsigned)_max_us_tim1, (unsigned)_max_deg_tim1);
}

/**
 * pulse_us: larghezza impulso [0, SERVO_MAX_US]; 0 disabilita l'uscita nel driver.
 * period_hz: frequenza portante PWM per questo canale (50 Hz TIM8, 50 Hz TIM1).
 */
static int compute_period_and_pulse(const struct pwm_dt_spec *spec, uint32_t pulse_us,
				    uint32_t period_hz, uint32_t *period_out, uint32_t *pulse_out)
{
	uint64_t cycles_per_sec = 0;
	int ret = pwm_get_cycles_per_sec(spec->dev, spec->channel, &cycles_per_sec);

	if (ret < 0 || cycles_per_sec == 0) {
		return -ENODEV;
	}

	if (period_hz == 0U) {
		return -EINVAL;
	}

	uint64_t period_cycles = cycles_per_sec / (uint64_t)period_hz;
	uint64_t pulse_cycles = (cycles_per_sec * (uint64_t)pulse_us) / 1000000ULL;

	if (period_cycles > SERVO_TIMER_MAX) {
		uint64_t prescale = (period_cycles + SERVO_TIMER_MAX) / (SERVO_TIMER_MAX + 1ULL);

		if (prescale == 0) {
			prescale = 1;
		}
		period_cycles /= prescale;
		pulse_cycles /= prescale;
	}

	if (period_cycles == 0) {
		return -EINVAL;
	}
	if (pulse_cycles > period_cycles) {
		pulse_cycles = period_cycles;
	}

	*period_out = (uint32_t)period_cycles;
	*pulse_out = (uint32_t)pulse_cycles;
	return 0;
}

static int servo_apply_pulse_us(const struct pwm_dt_spec *spec, uint32_t pulse_us,
				uint32_t period_hz)
{
	uint32_t period_cycles = 0;
	uint32_t pulse_cycles = 0;
	int ret = compute_period_and_pulse(spec, pulse_us, period_hz, &period_cycles, &pulse_cycles);

	if (ret < 0) {
		return ret;
	}
	return pwm_set_cycles(spec->dev, spec->channel, period_cycles, pulse_cycles, spec->flags);
}

bool servo_control_init(void)
{
	bool driver_ready = true;

	printk("[SERVO] TIM8: BASE/SPALLA/GOMITO/YAW @ %u Hz  %u..%u us  0..%u deg\n",
	       (unsigned)_pwm_hz_tim8, (unsigned)_min_us_tim8,
	       (unsigned)_max_us_tim8, (unsigned)_max_deg_tim8);
	printk("[SERVO] TIM1: PITCH/ROLL @ %u Hz  %u..%u us  0..%u deg\n",
	       (unsigned)_pwm_hz_tim1, (unsigned)_min_us_tim1,
	       (unsigned)_max_us_tim1, (unsigned)_max_deg_tim1);

	LOG_INF("TIM8 servo group: BASE/SPALLA/GOMITO/YAW @ %u Hz", (unsigned)_pwm_hz_tim8);
	LOG_INF("TIM1 servo group: PITCH/ROLL @ %u Hz", (unsigned)_pwm_hz_tim1);
	LOG_INF("Servo pulse range TIM8: %u-%u us", (unsigned)_min_us_tim8, (unsigned)_max_us_tim8);

	for (int i = 0; i < SERVO_COUNT; i++) {
		const struct pwm_dt_spec *spec = &servo_pwms[i];
		const uint32_t hz = servo_get_period_hz((servo_joint_t)i);

		if (!device_is_ready(spec->dev)) {
			LOG_ERR("[SERVO] PWM device not ready for servo %d", i);
			driver_ready = false;
			servo_ready[i] = false;
			continue;
		}
		servo_ready[i] = true;

		uint32_t period_cycles = 0;
		uint32_t pulse_cycles = 0;

		if (compute_period_and_pulse(spec, 0, hz, &period_cycles, &pulse_cycles) == 0) {
			(void)pwm_set_cycles(spec->dev, spec->channel, period_cycles, 0, spec->flags);
		}
	}

	if (driver_ready) {
		LOG_INF("[SERVO] PWM init OK (TIM8 CH1-4, TIM1 CH1+CH4)");
	} else {
		LOG_ERR("[SERVO] Some PWM devices not ready");
	}

	for (int i = 0; i < SERVO_COUNT; i++) {
		servo_angles[i] = 90;
		servo_last_sent_angle[i] = 255;
	}

	return driver_ready;
}

bool servo_set_angle(servo_joint_t joint, uint16_t angle_deg)
{
	if (joint >= SERVO_COUNT) {
		LOG_ERR("[SERVO] Invalid joint: %d", joint);
		printk("[SERVO_ERR] invalid_joint=%d\n", (int)joint);
		return false;
	}

	if (!servo_ready[joint]) {
		LOG_ERR("[SERVO] Servo %d not ready", joint);
		printk("[SERVO_ERR] not_ready joint=%d\n", (int)joint);
		return false;
	}

	/* Seleziona parametri del gruppo timer per questo giunto. */
	const bool is_tim1 = (joint >= SERVO_PITCH);
	const uint32_t min_us  = is_tim1 ? _min_us_tim1  : _min_us_tim8;
	const uint32_t max_us  = is_tim1 ? _max_us_tim1  : _max_us_tim8;
	const uint32_t max_deg = is_tim1 ? _max_deg_tim1 : _max_deg_tim8;

	uint32_t logical_deg = (angle_deg > max_deg) ? max_deg : (uint32_t)angle_deg;
	servo_angles[joint] = (uint8_t)CLAMP(logical_deg, 0U, 255U);

	/* pulse_us = min_us + (deg / max_deg) * (max_us - min_us) */
	uint32_t span_us = (max_us > min_us) ? (max_us - min_us) : 0U;
	uint32_t pulse_us = min_us + (logical_deg * span_us) / max_deg;
	pulse_us = CLAMP(pulse_us, min_us, max_us);

	const uint32_t period_hz = servo_get_period_hz(joint);
	const int ret = servo_apply_pulse_us(&servo_pwms[joint], pulse_us, period_hz);

	if (ret != 0) {
		LOG_ERR("[SERVO] Failed to set PWM for joint %d: %d", joint, ret);
		printk("[SERVO_ERR] pwm_set_fail joint=%d ret=%d pulse_us=%u cmd=%u\n", (int)joint, ret,
		       (unsigned)pulse_us, (unsigned)angle_deg);
		return false;
	}

	servo_last_pulse_us[joint] = (uint16_t)pulse_us;
	servo_last_sent_angle[joint] = logical_deg;

	if (joint == SERVO_YAW || joint == SERVO_PITCH || joint == SERVO_SPALLA) {
		static uint32_t pwm_us_log = 0;

		if ((pwm_us_log++ % 50U) == 0U) {
			printk("[PWM_US] joint=%d log_deg=%u pulse_us=%u cmd=%u\n", (int)joint,
			       (unsigned)logical_deg, (unsigned)pulse_us, (unsigned)logical_deg);
		}
	}

	static uint8_t last_logged[SERVO_COUNT] = {255, 255, 255, 255, 255, 255};
	static uint32_t last_log_time[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
	static uint32_t log_time_counter = 0;

	log_time_counter++;

	int diff = (int)last_logged[joint] - (int)logical_deg;
	if (diff < 0) {
		diff = -diff;
	}

	if ((last_logged[joint] == 255 || diff >= 5) &&
	    (log_time_counter - last_log_time[joint] >= 500)) {
		const char *joint_names[] = {"BASE", "SPALLA", "GOMITO", "YAW", "PITCH", "ROLL"};

		LOG_INF("[SERVO] %s = %u deg log (PWM: %u us)", joint_names[joint], logical_deg,
			pulse_us);
		last_logged[joint] = logical_deg;
		last_log_time[joint] = log_time_counter;
	}

	return true;
}

/**
 * servo_set_angle_f — versione float per movimento sub-degree.
 * Calcolo PWM con matematica float: nessuna quantizzazione a 1°.
 * Usato dal trajectory tick a 1 kHz per fluidità massima delle interpolazioni.
 */
bool servo_set_angle_f(servo_joint_t joint, float angle_deg)
{
	if (joint >= SERVO_COUNT) { return false; }
	if (!servo_ready[joint])  { return false; }

	const bool is_tim1 = (joint >= SERVO_PITCH);
	const float min_us  = (float)(is_tim1 ? _min_us_tim1  : _min_us_tim8);
	const float max_us  = (float)(is_tim1 ? _max_us_tim1  : _max_us_tim8);
	const float max_deg = (float)(is_tim1 ? _max_deg_tim1 : _max_deg_tim8);

	if (angle_deg < 0.0f)         { angle_deg = 0.0f; }
	if (angle_deg > max_deg)      { angle_deg = max_deg; }

	/* Update angolo logico (uint8 per servo_get_angle compatibility) */
	servo_angles[joint] = (uint8_t)(angle_deg + 0.5f);

	/* PWM pulse calculato in float: pulse_us = min + (deg/max_deg) * span */
	float span_us = (max_us > min_us) ? (max_us - min_us) : 0.0f;
	float pulse_us_f = min_us + (angle_deg * span_us) / max_deg;
	if (pulse_us_f < min_us) { pulse_us_f = min_us; }
	if (pulse_us_f > max_us) { pulse_us_f = max_us; }

	/* Conversione cycles_per_sec/1M con float per non perdere precisione sub-us.
	 * Riusa servo_apply_pulse_us con valore arrotondato all'us più vicino — ma
	 * dato che il timer ha 84 cicli/us su F446 (TIM8) e 84 cicli/us su TIM1,
	 * ogni us corrisponde a ~84 cicli, quindi la risoluzione minima del PWM
	 * (1 ciclo = ~12 ns ≈ 0.012 us) è già sub-degree.
	 * Per ottenere risoluzione decimale del pulse passiamo PWM cycles direttamente. */
	uint64_t cycles_per_sec = 0;
	const struct pwm_dt_spec *spec = &servo_pwms[joint];
	int ret = pwm_get_cycles_per_sec(spec->dev, spec->channel, &cycles_per_sec);
	if (ret < 0 || cycles_per_sec == 0) { return false; }

	const uint32_t period_hz = servo_get_period_hz(joint);
	if (period_hz == 0U) { return false; }

	uint64_t period_cycles = cycles_per_sec / (uint64_t)period_hz;
	/* pulse_cycles = cycles_per_sec * pulse_us / 1e6, calcolato in double per
	 * preservare la frazione sub-us (es. 1503.7 us → 126310 cicli a 84 MHz). */
	double pulse_cycles_d = ((double)cycles_per_sec * (double)pulse_us_f) / 1.0e6;
	uint64_t pulse_cycles = (uint64_t)(pulse_cycles_d + 0.5);

	/* Stesso adattamento prescaler dinamico di compute_period_and_pulse */
	if (period_cycles > SERVO_TIMER_MAX) {
		uint64_t prescale = (period_cycles + SERVO_TIMER_MAX) / (SERVO_TIMER_MAX + 1ULL);
		if (prescale == 0) { prescale = 1; }
		period_cycles /= prescale;
		pulse_cycles  /= prescale;
	}
	if (period_cycles == 0)              { return false; }
	if (pulse_cycles > period_cycles)    { pulse_cycles = period_cycles; }

	int rc = pwm_set_cycles(spec->dev, spec->channel,
	                        (uint32_t)period_cycles, (uint32_t)pulse_cycles, spec->flags);
	if (rc != 0) {
		LOG_ERR("[SERVO_F] pwm_set_cycles fail joint=%d ret=%d", (int)joint, rc);
		return false;
	}
	servo_last_pulse_us[joint] = (uint16_t)(pulse_us_f + 0.5f);
	servo_last_sent_angle[joint] = (uint8_t)(angle_deg + 0.5f);
	return true;
}

uint16_t servo_get_last_pulse_us(servo_joint_t joint)
{
	if (joint >= SERVO_COUNT) {
		return 0;
	}
	return servo_last_pulse_us[joint];
}

uint8_t servo_get_angle(servo_joint_t joint)
{
	if (joint >= SERVO_COUNT) {
		return 90;
	}
	return servo_angles[joint];
}

void servo_disable_all(void)
{
	for (int i = 0; i < SERVO_COUNT; i++) {
		servo_last_sent_angle[i] = 255;

		if (!servo_ready[i]) {
			continue;
		}
		int ret = servo_apply_pulse_us(&servo_pwms[i], 0, servo_get_period_hz((servo_joint_t)i));

		if (ret != 0) {
			LOG_ERR("[SERVO] disable servo %d failed (ret=%d)", i, ret);
			printk("[SERVO_ERR] disable_fail servo=%d ret=%d\n", i, ret);
		}
	}
}

void servo_relax_digital(void)
{
	for (int i = 0; i < SERVO_COUNT; i++) {
		if ((DIGITAL_SERVO_MASK & (1U << i)) == 0U) {
			continue;
		}
		if (!servo_ready[i]) {
			continue;
		}
		(void)servo_apply_pulse_us(&servo_pwms[i], 0, servo_get_period_hz((servo_joint_t)i));
		servo_last_sent_angle[i] = 255;
		servo_last_pulse_us[i] = 0;
	}
}

const uint8_t *servo_get_offset_deg(void)
{
	return _servo_offset_deg;
}

const uint8_t *servo_get_teleop_deg(void)
{
	return _servo_teleop_deg;
}

void servo_offset_set(const uint8_t values[SERVO_COUNT])
{
	for (int i = 0; i < SERVO_COUNT; i++) {
		_servo_offset_deg[i] = values[i];
	}
	printk("[SERVO] offset meccanici aggiornati: %u %u %u %u %u %u\n", _servo_offset_deg[0],
	       _servo_offset_deg[1], _servo_offset_deg[2], _servo_offset_deg[3], _servo_offset_deg[4],
	       _servo_offset_deg[5]);
}
