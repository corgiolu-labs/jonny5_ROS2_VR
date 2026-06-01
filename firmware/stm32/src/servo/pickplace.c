/*
 * pickplace.c — Pick & Place actuator driver (vedi pickplace.h).
 *
 * Implementazione minimale basata su pwm-leds DT alias (pwm-pp1, pwm-pp2)
 * definiti in firmware/stm32/zephyr/overlays/pwm.overlay.
 */

#include "servo/pickplace.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pickplace, LOG_LEVEL_INF);

/* Periodo PWM in ns: 50000 ns = 20 kHz. DEVE coincidere con quanto dichiarato
 * negli alias pwm-pp1/pwm-pp2 dell'overlay (period è encoded nel pwm-cell). */
#define PICKPLACE_PERIOD_NS   50000U

static const struct pwm_dt_spec pp_pwms[PICKPLACE_CH_COUNT] = {
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_pp1)),  /* PP1 = PA0 TIM2_CH1 (elettrovalvola) */
	PWM_DT_SPEC_GET(DT_ALIAS(pwm_pp2)),  /* PP2 = PA1 TIM2_CH2 (vacuum motors) */
};

static bool pp_ready[PICKPLACE_CH_COUNT] = { false, false };
static uint8_t pp_last_duty[PICKPLACE_CH_COUNT] = { 0U, 0U };

static const char *pp_label(uint8_t ch)
{
	return (ch == PICKPLACE_CH_PP1) ? "PP1" : (ch == PICKPLACE_CH_PP2) ? "PP2" : "PP?";
}

static int pp_apply_duty(uint8_t ch, uint8_t duty)
{
	const struct pwm_dt_spec *spec = &pp_pwms[ch];
	uint32_t pulse_ns = ((uint32_t)PICKPLACE_PERIOD_NS * (uint32_t)duty) / 100U;

	int ret = pwm_set_dt(spec, PICKPLACE_PERIOD_NS, pulse_ns);
	if (ret != 0) {
		LOG_ERR("[PP] %s pwm_set_dt failed (ret=%d duty=%u)",
			pp_label(ch), ret, (unsigned)duty);
		printk("[PP_ERR] %s pwm_set_fail ret=%d duty=%u\n",
		       pp_label(ch), ret, (unsigned)duty);
	}
	return ret;
}

bool pickplace_init(void)
{
	bool all_ok = true;

	for (uint8_t i = 0U; i < PICKPLACE_CH_COUNT; i++) {
		const struct pwm_dt_spec *spec = &pp_pwms[i];

		if (!device_is_ready(spec->dev)) {
			LOG_ERR("[PP] %s PWM device not ready", pp_label(i));
			printk("[PP_ERR] %s device_not_ready\n", pp_label(i));
			pp_ready[i] = false;
			all_ok = false;
			continue;
		}
		pp_ready[i] = true;

		/* Boot: duty 0 → MOSFET OFF immediato. */
		(void)pp_apply_duty(i, 0U);
		pp_last_duty[i] = 0U;
	}

	if (all_ok) {
		printk("[PP] init OK — PP1=PA0 PP2=PA1 @ 20 kHz duty=0\n");
		LOG_INF("[PP] init OK (PP1=PA0 PP2=PA1 @ 20 kHz, duty=0)");
	} else {
		LOG_ERR("[PP] init: some PWM devices not ready");
	}
	return all_ok;
}

bool pickplace_set_duty(uint8_t channel, uint8_t duty_0_100)
{
	if (channel >= PICKPLACE_CH_COUNT) {
		LOG_ERR("[PP] invalid channel %u", (unsigned)channel);
		return false;
	}
	if (!pp_ready[channel]) {
		LOG_ERR("[PP] %s not ready", pp_label(channel));
		return false;
	}
	uint8_t duty = (uint8_t)CLAMP((unsigned)duty_0_100,
				      (unsigned)PICKPLACE_DUTY_MIN,
				      (unsigned)PICKPLACE_DUTY_MAX);
	if (pp_apply_duty(channel, duty) != 0) {
		return false;
	}
	pp_last_duty[channel] = duty;
	LOG_INF("[PP] %s duty=%u%%", pp_label(channel), (unsigned)duty);
	return true;
}

uint8_t pickplace_get_duty(uint8_t channel)
{
	if (channel >= PICKPLACE_CH_COUNT) {
		return 0U;
	}
	return pp_last_duty[channel];
}

void pickplace_safe_off(void)
{
	for (uint8_t i = 0U; i < PICKPLACE_CH_COUNT; i++) {
		if (!pp_ready[i]) {
			continue;
		}
		(void)pp_apply_duty(i, 0U);
		pp_last_duty[i] = 0U;
	}
	LOG_INF("[PP] safe_off — entrambi i canali a 0");
}
