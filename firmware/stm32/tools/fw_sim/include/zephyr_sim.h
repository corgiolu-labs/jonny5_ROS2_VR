/*
 * zephyr_sim.h — the few Zephyr APIs the JONNY5 firmware logic uses, emulated on a PC.
 *
 * Only what is needed to compile src/ (minus the HAL SPI DMA and the BNO085 driver)
 * on the host: kernel time, spinlocks, semaphores (non-blocking), atomics, logging,
 * devicetree macros resolved to fixed simulator pins, GPIO, UART IRQ API, PWM, counter.
 * The simulated hardware lives in sim_hw.c.
 */
#ifndef ZEPHYR_SIM_H
#define ZEPHYR_SIM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

/* ---- Kconfig (as in zephyr/prj.conf) ---- */
#ifndef CONFIG_J5_CANONICAL_PADDED_128_MODE
#define CONFIG_J5_CANONICAL_PADDED_128_MODE 1
#endif
#define CONFIG_SOC_SERIES_STM32G4X 1

#define Z_SIM_XXXX1 Z_SIM_YYYY,
#define IS_ENABLED(config_macro) Z_SIM_IS_ENABLED1(config_macro)
#define Z_SIM_IS_ENABLED1(config_macro) Z_SIM_IS_ENABLED2(Z_SIM_XXXX##config_macro)
#define Z_SIM_IS_ENABLED2(one_or_two_args) Z_SIM_IS_ENABLED3(one_or_two_args 1, 0)
#define Z_SIM_IS_ENABLED3(ignore_this, val, ...) val

/* ---- util ---- */
#define ARG_UNUSED(x) (void)(x)
#define BUILD_ASSERT(cond, ...) _Static_assert(cond, "" __VA_ARGS__)
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#define CLAMP(val, low, high) (((val) <= (low)) ? (low) : MIN(val, high))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define __unused __attribute__((unused))
#define __packed __attribute__((packed))
#define SYS_INIT(fn, level, prio)

/* ---- logging / printk ---- */
int sim_printk(const char *fmt, ...);
#define printk sim_printk
#define LOG_MODULE_REGISTER(...)
#define LOG_MODULE_DECLARE(...)
#define LOG_INF(...) sim_log('I', __VA_ARGS__)
#define LOG_WRN(...) sim_log('W', __VA_ARGS__)
#define LOG_ERR(...) sim_log('E', __VA_ARGS__)
#define LOG_DBG(...) ((void)0)
void sim_log(char level, const char *fmt, ...);

/* ---- time ---- */
typedef struct { int64_t ms; } k_timeout_t;
#define K_MSEC(x) ((k_timeout_t){ (int64_t)(x) })
#define K_USEC(x) ((k_timeout_t){ 0 })
#define K_NO_WAIT ((k_timeout_t){ 0 })
#define K_FOREVER ((k_timeout_t){ -1 })
int64_t  k_uptime_get(void);
uint32_t k_uptime_get_32(void);
int32_t  k_sleep(k_timeout_t t);
int32_t  k_msleep(int32_t ms);
int32_t  k_usleep(int32_t us);
uint32_t k_cycle_get_32(void);
#define sys_clock_hw_cycles_per_sec() 170000000U
#define k_cyc_to_us_floor32(c) ((uint32_t)((c) / 170U))

/* ---- spinlock ---- */
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l) { (void)l; return 0; }
static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t k) { (void)l; (void)k; }

/* ---- semaphores / threads (never block, threads never start) ---- */
struct k_sem { unsigned int count; unsigned int limit; };
static inline int k_sem_init(struct k_sem *s, unsigned int c, unsigned int l) { s->count = c; s->limit = l; return 0; }
static inline int k_sem_take(struct k_sem *s, k_timeout_t t) { (void)t; if (s->count) { s->count--; return 0; } return -EAGAIN; }
static inline void k_sem_give(struct k_sem *s) { if (s->count < s->limit) s->count++; }
static inline unsigned int k_sem_count_get(struct k_sem *s) { return s->count; }
#define K_SEM_DEFINE(name, init, lim) struct k_sem name = { (init), (lim) }
struct k_thread { int unused; };
typedef struct k_thread *k_tid_t;
typedef char k_thread_stack_t;
#define K_THREAD_STACK_DEFINE(name, size) static k_thread_stack_t name[size]
#define K_THREAD_STACK_SIZEOF(s) sizeof(s)
#define K_PRIO_PREEMPT(x) (x)
#define K_PRIO_COOP(x) (-(x))
typedef void (*k_thread_entry_t)(void *, void *, void *);
static inline k_tid_t k_thread_create(struct k_thread *t, k_thread_stack_t *stack, size_t sz,
                                      k_thread_entry_t entry, void *a, void *b, void *c,
                                      int prio, uint32_t opts, k_timeout_t delay)
{ (void)stack; (void)sz; (void)entry; (void)a; (void)b; (void)c; (void)prio; (void)opts; (void)delay; return t; }
static inline int k_thread_name_set(k_tid_t t, const char *n) { (void)t; (void)n; return 0; }
static inline void k_fatal_halt(unsigned int reason) { (void)reason; }
static inline void k_yield(void) {}

/* ---- atomics ---- */
typedef long atomic_t;
typedef long atomic_val_t;
#define ATOMIC_INIT(x) (x)
static inline atomic_val_t atomic_get(const atomic_t *a) { return *a; }
static inline atomic_val_t atomic_set(atomic_t *a, atomic_val_t v) { atomic_val_t o = *a; *a = v; return o; }
static inline atomic_val_t atomic_or(atomic_t *a, atomic_val_t v) { atomic_val_t o = *a; *a |= v; return o; }
static inline atomic_val_t atomic_inc(atomic_t *a) { return (*a)++; }
static inline bool atomic_cas(atomic_t *a, atomic_val_t o, atomic_val_t n) { if (*a == o) { *a = n; return true; } return false; }

/* ---- devicetree: aliases resolve to simulator pins / PWM channels ---- */
#define DT_ALIAS(x) Z_SIM_ALIAS_##x
#define Z_SIM_ALIAS_estop 6          /* PC6 E-STOP */
#define Z_SIM_ALIAS_sw0 13           /* PC13 centre button */
#define Z_SIM_ALIAS_pwm_servo0 0
#define Z_SIM_ALIAS_pwm_servo1 1
#define Z_SIM_ALIAS_pwm_servo2 2
#define Z_SIM_ALIAS_pwm_servo3 3
#define Z_SIM_ALIAS_pwm_servo4 4
#define Z_SIM_ALIAS_pwm_servo5 5
#define DT_NODELABEL(x) 0
/* in #if an alias with no Z_SIM_ALIAS_* definition evaluates to 0 (node absent, e.g. pwm_pp1) */
#define DT_NODE_EXISTS(node) (node)
#define DT_NODE_HAS_STATUS(node, status) 1
#define DT_GPIO_CTLR(node, prop) 0
#define DT_GPIO_PIN(node, prop) (node)
#define DT_GPIO_FLAGS(node, prop) 0

struct device { const char *name; };
extern const struct device sim_device;
#define DEVICE_DT_GET(node) (&sim_device)
static inline bool device_is_ready(const struct device *d) { return d != NULL; }

/* ---- GPIO ---- */
typedef uint8_t gpio_pin_t;
typedef uint32_t gpio_flags_t;
#define GPIO_INPUT (1U << 16)
#define GPIO_OUTPUT (1U << 17)
#define GPIO_OPEN_DRAIN (1U << 1)
#define GPIO_PULL_UP (1U << 4)
#define GPIO_PULL_DOWN (1U << 5)
#define GPIO_ACTIVE_LOW (1U << 0)
#define GPIO_ACTIVE_HIGH 0
int sim_gpio_get(unsigned int pin);
static inline int gpio_pin_configure(const struct device *d, gpio_pin_t p, gpio_flags_t f) { (void)d; (void)p; (void)f; return 0; }
static inline int gpio_pin_get(const struct device *d, gpio_pin_t p) { (void)d; return sim_gpio_get(p); }
static inline int gpio_pin_set(const struct device *d, gpio_pin_t p, int v) { (void)d; (void)p; (void)v; return 0; }

/* ---- UART (IRQ RX + poll TX) ---- */
typedef void (*uart_irq_callback_user_data_t)(const struct device *dev, void *user_data);
void sim_uart_set_cb(uart_irq_callback_user_data_t cb, void *user);
int  sim_uart_rx_pending(void);
int  sim_uart_rx_pop(uint8_t *c);
void sim_uart_tx_byte(unsigned char c);
static inline int uart_irq_callback_user_data_set(const struct device *d, uart_irq_callback_user_data_t cb, void *u) { (void)d; sim_uart_set_cb(cb, u); return 0; }
static inline void uart_irq_rx_enable(const struct device *d) { (void)d; }
static inline int uart_irq_update(const struct device *d) { (void)d; return 1; }
static inline int uart_irq_rx_ready(const struct device *d) { (void)d; return sim_uart_rx_pending(); }
static inline int uart_fifo_read(const struct device *d, uint8_t *buf, int n) { (void)d; return (n > 0 && sim_uart_rx_pop(buf)) ? 1 : 0; }
static inline void uart_poll_out(const struct device *d, unsigned char c) { (void)d; sim_uart_tx_byte(c); }

/* ---- ring buffer ---- */
struct ring_buf { uint8_t *buf; uint32_t size, head, tail, count; };
static inline void ring_buf_init(struct ring_buf *r, uint32_t size, uint8_t *data) { r->buf = data; r->size = size; r->head = r->tail = r->count = 0; }
static inline uint32_t ring_buf_put(struct ring_buf *r, const uint8_t *d, uint32_t n)
{ uint32_t i = 0; for (; i < n && r->count < r->size; i++) { r->buf[r->head] = d[i]; r->head = (r->head + 1) % r->size; r->count++; } return i; }
static inline uint32_t ring_buf_get(struct ring_buf *r, uint8_t *d, uint32_t n)
{ uint32_t i = 0; for (; i < n && r->count > 0; i++) { d[i] = r->buf[r->tail]; r->tail = (r->tail + 1) % r->size; r->count--; } return i; }

/* ---- PWM (timers at 1 MHz like the G474 build: prescaler 169) ---- */
typedef uint32_t pwm_flags_t;
struct pwm_dt_spec { const struct device *dev; uint32_t channel; uint32_t period; pwm_flags_t flags; };
#define PWM_DT_SPEC_GET(node) { .dev = &sim_device, .channel = (node), .period = 0, .flags = 0 }
int sim_pwm_set(uint32_t channel, uint32_t period_cycles, uint32_t pulse_cycles);
static inline int pwm_get_cycles_per_sec(const struct device *d, uint32_t ch, uint64_t *cps) { (void)d; (void)ch; *cps = 1000000ULL; return 0; }
static inline int pwm_set_cycles(const struct device *d, uint32_t ch, uint32_t period, uint32_t pulse, pwm_flags_t f) { (void)d; (void)f; return sim_pwm_set(ch, period, pulse); }
static inline bool pwm_is_ready_dt(const struct pwm_dt_spec *s) { (void)s; return true; }

/* ---- counter (RT tick timer, never started in the simulator) ---- */
struct counter_top_cfg { uint32_t ticks; void (*callback)(const struct device *, void *); void *user_data; uint32_t flags; };
static inline uint32_t counter_us_to_ticks(const struct device *d, uint64_t us) { (void)d; return (uint32_t)us; }
static inline int counter_set_top_value(const struct device *d, const struct counter_top_cfg *c) { (void)d; (void)c; return 0; }
static inline int counter_start(const struct device *d) { (void)d; return 0; }
static inline uint32_t counter_get_frequency(const struct device *d) { (void)d; return 1000000U; }

/* ---- sensor (types only, for imu.h) ---- */
struct sensor_value { int32_t val1; int32_t val2; };

#endif /* ZEPHYR_SIM_H */
