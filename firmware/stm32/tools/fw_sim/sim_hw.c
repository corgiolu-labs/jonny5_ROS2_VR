/*
 * sim_hw.c — simulated board for the JONNY5 firmware logic (PC, no hardware).
 *
 * The firmware sources compiled with it are the real ones from src/: state machine,
 * E-STOP, RT loop step, J5 protocol + SPI frame logic, UART control plane, actuation,
 * HEAD, SETPOSE, servo_control (real pulse math on a 1 MHz PWM like the G474 timers).
 * Replaced here: the clock, GPIO pins, the UART device, the PWM peripheral, the SPI
 * circular DMA (emulated byte by byte) and the BNO085 (an IMU you set from the test).
 *
 * The sim_* functions below are the C API used by the Python tests (ctypes).
 */
#include <stdarg.h>
#include <stdlib.h>

#include "zephyr_sim.h"
#include "core/state_machine.h"
#include "core/estop.h"
#include "core/rt_loop.h"
#include "spi/j5_protocol.h"
#include "spi/spi_frames.h"
#include "uart/uart_control.h"
#include "servo/servo_control.h"
#include "servo/j5vr_setpose.h"
#include "imu/imu.h"

#if defined(_WIN32)
#define SIM_API __declspec(dllexport)
#else
#define SIM_API __attribute__((visibility("default")))
#endif

const struct device sim_device = { "sim" };

/* from core/fatal_diag.c (not compiled: no fatal handler on the PC) */
volatile uint32_t g_fatal_count = 0;
volatile uint32_t g_fatal_reason = 0xFFFFFFFFu;

/* ---------------- clock ---------------- */
static int64_t s_now_ms;
int64_t k_uptime_get(void) { return s_now_ms; }
uint32_t k_uptime_get_32(void) { return (uint32_t)s_now_ms; }
int32_t k_sleep(k_timeout_t t) { (void)t; return 0; }
int32_t k_msleep(int32_t ms) { (void)ms; return 0; }
int32_t k_usleep(int32_t us) { (void)us; return 0; }
uint32_t k_cycle_get_32(void) { return (uint32_t)(s_now_ms * 170000); }

/* ---------------- console ---------------- */
static int s_verbose;
static char s_console[1 << 16];
static size_t s_console_len;

static void console_append(const char *txt)
{
    size_t n = strlen(txt);
    if (s_console_len + n >= sizeof(s_console)) s_console_len = 0;   /* simple wrap */
    memcpy(s_console + s_console_len, txt, n);
    s_console_len += n;
    s_console[s_console_len] = 0;
    if (s_verbose) fputs(txt, stdout);
}

int sim_printk(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    console_append(buf);
    return n;
}

void sim_log(char level, const char *fmt, ...)
{
    char buf[512];
    int k = snprintf(buf, sizeof(buf), "<%c> ", level);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf + k, sizeof(buf) - (size_t)k - 1, fmt, ap);
    va_end(ap);
    strncat(buf, "\n", sizeof(buf) - strlen(buf) - 1);
    console_append(buf);
}

/* ---------------- GPIO ---------------- */
static int s_gpio[32];
int sim_gpio_get(unsigned int pin) { return pin < 32 ? s_gpio[pin] : 0; }

/* ---------------- UART ---------------- */
static uart_irq_callback_user_data_t s_uart_cb;
static void *s_uart_user;
static uint8_t s_rx_q[4096];
static size_t s_rx_head, s_rx_tail;
static char s_tx[1 << 15];
static size_t s_tx_len;

void sim_uart_set_cb(uart_irq_callback_user_data_t cb, void *user) { s_uart_cb = cb; s_uart_user = user; }
int sim_uart_rx_pending(void) { return s_rx_head != s_rx_tail; }
int sim_uart_rx_pop(uint8_t *c)
{
    if (s_rx_head == s_rx_tail) return 0;
    *c = s_rx_q[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % sizeof(s_rx_q);
    return 1;
}
void sim_uart_tx_byte(unsigned char c)
{
    if (s_tx_len + 1 < sizeof(s_tx)) { s_tx[s_tx_len++] = (char)c; s_tx[s_tx_len] = 0; }
}

/* ---------------- PWM ---------------- */
static uint32_t s_pwm_period[8], s_pwm_pulse[8];
int sim_pwm_set(uint32_t ch, uint32_t period, uint32_t pulse)
{
    if (ch < 8) { s_pwm_period[ch] = period; s_pwm_pulse[ch] = pulse; }
    return 0;
}

/* ---------------- IMU (BNO085 replaced) ---------------- */
static int s_imu_valid;
static float s_imu_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static uint32_t s_imu_counter;
int imu_init(void) { return s_imu_valid ? 0 : -1; }
void imu_update_orientation(float dt_s) { (void)dt_s; }
void imu_get_quaternion(struct imu_quat *out) { out->w = s_imu_q[0]; out->x = s_imu_q[1]; out->y = s_imu_q[2]; out->z = s_imu_q[3]; }
bool imu_get_snapshot(imu_snapshot_t *out)
{
    if (!s_imu_valid) return false;
    memset(out, 0, sizeof(*out));
    out->quat_w = s_imu_q[0]; out->quat_x = s_imu_q[1]; out->quat_y = s_imu_q[2]; out->quat_z = s_imu_q[3];
    out->accel_z = 9.81f;
    out->temp = 30.0f;
    out->sample_counter = ++s_imu_counter;
    out->timestamp_us = (uint32_t)(s_now_ms * 1000);
    return true;
}
void imu_i2c_bus_recovery(void) {}
bool imu_is_orientation_valid(void) { return s_imu_valid != 0; }
void imu_clear_orientation_state(void) {}
bool imu_is_available(void) { return s_imu_valid != 0; }

/* ---------------- SPI: circular DMA emulated byte by byte ---------------- */
static uint8_t s_spi_rx[2 * SPI_FRAME_LEN];
static uint8_t s_spi_tx[2 * SPI_FRAME_LEN];
static size_t s_spi_pos;

static void spi_reset_buffers(void)
{
    /* as hal_spi_slave_init(): both TX halves hold a STATUS frame (seq 0 and 1), tail zero */
    memset(s_spi_rx, 0, sizeof(s_spi_rx));
    memset(s_spi_tx, 0, sizeof(s_spi_tx));
    for (int i = 0; i < 2; i++) {
        j5_frame_t f;
        j5_build_frame(&f, J5_FRAME_TYPE_STATUS, (uint16_t)i);
        memcpy(&s_spi_tx[i * SPI_FRAME_LEN], &f, sizeof(f));
    }
    s_spi_pos = 0;
    spi_frames_reset();
}

/* ---------------- firmware glue (rt_loop.c is compiled in sim_rt.c) ---------------- */
void sim_rt_tick_once(void);   /* sim_rt.c: estop_poll_tick() + rt_loop_step() */

/* =================== API for the tests =================== */

SIM_API int sim_frame_len(void) { return (int)SPI_FRAME_LEN; }

SIM_API void sim_set_verbose(int on) { s_verbose = on; }

/** Boot like main(): state machine, UART, E-STOP, RT loop init, BOOT_READY. */
SIM_API void sim_boot(void)
{
    s_now_ms = 0;
    memset(s_gpio, 0, sizeof(s_gpio));
    s_gpio[13] = 1;            /* centre button: logical 1 = not pressed for this firmware */
    s_rx_head = s_rx_tail = 0;
    s_tx_len = 0; s_tx[0] = 0;
    s_console_len = 0; s_console[0] = 0;
    spi_reset_buffers();
    state_machine_init();
    uart_control_init();
    estop_init();
    rt_loop_init();
    uart_send_unsolicited("BOOT_READY");
}

/** Advance the simulated time by `ms` milliseconds: 1 kHz RT ticks, main loop every 10 ms. */
SIM_API void sim_run_ms(int ms)
{
    for (int i = 0; i < ms; i++) {
        s_now_ms++;
        sim_rt_tick_once();
        if ((s_now_ms % 10) == 0) {
            uart_control_process();
            estop_notify_poll();
            j5vr_setpose_notify_poll();
        }
    }
}

/** Full-duplex SPI transfer of `len` bytes (any length: the DMA is emulated byte by byte). */
SIM_API void sim_spi_transfer(const uint8_t *mosi, uint8_t *miso, int len)
{
    for (int i = 0; i < len; i++) {
        s_spi_rx[s_spi_pos] = mosi[i];
        miso[i] = s_spi_tx[s_spi_pos];
        s_spi_pos++;
        if (s_spi_pos == SPI_FRAME_LEN) {
            spi_frames_ingest_half(&s_spi_rx[0], &s_spi_tx[SPI_FRAME_LEN]);
        } else if (s_spi_pos == 2 * SPI_FRAME_LEN) {
            spi_frames_ingest_half(&s_spi_rx[SPI_FRAME_LEN], &s_spi_tx[0]);
            s_spi_pos = 0;
        }
    }
}

/** Bytes as if the Pi wrote them on the control UART. */
SIM_API void sim_uart_write(const char *data)
{
    for (const char *p = data; *p; p++) {
        s_rx_q[s_rx_head] = (uint8_t)*p;
        s_rx_head = (s_rx_head + 1) % sizeof(s_rx_q);
    }
    if (s_uart_cb) s_uart_cb(&sim_device, s_uart_user);   /* RX interrupt */
}

/** Everything the firmware sent on the control UART since the last call. */
SIM_API int sim_uart_read(char *out, int cap)
{
    int n = (int)s_tx_len < cap - 1 ? (int)s_tx_len : cap - 1;
    memcpy(out, s_tx, (size_t)n);
    out[n] = 0;
    memmove(s_tx, s_tx + n, s_tx_len - (size_t)n + 1);
    s_tx_len -= (size_t)n;
    return n;
}

SIM_API int sim_console_read(char *out, int cap)
{
    int n = (int)s_console_len < cap - 1 ? (int)s_console_len : cap - 1;
    memcpy(out, s_console, (size_t)n);
    out[n] = 0;
    s_console_len = 0;
    return n;
}

SIM_API void sim_set_gpio(int pin, int level) { if (pin >= 0 && pin < 32) s_gpio[pin] = level; }

SIM_API void sim_set_imu(int valid, float w, float x, float y, float z)
{
    s_imu_valid = valid; s_imu_q[0] = w; s_imu_q[1] = x; s_imu_q[2] = y; s_imu_q[3] = z;
}

/** Last PWM written per channel (0..5): pulse in microseconds (0 = output off) and period. */
SIM_API void sim_pwm(uint32_t pulse_us[6], uint32_t period_us[6])
{
    for (int i = 0; i < 6; i++) { pulse_us[i] = s_pwm_pulse[i]; period_us[i] = s_pwm_period[i]; }
}

SIM_API void sim_servo_angles(float out[6])
{
    for (int i = 0; i < 6; i++) out[i] = servo_get_angle_f((servo_joint_t)i);
}

SIM_API int sim_state(void) { return (int)state_machine_get_state(); }
SIM_API int sim_estop_active(void) { return estop_is_active() ? 1 : 0; }
SIM_API int64_t sim_now_ms(void) { return s_now_ms; }
