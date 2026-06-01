/*
 * JONNY5-4.0 - Bootstrap
 * Pipeline: HAL SPI slave + boundary + state machine + RT loop + UART control + IMU + servo.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include "spi/boundary_buffers.h"
#include "spi/hal_spi_slave.h"
#include "core/state_machine.h"
#include "core/rt_loop.h"
#include "uart/uart_control.h"
#include "imu/imu.h"
#include "servo/servo_control.h"
#include "servo/pickplace.h"
#include <zephyr/sys/printk.h>

/* Boundary init prima di HAL SPI (POST_KERNEL 50) */
static int boundary_init_sys_init(void)
{
    boundary_init();
    return 0;
}

SYS_INIT(boundary_init_sys_init, PRE_KERNEL_1, 1);

int main(void)
{
    printk("[BOOT] JONNY5-4.0\n");

    if (!hal_spi_slave_is_ready()) {
        printk("[BOOT] SPI not ready - SAFE\n");
        return 0;
    }

    state_machine_init();
    uart_control_init();
    (void)pickplace_init();   /* PA0/PA1 → MOSFET gate; duty=0 al boot */

    /* imu_init() NON viene chiamata qui: il thread IMU in rt_loop aspetta
     * che RT sia a regime (>2s) e che arrivi IMUON dal Pi, poi fa l'init.
     * Così main() non si blocca se il sensore non risponde sul bus I2C. */

    rt_loop_init();
    rt_loop_start();
    uart_send_unsolicited("BOOT_READY");

    printk("[BOOT] RT loop 1kHz started\n");

    while (1) {
        uart_control_process();
        k_msleep(10);
    }
    return 0;
}
