/*
 * Fatal diagnostics - Zephyr k_sys_fatal_error_handler override
 *
 * USART1 è dedicata esclusivamente al control plane (comandi/risposte verso il Pi).
 * In caso di fatal error si usa solo la console (USART2 / ST-Link VCP) per non
 * corrompere il canale control plane.
 */

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/printk.h>

volatile uint32_t g_fatal_count = 0;
volatile uint32_t g_fatal_reason = 0xFFFFFFFFu;

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
    (void)esf;
    g_fatal_count++;
    g_fatal_reason = (uint32_t)reason;

    /* Solo console (USART2 / ST-Link VCP). USART1 resta dedicata al control plane. */
    printk("[FATAL] reason=%u count=%u\n", (unsigned)reason, (unsigned)g_fatal_count);

    k_fatal_halt(reason);
}

