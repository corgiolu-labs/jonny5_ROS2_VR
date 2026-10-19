/*
 * hw_watchdog.c — vedi hw_watchdog.h.
 */
#include "core/hw_watchdog.h"

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

#if IS_ENABLED(CONFIG_J5_HW_WATCHDOG) && DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)

#include <zephyr/drivers/watchdog.h>

static const struct device *const j5_wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int j5_wdt_channel = -1;

int hw_watchdog_init(void)
{
    if (!device_is_ready(j5_wdt)) {
        printk("[WDT] device not ready\n");
        return -ENODEV;
    }
    const struct wdt_timeout_cfg cfg = {
        .window = { .min = 0U, .max = CONFIG_J5_HW_WATCHDOG_TIMEOUT_MS },
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };
    const int ch = wdt_install_timeout(j5_wdt, &cfg);
    if (ch < 0) {
        printk("[WDT] install_timeout failed: %d\n", ch);
        return ch;
    }
    /* In debug (core fermato su breakpoint) l'IWDG si mette in pausa. */
    const int rc = wdt_setup(j5_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (rc < 0) {
        printk("[WDT] setup failed: %d\n", rc);
        return rc;
    }
    j5_wdt_channel = ch;
    printk("[WDT] IWDG armed, timeout %u ms\n", (unsigned)CONFIG_J5_HW_WATCHDOG_TIMEOUT_MS);
    return 0;
}

void hw_watchdog_feed(void)
{
    if (j5_wdt_channel >= 0) {
        (void)wdt_feed(j5_wdt, j5_wdt_channel);
    }
}

#else

int hw_watchdog_init(void)
{
    return 1;
}

void hw_watchdog_feed(void)
{
}

#endif
