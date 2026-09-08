cmake_minimum_required(VERSION 3.13.1)
# Workaround spazi nel path ("PROGETTI GITHUB"): il preprocess devicetree di
# Zephyr splitta DTC_OVERLAY_FILE sugli spazi, e PlatformIO risolve la junction
# nel path reale. Se il path contiene spazi ed esiste la junction senza spazi
# (C:\Users\pinco\J5, vedi platformio.ini), usa quella per gli overlay.
set(J5_ZDIR "${CMAKE_CURRENT_SOURCE_DIR}")
if(J5_ZDIR MATCHES " " AND EXISTS "C:/Users/pinco/J5/firmware/stm32/zephyr")
    set(J5_ZDIR "C:/Users/pinco/J5/firmware/stm32/zephyr")
    # Anche CONF_FILE/APPLICATION_CONFIG_DIR arrivano dalla cache di PlatformIO
    # col path reale (spazioso): kconfig.cmake li splitta sugli spazi.
    set(CONF_FILE "${J5_ZDIR}/prj.conf" CACHE STRING "prj.conf via junction senza spazi" FORCE)
    set(APPLICATION_CONFIG_DIR "${J5_ZDIR}" CACHE PATH "config dir via junction senza spazi" FORCE)
endif()
# Overlay per board: la SHIELD rev3 (G474) ha pinatura e clock propri.
if(BOARD MATCHES "nucleo_g474re")
    set(J5_OVL "${J5_ZDIR}/overlays_g474")
    set(DTC_OVERLAY_FILE
        "${J5_OVL}/board.overlay"
        "${J5_OVL}/uart.overlay"
        "${J5_OVL}/i2c.overlay"
        "${J5_OVL}/pwm.overlay"
        "${J5_OVL}/rt_timer.overlay"
        # Fungo E-STOP su PC6 (SHIELD rev3): alias "estop" per src/core/estop.c
        "${J5_OVL}/gpio_estop.overlay"
    )
else()
set(DTC_OVERLAY_FILE
    "${J5_ZDIR}/overlays/uart.overlay"
    "${J5_ZDIR}/overlays/i2c.overlay"
    # Backend IMU: BNO085 (i2c_read/write raw da src/imu/bno085.c, nessun nodo DT).
    "${J5_ZDIR}/overlays/pwm.overlay"
    # TIM6 = sorgente di tick hardware a 1 kHz per il RT loop di controllo
    # (vedi rt_timer.overlay e src/core/rt_loop.c). NB: questa lista, non
    # board_build.zephyr.overlays in platformio.ini, e' la fonte effettiva
    # degli overlay applicati al devicetree.
    "${J5_ZDIR}/overlays/rt_timer.overlay"
)
endif()
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(JONNY5_3.0)

# Path assoluti: app in src/ con struttura modulare
get_filename_component(APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
target_include_directories(app PRIVATE 
    ${APP_ROOT}/src 
    ${APP_ROOT}/src/spi 
    ${APP_ROOT}/src/uart 
    ${APP_ROOT}/src/core 
    ${APP_ROOT}/src/common
    ${APP_ROOT}/src/servo
    ${APP_ROOT}/src/imu
)

# Compilazione ricorsiva di tutti i file .c in src/
file(GLOB_RECURSE APP_SOURCES ${APP_ROOT}/src/*.c)
target_sources(app PRIVATE ${APP_SOURCES})
