cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/uart.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/i2c.overlay
    # Backend IMU: BNO085 (i2c_read/write raw da src/imu/bno085.c, nessun nodo DT).
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/pwm.overlay
    # TIM6 = sorgente di tick hardware a 1 kHz per il RT loop di controllo
    # (vedi rt_timer.overlay e src/core/rt_loop.c). NB: questa lista, non
    # board_build.zephyr.overlays in platformio.ini, e' la fonte effettiva
    # degli overlay applicati al devicetree.
    ${CMAKE_CURRENT_SOURCE_DIR}/overlays/rt_timer.overlay
)
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
