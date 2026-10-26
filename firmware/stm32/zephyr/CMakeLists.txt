cmake_minimum_required(VERSION 3.13.1)
# Workaround spazi nel path ("PROGETTI GITHUB"): il preprocess devicetree di
# Zephyr splitta DTC_OVERLAY_FILE sugli spazi. Se il path contiene spazi si usa
# la junction senza spazi che punta a QUESTO checkout (una per worktree:
# J5 = JONNY5_ROS2, J5FW = JONNY5_ROS2_fw, J5MIN = JONNY5_ROS2_min).
set(J5_ZDIR "${CMAKE_CURRENT_SOURCE_DIR}")
if(J5_ZDIR MATCHES " ")
    # CMake REALPATH does not resolve Windows junctions: map the checkout folder name instead.
    if(J5_ZDIR MATCHES "/JONNY5_ROS2_min/")
        set(J5_CAND "C:/Users/pinco/J5MIN")
    elseif(J5_ZDIR MATCHES "/JONNY5_ROS2_fw/")
        set(J5_CAND "C:/Users/pinco/J5FW")
    else()
        set(J5_CAND "C:/Users/pinco/J5")
    endif()
    if(EXISTS "${J5_CAND}/firmware/stm32/zephyr")
        set(J5_ZDIR "${J5_CAND}/firmware/stm32/zephyr")
    endif()
    if(J5_ZDIR MATCHES " ")
        message(FATAL_ERROR "path with spaces and no junction to this checkout: create one (see platformio.ini)")
    endif()
    # Anche CONF_FILE/APPLICATION_CONFIG_DIR arrivano dalla cache di PlatformIO
    # col path reale (spazioso): kconfig.cmake li splitta sugli spazi.
    set(CONF_FILE "${J5_ZDIR}/prj.conf" CACHE STRING "prj.conf via junction senza spazi" FORCE)
    set(APPLICATION_CONFIG_DIR "${J5_ZDIR}" CACHE PATH "config dir via junction senza spazi" FORCE)
endif()
message(STATUS "JONNY5 overlays from ${J5_ZDIR}/overlays_g474")
# SHIELD rev3 (STM32G474, board Zephyr nucleo_g474re): unico target del firmware minimale.
set(J5_OVL "${J5_ZDIR}/overlays_g474")
set(DTC_OVERLAY_FILE
    "${J5_OVL}/board.overlay"
    "${J5_OVL}/uart.overlay"
    "${J5_OVL}/i2c.overlay"
    "${J5_OVL}/pwm.overlay"
    "${J5_OVL}/rt_timer.overlay"
    # Fungo E-STOP su PC6: alias "estop" per src/core/estop.c
    "${J5_OVL}/gpio_estop.overlay"
)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(JONNY5_minimal)

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
