/*
 * j5_board.h — selezione risorse hardware per piattaforma.
 *
 * Piattaforme supportate:
 *  - Nucleo-F446RE (robot v1):    I2C1 PB8/PB9, control-plane Pi su USART1
 *  - JONNY5 SHIELD rev3 (G474):   I2C2 PA8(SDA)/PC4(SCL) mappatura alternata,
 *                                 control-plane Pi su USART3 (PB10/PB11),
 *                                 USART1 (PA9/PA10, J10) = console/debug.
 *
 * La selezione avviene dal devicetree: se il nodo della shield e' "okay"
 * si usa quello, altrimenti fallback alla configurazione F446 storica.
 */
#ifndef J5_BOARD_H
#define J5_BOARD_H

#include <zephyr/devicetree.h>

/* ---- Bus I2C della IMU (BNO085) ---- */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
#define J5_I2C_NODE          DT_NODELABEL(i2c2)
#define J5_I2C_BUS_NAME      "I2C2"
#define J5_I2C_SCL_PORT_NODE DT_NODELABEL(gpioc)
#define J5_I2C_SCL_PIN       4                     /* PC4 */
#define J5_I2C_SDA_PORT_NODE DT_NODELABEL(gpioa)
#define J5_I2C_SDA_PIN       8                     /* PA8 */
#else
#define J5_I2C_NODE          DT_NODELABEL(i2c1)
#define J5_I2C_BUS_NAME      "I2C1"
#define J5_I2C_SCL_PORT_NODE DT_NODELABEL(gpiob)
#define J5_I2C_SCL_PIN       8                     /* PB8 */
#define J5_I2C_SDA_PORT_NODE DT_NODELABEL(gpiob)
#define J5_I2C_SDA_PIN       9                     /* PB9 */
#endif

/* ---- UART control-plane verso il Raspberry Pi ---- */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart3), okay)
#define J5_UART_NODE DT_NODELABEL(usart3)
#else
#define J5_UART_NODE DT_NODELABEL(usart1)
#endif

#endif /* J5_BOARD_H */
