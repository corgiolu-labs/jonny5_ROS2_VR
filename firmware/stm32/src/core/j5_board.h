/*
 * j5_board.h — risorse hardware della JONNY5 SHIELD rev3 (WeAct STM32G474CEU6).
 *
 *  - IMU BNO085 su I2C2: SDA PA8, SCL PC4 (mappatura alternata), 100 kHz, indirizzo 0x4B.
 *  - Control-plane verso Pi/Aura: USART3 PB10 (TX) / PB11 (RX), 115200 8N1.
 *  - Console Zephyr/printk: USART1 PA9/PA10 -> J10 -> ST-Link VCP.
 */
#ifndef J5_BOARD_H
#define J5_BOARD_H

#include <zephyr/devicetree.h>

/* ---- Bus I2C della IMU (BNO085) ---- */
#define J5_I2C_NODE          DT_NODELABEL(i2c2)
#define J5_I2C_BUS_NAME      "I2C2"
#define J5_I2C_SCL_PORT_NODE DT_NODELABEL(gpioc)
#define J5_I2C_SCL_PIN       4                     /* PC4 */
#define J5_I2C_SDA_PORT_NODE DT_NODELABEL(gpioa)
#define J5_I2C_SDA_PIN       8                     /* PA8 */

/* ---- UART control-plane verso il Raspberry Pi / Aura ---- */
#define J5_UART_NODE DT_NODELABEL(usart3)

#endif /* J5_BOARD_H */
