#ifndef __APP_BOARD_H__
#define __APP_BOARD_H__

#include "headfile.h"

/*
 * Board-level hardware mapping.
 * Change pins, UART ports and PWM channels here first when wiring changes.
 */

/* UART devices */
#define BOARD_UART_DEBUG          UART_1
#define BOARD_UART_TRACK          UART_1
#define BOARD_UART_MOTOR          UART_2

#define BOARD_UART_DEBUG_BAUD     115200
#define BOARD_UART_TRACK_BAUD     9600
#define BOARD_UART_MOTOR_BAUD     115200

/* I2C sensors: current ml_i2c uses PB10(SCL), PB11(SDA). */
#define BOARD_I2C_SCL_PORT        GPIO_B
#define BOARD_I2C_SCL_PIN         Pin_10
#define BOARD_I2C_SDA_PORT        GPIO_B
#define BOARD_I2C_SDA_PIN         Pin_11

/* MPU6050 interrupt pin used by the existing attitude code. */
#define BOARD_MPU_INT_PORT        GPIO_A
#define BOARD_MPU_INT_PIN         Pin_7

/* Optional user key default pin. Change to match your board. */
#define BOARD_KEY1_PORT           GPIO_C
#define BOARD_KEY1_PIN            Pin_13
#define BOARD_KEY_ACTIVE_LEVEL    0

/* Optional servo test output. TIM3_CH1 maps to PA6 in ml_pwm.c. */
#define BOARD_SERVO_TIM           TIM_3
#define BOARD_SERVO_CH            TIM3_CH1
#define BOARD_SERVO_FREQ_HZ       50

/* Optional ADC test channels. */
#define BOARD_ADC_UNIT            ADC_1
#define BOARD_ADC_CH0             ADC_Channel_0
#define BOARD_ADC_CH1             ADC_Channel_1

/* Optional software SPI pins. Change before using with real hardware. */
#define BOARD_SPI_SCK_PORT        GPIO_B
#define BOARD_SPI_SCK_PIN         Pin_13
#define BOARD_SPI_MISO_PORT       GPIO_B
#define BOARD_SPI_MISO_PIN        Pin_14
#define BOARD_SPI_MOSI_PORT       GPIO_B
#define BOARD_SPI_MOSI_PIN        Pin_15
#define BOARD_SPI_CS_PORT         GPIO_B
#define BOARD_SPI_CS_PIN          Pin_12

/* Optional AB encoder test pins. Change before wiring a real encoder. */
#define BOARD_ENCODER_A_PORT      GPIO_A
#define BOARD_ENCODER_A_PIN       Pin_0
#define BOARD_ENCODER_B_PORT      GPIO_A
#define BOARD_ENCODER_B_PIN       Pin_1

#endif
