#ifndef _CAR_BOARD_H_
#define _CAR_BOARD_H_

/*
 * L2 BSP - 天猛星 MSPM0G3507 核心板 + V1.0 扩展板资源映射。
 *
 * 来源：模块资料/天猛星扩展板v1.0.pdf。业务层不得直接使用这些宏。
 */
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"

#define CAR_BOARD_PWM_LEFT                    (PWM_TIM_A0_CH0_B14)
#define CAR_BOARD_PWM_RIGHT                   (PWM_TIM_A0_CH1_A7)

#define CAR_BOARD_MOTOR_LEFT_IN1              (B9)
#define CAR_BOARD_MOTOR_LEFT_IN2              (B10)
#define CAR_BOARD_MOTOR_RIGHT_IN1             (B7)
#define CAR_BOARD_MOTOR_RIGHT_IN2             (B6)

#define CAR_BOARD_ENCODER_LEFT_A              (B11)
#define CAR_BOARD_ENCODER_LEFT_B              (B12)
#define CAR_BOARD_ENCODER_RIGHT_A             (B4)
#define CAR_BOARD_ENCODER_RIGHT_B             (B5)

#define CAR_BOARD_GRAY_DATA                   (A25)
#define CAR_BOARD_GRAY_CLOCK                  (A27)

#define CAR_BOARD_OLED_SDA                    (A0)
#define CAR_BOARD_OLED_SCL                    (A1)

#define CAR_BOARD_KEY_1                       (B0)
#define CAR_BOARD_KEY_2                       (B22)
#define CAR_BOARD_KEY_3                       (B18)
#define CAR_BOARD_KEY_4                       (B24)

#define CAR_BOARD_SCHEDULER_PIT               (PIT_TIM_G12)

/* 无刷电机沿用源工程 UART1 PA8/PA9，MaixCAM 视觉沿用 UART2 PB16。 */
#define CAR_BOARD_BLDC_UART                    (UART_1)
#define CAR_BOARD_BLDC_UART_TX                 (UART1_TX_A8)
#define CAR_BOARD_BLDC_UART_RX                 (UART1_RX_A9)

#define CAR_BOARD_VISION_UART                  (UART_2)
#define CAR_BOARD_VISION_UART_TX               (UART2_TX_B15)
#define CAR_BOARD_VISION_UART_RX               (UART2_RX_B16)

#endif
