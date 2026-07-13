/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PITCH_PWM */
#define PITCH_PWM_INST                                                     TIMG0
#define PITCH_PWM_INST_IRQHandler                               TIMG0_IRQHandler
#define PITCH_PWM_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PITCH_PWM_INST_CLK_FREQ                                          5000000
/* GPIO defines for channel 0 */
#define GPIO_PITCH_PWM_C0_PORT                                             GPIOA
#define GPIO_PITCH_PWM_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_PITCH_PWM_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PITCH_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PITCH_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX

/* Defines for YAW_PWM */
#define YAW_PWM_INST                                                       TIMG6
#define YAW_PWM_INST_IRQHandler                                 TIMG6_IRQHandler
#define YAW_PWM_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define YAW_PWM_INST_CLK_FREQ                                            5000000
/* GPIO defines for channel 0 */
#define GPIO_YAW_PWM_C0_PORT                                               GPIOB
#define GPIO_YAW_PWM_C0_PIN                                        DL_GPIO_PIN_6
#define GPIO_YAW_PWM_C0_IOMUX                                    (IOMUX_PINCM23)
#define GPIO_YAW_PWM_C0_IOMUX_FUNC                   IOMUX_PINCM23_PF_TIMG6_CCP0
#define GPIO_YAW_PWM_C0_IDX                                  DL_TIMER_CC_0_INDEX




/* Defines for OLED */
#define OLED_INST                                                           I2C1
#define OLED_INST_IRQHandler                                     I2C1_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C1_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 400000
#define GPIO_OLED_SDA_PORT                                                 GPIOB
#define GPIO_OLED_SDA_PIN                                          DL_GPIO_PIN_3
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM16)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOB
#define GPIO_OLED_SCL_PIN                                          DL_GPIO_PIN_2
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM15)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for TRACKING_UART */
#define TRACKING_UART_INST                                                 UART0
#define TRACKING_UART_INST_FREQUENCY                                    40000000
#define TRACKING_UART_INST_IRQHandler                           UART0_IRQHandler
#define TRACKING_UART_INST_INT_IRQN                               UART0_INT_IRQn
#define GPIO_TRACKING_UART_RX_PORT                                         GPIOA
#define GPIO_TRACKING_UART_TX_PORT                                         GPIOA
#define GPIO_TRACKING_UART_RX_PIN                                 DL_GPIO_PIN_31
#define GPIO_TRACKING_UART_TX_PIN                                 DL_GPIO_PIN_28
#define GPIO_TRACKING_UART_IOMUX_RX                               (IOMUX_PINCM6)
#define GPIO_TRACKING_UART_IOMUX_TX                               (IOMUX_PINCM3)
#define GPIO_TRACKING_UART_IOMUX_RX_FUNC                IOMUX_PINCM6_PF_UART0_RX
#define GPIO_TRACKING_UART_IOMUX_TX_FUNC                IOMUX_PINCM3_PF_UART0_TX
#define TRACKING_UART_BAUD_RATE                                         (230400)
#define TRACKING_UART_IBRD_40_MHZ_230400_BAUD                               (10)
#define TRACKING_UART_FBRD_40_MHZ_230400_BAUD                               (54)
/* Defines for IMU601 */
#define IMU601_INST                                                        UART3
#define IMU601_INST_FREQUENCY                                           80000000
#define IMU601_INST_IRQHandler                                  UART3_IRQHandler
#define IMU601_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_IMU601_RX_PORT                                                GPIOA
#define GPIO_IMU601_TX_PORT                                                GPIOA
#define GPIO_IMU601_RX_PIN                                        DL_GPIO_PIN_25
#define GPIO_IMU601_TX_PIN                                        DL_GPIO_PIN_26
#define GPIO_IMU601_IOMUX_RX                                     (IOMUX_PINCM55)
#define GPIO_IMU601_IOMUX_TX                                     (IOMUX_PINCM59)
#define GPIO_IMU601_IOMUX_RX_FUNC                      IOMUX_PINCM55_PF_UART3_RX
#define GPIO_IMU601_IOMUX_TX_FUNC                      IOMUX_PINCM59_PF_UART3_TX
#define IMU601_BAUD_RATE                                                (115200)
#define IMU601_IBRD_80_MHZ_115200_BAUD                                      (43)
#define IMU601_FBRD_80_MHZ_115200_BAUD                                      (26)
/* Defines for CHASSIS_UART */
#define CHASSIS_UART_INST                                                  UART1
#define CHASSIS_UART_INST_FREQUENCY                                     40000000
#define CHASSIS_UART_INST_IRQHandler                            UART1_IRQHandler
#define CHASSIS_UART_INST_INT_IRQN                                UART1_INT_IRQn
#define GPIO_CHASSIS_UART_RX_PORT                                          GPIOA
#define GPIO_CHASSIS_UART_TX_PORT                                          GPIOA
#define GPIO_CHASSIS_UART_RX_PIN                                   DL_GPIO_PIN_9
#define GPIO_CHASSIS_UART_TX_PIN                                   DL_GPIO_PIN_8
#define GPIO_CHASSIS_UART_IOMUX_RX                               (IOMUX_PINCM20)
#define GPIO_CHASSIS_UART_IOMUX_TX                               (IOMUX_PINCM19)
#define GPIO_CHASSIS_UART_IOMUX_RX_FUNC                IOMUX_PINCM20_PF_UART1_RX
#define GPIO_CHASSIS_UART_IOMUX_TX_FUNC                IOMUX_PINCM19_PF_UART1_TX
#define CHASSIS_UART_BAUD_RATE                                          (115200)
#define CHASSIS_UART_IBRD_40_MHZ_115200_BAUD                                (21)
#define CHASSIS_UART_FBRD_40_MHZ_115200_BAUD                                (45)





/* Port definition for Pin Group MOTOR_DIR */
#define MOTOR_DIR_PORT                                                   (GPIOA)

/* Defines for MOTOR1_DIR: GPIOA.16 with pinCMx 38 on package pin 9 */
#define MOTOR_DIR_MOTOR1_DIR_PIN                                (DL_GPIO_PIN_16)
#define MOTOR_DIR_MOTOR1_DIR_IOMUX                               (IOMUX_PINCM38)
/* Defines for MOTOR2_DIR: GPIOA.17 with pinCMx 39 on package pin 10 */
#define MOTOR_DIR_MOTOR2_DIR_PIN                                (DL_GPIO_PIN_17)
#define MOTOR_DIR_MOTOR2_DIR_IOMUX                               (IOMUX_PINCM39)
/* Defines for MOTOR1_ENABLE: GPIOA.18 with pinCMx 40 on package pin 11 */
#define MOTOR_DIR_MOTOR1_ENABLE_PIN                             (DL_GPIO_PIN_18)
#define MOTOR_DIR_MOTOR1_ENABLE_IOMUX                            (IOMUX_PINCM40)
/* Defines for MOTOR2_ENABLE: GPIOA.15 with pinCMx 37 on package pin 8 */
#define MOTOR_DIR_MOTOR2_ENABLE_PIN                             (DL_GPIO_PIN_15)
#define MOTOR_DIR_MOTOR2_ENABLE_IOMUX                            (IOMUX_PINCM37)
/* Port definition for Pin Group LINE_SENSOR */
#define LINE_SENSOR_PORT                                                 (GPIOB)

/* Defines for LINE0: GPIOB.0 with pinCMx 12 on package pin 47 */
#define LINE_SENSOR_LINE0_PIN                                    (DL_GPIO_PIN_0)
#define LINE_SENSOR_LINE0_IOMUX                                  (IOMUX_PINCM12)
/* Defines for LINE1: GPIOB.1 with pinCMx 13 on package pin 48 */
#define LINE_SENSOR_LINE1_PIN                                    (DL_GPIO_PIN_1)
#define LINE_SENSOR_LINE1_IOMUX                                  (IOMUX_PINCM13)
/* Defines for LINE2: GPIOB.4 with pinCMx 17 on package pin 52 */
#define LINE_SENSOR_LINE2_PIN                                    (DL_GPIO_PIN_4)
#define LINE_SENSOR_LINE2_IOMUX                                  (IOMUX_PINCM17)
/* Defines for LINE3: GPIOB.5 with pinCMx 18 on package pin 53 */
#define LINE_SENSOR_LINE3_PIN                                    (DL_GPIO_PIN_5)
#define LINE_SENSOR_LINE3_IOMUX                                  (IOMUX_PINCM18)
/* Defines for LINE4: GPIOB.7 with pinCMx 24 on package pin 59 */
#define LINE_SENSOR_LINE4_PIN                                    (DL_GPIO_PIN_7)
#define LINE_SENSOR_LINE4_IOMUX                                  (IOMUX_PINCM24)
/* Defines for LINE5: GPIOB.8 with pinCMx 25 on package pin 60 */
#define LINE_SENSOR_LINE5_PIN                                    (DL_GPIO_PIN_8)
#define LINE_SENSOR_LINE5_IOMUX                                  (IOMUX_PINCM25)
/* Defines for LINE6: GPIOB.9 with pinCMx 26 on package pin 61 */
#define LINE_SENSOR_LINE6_PIN                                    (DL_GPIO_PIN_9)
#define LINE_SENSOR_LINE6_IOMUX                                  (IOMUX_PINCM26)
/* Defines for LINE7: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LINE_SENSOR_LINE7_PIN                                   (DL_GPIO_PIN_10)
#define LINE_SENSOR_LINE7_IOMUX                                  (IOMUX_PINCM27)
/* Defines for LINE8: GPIOB.11 with pinCMx 28 on package pin 63 */
#define LINE_SENSOR_LINE8_PIN                                   (DL_GPIO_PIN_11)
#define LINE_SENSOR_LINE8_IOMUX                                  (IOMUX_PINCM28)
/* Defines for LINE9: GPIOB.12 with pinCMx 29 on package pin 64 */
#define LINE_SENSOR_LINE9_PIN                                   (DL_GPIO_PIN_12)
#define LINE_SENSOR_LINE9_IOMUX                                  (IOMUX_PINCM29)
/* Defines for LINE10: GPIOB.13 with pinCMx 30 on package pin 1 */
#define LINE_SENSOR_LINE10_PIN                                  (DL_GPIO_PIN_13)
#define LINE_SENSOR_LINE10_IOMUX                                 (IOMUX_PINCM30)
/* Defines for LINE11: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LINE_SENSOR_LINE11_PIN                                  (DL_GPIO_PIN_14)
#define LINE_SENSOR_LINE11_IOMUX                                 (IOMUX_PINCM31)
/* Port definition for Pin Group BUTTONS */
#define BUTTONS_PORT                                                     (GPIOA)

/* Defines for MODE: GPIOA.3 with pinCMx 8 on package pin 43 */
#define BUTTONS_MODE_PIN                                         (DL_GPIO_PIN_3)
#define BUTTONS_MODE_IOMUX                                        (IOMUX_PINCM8)
/* Defines for UP: GPIOA.4 with pinCMx 9 on package pin 44 */
#define BUTTONS_UP_PIN                                           (DL_GPIO_PIN_4)
#define BUTTONS_UP_IOMUX                                          (IOMUX_PINCM9)
/* Defines for DOWN: GPIOA.5 with pinCMx 10 on package pin 45 */
#define BUTTONS_DOWN_PIN                                         (DL_GPIO_PIN_5)
#define BUTTONS_DOWN_IOMUX                                       (IOMUX_PINCM10)
/* Port definition for Pin Group EXTRA_BUTTONS */
#define EXTRA_BUTTONS_PORT                                               (GPIOA)

/* Defines for KEY1: GPIOA.6 with pinCMx 11 on package pin 46 */
#define EXTRA_BUTTONS_KEY1_PIN                                   (DL_GPIO_PIN_6)
#define EXTRA_BUTTONS_KEY1_IOMUX                                 (IOMUX_PINCM11)
/* Defines for KEY2: GPIOA.7 with pinCMx 14 on package pin 49 */
#define EXTRA_BUTTONS_KEY2_PIN                                   (DL_GPIO_PIN_7)
#define EXTRA_BUTTONS_KEY2_IOMUX                                 (IOMUX_PINCM14)
/* Defines for KEY3: GPIOA.10 with pinCMx 21 on package pin 56 */
#define EXTRA_BUTTONS_KEY3_PIN                                  (DL_GPIO_PIN_10)
#define EXTRA_BUTTONS_KEY3_IOMUX                                 (IOMUX_PINCM21)
/* Defines for KEY4: GPIOA.11 with pinCMx 22 on package pin 57 */
#define EXTRA_BUTTONS_KEY4_PIN                                  (DL_GPIO_PIN_11)
#define EXTRA_BUTTONS_KEY4_IOMUX                                 (IOMUX_PINCM22)
/* Defines for KEY5: GPIOA.13 with pinCMx 35 on package pin 6 */
#define EXTRA_BUTTONS_KEY5_PIN                                  (DL_GPIO_PIN_13)
#define EXTRA_BUTTONS_KEY5_IOMUX                                 (IOMUX_PINCM35)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PITCH_PWM_init(void);
void SYSCFG_DL_YAW_PWM_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_TRACKING_UART_init(void);
void SYSCFG_DL_IMU601_init(void);
void SYSCFG_DL_CHASSIS_UART_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
