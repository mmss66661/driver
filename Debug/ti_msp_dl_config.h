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




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PITCH_PWM_init(void);
void SYSCFG_DL_YAW_PWM_init(void);
void SYSCFG_DL_TRACKING_UART_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
