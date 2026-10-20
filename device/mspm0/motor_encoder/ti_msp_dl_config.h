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
 *  DEMO: manually adapted; see README. Originally generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

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



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_2
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM15)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM15_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_3
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (12499U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)





/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* B21 user key: GPIOB.21, IOMUX_PINCM49 (active low). */
#define KEY_key_PIN                                             (DL_GPIO_PIN_21)
#define KEY_key_IOMUX                                            (IOMUX_PINCM49)
/* Port definition for Pin Group LED */
#define STBY_PORT                                                         (GPIOA)

/* Defines for led: GPIOB.9 with pinCMx 26 on package pin 61 */
#define STBY_stby_PIN                                              (DL_GPIO_PIN_12)
#define STBY_stby_IOMUX                                            (IOMUX_PINCM34)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOA)

/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_16)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM38)
/* Defines for BIN2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_17)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM39)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_14)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM36)
/* Defines for AIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM35)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.25 with pinCMx 55 on package pin 26 */
// pins affected by this interrupt request:["E1A","E1B"]
#define ENCODERA_INT_IRQN                                       (GPIOA_INT_IRQn)
#define ENCODERA_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO25)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_25)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM55)
/* Defines for E1B: GPIOA.26 with pinCMx 59 on package pin 30 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO26)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_26)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM59)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.20 with pinCMx 48 on package pin 19 */
// pins affected by this interrupt request:["E2A","E2B"]
#define ENCODERB_INT_IRQN                                       (GPIOB_INT_IRQn)
#define ENCODERB_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO20)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_20)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM48)
/* Defines for E2B: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO24)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_24)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM52)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
