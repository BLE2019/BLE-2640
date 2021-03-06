/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
/** ============================================================================
 *  @file       CC2650_LAUNCHXL.h
 *
 *  @brief      CC2650 LaunchPad Board Specific header file.
 *
 *  NB! This is the board file for CC2650 LaunchPad PCB version 1.1
 *
 *  ============================================================================
 */
#ifndef __CC2650_LAUNCHXL_BOARD_H__
#define __CC2650_LAUNCHXL_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Same RF Configuration as 7x7 EM */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

/* Discrete outputs */
#define Board_RLED                  IOID_6
#define Board_GLED                  IOID_7
#define Board_LED_ON                1
#define Board_LED_OFF               0

/* Motor outputs */
#define Board_MOTOR                 IOID_18

/* Discrete inputs */
#define Board_BTN1                  IOID_13
#define Board_BTN2                  IOID_14

#define Board_ACC_INT               IOID_15   
/* I2C */
#define Board_I2C0_SCL0             IOID_4
#define Board_I2C0_SDA0             IOID_5

/* PWM outputs */
#define Board_PWMBuzzer             IOID_19
#define Board_PWMRED                IOID_20
#define Board_PWMGREEN              IOID_21
#define Board_PWMBLUE               IOID_22

/* ADC outputs */   
#define Board_DIO30_ADC             IOID_30   
   
/* SPI Board *///FOR LCD data and control
#define Board_SPI0_MISO             IOID_8
#define Board_SPI0_MOSI             IOID_9  //from 9 to 26
#define Board_SPI0_CLK              IOID_27
#define Board_SPI0_CSN              PIN_UNASSIGNED
#define                                                                                                                                                                              Board_SPI_FLASH_CSN         IOID_12

/* LCD Control pin */
#define Board_LCD_MODE              IOID_1   
#define Board_LCD_CSN               IOID_29 //from 11 to 29
#define Board_3V3_EN                IOID_0

///////////nb-iot bc26 data and control pin////////////1029
/* UART Board */// for NB B26, 1029                                                                                                                                                                
#define Board_UART_RX               IOID_3
#define Board_UART_TX               IOID_2
   
#define Board_NB_PEN                IOID_24
#define Board_NB_RST                IOID_25
#define Board_NB_RI                 IOID_26
/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC2650_LAUNCHXL_I2C0
   
/* Generic SPI instance identifiers */
#define Board_SPI                   CC2650_LAUNCHXL_SPI0

/* Generic UART instance identifiers */
#define Board_UART                  CC2650_LAUNCHXL_UART0

/* Generic PWM instance identifiers */
#define Board_PWMR                  CC2650_LAUNCHXL_PWM0
#define Board_PWMG                  CC2650_LAUNCHXL_PWM1
#define Board_PWMB                  CC2650_LAUNCHXL_PWM2
#define Board_PWMBUZZER             CC2650_LAUNCHXL_PWM3
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC2650_LAUNCHXL_CRYPTO0

/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             CC2650_LAUNCHXL_GPTIMER0A
#define Board_GPTIMER0B             CC2650_LAUNCHXL_GPTIMER0B
#define Board_GPTIMER1A             CC2650_LAUNCHXL_GPTIMER1A
#define Board_GPTIMER1B             CC2650_LAUNCHXL_GPTIMER1B
#define Board_GPTIMER2A             CC2650_LAUNCHXL_GPTIMER2A
#define Board_GPTIMER2B             CC2650_LAUNCHXL_GPTIMER2B
#define Board_GPTIMER3A             CC2650_LAUNCHXL_GPTIMER3A
#define Board_GPTIMER3B             CC2650_LAUNCHXL_GPTIMER3B
/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650_LAUNCHXL_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_I2CName {
    CC2650_LAUNCHXL_I2C0 = 0,

    CC2650_LAUNCHXL_I2CCOUNT
} CC2650_LAUNCHXL_I2CName;

/*!
 *  @def    CC2650_LAUNCHXL_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_CryptoName {
    CC2650_LAUNCHXL_CRYPTO0 = 0,

    CC2650_LAUNCHXL_CRYPTOCOUNT
} CC2650_LAUNCHXL_CryptoName;


/*!
 *  @def    CC2650_LAUNCHXL_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_SPIName {
    CC2650_LAUNCHXL_SPI0 = 0,
    CC2650_LAUNCHXL_SPI1,

    CC2650_LAUNCHXL_SPICOUNT
} CC2650_LAUNCHXL_SPIName;

/*!
 *  @def    CC2650_LAUNCHXL_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2650_LAUNCHXL_TRNGName {
    CC2650_LAUNCHXL_TRNG0 = 0,
    CC2650_LAUNCHXL_TRNGCOUNT
} CC2650_LAUNCHXL_TRNGName;

/*!
 *  @def    CC2650_LAUNCHXL_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650_LAUNCHXL_UARTName {
    CC2650_LAUNCHXL_UART0 = 0,

    CC2650_LAUNCHXL_UARTCOUNT
} CC2650_LAUNCHXL_UARTName;

/*!
 *  @def    CC2650_LAUNCHXL_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650_LAUNCHXL_UdmaName {
    CC2650_LAUNCHXL_UDMA0 = 0,

    CC2650_LAUNCHXL_UDMACOUNT
} CC2650_LAUNCHXL_UdmaName;

/*!
 *  @def    CC2650_LAUNCHXL_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2650_LAUNCHXL_GPTimerName
{
    CC2650_LAUNCHXL_GPTIMER0A = 0,
    CC2650_LAUNCHXL_GPTIMER0B,
    CC2650_LAUNCHXL_GPTIMER1A,
    CC2650_LAUNCHXL_GPTIMER1B,
    CC2650_LAUNCHXL_GPTIMER2A,
    CC2650_LAUNCHXL_GPTIMER2B,
    CC2650_LAUNCHXL_GPTIMER3A,
    CC2650_LAUNCHXL_GPTIMER3B,
    CC2650_LAUNCHXL_GPTIMERPARTSCOUNT
} CC2650_LAUNCHXL_GPTimerName;

/*!
 *  @def    CC2650_LAUNCHXL_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2650_LAUNCHXL_GPTimers
{
    CC2650_LAUNCHXL_GPTIMER0 = 0,
    CC2650_LAUNCHXL_GPTIMER1,
    CC2650_LAUNCHXL_GPTIMER2,
    CC2650_LAUNCHXL_GPTIMER3,
    CC2650_LAUNCHXL_GPTIMERCOUNT
} CC2650_LAUNCHXL_GPTimers;

/*!
 *  @def    CC2650_LAUNCHXL_PWM
 *  @brief  Enum of PWM outputs on the board
 */
typedef enum CC2650_LAUNCHXL_PWM
{
    CC2650_LAUNCHXL_PWM0 = 0,
    CC2650_LAUNCHXL_PWM1,
    CC2650_LAUNCHXL_PWM2,
    CC2650_LAUNCHXL_PWM3,
    CC2650_LAUNCHXL_PWMCOUNT
} CC2650_LAUNCHXL_PWM;

/*!
 *  @def    CC2650_LAUNCHXL_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum CC2650_LAUNCHXL_ADCBufName {
    CC2650_LAUNCHXL_ADCBuf0 = 0,
    CC2650_LAUNCHXL_ADCBufCOUNT
} CC2650_LAUNCHXL_ADCBufName;


/*!
 *  @def    CC2650_LAUNCHXL_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC2650_LAUNCHXL_ADCName {
    CC2650_LAUNCHXL_ADC0 = 0,
    CC2650_LAUNCHXL_ADCDCOUPL,
    CC2650_LAUNCHXL_ADCVSS,
    CC2650_LAUNCHXL_ADCVDDS,
    CC2650_LAUNCHXL_ADCCOUNT
} CC2650_LAUNCHXL_ADCName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2650_LAUNCHXL_BOARD_H__ */
