/**
 * \file
 *         Source codes based on specified MCU to operate sx1278
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-04-09 15:58
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#if 0
    #include "stm8l15x_gpio.h"
    #include "stm8l15x_exti.h"
    #include "rtimer.h" 0709
    #include "rtimer-arch.h"
    #include "spi.h"
#endif
#include "Util.h"
//#include "main.h"
#include "sx1278_ports.h"
#include "hw_spi.h"
#include "board.h"
#include "hw_gpio.h"
#include "hw_uart.h"
/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/**
* @brief  SX1278 DIO pins I/O definitions
*/
#define DIO0_IOPORT    GPIOD
#define DIO0_PIN    GPIO_Pin_6
#define DIO0_EXTI    EXTI_Pin_6

#define DIO1_IOPORT    GPIOD
#define DIO1_PIN    GPIO_Pin_5
#define DIO1_EXTI    EXTI_Pin_5

#define DIO2_IOPORT    GPIOD
#define DIO2_PIN    GPIO_Pin_4

#define DIO3_IOPORT    GPIOB
#define DIO3_PIN    GPIO_Pin_2

#define DIO4_IOPORT    GPIOB
#define DIO4_PIN    GPIO_Pin_1

#define DIO5_IOPORT    GPIOB
#define DIO5_PIN    GPIO_Pin_0

/* SX1278 NRESET pin connect to PC0 */
#define LORA_RST_IOPORT    GPIOC
#define LORA_RST_PIN    GPIO_Pin_0

/* 0=set RF to low power by turn off OCO, 1=set RF to run mode by turn on OCO */
#define LORA_IOVCC_PORT    GPIOF
#define LORA_IOVCC_PIN    GPIO_Pin_0
#define LORA_RXE_PORT    GPIOE
#define LORA_RXE_PIN    GPIO_Pin_2
#define LORA_TXE_PORT    GPIOD
#define LORA_TXE_PIN    GPIO_Pin_7

/* Private variables ---------------------------------------------------------*/
//static struct rtimer    s_stRTimer;  //0706

/* Private function prototypes -----------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/


/**
 * @addtogroup  Access registers of SX1278 by SPI port
 * @{
 */

/**
  * @brief  Write a block data to continuous registers of LoRa.
  * @param  addr: adress of the 1th register.
  * @param  buffer: point to data buffer by this pointer.
  * @param  size: size of data in buffer.
  * @retval  None
  */
//***å**å**å*å**å**å*0706*****å*å*å*å*å*å***//
//PIN_Config  LoRaPinCfg[]=
//{
//
//
//
//}
void SX1278StartTimer(uint16_t wMs)
{
}
void SX1278StopTimer(void)
{
}
void SX1278InitTimer(void (*Callback)(void))
{
}
//------------------end of 0706---------------
void HwSPITrans1(uint8_t a, uint8_t *txbuf, uint8_t *rxbuf, uint16_t size)
{}

void SX1278WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
//   bool ret = false;
//    uint8_t addrData = addr | 0x80;
#if 1
    uint8_t rxbuf[50] = {0};
    uint8_t txbuf[50] = {0};
    uint8_t i;
    txbuf[0] = addr | 0x80;
    
    if(size < 50) 
    {
        for (i = 1; i <= size; i++) 
        {
            txbuf[i] = buffer[i - 1];
        }    
        i = size;
        HwSPITrans(Board_SPI_FLASH_CSN, txbuf, rxbuf, (uint16_t)size + 1);
    }
#else
    BordSPIOpen();
    ret = spi_write(Board_SPI_FLASH_CSN, &addrData, 1);
    if (ret == true && size > 0) {
        ret = spi_write(Board_SPI_FLASH_CSN, buffer, size);
    }
    BordSPIClose();
#endif
    return ;
}
void SX1278WriteBuffer1(uint8_t addr, uint8_t *buffer, uint8_t size)
{
//   bool ret = false;
//    uint8_t addrData = addr | 0x80;
#if 1
    uint8_t rxbuf[40] = {0};
    uint8_t txbuf[40] = {0};
    uint8_t i;
    txbuf[0] = addr | 0x80;

    if (size < 40) 
    {
        for (i = 1; i <= size ; i++) 
        {
            txbuf[i] = buffer[i - 1];
        }
        HwSPITrans(Board_SPI_FLASH_CSN, txbuf, rxbuf, size + 1);
    }
#else
    BordSPIOpen();
    ret = spi_write(Board_SPI_FLASH_CSN, &addrData, 1);
    if (ret == true && size > 0) {
        ret = spi_write(Board_SPI_FLASH_CSN, buffer, size);
    }
    BordSPIClose();
#endif
    return ;
}

/**
  * @brief  Read a block data from continuous registers of LoRa.
  * @param  addr: adress of the 1th register.
  * @param  buffer: point to data buffer by this pointer.
  * @param  size: size of data desired to read.
  * @retval  None
  */
void SX1278ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
  //  bool ret = false;
    uint8_t addrData = addr & 0x7f;
    //static uint8_t rxbuf[10] = {0};
#if 1
    uint8_t rxbuf[40] = {0};
    uint8_t i;
    HwSPITrans(Board_SPI_FLASH_CSN, &addrData, rxbuf, size + 1) ;

    for (i = 0; i < size; i++) {
        buffer[i] = rxbuf[i + 1];
    }
#else
    BordSPIOpen();
    ret = spi_write(Board_SPI_FLASH_CSN, &addrData, 1);

    if (ret == true) {
        ret = spi_read(Board_SPI_FLASH_CSN, rxbuf, size);
    }
    BordSPIClose();
#endif
    return ;
}

/**
 * @}
 */


/**
 * @addtogroup  Operate pins of MCU that connected to SX1278.
 * @{
 */

/**
  * @brief  Initialize pins of MCU for operating SX1278.
  * @param  None.
  * @retval  None
  */
#if 1 ///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!0706
void SX1278InitPins(void)
{
    HwGPIOSet(LoRa_SW_POWER, 1);
    HwGPIOSet(LoRa_SW_CTRL, 1);
    // HwGPIOSet(LoRa_TXRX,1);
#if 0
    halIntState_t    intState;
    spi_Init();
    /* ATTENTION: that MUST disable global interrupts when configure PIN to EXTI,
        otherwise incurred chaos of external interrupts. */
    HAL_ENTER_CRITICAL_SECTION(intState);
    /* Configure DIO0 and DIO1: input floating, external interrupt, rising edge trigger */
    GPIO_Init(DIO0_IOPORT, DIO0_PIN, GPIO_Mode_In_FL_IT);
    EXTI_SetPinSensitivity(DIO0_EXTI, EXTI_Trigger_Rising);
    GPIO_Init(DIO1_IOPORT, DIO1_PIN, GPIO_Mode_In_FL_IT);
    EXTI_SetPinSensitivity(DIO1_EXTI, EXTI_Trigger_Rising);
    HAL_EXIT_CRITICAL_SECTION(intState);
    GPIO_Init(LORA_IOVCC_PORT, LORA_IOVCC_PIN, GPIO_Mode_Out_PP_High_Slow);
    GPIO_Init(LORA_RXE_PORT, LORA_RXE_PIN, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(LORA_TXE_PORT, LORA_TXE_PIN, GPIO_Mode_Out_PP_Low_Slow);
#endif
    return;
}

/**
  * @brief  Pull pin to LOW for RESET SX1278.
  * @param  None.
  * @retval  None
  */
void SX1278PullLowResetPin(void)
{
    /* Pull low to RESET pin */
    HwGPIOSet(LoRa_nRST, 0);
#if 0
    GPIO_ResetBits(LORA_RST_IOPORT, LORA_RST_PIN);
#endif
    return;
}
void SX1278PullHighResetPin(void)
{
    HwGPIOSet(LoRa_nRST, 1);
}
/**
  * @brief  Set pin that connected to RESET of SX1278 to INPUT.
  * @param  None.
  * @retval  None
  */
void SX1278SetInputResetPin(void)
{
#if 0
    /* Configure pin that connect to LORA_RST of LoRa to input */
    GPIO_Init(LORA_RST_IOPORT, LORA_RST_PIN, GPIO_Mode_In_FL_No_IT);
#endif
    return;
}

/**
  * @brief  Set pin that connected to RESET of SX1278 to OUTPUT.
  * @param  None.
  * @retval  None
  */
void SX1278SetOutputResetPin(void)
{
#if 0
    /* Configure pin that connect to LORA_RST of LoRa to output */
    GPIO_Init(LORA_RST_IOPORT, LORA_RST_PIN, GPIO_Mode_Out_PP_High_Slow);
#endif
    return;
}



void SX1278EnterLowPower(void)
{
#if 0
    /* Turn off TCXO. */
    GPIO_ResetBits(LORA_IOVCC_PORT, LORA_IOVCC_PIN);
    /* Set SPI_NSS to LOW for RF sleep, otherwise incurred more energy! */
    SET_NSS_LOW();
    /* Set TXE and RXE to LOW for low power. */
    GPIO_ResetBits(LORA_TXE_PORT, LORA_TXE_PIN);
    GPIO_ResetBits(LORA_RXE_PORT, LORA_RXE_PIN);
#endif
    return;
}

void SX1278ExitLowPower(void)
{
#if 0
    /* Turn on TCXO. */
    GPIO_SetBits(LORA_IOVCC_PORT, LORA_IOVCC_PIN);
    /* Set SPI_NSS to HIGH for RF work */
    SET_NSS_HIGH();
#endif
    return;
}

void SX1278SetSwPin2Tx(void)
{
    HwGPIOSet(LoRa_SW_CTRL, 0);
    HwGPIOSet(LoRa_SW_POWER, 1);
#if 0
    GPIO_ResetBits(LORA_RXE_PORT, LORA_RXE_PIN);
    GPIO_SetBits(LORA_TXE_PORT, LORA_TXE_PIN);
#endif
    return;
}

void SX1278SetSwPin2Rx(void)
{
    HwGPIOSet(LoRa_SW_CTRL, 1);
    HwGPIOSet(LoRa_SW_POWER, 1);
#if 0
    GPIO_SetBits(LORA_RXE_PORT, LORA_RXE_PIN);
    GPIO_SetBits(LORA_TXE_PORT, LORA_TXE_PIN);
#endif
    return;
}

/**
  * @brief  Delay a few of millisecond.
  * @note  This function ONLY delay a few of millisecond since that waiting and loop
  *            would consumes CPU.
  * @note  Be careful of overflow that capability of "uint16_t" is 65535, so the MAX count
  *            of rtimer equal=(65535 / RTIMER_ARCH_SECOND) seconds.
  * @param  wMs: count of millisecond.
  * @retval  None
  */
#if 0 //0712
void DelayMs(uint16_t wMs)
{
#if 0
    uint16_t    wCount;
    rtimer_clock_t    tStart;
    /* Conver millisecond to Hz of rtimer */
    wCount = (uint32_t)wMs * RTIMER_ARCH_SECOND / 1000;
    tStart = RTIMER_NOW();

    while (RTIMER_NOW() - tStart < wCount) ;

#endif
    return;
}
#endif
/**
  * @brief  Initialize the timer that trace Tx or Rx of radio.
  * @param  Callback: point to callback function by this pointer, this callback would
  *              been invoked when the timer expired.
  * @retval  None
  */
#if 0 //0712
void SX1278InitTimer(void (*Callback)(void))
{
#if 0
    /* Save this callback function */
    s_stRTimer.func = (rtimer_callback_t)Callback;
#endif
    return;
}

/**
  * @brief  Set as well as start the timer that trace Tx or Rx of radio.
  * @note  Be careful of overflow that capability of "uint16_t" is 65535, so the MAX count
  *            of rtimer equal=(65535 * 10000 / RTIMER_ARCH_SECOND).
  * @param  wMs: count of milliseconds of timeout for this timer.
  * @retval  None
  */
void SX1278StartTimer(uint16_t wMs)
{
#if 0
    uint16_t    wCount;
    /* Conver millisecond to Hz of rtimer */
    wCount = (uint32_t)wMs * RTIMER_ARCH_SECOND / 1000;
    /* Set rtimer as well as start it! */
    rtimer_set(&s_stRTimer, RTIMER_NOW() + wCount, 0, s_stRTimer.func, NULL);
#endif
    return;
}

/**
  * @brief  Stop the timer that trace Tx or Rx of radio.
  * @param  None.
  * @retval  None
  */
void SX1278StopTimer(void)
{
#if 0
    rtimer_reset();
#endif
    return;
}
#endif

#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/
