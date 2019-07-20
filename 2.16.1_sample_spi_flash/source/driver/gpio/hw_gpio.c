#include "board.h"

#include "hw_gpio.h"

/*********************************************************************
 * LOCAL PARAMETER
 */   
PIN_Handle GPIOHandle;
PIN_State GPIOState;
PIN_Config GPIOTable[] =
{
//  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LoRa_CLK_POWEREN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LoRa_nRST        | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LoRa_SW_POWER    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LoRa_SW_CTRL     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GY_GPIO_Init
 *
 * @brief   GPIO��ʼ��
 *
 * @param   .
 *
 * @return  None.
 */
void HwGPIOInit(void)
{
  GPIOHandle = PIN_open(&GPIOState, GPIOTable);
}

/*********************************************************************
 * @fn      GY_GPIO_SET
 *
 * @brief   GPIO���ú���
 *
 * @param   pin -> GPIO����
 *          flag -> GPIO��ƽ
 *
 * @return  None.
 */
void HwGPIOSet(uint32_t pin, uint8_t flag)
{
  PIN_setOutputValue(GPIOHandle, pin, flag);
}

