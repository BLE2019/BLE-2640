#include "board.h"

#include "hw_gpio.h"

/*********************************************************************
 * LOCAL PARAMETER
 */
PIN_Handle GPIOHandle;
PIN_State GPIOState;
PIN_Config GPIOTable[] =
{
  Board_SHT20 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  //Board_I2C0_SDA0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  //Board_I2C0_SCL0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

  PIN_TERMINATE
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GY_GPIO_Init
 *
 * @brief   GPIO初始化
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
 * @brief   GPIO配置函数
 *
 * @param   pin -> GPIO引脚
 *          flag -> GPIO电平
 *
 * @return  None.
 */
void HwGPIOSet(uint32_t pin, uint8_t flag)
{
  PIN_setOutputValue(GPIOHandle, pin, flag);
}

uint8_t HwGPIOGet(uint32_t pin)
{
    PIN_getInputValue(pin) ? 1 : 0;
}

