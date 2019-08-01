#include "board.h"

#include "hw_gpio.h"

/*********************************************************************
 * LOCAL PARAMETER
 */   
PIN_Handle GPIOHandle;
PIN_State GPIOState;
PIN_Config GPIOTable[] =
{
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  //Board_3V3_EN  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,//res 
  //Board_LCD_MODE| PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,//DC
  //Board_GLED    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,//BLK背光
  //Board_LCD_CSN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,//CS1
  //IOID_23    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, 
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
void HwGPIOSet(PIN_Id pin, uint8_t flag)
{
  PIN_setOutputValue(GPIOHandle, pin, flag);
}

/*********************************************************************
 * @fn      GY_GPIO_SET
 *
 * @brief   获取GPIO引脚高低电平状态函数
 *
 * @param   pin -> GPIO引脚
 *
 * @return  低电平0/高电平1.
 */
uint_t HwGPIOGet(PIN_Id pin)
{
  uint_t ret;
  ret = PIN_getInputValue(pin);
  return ret;
}