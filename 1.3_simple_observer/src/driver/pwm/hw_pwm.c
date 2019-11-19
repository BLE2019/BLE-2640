#include "board.h"
#include "ti/drivers/PWM.h"

#include "hw_pwm.h"
//#include "music.h"
/*********************************************************************
 * LOCAL PARAMETER
 */
PWM_Handle PWMHandle_LED;
PWM_Handle PWMHandle_MOTOR;

PWM_Params PWMparams;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GY_PwmTask_Init
 *
 * @brief   PWM任务初始化及启动
 *
 * @param   pin -> Board_PWM0 - Board_PWM7
 *
 * @return  None.
 */
void HwPWMInit(void)
{
  PWM_init();
  PWM_Params_init(&PWMparams);
  //RED
  PWMparams.idleLevel = PWM_IDLE_LOW;
  PWMparams.periodUnits = PWM_PERIOD_HZ;
  PWMparams.periodValue = 100;
  PWMparams.dutyUnits = PWM_DUTY_FRACTION;
  PWMparams.dutyValue = PWM_DUTY_FRACTION_MAX/2;
  PWMHandle_LED = PWM_open(Board_LCD_LED_ON, &PWMparams);
  //GREEN
  PWMparams.idleLevel = PWM_IDLE_LOW;
  PWMparams.periodUnits = PWM_PERIOD_HZ;
  PWMparams.periodValue = 100;
  PWMparams.dutyUnits = PWM_DUTY_FRACTION;
  PWMparams.dutyValue = PWM_DUTY_FRACTION_MAX/2;
  PWMHandle_MOTOR = PWM_open(Board_MOTOR, &PWMparams);
  }

/*********************************************************************
 * @fn      GY_PWM_Start
 *
 * @brief   开启当前PWM引脚
 *
 * @param   None
 *
 * @return  None.
 */
void HwPWMStart(uint8_t RGBHandle)
{
  switch(RGBHandle)
  {
    case 0:
      PWM_start(PWMHandle_LED);
      break;
    case 1:
      PWM_start(PWMHandle_MOTOR);
      break;
    
    default:
      break;
  }
}

/*********************************************************************
 * @fn      GY_PWM_Stop
 *
 * @brief   暂停当前PWM引脚
 *
 * @param   None
 *
 * @return  None.
 */
void HwPWMStop(uint8_t RGBHandle)
{
  switch(RGBHandle)
  {
    case 0:
      PWM_stop(PWMHandle_LED);
      break;
    case 1:
      PWM_stop(PWMHandle_MOTOR);
      break;
    default:
      break;
  }
}

/*********************************************************************
 * @fn      GY_PWM_Close
 *
 * @brief   关闭当前PWM引脚
 *
 * @param   None
 *
 * @return  None.
 */
void HwPWMClose(uint8_t RGBHandle)
{
  switch(RGBHandle)
  {
    case 0:
      PWM_stop(PWMHandle_LED);
      PWM_close(PWMHandle_LED);
      break;
    case 1:
      PWM_stop(PWMHandle_MOTOR);
      PWM_close(PWMHandle_MOTOR);
      break;
    default:
      break;
  }
}

/*********************************************************************
 * @fn      GY_PWM_CloseAll
 *
 * @brief   关闭所有PWM引脚
 *
 * @param   None
 *
 * @return  None.
 */
void HwPWMCloseAll(void)
{
  HwPWMStop(PWM_LED);
  HwPWMStop(PWM_MOTOR);
}
