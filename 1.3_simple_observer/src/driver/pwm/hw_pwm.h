#ifndef SERIAL_PWM_H
#define SERIAL_PWM_H

#ifdef __cplusplus
extern "C"
{
#endif

  
typedef enum CC2650_PWM_RGB {
    PWM_LED = 0,
    PWM_MOTOR = 1,
    PWM_COUNT
} CC2650_PWM_RGBHandle;

/*****************************************************
 * PWM任务初始化
*/  
void HwPWMInit(void);

/*****************************************************
 * 开启当前PWM引脚
*/ 
void HwPWMStart(uint8_t RGBHandle);

/*****************************************************
 * 暂停当前PWM引脚
*/ 
void HwPWMStop(uint8_t RGBHandle);

/*****************************************************
 * 关闭当前PWM引脚
*/ 
void HwPWMClose(uint8_t RGBHandle);

void HwPWMCloseAll(void);

void HwPWMMusic(uint8_t i);

#ifdef __cplusplus
{
#endif // extern "C"

#endif // SERIAL_PWM_H