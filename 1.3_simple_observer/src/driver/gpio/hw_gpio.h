#ifndef SERIAL_GPIO_H
#define SERIAL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * GPIO初始化函数
 */
void HwGPIOInit(void);
void HwGPIOClose(void);
/*********************************************************************
 * 设置GPIO电平
 */
void HwGPIOSet(PIN_Id pin, uint8_t flag);  
uint_t HwGPIOGet(PIN_Id pin);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_GPIO_H */
