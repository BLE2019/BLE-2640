#ifndef SERIAL_UART_H
#define SERIAL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************
 * ���������ʼ��
*/  
void HwUARTInit(void);

/*****************************************************
 * ����д����
*/
void HwUARTWrite(uint8_t *buf, uint16_t len);  //��ӡ�ַ���
void HwUARTRead(uint8_t *buf, uint16_t len);  //��ӡ�ַ���

void HwUARTPrintf(const char* format, ...);   //printf��ӡ
/*****************************************************
 * ���ڽ������ݻص�����������buf��len��
*/
typedef void (*GY_UartRxBufCallback)(uint8_t *buf, uint16_t len);

/*****************************************************
 * ע�ᴮ�ڽ��ջص����񣨽����ڽ��յ����ݴ���app����ȥ����
*/
void GY_UartTask_RegisterPacketReceivedCallback(GY_UartRxBufCallback callback);

#if 0
extern void Uart_Init(npiCB_t npiCBack);
extern void UART_WriteTransport (uint8 *str, uint8 len);
#endif

#ifdef __cplusplus
{
#endif // extern "C"

#endif // end of SERIAL_UART_H definition
