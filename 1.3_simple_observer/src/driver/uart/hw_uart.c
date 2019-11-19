#include "board.h"
#include <ti/drivers/uart/UARTCC26XX.h>
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "hw_uart.h"
#include "simple_observer.h"

/*********************************************************************
 * LOCAL PARAMETER
 */
UART_Handle UARTHandle;
UART_Params UARTparams;
uint8_t Uart_RxTempBuf[200];

GY_UartRxBufCallback GY_UartReviceDataCallback;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Uart_WriteCallback(UART_Handle handle, void *txBuf, size_t size);
void Uart_ReadCallback(UART_Handle handle, void *rxBuf, size_t size);

/*********************************************************************
 * @fn      Uart_ReadCallback
 *
 * @brief   ���ڶ��ص�
 *
 * @param   handle -> ����ͨ��
 *          rxBuf -> ���ڽ������ݵ�ָ��
 *          size -> ���ڽ������ݵĳ���
 *
 * @return  None.
 */
void Uart_ReadCallback(UART_Handle handle, void *rxBuf, size_t size)
{ 
  return;
  if(size <= 1)  //1102
    return;
  
  //UART_write(UARTHandle, rxBuf, size);       //���Դ�ӡ
  #if 0
     UART_read(handle, rxBuf, size);      //�ٴδ�һ�����ڶ�, delete in 1029����Ҫ�ɲ�Ҫ��������ʽ��д
#else
     UART_read(handle, Uart_RxTempBuf, 200); 
 #endif

 #if 0
    GY_UartReviceDataCallback(rxBuf, size);      //��app����һ�����ڶ��ص�
 #else
 test_UartReviceDataCallback(rxBuf, size);
 #endif
}

/*********************************************************************
 * @fn      Uart_WriteCallback
 *
 * @brief   ����д�ص�
 *
 * @param   handle -> ����ͨ��
 *          txBuf -> ���ڷ������ݵ�ָ��
 *          size -> ���ڷ������ݵĳ���
 *
 * @return  None.
 */
void Uart_WriteCallback(UART_Handle handle, void *txBuf, size_t size)
{
  //
}

/*********************************************************************
 * @fn      GY_UartTask_Init
 *
 * @brief   ���������ʼ��
 *
 * @param   None
 *
 * @return  None.
 */
void HwUARTInit(void)
{
  UART_init();                                      //��ʼ��ģ��Ĵ��ڹ���
  UART_Params_init(&UARTparams);                    //��ʼ�����ڲ���
  UARTparams.baudRate = 115200;                     //���ڲ�����115200
  UARTparams.dataLength = UART_LEN_8;               //��������λ8
  UARTparams.stopBits = UART_STOP_ONE;              //����ֹͣλ1
  UARTparams.readDataMode = UART_DATA_BINARY;       //���ڽ������ݲ�������
  UARTparams.writeDataMode = UART_DATA_BINARY;      //���ڷ������ݲ�������
  UARTparams.readMode = UART_MODE_CALLBACK;         //�����첽��
  UARTparams.writeMode = UART_MODE_CALLBACK;        //�����첽д
  UARTparams.readEcho = UART_ECHO_OFF;              //���ڲ�����
  #if 1  //1030
  UARTparams.readReturnMode = UART_RETURN_NEWLINE;  //�����յ����з�ʱ���ص�
  UARTparams.readCallback = Uart_ReadCallback;      //���ڶ��ص�
  UARTparams.writeCallback = Uart_WriteCallback;    //����д�ص�
  
  UARTHandle = UART_open(Board_UART0, &UARTparams); //�򿪴���ͨ��
  UART_control(UARTHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE,  NULL);   //������ղ��ֻص�
  UART_read(UARTHandle, Uart_RxTempBuf, 200);       //��һ�����ڶ�
  #endif
 //   UARTHandle = UART_open(Board_UART0, &UARTparams); //�򿪴���ͨ��

//  UART_read(UARTHandle, Uart_RxTempBuf, 1);       //��һ�����ڶ�


}

/*********************************************************************
 * @fn      GY_UartTask_Write
 *
 * @brief   ����д����
 *
 * @param   buf -> ��Ҫд������ָ��
 *          len -> ��Ҫд�����ݳ���
 *
 * @return  None.
 */
void HwUARTWrite(uint8_t *buf, uint16_t len)
{
#if 1
  UART_write(UARTHandle, buf, len);
#else
  uint8_t n = 0;
  while(buf[n] != '\0')
  {
    uint8_t tmp = buf[n]-1;  ////////////////////////1102
    UART_write(UARTHandle, &tmp, 1);
    n++;
  }
#endif
}
void HwUARTRead(uint8_t *buf, uint16_t len)
{
  UART_read(UARTHandle, buf, len);
}

/*********************************************************************
 * @fn      GY_UartTask_Printf
 *
 * @brief   ����д����������ϵͳprintf��
 *
 * @param   format -> ����������־λ������%d,%s��
 *          ... -> ��������
 *
 * @return  None.
 */
void HwUARTPrintf(const char* format, ...)
{
  va_list arg;
  va_start(arg,format);
  uint8_t buf[200];
  uint16_t len;
  len = vsprintf((char*)buf, format, arg);
  UART_write(UARTHandle, buf, len);
}
/*********************************************************************
 * @fn      GY_UartTask_RegisterPacketReceivedCallback
 *
 * @brief   ע�ᴮ�ڽ��ջص����񣨽����ڽ��յ����ݴ���app����ȥ����
 *
 * @param   callback -> ���ڽ������ݻص�����������buf��len��
 *
 * @return  None.
 */
void GY_UartTask_RegisterPacketReceivedCallback(GY_UartRxBufCallback callback)
{
  GY_UartReviceDataCallback = callback;
}


char tRxBuf[100]; //�����������飬����ʵ��������Ĵ�С
char tTxBuf[100]; //�����������飬����ʵ��������Ĵ�С

//static bool uartInitFlag = FALSE;

#if 0
void Uart_Init(npiCB_t npiCBack)//����ע���ʼ���ص��������ں��棩
{
     if(!uartInitFlag)
    {
	NPITLUART_initializeTransport(tRxBuf, tTxBuf, npiCBack);//������޸�UART�����ã�������������ڲ��޸ļ���
	uartInitFlag = TRUE;
    }
}
void UART_WriteTransport (uint8 *str, uint8 len) //�������ݵĺ������˺������str��ַ�����ݸ��Ƶ�tTxBuf���ͳ�ȥ
{
     if(uartInitFlag)
     {
	memcpy(tTxBuf, str, len);
	NPITLUART_writeTransport(len);
     }
}
#endif