#include "ds1302.h"
#include "board.h"
#include "hw_gpio.h"

static PIN_State  Pins;
static PIN_Handle hPins = NULL;

static PIN_Config PinsCfg[] =
{
    RTC_DIO | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  | PIN_PULLUP,
    PIN_TERMINATE
};


//void hwRTCInit(void)
//{
//  RTCHandle = PIN_open(&RTCState, RTCTable);
//}
void RTCGPIOSet(uint32_t pin, uint8_t flag)
{
  PIN_setOutputValue(hPins, pin, flag);
}

static void SDA_InputInitial(void)//���ö˿�Ϊ����
{
//����Ϊ����
PIN_setConfig(hPins, PIN_BM_OUTPUT_MODE | PIN_BM_INPUT_MODE, RTC_DIO | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP);
}

static void SDA_OutputInitial(void)//���ö˿�Ϊ���
{
//����Ϊ���
PIN_setConfig(hPins, PIN_BM_OUTPUT_MODE | PIN_BM_INPUT_MODE, RTC_DIO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX);
}

//DS1302��ʼ������
void ds1302_init(void) 
{
  hPins=PIN_open(&Pins, PinsCfg);
HwGPIOSet(RTC_nRST, 0);//RST�����õ�
HwGPIOSet(RTC_SCLK, 0);//CLK�����ø�
}

/**
 * @brief ���ֽ�д
 * @param ��д������
 */
void DS1302_WriteOneByte(uint8_t data)
{
	uint8_t index = 0;
         SDA_OutputInitial();
	for(index=0; index<8; index++)
	{
	 HwGPIOSet(RTC_SCLK, 0);
		if (data & 0x01) /* ������д���� */
           {
                      RTCGPIOSet(RTC_DIO,1);
            }
		else
           { 
                        RTCGPIOSet(RTC_DIO,0);
            }
        HwGPIOSet(RTC_SCLK, 1);
		data >>= 1;      /* �����λ��ʼд */
	 }
}

/**
 * @brief ��ָ����ַд��ָ������
 * @param ָ����ַ
 * @param ָ������
 */
void DS1302_WriteByte(uint8_t addr, uint8_t data)
{
   HwGPIOSet(RTC_nRST, 0);
   HwGPIOSet(RTC_SCLK, 0);
   HwGPIOSet(RTC_nRST, 1);//RST�øߣ�д������Ч
   
	DS1302_WriteOneByte(addr);
	DS1302_WriteOneByte(data);
        
   HwGPIOSet(RTC_nRST, 0);
   HwGPIOSet(RTC_SCLK, 0);
}

/**
 * @brief ���ֽڶ�
 * @param �����ĵ�ַ
 */
uint8_t DS1302_ReadByte(uint8_t addr)
{
	uint8_t index = 0, data = 0;
	HwGPIOSet(RTC_nRST, 0);
	HwGPIOSet(RTC_SCLK, 0);
	HwGPIOSet(RTC_nRST, 1);
	DS1302_WriteOneByte(addr);
        
        SDA_InputInitial();
        
	for(index=0; index<8; index++)
	{
          HwGPIOSet(RTC_SCLK, 1);
		if (PIN_getInputValue(RTC_DIO) == 1) /* �����ض����� */
               {
			data |= 0x80;
		}
		data >>= 1;
	  HwGPIOSet(RTC_SCLK, 0);
	}
	HwGPIOSet(RTC_SCLK, 0);
	return data;
}

/**
 * @brief DS1302����
 * @param ����/ʱ��ṹ��ָ��
 */
void DS1302_DateSet(DATE *date)
{
        DATE DS1302_DATE;
        DS1302_DATE.year=date->year;
        DS1302_DATE.mon=date->mon;
        DS1302_DATE.day=date->day;
        DS1302_DATE.hour=date->hour;
        DS1302_DATE.min=date->min;
        DS1302_DATE.sec=date->sec;
        DS1302_DATE.week=date->week;
        
	DS1302_WriteByte(WRITE_CONTROL_REG, 0x00); /* ȥ��д���� */
        DS1302_WriteByte(WRITE_SEC_REG,0x80);	   //��ͣʱ��
        
	DS1302_WriteByte(WRITE_SEC_REG,DS1302_DATE.sec);
	DS1302_WriteByte(WRITE_MIN_REG,DS1302_DATE.min);
	DS1302_WriteByte(WRITE_HOUR_REG,DS1302_DATE.hour);
	DS1302_WriteByte(WRITE_DAY_REG,DS1302_DATE.day);
	DS1302_WriteByte(WRITE_MON_REG,DS1302_DATE.mon);
	DS1302_WriteByte(WRITE_WEEK_REG,DS1302_DATE.week);
	DS1302_WriteByte(WRITE_YEAR_REG,DS1302_DATE.year);
        
        DS1302_WriteByte(WRITE_CONTROL_REG, 0x80); /* ����д���� */

}

/**
 * @brief DS1302��ȡ
 * @param ����/ʱ��ṹ��ָ��
 */
void DS1302_DateRead(DATE *date_read)
{
  
	date_read->sec  =DS1302_ReadByte(READ_SEC_REG);
	date_read->min  =DS1302_ReadByte(READ_MIN_REG);
	date_read->hour =DS1302_ReadByte(READ_HOUR_REG);
	date_read->day  =DS1302_ReadByte(READ_DAY_REG);
	date_read->mon  =DS1302_ReadByte(READ_MON_REG);
	date_read->week =DS1302_ReadByte(READ_WEEK_REG);
	date_read->year =DS1302_ReadByte(READ_YEAR_REG);
}

///**
// * @brief DS1302��ʼ��
// * @param ����/ʱ��ṹ��ָ��
// * @note  ����ǳ����,���Կ�������������µ�Դ�Ե�ص������繦��,ֻ�����쳣����²�ʹ�õ�ع���
// * @note  ֱ������DS1302Ƭ��RAMʵ�� 1>����ǵ�һ���ϵ�,����������/ʱ�� 2>����,�Ͳ���Ҫ��������/ʱ��
// */
//uint8_t DS1302_Init(DATE *date)
//{
//    if (DS1302_ReadByte(READ_RAM_REG) == 0x01) /* ������ǵ�һ���ϵ�,��ֱ���˳� */
//	{
//		return 1;
//	}
//    DS1302_WriteByte(WRITE_CONTROL_REG, 0x00); /* ȥ��д���� */
////  DS1302_WriteByte(WRITE_CHARGE_REG, 0xa9);  /* ʹ�ܵ�������繦��(һ��Ҫ�ǳ���زſ���ʹ�ô˹���!!!) */
//    DS1302_WriteByte(WRITE_RAM_REG,0x01);
//    DS1302_WriteByte(WRITE_CONTROL_REG, 0x80); /* ����д���� */
//    DS1302_DateSet(date);
//	return 0;
//}

