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

static void SDA_InputInitial(void)//设置端口为输入
{
//设置为输入
PIN_setConfig(hPins, PIN_BM_OUTPUT_MODE | PIN_BM_INPUT_MODE, RTC_DIO | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP);
}

static void SDA_OutputInitial(void)//设置端口为输出
{
//设置为输出
PIN_setConfig(hPins, PIN_BM_OUTPUT_MODE | PIN_BM_INPUT_MODE, RTC_DIO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX);
}

//DS1302初始化函数
void ds1302_init(void) 
{
  hPins=PIN_open(&Pins, PinsCfg);
HwGPIOSet(RTC_nRST, 0);//RST引脚置低
HwGPIOSet(RTC_SCLK, 0);//CLK引脚置高
}

/**
 * @brief 单字节写
 * @param 待写的数据
 */
void DS1302_WriteOneByte(uint8_t data)
{
	uint8_t index = 0;
         SDA_OutputInitial();
	for(index=0; index<8; index++)
	{
	 HwGPIOSet(RTC_SCLK, 0);
		if (data & 0x01) /* 上升沿写数据 */
           {
                      RTCGPIOSet(RTC_DIO,1);
            }
		else
           { 
                        RTCGPIOSet(RTC_DIO,0);
            }
        HwGPIOSet(RTC_SCLK, 1);
		data >>= 1;      /* 从最低位开始写 */
	 }
}

/**
 * @brief 向指定地址写入指定数据
 * @param 指定地址
 * @param 指定数据
 */
void DS1302_WriteByte(uint8_t addr, uint8_t data)
{
   HwGPIOSet(RTC_nRST, 0);
   HwGPIOSet(RTC_SCLK, 0);
   HwGPIOSet(RTC_nRST, 1);//RST置高，写数据有效
   
	DS1302_WriteOneByte(addr);
	DS1302_WriteOneByte(data);
        
   HwGPIOSet(RTC_nRST, 0);
   HwGPIOSet(RTC_SCLK, 0);
}

/**
 * @brief 单字节读
 * @param 待读的地址
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
		if (PIN_getInputValue(RTC_DIO) == 1) /* 上升沿读数据 */
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
 * @brief DS1302设置
 * @param 日期/时间结构体指针
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
        
	DS1302_WriteByte(WRITE_CONTROL_REG, 0x00); /* 去除写保护 */
        DS1302_WriteByte(WRITE_SEC_REG,0x80);	   //暂停时钟
        
	DS1302_WriteByte(WRITE_SEC_REG,DS1302_DATE.sec);
	DS1302_WriteByte(WRITE_MIN_REG,DS1302_DATE.min);
	DS1302_WriteByte(WRITE_HOUR_REG,DS1302_DATE.hour);
	DS1302_WriteByte(WRITE_DAY_REG,DS1302_DATE.day);
	DS1302_WriteByte(WRITE_MON_REG,DS1302_DATE.mon);
	DS1302_WriteByte(WRITE_WEEK_REG,DS1302_DATE.week);
	DS1302_WriteByte(WRITE_YEAR_REG,DS1302_DATE.year);
        
        DS1302_WriteByte(WRITE_CONTROL_REG, 0x80); /* 加上写保护 */

}

/**
 * @brief DS1302读取
 * @param 日期/时间结构体指针
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
// * @brief DS1302初始化
// * @param 日期/时间结构体指针
// * @note  如果是充电电池,可以开启在正常情况下电源对电池的涓流充电功能,只有在异常情况下才使用电池供电
// * @note  直接利用DS1302片上RAM实现 1>如果是第一次上电,则设置日期/时间 2>否则,就不需要设置日期/时间
// */
//uint8_t DS1302_Init(DATE *date)
//{
//    if (DS1302_ReadByte(READ_RAM_REG) == 0x01) /* 如果不是第一次上电,则直接退出 */
//	{
//		return 1;
//	}
//    DS1302_WriteByte(WRITE_CONTROL_REG, 0x00); /* 去除写保护 */
////  DS1302_WriteByte(WRITE_CHARGE_REG, 0xa9);  /* 使能电池涓流充电功能(一定要是充电电池才可以使用此功能!!!) */
//    DS1302_WriteByte(WRITE_RAM_REG,0x01);
//    DS1302_WriteByte(WRITE_CONTROL_REG, 0x80); /* 加上写保护 */
//    DS1302_DateSet(date);
//	return 0;
//}

