#ifndef _DS_1302_H
#define _DS_1302_H

#include "stdint.h"



#define DataToBcd(x) ((x/10)*16+(x%10))
#define BcdToData(x) ((x/16)*10+(x%16))

#define WRITE_SEC_REG	        0x80
#define WRITE_MIN_REG           0x82
#define WRITE_HOUR_REG          0x84
#define WRITE_DAY_REG           0x86
#define WRITE_MON_REG           0x88
#define WRITE_WEEK_REG          0x8a
#define WRITE_YEAR_REG          0x8c
#define READ_SEC_REG            0x81
#define READ_MIN_REG            0x83
#define READ_HOUR_REG           0x85
#define READ_DAY_REG            0x87
#define READ_MON_REG            0x89
#define READ_WEEK_REG           0x8b
#define READ_YEAR_REG           0x8d
#define WRITE_CONTROL_REG       0x8e
#define READ_CONTROL_REG        0x8f
#define WRITE_CHARGE_REG        0x90
#define READ_CHARGE_REG         0x91
#define WRITE_RAM_REG           0xc0
#define READ_RAM_REG            0xc1

typedef struct
{
	 uint8_t sec;
	 uint8_t min;
	 uint8_t hour;
	 uint8_t day;
	 uint8_t mon;
	 uint8_t week;
	 uint16_t year;
}DATE;
//void hwRTCInit(void);
void RTCGPIOSet(uint32_t pin, uint8_t flag);
static void SDA_InputInitial(void);
static void SDA_OutputInitial(void);

void ds1302_init(void);
static void SDA_InputInitial(void);
static void SDA_OutputInitial(void);
void DS1302_WriteOneByte(uint8_t data);
void DS1302_WriteByte( uint8_t addr, uint8_t data);
uint8_t DS1302_ReadByte(uint8_t addr);
void DS1302_DateSet(DATE *date);
void DS1302_DateRead(DATE *date_read);
//uint8_t DS1302_Init(DATE *date);


#endif

