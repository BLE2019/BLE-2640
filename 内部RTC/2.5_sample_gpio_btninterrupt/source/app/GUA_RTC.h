#ifndef _GUA_RTC_H_
#define _GUA_RTC_H_

 

/*********************ͷ�ļ�************************/ 

#include "UTC_Clock.h"

 

/*********************��������************************/ 

extern void GUA_RTC_Init(void);

extern void GUA_RTC_Set(UTCTimeStruct *pGUA_Timer);

extern void GUA_RTC_Get(UTCTimeStruct *pGUA_Timer);

 

#endif