#ifndef _GUA_RTC_H_
#define _GUA_RTC_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*********************ͷ�ļ�************************/
#include <stdint.h>
/*********************��������************************/
void RTC_SetTime(uint32_t timeNow);
uint32_t RTC_GetTime(void);
void RTC_ShowTime(void);

#ifdef __cplusplus
}
#endif
#endif