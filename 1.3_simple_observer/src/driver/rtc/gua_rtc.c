#include "GUA_RTC.h"
#include "_hal_types.h"
#include <time.h>
#include <ti/sysbios/hal/Seconds.h>
#include "string.h"
#include "simple_observer.h"

void RTC_SetTime(uint32_t timeNow)
{
    Seconds_set(timeNow);
}

uint32_t RTC_GetTime(void)
{
    return Seconds_get();
}

void RTC_ShowTime(void)
{
    uint32_t t;
    time_t t1;
    struct tm *ltm;
    char *curTime;

    t = Seconds_get();
    t1 = time(NULL);
    ltm = localtime(&t1);
    curTime = asctime(ltm);
    sprintf(g_lcdBuffer, "%s", curTime);
    LCD_ShowString(20, 158, g_lcdBuffer);
}


