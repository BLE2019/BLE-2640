/**
 * \file
 *         RtcAdapter.c
 * \description
 *         Adapter for RTC.c and Timer.c
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-04-01 11:42
 * \copyright
 *         (c) RimeLink (www.rimelink.com) All Rights Reserved.
 */


/* Includes ------------------------------------------------------------------*/
//#include "RTC.h"  0709!!!!!!!!!!!!!!!
#include "RtcAdapter.h"
#include "Util.h"
//#include "Dbg.h"
//#include "PwrManage.h"


/* Private typedef -----------------------------------------------------------*/
/**
* @brief  Radio LoRa modem parameters
*/


/* Private macro -------------------------------------------------------------*/
/**
* @brief  the RTC alarm is insured if timeout equal or great than 3 ms.
*/
#define RTC_TIMEOUT_MIN_MS    3
#define RTC_TIMEOUT_MAX_MS    (MAX_MS_OF_DAY - RTC_TIMEOUT_MIN_MS)


/* Private variables ---------------------------------------------------------*/
/**
* @brief  save the time that set alarm into RTC, -1 means have not initialized.
*/
static int32_t    s_lRTCTimeAtSetAlarm = -1;
static int32_t    s_lAlarmMs;


/* Private function prototypes -----------------------------------------------*/


/* Private Constants ---------------------------------------------------------*/


/**
  * @brief  Set RTC alarm.
  * @param  uint32_t timeout    period of RTC alarm the unit is ms.
  * @retval  None
  * @note  timeout would be adjusted to [RTC_TIMEOUT_MIN_MS, RTC_TIMEOUT_MAX_MS]
  */
void RtcSetTimeout(uint32_t timeout)
{
#if 0 //0709
    int32_t    lTimeoutMs;
    halIntState_t    intState;

    /* Insure the timeout to be a workable value. */
    if (timeout < RTC_TIMEOUT_MIN_MS)
    {
        lTimeoutMs = RTC_TIMEOUT_MIN_MS;
    }
    else if (RTC_TIMEOUT_MAX_MS < timeout)
    {
        lTimeoutMs = RTC_TIMEOUT_MAX_MS;
        RIME_DBG2(RIME_DBG_ON, "RtcSetTimeout(): timeout > MAX\r\n", 0, PRINTF_FORMAT_NONE);
    }
    else
    {
        lTimeoutMs = timeout;
    }

    /* ATTENTION: do NOT break the "Get->Calc->Set RTC Alarm"(needs 1ms) by ISR! */
    HAL_ENTER_CRITICAL_SECTION(intState);

    s_lRTCTimeAtSetAlarm = rtc_GetTimeMs();
    s_lAlarmMs = s_lRTCTimeAtSetAlarm + lTimeoutMs;
    if (MAX_MS_OF_DAY <= s_lAlarmMs)
    {
        s_lAlarmMs -= MAX_MS_OF_DAY; /* Wrap on 00:00:00.000 */
    }
    if (rtc_SetAlarm(s_lAlarmMs) < 0)
    {
        RIME_DBG2(RIME_DBG_ON, "RtcSetTimeout(): Set Alarm error!\r\n", 0, PRINTF_FORMAT_NONE);
    }

    HAL_EXIT_CRITICAL_SECTION(intState);
#endif
    return;
}

/*-------------------------------------------------------------------------*/
TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
#if EXTI_HALT_UART
    /* Don't enter low power mode if the timeout < 10ms for efficiency,
         the RTC_ISR would "Agree Halt". */
    if (timeout < 10)
    {
        pm_OpposeHalt(PWR_ID_RTC);
    }
    else
    {
        pm_AgrHalt(PWR_ID_RTC);
    }
#endif

    return (TimerTime_t)timeout;
}

#if 0 //0709!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*-------------------------------------------------------------------------*/
TimerTime_t RtcGetTimerValue( void )
{
    return (TimerTime_t)rtc_GetTimeMs();
}

/*-------------------------------------------------------------------------*/
TimerTime_t RtcGetElapsedAlarmTime( void )
{
    return RtcComputeElapsedTime(s_lRTCTimeAtSetAlarm);
}

/*-------------------------------------------------------------------------*/
TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
    int32_t    lFutureMs;
    int32_t    lCurrentMs;

    lFutureMs = (int32_t)futureEventInTime;
    if (lFutureMs < 0)
    {
        lFutureMs = 0;
    }
    else if (MAX_MS_OF_DAY < lFutureMs)
    {
        lFutureMs = MAX_MS_OF_DAY;
    }

    lCurrentMs = rtc_GetTimeMs();
    lFutureMs += lCurrentMs;
    if (MAX_MS_OF_DAY <= lFutureMs)
    {
        lFutureMs -= MAX_MS_OF_DAY;
    }

    return (TimerTime_t)lFutureMs;
}

/*-------------------------------------------------------------------------*/
TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
    int32_t    lEventMs;
    int32_t    lCurrentMs;
    int32_t    lElapsedMs;

    lEventMs = (int32_t)eventInTime;
    if (lEventMs < 0)
    {
        lEventMs = 0;
    }
    else if (MAX_MS_OF_DAY < lEventMs)
    {
        lEventMs = MAX_MS_OF_DAY;
    }

    lCurrentMs = rtc_GetTimeMs();
    lElapsedMs = CALC_RTC_PERIOD(lEventMs, lCurrentMs);

    return (TimerTime_t)lElapsedMs;
}

#endif
/*-------------------------------------------------------------------------*/
#if CATCH_EXCEPTION
uint8_t rtc_GetException(char *p_chBuf)
{
    uint8_t    byLen;
    int32_t    lCurrentMs;

    lCurrentMs = rtc_GetTimeMs();

    byLen = 0;
    byLen += FormatPrintf(&p_chBuf[byLen], "\r\nRTC: set=", s_lRTCTimeAtSetAlarm, PRINTF_FORMAT_DEC);
    byLen += FormatPrintf(&p_chBuf[byLen], ", cur=", lCurrentMs, PRINTF_FORMAT_DEC);
    byLen += FormatPrintf(&p_chBuf[byLen], ", alarm=", s_lAlarmMs, PRINTF_FORMAT_DEC);

    return byLen;
}
#endif


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

