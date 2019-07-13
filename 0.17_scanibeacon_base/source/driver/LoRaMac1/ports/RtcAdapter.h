/**
 * \file
 *         RtcAdapter.h
 * \description
 *         Head file for adapter for RTC.c and Timer.c
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-04-01 11:42
 * \copyright
 *         (c) RimeLink (www.rimelink.com) All Rights Reserved.
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_ADAPTER_H__
#define __RTC_ADAPTER_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "SchedTimer.h"


/* Exported variables ------------------------------------------------------- */


/* Exported constants --------------------------------------------------------*/


/* Exported macros -----------------------------------------------------------*/


/* Private macros ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Private types ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

/*!
 * \brief Start the RTC timer
 *
 * \remark The timer is based on the RTC Alarm running at 32.768KHz
 *
 * \param[IN] timeout Duration of the Timer
 */
void RtcSetTimeout( uint32_t timeout );

/*!
 * \brief Adjust the value of the timeout to a workable alarm time.
 *
 * \param[IN] timeout Duration of the Timer without compensation for wakeup time
 * \retval new value for the Timeout with compensations
 */
TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout );

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
TimerTime_t RtcGetTimerValue( void );

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm
 */
TimerTime_t RtcGetElapsedAlarmTime( void );

/*!
 * \brief Compute the timeout time of a future event in time
 *
 * \param[IN] futureEventInTime Value in time
 * \retval time Time between now and the futureEventInTime
 */
TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime );

/*!
 * \brief Compute the elapsed time since a fix event in time
 *
 * \param[IN] eventInTime Value in time
 * \retval elapsed Time since the eventInTime
 */
TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime );


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

