/*********************************************************************
 * INCLUDES
 */
#include "board.h"
#include "board_watchdog.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */
Watchdog_Handle g_watchdogHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/**
 @brief 软件看门狗的定时回调函数
 @param none
 @return none
*/
void watchdogCallback(uintptr_t handle)
{
    Watchdog_clear(g_watchdogHandle);        // 喂狗
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 @brief 软件看门狗的初始化函数
 @param none
 @return none
*/
void Watchdog_Init(void)
{
    Watchdog_Params params;

    Watchdog_init();
    Watchdog_Params_init(&params);
    params.callbackFxn    = (Watchdog_Callback) watchdogCallback;
    params.debugStallMode = Watchdog_DEBUG_STALL_ON;
    params.resetMode      = Watchdog_RESET_ON;

    g_watchdogHandle = Watchdog_open(Board_WATCHDOG0, &params);
    if(g_watchdogHandle == NULL)
    {
        /* Error opening Watchdog */
        while (1) {}
    }
    Watchdog_setReload(g_watchdogHandle, 1500000); // 1sec (WDT runs always at 48MHz/32)
}

/*************************************END OF FILE*************************************/
