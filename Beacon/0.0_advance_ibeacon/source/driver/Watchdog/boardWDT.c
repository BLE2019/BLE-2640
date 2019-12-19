#include "boardWDT.h"
#include "stdint.h"


Watchdog_Handle watchdog = NULL;

void wdtCallback(UArg a0) 
{
	Watchdog_clear(watchdog);
        //HAL_SYSTEM_RESET();
}
 
void wdtInitFxn() 
{
        Watchdog_Params wp;

	Watchdog_init();
        Watchdog_Params_init(&wp);
	wp.callbackFxn    = wdtCallback;
	wp.debugStallMode = Watchdog_DEBUG_STALL_ON;
	wp.resetMode      = Watchdog_RESET_ON;
 
	watchdog = Watchdog_open(Board_WATCHDOG0, &wp);
        if(watchdog == NULL)
        {
          /* Error opening Watchdog*/
          while(1){}
        }
	Watchdog_setReload(watchdog, 15000000); // 1sec (WDT runs always at 48MHz/32)
}