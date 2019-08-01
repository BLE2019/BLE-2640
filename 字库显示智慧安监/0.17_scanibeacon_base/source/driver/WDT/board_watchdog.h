#ifndef _BOARD_WATCHDOG_H_
#define _BOARD_WATCHDOG_H_

/*********************************************************************
 * EXTERN VARIABLES
 */
extern Watchdog_Handle g_watchdogHandle;

/*********************************************************************
 * API FUNCTIONS
 */
void Watchdog_Init(void);

#endif /* _BOARD_WATCHDOG_H_ */
