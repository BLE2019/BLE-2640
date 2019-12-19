#ifndef _board_WDT_H_
#define _board_WDT_H_
#include "board.h"
#include "Watchdog.h"

extern Watchdog_Handle watchdog;

void wdtInitFxn();
#endif