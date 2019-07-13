#ifdef SCANBEACON_ADVANCE
#ifndef _OLED_H
#define _OLED_H

#include "stdint.h"
#define OLED_SIZE           16
#define XLevelL             0x00
#define XLevelH             0x10
#define Max_Column          128
#define Max_Row             64
#define Brightness          0xFF
#define X_WIDTH             128
#define Y_WIDTH             64

typedef enum {
    OLED_DATA = 1,
    OLED_CMD  = 0
} OLEDDataType;

#define NOP(n)   \
    do { \
        for (uint32_t i = 0; i < n; i++); \
    } while(0)
      
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void OLED_Init(void);
#endif
#endif