#ifndef _GT20L16S1Y_H
#define _GT20L16S1Y_H
#include "stdint.h"


void GT20L_Init(void);
//uint32_t GT20L_Addr(uint8_t *hanzi);
void get_hanziaddr(uint8_t *hanzi,uint8_t *r_buffer);
#endif
