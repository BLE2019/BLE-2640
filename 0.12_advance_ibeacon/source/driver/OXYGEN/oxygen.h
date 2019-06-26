#ifndef _OXYGEN_H
#define _OXYGEN_H

#include <stdint.h>

#define NOP(n)   \
    do { \
        for (uint32_t i = 0; i < n; i++); \
    } while(0)


void IIC_WriteBytes(uint8_t WriteAddr,uint8_t* data,uint8_t dataLength);
void IIC_ReadBytes(uint8_t deviceAddr, uint8_t writeAddr,uint8_t* data,uint8_t dataLength);
void IIC_Read_One_Byte(uint8_t daddr,uint8_t addr,uint8_t* data);
void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
#endif