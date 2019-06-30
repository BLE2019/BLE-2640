#ifndef __CC26XXI2C_H
#define __CC26XXI2C_H
#include "board.h"
#include "ti/sysbios/knl/Task.h"

/* I2C */
#define CC2640R2_LAUNCHXL_I2C0_SCL0 	IOID_0 //i2c SCL时钟线
#define CC2640R2_LAUNCHXL_I2C0_SDA0 	IOID_1 //i2c SDA数据线

/*!
?* ?@def ? ?CC2640R2_LAUNCHXL_I2CName
?* ?@brief ?Enum of I2C names
?*/
typedef enum CC2640R2_LAUNCHXL_I2CName {
	CC2640R2_LAUNCHXL_I2C0 = 0,

	CC2640R2_LAUNCHXL_I2CCOUNT
} CC2640R2_LAUNCHXL_I2CName;

void i2c_init(void);
void i2c_write_one_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t Data);
void i2c_write_multi_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData, uint8_t Len);
void i2c_read_one_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData);
void i2c_read_multi_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData, uint8_t Len);
void i2c_write_two_byte_addr(uint8_t SlaveAddr,uint16_t Cmd,uint8_t *pData, uint8_t Len );
#endif
