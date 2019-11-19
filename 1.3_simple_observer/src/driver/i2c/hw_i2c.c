#include "board.h"
#include <ti/drivers/I2C.h>

#include "string.h"
#include "hw_i2c.h"
//#include "hw_uart.h"

/*********************************************************************
 * LOCAL PARAMETER
 */
I2C_Handle I2CHandle;
I2C_Params I2Cparams;

/*********************************************************************
 * @fn      GY_I2cTask_Init
 *
 * @brief   I2C初始化
 *
 * @param   .
 *
 * @return  None.
 */
void HwI2CInit(void)
{
  I2C_init();
  I2C_Params_init(&I2Cparams);
  I2Cparams.bitRate = I2C_400kHz;
  I2Cparams.custom = NULL;
  I2Cparams.transferCallbackFxn = NULL;
  I2Cparams.transferMode = I2C_MODE_BLOCKING;
  
  I2CHandle = I2C_open(CC2640R2_LAUNCHXL_I2C0,&I2Cparams);
}

/*********************************************************************
 * @fn      GY_I2C_SET
 *
 * @brief   配置寄存器（往寄存器中写值）
 *
 * @param   .
 *
 * @return  None.
 */
bool HwI2CSet(uint8_t Slave_Addr, uint8_t RegAddr, uint8_t WriteBuf)
{
  bool ret = false;
  uint8_t buf[2] = {0};
  buf[0] = RegAddr;
  buf[1] = WriteBuf;
  
  I2C_Transaction i2cTransaction;
  i2cTransaction.writeBuf = buf;
  i2cTransaction.writeCount = 2;
  i2cTransaction.readBuf = NULL;
  i2cTransaction.readCount = 0;
  i2cTransaction.slaveAddress = Slave_Addr;
  i2cTransaction.arg = NULL;
  ret = I2C_transfer(I2CHandle, &i2cTransaction);
  return ret;
}
//i2c写入数据
static void i2c_write(size_t slaveAddress,uint8_t *write_data,size_t len)
{
     I2C_Transaction i2cTransaction;

	 i2cTransaction.slaveAddress = slaveAddress;
	 i2cTransaction.writeBuf = write_data;
	 i2cTransaction.writeCount = len;
	 i2cTransaction.readBuf = NULL;
	 i2cTransaction.readCount = 0;
	 i2cTransaction.arg = NULL;
     I2C_transfer(I2CHandle, &i2cTransaction);
}
/*********************************************************************
 * @fn      GY_I2C_GET
 *
 * @brief   读取寄存器（读取寄存器中的值）
 *
 * @param   .
 *
 * @return  None.
 */
bool HwI2CGet(uint8_t Slave_Addr, uint8_t RegAddr, uint8_t *ReadBuf, uint8_t ReadLen)
{
  bool ret = false;
  I2C_Transaction i2cTransaction;
  i2cTransaction.writeBuf = &RegAddr;
  i2cTransaction.writeCount = 1;
  i2cTransaction.readBuf = ReadBuf;
  i2cTransaction.readCount = ReadLen;
  i2cTransaction.slaveAddress = Slave_Addr;
  i2cTransaction.arg = NULL;
  ret = I2C_transfer(I2CHandle, &i2cTransaction);
  return ret;
}


//i2c读取数据
static void i2c_read(uint8_t slaveAddress,uint8_t *read_data,uint8_t len)
{
    I2C_Transaction i2cTransaction;
    i2cTransaction.slaveAddress = slaveAddress;
	 i2cTransaction.writeBuf = NULL;
	 i2cTransaction.writeCount = 0;
	 i2cTransaction.readBuf = read_data;
	 i2cTransaction.readCount = len;
	 i2cTransaction.arg = NULL;
	 
     I2C_transfer(I2CHandle, &i2cTransaction);
}

//i2c写一个字节数据
void i2c_write_one_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t Data)
{
	 uint8_t buf[2] = {0};
	 buf[0] = Cmd;
	 buf[1] = Data;
	 i2c_write(SlaveAddr,buf,sizeof(buf));
}

//i2c写多个字节
void i2c_write_multi_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData, uint8_t Len)
{
    uint8_t buf[24];
    if(Len <24) {
        buf[0] = Cmd;
        memcpy(buf+1,pData,Len);
        i2c_write(SlaveAddr,buf,Len+1);
     }
}

//i2c读一个字节数据
void i2c_read_one_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData)
{
	 i2c_write(SlaveAddr,&Cmd,1);
	 i2c_read(SlaveAddr,pData,1);
}

//i2c读多个字节数据
void i2c_read_multi_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData, uint8_t Len)
{
	 i2c_write(SlaveAddr,&Cmd,1);
	 i2c_read(SlaveAddr,pData,Len);
}

//写入16bit地址数据
void i2c_write_two_byte_addr(uint8_t SlaveAddr,uint16_t Cmd,uint8_t *pData, uint8_t Len )
{
	 uint8_t addr[2] = {0};
	 addr[0] = Cmd >> 8;
	 addr[1] = Cmd;
	 i2c_write(SlaveAddr,addr,2);
	 i2c_write(SlaveAddr,pData,Len);
}

