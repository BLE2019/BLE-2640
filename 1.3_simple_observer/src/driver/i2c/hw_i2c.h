#ifndef SERIAL_IIC_H
#define SERIAL_IIC_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * I2C��ʼ������
 */
void HwI2CInit(void);

/*********************************************************************
 * I2C��ȡ�Ĵ�������
 */
bool HwI2CGet(uint8_t Slave_Addr,uint8_t RegAddr, uint8_t *ReadBuf, uint8_t ReadLen);

/*********************************************************************
 * I2C���üĴ���
 */
bool HwI2CSet(uint8_t Slave_Addr,uint8_t RegAddr, uint8_t WriteBuf);
void i2c_write_one_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t Data);
void i2c_write_multi_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData, uint8_t Len);
void i2c_read_one_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData);
void i2c_read_multi_byte(uint8_t SlaveAddr, uint8_t Cmd, uint8_t *pData, uint8_t Len);
void i2c_write_two_byte_addr(uint8_t SlaveAddr,uint16_t Cmd,uint8_t *pData, uint8_t Len );


#ifdef __cplusplus
}
#endif

#endif /* SERIAL_IIC_H */
