#ifndef SERIAL_SPI_H
#define SERIAL_SPI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <ti/drivers/SPI.h>

/*****************************************************
 * SPI任务初始化
*/
void HwSPIInit(void);

/*****************************************************
 * SPI通信函数
*/
void HwSPITrans(uint8_t csnPin, uint8_t *txbuf, uint8_t *rxbuf ,uint16_t len);
bool spi_write(uint8_t csnPin, uint8_t *buf, uint8_t len);
bool spi_read(uint8_t csnPin, uint8_t *buf, uint8_t len);
void BordSPIOpen();
void BordSPIClose();
#ifdef __cplusplus
{
#endif // extern "C"

#endif // SERIAL_SPI_H