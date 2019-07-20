#include "board.h"
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

#include "hw_spi.h"
#include "hw_uart.h"
/*********************************************************************
 * LOCAL PARAMETER
 */
SPI_Handle      SPIHandle;
SPI_Params      SPIparams;

/*********************************************************************
 * LOCAL FUNCTIONS
 */


/*********************************************************************
 * @fn      GY_SpiTask_Init
 *
 * @brief   SPI任务初始化及启动
 *
 * @param   spiIndex -> 0:Board_SPI0 | 1:Board_SPI1
 *          spiMode ->  0:SPI_MASTER | 1:SPI_SLAVE
 *
 * @return  None.
 */
static void transferCallback(SPI_Handle handle, SPI_Transaction *transaction)
{}
void HwSPIInit(void)
{
    SPI_init();
    SPI_Params_init(&SPIparams);
    SPIparams.bitRate  = 8000000;                    //1MHz
    SPIparams.dataSize = 8;
    SPIparams.frameFormat = SPI_POL1_PHA1;           //相位1极性1  ??????????????
    //SPIparams.frameFormat = SPI_POL0_PHA0;           //相位1极性1  ??????????????

    SPIparams.mode = SPI_MASTER;                     //SPI主从模式
    SPIparams.transferCallbackFxn = NULL;
//    SPIparams.transferCallbackFxn  = transferCallback;
    SPIparams.transferMode = SPI_MODE_BLOCKING;      //阻塞
//    SPIparams.transferMode = SPI_MODE_CALLBACK;
    SPIparams.transferTimeout = SPI_WAIT_FOREVER;
}

/*********************************************************************
 * @fn      GY_SPI_Trans
 *
 * @brief   SPI通信函数
 *
 * @param   csnPin -> CSN引脚
 *          txbuf -> 发送数据
 *          txbuf -> 接收数据
 *          len -> 通信的数据长度
 *
 * @return  None.
 */
 static uint8_t trans_cnt = 0;
void HwSPITrans(uint8_t csnPin, uint8_t *txbuf, uint8_t *rxbuf, uint16_t len)
{
    SPI_Transaction spiTransaction;
    spiTransaction.arg = NULL;
    spiTransaction.count = len;
    spiTransaction.txBuf = txbuf;
    spiTransaction.rxBuf = rxbuf;
    SPIHandle = SPI_open(Board_SPI, &SPIparams);
    
    uint8_t status_open = spiTransaction.status;
    
    trans_cnt += 1;
    if (trans_cnt>8)
    {
   //   DelayMs(1);
    }
    if(status_open>SPI_TRANSFER_CSN_DEASSERT)
    {
         status_open = status_open + 100;
 //        SPI_transferCancel(SPIHandle);
  //       SPI_close(SPIHandle);
    }
    int csnFlag;
    csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //选择CSN片选引脚

    //0717....
    //DelayMs(1);
    
     uint8_t status1_control = spiTransaction.status;
    if(status1_control>SPI_TRANSFER_CSN_DEASSERT)
    {//0717.............
         status1_control = status1_control + 150;
#if 0
         SPI_transferCancel(SPIHandle);
         SPI_close(SPIHandle);
         SPIHandle = SPI_open(Board_SPI, &SPIparams);
        csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //选择CSN片选引脚
#endif
    }
 
    if (csnFlag == SPI_STATUS_SUCCESS) 
    {
        uint8_t t1 = txbuf[0];
        uint8_t t2 = txbuf[1];
        SPI_transfer(SPIHandle, &spiTransaction);
    }
    
    //0717...
    //DelayMs(1);
    
       uint8_t status2_trans = spiTransaction.status;
    if(status2_trans>SPI_TRANSFER_CSN_DEASSERT)
    {
         status2_trans= status2_trans + 180;
 //        SPI_transferCancel(SPIHandle);
  //       SPI_close(SPIHandle);
    }
 
    SPI_transferCancel(SPIHandle);
    
    //0717
    //DelayMs(1);
    
    SPI_close(SPIHandle);
 
    uint8_t status3_close = spiTransaction.status;
    if(status3_close>SPI_TRANSFER_CSN_DEASSERT)
    {
         status3_close = status3_close + 200;
 //        SPI_transferCancel(SPIHandle);
  //       SPI_close(SPIHandle);
    }
    DelayMs(2);
}

void BordSPIOpen()
{
    SPIHandle = SPI_open(Board_SPI, &SPIparams);
}
void BordSPIClose()
{
    SPI_transferCancel(SPIHandle);
    SPI_close(SPIHandle);
}

bool spi_write(uint8_t csnPin, uint8_t *buf, uint8_t len)
{
    SPI_Transaction masterTransaction;
    int csnFlag;
    bool ret = false;
    masterTransaction.count  = len;
    masterTransaction.txBuf  = (void *)buf; //!< 将要传输的数据存放的地址赋给*txBuf
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = NULL;

    csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //选择CSN片选引脚

    if (csnFlag == SPI_STATUS_SUCCESS) {
        ret = SPI_transfer(SPIHandle, &masterTransaction) ? true : false ;//!< 调用SPI_transfer()写入数据
    }

    return ret;
}

bool spi_read(uint8_t csnPin, uint8_t *buf, uint8_t len)
{
    SPI_Transaction masterTransaction;
    bool ret = false;
    int csnFlag;

    masterTransaction.count = len;
    masterTransaction.rxBuf = buf;
    masterTransaction.txBuf = NULL;
    masterTransaction.arg = NULL;

    csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //选择CSN片选引脚

    if (csnFlag == SPI_STATUS_SUCCESS) {
        ret = SPI_transfer(SPIHandle, &masterTransaction) ? true : false ;
    }

    return ret;
}


