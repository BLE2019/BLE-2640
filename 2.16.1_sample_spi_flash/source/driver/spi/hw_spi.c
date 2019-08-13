#include "board.h"
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

#include "hw_spi.h"
#include "hw_uart.h"
#include "sx1278_src.h"
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
 * @brief   SPI�����ʼ��������
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
    SPIparams.frameFormat = SPI_POL1_PHA1;           //��λ1����1  ??????????????
    //SPIparams.frameFormat = SPI_POL0_PHA0;           //��λ1����1  ??????????????

    SPIparams.mode = SPI_MASTER;                     //SPI����ģʽ
    SPIparams.transferCallbackFxn = NULL;
//    SPIparams.transferCallbackFxn  = transferCallback;
    SPIparams.transferMode = SPI_MODE_BLOCKING;      //����
//    SPIparams.transferMode = SPI_MODE_CALLBACK;
    SPIparams.transferTimeout = SPI_WAIT_FOREVER;
    SPIparams.transferTimeout = 50000;//0727!!!!!!!!!!!!!!!!!!!
}

/*********************************************************************
 * @fn      GY_SPI_Trans
 *
 * @brief   SPIͨ�ź���
 *
 * @param   csnPin -> CSN����
 *          txbuf -> ��������
 *          txbuf -> ��������
 *          len -> ͨ�ŵ����ݳ���
 *
 * @return  None.
 */
 static uint8_t trans_cnt = 0;
void delay_spi(uint32_t cnt)  //0725!!!!!!!!!!!!!!!
{
	while(cnt >0)
	{
		cnt--;
	}
}
//0731, ��FIFO���ڴ����ֽڷ�������ϵͳ���������ֽڷ����м�ɼ�ʱ�ӣ����ж�״̬��������֮����ʹ��1278���Ϸ���
//������Ƿ��к�������Դ��ͻ�ĵط�
//��ע�͵��ź���һ��
void HwSPIFifoTrans(uint8_t csnPin, uint8_t *txbuf, uint8_t *rxbuf, uint16_t len)
{
  if(len == 1)
  {
    HwSPITrans(csnPin, txbuf, rxbuf, len);
  }
  else
  {
    uint8_t tx_tmp[1] = {0};
    uint8_t rx_tmp[1] = {0};
    
    for(uint16_t i = 0; i < len; i++)
    {
        tx_tmp[0] = txbuf[i];
        HwSPITrans(csnPin, tx_tmp, rx_tmp, 1);
        
        //just delay
        for(uint16_t m = 0; m < 1000; )
        {
          m++;
        }
    }
  }
  
}
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
    csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //ѡ��CSNƬѡ����

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
        csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //ѡ��CSNƬѡ����
#endif
    }
    
//   delay_spi(50000);  //0725!!!!!!!!!!!
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
    masterTransaction.txBuf  = (void *)buf; //!< ��Ҫ��������ݴ�ŵĵ�ַ����*txBuf
    masterTransaction.arg    = NULL;
    masterTransaction.rxBuf  = NULL;

    csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //ѡ��CSNƬѡ����

    if (csnFlag == SPI_STATUS_SUCCESS) {
        ret = SPI_transfer(SPIHandle, &masterTransaction) ? true : false ;//!< ����SPI_transfer()д������
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

    csnFlag = SPI_control(SPIHandle, SPICC26XXDMA_CMD_SET_CSN_PIN, &csnPin);  //ѡ��CSNƬѡ����

    if (csnFlag == SPI_STATUS_SUCCESS) {
        ret = SPI_transfer(SPIHandle, &masterTransaction) ? true : false ;
    }

    return ret;
}


