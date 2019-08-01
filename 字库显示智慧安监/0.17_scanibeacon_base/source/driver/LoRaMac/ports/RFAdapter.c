/**
 * \file
 *         RFAdapter.c
 * \description
 *         Adapter for sx1278_src.c and LoRaMac.c
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-03-30 17:13
 * \copyright
 *         (c) RimeLink (www.rimelink.com) All Rights Reserved.
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "radio.h"
#include "../radio/sx1278_src.h"
#include "../radio/sx1278_ports.h"
#include "../radio/sx1276Regs-LoRa.h"
#include "RFAdapter.h"


/* Private macro -------------------------------------------------------------*/

/* Private Constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/*-------------------------------------------------------------------------*/
static void SX1278SetModem( RadioModems_t modem )
{
    /* avoid complaint of compiler */
    modem = modem;
    return; /* Only support Lora-Mode. */
}

/*-------------------------------------------------------------------------*/
static bool RFIsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    /* avoid complaint of compiler */
    modem = modem;

    SX1278SetFreq(freq);
    return SX1278IsChannelFree(2, rssiThresh);
}

/**
  * @brief  Sets configured parameters for RF_Rx
  * @note  Only support: Lora-Mode
  *             Ignore: bandwidthAfc / freqHopOn / hopPeriod.
  * @param  uint32_t bandwidth  0=125k, 1=250k, 2=500k
  * @param  uint32_t datarate  6=SF6, 7=SF7, ..., 12=SF12
  * @param  uint8_t coderate  1=4/5, 2=4/6, ..., 4=4/8
  * @retval  None
  */
static void SX1278SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    RadioBW_t    tBW;
    RadioSF_t    tSF;
    RadioFEC_t    tFEC;
    bool    bOptimize;

    /* avoid complaint of compiler */
    bandwidthAfc = bandwidthAfc;
    freqHopOn = freqHopOn;
    hopPeriod = hopPeriod;

    /* convert parameters to call functions of SX1278. */
    tBW = (RadioBW_t)(RF_BW_125000 + bandwidth);
    tSF = (RadioSF_t)datarate;
    tFEC = (RadioFEC_t)coderate;

    /* for Low Datarate Optimize. */
    if ( (RF_BW_125000 == tBW && (RF_SF_11 == tSF || RF_SF_12 == tSF)) ||
          (RF_BW_250000 == tBW && RF_SF_12 == tSF) )
    {
        bOptimize = true;
    }
    else
    {
        bOptimize = false;    
    }

    /* for ERRATA 2.1: Sensitivity Optimization with a 500 kHz Bandwidth. */
    if (RF_BW_500000 == tBW)
    {
        SX1278Write(0x36, 0x02);
        SX1278Write(0x3A, 0x7F);    
    }
    else
    {
        SX1278Write(0x36, 0x03);
    }

    /* save parameters into SX1278. */
    SX1278SetModem(modem);
    SX1278SetBandwidth(tBW);
    SX1278SetSpreadingFactor(tSF);
    SX1278SetCodingRate(tFEC);
    SX1278SetLowDatarateOptimize(bOptimize);
    SX1278SetPreambleLen(preambleLen);
    SX1278SetSymbTimeout(symbTimeout);
    SX1278SetFixLen(fixLen);
    if (fixLen)
    {
        SX1278Write(REG_LR_PAYLOADLENGTH, payloadLen);
    }
    SX1278SetCrcOn(crcOn);
    SX1278SetLoRaSettings();

    SX1278SetRxContinuous(rxContinuous);
    SX1278SetIqInverted(iqInverted);

    return;
}

/**
  * @brief  Sets configured parameters for RF_Tx
  * @note  Only support: Lora-Mode
  *             Ignore: fdev / freqHopOn / hopPeriod.
  * @param  uint32_t bandwidth  0=125k, 1=250k, 2=500k
  * @param  uint32_t datarate  6=SF6, 7=SF7, ..., 12=SF12
  * @param  uint8_t coderate  1=4/5, 2=4/6, ..., 4=4/8
  * @retval  None
  */
static void SX1278SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    RadioBW_t    tBW;
    RadioSF_t    tSF;
    RadioFEC_t    tFEC;
    bool    bOptimize;

    /* avoid complaint of compliler */
    fdev = fdev;
    freqHopOn = freqHopOn;
    hopPeriod = hopPeriod;

    /* convert parameters to call functions of SX1278. */
    tBW = (RadioBW_t)(RF_BW_125000 + bandwidth);
    tSF = (RadioSF_t)datarate;
    tFEC = (RadioFEC_t)coderate;

    /* for Low Datarate Optimize. */
    if ( (RF_BW_125000 == tBW && (RF_SF_11 == tSF || RF_SF_12 == tSF)) ||
          (RF_BW_250000 == tBW && RF_SF_12 == tSF) )
    {
        bOptimize = true;
    }
    else
    {
        bOptimize = false;    
    }

    /* save parameters into SX1278. */
    SX1278SetModem(modem);
    SX1278SetTxPower(power);
    SX1278SetBandwidth(tBW);
    SX1278SetSpreadingFactor(tSF);
    SX1278SetCodingRate(tFEC);
    SX1278SetLowDatarateOptimize(bOptimize);
    SX1278SetPreambleLen(preambleLen);
    SX1278SetFixLen(fixLen);
    SX1278SetCrcOn(crcOn);
    SX1278SetLoRaSettings();

    SX1278SetTxTimeout(timeout);
    SX1278SetIqInverted(iqInverted);

    return;
}

/*-------------------------------------------------------------------------*/
static bool SX1278CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

/*-------------------------------------------------------------------------*/
static uint32_t RFGetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    /* Only support Lora-Mode. */
    return (uint32_t)SX1278GetTimeOnAir(pktLen);
}

/*-------------------------------------------------------------------------*/
static void RFSend( uint8_t *buffer, uint8_t size )
{
    SX1278Send(buffer, size);

    return;
}

/*-------------------------------------------------------------------------*/
static void RFSetRx( uint32_t timeout )
{
    SX1278Receive(timeout);

    return;
}

/*-------------------------------------------------------------------------*/
static void RFStartCad( void )
{
    SX1278SetCAD(0);

    return;
}

/*-------------------------------------------------------------------------*/
static void RFSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    /* Only do this on FSK-Mode, now we ONLY support Lora-Mode. */
    return; 
}

/*-------------------------------------------------------------------------*/
static int16_t RFReadRssi( RadioModems_t modem )
{
    return SX1278GetCurrentRssi();
}

/*-------------------------------------------------------------------------*/
static void RFSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    /* avoid complaint of compiler */
    modem = modem;

    SX1278Write(REG_LR_PAYLOADMAXLENGTH, max);

    return;
}

/*-------------------------------------------------------------------------*/
static void RFSetPublicNetwork( bool enable )
{
    if (enable)
    {
        SX1278Write(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    }
    else
    {
        SX1278Write(REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }

    return;
}


/* Private typedef -----------------------------------------------------------*/
/**
* @brief  Radio driver structure
*/
const struct Radio_s Radio =
{
    .Init = SX1278Init,
    .GetStatus = SX1278GetStatus,
    .SetModem = SX1278SetModem,
    .SetChannel = SX1278SetFreq,
    .IsChannelFree = RFIsChannelFree,
    .Random = SX1278Random,
    .SetRxConfig = SX1278SetRxConfig,
    .SetTxConfig = SX1278SetTxConfig,
    .CheckRfFrequency = SX1278CheckRfFrequency,
    .TimeOnAir = RFGetTimeOnAir,
    .Send = RFSend,
    .Sleep = SX1278SetSleep,
    .Standby = SX1278SetStandby,
    .Rx = RFSetRx,
    .StartCad = RFStartCad,
    .SetTxContinuousWave = RFSetTxContinuousWave,
    .Rssi = RFReadRssi,
    .Write = SX1278Write,
    .Read = SX1278Read,
    .WriteBuffer = SX1278WriteBuffer,
    .ReadBuffer = SX1278ReadBuffer,
    .SetMaxPayloadLength = RFSetMaxPayloadLength,
    .SetPublicNetwork = RFSetPublicNetwork,
};


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

