/**
 * \file
 *         Independent source codes to operate sx1278
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2017-04-01 16:41
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "sx1278_src.h"
#include "sx1278_ports.h"
#include "sx1276Regs-FSK.h"
#include "sx1276Regs-LoRa.h"
#include "Chip.h"
//#include "PwrManage.h"

/* Private typedef -----------------------------------------------------------*/

/**
* @brief  Radio LoRa modem parameters
*/
typedef struct
{
    uint32_t    lFrequency;
    uint16_t    wPreambleLen;
    uint16_t    wTxTimeout;
    uint16_t    wSymbTimeout;
    int8_t    chPower;
    RadioBW_t    tBW;
    RadioSF_t    tSF;
    RadioFEC_t    tFEC;
    bool    bLowDatarateOptimize;
    bool    bFixLen;
    bool    bCrcOn;
    bool    bRxContinuous;
    bool    bChanged;
    bool    bIqInverted;
} RadioLoRaSettings_t;

 /**
* @brief  LoRa packet handler state
*/
typedef struct
{
    uint8_t    bySize;
    int8_t    chSNR;
    int16_t    nRSSI;
}RadioLoRaPacketHandler_t;


/* Private macro -------------------------------------------------------------*/
#define RSSI_OFFSET_LF    -164


/* Private variables ---------------------------------------------------------*/
static const RadioEvents_t    *s_pstRadioEvents = NULL;

static volatile RadioState_t    s_tRadioState = RF_IDLE;

static RadioLoRaSettings_t    s_stLoRaSettings;

static RadioLoRaPacketHandler_t    s_stPacketHandler;

static uint8_t    s_abyRxBuf[RF_FIFO_SIZE];

/* Get count of TxTimeout and RxError. */
#if CATCH_EXCEPTION
static uint16_t    s_wRFTxTimeoutCnt = 0;
static uint16_t    s_wRFRxErrorCnt = 0;
#endif


/* Private function prototypes -----------------------------------------------*/

/* Private Constants ---------------------------------------------------------*/
/*!
 * Precomputed signal bandwidth log values used to compute the Packet RSSI value.
 */
/*
const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};
*/

/* Convert float data to integer for decrease computation */
const int8_t    SignalBwLogMulti10[] =
{
    39, /* 7.8 kHz */
    40, /* 10.4 kHz */
    42, /* 15.6 kHz */
    43, /* 20.8 kHz */
    45, /* 31.2 kHz */
    46, /* 41.6 kHz */
    48, /* 62.5 kHz */
    51, /* 125 kHz */
    54, /* 250 kHz */
    57, /* 500 kHz */
};

/*!
 * BandWidth used to calculate time of packet and preamble on air.
 */
const static double    s_adBW[] =    \
    {78e2, 104e2, 156e2, 208e2, 312e2, 414e2, 625e2, 125e3, 250e3, 500e3};


/*-------------------------------------------------------------------------*/
void SX1278Write(uint8_t addr, uint8_t data)
{
    SX1278WriteBuffer(addr, &data, 1);
}

/*-------------------------------------------------------------------------*/
uint8_t SX1278Read(uint8_t addr)
{
    uint8_t data;
	
    SX1278ReadBuffer(addr, &data, 1);
	
    return data;
}

/*-------------------------------------------------------------------------*/
static void SX1278WriteFifo(const uint8_t *p_byBuf, uint8_t bySize)
{
    SX1278WriteBuffer(0, (uint8_t *)p_byBuf, bySize);
}

/*-------------------------------------------------------------------------*/
static void SX1278ReadFifo(uint8_t *p_byBuf, uint8_t bySize)
{
    SX1278ReadBuffer(0, p_byBuf, bySize);
}

/*-------------------------------------------------------------------------*/
//-------------------0706----------------------
void DelayMs(uint16_t n)
{
	for(uint16_t tmp = 0; tmp < 100 * n; tmp++)
	{
		
	}
}
static void SX1278Reset( void )
{
    /* Pull down RESET pin to reset SX1278 */
    //SX1278SetOutputResetPin();
    SX1278PullLowResetPin();	

    /* Wait 1 ms */
    DelayMs(1);

    /* Configure RESET pin as input */
    //SX1278SetInputResetPin();
	
    /* Wait 30ms */
    //DelayMs(30);

    return;	
}

/*-------------------------------------------------------------------------*/
static void SX1278SetOpMode(uint8_t opMode )
{
    SX1278Write( REG_LR_OPMODE, 
                            (SX1278Read(REG_LR_OPMODE) & RF_OPMODE_MASK) | opMode );
}

/*-------------------------------------------------------------------------*/
//-----------------------0706------------------
void chip_LED2Off(void)
{
}
void chip_LED2On(void)
{
}
void chip_LEDOn(void)
{}
void chip_LEDOff(void)
{
}
static void TimeoutHandler(void)
{
    if (RF_TX_RUNNING == s_tRadioState) /* Tx timeout */
    {
        /* timeout too short or chip is abnormal, set to StandBy for NEW mode. */
        SX1278SetOpMode(RF_OPMODE_STANDBY);
        s_tRadioState = RF_IDLE;
     //   RIME_DBG2(RIME_DBG_ON, "Error: RF_TX timeout!\r\n", 0, PRINTF_FORMAT_NONE); //0706!!!!!!!!!!

        chip_LEDOff();		
        if (s_pstRadioEvents && s_pstRadioEvents->TxTimeout)
        {
            s_pstRadioEvents->TxTimeout();
        }

    #if CATCH_EXCEPTION
        ++s_wRFTxTimeoutCnt;
    #endif
    }
    else if (RF_RX_RUNNING == s_tRadioState) /* Rx timeout */
    {	
        if (FALSE == s_stLoRaSettings.bRxContinuous) /* Is RX_SINGLE */
        {
            s_tRadioState = RF_IDLE;
        }

        if (s_pstRadioEvents && s_pstRadioEvents->RxTimeout)
        {
            s_pstRadioEvents->RxTimeout();
        }    
    }
    else if (RF_CAD == s_tRadioState) /* CAD timeout */
    {    
        s_tRadioState = RF_IDLE;
 //       RIME_DBG2(RIME_DBG_ON, "Error: CAD timeout!\r\n", 0, PRINTF_FORMAT_NONE); //0706 !!!!!!!!!!!!!
    }
    else /* Error */
    {
   //     ASSERT(!"Error: RF is IDLE but is timeout!\r\n");  //0706
    }

#if EXTI_HALT_UART
    pm_AgrHalt(PWR_ID_RF);
#endif		

    return;	
}

/*-------------------------------------------------------------------------*/
int16_t SX1278GetCurrentRssi(void)
{
    int16_t    nRSSI;

    nRSSI = (int16_t)SX1278Read(REG_LR_RSSIVALUE);	
    nRSSI += RSSI_OFFSET_LF;

    return nRSSI;
}

/**
  * @brief  Initialize radio of SX1278.
  * @note  Insure callback functions are SMALL, FAST and SAFE that would been invoked by ISR. 
  * @param  p_stEvents: point to function array of event callback by this pointer.
  * @retval  None
  */
void SX1278Init(RadioEvents_t *p_stEvents)
{
    s_pstRadioEvents = p_stEvents;

    SX1278Reset();

    SX1278SetOpMode(RF_OPMODE_SLEEP);

    /* Initialize timer for TX and RX of radio */
    SX1278InitTimer(TimeoutHandler);

    /* Set the clock source=TCXO. */
    SX1278Write(REG_LR_TCXO, SX1278Read(REG_LR_TCXO) | RFLR_TCXO_TCXOINPUT_ON);

    /* Set to LoRa modem */	
    SX1278Write( REG_LR_OPMODE, 
                          (SX1278Read(REG_LR_OPMODE) & RF_OPMODE_LONGRANGEMODE_MASK) |
                          RF_OPMODE_LONGRANGEMODE_ON );
    SX1278Write(REG_LR_DIOMAPPING1, 0x00);
    SX1278Write(REG_LR_DIOMAPPING2, 0x00);

    s_tRadioState = RF_IDLE;	

    return;
}

/**
  * @brief  Get status(idle, tx or rx) of radio.
  * @param  None
  * @retval  RFState_t: status of radio, be one of the @ref RFState_t enumeration.
  */
RadioState_t SX1278GetStatus(void)
{
    return s_tRadioState;
}

/**
  * @brief  Check whether the radio is receiving.
  * @note  It would block the caller if "wPeriodMs" is too large, on the other side, it can not 
  *            detected accurately if "wPeriodMs" is too small. 
  *            Experience indicate 1 millisecond is OK. 
  * @note  Insure the modem is on RX mode.
  * @param  wPeriodMs: millisecond time to check receiving.
  * @retval  TRUE=is receiving; FALSE=is NOT receiving.
  */
bool SX1278IsReceiving(uint16_t wPeriodMs)
{
#if 0  // 0706!!!!!!!!!!!!!!!!!!!!
    uint8_t    byModemStat;
    lora_clock_t    tStart, tClock;

    /* Check whether the channel is free in a period */
    tStart = GET_LORA_CLOCK_NOW();
    tClock = wPeriodMs * MS_2_LORA_CLOCK;

    do
    {
        byModemStat = SX1278Read(REG_LR_MODEMSTAT);
        if (0x01 & byModemStat) /* Bit<0>=Signal detected */
        {
            break; /* Detected signal on air */
        }
    }
    while (GET_LORA_CLOCK_NOW() - tStart < tClock);

    return (byModemStat & 0x01) ? TRUE : FALSE;
#endif
    return FALSE; //0706 !!!!!!!!!
}

/**
  * @brief  Set frequency of SX1278.
  * @param  freq: value of the frequency.
  * @retval  None
  */
void SX1278SetFreq(uint32_t freq)
{
    s_stLoRaSettings.lFrequency = freq;

    freq = ( uint32_t )(( double )freq / ( double )FREQ_STEP);

    SX1278Write(REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF ));
    SX1278Write(REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    SX1278Write(REG_FRFLSB, (uint8_t)(freq & 0xFF));

    return;	
}

/**
  * @brief  Check whether the channel of specified frequency is free.
  * @note  It would block the caller if "wPeriodMs" is too large, on the other side, it can not 
  *            detected accurately if "wPeriodMs" is too small. 
  *            Experience indicate 1 millisecond is OK. 
  * @note  Only for reference that do NOT insure accuracy!
  * @param  wPeriodMs: millisecond time to check channel free.
  * @param  nRssiThresh: threshold of RSSI.
  * @retval  TRUE=channel is clear; FALSE=channel is NOT clear.
  */
bool SX1278IsChannelFree(uint16_t wPeriodMs, int16_t nRssiThresh)
{
    bool    bIsFree;
	#if 0 //0706!!!!!!!!!!!!!!!!!!!!!!!!
    uint8_t    byIrqMask;
    lora_clock_t    tStart, tClock;

    /* Save the state of IRQ mask of LoRa as well as disable all IRQ. */
    byIrqMask = SX1278Read(REG_LR_IRQFLAGSMASK);
    SX1278Write(REG_LR_IRQFLAGSMASK, 0xFF); /* Bit=1 to masks IRQ */
    
    SX1278SetOpMode(RF_OPMODE_RECEIVER);
    DelayMs(1); /* Delay 1 millisecond for waitting LoRa to steady. */

    /* Check whether the channel is free in a period */
    bIsFree = TRUE;

    tStart = GET_LORA_CLOCK_NOW();
    tClock = wPeriodMs * MS_2_LORA_CLOCK;
	
    while (GET_LORA_CLOCK_NOW() - tStart < tClock)
    {
        if (SX1278GetCurrentRssi() > nRssiThresh)
        {
            bIsFree = FALSE;
            break;			
        }
    }
    
    SX1278SetOpMode(RF_OPMODE_SLEEP);

    /* Restore the state of IRQ mask of LoRa */
    SX1278Write(REG_LR_IRQFLAGSMASK, byIrqMask);	
    
    return bIsFree;
	#else
	bIsFree = TRUE;
	return bIsFree;
	#endif
}

/*-------------------------------------------------------------------------*/
uint32_t SX1278Random(void)
{
    int8_t    chCnt;
    uint8_t    byIrqMask;
    uint32_t    ulRandVal;

    /* Save the state of IRQ mask of LoRa as well as disable all IRQ. */
    byIrqMask = SX1278Read(REG_LR_IRQFLAGSMASK);
    SX1278Write(REG_LR_IRQFLAGSMASK, 0xFF); /* Bit=1 to masks IRQ */

    /* Set radio in continuous reception. */
    SX1278SetOpMode(RF_OPMODE_RECEIVER);

    ulRandVal = 0;
    for (chCnt = 0; chCnt < (8 * sizeof(ulRandVal)); ++chCnt)
    {
        DelayMs(1); /* Disperse sampled time. */
        /* Unfiltered RSSI value reading. Only takes the LSB value */
        ulRandVal |= ((uint32_t)SX1278Read(REG_LR_RSSIWIDEBAND) & 0x01) << chCnt;
    }

    SX1278SetOpMode(RF_OPMODE_SLEEP);

    /* Restore the state of IRQ mask of LoRa */
    SX1278Write(REG_LR_IRQFLAGSMASK, byIrqMask);	

    return ulRandVal;
}

/**
  * @brief  Set PA output pin to PA_LF or PA_BOOST.
  * @note  That MUST call "SX1278SetTxPower()" to adjust power after invoked this function. 
  * @param  ePAOutput: the selected PA output pin.
  *              This parameter can be a value of @ref PA_OUTPUT_PIN_TypeDef
  * @retval  None
  */
void SX1278SetPAOutput(PA_OUTPUT_PIN_TypeDef ePAOutput)
{
	Trace_LoRaMac("SX1278SetPAOutput");
    SX1278Write( REG_LR_PACONFIG,
                          (SX1278Read(REG_LR_PACONFIG) & RFLR_PACONFIG_PASELECT_MASK) | 
                          ePAOutput );

    if (PA_OUTPUT_PIN_BOOST != ePAOutput)
    {
        /* High Power settings must be turned off when using PA_LF */
        SX1278Write( REG_PADAC,
                              (SX1278Read(REG_PADAC) & RF_PADAC_20DBM_MASK) |
                              RF_PADAC_20DBM_OFF );
    }
	
    return;
}

/**
  * @brief  Set radio transmit power.
  * @param  chPower: value the setted power.
  * @retval  None
  */
void SX1278SetTxPower(int8_t chPower)
{
    uint8_t    paConfig, paDac;

    if (chPower == s_stLoRaSettings.chPower)
    {
        return; /* Have nothings to do */
    }

    /* Set power to LoRa Radio */	
    paConfig = SX1278Read(REG_PACONFIG);
    paDac = SX1278Read(REG_PADAC);

    /* Set <6:4>=7: Select max output power. */
    paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK) |0x70;

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST)
    {
        if (chPower > 17)
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
		
        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON)
        {
            if (chPower < 5)
            {
                chPower = 5;
            }
            if (chPower > 20)
            {
                chPower = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) |
                            (uint8_t)((uint16_t)(chPower - 5) & 0x0F);
        }
        else
        {
            if (chPower < 2)
            {
                chPower = 2;
            }
            if (chPower > 17)
            {
                chPower = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | 
                            (uint8_t)((uint16_t)(chPower - 2) & 0x0F);
        }
    }
    else
    {
        if (chPower < -1)
        {
            chPower = -1;
        }
        if (chPower > 14)
        {
            chPower = 14;
        }
        paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | 
                        (uint8_t)((uint16_t)(chPower + 1) & 0x0F);
    }
	
    SX1278Write(REG_PACONFIG, paConfig);
    SX1278Write(REG_PADAC, paDac);

    /* Set OCP(Over Current Protection)=160mA, 
        the current is 120mA when RFOP=+20 dBm, on PA_BOOST */
    SX1278Write(REG_LR_OCP, 0x33);	

    s_stLoRaSettings.chPower = chPower;

    return;	
}

/**
  * @brief  Write the changed settings into registers of LoRa.
  * @note  MUST call this function after changed settings of LoRa.
  * @param  None
  * @retval  None
  */
void SX1278SetLoRaSettings(void)
{
    if (FALSE == s_stLoRaSettings.bChanged)
    {
        return; /* Have nothing to do */
    }
    else
    {
        s_stLoRaSettings.bChanged = FALSE; /* Clear flag */
    }

    /* Bw + CodingRate + ImplicitHeaderModeOn */
    SX1278Write( REG_LR_MODEMCONFIG1, 
                          ( SX1278Read(REG_LR_MODEMCONFIG1) &
                            RFLR_MODEMCONFIG1_BW_MASK &
                            RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                            RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           (s_stLoRaSettings.tBW << 4) | 
                           (s_stLoRaSettings.tFEC << 1) | 
                           s_stLoRaSettings.bFixLen );

    /* SpreadingFactor + RxPayloadCrcOn + SymbTimeout(9:8) */
    SX1278Write( REG_LR_MODEMCONFIG2,
                          ( SX1278Read(REG_LR_MODEMCONFIG2) &
                            RFLR_MODEMCONFIG2_SF_MASK &
                            RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                            RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           (s_stLoRaSettings.tSF << 4) |
                           (s_stLoRaSettings.bCrcOn << 2) |
                           ((s_stLoRaSettings.wSymbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) );

    /* SymbTimeout(7:0) */
    SX1278Write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(s_stLoRaSettings.wSymbTimeout & 0xFF));

    /* LowDataRateOptimize */
    SX1278Write( REG_LR_MODEMCONFIG3, 
                          ( SX1278Read( REG_LR_MODEMCONFIG3 ) &
                            RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           (s_stLoRaSettings.bLowDatarateOptimize << 3) );

    /* PreambleLength */
    SX1278Write(REG_LR_PREAMBLEMSB, (s_stLoRaSettings.wPreambleLen >> 8) & 0xFF);
    SX1278Write(REG_LR_PREAMBLELSB, s_stLoRaSettings.wPreambleLen & 0xFF);

    /* SF=6 is a special use case for the highest data rate transmission, 
        several settings must be activated. */
    if(RF_SF_6 == s_stLoRaSettings.tSF)
    {
        SX1278Write( REG_LR_DETECTOPTIMIZE, 
                              ( SX1278Read(REG_LR_DETECTOPTIMIZE) &
                                RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
        SX1278Write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
    }
    else
    {
        SX1278Write( REG_LR_DETECTOPTIMIZE,
                              ( SX1278Read(REG_LR_DETECTOPTIMIZE) &
                                RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
        SX1278Write(REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
    }

    return;
}

/**
  * @brief  Sets preamble length to both TX and RX settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  wLen: length of preamble
  * @retval  None
  */
void SX1278SetPreambleLen(uint16_t wLen)
{
    if (wLen != s_stLoRaSettings.wPreambleLen)
    {
        s_stLoRaSettings.wPreambleLen = wLen;
        s_stLoRaSettings.bChanged = TRUE;
    }

    return;
}

/**
  * @brief  Sets bandwidth to both TX and RX settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data p_byBuf.
  * @param  RadioBW_t: value of bandwidth.
  *              This parameter can be a value of @ref RadioBW_t
  * @retval  None
  */
void SX1278SetBandwidth(RadioBW_t tBW)
{
    if (tBW == s_stLoRaSettings.tBW)
    {
        return; /* Have nothings to do */
    }

    /* Avoid input parameter error */
    if (tBW < RF_BW_7800)
    {
        tBW = RF_BW_7800;
    }
    else if (tBW > RF_BW_500000)
    {
        tBW = RF_BW_500000;
    }	

    s_stLoRaSettings.tBW = tBW;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Sets spreading factor to both TX and RX settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  RadioSF_t: value of spreading factor.
  *              This parameter can be a value of @ref RadioSF_t
  * @retval  None
  */
void SX1278SetSpreadingFactor(RadioSF_t tSF)
{
    if (tSF == s_stLoRaSettings.tSF)
    {
        return; /* Have nothings to do */
    }

    /* Avoid input parameter error */
    if (tSF < RF_SF_6)
    {
        tSF = RF_SF_6;
    }
    else if (tSF > RF_SF_12)
    {
        tSF = RF_SF_12;
    }	

    s_stLoRaSettings.tSF = tSF;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Sets coding rate to both TX and RX settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  RadioFEC_t: value of coding rate.
  *              This parameter can be a value of @ref RadioFEC_t
  * @retval  None
  */
void SX1278SetCodingRate(RadioFEC_t tFEC)
{
    if (tFEC == s_stLoRaSettings.tFEC)
    {
        return; /* Have nothings to do */
    }

    /* Avoid input parameter error */
    if (tFEC < RF_FEC_4_5)
    {
        tFEC = RF_FEC_4_5;
    }
    else if (tFEC > RF_FEC_4_8)
    {
        tFEC = RF_FEC_4_8;
    }	

    s_stLoRaSettings.tFEC = tFEC;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Sets low data rate optimize to both TX and RX settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  bOptimize: ture=Enabled, FALSE=Disabled.
  * @retval  None
  */
void SX1278SetLowDatarateOptimize(bool bOptimize)
{
    if (bOptimize == s_stLoRaSettings.bLowDatarateOptimize)
    {
        return; /* Have nothings to do */
    }

    s_stLoRaSettings.bLowDatarateOptimize = bOptimize;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Sets fixed length to both TX and RX settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  bFixLen: ture=Implicit Header mode, FALSE=Explicit Header mode.
  * @retval  None
  */
void SX1278SetFixLen(bool bFixLen)
{
    if (bFixLen == s_stLoRaSettings.bFixLen)
    {
        return; /* Have nothings to do */
    }

    s_stLoRaSettings.bFixLen = bFixLen;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Sets Rx Payload CRC On to settings of radio.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  bCrcOn: ture=Header indicates CRC on, FALSE=Header indicates CRC off.
  * @retval  None
  */
void SX1278SetCrcOn(bool bCrcOn)
{
    if (bCrcOn == s_stLoRaSettings.bCrcOn)
    {
        return; /* Have nothings to do */
    }

    s_stLoRaSettings.bCrcOn = bCrcOn;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Sets SymbTimeout for RxSingle mode.
  * @note  MUST call "SX1278SetLoRaSettings()" that write radio registers, 
  *            this function ONLY save parameters to data buffer.
  * @param  uint16_t wSymbTimeout  range=[4, 1023] symbols.
  * @retval  None
  */
void SX1278SetSymbTimeout(uint16_t wSymbTimeout)
{
    if (wSymbTimeout == s_stLoRaSettings.wSymbTimeout)
    {
        return; /* Have nothings to do */
    }

    s_stLoRaSettings.wSymbTimeout = wSymbTimeout;
    s_stLoRaSettings.bChanged = TRUE;

    return;
}

/**
  * @brief  Set receive mode to RX_CONTINUOUS or RX_SINGLE.
  * @param  bRxContinuous: TRUE=RX_CONTINUOUS, FALSE=RX_SINGLE.
  * @retval  None
  */
void SX1278SetRxContinuous(bool bRxContinuous)
{
    s_stLoRaSettings.bRxContinuous = bRxContinuous;

    return;
}

/**
  * @brief  Set value of timeout for radio transmission.
  * @note  Be careful overflow that MAX of "unit16_t" is 65535.
  * @param  wTxTimeout: value of timeout that unit is millisecond.
  * @retval  None
  */
void SX1278SetTxTimeout(uint16_t wTxTimeout)
{
    s_stLoRaSettings.wTxTimeout = wTxTimeout;

    return;
}

/*-------------------------------------------------------------------------*/
void SX1278SetIqInverted(bool bIqInverted)
{
    s_stLoRaSettings.bIqInverted = bIqInverted;

    return;
}

/**
  * @brief  Calculate time of a packet that TX or RX by radio.
  * @param  byPayload: size of this packet.
  * @retval  Time of a packet TX or RX by radio that unit is millisecond.
  */
uint16_t SX1278GetTimeOnAir(uint8_t byPayload)
{
    double    dTS, dPayload;

    /* Get time of symbol rate */
    dTS =  (double)(1 << s_stLoRaSettings.tSF) / s_adBW[s_stLoRaSettings.tBW - RF_BW_7800];

    /* Symbol length of payload */
    dPayload = (double)( 8 * byPayload - 4 * s_stLoRaSettings.tSF + 28 +    \
                                  (s_stLoRaSettings.bCrcOn ? 16 : 0) - (s_stLoRaSettings.bFixLen ? 20 : 0) );
    dPayload /= (double)(4 * s_stLoRaSettings.tSF - (s_stLoRaSettings.bLowDatarateOptimize ? 8 : 0));
    dPayload = ceil(dPayload) * (double)(s_stLoRaSettings.tFEC + 4);
    dPayload = 8.0 + ((dPayload > 0) ? dPayload : 0.0);

    dPayload = dTS * (dPayload + s_stLoRaSettings.wPreambleLen + 4.25) * 1000.0; /* Multi 1000 for Sed=>MS */
    dPayload = dPayload * FXOSC_STD / FXOSC;

    return (uint16_t)ceil(dPayload); /* Return ms secs */
}

/**
  * @brief  Calculate time of preamble of a packet.
  * @param  none.
  * @retval  Time of preamble of a packet that unit is millisecond.
  */
uint16_t SX1278GetPreambleTime(void)
{
    double    dTemp;

    /* Get time of symbol rate */
    dTemp =  (double)(1 << s_stLoRaSettings.tSF) / s_adBW[s_stLoRaSettings.tBW - RF_BW_7800];

    dTemp *= (s_stLoRaSettings.wPreambleLen + 4.25);
    dTemp= dTemp * FXOSC_STD / FXOSC;
    dTemp *= 1000.0; /* Multi 1000 for Sed=>MS */

    return (uint16_t)ceil(dTemp); /* Return ms secs */
}

/**
  * @brief  Calculate the preamble number according to time.
  * @param  int16_t wMs    millisecond
  * @retval  preamble number.
  * @note  the number of preamble may LESS than zero.
  */
int16_t SX1278CalcPreambleNum(uint16_t wMs)
{
    double    dTemp;

    dTemp = s_adBW[s_stLoRaSettings.tBW - RF_BW_7800] / (double)(1 << s_stLoRaSettings.tSF);
    dTemp = dTemp * wMs * FXOSC / FXOSC_STD / 1000.0 - 4.25;

    return (int16_t)ceil(dTemp);
}

/**
  * @brief  Calculate the preamble number according to time+BW+SF.
  * @param  uint16_t wMs    millisecond
  * @param  RadioBW_t tBW    bandwidth
  * @param  RadioSF_t tSF    spread factor
  * @retval  preamble number.
  */
uint16_t SX1278CalcPreambleNumExt(uint16_t wMs, RadioBW_t tBW, RadioSF_t tSF)
{
 //   ASSERT( (RF_BW_7800 <= tBW && tBW <= RF_BW_500000) && 
    //              (RF_SF_6 <= tSF && tSF <= RF_SF_12) );  //0706

    double    dTemp;

    dTemp = s_adBW[tBW - RF_BW_7800] / (double)(1 << tSF);
    dTemp = dTemp * wMs * FXOSC / FXOSC_STD / 1000.0 - 4.25;

    return (uint16_t)ceil(dTemp);
}

/**
  * @brief  Calculate the period of CAD.
  * @param  none.
  * @retval  Time of CAD period that unit is millisecond.
  */
uint16_t SX1278CalcCADPeriod(void)
{
    /* Symbols=[1.7, 2.3] when SF=[6, 12] */
    #define MAX_SYMB_OF_CAD    2.3

    double    dTemp;

    /* Get time of symbol rate */
    dTemp =  (double)(1 << s_stLoRaSettings.tSF) / s_adBW[s_stLoRaSettings.tBW - RF_BW_7800];
    dTemp = MAX_SYMB_OF_CAD * dTemp * FXOSC_STD / FXOSC * 1000.0;

    return (uint16_t)ceil(dTemp); /* Return ms secs */
}

/**
  * @brief  Set SX1278 to sleep mode.
  * @param  None
  * @retval  None
  */
void SX1278SetSleep(void)
{
    s_tRadioState = RF_IDLE;

    /* Stop the timer for TX or RX of radio */
    SX1278StopTimer();	

    SX1278SetOpMode(RF_OPMODE_SLEEP);

    return;	
}

/**
  * @brief  Set SX1278 to standby mode.
  * @param  None
  * @retval  None
  */
void SX1278SetStandby(void)
{
    s_tRadioState = RF_IDLE;

    /* Stop the timer for TX or RX of radio */
   // SX1278StopTimer();	//0706!!!!!!!!!!!!!!!!!!!!

    SX1278SetOpMode(RF_OPMODE_STANDBY);

    return;	
}

/**
  * @brief  Set registers of FIFO as well as copy data to it but have not transmitted.
  * @note  To send a packet that MUST call "SX1278Transmit()" after invoked this function.
  * @param  p_byBuf: point to buffer that saved the desired transmitted data by this pointer.
  * @param  bySize: size of the desired transmitted data.
  * @retval  Result of invoked.
  *             -1=Error that radio is busy; 0=Set FIFO OK.
  */
int8_t SX1278PrepareTx(const uint8_t *p_byBuf, uint8_t bySize)
{
    bool    bPrevModeIsSleep;

  //  ASSERT(p_byBuf && bySize > 0); //0706!!!!!!!!!!!!!!!

    if (RF_TX_RUNNING == s_tRadioState)
    {
	#if 0 //0709	£¬Ç¿ÐÐ·¢
        return -1; /* Error: radio is tx now! */
	#endif
    }

    s_stPacketHandler.bySize = bySize;

    if (s_stLoRaSettings.bIqInverted)
    {
        SX1278Write(REG_LR_INVERTIQ, ((SX1278Read( REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
        SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
    }
    else
    {
        SX1278Write(REG_LR_INVERTIQ, ((SX1278Read( REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
        SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
    }

    /* Static configuration registers can be accessed in Sleep, Stand-by or FSTX mode. */
    SX1278Write(REG_LR_PAYLOADLENGTH, bySize);

    /* Full buffer used for Tx */
    SX1278Write(REG_LR_FIFOTXBASEADDR, 0);
    SX1278Write(REG_LR_FIFOADDRPTR, 0);

    /* Set mode to STAND-BY even the previous is this mode otherwise incurred TX failed. */
    bPrevModeIsSleep =    \
        RF_OPMODE_SLEEP == (SX1278Read(REG_OPMODE) & ~RF_OPMODE_MASK);
    SX1278SetOpMode(RF_OPMODE_STANDBY);
    if (bPrevModeIsSleep)
    {
		#if 0 //0709
        DelayMs(1); /* Delay 1 millisecond for waitting LoRa to steady. */
		#else
		uint16_t tmp = 1000;
		while(tmp>0)
		{
			tmp = tmp - 1;
		}
		#endif
    }

    /* The LoRa FIFO can only be filled in Stand-by Mode. */
    SX1278WriteFifo(p_byBuf, bySize);

    return 0;
}

/**
  * @brief  Fire radio to transmission assumed that prepared data have been finished.
  * @note  Before invoked this function that MUST call "SX1278PrepareTx()".
  * @param  None
  * @retval  None
  */
void SX1278Transmit(void)
{
    SX1278SetSwPin2Tx();

    /* Enable interrupt = TxDone */
    SX1278Write( REG_LR_IRQFLAGSMASK,
                         RFLR_IRQFLAGS_RXTIMEOUT |
                         RFLR_IRQFLAGS_RXDONE |
                         RFLR_IRQFLAGS_PAYLOADCRCERROR |
                         RFLR_IRQFLAGS_VALIDHEADER |
                         // RFLR_IRQFLAGS_TXDONE |
                         RFLR_IRQFLAGS_CADDONE |
                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                         RFLR_IRQFLAGS_CADDETECTED );

    /* DIO0<01> = TxDone */
    SX1278Write( REG_DIOMAPPING1, 
                          (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) |
                          RFLR_DIOMAPPING1_DIO0_01 );

    s_tRadioState = RF_TX_RUNNING;

    SX1278SetOpMode(RF_OPMODE_TRANSMITTER);

    //ASSERT(s_stLoRaSettings.wTxTimeout > 0); //0706

	#if 0 //0706
    SX1278StartTimer(s_stLoRaSettings.wTxTimeout);

    chip_LEDOn();
   #endif
   
#if EXTI_HALT_UART
    pm_OpposeHalt(PWR_ID_RF);
#endif

    return;
}

/**
  * @brief  Send a packet through LoRa modem.
  * @note  It determinated conduct to 2 results as Tx_Done or Tx_Timeout, so the caller
  *            needs to catch this 2 events.
  * @param  p_byBuf: point to buffer that saved the desired transmitted data by this pointer.
  * @param  bySize: size of the desired transmitted data.
  * @retval  Result of invoked.
  *             -1=Error that radio is busy; 0=Set FIFO OK.
  */
int8_t SX1278Send(const void *p_vBuf, uint8_t bySize)
{
 //   ASSERT(p_vBuf && bySize > 0);  //0706

    int8_t    chResult;

    chResult = SX1278PrepareTx((const uint8_t *)p_vBuf, bySize);
    if (chResult >= 0)
    {
        SX1278Transmit();
    }

    return chResult;		
}

/**
  * @brief  Enable receive of SX1278.
  * @param  wTimeout: value of timeout that unit is millisecond.
  * @retval  Result of invoked.
  *             -1=Error that radio is busy; 0=Set receive OK.
  */
int8_t SX1278Receive(uint16_t wTimeout)
{
    if (RF_IDLE != s_tRadioState)
    {
        return -1; /* Error: radio is busy! */
    }

    if (s_stLoRaSettings.bIqInverted)
    {
        SX1278Write(REG_LR_INVERTIQ, ((SX1278Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
        SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
    }
    else
    {
        SX1278Write(REG_LR_INVERTIQ, ((SX1278Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
        SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
    }

    /* ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal */
    if (s_stLoRaSettings.tBW < RF_BW_500000)
    {
        /* Set Bit 7 at address 0x31 to 0 */
        SX1278Write(REG_LR_DETECTOPTIMIZE, SX1278Read(REG_LR_DETECTOPTIMIZE) & 0x7F);
        SX1278Write(0x30, 0x00); /* Value at address 0x30 */
        switch (s_stLoRaSettings.tBW)
        {
            case RF_BW_7800:
                SX1278Write(0x2F, 0x48);
                SX1278SetFreq(s_stLoRaSettings.lFrequency + 7810);
                break;
            case RF_BW_10400:
                SX1278Write(0x2F, 0x44);
                SX1278SetFreq(s_stLoRaSettings.lFrequency + 10420);
                break;
            case RF_BW_15600:
                SX1278Write(0x2F, 0x44);
                SX1278SetFreq(s_stLoRaSettings.lFrequency + 15620);
                break;
            case RF_BW_20800:
                SX1278Write(0x2F, 0x44);
                SX1278SetFreq(s_stLoRaSettings.lFrequency + 20830);
                break;
            case RF_BW_31250:
                SX1278Write(0x2F, 0x44);
                SX1278SetFreq(s_stLoRaSettings.lFrequency + 31250);
                break;
            case RF_BW_41700:
                SX1278Write(0x2F, 0x44);
                SX1278SetFreq(s_stLoRaSettings.lFrequency + 41670);
                break;
            case RF_BW_62500:
                SX1278Write(0x2F, 0x40);
                break;
            case RF_BW_125000:
                SX1278Write(0x2F, 0x40);
                break;
            case RF_BW_250000:
                SX1278Write(0x2F, 0x40);
                break;
            default:
          //      ASSERT(!"Error: bad bandwidth!\r\n"); //0706
                break;
        }
    }
    else /* 500kHz */
    {
        /* Set Bit 7 at address 0x31 to 1 */
        SX1278Write(REG_LR_DETECTOPTIMIZE, SX1278Read(REG_LR_DETECTOPTIMIZE) | 0x80);
    }

    SX1278SetSwPin2Rx();

    /* Enable interrupt = RxTimeout + RxDone + PayloadCrcError */
    SX1278Write( REG_LR_IRQFLAGSMASK, 
                         // RFLR_IRQFLAGS_RXTIMEOUT |
                         // RFLR_IRQFLAGS_RXDONE |
                         // RFLR_IRQFLAGS_PAYLOADCRCERROR |
                         RFLR_IRQFLAGS_VALIDHEADER |
                         RFLR_IRQFLAGS_TXDONE |
                         RFLR_IRQFLAGS_CADDONE |
                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                         RFLR_IRQFLAGS_CADDETECTED );
            
    /* DIO0<00> = RxDone */
    SX1278Write( REG_DIOMAPPING1, 
                          (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | 
                          RFLR_DIOMAPPING1_DIO0_00 );

    /* Full buffer used for Rx */	
    SX1278Write(REG_LR_FIFORXBASEADDR, 0);
    SX1278Write(REG_LR_FIFOADDRPTR, 0);

    s_tRadioState = RF_RX_RUNNING;

    if (TRUE == s_stLoRaSettings.bRxContinuous)
    {
        /* EXPLAIN: On RxContinuous mode the chip can NOT generates IRQ of "RxTimeout",
	      that may needs a "timer" to make timeout. */
        SX1278SetOpMode(RFLR_OPMODE_RECEIVER);
    }
    else
    {
        /* EXPLAIN: If a preamble hasn't been found at the end of the time window in "RegSymbTimeout",
            the chip generates the RxTimeout interrupt and goes back to stand-by mode. */
        SX1278SetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
    }

    if (0 < wTimeout)
    {
        SX1278StartTimer(wTimeout);
    }

#if EXTI_HALT_UART
    pm_OpposeHalt(PWR_ID_RF);
#endif

    return 0;
}

/**
  * @brief  Set LoRa to CAD mode.
  * @note  The period of CAD is about 800 us.
  * @param  wTimeout: the unit is millisecond, 0=don't care.
  * @retval  Result of invoked.
  *             -1=Error that radio is busy; 0=Set CAD OK.
  */
int8_t SX1278SetCAD(uint16_t wTimeout)
{
    if (RF_IDLE != s_tRadioState)
    {
        return -1; /* Error: radio is busy! */
    }

    SX1278SetSwPin2Rx(); /* CAD needs receive signal */

    /* Enable interrupt = CadDone + CadDetected */
    SX1278Write( REG_LR_IRQFLAGSMASK, 
                         RFLR_IRQFLAGS_RXTIMEOUT |
                         RFLR_IRQFLAGS_RXDONE |
                         RFLR_IRQFLAGS_PAYLOADCRCERROR |
                         RFLR_IRQFLAGS_VALIDHEADER |
                         RFLR_IRQFLAGS_TXDONE |
                         //RFLR_IRQFLAGS_CADDONE |
                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL
                         //RFLR_IRQFLAGS_CADDETECTED 
                         );
            
    /* DIO0<10> = CadDone */
    SX1278Write( REG_DIOMAPPING1, 
                          (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | 
                          RFLR_DIOMAPPING1_DIO0_10 );

    s_tRadioState = RF_CAD;

    /* Set LoRa to CAD mode */
    SX1278SetOpMode(RFLR_OPMODE_CAD);

    /* Start timer for timeout of CAD */
    if (wTimeout > 0)
    {
        SX1278StartTimer(wTimeout);
    }

#if EXTI_HALT_UART
    pm_OpposeHalt(PWR_ID_RF);
#endif

    return 0;
}

/**
  * @brief  Handler for IRQ of DIO_0.
  * @note  Insure callback functions are SMALL, FAST and SAFE that would been invoked by ISR. 
  * @param  None
  * @retval  None
  */
void SX1278Dio0IrqHandler(void)
{
    uint8_t    byRegVal;

 //   SX1278StopTimer(); /* At first stop the timer */  //0706!!!!!!!!!!!!!!!!!!!!

    if (RF_TX_RUNNING == s_tRadioState) /* Tx Done */
    {
        /* Clear Irq of "TxDone" */
        SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);

        s_tRadioState = RF_IDLE;
		
    //    chip_LEDOff(); //0706!!!!!!!!!!!!!!
        if (s_pstRadioEvents && s_pstRadioEvents->TxDone)
        {
            s_pstRadioEvents->TxDone();
        }
    }
    else if (RF_RX_RUNNING == s_tRadioState) /* Rx Done */
    {
        /* Clear Irq of "RxDone" */
        SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

        if ( (SX1278Read(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) ==
              RFLR_IRQFLAGS_PAYLOADCRCERROR ) /* Is PayloadCrcError */
        {
            /* Clear Irq of "PayloadCrcError" */
            SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

            if (s_pstRadioEvents && s_pstRadioEvents->RxError)
            {
                s_pstRadioEvents->RxError();
            }    

        #if CATCH_EXCEPTION
            ++s_wRFRxErrorCnt;
        #endif
        }
        else /* Is a integrity packet */
        {
            int16_t    nRssi;

            /* Get value of SNR */
            byRegVal = SX1278Read(REG_LR_PKTSNRVALUE);			
            s_stPacketHandler.chSNR = (int8_t)byRegVal / 4; /* Divide by 4 */			

            /* Change calculate of "RSSI" according to "LoRaWAN-SX1276.c" */
            byRegVal = SX1278Read(REG_LR_PKTRSSIVALUE);
            nRssi = (int16_t)byRegVal;
            s_stPacketHandler.nRSSI = RSSI_OFFSET_LF + nRssi + nRssi >> 4;
            if (s_stPacketHandler.chSNR < 0)
            {
                s_stPacketHandler.nRSSI += s_stPacketHandler.chSNR;
            }

            /* Get length and data of packet */
            s_stPacketHandler.bySize = SX1278Read(REG_LR_RXNBBYTES);			
            SX1278ReadFifo(s_abyRxBuf, s_stPacketHandler.bySize);

      //      chip_LED2On(); //0706!!!!!!!!!!
            if (s_pstRadioEvents && s_pstRadioEvents->RxDone)
            {
                s_pstRadioEvents->RxDone( s_abyRxBuf,
                                                              s_stPacketHandler.bySize,
                                                              s_stPacketHandler.nRSSI,
                                                              s_stPacketHandler.chSNR );
            }
        }
		
        if (FALSE == s_stLoRaSettings.bRxContinuous)
        {
            s_tRadioState = RF_IDLE;
        }
    }
    else if (RF_CAD == s_tRadioState) /* CAD Done */
    {
        bool    bIsCadDetected;
		
        /* Clear Irq of "CadDone" */
        SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);

        if ( (SX1278Read(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED_MASK) ==
              RFLR_IRQFLAGS_CADDETECTED ) /* Is CadDetected */
        {
            /* Clear Irq of "CadDetected" */
            SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED);
        
            bIsCadDetected = TRUE;
        }
        else
        {
            bIsCadDetected = FALSE;
        }

        s_tRadioState = RF_IDLE;
		
        if (s_pstRadioEvents && s_pstRadioEvents->CadDone)
        {
            s_pstRadioEvents->CadDone(bIsCadDetected);
        }    
    }
    else /* Error */
    {
        /* EXPLAIN: run to here meas timeout is too short! */
 //       RIME_DBG2(RIME_DBG_ON, "Error: RF is IDLE but DIO0 IRQ!\r\n", 0, PRINTF_FORMAT_NONE); //0706!!!!!!!!
    }

#if EXTI_HALT_UART
    pm_AgrHalt(PWR_ID_RF);
#endif

    return;
}

/**
  * @brief  Handler for IRQ of DIO_1.
  * @note  Insure callback functions are SMALL, FAST and SAFE that would been invoked by ISR. 
  * @param  None
  * @retval  None
  */
void SX1278Dio1IrqHandler(void)
{
    /* stop timer */
//    SX1278StopTimer(); //0706!!!!!!!!!!!!!

    /* clear IRQ of "RxTimeout" */
    SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);

    s_tRadioState = RF_IDLE;
	
    /* invoke callback function */
    if (s_pstRadioEvents && s_pstRadioEvents->RxTimeout)
    {
        s_pstRadioEvents->RxTimeout();
    }    

#if EXTI_HALT_UART
    pm_AgrHalt(PWR_ID_RF);
#endif

    return;
}


/*-------------------------------------------------------------------------*/
#if CATCH_EXCEPTION
uint8_t rf_GetException(char *p_chBuf)
{
    uint8_t    byLen;
    RadioState_t    tRFState;

    static const char    *a_pchRF[] =
    {
        "RF_IDLE",
        "RF_RX_RUNNING",
        "RF_TX_RUNNING",
        "RF_CAD",
    };

    tRFState = SX1278GetStatus();

    byLen = 0;
    byLen += FormatPrintf(&p_chBuf[byLen], a_pchRF[tRFState], 0, PRINTF_FORMAT_NONE);
    byLen += FormatPrintf(&p_chBuf[byLen], ", tx timeout=", s_wRFTxTimeoutCnt, PRINTF_FORMAT_DEC);
    byLen += FormatPrintf(&p_chBuf[byLen], ", rx error=", s_wRFRxErrorCnt, PRINTF_FORMAT_DEC);

    return byLen;
}
#endif


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

