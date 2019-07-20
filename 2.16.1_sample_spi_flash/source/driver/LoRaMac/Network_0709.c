/**
 * \file
 *         Network.c
 * \description
 *         Operation of communicated to LoRa Gateway
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-03-30 16:56
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
//#include "pt.h"
#include "process.h"
#include "packetbuf.h"
#include "rtimer.h"
#include "etimer.h"
#include "main.h"
#include "SystSettings.h"
#include "Dbg.h"
#include "Comm2Trm.h"
#include "sx1278_src.h"
#include "Chip.h"
#include "PwrManage.h"
#include "LoRaMac.h"


/* Compile switch-------------------------------------------------------------*/
#define EVENT_STATUS_DBG    RIME_DBG_ON
#define RX_DATA_DBG    RIME_DBG_ON

/* MUST disable TEST on RELEASE mode. */
#if REL_VER
#undef EVENT_STATUS_DBG
#undef RX_DATA_DBG
#define EVENT_STATUS_DBG    RIME_DBG_OFF
#define RX_DATA_DBG    RIME_DBG_OFF
#endif

#define JOIN_OTAA    1 /* 0=ABP, 1=OTAA */

/* Private typedef ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/**
* @brief  Event for Network process
*/
#define NETWORK_EVENT_UPLINK    (PROCESS_EVENT_MAX + 1)


/* Private function prototypes --------------------------------------------------*/
static void MacMcpsConfirm(McpsConfirm_t *p_stMcpsConfirm);
static void MacMcpsIndication(McpsIndication_t *p_stMcpsIndication);
static void MacMlmeConfirm(MlmeConfirm_t *p_stMlmeConfirm);
static uint8_t GetBatteryLevel();


/* Private Constants ----------------------------------------------------------*/


/* Private variables -----------------------------------------------------------*/
PROCESS_NAME(NetworkProcess);

static McpsIndication_t    *s_pstMcpsIndication = NULL;

static LoRaMacPrimitives_t    s_stLoRaMacPrimitives =
{
    .MacMcpsConfirm = MacMcpsConfirm,
    .MacMcpsIndication = MacMcpsIndication,
    .MacMlmeConfirm = MacMlmeConfirm,
};

static LoRaMacCallback_t    s_stLoRaMacCallback =
{
    .GetBatteryLevel = GetBatteryLevel,
};

#if JOIN_OTAA
static bool    s_bJoined = false;
#else
const static uint8_t    s_abyNwkSKey[16] =
{ 0x2B, 0x7B, 0x50, 0xFC, 0x40, 0x82, 0x55, 0x04, 0x11, 0xC6, 0x8E, 0x8E, 0x00, 0x75, 0x9D, 0x9B };
const static uint8_t    s_abyAppSKey[16] =
{ 0xB2, 0x92, 0x2A, 0xEB, 0xF4, 0x5C, 0x5D, 0x4C, 0x41, 0x3A, 0x12, 0xCE, 0x44, 0xEC, 0x40, 0xE1 };
#endif

/**
* @brief  Print the string of "LoRaMacEventInfoStatus_t".
*/
#if (DBG_VER)
/*---------------------------------------------------------------------------*/
static char    *s_apchLoRaMacEventInfoStatus[] =
{
    "OK",
    "ERROR",
    "TX_TIMEOUT",
    "RX2_TIMEOUT",
    "RX2_ERROR",
    "JOIN_FAIL",
    "DOWNLINK_REPEATED",
    "TX_DR_PAYLOAD_SIZE_ERROR",
    "DOWNLINK_TOO_MANY_FRAMES_LOSS",
    "ADDRESS_FAIL",
    "MIC_FAIL",
};

/*---------------------------------------------------------------------------*/
static char* ConvertStatus2String(LoRaMacEventInfoStatus_t tStatus)
{
    int8_t    chIndex;

    chIndex = tStatus - LORAMAC_EVENT_INFO_STATUS_OK;
    if (0 <= chIndex && chIndex < SIZE_OF_ARRAY(s_apchLoRaMacEventInfoStatus))	
    {
        return s_apchLoRaMacEventInfoStatus[chIndex];
    }
    else
    {
        return "Error: bad LoRaMacEventInfoStatus_t!";
    }
}
#endif


/*---------------------------------------------------------------------------*/
static void MacMcpsConfirm(McpsConfirm_t *p_stMcpsConfirm)
{
    RIME_DBG2(EVENT_STATUS_DBG, "\r\nMcpsConfirm: Status=", 0, PRINTF_FORMAT_NONE);
    RIME_DBG2(EVENT_STATUS_DBG, ConvertStatus2String(p_stMcpsConfirm->Status), 0, PRINTF_FORMAT_NONE);

    return;
}

/*---------------------------------------------------------------------------*/
static void MacMcpsIndication(McpsIndication_t *p_stMcpsIndication)
{
    RIME_DBG2(EVENT_STATUS_DBG, "\r\nMcpsIndication: Status=", 0, PRINTF_FORMAT_NONE);
    RIME_DBG2(EVENT_STATUS_DBG, ConvertStatus2String(p_stMcpsIndication->Status), 0, PRINTF_FORMAT_NONE);

    if (LORAMAC_EVENT_INFO_STATUS_OK == p_stMcpsIndication->Status)
    {
        /* EXPLAIN: can NOT call "process_post()" since this called by RTC_ISR */
        s_pstMcpsIndication = p_stMcpsIndication;
        process_poll(&NetworkProcess);
    }

    return;
}

/*---------------------------------------------------------------------------*/
static void MacMlmeConfirm(MlmeConfirm_t *p_stMlmeConfirm)
{
    RIME_DBG2(EVENT_STATUS_DBG, "\r\nMlmeConfirm: Status=", 0, PRINTF_FORMAT_NONE);
    RIME_DBG2(EVENT_STATUS_DBG, ConvertStatus2String(p_stMlmeConfirm->Status), 0, PRINTF_FORMAT_NONE);

#if JOIN_OTAA
    if ( MLME_JOIN == p_stMlmeConfirm->MlmeRequest &&
          LORAMAC_EVENT_INFO_STATUS_OK == p_stMlmeConfirm->Status )
    {
        s_bJoined = true;
    }
#endif

    process_poll(&NetworkProcess);

    return;
}

/*---------------------------------------------------------------------------*/
static uint8_t GetBatteryLevel()
{
    /* 255: the node was not able to measure the battery level. */
    return 255;
}

/*---------------------------------------------------------------------------*/
static uint8_t GetTxPwrIdx(uint8_t pwr)
{
    uint8_t byPwrIdx;
    
    switch(pwr)
    {
        case 0x14:
            byPwrIdx = TX_POWER_20_DBM;
            break;
        case 0x11:
            byPwrIdx = TX_POWER_17_DBM;
            break;
        case 0x10:
            byPwrIdx = TX_POWER_16_DBM;
            break;
        case 0x0E:
            byPwrIdx = TX_POWER_14_DBM;
            break;
        case 0x0C:
            byPwrIdx = TX_POWER_12_DBM;
            break;
        case 0x0A:
            byPwrIdx = TX_POWER_10_DBM;
            break;
        case 0x07:
            byPwrIdx = TX_POWER_7_DBM;
            break;
        case 0x05:
            byPwrIdx = TX_POWER_5_DBM;
            break;
        case 0x02:
            byPwrIdx = TX_POWER_2_DBM;
            break;
        default:
            byPwrIdx = LORAMAC_DEFAULT_TX_POWER;
            break;
    }

    return byPwrIdx;
}

extern NetSettings_t    s_stNetSettingsC2T;
/*---------------------------------------------------------------------------*/
#if 0
PROCESS(NetworkProcess, "network process");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(NetworkProcess, ev, data)
{
    /* ATTENTION: stack variables can NOT cross any "XX_YIELD()". */
    MibRequestConfirm_t    l_tMibReq;

    PROCESS_BEGIN();

    rtc_EnAlarm();

    LoRaMacInitialization(&s_stLoRaMacPrimitives, &s_stLoRaMacCallback);

    /* Set this after INIT, iWL883A RF_TX pin connected to SX1278_BOOST. */
    SX1278SetPAOutput(PA_OUTPUT_PIN_BOOST);
    
    l_tMibReq.Type = MIB_DEVICE_CLASS;
    l_tMibReq.Param.Class=(s_stNetSettingsC2T.byClassType==0x02)?(CLASS_C):(CLASS_A);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_CHANNELS_TX_POWER;
    l_tMibReq.Param.ChannelsTxPower=GetTxPwrIdx(s_stNetSettingsC2T.byTxPwr);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_PUBLIC_NETWORK;
    l_tMibReq.Param.EnablePublicNetwork = true;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

#if JOIN_OTAA
    MlmeReq_t    l_tMlmeReq;
    while (!s_bJoined)
    {
        l_tMlmeReq.Type = MLME_JOIN;
        l_tMlmeReq.Req.Join.DevEui = chip_GetDevEUI();
        l_tMlmeReq.Req.Join.AppEui = s_stNetSettingsC2T.a_byAppEUI;
        l_tMlmeReq.Req.Join.AppKey = s_stNetSettingsC2T.a_byAppKey;
        l_tMlmeReq.Req.Join.NbTrials = 3;
        LoRaMacMlmeRequest(&l_tMlmeReq);

        PROCESS_YIELD_UNTIL(PROCESS_EVENT_POLL == ev);
    }
#else
    l_tMibReq.Type = MIB_DEV_ADDR;
    l_tMibReq.Param.DevAddr = 0x26041FDA;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_NWK_SKEY;
    l_tMibReq.Param.NwkSKey = (uint8_t *)s_abyNwkSKey;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_APP_SKEY;
    l_tMibReq.Param.AppSKey = (uint8_t *)s_abyAppSKey;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_NETWORK_JOINED;
    l_tMibReq.Param.IsNetworkJoined = true;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);
#endif /*#if JOIN_OTAA*/
    l_tMibReq.Type = MIB_ADR;
    l_tMibReq.Param.AdrEnable = (s_stNetSettingsC2T.byAdr==0x00)?(false):(true);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_CHANNELS_TX_POWER;
    l_tMibReq.Param.ChannelsTxPower=GetTxPwrIdx(s_stNetSettingsC2T.byTxPwr);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_CHANNELS_DATARATE;
    l_tMibReq.Param.ChannelsDatarate=s_stNetSettingsC2T.byDataRate;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    while (TRUE)
    {	
        /* Block process until received an event */
        PROCESS_YIELD();

        if (NETWORK_EVENT_UPLINK == ev)
        {
            UPLINK_DATA_BUF *p_stUplinkDataBuf;
            McpsReq_t    l_stMcpsReq;

            p_stUplinkDataBuf = (UPLINK_DATA_BUF *)data;
            if(p_stUplinkDataBuf->byType == 0)
            {
                l_stMcpsReq.Type = MCPS_UNCONFIRMED;
                l_stMcpsReq.Req.Unconfirmed.fPort = p_stUplinkDataBuf->byPort;
                l_stMcpsReq.Req.Unconfirmed.Datarate = s_stNetSettingsC2T.byDataRate;
                l_stMcpsReq.Req.Unconfirmed.fBuffer = (void *)&p_stUplinkDataBuf->a_byDataBuf[0];
                l_stMcpsReq.Req.Unconfirmed.fBufferSize = (uint16_t)p_stUplinkDataBuf->byDataSize;
            }
            else
            {
                l_stMcpsReq.Type = MCPS_CONFIRMED;
                l_stMcpsReq.Req.Confirmed.fPort = p_stUplinkDataBuf->byPort;
                l_stMcpsReq.Req.Confirmed.Datarate = s_stNetSettingsC2T.byDataRate;
                l_stMcpsReq.Req.Confirmed.NbTrials = 3;
                l_stMcpsReq.Req.Confirmed.fBuffer = (void *)&p_stUplinkDataBuf->a_byDataBuf[0];
                l_stMcpsReq.Req.Confirmed.fBufferSize = (uint16_t)p_stUplinkDataBuf->byDataSize;
            }
            LoRaMacMcpsRequest(&l_stMcpsReq);
        }
        else if (PROCESS_EVENT_POLL == ev)
        {
            if((s_pstMcpsIndication->BufferSize > 0) || (true == s_pstMcpsIndication->AckReceived))
            {
                comm2trm_RxWakeData((void *)s_pstMcpsIndication);
            }
        #if (RIME_DBG_ON == RX_DATA_DBG)
            RIME_DBG2(RX_DATA_DBG, ", size=", s_pstMcpsIndication->BufferSize, PRINTF_FORMAT_DEC);
            RIME_DBG2(RX_DATA_DBG, "\r\n", 0, PRINTF_FORMAT_NONE);
            dbg_PrintfBufData(s_pstMcpsIndication->Buffer, s_pstMcpsIndication->BufferSize);
        #endif
        }
        else
        {
            ASSERT(!"NetworkProcess: Bad event!\r\n");
        }
    }/*while (TRUE)*/

    PROCESS_END();
}

#endif
//***å*å*å*å*å*å**å*å*å*å*å**å*å*å*å*å*å**add in 0706-----------

/*---------------------------------------------------------------------------*/
void Trace_LoRaMac(char * str)
{
	//print str
}
void LoRaMac_init_and_register(void)
{
    /* ATTENTION: stack variables can NOT cross any "XX_YIELD()". */
   Trace_LoRaMac("LoRaMac_init_and_register"); //0709
   
    MibRequestConfirm_t    l_tMibReq;

#if 0
    PROCESS_BEGIN();

    rtc_EnAlarm();
#endif

    LoRaMacInitialization(&s_stLoRaMacPrimitives, &s_stLoRaMacCallback);

    /* Set this after INIT, iWL883A RF_TX pin connected to SX1278_BOOST. */
    SX1278SetPAOutput(PA_OUTPUT_PIN_BOOST);
    
    l_tMibReq.Type = MIB_DEVICE_CLASS;
    l_tMibReq.Param.Class=(s_stNetSettingsC2T.byClassType==0x02)?(CLASS_C):(CLASS_A);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_CHANNELS_TX_POWER;
    l_tMibReq.Param.ChannelsTxPower=GetTxPwrIdx(s_stNetSettingsC2T.byTxPwr);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_PUBLIC_NETWORK;
    l_tMibReq.Param.EnablePublicNetwork = true;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

#if JOIN_OTAA
    MlmeReq_t    l_tMlmeReq;
    while (!s_bJoined)
    {
        l_tMlmeReq.Type = MLME_JOIN;
        l_tMlmeReq.Req.Join.DevEui = chip_GetDevEUI();
        l_tMlmeReq.Req.Join.AppEui = s_stNetSettingsC2T.a_byAppEUI;
        l_tMlmeReq.Req.Join.AppKey = s_stNetSettingsC2T.a_byAppKey;
        l_tMlmeReq.Req.Join.NbTrials = 3;
        LoRaMacMlmeRequest(&l_tMlmeReq);

//        PROCESS_YIELD_UNTIL(PROCESS_EVENT_POLL == ev); //0709
    }
#else
    l_tMibReq.Type = MIB_DEV_ADDR;
    l_tMibReq.Param.DevAddr = 0x26041FDA;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_NWK_SKEY;
    l_tMibReq.Param.NwkSKey = (uint8_t *)s_abyNwkSKey;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_APP_SKEY;
    l_tMibReq.Param.AppSKey = (uint8_t *)s_abyAppSKey;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_NETWORK_JOINED;
    l_tMibReq.Param.IsNetworkJoined = true;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);
#endif /*#if JOIN_OTAA*/
    l_tMibReq.Type = MIB_ADR;
    l_tMibReq.Param.AdrEnable = (s_stNetSettingsC2T.byAdr==0x00)?(false):(true);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_CHANNELS_TX_POWER;
    l_tMibReq.Param.ChannelsTxPower=GetTxPwrIdx(s_stNetSettingsC2T.byTxPwr);
    LoRaMacMibSetRequestConfirm(&l_tMibReq);

    l_tMibReq.Type = MIB_CHANNELS_DATARATE;
    l_tMibReq.Param.ChannelsDatarate=s_stNetSettingsC2T.byDataRate;
    LoRaMacMibSetRequestConfirm(&l_tMibReq);
#if 0
    while (TRUE)
    {	
        /* Block process until received an event */
        PROCESS_YIELD();

        if (NETWORK_EVENT_UPLINK == ev)
        {
            UPLINK_DATA_BUF *p_stUplinkDataBuf;
            McpsReq_t    l_stMcpsReq;

            p_stUplinkDataBuf = (UPLINK_DATA_BUF *)data;
            if(p_stUplinkDataBuf->byType == 0)
            {
                l_stMcpsReq.Type = MCPS_UNCONFIRMED;
                l_stMcpsReq.Req.Unconfirmed.fPort = p_stUplinkDataBuf->byPort;
                l_stMcpsReq.Req.Unconfirmed.Datarate = s_stNetSettingsC2T.byDataRate;
                l_stMcpsReq.Req.Unconfirmed.fBuffer = (void *)&p_stUplinkDataBuf->a_byDataBuf[0];
                l_stMcpsReq.Req.Unconfirmed.fBufferSize = (uint16_t)p_stUplinkDataBuf->byDataSize;
            }
            else
            {
                l_stMcpsReq.Type = MCPS_CONFIRMED;
                l_stMcpsReq.Req.Confirmed.fPort = p_stUplinkDataBuf->byPort;
                l_stMcpsReq.Req.Confirmed.Datarate = s_stNetSettingsC2T.byDataRate;
                l_stMcpsReq.Req.Confirmed.NbTrials = 3;
                l_stMcpsReq.Req.Confirmed.fBuffer = (void *)&p_stUplinkDataBuf->a_byDataBuf[0];
                l_stMcpsReq.Req.Confirmed.fBufferSize = (uint16_t)p_stUplinkDataBuf->byDataSize;
            }
            LoRaMacMcpsRequest(&l_stMcpsReq);
        }
        else if (PROCESS_EVENT_POLL == ev)
        {
            if((s_pstMcpsIndication->BufferSize > 0) || (true == s_pstMcpsIndication->AckReceived))
            {
                comm2trm_RxWakeData((void *)s_pstMcpsIndication);
            }
        #if (RIME_DBG_ON == RX_DATA_DBG)
            RIME_DBG2(RX_DATA_DBG, ", size=", s_pstMcpsIndication->BufferSize, PRINTF_FORMAT_DEC);
            RIME_DBG2(RX_DATA_DBG, "\r\n", 0, PRINTF_FORMAT_NONE);
            dbg_PrintfBufData(s_pstMcpsIndication->Buffer, s_pstMcpsIndication->BufferSize);
        #endif
        }
        else
        {
            ASSERT(!"NetworkProcess: Bad event!\r\n");
        }
    }/*while (TRUE)*/

    PROCESS_END();
#endif	
}

/**
  * @brief  Fetch settings and set RF as well as initialize process.
  * @param  None
  * @retval  None
  */
void network_Init(void)
{
    /* Initialize Network-Process. */
    process_start(&NetworkProcess, NULL);

    return;
}

/**
  * @brief  Inform Network Process to uplink a frame to gateway.
  * @param  p_vUpData.
  * @retval  None.
  */
void network_Uplink(void *p_vUpData)
{
    process_post(&NetworkProcess, NETWORK_EVENT_UPLINK, p_vUpData);

    return;
}

/**
  * @brief  Get result of join procedure.
  * @param  None.
  * @retval  None.
  */
bool network_Checklink(void)
{
#if JOIN_OTAA
    return(s_bJoined);
#else
    return(true);
#endif
}


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

