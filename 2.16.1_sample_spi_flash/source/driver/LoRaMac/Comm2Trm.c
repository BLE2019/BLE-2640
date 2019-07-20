/**
 * \file
 *         Comm2Trm.c
 * \description
 *         Communicate to terminal like as User System.
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-04-02 15:15
 * \copyright
 *         (c) RimeLink (www.rimelink.com) All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
//#include "process.h"
//#include "main.h"
#include "Chip.h"
#include "Util.h"
//#include "Dbg.h"
#include "SystSettings.h"
//#include "CommPC.h"
#include "Network.h"
//#include "Tim4UART.h"
//#include "PwrManage.h"
#include "Comm2Trm.h"
#include "LoRaMac.h"


/* Compile switch-------------------------------------------------------------*/
#define EN_TEST_UPLINK    0
#if REL_VER
#undef EN_TEST_UPLINK
#define EN_TEST_UPLINK    0 /* MUST disable TEST on RELEASE mode. */
#endif

/* Private macro -------------------------------------------------------------*/
#define COMM_TRM_HEAD    0x3Cu
#define COMM_TRM_TAIL    0x0Du

/**
* @brief  Event for Comm2Trm process
*/
#define C2T_EVENT_RX_WAKE_DATA    (PROCESS_EVENT_MAX + 1)


/* Private typedef -----------------------------------------------------------*/
/**
* @brief  Status of received communication frame
*/
typedef enum
{
    STATUS_IDLE = (uint8_t)0,
    STATUS_HEAD, /* Rx Head=0x3C */
    STATUS_TYPE, /* Rx Type */
    STATUS_DATA, /* Data filed */
    STATUS_TAIL, /* Tail=0x0D */
    STATUS_END, /* End of this frame */
} COMM_TRM_STATUS_TypeDef;

/**
* @brief  Data object for received communication frame
*/
typedef struct
{
    uint8_t    byCnt; /* Count of 1 field */
    uint8_t    byDataLen; /* Length of data field */
    uint8_t    byFrameLen; /* Length of frame */
    COMM_TRM_STATUS_TypeDef    eRxStatus;
    uint8_t    a_byRxBuf[MAX_LEN_COMM_TRM_DATA];	
} COMM_TRM_DATA;


/* Private Constants ---------------------------------------------------------*/


/* Private variables ----------------------------------------------------------*/
//PROCESS_NAME(Comm2TrmProcess);

/**
* @brief  Data object for received communication frame.
* @note  Prevent race condition that accessed by both ISR and process.
*/
static COMM_TRM_DATA    s_stComm2TrmData;

/**
* @brief  MUST allocate buffer from "CommPC.c" for this before access it.
*/
static uint8_t*    s_pbyRespBuf = NULL;

/**
* @brief  Uplink data buffer accessed by Comm2Trm and Network process.
*/
static UPLINK_DATA_BUF    s_stUplinkDataBuf;

/**
* @brief  Settings of network.
*/
NetSettings_t    s_stNetSettingsC2T;

/* Private function prototypes -------------------------------------------------*/
/**
* @brief  Make response type of UART frame. 
*/
#define MAKE_UART_TYPE_RESP(byType)    (0x80u + (byType))

/**
* @brief  Lock for protected buffer of received communication frame. 
*/
static volatile bool    s_bIsLockedComm2TrmBuf = FALSE;
#define IS_LOCKED_COMM_2_TRM_BUF()    s_bIsLockedComm2TrmBuf
#define LOCK_COMM_2_TRM_BUF()    do { s_bIsLockedComm2TrmBuf = TRUE; } while (0)
#define UNLOCK_COMM_2_TRM_BUF()    do { s_bIsLockedComm2TrmBuf = FALSE; } while (0)


/**
  * @note  Block process if the UART_TX is BUSY to avoid overwrite!
  */
static void AllocRespBuf(void)
{
    s_pbyRespBuf = (uint8_t *)cpc_GetBuf();

    return;
}

/**
  * @note  MUST call "AllocRespBuf()" preceded.
  */
static uint8_t *GetRespBufDataPtr(void)
{
    return s_pbyRespBuf + sizeof(COMM_FRAME_HEAD);
}

/**
  * @note  MUST call "AllocRespBuf()" preceded.
  */
static void SetRespBufDataSize(uint8_t bySize)
{
    ((COMM_FRAME_HEAD *)s_pbyRespBuf)->byDataSize = bySize;

    return;
}

/**
  * @note  MUST call "AllocRespBuf()" preceded.
  */
static void MakeTxRespBuf(COMM_FRAME_TYPE_TypeDef eType)
{
    uint16_t    wHeadDataSize;

    ((COMM_FRAME_HEAD *)s_pbyRespBuf)->byHead = COMM_TRM_HEAD;
    ((COMM_FRAME_HEAD *)s_pbyRespBuf)->eType =    \
        (COMM_FRAME_TYPE_TypeDef)MAKE_UART_TYPE_RESP(eType);

    /* Calculate CheckSum */
    wHeadDataSize = ((COMM_FRAME_HEAD *)s_pbyRespBuf)->byDataSize;
    wHeadDataSize += sizeof(COMM_FRAME_HEAD);
    s_pbyRespBuf[wHeadDataSize] = util_CalcCS(s_pbyRespBuf, wHeadDataSize);

    s_pbyRespBuf[wHeadDataSize + 1] = COMM_TRM_TAIL; /* Add 1 for CS */

    /* Send this UART frame to terminal */
    cpc_Tx(wHeadDataSize + sizeof(COMM_FRAME_TAIL));
	
    return;
}

/*-------------------------------------------------------------------------*/
#define CUR_VER    "RNDU470LA V3.1.4 19-01-27"
static void GetID(void)
{
    AllocRespBuf();
#if CATCH_EXCEPTION
    char    *p_chBuf;
    uint8_t    byLen;

    extern uint8_t rf_GetException(char *p_chBuf);
    extern uint8_t rtc_GetException(char *p_chBuf);
    extern uint8_t pm_GetException(char *p_chBuf);

    byLen = 0;
    p_chBuf = (char *)GetRespBufDataPtr();

    p_chBuf[byLen++] = '\r';
    p_chBuf[byLen++] = '\n';
    byLen += rf_GetException(&p_chBuf[byLen]);
    byLen += rtc_GetException(&p_chBuf[byLen]);
    byLen += pm_GetException(&p_chBuf[byLen]);
    p_chBuf[byLen++] = '\r';
    p_chBuf[byLen++] = '\n';
    SetRespBufDataSize(byLen);
#else
    strcpy((char *)GetRespBufDataPtr(), CUR_VER);
    strcat((char *)GetRespBufDataPtr(), ", ID=");
    strcat((char *)GetRespBufDataPtr(), chip_GetID());
    SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1); /* Add 1 for '\0' */
#endif
    MakeTxRespBuf(TYPE_GET_VER);

    return;	
}

#define MAX_FRAME_PAYLOAD    36//51
/*-------------------------------------------------------------------------*/
static void SendRFPacket(void)
{
    uint8_t    *p_byBuf;

    if (s_stComm2TrmData.byDataLen <= MAX_FRAME_PAYLOAD) /* Valid number */
    {
        /* Save data into uplink data buffer. */
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;	
        s_stUplinkDataBuf.byDataSize = s_stComm2TrmData.byDataLen;
        s_stUplinkDataBuf.byType = 0;
        s_stUplinkDataBuf.byPort = 200;
        memcpy(s_stUplinkDataBuf.a_byDataBuf, p_byBuf, s_stComm2TrmData.byDataLen);

        /* Response the string of "TX OK" */
        AllocRespBuf();
        strcpy((char *)GetRespBufDataPtr(), "TX OK");
        SetRespBufDataSize(6);
        MakeTxRespBuf(TYPE_TX_RF_DATA);

        /* Inform Network Process to uplink this frame to gateway. */
        network_Uplink((void *)&s_stUplinkDataBuf);    
    }
    else /* Invalid number */
    {
        AllocRespBuf();
        strcpy((char *)GetRespBufDataPtr(), "TX too large,MAX is 36");
        SetRespBufDataSize(22);
        MakeTxRespBuf(TYPE_TX_RF_DATA);
    }

    return;
}

/*-------------------------------------------------------------------------*/
static void SendRFPacketOpt(void)
{
    uint8_t    *p_byBuf;
    uint8_t    byPort;
    uint8_t    byType;
    uint8_t    byOffset;
    
    byOffset = sizeof(COMM_FRAME_HEAD);
    byType = s_stComm2TrmData.a_byRxBuf[byOffset];
    byPort = s_stComm2TrmData.a_byRxBuf[byOffset+1];
    byOffset = 0;
    if((s_stComm2TrmData.byDataLen > 0) && (byType <= 1))
    {
        if((s_stComm2TrmData.byDataLen > 1) && (byPort <= 223) && (byPort >= 1))
        {
            byOffset = 2;
            if ((s_stComm2TrmData.byDataLen-byOffset) <= MAX_FRAME_PAYLOAD) /* Valid number */
            {
                /* Save data into uplink data buffer. */
                p_byBuf = sizeof(COMM_FRAME_HEAD) + byOffset + s_stComm2TrmData.a_byRxBuf;	
                s_stUplinkDataBuf.byDataSize = s_stComm2TrmData.byDataLen - byOffset;
                s_stUplinkDataBuf.byType = byType;
                s_stUplinkDataBuf.byPort = byPort;
                memcpy(s_stUplinkDataBuf.a_byDataBuf, p_byBuf, s_stUplinkDataBuf.byDataSize);

                /* Response the string of "TX OK" */
                AllocRespBuf();
                strcpy((char *)GetRespBufDataPtr(), "TX OK");
                SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1);
                MakeTxRespBuf(TYPE_TX_RF_DATA_OPT);

                /* Inform Network Process to uplink this frame to gateway. */
                network_Uplink((void *)&s_stUplinkDataBuf);    
            }
            else /* Invalid number */
            {
                AllocRespBuf();
                strcpy((char *)GetRespBufDataPtr(), "TX too large,MAX is 36");
                SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1);
                MakeTxRespBuf(TYPE_TX_RF_DATA_OPT);
            }
        }
        else
        {
            AllocRespBuf();
            strcpy((char *)GetRespBufDataPtr(), "TX rejected,Fport is 1~223");
            SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1);
            MakeTxRespBuf(TYPE_TX_RF_DATA_OPT);
        }
    }
    else
    {
        AllocRespBuf();
        strcpy((char *)GetRespBufDataPtr(), "TX rejected,MType is 0 or 1");
        SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1);
        MakeTxRespBuf(TYPE_TX_RF_DATA_OPT);
    }

    return;
}

/*-------------------------------------------------------------------------*/
static void GetLinkStatus(void)
{
    #define LINK_OK    "Network Joined"
    #define LINK_BAD "Network Not Joined"

    AllocRespBuf();
    if(true == network_Checklink())
    {
        strcpy((char *)GetRespBufDataPtr(), LINK_OK);
    }
    else
    {
        strcpy((char *)GetRespBufDataPtr(), LINK_BAD);
    }
    SetRespBufDataSize(strlen((char *)GetRespBufDataPtr()) + 1); /* Add 1 for '\0' */
    MakeTxRespBuf(TYPE_LINK_CHECK);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgClassType(void)
{
    uint8_t    *p_byBuf;

    if (1 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if ((s_stNetSettingsC2T.byClassType != p_byBuf[0])
            && ((0 == p_byBuf[0]) || (2 == p_byBuf[0])))
        {
            s_stNetSettingsC2T.byClassType = p_byBuf[0];
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    p_byBuf = GetRespBufDataPtr();
    p_byBuf[0]=s_stNetSettingsC2T.byClassType;
    SetRespBufDataSize(1);
    MakeTxRespBuf(TYPE_CFG_CLASS_TYPE);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgAppEUI(void)
{
    uint8_t    *p_byBuf;

    if (8 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if (0 != memcmp(s_stNetSettingsC2T.a_byAppEUI, p_byBuf, 8))
        {
            memcpy(s_stNetSettingsC2T.a_byAppEUI, p_byBuf, 8);
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    memcpy(GetRespBufDataPtr(), s_stNetSettingsC2T.a_byAppEUI, 8);
    SetRespBufDataSize(8);
    MakeTxRespBuf(TYPE_CFG_APP_EUI);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgAppKey(void)
{
    uint8_t    *p_byBuf;

    if (16 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if (0 != memcmp(s_stNetSettingsC2T.a_byAppKey, p_byBuf, 16))
        {
            memcpy(s_stNetSettingsC2T.a_byAppKey, p_byBuf, 16);
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    memcpy(GetRespBufDataPtr(), s_stNetSettingsC2T.a_byAppKey, 16);
    SetRespBufDataSize(16);
    MakeTxRespBuf(TYPE_CFG_APP_KEY);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgAdr(void)
{
    uint8_t    *p_byBuf;

    if (1 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if ((s_stNetSettingsC2T.byAdr != p_byBuf[0])
            && ((0 == p_byBuf[0]) || (1 == p_byBuf[0])))
        {
            s_stNetSettingsC2T.byAdr = p_byBuf[0];
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    p_byBuf = GetRespBufDataPtr();
    p_byBuf[0]=s_stNetSettingsC2T.byAdr;
    SetRespBufDataSize(1);
    MakeTxRespBuf(TYPE_CFG_ADR);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgTxPwr(void)
{
    uint8_t    *p_byBuf;

    if (1 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if ((s_stNetSettingsC2T.byTxPwr != p_byBuf[0])
            && (20 >= p_byBuf[0]))
        {
            s_stNetSettingsC2T.byTxPwr = p_byBuf[0];
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    p_byBuf = GetRespBufDataPtr();
    p_byBuf[0]=s_stNetSettingsC2T.byTxPwr;
    SetRespBufDataSize(1);
    MakeTxRespBuf(TYPE_CFG_TX_PWR);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgDataRate(void)
{
    uint8_t    *p_byBuf;

    if (1 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if ((s_stNetSettingsC2T.byDataRate != p_byBuf[0])
            && (5 >= p_byBuf[0]))
        {
            s_stNetSettingsC2T.byDataRate = p_byBuf[0];
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    p_byBuf = GetRespBufDataPtr();
    p_byBuf[0]=s_stNetSettingsC2T.byDataRate;
    SetRespBufDataSize(1);
    MakeTxRespBuf(TYPE_CFG_DR);

    return;
}

/*-------------------------------------------------------------------------*/
static void CfgStartCh(void)
{
    uint8_t    *p_byBuf;

    if (1 == s_stComm2TrmData.byDataLen)
    {
        p_byBuf = sizeof(COMM_FRAME_HEAD) + s_stComm2TrmData.a_byRxBuf;
        if ((s_stNetSettingsC2T.byStartCh != p_byBuf[0])
            && (95 >= p_byBuf[0])
            && (0 == (p_byBuf[0] % 8)))
        {
            s_stNetSettingsC2T.byStartCh = p_byBuf[0];
            ss_SaveNetSettings(&s_stNetSettingsC2T);
        }
    }

    /* Send responsed to USER */
    AllocRespBuf();
    p_byBuf = GetRespBufDataPtr();
    p_byBuf[0]=s_stNetSettingsC2T.byStartCh;
    SetRespBufDataSize(1);
    MakeTxRespBuf(TYPE_CFG_START_CH);

    return;
}

/*-------------------------------------------------------------------------*/
static int8_t ProcessUartFrame(void)
{
    uint8_t    byHeadDataSize, byCS;
    COMM_FRAME_TYPE_TypeDef    eType;	

    LOCK_COMM_2_TRM_BUF(); /* Exclude ISR of UART */

    /* Check whether the CS is valid */
    byHeadDataSize = s_stComm2TrmData.byDataLen + sizeof(COMM_FRAME_HEAD);
    byCS = util_CalcCS(&s_stComm2TrmData.a_byRxBuf[0], byHeadDataSize);
    if (byCS != s_stComm2TrmData.a_byRxBuf[byHeadDataSize])
    {
        UNLOCK_COMM_2_TRM_BUF();
    #if (!EXTI_HALT_UART)
        comm2trm_BeginRxData(); /* For RX the next UART frame. */
    #endif
        return -1; /* CS is invalid */
    }

    eType = ((COMM_FRAME_HEAD *)&s_stComm2TrmData.a_byRxBuf[0])->eType;
    switch (eType)
    {
        case TYPE_GET_VER:
            GetID();			
            break;
        case TYPE_TX_RF_DATA:
            SendRFPacket();
            break;
        case TYPE_LINK_CHECK:
            GetLinkStatus();
            break;
        case TYPE_TX_RF_DATA_OPT:
            SendRFPacketOpt();
            break;
        case TYPE_CFG_CLASS_TYPE:
            CfgClassType();
            break;
        case TYPE_CFG_APP_EUI:
            CfgAppEUI();
            break;
        case TYPE_CFG_APP_KEY:
            CfgAppKey();
            break;
        case TYPE_CFG_ADR:
            CfgAdr();
            break;
        case TYPE_CFG_TX_PWR:
            CfgTxPwr();
            break;
        case TYPE_CFG_DR:
            CfgDataRate();
            break;
        case TYPE_CFG_START_CH:
            CfgStartCh();
            break;
        default:
            ASSERT(!"Bad type of comm frame.\r\n");			
            break;    
    }

    UNLOCK_COMM_2_TRM_BUF();
#if (!EXTI_HALT_UART)
    comm2trm_BeginRxData(); /* For RX the next UART frame. */
#endif

    return 0;    	
}

/*-------------------------------------------------------------------------*/
static void SendWakeData(void* p_vWakeData)
{
    uint8_t    byCopySize;
    uint8_t    *p_byRespBuf;
    const McpsIndication_t    *p_stMcpsIndication;

    AllocRespBuf();

    p_stMcpsIndication = (const McpsIndication_t *)p_vWakeData;
    p_byRespBuf = GetRespBufDataPtr();
    byCopySize = MIN(p_stMcpsIndication->BufferSize, (MAX_LEN_UART_FRAME_DATA-3));
    memcpy(p_byRespBuf, p_stMcpsIndication->Buffer, byCopySize);
    p_byRespBuf += byCopySize;
    memcpy(p_byRespBuf, &p_stMcpsIndication->Rssi, 2);
    p_byRespBuf += 2;
    memcpy(p_byRespBuf, &p_stMcpsIndication->Snr, 1);
    byCopySize += 3;
    SetRespBufDataSize(byCopySize);

    /* the "MakeTxRespBuf()" would add 0x80. */
    MakeTxRespBuf((COMM_FRAME_TYPE_TypeDef)(TYPE_WAKE_DATA - 0x80));

    return;
}


#if 0                  //0709, 后续应该加上接收唤醒
/*---------------------------------------------------------------------------*/
PROCESS(Comm2TrmProcess, "communicate to terminal process");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(Comm2TrmProcess, ev, data)
{
    PROCESS_BEGIN();

    while (1)
    {
        PROCESS_YIELD();

        if (PROCESS_EVENT_POLL == ev)
        {
        #if EN_TEST_UPLINK 
            static uint8_t    s_bySentData = 0;
            s_stUplinkDataBuf.byDataSize = 16;
            ++s_bySentData;
            memset(s_stUplinkDataBuf.a_byDataBuf, s_bySentData, s_stUplinkDataBuf.byDataSize);
            network_Uplink((void *)&s_stUplinkDataBuf);
        #else
            ProcessUartFrame();
        #endif
        }
        else if (C2T_EVENT_RX_WAKE_DATA == ev)
        {
            SendWakeData(data);
        }
        else
        {
            ASSERT(!"Comm2TrmProcess: Bad event!\r\n");
        }
    }

    PROCESS_END();
}
#endif
/**
  * @brief  Put a data that received by UART into buffer.
  * @note  Prevent race condition this called by ISR. 
  * @param  uint8_t byData: the data received by UART.
  * @retval  None
  */
void comm2trm_RxUartData(uint8_t byData)
{
    if (IS_LOCKED_COMM_2_TRM_BUF())
    {
#if 0 //0706
        RIME_DBG2(RIME_DBG_ON, "Error: Comm to trm buffer is locked.\r\n", 0, PRINTF_FORMAT_NONE);
#endif 
        return; /* Exit that buffer is locked by Comm2TrmProcess */
    }

    /* Update status according to the received data */
    switch (s_stComm2TrmData.eRxStatus)
    {
        case STATUS_IDLE:
            if (COMM_TRM_HEAD == byData) /* Is Head */
            {
                s_stComm2TrmData.eRxStatus = STATUS_HEAD;
            #if (!EXTI_HALT_UART)
#if 0 //0706
                tim4uart_Start(MAX_TIME_UART_RX); /* Enable TIM for timeout of UART RX. */
#endif 
            #endif
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_HEAD:
            if (TYPE_INVALID_MIN < byData && byData < TYPE_INVALID_MAX) /* Valid type */
            {
                s_stComm2TrmData.eRxStatus = STATUS_TYPE;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_TYPE:
            if (byData <= MAX_LEN_UART_FRAME_DATA) /* Valid data size */
            {
                s_stComm2TrmData.eRxStatus = STATUS_DATA;
                s_stComm2TrmData.byDataLen = byData;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_DATA:
            if (s_stComm2TrmData.byCnt < s_stComm2TrmData.byDataLen)
            {
                ++s_stComm2TrmData.byCnt;
            }
            else
            {
                s_stComm2TrmData.eRxStatus = STATUS_TAIL;
            }
            break;
        case STATUS_TAIL:
            if (COMM_TRM_TAIL == byData)
            {
                tim4uart_Stop(); /* Stop the TIM that RX of UART is done. */
            #if EXTI_HALT_UART
                cpc_DisRx(); /* Disable RX of UART prevent disturbed. */
                exti4uart_End();
            #endif
 ///               process_poll(&Comm2TrmProcess); /* Tell process to deal with the received frame. */ 0706!!!!!!!!!!!!!!
            }
            else
            {
                goto rx_exception;
            }
            break;
        default:
            ASSERT(!"Error: Bad status of comm2trm_RxUartData().\r\n");
            break;
    }

    /* Save the received data */
    s_stComm2TrmData.a_byRxBuf[s_stComm2TrmData.byFrameLen++] = byData;
    return;

rx_exception:
    tim4uart_Stop(); /* Stop the TIM that RX of UART is done. */	
#if EXTI_HALT_UART
    cpc_DisRx(); /* Disable RX of UART prevent disturbed. */
    exti4uart_End(); /* Agree HALT and enable next RX. */
#else
    comm2trm_BeginRxData();
#endif

    return;	
}


/*---------------------------------------------------------------------------*/
#if EN_TEST_UPLINK
#include "SchedTimer.h"
static TimerEvent_t    s_stC2TTimer;
static void C2TTimerCallback(void)
{
    /* Inform process to sent a frame. */
    process_poll(&Comm2TrmProcess);

    /* For next sent. */
    TimerSetValue(&s_stC2TTimer, 5000); /* 5s */
    TimerStart(&s_stC2TTimer);

    return;
}
#endif


/**
  * @brief  Fetch Net-Settings from EEPROM and start Comm2Trm-Process.
  * @param  None.
  * @retval  None.
  */
void comm2trm_Init(void)
{
#if (REL_VER) /* Save ROM for Debug-Version. */
    /* Fetch settings from EEPROM. */
    ss_FetchNetSettings(&s_stNetSettingsC2T);
#endif

 //   process_start(&Comm2TrmProcess, NULL);  //0706!!!!!!!!!!!!

#if EN_TEST_UPLINK
    TimerInit(&s_stC2TTimer, C2TTimerCallback);
    TimerSetValue(&s_stC2TTimer, 1000); /* 1000ms */
    TimerStart(&s_stC2TTimer);
#endif
    uint8_t Appkey[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
                         0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    uint8_t AppEUI[8] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77};
    s_stNetSettingsC2T.byStartCh = 0x50;
    memcpy(s_stNetSettingsC2T.a_byAppEUI,AppEUI,8);
    memcpy(s_stNetSettingsC2T.a_byAppKey,Appkey,16);
    return;
}

/**
  * @brief  Begin RX a frame of UART data by clear control data structure.
  * @param  None.
  * @retval  None.
  */
void comm2trm_BeginRxData(void)
{
     /* Clear buffer for the arrival frame */	
    s_stComm2TrmData.byCnt = 0;
    s_stComm2TrmData.byDataLen = 0;
    s_stComm2TrmData.byFrameLen = 0;
    s_stComm2TrmData.eRxStatus = STATUS_IDLE;

    return;
}

/*---------------------------------------------------------------------------------------------*/
void comm2trm_RxWakeData(void *p_vWakeData)
{
    ASSERT(p_vWakeData);

//    process_post(&Comm2TrmProcess, C2T_EVENT_RX_WAKE_DATA, p_vWakeData); //0706!!!!!!!!!!!!

    return;
}

/*---------------------------------------------------------------------------------------------*/
uint8_t GetStartChannel(void)
{
    return (s_stNetSettingsC2T.byStartCh);
}

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

