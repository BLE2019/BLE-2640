#include <stdint.h>
#indlude "lora_comm.h"

/* Private variables ---------------------------------------------------------*/
/**
* @brief  Data object for received communication frame.
*/
static COMM_TRM_DATA    s_stComm2TrmData;

static uint8_t    s_abyTxUARTFrame[MAX_LEN_COMM_TRM_DATA];

static volatile bool    s_bRxUARTFrame = FALSE;

static TEMP_HUMI    s_stTempHumi;

/*-------------------------------------------------------------------------*/

uint8_t CalcCS(const void *p_vBuf, int16_t nSize)
{
    uint8_t    bySum;
    const uint8_t    *p_byBuf;

    bySum = 0;
    p_byBuf = (const uint8_t *)p_vBuf;
    
    while (nSize-- > 0)
    {
        bySum += *p_byBuf++;
    }

    return bySum;
}

void LoraCommInit(void)
{
	    /* Wait forever until the Node at work by got version from it. */
    GetLoRaNodeVersion();
    SetLoRaNodeMode();

}

/*-------------------------------------------------------------------------*/
static void ClearCommFrame(void)
{
     /* Clear frame for next frame */
    s_stComm2TrmData.byCnt = 0;
    s_stComm2TrmData.byDataLen = 0;
    s_stComm2TrmData.byFrameLen = 0;
    s_stComm2TrmData.eRxStatus = STATUS_IDLE;

    return;
}

/**
  * @brief  Put a data that received by UART into buffer.
  * @note  Prevent race condition this called by ISR. 
  * @param  uint8_t byData: the data received by UART.
  * @retval  None
  */
void RxUartData(uint8_t byData)
{
    /* Update status according to the received data */
    switch (s_stComm2TrmData.eRxStatus)
    {
        case STATUS_IDLE:
            if (COMM_TRM_HEAD == byData) /* Is Head */
            {
                s_stComm2TrmData.eRxStatus = STATUS_HEAD;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_HEAD:
            if ( (TYPE_WAKE_DATA == byData) ||
                 (MAKE_UART_TYPE_RESP(TYPE_INVALID_MIN) < byData) && 
                 (byData < MAKE_UART_TYPE_RESP(TYPE_INVALID_MAX)) ) /* Valid type */
            {
                s_stComm2TrmData.eRxStatus = STATUS_TYPE;
                if (TYPE_WAKE_DATA == byData)
                {
                 //   GPIO_SetBits(DOWNLOAD_LED_IOPORT, DOWNLOAD_LED_PIN);
                }
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
                s_bRxUARTFrame = TRUE;

                /* Tell main procedure to process WakeData. */
                const COMM_FRAME_HEAD    *p_stFrameHead;
                p_stFrameHead = (const COMM_FRAME_HEAD *)s_stComm2TrmData.a_byRxBuf;
                if (TYPE_WAKE_DATA == p_stFrameHead->eType)
                {
                    s_tSystEvent |= SYST_EVENT_GATEWAY;
                //    GPIO_ResetBits(DOWNLOAD_LED_IOPORT, DOWNLOAD_LED_PIN);
                }
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
//    GPIO_ResetBits(DOWNLOAD_LED_IOPORT, DOWNLOAD_LED_PIN);
    ClearCommFrame(); /* For RX next frame */

    return;	
}

#if 0
/**
  * @brief  Delay a few of millisecond.
  * @note  This function ONLY delay a few of millisecond since that waiting and loop 
  *            would consumes CPU.
  * @note  Be careful of overflow that capability of "uint16_t" is 65535, so the MAX count
  *            of rtimer equal=(65535 / RTIMER_ARCH_SECOND) seconds.
  * @param  wMs: count of millisecond.
  * @retval  None
  */
void DelayMs(uint16_t wMs)
{
    uint16_t    wCount;
    rtimer_clock_t    tStart;

    /* Convert millisecond to Hz of rtimer */
    wCount = (uint32_t)wMs * RTIMER_ARCH_SECOND / 1000;

    tStart = RTIMER_NOW();
    while (RTIMER_NOW() - tStart < wCount)
    {
        null();
    }

    return;
}
#endif

/*-------------------------------------------------------------------------*/
static int8_t SendUARTFrame(const void *p_vData, uint8_t bySize, COMM_FRAME_TYPE_TypeDef eType)
{
    COMM_FRAME_HEAD    *p_stHead;
    COMM_FRAME_TAIL    *p_stTail;

    if (MAX_LEN_UART_FRAME_DATA < bySize)
    {
        return -1; /* Bad size of data. */
    }

    /* Make head of frame. */
    p_stHead = (COMM_FRAME_HEAD *)&s_abyTxUARTFrame[0];
    p_stHead->byHead = COMM_TRM_HEAD;
    p_stHead->eType = eType;
    p_stHead->byDataSize = bySize;

    /* Copy payload into body of frame. */
    if (p_vData && (0 < bySize))
    {
        memcpy(&s_abyTxUARTFrame[sizeof(COMM_FRAME_HEAD)], p_vData, bySize);
    }

    /* Make tail of frame. */
    p_stTail = (COMM_FRAME_TAIL *)&s_abyTxUARTFrame[sizeof(COMM_FRAME_HEAD) + bySize];
    p_stTail->byCS = CalcCS(s_abyTxUARTFrame, bySize + sizeof(COMM_FRAME_HEAD));
    p_stTail->byTail = COMM_TRM_TAIL;

    /* Send this UART frame to RNDU470T */
    cpc_Tx( s_abyTxUARTFrame, 
                 bySize + sizeof(COMM_FRAME_HEAD) + sizeof(COMM_FRAME_TAIL));
    
    return 0;
}

#if 0
/*-------------------------------------------------------------------------*/
static void InitRTCEnableWakeUpInt(void)
{
    /* Configure RTC */
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);

    RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);

    RTC_SetWakeUpCounter(5); /* 5s */
    RTC_WakeUpCmd(ENABLE);

    /* Save more energy when enter Halt mode. */
    PWR_UltraLowPowerCmd(ENABLE);

    return;
}
#endif

/*-------------------------------------------------------------------------*/
static bool SendHumiTemp(COMM_FRAME_TYPE_TypeDef tFrameType)
{
    rtimer_clock_t    tTimeEnd = 0;

//    GPIO_SetBits(UPLINK_LED_IOPORT, UPLINK_LED_PIN);

    /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
    s_bRxUARTFrame = FALSE;
    
    /* Send the hum_temp to RNDU470T by UART port. */
    SendUARTFrame(&s_stTempHumi, sizeof(s_stTempHumi), tFrameType);

    /* Wait the responsed frame from RNDU470T. */
    tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
    while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
    {
        null();
    }

    DelayMs(100); /* for LED on more long */
    GPIO_ResetBits(UPLINK_LED_IOPORT, UPLINK_LED_PIN);

    if (s_bRxUARTFrame)
    {
        /* For RX next frame */
        ClearCommFrame();
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
/*-------------------------------------------------------------------------*/
/**
* @brief  Configure different parameters to each mode of LoRaWAN Node. 
*/
/*-------------------------------------------------------------------------*/
static void SetLoRaNodeMode(void)
{
    if(cpc_GetNodeMode() == TRUE)/*TDMA NODE*/
        return;

    /*LoRaWAN Node, Config different parameters to each mode*/
    if(key_CheckStatus())
    {
        /*distance test mode,ADR=OFF, SF12, TXPWR=20*/
        SetADR(FALSE);
        SetDataRate(SF12);  
        SendTxPwr(20);
        DiatanceTestModeLed();
    }
    else
    {
        /*default test mode,ADR=ON, SF12, TXPWR=17*/
        SetADR(TRUE);
        SetDataRate(SF12);  
        SendTxPwr(17);
        DefaultModeLed();
    }
    /*reboot node*/
    GPIO_WriteBit(REBOOT_PORT,REBOOT_PIN,RESET);
    DelayMs(10);
    GPIO_WriteBit(REBOOT_PORT,REBOOT_PIN,SET);
    return;
}


/*-------------------------------------------------------------------------*/
static bool AtomicTestClearBit(Atomic_t *p_tAtomic, uint8_t byEventBit)
{
    bool    bIsSet;
    halIntState_t    intState;

    ASSERT(p_tAtomic);

    HAL_ENTER_CRITICAL_SECTION(intState);
    if (*p_tAtomic & byEventBit)
    {
        *p_tAtomic &= ~byEventBit;
        bIsSet = TRUE;
    }
    else
    {
        bIsSet = FALSE;
    }
    HAL_EXIT_CRITICAL_SECTION(intState);

    return bIsSet;
}

/*-------------------------------------------------------------------------*/
static void AtomicSetBit(Atomic_t *p_tAtomic, uint8_t byEventBit)
{
    halIntState_t    intState;

    HAL_ENTER_CRITICAL_SECTION(intState);
    *p_tAtomic |= byEventBit;
    HAL_EXIT_CRITICAL_SECTION(intState);

    return;
}
static void SendTxPwr(int8_t pwr)
{
    int8_t pwrval;
    pwrval = pwr;
    rtimer_clock_t    tTimeEnd = 0;
    do{
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        /* Send the hum_temp to RNDU470T by UART port. */
        SendUARTFrame(&pwrval, sizeof(pwrval), TYPE_TX_PWR);

        /* Wait the responsed frame from RNDU470T. */
        tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }

        if (s_bRxUARTFrame)
        {
            /* For RX next frame */
            ClearCommFrame();
            return;
        }
    }while(TRUE);
}


/*-------------------------------------------------------------------------*/
static void SetADR(bool onoff)
{
    int8_t onoffstate;
    onoffstate = onoff;
    rtimer_clock_t    tTimeEnd = 0;

    do{
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        /* Send the hum_temp to RNDU470T by UART port. */
        SendUARTFrame(&onoffstate, sizeof(onoffstate), TYPE_ADR);

        /* Wait the responsed frame from RNDU470T. */
        tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }
        
        if (s_bRxUARTFrame)
        {
            /* For RX next frame */
            ClearCommFrame();
            return;
        }
    }while(TRUE);
}

static void SetDataRate(Datarate_e sf)
{
    int8_t datarate;
    datarate = sf;
    rtimer_clock_t    tTimeEnd = 0;

    do{
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        /* Send the hum_temp to RNDU470T by UART port. */
        SendUARTFrame(&datarate, sizeof(datarate), TYPE_DATARATE);

        /* Wait the responsed frame from RNDU470T. */
        tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }
        
        if (s_bRxUARTFrame)
        {
            /* For RX next frame */
            ClearCommFrame();
            return;
        }
    }while(TRUE);
}

/*-------------------------------------------------------------------------*/
static void GetLoRaNodeVersion(void)
{
    const char    *p_chRespStr;
    uint8_t     byRespSize;
    rtimer_clock_t    tTimeEnd;

    static int8_t    s_chTryCnt = 0;

    do {    
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        SendUARTFrame(NULL, 0, TYPE_GET_VER);
        ++s_chTryCnt;

        tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }

        if (s_bRxUARTFrame)
        {
            p_chRespStr = (const char *)&s_stComm2TrmData.a_byRxBuf[sizeof(COMM_FRAME_HEAD)];
            byRespSize = strlen(p_chRespStr) + 1; /* Add 1 for '\0' */
            dp_Tx(p_chRespStr, byRespSize);
            dp_Tx("\r\n", 2);

            /* Now, we can judge the node is TDMA or LoRaWAN! */
            if ('T' == p_chRespStr[7]) /* TDMA */
            {
                cpc_SetNodeMode(TRUE);
            }
            else /* LoRaWAN */
            {
                cpc_SetNodeMode(FALSE);
            }

            /* For RX next frame */
            ClearCommFrame();
            s_chTryCnt = 0;

            return;
        }
        else
        {
            if (10 <= s_chTryCnt)
            {
                /* The node is TDMA or LoRaWAN? Toggle mode and try it again! */
                cpc_ToggleNodeMode();
                s_chTryCnt = 0;
            }
        }
    } while (TRUE);
}

/*-------------------------------------------------------------------------*/
#if EN_TEST_NODE_UART
static void GetNetSettings(void)
{
    int8_t    chCnt;
    rtimer_clock_t    tTimeEnd;
    static int8_t    s_chMaxTry = 0;
    static int32_t    s_lTxCnt = 0;
    static int32_t    s_lRxCnt = 0;
    static int32_t    s_lFailedCnt = 0;
    static char    s_achPrintBuf[64];

#if 0 /* Change RF speed of NODE. */
    static const uint8_t    s_abySetBps7[] = 
    {0x3C, 0x03, 0x09, 0x55, 0xAA, 0x07, 0x1C, 0x03, 0xA1, 0x80, 0x6F, 0x3C, 0x39, 0x0D};

    while (TRUE)
    {
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        /* Send request command. */
        cpc_Tx(s_abySetBps7, sizeof(s_abySetBps7));

        /* Waiting responsed of net settings. */
        tTimeEnd = RTIMER_NOW() + 50; /* wait 50ms */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }

        /* Print count of request and response. */
        if (s_bRxUARTFrame)
        {
            break; /* Break from "while(TRUE)" */
        }
    }
#endif

#if 1 /* Test JiKang debug. */
    static const uint8_t   s_abyTestData[] =
    {0x00, 0x00, 0x00, 0x12, 0x7F, 0x45, 0x36, 0x1B, 0xE8, 0x45, 0x5D, 0x44, 0xC2, 0x40, 0x39};

    while (1)
    {
        /* Waiting until received a wake frame from SINK. */
        s_bRxUARTFrame = FALSE;
        while (!s_bRxUARTFrame)
        {
            nop();
        }
        ClearCommFrame(); /* For RX next frame */

        /* Delay 3000ms */
        DelayMs(3000);

        for (chCnt = 0; chCnt < 10; ++chCnt)
        {
            /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
            s_bRxUARTFrame = FALSE;

            /* Send a test frame to NODE. */
            SendUARTFrame(s_abyTestData, sizeof(s_abyTestData), TYPE_TX_RF_DATA);
            ++s_lTxCnt;

            /* Waiting responsed of net settings. */
            tTimeEnd = RTIMER_NOW() + 50; /* wait 50ms */
            while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
            {
                null();
            }

            /* Check whether received "TX OK" from NODE. */
            if (s_bRxUARTFrame)
            {
                ++s_lRxCnt;
                ClearCommFrame(); /* For RX next frame */
                break; /* Break from "for()" */
            }
            else
            {
                ++s_lFailedCnt;
            }
        }

        if (s_chMaxTry < chCnt)
        {
            s_chMaxTry = chCnt;
        }

        if (0 == s_lTxCnt % 1)
        {
            snprintf( s_achPrintBuf, 
                          sizeof(s_achPrintBuf), 
                          "Tx=%ld, Rx=%ld, Failed=%ld, MaxTry=%d\r\n", 
                          s_lTxCnt, s_lRxCnt, s_lFailedCnt, s_chMaxTry );
            dp_Tx(s_achPrintBuf, strlen(s_achPrintBuf));
        }
    }
#endif

}
#endif

/**
* @brief  Configure different parameters to each mode of LoRaWAN Node. 
*/
/*-------------------------------------------------------------------------*/
static void SetLoRaNodeMode(void)
{
    if(cpc_GetNodeMode() == TRUE)/*TDMA NODE*/
        return;

    /*LoRaWAN Node, Config different parameters to each mode*/
    if(key_CheckStatus())
    {
        /*distance test mode,ADR=OFF, SF12, TXPWR=20*/
        SetADR(FALSE);
        SetDataRate(SF12);  
        SendTxPwr(20);
        DiatanceTestModeLed();
    }
    else
    {
        /*default test mode,ADR=ON, SF12, TXPWR=17*/
        SetADR(TRUE);
        SetDataRate(SF12);  
        SendTxPwr(17);
        DefaultModeLed();
    }
    /*reboot node*/
    GPIO_WriteBit(REBOOT_PORT,REBOOT_PIN,RESET);
    DelayMs(10);
    GPIO_WriteBit(REBOOT_PORT,REBOOT_PIN,SET);
    return;
}


/*-------------------------------------------------------------------------*/
static bool AtomicTestClearBit(Atomic_t *p_tAtomic, uint8_t byEventBit)
{
    bool    bIsSet;
    halIntState_t    intState;

    ASSERT(p_tAtomic);

    HAL_ENTER_CRITICAL_SECTION(intState);
    if (*p_tAtomic & byEventBit)
    {
        *p_tAtomic &= ~byEventBit;
        bIsSet = TRUE;
    }
    else
    {
        bIsSet = FALSE;
    }
    HAL_EXIT_CRITICAL_SECTION(intState);

    return bIsSet;
}

/*-------------------------------------------------------------------------*/
static void AtomicSetBit(Atomic_t *p_tAtomic, uint8_t byEventBit)
{
    halIntState_t    intState;

    HAL_ENTER_CRITICAL_SECTION(intState);
    *p_tAtomic |= byEventBit;
    HAL_EXIT_CRITICAL_SECTION(intState);

    return;
}

