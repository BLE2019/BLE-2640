#include "lora_support.h"
#include "task_uart.h"
#define  MAX_UART_BUFF_SIZE  255
static uint8_t    s_abyTxUARTFrame[MAX_UART_BUFF_SIZE];

PIN_Handle g_LoraWakeUpHandle;
PIN_State  g_LoraWakeUpState;

const PIN_Config g_LoraWakeUpPINCfg[] =
{
  LORA_CON1_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  LORA_RESET_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

  PIN_TERMINATE
};

void LoraPinInit(void)
{
    g_LoraWakeUpHandle = PIN_open(&g_LoraWakeUpState, g_LoraWakeUpPINCfg);
}

void LoraResetPinHigh(void)
{
    PIN_setOutputValue(g_LoraWakeUpHandle, LORA_RESET_PIN, LORA_PIN_HIGH);
}

void LoraResetPinLow(void)
{
    PIN_setOutputValue(g_LoraWakeUpHandle, LORA_RESET_PIN, LORA_PIN_LOW);
}

void LoraWakeUpPinHigh(void)
{
    PIN_setOutputValue(g_LoraWakeUpHandle, LORA_CON1_PIN, LORA_PIN_HIGH);
}

void LoraWakeUpPinLow(void)
{
    PIN_setOutputValue(g_LoraWakeUpHandle, LORA_CON1_PIN, LORA_PIN_LOW);
}

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

static void WakeUpLoraModule(void)
{
    LoraWakeUpPinHigh();
    DELAY_US(1000);
}

static void SetADR(bool state)
{
    int8_t onoffstate;
    onoffstate = state;

    /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
    //s_bRxUARTFrame = FALSE;

    /* Send the hum_temp to RNDU470T by UART port. */
    SendUARTFrame(&onoffstate, sizeof(onoffstate), TYPE_ADR);

    /* Wait the responsed frame from RNDU470T. */
    DELAY_MS(1);
}

static void SetDataRate(Datarate_e sf)
{
    int8_t datarate;
    datarate = sf;

    /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */

    /* Send the hum_temp to RNDU470T by UART port. */
    SendUARTFrame(&datarate, sizeof(datarate), TYPE_DATARATE);

    /* Wait the responsed frame from RNDU470T. */
    DELAY_MS(1);
}

static void SendTxPwr(int8_t pwr)
{
    int8_t pwrval;
    pwrval = pwr;
    /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */

    /* Send the hum_temp to RNDU470T by UART port. */
    SendUARTFrame(&pwrval, sizeof(pwrval), TYPE_TX_PWR);
    /* Wait the responsed frame from RNDU470T. */
    DELAY_MS(1);
}

void SetLoRaNodeMode(void)
{
    /* default test mode */
    SetADR(TRUE);
    SetDataRate(SF12);
    SendTxPwr(17);
}

int8_t SendUARTFrame(const void *p_vData, uint8_t bySize, COMM_FRAME_TYPE_TypeDef eType)
{
    COMM_FRAME_HEAD    *p_stHead;
    COMM_FRAME_TAIL    *p_stTail;

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

    /* Before Send data WakeUp Lora first */
    WakeUpLoraModule();

    /* Send this UART frame to RNDU470T */
    TaskUARTWrite(s_abyTxUARTFrame,
        bySize + sizeof(COMM_FRAME_HEAD) + sizeof(COMM_FRAME_TAIL));

    LoraWakeUpPinLow();
    return 0;
}






