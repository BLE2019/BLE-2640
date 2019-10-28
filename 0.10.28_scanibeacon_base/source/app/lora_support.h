#ifndef _LORA_SUPPORT_H_
#define _LORA_SUPPORT_H_
#include <stdio.h>
#include <string.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include "util.h"
#include "LAUNCHIOT_CC2650_RGZ.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* Delay */
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))


#define LORA_RESET_PIN          IOID_25
#define LORA_CON1_PIN           IOID_24
#define LORA_PIN_HIGH           (1)
#define LORA_PIN_LOW            (0)

typedef enum
{
    TYPE_INVALID_MIN            = (uint8_t)0,
    TYPE_GET_VER                = 1, /*!< User System get version of this node. */
    TYPE_TX_RF_DATA             = 2, /*!< User System send data that need to TX by RF. */
    TYPE_GET_JOINEDSTATE        = 3, /*!< User System get Network Joined State. */
    TYPE_TX_RF_DATA_BYTYPE      = 4,
    TYPE_CLASS                  = 5,
    TYPE_APPEUI                 = 6,
    TYPE_APPKEY                 = 7,
    TYPE_ADR                    = 8,
    TYPE_TX_PWR                 = 9,
    TYPE_DATARATE               = 0x0a,
    TYPE_CHANNEL                = 0x0b,
    TYPE_INVALID_MAX            = 0x0C,
    TYPE_WAKE_DATA              = ((uint8_t)0xC0), /*!< Node send wake data to User System. */
} COMM_FRAME_TYPE_TypeDef;

typedef enum
{
  SF12=0,
  SF11,
  SF10,
  SF9,
  SF8,
  SF7,
} Datarate_e;


typedef struct
{
    uint8_t    byHead;
    COMM_FRAME_TYPE_TypeDef    eType;
    uint8_t    byDataSize;
} COMM_FRAME_HEAD;

typedef struct
{
    uint8_t    byCS;
    uint8_t    byTail;
} COMM_FRAME_TAIL;

#define COMM_TRM_HEAD    0x3Cu
#define COMM_TRM_TAIL    0x0Du

extern void LoraPinInit(void);
extern int8_t SendUARTFrame(const void *p_vData, uint8_t bySize, COMM_FRAME_TYPE_TypeDef eType);
extern void SetLoRaNodeMode(void);
extern void LoraResetPinHigh(void);
extern void LoraResetPinLow(void);

#endif
