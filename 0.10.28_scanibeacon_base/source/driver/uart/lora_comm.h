#ifndef __LORA_COMM_H__
#define __LORA_COMM_H__
/**
* @brief  Status of received communication frame
*/
typedef enum
{
    STATUS_IDLE = (uint8_t)0,
    STATUS_HEAD, /* Rx Head=0x3C */
    STATUS_TYPE, /* Rx Type */
    STATUS_DATA, /* Data field */
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

/**
* @brief  Type of received communication frame
*/
typedef enum
{
    TYPE_INVALID_MIN = (uint8_t)0,
    TYPE_GET_VER=1, /*!< User System get version of this node. */
    TYPE_TX_RF_DATA=2, /*!< User System send data that need to TX by RF. */
    TYPE_GET_JOINEDSTATE=3, /*!< User System get Network Joined State. */
    TYPE_TX_RF_DATA_BYTYPE=4, 
    TYPE_CLASS=5,
    TYPE_APPEUI=6,
    TYPE_APPKEY=7,
    TYPE_ADR=8,
    TYPE_TX_PWR=9, 
    TYPE_DATARATE=0x0a,
    TYPE_CHANNEL=0x0b,
    TYPE_INVALID_MAX=0x0C,
    TYPE_WAKE_DATA = ((uint8_t)0xC0), /*!< Node send wake data to User System. */
} COMM_FRAME_TYPE_TypeDef;

/**
* @brief  Type of Datarate
*/
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

typedef struct _temp_humi
{
    float    fTemp; /* temperature */
    float    fHumi; /* humidity */
    float    fDewPoint; /* dew point */
    int16_t  rssi;  /*RSSI*/
    int8_t   snr;   /*SNR*/
} TEMP_HUMI;


/* Private macro -------------------------------------------------------------*/
#define MAX_LEN_COMM_TRM_DATA    255u
#define MAX_LEN_UART_FRAME_DATA    \
    (MAX_LEN_COMM_TRM_DATA - sizeof(COMM_FRAME_HEAD) - sizeof(COMM_FRAME_TAIL))

#define COMM_TRM_HEAD    0x3Cu
#define COMM_TRM_TAIL    0x0Du

#define MAKE_UART_TYPE_RESP(byType)    (0x80u + (byType))

typedef enum
{
    TYPE_INVALID_MIN = (uint8_t)0,
    TYPE_GET_VER=1, /*!< User System get version of this node. */
    TYPE_TX_RF_DATA=2, /*!< User System send data that need to TX by RF. */
    TYPE_GET_JOINEDSTATE=3, /*!< User System get Network Joined State. */
    TYPE_TX_RF_DATA_BYTYPE=4, 
    TYPE_CLASS=5,
    TYPE_APPEUI=6,
    TYPE_APPKEY=7,
    TYPE_ADR=8,
    TYPE_TX_PWR=9, 
    TYPE_DATARATE=0x0a,
    TYPE_CHANNEL=0x0b,
    TYPE_INVALID_MAX=0x0C,
    TYPE_WAKE_DATA = ((uint8_t)0xC0), /*!< Node send wake data to User System. */
    TYPE_SOS = 0x10,
    TYPE_LOW_PWR = 0x11,
    TYPE_SCAN_BEACON = 0x12,
} COMM_FRAME_TYPE_TypeDef;



typedef struct
{
	uint8_t sos_flag;
	
}SOS_MSG;
typedef struct
{
	uint8_t pwr_ratio;
	
}PWR_DETECT_MSG;

#endif

