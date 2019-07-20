/**
 * \file
 *         Communicate to terminal like as PC or external device.
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-05-25 12:17
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMM_2_TRM_H__
#define __COMM_2_TRM_H__


/* Includes ------------------------------------------------------------------*/
//#include "Main.h"
#include "SystSettings.h"

/* Private macros ------------------------------------------------------------*/
#define MAX_LEN_COMM_TRM_DATA    255u
#define MAX_LEN_UART_FRAME_DATA    \
    (MAX_LEN_COMM_TRM_DATA - sizeof(COMM_FRAME_HEAD) - sizeof(COMM_FRAME_TAIL))

/* Exported types ------------------------------------------------------------*/
/**
* @brief  Type of received communication frame
*/
typedef enum
{
    TYPE_INVALID_MIN = (uint8_t)0,
    TYPE_GET_VER, /*!< User System get version of this node. */
    TYPE_TX_RF_DATA, /*!< User System send data that need to TX by RF. */
    TYPE_LINK_CHECK, /*!< User System check link status of this node. */
    TYPE_TX_RF_DATA_OPT,/*!< User System need to TX data by RF with options. */
    TYPE_CFG_CLASS_TYPE,/*!< User System set or get class type of this node. */
    TYPE_CFG_APP_EUI,/*!< User System set or get APPEUI of this node. */
    TYPE_CFG_APP_KEY,/*!< User System set or get APPKEY of this node. */
    TYPE_CFG_ADR,/*!< User System set or get ADR status of this node. */
    TYPE_CFG_TX_PWR,/*!< User System set or get tx power of this node. */
    TYPE_CFG_DR,/*!< User System set or get data rate of this node. */
    TYPE_CFG_START_CH,/*!< User System set or get start tx channel of this node. */

    TYPE_INVALID_MAX,

    TYPE_WAKE_DATA = ((uint8_t)0xC0), /*!< Node send wake data to User System. */
} COMM_FRAME_TYPE_TypeDef;


/* Exported variables ------------------------------------------------------- */
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

/**
* @brief  Uplink data buffer accessed by Comm2Trm and Network process.
*/
typedef struct
{
    uint8_t    byDataSize;
    uint8_t    byPort;
    uint8_t    byType;
    uint8_t    a_byDataBuf[MAX_LEN_UART_FRAME_DATA];
} UPLINK_DATA_BUF;


/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void comm2trm_Init(void);
void comm2trm_RxUartData(uint8_t byData);
void comm2trm_BeginRxData(void);
void comm2trm_RxWakeData(void *p_vWakeData);


#endif /*#ifndef __COMM_2_TRM_H__*/

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

