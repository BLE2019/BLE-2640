/**
 * \file
 *         RFAdapter.h
 * \description
 *         Head file for adapter for sx1278_src.c and LoRaMac.c
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-04-01 11:04
 * \copyright
 *         (c) RimeLink (www.rimelink.com) All Rights Reserved.
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RF_ADAPTER_H__
#define __RF_ADAPTER_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Exported variables ------------------------------------------------------- */


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macros -----------------------------------------------------------*/
/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_OSC_STARTUP                           1 // [ms]

/*!
 * Radio PLL lock and Mode Ready delay which can vary with the temperature
 */
#define RADIO_SLEEP_TO_RX                           2 // [ms]

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME                           ( RADIO_OSC_STARTUP + RADIO_SLEEP_TO_RX )

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34


/* Private macros ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

