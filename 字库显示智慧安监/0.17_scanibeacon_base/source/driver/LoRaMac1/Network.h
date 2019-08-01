/**
 * \file
 *         Network.h
 * \description
 *         Operation of communicated to Sink
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-17 10:02
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETWORK_H__
#define __NETWORK_H__

/* Includes ------------------------------------------------------------------*/
#include "SystSettings.h"

/* Exported variables ------------------------------------------------------- */


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Imported macros -----------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void network_Init(void);
void network_Uplink(void *p_vUpData);
//bool network_Checklink(void); //0709

void LoRaMac_init_and_register(void);
void Trace_LoRaMac(char*);
#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

