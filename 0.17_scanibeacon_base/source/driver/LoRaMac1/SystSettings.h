/**
 * \file
 *         SystSettings.h
 * \description
 *         Head file of System Settings
 * \author
 *         Jiang Jun <jiangjunjie_2005@126.com>
 * \date
 *         2015-11-13 12:09
 * \copyright
 *         (c) RimeLink (www.rimelink.com)
 */
 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYST_SETTINGS_H__
#define __SYST_SETTINGS_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
//#include "RTC.h" //0709


/* Exported variables ------------------------------------------------------- */

/* Exported macros --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
* @brief  Network settings
*/
typedef struct
{
    uint8_t    a_byAppEUI[8];
    uint8_t    a_byAppKey[16];
    uint8_t    byClassType;
    uint8_t    byAdr;
    uint8_t    byTxPwr;
    uint8_t    byDataRate;
    uint8_t    byStartCh;
} NetSettings_t;


/* Exported constants --------------------------------------------------------*/


/* Imported macros -----------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
int8_t ss_Init(void);
int16_t ss_FetchNetSettings(NetSettings_t *p_stSettings);
int8_t ss_SaveNetSettings(const NetSettings_t *p_stSettings);


#endif

/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

