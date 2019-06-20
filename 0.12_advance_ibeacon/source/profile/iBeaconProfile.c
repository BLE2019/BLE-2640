/**************************************************************************************************
  Filename:       iBeaconProfile.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    This file contains the Simple GATT profile sample GATT service 
                  profile for use with the BLE sample application.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "string.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "iBeaconProfile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        34

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 iBeaconProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_SERV_UUID), HI_UINT16(iBeaconProfile_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 iBeaconProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR1_UUID), HI_UINT16(iBeaconProfile_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 iBeaconProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR2_UUID), HI_UINT16(iBeaconProfile_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 iBeaconProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR3_UUID), HI_UINT16(iBeaconProfile_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 iBeaconProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR4_UUID), HI_UINT16(iBeaconProfile_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 iBeaconProfilechar5UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR5_UUID), HI_UINT16(iBeaconProfile_CHAR5_UUID)
};

// Characteristic 6 UUID: 0xFFF6
CONST uint8 iBeaconProfilechar6UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR6_UUID), HI_UINT16(iBeaconProfile_CHAR6_UUID)
};

// Characteristic 7 UUID: 0xFFF7
CONST uint8 iBeaconProfilechar7UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR7_UUID), HI_UINT16(iBeaconProfile_CHAR7_UUID)
};

// Characteristic 8 UUID: 0xFFF8
CONST uint8 iBeaconProfilechar8UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR8_UUID), HI_UINT16(iBeaconProfile_CHAR8_UUID)
};

// Characteristic 9 UUID: 0xFFF9
CONST uint8 iBeaconProfilechar9UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR9_UUID), HI_UINT16(iBeaconProfile_CHAR9_UUID)
};
// Characteristic 10 UUID: 0xFFFA
CONST uint8 iBeaconProfilechar10UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR10_UUID), HI_UINT16(iBeaconProfile_CHAR10_UUID)
};
// Characteristic 11 UUID: 0xFFFB
CONST uint8 iBeaconProfilechar11UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(iBeaconProfile_CHAR11_UUID), HI_UINT16(iBeaconProfile_CHAR11_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static iBeaconProfileCBs_t *iBeaconProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t iBeaconProfileService = { ATT_BT_UUID_SIZE, iBeaconProfileServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 iBeaconProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 iBeaconProfileChar1 = 0;

// Simple Profile Characteristic 1 User Description
static uint8 iBeaconProfileChar1UserDesp[15] = "Measured Power\0";


// Simple Profile Characteristic 2 Properties
static uint8 iBeaconProfileChar2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 2 Value
static uint8 iBeaconProfileChar2[iBeaconProfile_CHAR2_LEN] = { 0x00, 0x01};

// Simple Profile Characteristic 2 User Description
static uint8 iBeaconProfileChar2UserDesp[12] = "Major Value\0";


// Simple Profile Characteristic 3 Properties
static uint8 iBeaconProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 iBeaconProfileChar3[iBeaconProfile_CHAR3_LEN] = { 0x00, 0x01};

// Simple Profile Characteristic 3 User Description
static uint8 iBeaconProfileChar3UserDesp[12] = "Minor Value\0";


// Simple Profile Characteristic 4 Properties
static uint8 iBeaconProfileChar4Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 4 Value
static uint8 iBeaconProfileChar4[iBeaconProfile_CHAR4_LEN] = {0};
                                    
// Simple Profile Characteristic 4 User Description
static uint8 iBeaconProfileChar4UserDesp[14] = "ProximityUUID\0";

// Simple Profile Characteristic 5 Properties
static uint8 iBeaconProfileChar5Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 5 Value
static uint8 iBeaconProfileChar5 = 0;

// Simple Profile Characteristic 5 User Description
static uint8 iBeaconProfileChar5UserDesp[13] = "Adv Interval\0";


// Simple Profile Characteristic 6 Properties
static uint8 iBeaconProfileChar6Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 6 Value
static uint8 iBeaconProfileChar6[iBeaconProfile_CHAR6_LEN] = {0};

// Simple Profile Characteristic 6 User Description
static uint8 iBeaconProfileChar6UserDesp[8] = "Command\0";

// Simple Profile Characteristic 7 Properties
static uint8 iBeaconProfileChar7Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 7 Value
static uint8 iBeaconProfileChar7[iBeaconProfile_CHAR7_LEN] = { 0, 0, 0, 0, 0, 0};

// Simple Profile Characteristic 7 User Description
static uint8 iBeaconProfileChar7UserDesp[9] = "Password\0";


// Simple Profile Characteristic 8 Properties
static uint8 iBeaconProfileChar8Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 8 Value
static uint8 iBeaconProfileChar8[iBeaconProfile_CHAR8_LEN] = { 0, 0, 0, 0};

// Simple Profile Characteristic 8 User Description
static uint8 iBeaconProfileChar8UserDesp[11] = "DeviceName\0";


// Simple Profile Characteristic 9 Properties
static uint8 iBeaconProfileChar9Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 9 Value
static uint8 iBeaconProfileChar9 = 0;

// Simple Profile Characteristic 9 User Description
static uint8 iBeaconProfileChar9UserDesp[8] = "TxPower\0";


// Simple Profile Characteristic 10 Properties
static uint8 iBeaconProfileChar10Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 10 Value
static uint8 iBeaconProfileChar10[iBeaconProfile_CHAR10_LEN] = { 0};

// Simple Profile Characteristic 10 User Description
static uint8 iBeaconProfileChar10UserDesp[15] = "PasswordChecks\0";

// Simple Profile Characteristic 11 Properties
static uint8 iBeaconProfileChar11Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 11 Value
static uint8 iBeaconProfileChar11[iBeaconProfile_CHAR11_LEN] = {0x00};

// Simple Profile Characteristic 11 User Description
static uint8 iBeaconProfileChar11UserDesp[8] = "reserve\0";




/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t iBeaconProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&iBeaconProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &iBeaconProfileChar1 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar1UserDesp 
      },      

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar2Props 
    },

      // Characteristic Value 2
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconProfileChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar2UserDesp 
      },           
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        iBeaconProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar3UserDesp 
      },

    // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar4Props 
    },

      // Characteristic Value 4
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar4UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        iBeaconProfileChar4 
      },

      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar4UserDesp 
      },
      
    // Characteristic 5 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar5Props 
    },

      // Characteristic Value 5
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar5UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &iBeaconProfileChar5 
      },

      // Characteristic 5 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar5UserDesp 
      },

    // Characteristic 6 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar6Props 
    },

      // Characteristic Value 6
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar6UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        iBeaconProfileChar6 
      },

      // Characteristic 6 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar6UserDesp 
      },


    // Characteristic 7 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar7Props 
    },

      // Characteristic Value 7
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar7UUID },
        GATT_PERMIT_WRITE,
        0, 
        iBeaconProfileChar7 
      },

      // Characteristic 7 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar7UserDesp 
      },

    // Characteristic 8 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar8Props 
    },

      // Characteristic Value 8
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar8UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        iBeaconProfileChar8 
      },

      // Characteristic 8 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar8UserDesp 
      },

    // Characteristic 9 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar9Props 
    },

      // Characteristic Value 9
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar9UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &iBeaconProfileChar9 
      },

      // Characteristic 9 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar9UserDesp 
      },

    // Characteristic 10 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar10Props 
    },

      // Characteristic Value 10
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar10UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        iBeaconProfileChar10 
      },

      // Characteristic 10 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar10UserDesp 
      },
      
    // Characteristic 11 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &iBeaconProfileChar11Props 
    },
    
      // Characteristic Value 11
      { 
        { ATT_BT_UUID_SIZE, iBeaconProfilechar11UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        iBeaconProfileChar11 
      },
    
      // Characteristic 11 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        iBeaconProfileChar11UserDesp 
      },

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t iBeaconProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
static bStatus_t iBeaconProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t iBeaconProfileCBs =
{
  iBeaconProfile_ReadAttrCB,  // Read callback function pointer
  iBeaconProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      iBeaconProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t iBeaconProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  if ( services & iBeaconProfile_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( iBeaconProfileAttrTbl, 
                                          GATT_NUM_ATTRS( iBeaconProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &iBeaconProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      iBeaconProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t iBeaconProfile_RegisterAppCBs( iBeaconProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    iBeaconProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  

/*********************************************************************
 * @fn      iBeaconProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t iBeaconProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case iBeaconProfile_CHAR1:
      if ( len == sizeof ( uint8 ) ) 
      {
        iBeaconProfileChar1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR2:
      if ( len == iBeaconProfile_CHAR2_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar2, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR3:
      if ( len == iBeaconProfile_CHAR3_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar3, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR4:
      if ( len == iBeaconProfile_CHAR4_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar4, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
	  
    case iBeaconProfile_CHAR5:
      if ( len == sizeof ( uint8 ) ) 
      {
        iBeaconProfileChar5 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR6:
      if ( len == sizeof ( uint8 ) ) 
      {
        VOID memcpy( iBeaconProfileChar6, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
	  
    case iBeaconProfile_CHAR7:
      if ( len == iBeaconProfile_CHAR7_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar7, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR8:
      if ( len == iBeaconProfile_CHAR8_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar8, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR9:
      if ( len == iBeaconProfile_CHAR9_LEN ) 
      {
        iBeaconProfileChar9 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR10:
      if ( len == iBeaconProfile_CHAR10_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar10, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case iBeaconProfile_CHAR11:
      //device name,³¤¶È¿ÉÒÔ±ä»¯£¬Ð¡ÓÚ×î´óÖµ¼´¿É
      if ( len <= iBeaconProfile_CHAR11_LEN ) 
      {
        VOID memcpy( iBeaconProfileChar11, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

	  
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      iBeaconProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t iBeaconProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case iBeaconProfile_CHAR1:
      *((uint8*)value) = iBeaconProfileChar1;
      break;

    case iBeaconProfile_CHAR2:
      VOID memcpy( value, iBeaconProfileChar2, iBeaconProfile_CHAR2_LEN );
      break;      

    case iBeaconProfile_CHAR3:
      VOID memcpy( value, iBeaconProfileChar3, iBeaconProfile_CHAR3_LEN );
      break;  
	  
    case iBeaconProfile_CHAR4:
      VOID memcpy( value, iBeaconProfileChar4, iBeaconProfile_CHAR4_LEN );
      break;  
	  
    case iBeaconProfile_CHAR5:
      *((uint8*)value) = iBeaconProfileChar5;
      break;

    case iBeaconProfile_CHAR6:
      VOID memcpy( value, iBeaconProfileChar6, iBeaconProfile_CHAR6_LEN );
      break;

    case iBeaconProfile_CHAR7:
      VOID memcpy( value, iBeaconProfileChar7, iBeaconProfile_CHAR7_LEN );
      break;      

    case iBeaconProfile_CHAR8:
      VOID memcpy( value, iBeaconProfileChar8, iBeaconProfile_CHAR8_LEN );
      break;

    case iBeaconProfile_CHAR9:
      *((uint8*)value) = iBeaconProfileChar9;
      break;

    case iBeaconProfile_CHAR10:
      VOID memcpy( value, iBeaconProfileChar10, iBeaconProfile_CHAR10_LEN );
      break;

    case iBeaconProfile_CHAR11:
      VOID memcpy( value, iBeaconProfileChar11, iBeaconProfile_CHAR11_LEN );
      break;


    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          iBeaconProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static bStatus_t iBeaconProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method)
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case iBeaconProfile_CHAR1_UUID:
	  	*pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
		
      case iBeaconProfile_CHAR2_UUID:
	  	*pLen = iBeaconProfile_CHAR2_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR2_LEN );
        break;
		
	    case iBeaconProfile_CHAR3_UUID:
	  	*pLen = iBeaconProfile_CHAR3_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR3_LEN );
        break;
        
      case iBeaconProfile_CHAR4_UUID:
        *pLen = iBeaconProfile_CHAR4_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR4_LEN );
        break;
        
      case iBeaconProfile_CHAR5_UUID:
	  	*pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
		
      case iBeaconProfile_CHAR6_UUID:
        *pLen = iBeaconProfile_CHAR6_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR6_LEN );
        break;
      /*
      //Password can't read
      case iBeaconProfile_CHAR7_UUID:
        *pLen = iBeaconProfile_CHAR7_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR7_LEN );
        break;
		  */
      case iBeaconProfile_CHAR8_UUID:
        *pLen = iBeaconProfile_CHAR8_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR8_LEN );
        break;

      case iBeaconProfile_CHAR9_UUID:
        *pLen = iBeaconProfile_CHAR9_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR9_LEN );
        break;

      case iBeaconProfile_CHAR10_UUID:
        *pLen = iBeaconProfile_CHAR10_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR10_LEN );
        break;
        
      case iBeaconProfile_CHAR11_UUID:
        *pLen = iBeaconProfile_CHAR11_LEN;
        VOID memcpy( pValue, pAttr->pValue, iBeaconProfile_CHAR11_LEN );
        break;



      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      iBeaconProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t iBeaconProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case iBeaconProfile_CHAR1_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR1_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];
          notifyApp = iBeaconProfile_CHAR1;        

        }
             
        break;

	  case iBeaconProfile_CHAR2_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR2_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR2_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR2_LEN );
          notifyApp = iBeaconProfile_CHAR2;        
        }
	  	break;
		
	  case iBeaconProfile_CHAR3_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR3_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR3_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR3_LEN );
          notifyApp = iBeaconProfile_CHAR3;        
        }
		break;

      case iBeaconProfile_CHAR4_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR4_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR4_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR4_LEN );
          notifyApp = iBeaconProfile_CHAR4;        
        }
        break;
		
      case iBeaconProfile_CHAR5_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR5_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];
          notifyApp = iBeaconProfile_CHAR5;        

        }
        break;

      case iBeaconProfile_CHAR6_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > iBeaconProfile_CHAR6_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0x00,iBeaconProfile_CHAR6_LEN);
          //Èç¹ûÌØÕ÷ÖµµÄ³¤¶È¿É±ä£¬ÄÇÏÂÃæ±ØÐë¿½±´¾ßÌåµÄÊý¾Ý´óÐ¡£¬Èç¹û³¬¹ý¾ßÌåµÄÊý¾Ý£¬¾Í»á³öÏÖÂÒÂë£¬ÑÏÖØµÄ»áÄÚ´æ´íÎóËÀ»ú
          VOID memcpy( pCurValue, pValue, len );
          notifyApp = iBeaconProfile_CHAR6;        
        }
        break;

      case iBeaconProfile_CHAR7_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR7_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR7_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR7_LEN );
          notifyApp = iBeaconProfile_CHAR7;        
        }
        break;

      case iBeaconProfile_CHAR8_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR8_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR8_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR8_LEN );
          notifyApp = iBeaconProfile_CHAR8;        
        }
        break;

      case iBeaconProfile_CHAR9_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR9_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR9_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR9_LEN );
          notifyApp = iBeaconProfile_CHAR9;        
        }
        break;

      case iBeaconProfile_CHAR10_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != iBeaconProfile_CHAR10_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0,iBeaconProfile_CHAR10_LEN);
          VOID memcpy( pCurValue, pValue, iBeaconProfile_CHAR10_LEN );
          notifyApp = iBeaconProfile_CHAR10;        
        }
        break;

      case iBeaconProfile_CHAR11_UUID:
	  	//Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > iBeaconProfile_CHAR11_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          memset(pCurValue,0x00,iBeaconProfile_CHAR11_LEN);
          //Èç¹ûÌØÕ÷ÖµµÄ³¤¶È¿É±ä£¬ÄÇÏÂÃæ±ØÐë¿½±´¾ßÌåµÄÊý¾Ý´óÐ¡£¬Èç¹û³¬¹ý¾ßÌåµÄÊý¾Ý£¬¾Í»á³öÏÖÂÒÂë£¬ÑÏÖØµÄ»áÄÚ´æ´íÎóËÀ»ú
          VOID memcpy( pCurValue, pValue, len );
          notifyApp = iBeaconProfile_CHAR11;        
        }
        break;


     
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && iBeaconProfile_AppCBs && iBeaconProfile_AppCBs->pfniBeaconProfileChange )
  {
    iBeaconProfile_AppCBs->pfniBeaconProfileChange( notifyApp );  
  }
  
  return ( status );
}


/*********************************************************************
*********************************************************************/
