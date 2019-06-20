/**************************************************************************************************
  Filename:       iBeaconProfile.h
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple GATT profile definitions and
                  prototypes.

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

#ifndef IBEACONPROFILE_H
#define IBEACONPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define iBeaconProfile_CHAR1                   0  // RW uint8 - Profile Measured Power value 
#define iBeaconProfile_CHAR2                   1  // RW uint8 - Profile Major Value
#define iBeaconProfile_CHAR3                   2  // RW uint8 - Profile Minor Value
#define iBeaconProfile_CHAR4                   3  // RW uint8 - Profile Password value
#define iBeaconProfile_CHAR5                   4  // RW uint8 - Profile uuid1-4 value
#define iBeaconProfile_CHAR6                   5  // RW uint8 - Profile uuid5-8 value
#define iBeaconProfile_CHAR7                   6  // RW uint8 - Profile uuid9-12 value
#define iBeaconProfile_CHAR8                   7  // RW uint8 - Profile uuid13-16 value
#define iBeaconProfile_CHAR9                   8  // RW uint8 - Profile adv value
#define iBeaconProfile_CHAR10                  9  // RW uint8 - Profile control value
#define iBeaconProfile_CHAR11                  10  // RW uint8 - Profile control value


// Simple Profile Service UUID
#define iBeaconProfile_SERV_UUID               0xFFF0
    
// Key Pressed UUID
#define iBeaconProfile_CHAR1_UUID            0xFFF1
#define iBeaconProfile_CHAR2_UUID            0xFFF2
#define iBeaconProfile_CHAR3_UUID            0xFFF3
#define iBeaconProfile_CHAR4_UUID            0xFFF4
#define iBeaconProfile_CHAR5_UUID            0xFFF5
#define iBeaconProfile_CHAR6_UUID            0xFFF6
#define iBeaconProfile_CHAR7_UUID            0xFFF7
#define iBeaconProfile_CHAR8_UUID            0xFFF8
#define iBeaconProfile_CHAR9_UUID            0xFFF9
#define iBeaconProfile_CHAR10_UUID           0xFFFA
#define iBeaconProfile_CHAR11_UUID           0xFFFB


// Simple Keys Profile Services bit fields
#define iBeaconProfile_SERVICE               0x00000001

// Length of MeasuredPower
#define iBeaconProfile_CHAR1_LEN           1 
// Length of Major
#define iBeaconProfile_CHAR2_LEN           2  
// Length of Minor
#define iBeaconProfile_CHAR3_LEN           2 
// Length of ProximityUUID
#define iBeaconProfile_CHAR4_LEN           16 
// Length of AdvInterval
#define iBeaconProfile_CHAR5_LEN           1 

// Length of Commond
#define iBeaconProfile_CHAR6_LEN           18
// Length of Password
#define iBeaconProfile_CHAR7_LEN           6
// Length of DeviceName
#define iBeaconProfile_CHAR8_LEN           9
// Length of TxPower
#define iBeaconProfile_CHAR9_LEN           1
// Length of Password Check
#define iBeaconProfile_CHAR10_LEN          6
// Length of 
#define iBeaconProfile_CHAR11_LEN          18


/*********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*iBeaconProfileChange_t)( uint8 paramID );

typedef struct
{
  iBeaconProfileChange_t        pfniBeaconProfileChange;  // Called when characteristic value changes
} iBeaconProfileCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * iBeaconProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t iBeaconProfile_AddService( uint32 services );

/*
 * iBeaconProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t iBeaconProfile_RegisterAppCBs( iBeaconProfileCBs_t *appCallbacks );

/*
 * iBeaconProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t iBeaconProfile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * iBeaconProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t iBeaconProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* IBEACONPROFILE_H */
