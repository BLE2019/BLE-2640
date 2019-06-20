/******************************************************************************

 @file  simple_central.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/
/*内存划分说明:将nvFLASH中的0x80到0x8f段作为内存存储空间，其中每个段可用1k内存空间,
 *定义每段中的每个消息占用25个字节，每次读取255个字节，其中前五个字节为标记字段





*/
/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simple_gatt_profile.h"
#include <ti/mw/display/Display.h>
#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "Iotboard_key.h"
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include "task_uart.h"
#include "hw_gpio.h"
#include "hw_pwm.h"
#include "hw_i2c.h"
#include "hw_bma250e.h"
#include "watchdog.h"
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include "hw_spi.h"
#include "osal_snv.h"
//#include "C:\ti\simplelink\ble_sdk_2_02_01_18\examples\cc2650iot\5.11_advance_scanibeacon\source\driver\pwm\hw_pwm.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
//单次读取内存的大小
#define           SIZE                         255
uint8_t databuf[200];
uint8_t msg_receive;
Datasegment datasegment;

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020




//#define SBC_ALLMSG_READ_EVT                   0x0400 //0323
//#define SBC_ONEMSG_READ_EVT                   0x0800 //0323
#define SBC_LORA_RX_EVT                       0x1000 //0323


#define DISPLAY_ENABLE  1
   
//扫描超时事件
#define SBC_SCAN_TIMEOUT_EVT                  0x0040
//报警循环事件
#define ADC_DETECT_EVT                        0x0080 
//按键保持检测事件
#define BUTTON_DETECT_EVT                     0x0100 
//加速度定时检测事件
#define ACCELEROMETER_EVT                     0x0200 
//定时唤醒事件   
#define WAKEUP_EVT                            0x2000
//Beacon扫描检测事件   
#define SCAN_DETECT_EVT                       0x4000 
//外设状态切换事件
#define PERIPHERAL_CHANGE_EVT                 0x8000

//唤醒时间
#if DISPLAY_ENABLE
#define WAKEUP_DELAY                          180000 
#else  
#define WAKEUP_DELAY                          60000 
#endif // 有显示终端的唤醒时间为8秒，无显示终端的唤醒时间为12秒
//Beacon扫描检测时间 
#define SCAN_DETECT_DELAY                     5000
//外设状态切换时间
#define PERIPHERAL_CHANGE_DELAY               250
//加速度检测时间
#define ACCELEROMETER_DELAY                   4000 
//按键保持检测时间
#define BUTTON_DETECT_DELAY                   50 
//SOS状态循环时间
#define ADC_DETECT_DELAY                      2400
// 扫描超时时间
#define SCAN_TIMEOUT_DELAY                    4200
//按键有效检测时间
//#define DEFAULT_SOS_BUTTON_DELAY              100  

//定义运行状态标记变量   
#define RUNNING_STATE                         0
#define SLEEP_STATE                           1
#define WAKEUP_CLOCK_STATE                    2
#define ACCELEROMETER_STATE                   3  
#define LORA_INTERRUPT_STATE                  4
#define SOS_BUTTON_STATE                      5
#define READ_BUTTON_STATE                     6


#define GENERAL_NO_SOS                        0
#define SOS_ALARM                             1
#define NO_SOS_NO_DISPLAY                     2
#define NO_SOS_DISPLAY                        3
#define SOS_DISPLAY                           4
#define SOS_NO_DISPLAY                        5


//当前运行状态变量  
uint16_t wakeup_event; 
uint16_t laster_state;
uint16_t peripheral_hold_time; 
int16_t batterypower;
uint16_t Peripheral=0;
uint8_t Beacon_data[10][31];
int8_t Beacon_rssi[10][1];
uint8_t button;
uint8_t button_state=0;
uint8_t peripheral_state;
uint8_t read_wait;
uint8_t test_times;
uint8_t seqNum;

//休眠唤醒定时事件时钟
static Clock_Struct wakeup_Clock;
//唤醒定时扫描检测事件时钟
static Clock_Struct scan_detect_Clock;
//外设状态切换时钟
static Clock_Struct peripheral_change_Clock;
//加速度检测时钟
static Clock_Struct accelerometer_Clock;
//按键保持时间检测时钟
static Clock_Struct button_detect_Clock; 
//SOS状态循环时钟
static Clock_Struct adc_detect_Clock; 
//扫描超时时钟
static Clock_Struct scanTimeoutClock;

//定时唤醒时间中断服务
static void Wakeup_Handler(UArg a0);
//扫描检测事件中断服务
static void Scan_detect_Handler(UArg a0);
//外设状态切换事件中断服务
static void Peripheral_change_Handler(UArg a0);   
//加速度检测事件中断服务
static void Accelerometer_Handler(UArg a0);
//按键保持时间检测事件中断服务
static void Button_detect_Handler(UArg a0);
//SOS状态循环事件中断服务
static void Adc_detect_Handler(UArg a0);

static void send_msg_to_lora();


// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  50



// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPCENTRALROLE_PARAM_UPDATE_REQ_AUTO_ACCEPT

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      40

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      40

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          2

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           4000

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000 


#define DEFAULT_ALLMSG_BUTTON_DELAY 3000 //0324
#define DEFAULT_ONEMSG_READ_DELAY 6000 //0324, show msg duration


// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct
{
  uint16_t period;      // how often to read RSSI
  uint16_t connHandle;  // connection handle
  Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;




static Clock_Struct onemsgReadClock; //0324
static Clock_Struct allmsgReadClock; //0324



// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Queue object used for uart messages
static Queue_Struct uartRxMsg;
static Queue_Handle uartRxQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes;




// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;


// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Array of RSSI read structures
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];


#define BEACON_UUID_LEN    16
const uint8 BEACON_UUID[BEACON_UUID_LEN]	={0xFD,0xA5,0x06,0x93,  0xA4,0xE2,0x4F,0xB1,  0xAF,0xCF,0xC6,0xEB,  0x07,0x64,0x78,0x25};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle);

static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle);
static void SimpleBLECentral_RssiFree(uint16_t connHandle);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_scanTimeoutHandler(UArg a0);
void SimpleBLECentral_sosButtonHandler(UArg a0); //0323
void SimpleBLECentral_keyChangeHandler(uint8 keys);
void SimpleBLECentral_readRssiHandler(UArg a0);

void Watchdog_Init(void);//0419
static void TransBMA250EDataCallback(uint8_t *buf);


static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);

#ifdef FPGA_AUTO_CONNECT
static void SimpleBLECentral_startGapDiscovery(void);
static void SimpleBLECentral_connectToFirstDevice(void);
#endif // FPGA_AUTO_CONNECT

void TransUartReceiveDataCallback(uint8_t *buf, uint16_t len);
void Simple_Central_WriteData(uint8_t *buf,uint8_t len);
void SimpleBLECentral_printfScanHandler(UArg a0);
uint16_t Calculate8To16(uint8_t *buf);
char *Hex2Str(uint8_t *pAddr, uint8_t len);
void read_ten_msg(void);
void read_new_msg(void);
void send_reply_to_lora();
uint8 read_pin_get_value(void);
uint32_t ADC_Read(void);
void send_battary_to_lora(uint16_t value);
uint8_t crc_high_first(uint8_t *ptr, uint8_t len);
void send_sos_to_lora();
void send_batterypower_to_lora();
//-----------0323--------------
#if 201903

char msg_buff[30] ="the latest msg for test";

uint8  sos_pin_get_value( );


void SimpleBLECentral_onemsgReadHandler(UArg a0);
void SimpleBLECentral_allmsgReadHandler(UArg a0);
void SimpleBLECentral_startDiscovery_new(void);
void send_scanbeacon_to_lora(void);
void send_low_pwr_to_lora(void);
void send_sos_to_lora(void);

static void Msg_init(void);


uint8 pwr_pin_get_value(void)
{
	uint8 pwr_ratio = 85;
	return pwr_ratio;
}

uint8 sos_pin_get_value(void)
{
	uint8 sos_pin = 0;
          sos_pin=HwGPIOGet(Board_BTN1);
	return sos_pin;
}
uint8 read_pin_get_value(void)
{
	uint8 read_pin = 0;
          read_pin=HwGPIOGet(Board_BTN2);
	return read_pin;
}
void sleep_mode_enter(void)
{
}
void buzzer_change(uint8_t state)
{
  HwGPIOSet(Board_MOTOR,state);
  if(state==0)HwPWMStop(PWM_Buzzer);
  else HwPWMStart(PWM_Buzzer);

}
void buzzer_on(uint8_t state)
{
  HwGPIOSet(Board_MOTOR,state);
  HwPWMStart(PWM_Buzzer);

}
void buzzer_off(void)
{
	HwGPIOSet(Board_MOTOR,0);
	HwPWMStop(PWM_Buzzer);

}
void sos_led_on(void)
{
	  HwGPIOSet(Board_RLED,1);
}
void sos_led_off(void)
{
	  HwGPIOSet(Board_RLED,0);
}
void 	show_latest_msg()
{
		Util_startClock(&onemsgReadClock);
		
	     dispHandle = Display_open(Display_Type_LCD, NULL);
	    Display_print0(dispHandle, 3, 0, msg_buff); //just for test 0324
	    
	    // if latest msg have not read, 
	    //latest_msg_read = 1;
	    //send_have_read_latest_lora();	    
}
void send_soscancel_to_lora()
{
  TaskUARTWrite("SoScancel!\r\n",12);
}
void send_sos_to_lora()
{
  uint8_t buf1[80];
  buf1[0]=(1&0xf0)|(seqNum&0x0f);  //数据类型为0，上报扫描
  Uart_TxTempLen=1;
  Uart_TxTempBuf[0]=0x3c;
  Uart_TxTempBuf[1]=0x02;
  Uart_TxTempBuf[2]=Uart_TxTempLen;
  for(uint8_t i=0;i<Uart_TxTempLen;i++)
     Uart_TxTempBuf[i+3]=*(buf1+i);
  Uart_TxTempBuf[3+Uart_TxTempLen]=CalcCS1(Uart_TxTempBuf,Uart_TxTempLen+3);
  Uart_TxTempBuf[4+Uart_TxTempLen]=0x0d;
  TaskUARTWrite(Uart_TxTempBuf,Uart_TxTempLen+5);
}

void send_batterypower_to_lora()
{
  uint8_t buf1[80];
  buf1[0]=(2&0xf0)|(seqNum&0x0f);  //数据类型为0，上报扫描
  buf1[1]=((batterypower&0xff00)>>8);
  buf1[2]=(batterypower&0x00ff);
  Uart_TxTempLen=3;
  Uart_TxTempBuf[0]=0x3c;
  Uart_TxTempBuf[1]=0x02;
  Uart_TxTempBuf[2]=Uart_TxTempLen;
  for(uint8_t i=0;i<Uart_TxTempLen;i++)
     Uart_TxTempBuf[i+3]=*(buf1+i);
  Uart_TxTempBuf[3+Uart_TxTempLen]=CalcCS1(Uart_TxTempBuf,Uart_TxTempLen+3);
  Uart_TxTempBuf[4+Uart_TxTempLen]=0x0d;
  TaskUARTWrite(Uart_TxTempBuf,Uart_TxTempLen+5);
  
  
  
}


void send_reply_to_lora()
{
  TaskUARTWrite("OK!\r\n",5);


}
//static void send_msg_to_lora()
//{
//  Uart_TxTempBuf[0]=0x3c;
//  Uart_TxTempBuf[1]=0x01;
//  Uart_TxTempBuf[2]=0x00;
//  Uart_TxTempBuf[3]=0x3d;
//  Uart_TxTempBuf[4]=0x0d;
//  TaskUARTWrite(Uart_TxTempBuf,5);
//}

void send_data_to_lora(uint8_t *buf,uint8_t len)
{
  Uart_TxTempBuf[0]=0x3c;
  Uart_TxTempBuf[1]=0x82;
  Uart_TxTempBuf[2]=len;
  for(uint8_t i=0;i<len;i++)
  Uart_TxTempBuf[i+3]=*(buf+i);
  Uart_TxTempBuf[3+len]=CalcCS1(Uart_TxTempBuf,len+3);
  Uart_TxTempBuf[4+len]=0x0d;
  TaskUARTWrite(Uart_TxTempBuf,len+5);
}
void send_scanbeacon_to_lora()
{
  uint8_t i=0;
  uint8_t buf1[80];
  //根据所扫描到的蓝牙数目，发送信息给Lora模块
  buf1[0]=(0&0xf0)|(seqNum&0x0f);  //数据类型为0，上报扫描
  for(i=0;i<scanRes;i++)
  {
//    TaskUARTdoWrite_addframe(NULL, NULL, "%s,%-.32s,%ld,%ld,%d,%d\r\n",
//                    Util_convertBdAddr2Str(Beacon_data[i]),
//                    Hex2Str(Beacon_data[i]+9,4),     //uuid 16的最后4位
//                    Calculate8To16(Beacon_data[i]+13),    //major 2
//                    Calculate8To16(Beacon_data[i]+15),
//                    *(Beacon_data[i]+17),    //minor 2
//                    Beacon_rssi[i][0]);          //rssi  
//        TaskUARTdoWrite_addframe(NULL, NULL, "%s,%-.32s,%d\r\n",
//                    Util_convertBdAddr2Str(Beacon_data[i]),
//                    Hex2Str(Beacon_data[i]+9,4),     //uuid 16的最后4位
//                    Beacon_rssi[i][0]);          //rssi
    buf1[5*i+1]=Beacon_data[i][13];
    buf1[5*i+2]=Beacon_data[i][14];
    buf1[5*i+3]=Beacon_data[i][15];
    buf1[5*i+4]=Beacon_data[i][16];
    buf1[5*i+5]=Beacon_rssi[i][0];
  }
      Uart_TxTempLen=5*i+1;
      Uart_TxTempBuf[0]=0x3c;
      Uart_TxTempBuf[1]=0x02;
      Uart_TxTempBuf[2]=Uart_TxTempLen;
      for(uint8_t i=0;i<Uart_TxTempLen;i++)
         Uart_TxTempBuf[i+3]=*(buf1+i);
      Uart_TxTempBuf[3+Uart_TxTempLen]=CalcCS1(Uart_TxTempBuf,Uart_TxTempLen+3);
      Uart_TxTempBuf[4+Uart_TxTempLen]=0x0d;
      TaskUARTWrite(Uart_TxTempBuf,Uart_TxTempLen+5);
}
#endif
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  (pfnPasscodeCB_t)SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB                  // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#ifdef FPGA_AUTO_CONNECT
/*********************************************************************
 * @fn      SimpleBLECentral_startGapDiscovery
 *
 * @brief   Start discovering devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_startGapDiscovery(void)
{
  // Start discovery
  if ((state != BLE_STATE_CONNECTED) && (!scanningStarted))
  {
    scanningStarted = TRUE;
    scanRes = 0;

    Display_print0(dispHandle, 2, 0, "Discovering...");
    Display_clearLines(dispHandle, 3, 4);

    GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                  DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                  DEFAULT_DISCOVERY_WHITE_LIST);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_connectToFirstDevice
 *
 * @brief   Connect to first device in list of discovered devices
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_connectToFirstDevice(void)
{
  uint8_t addrType;
  uint8_t *peerAddr;

  scanIdx = 0;

  if (state == BLE_STATE_IDLE)
  {
    // connect to current device in scan result
    peerAddr = devList[scanIdx].addr;
    addrType = devList[scanIdx].addrType;

    state = BLE_STATE_CONNECTING;

    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 addrType, peerAddr);

    Display_print0(dispHandle, 2, 0, "Connecting");
    Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddr));
    Display_clearLine(dispHandle, 4);
  }
}
#endif // FPGA_AUTO_CONNECT

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  
  // Create an RTOS queue for message from profile to be sent to app.
  uartRxQueue = Util_constructQueue(&uartRxMsg);
  
  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);
                   
  Util_constructClock(&scanTimeoutClock, SimpleBLECentral_scanTimeoutHandler,
                      SCAN_TIMEOUT_DELAY, 0, false, SBC_SCAN_TIMEOUT_EVT);

//0323--------------------------------------

  Util_constructClock(&adc_detect_Clock, Adc_detect_Handler,
                      ADC_DETECT_DELAY, ADC_DETECT_DELAY, true, ADC_DETECT_EVT);

 

// Util_constructClock(&onemsgReadClock, SimpleBLECentral_onemsgReadHandler,
//                      DEFAULT_ONEMSG_READ_DELAY, 0, false, 0);
// Util_constructClock(&allmsgReadClock, SimpleBLECentral_allmsgReadHandler,
//                      DEFAULT_ALLMSG_BUTTON_DELAY, 0, false, 0);
  //定时唤醒定时器，用来周期性唤醒外设
  Util_constructClock(&wakeup_Clock, Wakeup_Handler,
                      WAKEUP_DELAY, WAKEUP_DELAY, true, WAKEUP_EVT);
  //加速度检测定时器，周期性检测胸卡状态
  Util_constructClock(&accelerometer_Clock, Accelerometer_Handler,
                      ACCELEROMETER_DELAY, ACCELEROMETER_DELAY, true,ACCELEROMETER_EVT);
  //外设状态切换定时器，用来切换外设的状态
  Util_constructClock(&peripheral_change_Clock, Peripheral_change_Handler,
                      PERIPHERAL_CHANGE_DELAY, 0, false, PERIPHERAL_CHANGE_EVT);
  //扫描检测定时器，5S内是否执行过扫描
  Util_constructClock(&scan_detect_Clock, Scan_detect_Handler,
                      SCAN_DETECT_DELAY, 0, false, SCAN_DETECT_EVT);
  //按键保持时间检测，50MS检测一次
  Util_constructClock(&button_detect_Clock, Button_detect_Handler,
                      BUTTON_DETECT_DELAY, 0, false, BUTTON_DETECT_EVT);
   Board_initKeys(SimpleBLECentral_keyChangeHandler);

  dispHandle = Display_open(Display_Type_LCD, NULL);


  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
    readRssi[i].pClock = NULL;
  }

  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                &scanRes);
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, SCAN_TIMEOUT_DELAY);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, SCAN_TIMEOUT_DELAY);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  HCI_LE_ReadMaxDataLenCmd();
//  Display_print0(dispHandle, 0, 0, "Scan Beacon");
//  
//  TaskUARTdoWrite(NULL, NULL, "%s\r\n", "Scan Beacon");
  GY_UartTask_RegisterPacketReceivedCallback(TransUartReceiveDataCallback);

  HwGPIOInit();
  //----0324----
   // I2C
  HwI2CInit();
  BMA250E_Init();
  GY_BMA250E_RegisterPacketReceivedCallback(TransBMA250EDataCallback);
  //   PWM
  HwPWMInit();
  //看门狗初始化
  Watchdog_Init();
  //SPI通信初始化
  HwSPIInit();
  //LoraModuleInit();
  Msg_init();
  HwGPIOSet(Board_GLED, 0);
  HwGPIOSet(Board_RLED, 0);
  HwGPIOSet(Board_YLED, 0);
}

/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
 //0323
 uint8 tick = 0;
#define MAX_SOS_TICK  30
int SOS_MODE = GENERAL_NO_SOS; //0323;
uint16 sosLoopCnt = 0;

static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLECentral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLECentral_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }
    //如果串口RX接收数据队列不为空，处理串口数据 
    while (!Queue_empty(uartRxQueue))                
    {
      uint8_t *pMsg = (uint8_t *)Util_dequeueMsg(uartRxQueue);  //获取队列信号
      if (pMsg)
      {
        
        Simple_Central_WriteData(pMsg+1,pMsg[0]);   //调用发送函数
        ICall_free(pMsg);
      }
    }
    if (events & BUTTON_DETECT_EVT)                           //按键检测
    {
      events &= ~BUTTON_DETECT_EVT;
      if(button==1)                                           //SOS按键
      {
        uint8 sos_pin = sos_pin_get_value();
        // 如果持续按下SOS 
        if (sos_pin == 0)
        {
          tick++;
          if(tick > MAX_SOS_TICK)
          {
            wakeup_event=SOS_BUTTON_STATE;
            SOS_MODE = SOS_ALARM;
	  tick = 0;	
            for(uint8_t i=0;i<252;i++)databuf[i]=0;           
            databuf[0]=FLASH_SAVE;
            databuf[1]=0x00;
            databuf[2]=0x00;
            databuf[3]=0x00;
            databuf[4]=0x19;
            datasegment.new_msg_address=databuf[0];
            datasegment.new_msg_num=databuf[1];
            datasegment.all_msg_num=databuf[2];
            datasegment.local_msg_num=databuf[3];
            datasegment.msg_length=databuf[4];
            osal_snv_write(FLASH_SAVE,252, databuf);
            Util_startClock(&scan_detect_Clock);
            Util_stopClock(&button_detect_Clock);
            Util_stopClock(&button_detect_Clock);
            Util_startClock(&peripheral_change_Clock);
            //开启扫描检测定时器，定时5s，没有进行扫描即进行扫描
            HwGPIOSet(Board_RLED,1);
            Util_startClock(&scanTimeoutClock);
            bStatus_t status; 
            //发起新的扫描之前，先清除扫描的设备个数
            scanRes = 0;
            //发起新的扫描之前，先取消原有行动
            status = GAPCentralRole_CancelDiscovery();
            if(status != SUCCESS)
               GAPCentralRole_CancelDiscovery();
            //start scanning
            GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST); 
	  button=0;		
          }
          else
          {
	  Util_startClock(&button_detect_Clock);	
          }
        }
        else
        {
          if(SOS_MODE==SOS_ALARM)                              //处于一键报警状态，进行取消操作
          {
            Util_stopClock(&button_detect_Clock);
            Util_stopClock(&scan_detect_Clock);
            Util_stopClock(&scanTimeoutClock);
            send_soscancel_to_lora();
          }
          buzzer_change(0);
          HwGPIOSet(Board_GLED, 0);
          HwGPIOSet(Board_RLED, 0);
          SOS_MODE = GENERAL_NO_SOS;                           //恢复一般状态
          tick = 0;
          button=0;
          /*  debug  */
          osal_snv_write(FLASH_SAVE, 50, databuf);
          TaskUARTWrite(databuf,49);
          /*  debug  */
        }
      }
      if(button==2)                                                      //READ按键
      {
        uint8 read_pin = read_pin_get_value();
        if (read_pin == 0)
        {
          tick++;
          //Display_print1(dispHandle, 6, 0, "Get Value: %d", sos_tick);
          if(tick > 60)
          {
            wakeup_event=READ_BUTTON_STATE;
	  tick=0;
            read_ten_msg();
            send_reply_to_lora();
	  button=0;
            Display_print0(dispHandle, 0, 0, "**new message**");
            if(datasegment.all_msg_num==0)
            {
              Display_print0(dispHandle, 1, 0, "  no message  ");
            }
            else 
            {
              if(datasegment.all_msg_num<6)
              {
                for(uint8_t i=0;i<datasegment.all_msg_num;i++)
                {
                  Display_print1(dispHandle, i+1, 0, "%s", databuf+5+25*i);
                }
              }
              else
              {
                for(uint8_t i=datasegment.all_msg_num-5;i<datasegment.all_msg_num;i++)
                {
                  Display_print1(dispHandle, 6-(datasegment.all_msg_num-i), 0, "%s", databuf+5+25*i);
                }
              } 
            }
            databuf[0]=datasegment.new_msg_address;
            databuf[1]=datasegment.new_msg_num;
            databuf[2]=datasegment.all_msg_num;
            databuf[3]=datasegment.local_msg_num;
            databuf[4]=datasegment.msg_length;
            SOS_MODE = SOS_DISPLAY;
            read_wait=100;
            Util_startClock(&peripheral_change_Clock);
          }
         else
         {
	 Util_startClock(&button_detect_Clock);
         }
        }
        else
        {
          
          if(read_wait==0&&(wakeup_event!=READ_BUTTON_STATE))            //按键唤醒模式，短时按按键阅读最新消息
          {
            button=0;
            tick=0;
            SOS_MODE = SOS_DISPLAY;
            read_wait=100;
            Display_clear(dispHandle);
            buzzer_change(0);
            HwGPIOSet(Board_GLED, 0);
            HwGPIOSet(Board_RLED, 0);
            osal_snv_read(FLASH_SAVE, 252, databuf);
            if(datasegment.new_msg_num>0)
            {
              Display_print0(dispHandle, 0, 0, "**new message**");
              Display_print1(dispHandle, 1, 0, "%s", databuf+5+(datasegment.new_msg_num-1)*25);
            }
            else if(datasegment.new_msg_num==0)
            {
              Display_print0(dispHandle, 0, 0, "**new message**");
              Display_print0(dispHandle, 1, 0, "**** no one ****");
            }
            //osal_snv_write(FLASH_SAVE, 252, databuf);
            Util_startClock(&peripheral_change_Clock);
          }
          else if(read_wait>0&&read_wait<100)                           //处于阅读等待状态，按下阅读按键阅读消息
          {
            read_wait=100;
            Display_clear(dispHandle);
            buzzer_change(0);
            HwGPIOSet(Board_GLED, 0);
            HwGPIOSet(Board_RLED, 0);
            if(datasegment.new_msg_num>0)
            {
              Display_print0(dispHandle, 0, 0, "**new message**");
              Display_print1(dispHandle, 1, 0, "%s", databuf+5+(datasegment.new_msg_num-1)*25);
            }
            else if(datasegment.new_msg_num==0)
            {
              Display_print0(dispHandle, 0, 0, "**new message**");
              Display_print0(dispHandle, 1, 0, "**** no one ****");
            }
            osal_snv_write(FLASH_SAVE, 252, databuf);
            Util_startClock(&peripheral_change_Clock);
          }
          else if(read_wait>100|wakeup_event==READ_BUTTON_STATE)        //第一次按下阅读按键后，等待显示5s后消失
          {
            read_wait=0;
            buzzer_change(0);
            HwGPIOSet(Board_GLED, 0);
            HwGPIOSet(Board_RLED, 0);
            Peripheral=0;
            peripheral_hold_time=0;
            Util_startClock(&wakeup_Clock);
            Util_stopClock(&peripheral_change_Clock);
            send_reply_to_lora();
            SOS_MODE=GENERAL_NO_SOS;
            wakeup_event=SLEEP_STATE;
            Display_clear(dispHandle);
          }
        }
      }
    }
    if (events & SBC_START_DISCOVERY_EVT)                                //扫描检测
    {
      events &= ~SBC_START_DISCOVERY_EVT;

      SimpleBLECentral_startDiscovery();
    }
    if (events & SBC_SCAN_TIMEOUT_EVT)                                   //扫描超时检测
    {
      events &= ~SBC_SCAN_TIMEOUT_EVT;

      //HwGPIOSet(Board_RLED,0);
      // 如果扫描周期内没有扫描到BEACON设备，打印扫描超时
      if (scanRes == 0)
      {
//        TaskUARTdoWrite(NULL, NULL, "+SCAN=TIMEOUT\r\n", NULL);
        //wakeup_event=SLEEP_STATE;
        //Display_clear(dispHandle);
      }
      else
      {
        send_scanbeacon_to_lora(); //扫描超时，有多于一个，也发
        //发起新的扫描之前，先清除扫描的设备个数
        bStatus_t status; 
            //发起新的扫描之前，先清除扫描的设备个数
        scanRes = 0;
            //发起新的扫描之前，先取消原有行动
        status = GAPCentralRole_CancelDiscovery();
        if(status != SUCCESS)
           GAPCentralRole_CancelDiscovery();
            //start scanning
        //Util_stopClock(&scanTimeoutClock);
        //wakeup_event=SLEEP_STATE;
        Display_clear(dispHandle);
      }
      if(SOS_MODE==SOS_ALARM)
      {
         Util_startClock(&scanTimeoutClock);
         bStatus_t status; 
            //发起新的扫描之前，先清除扫描的设备个数
         scanRes = 0;
            //发起新的扫描之前，先取消原有行动
         status = GAPCentralRole_CancelDiscovery();
         if(status != SUCCESS)
            GAPCentralRole_CancelDiscovery();
            //start scanning
         GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST);   
      }
    }
    WatchdogReloadSet(50000 * (1000 / Clock_tickPeriod));                //看门狗重载五秒	
    if (events & ACCELEROMETER_EVT)                                      //加速度传感器中断检测
    {
       bma_warn=BMA250E_GetData();
       events &= ~ACCELEROMETER_EVT;
       if(bma_warn==0)//胸卡平放
       {
         Util_startClock(&accelerometer_Clock);
         Util_startClock(&adc_detect_Clock);
         if(wakeup_event==SLEEP_STATE)
         {
           Util_restartClock(&wakeup_Clock,WAKEUP_DELAY);
           laster_state=ACCELEROMETER_STATE;
           wakeup_event=RUNNING_STATE; 
           //电量检测
           batterypower=ADC_Read();
           //Display_print1(dispHandle, 4, 0, "ADC: %ld ", batterypower);
           if(batterypower<2500)                                 //低电量
           {
               Util_startClock(&peripheral_change_Clock);
               laster_state=WAKEUP_CLOCK_STATE;
               wakeup_event=RUNNING_STATE;
               //TaskUARTPrintf("ADC: %ld, %.2fV\r\n",batterypower,batterypower*4.3/4095);
               send_battary_to_lora(batterypower);
           }
           else
           {
             //开启扫描检测定时器，定时5s，没有进行扫描即进行扫描
             HwGPIOSet(Board_RLED,1);
             Util_startClock(&scanTimeoutClock);
             bStatus_t status; 
             //发起新的扫描之前，先清除扫描的设备个数
             scanRes = 0;
             //发起新的扫描之前，先取消原有行动
             status = GAPCentralRole_CancelDiscovery();
             if(status != SUCCESS)
                GAPCentralRole_CancelDiscovery();
             //start scanning
             GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST); 

            }
          }
       }
       else         //胸卡佩戴
       {
         Util_startClock(&accelerometer_Clock);
       }
    }

    if (events & ADC_DETECT_EVT)                                         //ADC检测及串口数据处理
    {
      events &= ~ADC_DETECT_EVT;
      test_times++;
      batterypower=ADC_Read();
      HwGPIOSet(Board_YLED, 0);
      if(test_times==100)test_times=0;
      if(databuf[0]!=0&&msg_receive==1)                       //配置状态
      {
        osal_snv_write(FLASH_SAVE, 252, databuf);             //将数据写入nvFLASH
        msg_receive=0;
        osal_snv_read(FLASH_SAVE, 252, databuf);              //从nvFLASH中读取数据
        TaskUARTWrite(databuf,252);
        datasegment.new_msg_address=databuf[0];
        datasegment.new_msg_num=databuf[1];
        datasegment.all_msg_num=databuf[2];
        datasegment.local_msg_num=databuf[3];
        datasegment.msg_length=databuf[4];
      } 
      else if(databuf[0]!=0&&msg_receive==2)                      //接收到新消息
      {
        osal_snv_write(FLASH_SAVE,252, databuf);                     //将数据写入nvFLASH 
        datasegment.all_msg_num++;
        datasegment.new_msg_num++;
        datasegment.local_msg_num++;
        databuf[0]=datasegment.new_msg_address;
        databuf[1]=datasegment.new_msg_num;
        databuf[2]=datasegment.all_msg_num;
        databuf[3]=datasegment.local_msg_num;
        databuf[4]=datasegment.msg_length;
        osal_snv_write(FLASH_SAVE, 252, databuf);
        TaskUARTPrintf("%ld",datasegment.local_msg_num);
        msg_receive=0;
      } 
      else if(databuf[0]!=0&&msg_receive==3)                      //SOS命令
      {
        //存储SOS消息
        //osal_snv_write(FLASH_SAVE,252, databuf);                     //将数据写入nvFLASH 
        datasegment.all_msg_num++;
        datasegment.new_msg_num++;
        datasegment.local_msg_num++;
        databuf[0]=datasegment.new_msg_address;
        databuf[1]=datasegment.new_msg_num;
        databuf[2]=datasegment.all_msg_num;
        databuf[3]=datasegment.local_msg_num;
        databuf[4]=datasegment.msg_length;
        osal_snv_write(FLASH_SAVE, 252, databuf);                           //新消息存入
        TaskUARTPrintf("%ld",datasegment.local_msg_num);
        msg_receive=0;
        //存储SOS消息
        if(DISPLAY_ENABLE==1)
        {
          SOS_MODE=SOS_DISPLAY;                                              //主动接收到SOS消息,有显示模块,进入外设切换
          Util_startClock(&peripheral_change_Clock);
        }
        else 
        {
          SOS_MODE=SOS_NO_DISPLAY;                                          //主动接收到SOS消息,无显示模块，进入外设切换
          Util_startClock(&peripheral_change_Clock);
        }
      } 
       else if(databuf[0]!=0&&msg_receive==4)                                             //其他非SOS消息
      {
        wakeup_event=SLEEP_STATE;
        msg_receive=0;
      }
      else if(databuf[0]!=0&&msg_receive==5)                                             //其他非SOS消息
      {
        if(DISPLAY_ENABLE==0)                                              //非SOS消息，且无显示器，直接进入休眠模式
        {
          SOS_MODE=NO_SOS_NO_DISPLAY;
          wakeup_event=SLEEP_STATE;
          msg_receive=0;
          SOS_MODE=GENERAL_NO_SOS;
        }
        else                                                              //非SOS消息，有显示器,已完成
        {
          SOS_MODE=NO_SOS_DISPLAY;                                        //有显示器，进入外设状态切换
          Util_startClock(&peripheral_change_Clock);
          msg_receive=0;
        }
      } 
       else if(databuf[0]!=0&&msg_receive==6)                                             //其他非SOS消息
      {

        Display_print0(dispHandle, 1, 0, "Get received");
      } 
    }
    if (events & SCAN_DETECT_EVT)                                        //唤醒扫描时间检测
    {
      events &= ~SCAN_DETECT_EVT;   
    }
    if (events & PERIPHERAL_CHANGE_EVT)                                  //外设状态切换，已完成
    {
      peripheral_hold_time++;
      events &= ~PERIPHERAL_CHANGE_EVT;
      if(SOS_MODE == GENERAL_NO_SOS)                                     //一般情况调用外设切换-电量不足
      {
        HwGPIOSet(Board_GLED, 0);
        Peripheral++;
        HwGPIOSet(Board_YLED, Peripheral%2);
        Util_stopClock(&wakeup_Clock);
        Util_startClock(&peripheral_change_Clock);
        //Display_print1(dispHandle, 6, 0, "Num: %ld ", Peripheral);
        buzzer_change(Peripheral%2);
        if(peripheral_hold_time<2)
          send_batterypower_to_lora();
        else if(peripheral_hold_time>20)
        {
          buzzer_change(0);
          HwGPIOSet(Board_GLED, 0);
          HwGPIOSet(Board_RLED, 0);
          Peripheral=0;
          peripheral_hold_time=0;
          Util_startClock(&wakeup_Clock);
          Util_stopClock(&peripheral_change_Clock);
          wakeup_event=SLEEP_STATE;
         }
      }
      else if(SOS_MODE == SOS_ALARM)                                     //一键报警模式下外设状态切换
      {
        HwGPIOSet(Board_GLED, 0);
        Peripheral++;
        HwGPIOSet(Board_YLED, Peripheral%2);
        Util_startClock(&peripheral_change_Clock);
        Util_stopClock(&wakeup_Clock);
        wakeup_event=SOS_BUTTON_STATE;
        //Display_print1(dispHandle, 6, 0, "Num: %ld ", Peripheral);
        buzzer_change(Peripheral%2);
        if(Peripheral%20==0)
        //上报报警信息
          send_sos_to_lora();
        if(Peripheral>200)Peripheral=0;
        if(peripheral_hold_time>200)peripheral_hold_time=0;
      }
      else if(SOS_MODE == SOS_NO_DISPLAY)                                //接收到SOS命令，无显示模块，外设切换5S后进入休眠状态
      {
        HwGPIOSet(Board_GLED, 0);
        Peripheral++;
        buzzer_change(Peripheral%2);
        HwGPIOSet(Board_RLED, Peripheral%2);
        Util_stopClock(&wakeup_Clock);
        Util_startClock(&peripheral_change_Clock);
        if(peripheral_hold_time>20)
        {
          buzzer_change(0);
          HwGPIOSet(Board_GLED, 0);
          HwGPIOSet(Board_RLED, 0);
          Peripheral=0;
          peripheral_hold_time=0;
          Util_startClock(&wakeup_Clock);
          Util_stopClock(&peripheral_change_Clock);
          wakeup_event=SLEEP_STATE;
        }
      }
      else if(SOS_MODE == SOS_DISPLAY)                                   //接收到SOS命令，有显示模块，持续进行外设状态切换
      {
        if(read_wait<20)                                                //刚接收消息的前5s
        {
          HwGPIOSet(Board_GLED, 0);
          Peripheral++;
          read_wait++;
          HwGPIOSet(Board_RLED, Peripheral%2);
          Util_startClock(&peripheral_change_Clock);
          Util_stopClock(&wakeup_Clock);
          buzzer_change(Peripheral%2);
          if(peripheral_hold_time>200)peripheral_hold_time=0;
        }
        else if(read_wait>=20)                                         //阅读等待5s时间到
        {
           if(read_wait==20)
           {
             buzzer_change(0);
             HwGPIOSet(Board_GLED, 0);
             HwGPIOSet(Board_RLED, 0);
           }
           read_wait++;
           Util_stopClock(&wakeup_Clock);
           Util_startClock(&peripheral_change_Clock);
           if(read_wait==100|read_wait==120)
           {
             Peripheral=0;
             peripheral_hold_time=0;
             read_wait=0;
             Util_stopClock(&peripheral_change_Clock);
             Util_startClock(&wakeup_Clock);
             SOS_MODE=GENERAL_NO_SOS;
             wakeup_event=SLEEP_STATE;
             Display_clear(dispHandle);
           } 
        }
      }
      else if(SOS_MODE == NO_SOS_DISPLAY)                               //接收到非SOS命令，有显示模块，外设切换10S后进入休眠状态
      {
        HwGPIOSet(Board_GLED, 0);
        Peripheral++;
        buzzer_change(Peripheral%2);
        Util_startClock(&peripheral_change_Clock);
        Util_stopClock(&wakeup_Clock);
        if(peripheral_hold_time>40)
        {
          buzzer_change(0);
          HwGPIOSet(Board_GLED, 0);
          HwGPIOSet(Board_RLED, 0);
          Peripheral=0;
          peripheral_hold_time=0;
          Util_startClock(&wakeup_Clock);
          Util_stopClock(&peripheral_change_Clock);
          SOS_MODE=GENERAL_NO_SOS;
          wakeup_event=SLEEP_STATE;
        }
      }
      else                                                              //其他状态，停止外设切换
      {
        buzzer_change(0);
        HwGPIOSet(Board_GLED, 0);
        HwGPIOSet(Board_RLED, 0);
        Peripheral=0;
        peripheral_hold_time=0;
        Util_startClock(&wakeup_Clock);
        Util_stopClock(&peripheral_change_Clock);
      }
    }
    if (events & WAKEUP_EVT)                                            //定时器唤醒处理事件，已完成
    {
      events &= ~WAKEUP_EVT;
      //唤醒显示
      Display_print0(dispHandle, 6, 0, "have waked up"); 
      Util_startClock(&adc_detect_Clock);
      //电量检测
      batterypower=ADC_Read(); 
      if(batterypower<2500)                                 //低电量
      {
        Util_startClock(&peripheral_change_Clock);
        laster_state=WAKEUP_CLOCK_STATE;
        wakeup_event=RUNNING_STATE;;
      }
      else
      {
        laster_state=WAKEUP_CLOCK_STATE;
        wakeup_event=RUNNING_STATE;
        HwGPIOSet(Board_RLED,1);
        Util_startClock(&scanTimeoutClock);
        bStatus_t status; 
        //发起新的扫描之前，先清除扫描的设备个数
        scanRes = 0;
        //发起新的扫描之前，先取消原有行动
        status = GAPCentralRole_CancelDiscovery();
        if(status != SUCCESS)
           GAPCentralRole_CancelDiscovery();
        //start scanning
        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST); 
      }
    }

//    switch(wakeup_event)
//    {
//      case RUNNING_STATE:                                     //运行状态
//        {
//          Display_print0(dispHandle, 7, 0, "running");
//          break;
//        }
//      case SLEEP_STATE:                                       //休眠状态
//        {
//          
//          Display_print0(dispHandle, 7, 0, "sleep");  
//          break;
//        }
//      case WAKEUP_CLOCK_STATE:                                //休眠定时唤醒,工作模式1，
//        {
//          Display_print0(dispHandle, 7, 0, "wakeup"); 
//          break;
//        }
//      case ACCELEROMETER_STATE:                              //加速度传感器唤醒，工作模式1
//        {
//          Display_print0(dispHandle, 7, 0, "accelerometer");   
//          break;
//        }
//      case LORA_INTERRUPT_STATE:                              //LORA唤醒，工作模式2
//        {
//          Display_print0(dispHandle, 7, 0, "Lora");  
//          break;
//        }
//      case SOS_BUTTON_STATE:                                 //报警按键唤醒，工作模式3
//        {
//          Display_print0(dispHandle, 7, 0, "SOS");   
//          break;
//        }
//      case READ_BUTTON_STATE:                                //阅读按键唤醒，工作模式4
//        {
//          Display_print0(dispHandle, 7, 0, "Read");   
//          break;
//        }
//      default:break;
//    }
  }
     
  
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT: //general access profile
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT: // host control interface
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->hdr.state);
      break;

    case SBC_RSSI_READ_EVT:
      {
        readRssi_t *pRssi = (readRssi_t *)pMsg->pData;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
            linkDB_Up(pRssi->connHandle))
        {
          // Restart timer
          Util_restartClock(pRssi->pClock, pRssi->period);

          // Read RSSI
          VOID HCI_ReadRssiCmd(pRssi->connHandle);
        }
      }
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleBLECentral_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SimpleBLECentral_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  uint8_t i;
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        maxPduSize = pEvent->initDone.dataPktLen;

        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        Display_print0(dispHandle, 2, 0, "Initialized");

#ifdef FPGA_AUTO_CONNECT
        SimpleBLECentral_startGapDiscovery();
#endif // FPGA_AUTO_CONNECT
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // 如果扫描的设备数据是扫描回调数据
        if(pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND)
        {
          // 扫描到的BEACON数据是否为我们的BEACON，用户可自定义此数据
          if(memcmp(pEvent->deviceInfo.pEvtData+9,BEACON_UUID,BEACON_UUID_LEN-4) == 0)
          {
            // 每扫描到一个符合BEACON UUID的设备，扫描结果加1
            scanRes++;
            for(i=0;i<9;i++)
            {
              Beacon_data[scanRes-1][i]=*(pEvent->deviceInfo.addr+i);
            }
            for(i=9;i<13;i++)                //传感器数据
            {
              Beacon_data[scanRes-1][i]=*(pEvent->deviceInfo.pEvtData+i+12);
            }
            for(i=13;i<15;i++)               //major
            {
              Beacon_data[scanRes-1][i]=*(pEvent->deviceInfo.pEvtData+i+12);
            }
            for(i=15;i<17;i++)               //minor
            {
              Beacon_data[scanRes-1][i]=*(pEvent->deviceInfo.pEvtData+i+12);
            }
            Beacon_data[scanRes-1][17]=laster_state;
            Beacon_rssi[scanRes-1][0]=pEvent->deviceInfo.rssi;

           //----0323------------
           if(scanRes > 4) //扫到足够的站点
           {
             //Util_stopClock(&scanTimeoutClock);
	   send_scanbeacon_to_lora(); // 
             bStatus_t status; 
             //发起新的扫描之前，先清除扫描的设备个数
             scanRes = 0;
             //发起新的扫描之前，先取消原有行动
             status = GAPCentralRole_CancelDiscovery();
             if(status != SUCCESS)
                GAPCentralRole_CancelDiscovery();
               HwGPIOSet(Board_RLED, 1);
           }
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        Display_print1(dispHandle, 2, 0, "Beacon Found %d", scanRes);        
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

          Display_print0(dispHandle, 2, 0, "Connect Failed");
          Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;

        // Cancel RSSI reads
        SimpleBLECentral_CancelRssi(pEvent->linkTerminate.connectionHandle);

        Display_print0(dispHandle, 2, 0, "Disconnected");
        Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
        Display_clearLine(dispHandle, 4);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 
 */

//static void SimpleBLECentral_startDiscovery_new(void)
//{
//	Util_startClock(&scanTimeoutClock);
//    
//    bStatus_t status; 
//    //发起新的扫描之前，先清除扫描的设备个数
//    scanRes = 0;
//    //发起新的扫描之前，先取消原有行动
//    status = GAPCentralRole_CancelDiscovery();
//    
//    if(status != SUCCESS)
//       GAPCentralRole_CancelDiscovery();
//    
//    //start scanning
//    GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                  DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                  DEFAULT_DISCOVERY_WHITE_LIST); 
//
//}
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{

  if (keys & KEY_BTN1) //SOS按钮
  {
    button=1;
  }
  if(keys & KEY_BTN2)  //READ按钮
  {
    button=2;
  }
  Util_startClock(&button_detect_Clock);
  return;
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
      Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
        Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal++);
      }

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
    {
      TaskUARTdoWrite(pMsg->msg.handleValueNoti.pValue,
                      pMsg->msg.handleValueNoti.len, NULL, NULL);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleBLECentral_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
        int8 rssi = (int8)pMsg->pReturnParam[3];

        Display_print1(dispHandle, 4, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static bStatus_t SimpleBLECentral_CancelRssi(uint16_t connHandle)
{
  readRssi_t *pRssi;

  if ((pRssi = SimpleBLECentral_RssiFind(connHandle)) != NULL)
  {
    // Stop timer
    Util_stopClock(pRssi->pClock);

    // Free RSSI structure
    SimpleBLECentral_RssiFree(connHandle);

    return SUCCESS;
  }

  // Not found
  return bleIncorrectMode;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFind
 *
 * @brief   Find an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  pointer to structure or NULL if not found.
 */
static readRssi_t *SimpleBLECentral_RssiFind(uint16_t connHandle)
{
  uint8_t i;

  // Find free RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      return &readRssi[i];
    }
  }

  // Not found
  return NULL;
}

/*********************************************************************
 * @fn      gapCentralRole_RssiFree
 *
 * @brief   Free an RSSI structure.
 *
 * @param   connHandle - Connection handle
 *
 * @return  none
 */
static void SimpleBLECentral_RssiFree(uint16_t connHandle)
{
  uint8_t i;

  // Find RSSI structure
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (readRssi[i].connHandle == connHandle)
    {
      readRssi_t *pRssi = &readRssi[i];
      if (pRssi->pClock)
      {
        Clock_destruct(pRssi->pClock);

        // Free clock struct
        ICall_free(pRssi->pClock);
        pRssi->pClock = NULL;
      }

      pRssi->connHandle = GAP_CONNHANDLE_ALL;
      break;
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
      Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
      Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      // Just in case we're using the default MTU size (23 octets)
      Display_print1(dispHandle, 4, 0, "MTU Size: %d", ATT_MTU_SIZE);

      discState = BLE_DISC_STATE_SVC;

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);

      Display_print0(dispHandle, 2, 0, "Simple Svc Found");
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}



/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT,
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  events |= SBC_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}

void SimpleBLECentral_scanTimeoutHandler(UArg a0)
{
  events |= SBC_SCAN_TIMEOUT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}

//中断服务函数
void Button_detect_Handler(UArg a0)
{
  events |= BUTTON_DETECT_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}

void Accelerometer_Handler(UArg a0)
{
  events |= ACCELEROMETER_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}
//void SimpleBLECentral_onemsgReadHandler(UArg a0)
//{
//  events |= SBC_ONEMSG_READ_EVT;
//
//  // Wake up the application thread when it waits for clock event
//  Semaphore_post(sem);
//
//}
//
//void SimpleBLECentral_allmsgReadHandler(UArg a0)
//{
//  events |= SBC_ALLMSG_READ_EVT;
//
//  // Wake up the application thread when it waits for clock event
//  Semaphore_post(sem);
//
//}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_readRssiHandler
 *
 * @brief   Read RSSI handler function
 *
 * @param   a0 - read RSSI index
 *
 * @return  none
 */
void SimpleBLECentral_readRssiHandler(UArg a0)
{
  SimpleBLECentral_enqueueMsg(SBC_RSSI_READ_EVT, SUCCESS,
                              (uint8_t *)&readRssi[a0]);
}

void Wakeup_Handler(UArg a0)
{
  events |= WAKEUP_EVT;
//  laster_state=wakeup_event;
//  wakeup_event=WAKEUP_CLOCK_STATE;
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}

void Scan_detect_Handler(UArg a0)
{
  events |= SCAN_DETECT_EVT;
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}
void Peripheral_change_Handler(UArg a0)
{
  events |= PERIPHERAL_CHANGE_EVT;
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

} 

void Adc_detect_Handler(UArg a0)
{
  events |= ADC_DETECT_EVT;
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);

}









/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      TransUartReceiveDataCallback
 *
 * @brief   .
 *
 * @param   buf - .
 * @param   len - .
 *
 * @return  none.
 */
static void TransUartReceiveDataCallback(uint8_t *buf, uint16_t len)
{
  uint8_t *pMsg;
  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(200)))
  {
    pMsg[0] = len;
    memcpy(pMsg+1, buf, len);
    // Enqueue the message.
    Util_enqueueMsg(uartRxQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
 * @fn      Simple_Central_WriteData
 *
 * @brief   .
 *
 * @param   buf - .
 * @param   len - .
 *
 * @return  none.
 */
void Simple_Central_WriteData(uint8_t *buf,uint8_t len)
{
  bStatus_t status;
  attWriteReq_t req; 
  req.handle = 0x1E;
  req.len = len>20 ? 20 : len;
  req.sig = 0;
  req.cmd = TRUE;
  
  req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 23, NULL);
  
  if ( req.pValue != NULL )
  {
    memcpy(req.pValue,buf,len);
    status = GATT_WriteNoRsp(connHandle, &req);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }
}

/*********************************************************************
 * @fn      Hex2Str
 *
 * @brief   将16进制hex数据转换成字符串用做打印.
 *
 * @param   pAddr - 16进制指针.
 * @param   len - 长度.
 *
 * @return  str - 字符串.
 */
char *Hex2Str(uint8_t *pAddr, uint8_t len)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[50];
  char        *pStr = str;

  for (charCnt = len; charCnt > 0; charCnt--)
  {
    *pStr++ = hex[*pAddr >> 4];
    *pStr++ = hex[*pAddr++ & 0x0F];
  }
  pStr = NULL;

  return str;
}

/*********************************************************************
 * @fn      Hex2Str
 *
 * @brief   将两个高低位的uint8型转换成uint16型.
 *
 * @param   buf - 包含两个uint8型的字节.
 *
 * @return  uint16 - uint16型的字节.
 */
uint16_t Calculate8To16(uint8_t *buf)
{
  uint16_t value;
  value = (buf[0] << 8) | (buf[1]);
  return value;
}

/*********************************************************************
*********************************************************************/
//加速度传感器数据回调函数


static void TransBMA250EDataCallback(uint8_t *buf)
{
  uint8_t *pMsg;
  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(200)))
  {
    pMsg[0] = 3;
    pMsg[1] = buf[1];
    pMsg[2] = buf[3];
    pMsg[3] = buf[5];
    // Enqueue the message.
    Util_enqueueMsg(uartRxQueue, sem, (uint8*)pMsg);
  }
}



//watchdog

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 @brief 软件看门狗的初始化函数
 @param none
 @return none
*/
void Watchdog_Init(void)
{

  WatchdogReloadSet(50000 * (1000 / Clock_tickPeriod)); //五秒
  WatchdogResetEnable();
  WatchdogEnable();
}
//watchdog
void read_ten_msg(void)
{
  
  
}
void read_new_msg(void)
{
  
  
}
uint32_t ADC_Read(void) 
{ 
  AUXWUCClockEnable(AUX_WUC_MODCLKEN0_ANAIF_M|AUX_WUC_MODCLKEN0_AUX_ADI4_M); 
  AUXADCSelectInput(ADC_COMPB_IN_AUXIO0); 
  AUXADCEnableSync(AUXADC_REF_FIXED,AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL); 
  AUXADCGenManualTrigger(); 
  uint32_t ADCVal = AUXADCReadFifo(); 
  AUXADCDisable(); 
  return ADCVal; 
}

void send_battary_to_lora(uint16_t value)
{
  TaskUARTWrite("Low battary!\r\n",14);
  
  
  
}
//存储消息初始化
static void Msg_init(void)                               
{

  for(uint8_t i=0;i<252;i++)databuf[i]=0;
  //VOID osal_snv_read(FLASH_SAVE, 252, databuf);
  datasegment.new_msg_address=databuf[0];                       //最新消息所在段 
  datasegment.new_msg_num=databuf[1];                           //最新未读消息数
  datasegment.all_msg_num=databuf[2];                           //所有消息数（最多80条）
  datasegment.local_msg_num=databuf[3];                         //当前段消息数
  datasegment.msg_length=databuf[4];                            //消息长度
//  if(datasegment.new_msg_address!=BLE_NVID_CUST_START)       //不是最新消息段，更新读取最新段
//  {
//    osal_snv_read(datasegment.new_msg_address, 5, value);
//    datasegment.new_msg_address=value[0]+BLE_NVID_CUST_START;
//    datasegment.new_msg_num=value[1];
//    datasegment.all_msg_num=value[2];
//    datasegment.local_msg_num=value[3];
//    datasegment.msg_length=value[4];
//  }
//  TaskUARTPrintf("address:%d\r\n",datasegment.new_msg_address);
}
uint8_t crc_high_first(uint8_t *ptr, uint8_t len)                 //crc8校验
{
    uint8_t i;
    uint8_t crc=0x00; /* 计算的初始crc值 */ 

    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
                else crc >>= 1;
        }

    }

    return (crc); 
}


