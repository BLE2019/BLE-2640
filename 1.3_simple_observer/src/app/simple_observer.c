/******************************************************************************

 @file       simple_observer.c

 @brief This file contains the Simple Observer sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2011-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdio.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>
#include <driverlib/aon_batmon.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "observer.h"
#include "iotboard_key.h"
#include "board.h"

#include <ti/sysbios/knl/semaphore.h>
#include "simple_observer.h"

#include "hw_gpio.h"
#include "hw_spi.h"
#include "hw_i2c.h"
#include "hw_bma250e.h"
#include "hw_uart.h"
#include "GT20L16S1Y.h"
#include "TFT_touch.h"
#include "oxygen.h"
#include "gua_rtc.h"
#include "hw_pwm.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  4  //from 8 to 4

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 3000 //from 4000 to 3000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
#if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
#define SBO_DISPLAY_TYPE Display_Type_LCD
#elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
#define SBO_DISPLAY_TYPE Display_Type_UART
#else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
#define SBO_DISPLAY_TYPE 0 // Option not supported
#endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#define SBO_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define SBO_TASK_PRIORITY                     1

#ifndef SBO_TASK_STACK_SIZE
#define SBO_TASK_STACK_SIZE                   660
#endif

// ScanDetec duration in ms
#define DEFAULT_SVC_DISCOVERY_DELAY          1000
#define SOS_DETECT_DURATION                    2000      //按键扫描检测时长
#define LED_ON_DURATION                      3000
// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4100      //扫描超时时间
// ScanDetec duration in ms
#define SCANDETEC_DURATION                    2000      //按键扫描检测时长
// Awaken duration in ms
#define AWAKEN_DURATION                       2000      //定时唤醒时长
// Awaken duration in ms
#define PERIPHERAL_CHANGE_DELAY               250      //外设状态切换时长
// Sensor duration in ms
#define SENSOR_DATA_DELAY                     600000      //外设状态切换时长


// Internal Events for RTOS application
#define SBO_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBO_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBO_STATE_CHANGE_EVT                  Event_Id_00
#define SBO_KEY_CHANGE_EVT                      Event_Id_01
#define SBC_START_DISCOVERY_EVT             Event_Id_02
#define SBC_RSSI_READ_EVT                         Event_Id_03
#define SBC_LED_ON_EVT                               Event_Id_04            //扫描超时事件
#define SBC_SCAN_TIMEOUT_EVT                  Event_Id_05            //扫描超时事件
#define SBC_KEY_DETEC_EVT                        Event_Id_06            //按键扫描检测事件
#define SBC_AWAKEN_EVT                            Event_Id_07            //定时唤醒事件
#define PERIPHERAL_CHANGE_EVT              Event_Id_08            //外设状态切换事件
#define SENSOR_DATA_EVT                          Event_Id_09            //传感器数据上报事件



#define SBO_ALL_EVENTS                        (SBO_ICALL_EVT          | \
                                                                   SBO_QUEUE_EVT        | \
                                                                   SBO_STATE_CHANGE_EVT | \
                                                                    SBO_KEY_CHANGE_EVT | \
                                                                    SBC_START_DISCOVERY_EVT | \
                                                                    SBC_RSSI_READ_EVT | \
                                                                    SBC_LED_ON_EVT | \
                                                                    SBC_SCAN_TIMEOUT_EVT | \
                                                                     SBC_KEY_DETEC_EVT | \
                                                                     SBC_AWAKEN_EVT | \
                                                                     PERIPHERAL_CHANGE_EVT | \
                                                                     SENSOR_DATA_EVT)
#define      SOS_ALARM          0
#define      SOS_CANCEL         1
#define BEACON_UUID_LEN    12
const uint8 BEACON_UUID[BEACON_UUID_LEN]    = {0x57, 0x69, 0x73, 0x64, 0x6f, 0x6d, 0x53, 0x61, 0x66, 0x65, 0x74, 0x79}; //Beacon的UUID,自己定义修改的

char g_lcdBuffer[MAX_LCD_BUFF_LEN] = {0};
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct {
    appEvtHdr_t hdr; // event header
    uint8_t *pData;  // event data
} sboEvt_t;

enum MODE {                       //定义运行状态-1106
    NORMAL_MODE = 0,
    SOS_MODE = 1,
    READ_MODE = 2,
    WAKEUP_MODE = 3
} Running_Mode;



// App event passed from profiles.
typedef struct {
    appEvtHdr_t hdr; // event header
    uint8_t *pData;  // event data
} sbcEvt_t;

// RSSI read data structure
typedef struct {
    uint16_t period;      // how often to read RSSI
    uint16_t connHandle;  // connection handle
    Clock_Struct *pClock; // pointer to clock struct
} readRssi_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
//User-define -10.25
//static uint16_t ADC_Value = 0;                          //ADC采集电量值
static uint8_t Battery_Level = 0;                       //电池电量
static uint8_t SeqNum = 0;                              //数据数目
static uint8_t MsgType = 0;                             //数据类型
static uint8_t BEACON_DATA[5][6];                       //Beacon数据
static uint8_t SOS_KEY = 0;                             //SOS按键
static uint8_t READ_KEY = 0;                            //阅读按键

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct keyChangeClock;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

//自定义功能需求时钟-1022
static Clock_Struct scanTimeoutClock;           //扫描超时定时器
static Clock_Struct keyDetecClock;              //按键检测定时器
static Clock_Struct ledOnClock;                 //亮屏定时器
static Clock_Struct awakenClock;                //唤醒外设定时器
static Clock_Struct peripheralChangeClock;      //外设状态切换时钟
static Clock_Struct sensorDataClock;            //传感器数据时钟

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sboTask;
Char sboTaskStack[SBO_TASK_STACK_SIZE];

// GAP GATT Attributes
//static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Observer";

// Number of scan results and scan result index
static uint8 scanRes = 0 ;
static int8 scanIdx = -1;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 scanning = FALSE;
// Array of RSSI read structures
#define MAX_NUM_BLE_CONNS 1
static readRssi_t readRssi[MAX_NUM_BLE_CONNS];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLEObserver_init ( void );
static void SimpleBLEObserver_taskFxn ( UArg a0, UArg a1 );

static void SimpleBLEObserver_handleKeys ( uint8_t shift, uint8_t keys );
static void SimpleBLEObserver_processStackMsg ( ICall_Hdr *pMsg );
static void SimpleBLEObserver_processAppMsg ( sboEvt_t *pMsg );
static void SimpleBLEObserver_processRoleEvent ( gapObserverRoleEvent_t *pEvent );
static void SimpleBLEObserver_addDeviceInfo ( uint8 *pAddr, uint8 addrType );

static uint8_t SimpleBLEObserver_eventCB ( gapObserverRoleEvent_t *pEvent );

static uint8_t SimpleBLEObserver_enqueueMsg ( uint8_t event, uint8_t status,
        uint8_t *pData );

void SimpleBLEObserver_initKeys ( void );
void SimpleBLECentral_startDiscHandler ( UArg a0 );
void SimpleBLECentral_ledOnHandler ( UArg a0 );
void SimpleBLEObserver_keyChangeHandler ( uint8 keys );
void SimpleBLECentral_scanTimeoutHandler ( UArg a0 );
void SimpleBLECentral_readRssiHandler ( UArg a0 );
void SimpleBLECentral_keyDetecHandler ( UArg a0 );
void SimpleBLECentral_awaken_Handler ( UArg a0 );
void Peripheral_change_Handler ( UArg a0 );
void Sensor_Data_Handler ( UArg a0 );

//自定义功能函数
void SendSOStoCloud ( uint8_t SOS_TYPE );
void SendBeaconDatatoCloud ( void );                    //Beacon数据上报
void SendLowBatteryDatatoCloud ( void );                //低电量数据上报
void SendSensorDatatoCloud ( void );                    //扩展信息数据上报
uint16_t Calculate8To16 ( uint8_t *buf );
char *Hex2Str ( uint8_t *pAddr, uint8_t len );
void start_scan_beacon ( void );                          //启动扫描-1113

void delay_ms ( uint16_t n );
uint8_t read_inner_batt();
void read_outer_batt();
void test_rtc();
void test_NB_Module();
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB = {
    SimpleBLEObserver_eventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      SimpleBLEObserver_createTask
 *
 * @brief   Task creation function for the Simple Observer.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_createTask ( void )
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init ( &taskParams );
    taskParams.stack = sboTaskStack;
    taskParams.stackSize = SBO_TASK_STACK_SIZE;
    taskParams.priority = SBO_TASK_PRIORITY;

    Task_construct ( &sboTask, SimpleBLEObserver_taskFxn, &taskParams, NULL );
}

/*********************************************************************
 * @fn      SimpleBLEObserver_init
 *
 * @brief   Initialization function for the Simple Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
extern uint32_t acc_cnt;
uint32_t acc_int_cnt = 0;
void SimpleBLEObserver_init ( void )
{
    // ******************************************************************
    // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp ( &selfEntity, &syncEvent );

    // Hard code the DB Address till CC2650 board gets its own IEEE address
    //uint8 bdAddress[B_ADDR_LEN] = { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44 };
    //HCI_EXT_SetBDADDRCmd(bdAddress);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue ( &appMsg );

#if 0
    // Setup discovery delay as a one-shot timer
    Util_constructClock ( &startDiscClock, SimpleBLECentral_startDiscHandler,
                          DEFAULT_SVC_DISCOVERY_DELAY, DEFAULT_SVC_DISCOVERY_DELAY, true, SBC_START_DISCOVERY_EVT );
#endif
    //定时唤醒定时器，用来周期性唤醒外设
    acc_int_cnt = acc_cnt;
    Util_constructClock ( &awakenClock, SimpleBLECentral_awaken_Handler,
                          AWAKEN_DURATION, 0, false, SBC_AWAKEN_EVT );

    //扫描超时定时器2*2+0.1 = 4.1s
    Util_constructClock ( &scanTimeoutClock, SimpleBLECentral_scanTimeoutHandler,
                          DEFAULT_SCAN_DURATION, 0, false, SBC_SCAN_TIMEOUT_EVT );
    //按键状态检测定时器，超过1.5s为长按SOS报警，否则为睡眠熄屏
    Util_constructClock ( &keyDetecClock, SimpleBLECentral_keyDetecHandler,
                          SOS_DETECT_DURATION, 0, false, SBC_KEY_DETEC_EVT );

    //亮屏定时器
    Util_constructClock ( &ledOnClock, SimpleBLECentral_ledOnHandler,
                          LED_ON_DURATION, 0, false, SBC_LED_ON_EVT );

    //外设状态切换定时器，用来切换外设的状态
    Util_constructClock ( &peripheralChangeClock, Peripheral_change_Handler,
                          PERIPHERAL_CHANGE_DELAY, 0, false, PERIPHERAL_CHANGE_EVT );
    //传感器采集时钟，定时10分钟上报一次传感器数据
    Util_constructClock ( &sensorDataClock, Sensor_Data_Handler,
                          SENSOR_DATA_DELAY, 0, false, SENSOR_DATA_EVT );

    Board_initKeys(SimpleBLEObserver_keyChangeHandler);

    // Initialize internal data
    for ( uint8_t i = 0; i < MAX_NUM_BLE_CONNS; i++ ) {
        readRssi[i].connHandle = GAP_CONNHANDLE_ALL;
        readRssi[i].pClock = NULL;
    }

    // Setup Observer Profile
    {
        uint8 scanRes = DEFAULT_MAX_SCAN_RES;
        GAPObserverRole_SetParameter ( GAPOBSERVERROLE_MAX_SCAN_RES, sizeof ( uint8_t ), &scanRes );
    }

    // Setup GAP
    GAP_SetParamValue ( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
    GAP_SetParamValue ( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

    // Start the Device
    VOID GAPObserverRole_StartDevice ( ( gapObserverRoleCB_t * ) &simpleBLERoleCB );

    HwGPIOInit();
#if 1
    HwUARTInit();
    HwUARTWrite ( "*****start*****\r\n", 17 );

    ///////ACC///////////
    HwI2CInit();
    BMA250E_Init();

    //////LCD//////////
#endif
    HwSPIInit();
    GT20L_Init();
    TFT_Init_GX();

    //#define TEST_PWM 1
#ifdef TEST_PWM
    HwPWMInit();
    //  HwPWMStart(PWM_LED);
#else
    HwGPIOSet ( Board_LCD_LED_ON, 1 );
#endif
    //  LCD_ShowString(60, 80, "HELLO CHEM CHINA");

    TFT_Clear ( BLUE );
    LCD_ShowString ( 60, 80, "HELLO CHEM CHINA" );
    LCD_ShowMixString ( 60, 100, "智慧安监欢迎您" );
#ifdef TEST_PWM
    HwPWMStart ( PWM_LED );
#else
    HwGPIOSet ( Board_LCD_LED_ON, 1 );
#endif

    read_inner_batt();
#ifdef OUT_BATT_TEST
    read_outer_batt();
#endif
    RTC_SetTime(1412800000);
    RTC_ShowTime();
#ifdef MOTOR_TEST
#ifdef TEST_PWM
    HwPWMStart ( PWM_MOTOR );
    delay_ms ( 100 );
    HwPWMStop ( PWM_MOTOR );
#else
    HwGPIOSet ( Board_MOTOR, 1 );
    delay_ms ( 100 );
    HwGPIOSet ( Board_MOTOR, 0 );
#endif
#endif

#define NB_TEST 1
#ifdef NB_TEST
    HwGPIOSet(Board_NB_PEN, 1);
    //HwGPIOSet ( Board_NB_RST, 0 );
    //delay_ms ( 5 );
    //HwGPIOSet ( Board_NB_RST, 1 );
    delay_ms ( 5 );
    // HwUARTWrite ( "AT+QRST=1\r\n", 11 );
    delay_ms ( 10 );
    test_NB_Module();
    //HwGPIOSet ( Board_NB_PEN, 1 );
#endif
    start_scan_beacon();
    Util_startClock(&awakenClock);
#ifdef  OXY_TEST
    max30102_init();
#endif


}

/*********************************************************************
 * @fn      SimpleBLEObserver_taskFxn
 *
 * @brief   Application task entry point for the Simple Observer.
 *
 * @param   none
 *
 * @return  none
 */
static uint8_t sos_timer_running = 0;
static uint8_t wake_gap = 0;
static void SimpleBLEObserver_taskFxn ( UArg a0, UArg a1 )
{
    // Initialize application
    SimpleBLEObserver_init();

    // Application main loop
    for ( ;; ) {
        uint32_t events;

        events = Event_pend ( syncEvent, Event_Id_NONE, SBO_ALL_EVENTS,
                              ICALL_TIMEOUT_FOREVER );

        if ( events ) {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if ( ICall_fetchServiceMsg ( &src, &dest,
                                         ( void ** ) &pMsg ) == ICALL_ERRNO_SUCCESS ) {
                if ( ( src == ICALL_SERVICE_CLASS_BLE ) && ( dest == selfEntity ) ) {
                    // Process inter-task message
                    SimpleBLEObserver_processStackMsg ( ( ICall_Hdr * ) pMsg );
                }

                if ( pMsg ) {
                    ICall_freeMsg ( pMsg );
                }
            }
        }

        // If RTOS queue is not empty, process app message
        sprintf(g_lcdBuffer, "Evnet:0x%x", events);
        if ( events & SBO_QUEUE_EVT ) {
            while ( !Queue_empty ( appMsgQueue ) ) {
                sboEvt_t *pMsg = ( sboEvt_t * ) Util_dequeueMsg ( appMsgQueue );
                if ( pMsg ) {
                    // Process message
                    SimpleBLEObserver_processAppMsg ( pMsg );

                    // Free the space from the message
                    ICall_free ( pMsg );
                }
            }
        }

        if ( events & PERIPHERAL_CHANGE_EVT ) {   //外设状态切换
            events &= ~PERIPHERAL_CHANGE_EVT;
            if ( Running_Mode == SOS_MODE ) {
                Running_Mode = NORMAL_MODE;
            }
        }
        if ( events & SBC_AWAKEN_EVT ) {   //周期性定时唤醒
            static int wakeUpCnt = 0;
            sprintf(g_lcdBuffer, "WakeUp:%d", wakeUpCnt++);
            LCD_ShowString ( 40, 56, g_lcdBuffer );
            RTC_ShowTime();
            events &= ~SBC_AWAKEN_EVT;
            Util_startClock(&awakenClock);
            if ( acc_int_cnt != acc_cnt ) { //中间有运动
                acc_int_cnt = acc_cnt;
                wake_gap = 0;
                static int mvCnt = 0;
                sprintf(g_lcdBuffer, "DeviceMoved:%d", mvCnt++);
                LCD_ShowString ( 40, 126, g_lcdBuffer);
                //      HwADCInit();
                //      ADC_Value = HwADCRead();
                //      Battery_Level = (ADC_Value*100 )/ 4095;
                //      uint8_t Battery_Level;
                Battery_Level = read_inner_batt();
                if ( Battery_Level <= 20 ) {      //电量小于20%，屏显5s，熄屏上报
                    //Display_print();        //显示屏显示5s
                    SendLowBatteryDatatoCloud();
                } else {                          //电量大于20%，立即启动2640扫描Beacon
                    start_scan_beacon();
                }
            } else {
                wake_gap++;
                if ( wake_gap == 4 ) {
                    wake_gap = 0;
                    Battery_Level = read_inner_batt();
                    if ( Battery_Level <= 20 ) {      //电量小于20%，屏显5s，熄屏上报
                        //Display_print();        //显示屏显示5s
                        SendLowBatteryDatatoCloud();
                    } else {                          //电量大于20%，立即启动2640扫描Beacon
                        start_scan_beacon();
                    }
                }
            }
        }
        if ( events & SBC_SCAN_TIMEOUT_EVT ) {      //最大扫描超时时间-4.2ms
            events &= ~SBC_SCAN_TIMEOUT_EVT;

            if ( scanRes == 0 ) {                     // 如果扫描周期内没有扫描到BEACON设备，打印扫描超时
                HwUARTWrite ( "+SCAN=TIMEOUT, 0 FOUND\r\n", 24 );       //Debug调试
                LCD_ShowString ( 40, 70, "ScanTimeOut" );
            }
            if ( scanRes >= 1 ) {
                HwUARTWrite ( "+SCAN=TIMEOUT, 1-3 FOUND\r\n", 26 );     //Debug调试
                SendBeaconDatatoCloud();              //发送扫描的Beacon数据
                LCD_ShowString ( 40, 120, "FoundDevice" );
            }
        }
        if ( events & SBC_LED_ON_EVT ) {        //按键状态检测
            HwPWMStop ( PWM_LED );
            SOS_KEY = 0;
        }
        if ( events & SBC_KEY_DETEC_EVT ) {        //按键状态检测
            events &= ~SBC_KEY_DETEC_EVT;
            if ( SOS_KEY >= 15 ) {
                SOS_KEY = 0;
                if ( sos_timer_running ) {
                    sos_timer_running = 0;
                    Util_stopClock ( &keyDetecClock );
                }

                SendSOStoCloud(SOS_ALARM);  //SOS报警
                HwPWMStart ( PWM_LED );
                Util_stopClock ( &ledOnClock );
                Util_startClock ( &ledOnClock );
                LCD_ShowString ( 60, 80, "SOS IN PROGRESS..." );

                start_scan_beacon();
                //HwGPIOSet(Board_RLED,);
                Running_Mode = SOS_MODE;
            } else {
                HwPWMStop ( PWM_LED );
                SendSOStoCloud(SOS_CANCEL);     //SOS取消
                Running_Mode = NORMAL_MODE;
                //立即休眠
            }
        }
        if ( events & SENSOR_DATA_EVT ) {        //传感器数据上报
            events &= ~SENSOR_DATA_EVT;
            //        read_oxy_data();

            SendSensorDatatoCloud (); //待完善-1113
        }
    }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEObserver_processStackMsg ( ICall_Hdr *pMsg )
{
    switch(pMsg->event) {
        case GAP_MSG_EVENT:
            SimpleBLEObserver_processRoleEvent((gapObserverRoleEvent_t * )pMsg);
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processAppMsg ( sboEvt_t *pMsg )
{
    switch ( pMsg->hdr.event ) {
        case SBO_STATE_CHANGE_EVT:
            SimpleBLEObserver_processStackMsg ( ( ICall_Hdr * ) pMsg->pData );

            // Free the stack message
            ICall_freeMsg ( pMsg->pData );
            break;

        case SBO_KEY_CHANGE_EVT:
            SimpleBLEObserver_handleKeys ( 0, pMsg->hdr.state );
            break;

        case SBC_RSSI_READ_EVT: {
            readRssi_t *pRssi = ( readRssi_t * ) pMsg->pData;

            // If link is up and RSSI reads active
            if ( pRssi->connHandle != GAP_CONNHANDLE_ALL &&
                    linkDB_Up ( pRssi->connHandle ) ) {
                // Restart timer
                Util_restartClock ( pRssi->pClock, pRssi->period );

                // Read RSSI
                VOID HCI_ReadRssiCmd ( pRssi->connHandle );
            }
        }
        break;

        default:
            // Do nothing.
            break;
    }
}


void start_scan_beacon ( void )
{
    // Util_startClock(&scanTimeoutClock);

    //发起新的扫描之前，先清除扫描的设备个数
    //发起新的扫描之前，先取消原有行动
    if ( !scanning ) {
        //start scanning
        scanning = 1;
        scanRes = 0;

        GAPObserverRole_StartDiscovery ( DEFAULT_DISCOVERY_MODE,
                                         DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                         DEFAULT_DISCOVERY_WHITE_LIST );
    } else {
        // Cancel Scanning
        GAPObserverRole_CancelDiscovery();
    }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_handleKeys
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
static void SimpleBLEObserver_handleKeys ( uint8 shift, uint8 keys )
{
    HwPWMStart ( PWM_LED );
    Util_startClock ( &ledOnClock );

    if ( keys & KEY_BTN1 ) {              //Debug-待完善
        HwUARTWrite ( "---BTN 1 PRESSED---\r\n", 21 );
        if ( !sos_timer_running ) {
            Util_startClock ( &keyDetecClock );
            sos_timer_running = 1;
        }
        SOS_KEY ++ ;
    }

    if ( keys & KEY_BTN2 ) {            //待完善
        HwUARTWrite ( "---BTN 2 PRESSED---\r\n", 21 );

        READ_KEY = 1;
    }
    return;

}
#if 0
static void SimpleBLEObserver_handleKeys_BK ( uint8 shift, uint8 keys )
{
    ( void ) shift; // Intentionally unreferenced parameter

    // Left key determines action to take
    if ( keys & KEY_BTN1 ) {
        if ( !scanning ) {
            // Increment index
            scanIdx++;

            if ( scanIdx >= scanRes ) {
                // Prompt the user to begin scanning again.
                scanIdx = -1;
                Display_print0 ( dispHandle, 2, 0, "" );
                Display_print0 ( dispHandle, 3, 0, "" );
                Display_print0 ( dispHandle, 5, 0, "Discover ->" );
            } else {
                // Display the indexed scanned device.
                Display_print1 ( dispHandle, 2, 0, "Device %d", ( scanIdx + 1 ) );
                Display_print0 ( dispHandle, 3, 0, Util_convertBdAddr2Str ( devList[scanIdx].addr ) );
                Display_print0 ( dispHandle, 5, 0, "" );
                Display_print0 ( dispHandle, 6, 0, "<- Next Option" );
            }
        }
    }

    // Right key takes the actio the user has selected.
    if ( keys & KEY_BTN2 ) {
        if ( scanIdx == -1 ) {
            if ( !scanning ) {
                scanning = TRUE;
                scanRes = 0;

                Display_print0 ( dispHandle, 2, 0, "Discovering..." );
                Display_print0 ( dispHandle, 3, 0, "" );
                Display_print0 ( dispHandle, 4, 0, "" );
                Display_print0 ( dispHandle, 5, 0, "Cancel Discovery ->" );
                Display_print0 ( dispHandle, 6, 0, "" );

                GAPObserverRole_StartDiscovery ( DEFAULT_DISCOVERY_MODE,
                                                 DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                                 DEFAULT_DISCOVERY_WHITE_LIST );
            } else {
                // Cancel Scanning
                GAPObserverRole_CancelDiscovery();
            }
        }
    }
}
#endif
/*********************************************************************
 * @fn      SimpleBLEObserver_processRoleEvent
 *
 * @brief   Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processRoleEvent ( gapObserverRoleEvent_t *pEvent )
{
    char lcdBuffer[40] = {0};
    switch ( pEvent->gap.opcode ) {
        case GAP_DEVICE_INIT_DONE_EVENT:
            sprintf ( lcdBuffer, "Init Done" );
            break;

        case GAP_DEVICE_INFO_EVENT:
            SimpleBLEObserver_addDeviceInfo ( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
            scanIdx++;
            // 如果扫描的设备数据是扫描回调数据
            // 扫描到的BEACON数据是否为我们的BEACON，用户可自定义此数据
            if ( pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND ) {
                {
                    // 每扫描到一个符合BEACON UUID的设备，扫描结果加1
                    scanRes++;
                    // 打印扫描到的设备的MAC，UUID，MAJOR，MINOR, RSSI
                    //            TaskUARTdoWrite(NULL, NULL, "+SCAN=%s,%-.32s,%ld,%ld,%d\r\n",
                    //                            Util_convertBdAddr2Str(pEvent->deviceInfo.addr),
                    //                            Hex2Str(pEvent->deviceInfo.pEvtData+9,12),          //uuid 16
                    //                            Calculate8To16(pEvent->deviceInfo.pEvtData+21),     //major 2
                    //                            Calculate8To16(pEvent->deviceInfo.pEvtData+23),     //minor 2
                    //                            pEvent->deviceInfo.rssi);                           //rssi
                    memcpy ( &BEACON_DATA[scanRes][0], ( pEvent->deviceInfo.pEvtData + 21 ), 5 ); //Debug调试-10.25
                    BEACON_DATA[scanRes - 1][5] = pEvent->deviceInfo.rssi + 25;

                    if ( scanRes > 3 ) {
                        Util_stopClock ( &scanTimeoutClock );
                        SendBeaconDatatoCloud();                    //待完善-10.27
                        GAPObserverRole_CancelDiscovery();
                    }
                    sprintf ( lcdBuffer, "Found Deivce: %d", pEvent->deviceInfo.rssi + 25 );
                }
            }
            break;

        case GAP_DEVICE_DISCOVERY_EVENT: {
            // Discovery complete.
            scanning = FALSE;

            // Copy results.
            scanRes = pEvent->discCmpl.numDevs;
            memcpy ( devList, pEvent->discCmpl.pDevList,
                     ( sizeof ( gapDevRec_t ) * pEvent->discCmpl.numDevs ) );
            // Initialize scan index.
            scanIdx = -1;
        }
        break;

        default:
            break;
    }
    LCD_ShowMixString ( 40, 40, lcdBuffer );
}

/*********************************************************************
 * @fn      SimpleBLEObserver_eventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLEObserver_eventCB ( gapObserverRoleEvent_t *pEvent )
{
    // Forward the role event to the application
    if ( SimpleBLEObserver_enqueueMsg ( SBO_STATE_CHANGE_EVT,
                                        SUCCESS, ( uint8_t * ) pEvent ) ) {
        // App will process and free the event
        return FALSE;
    }

    // Caller should free the event
    return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLEObserver_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLEObserver_addDeviceInfo ( uint8 *pAddr, uint8 addrType )
{
    uint8 i;

    // If result count not at max
    if ( scanRes < DEFAULT_MAX_SCAN_RES ) {
        // Check if device is already in scan results
        for ( i = 0; i < scanRes; i++ ) {
            if ( memcmp ( pAddr, devList[i].addr, B_ADDR_LEN ) == 0 ) {
                return;
            }
        }

        // Add addr to scan result list
        memcpy ( devList[scanRes].addr, pAddr, B_ADDR_LEN );
        devList[scanRes].addrType = addrType;

        // Increment scan result count
        scanRes++;
    }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys pressed
 *
 * @return  none
 */
void SimpleBLEObserver_keyChangeHandler ( uint8 keys )
{
    SimpleBLEObserver_enqueueMsg ( SBO_KEY_CHANGE_EVT, keys, NULL );
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
void SimpleBLECentral_readRssiHandler ( UArg a0 )
{
    SimpleBLEObserver_enqueueMsg ( SBC_RSSI_READ_EVT, SUCCESS,
                                   ( uint8_t * ) &readRssi[a0] );
}


/*********************************************************************
 * @fn      SimpleBLEObserver_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLEObserver_enqueueMsg ( uint8_t event, uint8_t state,
        uint8_t *pData )
{
    sboEvt_t *pMsg;

    // Create dynamic pointer to message.
    if ( pMsg = ICall_malloc ( sizeof ( sboEvt_t ) ) ) {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

        // Enqueue the message.
        return Util_enqueueMsg ( appMsgQueue, syncEvent, ( uint8_t * ) pMsg );
    }

    return FALSE;
}
/*********************************************************************
 * @fn      SimpleBLECentral_scanTimeoutHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_scanTimeoutHandler ( UArg a0 )
{
    Event_post(syncEvent, a0);
}
void SimpleBLECentral_ledOnHandler ( UArg a0 )
{
    Event_post(syncEvent, a0);
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
void SimpleBLECentral_startDiscHandler ( UArg a0 )
{
    Event_post(syncEvent, a0);
}

/*********************************************************************
 * @fn      SimpleBLECentral_scanDetecHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyDetecHandler ( UArg a0 )
{
    Event_post(syncEvent, a0);
}
/*********************************************************************
 * @fn      SimpleBLECentral_awaken_Handler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_awaken_Handler ( UArg a0 )
{
    Event_post(syncEvent, a0);
}
/*********************************************************************
 * @fn      Peripheral_change_Handler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void Peripheral_change_Handler ( UArg a0 )
{
    Event_post(syncEvent, a0);
}

/*********************************************************************
 * @fn      Sensor_Data_Handler    //传感器数据上报-1105
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void Sensor_Data_Handler ( UArg a0 )
{
    Event_post(syncEvent, a0);
}


uint8_t read_inner_batt ( void )
{
    uint32_t batval = 0;
    uint8_t tmp[4] = {0};
    uint16_t bat_vol = 0;
    //  OVVIProfile_GetParameter(OVVIPROFILE_CHAR1, nbOVVI_Char1);
    AONBatMonEnable();
    batval = AONBatMonBatteryVoltageGet();
    tmp[0] = batval >> 24;
    tmp[1] = batval >> 16;
    tmp[2] = batval >> 8;
    tmp[3] = batval >> 0;

    bat_vol = tmp[2] * 100 + tmp[3] * 100 / 255;
    sprintf ( g_lcdBuffer, "Vol:%d mV", bat_vol * 10 );
    LCD_ShowString(40, 142, g_lcdBuffer);

    uint8_t level;
    level = tmp[2] * 10;
    return level;
}
void read_outer_batt()
{
    //   uint32_t turnedOnClocks = 0;
    uint8_t auxIo = Board_VOL;

    HwGPIOSet ( Board_SW_BATT_VOL, 1 );
#if 0
    //////////// Config clock/////////////////////
    // Only turn on clocks that are not already enabled. Not thread-safe, obviously.
    turnedOnClocks |= AUXWUCClockStatus ( AUX_WUC_ADC_CLOCK ) ? 0 : AUX_WUC_ADC_CLOCK;
    turnedOnClocks |= AUXWUCClockStatus ( AUX_WUC_ADI_CLOCK ) ? 0 : AUX_WUC_ADI_CLOCK;
    //    turnedOnClocks |= AUXWUCClockStatus(AUX_WUC_SOC_CLOCK) ? 0 : AUX_WUC_SOC_CLOCK;
    // Enable clocks and wait for ready
    AUXWUCClockEnable ( turnedOnClocks );
    while ( AUX_WUC_CLOCK_OFF == AUXWUCClockStatus ( turnedOnClocks ) );
    /////// Seclect auxIO  /////////////
    AUXADCSelectInput ( auxIo );
    ////////// Enable ///////////
    AUXADCEnableSync ( AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL );
    delay_ms ( 1 );
    //Scaling disable
    AUXADCDisableInputScaling();
    AUXADCGenManualTrigger();       // Trigger sample
    uint32_t adcValue = AUXADCReadFifo();
    AUXADCDisable();//Power_Saving
#else
    AUXWUCClockEnable ( AUX_WUC_MODCLKEN0_ANAIF_M | AUX_WUC_MODCLKEN0_AUX_ADI4_M );
    AUXADCSelectInput ( auxIo );
    AUXADCEnableSync ( AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL );
    AUXADCGenManualTrigger();
    uint32_t adcValue = AUXADCReadFifo();
    AUXADCDisable();
#endif
    //    return adcValue;
    HwGPIOSet ( Board_SW_BATT_VOL, 0 );

    uint8_t tmp[4];
    uint16_t bat_vol;

    tmp[0] = adcValue >> 24;
    tmp[1] = adcValue >> 16;
    tmp[2] = adcValue >> 8;
    tmp[3] = adcValue >> 0;
    char str[30] = {0};
    sprintf ( str, "batt adc is %d %d %d %d\r\n", tmp[0], tmp[1], tmp[2], tmp[3] );

    bat_vol = tmp[2] * 100 + tmp[3] * 100 / 255;
    sprintf ( str, "batt OUTER vol is %d mV\r\n", bat_vol * 10 ); //*14/10
    HwUARTWrite ( ( uint8_t * ) str, 30 );


}

void SendBeaconDatatoCloud ( void )
{
    uint8_t i;
    uint8_t buf[32];
    buf[0] = 0x31;                              //电信云地址识别域
    MsgType = 0x10;                             //数据类型
    SeqNum = scanRes;                           //Beacon数目
    buf[1] = ( MsgType & 0xf0 ) | ( SeqNum & 0x0f ); //数据类型为0，上报扫描
    for ( i = 0; i < scanRes; i++ ) {
        memcpy ( &buf[2 + i * 6], &BEACON_DATA[i][0], 6 );
    }
    HwUARTWrite ( buf, scanRes * 6 + 2 );

}
void SendSOStoCloud ( uint8_t SOS_TYPE )
{
    uint8_t buf[2];
    buf[0] = 0x32;                              //电信云地址识别域
    MsgType = 0x20;                             //数据类型
    SeqNum = SOS_TYPE;                           //sos数据类型
    buf[1] = (MsgType & 0xf0) | (SeqNum & 0x0f);
    HwUARTWrite ( buf, 2 );
}
/*********************************************************************
 * @fn      SendLowBatteryDatatoCloud
 *
 * @brief   发送终端低电量数据
 *
 * @param   无.
 *
 * @return  无.
 */
void SendLowBatteryDatatoCloud ( void )
{
    uint8_t buf[2];
    buf[0] = 0x33;                              //电信云地址识别域
    MsgType = 0x30;                             //数据类型
    SeqNum = 0;                                 //Beacon数目
    buf[1] = ( MsgType & 0xf0 ) | ( SeqNum & 0x0f ); //数据类型为0，上报扫描
    HwUARTWrite ( buf, 2 );
}

/*********************************************************************
 * @fn      SendSensorDatatoCloud
 *
 * @brief   发送扩展信息数据
 *
 * @param   无.
 *
 * @return  无.
 */
void SendSensorDatatoCloud ( void )         //待完善-1113
{
    static uint8_t VBat = 0;
    uint8_t buf[7];
    buf[0] = 0x34;                                              //电信云地址识别域
    MsgType = 0x40;                                             //数据类型
    SeqNum = 0x04;                                               //Beacon数目
    buf[1] = ( MsgType & 0xf0 ) | ( SeqNum & 0x0f );            //数据类型为0，上报扫描
    
    VBat = read_inner_batt();
    max30102_init();
    oxygen_get_value();
    buf[2] = VBat;                              //终端手表电量
    buf[3] = rxbuf[0];                          //心率        
    buf[4] = rxbuf[1];                          //血氧
    buf[5] = (Count_step >> 8) & 0x00ff;        //计步                
    buf[6] = Count_step & 0x00ff;
    HwUARTWrite ( buf, 7 );
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
char *Hex2Str ( uint8_t *pAddr, uint8_t len )
{
    uint8_t     charCnt;
    char        hex[] = "0123456789ABCDEF";
    static char str[50];
    char        *pStr = str;

    for ( charCnt = len; charCnt > 0; charCnt-- ) {
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
uint16_t Calculate8To16 ( uint8_t *buf )
{
    uint16_t value;
    value = ( buf[0] << 8 ) | ( buf[1] );
    return value;
}

void test_NB_Module()
{
    uint8_t delayTime = 4;
    HwUARTWrite ( "AT+CSQ\r\n", 8 );
    delay_ms ( delayTime * 2 );
    HwUARTWrite ( "AT+CESQ\r\n", 9 );
    delay_ms ( delayTime * 2 );

    HwUARTWrite ( "AT+QLWSERV=180.101.147.115,5683\r\n", 33 );
    delay_ms ( delayTime );
    //AT_QLWSERV_SENT;
    HwUARTWrite ( "AT+CGSN=1\r\n", 11 );
    delay_ms ( delayTime );
    //#if 1
    //设置EndPoint
    HwUARTWrite ( "AT+QLWCONF=\"866971037166019\"\r\n", 30 ); //IMSI866971037147407
    // HwUARTWrite ( "AT+QLWCONF=\"866971037147407\"\r\n", 30 ); //IMSI866971037147407
    sprintf(g_lcdBuffer, "%s", "IMSI:866971037166019");
    LCD_ShowString(60, 100, g_lcdBuffer);
    delay_ms ( delayTime );
    //配置LWM2M Object
    HwUARTWrite ( "AT+QLWADDOBJ=19,0,1,\"0\"\r\n", 25 );
    delay_ms ( delayTime );
    HwUARTWrite ( "AT+QLWADDOBJ=19,1,1,\"0\"\r\n", 25 );
    delay_ms ( delayTime );
    //发送请求更新
    HwUARTWrite ( "AT+QLWOPEN=0\r\n", 14 ); //open=1
    delay_ms ( 1000 );

    HwUARTWrite ( "AT+CGATT?\r\n", 11 );
    delay_ms ( delayTime * 2 );
}

void delay_ms ( uint16_t n )
{
    uint16_t delay = 20000;
    for ( int j = 0; j < n; j++ ) {
        for ( int i = 0; i < delay; i++ ) {
            asm ( "NOP" );
        }
    }
}
/*********************************************************************
*********************************************************************/
