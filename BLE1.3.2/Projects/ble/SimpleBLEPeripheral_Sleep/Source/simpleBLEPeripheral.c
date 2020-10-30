/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "myApplication.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif
#include "npi.h"
#include "stdio.h"

#include "hal_i2c.h"
 
#include "MAX30205.h" 
#include "MAX30102.h"
#include "RN5T618.h"

#include "hal_flash.h"
#include "eeprom.h"

#include "algorithm.h"
#include "Crc16.h"
    
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   1000
#define SBP_PERIODIC_EVT_PERIOD2                    3// 发送数据的周期，可以修改，现在是 1 秒发一次

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          (160*10)

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     800//20

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800//20

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000//500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6
// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

int timerCount = 0;
int8 LCD_NeedRun = 1;
uint8_t measure_flag;
uint16_t measure_pos;

uint8 heart_flag = 0;
uint8 heart_cnt;
uint8 KEY_NeedPrcoess = 0;
uint8 DIS_Item = 0;

uint16 nGUA_ConnHandle;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
 /* 0x14,   // length of this data
  
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'
*/
  0x0b,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x68,   // 'h'
  0x6f,   // 'o'
  0x77,   // 'w'
  0x57,   // 'W'
  0x47,   // 'G'
  0x75,   // 'u'
  0x61,   // 'a'
  0x72,   // 'r'
  0x64,   // 'd'
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
// 设备名称，可以自己改
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "BlueTooth";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

// 发送数据到主机的接口
//void simpleBLE_SendData(uint8* buffer, uint8 sendBytes);
// 定频率的发送数据到蓝牙主机上去
static void sendTask( void );

#if (HAL_UART==TRUE)  // 串口打印    
// 串口回调函数
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  uint8 i;
  
  simpleBLEPeripheral_TaskID = task_id;

#if (HAL_UART==TRUE)  // 串口打印      
  // 串口初始化
  NPI_InitTransport(simpleBLE_NpiSerialCallback);
  NPI_WriteTransport("SimpleBLEPeripheral_Init\r\n", 26);  
#endif
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000", 密码可以在程序里修改
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = {0}; 
    uint8 charValue7[SIMPLEPROFILE_CHAR7_LEN] = {0}; 
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, charValue7 );
  }

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
 // HalLcdWriteString( "      ShowWGuard", HAL_LCD_LINE_1 );  //***********************************
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

    /* 设置系统时间 */
  setUTCTime( 5, 0, 0, 24, 10, 2020 );
  
  /* 得到 mac 地址 */
  GAPRole_GetParameter( GAPROLE_CONN_BD_ADDR, macAddr );

//10  标志SBP_START_DEVICE_EVT启动该event
  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  //HalLcdWriteString( "   System Init...", HAL_LCD_LINE_8 );
  HalI2CInit( i2cClock_123KHZ );
  PowerRN5T618_init();
  HalAdcInit();
  HalAdcSetReference( HAL_ADC_REF_AVDD );
  //读几次ADC稳定
  HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
  HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
  HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
  HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
  HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
  GPIO_Init(0,6);
 // Uart_Init();
  Motor_OFF();
  maxim_max30102_reset();
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);
  maxim_max30102_init();
  getTempCali(&tmpCaliK,&tmpCaliB);
  //*****************heart***********************
  un_min=0x3FFFF;
  un_max=0;
  n_ir_buffer_length = 150;
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(max30102_INTPin == 1);
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
    if(un_min>aun_red_buffer[i])
      un_min=aun_red_buffer[i];    //update signal min
    if(un_max<aun_red_buffer[i])
      un_max=aun_red_buffer[i];    //update signal max
    
  }
  un_prev_data = aun_red_buffer[i];
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  
  HalLcdClear();
  HalLcdDisOff();
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    // 再开一个定时器用来发送决定发送数据的频率的
   // osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT2, SBP_PERIODIC_EVT_PERIOD2 );
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT ){
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD ){
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }
    // Perform periodic application task
    performPeriodicTask();
    return (events ^ SBP_PERIODIC_EVT);
  }
 
  if ( events & SBP_PERIODIC_EVT2 ){
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD2 ){
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT2, SBP_PERIODIC_EVT_PERIOD2 );
    }
    // Perform periodic application task
    sendTask();
    return (events ^ SBP_PERIODIC_EVT2);
  }

#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
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
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;  // Intentionally unreferenced parameter
  
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED: {
        state = GAPROLE_STARTED;
      
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        LCD_NeedRun = 1;
          // Display device address
          //HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( " Initialized",  HAL_LCD_LINE_3 );
      }
      break;

    case GAPROLE_ADVERTISING: {
        state = GAPROLE_ADVERTISING;
        
         LCD_NeedRun = 1;
          HalLcdWriteString( " Advertising",  HAL_LCD_LINE_3 );
      }
      break;

    case GAPROLE_CONNECTED: {
        state = GAPROLE_CONNECTED;
         LCD_NeedRun = 1;
          HalLcdWriteString( " Connected",  HAL_LCD_LINE_3 );
      }
      break;

    case GAPROLE_WAITING: {
        state = GAPROLE_WAITING;
      
          LCD_NeedRun = 1;
          HalLcdWriteString( " Disconnected",  HAL_LCD_LINE_3 );
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT: {
        state = GAPROLE_WAITING_AFTER_TIMEOUT;
      
        LCD_NeedRun = 1;
          HalLcdWriteString( " TimedOut",  HAL_LCD_LINE_3 );
      }
      break;

    case GAPROLE_ERROR: {
        state = GAPROLE_ERROR;
      
        LCD_NeedRun = 1;
          HalLcdWriteString( " Error",  HAL_LCD_LINE_3 );
      }
      break;

    default:
      {
        if( LCD_NeedRun ) {
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        }
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

float f_temp;
void heart_get_task()
{
  
  if(heart_flag  == 1)
  {
    if(heart_cnt < 100)
    {
      aun_red_buffer[heart_cnt+50 - 50] = aun_red_buffer[heart_cnt+50];
      aun_ir_buffer[heart_cnt+50 - 50] = aun_ir_buffer[heart_cnt+50];

      //update the signal min and max
      if(un_min > aun_red_buffer[heart_cnt+50])
          un_min = aun_red_buffer[heart_cnt+50];
      if(un_max < aun_red_buffer[heart_cnt+50])
          un_max = aun_red_buffer[heart_cnt+50];
      
      heart_cnt ++;
    }
    else if((heart_cnt >= 100)&&(heart_cnt < 150))
    {
      un_prev_data = aun_red_buffer[heart_cnt - 1];
      while(max30102_INTPin == 1)
      //{
        maxim_max30102_read_fifo((aun_red_buffer + heart_cnt ), (aun_ir_buffer + heart_cnt ));

      //calculate the brightness of the LED
        if(aun_red_buffer[heart_cnt + 100] > un_prev_data)
        {
            f_temp = aun_red_buffer[heart_cnt + 100] - un_prev_data;
            f_temp /= (un_max - un_min);
            f_temp *= 255;
            f_temp = un_brightness - f_temp;
            if(f_temp < 0)
                un_brightness = 0;
            else
                un_brightness = (int)f_temp;
        }
        else
        {
            f_temp = un_prev_data - aun_red_buffer[heart_cnt + 100];
            f_temp /= (un_max - un_min);
            f_temp *= 255;
            un_brightness += (int)f_temp;
            if(un_brightness > 255)
                un_brightness = 255;
        }
        
        heart_cnt ++;
     // }
       
    }
    
    if(heart_cnt >= 150)
    {
      heart_flag = 2;
      osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT2);
    }
  }
  
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
uint8 timerCount2 = 0;
uint8 timerCount3 = 0;
bool dis_change_flag = 0;;
static void performPeriodicTask( void )
{
  uint8 tempbuf[12] = {0};
  char displayBuf[20];
  float tmpf;
  uint16 CurrentTemp;
  uint8 num_pos;
  //uint32 temp32;
  uint16_t BatteryCap;
  UTCTimeStruct Ti;

  osalTimeUpdate(  );
  osal_ConvertUTCTime(&Ti,osal_getClock());
  HalLcdWriteStringValue("      heart_cnt",heart_cnt,10,HAL_LCD_LINE_8);
  if(KEY_NeedPrcoess == 1)
  {
   // KEY_NeedPrcoess = 0;
    switch(DIS_Item)
    {
      case 0:
      case 1:
        {
         // HalLcdClear();
         // HalLcdDisOn();
       //   timerCount2 = 0;
          tmpf = GetTemperature1();
          
          sprintf(displayBuf,"%.1f`C",tmpf);
          HalLcdShowString(18,3,displayBuf,32);
          tmpf = GetTemperature();
          sprintf(displayBuf,"%.1f`C",tmpf);
          HalLcdWriteString(displayBuf,HAL_LCD_LINE_8);
          
          if(state == GAPROLE_CONNECTED)
          {
            HalLcdDrawBt(70,0,1);
          }
          else 
          {
            HalLcdDrawBt(70,0,0);
          }
          
          if(tempDevice == 1)
          {
            sprintf(displayBuf,"%02d:%02d n",Ti.hour,Ti.minutes);
          }
          else 
          {
            sprintf(displayBuf,"%02d:%02d m",Ti.hour,Ti.minutes);
          }

          HalLcdShowString(10,0,displayBuf,16);
          if(Get_BatChargeState())
          {
            HalLcdDrawBattery(90,0,0xff);
          }
          else
          {
            BatteryCap = Get_BatCap() ;
            HalLcdDrawBattery(90,0,BatteryCap);
          }
        }break;
      case 2:
        {
         // HalLcdClear();
         // HalLcdDisOn();
         // timerCount2 = 0;
          if(Get_BatChargeState())
          {
            HalLcdDrawBattery(90,0,0xff);
          }
          else
          {
            BatteryCap = Get_BatCap() ;
            HalLcdDrawBattery(90,0,BatteryCap);
          }
          sprintf(displayBuf,"%02d:%02d:%02d",Ti.hour,Ti.minutes,Ti.seconds);
          HalLcdShowString(0,3,displayBuf,32);
          sprintf(displayBuf,"%04d-%02d-%02d",Ti.year,Ti.month,Ti.day);
          HalLcdShowString(5,0,displayBuf,16);
        }break;
      case 3:
        {
          //heart_flag;
          //heart_cnt;
          
          if((heart_flag == 1)&&(heart_cnt==0))
          {
            un_min = 0x3FFFF;
            un_max = 0;
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT2, SBP_PERIODIC_EVT_PERIOD2 );
          }
          if(heart_flag == 0)
          {
            timerCount3 = 0;
            sprintf(displayBuf,"bmp");
            HalLcdShowString(85,5,displayBuf,16);
            sprintf(displayBuf,"%02d",n_heart_rate);
            HalLcdShowString(30,3,displayBuf,32);
            if(state == GAPROLE_CONNECTED)
            {
              HalLcdDrawBt(70,0,1);
            }
            else 
            {
              HalLcdDrawBt(70,0,0);
            }
            
            sprintf(displayBuf,"%02d:%02d",Ti.hour,Ti.minutes);
            HalLcdShowString(10,0,displayBuf,16);
            if(Get_BatChargeState())
            {
              HalLcdDrawBattery(90,0,0xff);
            }
            else
            {
              BatteryCap = Get_BatCap() ;
              HalLcdDrawBattery(90,0,BatteryCap);
            }
          }
          else if(heart_flag == 1)
          {
            sprintf(displayBuf,"bmp");
            HalLcdShowString(85,5,displayBuf,16);
            //dis_change_flag ~= dis_change_flag;
            if(dis_change_flag)
            {
              sprintf(displayBuf,"--");
              HalLcdShowString(30,3,displayBuf,32);
            }
            else
            {
              sprintf(displayBuf,"--");
              HalLcdShowString(30,3,displayBuf,32);
            }
            
            if(state == GAPROLE_CONNECTED)
            {
              HalLcdDrawBt(70,0,1);
            }
            else 
            {
              HalLcdDrawBt(70,0,0);
            }
            
            sprintf(displayBuf,"%02d:%02d",Ti.hour,Ti.minutes);
            HalLcdShowString(10,0,displayBuf,16);
            if(Get_BatChargeState())
            {
              HalLcdDrawBattery(90,0,0xff);
            }
            else
            {
              BatteryCap = Get_BatCap() ;
              HalLcdDrawBattery(90,0,BatteryCap);
            }
          }
          else if(heart_flag == 2)
          {
            maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
            
            HalLcdClear();
            HalLcdDisOn();
          //  n_heart_rate -= 25;
            sprintf(displayBuf,"%02d",n_heart_rate);
            HalLcdShowString(30,3,displayBuf,32);
            
            heart_flag = 0;
            heart_cnt = 0;
          }
          
          //DIS_Item = 0;
        }break;
      case 4:
        {
          timerCount3 ++;
          //sprintf(displayBuf,"--");
          //HalLcdShowString(40,3,displayBuf,32);
        }break;
    //  case 5:timerCount3 ++;
      case 5:  
        {
          if(timerCount3 < 2)
          {
            DIS_Item = 3;
            if(heart_flag == 0)
            {
              heart_flag = 1;
              heart_cnt = 0;
            }
           // sprintf(displayBuf,"--");
           // HalLcdShowString(40,3,displayBuf,32);
          }
          else 
          {
           // timerCount3 = 0;
            heart_flag = 0;
            heart_cnt = 0;
           // DIS_Item = 0;
          }
        }
        break;
    }
  }
    
  timerCount ++;
  timerCount2 ++;
 /*  tempbuf[0] = 0x20;
  tempbuf[1] = (Ti.year) / 1000 + '0';
  tempbuf[2] = Ti.year / 100 % 10 + '0';
  tempbuf[3] = Ti.year / 10 % 10 + '0';
  tempbuf[4] = Ti.year % 10 + '0';
  tempbuf[5] = '.';
  tempbuf[6] = Ti.month / 10 + '0';
  tempbuf[7] = Ti.month % 10 + '0';
  tempbuf[8] = '.';
  tempbuf[9] = Ti.day / 10 + '0';
  tempbuf[10] = Ti.day % 10 + '0';
  tempbuf[11] = 0x20;
  tempbuf[12]=(Ti.hour)/10+'0';  
  tempbuf[13]=(Ti.hour)%10+'0';
  tempbuf[14] = ':';
  tempbuf[15]=(Ti.minutes)/10+'0';  
  tempbuf[16]=(Ti.minutes)%10+'0';
  tempbuf[17] = ':';
  tempbuf[18]=(Ti.seconds)/10+'0';  
  tempbuf[19]=(Ti.seconds)%10+'0'; */
  
  if( timerCount > 900 ){ // 15分钟的定时
  //  timerUserFunc(); // 执行用户自定义的函数
    if((Ti.minutes <= 15) && (Ti.minutes > 0))
      num_pos = 1;
    else if((Ti.minutes <= 30) && (Ti.minutes > 15))
      num_pos = 2;
    else if((Ti.minutes <= 45) && (Ti.minutes > 30))
      num_pos = 3;
    else if((Ti.minutes <= 60) && (Ti.minutes > 45))
      num_pos = 4;
    else 
      num_pos = 1;
    num_pos = num_pos + (Ti.hour*4);
    tmpf = GetTemperature1();
    CurrentTemp = tmpf * 10;
    saveTempPoint(Ti.year,Ti.month,Ti.day,num_pos,CurrentTemp);
    LCD_NeedRun = 1;
   // HalLcdWriteString( "Alarm Clock", HAL_LCD_LINE_7 );
    timerCount = 0;
  }
  
  if( timerCount2 > 20  ) {
    timerCount2 = 0;
    LCD_NeedRun = 0;
    KEY_NeedPrcoess = 0;
    HalLcdDisOff();
  }
  
  if( LCD_NeedRun ){   
    switch(state){
      case GAPROLE_INIT: HalLcdWriteString( " Initialized", HAL_LCD_LINE_3 );break;
      case GAPROLE_STARTED: HalLcdWriteString( " Started", HAL_LCD_LINE_3 );break;
      case GAPROLE_ADVERTISING:HalLcdWriteString( " Advertising", HAL_LCD_LINE_3 );break;
      case GAPROLE_WAITING:HalLcdWriteString( " Disconnected",  HAL_LCD_LINE_3 );break;
      case GAPROLE_WAITING_AFTER_TIMEOUT:HalLcdWriteString( " TimeOut", HAL_LCD_LINE_3 );break;
      case GAPROLE_CONNECTED:HalLcdWriteString( " Connected", HAL_LCD_LINE_3 );break;
      case GAPROLE_ERROR:HalLcdWriteString( " Error ...", HAL_LCD_LINE_3 );break;
      default:break;
    }
  }
}

/* */


void BLESendCurrentTemp(void)
{
  uint16_t crc;
  uint16_t tmp16;
  uint8_t bleSendBuf[20];

  tmp16 = GetTemperature1()*100;
  if((tmp16%10)>=5)
  {
    tmp16 = tmp16/10 + 1;
  }
  else 
  {
    tmp16 = tmp16/10;
  }
  bleSendBuf[0] = 0x03;
  bleSendBuf[1] = 0x10;
  bleSendBuf[2] = macAddr[0];
  bleSendBuf[3] = macAddr[1];
  bleSendBuf[4] = macAddr[2];
  bleSendBuf[5] = macAddr[3];
  bleSendBuf[6] = macAddr[4];
  bleSendBuf[7] = macAddr[5];
  bleSendBuf[8] = 0;
  bleSendBuf[9] = 0;//CurrentPoint;
  bleSendBuf[10] = 1;
  bleSendBuf[11] = tmp16 >> 8;
  bleSendBuf[12] = tmp16 & 0xff;

  crc = CRC16(bleSendBuf,13);

  bleSendBuf[13] = crc >> 8;
  bleSendBuf[14] = crc & 0xff;

  simpleBLE_SendData(nGUA_ConnHandle, bleSendBuf, 15);
}
void BLESendReply(uint8_t date,uint8_t point,uint8_t num)
{
  uint16_t crc;
  uint16_t data;
  UTCTimeStruct Ti;
  osalTimeUpdate(  );
  osal_ConvertUTCTime(&Ti,osal_getClock());
  
  uint8_t bleSendBuf[20];
  if((point == 0)&&(num == 0))
  {
      BLESendCurrentTemp();
      return;
  }
  data = readTempPoint(Ti.year,Ti.month,Ti.day,point);
  bleSendBuf[0] = 0x03;
  bleSendBuf[1] = 0x10;
  bleSendBuf[2] = macAddr[0];
  bleSendBuf[3] = macAddr[1];
  bleSendBuf[4] = macAddr[2];
  bleSendBuf[5] = macAddr[3];
  bleSendBuf[6] = macAddr[4];
  bleSendBuf[7] = macAddr[5];
  bleSendBuf[8] = 0;
  bleSendBuf[9] = point;
  bleSendBuf[10] = 1;
  bleSendBuf[11] = data >> 8;
  bleSendBuf[12] = data & 0xff;
  
  crc = CRC16(bleSendBuf,13);
  
  bleSendBuf[13] = crc >> 8;
  bleSendBuf[14] = crc & 0xff;
  simpleBLE_SendData(nGUA_ConnHandle, bleSendBuf, 15);
}

void BLESendRece(uint8_t cmd,uint8_t status)
{
    uint16_t crc;
    uint8_t bleSendBuf[20];
    bleSendBuf[0] = 0x03;
    bleSendBuf[1] = cmd;
    bleSendBuf[2] = macAddr[0];
    bleSendBuf[3] = macAddr[1];
    bleSendBuf[4] = macAddr[2];
    bleSendBuf[5] = macAddr[3];
    bleSendBuf[6] = macAddr[4];
    bleSendBuf[7] = macAddr[5];
    
    crc = CRC16(bleSendBuf,8);
    
    bleSendBuf[8] = crc >> 8;
    bleSendBuf[9] = crc & 0xff;
    
    simpleBLE_SendData(nGUA_ConnHandle,bleSendBuf,10);
}

void BLESendToCard(void)
{
    uint8_t i;
    uint16_t crc;
    uint16_t data;
    uint8_t bleSendBuf[205];
    UTCTimeStruct Ti;
    osalTimeUpdate(  );
    
    bleSendBuf[0] = 0x03;
    bleSendBuf[1] = 0x40;//0x11;
    bleSendBuf[2] = macAddr[0];
    bleSendBuf[3] = macAddr[1];
    bleSendBuf[4] = macAddr[2];
    bleSendBuf[5] = macAddr[3];
    bleSendBuf[6] = macAddr[4];
    bleSendBuf[7] = macAddr[5];
    bleSendBuf[8] = 0;
    bleSendBuf[9] = 1;
    bleSendBuf[10] = 96;
    
    for(i=0;i<96;i++)
    {
        data = readTempPoint(Ti.year,Ti.month,Ti.day,i+1);
        bleSendBuf[11 + i*2] = data >> 8;
        bleSendBuf[12 + i*2] = data & 0xff;
    }
    
    crc = CRC16(bleSendBuf,11 + 96*2);
    
    bleSendBuf[11 + 96*2] = crc >> 8;
    bleSendBuf[12 + 96*2] = crc & 0xff;
    
    
    // simpleBLE_SendData(nGUA_ConnHandle,bleSendBuf,13 + 96*2); 
   
    for(i=0;i<10;i++)
      simpleBLE_SendData(nGUA_ConnHandle,bleSendBuf+20*i,20);  
    simpleBLE_SendData(nGUA_ConnHandle,bleSendBuf+200,5);  
}

static void sendTask( void )
{  
  /* test of adc */
 /* uint16 value;
  HalAdcInit();
   
  HalAdcSetReference( HAL_ADC_REF_125V );
  
  value = HalAdcRead( HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12 );
    
  HalLcdWriteStringValue( " value2:", value, 10, HAL_LCD_LINE_8 );*/
 // BLESendCurrentTemp();
  heart_get_task();
  
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint8 newValueBuf[20]={0};
  uint8 returnBytes;
  
  switch( paramID ) {
    case SIMPLEPROFILE_CHAR1:{
      //SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValueBuf, &returnBytes );
      if( returnBytes > 0 ) {
        LCD_NeedRun = 1;
        HalLcdWriteString(" CHAR6 received: ", HAL_LCD_LINE_6 );
        HalLcdWriteString((char*)newValueBuf, HAL_LCD_LINE_7 );
      }
      break;
    }

    case SIMPLEPROFILE_CHAR3: {
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue, &returnBytes );
      HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_7 );
      break;
    }

    // 添加的代码， 表示收到主机对 CHAR6 特征值发送过来的数据了
    case SIMPLEPROFILE_CHAR6:{
      uint8 newChar6Value[SIMPLEPROFILE_CHAR6_LEN];
      uint16_t crc;
      uint8_t buf_D[4];
      uint32_t tmp32;
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &nGUA_ConnHandle); 
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, newChar6Value, &returnBytes );
      
      if(returnBytes > 0) {
        LCD_NeedRun = 1;
        HalLcdDisOn();
        HalLcdWriteString(" Received: ", HAL_LCD_LINE_8 );
        //HalLcdWriteString((char*)newChar6Value, HAL_LCD_LINE_7 );
     //   BLESendCurrentTemp();
        crc = CRC16(newChar6Value,returnBytes);
        if(crc != 0)
            ;//return;
        if(newChar6Value[0] == 0x01)
        {
          if((newChar6Value[1] == 0x10)&&(returnBytes > 10))
          {
              BLESendReply(newChar6Value[2],newChar6Value[3],newChar6Value[4]);
              setUTCTime(newChar6Value[11],newChar6Value[10],newChar6Value[9],newChar6Value[8],newChar6Value[7],(((uint16_t)newChar6Value[5]<<8) + newChar6Value[6]));
          }
          else if((newChar6Value[1] == 0x01)&&(returnBytes>13))
          {
              buf_D[0] = *(newChar6Value+5);
              buf_D[1] = *(newChar6Value+4);
              buf_D[2] = *(newChar6Value+3);
              buf_D[3] = *(newChar6Value+2);
              tmp32 = *(int32_t*)buf_D;

              tmpCaliK = (float)tmp32/10000;

              buf_D[0] = *(newChar6Value+9);
              buf_D[1] = *(newChar6Value+8);
              buf_D[2] = *(newChar6Value+7);
              buf_D[3] = *(newChar6Value+6);
              tmp32 = *(int32_t*)buf_D;

              tmpCaliB = (float)tmp32/10000;

              saveTempCali(tmpCaliK,tmpCaliB);
              BLESendRece(newChar6Value[1],0);
          }
          else if(newChar6Value[1] == 0x40)
          {
            BLESendToCard();
          }
        }

        
      }
    }  
    break;
    
    case SIMPLEPROFILE_CHAR7:{    
      uint8 nbGUA_Char7[20] = {0};  
      //获取连接句柄
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &nGUA_ConnHandle); 
      //读取char7的数值
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR7, nbGUA_Char7, &returnBytes);
     // if(returnBytes > 0) {
      //  LCD_NeedRun = 1;
      //  HalLcdWriteString(" Received: ", HAL_LCD_LINE_6 );
      //  HalLcdWriteString((char*)nbGUA_Char7, HAL_LCD_LINE_7 );
      //发送数据  
      //simpleBLE_SendData(nGUA_ConnHandle, nbGUA_Char7, 20);
    //  }
    }  
    break;

    default: break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)


#if (HAL_UART==TRUE)  // 串口打印      
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port;
      
    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))   //串口有数据
    {
        uint8 numBytes = 0;

        numBytes = NPI_RxBufLen();           //读出串口缓冲区有多少字节
        
        if(numBytes > 0)
        {
            uint8 *buffer = osal_mem_alloc(numBytes);            
            if(buffer)
            {
                NPI_ReadTransport(buffer,numBytes);  
                
                NPI_WriteTransport(buffer,numBytes);  
                    
                osal_mem_free(buffer);
            }
        }
    }
}
#endif
/*********************************************************************
*********************************************************************/
