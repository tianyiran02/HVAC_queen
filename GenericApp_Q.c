/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful - it is
  intended to be a simple example of an application's structure.

  This application periodically sends a "Hello World" message to
  another "Generic" application (see 'txMsgDelay'). The application
  will also receive "Hello World" packets.

  This application doesn't have a profile, so it handles everything
  directly - by itself.

  Key control:
    SW1:  changes the delay between TX packets
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp_Q.h"
#include "DebugTrace.h"

#include "math.h"
#include "string.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_led.h"
#include "hal_uart.h"

/* MT */
#include "MT_UART.h"
#include "MT.h"

/* NWK */
#include "AddrMgr.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

/*********************************************************************
 * MACROS
 */

// For every different configure
// Maximun drone number
#define MAXDEVNUM 3 
#define QUEEN_ID "2F691110"

#define Transducer0Offset   0.00
#define Transducer1Offset   0.00
      
// CMD with drones
#define DRONE_RESEND_INIT       0x0d
#define DRONE_DATA_ACK          0x0f
#define DRONE_SEND_REQ          0x0a

#define NOT_READY       3
#define AVAILABLE       1
#define UNAVAILABLE     0

// WCDMA related
#define WCDMA_OVERTIME 7 // overtime second * 2 -> 14s
// Note this must not be greater than 60(ipopen response maximun)
#define WCDMA_OVERTIMERESET     2       // overtime 2 times, 3G reset
#define WCDMA_RESETTIMER        4       // Reset 3G Module every 2 hours, largest value, 18hours
#define WCDMA_RESTARTTIMER      80      // if not restart in 120s, resend reset cmd

#define CCLKCMDReSend_Sec 5 // CCLK Send Delay second

#define WCDMAModule_IPINITResend_MAX 15

#define WCDMAModule_NotAvailable  0
#define WCDMAModule_Available     1
#define WCDMAModule_Ready         2

#define WCDMAsetup_NotReady     0
#define WCDMAsetup_3GConnected  1
#define WCDMAsetup_IPINITsend   2
#define WCDMAsetup_CCLKsend     3
#define WCDMAsetup_CNMIsend     4
#define WCDMAsetup_IPOPENsend   5
#define WCDMAsetup_IPENTRANSsend        6
#define WCDMAsetup_IPENTRANSEnd         7
#define WCDMAsetup_RESTART      8

#define WCDMA_DATA_DEVICEID       68
#define WCDMA_DATA_DEVICEID_L     10 //2,4+6

#define WCDMA_DATA_STATUS         98 //90,92+6
#define WCDMA_DATA_STATUS_L       1

#define WCDMA_DATA_HPRESS         126 //118,120+6
#define WCDMA_DATA_HPRESS_L       4

#define WCDMA_DATA_LPRESS         156 //149,151+6-1
#define WCDMA_DATA_LPRESS_L       4

#define WCDMA_DATA_CURRENT        181 //175,177+6-2
#define WCDMA_DATA_CURRENT_L      4

#define WCDMA_DATA_TEMP           210 //204,206+6-2
#define WCDMA_DATA_TEMP_L         4

#define WCDMA_DATA_TIME           237 //232,234+6-3
#define WCDMA_DATA_TIME_L         16

#define Ringbuffer_Length         88    

/*********************************************************************
 * CONSTANTS
 */

/* HUAWEI AT Command */
// HUAWEI AT
uint8 EM770_AT[4] = {0x41,0x54,0x0D,0x0A}; 

// HUAWEI ATE0
uint8 EM770_ATE0[6] = {0x41,0x54,0x45,0x30,0x0D,0x0A};

// HUAWEI CCLK: "AT+CCLK?"
uint8 EM770_CCLK[10] = {0x41,0x54,0x2B,0x43,0x43,0x4C,0x4B,0x3F,0x0D,0x0A};

/*IP INIT CMD
 *
 * Select different operator by MACRO
 *
 * 1. UK lycamobile version: define LYCAMOBILE
 * 2. China UNICOM version: define CHINAUNICOM
 * 3. USA version: define USAMOBILE
 *
 */

#ifdef LYCAMOBILE
// HUAWEI Init data: "AT%IPINIT="data.lycamobile.co.uk","lmuk","plus""
uint8 EM770_IPINIT[49] = {0x41,0x54,0x25,0x49,0x50,0x49,0x4e,0x49,0x54,0x3d,
                          0x22,0x64,0x61,0x74,0x61,0x2e,0x6c,0x79,0x63,0x61,
                          0x6d,0x6f,0x62,0x69,0x6c,0x65,0x2e,0x63,0x6f,0x2e,
                          0x75,0x6b,0x22,0x2c,0x22,0x6c,0x6d,0x75,0x6b,0x22,
                          0x2c,0x22,0x70,0x6c,0x75,0x73,0x22,0x0d,0x0a};
#endif

#ifdef CHINAUNICOM
// HUAWEI Init data: "AT%IPINIT="3GNET""
uint8 EM770_IPINIT[19] = {0x41,0x54,0x25,0x49,0x50,0x49,0x4E,0x49,0x54,0x3D,
                          0x22,0x33,0x47,0x4E,0x45,0x54,0x22,0x0D,0x0A};
#endif

#ifdef USAMOBILE
// HUAWEI Init data:"AT%IPINIT="epc.tmobile.com""
uint8 EM770_IPINIT[29] = {0x41,0x54,0x25,0x49,0x50,0x49,0x4e,0x49,0x54,0x3d,
                          0x22,0x65,0x70,0x63,0x2e,0x74,0x6d,0x6f,0x62,0x69,
                          0x6c,0x65,0x2e,0x63,0x6f,0x6d,0x22,0x0d,0x0a};
#endif

#ifdef USAMOBILE_WYLESS
// HUAWEI Init data:"AT%IPINIT="telargo.t-mobile.com""
uint8 EM770_IPINIT[34] = {0x41,0x54,0x25,0x49,0x50,0x49,0x4E,0x49,0x54,0x3D,
                          0x22,0x74,0x65,0x6C,0x61,0x72,0x67,0x6F,0x2E,0x74,
                          0x2D,0x6D,0x6F,0x62,0x69,0x6C,0x65,0x2E,0x63,0x6F,
                          0x6D,0x22,0x0D,0x0A};
#endif

// HUAWEI IPOPEN: "AT%IPOPEN=1,"TCP","70.39.150.200",80"
uint8 EM770_IPOPEN[38] = {0x41,0x54,0x25,0x49,0x50,0x4F,0x50,0x45,0x4E,0x3D,
                          0x31,0x2C,0x22,0x54,0x43,0x50,0x22,0x2C,0x22,0x37,
                          0x30,0x2E,0x33,0x39,0x2E,0x31,0x35,0x30,0x2E,0x32,
                          0x30,0x30,0x22,0x2C,0x38,0x30,0x0D,0x0A};

// HUAWEI IPCLOSE: "AT%IPCLOSE=1"
uint8 EM770_IPCLOSE[14] = {0x41,0x54,0x25,0x49,0x50,0x43,0x4C,0x4F,0x53,
                           0x45,0x3D,0x31,0x0D,0x0A};

// HUAWEI IPENTRANS: "AT%IPENTRANS"
uint8 EM770_IPENTRANS[14] = {0x41,0x54,0x25,0x49,0x50,0x45,0x4E,0x54,0x52,
                              0x41,0x4E,0x53,0x0D,0x0A};

// HUAWEI RESET: "AT%RESET=2"
uint8 EM770_RESET[12] = {0x41,0x54,0x25,0x52,0x45,0x53,0x45,0x54,0x3D,0x32,
                         0x0D,0x0A};

// HUAWEI CNMI: "AT+CNMI=0" close SMS upload
uint8 EM770_CNMI[11] = {0x41,0x54,0x2B,0x43,0x4E,0x4D,0x49,0x3D,0x30,0x0D,0x0A};


// HUAWEI IPENTRANS STOP: 0x11
uint8 EM770_IPENTRANS_STOP = 0x11;

// HUAWEI AT ACK 
const char EM770_ATE0_ACK[12] = {0x41,0x54,0x45,0x30,0x0D,0x0D,0x0A,0x4F,
                           0x4B,0x0D,0x0A,0x00}; // ATE0   OK

const char EM770_READY_MSG[17] = {0x0D,0x0A,0x45,0x4D,0x37,0x37,0x30,0x57,0x20,
                            0x52,0x45,0x41,0x44,0x59,0x0D,0x0A,0x00}; //EM770W READY

const char EM770_ACK[7] = {0x0D,0x0A,0x4F,0x4B,0x0D,0x0A,0x00}; //OK

const char EM770_ERROR_ACK[10] = {0x0D,0x0A,0x45,0x52,0x52,0x4F,0x52,0x0D,0x0A,0x00};//ERROR

const char EM770_CCLK_ACK[10] = {0x0D,0x0A,0x2B,0x43,0x43,0x4C,
                                 0x4B,0x3A,0x20,0x00}; // +CCLK: 

const char EM770_ENTRANS_ACK[5] = {0x0D,0x0A,0x3E,0x20,0x00}; // >

const char EM770_ENDTRANS_ACK[9] = {0x0D,0x0A,0x0D,0x0A,0x4F,0x4B,0x0D,0x0A,0x00}; // OK

const char EM770_IPSEND_ACK[21] = {0x0D,0x0A,0x25,0x49,0x50,0x53,0x45,0x4E,0x44,
                             0x3A,0x20,0x31,0x0D,0x0A,0x0D,0x0A,0x4F,0x4B,
                             0x0D,0x0A,0x00}; // %IPSEND: 1   OK

// Get command 329+2+6-1-1-1=334
uint8 EM770_Sending[334] = {0x47,0x45,0x54,0x20,0x68,0x74,0x74,0x70,0x3A,0x2F,
0x2F,0x77,0x77,0x77,0x2E,0x61,0x63,0x66,0x75,0x6C,
0x6C,0x65,0x72,0x77,0x69,0x72,0x65,0x6C,0x65,0x73,
0x73,0x2E,0x6E,0x65,0x74,0x2F,0x72,0x65,0x70,0x6F,
0x72,0x74,0x2E,0x70,0x68,0x70,0x2F,0x3F,0x72,0x65,
0x70,0x6F,0x72,0x74,0x3D,0x7B,0x25,0x32,0x32,0x69,
0x64,0x25,0x32,0x32,0x3A,0x25,0x32,0x32,0x31,0x32,
0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x30,
0x25,0x32,0x32,0x2C,0x25,0x32,0x32,0x73,0x74,0x61,
0x74,0x75,0x73,0x25,0x32,0x32,0x3A,0x25,0x32,0x32,
0x31,0x25,0x32,0x32,0x2C,0x25,0x32,0x32,0x68,0x69,
0x67,0x68,0x5F,0x70,0x72,0x65,0x73,0x73,0x75,0x72,
0x65,0x25,0x32,0x32,0x3A,0x25,0x32,0x32,0x30,0x30,
0x30,0x30,0x25,0x32,0x32,0x2C,0x25,0x32,0x32,
0x6C,0x6F,0x77,0x5F,0x70,0x72,0x65,0x73,0x73,0x75,
0x72,0x65,0x25,0x32,0x32,0x3A,0x25,0x32,0x32,0x30,
0x30,0x30,0x30,0x25,0x32,0x32,0x2C,0x25,0x32,
0x32,0x63,0x75,0x72,0x72,0x65,0x6E,0x74,0x25,0x32,
0x32,0x3A,0x25,0x32,0x32,0x30,0x30,0x30,0x30,0x25,
0x32,0x32,0x2C,0x25,0x32,0x32,0x74,0x65,0x6D,0x70,
0x65,0x72,0x61,0x74,0x75,0x72,0x65,0x25,0x32,0x32,
0x3A,0x25,0x32,0x32,0x30,0x30,0x30,0x30,0x25,
0x32,0x32,0x2C,0x25,0x32,0x32,0x74,0x69,0x6D,0x65,
0x73,0x74,0x61,0x6D,0x70,0x25,0x32,0x32,0x3A,0x25,
0x32,0x32,0x30,0x30,0x30,0x30,0x2F,0x30,0x30,0x2F,
0x30,0x30,0x2C,0x30,0x30,0x3A,0x30,0x30,0x25,0x32,
0x32,0x2C,0x25,0x32,0x32,0x65,0x72,0x72,0x6F,0x72,
0x25,0x32,0x32,0x3A,0x25,0x32,0x32,0x25,0x32,0x32,
0x7D,0x20,0x48,0x54,0x54,0x50,0x2F,0x31,0x2E,0x31,
0x0D,0x0A,0x48,0x6F,0x73,0x74,0x3A,0x77,0x77,0x77,
0x2E,0x61,0x63,0x66,0x75,0x6C,0x6C,0x65,0x72,0x77,
0x69,0x72,0x65,0x6C,0x65,0x73,0x73,0x2E,0x6E,0x65,
0x74,0x0D,0x0A,0x41,0x63,0x63,0x65,0x70,0x74,0x3A,
0x20,0x2A,0x2F,0x2A,0x0D,0x0A,0x1A,0x0D,0x0A};

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8               *pIn;
  uint8               *pOut;
  uint8               counter;
} WCDMA_Ringbuffer_t; // Receive Ring buffer

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.

devStates_t GenericApp_NwkState;

byte GenericApp_TransID;  // This is the unique message ID (counter)

// Transplant from version 4 2014/11/03. ver5 bak1
// 
//
uint16 AddressManageBUF[MAXDEVNUM] = {0};  // Address Manage buffer

afAddrType_t queen_CMD_DstAddr;

/* WCDMA related */
uint8 WCDMAModuleSTATUS = 0; // 0 - not available, 1 - available, 2 - ready
uint8 WCDMAModuleSTEP = 0; // 0-x, indicate the different step in setup
uint8 WCDMAModule_SEND = 0; // Data need to send flag
uint8 WCDMAModule_IPINITResend_Count = 0; //Resend counter
uint8 WCDMAModule_IPINITResend = 0; // Resend flag and conter
uint8 WCDMAModule_SendOverTime = 0; // Send Counter, if Send more than WCDMA_OVERTIME s, sending abort
uint8 WCDMAModule_CCLKDelayCount = 0; // CCLK CMD delay counter
uint8 WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET;
uint16 WCDMAModule_ResetTimer = (WCDMA_RESETTIMER*60*60/2);
uint8 WCDMAModule_RestartTimer = FALSE;

// Time Stamp
uint8 JSON_TimeStamp[16] = {0}; // TimeStamp
uint8 JSON_TimeStamp_twos_counter = 0; // Two seconds counter
short JSON_TimeStamp_Upgrade_flag = 0;

// ring buffer
uint8 WCDMA_DATA_Ringbuffer[Ringbuffer_Length] = {0}; // Ring buffer
WCDMA_Ringbuffer_t WCDMA_Ringbuffer;  // Ring buffer structure
//
// end

// request-send cmd
short queen_Available = NOT_READY;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void queen_CMDReact(afIncomingMSGPacket_t *Msg);

// Transplant from version 4 2014/11/03. ver5 bak1
// 
//
static void queen_HandleUART( mtOSALSerialData_t *CMDMsg );
static void queen_PERIODICDATA_SERVICE( afIncomingMSGPacket_t *Msg );
static uint8 GetBufSample(void);
static uint8 PushBufSample(uint8);
static uint8 BufHasSample(void);
static void prepareTheSendingBUF(void);
static void queen_Reset3GModule( void );
static void myBlockingHalUARTWrite(uint8, uint8 *, uint8);
//
// end

#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
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
void GenericApp_Init( uint8 task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Initialize ring buffer
  WCDMA_Ringbuffer.pIn = WCDMA_DATA_Ringbuffer;
  WCDMA_Ringbuffer.pOut = WCDMA_DATA_Ringbuffer;
  WCDMA_Ringbuffer.counter = 0;
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  
  // off all status LED
  HalLedInit ();
  HalLedSet (HAL_LED_4, HAL_LED_MODE_ON); // LED4, D2 negative logic 
  HalLedSet (HAL_LED_3, HAL_LED_MODE_OFF); // LED3, D1 positive logic
  HalLedSet (HAL_LED_2, HAL_LED_MODE_ON); // LED2, D3 negative logic
  
  // Initialize UART
  MT_UartInit ();
  MT_UartRegisterTaskID(task_id);

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Start WDT reset timer
  osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_WDT_CLEAR_EVT,
                      GENERICAPP_WDT_CLEAR_TIMEOUT );
  
  // restart 3G module if it's already started
  queen_Reset3GModule();
  
  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif
  
#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt; 
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case CMD_SERIAL_MSG: // UART, communicate with Cellular module
          queen_HandleUART((mtOSALSerialData_t *)MSGpkt);
          break;
          
        case AF_INCOMING_MSG_CMD: // Zigbee data
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE: 
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD) ||
               (GenericApp_NwkState == DEV_ROUTER) ||
               (GenericApp_NwkState == DEV_END_DEVICE) )
          {

          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  /*
   * WDT Reload Timer Event
   *
   */
  if(events & GENERICAPP_WDT_CLEAR_EVT)
  {
    // clear timer     
    WDCTL |= WDCLP1; 
    WDCTL |= WDCLP2; 
    
    // reload timer 
    osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_WDT_CLEAR_EVT,
                      GENERICAPP_WDT_CLEAR_TIMEOUT );
    return (events ^ GENERICAPP_WDT_CLEAR_EVT);
  }

  /* 1s Delay Timer Event 
   *
   *  1. CCLK send Delay
   *  2. IPINIT send delay
   *  3. Send TCP/IP package to UART
   *  4. IPENTRANS end CMD
   *  5. 3G restart timer (avoid error)
   *
   */
  if(events & GENERICAPP_EM770W_WAIT_EVT)
  {
    
    if(WCDMAModuleSTATUS != WCDMAModule_Ready) // Send "AT" to push    
    {
      /* 1st function, CCLK send delay*/
      if(WCDMAModuleSTEP == WCDMAsetup_IPINITsend)
      {
        if(WCDMAModule_CCLKDelayCount <= CCLKCMDReSend_Sec)
        {
          // delay unfinish, set this timer again
          osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_EM770W_WAIT_EVT,
                      GENERICAPP_EM770W_WAIT_TIMEOUT );
          WCDMAModule_CCLKDelayCount ++;
        }
        else
        {
          // delay finish, send CCLK, no need to start timer
          myBlockingHalUARTWrite(0,EM770_CCLK,10);  // CCLK send delay
          WCDMAModuleSTEP = WCDMAsetup_CCLKsend;
        }
      }
      /* 2nd function, IPINIT delay */
      else if(WCDMAModuleSTEP == WCDMAsetup_3GConnected)
        myBlockingHalUARTWrite(0,EM770_AT,4); //Power on IPINIT Delay
    }
    
    /* 3nd function, send TCP/IP data package */
    else if(WCDMAModuleSTEP == WCDMAsetup_IPENTRANSsend)
    {
      myBlockingHalUARTWrite(0,EM770_Sending,100);
      myBlockingHalUARTWrite(0,&EM770_Sending[100],100);
      myBlockingHalUARTWrite(0,&EM770_Sending[200],100);
      myBlockingHalUARTWrite(0,&EM770_Sending[300],34);
      
      WCDMAModuleSTEP = WCDMAsetup_IPENTRANSEnd;
      
      // set this timer, need to send end CMD in 1s
      osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_EM770W_WAIT_EVT,
                      GENERICAPP_EM770W_WAIT_TIMEOUT );
    }
    
    /* 4th functions, send IPENTRANS end CMD */
    else if(WCDMAModuleSTEP == WCDMAsetup_IPENTRANSEnd) // Send end command 0x11
      myBlockingHalUARTWrite(0,&EM770_IPENTRANS_STOP,1);
      
    /* 5th functions, cellular module restart timer */
    if (WCDMAModule_RestartTimer)
    {
      WCDMAModule_RestartTimer --;
      
      if(WCDMAModule_RestartTimer <= 2)
      {
        queen_Reset3GModule();
      }
      else
      {
        osal_start_timerEx( GenericApp_TaskID,
                    GENERICAPP_EM770W_WAIT_EVT,
                    GENERICAPP_EM770W_WAIT_TIMEOUT ); 
      } 
    }
    
    return (events ^ GENERICAPP_EM770W_WAIT_EVT);
  }  


  /* 2 Seconds Timer
   *
   * 1. 3G Module Ready
   *  1. Update Time Stamp
   *  2. Chech whether need to restart 3G Module (every few hours)
   *  3. Check whether need to push AT (updata Time or send)
   *  4. Send overtime detect
   *
   * 2. Initialize Stag
   *  1. Initialize Stage: Reconnect the internet
   *
   **/
  if(events & GENERICAPP_TIMER_TWOSEC_EVT)
  { 
    
    // 1. 3G Module Ready
    if(WCDMAModuleSTATUS == WCDMAModule_Ready)
    {
      WCDMAModule_ResetTimer --; // auto reset 3G module every setting time
      
      /* 1. Operation to TimeStamp */
      JSON_TimeStamp_twos_counter ++;
      if(JSON_TimeStamp_twos_counter == 30)
      {
        JSON_TimeStamp_twos_counter = 0;
        
        JSON_TimeStamp[15] ++;
        if(JSON_TimeStamp[15] == 58)
        {
          JSON_TimeStamp[15] = 48;
  
          JSON_TimeStamp[14] ++;
          if(JSON_TimeStamp[14] == 54)
          {
            JSON_TimeStamp_Upgrade_flag = 1;
            
            /* If one hour, set back to 59, calibrate the time */
            JSON_TimeStamp[14] = 53;
            JSON_TimeStamp[15] = 57;
          }
        }
      }
      
      /* 2. Check whether need to reset 3G Module */
      if(!WCDMAModule_ResetTimer)
      {
        queen_Reset3GModule(); // reset 3G
        return (events ^ GENERICAPP_TIMER_TWOSEC_EVT); // finish this loop
      }
      
      /* 3. Check whether need to push AT */
      // If device not busy, and timstamp need update or has sample to send
      if((JSON_TimeStamp_Upgrade_flag || BufHasSample()) && (!WCDMAModule_SEND))
      {
        if(BufHasSample())
        {
          WCDMAModule_SEND = 1;
          WCDMAModule_SendOverTime = 1;
          prepareTheSendingBUF();
        }
        myBlockingHalUARTWrite(0,EM770_AT,4);
      }
      
      /* 4. Send overtime detect */
      if(WCDMAModule_SendOverTime)
      {
        WCDMAModule_SendOverTime ++;
        
        // Send more than WCDMA_OVERTIME s, sending abandon
        if(WCDMAModule_SendOverTime >= (WCDMA_OVERTIME + 1))
        {       
          WCDMAModule_SEND = 0;
          WCDMAModuleSTEP = WCDMAsetup_CNMIsend; // Jump back, wait for next try
          WCDMAModule_SendOverTime = 0; // Send finish, end timer
          
          // if overtime twice, reset 3G module
          WCDMAModule_ReConOverTries --;
          
          if(!WCDMAModule_ReConOverTries)
          {                       
            queen_Reset3GModule();
          }
        }
      }
    }
    
    // 2. Initialize Stage
    else if(WCDMAModule_IPINITResend)
    {
      WCDMAModule_IPINITResend ++;
      
      // Resend IPINIT every 10s
      if(WCDMAModule_IPINITResend>= 6)
      {
        WCDMAModule_IPINITResend = 1;
        WCDMAModule_IPINITResend_Count ++;
        
        if(WCDMAModule_IPINITResend_Count <= WCDMAModule_IPINITResend_MAX)
        {
#ifdef  LYCAMOBILE
          myBlockingHalUARTWrite(0,EM770_IPINIT,49);
#endif
        
#ifdef  CHINAUNICOM
          myBlockingHalUARTWrite(0,EM770_IPINIT,19);
#endif
        
#ifdef  USAMOBILE
          myBlockingHalUARTWrite(0,EM770_IPINIT,29);
#endif    
        
#ifdef  USAMOBILE_WYLESS
          myBlockingHalUARTWrite(0,EM770_IPINIT,34);
#endif
        
        }
        else // Skip process
        {
          WCDMAModule_IPINITResend_Count = 0;
          
          WCDMAModule_IPINITResend = 0;  
          HalLedSet (HAL_LED_4, HAL_LED_MODE_OFF);
          // 3G Signal Available, enable statu led4, d2
        
          osal_start_timerEx( GenericApp_TaskID,
                          GENERICAPP_EM770W_WAIT_EVT,
                          GENERICAPP_EM770W_WAIT_TIMEOUT );
        }
      }    
    }
      
    osal_start_timerEx( GenericApp_TaskID, GENERICAPP_TIMER_TWOSEC_EVT,
    GENERICAPP_TIMER_TWOSEC_TIMEOUT);
    
    return (events ^ GENERICAPP_TIMER_TWOSEC_EVT);
  }
  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case QUEEN_CMD_CLUSTERID: // Zigbee control 
      queen_CMDReact(pkt);
      break;
    
    case GENERICAPP_CLUSTERID: // Data service
      queen_PERIODICDATA_SERVICE(pkt);
      
#if defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;  
      
    default:
      break;
  }
}


/*********************************************************************
 * @fn      queen_CMDReact
 *
 * @brief   response and react to the CMD
 *          1. Initial msg
 *          2. Send Request
 *
 * @param   
 *
 * @return  none
 */
static void queen_CMDReact(afIncomingMSGPacket_t *Msg)
{
  uint8 i = 0;
  uint8 temp[15] = {0};
  uint8 Msgbuf[3] = {0x23,0x00,0x00};
  short bufincluded = 0;
  
  osal_memcpy(temp,&Msg->cmd.Data[0],3);
  
  /* First handle INIT information. If it is an Initilize information, then
   * it is not needed to check bufincluded. For all the other CMD then should
   * be processed by standard procedure.
   */
  
  /* Handle INIT MSG */
  if(temp[1] == DRONE_RESEND_INIT)
  {
    osal_memcpy(temp,Msg->cmd.Data + 3,8);
    
    // Setup for the command's destination address
    queen_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    queen_CMD_DstAddr.endPoint = GENERICAPP_ENDPOINT;
    queen_CMD_DstAddr.addr.shortAddr = Msg->srcAddr.addr.shortAddr;
    
    Msgbuf[0] = 0x23;
    Msgbuf[1] = DRONE_DATA_ACK;
    Msgbuf[2] = 0x00;
    
    // Send CMD
    if ( AF_DataRequest( &queen_CMD_DstAddr, &GenericApp_epDesc,
                         QUEEN_CMD_CLUSTERID,
                         3,
                         Msgbuf,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
      
    AddressManageBUF[Msg->cmd.Data[2] - 1] = Msg->srcAddr.addr.shortAddr;
    HalLedBlink( HAL_LED_3, 4, 50, 500 );
  }
  
  /* for all the other CMD except INIT Msg*/
  else
  {
    /* First decide msg is from which device */
    for(i = 0;i <= (MAXDEVNUM - 1);i++)
    {
      if(AddressManageBUF[i] == (Msg->srcAddr.addr.shortAddr))
      {
        i += (1 + 48);
        bufincluded = 1;
        break;
      }
    } 
    
    /* If device is already registed to network, then process CMD */
    if(bufincluded)
    {
      /* CMD type: DRONE_SEND_REQ */
      if(temp[1] == DRONE_SEND_REQ)
      {
        /* queen available, receive and send data */
        if(queen_Available == AVAILABLE)
        {
          // set flag unavailable
          queen_Available = UNAVAILABLE;
          
          Msgbuf[1] = DRONE_SEND_REQ;
          Msgbuf[2] = AVAILABLE;
          // Setup for the command's destination address
          queen_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
          queen_CMD_DstAddr.endPoint = GENERICAPP_ENDPOINT;
          queen_CMD_DstAddr.addr.shortAddr = Msg->srcAddr.addr.shortAddr;
        }
        
        /* queen not available, send unavailable msg */
        else
        {
          Msgbuf[1] = DRONE_SEND_REQ;
          Msgbuf[2] = UNAVAILABLE;
          // Setup for the command's destination address
          queen_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
          queen_CMD_DstAddr.endPoint = GENERICAPP_ENDPOINT;
          queen_CMD_DstAddr.addr.shortAddr = Msg->srcAddr.addr.shortAddr;
        }
        
        /* Send msg to Drone */
        if ( AF_DataRequest( &queen_CMD_DstAddr, &GenericApp_epDesc,
                       QUEEN_CMD_CLUSTERID,
                       3,
                       Msgbuf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
        {
        }
        else
        {
          // Error occurred in request to send.
        }
      }// if(temp[1] == DRONE_SEND_REQ)
    } // if included
    
    /* otherwise ask to resend init information*/
    else
    {
      
      // Setup for the command's destination address
      queen_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
      queen_CMD_DstAddr.endPoint = GENERICAPP_ENDPOINT;
      queen_CMD_DstAddr.addr.shortAddr = Msg->srcAddr.addr.shortAddr;

      Msgbuf[0] = 0x23;
      Msgbuf[1] = DRONE_SEND_REQ;
      
      // Send CMD
      if ( AF_DataRequest( &queen_CMD_DstAddr, &GenericApp_epDesc,
                         QUEEN_CMD_CLUSTERID,
                         3,
                         Msgbuf,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {
      // Error occurred in request to send.
      }   
    }
  }
}


/*********************************************************************
 * @fn      queen_HandleUART
 *
 * @brief   Handle the UART messagne, communicate with 3G module
 *
 * @param   
 *
 * @return  none
 */
static void queen_HandleUART(mtOSALSerialData_t *CMDMsg)
{
  uint8 i = 0;
  char EM770_BUF[36] = {0}; 
  char EM770_CCLK_CMP_BUF[10] = {0};
#ifdef REACTDRONE
  uint8 uartbuf;
  uint16 deviceshortaddr = 0;
#endif  
  
  osal_memcpy(EM770_BUF,&CMDMsg->msg[1],CMDMsg->msg[0]);
  // Copy data to buffer
  
  /* Four main stages in initialize EM770W 
   *
   * 1. Wait for EM770W READY 
   * 2. Send ATE0, and delay 2s
   * 3. Send IPINIT
   * 4. Send CCLK
   *  
   **/
  if(WCDMAModuleSTATUS != WCDMAModule_Ready)
  {
    // Wait for EM770W READY and Send ATE0
    if(WCDMAModuleSTATUS == WCDMAModule_NotAvailable)
    {
      if(strcmp(EM770_BUF,EM770_READY_MSG) == 0)
      {
        WCDMAModuleSTATUS = WCDMAModule_Available;
        HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
        // 3G Available, enable statu led2 d3
         
        myBlockingHalUARTWrite(0,EM770_ATE0,6);
        memset(EM770_BUF,0,sizeof(EM770_BUF));
      }
    }
    
    // Delay 1s
    else if((WCDMAModuleSTATUS == WCDMAModule_Available) && 
            ((WCDMAModuleSTEP == WCDMAsetup_NotReady)) || 
              (WCDMAModuleSTEP == WCDMAsetup_RESTART))
    {
      if(strcmp(EM770_BUF,EM770_ATE0_ACK) == 0)
      {
        osal_start_timerEx( GenericApp_TaskID,
                                GENERICAPP_EM770W_WAIT_EVT,
                                GENERICAPP_EM770W_WAIT_TIMEOUT );
        memset(EM770_BUF,0,sizeof(EM770_BUF));
        WCDMAModuleSTEP = WCDMAsetup_3GConnected; // Step 1, receive "EM770W READY", wait 1s     
      }  
    }
    else 
    {   
      // Delay finish, send IPINIT
      if((strcmp(EM770_BUF,EM770_ACK) == 0) && (WCDMAModuleSTEP == WCDMAsetup_3GConnected))
      {
        //Start timer to resend IPINIT. Avoid signal problem
        osal_start_timerEx( GenericApp_TaskID, GENERICAPP_TIMER_TWOSEC_EVT,
        GENERICAPP_TIMER_TWOSEC_TIMEOUT); 
        WCDMAModule_IPINITResend = 1;
        
        // Send IPINIT according to different select
        
#ifdef  LYCAMOBILE
        myBlockingHalUARTWrite(0,EM770_IPINIT,49);
#endif
        
#ifdef  CHINAUNICOM
        myBlockingHalUARTWrite(0,EM770_IPINIT,19);
#endif
        
#ifdef  USAMOBILE
        myBlockingHalUARTWrite(0,EM770_IPINIT,29);
#endif
        
#ifdef  USAMOBILE_WYLESS
        myBlockingHalUARTWrite(0,EM770_IPINIT,34);
#endif
        
        memset(EM770_BUF,0,sizeof(EM770_BUF));
        WCDMAModuleSTEP = WCDMAsetup_IPINITsend; // receive "EM770W READY", send IPINIT     
      }
      
      // IPINIT finish, send CCLK
      else if((strcmp(EM770_BUF,EM770_ACK) == 0) && WCDMAModuleSTEP == WCDMAsetup_IPINITsend)
      {
        WCDMAModule_IPINITResend = 0;  
        HalLedSet (HAL_LED_4, HAL_LED_MODE_OFF);
        // 3G Signal Available, enable statu led4, d2
        
        osal_start_timerEx( GenericApp_TaskID,
                          GENERICAPP_EM770W_WAIT_EVT,
                          GENERICAPP_EM770W_WAIT_TIMEOUT );
        memset(EM770_BUF,0,sizeof(EM770_BUF));
      }
      
      // Initialize TimeStamp
      else if(WCDMAModuleSTEP == WCDMAsetup_CCLKsend)
      {
        osal_memcpy(EM770_CCLK_CMP_BUF,EM770_BUF,9);
        if(strcmp(EM770_CCLK_CMP_BUF,EM770_CCLK_ACK) == 0)
        {
          for(i=0;i<=15;i++)
            JSON_TimeStamp[i] = EM770_BUF[i + 9];
          
          osal_start_timerEx( GenericApp_TaskID, GENERICAPP_TIMER_TWOSEC_EVT,
          GENERICAPP_TIMER_TWOSEC_TIMEOUT); // Start 2s timer
          
          myBlockingHalUARTWrite(0,EM770_CNMI,11);
          WCDMAModuleSTEP = WCDMAsetup_CNMIsend;
        }
      }
      else if((strcmp(EM770_BUF,EM770_ACK) == 0) && (WCDMAModuleSTEP == WCDMAsetup_CNMIsend))
      {
        
        HalLedSet (HAL_LED_4, HAL_LED_MODE_OFF);
        HalLedBlink( HAL_LED_4, 8, 50, 500 );
        HalLedBlink( HAL_LED_2, 8, 50, 500 );
        // D2, D3 flash indicate connection established
        
        //set module states parameters
        WCDMAModuleSTATUS = WCDMAModule_Ready; //Initialize Finish
        WCDMAModule_RestartTimer = FALSE; // Clear Restart Timer
        
        // set Queen upload states parameters
        queen_Available = AVAILABLE;
      }
    }
  }
  else //3G Module Ready, Send data
  {
    if(WCDMAModule_SEND)
    {
      WCDMAModule_SendOverTime = 1;
      
      // Operate according to send step
      switch(WCDMAModuleSTEP)
      {
      case WCDMAsetup_CNMIsend: // wait for ACK
        if(strcmp(EM770_BUF,EM770_ACK) == 0)
        {       
          myBlockingHalUARTWrite(0,EM770_IPOPEN,38);
          memset(EM770_BUF,0,sizeof(EM770_BUF));
          HalLedBlink( HAL_LED_4, 40, 50, 50 ); 
          WCDMAModuleSTEP = WCDMAsetup_IPOPENsend;
        }
        break;
        
      case WCDMAsetup_IPOPENsend: // has sent IPOPEN command, wait for IPOPEN AKC
        if(strcmp(EM770_BUF,EM770_ACK) == 0)
        {
          myBlockingHalUARTWrite(0,EM770_IPENTRANS,14);
          memset(EM770_BUF,0,sizeof(EM770_BUF));
          WCDMAModuleSTEP = WCDMAsetup_IPENTRANSsend;
        }
        
        // If no network connection, send finish
        else if(strcmp(EM770_BUF,EM770_ERROR_ACK) == 0)
        {
          HalLedSet (HAL_LED_4, HAL_LED_MODE_ON);
          // D2 off indicate connection error
          
          WCDMAModuleSTATUS = WCDMAModule_Available; 
          WCDMAModuleSTEP = WCDMAsetup_CNMIsend; // Jump back to stpe 1, reconnection
          WCDMAModule_IPINITResend = 1; // Set resend IPINIT flag
          
          WCDMAModule_SEND = 0; // No network, send finish
          WCDMAModule_SendOverTime = 0; // Send finish, end timer
          
          queen_Available = UNAVAILABLE; // set queen unavailable
        }
        
        break;
        
      case WCDMAsetup_IPENTRANSsend: // send IPENTRANS, wait for ACK. Send data, 1s later, send 11.
        if(strcmp(EM770_BUF,EM770_ENTRANS_ACK) == 0)
        {       
          memset(EM770_BUF,0,sizeof(EM770_BUF));
          osal_start_timerEx( GenericApp_TaskID,
                          GENERICAPP_EM770W_WAIT_EVT,
                          GENERICAPP_EM770W_WAIT_TIMEOUT );
        }
        break;
        
      case WCDMAsetup_IPENTRANSEnd: // Step 5, data sent, end 0x11 send, wait for ack
        if(strcmp(EM770_BUF,EM770_ENDTRANS_ACK) == 0)
        {       
          myBlockingHalUARTWrite(0,EM770_IPCLOSE,14);
          memset(EM770_BUF,0,sizeof(EM770_BUF)); 
          WCDMAModule_SEND = 0; // Send finish
          WCDMAModuleSTEP = WCDMAsetup_CNMIsend; // wait for IPOPEN
          WCDMAModule_SendOverTime = 0; // Send finish, end timer
          WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET; // Send success, reset value
          
          // set queen states parameter
          queen_Available = AVAILABLE;
        }
        break;
        
      default:
          break;
      }
    }
    
    else if(JSON_TimeStamp_Upgrade_flag)
    {
      if(strcmp(EM770_BUF,EM770_ACK) == 0)
        myBlockingHalUARTWrite(0,EM770_CCLK,10);
      else 
      {
        osal_memcpy(EM770_CCLK_CMP_BUF,EM770_BUF,9);
        
        if(strcmp(EM770_CCLK_CMP_BUF,EM770_CCLK_ACK) == 0)
        {
          for(i=0;i<=15;i++)
            JSON_TimeStamp[i] = EM770_BUF[i + 9];
          
          JSON_TimeStamp_Upgrade_flag = 0; // Clear flag
            
          // If have unsend Msg, send AT to push
          if(BufHasSample() && (!WCDMAModule_SEND))
          {
            WCDMAModule_SEND = 1;
            myBlockingHalUARTWrite(0,EM770_AT,4);
          }
        }
      }
    }
  }

#ifdef REACTDRONE
    /* Device Control CMD */
  if((CMDMsg->msg[2]) == 0x41)
  {
    // Prepare data for send
    i = ((CMDMsg->msg[4] & 0xf0) >> 4);
    uartbuf = CMDMsg->msg[4] & 0x01;
       
    // Setup for the command's destination address
    Receiver_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    Receiver_CMD_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    Receiver_CMD_DstAddr.addr.shortAddr = AddressManageBUF[i-1];
    
    // Send CMD
    if ( AF_DataRequest( &queen_CMD_DstAddr, &GenericApp_epDesc,
                         QUEEN_CMD_CLUSTERID,
                         1,
                         &uartbuf,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
    
    // Send information on uart
    switch(uartbuf)
    {
    case 0:
      //myBlockingHalUARTWrite(0,"Device Off CMD has been send.\n",30);
      break;
    case 1:
      //myBlockingHalUARTWrite(0,"Device On CMD has been send.\n",29);
      break;
    default:
      break;
    }     
  }
#endif
  
}


/*********************************************************************
 * @fn      queen_PERIODICDATA_SERVICE
 *
 * @brief   Receive periodic data msg and react
 *
 * @param   none
 *
 * @return  none
 */
static void queen_PERIODICDATA_SERVICE( afIncomingMSGPacket_t *Msg )
{
  uint8 i,j = 0;
  uint8 bufincluded = 0;
  uint8 temp[25] = {0};
  uint8 Msgbuf[3] = {0x23,0x00,0x00};
  
  osal_memcpy(temp,&Msg->cmd.Data[0],Msg->cmd.Data[2]);
  
  /* Decide whether wrong receive */
  j = temp[temp[2] - 1];
  temp[temp[2] - 1] = 0;
  for(i = 0;i <= (temp[2] - 2);i++)
  {
    temp[temp[2] - 1] += temp[i];
  }
  if(j != temp[temp[2] - 1])
    return;
  
  /* Decide msg is from which device */
  for(i = 0;i <= (MAXDEVNUM - 1);i++)
  {
    if(AddressManageBUF[i] == (Msg->srcAddr.addr.shortAddr))
    {
      i += (1 + 48);
      bufincluded = 1;
      break;
    }
  }
  
  /* If device is already in look-up table, send DATA ACK */
  if(bufincluded)
  {
    /* send ACK */
    Msgbuf[1] = DRONE_DATA_ACK;
    // Setup for the command's destination address
    queen_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    queen_CMD_DstAddr.endPoint = GENERICAPP_ENDPOINT;
    queen_CMD_DstAddr.addr.shortAddr = Msg->srcAddr.addr.shortAddr;
    
    // Send CMD
    if ( AF_DataRequest( &queen_CMD_DstAddr, &GenericApp_epDesc,
                         QUEEN_CMD_CLUSTERID,
                         3,
                         Msgbuf,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
    /* Data processing */
    
    /* Data sending 
     * 
     * 1. Push data to ring buffer
     * 2. If not busy, send directly
     *
     **/
    
      /* Modify the data 
       *
       * 1. Add time stamp
       * 2. Add device num
       *
       */
    
    if(temp[3] && 0x40) // Data Frame
    {
      /* Push the data into ring buffer */
      HalLedBlink( HAL_LED_3, 40, 50, 50 );
      PushBufSample(i); // Save the device number
      
      if(temp[3] && 0x80) // Whether the device contain status information?
      {
        switch(temp[3] && 0x01)
        {
          case 0x01:
            PushBufSample(0x31); // Device on
            break;
          case 0x00:
            PushBufSample(0x30); // Device off
            break;
          default:
            break;
        }
      }
      else
      {
        PushBufSample(0x30); // Default device off
      } //if(temp[3] && 0x80) 
      
      PushBufSample(temp[5]); // Save the data1 - Temperature
      PushBufSample(temp[6]); // Save the data2 - Current
      PushBufSample(temp[7]); // Save the data3 - Pressure Transducer (H)
      PushBufSample(temp[8]); // Save the data4 - Pressure Transducer (L)
      
      for(i=0;i<=15;i++) // Save the Time Stamp
        PushBufSample(JSON_TimeStamp[i]);
      
      // Change Ringbuffer information
      WCDMA_Ringbuffer.counter ++;
      if(WCDMA_Ringbuffer.counter > 2)
        WCDMA_Ringbuffer.counter = 2;
    }
    else
    {
      // If not Data frame response here(only status)
    } // if(temp[3] && 0x40) // Data Frame
  }// if(bufincluded)
  
  /* Device does not exist in look-up table, ask for initial msg */
  else
  {
    Msgbuf[1] = DRONE_SEND_REQ;
    // Setup for the command's destination address
    queen_CMD_DstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    queen_CMD_DstAddr.endPoint = GENERICAPP_ENDPOINT;
    queen_CMD_DstAddr.addr.shortAddr = Msg->srcAddr.addr.shortAddr;
    
    // Send CMD
    if ( AF_DataRequest( &queen_CMD_DstAddr, &GenericApp_epDesc,
                         QUEEN_CMD_CLUSTERID,
                         3,
                         Msgbuf,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
  }
}


/******************************************************************************
                                RingBuffer
******************************************************************************/

/*********************************************************************
 * @fn      GetBufSample
 *
 * @brief   Read data out from the ring buffer
 *
 * @param   
 *
 * @return  one byte data
 */
static uint8 GetBufSample(void)
{
  uint8 value = 0;
  
  value = *WCDMA_Ringbuffer.pOut;
  
  if(WCDMA_Ringbuffer.pOut == &WCDMA_DATA_Ringbuffer[Ringbuffer_Length - 1])
    WCDMA_Ringbuffer.pOut = WCDMA_DATA_Ringbuffer;
  else
    WCDMA_Ringbuffer.pOut ++;
  
  return value;
}

/*********************************************************************
 * @fn      PushBufSample
 *
 * @brief   Push one byte data to the ring buffer
 *
 * @param   
 *
 * @return  one byte data
 */
static uint8 PushBufSample(uint8 value)
{  
  *WCDMA_Ringbuffer.pIn = value;
  
  if(WCDMA_Ringbuffer.pIn == &WCDMA_DATA_Ringbuffer[Ringbuffer_Length - 1])
    WCDMA_Ringbuffer.pIn = WCDMA_DATA_Ringbuffer;
  else
    WCDMA_Ringbuffer.pIn ++;
 
  return value;
}


/*********************************************************************
 * @fn      BufHasSample
 *
 * @brief   Check whether data 
 *
 * @param   
 *
 * @return  1 - has, 0 - not
 */
static uint8 BufHasSample(void)
{
  return (WCDMA_Ringbuffer.counter != 0);
}


/*********************************************************************
                          others
*********************************************************************/

/*********************************************************************
 * @fn      prepareTheSendingBUF
 *
 * @brief   Prepare the sending buf to send 
 *
 * @param   none
 *
 * @return  
 */
static void prepareTheSendingBUF(void)
{
  float b;
  uint8 temp;
  
  //Adjust here to modifiy the Queen ID
  uint8 SendingBUF_QueenID[8] = QUEEN_ID;
  uint8 SendingBUF_TimeStamp[16] = {0};
  
  /* Check error in the ring buffer */
  if(((WCDMA_Ringbuffer.pIn != WCDMA_DATA_Ringbuffer)
     && (WCDMA_Ringbuffer.pIn != &WCDMA_DATA_Ringbuffer[22])
     && (WCDMA_Ringbuffer.pIn != &WCDMA_DATA_Ringbuffer[44])
     && (WCDMA_Ringbuffer.pIn != &WCDMA_DATA_Ringbuffer[66])) ||
     ((WCDMA_Ringbuffer.pOut != WCDMA_DATA_Ringbuffer)
     && (WCDMA_Ringbuffer.pOut != &WCDMA_DATA_Ringbuffer[22])
     && (WCDMA_Ringbuffer.pOut != &WCDMA_DATA_Ringbuffer[44])
     && (WCDMA_Ringbuffer.pOut != &WCDMA_DATA_Ringbuffer[66]))
     )
  {
    // reset ring buffer
    WCDMA_Ringbuffer.pIn = WCDMA_DATA_Ringbuffer;
    WCDMA_Ringbuffer.pOut = WCDMA_DATA_Ringbuffer;
    
    WCDMA_Ringbuffer.counter = 0;
    return;
  }
  

  /* Modify on the sending buffer (temperature) */
  
  /* Prepare Data, the order is according to the FIFO ring buffer
   *
   * 1. Change device number
   * 2. Change device status
   * 3. Change device temperature
   * 4. Change device current
   * 5. Change device h-pressure
   * 6. Change device l-pressure
   * 7. Change device timestamp
   * 8. Change ringbuffer information
   *
   **/
  
  // 1. Change device number
  // Change Source Indicate according to Define Symbols
  
  //previews Multi ID version is ver3 backup7 (use macro)

  for(temp = 0;temp <= 7; temp++)
    EM770_Sending[WCDMA_DATA_DEVICEID + temp] = SendingBUF_QueenID[temp];

  EM770_Sending[WCDMA_DATA_DEVICEID + 8] = 0x30;
  EM770_Sending[WCDMA_DATA_DEVICEID + 9] = GetBufSample(); // Drone Number
  
  // 2. Change status
  EM770_Sending[WCDMA_DATA_STATUS] = GetBufSample();

  // 3. Change temperature
  temp = GetBufSample();
  b = (1000*(3.3*(float)temp/255));
  
  temp = (uint8)(b/1000)+ 48;
  EM770_Sending[WCDMA_DATA_TEMP] = temp;
  temp = (uint16)b%1000/100 + 48;
  EM770_Sending[WCDMA_DATA_TEMP + 1] = temp;
  temp = (uint16)b%100/10 + 48;
  EM770_Sending[WCDMA_DATA_TEMP + 2] = temp;
  temp = (uint16)b%10 + 48;
  EM770_Sending[WCDMA_DATA_TEMP + 3] = temp;
    
  // 4. Change Current 
  temp = GetBufSample();
  
#ifdef NEW_BOARD
  b = (1000*(3.3*(float)temp/255)*5/2);
#endif
  
#ifdef OLD_BOARD
  b = (1000*(3.3*(float)temp/255)); 
#endif
  
  temp = (uint8)(b/1000)+ 48;
  EM770_Sending[WCDMA_DATA_CURRENT] = temp;
  temp = (uint16)b%1000/100 + 48;
  EM770_Sending[WCDMA_DATA_CURRENT + 1] = temp;
  temp = (uint16)b%100/10 + 48;
  EM770_Sending[WCDMA_DATA_CURRENT + 2] = temp;
  temp = (uint16)b%10 + 48;
  EM770_Sending[WCDMA_DATA_CURRENT + 3] = temp;
  
  /* 5. Change h-preesure
   * 
   */
  
  temp = GetBufSample();
  
#ifdef NEW_BOARD
  b = (1000*(3.3*(float)temp/255)*5/2);
#endif
  
#ifdef OLD_BOARD
  b = (1000*(3.3*(float)temp/255)); 
#endif
  
  temp = (uint8)(b/1000)+ 48;
  EM770_Sending[WCDMA_DATA_HPRESS] = temp;
  temp = (uint16)b%1000/100 + 48;
  EM770_Sending[WCDMA_DATA_HPRESS + 1] = temp;
  temp = (uint16)b%100/10 + 48;
  EM770_Sending[WCDMA_DATA_HPRESS + 2] = temp;
  temp = (uint16)b%10 + 48;
  EM770_Sending[WCDMA_DATA_HPRESS + 3] = temp;

  
  /* 6. Change l-preesure
   * 
   */
  
  temp = GetBufSample();
  
#ifdef NEW_BOARD
  b = (1000*(3.3*(float)temp/255)*5/2);
#endif
  
#ifdef OLD_BOARD
  b = (1000*(3.3*(float)temp/255)); 
#endif
  
  temp = (uint8)(b/1000)+ 48;
  EM770_Sending[WCDMA_DATA_LPRESS] = temp;
  temp = (uint16)b%1000/100 + 48;
  EM770_Sending[WCDMA_DATA_LPRESS + 1] = temp;
  temp = (uint16)b%100/10 + 48;
  EM770_Sending[WCDMA_DATA_LPRESS + 2] = temp;
  temp = (uint16)b%10 + 48;
  EM770_Sending[WCDMA_DATA_LPRESS + 3] = temp; 
  

  // 7. Change TimeStamp
  for(temp=0;temp<=15;temp++)
    SendingBUF_TimeStamp[temp] = GetBufSample();
  osal_memcpy(&EM770_Sending[WCDMA_DATA_TIME],SendingBUF_TimeStamp,WCDMA_DATA_TIME_L);

  // 8. Change Ringbuffer information
  WCDMA_Ringbuffer.counter --;
}

/*********************************************************************
 * @fn      queen_Reset3GModule
 *
 * @brief   clear flags, reset 3G Module
 *
 * @param   none
 *
 * @return  none
 */
static void queen_Reset3GModule( void )
{
  // reload reconnect failure reset times (2 times default)
  WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET;
  // auto restart 3G module every setting time
  WCDMAModule_ResetTimer = (WCDMA_RESETTIMER*60*60/2);
  // set restart timer (time to send reset cmd again)
  WCDMAModule_RestartTimer = WCDMA_RESTARTTIMER;  

  // reset 3g flags
  WCDMAModuleSTATUS = 0; // 0 - not available, 1 - available, 2 - ready
  WCDMAModuleSTEP = WCDMAsetup_RESTART; // 0-x, indicate the different step in setup
  WCDMAModule_SEND = 0; // Data need to send flag
  WCDMAModule_IPINITResend_Count = 0; //Resend counter
  WCDMAModule_IPINITResend = 0; // Resend flag and conter
  WCDMAModule_SendOverTime = 0; // Send Counter, if Send more than WCDMA_OVERTIME s, sending abort
  WCDMAModule_CCLKDelayCount = 0; // CCLK CMD delay counter   

  
  
  // set timer to count restart time
  osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_EM770W_WAIT_EVT,
                      GENERICAPP_EM770W_WAIT_TIMEOUT );  
  
  // off LED
  HalLedSet (HAL_LED_2, HAL_LED_MODE_ON); // LED2, D3 negative logic
  HalLedSet (HAL_LED_4, HAL_LED_MODE_ON); // LED4, D2 negative logic
  
  myBlockingHalUARTWrite(0,EM770_RESET,12);
}

/*********************************************************************
 * @fn      myBlockingHalUARTWrite
 *
 * @brief   blocking Halwrite function
 *
 * @param   none
 *
 * @return  none
 */
static void myBlockingHalUARTWrite(uint8 port, uint8 *msg, uint8 len)
{
  while (HalUARTWrite(port, msg, len) != len)
  {
    HalUARTPoll();
  }
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }

      default:
        break;  /* Ignore unknown command */
    }
  }
}
#endif

/*********************************************************************
 */
