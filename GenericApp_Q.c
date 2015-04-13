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

    D1 - LED3 : Zigbee Network Indicator
      
    D2 - LED4 : Cellular Status Indicator
      Not detect cellular module: LED off
      Detected module, initializing: Flashing in 1s cycle
      Normal Status: LED on
      Finish a upload: LED flash in 0.5s/2s
    
    D3 - LED2 : Signal Indicator
      No Signal, LED ON
      Has signal, but services restricted, flash LED
      Signal Normal, LED ON
    
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

#include "Cellular_Module.h"
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

// Sending Buffer related
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

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

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

uint16 AddressManageBUF[MAXDEVNUM] = {0};  // Address Manage buffer

afAddrType_t queen_CMD_DstAddr;
// end

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void queen_CMDReact(afIncomingMSGPacket_t *Msg);

static void queen_HandleUART( mtOSALSerialData_t *CMDMsg );
static void queen_PERIODICDATA_SERVICE( afIncomingMSGPacket_t *Msg );
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
              osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_MU609_WAIT_EVT,
                      GENERICAPP_MU609_WAIT_TIMEOUT ); 
              // Start the time after Zigbee network setup
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

  /* 1s Delay Timer Event, setup at GeneriApp_Init. Run forever.
   */
  if(events & GENERICAPP_MU609_WAIT_EVT)
  {

    Cellular_OneSecondTimerServer(GenericApp_TaskID,GENERICAPP_MU609_WAIT_EVT,
                                  GENERICAPP_MU609_WAIT_TIMEOUT);
    
    // reload timer 
    osal_start_timerEx( GenericApp_TaskID,
                      GENERICAPP_MU609_WAIT_EVT,
                      GENERICAPP_MU609_WAIT_TIMEOUT );
    
    return (events ^ GENERICAPP_MU609_WAIT_EVT);
  }  

  
#ifdef WDT_IN_PM1
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
#endif

  
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
    // Zigbee control(ACK, sending request, etc.) 
    case QUEEN_CMD_CLUSTERID: 
      queen_CMDReact(pkt);
      break;
    
    // Data service (drone allowed to send only after get approval from queen)
    case GENERICAPP_CLUSTERID: 
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
  Cellular_UART(CMDMsg);
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
  short bufincluded = 0;
  uint8 temp[25] = {0};
  uint8 Msgbuf[3] = {0x23,0x00,0x00};
  float b;
  uint8 tempNum;
  
  //Adjust here to modifiy the Queen ID
  uint8 SendingBUF_QueenID[8] = QUEEN_ID;
  
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
    if(temp[3] && 0x40) // Data Frame
    {
      /* put data into sending buffer */
      HalLedBlink( HAL_LED_3, 40, 50, 50 );
      
      // 1. modify the ID part
      for(j = 0;j <= 7; j++)
          MU609_Sending[WCDMA_DATA_DEVICEID + j] = SendingBUF_QueenID[j];      
      
      MU609_Sending[WCDMA_DATA_DEVICEID + 8] = 0x30;
      MU609_Sending[WCDMA_DATA_DEVICEID + 9] = i; // Drone Number
      
      // 2. Whether the device contain status information?
      if(temp[3] && 0x80) 
      {
        switch(temp[3] && 0x01)
        {
          case 0x01:
            MU609_Sending[WCDMA_DATA_STATUS] = 0x31; // Device on
            break;
          case 0x00:
            MU609_Sending[WCDMA_DATA_STATUS] = 0x30; // Device off
            break;
          default:
            break;
        }
      }
      else
      {
        MU609_Sending[WCDMA_DATA_STATUS] = 0x30; // Default device off
      } //if(temp[3] && 0x80)
      
      // 3. Prepare temperature
      tempNum = temp[5];
      b = (1000*(3.3*(float)tempNum/255));
      
      tempNum = (uint8)(b/1000)+ 48;
      MU609_Sending[WCDMA_DATA_TEMP] = tempNum;
      tempNum = (uint16)b%1000/100 + 48;
      MU609_Sending[WCDMA_DATA_TEMP + 1] = tempNum;
      tempNum = (uint16)b%100/10 + 48;
      MU609_Sending[WCDMA_DATA_TEMP + 2] = tempNum;
      tempNum = (uint16)b%10 + 48;
      MU609_Sending[WCDMA_DATA_TEMP + 3] = tempNum;
      
      // 4. Prepare current
      b = (1000*(3.3*(float)temp[6]/255)*5/2);
      tempNum = (uint8)(b/1000)+ 48;
      MU609_Sending[WCDMA_DATA_CURRENT] = tempNum;
      tempNum = (uint16)b%1000/100 + 48;
      MU609_Sending[WCDMA_DATA_CURRENT + 1] = tempNum;
      tempNum = (uint16)b%100/10 + 48;
      MU609_Sending[WCDMA_DATA_CURRENT + 2] = tempNum;
      tempNum = (uint16)b%10 + 48;
      MU609_Sending[WCDMA_DATA_CURRENT + 3] = tempNum;
      
      // 5. High Pressure
      b = (1000*(3.3*(float)temp[7]/255)*5/2);
      tempNum = (uint8)(b/1000)+ 48;
      MU609_Sending[WCDMA_DATA_HPRESS] = tempNum;
      tempNum = (uint16)b%1000/100 + 48;
      MU609_Sending[WCDMA_DATA_HPRESS + 1] = tempNum;
      tempNum = (uint16)b%100/10 + 48;
      MU609_Sending[WCDMA_DATA_HPRESS + 2] = tempNum;
      tempNum = (uint16)b%10 + 48;
      MU609_Sending[WCDMA_DATA_HPRESS + 3] = tempNum;
      
      // 6. Low Pressure
      b = (1000*(3.3*(float)temp[8]/255)*5/2);
      tempNum = (uint8)(b/1000)+ 48;
      MU609_Sending[WCDMA_DATA_LPRESS] = tempNum;
      tempNum = (uint16)b%1000/100 + 48;
      MU609_Sending[WCDMA_DATA_LPRESS + 1] = tempNum;
      tempNum = (uint16)b%100/10 + 48;
      MU609_Sending[WCDMA_DATA_LPRESS + 2] = tempNum;
      tempNum = (uint16)b%10 + 48;
      MU609_Sending[WCDMA_DATA_LPRESS + 3] = tempNum; 
     
      // 7. TimeStamp
      osal_memcpy(&MU609_Sending[WCDMA_DATA_TIME],JSON_TimeStamp,WCDMA_DATA_TIME_L);
    }
    else
    {
      // If not Data frame response here(only status)
    } // if(temp[3] && 0x40) // Data Frame
    
    // send AT to push, enable timer
    myBlockingHalUARTWrite(0,MU609_AT,4);
    // change status
    WCDMAModuleSTEP = WCDMAsetup_ATGO; 
    // Timer, at push resend timer
    setWCDMAoneSecondStepTimer(ENABLE,WCDMA_10SDELAY,10);// Start timer/counter
      
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

/*********************************************************************
                          others
*********************************************************************/

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
