/**************************************************************************************************
  Filename:       GenericApp.h
  Revised:        $Date: 2012-02-12 16:04:42 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29217 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef GENERICAPP_H
#define GENERICAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "Config_APP.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define GENERICAPP_ENDPOINT           23

#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x0001
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#define GENERICAPP_MAX_CLUSTERS       3
#define GENERICAPP_CLUSTERID          1
#define QUEEN_CMD_CLUSTERID           2
  
// Send Message Timeout
#define GENERICAPP_MU609_WAIT_TIMEOUT           1000     // Delay 1 seconds
#define GENERICAPP_WDT_CLEAR_TIMEOUT            125      // Every 125 ms  
  
// Application Events (OSAL) - These are bit weighted definitions.     
#define GENERICAPP_MU609_WAIT_EVT       0x0001
#define GENERICAPP_WDT_CLEAR_EVT        0x0002
  
// CMD with drones
#define DRONE_RESEND_INIT       0x0d
#define DRONE_DATA_ACK          0x0f
#define DRONE_SUCCESS_ACK       0x08
#define DRONE_FAIL_ACK          0x04
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
   
#if defined( IAR_ARMCM3_LM )
#define GENERICAPP_RTOS_MSG_EVT       0x0002
#endif  

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
 * EXTERNAL VARIABLES
 */
#ifdef RESETQUEEN
// flag set when needs to reset Queen
extern uint8 resetQueen;
#endif

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */
