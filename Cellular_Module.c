/*********************************************************************
  This .c file provide essential function to control Cellular Module

  Function including initialize, send and all sub function needed in 
  main GenericApp.c file.

Coupling with other module description:
Lower level: UART must provide myBlockingHalUARTWrite(;;) function.


Function list:


  @Author Yiran Tian 2015/3/4
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
#include "math.h"
#include "string.h"

/* MT */
#include "MT_UART.h"
#include "MT.h"

/* HAL */
#include "hal_led.h"
#include "hal_uart.h"

#include "Cellular_Module.h"

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/* HUAWEI AT Command Related
 */
#include "AT_CMDdata.h" // all the CMD is in this h-file

/* Control Related
 */

uint8 WCDMAModuleSTEP = 0; 
// 0-x, indicate the different step in setup
uint8 WCDMASignalState = NoService;
// Signal indicator

// ----------------- counter -----------------
static uint16 WCDMAModule_ResetTimer = (WCDMA_RESETTIMER*60*60/2);
// After device setup, cellular module will automatically restart every setting time

static uint8 WCDMAModule_RestartTimer = FALSE;
// After MC send reset CMD, timer start. Once the cellular module restart, 
// timer clear. If overtime, resend CMD.

/* upload time counter and failure counter */
static uint8 WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET; 
// Timer start immediantly after start uploading process. If upload takes more
// than setting time, one connection over time ocour. If it happen more than
// WCDMA_OVERTIMERESET times, cellular module restart.

static short WCDMAModuleTimerEnable = FALSE;
static uint8 WCDMAModuleTimerCounter = 0;
static uint8 WCDMAModuleFailureTimes = 0;

// ----------------- flags ----------------- 

// Time Stamp
uint8 JSON_TimeStamp[16] = {0}; // TimeStamp
static uint8 JSON_TimeCounter = 60;
static uint8 LEDAvoidControl = 0;

// Module Available
short queen_Available = NOT_READY;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void uartWriteIPINIT(void);
void setWCDMAoneSecondStepTimer(short, uint8, uint8);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
   
/*********************************************************************
 * @fn      Cellular_OneSecondTimerServer
 *
 * @brief   Function to control MU609. This function should be located
 *          in 1s timer response.
 *
 *          1. timer between steps;
 *          2. LED control;
 *          3. restart time timer;
 *          4. update timestamp;
 *          5. Reset Cellular Module every setting time
 *
 * @param   none
 *
 * @return  none
 */
void Cellular_OneSecondTimerServer( byte TaskID, uint32 Evt_ID, uint32 Evt_Timeout)
{
  /* 1. timer between steps.
   *    1. IPINIT overtime WCDMA_10SDELAY * WCDMAModule_IPINITResend_MAX
   *       resend IPINIT;
   *       reset cellular module;
   *
   *    2. One upload period WCDMA_SENDTIME * WCDMAModule_ReConOverTries(WCDMA_OVERTIMERESET)
   *       abandon current upload, wait for next try;
   *       reset cellular module;
   *
   *    3. IPCLOSE overtime
   */
  if(WCDMAModuleTimerEnable) // timer enable
  {
    WCDMAModuleTimerCounter --;
    
    if(WCDMAModuleTimerCounter == 0) // timeout
    {
      
      /* react according to different steps
       */
      // 1. IPINIT sending steps
      if(WCDMAModuleSTEP == WCDMAsetup_IPINITsend) 
      {
        WCDMAModuleFailureTimes --; // how many time it fail?
        
        if(WCDMAModuleFailureTimes) // still trying
        {
          WCDMAModuleTimerCounter = WCDMA_10SDELAY;
          uartWriteIPINIT(); // re-send IPINIT
        }
        else // send failed
        {
          queen_Reset3GModule(); // reset cellular module
        } 
      }
      
      // 2. AT PUSH resend
      else if(WCDMAModuleSTEP == WCDMAsetup_ATGO)
      {
        WCDMAModuleFailureTimes --; // how many time it fail?
        
        if(WCDMAModuleFailureTimes) // still trying
        {
          WCDMAModuleTimerCounter = WCDMA_10SDELAY;
          uartWriteIPINIT(); // re-send IPINIT
        }
        else // send failed
        {
          queen_Reset3GModule(); // reset cellular module
        } 
        
      }
      
      // 2. send during time
      else if((WCDMAModuleSTEP > WCDMAsetup_ATGO) 
              && (WCDMAModuleSTEP <= WCDMAsetup_IPCLOSEsend))
      {
        WCDMAModule_ReConOverTries --;
        if(WCDMAModule_ReConOverTries == 0) // if happen more than default times
        {
          queen_Reset3GModule(); // reset cellular module
        }
        else
        {
          WCDMAModuleSTEP = WCDMAsetup_3GReady; // wait for next try
          setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
        }       
      }
      
      // 3. IPCLOSE sending step
      else if(WCDMAModuleSTEP == WCDMAsetup_IPCLOSEsend)
      {
        WCDMAModuleFailureTimes --; // how many time it fail?
      
        if(WCDMAModuleFailureTimes)
        {
          WCDMAModuleTimerCounter = WCDMA_10SDELAY; // reload timer
          myBlockingHalUARTWrite(0,MU609_IPCLOSE,14); // re-send IPCLOSE
        }
        else
        {
          queen_Reset3GModule(); // reset cellular module
        } 
      }
    }
  }
  
  /* 2. LED control
   *
   * LED2, D3, Signal Indicator LED
   * LED4, D2, Cellular Module Status LED
   *
   */
  
   /* LED2, D3, Signal Indicator LED */

  // 1. No Service, LED OFF
  if(WCDMASignalState == NoService) 
  {
    HalLedSet (HAL_LED_2, HAL_LED_MODE_ON); // off LED2, internet LED
  }
  
  // 2. Has signal, but services restricted, flash LED
  else if(WCDMASignalState != ValServices)
  {
    //HalLedSet (HAL_LED_2, HAL_LED_MODE_ON); // off LED2
    HalLedBlink( HAL_LED_2, 2, 50, 1000 ); // flash 1s
  }
  
  // 3. Signal Normal, LED ON
  else
  {
    HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF); // Service available, enable statu led2 d3
  }
  
   /* LED4, D2, Cellular Status LED */
  // When Initialize Cellular Module, flash in 1s duty
  if((WCDMAModuleSTEP < WCDMAsetup_3GReady) && (WCDMAModuleSTEP != WCDMAsetup_NotReady))
  {
    HalLedBlink( HAL_LED_4, 2, 50, 1000 ); // flash 1s
  }
  
  // Not detect Cellular module, turn off LED
  else if(WCDMAModuleSTEP == WCDMAsetup_NotReady)
  {
    HalLedSet (HAL_LED_4, HAL_LED_MODE_ON); // Module Available, enable statu led4 d2
  }
  
  // Otherwise, if not flashing (successful upload), keep LED on
  else
  {
    // LEDAvoidControl was used to perform 0.5s/2s flash when successfully upload one datapoint.
    if(LEDAvoidControl == 0)
    {
      HalLedSet (HAL_LED_4, HAL_LED_MODE_OFF); // Module Available, enable statu led4 d2
    }
    else
      LEDAvoidControl --;
  }
  
  
  /* 3. check restart timer
   */
  if(WCDMAModule_RestartTimer)
  {
    WCDMAModule_RestartTimer --;
    
    if(!WCDMAModule_RestartTimer) // If timeout reset cellular module
    {
      queen_Reset3GModule();
    }
  }
  
  /* 4. update timestamp
   */
  if(WCDMAModuleSTEP > WCDMAsetup_NWTIMEwait)
  {
    JSON_TimeCounter --;
    if(JSON_TimeCounter == 0)
    {
      JSON_TimeCounter = 60; // reload timer
      
      JSON_TimeStamp[15] ++;
      if(JSON_TimeStamp[15] == 58)
      {
        JSON_TimeStamp[15] = 48;

        JSON_TimeStamp[14] ++;
        if(JSON_TimeStamp[14] == 54)
        {    
          /* If one hour, set back to 59, calibrate the time */
          JSON_TimeStamp[14] = 53;
          JSON_TimeStamp[15] = 57;
          
          if(queen_Available == AVAILABLE)
          {         
            // set unavailable
            queen_Available = UNAVAILABLE;
            
            // set steps to time update
            WCDMAModuleSTEP = WCDMAsetup_TIMESTAMPUPDATE;
            
            // send AT to push
            myBlockingHalUARTWrite(0,MU609_AT,4);
          }
          else
          {
            // device busy, check 1s later
            JSON_TimeCounter = 1; 
          }
        }
      }
    }
  }
  
  /* 5. Reset Cellular Module every setting time
   */
  WCDMAModule_ResetTimer --;
  if(!WCDMAModule_ResetTimer) // If cellular not busy, reset immediantly
  {
    if(queen_Available == AVAILABLE)
    {
      queen_Reset3GModule(); // reset 3G
    }
    else
      WCDMAModule_ResetTimer = 10; // If cellular busy, check 10s later
  }
  
}
   
   
/*********************************************************************
 * @fn      Cellular_UART
 *
 * @brief   Function to control MU609, a state machine. Different state
 *          detail goes to .h file.
 *
 * @param   none
 *
 * @return  none
 */
void Cellular_UART(mtOSALSerialData_t *CMDMsg)
{
  uint8 i = 0;
  char MU609_BUF[45] = {0}; 
  char MU609_TEMP[10] = {0};
  
#ifdef REACTDRONE
  uint8 uartbuf;
  uint16 deviceshortaddr = 0;
#endif  
  
  osal_memcpy(MU609_BUF,&CMDMsg->msg[1],CMDMsg->msg[0]);
  // Copy data to buffer

  /* Signal detection
   *
   * By receiving SRVST report, the TE will know the signal status
   * 0 - no service      1 - Restricted services
   * 2 - Valid service   3 - Restricted regional service 
   * 4 - Power saving or hibernated 
   *
   **/
  osal_memcpy(MU609_TEMP,MU609_BUF,8); // copy the fist 8 char, "^SRVST"
  if(strcmp(MU609_TEMP,MU609_SRVST_ACK) == 0) // signal update information 
  {
    /* update signal status */
    WCDMASignalState = MU609_BUF[9] - 48; // obtain the signal state
  }
  
  /* If this is not about signal update, then goes to state machine
   **/
  else
  {
    /* Start state machine */
    switch(WCDMAModuleSTEP)
    {
      // -------------------- init -----------------------
      
      // 1. p:After power up 
    case (WCDMAsetup_NotReady): 
      if(strcmp(MU609_BUF,MU609_READY_MSG) == 0)
      {
        WCDMAModule_RestartTimer = WCDMA_RESTARTTIMER; // first timer finish, next stage
          
        myBlockingHalUARTWrite(0,MU609_CURC,17); // send CURC
        WCDMAModuleSTEP = WCDMAsetup_Connected; // change state
      }
      break;
      
      // 2. p:Received poweron Msg, send CURC
    case WCDMAsetup_Connected:   
      if(strcmp(MU609_BUF,MU609_CURC_ACK) == 0)
      {
        myBlockingHalUARTWrite(0,MU609_ATE0,6); // send ATE0
        WCDMAModuleSTEP = WCDMAsetup_ATE0send; // change state  
      }
      break;
    
      // 3.   p:Received 'OK', send ATE0
    case WCDMAsetup_ATE0send:
      if(strcmp(MU609_BUF,MU609_ATE0_ACK) == 0)
      {
        WCDMAModuleSTEP = WCDMAsetup_NWTIMEwait;
        // Start timer/counter
      }
      break;
      
      // 4. p:Received 'OK', wait for NWTIME report
    case WCDMAsetup_NWTIMEwait:
      osal_memcpy(MU609_TEMP,MU609_BUF,9);

      if(strcmp(MU609_TEMP,MU609_NWTIME_ACK) == 0) // check NWTIME, update timestamp
      {
        // Put the false here also allow the device recover from no NWTIME response
        WCDMAModule_RestartTimer = FALSE; // disable restartTimer
        // Stop timer/counter
        
        JSON_TimeStamp[0] = '2'; // update timestamp
        JSON_TimeStamp[1] = '0';
        
        for(i=2;i<=15;i++)
          JSON_TimeStamp[i] = MU609_BUF[i+9];

        uartWriteIPINIT(); // send IPINIT
        
        setWCDMAoneSecondStepTimer(ENABLE,WCDMA_10SDELAY,WCDMAModule_IPINITResend_MAX);
        
        WCDMAModuleSTEP = WCDMAsetup_IPINITsend; //change state
      }
      break;
      
      // 5. p:Received timestamp, send IPINIT
    case WCDMAsetup_IPINITsend:
      if(strcmp(MU609_BUF,MU609_ACK) == 0)
      {
        setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
        queen_Available = AVAILABLE; // set Queen upload states parameters (may imerge to others)
        
        WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state, initialize finish
      }
      break;
      
      // -------------------- Send -----------------------
      
      // 6. p: Received 'OK', initialize finish
    case WCDMAsetup_ATGO:  
      if((strcmp(MU609_BUF,MU609_ACK) == 0) && (queen_Available == UNAVAILABLE))
      {
        setWCDMAoneSecondStepTimer(ENABLE,WCDMA_SENDTIME,0);// Start new timer/counter
       
        myBlockingHalUARTWrite(0,MU609_IPOPEN,38); // send IPOPEN
         
        WCDMAModuleSTEP = WCDMAsetup_IPOPENsend; // change state
      }
      break;
      
      // 7. p: Check previos steps, send IPOPEN
    case WCDMAsetup_IPOPENsend:
      if(strcmp(MU609_BUF,MU609_ACK) == 0)
      {
        myBlockingHalUARTWrite(0,MU609_IPSENDEX,21); // send IPSENDEX
        
        WCDMAModuleSTEP = WCDMAsetup_IPSENDEXsend; // change state
      }
      else // error condition
      {
        // error happen, jump back, resend IPINIT (May Not Needed!!!!!!!!)
        /*if(strcmp(MU609_BUF,MU609_ERROR_ACK) == 0)
        {
          WCDMAModuleSTEP = WCDMAsetup_IPINITsend; // change state
          uartWriteIPINIT(); // send IPINIT
          setWCDMAoneSecondStepTimer(ENABLE,WCDMA_10SDELAY,WCDMAModule_IPINITResend_MAX);
          
          queen_Available = UNAVAILABLE; // set queen unavailable
        }*/
        
        // error type2, poor signal. Normal problem. Upload aboard
        memset(MU609_BUF,0,sizeof(MU609_BUF));
        osal_memcpy(MU609_BUF,&CMDMsg->msg[1],12);
        
        if(strcmp(MU609_BUF,MU609_IPOPEN_ERROR) == 0)
        {
          WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
          WCDMAModule_ReConOverTries --; 
          queen_Available = AVAILABLE; // set queen states parameter
        }
      }
      break;
      
      // 8. p: Received 'OK', send IPSENDEX
    case WCDMAsetup_IPSENDEXsend:
      if(strcmp(MU609_BUF,MU609_ACK) == 0)
      {
        myBlockingHalUARTWrite(0,MU609_Sending,100); // send data
        myBlockingHalUARTWrite(0,&MU609_Sending[100],100);
        myBlockingHalUARTWrite(0,&MU609_Sending[200],100);
        myBlockingHalUARTWrite(0,&MU609_Sending[300],33);
        
        WCDMAModuleSTEP = WCDMAsetup_OKtoSend; // Change state
      }
      else // error condition
      {
        memset(MU609_BUF,0,sizeof(MU609_BUF));
        osal_memcpy(MU609_BUF,&CMDMsg->msg[1],12);
        
        if(strcmp(MU609_BUF,MU609_IPOPEN_ERROR) == 0)
        {
          WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
          WCDMAModule_ReConOverTries --; 
          queen_Available = AVAILABLE; // set queen states parameter
        }
      }
      break;
      
      // 9. p: Received 'OK', send data 
    case WCDMAsetup_OKtoSend:
      if(strcmp(MU609_BUF,MU609_IPSENDEX_ACK) == 0)
      {
        myBlockingHalUARTWrite(0,MU609_IPCLOSE,14); // send IPCLOSE
        setWCDMAoneSecondStepTimer(ENABLE,WCDMA_10SDELAY,10);// Start timer/counter// set resend flag
        
        WCDMAModuleSTEP = WCDMAsetup_IPCLOSEsend; // change state
      }
      else
      {
        memset(MU609_BUF,0,sizeof(MU609_BUF));
        osal_memcpy(MU609_BUF,&CMDMsg->msg[1],12);
        
        if(strcmp(MU609_BUF,MU609_IPOPEN_ERROR) == 0)
        {
          WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
          WCDMAModule_ReConOverTries --; 
          queen_Available = AVAILABLE; // set queen states parameter
        }
      }
      break;
      
      // 10. p: Received IPSENDEX ACK, send IPCLOSE
    case WCDMAsetup_IPCLOSEsend:
      if(strcmp(MU609_BUF,MU609_ACK) == 0)
      {
        setWCDMAoneSecondStepTimer(DISABLE,0,0); //clear resend flag/counter
        HalLedBlink( HAL_LED_4, 4, 50, 500 ); // Flash LED, send success
        LEDAvoidControl = 2; // avoid timer control LED for next 2s
        
        WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
        WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET; 
        queen_Available = AVAILABLE; // set queen states parameter
      }
      break;
      
      // Others, update timestamp
    case WCDMAsetup_TIMESTAMPUPDATE:
      if(strcmp(MU609_BUF,MU609_ACK) == 0)
      {
        myBlockingHalUARTWrite(0,MU609_NWTIME,12);// send NWTIME
        
        WCDMAModuleSTEP = WCDMAsetup_TIMESTAMPUPDATES2; // change state
      }
      break;
      
      // update timetamp stage 2
    case WCDMAsetup_TIMESTAMPUPDATES2:
      osal_memcpy(MU609_TEMP,MU609_BUF,9);

      if(strcmp(MU609_TEMP,MU609_NWTIME_ACK) == 0) // check NWTIME, update timestamp
      {
        // Stop timer/counter
        
        JSON_TimeStamp[0] = '2'; // update timestamp
        JSON_TimeStamp[1] = '0';
        
        for(i=2;i<=15;i++)
          JSON_TimeStamp[i] = MU609_BUF[i+9];
        
        WCDMAModuleSTEP = WCDMAsetup_3GReady; //change state
        queen_Available = AVAILABLE; // set queen states parameter
      }
      break;
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
 * @fn      queen_Reset3GModule
 *
 * @brief   clear flags, reset 3G Module
 *
 * @param   none
 *
 * @return  none
 */
void queen_Reset3GModule( void )
{
  // reload reconnect failure reset times (2 times default)
  WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET;
  // auto restart 3G module every setting time
  WCDMAModule_ResetTimer = (WCDMA_RESETTIMER*60*60/2);
  // set restart timer (time to send reset cmd again)
  WCDMAModule_RestartTimer = WCDMA_RESTARTTIMER;  

  // reset 3g flags
  WCDMAModuleSTEP = WCDMAsetup_NotReady; // 0-x, indicate the different step in setup
  queen_Available = UNAVAILABLE; // set module unavailable
  
  setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter  
  
  // off LED
  HalLedSet (HAL_LED_2, HAL_LED_MODE_ON); // LED2, D3 negative logic
  HalLedSet (HAL_LED_4, HAL_LED_MODE_ON); // LED4, D2 negative logic
  
  myBlockingHalUARTWrite(0,MU609_RESET,13);
}


/*********************************************************************
 * @fn      uartWriteIPINIT
 *
 * @brief   integrate all the IPINIT together
 *
 * @param   none
 *
 * @return  none
 */
void uartWriteIPINIT(void)
{
#ifdef  LYCAMOBILE
        myBlockingHalUARTWrite(0,MU609_IPINIT,49);
#endif
        
#ifdef  CHINAUNICOM
        myBlockingHalUARTWrite(0,MU609_IPINIT,19);
#endif
        
#ifdef  USAMOBILE
        myBlockingHalUARTWrite(0,MU609_IPINIT,29);
#endif
        
#ifdef  USAMOBILE_WYLESS
        myBlockingHalUARTWrite(0,MU609_IPINIT,34);
#endif
}

/*********************************************************************
 * @fn      setWCDMAoneSecondStepTimer
 *
 * @brief   
 *
 * @param   none
 *
 * @return  none
 */
void setWCDMAoneSecondStepTimer(short State, uint8 timeInterval, uint8 allowedFailureTimes)
{
  WCDMAModuleTimerEnable = State;
  WCDMAModuleTimerCounter = timeInterval;
  WCDMAModuleFailureTimes = allowedFailureTimes;
}