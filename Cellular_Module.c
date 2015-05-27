/*********************************************************************
  This .c file provide essential function to control Cellular Module

  Function including initialize, send and all sub function needed in 
  main GenericApp.c file.

Coupling with other module description:
Lower level: UART must provide myBlockingHalUARTWrite(;;) function.


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
#include "Config_APP.h"
#include "BM_StringMatch.h"
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
// 0-x, indicate the different step in setup
uint8 WCDMAModuleSTEP = 0; 

// Signal indicator
static uint8 WCDMASignalState = NoService;

// if signal recover, set unavailable to available
static uint8 WCDMA_SignalRecover = 0;

// Error Code
uint8 ErrCode = No_Error;

// ----------------- counter -----------------
static uint16 WCDMAModule_ResetTimer = (WCDMA_RESETTIMER*60*60/2);
// After device setup, cellular module will automatically restart every setting time

static uint8 WCDMAModule_RestartTimer = FALSE;
// After MC send reset CMD, timer start. Once the cellular module restart, 
// timer clear. If overtime, resend CMD. 
// This time can be also used to perform a reset in certain delay.

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
static void uartWriteIPINIT(void);
static short ACK_BMStringSearch(BMStringMatching_t, const char *, uint8 *);

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
   *    1. Start-ATE0 duration
   *       After microcontroller received power on message, this timer will start.
   *       Normally only few second will takes to go through this process. The 
   *       timer was used to recover system from unknown error.
   *
   *       restart MU609;
   *
   *    2. NWTIME response waiting
   *       After power up, the device will automatically connect to the internet
   *       and obtain the Network time from server. However, if the signal is 
   *       poor or the local ISP doen't provide NWTIME service, this process 
   *       will not able to finish. 
   *       After receiving ATE0 response, this timer will start with a inital 
   *       time (default 120s). Once the device obtain a valid service, the 
   *       time will cut to 20s. This will improve the error response time.
   *
   *       reset cellular module; 
   *    3. IPINIT overtime WCDMA_10SDELAY * WCDMAModule_IPINITResend_MAX
   *       Its normal to have a CME error response even in good signal condition,
   *       the IPINIT will be resend untill it reach a maximum attempts and then
   *       Module will reset.
   *       
   *       resend IPINIT;
   *       reset cellular module;
   *
   *    4. AT push re-send (start a upload process)
   *       If the Module is in a abnormal condition, i.e processing other CMD, 
   *       the AT push may not be received by Module. Resend AT command. It the
   *       Problem doesn't solve, reset the module.
   *
   *       resend AT;
   *       reset cellular module;
   *
   *    5. One upload period WCDMA_SENDTIME * WCDMAModule_ReConOverTries(WCDMA_OVERTIMERESET)
   *       Normally a whole upload will only take few seconds. This timer was 
   *       design to avoid some unknown situation that will interrupt the upload
   *       process. Once time out, upload will be abandon. And an error upload
   *       will be loged. Any success upload after that will indicate that the 
   *       environment back to normal and will reset the log. Once the error has
   *       exceed a certain number, the module will reset.
   *
   *       abandon current upload, wait for next try;
   *       reset cellular module;
   *
   *    6. IPCLOSE overtime
   *       In poor signal condision, IPCLOSE might fail, and this timer was designed
   *       to resend IPCLOSE CMD. After certain number of attempts, the module 
   *       will reset.
   *
   *       resent IPCLOSE;
   *       reset cellular module;
   *
   *    7. NWTIME overtime
   *       In poor signal condition, the NWTIM might fail, this timer will
   *       resend NWTIME CMD. After certain number of attempts, the module will
   *       reset.
   *    
   *       resent NWTIME;
   *       reset cellular module;
   */
  if(WCDMAModuleTimerEnable) // timer enable
  {
    WCDMAModuleTimerCounter --;
    
    if(WCDMAModuleTimerCounter == 0) // timeout
    {
      /* react according to different steps
       */
      
      // 1.Start-ATE0 duration
      if((WCDMAModuleSTEP > WCDMAsetup_NotReady) && (WCDMAModuleSTEP < WCDMAsetup_NWTIMEwait))
      {
        queen_Reset3GModule(); // reset cellular module
      }
      
      // 2. NWTIME response waiting 
      else if(WCDMAModuleSTEP == WCDMAsetup_NWTIMEwait)
      {
        ErrCode = NWTIMEFail;
        setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter 
        HalLedBlink( HAL_LED_2, 4, 90, 1000 ); // flash 0.25s/10s/10%
        
        // set the restart timer to restart in 10s
        WCDMAModule_RestartTimer = WCDMA_10SDELAY;
      }
      
      // 3. IPINIT sending steps
      else if(WCDMAModuleSTEP == WCDMAsetup_IPINITsend) 
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
      
      // 4. AT PUSH resend
      else if(WCDMAModuleSTEP == WCDMAsetup_ATGO)
      {
        WCDMAModuleFailureTimes --; // how many time it fail?
        
        if(WCDMAModuleFailureTimes) // still trying
        {
          WCDMAModuleTimerCounter = WCDMA_10SDELAY;
          myBlockingHalUARTWrite(0,MU609_AT,4); // re-send AT
        }
        else // send failed
        {
          queen_Reset3GModule(); // reset cellular module
        } 
        
      }
      
      // 5. send during time
      else if((WCDMAModuleSTEP > WCDMAsetup_ATGO) 
              && (WCDMAModuleSTEP < WCDMAsetup_IPCLOSEsend))
      {
        WCDMAModule_ReConOverTries --;
        if(WCDMAModule_ReConOverTries == 0) // if happen more than default times
        {
          queen_Reset3GModule(); // reset cellular module
        }
        else
        {
          // back to ready status, wait for another try.
          WCDMAModuleSTEP = WCDMAsetup_3GReady; // wait for next try
          queen_Available = AVAILABLE; 
          setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
        }       
      }
      
      // 6. IPCLOSE sending step
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
      
      // 7. NWTIME overtime
      else if(WCDMAModuleSTEP == WCDMAsetup_TIMESTAMPUPDATE)
      {
        WCDMAModuleFailureTimes --; // how many time it fail?
      
        if(WCDMAModuleFailureTimes)
        {
          WCDMAModuleTimerCounter = WCDMA_20SDELAY; // reload timer
          myBlockingHalUARTWrite(0,MU609_NWTIME,12);// resend NWTIME 
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
   * If any error occur, error indicator perform in higher priority. If not,
   * update LED according to current status.
   * LED2, D3, Signal Indicator LED
   * LED4, D2, Cellular Module Status LED
   *
   */
  
  /* If any error happen, check the error code can make the device has proper
     react. This version only contain limited error */
  // Any error?
  if(ErrCode)
  {
    // For now, do almost nothing. The only error indicator was for NWTIME. And the 
    // LED flashing is already set in timer(the place detect error).
    switch(ErrCode)
    {
    case NWTIMEFail:
       HalLedBlink( HAL_LED_2, 4, 90, 1000 ); // flash 0.25s/10s
       break;
       
    default:
      break;
    }
  }
  
  // No error, update LED according to status
  else
  {
  /* LED2, D3, Signal Indicator LED */

  // 1. No Service, LED OFF
  if((WCDMASignalState == NoService) || (WCDMASignalState == Hibernate)) 
  {
    // if queen available, set unavailable to avoid upload
    if(WCDMAModuleSTEP >= WCDMAsetup_3GReady)
    {
      queen_Available = UNAVAILABLE;
      WCDMA_SignalRecover = 1;
    }
    HalLedSet (HAL_LED_2, HAL_LED_MODE_ON); // off LED2, internet LED
  }
  
  // 2. Has signal, but services restricted, flash LED
  else if(WCDMASignalState != ValServices)
  {
    if(WCDMAModuleSTEP >= WCDMAsetup_3GReady)
    {
      queen_Available = UNAVAILABLE;
      WCDMA_SignalRecover = 1;
    }
    HalLedBlink( HAL_LED_2, 2, 50, 1000 ); // flash 1s
  }
  
  // 3. Signal Normal, LED ON
  else
  {
    if((WCDMAModuleSTEP >= WCDMAsetup_3GReady) && (WCDMA_SignalRecover) && (queen_Available == UNAVAILABLE))
    {
      WCDMA_SignalRecover = 0;
      queen_Available = AVAILABLE;
    }
    
    // wait for signal while waiting NWTIME RESPONSE at initial stage. 
    // Once got signal, set restart waiting time to 20s
    // Decrease response time when non-signal error during initial
    if((WCDMAModuleSTEP == WCDMAsetup_NWTIMEwait) && (WCDMAModuleTimerCounter > 20))
    {
      // set restart timer to a smaller value
      // the original should be more than this (count down from 120s)
      setWCDMAoneSecondStepTimer(ENABLE,WCDMA_20SDELAY,0);
    }
    
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
  } // if(ErrCode)
  
  
  
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
          
          if((queen_Available == AVAILABLE) && (WCDMAModuleSTEP == WCDMAsetup_3GReady))
          {         
            // set unavailable
            queen_Available = UNAVAILABLE;
            
            // set timer
            setWCDMAoneSecondStepTimer(ENABLE,WCDMA_20SDELAY,6); 
            
            // set steps to time update
            WCDMAModuleSTEP = WCDMAsetup_TIMESTAMPUPDATE;
            
            // send NWTIME to push
            myBlockingHalUARTWrite(0,MU609_NWTIME,12);
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
  uint8 matchResultTemp[4] = {0};
  
  char MU609_BUF[45] = {0}; 
   
  BMStringMatching_t BMSearchTemp; // This is for the rest
  
#ifdef REACTDRONE
  uint8 uartbuf;
  uint16 deviceshortaddr = 0;
#endif  
  
  osal_memcpy(MU609_BUF,&CMDMsg->msg[1],CMDMsg->msg[0]);
  // Copy data to buffer
  
  BMSearchTemp.PatternAddr = MU609_SRVST_ACK; // Initialize SVRST Matching
  BMSearchTemp.PatternLength = 8;
  BMSearchTemp.StringAddr = MU609_BUF;
  BMSearchTemp.StringLength = 45;
  BMSearchTemp.MatchingPoint = matchResultTemp;
  BMSearchTemp.MatchFlag = 0;
  
  
  /* Signal detection
   *
   * By receiving SRVST report, the TE will know the signal status
   * 0 - no service      1 - Restricted services
   * 2 - Valid service   3 - Restricted regional service 
   * 4 - Power saving or hibernated 
   *
   **/
  if(BMsearch(BMSearchTemp)) // signal update information 
  {
    /* update signal status */
    // update signal using the first matching point
    WCDMASignalState = MU609_BUF[matchResultTemp[0] + 10] - 48;
      
    // update by other matching point
    for(i = 1; i <=3; i++)
    {
      if(matchResultTemp[i] != 0)
      {
        WCDMASignalState = MU609_BUF[matchResultTemp[i] + 10] - 48;
      }
    }
  }


  /* Start state machine */
  BMSearchTemp.StringAddr = MU609_BUF;
  BMSearchTemp.StringLength = strlen(MU609_BUF);
  
  switch(WCDMAModuleSTEP)
  {
    // -------------------- init ----------------------- 
    
    // 1. p:After power up 
  case WCDMAsetup_NotReady: 
    if(ACK_BMStringSearch(BMSearchTemp,MU609_READY_MSG,matchResultTemp))
    {
      // Restart detected, disable restart timer.
      WCDMAModule_RestartTimer = FALSE; 
      
      // Start a new timer. Start from here to receive ATE0 response
      setWCDMAoneSecondStepTimer(ENABLE,WCDMA_20SDELAY,0);
        
      myBlockingHalUARTWrite(0,MU609_CURC,17); // send CURC
      WCDMAModuleSTEP = WCDMAsetup_Connected; // change state
    }
    break;
    
    // 2. p:Received poweron Msg, send CURC
  case WCDMAsetup_Connected:   
    if(ACK_BMStringSearch(BMSearchTemp,MU609_CURC_ACK,matchResultTemp))
    {
      myBlockingHalUARTWrite(0,MU609_ATE0,6); // send ATE0
      WCDMAModuleSTEP = WCDMAsetup_ATE0send; // change state  
    }
    break;
  
    // 3.   p:Received 'OK', send ATE0
  case WCDMAsetup_ATE0send:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_ATE0_ACK,matchResultTemp))
    {
      WCDMAModuleSTEP = WCDMAsetup_NWTIMEwait;
      
      // Start timer/counter
      // extra delay, wait for searching signal
      setWCDMAoneSecondStepTimer(ENABLE,WCDMA_120SDELAY,0);
    }
    break;
    
    // 4. p:Received 'OK', wait for NWTIME report
  case WCDMAsetup_NWTIMEwait:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_NWTIME_ACK,matchResultTemp)) // check NWTIME, update timestamp
    {
      // Stop previews timer, start new one
      setWCDMAoneSecondStepTimer(ENABLE,WCDMA_10SDELAY,WCDMAModule_IPINITResend_MAX);
     
      JSON_TimeStamp[0] = '2'; // update timestamp
      JSON_TimeStamp[1] = '0';
      
      // use the first matching point to obtain the time
      for(i=2;i<=15;i++)
        JSON_TimeStamp[i] = MU609_BUF[matchResultTemp[0] + i + 9];
          
      uartWriteIPINIT(); // send IPINIT
                 
      WCDMAModuleSTEP = WCDMAsetup_IPINITsend; //change state
    }
    break;
    
    // 5. p:Received timestamp, send IPINIT
  case WCDMAsetup_IPINITsend:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_ACK,matchResultTemp))
    {
      setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
      queen_Available = AVAILABLE; // set Queen upload states parameters (may imerge to others)
      
      WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state, initialize finish
    }
    break;
    
    // -------------------- Send -----------------------
    
    // 6. p: Received 'OK', initialize finish
  case WCDMAsetup_ATGO:  
    if(ACK_BMStringSearch(BMSearchTemp,MU609_ACK,matchResultTemp) && (queen_Available == UNAVAILABLE))
    {
      // Start new timer/counter, this is to count the uploading duration time.
      // Timer ends when send IPCLOSE
      setWCDMAoneSecondStepTimer(ENABLE,WCDMA_SENDTIME,0);
      
      myBlockingHalUARTWrite(0,MU609_IPOPEN,38); // send IPOPEN
       
      WCDMAModuleSTEP = WCDMAsetup_IPOPENsend; // change state
    }
    break;
    
    // 7. p: Check previos steps, send IPOPEN
  case WCDMAsetup_IPOPENsend:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_ACK,matchResultTemp))
    {
      myBlockingHalUARTWrite(0,MU609_IPSENDEX,21); // send IPSENDEX
      
      WCDMAModuleSTEP = WCDMAsetup_IPSENDEXsend; // change state
    }
    else // error condition
    { 
      if(ACK_BMStringSearch(BMSearchTemp,MU609_CME_ERROR,matchResultTemp))
      {
        // error type1, link already established. Upload directly
        if(ACK_BMStringSearch(BMSearchTemp,MU609_IPOPEN_LINKEXIST,matchResultTemp))
        {
          // In this case, process continue
          myBlockingHalUARTWrite(0,MU609_IPSENDEX,21); // send IPSENDEX
        
          WCDMAModuleSTEP = WCDMAsetup_IPSENDEXsend; // change state
        }
        
        // error type2, poor signal. Normal problem. Upload aboard  
        else
        {
          setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
          
          WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
          WCDMAModule_ReConOverTries --; 
          
          if(WCDMASignalState == ValServices)
            queen_Available = AVAILABLE; // set queen states parameter
        }
      }
    }
    break;
    
    // 8. p: Received 'OK', send IPSENDEX
  case WCDMAsetup_IPSENDEXsend:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_ACK,matchResultTemp))
    {
      // send data
      myBlockingHalUARTWrite(0,MU609_Sending,100); 
      myBlockingHalUARTWrite(0,&MU609_Sending[100],100);
      myBlockingHalUARTWrite(0,&MU609_Sending[200],100);
      myBlockingHalUARTWrite(0,&MU609_Sending[300],33);
      
      WCDMAModuleSTEP = WCDMAsetup_OKtoSend; // Change state
    }
    else // error condition
    {     
      if(ACK_BMStringSearch(BMSearchTemp,MU609_CME_ERROR,matchResultTemp))
      {
        setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
        
        WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
        WCDMAModule_ReConOverTries --; 
        
        if(WCDMASignalState == ValServices)
          queen_Available = AVAILABLE; // set queen states parameter
      }
    }
    break;
    
    // 9. p: Received 'OK', send data 
  case WCDMAsetup_OKtoSend:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_IPSENDEX_ACK,matchResultTemp))
    {
      myBlockingHalUARTWrite(0,MU609_IPCLOSE,14); // send IPCLOSE
      setWCDMAoneSecondStepTimer(ENABLE,WCDMA_10SDELAY,10);// Start timer/counter// set resend flag
      
      WCDMAModuleSTEP = WCDMAsetup_IPCLOSEsend; // change state
    }
    else
    {    
      if(ACK_BMStringSearch(BMSearchTemp,MU609_CME_ERROR,matchResultTemp))
      {
        setWCDMAoneSecondStepTimer(DISABLE,0,0);// clear timer/counter
        
        WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
        WCDMAModule_ReConOverTries --; 
        
        if(WCDMASignalState == ValServices)
          queen_Available = AVAILABLE; // set queen states parameter
      }
    }
    break;
    
    // 10. p: Received IPSENDEX ACK, send IPCLOSE
  case WCDMAsetup_IPCLOSEsend:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_ACK,matchResultTemp))
    {
      setWCDMAoneSecondStepTimer(DISABLE,0,0); //clear resend flag/counter
      HalLedBlink( HAL_LED_4, 4, 50, 500 ); // Flash LED, send success
      LEDAvoidControl = 2; // avoid timer control LED for next 2s
      
      WCDMAModuleSTEP = WCDMAsetup_3GReady; // change state  
      WCDMAModule_ReConOverTries = WCDMA_OVERTIMERESET; 
      
      if(WCDMASignalState == ValServices)
        queen_Available = AVAILABLE; // set queen states parameter
    }
    break;
    
    // 12. update timetamp stage 2
  case WCDMAsetup_TIMESTAMPUPDATE:
    if(ACK_BMStringSearch(BMSearchTemp,MU609_NWTIME_ACK,matchResultTemp)) // check NWTIME, update timestamp
    {
      // Stop timer/counter
      setWCDMAoneSecondStepTimer(DISABLE,0,0); //clear resend flag/counter
      
      JSON_TimeStamp[0] = '2'; // update timestamp
      JSON_TimeStamp[1] = '0';
      
      for(i=2;i<=15;i++)
        JSON_TimeStamp[i] = MU609_BUF[matchResultTemp[0] + i + 9];
      
      WCDMAModuleSTEP = WCDMAsetup_3GReady; //change state
      
      if(WCDMASignalState == ValServices)
        queen_Available = AVAILABLE; // set queen states parameter
    }
    break;
    
  default:
    break;
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
  // set error code as default - no error
  ErrCode = No_Error;
    
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
#ifdef  GIFFGAFF
        myBlockingHalUARTWrite(0,MU609_IPINIT,26);
#endif
  
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
 * @brief   Set the parameters in the one second timer.
 *
 * @param       State - ENABLE or DISABLE
 *              timeInterval - Counter loading time
 *              allowedFailureTimes - repeatance
 *
 * @return  none
 */
void setWCDMAoneSecondStepTimer(short State, uint8 timeInterval, uint8 allowedFailureTimes)
{
  WCDMAModuleTimerEnable = State;
  WCDMAModuleTimerCounter = timeInterval;
  WCDMAModuleFailureTimes = allowedFailureTimes;
}

/*********************************************************************
 * @fn      BMStringSearch
 *
 * @brief   Set up structure and call the BMsearch function. 
 *
 * @param       BMStructure - the structure used to do BM searching
 *              pat - pattern address
 *              string - string address
 *              matchResult - Maching point result
 *
 * @return  
 */
short ACK_BMStringSearch(BMStringMatching_t BMStructure, const char *pat, uint8 *matchResult)
{
  // clear buffer before use
  osal_memset(matchResult,'0',sizeof(matchResult));
    
  // Configure structure
  BMStructure.PatternAddr = pat;
  BMStructure.PatternLength = strlen(pat);
  BMStructure.MatchingPoint = matchResult;
  
  // return matching result
  return (BMsearch(BMStructure));
}