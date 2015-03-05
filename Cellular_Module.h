/*********************************************************************
  Filename:       Cellular_Modle.h

  @Descrip work with Cellular_Module.c file. provide extern link to 
  GenericApp_Q.c

  @Author Yiran Tian 2015/3/4
*********************************************************************/

#ifndef CELLULAR_MODULE_H
#define CELLULAR_MODULE_H

/*********************************************************************
 * MACROS
 */
// Reset Value
#define WCDMA_OVERTIMERESET     2       // upload failer over 2 times, 3G reset
#define WCDMAModule_IPINITResend_MAX 15 // IPINIT overtime 15 times, undone react
#define WCDMA_RESTARTTIMER      60      // if not restart in 60s, resend reset cmd
#define WCDMA_RESETTIMER        4       // Reset 3G Module every 4 hours, largest value, 18hours
      
#define WCDMA_10SDELAY                  10 // normal time interval, 10s
#define WCDMA_SENDTIME                  20 // normal upload time, 15s  


// Cellular module steps
// Initialize stage
#define WCDMAsetup_NotReady             0  // After power up
#define WCDMAsetup_Connected            1  // Received poweron Msg, send ATE0
#define WCDMAsetup_CURCsend             2  // Received 'OK', send CURC=2,40,40
#define WCDMAsetup_NWTIMEwait           3  // Received 'OK', wait for NWTIME report
#define WCDMAsetup_IPINITsend           4  // Received timestamp, send IPINIT
#define WCDMAsetup_3GReady              5  // Received 'OK', initialize finish
// Send Stage
#define WCDMAsetup_IPOPENsend           6  // Check previos steps, send IPOPEN
#define WCDMAsetup_IPSENDEXsend         7  // Received 'OK', send IPSENDEX
#define WCDMAsetup_OKtoSend             8  // Received 'OK', send data      
#define WCDMAsetup_IPCLOSEsend          9  // Received IPSENDEX ACK, send IPCLOSE
#define WCDMAsetup_SendFinish           10 // Send finish and success. Cheers! (not realy use)
// Others
#define WCDMAsetup_TIMESTAMPUPDATE      12 // Update timestamp
#define WCDMAsetup_TIMESTAMPUPDATES2    13 // Update timestamp step 2       

#define NOT_READY       3
#define AVAILABLE       1
#define UNAVAILABLE     0

#define ENABLE          1
#define DISABLE         0
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 WCDMAModule_ReConOverTries;
extern uint16 WCDMAModule_ResetTimer;
extern uint8 WCDMAModule_RestartTimer;
extern uint8 WCDMAModuleSTEP;
extern short queen_Available;

extern uint8 JSON_TimeStamp[16];
extern uint8 MU609_Sending[333];

/* below are IPINIT */
#ifdef LYCAMOBILE
extern uint8 MU609_IPINIT[49];
#endif

#ifdef CHINAUNICOM
extern uint8 MU609_IPINIT[19];
#endif

#ifdef USAMOBILE
extern uint8 MU609_IPINIT[29];
#endif

#ifdef USAMOBILE_WYLESS
extern uint8 MU609_IPINIT[34];
#endif


/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Cellular main control function, put in UART Msg service. 
 */
extern void Cellular_UART(mtOSALSerialData_t *);

/*
 * Cellular module reset function, send reset CMD and clear flags 
 */
extern void queen_Reset3GModule( void );

/*
 * 1 second system timer service function, essential to control MU609 
 */
extern void Cellular_OneSecondTimerServer( byte, uint32, uint32);

#endif