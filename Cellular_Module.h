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
#define WCDMA_10SDELAY                  10 // normal time interval, 10s
#define WCDMA_20SDELAY                  20 // normal time interval, 20s        
#define WCDMA_SENDTIME                  60 // acceptable upload time, 60s  
#define WCDMA_120SDELAY                 120 // normal time interval, 120s        


// Cellular module steps
// Initialize stage
#define WCDMAsetup_NotReady             0  // After power up
#define WCDMAsetup_Connected            1  // Received poweron Msg, send ATE0
#define WCDMAsetup_ATE0send             2  // Received 'OK', send CURC=2,40,40
#define WCDMAsetup_NWTIMEwait           3  // Received 'OK', wait for NWTIME report
#define WCDMAsetup_IPINITsend           4  // Received timestamp, send IPINIT
#define WCDMAsetup_3GReady              5  // Received 'OK', initialize finish
// Send Stage
#define WCDMAsetup_ATGO                 6  // Send AT push CMD
#define WCDMAsetup_IPOPENsend           7  // Check previos steps, send IPOPEN
#define WCDMAsetup_IPSENDEXsend         8  // Received 'OK', send IPSENDEX
#define WCDMAsetup_OKtoSend             9  // Received 'OK', send data      
#define WCDMAsetup_IPCLOSEsend          10  // Received IPSENDEX ACK, send IPCLOSE
#define WCDMAsetup_SendFinish           11 // Send finish and success. Cheers! (not realy use)
// Others
#define WCDMAsetup_TIMESTAMPUPDATE      12 // Update timestamp
     

   
//Error Code
#define No_Error                0 // No error
#define NWTIMEFail              1 // Fail to obtain NWTIME, flash signal LED for 0.25s/10s/10% then reset
   
#define NOT_READY       3
#define AVAILABLE       1
#define UNAVAILABLE     0

#define ENABLE          1
#define DISABLE         0

#define NoService       0
#define ResServices     1       // Restricted Services
#define ValServices     2       // Valid Services
#define ResReServices   3       // Restricted Regional Service
#define Hibernate       4       


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 WCDMAModuleSTEP;
extern short queen_Available;
extern uint8 ErrCode;

extern uint8 JSON_TimeStamp[16];
extern uint8 MU609_Sending[333];

extern uint8 MU609_AT[4];
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

#ifdef GIFFGAFF
extern uint8 MU609_IPINIT[26];
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

/*
 * 1 second system timer service function, essential to control MU609 
 */
extern void setWCDMAoneSecondStepTimer(short, uint8, uint8);
#endif