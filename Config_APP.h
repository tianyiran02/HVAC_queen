/*********************************************************************
  This .h file contain essential parameters to setup firmware

  Use Macro to define different parameters, for example, reset time,
  queen ID, PAN ID

  @Author Yiran Tian 2015/5/27
*********************************************************************/

#ifndef CONFIG_APP_H
#define CONFIG_APP_H

/*********** Essential ***************/
// Maximun drone number
#define MAXDEVNUM 4 
// Queen ID
#define QUEEN_ID "2F701220"


/********* Detail ***********/
// Reset Value
#define WCDMA_OVERTIMERESET     2       // upload failer over 2 times, 3G reset
#define WCDMAModule_IPINITResend_MAX 10 // IPINIT overtime 10 times, undone react
#define WCDMA_RESTARTTIMER      20      // if not restart in 20s, resend reset cmd
#define WCDMA_RESETTIMER        5       // Reset 3G Module every 5 hours, largest value, 18hours
#define WCDMA_ATPUSHDELAY       5       // AT push resend delay 5s   
#define WCDMA_CFUNRESET         10      // AT+CFUN no responed 10 times, 3g reset.
#define WCDMA_UART_SUSTIME      40      // Suspend UART for 40 * WDT time after send data. avoid overflow

// Others
#define TIMESTAMP_OFFSET        0       // Timestamp offset, 0s per upload

#endif