/*********************************************************************
  Filename:       AT_CMDdata.h

  @Descrip provide all the AT CMD

  @Author Yiran Tian 2015/3/4
*********************************************************************/

#ifndef AT_CMDDATA_H
#define AT_CMDDATA_H

// HUAWEI AT
uint8 MU609_AT[4] = {0x41,0x54,0x0D,0x0A}; 

// HUAWEI ATE0
uint8 MU609_ATE0[6] = {0x41,0x54,0x45,0x30,0x0D,0x0A};

// HUAWEI AT^CURC=2,48,48
uint8 MU609_CURC[17] = {0x41,0x54,0x5E,0x43,0x55,0x52,0x43,0x3D,0x32,0x2C,
                        0x34,0x38,0x2C,0x34,0x38,0x0D,0x0A };

// HUAWEI IPOPEN: "AT^IPOPEN=1,"TCP","70.39.150.200",80"
uint8 MU609_IPOPEN[38] = {0x41,0x54,0x5e,0x49,0x50,0x4F,0x50,0x45,0x4E,0x3D,
                          0x31,0x2C,0x22,0x54,0x43,0x50,0x22,0x2C,0x22,0x37,
                          0x30,0x2E,0x33,0x39,0x2E,0x31,0x35,0x30,0x2E,0x32,
                          0x30,0x30,0x22,0x2C,0x38,0x30,0x0D,0x0A};

// HUAWEI IPCLOSE: "AT^IPCLOSE=1"
uint8 MU609_IPCLOSE[14] = {0x41,0x54,0x5e,0x49,0x50,0x43,0x4C,0x4F,0x53,
                           0x45,0x3D,0x31,0x0D,0x0A};


// HUAWEI RESET: "AT+CFUN=1,1"
uint8 MU609_RESET[13] = {0x41,0x54,0x2B,0x43,0x46,0x55,0x4E,0x3D,0x31,0x2C,
                         0x31,0x0D,0x0A};

// HUAWEI NWTIME: "AT^NWTIME?"
uint8 MU609_NWTIME[12] = {0x41,0x54,0x5E,0x4E,0x57,0x54,0x49,0x4D,0x45,0x3F,
                          0x0D,0x0A};

// HUAWEI IPSENDEX: "AT^IPSENDEX=1,2,333"
uint8 MU609_IPSENDEX[21] = {0x41,0x54,0x5E,0x49,0x50,0x53,0x45,0x4E,0x44,0x45,
                          0x58,0x3D,0x31,0x2C,0x32,0x2C,0x33,0x33,0x33,0x0D,
                          0x0A};

/*IP INIT CMD
 *
 * Select different operator by MACRO
 *
 * 1. UK lycamobile version: define LYCAMOBILE
 * 2. China UNICOM version: define CHINAUNICOM
 * 3. USA version: define USAMOBILE
 * 4. USA wyless version: define USAMOBILE_WYLESS
 * 5. UK giffgaff version: define GIFFGAFF
 */

#ifdef LYCAMOBILE
// HUAWEI Init data: "AT^IPINIT="data.lycamobile.co.uk","lmuk","plus""
uint8 MU609_IPINIT[49] = {0x41,0x54,0x5e,0x49,0x50,0x49,0x4e,0x49,0x54,0x3d,
                          0x22,0x64,0x61,0x74,0x61,0x2e,0x6c,0x79,0x63,0x61,
                          0x6d,0x6f,0x62,0x69,0x6c,0x65,0x2e,0x63,0x6f,0x2e,
                          0x75,0x6b,0x22,0x2c,0x22,0x6c,0x6d,0x75,0x6b,0x22,
                          0x2c,0x22,0x70,0x6c,0x75,0x73,0x22,0x0d,0x0a};
#endif

#ifdef CHINAUNICOM
// HUAWEI Init data: "AT^IPINIT="3GNET""
uint8 MU609_IPINIT[19] = {0x41,0x54,0x5e,0x49,0x50,0x49,0x4E,0x49,0x54,0x3D,
                          0x22,0x33,0x47,0x4E,0x45,0x54,0x22,0x0D,0x0A};
#endif

#ifdef USAMOBILE
// HUAWEI Init data:"AT^IPINIT="epc.tmobile.com""
uint8 MU609_IPINIT[29] = {0x41,0x54,0x5e,0x49,0x50,0x49,0x4e,0x49,0x54,0x3d,
                          0x22,0x65,0x70,0x63,0x2e,0x74,0x6d,0x6f,0x62,0x69,
                          0x6c,0x65,0x2e,0x63,0x6f,0x6d,0x22,0x0d,0x0a};
#endif

#ifdef USAMOBILE_WYLESS
// HUAWEI Init data:"AT^IPINIT="telargo.t-mobile.com""
uint8 MU609_IPINIT[34] = {0x41,0x54,0x5e,0x49,0x50,0x49,0x4E,0x49,0x54,0x3D,
                          0x22,0x74,0x65,0x6C,0x61,0x72,0x67,0x6F,0x2E,0x74,
                          0x2D,0x6D,0x6F,0x62,0x69,0x6C,0x65,0x2E,0x63,0x6F,
                          0x6D,0x22,0x0D,0x0A};
#endif

#ifdef GIFFGAFF
uint8 MU609_IPINIT[26] = {0x41,0x54,0x5e,0x49,0x50,0x49,0x4E,0x49,0x54,0x3D,
                          0x22,0x67,0x69,0x66,0x66,0x67,0x61,0x66,0x66,0x2e,
                          0x63,0x6f,0x6d,0x22,0x0D,0x0A};
#endif

// Get command 333
uint8 __xdata MU609_Sending[333] = {0x47,0x45,0x54,0x20,0x68,0x74,0x74,0x70,0x3A,0x2F,
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
0x20,0x2A,0x2F,0x2A,0x0D,0x0A,0x0D,0x0A};

// HUAWEI AT ACK 
// because using Boyer Moore Algoritm, no need to save the whole string now.
// This can save ROM space, also save XDATA STACK space while running program.

char const __code MU609_ATE0_ACK[9] = {0x41,0x54,0x45,0x30,0x0D,0x0D,0x0A,0x4F,0x00}; // ATE0   OK

char const __code MU609_CURC_ACK[13] = {0x41,0x54,0x5E,0x43,0x55,0x52,0x43,0x3D,0x32,0x2C,
                                0x34,0x38,0x00};

char const __code MU609_READY_MSG[14] = {0x0D,0x0A,0x5E,0x53,0x59,0x53,0x53,0x54,0x41,0x52,
                                  0x54,0x0D,0x0A,0x00}; //^SYSSTART

char const __code MU609_ACK[7] = {0x0D,0x0A,0x4F,0x4B,0x0D,0x0A,0x00}; //OK

char const __code MU609_ERROR_ACK[10] = {0x0D,0x0A,0x45,0x52,0x52,0x4F,0x52,0x0D,0x0A,0x00};//ERROR

char const __code MU609_CCLK_ACK[10] = {0x0D,0x0A,0x2B,0x43,0x43,0x4C,
                                 0x4B,0x3A,0x20,0x00}; // +CCLK: 

char const __code MU609_NWTIME_ACK[10] = {0x0D,0x0A,0x5E,0x4E,0x57,0x54,0x49,
                                    0x4D,0x45,0x00}; // ^NWTIME

char const __code MU609_SRVST_ACK[9] = {0x0D,0x0A,0x5E,0x53,0x52,0x56,0x53,
                                    0x54,0x00}; // ^SRVST

char const __code MU609_IPSENDEX_ACK[13] = {0x0D,0x0A,0x5E,0x49,0x50,0x53,0x45,0x4E,0x44,0x45,
                                    0x58,0x3A,0x00}; // ^IPSENDEX: 1...

char const __code MU609_CME_ERROR[9] = {0x0D,0x0A,0x2B,0x43,0x4D,0x45,0x20,0x45,0x00}; // +CME ERROR

char const __code MU609_IPOPEN_LINKEXIST[13] = {0x4F,0x52,0x3A,0x20,0x54,0x68,0x65,0x20,0x6C,0x69,
                                          0x6E,0x6B,0x00}; //OR: The link
#endif