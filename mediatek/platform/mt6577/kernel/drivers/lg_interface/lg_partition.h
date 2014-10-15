#ifndef _LG_PARTITION_H
#define __LG_PARTITION_H
/*Data layout & length*/
//                                                                                            

#define LGE_FAC_PID_PART_1_LEN 22
#define LGE_FAC_PID_PART_2_LEN 10
#define LGE_FAC_PID_LEN ( LGE_FAC_PID_PART_1_LEN + LGE_FAC_PID_PART_2_LEN ) /* decimal(22) + ASCII(10) */
//                                                                                            
#define LGE_FAC_HEADER_IDX                        0
#define LGE_FAC_HEADER_SIZE                        4

#define LGE_FAC_DATA_VER_IDX                    2     /* facDataVersion */
#define LGE_FAC_DATA_VER_SIZE                    4

#define LGE_FAC_PID_IDX                            4     /* facPid */
#define LGE_FAC_PID_SIZE                        LGE_FAC_PID_LEN

#define LGE_FAC_BT_IDX                            6     /* facBtAddr */
#define LGE_FAC_BT_SIZE                            6

#define LGE_FAC_IMEI_IDX                        8     /* facImei */
#define LGE_FAC_IMEI2_IDX                        9     /* facImeiSlave */
#define LGE_FAC_IMEI3_IDX                        10     /* temp index for the future */
#define LGE_FAC_IMEI4_IDX                        11     /* temp index for the future */
#define LGE_FAC_IMEI_SIZE                        15

#define LGE_FAC_NETWORK_CODE_LIST_IDX           14     /* facNetworkCodeListNum */
#define LGE_FAC_NETWORK_CODE_LIST_SIZE          4

#define LGE_FAC_SIMLOCK_TYPE_IDX                16     /* facSimLockType */
#define LGE_FAC_SIMLOCK_TYPE_SIZE                4

#define LGE_FAC_FUSG_FLAG_IDX                    18    /* facFusgFlag */
#define LGE_FAC_FUSG_FLAG_SIZE                    1

#define LGE_FAC_IMPL_FLAG_IDX                    20    /* facImplFlag */
#define LGE_FAC_IMPL_FLAG_SIZE                    1

#define LGE_FAC_ATCMDLOCK_FLAG_IDX                22    /* facAtCmdLockFlag */
#define LGE_FAC_ATCMDLOCK_FLAG_SIZE                1

#define LGE_FAC_UNLOCKCODE_VERIFY_FAIL_IDX        24    /* facUnlockCodeVerifyFailCount */
#define LGE_FAC_UNLOCKCODE_VERIFY_FAIL_SIZE     1

#define LGE_FAC_UNLOCKCODE_FAIL_COUNT_IDX        26    /* facUnlockFailCount */
#define LGE_FAC_UNLOCKCODE_FAIL_COUNT_SIZE        sizeof(FactoryUnlockFailCount)

#define LGE_FAC_WIFI_IDX                        28    /* facWifiMacAddr */
#define LGE_FAC_WIFI_SIZE                        6

/***Reunited UNLOCK CODE***/
#define LGE_FAC_UNLOCK_CODE_IDX                    30
#define LGE_FAC_NETWORK_CODE_SIZE                sizeof(FactoryNetworkCode)*LGE_FAC_MAX_NETWORK_CODE_LIST_NUM
#define LGE_FAC_UNLOCK_CODE_COMMON_SIZE            16
/***Reunited UNLOCK CODE***/

#define LGE_FAC_WEB_DOWNLOAD_IDX                32
#define LGE_FAC_WEB_DOWNLAOD_SIZE                1

#define LGE_FAC_LCD_CALIBRATION_IDX                34
#define LGE_FAC_LCD_CALIBRATION_SIZE            4

#define LGE_FAC_CALL_DURATION_IDX                36    /* FacDataCallDuration */
#define LGE_FAC_CALL_DURATION_SIZE                sizeof(unsigned long)*2    /* Master/ Slave */


#define LGE_FAC_FACTORY_RESET_IDX                38    /*FacFactoryResetStatusFlag*/
#define LGE_FAC_FACTORY_RESET_SIZE                1

#define LGE_FAC_AUTO_RESULT01_IDX                40    /* FacDataAllAutoResult */
#define LGE_FAC_AUTO_RESULT01_SIZE                32    // 4

#define LGE_FAC_AUTO_RESULT02_IDX                41    /* FacDataAllAutoResult */
#define LGE_FAC_AUTO_RESULT02_SIZE                32    // 4

#define LGE_FAC_AUTO_RESULT03_IDX               42    /* FacDataAllAutoResult */
#define LGE_FAC_AUTO_RESULT03_SIZE                100    // 4
#define LGE_FAC_AAT_ORDERSET_IDX                43
#define LGE_FAC_AAT_ORDERSET_SIZE                 100

#define LGE_FAC_SOFTWARE_VER_IDX                44        /* FacSoftwareVersion */
#define LGE_FAC_SOFTWARE_VER_SIZE                100

#define LGE_FAC_ORIGINAL_SOFTWARE_VER_IDX                45        /* FacSoftwareVersion */
#define LGE_FAC_ORIGINAL_SOFTWARE_VER_SIZE                100

#define LGE_FAC_NETWORK_CODE_IDX01                46    /* facNetworkCode */
#define LGE_FAC_NETWORK_CODE_SIZE01                sizeof(FactoryNetworkCode)*16

#define LGE_FAC_NETWORK_CODE_IDX02                47    /* facNetworkCode */
#define LGE_FAC_NETWORK_CODE_SIZE02                sizeof(FactoryNetworkCode)*16

#define LGE_FAC_NETWORK_CODE_IDX03                48    /* facNetworkCode */
#define LGE_FAC_NETWORK_CODE_SIZE03                sizeof(FactoryNetworkCode)*8

#define LGE_FAC_PROXIMITY_CALIBRATION_IDX       50
#define LGE_FAC_PROXIMITY_CALIBRATION_SIZE        4

#define LGE_FAC_ACCELEROMETER_CALIBRATION_IDX     52    /*Reserved - It is not used now, and it will be used after decide SIZE(4 is temp size)*/
#define LGE_FAC_ACCELEROMETER_CALIBRATION_SIZE    sizeof(unsigned int)*3    //12

#define LGE_FAC_RESERVED01_CALIBRATION_IDX         54    /*Reserved - It is not used now, and it will be used after decide SIZE(4 is temp size)*/
#define LGE_FAC_RESERVED01_CALIBRATION_SIZE        4

#define LGE_FAC_RESERVED02_CALIBRATION_IDX         56    /*Reserved - It is not used now, and it will be used after decide SIZE(4 is temp size)*/
#define LGE_FAC_RESERVED02_CALIBRATION_SIZE        4

#define LGE_FAC_RESERVED03_CALIBRATION_IDX         58    /*Reserved - It is not used now, and it will be used after decide SIZE(4 is temp size)*/
#define LGE_FAC_RESERVED03_CALIBRATION_SIZE        4

#define LGE_FAC_HARDWARE_VERSION_IDX        60    /* facHardwareVersion */
#define LGE_FAC_HARDWARE_VERSION_SIZE        4

#define LGE_FAC_QEM_IDX                            62    /*FacQEM - After find alternative storage, it will be removed */
#define LGE_FAC_QEM_SIZE                        1

#if 1 // ADD_SWFV
#define LGE_FAC_FIXED_SOFTWARE_VER_IDX                63        /* FacSoftwareVersion */
#define LGE_FAC_FIXED_SOFTWARE_VER_SIZE                100
#endif

#define LGE_FAC_TAIL_IDX                        4095
#define LGE_FAC_TAIL_SIZE                        4
//                                                                                            
//                                                                                           
#define LGE_FAC_MAX_NETWORK_CODE_LIST_NUM 2 /* This number may be increased in the future */
//                                                                                           

#define LGE_FAC_MAX_CALL_DURATION_TO_SAVE    (10*60*60)

#if 0
/* DATA layout*/
#define LGE_FAC_WIFI_MAC_ADDR_OFFSET    (1)
#define LGE_FAC_BT_ADDR_OFFSET (2)
#define LGE_FAC_IMEI_MASTER_OFFSET    (3)
#define LGE_FAC_IMEI_NOT_MASTER_OFFSET (4)
#define LGE_FAC_SIM_LOCK_TYPE_OFFSET    (5)
#define LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET    (6)
#define LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET    (7)
#define LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET    (8)
#define LGE_FAC_UNLOCK_CODE_OFFSET    (9)
#define LGE_FAC_VERIFY_UNLOCK_CODE_OFFSET    (10)
#define LGE_FAC_UNLOCK_CODE_VALIDNESS_OFFSET    (11)
#define LGE_FAC_NETWORK_CODE_OFFSET    (12)
#define LGE_FAC_NETWORK_CODE_VALIDNESS_OFFSET    (13)
#define LGE_FAC_INIT_SIM_LOCK_DATA_OFFSET    (14)
#define LGE_FAC_FUSG_FLAG_OFFSET    (15)
#define LGE_FAC_DATA_VERSION_OFFSET    (16)
#define LGE_FAC_PID_OFFSET    (17)
#define LGE_FAC_SOFTWARE_VERSION_OFFSET (18)
//                                                                                                
#define LGE_FAC_QEM_OFFSET (19)
#define LGE_FAC_FACTORY_RESET_OFFSET (20)
//                                                                                                
#endif

/*data length*/
//                                  
#define LGE_FAC_FUSG_FLAG    1
//                                          
//                                                      
#define LGE_FAC_VERIFY_UNLOCK_CODE_LEN    1
//                                              
//                                      
//                                                     
//                                             
//                                     
#define LGE_FAC_SUFFIX_STR_LEN (15)
#define LGE_FAC_NC_MCC_LEN 3
#define LGE_FAC_NC_MNC_LEN 3
#define LGE_FAC_NC_GID1_LEN 8
#define LGE_FAC_NC_GID2_LEN 8
#define LGE_FAC_NC_SUBSET_LEN 2
//                                                                                                                                                
#define LGE_FAC_SV_LEN    60
//                               
//                                                                                           
#define LGE_FAC_MAX_NETWORK_CODE_LIST_NUM (2)  /* This number may be increased in the future */
//                                                                                           
#define LGE_FAC_UNLOCK_CODE_LEN (16)
#define LGE_FAC_SLTYPE_VALID_MASK 0x1F
#define LGE_FAC_SLTYPE_MASK_NETWORK 0x01
#define LGE_FAC_SLTYPE_MASK_SERVICE_PROVIDER 0x02
#define LGE_FAC_SLTYPE_MASK_NETWORK_SUBSET 0x04
#define LGE_FAC_SLTYPE_MASK_COOPERATE 0x08
#define LGE_FAC_SLTYPE_MASK_LOCK_TO_SIM 0x10
#define LGE_FAC_SLTYPE_MASK_HARDLOCK 0x20
#define LGE_FAC_SLTYPE_MASK_RESERVED_1 0x40 /* T.B.D */
#define LGE_FAC_SLTYPE_MASK_RESERVED_2 0x80 /* T.B.D */
#define LGE_FAC_MAX_UNLOCK_CODE_VERIFY_FAIL_COUNT 3
//                                                                                                
//                         
//                                   
//                                                                                                


#define EMMC_BLOCK_SIZE    512

#ifndef bool
#define bool         unsigned char
#define true 1
#define false 0
#endif

typedef struct FactoryNetworkCodeTag
{
    unsigned char Mcc[LGE_FAC_NC_MCC_LEN];  /* Ex) { 2, 4, 5 } */
    unsigned char Mnc[LGE_FAC_NC_MNC_LEN];  /* Ex) { 4, 3, 0xF } */
    unsigned char Gid1[LGE_FAC_NC_GID1_LEN];    /* Ex) { 0xB, 2, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF } */
    unsigned char Gid2[LGE_FAC_NC_GID2_LEN];    /* Ex) { 8, 0xA, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF } */
    unsigned char Subset[LGE_FAC_NC_SUBSET_LEN];    /* Ex) { 6, 2 } */
    unsigned char dummy[8];
} FactoryNetworkCode;

typedef struct FactoryUnlockCodeTag
{
    unsigned char network[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char serviceProvider[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char networkSubset[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char cooperate[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char lockToSim[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char hardlock[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char reserved_1[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char reserved_2[LGE_FAC_UNLOCK_CODE_LEN];
} FactoryUnlockCode;

typedef struct FactoryUnlockFailCountTag
{
    unsigned char  network;
    unsigned char  serviceProvider;
    unsigned char  networkSubset;
    unsigned char  cooperate;
    unsigned char  lockToSim;
    unsigned char  hardlock;
    unsigned char  reserved_1;
    unsigned char  reserved_2;
}FactoryUnlockFailCount;
//                                                                                            


bool LGE_FacWriteWifiMacAddr(unsigned char *wifiMacAddr);
bool LGE_FacReadWifiMacAddr(unsigned char *wifiMacAddr);
bool LGE_FacWriteBtAddr(unsigned char *btAddr);
bool LGE_FacReadBtAddr(unsigned char *btAddr);
bool LGE_FacWriteImei(unsigned int imeiNum, unsigned char *imei);
bool LGE_FacReadImei(unsigned int imeiNum, unsigned char *imei);
bool LGE_FacWriteSimLockType(unsigned char simLockType);
bool LGE_FacReadSimLockType(unsigned char *simLockType);
bool LGE_FacWriteUnlockCodeVerifyFailCount(unsigned char failCount);
bool LGE_FacReadUnlockCodeVerifyFailCount(unsigned char *failCount);
bool LGE_FacWriteUnlockFailCount(unsigned char simLockType, unsigned char failCount);
bool LGE_FacReadUnlockFailCount(unsigned char simLockType, unsigned char *failCount);
bool LGE_FacWriteUnlockCode(FactoryUnlockCode * unlockCode);
bool LGE_FacVerifyUnlockCode(unsigned char simLockType, unsigned char *unlockCode, bool * isOk);
bool LGE_FacCheckUnlockCodeValidness(bool * isValid);
bool LGE_FacWriteNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum);
bool LGE_FacReadNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum);
bool LGE_FacWriteNetworkCodeListNum(unsigned short networkCodeListNum);
bool LGE_FacReadNetworkCodeListNum(unsigned short *networkCodeListNum);
bool LGE_FacCheckNetworkCodeValidness(unsigned char simLockType, bool * isValid);
bool LGE_FacInitSimLockData(void);
bool LGE_FacReadFusgFlag(unsigned char *fusgFlag);
bool LGE_FacWriteFusgFlag(unsigned char fusgFlag);
bool LGE_FacReadDataVersion(unsigned char *dataVersion);
bool LGE_FacWriteDataVersion(unsigned char *dataVersion);
bool LGE_FacReadPid(unsigned char *pid);
bool LGE_FacWritePid(unsigned char *pid);
void LGE_FacGetSoftwareversion(bool isOriginalVersion, unsigned char *pVersion);
//                                                                                                
bool LGE_FacWriteQEMFlag(unsigned char qemFlag);
bool LGE_FacReadQEMFlag(unsigned char *qemFlag);
bool LGE_FacWriteFactoryResetStatusFlag(unsigned char factoryResetFlag);
bool LGE_FacReadFactoryResetStatusFlag(unsigned char *factoryResetFlag);
//                                                                                                
int LGE_API_test(void);

#endif
