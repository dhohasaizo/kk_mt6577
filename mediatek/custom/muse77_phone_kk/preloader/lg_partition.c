#include "mmc_core.h"
#include "boot_device.h"
#include "addr_trans.h"
#include "lg_partition.h"
#include "part.h"
#include "blkdev.h"
#include "partition_define.h"
#include "platform.h"
#ifdef FEATURE_DOWNLOAD_INFO

#define BLKS			1
#define PRINT	print

bool _LGE_GENERIC_WRITE_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{
    unsigned char tmp[EMMC_BLOCK_SIZE];
    memset(tmp, 0, EMMC_BLOCK_SIZE);
    memcpy(tmp, buff, length);
    
    part_t *part = part_get(PART_MISC2);
       
    int blknr = (part->startblk + offset);
    
    blkdev_t *dev = blkdev_get(BOOTDEV_SDMMC);
    if (0 != dev->bwrite(dev, blknr, BLKS, (unsigned char *)tmp))
    {
        print("[%s]write misc2 partition error \n", __FUNCTION__);
        return false;        
    }        
    
    return true;
}

bool _LGE_GENERIC_READ_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{
    unsigned char tmp[EMMC_BLOCK_SIZE];
    memset(tmp, 0, EMMC_BLOCK_SIZE);
    
    part_t *part = part_get(PART_MISC2);
        
    int addr = (part->startblk + offset);
    int blknr = (part->startblk + offset);
    
    blkdev_t *dev = blkdev_get(BOOTDEV_SDMMC);
    if (0 != dev->bread(dev, blknr, BLKS, (unsigned char *)tmp))
    {
        print("[%s]read misc2 partition error \n", __FUNCTION__);
        return false;
    }
    memcpy(buff, tmp, length);        
    
    return true;
}

bool LGE_FacWriteWifiMacAddr(unsigned char *wifiMacAddr, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(wifiMacAddr, LGE_FAC_WIFI_MAC_ADDR_OFFSET, LGE_FAC_WIFI_MAC_ADDR_LEN);
}

bool LGE_FacReadWifiMacAddr(unsigned char *wifiMacAddr)
{
    return _LGE_GENERIC_READ_FUN(wifiMacAddr, LGE_FAC_WIFI_MAC_ADDR_OFFSET, LGE_FAC_WIFI_MAC_ADDR_LEN);
}

bool LGE_FacWriteBtAddr(unsigned char *btAddr, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(btAddr, LGE_FAC_BT_ADDR_OFFSET, LGE_FAC_BT_ADDR_LEN);
}

bool LGE_FacReadBtAddr(unsigned char *btAddr)
{
    return _LGE_GENERIC_READ_FUN(btAddr, LGE_FAC_BT_ADDR_OFFSET, LGE_FAC_BT_ADDR_LEN);
}

bool LGE_FacWriteImei(bool isMaster, unsigned char *imei, bool needFlashProgram)
{
    if (isMaster == true)
        return _LGE_GENERIC_WRITE_FUN(imei, LGE_FAC_IMEI_MASTER_OFFSET, LGE_FAC_IMEI_LEN);
    else
        return _LGE_GENERIC_WRITE_FUN(imei, LGE_FAC_IMEI_NOT_MASTER_OFFSET, LGE_FAC_IMEI_LEN);
}

bool LGE_FacReadImei(bool isMaster, unsigned char *imei)
{
    if (isMaster == true)
        return _LGE_GENERIC_READ_FUN(imei, LGE_FAC_IMEI_MASTER_OFFSET, LGE_FAC_IMEI_LEN);
    else
        return _LGE_GENERIC_READ_FUN(imei, LGE_FAC_IMEI_NOT_MASTER_OFFSET, LGE_FAC_IMEI_LEN);
}

bool LGE_FacWriteSimLockType(unsigned char simLockType, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&simLockType, LGE_FAC_SIM_LOCK_TYPE_OFFSET, LGE_FAC_SIM_LOCK_TYPE_LEN);
}

bool LGE_FacReadSimLockType(unsigned char *simLockType)
{
    return _LGE_GENERIC_READ_FUN(simLockType, LGE_FAC_SIM_LOCK_TYPE_OFFSET, LGE_FAC_SIM_LOCK_TYPE_LEN);
}

bool LGE_FacWriteNetworkCodeListNum(unsigned short networkCodeListNum, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&networkCodeListNum, LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET, LGE_FAC_NETWORK_CODE_LIST_NUM_LEN);
}

bool LGE_FacReadNetworkCodeListNum(unsigned short *networkCodeListNum)
{
    return _LGE_GENERIC_READ_FUN(networkCodeListNum, LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET, LGE_FAC_NETWORK_CODE_LIST_NUM_LEN);
}

bool LGE_FacWriteUnlockCodeVerifyFailCount(unsigned char failCount, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&failCount, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN);
}

bool LGE_FacReadUnlockCodeVerifyFailCount(unsigned char *failCount)
{
    return _LGE_GENERIC_READ_FUN(failCount, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN);
}

bool LGE_FacWriteUnlockFailCount(unsigned char simLockType, unsigned char failCount, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&failCount, LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_FAIL_COUNT_LEN);
}

bool LGE_FacReadUnlockFailCount(unsigned char simLockType, unsigned char *failCount)
{
    return _LGE_GENERIC_READ_FUN(failCount, LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_FAIL_COUNT_LEN);

}

bool LGE_FacWriteUnlockCode(FactoryUnlockCode * unlockCode, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(unlockCode, LGE_FAC_UNLOCK_CODE_OFFSET, LGE_FAC_UNLOCK_CODE_LEN);
}

bool LGE_FacVerifyUnlockCode(unsigned char simLockType, unsigned char *unlockCode, bool * isOk)
{
    *isOk = true;
    return true;
}

bool LGE_FacCheckUnlockCodeValidness(bool * isValid)
{
    *isValid = true;
    return true;
}

bool LGE_FacWriteNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(networkCode, LGE_FAC_NETWORK_CODE_OFFSET, LGE_FAC_NETWORK_CODE_LEN);
}

bool LGE_FacReadNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum)
{
    return _LGE_GENERIC_READ_FUN(networkCode, LGE_FAC_NETWORK_CODE_OFFSET, LGE_FAC_NETWORK_CODE_LEN);

}

bool LGE_FacCheckNetworkCodeValidness(unsigned char simLockType, bool * isValid)
{
    *isValid = true;
    return true;
}

bool LGE_FacInitSimLockData(void)
{
    return true;
}

bool LGE_FacReadFusgFlag(unsigned char *fusgFlag)
{
    return _LGE_GENERIC_READ_FUN(fusgFlag, LGE_FAC_FUSG_FLAG_OFFSET, LGE_FAC_FUSG_FLAG_LEN);
}

bool LGE_FacWriteFusgFlag(unsigned char fusgFlag, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&fusgFlag, LGE_FAC_FUSG_FLAG_OFFSET, LGE_FAC_FUSG_FLAG_LEN);
}

bool LGE_FacReadDataVersion(unsigned char *dataVersion)
{
    return _LGE_GENERIC_READ_FUN(dataVersion, LGE_FAC_DATA_VERSION_OFFSET, LGE_FAC_DATA_VERSION_LEN);
}

bool LGE_FacWriteDataVersion(unsigned char *dataVersion, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(dataVersion, LGE_FAC_DATA_VERSION_OFFSET, LGE_FAC_DATA_VERSION_LEN);
}

bool LGE_FacReadPid(unsigned char *pid)
{
    return _LGE_GENERIC_READ_FUN(pid, LGE_FAC_PID_OFFSET, LGE_FAC_PID_LEN);
}

bool LGE_FacWritePid(unsigned char *pid, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(pid, LGE_FAC_PID_OFFSET, LGE_FAC_PID_LEN);
}

void LGE_FacGetSoftwareversion(bool isOriginalVersion, unsigned char *pVersion)
{
    int index = 0;
    for (index = 0; index < LGE_FAC_SV_LEN; index++)
    {
        pVersion[index] = index;
    }
    return true;
}

int LGE_API_test(void)
{
    unsigned char buff[EMMC_BLOCK_SIZE];
    int index = 0;
    memset(buff, 0xa, EMMC_BLOCK_SIZE);
    for (index = 0; index < LGE_FAC_WIFI_MAC_ADDR_LEN; index++)
    {
        buff[index] = LGE_FAC_WIFI_MAC_ADDR_OFFSET+index;
    }
    LGE_FacWriteImei(1, buff, true);
    memset(buff, 0xb, EMMC_BLOCK_SIZE);
    LGE_FacReadImei(1, buff);
	
	PRINT("preloader imei:");
    for (index = 0; index < LGE_FAC_IMEI_LEN; index++)
    {
        PRINT(" 0x%x", buff[index]);
    }
    PRINT("\n");

    memset(buff, 0xc, EMMC_BLOCK_SIZE);
    for (index = 0; index < LGE_FAC_BT_ADDR_LEN; index++)
    {
        buff[index] = LGE_FAC_BT_ADDR_OFFSET+index;
    }
    
    LGE_FacWritePid(buff, true);
    memset(buff, 0xd, EMMC_BLOCK_SIZE);
    LGE_FacReadPid(buff);
	  
	PRINT("preloader Pid:");
    for (index = 0; index < LGE_FAC_PID_LEN; index++)
    {
        PRINT(" 0x%x", buff[index]);
    }
    PRINT("\n");
    return 0;
}

#endif
