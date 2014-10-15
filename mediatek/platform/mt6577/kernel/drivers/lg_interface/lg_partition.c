#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... filp_open */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>      /*proc */
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/aio.h>
#include <asm/uaccess.h>        /*set_fs get_fs mm_segment_t */
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/unistd.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/autoconf.h>
#include "lg_partition.h"

#define PART_NAME "/dev/misc2"
#define PRINT    printk
int g_init_write_size = 1;

bool _LGE_GENERIC_WRITE_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{

    int ret;
    struct file *filp;
    unsigned char *tmp;
    mm_segment_t curr_fs;
    filp = filp_open(PART_NAME, O_RDWR, 0666);
    if (IS_ERR(filp))
    {
        ret = PTR_ERR(filp);
        printk("Open MISC2 partition fail! errno=%d\n", ret);
        return -1;
    }
    if (g_init_write_size ==0)
    {
        struct mtd_info_user info;
        if (filp->f_op->unlocked_ioctl)
        {
            filp->f_op->unlocked_ioctl(filp, MEMGETINFO, &info);
        } else if (filp->f_op->compat_ioctl)
        {
            filp->f_op->compat_ioctl(filp, MEMGETINFO, &info);
        }
        if (info.writesize != EMMC_BLOCK_SIZE)
        {
            printk("write size error!info.writesize=%d,EMMC_BLOCK_SIZE=%d\n", info.writesize, EMMC_BLOCK_SIZE);
            g_init_write_size = 0;
            filp_close(filp, NULL);
            return false;
        } else
        {
            g_init_write_size = 1;
        }
    }

    filp->f_op->llseek(filp, offset * EMMC_BLOCK_SIZE, SEEK_SET);
    tmp = kzalloc(EMMC_BLOCK_SIZE, GFP_KERNEL);
    if (!tmp)
    {
        printk("malloc memory fail!\n");
        filp_close(filp, NULL);
        return false;
    }
    memset(tmp, 0x0, EMMC_BLOCK_SIZE);
    curr_fs = get_fs();
    set_fs(KERNEL_DS);
    memcpy(tmp, buff, length);
    ret = filp->f_op->write(filp, tmp, EMMC_BLOCK_SIZE, &(filp->f_pos));
    if (EMMC_BLOCK_SIZE != ret)
    {
        printk("write fail!errno=%d\n", ret);
        filp_close(filp, NULL);
        kfree(tmp);
        set_fs(curr_fs);
        return false;

    }
    set_fs(curr_fs);
    kfree(tmp);
    filp_close(filp, NULL);
    return true;
}

bool _LGE_GENERIC_READ_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{

    int ret;
    struct file *filp;
    unsigned char *tmp;
    mm_segment_t curr_fs;
    filp = filp_open(PART_NAME, O_RDWR, 0666);
    if (IS_ERR(filp))
    {
        ret = PTR_ERR(filp);
        printk("Open MISC2 partition fail! errno=%d\n", ret);
        return -1;
    }
    if (g_init_write_size == 0)
    {
        struct mtd_info_user info;
        if (filp->f_op->unlocked_ioctl)
        {
            filp->f_op->unlocked_ioctl(filp, MEMGETINFO, &info);
        } else if (filp->f_op->compat_ioctl)
        {
            filp->f_op->compat_ioctl(filp, MEMGETINFO, &info);
        }
        if (info.writesize != EMMC_BLOCK_SIZE)
        {
            printk("write size error!info.writesize=%d,EMMC_BLOCK_SIZE=%d\n", info.writesize, EMMC_BLOCK_SIZE);
            g_init_write_size = 0;
            filp_close(filp, NULL);
            return false;
        } else
        {
            g_init_write_size = 1;
        }
    }

    filp->f_op->llseek(filp, offset * EMMC_BLOCK_SIZE, SEEK_SET);
    tmp = kzalloc(EMMC_BLOCK_SIZE, GFP_KERNEL);
    if (!tmp)
    {
        printk("malloc memory fail!\n");
        filp_close(filp, NULL);
        return false;
    }
    memset(tmp, 0x0, EMMC_BLOCK_SIZE);
    curr_fs = get_fs();
    set_fs(KERNEL_DS);

    ret = filp->f_op->read(filp, tmp, EMMC_BLOCK_SIZE, &(filp->f_pos));
    if (EMMC_BLOCK_SIZE != ret)
    {
        printk("read fail!errno=%d\n", ret);
        filp_close(filp, NULL);
        kfree(tmp);
        set_fs(curr_fs);
        return false;

    }
    memcpy(buff, tmp, length);
    set_fs(curr_fs);
    kfree(tmp);
    filp_close(filp, NULL);
    return true;
}
//                                                                                            
bool LGE_FacWriteWifiMacAddr(unsigned char *wifiMacAddr)
{
    //                                                                                                    
    return _LGE_GENERIC_WRITE_FUN(wifiMacAddr, LGE_FAC_WIFI_IDX, LGE_FAC_WIFI_SIZE);
}

bool LGE_FacReadWifiMacAddr(unsigned char *wifiMacAddr)
{
    //                                                                                                   
    return _LGE_GENERIC_READ_FUN(wifiMacAddr, LGE_FAC_WIFI_IDX, LGE_FAC_WIFI_SIZE);
}

bool LGE_FacWriteBtAddr(unsigned char *btAddr)
{
    //                                                                                   
    return _LGE_GENERIC_WRITE_FUN(btAddr, LGE_FAC_BT_IDX, LGE_FAC_BT_SIZE);
}

bool LGE_FacReadBtAddr(unsigned char *btAddr)
{
    //                                                                                  
    return _LGE_GENERIC_READ_FUN(btAddr, LGE_FAC_BT_IDX, LGE_FAC_BT_SIZE);
}

bool LGE_FacWriteImei(unsigned int imeiNum, unsigned char *imei)
{
    if (imeiNum == 1)
    {
        //                                                                                  
        return _LGE_GENERIC_WRITE_FUN(imei, LGE_FAC_IMEI_IDX, LGE_FAC_IMEI_SIZE);
    }
    else if(imeiNum == 2)
    {
        //                                                                                      
        return _LGE_GENERIC_WRITE_FUN(imei, LGE_FAC_IMEI2_IDX, LGE_FAC_IMEI_SIZE);
    }
    else if(imeiNum == 3)
    {
        //                                                                                      
        return _LGE_GENERIC_WRITE_FUN(imei, LGE_FAC_IMEI3_IDX, LGE_FAC_IMEI_SIZE);
    }
    else if(imeiNum == 4)
    {
        //                                                                                      
        return _LGE_GENERIC_WRITE_FUN(imei, LGE_FAC_IMEI4_IDX, LGE_FAC_IMEI_SIZE);
    }
}

bool LGE_FacReadImei(unsigned int imeiNum, unsigned char *imei)
{
    if (imeiNum == 1)
    {
        //                                                                                 
        return _LGE_GENERIC_READ_FUN(imei, LGE_FAC_IMEI_IDX, LGE_FAC_IMEI_SIZE);
    }
    else if(imeiNum == 2)
    {
        //                                                                                     
        return _LGE_GENERIC_READ_FUN(imei, LGE_FAC_IMEI2_IDX, LGE_FAC_IMEI_SIZE);
    }
    else if(imeiNum == 3)
    {
        //                                                                                     
        return _LGE_GENERIC_READ_FUN(imei, LGE_FAC_IMEI3_IDX, LGE_FAC_IMEI_SIZE);
    }
    else if(imeiNum == 4)
    {
        //                                                                                     
        return _LGE_GENERIC_READ_FUN(imei, LGE_FAC_IMEI4_IDX, LGE_FAC_IMEI_SIZE);
    }
}

bool LGE_FacWriteSimLockType(unsigned char simLockType)
{
    //                                                                                                     
    return _LGE_GENERIC_WRITE_FUN(&simLockType, LGE_FAC_SIMLOCK_TYPE_IDX, LGE_FAC_SIMLOCK_TYPE_SIZE);
}

bool LGE_FacReadSimLockType(unsigned char *simLockType)
{
    //                                                                                                   
    return _LGE_GENERIC_READ_FUN(simLockType, LGE_FAC_SIMLOCK_TYPE_IDX, LGE_FAC_SIMLOCK_TYPE_SIZE);
}

bool LGE_FacWriteNetworkCodeListNum(unsigned short networkCodeListNum)
{
    //                                                                                                                            
    return _LGE_GENERIC_WRITE_FUN(&networkCodeListNum, LGE_FAC_NETWORK_CODE_LIST_IDX, LGE_FAC_NETWORK_CODE_LIST_SIZE);
}

bool LGE_FacReadNetworkCodeListNum(unsigned short *networkCodeListNum)
{
    //                                                                                                                          
    return _LGE_GENERIC_READ_FUN(networkCodeListNum, LGE_FAC_NETWORK_CODE_LIST_IDX, LGE_FAC_NETWORK_CODE_LIST_SIZE);
}

bool LGE_FacWriteUnlockCodeVerifyFailCount(unsigned char failCount)
{
    //                                                                                                                                   
    return _LGE_GENERIC_WRITE_FUN(&failCount, LGE_FAC_UNLOCKCODE_VERIFY_FAIL_IDX, LGE_FAC_UNLOCKCODE_VERIFY_FAIL_SIZE);
}

bool LGE_FacReadUnlockCodeVerifyFailCount(unsigned char *failCount)
{
    //                                                                                                                                 
    return _LGE_GENERIC_READ_FUN(failCount, LGE_FAC_UNLOCKCODE_VERIFY_FAIL_IDX, LGE_FAC_UNLOCKCODE_VERIFY_FAIL_SIZE);
}

bool LGE_FacWriteUnlockFailCount(unsigned char simLockType, unsigned char failCount)
{
    //                                                                                                           
    return _LGE_GENERIC_WRITE_FUN(&failCount, LGE_FAC_UNLOCKCODE_FAIL_COUNT_IDX, LGE_FAC_UNLOCKCODE_FAIL_COUNT_SIZE);
}

bool LGE_FacReadUnlockFailCount(unsigned char simLockType, unsigned char *failCount)
{
    //                                                                                                         
    return _LGE_GENERIC_READ_FUN(failCount, LGE_FAC_UNLOCKCODE_FAIL_COUNT_IDX, LGE_FAC_UNLOCKCODE_FAIL_COUNT_SIZE);

}

bool LGE_FacWriteUnlockCode(FactoryUnlockCode * unlockCode)
{
    //                                                                                               
    return true;    //This api is not used kernel.
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
//                                                                                            
#if 0
bool LGE_FacWriteNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum)
{
    //                                                                                                  
    return _LGE_GENERIC_WRITE_FUN(networkCode, LGE_FAC_NETWORK_CODE_IDX, LGE_FAC_NETWORK_CODE_SIZE);
}

bool LGE_FacReadNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum)
{
    //                                                                                                 
    return _LGE_GENERIC_READ_FUN(networkCode, LGE_FAC_NETWORK_CODE_IDX, LGE_FAC_NETWORK_CODE_SIZE);

}
#endif
//                                                                                            
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
    //                                                                                        
    return _LGE_GENERIC_READ_FUN(fusgFlag, LGE_FAC_FUSG_FLAG_IDX, LGE_FAC_FUSG_FLAG_SIZE);
}

bool LGE_FacWriteFusgFlag(unsigned char fusgFlag)
{
    //                                                                                          
    return _LGE_GENERIC_WRITE_FUN(&fusgFlag, LGE_FAC_FUSG_FLAG_IDX, LGE_FAC_FUSG_FLAG_SIZE);
}

bool LGE_FacReadDataVersion(unsigned char *dataVersion)
{
    //                                                                                                 
    return _LGE_GENERIC_READ_FUN(dataVersion, LGE_FAC_DATA_VER_IDX, LGE_FAC_DATA_VER_SIZE);
}

bool LGE_FacWriteDataVersion(unsigned char *dataVersion)
{
    //                                                                                                  
    return _LGE_GENERIC_WRITE_FUN(dataVersion, LGE_FAC_DATA_VER_IDX, LGE_FAC_DATA_VER_SIZE);
}

bool LGE_FacReadPid(unsigned char *pid)
{
    //                                                                       
    return _LGE_GENERIC_READ_FUN(pid, LGE_FAC_PID_IDX, LGE_FAC_PID_SIZE);
}

bool LGE_FacWritePid(unsigned char *pid)
{
    //                                                                        
    return _LGE_GENERIC_WRITE_FUN(pid, LGE_FAC_PID_IDX, LGE_FAC_PID_SIZE);
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
//                                                                                                
//                                                                                            
#if 0
bool LGE_FacReadQEMFlag(unsigned char *qemFlag)
{
    //                                                                           
    return _LGE_GENERIC_READ_FUN(qemFlag, LGE_FAC_QEM_IDX, LGE_FAC_QEM_SIZE);
}
bool LGE_FacWriteQEMFlag(unsigned char qemFlag)
{
    //                                                                             
    return _LGE_GENERIC_WRITE_FUN(&qemFlag, LGE_FAC_QEM_IDX, LGE_FAC_QEM_SIZE);
}
#endif
//                                                                                            
bool LGE_FacReadFactoryResetStatusFlag(unsigned char *factoryResetFlag)
{
    //                                                                                                        
    return _LGE_GENERIC_READ_FUN(factoryResetFlag, LGE_FAC_FACTORY_RESET_IDX, LGE_FAC_FACTORY_RESET_SIZE);
}
bool LGE_FacWriteFactoryResetStatusFlag(unsigned char factoryResetFlag)
{
    //                                                                                                          
    return _LGE_GENERIC_WRITE_FUN(&factoryResetFlag, LGE_FAC_FACTORY_RESET_IDX, LGE_FAC_FACTORY_RESET_SIZE);
}
//                                                                                                

//                                                           
bool LGE_FacReadLCDCalibration(unsigned char *lcd_kcal)
{
    return _LGE_GENERIC_READ_FUN(lcd_kcal, LGE_FAC_LCD_CALIBRATION_IDX, LGE_FAC_LCD_CALIBRATION_SIZE);
}

bool LGE_FacWriteLCDCalibration(unsigned char *lcd_kcal)
{
    return _LGE_GENERIC_WRITE_FUN(lcd_kcal, LGE_FAC_LCD_CALIBRATION_IDX, LGE_FAC_LCD_CALIBRATION_SIZE);
}
//                                                           

int LGE_API_test(void)
{
    unsigned char buff[EMMC_BLOCK_SIZE];
    int index = 0;
    memset(buff, 0xa, EMMC_BLOCK_SIZE);
    //                                                           
    for (index = 0; index < LGE_FAC_WIFI_SIZE; index++)
    {
        //                                               
        buff[index] = LGE_FAC_WIFI_IDX+0xa;
    }
    LGE_FacWriteWifiMacAddr(buff);
    memset(buff, 0xb, EMMC_BLOCK_SIZE);
    LGE_FacReadWifiMacAddr(buff);
    PRINT("kernel wifi data:");
    for (index = 0; index < 100; index++)
    {
        PRINT(" 0x%x", buff[index]);
    }
    PRINT("\n");

    memset(buff, 0xc, EMMC_BLOCK_SIZE);
    //                                                     
    for (index = 0; index < LGE_FAC_BT_SIZE; index++)
    {
        //                                         
        buff[index] = LGE_FAC_BT_IDX+0xc;
    }
    LGE_FacWriteBtAddr(buff);
    memset(buff, 0xd, EMMC_BLOCK_SIZE);
    LGE_FacReadBtAddr(buff);
    PRINT("kernel BT data:");
    for (index = 0; index < 100; index++)
    {
        PRINT(" 0x%x", buff[index]);
    }
    PRINT("\n");
    return 0;
}
//                                                                                            
EXPORT_SYMBOL(LGE_FacWriteWifiMacAddr);
EXPORT_SYMBOL(LGE_FacReadWifiMacAddr);
EXPORT_SYMBOL(LGE_FacWriteBtAddr);
EXPORT_SYMBOL(LGE_FacReadBtAddr);
EXPORT_SYMBOL(LGE_FacWriteImei);
EXPORT_SYMBOL(LGE_FacReadImei);
EXPORT_SYMBOL(LGE_FacWriteSimLockType);
EXPORT_SYMBOL(LGE_FacReadSimLockType);
EXPORT_SYMBOL(LGE_FacWriteUnlockCodeVerifyFailCount);
EXPORT_SYMBOL(LGE_FacReadUnlockCodeVerifyFailCount);
EXPORT_SYMBOL(LGE_FacWriteUnlockFailCount);
EXPORT_SYMBOL(LGE_FacReadUnlockFailCount);
EXPORT_SYMBOL(LGE_FacWriteUnlockCode);
EXPORT_SYMBOL(LGE_FacVerifyUnlockCode);
EXPORT_SYMBOL(LGE_FacCheckUnlockCodeValidness);
//                                                                                            
//                                       
//                                      
//                                                                                            
EXPORT_SYMBOL(LGE_FacWriteNetworkCodeListNum);
EXPORT_SYMBOL(LGE_FacReadNetworkCodeListNum);
EXPORT_SYMBOL(LGE_FacCheckNetworkCodeValidness);
EXPORT_SYMBOL(LGE_FacInitSimLockData);
EXPORT_SYMBOL(LGE_FacReadFusgFlag);
EXPORT_SYMBOL(LGE_FacWriteFusgFlag);
EXPORT_SYMBOL(LGE_FacReadDataVersion);
EXPORT_SYMBOL(LGE_FacWriteDataVersion);
EXPORT_SYMBOL(LGE_FacReadPid);
EXPORT_SYMBOL(LGE_FacWritePid);
EXPORT_SYMBOL(LGE_FacGetSoftwareversion);
//                                                                                                
//                                                                                            
//                                  
//                                   
//                                                                                            
EXPORT_SYMBOL(LGE_FacReadFactoryResetStatusFlag);
EXPORT_SYMBOL(LGE_FacWriteFactoryResetStatusFlag);
//                                                                                                
//                                                           
EXPORT_SYMBOL(LGE_FacReadLCDCalibration);
EXPORT_SYMBOL(LGE_FacWriteLCDCalibration);
//                                                           

EXPORT_SYMBOL(LGE_API_test);

MODULE_AUTHOR("Kai Zhu@mediatek.com");
MODULE_DESCRIPTION("access partition API");
MODULE_LICENSE("GPL");
