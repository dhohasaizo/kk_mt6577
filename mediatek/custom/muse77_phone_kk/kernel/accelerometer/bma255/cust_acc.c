#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


extern unsigned int system_rev;
/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 1,
    .direction = 6,
    .power_id = MT65XX_POWER_LDO_VCAM_AF,  /*!< LDO is not used */
    .power_vol= VOL_3000,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};
static struct acc_hw cust_acc_hw_revB = {
    .i2c_num = 1,
    .direction = 2,
    .power_id = MT65XX_POWER_LDO_VCAM_AF,  /*!< LDO is not used */
    .power_vol= VOL_3000,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw(void)
{
    if(system_rev<2||system_rev>4){
    	 return &cust_acc_hw;
    }else{
    	 return &cust_acc_hw_revB;
    }
}
