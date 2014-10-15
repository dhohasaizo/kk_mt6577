#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

extern unsigned int system_rev;

static struct mag_hw cust_mag_hw = {
    .i2c_num = 1,
    .direction = 6,
    .power_id = MT65XX_POWER_LDO_VCAM_AF,/*!< LDO is not used */
    .power_vol = VOL_3000,        /*!< LDO is not used */
};

static struct mag_hw cust_mag_hw_revB = {
    .i2c_num = 1,
    .direction = 2,
    .power_id = MT65XX_POWER_LDO_VCAM_AF,/*!< LDO is not used */
    .power_vol = VOL_3000,        /*!< LDO is not used */
};

struct mag_hw* get_cust_mag_hw(void) 
{
    if(system_rev<2||system_rev>4){
		return &cust_mag_hw;
    }else{
		return &cust_mag_hw_revB;
    }
}
