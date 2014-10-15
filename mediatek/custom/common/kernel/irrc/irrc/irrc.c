/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  	: mediatek\custom\common\kernel\alsps\apds9130.c
 *    Author(s)   :  Kang Jun Mo < junmo.kang@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/interrupt.h>
#include <linux/module.h> 
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include "irrc.h"
#include "cust_gpio_usage.h"
#include <mach/mt_pwm.h>
#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/

/****************************************************************************
 * Macros
 ****************************************************************************/
#define APS_DEBUG

#define APS_TAG "[IRRC] "
#define TRUE 1
#define APS_ERR(fmt, args...)    printk(KERN_ERR APS_TAG"[ERROR] %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)

#if defined ( APS_DEBUG )
/* You need to select proper loglevel to see the log what you want. ( Currently, you can't see "KERN_INFO" level ) */
#define APS_FUN(f)  	   printk(KERN_ERR APS_TAG"%s()\n", __FUNCTION__)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#else
#define APS_FUN(f)  	   printk(KERN_INFO APS_TAG"%s()\n", __FUNCTION__)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#endif

#ifndef GPIO_RC_LED_LDO_EN
#define GPIO_RC_LED_LDO_EN	GPIO12
#endif

#ifndef GPIO_RC_LED_LDO_EN_M_GPIO
#define GPIO_RC_LED_LDO_EN_M_GPIO	GPIO_MODE_00
#endif

#ifndef GPIO_IRRC_LDO_EN
#define GPIO_IRRC_LDO_EN GPIO20
#endif

#ifndef GPIO_IRRC_LDO_EN_M_GPIO
#define GPIO_IRRC_LDO_EN_M_GPIO	GPIO_MODE_00
#endif

void mt_pwm_disable(U32 pwm_no, BOOL pmic_pad);
static struct irrc_ts_data *ts;

struct irrc_compr_params {
	int HD_size;
	int LD_size;
};

struct irrc_ts_data {
    struct workqueue_struct *workqueue;
    struct delayed_work input_PWM_off_work;
	struct delayed_work input_LDO_off_work;
};


static ssize_t show_test_value(struct device_driver *ddri, char *buf)
{
	int err = 0;
	return err;
}

static ssize_t store_test_value(struct device_driver *ddri, char *buf, size_t count)
{
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(test,      S_IRUGO | S_IWUSR, show_test_value, store_test_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *irrc_attr_list[] = {
	&driver_attr_test,
};
/*----------------------------------------------------------------------------*/
static int irrc_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(irrc_attr_list)/sizeof(irrc_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, irrc_attr_list[idx]))
		{
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int irrc_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(irrc_attr_list)/sizeof(irrc_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, irrc_attr_list[idx]);
	}

	return 0;
}
static int IRRC_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int IRRC_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t IRRC_write(struct file *file, const char __user *buf,
	size_t count, loff_t *pos)
{
	return 0;
}

int IRRCpcm_fsync(struct file *file, loff_t a, loff_t b, int datasync)
{
	return 0;
}


static int irrc_pwmon(int HD_size,int LD_size)			
{
	struct pwm_spec_config pwm_setting;
	int err;
	
	mt_pwm_disable(1,TRUE);
	pwm_setting.pwm_no = 1;
	pwm_setting.mode = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV2;
	pwm_setting.pmic_pad = 0;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = HD_size - 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = LD_size - 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0x55555555;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x55555555;
	
	err = pwm_set_spec_config(&pwm_setting);
	
	printk(KERN_INFO "err: %d",err);
	return 0;	
}

extern unsigned int system_rev;
static void irrc_pwmoff_work_func(struct work_struct *work)
{
	printk(KERN_INFO "gpio_signal_off!\n");	
	mt_pwm_disable(1,TRUE);
}

static void irrc_ldo_off_work_func(struct work_struct *work)
{
	printk(KERN_INFO "pwm_signal_off!\n");	/*
	if(system_rev<2){
	   mt_set_gpio_out(GPIO_RC_LED_LDO_EN,GPIO_OUT_ZERO);
	}else if(system_rev==2){
	   hwPowerDown(MT65XX_POWER_LDO_VCAMA,"2V8_IrRC_LDO_S");
	}else{*/
	   mt_set_gpio_out(GPIO_IRRC_LDO_EN,GPIO_OUT_ZERO);
	//}
}


static long IRRC_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	int HD_size;
	int LD_size;
	
	struct irrc_compr_params test;
	
	rc = copy_from_user(&test, (void __user *)arg, sizeof(test));
	
	HD_size = test.HD_size;
	LD_size = test.LD_size;
	printk(KERN_INFO "IRRC_ioctl First %d %d",HD_size,LD_size);
	switch (cmd) {
	case IRRC_START:
		printk(KERN_INFO "IRRC_START\n");
	//	if(system_rev<2){
	//	}else if(system_rev==2){
	//		hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, "2V8_IrRC_LDO_S");
	//	}else{
			mt_set_gpio_mode(GPIO_IRRC_LDO_EN,GPIO_MODE_00);
			mt_set_gpio_pull_enable(GPIO_IRRC_LDO_EN, GPIO_PULL_ENABLE);
			mt_set_gpio_dir(GPIO_IRRC_LDO_EN,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_IRRC_LDO_EN,GPIO_OUT_ONE);
	//	}
		irrc_pwmon(HD_size,LD_size);	
		cancel_delayed_work_sync(&ts->input_PWM_off_work); 
		cancel_delayed_work_sync(&ts->input_LDO_off_work);  
		break;
	case IRRC_STOP:
		printk(KERN_INFO "IRRC_STOP\n");
		cancel_delayed_work_sync(&ts->input_PWM_off_work);  
		queue_delayed_work(ts->workqueue,&ts->input_PWM_off_work, msecs_to_jiffies(1500));		
		cancel_delayed_work_sync(&ts->input_LDO_off_work);  
		queue_delayed_work(ts->workqueue,&ts->input_LDO_off_work, msecs_to_jiffies(1500));	
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}
static const struct file_operations IRRC_pcm_fops = {
	.owner		= THIS_MODULE,
	.open		= IRRC_open,
	.release	= IRRC_release,
	.write		= IRRC_write,
	.unlocked_ioctl	= IRRC_ioctl,
	.fsync = IRRCpcm_fsync,
};
struct miscdevice IRRC_pcm_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_IRRC_pcm_dec",
	.fops	= &IRRC_pcm_fops,
};
static int irrc_probe ( struct platform_device *pdev )
{
	int err;
	ts = kzalloc(sizeof(struct irrc_ts_data), GFP_KERNEL);
	if (ts == NULL) {
			ERR_MSG("Can not allocate memory\n");
	}
	ts->workqueue = create_workqueue("irrc_ts_workqueue");	
	if (!ts->workqueue) {
			ERR_MSG("Unable to create workqueue\n");
	}
	INIT_DELAYED_WORK(&ts->input_PWM_off_work, irrc_pwmoff_work_func);
	INIT_DELAYED_WORK(&ts->input_LDO_off_work, irrc_ldo_off_work_func);
	misc_register(&IRRC_pcm_misc);
	return 0;
}
static int irrc_remove ( struct platform_device *pdev )
{
	
	return 0;
}

static struct platform_driver irrc_driver = { .probe = irrc_probe, .remove = irrc_remove, .driver = { .name = "irrc", } };


/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/



static int __init irrc_init ( void )
{
	APS_FUN ();
	if ( platform_driver_register ( &irrc_driver ) )
	{
		APS_ERR ( "failed to register platform driver\n" );
		return -ENODEV;
	}
	return 0;
}

static void __exit irrc_exit ( void )
{
	APS_FUN ();
	platform_driver_unregister ( &irrc_driver );
}


module_init ( irrc_init );
module_exit ( irrc_exit );

MODULE_AUTHOR ( "Jung Sung Soo" );
MODULE_DESCRIPTION ( "Irrc driver" );
MODULE_LICENSE ( "GPL" );

/* End Of File */

