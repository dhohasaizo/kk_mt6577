/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/jiffies.h>


static bool lge_kcal_debug_mode = false;

extern int get_kcal_value(unsigned char *rsp);
extern int set_kcal_value(unsigned char r, unsigned char g, unsigned char b);
extern int update_kcal_register(unsigned char r, unsigned char g, unsigned char b);

extern int mtkfb_display_kcal(int is_white);

#define lcd_kcal_dprintk(fmt, args...)    \
do {                                    \
    if (lge_kcal_debug_mode) {            \
        printk(fmt, ##args);            \
    }                                   \
} while(0)

static ssize_t show_lcd_kcal(struct device *dev,struct device_attribute *attr, const char *buf)
{
	unsigned char data[8];
	int ret_val = 0;

	memset(data, 0, sizeof(data));
	ret_val = get_kcal_value(data);

	if(ret_val != 0)
		get_kcal_value(data);
	else
		return 0;
	
	return sprintf(buf, "%d|%d|%d|%d\n", data[0],data[1],data[2],data[3]);
}

static ssize_t store_lcd_kcal(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int r = 255;
	int g = 255;
	int b = 255;
	char is_store = 'k';
	
	sscanf(buf, "%d|%d|%d|%c", &r, &g, &b, &is_store);
	printk("\n\n\n\n    r =   %d, g =   %d,  b = %d, ,is_store = %c  %d \n\n",r,g,b,is_store,is_store);
	if(is_store == 75/*'K'*/) { // ascii code 'K' == 75
		set_kcal_value(r, g, b);       //store and..
		update_kcal_register(r, g, b); //register reflash
	} else {
		update_kcal_register(r, g, b); //not store, just reflash for lcd register
	}

	return size;
}

static DEVICE_ATTR(lcd_kcal, 0664, show_lcd_kcal, store_lcd_kcal);

static ssize_t show_lcd_display_kcal(struct device *dev,struct device_attribute *attr, const char *buf)
{
	return 0;
}

static ssize_t store_lcd_display_kcal(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int is_white = 0;
	sscanf(buf, "%d",&is_white);
	
	if (is_white)
		mtkfb_display_kcal(1); //white display
	else
		mtkfb_display_kcal(0); //normal display
	
	return size;
}

static DEVICE_ATTR(lcd_kcal_white, 0664, show_lcd_display_kcal, store_lcd_display_kcal);


/*******************************************
* lcd-kcal platform driver callback function
********************************************/
static int lcd_kcal_pdrv_probe(struct platform_device *pdev)
{
	int ret;
	
	ret = device_create_file(&(pdev->dev), &dev_attr_lcd_kcal);
	ret = device_create_file(&(pdev->dev), &dev_attr_lcd_kcal_white);
	
	if(ret)
		xlog_printk(ANDROID_LOG_INFO, "LCD/KCAL", " lcd calibration create attribute err = %d\n", ret);

	return 0;
}

/***************************************
* this function should never be called
****************************************/
static int lcd_kcal_pdrv_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver lge_lcd_kcal_pdrv = {
    .probe      = lcd_kcal_pdrv_probe,
    .remove     = lcd_kcal_pdrv_remove,
    .suspend    = NULL,
    .resume     = NULL,
    .driver     = {
        .name   = "lge-lcdkcal",
        .owner  = THIS_MODULE,
    },
};


/***********************************************************
* lcd kal initialization to register lcd -kcal platform driver
************************************************************/
static int __init lcd_kcal_init(void)
{
    int ret = 0;

    ret = platform_driver_register(&lge_lcd_kcal_pdrv);
    if (ret)
    {
        xlog_printk(ANDROID_LOG_ERROR, "LCD/KCAL", "failed to register lcd k-calibration driver\n");
        return ret;
    }
    else
    {
        xlog_printk(ANDROID_LOG_ERROR, "LCD/KCAL", "lcd k-calibration driver registration done\n");
        return 0;
    }
}
module_init(lcd_kcal_init);

static void __exit lcd_kcal_exit(void)
{
    return 0;
}
module_exit(lcd_kcal_exit);

MODULE_DESCRIPTION("LCD K Calibration driver");
MODULE_LICENSE("GPL");
