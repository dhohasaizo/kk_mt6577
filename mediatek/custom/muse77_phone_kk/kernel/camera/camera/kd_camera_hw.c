#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#endif

/******************************************************************************
 * external fuction
******************************************************************************/
extern void mt_isp_mclk_ctrl(MINT32 en); //turn on off Mclk output
extern void cam_vcm_power(int on_off);
extern void cam_dvdd_power(int on_off);
extern void cam_avdd_power(int on_off);
extern void cam_vio_power(int on_off);

/******************************************************************************
 * MIPI Switch Pin Set
******************************************************************************/
extern unsigned int system_rev;
#define GPIO_CAM_MIPI_SW_OE_N_REVB GPIO16
#define GPIO_CAM_MIPI_SW_SEL_REVB GPIO19
#define GPIO_CAM_MIPI_SW_OE_N_REVA GPIO219
#define GPIO_CAM_MIPI_SW_SEL_REVA GPIO223

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

// MIPI Switch Pin Set
PK_DBG("revision(%d) \n",system_rev);
u32 GPIO_CAM_MIPI_SW_OE_N_pin;
u32 GPIO_CAM_MIPI_SW_SEL_pin;
if(system_rev<2)
{ // EVB or Rev A
    GPIO_CAM_MIPI_SW_OE_N_pin = GPIO_CAM_MIPI_SW_OE_N_REVA;
    GPIO_CAM_MIPI_SW_SEL_pin = GPIO_CAM_MIPI_SW_SEL_REVA;
}
else
{
    GPIO_CAM_MIPI_SW_OE_N_pin = GPIO_CAM_MIPI_SW_OE_N_REVB;
    GPIO_CAM_MIPI_SW_SEL_pin = GPIO_CAM_MIPI_SW_SEL_REVB;
}

u32 pinSet[3][8] = {
                    //for main sensor 
                    {GPIO_CAMERA_CMRST_PIN,
                        GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN_PIN,
                        GPIO_CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    },
                    //for sub sensor 
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    },
                    //for main_2 sensor 
                    {GPIO_CAMERA_2_CMRST_PIN,
                        GPIO_CAMERA_2_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_2_CMPDN_PIN,
                        GPIO_CAMERA_2_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ZERO,
                        GPIO_OUT_ONE,
                    }
                   };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_SECOND_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

    //power ON
    if (On) {
        //in case

#if 0 //TODO: depends on HW layout. Should be notified by SA.
        printk("Set CAMERA_POWER_PULL_PIN for power \n"); 
        if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n"); }
        if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
#endif

/*
	For Lenovo75 Project: OV56487_Raw(Main) & MT9V114_YUV(Sub)
	Power Configuration:
		OV5647: VCAM_D:1.5V,VCAM_D2:1.8V,VCAM_A1:2.8V, VCAM_A2:2.8V For AF use
		MT9V114:VCAM_D2:1.8V,VCAM_A2:2.8V
*/

		msleep(10);
		mt_isp_mclk_ctrl(0); // example : turn off the Mclk before begining power sequence
		msleep(10); // delay
		
		printk("UP CAMERA_POWER_UP SID(%d) PID(%d)\n",SensorIdx,pinSetIdx);

#if defined(CONFIG_MINIABB_CHARGER)
		cam_vio_power(1); //Turn on VIO.
		usleep_range(10, 20); //                                                                 
		cam_avdd_power(1); //Turn on AVDD.
		usleep_range(10, 20); //                                                                 
		cam_dvdd_power(1); //Turn on DVDD.
		
		cam_vcm_power(1); //Turn on VCM/
#else

#if 0 //origin code
  //      if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV3640_YUV,currSensorName)))
		//1.


        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }                    
        mdelay(1);
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
        
        mdelay(10);
		//3.
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
        {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
        }
        
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        } 
#endif
#if 1 //edit code
        if(pinSetIdx == 0){ //main sensor
		
		  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX111_MIPI_RAW,currSensorName))) {
		  	printk("UP CAMERA_POWER_UP SENSOR_DRVNAME_IMX111_MIPI_RAW \n");
            if(mt_set_gpio_mode(GPIO_CAM_AVDD_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN failed!! \n");}
            usleep_range(200, 210); //                                                                 
            
            if(mt_set_gpio_mode(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN failed!! \n");}
            usleep_range(120, 130); //                                                                 
            
            if(mt_set_gpio_mode(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN failed!! \n");}

            if(mt_set_gpio_mode(GPIO_CAM_AF_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_AF_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_AF_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN failed!! \n");}
			
	} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX219_MIPI_RAW,currSensorName))) {

              //cam_avdd_power(1); //Turn on AVDD. 2.8V
              printk("UP CAMERA_POWER_UP SENSOR_DRVNAME_IMX219_MIPI_RAW \n");
              if(mt_set_gpio_mode(GPIO_CAM_AVDD_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN mode failed!! \n");}
              if(mt_set_gpio_dir(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN dir failed!! \n");}
              if(mt_set_gpio_out(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
              //msleep(10);
              mdelay(1);
              //cam_vio_power(1); //Turn on VIO. 1.8V		
              if(mt_set_gpio_mode(GPIO_CAM_IOVDD_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN mode failed!! \n");}
              if(mt_set_gpio_dir(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN dir failed!! \n");}
              if(mt_set_gpio_out(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
              //msleep(10);
              //mdelay(1);
              
              //cam_dvdd_power(1); //Turn on DVDD. 1.2V
              if(mt_set_gpio_mode(GPIO_CAM_DVDD_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN mode failed!! \n");}
              if(mt_set_gpio_dir(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN dir failed!! \n");}
              if(mt_set_gpio_out(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
			  
              if(mt_set_gpio_mode(GPIO_CAM_AF_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN mode failed!! \n");}
              if(mt_set_gpio_dir(GPIO_CAM_AF_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN dir failed!! \n");}
              if(mt_set_gpio_out(GPIO_CAM_AF_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN failed!! \n");}
              msleep(10);
			}		
				
        } else if (pinSetIdx == 1){ //sub sensor
            if(mt_set_gpio_mode(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN failed!! \n");}
            usleep_range(80, 90); //                                                                 
            
            if(mt_set_gpio_mode(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN failed!! \n");}
            usleep_range(10, 20); //                                                                 
            
            if(mt_set_gpio_mode(GPIO_CAM_AVDD_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN failed!! \n");}
        } 	     
#endif
#endif
        //MIPI SWITCH START SET
        if(mt_set_gpio_mode(GPIO_CAM_MIPI_SW_OE_N_pin,GPIO_MODE_00)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_OE_N mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_MIPI_SW_OE_N_pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_OE_N dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAM_MIPI_SW_OE_N_pin,GPIO_OUT_ZERO)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_OE_N failed!!\n");} //low == connect
        usleep_range(10, 20); //                                                                 
        
        if(mt_set_gpio_mode(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_MODE_00)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL dir failed!! \n");}
        if(pinSetIdx == 0){
            if(mt_set_gpio_out(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_OUT_ZERO)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL failed!! \n");} //SEL low == main
        }
        else if (pinSetIdx == 1){
            if(mt_set_gpio_out(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_OUT_ONE)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL failed!! \n");} //SEL high == sub
        } 

        // wait power to be stable 
        usleep_range(1000, 1100);  //                                                                 

        //disable inactive sensor
        if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[1-pinSetIdx][IDX_PS_CMRST],pinSet[1-pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[1-pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}            
            if(mt_set_gpio_out(pinSet[1-pinSetIdx][IDX_PS_CMRST],pinSet[1-pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor            
        }        

        //enable active sensor
        //RST pin
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            usleep_range(10000, 10100); //                                                                 
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            usleep_range(1000, 1100);  //                                                                             
        }
        msleep(10); // delay
        mt_isp_mclk_ctrl(1);
        msleep(1);
    }
    else {//power OFF

#if 0 //TODO: depends on HW layout. Should be notified by SA.
        printk("Set GPIO 94 for power OFF\n"); 
        if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[CAMERA SENSOR] Set GPIO94 PULL DISABLE ! \n"); }
        if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}    	    
#endif
        PK_DBG("[OFF]sensorIdx:%d \n",SensorIdx);
        usleep_range(10000, 10100); //                                                                 
        mt_isp_mclk_ctrl(0);
        msleep(1);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}            
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}           
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor    	   
        }
#if defined(CONFIG_MINIABB_CHARGER)
		cam_dvdd_power(0); //Turn off DVDD.
		cam_avdd_power(0); //Turn off AVDD.
		cam_vio_power(0); //Turn off VIO.
		cam_vcm_power(0); //Turn off VCM/
#else

#if 0  //origin code
    	if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }     	
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }                    
#endif
#if 1 //edit code
        if(mt_set_gpio_mode(GPIO_CAM_AVDD_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAM_AVDD_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AVDD_LDO_EN_PIN failed!! \n");}
        
        if(mt_set_gpio_mode(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAM_DVDD_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_DVDD_LDO_EN_PIN failed!! \n");}
        
        if(mt_set_gpio_mode(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAM_IOVDD_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_IOVDD_LDO_EN_PIN failed!! \n");}

        if(pinSetIdx == 0){ //main sensor
            if(mt_set_gpio_mode(GPIO_CAM_AF_LDO_EN_PIN, GPIO_MODE_00)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAM_AF_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAM_AF_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[[CAMERA SENSOR] set GPIO_CAM_AF_LDO_EN_PIN failed!! \n");}
        }
#endif

#endif
        //MIPI SWITCH END SET
        if(mt_set_gpio_mode(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_MODE_00)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAM_MIPI_SW_SEL_pin,GPIO_OUT_ZERO)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_SEL failed!! \n");} //SEL low == main
        if(mt_set_gpio_mode(GPIO_CAM_MIPI_SW_OE_N_pin,GPIO_MODE_00)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_OE_N mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAM_MIPI_SW_OE_N_pin,GPIO_DIR_OUT)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_OE_N dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAM_MIPI_SW_OE_N_pin,GPIO_OUT_ONE)){PK_DBG("[CAMERA MIPI SW] set GPIO_CAM_MIPI_SW_OE_N failed!!\n");} //high == disconnect
    }//

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);



