/*****************************************************************************
 *
 * Filename:
 * ---------
 *   IMX119mipiraw_sensor.c
 *
 * Project:
 * --------
 *   RAW
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *

 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx119mipiraw_Sensor.h"
#include "imx119mipiraw_Camera_Sensor_para.h"
#include "imx119mipiraw_CameraCustomized.h"

kal_bool  IMX119MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool IMX119MIPI_Auto_Flicker_mode = KAL_FALSE;


kal_uint8 IMX119MIPI_sensor_write_I2C_address = IMX119MIPI_WRITE_ID;
kal_uint8 IMX119MIPI_sensor_read_I2C_address = IMX119MIPI_READ_ID;


static struct IMX119MIPI_sensor_STRUCT IMX119MIPI_sensor={
	IMX119MIPI_WRITE_ID,	// kal_uint16 i2c_write_id;
	IMX119MIPI_READ_ID,		// kal_uint16 i2c_read_id;
	KAL_TRUE,				// kal_bool first_init;
	KAL_FALSE,				// kal_bool fix_video_fps;
	KAL_TRUE,				// kal_bool pv_mode; 				//True: Preview Mode; False: Capture Mode
	KAL_FALSE,				// kal_bool night_mode;				//True: Night Mode; False: Auto Mode
	KAL_FALSE,				// kal_uint8 mirror_flip;
	45600000,				// kal_uint32 pv_pclk;				//Preview Pclk
	45600000,				// kal_uint32 cp_pclk;				//Capture Pclk
	0,					// kal_uint32 pv_shutter;		   
	0,						// kal_uint32 cp_shutter;
	64,						// kal_uint32 pv_gain;
	64,						// kal_uint32 cp_gain;
	1392,					// kal_uint32 pv_line_length; 	// 0x570
	1064,					// kal_uint32 pv_frame_length; 	// 0x432
	1392,					// kal_uint32 cp_line_length; 	// 0x570
	1064,					// kal_uint32 cp_frame_length; 	// 0x432
	0,						// kal_uint16 pv_dummy_pixels;		   //Dummy Pixels:must be 12s
	0,						// kal_uint16 pv_dummy_lines;		   //Dummy Lines
	0,						// kal_uint16 cp_dummy_pixels;		   //Dummy Pixels:must be 12s
	0,						// kal_uint16 cp_dummy_lines;		   //Dummy Lines			
	30						// kal_uint16 video_current_frame_rate;
};

MSDK_SCENARIO_ID_ENUM CurrentSubScenarioId = ACDK_SCENARIO_ID_CAMERA_PREVIEW;


kal_uint16  IMX119MIPI_sensor_gain_base=0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 IMX119MIPI_MAX_EXPOSURE_LINES = IMX119MIPI_PV_FRAME_LENGTH_LINES-3;//650;
kal_uint8  IMX119MIPI_MIN_EXPOSURE_LINES = 2;
kal_uint32 IMX119MIPI_isp_master_clock;
static DEFINE_SPINLOCK(imx119_drv_lock);

#define SENSORDB(fmt, arg...) printk( "[IMX119MIPIRaw] "  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT
UINT8 IMX119MIPIPixelClockDivider=0;
kal_uint16 IMX119MIPI_sensor_id=0;
MSDK_SENSOR_CONFIG_STRUCT IMX119MIPISensorConfigData;
kal_uint32 IMX119MIPI_FAC_SENSOR_REG;
kal_uint16 IMX119MIPI_sensor_flip_value; 
#define IMX119MIPI_MaxGainIndex 87																				 // Gain Index
kal_uint16 IMX119MIPI_sensorGainMapping[IMX119MIPI_MaxGainIndex][2] = {
    {64 ,  0}, {68 , 12}, {71 , 23}, {72 , 33}, { 74, 42}, {77 , 51}, { 80, 59}, { 84, 66}, { 87, 74}, { 91, 79},
    {93 , 85}, {96 , 91}, {100, 96}, {103,102}, {107,105}, {109,110}, {113,114}, {116,118}, {119,122}, {123,125},
    {126,128}, {128,131}, {132,134}, {135,137}, {138,139}, {141,142}, {144,145}, {148,148}, {152,150}, {155,152},
    {161,154}, {164,156}, {168,158}, {171,160}, {173,161}, {176,163}, {180,165}, {182,166}, {187,168}, {191,170}, 
    {193,171}, {196,172}, {198,173}, {200,174}, {203,175}, {208,177}, {213,179}, {216,180}, {219,181}, {222,182},    
	{225,183}, {228,184}, {232,185}, {235,186}, {238,187}, {241,188}, {245,189}, {249,190}, {253,191}, {256,192}, 
	{260,193}, {265,194}, {269,195}, {274,196}, {278,197}, {283,198}, {293,200}, {298,201}, {304,202}, {310,203},
	{315,204}, {328,206}, {336,207}, {342,208}, {349,209}, {357,210}, {365,211}, {373,212}, {381,213}, {400,215}, 
	{420,217}, {432,218}, {443,219}, {468,221}, {480,222}, {497,223}, {512,224}};

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT IMX119MIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT IMX119MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//extern int TempSubCamReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
//extern int TempSubCamWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define IMX119MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX119MIPI_WRITE_ID)
//#define IMX119MIPI_write_cmos_sensor(addr, para) TempSubCamWriteReg((u16) addr , (u32) para , 1, IMX119MIPI_WRITE_ID)

kal_uint16 IMX119MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,IMX119MIPI_WRITE_ID);
    //TempSubCamReadReg((u16) addr ,(u8*)&get_byte,IMX119MIPI_WRITE_ID);
    return get_byte;
}

void IMX119MIPI_write_shutter(kal_uint16 shutter)
{
	SENSORDB("[IMX119MIPI]enter %s()\n",__FUNCTION__);
	kal_uint32 frame_length = 0,line_length=0;
    kal_uint32 extra_lines = 0;
    kal_uint32 framerate = 0;
	kal_uint32 max_exp_shutter = 0;
	unsigned long flags;
    if (IMX119MIPI_sensor.pv_mode == KAL_TRUE) 
	 {
	   max_exp_shutter = IMX119MIPI_PV_FRAME_LENGTH_LINES + IMX119MIPI_sensor.pv_dummy_lines-3;
     }
     else
     {
       max_exp_shutter = IMX119MIPI_FULL_FRAME_LENGTH_LINES + IMX119MIPI_sensor.cp_dummy_lines-3;
	 }	 
	 if(shutter > max_exp_shutter)
	   extra_lines = shutter - max_exp_shutter;
	 else 
	   extra_lines = 0;
	 if (IMX119MIPI_sensor.pv_mode == KAL_TRUE) 
	 {
       frame_length =IMX119MIPI_PV_FRAME_LENGTH_LINES+ IMX119MIPI_sensor.pv_dummy_lines + extra_lines;
	   line_length = IMX119MIPI_PV_LINE_LENGTH_PIXELS+ IMX119MIPI_sensor.pv_dummy_pixels;
	   framerate = (10 * IMX119MIPI_sensor.pv_pclk) / (frame_length * line_length);
	   if(IMX119MIPI_Auto_Flicker_mode==KAL_TRUE)
	   {
		   if(framerate>=285)
		   	{
		   	   framerate=285;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=220&&framerate<285)
		   	{
		   	   framerate=220;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=194&&framerate<220)
		   	{
		   	   framerate=194;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=153&&framerate<194)
		   	{
		   	   framerate=153;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=147&&framerate<153)
		   	{
		   	   framerate=147;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=127&&framerate<147)
		   	{
		   	   framerate=127;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=118&&framerate<127)
		   	{
		   	   framerate=118;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=102&&framerate<118)
		   	{
		   	   framerate=102;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=99&&framerate<102)
		   	{
		   	   framerate=99;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=96&&framerate<99)
		   	{
		   	   framerate=96;
			   frame_length = (10 * IMX119MIPI_sensor.pv_pclk) / (framerate * line_length);
		   	}
	   }
	   spin_lock_irqsave(&imx119_drv_lock,flags);
	   IMX119MIPI_sensor.pv_line_length = line_length;
	   IMX119MIPI_sensor.pv_frame_length = frame_length;
	   spin_unlock_irqrestore(&imx119_drv_lock,flags);
	 }
	 else
     {
	    frame_length = IMX119MIPI_FULL_FRAME_LENGTH_LINES+ IMX119MIPI_sensor.cp_dummy_lines + extra_lines;
		line_length =IMX119MIPI_FULL_LINE_LENGTH_PIXELS + IMX119MIPI_sensor.cp_dummy_pixels;
		spin_lock_irqsave(&imx119_drv_lock,flags);
		framerate = (10 * IMX119MIPI_sensor.cp_pclk) / (frame_length * line_length);
		if(IMX119MIPI_Auto_Flicker_mode==KAL_TRUE)
		{
		   if(framerate>=285)
		   	{
		   	   framerate=285;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=220&&framerate<285)
		   	{
		   	   framerate=220;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=194&&framerate<220)
		   	{
		   	   framerate=194;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=153&&framerate<194)
		   	{
		   	   framerate=153;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=147&&framerate<153)
		   	{
		   	   framerate=147;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=127&&framerate<147)
		   	{
		   	   framerate=127;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=118&&framerate<127)
		   	{
		   	   framerate=118;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=102&&framerate<118)
		   	{
		   	   framerate=102;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=99&&framerate<102)
		   	{
		   	   framerate=99;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		   else if(framerate>=96&&framerate<99)
		   	{
		   	   framerate=96;
			   frame_length = (10 * IMX119MIPI_sensor.cp_pclk) / (framerate * line_length);
		   	}
		}
		IMX119MIPI_sensor.cp_line_length = line_length;
	    IMX119MIPI_sensor.cp_frame_length = frame_length;
		spin_unlock_irqrestore(&imx119_drv_lock,flags);
	 } 
    SENSORDB("[IMX119MIPI]Write_shutter:pv_mode =%d,shutter=%d,frame_length=%d\n",IMX119MIPI_sensor.pv_mode,shutter,frame_length);
    IMX119MIPI_write_cmos_sensor(0x0104, 1);        
	IMX119MIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
    IMX119MIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	

    IMX119MIPI_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
    IMX119MIPI_write_cmos_sensor(0x0203, shutter  & 0xFF);
    IMX119MIPI_write_cmos_sensor(0x0104, 0);  
	SENSORDB("[IMX119MIPI]exit %s()\n",__FUNCTION__);
}   /* write_IMX119MIPI_shutter */
static kal_uint16 IMX119MIPIReg2Gain(const kal_uint8 iReg)
{
    SENSORDB("[IMX119MIPI]enter IMX119MIPIReg2Gain function ireg=%d \n",iReg);
    kal_uint8 iI;
    // Range: 1x to 8x
    for (iI = 0; iI < IMX119MIPI_MaxGainIndex; iI++) 
	{
        if(iReg < IMX119MIPI_sensorGainMapping[iI][1])
		{
            break;
        }
		if(iReg == IMX119MIPI_sensorGainMapping[iI][1])			
		{			
			return IMX119MIPI_sensorGainMapping[iI][0];
		}    
    }
	SENSORDB("[IMX119MIPI]exit IMX119MIPIReg2Gain function\n");
    return IMX119MIPI_sensorGainMapping[iI-1][0];
}
static kal_uint8 IMX119MIPIGain2Reg(const kal_uint16 iGain)
{
	kal_uint8 iI;
    SENSORDB("[IMX119MIPI]enter IMX119MIPIGain2Reg function\n");
    for (iI = 0; iI < (IMX119MIPI_MaxGainIndex-1); iI++) 
	{
        if(iGain <= IMX119MIPI_sensorGainMapping[iI][0])
		{    
            break;
        }
    }
    if(iGain != IMX119MIPI_sensorGainMapping[iI][0])
    {
         printk("[IMX119MIPIGain2Reg] Gain mapping don't correctly:%d %d \n", iGain, IMX119MIPI_sensorGainMapping[iI][0]);
    }
	SENSORDB("[IMX119MIPI]exit IMX119MIPIGain2Reg function\n");
    return IMX119MIPI_sensorGainMapping[iI-1][1];
}

/*************************************************************************
* FUNCTION
*    IMX119MIPI_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX119MIPI_SetGain(UINT16 iGain)
{
    kal_uint8 iReg;
	printk("[IMX119MIPI_SetGain] SetGain:%d\n",  iGain);
    iReg = IMX119MIPIGain2Reg(iGain);
	printk("[IMX119MIPI_SetGain ] RegisterGain:%d\n", iReg);
	IMX119MIPI_write_cmos_sensor(0x0104,0x01);
    IMX119MIPI_write_cmos_sensor(0x0205,(kal_uint8)iReg);
    IMX119MIPI_write_cmos_sensor(0x0104,0x00);
}   /*  IMX119MIPI_SetGain_SetGain  */
/*************************************************************************
* FUNCTION
*    read_IMX119MIPI_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_IMX119MIPI_gain(void)
{	
    return (kal_uint16)((IMX119MIPI_read_cmos_sensor(0x0204)<<8) | IMX119MIPI_read_cmos_sensor(0x0205)) ;
}  /* read_IMX119MIPI_gain */

void write_IMX119MIPI_gain(kal_uint16 gain)
{
    IMX119MIPI_SetGain(gain);
}
void IMX119MIPI_camera_para_to_sensor(void)
{

	kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=IMX119MIPISensorReg[i].Addr; i++)
    {
        IMX119MIPI_write_cmos_sensor(IMX119MIPISensorReg[i].Addr, IMX119MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=IMX119MIPISensorReg[i].Addr; i++)
    {
        IMX119MIPI_write_cmos_sensor(IMX119MIPISensorReg[i].Addr, IMX119MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        IMX119MIPI_write_cmos_sensor(IMX119MIPISensorCCT[i].Addr, IMX119MIPISensorCCT[i].Para);
    }

}


/*************************************************************************
* FUNCTION
*    IMX119MIPI_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX119MIPI_sensor_to_camera_para(void)
{

	kal_uint32    i,temp_data;
    for(i=0; 0xFFFFFFFF!=IMX119MIPISensorReg[i].Addr; i++)
    {
		temp_data=IMX119MIPI_read_cmos_sensor(IMX119MIPISensorReg[i].Addr);
		spin_lock(&imx119_drv_lock);
		IMX119MIPISensorReg[i].Para = temp_data;
		spin_unlock(&imx119_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=IMX119MIPISensorReg[i].Addr; i++)
    {
    	temp_data=IMX119MIPI_read_cmos_sensor(IMX119MIPISensorReg[i].Addr);
         spin_lock(&imx119_drv_lock);
        IMX119MIPISensorReg[i].Para = temp_data;
		spin_unlock(&imx119_drv_lock);
    }

}

/*************************************************************************
* FUNCTION
*    IMX119MIPI_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  IMX119MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void IMX119MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void IMX119MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 SENSORDB("[IMX105MIPI][Error]get_sensor_item_info error!!!\n");
          }
           	spin_lock(&imx119_drv_lock);    
            temp_para=IMX119MIPISensorCCT[temp_addr].Para;	
			spin_unlock(&imx119_drv_lock);
            temp_gain = IMX119MIPIReg2Gain(temp_para);
            temp_gain=(temp_gain*1000)/BASEGAIN;
            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min=1000;
            info_ptr->Max=8192;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                    //temp_reg=IMX119MIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }
                
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=IMX119MIPI_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}
kal_bool IMX119MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 SENSORDB("[IMX119MIPI][Error]set_sensor_item_info error!!!\n");
          }
            temp_para = IMX119MIPIGain2Reg(ItemValue);
            spin_lock(&imx119_drv_lock);    
            IMX119MIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&imx119_drv_lock);
            IMX119MIPI_write_cmos_sensor(IMX119MIPISensorCCT[temp_addr].Addr,temp_para);
			temp_para=read_IMX119MIPI_gain();	
            spin_lock(&imx119_drv_lock);    
            IMX119MIPI_sensor_gain_base=temp_para;
			spin_unlock(&imx119_drv_lock);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {			
                    spin_lock(&imx119_drv_lock);    
                        IMX119MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
					spin_unlock(&imx119_drv_lock);
                        //IMX119MIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                    	spin_lock(&imx119_drv_lock);    
                        IMX119MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
						spin_unlock(&imx119_drv_lock);
                        //IMX119MIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                    	spin_lock(&imx119_drv_lock);    
                        IMX119MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
						spin_unlock(&imx119_drv_lock);
                        //IMX119MIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                    	spin_lock(&imx119_drv_lock);    
                        IMX119MIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
						spin_unlock(&imx119_drv_lock);
                        //IMX119MIPI_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&imx119_drv_lock);    
                    IMX119MIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&imx119_drv_lock);
                    break;
                case 1:
                    IMX119MIPI_write_cmos_sensor(IMX119MIPI_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void IMX119MIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
  kal_uint32 frame_length = 0, line_length = 0;
   if(IMX119MIPI_sensor.pv_mode == KAL_TRUE)
   	{
   	 spin_lock(&imx119_drv_lock);    
   	 IMX119MIPI_sensor.pv_dummy_pixels = iPixels;
	 IMX119MIPI_sensor.pv_dummy_lines = iLines;
   	 IMX119MIPI_sensor.pv_line_length = IMX119MIPI_PV_LINE_LENGTH_PIXELS + iPixels;
	 IMX119MIPI_sensor.pv_frame_length = IMX119MIPI_PV_FRAME_LENGTH_LINES + iLines;
	 spin_unlock(&imx119_drv_lock);
	 line_length = IMX119MIPI_sensor.pv_line_length;
	 frame_length = IMX119MIPI_sensor.pv_frame_length;
	 	
   	}
   else
   	{
	  spin_lock(&imx119_drv_lock);	
   	  IMX119MIPI_sensor.cp_dummy_pixels = iPixels;
	  IMX119MIPI_sensor.cp_dummy_lines = iLines;
	  IMX119MIPI_sensor.cp_line_length = IMX119MIPI_FULL_LINE_LENGTH_PIXELS + iPixels;
	  IMX119MIPI_sensor.cp_frame_length = IMX119MIPI_FULL_FRAME_LENGTH_LINES + iLines;
	   spin_unlock(&imx119_drv_lock);
	  line_length = IMX119MIPI_sensor.cp_line_length;
	  frame_length = IMX119MIPI_sensor.cp_frame_length;
    }
      IMX119MIPI_write_cmos_sensor(0x0104,0x01);        	  
      IMX119MIPI_write_cmos_sensor(0x0340, (frame_length >>8) & 0xFF);
      IMX119MIPI_write_cmos_sensor(0x0341, frame_length & 0xFF);	
      IMX119MIPI_write_cmos_sensor(0x0342, (line_length >>8) & 0xFF);
      IMX119MIPI_write_cmos_sensor(0x0343, line_length & 0xFF);
      IMX119MIPI_write_cmos_sensor(0x0104, 0);
	  SENSORDB("[IMX119MIPI]%s(),dumy_pixel=%d,dumy_line=%d,\n",__FUNCTION__,iPixels,iLines);
	  SENSORDB("[IMX119MIPI]pv_mode=%d,line_length=%d,frame_length=%d,\n",IMX119MIPI_sensor.pv_mode,line_length,frame_length);
	  SENSORDB("[IMX119MIPI]0x340=%x,0x341=%x\n",IMX119MIPI_read_cmos_sensor(0x0340),IMX119MIPI_read_cmos_sensor(0x0341));
	  SENSORDB("[IMX119MIPI]0x342=%x,0x343=%x\n",IMX119MIPI_read_cmos_sensor(0x0342),IMX119MIPI_read_cmos_sensor(0x0343));
}   /*  IMX119MIPI_SetDummy */
UINT32 IMX119SetMaxFrameRate(UINT16 u2FrameRate)
{

	kal_uint16 FrameHeight = 0;		
	SENSORDB("[IMX119SetMaxFrameRate]u2FrameRate=%d",u2FrameRate);
	if(IMX119MIPI_sensor.pv_mode==KAL_TRUE || IMX119MIPI_MPEG4_encode_mode==KAL_TRUE  )

	{
     	FrameHeight = (kal_uint16)((IMX119MIPI_sensor.pv_pclk*10/u2FrameRate)/IMX119MIPI_sensor.pv_line_length);		
		if (FrameHeight > IMX119MIPI_PV_FRAME_LENGTH_LINES/*IMX119MIPI_sensor.pv_frame_length*/) 
		{
			spin_lock(&imx119_drv_lock);    
			IMX119MIPI_sensor.pv_frame_length = FrameHeight;
			IMX119MIPI_sensor.pv_dummy_lines = IMX119MIPI_sensor.pv_frame_length-IMX119MIPI_PV_FRAME_LENGTH_LINES;
			spin_unlock(&imx119_drv_lock);
			SENSORDB("[IMX119MIPI]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,IMX119MIPI_sensor.pv_frame_length,IMX119MIPI_sensor.pv_dummy_lines);
			IMX119MIPI_SetDummy(IMX119MIPI_sensor.pv_dummy_pixels,IMX119MIPI_sensor.pv_dummy_lines);
		}
	}
	else
	{
		FrameHeight = (kal_uint16)((IMX119MIPI_sensor.cp_pclk*10/u2FrameRate)/IMX119MIPI_sensor.cp_line_length);
		if (FrameHeight > IMX119MIPI_FULL_FRAME_LENGTH_LINES/*IMX119MIPI_sensor.pv_frame_length*/) 
		{
			spin_lock(&imx119_drv_lock);    
			IMX119MIPI_sensor.cp_frame_length = FrameHeight;
			IMX119MIPI_sensor.cp_dummy_lines = IMX119MIPI_sensor.cp_frame_length-IMX119MIPI_FULL_FRAME_LENGTH_LINES;
			spin_unlock(&imx119_drv_lock);
			SENSORDB("[IMX111MIPI]%s():frame_length=%d,dummy_lines=%d\n",__FUNCTION__,IMX119MIPI_sensor.cp_frame_length,IMX119MIPI_sensor.cp_dummy_lines);
			IMX119MIPI_SetDummy(IMX119MIPI_sensor.cp_dummy_pixels,IMX119MIPI_sensor.cp_dummy_lines);
		}
	}
}

static void IMX119MIPI_Sensor_Init(void)
{
	printk("[IMX119MIPIRaw] Set IMX119MIPI_Sensor_Init setting  start\n"); 
	IMX119MIPI_write_cmos_sensor(0x0100,0x00);
	IMX119MIPI_write_cmos_sensor(0x0305,0x01);
	IMX119MIPI_write_cmos_sensor(0x0307,0x13);
	IMX119MIPI_write_cmos_sensor(0x3025,0x0A);
	IMX119MIPI_write_cmos_sensor(0x302B,0x4B);
	IMX119MIPI_write_cmos_sensor(0x0112,0x0A);
	IMX119MIPI_write_cmos_sensor(0x0113,0x0A);
	IMX119MIPI_write_cmos_sensor(0x0101,0x03); // xy flip
	IMX119MIPI_write_cmos_sensor(0x301C,0x02);
	IMX119MIPI_write_cmos_sensor(0x302C,0x85);
	IMX119MIPI_write_cmos_sensor(0x303A,0xA4);
	IMX119MIPI_write_cmos_sensor(0x3108,0x25);
	IMX119MIPI_write_cmos_sensor(0x310A,0x27);
	IMX119MIPI_write_cmos_sensor(0x3122,0x26);
	IMX119MIPI_write_cmos_sensor(0x3138,0x26);
	IMX119MIPI_write_cmos_sensor(0x313A,0x27);
	IMX119MIPI_write_cmos_sensor(0x316D,0x0A);
	IMX119MIPI_write_cmos_sensor(0x308C,0x00);
	IMX119MIPI_write_cmos_sensor(0x302E,0x8C);
	IMX119MIPI_write_cmos_sensor(0x302F,0x81);
	IMX119MIPI_write_cmos_sensor(0x0340,0x04);
	IMX119MIPI_write_cmos_sensor(0x0341,0x28);
	IMX119MIPI_write_cmos_sensor(0x0342,0x05);
	IMX119MIPI_write_cmos_sensor(0x0343,0x70);
	IMX119MIPI_write_cmos_sensor(0x0346,0x00);
	IMX119MIPI_write_cmos_sensor(0x0347,0x00);
	IMX119MIPI_write_cmos_sensor(0x034A,0x04);
	IMX119MIPI_write_cmos_sensor(0x034B,0x0F);
	IMX119MIPI_write_cmos_sensor(0x034C,0x05);
	IMX119MIPI_write_cmos_sensor(0x034D,0x10);
	IMX119MIPI_write_cmos_sensor(0x034E,0x04);
	IMX119MIPI_write_cmos_sensor(0x034F,0x10);
	IMX119MIPI_write_cmos_sensor(0x0381,0x01);
	IMX119MIPI_write_cmos_sensor(0x0383,0x01);
	IMX119MIPI_write_cmos_sensor(0x0385,0x01);
	IMX119MIPI_write_cmos_sensor(0x0387,0x01);
	IMX119MIPI_write_cmos_sensor(0x3001,0x00);
	IMX119MIPI_write_cmos_sensor(0x3016,0x02);
	IMX119MIPI_write_cmos_sensor(0x3060,0x30);
	IMX119MIPI_write_cmos_sensor(0x30E8,0x00);
	IMX119MIPI_write_cmos_sensor(0x3301,0x05);
	IMX119MIPI_write_cmos_sensor(0x308A,0x43);
	IMX119MIPI_write_cmos_sensor(0x3305,0x03);
	IMX119MIPI_write_cmos_sensor(0x3309,0x06);
	IMX119MIPI_write_cmos_sensor(0x330B,0x03);
	IMX119MIPI_write_cmos_sensor(0x330D,0x06);
	IMX119MIPI_write_cmos_sensor(0x0100,0x01);

    spin_lock(&imx119_drv_lock);  
    IMX119MIPI_Auto_Flicker_mode = KAL_FALSE;     // reset the flicker status    
	spin_unlock(&imx119_drv_lock);
     printk("[IMX119MIPIRaw] Set IMX119MIPI_Sensor_Init setting  end\n"); 
}   /*  IMX119MIPI_Sensor_Init  */
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   IMX119MIPIOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 IMX119MIPIOpen(void)
{
    int  retry = 0; 
    // check if sensor ID correct
    retry = 3; 
    do {
       SENSORDB("Read ID in the Open function"); 
	   //spin_lock(&imx119_drv_lock);    
	   IMX119MIPI_sensor_id =(kal_uint16)((IMX119MIPI_read_cmos_sensor(0x0000)<<8) | IMX119MIPI_read_cmos_sensor(0x0001));  
	   //spin_unlock(&imx119_drv_lock);
		if (IMX119MIPI_sensor_id == IMX119MIPI_SENSOR_ID)
		break; 
		SENSORDB("Read Sensor ID Fail = 0x%04x\n", IMX119MIPI_sensor_id); 
		retry--; 
	    }
	while (retry > 0);
    SENSORDB("Read Sensor ID = 0x%04x\n", IMX119MIPI_sensor_id); 
    if (IMX119MIPI_sensor_id != IMX119MIPI_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;
    IMX119MIPI_Sensor_Init();
	//spin_lock(&imx119_drv_lock);	
    IMX119MIPI_sensor_gain_base = read_IMX119MIPI_gain();
	//spin_unlock(&imx119_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   IMX119MIPIGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX119MIPIGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
    // check if sensor ID correct
    do {		
	   *sensorID =(kal_uint16)((IMX119MIPI_read_cmos_sensor(0x0000)<<8) | IMX119MIPI_read_cmos_sensor(0x0001)); 
        if (*sensorID == IMX119MIPI_SENSOR_ID)
            break;
        SENSORDB("Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        retry--; 
		
    } while (retry > 0);

    if (*sensorID != IMX119MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   IMX119MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of IMX119MIPI to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX119MIPI_SetShutter(kal_uint16 iShutter)
{

	 SENSORDB("[IMX119MIPI]%s():shutter=%d\n",__FUNCTION__,iShutter);
   
    if (iShutter < 1)
        iShutter = 1; 
	else if(iShutter > 0xffff)
		iShutter = 0xffff;
	unsigned long flags;
	spin_lock_irqsave(&imx119_drv_lock,flags);
    IMX119MIPI_sensor.pv_shutter = iShutter;	
	spin_unlock_irqrestore(&imx119_drv_lock,flags);
    IMX119MIPI_write_shutter(iShutter);
}   /*  IMX119MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   IMX119MIPI_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 IMX119MIPI_read_shutter(void)
{
    return (UINT16)( (IMX119MIPI_read_cmos_sensor(0x0202)<<8) | IMX119MIPI_read_cmos_sensor(0x0203) );
}

/*************************************************************************
* FUNCTION
*   IMX119MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of IMX119MIPI.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void IMX119MIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("[IMX119MIPI]enter IMX119MIPI_NightMode");

}/*	IMX119MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   IMX119MIPIClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX119MIPIClose(void)
{
    //IMX119MIPI_write_cmos_sensor(0x0100,0x00);
    return ERROR_NONE;
}	/* IMX119MIPIClose() */

void IMX119MIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
	#if 0
    iTemp = IMX119MIPI_read_cmos_sensor(0x0101) & 0x03;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            IMX119MIPI_write_cmos_sensor(0x0101, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            IMX119MIPI_write_cmos_sensor(0x0101, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            IMX119MIPI_write_cmos_sensor(0x0101, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            IMX119MIPI_write_cmos_sensor(0x0101, 0x00);	//Set mirror and flip
            break;
    }
	#endif
}


/*************************************************************************
* FUNCTION
*   IMX119MIPIPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 IMX119MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;
	spin_lock(&imx119_drv_lock);    
	IMX119MIPI_sensor.pv_mode=KAL_TRUE;
	spin_unlock(&imx119_drv_lock);
    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
    	spin_lock(&imx119_drv_lock);    
        IMX119MIPI_MPEG4_encode_mode = KAL_TRUE;   
		spin_unlock(&imx119_drv_lock);
    }
    else
    {
    	spin_lock(&imx119_drv_lock);    
        IMX119MIPI_MPEG4_encode_mode = KAL_FALSE;
		spin_unlock(&imx119_drv_lock);
    }
    iStartX += IMX119MIPI_IMAGE_SENSOR_PV_STARTX;
    iStartY += IMX119MIPI_IMAGE_SENSOR_PV_STARTY;
	spin_lock(&imx119_drv_lock);	
    IMX119MIPI_sensor.cp_dummy_pixels = 0;
    IMX119MIPI_sensor.cp_dummy_lines = 0;
    IMX119MIPI_sensor.pv_dummy_pixels = 0;
    IMX119MIPI_sensor.pv_dummy_lines = 0;
	IMX119MIPI_sensor.pv_line_length = IMX119MIPI_PV_LINE_LENGTH_PIXELS+IMX119MIPI_sensor.pv_dummy_pixels; 
	IMX119MIPI_sensor.pv_frame_length = IMX119MIPI_PV_FRAME_LENGTH_LINES+IMX119MIPI_sensor.pv_dummy_lines;
    spin_unlock(&imx119_drv_lock);
	
	IMX119MIPI_SetDummy(IMX119MIPI_sensor.pv_dummy_pixels,IMX119MIPI_sensor.pv_dummy_lines);
	IMX119MIPI_SetShutter(IMX119MIPI_sensor.pv_shutter);
    memcpy(&IMX119MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    image_window->GrabStartX= iStartX;
    image_window->GrabStartY= iStartY;
    image_window->ExposureWindowWidth= IMX119MIPI_REAL_PV_WIDTH ;
    image_window->ExposureWindowHeight= IMX119MIPI_REAL_PV_HEIGHT ;
    SENSORDB("Preview resolution:%d %d %d %d\n", image_window->GrabStartX, image_window->GrabStartY, image_window->ExposureWindowWidth, image_window->ExposureWindowHeight); 
    return ERROR_NONE;
}	/* IMX119MIPIPreview() */

UINT32 IMX119MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter=IMX119MIPI_sensor.pv_shutter;
    kal_uint16 iStartX = 0, iStartY = 0;
	spin_lock(&imx119_drv_lock);	
    IMX119MIPI_sensor.pv_mode=KAL_FALSE;
    IMX119MIPI_MPEG4_encode_mode = KAL_FALSE; 
    IMX119MIPI_Auto_Flicker_mode = KAL_FALSE;   
	spin_unlock(&imx119_drv_lock);
    SENSORDB("Preview Shutter = %d, Gain = %d\n", shutter, read_IMX119MIPI_gain());     

    if(sensor_config_data->EnableShutterTansfer==KAL_TRUE)
        shutter=sensor_config_data->CaptureShutter;
	spin_lock(&imx119_drv_lock);    
    IMX119MIPI_sensor.cp_dummy_pixels= 0;
    IMX119MIPI_sensor.cp_dummy_lines = 0;   
	spin_unlock(&imx119_drv_lock);  
    IMX119MIPISetFlipMirror(sensor_config_data->SensorImageMirror); 
	spin_lock(&imx119_drv_lock);    
	IMX119MIPI_sensor.cp_line_length=IMX119MIPI_FULL_LINE_LENGTH_PIXELS+IMX119MIPI_sensor.cp_dummy_pixels;
	IMX119MIPI_sensor.cp_frame_length=IMX119MIPI_FULL_FRAME_LENGTH_LINES+IMX119MIPI_sensor.cp_dummy_lines;
	spin_unlock(&imx119_drv_lock);
	shutter = (shutter * IMX119MIPI_sensor.pv_line_length)/IMX119MIPI_sensor.cp_line_length;
	SENSORDB("[IMX119MIPI]cp_shutter=%d,cp_length=%d\n",shutter,IMX119MIPI_sensor.cp_line_length); 
	iStartX = IMX119MIPI_IMAGE_SENSOR_CAP_STARTX;
	iStartY = IMX119MIPI_IMAGE_SENSOR_CAP_STARTY;
	image_window->GrabStartX=iStartX;
	image_window->GrabStartY=iStartY;
	image_window->ExposureWindowWidth=IMX119MIPI_REAL_CAP_WIDTH ;
	image_window->ExposureWindowHeight=IMX119MIPI_REAL_CAP_HEIGHT;
    sensor_config_data->Lines = image_window->ExposureWindowHeight;
    sensor_config_data->Shutter =shutter;
    IMX119MIPI_SetDummy(IMX119MIPI_sensor.cp_dummy_pixels, IMX119MIPI_sensor.cp_dummy_lines);
    IMX119MIPI_write_shutter(shutter);
    SENSORDB("Capture Shutter = %d, Gain = %d\n", shutter, read_IMX119MIPI_gain());     
	spin_lock(&imx119_drv_lock);	
    memcpy(&IMX119MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&imx119_drv_lock);
    return ERROR_NONE;
}	/* IMX119MIPICapture() */

UINT32 IMX119MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    pSensorResolution->SensorPreviewWidth	= IMX119MIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= IMX119MIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= IMX119MIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= IMX119MIPI_REAL_CAP_HEIGHT;
    SENSORDB("IMX119MIPIGetResolution :8-14");    

    return ERROR_NONE;
}   /* IMX119MIPIGetResolution() */

UINT32 IMX119MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch(ScenarioId){
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
				pSensorInfo->SensorPreviewResolutionX=IMX119MIPI_REAL_CAP_WIDTH;
				pSensorInfo->SensorPreviewResolutionY=IMX119MIPI_REAL_CAP_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=15;
			break;

		default:
        pSensorInfo->SensorPreviewResolutionX=IMX119MIPI_REAL_PV_WIDTH;
        pSensorInfo->SensorPreviewResolutionY=IMX119MIPI_REAL_PV_HEIGHT;
				pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}
//	pSensorInfo->SensorPreviewResolutionX=IMX119MIPI_REAL_CAP_WIDTH;
   //     pSensorInfo->SensorPreviewResolutionY=IMX119MIPI_REAL_CAP_HEIGHT;

    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=15;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    //pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gb;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    pSensorInfo->SensorDriver3D = 0;   // the sensor driver is 2D
    
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_2M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_2M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_05M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=TRUE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_05M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_05M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=TRUE;

    pSensorInfo->CaptureDelayFrame = 5; 
    pSensorInfo->PreviewDelayFrame = 2; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 1;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = IMX119MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = IMX119MIPI_IMAGE_SENSOR_PV_STARTY;           		
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	     pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	     pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = IMX119MIPI_IMAGE_SENSOR_CAP_STARTX;	//2*IMX119MIPI_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = IMX119MIPI_IMAGE_SENSOR_CAP_STARTY;	//2*IMX119MIPI_IMAGE_SENSOR_PV_STARTY;          			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 1; 
            pSensorInfo->SensorGrabStartY = 1;             
            break;
    }
	spin_lock(&imx119_drv_lock);	

    IMX119MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &IMX119MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&imx119_drv_lock);

    return ERROR_NONE;
}   /* IMX119MIPIGetInfo() */


UINT32 IMX119MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&imx119_drv_lock);	
		CurrentSubScenarioId = ScenarioId;
		spin_unlock(&imx119_drv_lock);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            IMX119MIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
            IMX119MIPICapture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return TRUE;
} /* IMX119MIPIControl() */

UINT32 IMX119MIPISetVideoMode(UINT16 u2FrameRate)
{
	SENSORDB("[IMX119MIPISetVideoMode] frame rate = %d\n", u2FrameRate);
	kal_uint16 IMX119MIPI_Video_Max_Expourse_Time = 0;
	spin_lock(&imx119_drv_lock);	
	IMX119MIPI_MPEG4_encode_mode = KAL_TRUE; 
	spin_unlock(&imx119_drv_lock);
	SENSORDB("[IMX119MIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
	if(u2FrameRate>=24)
	{
		u2FrameRate=30;
		spin_lock(&imx119_drv_lock);
		IMX119MIPI_sensor.night_mode=KAL_FALSE;
		spin_unlock(&imx119_drv_lock);
	}
	else
	{
		u2FrameRate=15;
		spin_lock(&imx119_drv_lock);
		IMX119MIPI_sensor.night_mode=KAL_FALSE;
		spin_unlock(&imx119_drv_lock);
	}
	spin_lock(&imx119_drv_lock);
	IMX119MIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&imx119_drv_lock);
	u2FrameRate=u2FrameRate*10;//10*FPS
	SENSORDB("[IMX119MIPI][Enter Fix_fps func] IMX119MIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
	IMX119SetMaxFrameRate(u2FrameRate);	
    return TRUE;
}

UINT32 IMX119MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	kal_uint32 pv_max_frame_rate_lines = IMX119MIPI_MAX_EXPOSURE_LINES;
    SENSORDB("[IMX119MIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) 
	{   // enable auto flicker   
    	spin_lock(&imx119_drv_lock);    
        IMX119MIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&imx119_drv_lock);
        if(IMX119MIPI_MPEG4_encode_mode == KAL_TRUE && IMX119MIPI_sensor.night_mode==KAL_FALSE) 
		{    // in the video mode, reset the frame rate
            IMX119SetMaxFrameRate(285);        	
        }
		else if((IMX119MIPI_MPEG4_encode_mode == KAL_TRUE && IMX119MIPI_sensor.night_mode==KAL_TRUE) ||(IMX119MIPI_sensor.pv_mode==KAL_FALSE))
		{
			IMX119SetMaxFrameRate(147);
		}
		else
		{			
			IMX119SetMaxFrameRate(285);
		}
    } 
	else 
	{
    	spin_lock(&imx119_drv_lock);    
        IMX119MIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&imx119_drv_lock);
        if(IMX119MIPI_MPEG4_encode_mode == KAL_TRUE && IMX119MIPI_sensor.night_mode==KAL_FALSE) 
		{    // in the video mode, reset the frame rate     
			IMX119SetMaxFrameRate(300);
        }
		else if((IMX119MIPI_MPEG4_encode_mode == KAL_TRUE && IMX119MIPI_sensor.night_mode==KAL_TRUE) ||(IMX119MIPI_sensor.pv_mode==KAL_FALSE))
		{
			IMX119SetMaxFrameRate(150);
		}
		else
		{		
			IMX119SetMaxFrameRate(300);
		}
        printk("Disable Auto flicker\n");    
    }
    return TRUE;
}

UINT32 IMX119MIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[IMX119MIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        IMX119MIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        IMX119MIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        IMX119MIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
        IMX119MIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return TRUE;
}

UINT32 IMX119MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=IMX119MIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=IMX119MIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(CurrentSubScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		  case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
 		            *pFeatureReturnPara16++=IMX119MIPI_sensor.cp_line_length;  
 		            *pFeatureReturnPara16=IMX119MIPI_sensor.cp_frame_length;
		            SENSORDB("Sensor period:%d %d\n",IMX119MIPI_sensor.cp_line_length, IMX119MIPI_sensor.cp_frame_length); 
		            *pFeatureParaLen=4;        				
        				break;
        			
        			default:	
					*pFeatureReturnPara16++=IMX119MIPI_sensor.pv_line_length;  
					*pFeatureReturnPara16=IMX119MIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", IMX119MIPI_sensor.pv_line_length, IMX119MIPI_sensor.pv_frame_length); 
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(CurrentSubScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara32 = IMX119MIPI_sensor.cp_pclk; //19500000;
		            *pFeatureParaLen=4;		         	
		         		break;
		         		
		         		default:
		            *pFeatureReturnPara32 = IMX119MIPI_sensor.pv_pclk;//57600000; //19500000;
		            *pFeatureParaLen=4;
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            IMX119MIPI_SetShutter(*pFeatureData16);
			break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC:
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            IMX119MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
           IMX119MIPI_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&imx119_drv_lock);    
            IMX119MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&imx119_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			//iWriteReg((u16) pSensorRegData->RegAddr , (u32) pSensorRegData->RegData , 1, 0x66);//to test the AF
			IMX119MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = IMX119MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&imx119_drv_lock);    
                IMX119MIPISensorCCT[i].Addr=*pFeatureData32++;
                IMX119MIPISensorCCT[i].Para=*pFeatureData32++; 
				spin_unlock(&imx119_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=IMX119MIPISensorCCT[i].Addr;
                *pFeatureData32++=IMX119MIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&imx119_drv_lock);    
                IMX119MIPISensorReg[i].Addr=*pFeatureData32++;
                IMX119MIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&imx119_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=IMX119MIPISensorReg[i].Addr;
                *pFeatureData32++=IMX119MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=IMX119MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, IMX119MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, IMX119MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &IMX119MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            IMX119MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            IMX119MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=IMX119MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            IMX119MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            IMX119MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            IMX119MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gb;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            IMX119MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            IMX119MIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            IMX119MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            IMX119MIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* IMX119MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncIMX119MIPI=
{
    IMX119MIPIOpen,
    IMX119MIPIGetInfo,
    IMX119MIPIGetResolution,
    IMX119MIPIFeatureControl,
    IMX119MIPIControl,
    IMX119MIPIClose
};

UINT32 IMX119_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncIMX119MIPI;

    return ERROR_NONE;
}   /* SensorInit() */

