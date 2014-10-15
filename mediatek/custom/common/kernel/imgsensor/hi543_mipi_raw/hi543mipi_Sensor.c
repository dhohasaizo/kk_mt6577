/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   YUSU
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Jackie Su (MTK02380)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 01 07 2013 kongming.hu
 * [ALPS00431401] camera preview is shown as dark on ISO 100
 * revise sensor driver
 *
 * 02 19 2012 koli.lin
 * [ALPS00237113] [Performance][Video recording]Recording preview the screen have flash
 * [Camera] 1. Modify the AE converge speed in the video mode.
 *                2. Modify the isp gain delay frame with sensor exposure time and gain synchronization.
 *
 
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
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
#include <linux/slab.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi543mipi_Sensor.h"
#include "hi543mipi_Camera_Sensor_para.h"
#include "hi543mipi_CameraCustomized.h"

//#define CAPTURE_15FPS
#define HI543MIPI_DEBUG
#ifdef HI543MIPI_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

struct HI543MIPI_SENSOR_STRUCT HI543MIPI_sensor= 
{	
	.i2c_write_id = 0x40,
	.i2c_read_id  = 0x41,
	
	.preview_vt_clk = 130000000,  //v0.07
	.capture_vt_clk = 130000000,

	.frame_length = HI543MIPI_FULL_PERIOD_LINE_NUMS,
	.line_length = HI543MIPI_FULL_PERIOD_PIXEL_NUMS,
	.shutter= 0x04a7,
};
#define AUTO_FLICKER_NO 10

kal_uint16 Hi543_FixedFps =29;
kal_bool HI543MIPI_Auto_Flicker_mode = KAL_FALSE;
kal_bool HI543MIPI_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 HI543_Frame_Length_preview = 0;


kal_uint16 HI543MIPI_dummy_pixels=0, HI543MIPI_dummy_lines=0;
kal_uint16 HI543MIPI_PV_dummy_pixels=0,HI543MIPI_PV_dummy_lines=0;

kal_uint16 HI543MIPI_exposure_lines=0x100;
kal_uint16 HI543MIPI_sensor_global_gain=BASEGAIN, HI543MIPI_sensor_gain_base=BASEGAIN;
kal_uint16 HI543MIPI_sensor_gain_array[2][5]={{0x0204,0x0208, 0x0206, 0x020C, 0x020A},{0x08,0x8, 0x8, 0x8, 0x8}};


unsigned short ini_frm_length = 0xffff; //                                                                 

MSDK_SENSOR_CONFIG_STRUCT HI543MIPISensorConfigData;
kal_uint32 HI543MIPI_FAC_SENSOR_REG;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HI543MIPISensorCCT[FACTORY_END_ADDR]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT HI543MIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
MSDK_SCENARIO_ID_ENUM HI543_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_ZSD;

typedef enum
{
  HI543MIPI_MODE_PREVIEW,  
  HI543MIPI_MODE_CAPTURE  
} HI543MIPI_MODE;
HI543MIPI_MODE g_iHI543MIPI_Mode = HI543MIPI_MODE_PREVIEW;


typedef enum
{
  HI543_720P,       //for video
  HI543_5M,     //for zsd 
} HI543_RES_TYPE;
HI543_RES_TYPE HI543_g_RES=HI543_720P;

static DEFINE_SPINLOCK(hi543mipi_drv_lock);



extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 HI543MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,HI543MIPI_WRITE_ID);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

void HI543MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{

	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 4,HI543MIPI_WRITE_ID);
}


kal_uint16 HI543MIPI_read_cmos_sensor_8(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,HI543MIPI_WRITE_ID);
    return get_byte;
}

void HI543MIPI_write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{

	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 3,HI543MIPI_WRITE_ID);
}




/*************************************************************************
* FUNCTION
*    read_HI543MIPI_gain
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
kal_uint16 read_HI543MIPI_gain(void)
{
     kal_uint16 reg_value=0x00;
	 kal_uint16 iGain=8;

     reg_value =HI543MIPI_read_cmos_sensor_8(0x0205);

	 iGain = 256*BASEGAIN/(reg_value + 32);

}  /* read_HI543MIPI_gain */

static kal_uint16 HI543Reg2Gain(const kal_uint8 iReg)
{
    kal_uint16 iGain = 64;    // 1x-gain base

	iGain = iGain*256/(iReg + 32);

    return iGain;
}

static kal_uint16 HI543Gain2Reg(const kal_uint8 iGain)
{
    kal_uint8 iReg;
	kal_uint8 gain_value;
	kal_uint8 min_gain_value=80;//1.25x basegain
	
    gain_value = iGain;
	
	if(gain_value < min_gain_value){
		gain_value=min_gain_value;
	}
	else if(gain_value > 8*BASEGAIN){
		gain_value=8*BASEGAIN;
	}

	printk("[HI543]:HI543Gain2Reg iGain :0x%x\n", iGain); 

	iReg = 256*BASEGAIN/gain_value - 32;
    return iReg;
}


void write_HI543MIPI_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
	kal_uint8 min_gain_value=80;//1.25x basegain
	
    //SENSORDB("HI543MIPI gain value from mtk =%d \n ",gain);
	
	if(gain < min_gain_value){
		gain=min_gain_value;
	}
	else if(gain > 8*BASEGAIN){
		gain=8*BASEGAIN;
	}

	
		reg_gain = (256*BASEGAIN/gain)-32;
		//SENSORDB("HI543MIPI gain value to reg : =%d \n ",reg_gain);

		//HI543MIPI_write_cmos_sensor_8(0x0104,0x01);
		HI543MIPI_write_cmos_sensor_8(0x0205,reg_gain);
		//HI543MIPI_write_cmos_sensor_8(0x0104,0x00);
	
}

/*************************************************************************
* FUNCTION
* set_HI543MIPI_gain
*
* DESCRIPTION
* This function is to set global gain to sensor.
*
* PARAMETERS
* gain : sensor global gain(base: 0x40)
*
* RETURNS
* the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 HI543_Set_gain(kal_uint16 gain)
{
      write_HI543MIPI_gain(gain);

}

/*******************************************************************************
* 
********************************************************************************/
void HI543MIPI_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=HI543MIPISensorReg[i].Addr; i++)
    {
        HI543MIPI_write_cmos_sensor_8(HI543MIPISensorReg[i].Addr, HI543MIPISensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI543MIPISensorReg[i].Addr; i++)
    {
        HI543MIPI_write_cmos_sensor_8(HI543MIPISensorReg[i].Addr, HI543MIPISensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HI543MIPI_write_cmos_sensor_8(HI543MIPISensorCCT[i].Addr, HI543MIPISensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    HI543MIPI_sensor_to_camera_para
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
void HI543MIPI_sensor_to_camera_para(void)
{
    kal_uint32    i,temp_data;
    for(i=0; 0xFFFFFFFF!=HI543MIPISensorReg[i].Addr; i++)
    {
        temp_data = HI543MIPI_read_cmos_sensor_8(HI543MIPISensorReg[i].Addr);
		
		spin_lock(&hi543mipi_drv_lock);
        HI543MIPISensorReg[i].Para = temp_data;
		spin_unlock(&hi543mipi_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI543MIPISensorReg[i].Addr; i++)
    {
        temp_data = HI543MIPI_read_cmos_sensor_8(HI543MIPISensorReg[i].Addr);
		
		spin_lock(&hi543mipi_drv_lock);
		HI543MIPISensorReg[i].Para = temp_data;
		spin_unlock(&hi543mipi_drv_lock);
		
    }
}


/*************************************************************************
* FUNCTION
*    HI543MIPI_get_sensor_group_count
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
kal_int32  HI543MIPI_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HI543MIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 5;
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

void HI543MIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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
                 ASSERT(0);
          }

            temp_para=HI543MIPISensorCCT[temp_addr].Para;

            temp_gain = HI543Reg2Gain(temp_para);

            temp_gain=(temp_gain*1000)/BASEGAIN;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min=1000;
            info_ptr->Max=15875;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                    //temp_reg=HI543MIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
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
                    info_ptr->ItemValue=    111;  //HI543MIPI_MAX_EXPOSURE_LINES;
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



kal_bool HI543MIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

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
                 ASSERT(0);
          }

            temp_para = HI543Gain2Reg(ItemValue);

			spin_lock(&hi543mipi_drv_lock);
            HI543MIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&hi543mipi_drv_lock);
			
            HI543MIPI_write_cmos_sensor_8(HI543MIPISensorCCT[temp_addr].Addr,temp_para);

			temp_para= read_HI543MIPI_gain();
			
			spin_lock(&hi543mipi_drv_lock);
			HI543MIPI_sensor_gain_base = temp_para;
			spin_unlock(&hi543mipi_drv_lock);
			
            break;
			
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
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
					spin_lock(&hi543mipi_drv_lock);
                    HI543MIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&hi543mipi_drv_lock);
                    break;
                case 1:
                    HI543MIPI_write_cmos_sensor_8(HI543MIPI_FAC_SENSOR_REG,ItemValue);
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


/*******************************************************************************
*
********************************************************************************/
static void HI543MIPI_Init_setting(void)
{
	//full size setting ,v0.08<=>20121106
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); //                                                                        
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
    HI543MIPI_write_cmos_sensor_8(0x0103, 0x01); 
    HI543MIPI_write_cmos_sensor_8(0x0103, 0x00); 

	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xE048, 0x28); 
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0106, 0x01); 
	mDELAY(5);	
	HI543MIPI_write_cmos_sensor_8(0x0101, 0x03); //                                                                        
	
	//---< PLL & Divider >---------------------------//
	////PLL Output clk control////
	//Mclk/Pclk = 24Mhz/130Mhz(=Each Lane = 650Mhz )
	HI543MIPI_write_cmos_sensor_8(0xf003, 0x3a); 
	HI543MIPI_write_cmos_sensor_8(0x0304, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0305, 0x18); 
	HI543MIPI_write_cmos_sensor_8(0x0306, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0x0307, 0x8A); 
	
	//---< Analog configuration >---------------------------//
	////Display Control////
	HI543MIPI_write_cmos_sensor_8(0x5400, 0x04); 
	HI543MIPI_write_cmos_sensor_8(0x0b04, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe039, 0x0F); 
	HI543MIPI_write_cmos_sensor_8(0xe04e, 0x22); 
	HI543MIPI_write_cmos_sensor_8(0xe051, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe053, 0xFF); 
	HI543MIPI_write_cmos_sensor_8(0xe041, 0x2B); 
	HI543MIPI_write_cmos_sensor_8(0xe042, 0xF9); 
	HI543MIPI_write_cmos_sensor_8(0xe043, 0x7F); 
	HI543MIPI_write_cmos_sensor_8(0xe044, 0xFF); 

	HI543MIPI_write_cmos_sensor_8(0xe001,0x4a); //FOR SHUTTER//GAIN delay setting;must set
	
	HI543MIPI_write_cmos_sensor_8(0xe0a1, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0xe0a2, 0x84); //                                                     
	HI543MIPI_write_cmos_sensor_8(0xe0ae, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe054, 0x55); 
	HI543MIPI_write_cmos_sensor_8(0xe04b, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0xe045, 0x7F); 
	HI543MIPI_write_cmos_sensor_8(0xe047, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe196, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe197, 0xD0); 
	HI543MIPI_write_cmos_sensor_8(0xe198, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe199, 0xDF); 
	HI543MIPI_write_cmos_sensor_8(0xe19a, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe19b, 0xD0); 
	HI543MIPI_write_cmos_sensor_8(0xe19c, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe19d, 0xD1); 
	HI543MIPI_write_cmos_sensor_8(0xe186, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe187, 0x40); 
	HI543MIPI_write_cmos_sensor_8(0xe188, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe189, 0xEA); 
	HI543MIPI_write_cmos_sensor_8(0xe162, 0x20); 
	HI543MIPI_write_cmos_sensor_8(0xe163, 0x2F); 
	HI543MIPI_write_cmos_sensor_8(0xe14d, 0x7C); 
	HI543MIPI_write_cmos_sensor_8(0xe17a, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe17d, 0x48); 
	HI543MIPI_write_cmos_sensor_8(0xe16e, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe16f, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe170, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0xe171, 0x0C); 
	HI543MIPI_write_cmos_sensor_8(0xe132, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe133, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe138, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe139, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe104, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe105, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe10c, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe10d, 0xBC); 
	
	////Dynamic Signal Setting Start --> Must set////
	HI543MIPI_write_cmos_sensor_8(0xe18a, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe18b, 0x90); 
	HI543MIPI_write_cmos_sensor_8(0xe18c, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0xe18d, 0x6c); 
	HI543MIPI_write_cmos_sensor_8(0xe18e, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe18f, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe190, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe191, 0x50); 
	HI543MIPI_write_cmos_sensor_8(0xe05d, 0xff); 
	HI543MIPI_write_cmos_sensor_8(0xe05a, 0xbb); 
	HI543MIPI_write_cmos_sensor_8(0xe05b, 0xcc); 
	HI543MIPI_write_cmos_sensor_8(0xe0a0, 0x00); //                                                     
	HI543MIPI_write_cmos_sensor_8(0xe0b4, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b5, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b6, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b7, 0x09);

#if 0
	//Analog Gain control //
	HI543MIPI_write_cmos_sensor_8(0x0205, 0x00); //Analog Gain,  0xE0=x1, 0xC3=x1.25, 0x00=x8
	HI543MIPI_write_cmos_sensor_8(0x0200, 0x03); //fine_integration_time[Hi]							  
	HI543MIPI_write_cmos_sensor_8(0x0201, 0x02); //fine_integration_time[Lo]							  
	HI543MIPI_write_cmos_sensor_8(0x0202, 0x07); //coarse_integration_time[Hi]							  
	HI543MIPI_write_cmos_sensor_8(0x0203, 0xAD); //coarse_integration_time[Lo]							  
#endif

	//---< Pedestal value '0' >-------//	 
	HI543MIPI_write_cmos_sensor_8(0x0008, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0009, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xE00D, 0x08); //PD Flushing for sub-sampling bit[3]='0' off, '1' enable.
	
	//---< Noise Redutcion & DPC set >-------//
	HI543MIPI_write_cmos_sensor_8(0x4042, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4043, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4044, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4013, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0x4014, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0x4015, 0x06); 
	HI543MIPI_write_cmos_sensor_8(0x4011, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x4010, 0x13); 
	HI543MIPI_write_cmos_sensor_8(0x7040, 0x45); 
	HI543MIPI_write_cmos_sensor_8(0x7047, 0x11); 

	HI543MIPI_write_cmos_sensor_8(0x5217, 0x0e); //                                                                        
	HI543MIPI_write_cmos_sensor_8(0x5218, 0x06); //                                                                        
	
	//                                                         
	HI543MIPI_write_cmos_sensor_8(0x0340, 0x07); // frame_length_lines_h  (= 1947 )
	HI543MIPI_write_cmos_sensor_8(0x0341, 0x9B); // frame_length_lines_l   
	HI543MIPI_write_cmos_sensor_8(0x0342, 0x0B); // Line_length_pck_h	  (= 2844 )
	HI543MIPI_write_cmos_sensor_8(0x0343, 0x1c); // Line_length_pck_l
	//----< view Size : >-------------//
	HI543MIPI_write_cmos_sensor_8(0x0344, 0x00); // x_addre_start_h  (width = 26 )
	HI543MIPI_write_cmos_sensor_8(0x0345, 0x1A); // x_addre_start_l   
	HI543MIPI_write_cmos_sensor_8(0x0346, 0x00); // y_addre_start_h  (height = 30)
	HI543MIPI_write_cmos_sensor_8(0x0347, 0x1E); // y_addre_start_l
	HI543MIPI_write_cmos_sensor_8(0x0348, 0x0A); // x_addre_end_h  (width =2589)
	HI543MIPI_write_cmos_sensor_8(0x0349, 0x1D); // x_addre_end_l	
	HI543MIPI_write_cmos_sensor_8(0x034a, 0x07); // y_addre_end_h  (height =1953 )
	HI543MIPI_write_cmos_sensor_8(0x034b, 0xA1); // y_addre_end_l
	HI543MIPI_write_cmos_sensor_8(0x034C, 0x0A); // x_output_size_h  (width = 2560)
	HI543MIPI_write_cmos_sensor_8(0x034D, 0x00); // x_output_size_l   
	HI543MIPI_write_cmos_sensor_8(0x034E, 0x07); // y_output_size_h  (height = 1920)
	HI543MIPI_write_cmos_sensor_8(0x034F, 0x80); // y_output_size_l
	HI543MIPI_write_cmos_sensor_8(0x0380, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0381, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0x0382, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0383, 0x01); // x_odd_inc_l sub_1/2(0x03), sub_1/4(0x07), sub_1/8(0x0f)
	HI543MIPI_write_cmos_sensor_8(0x0384, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0385, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0x0386, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0387, 0x01); // y_odd_inc_l sub_1/2(0x03), sub_1/4(0x07), sub_1/8(0x0f)
	HI543MIPI_write_cmos_sensor_8(0x5000, 0x09);  //ADPC[0], LSC[1], DGA[2], LENSD(=BayerNR)[3], windowing[4]
	HI543MIPI_write_cmos_sensor_8(0x2300, 0x00);  //LSC bit[0] 1=LSC on, 0=LSC off
	
	//---< Parallel / MIPI selection >-----//
	//MIPI									 
	HI543MIPI_write_cmos_sensor_8(0xf010, 0x3f);	
	HI543MIPI_write_cmos_sensor_8(0x6011, 0x0c);	
	HI543MIPI_write_cmos_sensor_8(0x6010, 0x02);	
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x01); 
	
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x01); //                                                                        
	mDELAY(10);
   	HI543MIPI_write_cmos_sensor_8(0x5201, 0x30); //                                                                        

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_Auto_Flicker_mode = KAL_FALSE;
	spin_unlock(&hi543mipi_drv_lock);
	
	mDELAY(5);
	
    ini_frm_length = 0xffff; //                                                                                                                                                 

}   /*  HI543MIPI_Sensor_Init  */

kal_uint16 HI543MIPI_PowerOn(void)
{
	kal_uint16 sensor_id = 0xffff;

	sensor_id = (HI543MIPI_read_cmos_sensor_8(0x0000)<<8)|(HI543MIPI_read_cmos_sensor_8(0x0001));

	return sensor_id;
}
/*************************************************************************
* FUNCTION
*   HI543MIPIOpen
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

UINT32 HI543MIPIOpen(void)
{
	kal_uint16 sensor_id = 0,temp_data = 0;
	
	sensor_id = HI543MIPI_PowerOn() ;
	
	SENSORDB("HI543MIPIOpen sensor_id is %x \n", sensor_id);
    if (sensor_id != HI543_MIPI_RAW_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

    HI543MIPI_Init_setting();

    temp_data= read_HI543MIPI_gain();

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_sensor_gain_base = temp_data;
    g_iHI543MIPI_Mode = HI543MIPI_MODE_PREVIEW;
	spin_unlock(&hi543mipi_drv_lock);
	
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   HI543MIPIGetSensorID
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
UINT32 HI543MIPIGetSensorID(UINT32 *sensorID) 
{
		*sensorID  = HI543MIPI_PowerOn() ;
		
		SENSORDB("HI543MIPIOpen sensor_id is %x\n", *sensorID );
		
 
    if (*sensorID != HI543_MIPI_RAW_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

//                                                                   
void HI543MIPI_Write_Shutter(kal_uint16 iShutter)
{
	kal_uint16 realtime_fp = 0;
	unsigned short temp_frame_length, pre_c_int_time, new_c_int_time; 
    
    if (iShutter < 4 )
		iShutter = 4;

	temp_frame_length = (HI543MIPI_read_cmos_sensor_8(0x0340)<<8)|(HI543MIPI_read_cmos_sensor_8(0x0341)); //In case of 1Byte read

	if(ini_frm_length > temp_frame_length)             // default frame length store
		ini_frm_length = temp_frame_length;
	
	pre_c_int_time = (unsigned short) (HI543MIPI_read_cmos_sensor_8(0x0202)<<8)|(HI543MIPI_read_cmos_sensor_8(0x0203)); //In case of 1Byte read, read shutter time
	new_c_int_time = iShutter;                                                                      // new shutter time update from the AE function
	// frame length change when the difference between previous shutter time and new shutter time is even number

	if (new_c_int_time < (ini_frm_length - 6) ) 			
		HI543MIPI_sensor.frame_length = ini_frm_length + ((0x0001&ini_frm_length) ? (0x0001 & ~((pre_c_int_time ^ new_c_int_time))) : (0x0001 & ((pre_c_int_time ^ new_c_int_time))) );			
	else			
		HI543MIPI_sensor.frame_length = (new_c_int_time+8) + ((0x0001&(new_c_int_time+8)) ? (0x0001 & ~((pre_c_int_time ^ new_c_int_time))) : (0x0001 & ((pre_c_int_time ^ new_c_int_time))) ); 
    
    //add for ZSD 15 fps flicker
    if((HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD) || (HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)) 
	{
    //		HI543MIPI_sensor.frame_length = HI543MIPI_FULL_PERIOD_LINE_NUMS  + HI543MIPI_dummy_lines;
		HI543MIPI_sensor.line_length = HI543MIPI_FULL_PERIOD_PIXEL_NUMS  + HI543MIPI_dummy_pixels;
		
		realtime_fp = HI543MIPI_sensor.preview_vt_clk *10 / HI543MIPI_sensor.line_length /iShutter;

	    if((realtime_fp >= 147)&&(realtime_fp <= 153))
	    {
           realtime_fp =146; 
		   HI543MIPI_sensor.frame_length = HI543MIPI_sensor.preview_vt_clk *10 / HI543MIPI_sensor.line_length / realtime_fp;
		   SENSORDB("[autofliker realtime_fp=15,extern heights slowdown to 14.6fps][height:%d]",HI543MIPI_sensor.frame_length);

		   if(HI543MIPI_sensor.frame_length < HI543MIPI_FULL_PERIOD_LINE_NUMS + HI543MIPI_dummy_lines)
		   	HI543MIPI_sensor.frame_length = HI543MIPI_FULL_PERIOD_LINE_NUMS  + HI543MIPI_dummy_lines;
		}
    }

	HI543MIPI_write_cmos_sensor_8(0x0340, (HI543MIPI_sensor.frame_length>>8)&0xFF); // frame_length_lines_h
	HI543MIPI_write_cmos_sensor_8(0x0341, HI543MIPI_sensor.frame_length&0xFF); // frame_length_lines_l 

	HI543MIPI_write_cmos_sensor_8(0x0202,(iShutter >> 8) & 0xFF);
	HI543MIPI_write_cmos_sensor_8(0x0203,iShutter);
}
//                                                  
/*************************************************************************
* FUNCTION
*   HI543MIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HI543MIPI to change exposure time.
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
void HI543MIPI_SetShutter(kal_uint16 iShutter)
{
	unsigned long flags;

	 if (iShutter < 4 )
		 iShutter = 4;

	 SENSORDB("HI543MIPI_SetShutter: iShutter =%d \n ",iShutter);


//	 if(HI543MIPI_sensor.shutter == iShutter)
//		 return;
 
	spin_lock_irqsave(&hi543mipi_drv_lock,flags);
	HI543MIPI_sensor.shutter= iShutter;
	spin_unlock_irqrestore(&hi543mipi_drv_lock,flags);
	HI543MIPI_Write_Shutter(iShutter);
	
	return;
}   /*  HI543MIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   HI543MIPI_read_shutter
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
UINT16 HI543MIPI_read_shutter(void)
{
    kal_uint16 ishutter;
	ishutter = (HI543MIPI_read_cmos_sensor_8(0x0202)<<8)|(HI543MIPI_read_cmos_sensor_8(0x0203)); /* course_integration_time */
	return ishutter;
}




/*************************************************************************
* FUNCTION
*   HI543MIPI_night_mode
*
* DESCRIPTION
*   This function night mode of HI543MIPI.
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
void HI543MIPI_NightMode(kal_bool bEnable)
{
    // frame rate will be control by AE table 
     
}/*	HI543MIPI_NightMode */



/*************************************************************************
* FUNCTION
*   HI543MIPIClose
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
UINT32 HI543MIPIClose(void)
{
    return ERROR_NONE;
}	/* HI543MIPIClose() */

void HI543MIPI_Set_Mirror_Flip(kal_uint8 image_mirror)
{
#if 0 //                                                                         
	switch (image_mirror)
	{
		case IMAGE_NORMAL:         
			HI543MIPI_write_cmos_sensor_8(0x0101,0x00);
		break;
		case IMAGE_H_MIRROR:      
			HI543MIPI_write_cmos_sensor_8(0x0101,0x01);
		break;
		case IMAGE_V_MIRROR:     
			HI543MIPI_write_cmos_sensor_8(0x0101,0x02);
		break;
		case IMAGE_HV_MIRROR:     
			HI543MIPI_write_cmos_sensor_8(0x0101,0x03);
		break;
	}
#endif
}


static void HI543MIPI_preview_setting(void)
{
    //V0.08
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); //                                                                        
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
    HI543MIPI_write_cmos_sensor_8(0x0103, 0x01); 
    HI543MIPI_write_cmos_sensor_8(0x0103, 0x00); 

    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xE048, 0x28);
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00);
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00);
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00);
	HI543MIPI_write_cmos_sensor_8(0x0106, 0x01);
	mDELAY(5);	
	HI543MIPI_write_cmos_sensor_8(0x0101, 0x03); //                                                                        

	//---< PLL & Divider >---------------------------//
	////PLL Output clk control////
	//Mclk/Pclk = 24Mhz/130Mhz(=Each Lane = 650Mhz )
	HI543MIPI_write_cmos_sensor_8(0xf003, 0x3a);
	HI543MIPI_write_cmos_sensor_8(0x0304, 0x00);
	HI543MIPI_write_cmos_sensor_8(0x0305, 0x18);
	HI543MIPI_write_cmos_sensor_8(0x0306, 0x02);
	HI543MIPI_write_cmos_sensor_8(0x0307, 0x8A);

	//---< Analog configuration >---------------------------//
	////Display Control////
	HI543MIPI_write_cmos_sensor_8(0x5400, 0x04);
	HI543MIPI_write_cmos_sensor_8(0x0b04, 0x01);

	//Static Signal Setting Start --> Must set////
	HI543MIPI_write_cmos_sensor_8(0xe039, 0x0F);
	HI543MIPI_write_cmos_sensor_8(0xe04e, 0x22);
	HI543MIPI_write_cmos_sensor_8(0xe051, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe053, 0xFF);
	HI543MIPI_write_cmos_sensor_8(0xe041, 0x2B);
	HI543MIPI_write_cmos_sensor_8(0xe042, 0xF9);
	HI543MIPI_write_cmos_sensor_8(0xe043, 0x7F);
	HI543MIPI_write_cmos_sensor_8(0xe044, 0xFF);

	HI543MIPI_write_cmos_sensor_8(0xe001,0x4a); //FOR SHUTTER//GAIN delay setting 

	HI543MIPI_write_cmos_sensor_8(0xe0a1, 0x03); //
	HI543MIPI_write_cmos_sensor_8(0xe0a2, 0x84); //                                                                      
	HI543MIPI_write_cmos_sensor_8(0xe0ae, 0x08); //

	HI543MIPI_write_cmos_sensor_8(0xe054, 0x55);// Blacksun Level
	HI543MIPI_write_cmos_sensor_8(0xe04b, 0x03);// PX Bias Manual Control

	// Timing setting : start ////
	HI543MIPI_write_cmos_sensor_8(0xe045, 0x7F);
	HI543MIPI_write_cmos_sensor_8(0xe047, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe196, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe197, 0xD0);
	HI543MIPI_write_cmos_sensor_8(0xe198, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe199, 0xDF);
	HI543MIPI_write_cmos_sensor_8(0xe19a, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe19b, 0xD0);
	HI543MIPI_write_cmos_sensor_8(0xe19c, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe19d, 0xD1);
	HI543MIPI_write_cmos_sensor_8(0xe186, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe187, 0x40);
	HI543MIPI_write_cmos_sensor_8(0xe188, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xe189, 0xEA);
	HI543MIPI_write_cmos_sensor_8(0xe162, 0x20);
	HI543MIPI_write_cmos_sensor_8(0xe163, 0x2F);
	HI543MIPI_write_cmos_sensor_8(0xe14d, 0x7C);
	HI543MIPI_write_cmos_sensor_8(0xe17a, 0x08);
	HI543MIPI_write_cmos_sensor_8(0xe17d, 0x48);
	HI543MIPI_write_cmos_sensor_8(0xe16e, 0x01);
	HI543MIPI_write_cmos_sensor_8(0xe16f, 0xC2);
	HI543MIPI_write_cmos_sensor_8(0xe170, 0x02);
	HI543MIPI_write_cmos_sensor_8(0xe171, 0x0C);
	HI543MIPI_write_cmos_sensor_8(0xe132, 0x01);
	HI543MIPI_write_cmos_sensor_8(0xe133, 0xC2);
	HI543MIPI_write_cmos_sensor_8(0xe138, 0x01);
	HI543MIPI_write_cmos_sensor_8(0xe139, 0xC2);
	HI543MIPI_write_cmos_sensor_8(0xe104, 0x01);
	HI543MIPI_write_cmos_sensor_8(0xe105, 0xC2);
	HI543MIPI_write_cmos_sensor_8(0xe10c, 0x01);
	HI543MIPI_write_cmos_sensor_8(0xe10d, 0xBC);

	//Dynamic Signal Setting Start --> Must set////
	HI543MIPI_write_cmos_sensor_8(0xe18a, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe18b, 0x90); 
	HI543MIPI_write_cmos_sensor_8(0xe18c, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0xe18d, 0x6c); 
	HI543MIPI_write_cmos_sensor_8(0xe18e, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe18f, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe190, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe191, 0x50); 

	//NCP On Start////
	HI543MIPI_write_cmos_sensor_8(0xe05d, 0xff);
	HI543MIPI_write_cmos_sensor_8(0xe05a, 0xbb);
	HI543MIPI_write_cmos_sensor_8(0xe05b, 0xcc);
	HI543MIPI_write_cmos_sensor_8(0xe0a0, 0x00); //                                                     
	HI543MIPI_write_cmos_sensor_8(0xe0b4, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b5, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b6, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b7, 0x09);
	
#if 0
	//Analog Gain control //
	HI543MIPI_write_cmos_sensor_8(0x0205, 0x00); //Analog Gain,  0xE0=x1, 0xC3=x1.25, 0x00=x8

	//---< Integration Time >---------------------------//
	HI543MIPI_write_cmos_sensor_8(0x0200, 0x02); //fine_integration_time[Hi]                 
	HI543MIPI_write_cmos_sensor_8(0x0201, 0x71); //fine_integration_time[Lo]                 
	HI543MIPI_write_cmos_sensor_8(0x0202, 0x05); //coarse_integration_time[Hi]               
	HI543MIPI_write_cmos_sensor_8(0x0203, 0xF0); //coarse_integration_time[Lo]               
#endif

	//---< Pedestal value '0' >-------//	 
	HI543MIPI_write_cmos_sensor_8(0x0008, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0009, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xE00D, 0x88); //                                                                                                                                        
		
	//---< Noise Redutcion & DPC set >-------//
	HI543MIPI_write_cmos_sensor_8(0x4042, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4043, 0x07);
	HI543MIPI_write_cmos_sensor_8(0x4044, 0x07);
	HI543MIPI_write_cmos_sensor_8(0x4013, 0x03);
	HI543MIPI_write_cmos_sensor_8(0x4014, 0x03);
	HI543MIPI_write_cmos_sensor_8(0x4015, 0x06);
	HI543MIPI_write_cmos_sensor_8(0x4011, 0x00);
	HI543MIPI_write_cmos_sensor_8(0x4010, 0x13);
	HI543MIPI_write_cmos_sensor_8(0x7040, 0x45);
	HI543MIPI_write_cmos_sensor_8(0x7047, 0x11);
	
	HI543MIPI_write_cmos_sensor_8(0x5217, 0x0e); //                                                                        
	HI543MIPI_write_cmos_sensor_8(0x5218, 0x06); //                                                                        
	//----< frame / line lenght >-------------//
	HI543MIPI_write_cmos_sensor_8(0x0340, 0x05); // frame_length_lines_h  (= 1519 )
	HI543MIPI_write_cmos_sensor_8(0x0341, 0xEF); // frame_length_lines_l   
	HI543MIPI_write_cmos_sensor_8(0x0342, 0x0B); // Line_length_pck_h     (= 2844 )
	HI543MIPI_write_cmos_sensor_8(0x0343, 0x1c); // Line_length_pck_l
		
	//----< view Size :  >-------------//
	HI543MIPI_write_cmos_sensor_8(0x0344, 0x00); // x_addre_start_h  (width = 24 )
	HI543MIPI_write_cmos_sensor_8(0x0345, 0x18); // x_addre_start_l   
	HI543MIPI_write_cmos_sensor_8(0x0346, 0x00); // y_addre_start_h  (height = 28 )
	HI543MIPI_write_cmos_sensor_8(0x0347, 0x1C); // y_addre_start_l
	HI543MIPI_write_cmos_sensor_8(0x0348, 0x0A); // x_addre_end_h  (width = 2591 )
	HI543MIPI_write_cmos_sensor_8(0x0349, 0x1F); // x_addre_end_l   
	HI543MIPI_write_cmos_sensor_8(0x034a, 0x07); // y_addre_end_h  (height = 1955 )
	HI543MIPI_write_cmos_sensor_8(0x034b, 0xa3); // y_addre_end_l
	HI543MIPI_write_cmos_sensor_8(0x034C, 0x05); // x_output_size_h  (width = 1280)
	HI543MIPI_write_cmos_sensor_8(0x034D, 0x00); // x_output_size_l   
	HI543MIPI_write_cmos_sensor_8(0x034E, 0x03); // y_output_size_h  (height = 960)
	HI543MIPI_write_cmos_sensor_8(0x034F, 0xc0); // y_output_size_l

	//----< subsampling mode set : sub 1/2 >-------------------//
	HI543MIPI_write_cmos_sensor_8(0x0380, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0381, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0x0382, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0383, 0x03); // x_odd_inc_l sub_1/2(0x03), sub_1/4(0x07), sub_1/8(0x0f)
	HI543MIPI_write_cmos_sensor_8(0x0384, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0385, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0x0386, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0387, 0x03); // y_odd_inc_l sub_1/2(0x03), sub_1/4(0x07), sub_1/8(0x0f)
	HI543MIPI_write_cmos_sensor_8(0x5000, 0x09); //ADPC[0], LSC[1], DGA[2], LENSD(=BayerNR)[3], windowing[4]
	HI543MIPI_write_cmos_sensor_8(0x2300, 0x00); //LSC bit[0] 1=LSC on, 0=LSC off

	//---< Parallel / MIPI selection >-----//
	//MIPI									 
	HI543MIPI_write_cmos_sensor_8(0xf010, 0x3f);
	HI543MIPI_write_cmos_sensor_8(0x6011, 0x0c);
	HI543MIPI_write_cmos_sensor_8(0x6010, 0x02);
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x01);	

	HI543MIPI_write_cmos_sensor_8(0x0100, 0x01); //                                                                        
	mDELAY(10);
   	HI543MIPI_write_cmos_sensor_8(0x5201, 0x30); //                                                                        

    spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_Auto_Flicker_mode = KAL_FALSE;
	spin_unlock(&hi543mipi_drv_lock);

	mDELAY(5);

    ini_frm_length = 0xffff;  //                                                                                                                                                

}


static void HI543MIPI_capture_setting(void)
{

//                                                                                   
    kal_uint16 gain;
    
//    shutter_msb = HI543MIPI_read_cmos_sensor(0x0202);
//    shutter_lsb = HI543MIPI_read_cmos_sensor(0x0203);
    gain = HI543MIPI_read_cmos_sensor(0x0205);
//                                                   

	//full size setting ,v0.08<=>20121106
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); //                                                                        
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
    HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
    HI543MIPI_write_cmos_sensor_8(0x0103, 0x01); 
    HI543MIPI_write_cmos_sensor_8(0x0103, 0x00); 

	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00);
	HI543MIPI_write_cmos_sensor_8(0xE048, 0x28); 
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0106, 0x01); 
	mDELAY(5);	
	HI543MIPI_write_cmos_sensor_8(0x0101, 0x03); //                                                                        
	
	//---< PLL & Divider >---------------------------//
	////PLL Output clk control////
	//Mclk/Pclk = 24Mhz/130Mhz(=Each Lane = 650Mhz )
	HI543MIPI_write_cmos_sensor_8(0xf003, 0x3a); 
	HI543MIPI_write_cmos_sensor_8(0x0304, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0305, 0x18); 
	HI543MIPI_write_cmos_sensor_8(0x0306, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0x0307, 0x8A); 
	
	//---< Analog configuration >---------------------------//
	////Display Control////
	HI543MIPI_write_cmos_sensor_8(0x5400, 0x04); 
	HI543MIPI_write_cmos_sensor_8(0x0b04, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe039, 0x0F); 
	HI543MIPI_write_cmos_sensor_8(0xe04e, 0x22); 
	HI543MIPI_write_cmos_sensor_8(0xe051, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe053, 0xFF); 
	HI543MIPI_write_cmos_sensor_8(0xe041, 0x2B); 
	HI543MIPI_write_cmos_sensor_8(0xe042, 0xF9); 
	HI543MIPI_write_cmos_sensor_8(0xe043, 0x7F); 
	HI543MIPI_write_cmos_sensor_8(0xe044, 0xFF); 

	HI543MIPI_write_cmos_sensor_8(0xe001,0x4a); //FOR SHUTTER//GAIN delay setting;must set
	
	HI543MIPI_write_cmos_sensor_8(0xe0a1, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0xe0a2, 0x84); //                                                     
	HI543MIPI_write_cmos_sensor_8(0xe0ae, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe054, 0x55); 
	HI543MIPI_write_cmos_sensor_8(0xe04b, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0xe045, 0x7F); 
	HI543MIPI_write_cmos_sensor_8(0xe047, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe196, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe197, 0xD0); 
	HI543MIPI_write_cmos_sensor_8(0xe198, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe199, 0xDF); 
	HI543MIPI_write_cmos_sensor_8(0xe19a, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe19b, 0xD0); 
	HI543MIPI_write_cmos_sensor_8(0xe19c, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe19d, 0xD1); 
	HI543MIPI_write_cmos_sensor_8(0xe186, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe187, 0x40); 
	HI543MIPI_write_cmos_sensor_8(0xe188, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe189, 0xEA); 
	HI543MIPI_write_cmos_sensor_8(0xe162, 0x20); 
	HI543MIPI_write_cmos_sensor_8(0xe163, 0x2F); 
	HI543MIPI_write_cmos_sensor_8(0xe14d, 0x7C); 
	HI543MIPI_write_cmos_sensor_8(0xe17a, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe17d, 0x48); 
	HI543MIPI_write_cmos_sensor_8(0xe16e, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe16f, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe170, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0xe171, 0x0C); 
	HI543MIPI_write_cmos_sensor_8(0xe132, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe133, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe138, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe139, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe104, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe105, 0xC2); 
	HI543MIPI_write_cmos_sensor_8(0xe10c, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe10d, 0xBC); 
	
	////Dynamic Signal Setting Start --> Must set////
	HI543MIPI_write_cmos_sensor_8(0xe18a, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe18b, 0x90); 
	HI543MIPI_write_cmos_sensor_8(0xe18c, 0x02); 
	HI543MIPI_write_cmos_sensor_8(0xe18d, 0x6c); 
	HI543MIPI_write_cmos_sensor_8(0xe18e, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xe18f, 0x08); 
	HI543MIPI_write_cmos_sensor_8(0xe190, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0xe191, 0x50); 
	HI543MIPI_write_cmos_sensor_8(0xe05d, 0xff); 
	HI543MIPI_write_cmos_sensor_8(0xe05a, 0xbb); 
	HI543MIPI_write_cmos_sensor_8(0xe05b, 0xcc); 
	HI543MIPI_write_cmos_sensor_8(0xe0a0, 0x00); //                                                     
	HI543MIPI_write_cmos_sensor_8(0xe0b4, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b5, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b6, 0x09);
	HI543MIPI_write_cmos_sensor_8(0xe0b7, 0x09);

#if 0
	//Analog Gain control //
	HI543MIPI_write_cmos_sensor_8(0x0205, 0x00); //Analog Gain,  0xE0=x1, 0xC3=x1.25, 0x00=x8
	HI543MIPI_write_cmos_sensor_8(0x0200, 0x03); //fine_integration_time[Hi]							  
	HI543MIPI_write_cmos_sensor_8(0x0201, 0x02); //fine_integration_time[Lo]							  
	HI543MIPI_write_cmos_sensor_8(0x0202, 0x07); //coarse_integration_time[Hi]							  
	HI543MIPI_write_cmos_sensor_8(0x0203, 0xAD); //coarse_integration_time[Lo]							  
#endif
//                                                                                   
	//Analog Gain control //
	HI543MIPI_write_cmos_sensor_8(0x0205, (kal_uint32) gain); //Analog Gain,  0xE0=x1, 0xC3=x1.25, 0x00=x8
//	HI543MIPI_write_cmos_sensor_8(0x0202, (kal_uint32) shutter_msb); //coarse_integration_time[Hi]							  
//	HI543MIPI_write_cmos_sensor_8(0x0203, (kal_uint32) shutter_lsb); //coarse_integration_time[Lo]							  
//                                                  
	//---< Pedestal value '0' >-------//	 
	HI543MIPI_write_cmos_sensor_8(0x0008, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0009, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0xE00D, 0x08); //PD Flushing for sub-sampling bit[3]='0' off, '1' enable.
	
	//---< Noise Redutcion & DPC set >-------//
	HI543MIPI_write_cmos_sensor_8(0x4042, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4043, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4044, 0x07); 
	HI543MIPI_write_cmos_sensor_8(0x4013, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0x4014, 0x03); 
	HI543MIPI_write_cmos_sensor_8(0x4015, 0x06); 
	HI543MIPI_write_cmos_sensor_8(0x4011, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x4010, 0x13); 
	HI543MIPI_write_cmos_sensor_8(0x7040, 0x45); 
	HI543MIPI_write_cmos_sensor_8(0x7047, 0x11); 
	
	HI543MIPI_write_cmos_sensor_8(0x5217, 0x0e); //                                                                        
	HI543MIPI_write_cmos_sensor_8(0x5218, 0x06); //                                                                        

	//                                                         
	HI543MIPI_write_cmos_sensor_8(0x0340, 0x07); // frame_length_lines_h  (= 1947 )
	HI543MIPI_write_cmos_sensor_8(0x0341, 0x9B); // frame_length_lines_l   
	HI543MIPI_write_cmos_sensor_8(0x0342, 0x0B); // Line_length_pck_h	  (= 2844 )
	HI543MIPI_write_cmos_sensor_8(0x0343, 0x1c); // Line_length_pck_l
	//----< view Size : >-------------//
	HI543MIPI_write_cmos_sensor_8(0x0344, 0x00); // x_addre_start_h  (width = 26 )
	HI543MIPI_write_cmos_sensor_8(0x0345, 0x1A); // x_addre_start_l   
	HI543MIPI_write_cmos_sensor_8(0x0346, 0x00); // y_addre_start_h  (height = 30)
	HI543MIPI_write_cmos_sensor_8(0x0347, 0x1E); // y_addre_start_l
	HI543MIPI_write_cmos_sensor_8(0x0348, 0x0A); // x_addre_end_h  (width =2589)
	HI543MIPI_write_cmos_sensor_8(0x0349, 0x1D); // x_addre_end_l	
	HI543MIPI_write_cmos_sensor_8(0x034a, 0x07); // y_addre_end_h  (height =1953 )
	HI543MIPI_write_cmos_sensor_8(0x034b, 0xA1); // y_addre_end_l
	HI543MIPI_write_cmos_sensor_8(0x034C, 0x0A); // x_output_size_h  (width = 2560)
	HI543MIPI_write_cmos_sensor_8(0x034D, 0x00); // x_output_size_l   
	HI543MIPI_write_cmos_sensor_8(0x034E, 0x07); // y_output_size_h  (height = 1920)
	HI543MIPI_write_cmos_sensor_8(0x034F, 0x80); // y_output_size_l
	HI543MIPI_write_cmos_sensor_8(0x0380, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0381, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0x0382, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0383, 0x01); // x_odd_inc_l sub_1/2(0x03), sub_1/4(0x07), sub_1/8(0x0f)
	HI543MIPI_write_cmos_sensor_8(0x0384, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0385, 0x01); 
	HI543MIPI_write_cmos_sensor_8(0x0386, 0x00); 
	HI543MIPI_write_cmos_sensor_8(0x0387, 0x01); // y_odd_inc_l sub_1/2(0x03), sub_1/4(0x07), sub_1/8(0x0f)
	HI543MIPI_write_cmos_sensor_8(0x5000, 0x09);  //ADPC[0], LSC[1], DGA[2], LENSD(=BayerNR)[3], windowing[4]
	HI543MIPI_write_cmos_sensor_8(0x2300, 0x00);  //LSC bit[0] 1=LSC on, 0=LSC off
	
	//---< Parallel / MIPI selection >-----//
	//MIPI									 
	HI543MIPI_write_cmos_sensor_8(0xf010, 0x3f);	
	HI543MIPI_write_cmos_sensor_8(0x6011, 0x0c);	
	HI543MIPI_write_cmos_sensor_8(0x6010, 0x02);	
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x01); 
	
	HI543MIPI_write_cmos_sensor_8(0x0100, 0x01); //                                                                        
	mDELAY(10);
   	HI543MIPI_write_cmos_sensor_8(0x5201, 0x30); //                                                                        
	
    spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_Auto_Flicker_mode = KAL_FALSE;
	spin_unlock(&hi543mipi_drv_lock);
	
	mDELAY(5);
	
    ini_frm_length = 0xffff;  //                                                                                                                                                

}



/*************************************************************************
* FUNCTION
*   HI543MIPI_SetDummy
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   mode  ture : preview mode
*             false : capture mode
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void HI543MIPI_SetDummy(kal_bool mode,const kal_uint16 iDummyPixels, const kal_uint16 iDummyLines)
{
    kal_uint16 Line_length_pclk, Frame_length_lines;

    SENSORDB("Enter HI543MIPI_SetDummy \n");
	if(mode == KAL_TRUE) //preview
	{
		Line_length_pclk   = HI543MIPI_PV_PERIOD_PIXEL_NUMS + iDummyPixels;
		Frame_length_lines = HI543MIPI_PV_PERIOD_LINE_NUMS  + iDummyLines;

		spin_lock(&hi543mipi_drv_lock);
		HI543_Frame_Length_preview = Frame_length_lines;
		spin_unlock(&hi543mipi_drv_lock);
	}
	else   //capture
	{
		Line_length_pclk   = HI543MIPI_FULL_PERIOD_PIXEL_NUMS + iDummyPixels;
		Frame_length_lines = HI543MIPI_FULL_PERIOD_LINE_NUMS  + iDummyLines;

	}
    SENSORDB("Enter HI543MIPI_SetDummy Frame_length_lines=%d, Line_length_pclk=%d\n",Frame_length_lines,Line_length_pclk);

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_sensor.frame_length =Frame_length_lines;
	HI543MIPI_sensor.line_length = Line_length_pclk;
	spin_unlock(&hi543mipi_drv_lock);
	
	HI543MIPI_write_cmos_sensor_8(0x0340, (Frame_length_lines >> 8) & 0xFF);
    HI543MIPI_write_cmos_sensor_8(0x0341, Frame_length_lines & 0xFF);

    HI543MIPI_write_cmos_sensor_8(0x0342, (Line_length_pclk >> 8) & 0xFF);
    HI543MIPI_write_cmos_sensor_8(0x0343, Line_length_pclk & 0xFF);

}   /*  HI543MIPI_SetDummy */


UINT32 HI543SetMaxFrameRate(UINT16 u2FrameRate)
{

	kal_int16 dummy_line;
	kal_uint16 FrameHeight = HI543MIPI_PV_PERIOD_LINE_NUMS;
	unsigned long flags;
		
	printk("[soso][HI543SetMaxFrameRate]u2FrameRate=%d",u2FrameRate);

    if((HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD) || (HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)) {
	FrameHeight= (10 * HI543MIPI_sensor.capture_vt_clk) / u2FrameRate / HI543MIPI_FULL_PERIOD_PIXEL_NUMS;

	if(FrameHeight<HI543MIPI_FULL_PERIOD_LINE_NUMS)
	{
		FrameHeight=HI543MIPI_FULL_PERIOD_LINE_NUMS;
	}

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_sensor.frame_length = FrameHeight;
	spin_unlock(&hi543mipi_drv_lock);
	
	dummy_line = FrameHeight - HI543MIPI_FULL_PERIOD_LINE_NUMS;
	HI543MIPI_SetDummy(KAL_FALSE,0, dummy_line); /* modify dummy_pixel must gen AE table again */
    	
    } 
	else 
    {
	FrameHeight= (10 * HI543MIPI_sensor.preview_vt_clk) / u2FrameRate / HI543MIPI_PV_PERIOD_PIXEL_NUMS;

	if(FrameHeight<HI543MIPI_PV_PERIOD_LINE_NUMS)
	{
		FrameHeight=HI543MIPI_PV_PERIOD_LINE_NUMS;
	}

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_sensor.frame_length = FrameHeight;
	spin_unlock(&hi543mipi_drv_lock);
	
	dummy_line = FrameHeight - HI543MIPI_PV_PERIOD_LINE_NUMS;
	HI543MIPI_SetDummy(KAL_TRUE,0, dummy_line); /* modify dummy_pixel must gen AE table again */
    }
}


/*************************************************************************
* FUNCTION
*   HI543MIPIPreview
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
UINT32 HI543MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("Enter HI543MIPIPreview \n ");

	spin_lock(&hi543mipi_drv_lock);
    HI543MIPI_PV_dummy_pixels = 0;
	HI543MIPI_PV_dummy_lines  = 0;  
	spin_unlock(&hi543mipi_drv_lock);
	
	sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR; //                                                          

	 spin_lock(&hi543mipi_drv_lock);
	 if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        HI543MIPI_MPEG4_encode_mode = KAL_TRUE;
    }
    else
    {
        HI543MIPI_MPEG4_encode_mode = KAL_FALSE;
    }
	spin_unlock(&hi543mipi_drv_lock);

	
	HI543MIPI_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);
	HI543MIPI_preview_setting();
	
	HI543MIPI_SetDummy(KAL_TRUE,HI543MIPI_PV_dummy_pixels,HI543MIPI_PV_dummy_lines);

	spin_lock(&hi543mipi_drv_lock);
    HI543MIPI_sensor.frame_length = HI543MIPI_PV_PERIOD_LINE_NUMS  + HI543MIPI_PV_dummy_lines;
    HI543MIPI_sensor.line_length = HI543MIPI_PV_PERIOD_PIXEL_NUMS  + HI543MIPI_PV_dummy_pixels;
	spin_unlock(&hi543mipi_drv_lock);

    image_window->GrabStartX= HI543MIPI_PV_START_X;
    image_window->GrabStartY= HI543MIPI_PV_START_Y;
    image_window->ExposureWindowWidth= HI543MIPI_IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight= HI543MIPI_IMAGE_SENSOR_PV_HEIGHT;

	spin_lock(&hi543mipi_drv_lock);
    g_iHI543MIPI_Mode = HI543MIPI_MODE_PREVIEW;
	spin_unlock(&hi543mipi_drv_lock);
	
    //HI543MIPI_SetShutter(HI543MIPI_exposure_lines);
    
	memcpy(&HI543MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}	/* HI543MIPIPreview() */



/*******************************************************************************
*
********************************************************************************/
UINT32 HI543MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 shutter=HI543MIPI_sensor.shutter;

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_Auto_Flicker_mode = KAL_FALSE;
	HI543MIPI_MPEG4_encode_mode = KAL_FALSE;
	spin_unlock(&hi543mipi_drv_lock);

	sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR; //                                                          
	if ((image_window->ImageTargetWidth<= HI543MIPI_IMAGE_SENSOR_PV_WIDTH-1) &&
		  (image_window->ImageTargetHeight<= HI543MIPI_IMAGE_SENSOR_PV_HEIGHT-1))
	{	
		SENSORDB("Enter HI543MIPICapture small size  \n ");
		
		spin_lock(&hi543mipi_drv_lock);
		HI543MIPI_dummy_pixels = 0;
		HI543MIPI_dummy_lines  = 0;
		HI543MIPI_sensor.capture_vt_clk = HI543MIPI_sensor.preview_vt_clk;
		spin_unlock(&hi543mipi_drv_lock);
		
		HI543MIPI_SetDummy(KAL_FALSE,HI543MIPI_dummy_pixels,HI543MIPI_dummy_lines);

		spin_lock(&hi543mipi_drv_lock);
		HI543MIPI_sensor.frame_length = HI543MIPI_FULL_PERIOD_LINE_NUMS  + HI543MIPI_dummy_lines;
		HI543MIPI_sensor.line_length = HI543MIPI_FULL_PERIOD_PIXEL_NUMS  + HI543MIPI_dummy_pixels;
		spin_unlock(&hi543mipi_drv_lock);
		
		shutter = (HI543MIPI_PV_PERIOD_PIXEL_NUMS + HI543MIPI_PV_dummy_pixels)/(HI543MIPI_PV_PERIOD_PIXEL_NUMS +HI543MIPI_dummy_pixels)*shutter;
		shutter = (HI543MIPI_sensor.capture_vt_clk / HI543MIPI_sensor.preview_vt_clk)*shutter;
		
		image_window->GrabStartX           = HI543MIPI_PV_START_X;
        image_window->GrabStartY           = HI543MIPI_PV_START_Y;
        image_window->ExposureWindowWidth  = HI543MIPI_IMAGE_SENSOR_PV_WIDTH;
        image_window->ExposureWindowHeight = HI543MIPI_IMAGE_SENSOR_PV_WIDTH;
	}
	else  //
	{
		SENSORDB("Enter HI543MIPICapture 5M size  \n ");
		
	       if(g_iHI543MIPI_Mode == HI543MIPI_MODE_CAPTURE) {
			//   SENSORDB("Enter HI543MIPICapture 5M size--return  \n ");
	        //	return;
	       }
		   else
		   {
			   HI543MIPI_capture_setting();
		   }

		spin_lock(&hi543mipi_drv_lock);
	    g_iHI543MIPI_Mode = HI543MIPI_MODE_CAPTURE;
		HI543MIPI_dummy_pixels = 0;
		HI543MIPI_dummy_lines  = 0;
		spin_unlock(&hi543mipi_drv_lock);
		
		SENSORDB("Enter HI543MIPICapture full size  \n ");

		HI543MIPI_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);

		//HI543MIPI_capture_setting();
		
		HI543MIPI_SetDummy(KAL_FALSE,HI543MIPI_dummy_pixels,HI543MIPI_dummy_lines);

		spin_lock(&hi543mipi_drv_lock);
		HI543MIPI_sensor.frame_length = HI543MIPI_FULL_PERIOD_LINE_NUMS  + HI543MIPI_dummy_lines;
		HI543MIPI_sensor.line_length = HI543MIPI_FULL_PERIOD_PIXEL_NUMS  + HI543MIPI_dummy_pixels;
		spin_unlock(&hi543mipi_drv_lock);


		if(HI543_CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
		{
			printk("HI543_Capture exit ZSD!!\n");
			return;
		}

		//SENSORDB("preview shutter =%d \n",shutter);

		shutter = shutter * (HI543MIPI_PV_PERIOD_PIXEL_NUMS + HI543MIPI_PV_dummy_pixels)/(HI543MIPI_FULL_PERIOD_PIXEL_NUMS +HI543MIPI_dummy_pixels);
		//SENSORDB("Enter HI543MIPICapture full size shutter_1=%d \n ",shutter);

		shutter = shutter * ((HI543MIPI_sensor.capture_vt_clk/1000000) / (HI543MIPI_sensor.preview_vt_clk/1000000));
		//SENSORDB("capture  shutter =%d ,gain = %d\n",shutter,HI543MIPI_read_cmos_sensor_8(0x204));
		
		image_window->GrabStartX           = HI543MIPI_FULL_START_X;
        image_window->GrabStartY           = HI543MIPI_FULL_START_Y;
        image_window->ExposureWindowWidth  = HI543MIPI_IMAGE_SENSOR_FULL_WIDTH;
        image_window->ExposureWindowHeight = HI543MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	}
	
	if(shutter < 1)
	    shutter = 1;
	
    HI543MIPI_Write_Shutter(shutter);
	
	memcpy(&HI543MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

	return ERROR_NONE;
}	/* HI543MIPICapture() */

UINT32 HI543MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth     =  HI543MIPI_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight    =  HI543MIPI_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth  =  HI543MIPI_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight =  HI543MIPI_IMAGE_SENSOR_PV_HEIGHT;

    return ERROR_NONE;
}   /* HI543MIPIGetResolution() */

UINT32 HI543MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	
	switch(ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=HI543MIPI_IMAGE_SENSOR_FULL_WIDTH;
			pSensorInfo->SensorPreviewResolutionY=HI543MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=24;
		break;

		default:
        	pSensorInfo->SensorPreviewResolutionX=HI543MIPI_IMAGE_SENSOR_PV_WIDTH;
       		pSensorInfo->SensorPreviewResolutionY=HI543MIPI_IMAGE_SENSOR_PV_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=24;
		break;
	}
    //pSensorInfo->SensorPreviewResolutionX =  HI543MIPI_IMAGE_SENSOR_PV_WIDTH;
    //pSensorInfo->SensorPreviewResolutionY =  HI543MIPI_IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX    =  HI543MIPI_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY    =  HI543MIPI_IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=24;
    pSensorInfo->SensorVideoFrameRate=29;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;

	pSensorConfigData->SensorImageMirror = IMAGE_HV_MIRROR; //                                                          
	switch(pSensorConfigData->SensorImageMirror)
	{
		case IMAGE_NORMAL:
			 pSensorInfo->SensorOutputDataFormat	= SENSOR_OUTPUT_FORMAT_RAW_Gr; 
             break;
		case IMAGE_H_MIRROR:
			pSensorInfo->SensorOutputDataFormat	 = SENSOR_OUTPUT_FORMAT_RAW_R;
			break;
		case IMAGE_V_MIRROR:
			pSensorInfo->SensorOutputDataFormat	  = SENSOR_OUTPUT_FORMAT_RAW_B;
			break;
		case IMAGE_HV_MIRROR:
			pSensorInfo->SensorOutputDataFormat    = SENSOR_OUTPUT_FORMAT_RAW_Gb;
			break;
	}

    pSensorInfo->SensorClockPolarity        = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity        = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity        = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines  = 1;
    
	#ifdef MIPI_INTERFACE
   		pSensorInfo->SensroInterfaceType        = SENSOR_INTERFACE_TYPE_MIPI;
   	#else
   		pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_PARALLEL;
   	#endif
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

    pSensorInfo->CaptureDelayFrame = 4; 
    pSensorInfo->PreviewDelayFrame = 4;//2; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;      
    pSensorInfo->AEShutDelayFrame = 0;	
    pSensorInfo->AESensorGainDelayFrame = 0;   //test with reg revise 
    pSensorInfo->AEISPGainDelayFrame = 2;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HI543MIPI_PV_START_X; 
            pSensorInfo->SensorGrabStartY = HI543MIPI_PV_START_Y;    
            
			#ifdef MIPI_INTERFACE
	            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
	        #endif
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HI543MIPI_FULL_START_X; 
            pSensorInfo->SensorGrabStartY = HI543MIPI_FULL_START_X;   
            
			#ifdef MIPI_INTERFACE
	            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
	            pSensorInfo->SensorPacketECCOrder = 1;
	        #endif
            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = HI543MIPI_PV_START_X; 
            pSensorInfo->SensorGrabStartY = HI543MIPI_PV_START_X;             
            break;
    }

   // HI543MIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &HI543MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* HI543MIPIGetInfo() */


UINT32 HI543MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	spin_lock(&hi543mipi_drv_lock);
	HI543_CurrentScenarioId = ScenarioId;
	spin_unlock(&hi543mipi_drv_lock);
	
    printk("HI543MIPIControl() ScenarioId=%d \n",ScenarioId);
	
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
          {
            HI543MIPIPreview(pImageWindow, pSensorConfigData);
          }
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HI543MIPICapture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
           
    }
    return TRUE;
} /* HI543MIPIControl() */


UINT32 HI543MIPISetVideoMode(UINT16 u2FrameRate)
{
	kal_uint16 MAX_Frame_length =0;
    //SENSORDB("[HI543MIPISetVideoMode] u2FrameRate(10base) = %d\n", u2FrameRate);

	spin_lock(&hi543mipi_drv_lock);
	HI543MIPI_MPEG4_encode_mode = KAL_TRUE;
	Hi543_FixedFps = u2FrameRate;
	spin_unlock(&hi543mipi_drv_lock);

	//if(u2FrameRate >30 || u2FrameRate <5)
	  //  SENSORDB("Error frame rate seting");

	if(u2FrameRate==0)
	{
		SENSORDB("Dynamic video frame_rate: FrameRate=%d\n",u2FrameRate);
	  HI543MIPI_MPEG4_encode_mode = KAL_FALSE;		
		return;
	}


	MAX_Frame_length = HI543MIPI_sensor.preview_vt_clk/(HI543MIPI_PV_PERIOD_PIXEL_NUMS+HI543MIPI_PV_dummy_pixels)/u2FrameRate;
	if(MAX_Frame_length < HI543MIPI_PV_PERIOD_LINE_NUMS )
		MAX_Frame_length = HI543MIPI_PV_PERIOD_LINE_NUMS;

	spin_lock(&hi543mipi_drv_lock);
    HI543MIPI_PV_dummy_lines = MAX_Frame_length - HI543MIPI_PV_PERIOD_LINE_NUMS ;  
	spin_unlock(&hi543mipi_drv_lock);
	
	HI543MIPI_SetDummy(KAL_TRUE,HI543MIPI_PV_dummy_pixels,HI543MIPI_PV_dummy_lines);
	
    return KAL_TRUE;
}


UINT32 HI543MIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{

    SENSORDB("[HI543MIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);

	if(bEnable) 
	{   // enable auto flicker   
		spin_lock(&hi543mipi_drv_lock);
        HI543MIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&hi543mipi_drv_lock);
		
		if((Hi543_FixedFps== 30)&&(HI543MIPI_MPEG4_encode_mode==KAL_TRUE))
		{
			HI543SetMaxFrameRate(296);
		}
		else if((HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD) || (HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)) 
		{
		    HI543SetMaxFrameRate(234);
	    }
		else
	    {
            HI543SetMaxFrameRate(296);	       
	    }
    } 
	else 
    {
		spin_lock(&hi543mipi_drv_lock);
        HI543MIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&hi543mipi_drv_lock);
		
		if((Hi543_FixedFps== 30)&&(HI543MIPI_MPEG4_encode_mode==KAL_TRUE))
		{
			HI543SetMaxFrameRate(300);
		}	
		else if((HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD) || (HI543_CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)) 
		{
		    HI543SetMaxFrameRate(234);
	    }
		else
	    {
            HI543SetMaxFrameRate(300);	       
	    }
    }
    return TRUE;
}


UINT32 HI543MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++=HI543MIPI_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16=HI543MIPI_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        	*pFeatureReturnPara16++= HI543MIPI_sensor.line_length;
        	*pFeatureReturnPara16=HI543MIPI_sensor.frame_length;
       		*pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ://for calculate shutter
			switch(HI543_CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	            	*pFeatureReturnPara32 = HI543MIPI_sensor.preview_vt_clk;
	           		*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = HI543MIPI_sensor.preview_vt_clk;
	           		*pFeatureParaLen=4;
					break;
			}
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            HI543MIPI_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HI543MIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            HI543_Set_gain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
           // HI543MIPI_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            HI543MIPI_write_cmos_sensor_8(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HI543MIPI_read_cmos_sensor_8(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
				spin_lock(&hi543mipi_drv_lock);
                HI543MIPISensorCCT[i].Addr=*pFeatureData32++;
                HI543MIPISensorCCT[i].Para=*pFeatureData32++;
			    spin_unlock(&hi543mipi_drv_lock); 
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI543MIPISensorCCT[i].Addr;
                *pFeatureData32++=HI543MIPISensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
				spin_lock(&hi543mipi_drv_lock);
                HI543MIPISensorReg[i].Addr=*pFeatureData32++;
                HI543MIPISensorReg[i].Para=*pFeatureData32++;
			    spin_unlock(&hi543mipi_drv_lock); 
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI543MIPISensorReg[i].Addr;
                *pFeatureData32++=HI543MIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HI543_MIPI_RAW_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, HI543MIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HI543MIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HI543MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            HI543MIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            HI543MIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=HI543MIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            HI543MIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            HI543MIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            HI543MIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 221;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_Gr;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            HI543MIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
			SENSORDB("feature control :SENSOR_FEATURE_CHECK_SENSOR_ID");
            HI543MIPIGetSensorID(pFeatureReturnPara32); 
            break; 
   		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            HI543MIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        default:
            break;
    }

    return ERROR_NONE;
}	/* HI543MIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncHI543MIPI=
{
    HI543MIPIOpen,
    HI543MIPIGetInfo,
    HI543MIPIGetResolution,
    HI543MIPIFeatureControl,
    HI543MIPIControl,
    HI543MIPIClose
};

UINT32 HI543MIPISensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI543MIPI;

    return ERROR_NONE;
}   /* SensorInit() */

