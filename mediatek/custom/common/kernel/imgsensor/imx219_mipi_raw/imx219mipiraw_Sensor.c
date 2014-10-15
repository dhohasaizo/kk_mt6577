/*******************************************************************************************/


/*******************************************************************************************/

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

#include "imx219mipiraw_Sensor.h"
#include "imx219mipiraw_Camera_Sensor_para.h"
#include "imx219mipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(IMX219mipiraw_drv_lock);


#define IMX219DB(fmt, arg...) printk( "[IMX219MipiRaw] "  fmt, ##arg)


kal_uint32 IMX219_FeatureControl_PERIOD_PixelNum=IMX219_PV_PERIOD_PIXEL_NUMS;
kal_uint32 IMX219_FeatureControl_PERIOD_LineNum=IMX219_PV_PERIOD_LINE_NUMS;

UINT16 IMX219_VIDEO_MODE_TARGET_FPS = 30;

#define IMX219MIPI_MaxGainIndex (97)
kal_uint16 IMX219MIPI_sensorGainMapping[IMX219MIPI_MaxGainIndex][2] ={
{ 64 ,0  },
{ 68 ,12 },
{ 71 ,23 },
{ 74 ,33 },
{ 77 ,42 },
{ 81 ,52 },
{ 84 ,59 },
{ 87 ,66 },
{ 90 ,73 },
{ 93 ,79 },
{ 96 ,85 },
{ 100,91 },
{ 103,96 },
{ 106,101},
{ 109,105},
{ 113,110},
{ 116,114},
{ 120,118},
{ 122,121},
{ 125,125},
{ 128,128},
{ 132,131},
{ 135,134},
{ 138,137},
{ 141,139},
{ 144,142},
{ 148,145},
{ 151,147},
{ 153,149},
{ 157,151},
{ 160,153},
{ 164,156},
{ 168,158},
{ 169,159},
{ 173,161},
{ 176,163},
{ 180,165},
{ 182,166},
{ 187,168},
{ 189,169},
{ 193,171},
{ 196,172},
{ 200,174},
{ 203,175},
{ 205,176},
{ 208,177},
{ 213,179},
{ 216,180},
{ 219,181},
{ 222,182},
{ 225,183},
{ 228,184},
{ 232,185},
{ 235,186},
{ 238,187},
{ 241,188},
{ 245,189},
{ 249,190},
{ 253,191},
{ 256,192},
{ 260,193},
{ 265,194},
{ 269,195},
{ 274,196},
{ 278,197},
{ 283,198},
{ 288,199},
{ 293,200},
{ 298,201},
{ 304,202},
{ 310,203},
{ 315,204},
{ 322,205},
{ 328,206},
{ 335,207},
{ 342,208},
{ 349,209},
{ 357,210},
{ 365,211},
{ 373,212},
{ 381,213},
{ 400,215},
{ 420,217},
{ 432,218},
{ 443,219},
{ 468,221},
{ 482,222},
{ 497,223},
{ 512,224},
{ 529,225},
{ 546,226},
{ 566,227},
{ 585,228},
{ 607,229},
{ 631,230},
{ 656,231},
{ 683,232}
};


MSDK_SENSOR_CONFIG_STRUCT IMX219SensorConfigData;

kal_uint32 IMX219_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM IMX219CurrentScenarioId = ACDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT IMX219SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT IMX219SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static IMX219_PARA_STRUCT IMX219;


extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define IMX219MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, IMX219MIPI_WRITE_ID)

kal_uint16 IMX219MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,IMX219MIPI_WRITE_ID);
    return get_byte;
}


kal_uint32 GetIMX219LineLength()
{
	kal_uint32 IMX219_line_length = 0;

	if ( SENSOR_MODE_PREVIEW == IMX219.sensorMode )
	{
		IMX219_line_length = IMX219_PV_PERIOD_PIXEL_NUMS + IMX219.DummyPixels;
	}
	else
	{
		IMX219_line_length = IMX219_FULL_PERIOD_PIXEL_NUMS + IMX219.DummyPixels;
	}

    return IMX219_line_length;
}

kal_uint32 GetIMX219FrameLength()
{
	kal_uint32 IMX219_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == IMX219.sensorMode )
	{
		IMX219_frame_length = IMX219_PV_PERIOD_LINE_NUMS + IMX219.DummyLines ;
	}
	else
	{
		IMX219_frame_length = IMX219_FULL_PERIOD_LINE_NUMS + IMX219.DummyLines ;
	}

	return IMX219_frame_length;
}

kal_uint32 IMX219_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	if (shutter_value <4 ){
		shutter_value = 4;
	}

	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

    return extra_lines;

}

kal_uint32 IMX219_CalcFrameLength_For_AutoFlicker(kal_uint32 LineLength, kal_uint32 FrameLength )
{
	kal_uint32 framerate = 0;
	kal_uint32 AutoFlickerFrameLength=0;

	if( SENSOR_MODE_PREVIEW == IMX219.sensorMode )
	{

		framerate = (10 * IMX219.pvPclk) / (FrameLength * LineLength);

			if(framerate>=285)
			 {
				framerate=285;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=220&&framerate<285)
			 {
				framerate=220;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=194&&framerate<220)
			 {
				framerate=194;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=153&&framerate<194)
			 {
				framerate=153;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=147&&framerate<153)
			 {
				framerate=147;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=127&&framerate<147)
			 {
				framerate=127;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=118&&framerate<127)
			 {
				framerate=118;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=102&&framerate<118)
			 {
				framerate=102;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=99&&framerate<102)
			 {
				framerate=99;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
			else if(framerate>=96&&framerate<99)
			 {
				framerate=96;
				AutoFlickerFrameLength = (10 *  IMX219.pvPclk) / (framerate * LineLength);
			 }
	}
	else
	{
		framerate = (10 * IMX219.capPclk) / (FrameLength * LineLength);

	   if(framerate>=285)
		{
		   framerate=285;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=220&&framerate<285)
		{
		   framerate=220;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=194&&framerate<220)
		{
		   framerate=194;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=153&&framerate<194)
		{
		   framerate=153;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=147&&framerate<153)
		{
		   framerate=147;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=127&&framerate<147)
		{
		   framerate=127;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=118&&framerate<127)
		{
		   framerate=118;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=102&&framerate<118)
		{
		   framerate=102;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=99&&framerate<102)
		{
		   framerate=99;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	   else if(framerate>=96&&framerate<99)
		{
		   framerate=96;
		   AutoFlickerFrameLength = (10 * IMX219.capPclk) / (framerate * LineLength);
		}
	}


	IMX219DB("[AutoFlicker]:framerate =%d,IMX219CurrentScenarioId =%d\n", framerate,IMX219CurrentScenarioId);
	IMX219DB("[AutoFlicker]:IMX219.capPclk =%d,LineLength =%d,AutoFlickerFrameLength =%d\n", IMX219.capPclk,LineLength,AutoFlickerFrameLength);


	return AutoFlickerFrameLength;

}



void IMX219_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = IMX219_PV_PERIOD_LINE_NUMS, max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

    line_length  = GetIMX219LineLength();
	frame_length = GetIMX219FrameLength();

	max_shutter  = frame_length-IMX219_SHUTTER_MARGIN;

    frame_length = frame_length + IMX219_CalcExtra_For_ShutterMargin(shutter,max_shutter);


	if(IMX219.IMX219AutoFlickerMode == KAL_TRUE)
	{
        min_framelength = IMX219_CalcFrameLength_For_AutoFlicker(line_length,frame_length);

        if(frame_length < min_framelength)
			frame_length = min_framelength;
	}


	spin_lock_irqsave(&IMX219mipiraw_drv_lock,flags);
	IMX219_FeatureControl_PERIOD_PixelNum = line_length;
	IMX219_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&IMX219mipiraw_drv_lock,flags);

	//Set total frame length
	IMX219MIPI_write_cmos_sensor(0x0160, (frame_length >> 8) & 0xFF);
	IMX219MIPI_write_cmos_sensor(0x0161, frame_length & 0xFF);

	//Set shutter
    IMX219MIPI_write_cmos_sensor(0x015a, (shutter >> 8) & 0xFF);
    IMX219MIPI_write_cmos_sensor(0x015b, shutter & 0xFF);


	IMX219DB("IMX219 write shutter=%d, line_length=%d, frame_length=%d\n", shutter, line_length, frame_length);

}


static kal_uint16 IMX219Reg2Gain(const kal_uint8 iReg)
{
    kal_uint8 GainTCheckIndex;

	//SENSORDB("IMX219Reg2Gain enter:\n");

    for (GainTCheckIndex = 0; GainTCheckIndex < IMX219MIPI_MaxGainIndex; GainTCheckIndex++)
	{
        if(iReg < IMX219MIPI_sensorGainMapping[GainTCheckIndex][1])
		{
            break;
        }
		if(iReg == IMX219MIPI_sensorGainMapping[GainTCheckIndex][1])
		{
			return IMX219MIPI_sensorGainMapping[GainTCheckIndex][0];
		}
    }
    return IMX219MIPI_sensorGainMapping[GainTCheckIndex-1][0];
}


static kal_uint8 IMX219Gain2Reg(const kal_uint16 iGainValue)
{
	kal_uint8 GainTCheckIndex;

    for (GainTCheckIndex = 0; GainTCheckIndex < (IMX219MIPI_MaxGainIndex-1); GainTCheckIndex++)
	{
        if(iGainValue <IMX219MIPI_sensorGainMapping[GainTCheckIndex][0])
		{
            break;
        }

		if(iGainValue < IMX219MIPI_sensorGainMapping[GainTCheckIndex][0])
		{
			return IMX219MIPI_sensorGainMapping[GainTCheckIndex][1];
		}

    }

#if 0
    if(iGainValue != IMX219MIPI_sensorGainMapping[GainTCheckIndex][0])
   {
         printk("[IMX219MIPIGain2Reg] Gain mapping don't correctly:%d %d \n", iGainValue, IMX219MIPI_sensorGainMapping[GainTCheckIndex][0]);
    }
#endif

    return IMX219MIPI_sensorGainMapping[GainTCheckIndex-1][1];

}



void IMX219_SetGain(UINT16 iGain)
{
    kal_uint8 iReg;

	IMX219DB("IMX219_SetGain iGain=%d:\n",iGain);

    iReg = IMX219Gain2Reg(iGain);
    IMX219MIPI_write_cmos_sensor(0x0157,iReg);

    //IMX219DB("IMX219_SetGain:BBvalue =%d,registerValue=%d\n",iGain,iReg);

}


kal_uint16 read_IMX219_gain(void)
{
	kal_uint8 iReg;

	//IMX219DB("read_IMX219_gain enter:\n");

    iReg = IMX219MIPI_read_cmos_sensor(0x0157);

	return IMX219Reg2Gain(iReg);
}


void IMX219_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=IMX219SensorReg[i].Addr; i++)
    {
        IMX219MIPI_write_cmos_sensor(IMX219SensorReg[i].Addr, IMX219SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=IMX219SensorReg[i].Addr; i++)
    {
        IMX219MIPI_write_cmos_sensor(IMX219SensorReg[i].Addr, IMX219SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        IMX219MIPI_write_cmos_sensor(IMX219SensorCCT[i].Addr, IMX219SensorCCT[i].Para);
    }
}


void IMX219_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;

    for(i=0; 0xFFFFFFFF!=IMX219SensorReg[i].Addr; i++)
    {
         temp_data = IMX219MIPI_read_cmos_sensor(IMX219SensorReg[i].Addr);
		 spin_lock(&IMX219mipiraw_drv_lock);
		 IMX219SensorReg[i].Para =temp_data;
		 spin_unlock(&IMX219mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=IMX219SensorReg[i].Addr; i++)
    {
        temp_data = IMX219MIPI_read_cmos_sensor(IMX219SensorReg[i].Addr);
		spin_lock(&IMX219mipiraw_drv_lock);
		IMX219SensorReg[i].Para = temp_data;
		spin_unlock(&IMX219mipiraw_drv_lock);
    }
}


kal_int32  IMX219_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void IMX219_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void IMX219_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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

            temp_para= IMX219SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/IMX219.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= IMX219_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= IMX219_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
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
                    info_ptr->ItemValue=    111;
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



kal_bool IMX219_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
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

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * IMX219.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&IMX219mipiraw_drv_lock);
          IMX219SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&IMX219mipiraw_drv_lock);
          IMX219MIPI_write_cmos_sensor(IMX219SensorCCT[temp_addr].Addr,temp_para);

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
					spin_lock(&IMX219mipiraw_drv_lock);
                    IMX219_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&IMX219mipiraw_drv_lock);
                    break;
                case 1:
                    IMX219MIPI_write_cmos_sensor(IMX219_FAC_SENSOR_REG,ItemValue);
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

static void IMX219_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == IMX219.sensorMode )
	{
		line_length = IMX219_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = IMX219_PV_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = IMX219_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = IMX219_FULL_PERIOD_LINE_NUMS + iLines;
	}

	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219_FeatureControl_PERIOD_PixelNum = line_length;
	IMX219_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&IMX219mipiraw_drv_lock);

	//Set total frame length
	IMX219MIPI_write_cmos_sensor(0x0160, (frame_length >>8) & 0xFF);
	IMX219MIPI_write_cmos_sensor(0x0161, frame_length & 0xFF);

	//Set total line length
	IMX219MIPI_write_cmos_sensor(0x0162, (line_length >>8) & 0xFF);
	IMX219MIPI_write_cmos_sensor(0x0163, line_length & 0xFF);

}   /*  IMX219_SetDummy */

void IMX219PreviewSetting(void)
{
    IMX219DB("IMX219PreviewSetting enter :\n ");

	IMX219MIPI_write_cmos_sensor(0x30EB,  0x05);
	IMX219MIPI_write_cmos_sensor(0x30EB,  0x0C);
	IMX219MIPI_write_cmos_sensor(0x300A,  0xFF);
	IMX219MIPI_write_cmos_sensor(0x300B,  0xFF);
	IMX219MIPI_write_cmos_sensor(0x30EB,  0x05);
	IMX219MIPI_write_cmos_sensor(0x30EB,  0x09);

	IMX219MIPI_write_cmos_sensor(0x0114,  0x01);  //lane mode: 01-2lane// 03-4lane

	IMX219MIPI_write_cmos_sensor(0x0128,  0x00);
	IMX219MIPI_write_cmos_sensor(0x012A,  0x18);
	IMX219MIPI_write_cmos_sensor(0x012B,  0x00);

	IMX219MIPI_write_cmos_sensor(0x0160,  0x04); //1265  //frame length  Y total
	IMX219MIPI_write_cmos_sensor(0x0161,  0xF1);

	IMX219MIPI_write_cmos_sensor(0x0162,  0x0D);//3448   //line length  X total
	IMX219MIPI_write_cmos_sensor(0x0163,  0x78);

	IMX219MIPI_write_cmos_sensor(0x0164,  0x00);
	IMX219MIPI_write_cmos_sensor(0x0165,  0x00);
	IMX219MIPI_write_cmos_sensor(0x0166,  0x0C);
	IMX219MIPI_write_cmos_sensor(0x0167,  0xCF);
	IMX219MIPI_write_cmos_sensor(0x0168,  0x00);
	IMX219MIPI_write_cmos_sensor(0x0169,  0x00);
	IMX219MIPI_write_cmos_sensor(0x016A,  0x09);
	IMX219MIPI_write_cmos_sensor(0x016B,  0x9F);

	IMX219MIPI_write_cmos_sensor(0x016C,  0x06);//1640 //X output size
	IMX219MIPI_write_cmos_sensor(0x016D,  0x68);

	IMX219MIPI_write_cmos_sensor(0x016E,  0x04); //1232 //Y output size
	IMX219MIPI_write_cmos_sensor(0x016F,  0xD0);

	IMX219MIPI_write_cmos_sensor(0x0170,  0x01);
	IMX219MIPI_write_cmos_sensor(0x0171,  0x01);
	IMX219MIPI_write_cmos_sensor(0x0174,  0x01);
	IMX219MIPI_write_cmos_sensor(0x0175,  0x01);
	IMX219MIPI_write_cmos_sensor(0x018C,  0x0A);
	IMX219MIPI_write_cmos_sensor(0x018D,  0x0A);
	IMX219MIPI_write_cmos_sensor(0x0301,  0x05);
	IMX219MIPI_write_cmos_sensor(0x0303,  0x01);
	IMX219MIPI_write_cmos_sensor(0x0304,  0x03);
	IMX219MIPI_write_cmos_sensor(0x0305,  0x03);
	IMX219MIPI_write_cmos_sensor(0x0306,  0x00);

	IMX219MIPI_write_cmos_sensor(0x0307,  0x29); //PLL_VT_MPY[7:0]

	IMX219MIPI_write_cmos_sensor(0x0309,  0x0A);
	IMX219MIPI_write_cmos_sensor(0x030B,  0x01);
	IMX219MIPI_write_cmos_sensor(0x030C,  0x00);

	IMX219MIPI_write_cmos_sensor(0x030D,  0x52); //PLL_OP_MPY[7:0]
#if 0	// Remove register setting as following sample version, latest version should be removed Sony korea recommended.
	IMX219MIPI_write_cmos_sensor(0x455E,  0x00);

	IMX219MIPI_write_cmos_sensor(0x471E,  0x4B);
#endif
	IMX219MIPI_write_cmos_sensor(0x4767,  0x0F);
	IMX219MIPI_write_cmos_sensor(0x4750,  0x14);
#if 0	// Remove register setting as following sample version, latest version should be removed Sony korea recommended.
	IMX219MIPI_write_cmos_sensor(0x4540,  0x00);
#endif
	IMX219MIPI_write_cmos_sensor(0x47B4,  0x14);
#if 0 // Remove register setting as following sample version, latest version should be removed Sony korea recommended.
	IMX219MIPI_write_cmos_sensor(0x4713,  0x30);
	IMX219MIPI_write_cmos_sensor(0x478B,  0x10);
	IMX219MIPI_write_cmos_sensor(0x478F,  0x10);
	IMX219MIPI_write_cmos_sensor(0x4793,  0x10);
	IMX219MIPI_write_cmos_sensor(0x4797,  0x0E);
	IMX219MIPI_write_cmos_sensor(0x479B,  0x0E);
#endif
	IMX219MIPI_write_cmos_sensor(0x0100,  0x01);

    IMX219DB("IMX219PreviewSetting exit :\n ");
}

void IMX219CaptureSetting(void)
{
    IMX219DB("IMX219CaptureSetting enter :\n ");

	IMX219MIPI_write_cmos_sensor(0x30EB,   0x05);
	IMX219MIPI_write_cmos_sensor(0x30EB,   0x0C);
	IMX219MIPI_write_cmos_sensor(0x300A,   0xFF);
	IMX219MIPI_write_cmos_sensor(0x300B,   0xFF);
	IMX219MIPI_write_cmos_sensor(0x30EB,   0x05);
	IMX219MIPI_write_cmos_sensor(0x30EB,   0x09);

	IMX219MIPI_write_cmos_sensor(0x0114,   0x01); // 2 lane

	IMX219MIPI_write_cmos_sensor(0x0128,   0x00);
	IMX219MIPI_write_cmos_sensor(0x012A,   0x18);
	IMX219MIPI_write_cmos_sensor(0x012B,   0x00);

	IMX219MIPI_write_cmos_sensor(0x0160,   0x09);
	IMX219MIPI_write_cmos_sensor(0x0161,   0xE9); //Frame length ;Y Total 2537

	IMX219MIPI_write_cmos_sensor(0x0162,   0x0D); //line length ; X Total 3448
	IMX219MIPI_write_cmos_sensor(0x0163,   0x78);

	IMX219MIPI_write_cmos_sensor(0x0164,   0x00);
	IMX219MIPI_write_cmos_sensor(0x0165,   0x00);
	IMX219MIPI_write_cmos_sensor(0x0166,   0x0C);
	IMX219MIPI_write_cmos_sensor(0x0167,   0xCF);
	IMX219MIPI_write_cmos_sensor(0x0168,   0x00);
	IMX219MIPI_write_cmos_sensor(0x0169,   0x00);
	IMX219MIPI_write_cmos_sensor(0x016A,   0x09);
	IMX219MIPI_write_cmos_sensor(0x016B,   0x9F);

	IMX219MIPI_write_cmos_sensor(0x016C,   0x0C); //X output size /3280
	IMX219MIPI_write_cmos_sensor(0x016D,   0xD0);

	IMX219MIPI_write_cmos_sensor(0x016E,   0x09); //Y output size /2464
	IMX219MIPI_write_cmos_sensor(0x016F,   0xA0);

	IMX219MIPI_write_cmos_sensor(0x0170,   0x01);
	IMX219MIPI_write_cmos_sensor(0x0171,   0x01);
	IMX219MIPI_write_cmos_sensor(0x0174,   0x00);
	IMX219MIPI_write_cmos_sensor(0x0175,   0x00);
	IMX219MIPI_write_cmos_sensor(0x018C,   0x0A);
	IMX219MIPI_write_cmos_sensor(0x018D,   0x0A);
	IMX219MIPI_write_cmos_sensor(0x0301,   0x05);
	IMX219MIPI_write_cmos_sensor(0x0303,   0x01);
	IMX219MIPI_write_cmos_sensor(0x0304,   0x03);
	IMX219MIPI_write_cmos_sensor(0x0305,   0x03);
	IMX219MIPI_write_cmos_sensor(0x0306,   0x00);

	IMX219MIPI_write_cmos_sensor(0x0307,   0x29); //PLL_VT_MPY[7:0]

	IMX219MIPI_write_cmos_sensor(0x0309,   0x0A);
	IMX219MIPI_write_cmos_sensor(0x030B,   0x01);
	IMX219MIPI_write_cmos_sensor(0x030C,   0x00);

	IMX219MIPI_write_cmos_sensor(0x030D,   0x52); //PLL_OP_MPY[7:0]

#if 0	// Remove register setting as following sample version, latest version should be removed Sony korea recommended.
	IMX219MIPI_write_cmos_sensor(0x455E,   0x00);
	IMX219MIPI_write_cmos_sensor(0x471E,   0x4B);
#endif
	IMX219MIPI_write_cmos_sensor(0x4767,   0x0F);
	IMX219MIPI_write_cmos_sensor(0x4750,   0x14);
#if 0	// Remove register setting as following sample version, latest version should be removed Sony korea recommended.
	IMX219MIPI_write_cmos_sensor(0x4540,   0x00);
#endif
	IMX219MIPI_write_cmos_sensor(0x47B4,   0x14);
#if 0 // Remove register setting as following sample version, latest version should be removed Sony korea recommended.
	IMX219MIPI_write_cmos_sensor(0x4713,   0x30);
	IMX219MIPI_write_cmos_sensor(0x478B,   0x10);
	IMX219MIPI_write_cmos_sensor(0x478F,   0x10);
	IMX219MIPI_write_cmos_sensor(0x4793,   0x10);
	IMX219MIPI_write_cmos_sensor(0x4797,   0x0E);
	IMX219MIPI_write_cmos_sensor(0x479B,   0x0E);
#endif
	IMX219MIPI_write_cmos_sensor(0x0100,   0x01);

	IMX219DB("IMX219CaptureSetting exit :\n ");
}


static void IMX219_Sensor_Init(void)
{
	IMX219DB("enter IMX219_Sensor_Init: need nothing \n ");

}


UINT32 IMX219Open(void)
{

    int  retry = 2;
	kal_uint16 sensorid=0;

	IMX219DB("[IMX219MIPI]enter IMX219MIPIOpen function\n");

    do {
	   sensorid=(kal_uint16)(((IMX219MIPI_read_cmos_sensor(0x0000)&&0x0f)<<8) | IMX219MIPI_read_cmos_sensor(0x0001));
		if (sensorid == IMX219MIPI_SENSOR_ID)
			break;
		retry--;
	    }
	while (retry > 0);

    IMX219DB("Read Sensor ID = 0x%x\n", sensorid);
    if (sensorid != IMX219MIPI_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

	IMX219_Sensor_Init();

	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219.sensorMode = SENSOR_MODE_INIT;
	IMX219.IMX219AutoFlickerMode = KAL_FALSE;
	IMX219.DummyLines= 0;
	IMX219.DummyPixels= 0;
	IMX219.pvPclk =  IMX219_PREVIEW_PCLK;
	IMX219.capPclk = IMX219_CAPTURE_PCLK;
	IMX219.shutter = 0;
	spin_unlock(&IMX219mipiraw_drv_lock);

	IMX219DB("IMX219Open exit :\n ");

    return ERROR_NONE;
}


UINT32 IMX219GetSensorID(UINT32 *sensorID)
{
    int  retry = 2;

	IMX219DB("IMX219GetSensorID enter :\n ");

	do {
	   *sensorID =(kal_uint16)(((IMX219MIPI_read_cmos_sensor(0x0000)&&0x0f)<<8) | IMX219MIPI_read_cmos_sensor(0x0001));
		if (*sensorID == IMX219MIPI_SENSOR_ID)
		{
			IMX219DB("Sensor ID = 0x%x\n", *sensorID);
			break;
		}
		retry--;

	} while (retry > 0);

	if (*sensorID != IMX219MIPI_SENSOR_ID) {
		*sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
    *sensorID = IMX219_SENSOR_ID;
	return ERROR_NONE;

}


void IMX219_SetShutter(kal_uint32 iShutter)
{

	//if(IMX219.shutter == iShutter)
	//	return;


   IMX219DB("IMX219_SetShutter iShutter=%d:\n",iShutter);

   spin_lock(&IMX219mipiraw_drv_lock);
   IMX219.shutter= iShutter;
   spin_unlock(&IMX219mipiraw_drv_lock);

   IMX219_write_shutter(iShutter);
   return;
}

UINT16 IMX219_read_shutter(void)
{
    return (UINT16)((IMX219MIPI_read_cmos_sensor(0x015a)<<8)| IMX219MIPI_read_cmos_sensor(0x015b));
}

void IMX219_NightMode(kal_bool bEnable)
{

}


UINT32 IMX219Close(void)
{
    return ERROR_NONE;
}


void IMX219SetFlipMirror(kal_int32 imgMirror)
{
	//need check
#if 0
    kal_uint8  iTemp;

	IMX219DB("[IMX219MIPI]enter IMX219MIPISetFlipMirror function: imgMirror=%d\n",imgMirror);

    iTemp = IMX219MIPI_read_cmos_sensor(0x0172) & 0x03;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            IMX219MIPI_write_cmos_sensor(0x0172, 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            IMX219MIPI_write_cmos_sensor(0x0172, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            IMX219MIPI_write_cmos_sensor(0x0172, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            IMX219MIPI_write_cmos_sensor(0x0172, 0x00);	//Set mirror and flip
            break;
    }
#endif
}

kal_uint32 PrvCapShutterConvert(kal_uint32 previewShutter)
{
	kal_uint32 pv_line_length=0 , cap_line_length=0;
	kal_uint32 CAPShutter=0;

	pv_line_length = IMX219_PV_PERIOD_PIXEL_NUMS + IMX219.DummyPixels;
	cap_line_length = IMX219_FULL_PERIOD_PIXEL_NUMS + IMX219.DummyPixels;

#if 0
	//CAPShutter =  previewShutter * pv_line_length/cap_line_length;
	//CAPShutter = CAPShutter *IMX219.capPclk/IMX219.pvPclk;
	//shutter *= 2; //preview binning Average or sum
#else
    //for Imx219
    //pv pixel num = cap pixel num
    //pv pixelclk = cap pixelclk
    //pv binng average
	CAPShutter = previewShutter;
#endif

    return CAPShutter;

}


UINT32 IMX219Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	IMX219DB("IMX219Preview enter:\n");

	if(IMX219.sensorMode == SENSOR_MODE_PREVIEW)
	{
		// Do nothing,for CCT flow
	}
	else
	{
		IMX219PreviewSetting();
	}

	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219.sensorMode = SENSOR_MODE_PREVIEW;
	IMX219.DummyPixels =0;
	IMX219.DummyLines  =0;
	IMX219.imgMirror = sensor_config_data->SensorImageMirror;

	IMX219_FeatureControl_PERIOD_PixelNum=IMX219_PV_PERIOD_PIXEL_NUMS+ IMX219.DummyPixels;
	IMX219_FeatureControl_PERIOD_LineNum=IMX219_PV_PERIOD_LINE_NUMS+IMX219.DummyLines;
	spin_unlock(&IMX219mipiraw_drv_lock);

	IMX219SetFlipMirror(sensor_config_data->SensorImageMirror);

	IMX219DB("IMX219Preview exit:\n");
    return ERROR_NONE;
}


UINT32 IMX219Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 CAPshutter = 0;

	IMX219DB("IMX219Capture enter:\n");

	if( SENSOR_MODE_CAPTURE != IMX219.sensorMode)
	{
		//shutter=IMX219_read_shutter();
		IMX219CaptureSetting();
	}

	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219.sensorMode = SENSOR_MODE_CAPTURE;
	IMX219.imgMirror = sensor_config_data->SensorImageMirror;
	IMX219.DummyPixels =0;
	IMX219.DummyLines  =0;
	IMX219_FeatureControl_PERIOD_PixelNum=IMX219_FULL_PERIOD_PIXEL_NUMS+ IMX219.DummyPixels;
	IMX219_FeatureControl_PERIOD_LineNum=IMX219_FULL_PERIOD_LINE_NUMS+IMX219.DummyLines;
	spin_unlock(&IMX219mipiraw_drv_lock);
	IMX219SetFlipMirror(sensor_config_data->SensorImageMirror);

    if(IMX219CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
    {
		IMX219DB("IMX219Capture exit ZSD!!\n");
		return;
    }

    CAPshutter = PrvCapShutterConvert(IMX219.shutter);

	if(CAPshutter < 4)
	    CAPshutter = 4;

	IMX219_write_shutter(CAPshutter);

	IMX219DB("IMX219Capture exit:\n");
    return ERROR_NONE;
}	/* IMX219Capture() */


UINT32 IMX219GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    IMX219DB("IMX219GetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= IMX219_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= IMX219_IMAGE_SENSOR_PV_HEIGHT;

    pSensorResolution->SensorFullWidth		= IMX219_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= IMX219_IMAGE_SENSOR_FULL_HEIGHT;

    return ERROR_NONE;
}

UINT32 IMX219GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&IMX219mipiraw_drv_lock);

   //	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_R;
   //	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gr;
	//	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gb;

    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

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

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;
    pSensorInfo->AESensorGainDelayFrame = 1 ;
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = IMX219_PV_X_START;
            pSensorInfo->SensorGrabStartY = IMX219_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;

        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = IMX219_FULL_X_START;
            pSensorInfo->SensorGrabStartY = IMX219_FULL_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = IMX219_PV_X_START;
            pSensorInfo->SensorGrabStartY = IMX219_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &IMX219SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* IMX219GetInfo() */


UINT32 IMX219Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219CurrentScenarioId = ScenarioId;
	spin_unlock(&IMX219mipiraw_drv_lock);

	IMX219DB("IMX219CurrentScenarioId=%d\n",IMX219CurrentScenarioId);

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            IMX219Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            IMX219Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return TRUE;
} /* IMX219Control() */


UINT32 IMX219SetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MAX_Frame_length =0,frameRate=0,extralines=0;
    IMX219DB("IMX219SetVideoMode frame rate = %d\n", u2FrameRate);

	if(u2FrameRate==0)
	{
		IMX219DB("Disable Video Mode or Dynamic fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    IMX219DB("error frame rate seting\n");

	spin_lock(&IMX219mipiraw_drv_lock);
	IMX219_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&IMX219mipiraw_drv_lock);

    if(IMX219.sensorMode == SENSOR_MODE_PREVIEW)
    {
    	if(IMX219.IMX219AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 296;
			else if(u2FrameRate==15)
				frameRate= 146;
			else
				frameRate=u2FrameRate*10;

			MAX_Frame_length = (IMX219.pvPclk)/(IMX219_PV_PERIOD_PIXEL_NUMS + IMX219.DummyPixels)/frameRate*10;
    	}
		else
			MAX_Frame_length = (IMX219.pvPclk) /(IMX219_PV_PERIOD_PIXEL_NUMS + IMX219.DummyPixels)/u2FrameRate;

		if((MAX_Frame_length <=IMX219_PV_PERIOD_LINE_NUMS))
		{
			MAX_Frame_length = IMX219_PV_PERIOD_LINE_NUMS;
			//IMX219DB("[IMX219SetVideoMode]current fps = %d\n", (IMX219.pvPclk)  /(IMX219_PV_PERIOD_PIXEL_NUMS)/IMX219_PV_PERIOD_LINE_NUMS);
		}
		IMX219DB("[IMX219SetVideoMode]current fps (10 base)= %d\n", (IMX219.pvPclk)*10/(IMX219_PV_PERIOD_PIXEL_NUMS + IMX219.DummyPixels)/MAX_Frame_length);
		extralines = MAX_Frame_length - IMX219_PV_PERIOD_LINE_NUMS;

		spin_lock(&IMX219mipiraw_drv_lock);
		IMX219.DummyPixels =0;
		IMX219.DummyLines  =extralines;
		spin_unlock(&IMX219mipiraw_drv_lock);

		IMX219_SetDummy(IMX219.DummyPixels,extralines);
    }
	else if(IMX219.sensorMode == SENSOR_MODE_CAPTURE)//also could be used in 1080p video-record
	{
		IMX219DB("-------[IMX219SetVideoMode]ZSD? /1080p Record?---------\n");
		if(IMX219.IMX219AutoFlickerMode == KAL_TRUE)
    	{
			if (u2FrameRate==15)
				frameRate= 148;//For ZSD	 mode//15fps
			else
				frameRate=u2FrameRate*10;
			MAX_Frame_length = (IMX219.capPclk) /(IMX219_FULL_PERIOD_PIXEL_NUMS + IMX219.DummyPixels)/frameRate*10;
    	}
		else
			MAX_Frame_length = (IMX219.capPclk) /(IMX219_FULL_PERIOD_PIXEL_NUMS + IMX219.DummyPixels)/u2FrameRate;

		if((MAX_Frame_length <=IMX219_FULL_PERIOD_LINE_NUMS))
		{
			MAX_Frame_length = IMX219_FULL_PERIOD_LINE_NUMS;
			//IMX219DB("[IMX219SetVideoMode]current fps = %d\n", (IMX219.capPclk) /(IMX219_FULL_PERIOD_PIXEL_NUMS)/IMX219_FULL_PERIOD_LINE_NUMS);

		}
		IMX219DB("[IMX219SetVideoMode]current fps (10 base)= %d\n", (IMX219.capPclk)*10/(IMX219_FULL_PERIOD_PIXEL_NUMS + IMX219.DummyPixels)/MAX_Frame_length);

		extralines = MAX_Frame_length - IMX219_FULL_PERIOD_LINE_NUMS;

		spin_lock(&IMX219mipiraw_drv_lock);
		IMX219.DummyPixels =0;
		IMX219.DummyLines  =extralines;
		spin_unlock(&IMX219mipiraw_drv_lock);

		IMX219_SetDummy(IMX219.DummyPixels,extralines);
	}
	IMX219DB("[IMX219SetVideoMode]MAX_Frame_length=%d,IMX219.DummyLines=%d\n",MAX_Frame_length,IMX219.DummyLines);

    return KAL_TRUE;
}

UINT32 IMX219SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{

	if(bEnable) {
		spin_lock(&IMX219mipiraw_drv_lock);
		IMX219.IMX219AutoFlickerMode = KAL_TRUE;
		spin_unlock(&IMX219mipiraw_drv_lock);
    } else {
    	spin_lock(&IMX219mipiraw_drv_lock);
        IMX219.IMX219AutoFlickerMode = KAL_FALSE;
		spin_unlock(&IMX219mipiraw_drv_lock);
    }

    return TRUE;
}

UINT32 IMX219SetTestPatternMode(kal_bool bEnable)
{
    IMX219DB("[IMX219SetTestPatternMode] Test pattern enable:%d\n", bEnable);

    return TRUE;
}

UINT32 IMX219FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
            *pFeatureReturnPara16++= IMX219_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= IMX219_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++= IMX219_FeatureControl_PERIOD_PixelNum;
			*pFeatureReturnPara16= IMX219_FeatureControl_PERIOD_LineNum;
			*pFeatureParaLen=4;
			break;

        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
    			switch(IMX219CurrentScenarioId)
    			{
					case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
					case MSDK_SCENARIO_ID_CAMERA_ZSD:
						*pFeatureReturnPara32 = IMX219_CAPTURE_PCLK;
	            	 	*pFeatureParaLen=4;
	         			 break;

					case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
	         		default:
	            		*pFeatureReturnPara32 = IMX219_PREVIEW_PCLK;
	            		*pFeatureParaLen=4;
	            		break;
	        	}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            IMX219_SetShutter(*pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_NIGHTMODE:
            IMX219_NightMode((BOOL) *pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_GAIN:
            IMX219_SetGain((UINT16) *pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;

        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;

        case SENSOR_FEATURE_SET_REGISTER:
            IMX219MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;

        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = IMX219MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;

        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&IMX219mipiraw_drv_lock);
                IMX219SensorCCT[i].Addr=*pFeatureData32++;
                IMX219SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&IMX219mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=IMX219SensorCCT[i].Addr;
                *pFeatureData32++=IMX219SensorCCT[i].Para;
            }
            break;

        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&IMX219mipiraw_drv_lock);
                IMX219SensorReg[i].Addr=*pFeatureData32++;
                IMX219SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&IMX219mipiraw_drv_lock);
            }
            break;

        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=IMX219SensorReg[i].Addr;
                *pFeatureData32++=IMX219SensorReg[i].Para;
            }
            break;

        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=IMX219_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, IMX219SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, IMX219SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;

        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &IMX219SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;

        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            IMX219_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            IMX219_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=IMX219_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            IMX219_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            IMX219_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            IMX219_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
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
            IMX219SetVideoMode(*pFeatureData16);
            break;

        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            IMX219GetSensorID(pFeatureReturnPara32);
            break;

        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            IMX219SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;

        case SENSOR_FEATURE_SET_TEST_PATTERN:
            IMX219SetTestPatternMode((BOOL)*pFeatureData16);
            break;

        default:
            break;
    }
    return ERROR_NONE;
}	/* IMX219FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncIMX219=
{
    IMX219Open,
    IMX219GetInfo,
    IMX219GetResolution,
    IMX219FeatureControl,
    IMX219Control,
    IMX219Close
};

UINT32 IMX219_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncIMX219;

    return ERROR_NONE;
}   /* SensorInit() */

