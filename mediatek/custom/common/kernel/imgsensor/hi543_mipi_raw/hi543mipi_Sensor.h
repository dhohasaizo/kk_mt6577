/*****************************************************************************
 *
 * Filename:
 * ---------
 *   mt9p017mipi_Sensor.h
 *
 * Project:
 * --------
 *   YUSU
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   Guangye Yang (mtk70662)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef _HI543MIPI_SENSOR_H
#define _HI543MIPI_SENSOR_H

typedef enum group_enum {
    PRE_GAIN=0,
    CMMCLK_CURRENT,
    FRAME_RATE_LIMITATION,
    REGISTER_EDITOR,
    GROUP_TOTAL_NUMS
} FACTORY_GROUP_ENUM;


#define ENGINEER_START_ADDR 10
#define FACTORY_START_ADDR 0
    

typedef enum register_index
{
	SENSOR_BASEGAIN=FACTORY_START_ADDR,
	PRE_GAIN_R_INDEX,
	PRE_GAIN_Gr_INDEX,
	PRE_GAIN_Gb_INDEX,
	PRE_GAIN_B_INDEX,
	FACTORY_END_ADDR
} FACTORY_REGISTER_INDEX;

typedef enum engineer_index
{
    CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
    ENGINEER_END
} FACTORY_ENGINEER_INDEX;



typedef struct
{
    SENSOR_REG_STRUCT	Reg[ENGINEER_END];
    SENSOR_REG_STRUCT	CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;



#define CURRENT_MAIN_SENSOR                HI543MIPI_MICRON
//if define RAW10, MIPI_INTERFACE must be defined
//if MIPI_INTERFACE is marked, RAW10 must be marked too
#define MIPI_INTERFACE
#define RAW10

#define HI543MIPI_WRITE_ID	0x40
#define HI543MIPI_READ_ID	0x41

#define HI543MIPI_IMAGE_SENSOR_FULL_HACTIVE		2560//V0.07
#define HI543MIPI_IMAGE_SENSOR_FULL_VACTIVE		1920//v0.07

#define HI543MIPI_IMAGE_SENSOR_PV_HACTIVE		1280//V0.07
#define HI543MIPI_IMAGE_SENSOR_PV_VACTIVE		960//V0.07

#define HI543MIPI_FULL_START_X					6
#define HI543MIPI_FULL_START_Y					2
#define HI543MIPI_IMAGE_SENSOR_FULL_WIDTH		(2560-56)
#define HI543MIPI_IMAGE_SENSOR_FULL_HEIGHT		(1920-42)

#define HI543MIPI_PV_START_X					2
#define HI543MIPI_PV_START_Y					4
#define HI543MIPI_IMAGE_SENSOR_PV_WIDTH			(1280-28)
#define HI543MIPI_IMAGE_SENSOR_PV_HEIGHT		(960-21)
   
//this need check with FAE//////////
#define	HI543MIPI_IMAGE_SENSOR_FULL_HBLANKING	284//V0.07
#define HI543MIPI_IMAGE_SENSOR_FULL_VBLANKING	27//V0.07

#define	HI543MIPI_IMAGE_SENSOR_PV_HBLANKING		1564//V0.07
#define HI543MIPI_IMAGE_SENSOR_PV_VBLANKING		559//V0.7
////////////////////

#define HI543MIPI_FULL_PERIOD_PIXEL_NUMS			(HI543MIPI_IMAGE_SENSOR_FULL_HACTIVE + HI543MIPI_IMAGE_SENSOR_FULL_HBLANKING)  //2576+4+264 = 2844
#define HI543MIPI_FULL_PERIOD_LINE_NUMS			(HI543MIPI_IMAGE_SENSOR_FULL_VACTIVE + HI543MIPI_IMAGE_SENSOR_FULL_VBLANKING)  //1936+4+23 =1963 
#define HI543MIPI_PV_PERIOD_PIXEL_NUMS			(HI543MIPI_IMAGE_SENSOR_PV_HACTIVE + HI543MIPI_IMAGE_SENSOR_PV_HBLANKING)	 //2576 +xxxx =
#define HI543MIPI_PV_PERIOD_LINE_NUMS				(HI543MIPI_IMAGE_SENSOR_PV_VACTIVE + HI543MIPI_IMAGE_SENSOR_PV_VBLANKING)    //1936 + xxxx =



#define HI543MIPI_FRAME_RATE_UNIT		10
#define HI543MIPI_set_frame_rate(a) 	(a * HI543MIPI_FRAME_RATE_UNIT)
#define HI543MIPI_1X_ZOOM_IN_CAPTURE_FRAME	9



/* SENSOR PRIVATE STRUCT *///
struct HI543MIPI_SENSOR_STRUCT
{
	kal_uint8 i2c_write_id;
	kal_uint8 i2c_read_id;
	kal_uint32 preview_vt_clk;
	kal_uint32 capture_vt_clk;
	kal_uint16 frame_length;
	kal_uint16 line_length;  
	kal_uint32 shutter;
};


/* FRAME RATE */
#define HI543MIPI_FPS(x)                          ((kal_uint32)(10 * (x)))

#endif /* _HI543MIPI_SENSOR_H */

