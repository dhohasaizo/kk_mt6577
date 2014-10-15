/*******************************************************************************************/


/*******************************************************************************************/

/* SENSOR FULL SIZE */
#ifndef __IMX219_MIPIRAW_SENSOR_H
#define __IMX219_MIPIRAW_SENSOR_H

typedef enum group_enum {
    PRE_GAIN=0,
    CMMCLK_CURRENT,
    FRAME_RATE_LIMITATION,
    REGISTER_EDITOR,
    GROUP_TOTAL_NUMS
} FACTORY_GROUP_ENUM;


#define ENGINEER_START_ADDR 10
#define FACTORY_START_ADDR 0

typedef enum engineer_index
{
    CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
    ENGINEER_END
} FACTORY_ENGINEER_INDEX;

typedef enum register_index
{
	SENSOR_BASEGAIN=FACTORY_START_ADDR,
	PRE_GAIN_R_INDEX,
	PRE_GAIN_Gr_INDEX,
	PRE_GAIN_Gb_INDEX,
	PRE_GAIN_B_INDEX,
	FACTORY_END_ADDR
} FACTORY_REGISTER_INDEX;

typedef struct
{
    SENSOR_REG_STRUCT	Reg[ENGINEER_END];
    SENSOR_REG_STRUCT	CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;

typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_CAPTURE
} IMX219_SENSOR_MODE;


typedef struct
{
	kal_uint32 DummyPixels;
	kal_uint32 DummyLines;
	
	kal_uint32 pvShutter;
	
	kal_uint32 pvPclk;  // x10 480 for 48MHZ
	kal_uint32 capPclk; // x10
	
	kal_uint32 shutter;

	kal_int16 imgMirror;

	IMX219_SENSOR_MODE sensorMode;

	kal_bool IMX219AutoFlickerMode;
	
}IMX219_PARA_STRUCT,*PIMX219_PARA_STRUCT;


	#define IMX219_PV_X_START					(2)
	#define IMX219_PV_Y_START					(2)
	#define IMX219_FULL_X_START 				(2)
	#define IMX219_FULL_Y_START 				(2)

	#define IMX219_IMAGE_SENSOR_PV_WIDTH		(1632)
	#define IMX219_IMAGE_SENSOR_PV_HEIGHT		(1224)
	#define IMX219_IMAGE_SENSOR_FULL_WIDTH		(3264)	
	#define IMX219_IMAGE_SENSOR_FULL_HEIGHT 	(2448)

	#define IMX219_PV_PERIOD_PIXEL_NUMS 		(0x0d78)//3448	
	#define IMX219_PV_PERIOD_LINE_NUMS			(0x04f1)//1265	
	#define IMX219_FULL_PERIOD_PIXEL_NUMS		(0x0d78)//3448	
	#define IMX219_FULL_PERIOD_LINE_NUMS		(0x09e9)//2537	

	

	#define IMX219_MAX_ANALOG_GAIN					(16)
	#define IMX219_MIN_ANALOG_GAIN					(1)
	#define IMX219_ANALOG_GAIN_1X					(0x0020)


	
	#define IMX219MIPI_WRITE_ID	(0x34)
	#define IMX219MIPI_READ_ID	(0x35)


	//#define IMX219MIPI_SENSOR_ID            IMX219_SENSOR_ID
	#define IMX219MIPI_SENSOR_ID            0x0119

    #define IMX219_SHUTTER_MARGIN (4)
	#define IMX219_AUTOFLICKER_OFFSET_30  (296)
	#define IMX219_AUTOFLICKER_OFFSET_15  (146)

	#define IMX219_PREVIEW_PCLK (131200000)
	#define IMX219_CAPTURE_PCLK (131200000)



//export functions
UINT32 IMX219MIPIOpen(void);
UINT32 IMX219MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 IMX219MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 IMX219MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 IMX219MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 IMX219MIPIClose(void);


#endif

