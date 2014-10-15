#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

//#include "msdk_nvram_camera_exp.h"
#include "camera_custom_nvram.h"
//#include "msdk_sensor_exp.h"
#include "camera_custom_sensor.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
//#include "image_sensor.h"
//TODO:remove once build system ready
//#include "camera_custom_cfg.h"

/*
#if (defined(DRV_ISP_6516_SERIES))

#if !defined(ISP_SUPPORT)
	// DO NOT delete this section!!!
	// When ISP_SUPPORT is not defined, NVRAM still need the dummy structure
	// and default value to initialize NVRAM_EF_CAMERA_PARA_LID.
	//#include "camera_para.h"
	const nvram_camera_para_struct HI704_YUV_CAMERA_PARA_DEFAULT_VALUE={0};
	const nvram_camera_3a_struct HI704_YUV_CAMERA_3A_NVRAM_DEFAULT_VALUE={0};
#else
//#include "camera_para.h"
//#include "camera_sensor_para_HI704_YUV.h"
//#include "camera_af_para.h"
*/
#define SENSOR_ID   HI704_SENSOR_ID

typedef NSFeature::YUVSensorInfo<SENSOR_ID> SensorInfoSingleton_T;
namespace NSFeature {
template <>
UINT32
SensorInfoSingleton_T::
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const
{
    return  NULL;
}};  //  NSFeature











//PFUNC_GETCAMERADEFAULT pHI704_YUV_getDefaultData = HI704_YUV_getDefaultData;

/*
#endif //#if !defined(ISP_SUPPORT)

#endif //#if (defined(DRV_ISP_6516_SERIES))
*/

