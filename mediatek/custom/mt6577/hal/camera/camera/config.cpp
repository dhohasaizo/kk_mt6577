#include <stdlib.h>
#include <stdio.h>
#include "camera_custom_if.h"

namespace NSCamCustom
{
/*******************************************************************************
*
*******************************************************************************/
#include "cfg_tuning.h"
#include "cfg_facebeauty_tuning.h"
#include "flicker_tuning.h"
//
#include "cfg_setting_imgsensor.h"
#include "cfg_tuning_imgsensor.h"

/*******************************************************************************
* custom exif
*******************************************************************************/
#define EN_CUSTOM_EXIF_INFO
/*                                                    */
#define CUSTOM_EXIF_LGE

#ifdef CUSTOM_EXIF_LGE
static customExifInfo_t exifTag;
#endif
/*                                                    */

MINT32 custom_SetExif(void **ppCustomExifTag)
{
#ifdef EN_CUSTOM_EXIF_INFO
/*                                                    */
#ifdef CUSTOM_EXIF_LGE
    char exif_string_make[PROPERTY_VALUE_MAX] = {"LG Electronics"};
    char exif_string_model[PROPERTY_VALUE_MAX];
    char  exif_string_software[PROPERTY_VALUE_MAX];
#if 0
    if (!property_get("ro.product.manufacturer", exif_string_make , NULL) || strlen((const char*)exif_string_make ) > 32)
    {
        return -1;
    }
#endif
    if (!property_get("ro.product.model", exif_string_model , NULL) || strlen((const char*)exif_string_model ) > 32)
    {
        return -1;
    }

    if (!property_get("ro.lge.swversion", exif_string_software , NULL) || strlen((const char*)exif_string_software ) > 32)
    {
        return -1;
    }

    strcpy((char*)exifTag.strMake, exif_string_make);
    strcpy((char*)exifTag.strModel, exif_string_model);
    strcpy((char*)exifTag.strSoftware, exif_string_software);
#else
#define CUSTOM_EXIF_STRING_MAKE  "custom make"
#define CUSTOM_EXIF_STRING_MODEL "custom model"
#define CUSTOM_EXIF_STRING_SOFTWARE "custom software"
    static customExifInfo_t exifTag = {CUSTOM_EXIF_STRING_MAKE,CUSTOM_EXIF_STRING_MODEL,CUSTOM_EXIF_STRING_SOFTWARE};
#endif
/*                                                    */

    if (0 != ppCustomExifTag) {
        *ppCustomExifTag = (void*)&exifTag;
    }
    return 0;
#else
    return -1;
#endif
}
//
customExif_t const&
getCustomExif()
{
    static customExif_t inst = {
        bEnCustom       :   false,  // default value: false.
        u4ExpProgram    :   0,      // default value: 0.    '0' means not defined, '1' manual control, '2' program normal
    };
    return inst;
}
//
/*******************************************************************************
* LCM physical orienation
*   Return:
*       0   : no inverse
*       1   : inverse
*       -1  : error
*******************************************************************************/
MUINT32
getLCMPhysicalOrientation()
{
    return ::atoi(MTK_LCM_PHYSICAL_ROTATION);
}

/*******************************************************************************
* Author : cotta
* Functionality : custom flashlight gain between preview/capture flash
*******************************************************************************/
//                                                                            
#define FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X 30
MUINT32 custom_GetFlashlightGain10X(void)
{
    // x10 , 1 mean 0.1x gain
    //10 means no difference. use torch mode for preflash and cpaflash
    //> 10 means capture flashlight is lighter than preflash light. < 10 is opposite condition.

    return (MUINT32)FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X;
}

MUINT32 custom_BurstFlashlightGain10X(void)
{
    return (MUINT32)FLASHLIGHT_CALI_LED_GAIN_PRV_TO_CAP_10X;
}

MUINT32 custom_getFlashLevels(int* torchLevel, int* preflashLevel, int* afLevel, int* captureLevel)
{
	*torchLevel = 1;
	*preflashLevel = 12;
	*afLevel = 20;
	*captureLevel = 255; //for the case if(custom_GetFlashlightGain10X != 10).  if(custom_GetFlashlightGain10X==10) capture level = preflashLevel
	return 0;
}
/*******************************************************************************
* Author : Jiale
* Functionality : custom yuv flashlight threshold
*******************************************************************************/

#define FLASHLIGHT_YUV_THRESHOlD 3.0
double custom_GetYuvFlashlightThreshold(void)
{
    return (double)FLASHLIGHT_YUV_THRESHOlD;
}
/*******************************************************************************
* Author : Jiale
* Functionality : custom yuv sensor convergence frame count
*******************************************************************************/

#define FLASHLIGHT_YUV_CONVERGENCE_FRAME 7
int custom_GetYuvFlashlightFrameCnt(void)
{
    return (int)FLASHLIGHT_YUV_CONVERGENCE_FRAME;
}

/*******************************************************************************
*
*******************************************************************************/
};  //NSCamCustom

