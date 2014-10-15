#ifndef __CAMERA_CUSTOM_FLICKER_H__
#define __CAMERA_CUSTOM_FLICKER_H__


#include "camera_custom_flicker_para.h"


void cust_getMainFlickerSensorPara(int senMode, FLICKER_CUST_PARA* p);
void cust_getSubFlickerSensorPara(int senMode, FLICKER_CUST_PARA* p);
void cust_getFlickerHalPara(int* defaultHz, int* maxDetExpUs);



#endif