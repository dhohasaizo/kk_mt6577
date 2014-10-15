#include "aaa_param.h"
#include "camera_custom_flicker.h"
#include "camera_custom_flicker_para.h"


void cust_getMainFlickerSensorPara(int senMode, FLICKER_CUST_PARA* para)
{

  //e_sensorModePreview=0,
  //e_sensorModeVideoPreview,
  //e_sensorModeCapture,
  //e_sensorModeZsd,

  //if(senMode==e_sensorModePreview)
  //{ ... }
  //else if(senMode==e_sensorModeVideoPreview)
  //...
  if(senMode==e_sensorModeZsd || senMode==e_sensorModeCapture)
  {
  	int i;
	  int freq[9] =  { 70, 80, 90, 100, 110, 120, 130, 140, 170};
	  FLICKER_CUST_STATISTICS EV50_L50 = {-194, 4721, 381, -766};
	  FLICKER_CUST_STATISTICS EV50_L60 = {3106, 149, 279, -132};
	  FLICKER_CUST_STATISTICS EV60_L50 = {3704, 157, 416, -206};
	  FLICKER_CUST_STATISTICS EV60_L60 = {-162, 2898, 247, -642};
	  
	  for(i=0;i<9;i++)
	  {
	    para->flickerFreq[i]=freq[i];
	  }
	  para->flickerGradThreshold=28;
	  para->flickerSearchRange=40;
	  para->minPastFrames=3;
	  para->maxPastFrames=14;
	  para->EV50_L50=EV50_L50;
	  para->EV50_L60=EV50_L60;
	  para->EV60_L50=EV60_L50;
	  para->EV60_L60=EV60_L60;
	  para->EV50_thresholds[0]=-30;
	  para->EV50_thresholds[1]=8;
	  para->EV60_thresholds[0]=-30;
	  para->EV60_thresholds[1]=8;
	  para->freq_feature_index[0]=5;
	  para->freq_feature_index[1]=3;
  }
  else
  {
	  int i;
	  int freq[9] =  { 70, 80, 90, 100, 120, 130, 140, 160, 170};
	  FLICKER_CUST_STATISTICS EV50_L50 = {-194, 4721, 381, -766};
	  FLICKER_CUST_STATISTICS EV50_L60 = {1125, 413, 772, -392};
	  FLICKER_CUST_STATISTICS EV60_L50 = {1341, 435, 1149, -466};
	  FLICKER_CUST_STATISTICS EV60_L60 = {-162, 2898, 247, -642};
	
	  for(i=0;i<9;i++)
	  {
	    para->flickerFreq[i]=freq[i];
	  }
	  para->flickerGradThreshold=28;
	  para->flickerSearchRange=20;
	  para->minPastFrames=3;
	  para->maxPastFrames=14;
	  para->EV50_L50=EV50_L50;
	  para->EV50_L60=EV50_L60;
	  para->EV60_L50=EV60_L50;
	  para->EV60_L60=EV60_L60;
	  para->EV50_thresholds[0]=-30;
	  para->EV50_thresholds[1]=8;
	  para->EV60_thresholds[0]=-30;
	  para->EV60_thresholds[1]=8;
	  para->freq_feature_index[0]=4;
	  para->freq_feature_index[1]=3;
	}
	
}
void cust_getSubFlickerSensorPara(int senMode, FLICKER_CUST_PARA* para)
{
  int i;
  int freq[9] =  { 80, 90, 100, 120, 130, 140, 170, 210, 230};
  FLICKER_CUST_STATISTICS EV50_L50 = {-194, 4721, 381, -766};
  FLICKER_CUST_STATISTICS EV50_L60 = {1029, 452, 844, -415};
  FLICKER_CUST_STATISTICS EV60_L50 = {1227, 476, 1256, -489};
  FLICKER_CUST_STATISTICS EV60_L60 = {-162, 2898, 247, -642};

  for(i=0;i<9;i++)
  {
    para->flickerFreq[i]=freq[i];
  }
  para->flickerGradThreshold=29;
  para->flickerSearchRange=16;
  para->minPastFrames=3;
  para->maxPastFrames=14;
  para->EV50_L50=EV50_L50;
  para->EV50_L60=EV50_L60;
  para->EV60_L50=EV60_L50;
  para->EV60_L60=EV60_L60;
  para->EV50_thresholds[0]=-30;
  para->EV50_thresholds[1]=8;
  para->EV60_thresholds[0]=-30;
  para->EV60_thresholds[1]=8;
  para->freq_feature_index[0]=3;
  para->freq_feature_index[1]=2;
}

void cust_getFlickerHalPara(int* defaultHz, int* maxDetExpUs)
{
	*defaultHz = 50;
	*maxDetExpUs = 70000;
}