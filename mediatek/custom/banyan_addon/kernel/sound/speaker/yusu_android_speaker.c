/*                                                                              
  
            
            
                         
  
           
           
         
  
               
               
                   
  
          
          
                             
  
  
                                                                                
             
             
         
  
                       
                                                                                                                              
    
  
                       
                                                                                       
    
  
                       
                                                                                                             
    
  
                       
                                                                                             
    
  
                       
                                                
    
  
                           
                                                                       
                                    
  
                                                                               */
#include "yusu_android_speaker.h"

bool Speaker_Init(void)
{
    return true;
}

bool Speaker_DeInit(void)
{
	return false;
}

void Sound_SpeakerL_SetVolLevel(int level)
{
    return;
}

void Sound_SpeakerR_SetVolLevel(int level)
{
    return;
}

void Sound_Speaker_Turnon(int channel)
{
    return;
}

void Sound_Speaker_Turnoff(int channel)
{
    return;
}

void Sound_Speaker_SetVolLevel(int level)
{
    return;
}


void Sound_Headset_Turnon(void)
{
    return;
}
void Sound_Headset_Turnoff(void)
{
    return;
}

//kernal use
void AudioAMPDevice_Suspend(void)
{
   return; 
}
void AudioAMPDevice_Resume(void)
{
   return; 
}
void AudioAMPDevice_SpeakerLouderOpen(void)
{
	return ;

}
void AudioAMPDevice_SpeakerLouderClose(void)
{
   return; 
}
void AudioAMPDevice_mute(void)
{
   return; 
}

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
	return 0;
}

