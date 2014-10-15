#include "cust_leds.h"
#include <platform/mt_pwm.h>
#include <platform/i2c.h> 
#include <platform/mt_gpio.h>

U32 lm3530_i2c_write (U8 chip, U8 *cmdBuffer, int cmdBufferLen, U8 *dataBuffer, int dataBufferLen)
{
    U32 ret_code = I2C_OK;
    U8 write_data[I2C_FIFO_SIZE];
    int transfer_len = cmdBufferLen + dataBufferLen;
    int i=0, cmdIndex=0, dataIndex=0;

    if(I2C_FIFO_SIZE < (cmdBufferLen + dataBufferLen))
    {
        printf("[lm3530_i2c_write] exceed I2C FIFO length!! \n");
        return 0;
    }

    //write_data[0] = cmd;
    //write_data[1] = writeData;

    while(cmdIndex < cmdBufferLen)
    {
        write_data[i] = cmdBuffer[cmdIndex];
        cmdIndex++;
        i++;
    }

    while(dataIndex < dataBufferLen)
    {
        write_data[i] = dataBuffer[dataIndex];
        dataIndex++;
        i++;
    }

    /* dump write_data for check */
    for( i=0 ; i < transfer_len ; i++ )
    {
        //printf("[lm3530_i2c_write] write_data[%d]=%x\n", i, write_data[i]);
    }

    ret_code = i2c_v1_write(I2C1 ,chip, write_data, transfer_len);

    //printf("[lm3530_i2c_write] Done\n");

    return ret_code;
}

/*                                                             */
#if 0 //LM3530
int Cust_SetBacklight(int level)
{
	U8 chip_slave_address = 0x70;
    U8 cmd = 0x0;
    int cmd_len = 1;
    U8 data = 0x00;
    int data_len = 1;	
    U32 result_tmp;
    int i=0;

    if(level !=0) // backlight need to turn on
    {
      i2c1_v1_init();

      mt_set_gpio_mode(GPIO222, GPIO_MODE_00);
      mt_set_gpio_mode(GPIO224, GPIO_MODE_00);

      mt_set_gpio_mode(GPIO110, GPIO_MODE_04);
      mt_set_gpio_mode(GPIO107, GPIO_MODE_04);

      mt_set_gpio_mode(GPIO106, GPIO_MODE_00); /* GPIO mode */
      mt_set_gpio_dir(GPIO106, GPIO_DIR_OUT);

      mt_set_gpio_out(GPIO106,GPIO_OUT_ONE);
      udelay(100);

      /* EN set to LOW(shutdown) -> HIGH(enable) */
      mt_set_gpio_out(GPIO106,GPIO_OUT_ZERO);
      udelay(10000);
      mt_set_gpio_out(GPIO106,GPIO_OUT_ONE);

      udelay(10);

      printf("[Cust_SetBacklight_BL] init to reset\n");
      
      cmd = 0x10;	
      data = 0x11;
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);
      
      if(result_tmp != I2C_OK)
      {
          printf("[Cust_SetBacklight_BL] Reg[%x]=0x%x\n", cmd, data);
          printf("[Cust_SetBacklight_BL] -------------------------\n");
          return result_tmp;
      }
  
      cmd = 0xA0;	
      data = 0x73;
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);
      
      if(result_tmp != I2C_OK)
      {
          printf("[Cust_SetBacklight_BL] Reg[%x]=0x%x\n", cmd, data);
          printf("[Cust_SetBacklight_BL] -------------------------\n");
          return result_tmp;
      }
  
    }
    else
    {
      // Off backlight 
      cmd = 0xA0;	
      data = 0x00;
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);      
    }
    return 1;
    
}
#else //LM3639
int Cust_SetBacklight(int level)
{
	U8 chip_slave_address = 0x72;
    U8 cmd = 0x0;
    int cmd_len = 1;
    U8 data = 0x00;
    int data_len = 1;	
    U32 result_tmp;
    int i=0;

    if(level !=0) // backlight need to turn on
    {
      //i2c1_v1_init();

      mt_set_gpio_mode(GPIO222, GPIO_MODE_00);
      mt_set_gpio_mode(GPIO224, GPIO_MODE_00);

      mt_set_gpio_mode(GPIO110, GPIO_MODE_04);
      mt_set_gpio_mode(GPIO107, GPIO_MODE_04);

      mt_set_gpio_mode(GPIO106, GPIO_MODE_00); /* GPIO mode */
      mt_set_gpio_dir(GPIO106, GPIO_DIR_OUT);

      mt_set_gpio_out(GPIO106,GPIO_OUT_ONE);
      udelay(100);

      /* EN set to LOW(shutdown) -> HIGH(enable) */
      mt_set_gpio_out(GPIO106,GPIO_OUT_ZERO);
      udelay(10000);
      mt_set_gpio_out(GPIO106,GPIO_OUT_ONE);

      udelay(10);

      printf("[Cust_SetBacklight_BL] init to reset\n");
      
      cmd = 0x04;	
      data = 0x70;
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);
      
      if(result_tmp != I2C_OK)
      {
          printf("[Cust_SetBacklight_BL] Reg[%x]=0x%x\n", cmd, data);
          printf("[Cust_SetBacklight_BL] -------------------------\n");
          return result_tmp;
      }
      cmd = 0x05;	

      //                                                                                    
      if(level == 33) { //download mode
          data = 0x05;
      } else {
      data = 0x70;
      }

      //                                                                                    
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);
      
      if(result_tmp != I2C_OK)
      {
          printf("[Cust_SetBacklight_BL] Reg[%x]=0x%x\n", cmd, data);
          printf("[Cust_SetBacklight_BL] -------------------------\n");
          return result_tmp;
      }
  
      cmd = 0x0A;	
      data = 0x19;
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);
  
    }
    else
    {
      // Off backlight 
      cmd = 0x0A;	
      data = 0x00;
      result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);      
    }
    return 1;
    
}
#endif
/*                                                              */ 

void dsv_init(void)
{
	U8 chip_slave_address = 0x7C;
	U8 cmd = 0x0;
	int cmd_len = 1;
	U8 data = 0x00;
	int data_len = 1;
	U32 result_tmp;

	cmd = 0x03;	
	data = 0x00;
	result_tmp = lm3530_i2c_write(chip_slave_address, &cmd, cmd_len, &data, data_len);  
}

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_BUTTON,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST, (int)Cust_SetBacklight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}
