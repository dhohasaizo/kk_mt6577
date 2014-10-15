#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/mt6577_gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>      // might need to get fuel gauge info
#include <linux/power_supply.h>     // might need to get fuel gauge info
#include "lge_miniabb.h"
#include <linux/xlog.h>
#include <linux/kobject.h>

#include <cust_gpio_usage.h>
#include <cust_gpio_boot.h>

#define TESTCODE 0

void charger_enable();

/*                                                                                                */
enum
{
    MINI_ABB_UNINITIALIZE_CABLE     = -1,
    MINI_ABB_NORMAL_MODE            = 0,
    MINI_ABB_FACTORY_MODE           = 1
} MINI_ABB_PTM_MODE;
static int miniabb_is_PTM = MINI_ABB_UNINITIALIZE_CABLE;
static int miniabb_high_curr = 0;

extern int Is_Not_FactoryCable_PowerOn();
/*                                                                                                */

static int miniabb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int miniabb_i2c_remove(struct i2c_client *client);

static struct i2c_client *miniabb_i2c_client = NULL;
static struct platform_driver miniabb_driver;
static struct miniabb_i2c_data *i2c_data = NULL;
static const struct i2c_device_id miniabb_i2c_id[] = {{MINIABB_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_MINIABB = {I2C_BOARD_INFO(MINIABB_DEV_NAME, 0x44)};

extern int CAMERA_HW_i2C_init(void);

struct miniabb_i2c_data {
    struct i2c_client *client;
    int blue;
    int green;
    int red;
    int keyleds;
#if TESTCODE
    int cam_avdd;
    int prox_abb;
    int cam_vio;
    int cam_vcm;
    int chg_current;
    int chg_stop;
#endif
};

static int MINIABB_CheckDevID(struct i2c_client *client)
{
    int tempvalue;

    if((!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))) {
        printk("[MINIABB] I2C check ERROR!!\n");
    }

    tempvalue = 0;
    tempvalue = i2c_smbus_read_word_data(client,0x00);

    if(tempvalue < 0){
        tempvalue = i2c_smbus_read_word_data(client,0x00);
        if(tempvalue < 0 )
        {
            printk("[MINIABB] Check ID error!!\n");
            return -1;
        }
    }
    else {
        printk("[MINIABB]Device ID = [%02x]\n",tempvalue);
    }

    return 0;
}

static void MINIABB_SetGpio(void)
{
    // i2c0 pin setting
    mt_set_gpio_mode(GPIO_I2C0_SCA_PIN, GPIO_I2C0_SCA_PIN_M_SCL);
    mt_set_gpio_mode(GPIO_I2C0_SDA_PIN, GPIO_I2C0_SDA_PIN_M_SDA);

    // miniabb_int pin setting
    mt_set_gpio_mode(GPIO_EOC_PIN, GPIO_EOC_PIN_M_EINT);

    printk("GPIO_SCL[87] Mode:%d\nGPIO_SDA[88] Mode:%d\n"
            ,mt_get_gpio_mode(87),mt_get_gpio_mode(88));

}

/////////////////////////////////////////////////////////////
/*  CAM LDO control API                                    */
/////////////////////////////////////////////////////////////
void cam_avdd_power(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("cam_avdd_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO1_VSET;  // 2.8V power
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
}
EXPORT_SYMBOL(cam_avdd_power);

void cam_dvdd_power(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("cam_dvdd_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO2_VSET;  // 1.2 or 1.8V power
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
}
EXPORT_SYMBOL(cam_dvdd_power);

void cam_vio_power(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("cam_vio_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET2);
        data |= LDO3_VSET;  //1.8V power
        i2c_smbus_write_byte_data(client,LDO_VSET2, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
}
EXPORT_SYMBOL(cam_vio_power);

void cam_vcm_power(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("cam_vcm_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET2);
        data |= LDO4_VSET; //2.8V power
        i2c_smbus_write_byte_data(client,LDO_VSET2, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
}
EXPORT_SYMBOL(cam_vcm_power);


/////////////////////////////////////////////////////////////
/*  RGB LED backlight API                                  */
/////////////////////////////////////////////////////////////
void Blue_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
}
EXPORT_SYMBOL(Blue_LED_control);

void Green_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
}
EXPORT_SYMBOL(Green_LED_control);

void Red_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
}
EXPORT_SYMBOL(Red_LED_control);

void Button_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
}
EXPORT_SYMBOL(Button_LED_control);


/////////////////////////////////////////////////////////////
/*  charging API                                           */
/////////////////////////////////////////////////////////////

int check_EOC_status()
{
    struct i2c_client *client = miniabb_i2c_client;
    int data;

    // EOC check
    data = i2c_smbus_read_word_data(client,0x06);

    printk("[Miniabb] %s : %d \n",__func__, data);

    if(data == 48) {
        return TRUE;
    }
    else {
        return FALSE;
    }

}

void charger_enable()
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;
    int i;

/*                                                                                                */
#if 1
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        if (miniabb_high_curr == 0)
        {
            for(i=0;i<3;i++)
            {
                data = CHGEN | EXPDETEN | CHG_OFF;
                i2c_smbus_write_byte_data(client,CHG_CTRL1, data);
                data = CHGEN | EXPDETEN;  // 0xC0
                i2c_smbus_write_byte_data(client,CHG_CTRL1, data);
                mdelay(2);
            }
            miniabb_high_curr = 1;
        }
        else
        {
            data = CHGEN | EXPDETEN;  //0xC0
            i2c_smbus_write_byte_data(client,CHG_CTRL1, data);
            miniabb_high_curr = 1;
        }
    }
#else
    //charger enable

    if(miniabb_is_PTM)
    {
        data = 0xE0; //|= CHGEN|EXPDETEN|PTM;// factory mode setting
    } else {
        data = 0xC0; //|= CHGEN|EXPDETEN;
    }

    i2c_smbus_write_byte_data(client,CHG_CTRL1, data);
#endif
/*                                                                                                */
}

//                                                                               
int is_charging_enable(void)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;
    
    // CHGEN check
    data = i2c_smbus_read_word_data(client,CHG_CTRL1);
    
    printk("[Miniabb] %s : %x \n",__func__, data);
    
    if((data & CHGEN)== CHGEN) {
        return TRUE;
    }
    else {
        return FALSE;
    }
}
//                                                                               
#if 1
void set_charger_start_mode(CHG_TYPE chg_type)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        /* kisung.song : current/EOC LEVEL setting, 4.35V setting (CHG_VSET)*/
        switch(chg_type) {
            case CHG_TA :
                data = CHG_CURR_700mA | CHG_VSET | EOC_LVL_20PER;
                break;
            case CHG_600 :
                data = CHG_CURR_600mA | CHG_VSET | EOC_LVL_10PER;
                break;
            case CHG_500 :
                data = CHG_CURR_500mA | CHG_VSET | EOC_LVL_10PER;
                break;
            case CHG_USB :
                data = CHG_CURR_400mA | CHG_VSET | EOC_LVL_33PER;
                break;
            case CHG_100 :
                data = CHG_CURR_100mA | CHG_VSET | EOC_LVL_10PER;
                break;
            case CHG_90 :
                data = CHG_CURR_90mA | CHG_VSET | EOC_LVL_10PER;
                break;
            default :
                data = CHG_CURR_400mA | CHG_VSET | EOC_LVL_33PER;
                break;
        }

        i2c_smbus_write_byte_data(client,CHG_CTRL2, data);

        charger_enable();
    }
    printk("[Miniabb] Charging Current %dmA\n",chg_type);
/*                                                                                                */

}
#else
void set_charger_start_mode(int value)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        /* kisung.song : current/EOC LEVEL setting, 4.35V setting (CHG_VSET)*/
        if(value == CHG_TA) {
            data = 0x63|CHG_VSET; //CHG_CURR_700mA|EOC_LVL_20PER;
        }
        else {
            data = 0x25|CHG_VSET; //CHG_CURR_400mA|EOC_LVL_33PER;
        }
        i2c_smbus_write_byte_data(client,CHG_CTRL2, data);

        charger_enable();
    }
    printk("[Miniabb] Charging Current %dmA\n",value);
/*                                                                                                */

}
#endif
EXPORT_SYMBOL(set_charger_start_mode);

void set_charger_factory_mode()
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
#if 0
    /* kisung.song : 4.35V setting (CHG_VSET)  */
    data = 0x91|CHG_VSET; //CHG_CURR_1000mA|EOC_LVL_10PER;
    i2c_smbus_write_byte_data(client,CHG_CTRL2, data);
    data = i2c_smbus_read_word_data(client,CHG_CTRL2);

    miniabb_is_PTM = 1;

    charger_enable();
#endif
/*                                                                                                */

    printk("[Miniabb] FACTORY charging :  0x%02x \n",data);
}
EXPORT_SYMBOL(set_charger_factory_mode);


void set_charger_stop_mode()
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        i2c_smbus_write_byte_data(client,CHG_CTRL1, 0x40);
    }
/*                                                                                                */

    miniabb_high_curr = 0;

    printk("[Miniabb]charging off!\n");

}
EXPORT_SYMBOL(set_charger_stop_mode);

/******************************************************************************/

static ssize_t show_ledblue_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->blue);
}

static ssize_t store_ledblue_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Blue_LED_control(ON);
    } else {
        Blue_LED_control(OFF);
    }
    i2c_data->blue = onoff;

    return count;
}

static ssize_t show_ledgreen_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->green);
}

static ssize_t store_ledgreen_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Green_LED_control(ON);
    } else {
        Green_LED_control(OFF);
    }
    i2c_data->green = onoff;

    return count;
}

static ssize_t show_ledred_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->red);
}

static ssize_t store_ledred_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Red_LED_control(ON);
    } else {
        Red_LED_control(OFF);
    }
    i2c_data->red = onoff;

    return count;
}

static ssize_t show_keyleds_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->keyleds);
}

static ssize_t store_keyleds_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Button_LED_control(ON);
    } else {
        Button_LED_control(OFF);
    }
    i2c_data->keyleds = onoff;

    return count;
}

#if TESTCODE
static ssize_t show_cam_avdd_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->cam_avdd);
}

static ssize_t store_cam_avdd_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        cam_avdd_power(ON);
        printk("cam_avdd power on!\n");
    } else {
        cam_avdd_power(OFF);
        printk("cam_avdd power off!\n");
    }
    i2c_data->cam_avdd = onoff;

    return count;
}

static ssize_t show_prox_abb_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->prox_abb);
}

static ssize_t store_prox_abb_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        proxi_power(ON);
        printk("proxi power on!\n");
    } else {
        proxi_power(OFF);
        printk("proxi power off!\n");
    }
    i2c_data->prox_abb = onoff;

    return count;
}


static ssize_t show_cam_vio_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->cam_vio);
}

static ssize_t store_cam_vio_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        cam_vio_power(ON);
        printk("cam_vio power on!\n");
    } else {
        cam_vio_power(OFF);
        printk("cam_vio power off!\n");
    }
    i2c_data->cam_vio = onoff;

    return count;
}


static ssize_t show_cam_vcm_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->cam_vcm);
}

static ssize_t store_cam_vcm_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        cam_vcm_power(ON);
        printk("cam_vcm power on!\n");
    } else {
        cam_vcm_power(OFF);
        printk("cam_vcm power off!\n");
    }
    i2c_data->cam_vcm = onoff;

    return count;
}

static ssize_t show_chg_start_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->chg_current);
}

static ssize_t store_chg_start_value(struct device_driver *ddri, char *buf, size_t count)
{
    int chg_curr;

    sscanf(buf, "%d", &chg_curr);

    if(chg_curr == CHG_USB ) {
        set_charger_start_mode(CHG_USB);
    } else {
        set_charger_start_mode(CHG_TA);
    }
    i2c_data->chg_current = chg_curr;

    return count;
}

static ssize_t show_chg_stop_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->chg_stop);
}


static ssize_t store_chg_stop_value(struct device_driver *ddri, char *buf, size_t count)
{
    int chg_stop;

    sscanf(buf, "%d", &chg_stop);

    if(chg_stop) {
        set_charger_stop_mode();
    }
    i2c_data->chg_stop = chg_stop;

    return count;
}

#endif


/******************************************************************************/
static DRIVER_ATTR(ledblue,    S_IWUSR | S_IRUGO | S_IWGRP, show_ledblue_value, store_ledblue_value);
static DRIVER_ATTR(ledgreen,    S_IWUSR | S_IRUGO | S_IWGRP, show_ledgreen_value, store_ledgreen_value);
static DRIVER_ATTR(ledred,    S_IWUSR | S_IRUGO | S_IWGRP, show_ledred_value, store_ledred_value);
static DRIVER_ATTR(keyleds,    S_IWUSR | S_IRUGO | S_IWGRP, show_keyleds_value, store_keyleds_value);
#if TESTCODE
static DRIVER_ATTR(cam_avdd,    S_IWUSR | S_IRUGO | S_IWGRP, show_cam_avdd_value, store_cam_avdd_value);
static DRIVER_ATTR(prox_abb,    S_IWUSR | S_IRUGO | S_IWGRP, show_prox_abb_value, store_prox_abb_value);
static DRIVER_ATTR(cam_vio,    S_IWUSR | S_IRUGO | S_IWGRP, show_cam_vio_value, store_cam_vio_value);
static DRIVER_ATTR(cam_vcm,    S_IWUSR | S_IRUGO | S_IWGRP, show_cam_vcm_value, store_cam_vcm_value);
static DRIVER_ATTR(chg_start,    S_IWUSR | S_IRUGO | S_IWGRP, show_chg_start_value, store_chg_start_value);
static DRIVER_ATTR(chg_stop,    S_IWUSR | S_IRUGO | S_IWGRP, show_chg_stop_value, store_chg_stop_value);
#endif

static struct driver_attribute *miniabb_attr_list[] = {
    &driver_attr_ledblue,   /* blue led control */
    &driver_attr_ledgreen,   /* blue led control */
    &driver_attr_ledred,   /* blue led control */
    &driver_attr_keyleds,   /* button led control */
#if TESTCODE
    &driver_attr_cam_avdd,   /* +2V8_CAM_AVDD_MTK_S */
    &driver_attr_prox_abb,   /* +3V0_PROX_ABB */
    &driver_attr_cam_vio,   /* +1V8_CAM_VIO_MTK_S */
    &driver_attr_cam_vcm,   /* +2V8_CAM_VCM_S */
    &driver_attr_chg_start,   /* start charger */
    &driver_attr_chg_stop,   /* stop charger */
#endif
};

/***********************************************************************************/
static int miniabb_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(miniabb_attr_list)/sizeof(miniabb_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, miniabb_attr_list[idx]))
        {
            printk("driver_create_file (%s) = %d\n", miniabb_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}

/***********************************************************************************/
static int miniabb_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(miniabb_attr_list)/sizeof(miniabb_attr_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }


    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, miniabb_attr_list[idx]);
    }


    return err;
}

/***********************************************************************************/
static int miniabb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {

    printk("[Miniabb] :: miniabb i2c probe \n");

    struct i2c_client *new_client;
    struct miniabb_i2c_data *obj;
    u8 databuf[2];

    int err = 0;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        miniabb_i2c_client = NULL;
        return -ENOMEM;
    }

    memset(obj, 0, sizeof(struct miniabb_i2c_data));

    i2c_data = obj;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client,obj);

    miniabb_i2c_client = client;

    //printk("miniabb_i2c_client number = 0x%x.\n", miniabb_i2c_client);

    if (MINIABB_CheckDevID(client) < 0)
    {
        obj = i2c_get_clientdata(new_client);
        kfree(obj);
        miniabb_i2c_client = NULL;
        return -1;
    }

    if(err = miniabb_create_attr(&miniabb_driver.driver))
    {
        printk("miniabb create attribute err = %d\n", err);
    }

/*                                                                                                */
    if (Is_Not_FactoryCable_PowerOn() == 0)
    {
        miniabb_is_PTM = MINI_ABB_FACTORY_MODE;
    }
    else

    {
        miniabb_is_PTM = MINI_ABB_NORMAL_MODE;
    }
/*                                                                                                */

    return 0;

}

static int miniabb_i2c_remove(struct i2c_client *client)
{
    int err;

    if(err = miniabb_delete_attr(&miniabb_driver.driver))
    {
        printk("miniabb delete attribute err = %d\n", err);
    }

    return 0;
}


static int miniabb_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    printk("[Miniabb] ::  miniabb_i2c_detect\n");
    strcpy(info->type, MINIABB_DEV_NAME);
    return 0;
}


struct i2c_driver miniabb_i2c_driver = {
    .probe = miniabb_i2c_probe,
    .remove = miniabb_i2c_remove,
    .detect = miniabb_i2c_detect,
    .driver.name = MINIABB_DEV_NAME,
    .id_table = miniabb_i2c_id,
};



static int miniabb_probe(struct platform_device *dev)
{

    printk("[Miniabb] :: charging IC Initialization is done\n");

    MINIABB_SetGpio();

    if(i2c_add_driver(&miniabb_i2c_driver))
    {
        printk("[Miniabb] :: add miniabb i2c driver error !\n");
        return -1;
    }

    return 0;
}

static int miniabb_remove(struct platform_device *dev)
{
    //
    i2c_del_driver(&miniabb_i2c_driver);
    return 0;
}
/*
static int miniabb_suspend(struct platform_device *dev)
{
    return 0;
}

static int miniabb_resume(struct platform_device *dev)
{
    return 0;
}
*/


static struct platform_driver miniabb_driver = {
    .probe = miniabb_probe,
    .remove = miniabb_remove,
//    .suspend = miniabb_suspend,
//    .resume = miniabb_resume,
    .driver = {
        .name = "miniabb",
        .owner = THIS_MODULE,
    },
};

static int __init miniabb_driver_init(void) {
    int ret ;

    printk("MiniABB driver init!!\n");

    ret = i2c_register_board_info(0, &i2c_MINIABB, 1);
    if(ret)
    {
        printk("failed to i2c register driver. (%d) \n",ret);
        return ret;
    } else {
        printk("success to i2c register driver. (%d) \n",ret);
    }
    ret = platform_driver_register(&miniabb_driver);
    if(ret)
    {
        printk("failed to register driver. (%d) \n",ret);
        return ret;
    } else {
        printk("success to register driver. (%d) \n",ret);
    }

    /*In order to prevent sequence that MINI_ABB doesn't be
      initialized and sensor try to start power on sequence for initialization.*/
    CAMERA_HW_i2C_init();

    return ret;
}

static void __exit miniabb_driver_exit(void) {
    printk("MiniABB driver exit!!\n");
    platform_driver_unregister(&miniabb_driver);
}

module_init(miniabb_driver_init);
module_exit(miniabb_driver_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("miniabb Driver");
MODULE_LICENSE("GPL");
