#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <mach/eint.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <cust_eint.h>
#include <linux/jiffies.h>

#include "tpd.h"


#include <linux/spinlock.h>
#include <mach/mt_wdt.h>
#include <mach/mt_gpt.h>
#include <mach/mt_reg_base.h>

#include "mms100_ISP_download.h"
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>

#include <linux/wakelock.h>


#define TP_DEV_NAME "mms134"
#define I2C_RETRY_CNT 5 //Fixed value
#define DOWNLOAD_RETRY_CNT 5 //Fixed value
#define MELFAS_DOWNLOAD 1 //Fixed value

#define PRESS_KEY 1 //Fixed value
#define RELEASE_KEY 0 //Fixed value

#define TS_READ_LEN_ADDR 0x0F //Fixed value
#define TS_READ_START_ADDR 0x10 //Fixed value
#define TS_READ_REGS_LEN 66 //Fixed value
#define TS_WRITE_REGS_LEN 16 //Fixed value

#define TS_MAX_TOUCH 	10 //Model Dependent

#define TS_READ_HW_VER_ADDR 0xF1 //Model Dependent
#define TS_READ_SW_VER_ADDR 0xF0 //Model Dependent
#define MELFAS_FW_VERSION   0xF3 //Model Dependent

#ifdef LGE_USE_DOME_KEY
//vee5ds_open_br   : 4 key
//vee5nfc_open_eu  : 2 key(domekey)
//vee5ss_tcl_mx     : 2 key(domekey)
//vee5ss_viv_br     : 2 key(domekey)
#define MELFAS_CURRENT_FW_VERSION 0x2
#define MELFAS_CURRENT_FW_VERSION_R50 0x14 //0x13//0xAB //0xAA //0x11 //0x9 //0x8 // 5
#define MELFAS_CURRENT_FW_VERSION_R51 0x15 //0x16 //0x15
#else
#define MELFAS_CURRENT_FW_VERSION 0x7
#define MELFAS_CURRENT_FW_VERSION_R50 0x15 //0x14//0x13 //0x12 //0x11 // 8
#define MELFAS_CURRENT_FW_VERSION_R51 0x16
#endif

#define MELFAS_HW_REVISON 0x01 //Model Dependent

#define MELFAS_MAX_TRANSACTION_LENGTH 66
#define MELFAS_MAX_I2C_TRANSFER_SIZE  7
#define MELFAS_I2C_DEVICE_ADDRESS_LEN 1
//#define I2C_MASTER_CLOCK       400
#define MELFAS_I2C_MASTER_CLOCK       100
#define MELFAS_I2C_ADDRESS   0x48


#define GPT_IRQEN       (APMCU_GPTIMER_BASE + 0x0000)



#define REPORT_MT_DOWN(touch_number, x, y, width, strength) \
do {     \
	input_report_abs(tpd->dev, ABS_PRESSURE, strength);  \
    input_report_key(tpd->dev, BTN_TOUCH, 1);   \
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, width);         \
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, strength); \
	input_mt_sync(tpd->dev);                                      \
} while (0)

#define REPORT_MT_UP(touch_number, x, y, width, strength) \
do {     \
	input_report_abs(tpd->dev, ABS_PRESSURE, strength);  \
    input_report_key(tpd->dev, BTN_TOUCH, 0);   \
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, width);         \
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, strength); \
	input_mt_sync(tpd->dev);                                      \
} while (0)


static int melfas_tpd_flag = 0;
unsigned char  touch_fw_version = 0;

#define TPD_HAVE_BUTTON 

enum {
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info {
	int strength;
//	int width
	int area;
	int posX;
	int posY;
	int status;
	int pressure;
};

static struct muti_touch_info g_Mtouch_info[TS_MAX_TOUCH];

/*                                                              
                        
                        
                         
                         
*/
#ifdef LGE_USE_DOME_KEY
#define TPD_KEY_COUNT	2
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK , KEY_MENU};
#else
#define TPD_KEY_COUNT	4
static int tpd_keys_local[TPD_KEY_COUNT] = {KEY_BACK ,KEY_HOMEPAGE, KEY_MENU, KEY_SSK};
#endif
/*                                                              */
#ifdef CONFIG_LGE_AAT_KEY
static int aat_keys_local[TPD_KEY_COUNT] = {KEY_F9, KEY_F11};
#endif

/*                                                                             */
struct delayed_work ghost_monitor_work;
static int ghost_touch_cnt = 0;
static int ghost_x = 1000;
static int ghost_y = 1000;
/*                                                                             */

/*                                                                                           */
static int before_touch_time = 0;
static int current_key_time = 0;
static int is_touch_pressed = 0;
/*                                                                                           */

static int is_key_pressed = 0;
static int pressed_keycode = 0;
#define CANCEL_KEY 0xff


static DECLARE_WAIT_QUEUE_HEAD(melfas_waiter);

struct i2c_client *melfas_i2c_client = NULL;

static const struct i2c_device_id melfas_tpd_id[] = {{TP_DEV_NAME,0},{}};
static struct i2c_board_info __initdata melfas_i2c_tpd={ I2C_BOARD_INFO(TP_DEV_NAME, MELFAS_I2C_ADDRESS)};

static int melfas_tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int melfas_tpd_i2c_remove(struct i2c_client *client);
static int melfas_tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

extern int mms100_ISC_download_binary_data(int hw_ver);
int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 len, u8 *rxbuf);

static void melfas_ts_release_all_finger(void);
static int melfas_firmware_update(struct i2c_client *client);
static int melfas_ts_fw_load(struct i2c_client *client, int hw_ver);


extern struct tpd_device *tpd;



static struct i2c_driver melfas_tpd_i2c_driver =
{                       
    .probe = melfas_tpd_i2c_probe,                                   
    .remove = __devexit_p(melfas_tpd_i2c_remove),                           
    .detect = melfas_tpd_i2c_detect,                           
    .driver.name = "mtk-tpd", 
    .id_table = melfas_tpd_id,                             
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = melfas_ts_suspend, 
    .resume = melfas_ts_resume,
#endif
    
}; 

#define VAR_CHAR_NUM_MAX      20

#define GN_MTK_BSP

#if defined (GN_MTK_BSP)
static ssize_t show_update_pro(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
//   char ver[20];
//	snprintf(buf, VAR_CHAR_NUM_MAX, "%s", ver);
	return 0;
}
static ssize_t store_update_pro(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int hw_ver;
	sscanf(buf, "%d", &hw_ver);

	cancel_delayed_work(&ghost_monitor_work);
	melfas_ts_fw_load(melfas_i2c_client, hw_ver);

	schedule_delayed_work(&ghost_monitor_work, msecs_to_jiffies(HZ * 50));
    return count;        
}

#if 1
static DEVICE_ATTR(update, 0664, show_update_pro, store_update_pro);
#else
static struct kobj_attribute update_firmware_attribute = {
	.attr = {.name = "update", .mode = 0664},
	.show = show_update_pro,
	.store = store_update_pro,
};
static int tpd_create_attr(void) 
{
	int ret;
	struct kobject *kobject_ts;
	kobject_ts = kobject_create_and_add("touch_screen", NULL);
	if (!kobject_ts)
	{
		printk("create kobjetct error!\n");
		return -1;
	}
	
	ret = sysfs_create_file(kobject_ts, &update_firmware_attribute.attr);
	if (ret) {
		kobject_put(kobject_ts);
		printk("create file error\n");
		return -1;
	}
	return 0;	

}
#endif
#endif

void tpd_hw_enable(void)
{
    hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_3000, "TP");//TOUCH  LDO
    msleep(10);
}

void tpd_hw_disable(void)
{
    hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");//TOUCH  LDO
}

static void esd_rest_tp(void)
{
	printk("==========tp have inter esd =============\n");
    melfas_ts_release_all_finger();
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	tpd_hw_disable();
	msleep(100);
	tpd_hw_enable();
	msleep(100);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}


static int melfas_touch_event_handler(void *unused)
{
	uint8_t buf[TS_READ_REGS_LEN] = { 0 };
	int i, read_num, fingerID, Touch_Type = 0, touchState = 0;
	int keyID = 0, reportID = 0;

	int press_count = 0;
	int is_touch_mix = 0;

    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 

    sched_setscheduler(current, SCHED_RR, &param); 

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
		
        wait_event_interruptible(melfas_waiter, melfas_tpd_flag != 0);
		
        melfas_tpd_flag = 0;
        set_current_state(TASK_RUNNING); 

		melfas_i2c_read(melfas_i2c_client, TS_READ_LEN_ADDR, 1, buf);
		read_num = buf[0];

//                                                                                        
        if(read_num >= TS_READ_REGS_LEN) {
            read_num = TS_READ_REGS_LEN-1;
        }
//                                             

		if(read_num)
		{
			melfas_i2c_read(melfas_i2c_client, TS_READ_START_ADDR, read_num, buf);
        	
          	if(0x0F == buf[0])
           	{
            	esd_rest_tp();
             	continue;
		 	}

			for (i = 0; i < read_num; i = i + 6)
            {
                Touch_Type = (buf[i] >> 5) & 0x03;
				fingerID = (buf[i] & 0x0F) - 1;
				touchState = ((buf[i] & 0x80)==0x80);
				reportID = (buf[i] & 0x0F);
				keyID = reportID;

                /* touch type is panel */
				if (Touch_Type == TOUCH_SCREEN)
                {
                    g_Mtouch_info[fingerID].posX = (uint16_t)(buf[i + 1] & 0x0F) << 8 | buf[i + 2];
                    g_Mtouch_info[fingerID].posY = (uint16_t)(buf[i + 1] & 0xF0) << 4 | buf[i + 3];
                    g_Mtouch_info[fingerID].area = buf[i + 4];

					g_Mtouch_info[fingerID].status = touchState;

                    if (touchState)
						 g_Mtouch_info[fingerID].pressure = buf[i + 5];
						 //g_Mtouch_info[fingerID].pressure = 10;
                        //g_Mtouch_info[fingerID].pressure = buf[i + 5];
                    else {
                        g_Mtouch_info[fingerID].pressure = 0;
						
						/*                                                                             */
						if(ghost_touch_cnt == 0)
						{
							ghost_x = g_Mtouch_info[fingerID].posX;
							ghost_y = g_Mtouch_info[fingerID].posY;
							ghost_touch_cnt++;
						}
						else
						{
							if(ghost_x + 40 >= g_Mtouch_info[fingerID].posX && ghost_x - 40 <= g_Mtouch_info[fingerID].posX)
								if(ghost_y + 40 >= g_Mtouch_info[fingerID].posY && ghost_y - 40 <= g_Mtouch_info[fingerID].posY)
									ghost_touch_cnt++;
						}
						/*                                                                             */
                   }

					if(is_key_pressed == PRESS_KEY)
					{
						//printk(" ++++++++ KEY_CANCEL!!!!!!!!\n\n");
						input_report_key(tpd->dev, pressed_keycode, CANCEL_KEY);
						input_sync(tpd->dev);
						is_key_pressed = CANCEL_KEY;
					}
					
					is_touch_mix = 1;
                }
				else if (Touch_Type == TOUCH_KEY)
				{

					current_key_time = jiffies_to_msecs(jiffies);
					if(before_touch_time > 0)
					{
						if(current_key_time - before_touch_time > 150)// 100)
						{
							is_touch_pressed = 0;
						}
						else
						{
							continue;
						}
					}
					
					before_touch_time = 0;
					current_key_time = 0;
					// Ignore Key event during touch event actioned
					if(is_touch_mix || is_touch_pressed)
					{
						continue;
					}

					printk(KERN_DEBUG  "[MELFAS] Touch Key pressed: keyID = 0x%x\n", keyID);
					if (keyID == 0x1)
						input_report_key(tpd->dev, tpd_keys_local[keyID-1], touchState ? PRESS_KEY : RELEASE_KEY);
					else if (keyID == 0x2)
						input_report_key(tpd->dev, tpd_keys_local[keyID-1], touchState ? PRESS_KEY : RELEASE_KEY);
					else if (keyID == 0x3)
						input_report_key(tpd->dev, tpd_keys_local[keyID-1], touchState ? PRESS_KEY : RELEASE_KEY);
					else if (keyID == 0x4)
						input_report_key(tpd->dev, tpd_keys_local[keyID-1], touchState ? PRESS_KEY : RELEASE_KEY);
					else {
						printk("  Error keyID is %d!!!!!!!!!!\n", keyID);
						keyID = 0x00;
					}

					if (keyID != 0) {
						pressed_keycode = tpd_keys_local[keyID-1];
						
						if (touchState)
							is_key_pressed = PRESS_KEY;
						else
							is_key_pressed = RELEASE_KEY;
					}
				}
            }

			press_count = 0;
			if(is_touch_mix)
			{		
				for (i = 0; i < TS_MAX_TOUCH; i++)
				{
					if (g_Mtouch_info[i].pressure == -1)
						continue;

					if (g_Mtouch_info[i].status == 0)
					{
						is_touch_pressed = 0;
						g_Mtouch_info[i].status = -1;
						continue;
					}
					if(g_Mtouch_info[i].status == 1)
					{
						input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, i);
						input_report_abs(tpd->dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
						input_report_abs(tpd->dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
						input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].area);
						//input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].pressure);
						input_report_abs(tpd->dev, ABS_MT_PRESSURE, g_Mtouch_info[i].pressure);
		
						is_touch_pressed = 1;
						input_mt_sync(tpd->dev);
						press_count++;
					}
					if (g_Mtouch_info[i].pressure == 0)
						g_Mtouch_info[i].pressure = -1;
				}
				
				if(press_count == 0) 
					input_mt_sync(tpd->dev);

				before_touch_time = jiffies_to_msecs(jiffies);
			}
			is_touch_mix = 0;
			input_sync(tpd->dev);
		}
    } while ( !kthread_should_stop() ); 

    return 0;
}

static void melfas_i2c_tpd_eint_interrupt_handler(void)
{ 
    TPD_DEBUG_PRINT_INT;
    melfas_tpd_flag=1;
    wake_up_interruptible(&melfas_waiter);
} 

int melfas_i2c_write_bytes( struct i2c_client *client, u16 addr, int len, u8 *txbuf )
{
    u8 buffer[MELFAS_MAX_TRANSACTION_LENGTH]={0};
    u16 left = len;
    u8 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg = 
    {
        .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
        .flags = 0,
        .buf = buffer,
        .timing = MELFAS_I2C_MASTER_CLOCK,
    };


    if ( txbuf == NULL )
        return -1;

    TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        retry = 0;

        buffer[0] = (u8)addr+offset;

        if ( left > MELFAS_MAX_I2C_TRANSFER_SIZE )
        {
            memcpy( &buffer[MELFAS_I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MELFAS_MAX_I2C_TRANSFER_SIZE );
            msg.len = MELFAS_MAX_TRANSACTION_LENGTH;
            left -= MELFAS_MAX_I2C_TRANSFER_SIZE;
            offset += MELFAS_MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy( &buffer[MELFAS_I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
            msg.len = left + MELFAS_I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

        TPD_DEBUG("byte left %d offset %d\n", left, offset );

        while ( i2c_transfer( client->adapter, &msg, 1 ) != 1 )
        {
            retry++;
            if ( retry == I2C_RETRY_CNT )
            {
                TPD_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                TPD_DMESG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
            else
                 TPD_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);
        }
    }

    return 0;
}

int melfas_i2c_write_data(struct i2c_client *client, int len, u8 *txbuf)
{
    u16 ret = 0;

	client->addr = client->addr&I2C_MASK_FLAG;

    if ( txbuf == NULL )
        return -1;

	ret = i2c_master_send(client, txbuf, len);

	if (ret != len)
		return -1;

    return 0;
}

int melfas_i2c_read_data(struct i2c_client *client, int len, u8 *rxbuf)
{
    u16 ret = 0;

	client->addr = client->addr&I2C_MASK_FLAG;

    if ( rxbuf == NULL )
        return -1;

	ret = i2c_master_recv(client, rxbuf, len);

	if (ret != len)
		return -1;

    return 0;
}

int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 len, u8 *rxbuf)
{
    u8 buffer[MELFAS_I2C_DEVICE_ADDRESS_LEN]={0};
    u8 retry;
    u16 left = len;
    u8 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
            .flags = 0,
            .buf = buffer,
            .len = MELFAS_I2C_DEVICE_ADDRESS_LEN,
            .timing = MELFAS_I2C_MASTER_CLOCK
        },
        {
            .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
            .flags = I2C_M_RD,
            .timing = MELFAS_I2C_MASTER_CLOCK
        },
    };

    if ( rxbuf == NULL )
        return -1;

    TPD_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        buffer[0] = (u8)addr+offset;

        msg[1].buf = &rxbuf[offset];

        if ( left > MELFAS_MAX_TRANSACTION_LENGTH )
        {
            msg[1].len = MELFAS_MAX_TRANSACTION_LENGTH;
            left -= MELFAS_MAX_TRANSACTION_LENGTH;
            offset += MELFAS_MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while ( i2c_transfer( client->adapter, &msg[0], 2 ) != 2 )
        {
            retry++;

            if ( retry == I2C_RETRY_CNT )
            {
                TPD_DEBUG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}

static DEFINE_SPINLOCK(touch_reg_operation_spinlock);

static unsigned short g_wdt_original_data;

/* This function will disable watch dog */
void mtk_touch_wdt_disable(void)
{
	unsigned short tmp;

	spin_lock(&touch_reg_operation_spinlock);

	tmp = DRV_Reg16(MTK_WDT_MODE);
	g_wdt_original_data = tmp;
	tmp |= MTK_WDT_MODE_KEY;
	tmp &= ~MTK_WDT_MODE_ENABLE;

	DRV_WriteReg16(MTK_WDT_MODE,tmp);

	spin_unlock(&touch_reg_operation_spinlock);
}

void mtk_touch_wdt_restart(void)
        {
	unsigned short tmp;

    // Reset WatchDogTimer's counting value to time out value
    // ie., keepalive()

//    DRV_WriteReg16(MTK_WDT_RESTART, MTK_WDT_RESTART_KEY);

	spin_lock(&touch_reg_operation_spinlock);

	DRV_WriteReg16(MTK_WDT_MODE,g_wdt_original_data);
	tmp = DRV_Reg16(MTK_WDT_MODE);

	spin_unlock(&touch_reg_operation_spinlock);
	
        }

static DEFINE_SPINLOCK(touch_gpt_lock);
static unsigned long touch_gpt_flags;

const UINT32 touch_gpt_mask[GPT_TOTAL_COUNT+1] = {
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0
};

void TOUCH_GPT_DisableIRQ(GPT_NUM numGPT)
            {
    if (numGPT == GPT1 || numGPT == GPT2)
        return;

    spin_lock_irqsave(&touch_gpt_lock, touch_gpt_flags);
    
    DRV_ClrReg32(GPT_IRQEN, touch_gpt_mask[numGPT]);

    spin_unlock_irqrestore(&touch_gpt_lock, touch_gpt_flags);
}

void TOUCH_GPT_EnableIRQ(GPT_NUM numGPT)
{
    if (numGPT == GPT1 || numGPT == GPT2)
        return;
 
    spin_lock_irqsave(&touch_gpt_lock, touch_gpt_flags);

    DRV_SetReg32(GPT_IRQEN, touch_gpt_mask[numGPT]);

    spin_unlock_irqrestore(&touch_gpt_lock, touch_gpt_flags);
}


static int melfas_ts_fw_load(struct i2c_client *client, int hw_ver)
{
    int ret = 0;
    
    printk("[TSP] %s: \n", __func__);

   	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mtk_touch_wdt_disable();
	TOUCH_GPT_DisableIRQ(GPT5);

	ret = mms100_ISC_download_binary_data(hw_ver);
	if (ret)
		printk("<MELFAS> SET Download ISP Fail\n");
	
	TOUCH_GPT_EnableIRQ(GPT5);
	mtk_touch_wdt_restart();

	tpd_hw_disable();
	msleep(100);
	tpd_hw_enable();
	msleep(200);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    return ret;
}

static int melfas_check_firmware(struct i2c_client *client, u8 *val)
{
    int ret = 0;
   
    ret = melfas_i2c_read(client, TS_READ_HW_VER_ADDR, 1, &val[0]);
	if (ret != 0)
	{
		return ret;
	}
    ret = melfas_i2c_read(client, MELFAS_FW_VERSION, 1, &val[1]);

    printk("[melfas_tpd]:HW Version[0x%02x] SW Version[0x%02x] \n", val[0], val[1]);
	
	return ret;
}

static int melfas_firmware_update(struct i2c_client *client)
{
    int ret = 0;
    uint8_t fw_ver[2] = {0, };
	
    ret = melfas_check_firmware(client, fw_ver);
    if (ret < 0)
	{
		printk("[melfas_tpd] check_firmware fail! [%d]", ret);
		tpd_hw_disable();
		msleep(100);
		tpd_hw_enable();
		msleep(100);
		return -1;
	}
    else
    {
#ifdef LGE_USE_DOME_KEY
		if (fw_ver[1] == 0xAB)
			ret = melfas_ts_fw_load(client, 1);
#endif

		if ((fw_ver[0] == 0x71) || (fw_ver[0] == 0x51)) { //HW version (R0x71 or R0x51)
			if (fw_ver[1] < MELFAS_CURRENT_FW_VERSION_R51) {
				ret = melfas_ts_fw_load(client, 2);
			}
		} else if ((fw_ver[0] == 0x70) || (fw_ver[0] == 0x50)) {
			if (fw_ver[1] < MELFAS_CURRENT_FW_VERSION_R50) {
				ret = melfas_ts_fw_load(client, 1);
			}
		}
    }

    return ret;
}

static ssize_t melfas_FW_show(struct device *dev,  struct device_attribute *attr,  char *buf)
{
	int r;
	u8 product_id;
	u8 product_id2;

	r = snprintf(buf, PAGE_SIZE, "%d\n", touch_fw_version);
	
	melfas_i2c_read(melfas_i2c_client, MELFAS_FW_VERSION, sizeof(product_id), &product_id);
	melfas_i2c_read(melfas_i2c_client, TS_READ_HW_VER_ADDR, sizeof(product_id2), &product_id2);

	return sprintf(buf, "%u : FW_VERSION, 0x%02x : HW_VERSION \n", product_id, product_id2);
}

static DEVICE_ATTR(fw, 0664, melfas_FW_show, NULL);


static ssize_t show_melfas_reset(struct device *dev,  struct device_attribute *attr,  char *buf)
{
	tpd_hw_disable();
	msleep(100);
	tpd_hw_enable();
	msleep(100);
	
	return 0;
}

/*read dummy interrupts.*/
static ssize_t store_melfas_reset(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer100, size_t count)
{
	uint8_t buf[TS_READ_REGS_LEN] = {0,};
	int read_num = 0;

	melfas_i2c_read(melfas_i2c_client, TS_READ_LEN_ADDR, 1, buf);
	read_num = buf[0];
	if(read_num)
	{
		melfas_i2c_read(melfas_i2c_client, TS_READ_START_ADDR, read_num, buf);

		if(0x0F == buf[0])
			esd_rest_tp();
	}
		
	return count;        
}
static DEVICE_ATTR(reset, 0664, show_melfas_reset, store_melfas_reset);

/*                                    */
void touch_keylock_enable(int key_lock)
{
	if(!key_lock)
	{
		__set_bit(ABS_X, tpd->dev->absbit);
		__set_bit(ABS_Y, tpd->dev->absbit);
		__set_bit(ABS_PRESSURE, tpd->dev->absbit);
		__set_bit(ABS_MT_POSITION_X, tpd->dev->absbit);
		__set_bit(ABS_MT_POSITION_Y, tpd->dev->absbit);
		__set_bit(ABS_MT_TOUCH_MAJOR, tpd->dev->absbit);
		__set_bit(ABS_MT_TRACKING_ID, tpd->dev->absbit);
		__set_bit(ABS_MT_PRESSURE, tpd->dev->absbit);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	} 
	else 
	{
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		__clear_bit(ABS_X, tpd->dev->absbit);
		__clear_bit(ABS_Y, tpd->dev->absbit);
		__clear_bit(ABS_PRESSURE, tpd->dev->absbit);
		__clear_bit(ABS_MT_POSITION_X, tpd->dev->absbit);
		__clear_bit(ABS_MT_POSITION_Y, tpd->dev->absbit);
		__clear_bit(ABS_MT_TOUCH_MAJOR, tpd->dev->absbit);
		__clear_bit(ABS_MT_TRACKING_ID, tpd->dev->absbit);
		__clear_bit(ABS_MT_PRESSURE, tpd->dev->absbit);
	}
 	return;        
}
EXPORT_SYMBOL(touch_keylock_enable);
/*                                    */

static void read_dummy_interrupt(void)
{
	uint8_t buf[TS_READ_REGS_LEN] = {0,};
	int read_num = 0;
	int ret_val = 0;

	ret_val = melfas_i2c_read(melfas_i2c_client, TS_READ_LEN_ADDR, 1, buf);
	TPD_DMESG("[melfas_tpd] ret_val: %d, %d \n", ret_val, buf[0]);
	if (ret_val)
		return;
	read_num = buf[0];

//                                                                                        
    if(read_num >= TS_READ_REGS_LEN) {
        read_num = TS_READ_REGS_LEN-1;
    }
//                                             

	if(read_num)
	{
		melfas_i2c_read(melfas_i2c_client, TS_READ_START_ADDR, read_num, buf);

		if(0x0F == buf[0])
			esd_rest_tp();
	}
    return;
}

/*                                                                             */
static void monitor_ghost_finger(struct work_struct *work)
{
	if (ghost_touch_cnt >= 45){
		printk("<MELFAS> ghost finger pattern DETECTED! : %d \n\n", ghost_touch_cnt);

		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		wait_event_interruptible(melfas_waiter, melfas_tpd_flag == 0);

		tpd_hw_disable();
		msleep(100);
		//melfas_tpd_flag=1;
		melfas_ts_release_all_finger();
		input_mt_sync(tpd->dev);
		input_sync(tpd->dev);
		tpd_hw_enable();
		msleep(200);
		msleep(100);

		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}
	
	schedule_delayed_work(&ghost_monitor_work, msecs_to_jiffies(HZ * 50));

	ghost_touch_cnt = 0;
	ghost_x = 1000;
	ghost_y = 1000;
	return;
}
/*                                                                             */

static int melfas_tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{             
    int err = 0, i;
	int ret = 0;
	struct task_struct *thread = NULL;
	
	TPD_DMESG("[melfas_tpd] %s\n", __func__);
	
	//power on
	tpd_hw_enable();
    msleep(200);
	msleep(100);

    melfas_i2c_client = client;
	
#if 0//defined (GN_MTK_BSP)
		err = tpd_create_attr();
		if(err)
		{
			TPD_DMESG(" tpd create attribute err = %d\n", err);
		}
#else
	err = device_create_file(&client->dev, &dev_attr_update);
	if (err) {
		printk( "Touchscreen : update_touch device_create_file: Fail\n");
		device_remove_file(&client->dev, &dev_attr_update);
		return err;
	}
#endif

	err = device_create_file(&client->dev, &dev_attr_fw);
	if (err) {
		printk( "Touchscreen : fw_touch device_create_file: Fail\n");
		device_remove_file(&client->dev, &dev_attr_fw);
		return err;
	}
	err = device_create_file(&client->dev, &dev_attr_reset);
	if (err) {
		printk( "Touchscreen : reset_touch device_create_file: Fail\n");
		device_remove_file(&client->dev, &dev_attr_reset);
		return err;
	}
	
#ifdef TPD_HAVE_BUTTON
	for(i =0; i < TPD_KEY_COUNT; i ++){
		input_set_capability(tpd->dev,EV_KEY,tpd_keys_local[i]);
	}
#endif
#ifdef CONFIG_LGE_AAT_KEY
	for(i =0; i < 2; i ++){
		input_set_capability(tpd->dev,EV_KEY,aat_keys_local[i]);
	}
#endif
#if defined(FEATURE_LGE_V4_HW) //                                              
	/* Do not update touch firmware */
#else
	ret = melfas_firmware_update(client);
#endif //                                       

    thread = kthread_run(melfas_touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread))
    { 
        err = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE "[melfas_tpd] failed to create kernel thread: %d\n", err);
    }
 
    // set INT mode
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	
    msleep(50);

    mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, melfas_i2c_tpd_eint_interrupt_handler, 1);

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    tpd_load_status = 1;

	/*                                                                             */
	INIT_DELAYED_WORK(&ghost_monitor_work, monitor_ghost_finger);
	schedule_delayed_work(&ghost_monitor_work, msecs_to_jiffies(HZ * 50));
	/*                                                                             */

	if (ret == 0) 
	{
		read_dummy_interrupt();
	}


    return 0;

}
static int melfas_tpd_i2c_remove(struct i2c_client *client)
{
    return 0;
}
static int melfas_tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	TPD_DMESG("[melfas_tpd] %s\n", __func__);
    strcpy(info->type, "mtk-tpd");
    return 0;
}
static int melfas_tpd_local_init(void) 
{

	TPD_DMESG("[melfas_tpd] end %s, %d\n", __FUNCTION__, __LINE__);  
    if(i2c_add_driver(&melfas_tpd_i2c_driver)!=0)
    {
        TPD_DMESG("[melfas_tpd] unable to add i2c driver.\n");
        return -1;
    }
    if(tpd_load_status == 0)
    {
    	TPD_DMESG("[melfas_tpd] add error touch panel driver.\n");
    	i2c_del_driver(&melfas_tpd_i2c_driver);
    	return -1;
    }
  	#if 0//def TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
	#endif   

   
    tpd_type_cap = 1;

    return 0;
}

static void melfas_ts_release_all_finger(void)
{
	int i;
    TPD_DMESG("[melfas_tpd] %s\n", __func__);

	for(i=0; i<TS_MAX_TOUCH; i++)
	{
		g_Mtouch_info[i].pressure = -1;
	}
	input_mt_sync(tpd->dev); 
	input_sync(tpd->dev);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_tpd_early_suspend(struct early_suspend *h)
{
//    TPD_DMESG("[melfas_tpd] %s\n", __func__);
    melfas_ts_release_all_finger();
	//irq mask
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	//power down
	tpd_hw_disable();
	/*                                                                             */
	cancel_delayed_work(&ghost_monitor_work);
	/*                                                                             */
}

static void melfas_tpd_late_resume(struct early_suspend *h)
{
//    TPD_DMESG("[melfas_tpd] %s\n", __func__);
    //power on
	tpd_hw_enable();
	melfas_ts_release_all_finger();
	/*                                                            */
	msleep(100);
	read_dummy_interrupt();
   	//irq unmask
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
	/*                                                                             */
	ghost_touch_cnt = 0;
	schedule_delayed_work(&ghost_monitor_work, msecs_to_jiffies(HZ * 50));
	/*                                                                             */
}
#endif

static struct tpd_driver_t melfas_tpd_device_driver =
{
    .tpd_device_name = "melfas_mms134",
    .tpd_local_init = melfas_tpd_local_init,
#ifdef CONFIG_HAS_EARLYSUSPEND    
    .suspend = melfas_tpd_early_suspend,
    .resume = melfas_tpd_late_resume,
#endif
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif		
};

/* called when loaded into kernel */
static int __init melfas_tpd_driver_init(void)
{
    TPD_DMESG("[melfas_tpd] %s\n", __func__);
	i2c_register_board_info(0, &melfas_i2c_tpd, 1);
    if ( tpd_driver_add(&melfas_tpd_device_driver) < 0)
        TPD_DMESG("[melfas_tpd] add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit melfas_tpd_driver_exit(void)
{
    TPD_DMESG("[melfas_tpd] %s\n", __func__);
    tpd_driver_remove(&melfas_tpd_device_driver);
}


module_init(melfas_tpd_driver_init);
module_exit(melfas_tpd_driver_exit);



