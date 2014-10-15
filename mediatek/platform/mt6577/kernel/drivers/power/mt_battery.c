/*****************************************************************************
 *
 * Filename:
 * ---------
 *    mt6329_battery.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of mt6329 Battery charging algorithm
 *   and the Anroid Battery service for updating the battery status
 *
 * Author:
 * -------
 * James Lo
 *
 ****************************************************************************/
#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/xlog.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/mt_gpt.h>
//                                                                
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <cust_adc.h>
#include <mach/mt_boot.h>
//                                                                
#include <cust_battery.h>
#include "mt_battery.h"
//                                                           

#if defined(CONFIG_MAX8971_CHARGER)
#include "max8971.h"
#elif defined(CONFIG_SINGLE_CHARGER)
#include "charger_rt9536.h"
#elif defined(CONFIG_MINIABB_CHARGER)
#include "lge_miniabb.h"
#else
#error "[BATTERY] : NOT Charger defined"
#endif
//                                                           

#include <mach/pmic_mt6329_hw_bank1.h>
#include <mach/pmic_mt6329_sw_bank1.h>
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

#include <mach/system.h>

#if defined ( LGE_BSP_LGBM ) //                                              

#include <cust_fuel_gauge.h>

/*
** A stupid charger is a charger that charger voltage drops under 4.2[V].
** So charger exist and Not exist interrupt happens many times, finally charging control can't be done properly.
** The solution is to check the voltage of charger when the charger was removed.
*/
//#define ENABLE_SUPPORT_STUPID_CHARGER

#define ENABLE_LGBM_DEBUG

#define LGBM_TAG "LGBM"

#define LGBM_ERR(fmt, args...)  xlog_printk(ANDROID_LOG_ERROR, LGBM_TAG, "[ERROR] %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)

#if defined ( ENABLE_LGBM_DEBUG )
/* You need to select proper loglevel to see the log what you want. */
#if 1 //                                              
#define LGBM_ENTRY(f)       ;
#define LGBM_EXIT(f)        ;
#else
#define LGBM_ENTRY(f)       xlog_printk(ANDROID_LOG_ERROR, LGBM_TAG, "ENTER : %s()\n", __FUNCTION__)
#define LGBM_EXIT(f)        xlog_printk(ANDROID_LOG_ERROR, LGBM_TAG, "EXIT : %s()\n", __FUNCTION__)
#endif //                                              
#define LGBM_LOG(fmt, args...)  xlog_printk(ANDROID_LOG_ERROR, LGBM_TAG, fmt, ##args)
#define LGBM_DBG(fmt, args...)  xlog_printk(ANDROID_LOG_ERROR, LGBM_TAG, fmt, ##args)
#else
#define LGBM_ENTRY(f)           xlog_printk(ANDROID_LOG_DEBUG, LGBM_TAG, "ENTER : %s()\n", __FUNCTION__)
#define LGBM_EXIT(f)            xlog_printk(ANDROID_LOG_DEBUG, LGBM_TAG, "EXIT : %s()\n", __FUNCTION__)
#define LGBM_LOG(fmt, args...)  xlog_printk(ANDROID_LOG_INFO, LGBM_TAG, fmt, ##args)
#define LGBM_DBG(fmt, args...)  xlog_printk(ANDROID_LOG_DEBUG, LGBM_TAG, fmt, ##args)
#endif

#define LGBM_BAT_VOLT_BUF_SIZE 12

static CHARGER_TYPE g_LGBM_charger_type = CHARGER_UNKNOWN; /* unknown means no charger */
int g_LGBM_wakeup_by_charger = 0;
kal_int32 g_FuelGaugeTrimFactor = 0;
int g_FuelGaugeResistance = 0;
int g_lgbmQmax = LGBM_BAT_CAPACITY_MID;
int get_pmic_flag=0;
LGBmCableId g_lgbmBootUsbCableId = LGBM_CABLE_ID_UNKNOWN;

#define R_FG_VALUE      20 // mOhm
#define R_FG_OFFSET     23 // mOhm

#define UNIT_FGCURRENT     (158122)     // 158.122 uA
int CAR_TUNE_VALUE = 106;
extern kal_int32 chip_diff_trim_value_en;
extern kal_int32 use_chip_trim_value(kal_int32 not_trim_val);
extern kal_uint32 fg_get_data_ready_status(void);

kal_int32 LGBM_ReadFuelGaugeTrimFactor( void );
kal_int32 LGBM_GetTrimmedValue(kal_int32 oriValue);

typedef struct{
    INT32 BatteryTemp;
    INT32 TemperatureR;
}BATT_TEMPERATURE;

#if defined(CONFIG_MAX8971_CHARGER)
BATT_TEMPERATURE lgbmBatTempTbl[] = {
    {-20,549674},  // at -20C, Rntc, -20C = 660.000 k-ohm
    {-15,372164},
    {-10,285927},
    { -5,221581},
    {  0,167050},
    {  5,132667},
    { 10,103469},
    { 15,81661},
    { 20,64932},
    { 25,52595},
    { 30,42894},
    { 35,34652},
    { 40,28701},
    { 45,23381},
    { 50,18000},
    { 55,15700},
    { 60,11800},
    { 65,8800}
};
#elif defined (CONFIG_MINIABB_CHARGER)
BATT_TEMPERATURE lgbmBatTempTbl[] = { 
    {-30,738998},           // at -20C, Rntc, -20C = 738.998 k-ohm
    {-20,547455},
    {-15,445599},
    {-10,338492},
    { -5,250940},
    {  0,198590},
    {  5,152100},
    { 10,120588},
    { 15,95063},
    { 20,75350},
    { 25,61070},
    { 30,49541},
    { 35,39117},
    { 40,33965},
    { 45,27553},
    { 50,22470},
    { 55,18426},
    { 60,15186},
    { 65,12966}
};
#else
BATT_TEMPERATURE lgbmBatTempTbl[] = {
    {-20,660000},  // at -20C, Rntc, -20C = 660.000 k-ohm
    {-15,500000},
    {-10,390000},
    { -5,290000},
    {  0,225000},
    {  5,170000},
    { 10,135000},
    { 15,103000},
    { 20,84000},
    { 25,67000},
    { 30,54000},
    { 35,43000},
    { 40,34000},
    { 45,28000},
    { 50,23000},
    { 55,18000},
    { 60,15700},
    { 65,11800}
};
#endif
#define LGBM_TEMPERATURE_M10    (-10)
#define LGBM_TEMPERATURE_0  (0)
#define LGBM_TEMPERATURE_25     (25)
#define LGBM_TEMPERATURE_50     (50)

#define LGBM_BAT_REMOVE_ADC_TH (1180)
#define LGBM_EOC_VERIFY_COUNT (3)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

#define LGBM_CABLE_ID_56K_ADC_MIN       (235)
#define LGBM_CABLE_ID_56K_ADC       (473) /* 0.473[V] */
#define LGBM_CABLE_ID_56K_ADC_MAX       (675)
#define LGBM_CABLE_ID_130K_ADC_MIN  (676)
#define LGBM_CABLE_ID_130K_ADC  (878) /* 0.878[V] */
#ifdef TARGET_S1_75
#define LGBM_CABLE_ID_130K_ADC_MAX  (990)
#define LGBM_CABLE_ID_180K_ADC_MIN  (991) /* .991[V] */
#define LGBM_CABLE_ID_180K_ADC  (1171) /* 1.171[V] */
#define LGBM_CABLE_ID_180K_ADC_MAX  (1524)
#else
#define LGBM_CABLE_ID_130K_ADC_MAX  (1100)
#define LGBM_CABLE_ID_180K_ADC_MIN  (1101) /* 1.071[V] */
#define LGBM_CABLE_ID_180K_ADC  (1171) /* 1.071[V] */
#define LGBM_CABLE_ID_180K_ADC_MAX  (1524)
#endif
#define LGBM_CABLE_ID_910K_ADC_MIN  (1525)
#define LGBM_CABLE_ID_910K_ADC  (1978) /* 1.978[V] */
#define LGBM_CABLE_ID_910K_ADC_MAX  (2238)
#define LGBM_CABLE_ID_OPEN_ADC_MIN  (2239)
#define LGBM_CABLE_ID_OPEN_ADC  (2500) /* 2.500[V] */

#if 1 //                                              
#else
kal_int32 g_lgbmSoc15Volt = 0;
#endif //                                              

typedef struct LGBmVitalTag
{
    kal_int32 batTempAdc;
    kal_int32 batVoltAdc;
    kal_int32 chgVoltAdc;
    kal_int32 accColumb;
    kal_int32 batCurrent;
    kal_int32 batTemp;
    kal_int32 batVolt;
    kal_int32 chgVolt;
} LGBmVital;

#define LGBM_OTP_STOP_MIN_TEMP          (-10)
#define LGBM_OTP_STOP_MAX_TEMP          (55)
#define LGBM_OTP_DECREASE_TEMP          (45)

#define LGBM_OTP_STOP_TO_NORMAL_TEMP     (-5)
#define LGBM_OTP_DECREASE_TO_NORMAL_TEMP (42)
#define LGBM_OTP_STOP_TO_DECREASE_TEMP   (52)

#define LGBM_OTP_STOP_VOLTAGE       (4000)  // STOP CHARGING WHEN TEMPERATURE IS 45 C ~ 55C AND VOLTAGE IS OVER 4.0V.
#define LGBM_OTP_HYST_VOLTAGE       (3900)  // STOP TO DECREASE CHARGINHG VOLTAGE WHEN TEMPERATURE IS 45 C ~ 55C.

#define LGBM_CHARGING_FULL_TO_CHARGING_SOC  (99)
#define LGBM_RECHARGING_SOC (100)


#define LGBM_SW_EOC_CURRENT_TH (50)

#define LGBM_Q_MAX_UPDATE_TH (LGBM_BAT_CAPACITY_MID*85/100)

#if defined (CONFIG_MAX8971_CHARGER)
#define LGBM_CUT_OFF_VOLTAGE 3400
#else
#define LGBM_CUT_OFF_VOLTAGE 3300
#endif
#if defined (CONFIG_MAX8971_CHARGER)
#define LGBM_SW_EOC_VOLTAGE_TH (4280)
#else
#define LGBM_SW_EOC_VOLTAGE_TH (4300)
#endif


typedef enum LGBmTempStateTag
{
    LGBM_TEMP_LOW = 0,
    LGBM_TEMP_MIDDLE,
    LGBM_TEMP_HIGH,
    LGBM_TEMP_ULTRA_HIGH,
    LGBM_TEMP_UNKNOWN
} LGBmTempState;

typedef enum {
    LGBM_TS_INIT = 0,
    LGBM_TS_NO_CHARGER,
    LGBM_TS_CHARGING,
    LGBM_TS_CHARGING_FULL,
    LGBM_TS_FACTORY,
    LGBM_TS_INSERT_BATTERY,
    LGBM_TS_UNKNOWN
} LGBmState;

typedef enum LGBmChargingCurrentTag
{
    LGBM_CC_OFF = 0,
    LGBM_CC_USB_100,
    LGBM_CC_USB_500,
    LGBM_CC_I_SET,
    LGBM_CC_FACTORY,
    LGBM_CC_UNKNOWN,
} LGBmChargingCurrent;

typedef enum {
    LGBM_OTP_NORMAL_CHARGING = 0,
    LGBM_OTP_DECREASE_CHARGING,
    LGBM_OTP_STOP_CHARGING,
    LGBM_OTP_UNKNOWN,
} LGBmOTPState;

typedef enum LGBmSocTrackStateTag
{
    LGBM_SOC_UN_TRACKED = 0,
    LGBM_SOC_ON_TRACKING,
    LGBM_SOC_TRACKED
} LGBmSocTrackState;

typedef struct LGBmDataTag
{
    kal_int32 initSoc;
    kal_int32 curSoc;
    kal_int32 batExist;
    CHARGER_TYPE charger;
    LGBmState bmState;
    LGBmOTPState bmOTPState;
    kal_bool  bmOTPChanged;
    LGBmSocTrackState socTrackState;
    LGBmTempState tempState;
} LGBmData;

LGBmVital *g_pAtCmdBatVital = NULL; /* bad design but for easy implementation, it's read only variable for atci_service process */
LGBmData *g_pAtCmdBmData = NULL; /* bad design but for easy implementation, it's read only variable for atci_service process */
kal_int32 g_AtCmdChargeMode = 0;
//                                                                              
unsigned int g_AtCmdDeviceID = 0;
//                                                                              

kal_int32 g_AtCmdBatSocUI = 0;
int g_AtCmdBatFullUI = 0;
static LGBmChargingCurrent prevChgCurrent = LGBM_CC_UNKNOWN;
static LGBmState prevBmState = LGBM_TS_UNKNOWN;


#if defined ( ENABLE_SUPPORT_STUPID_CHARGER )
kal_int32 g_stupidCharger = 0;
#endif

void BatThread_XGPTConfig(void);
kal_bool LGBM_ReadChargerExistance(void);


#endif //                                              

int Enable_BATDRV_LOG = 0;
//int Enable_BATDRV_LOG = 1;

//                                                                              
#if 1
/*                                  */
#define CONFIG_LGE_NVRAM_ALGORITHM
#endif
//                                                                              


///////////////////////////////////////////////////////////////////////////////////////////
//// Thermal related flags
///////////////////////////////////////////////////////////////////////////////////////////

//                                                       
#if 1
int g_battery_thermal_throttling_flag=2; // 0:nothing, 1:enable batTT&chrTimer, 2:disable batTT&chrTimer, 3:enable batTT, disable chrTimer
#else
int g_battery_thermal_throttling_flag=3; // 0:nothing, 1:enable batTT&chrTimer, 2:disable batTT&chrTimer, 3:enable batTT, disable chrTimer
#endif
int battery_cmd_thermal_test_mode=0;
int battery_cmd_thermal_test_mode_value=0;
#if 1
int g_battery_tt_check_flag=1; // 0:default enable check batteryTT, 1:default disable check batteryTT
#else
int g_battery_tt_check_flag=0; // 0:default enable check batteryTT, 1:default disable check batteryTT
#endif
//                                                       

int g_updateBatStateFlag = 0;

///////////////////////////////////////////////////////////////////////////////////////////
//// JEITA
///////////////////////////////////////////////////////////////////////////////////////////
#if defined(MTK_JEITA_STANDARD_SUPPORT)
int g_jeita_recharging_voltage=4110;
int gFGsyncTimer_jeita = 0;
int g_default_sync_time_out_jeita= CUST_SOC_JEITA_SYNC_TIME;
int g_temp_status=TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag =KAL_TRUE;
int mtk_jeita_support_flag=1;
#else
int mtk_jeita_support_flag=0;
#endif

#if defined(CONFIG_POWER_VERIFY)
//
#else
///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC FGADC Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
extern void mt_power_off(void);
extern kal_int32 FGADC_Get_BatteryCapacity_CoulombMothod(void);
extern kal_int32 FGADC_Get_BatteryCapacity_VoltageMothod(void);
extern void FGADC_Reset_SW_Parameter(void);
extern void FGADC_thread_kthread(void);
extern void fgauge_initialization(void);
extern CHARGER_TYPE mt_charger_type_detection(void);
extern bool mt_usb_is_device(void);
extern kal_int32 fgauge_read_capacity_by_v(void);
extern kal_int32 fgauge_read_current(void);
extern int get_r_fg_value(void);
extern kal_bool get_gFG_Is_Charging(void);

extern kal_int32 gFG_current;
extern kal_int32 gFG_voltage;
extern kal_int32 gFG_DOD0;
extern kal_int32 gFG_DOD1;
extern kal_int32 gFG_columb;
extern kal_bool gFG_Is_Charging;
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
extern int g_switch_to_i2c_polling_mode;

extern int gFG_15_vlot;
extern kal_int32 gFG_capacity_by_v;
#endif

#if defined ( LGE_BSP_LGBM ) //                                              
#else
//                                                                
USB_ID_TYPE readUSB_ID_Value();
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
//                                                                
#endif //                                              

//                                                                 
extern char *batt_exist;
//                                                                 

//                                                                             
static int fake_batt_mode = 0;
//                                                                             

//                                                                   
extern char *batt_id_info;
int batt_id_check=0;
//                                                                   

//                                                                               
extern char offChargingBatteryFull;
//                                                                               

/*                                                                  */
#if defined (LGE_BSP_LGBM)
extern void ist30xx_set_ta_mode ( bool charging );
#endif 
/*                                                                  */

#if defined(CONFIG_POWER_VERIFY)
//
#else
///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC AUXADC Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
#define AUXADC_BATTERY_VOLTAGE_CHANNEL  0
#define AUXADC_REF_CURRENT_CHANNEL         1
#define AUXADC_CHARGER_VOLTAGE_CHANNEL  2
#define AUXADC_TEMPERATURE_CHANNEL         3

#define VOLTAGE_FULL_RANGE     1200
#define ADC_PRECISE         1024 // 10 bits

void get_pmic_auxadc_reg(void)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:ADC-CH0] Bank0[0xDB]=0x%x, Bank0[0xDA]=0x%x, Bank0[0xE7]=0x%x, Bank0[0xE6]=0x%x\r\n",
        upmu_get_reg_value(0xDB), upmu_get_reg_value(0xDA), upmu_get_reg_value(0xE7), upmu_get_reg_value(0xE6));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:ADC-CH1] Bank0[0xDD]=0x%x, Bank0[0xDC]=0x%x\r\n",
        upmu_get_reg_value(0xDD), upmu_get_reg_value(0xDC));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:ADC-CH2] Bank0[0xDF]=0x%x, Bank0[0xDE]=0x%x\r\n",
        upmu_get_reg_value(0xDF), upmu_get_reg_value(0xDE));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:ADC-CH3] Bank0[0xE1]=0x%x, Bank0[0xE0]=0x%x\r\n",
        upmu_get_reg_value(0xE1), upmu_get_reg_value(0xE0));
}

void get_pmic_auxadc_reg_all(void)
{
    //get_pmic_auxadc_reg();
    //get_pmic_auxadc_reg();
    //get_pmic_auxadc_reg();
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "/-------------------------------------------------\r\n");
}

static DEFINE_MUTEX(pmic_adc_mutex);

kal_uint32 g_eco_version = 0;
#define PMIC6329_E1_CID_CODE    0x0029
int g_E1_vbat_sense = 0;

int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount)
{
    kal_int32 u4Sample_times = 0;
    kal_int32 u4channel[4] = {0,0,0,0};
    kal_int32 adc_result=0;
    kal_int32 adc_result_temp=0;
    kal_int32 r_val_temp=0;

    kal_uint32 count=0;
    kal_uint32 count_time_out=1000;
    kal_uint32 ret_data=0;
    kal_uint32 temp_data_7_0=0;
    kal_uint32 temp_data_9_8=0;
    //kal_uint32 i=0;

    kal_int32 data_55_48 = 0;
    kal_int32 data_63_56 = 0;
    kal_int32 otp_gain_trim_data = 0;
    kal_int32 otp_offset_trim_data = 0;

    mutex_lock(&pmic_adc_mutex);

    #if 0
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:ADC-Before] Bank0[0xE8]=0x%x, Bank0[0xE9]=0x%x, Bank1[0x14]=0x%x\r\n",
            upmu_get_reg_value(0xE8), upmu_get_reg_value(0xE9),
            upmu_get_reg_value_bank1(0x14) );
    }
    #endif

    //upmu_auxadc_ch_sel(dwChannel);

    data_55_48=upmu_otpc_otp_pdo_55_48();
    data_63_56=upmu_otpc_otp_pdo_63_56();
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] data_55_48=0x%x, data_63_56=0x%x \r\n",
            data_55_48, data_63_56);
    }
    otp_gain_trim_data = (data_55_48 & 0x7F); //[54:48]
    if(otp_gain_trim_data > 64)
    {
        otp_gain_trim_data = otp_gain_trim_data - 128;
    }
    data_55_48 = (data_55_48 & 0x80) >> 7;
    data_63_56 = (data_63_56 & 0x1F) << 1;
    otp_offset_trim_data = data_63_56 | data_55_48; //[60:55]
    if(otp_offset_trim_data > 32)
    {
        otp_offset_trim_data = otp_offset_trim_data - 64;
    }
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] otp_gain_trim_data=%d, otp_offset_trim_data=%d \r\n",
            otp_gain_trim_data, otp_offset_trim_data);
    }

    if( dwChannel == AUXADC_BATTERY_VOLTAGE_CHANNEL )
    {
        u4Sample_times=0;
        if (g_eco_version == PMIC6329_E1_CID_CODE)
        {
        upmu_chr_adcin_vbat_en(1);
        }
        upmu_auxadc_spl_num(0x1);
        do
        {
            //upmu_auxadc_start(0);
            //upmu_auxadc_start(1);
            upmu_write_reg_value(0xE8,0x02,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }
            upmu_write_reg_value(0xE8,0x03,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }

            //reset parameter
            ret_data=0;
            temp_data_7_0=0;
            temp_data_9_8=0;

            count=0;
            while( upmu_auxadc_get_ch0_ready() != 1 )
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "0");

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xDB]=0x%x, Bank0[0xDD]=0x%x, Bank0[0xDF]=0x%x, Bank0[0xE1]=0x%x\r\n",
                        upmu_get_reg_value(0xDB), upmu_get_reg_value(0xDD), upmu_get_reg_value(0xDF), upmu_get_reg_value(0xE1));

                    upmu_write_reg_value_bank1(0x00,0,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,1,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit0 is auxadc_ck

                    upmu_write_reg_value_bank1(0x00,0x02,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,0x00,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit3, bit2 for AUXADC FSM state
                }

                if( (count++) > count_time_out)
                {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_0] Time out!\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
                    break;
                }
            }
            //temp_data_7_0=upmu_auxadc_get_ch0_data_7_0();
            temp_data_7_0=upmu_auxadc_get_trimming_data_7_0();
            ret_data = temp_data_7_0;
            //temp_data_9_8=upmu_auxadc_get_ch0_data_9_8();
            temp_data_9_8=upmu_auxadc_get_trimming_data_9_8();
            ret_data |= (temp_data_9_8 << 8);
            //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[CH0] ret_data=%d (9_8=%x,7_0=%x)\r\n", ret_data, temp_data_9_8, temp_data_7_0);
            u4channel[0] += ret_data;

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_0] ret_data=%d (9_8=%x,7_0=%x)\r\n", ret_data, temp_data_9_8, temp_data_7_0);
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_0] %d\r\n", ret_data);
            }

            if (Enable_BATDRV_LOG == 1) {
                //HW debug
                ret_data=0;
                temp_data_7_0=0;
                temp_data_9_8=0;
                temp_data_7_0=upmu_auxadc_get_ch0_data_7_0();
                ret_data = temp_data_7_0;
                temp_data_9_8=upmu_auxadc_get_ch0_data_9_8();
                ret_data |= (temp_data_9_8 << 8);
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_0] not trim=%d\r\n", ret_data);
            }

            u4Sample_times++;

            if (Enable_BATDRV_LOG == 1) {
                get_pmic_auxadc_reg_all();
            }

        }
        while (u4Sample_times < deCount);

        /* Value averaging  */
        u4channel[0]  = u4channel[0]  / deCount;

    }
    else if( dwChannel == AUXADC_REF_CURRENT_CHANNEL )
    {
        u4Sample_times=0;
        if (g_eco_version == PMIC6329_E1_CID_CODE)
        {
        upmu_chr_adcin_vsen_en(1);
        }
        upmu_auxadc_spl_num(0x1);
        do
        {
            //upmu_auxadc_start(0);
            //upmu_auxadc_start(1);
            upmu_write_reg_value(0xE8,0x12,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }
            upmu_write_reg_value(0xE8,0x13,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }

            //reset parameter
            ret_data=0;
            temp_data_7_0=0;
            temp_data_9_8=0;

            count=0;
            while( upmu_auxadc_get_ch1_ready() != 1 )
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "1");

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xDB]=0x%x, Bank0[0xDD]=0x%x, Bank0[0xDF]=0x%x, Bank0[0xE1]=0x%x\r\n",
                        upmu_get_reg_value(0xDB), upmu_get_reg_value(0xDD), upmu_get_reg_value(0xDF), upmu_get_reg_value(0xE1));

                    upmu_write_reg_value_bank1(0x00,0,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,1,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit0 is auxadc_ck

                    upmu_write_reg_value_bank1(0x00,0x02,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,0x00,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit3, bit2 for AUXADC FSM state
                }

                if( (count++) > count_time_out)
                {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_1] Time out!\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
                    break;
                }
            }
            temp_data_7_0=upmu_auxadc_get_ch1_data_7_0();
            ret_data = temp_data_7_0;
            temp_data_9_8=upmu_auxadc_get_ch1_data_9_8();
            ret_data |= (temp_data_9_8 << 8);
            u4channel[1] += ret_data;

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_1] ret_data=%d (9_8=%x,7_0=%x)\r\n", ret_data, temp_data_9_8, temp_data_7_0);
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_1] %d\r\n", ret_data);
            }

            u4Sample_times++;

            if (Enable_BATDRV_LOG == 1) {
                get_pmic_auxadc_reg_all();
            }

        }
        while (u4Sample_times < deCount);

        /* Value averaging  */
        u4channel[1]  = u4channel[1]  / deCount;

    }
    else if( dwChannel == AUXADC_CHARGER_VOLTAGE_CHANNEL )
    {
        u4Sample_times=0;
        upmu_chr_adcin_vchr_en(1); // for mode precise - start
        upmu_auxadc_spl_num(0xF);
        do
        {
            //upmu_auxadc_start(0);
            //upmu_auxadc_start(1);
            upmu_write_reg_value(0xE8,0x22,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }
            upmu_write_reg_value(0xE8,0x23,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }

            //reset parameter
            ret_data=0;
            temp_data_7_0=0;
            temp_data_9_8=0;

            count=0;
            while( upmu_auxadc_get_ch2_ready() != 1 )
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "2");

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xDB]=0x%x, Bank0[0xDD]=0x%x, Bank0[0xDF]=0x%x, Bank0[0xE1]=0x%x\r\n",
                        upmu_get_reg_value(0xDB), upmu_get_reg_value(0xDD), upmu_get_reg_value(0xDF), upmu_get_reg_value(0xE1));

                    upmu_write_reg_value_bank1(0x00,0,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,1,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit0 is auxadc_ck

                    upmu_write_reg_value_bank1(0x00,0x02,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,0x00,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit3, bit2 for AUXADC FSM state
                }

                if( (count++) > count_time_out)
                {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_2] Time out !\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
                    break;
                }
            }
            temp_data_7_0=upmu_auxadc_get_ch2_data_7_0();
            ret_data = temp_data_7_0;
            temp_data_9_8=upmu_auxadc_get_ch2_data_9_8();
            ret_data |= (temp_data_9_8 << 8);
            if(u4Sample_times > 0) //skip the first value
            {
                u4channel[2] += ret_data;
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_2] ret_data=%d (9_8=%x,7_0=%x)\r\n", ret_data, temp_data_9_8, temp_data_7_0);
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_2] %d\r\n", ret_data);
                }
            }

            u4Sample_times++;

            if (Enable_BATDRV_LOG == 1) {
                get_pmic_auxadc_reg_all();
            }

        }
        while (u4Sample_times < deCount);

        /* Value averaging  */
        u4channel[2]  = u4channel[2]  / (deCount-1);

        upmu_chr_adcin_vchr_en(0); // for mode precise - end
        upmu_auxadc_spl_num(0x1);

    }
    else if( dwChannel == AUXADC_TEMPERATURE_CHANNEL )
    {
        upmu_auxadc_spl_num(0xF);

        u4Sample_times=0;
        if (g_eco_version == PMIC6329_E1_CID_CODE)
        {
        upmu_chr_baton_tdet_en(1);
        }
        //Workaround-----------------
        //upmu_auxadc_start(0);
        //upmu_auxadc_start(1);
        //upmu_chr_baton_tdet_en(1);
        //---------------------------
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Reg[0xe8]=0x%x\r\n", upmu_get_reg_value(0xe8) );
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Reg[0xe9]=0x%x\r\n", upmu_get_reg_value(0xe9) );
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Reg[0x38]=0x%x\r\n", upmu_get_reg_value(0x38) );
        do
        {
            //upmu_auxadc_start(0);
            //upmu_auxadc_start(1);
            upmu_write_reg_value(0xE8,0x32,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }
            upmu_write_reg_value(0xE8,0x33,0xFF,0);
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
            }

            //reset parameter
            ret_data=0;
            temp_data_7_0=0;
            temp_data_9_8=0;

            count=0;
            while( upmu_auxadc_get_ch3_ready() != 1 )
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "3");

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xDB]=0x%x, Bank0[0xDD]=0x%x, Bank0[0xDF]=0x%x, Bank0[0xE1]=0x%x\r\n",
                        upmu_get_reg_value(0xDB), upmu_get_reg_value(0xDD), upmu_get_reg_value(0xDF), upmu_get_reg_value(0xE1));

                    upmu_write_reg_value_bank1(0x00,0,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,1,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_0] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit0 is auxadc_ck

                    upmu_write_reg_value_bank1(0x00,0x02,0xFF,0);
                    upmu_write_reg_value_bank1(0x01,0x00,0xFF,0);

                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x00]=0x%x, Bank1[0x01]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x00), upmu_get_reg_value_bank1(0x01));
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_1] Bank1[0x0a]=0x%x, Bank1[0x0a]=0x%x\r\n",
                        upmu_get_reg_value_bank1(0x0a), upmu_get_reg_value_bank1(0x0a)); // bit3, bit2 for AUXADC FSM state
                }

                if( (count++) > count_time_out)
                {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_3] Time out!\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
                    break;
                }
            }
            temp_data_7_0=upmu_auxadc_get_ch3_data_7_0();
            ret_data = temp_data_7_0;
            temp_data_9_8=upmu_auxadc_get_ch3_data_9_8();
            ret_data |= (temp_data_9_8 << 8);
            u4channel[3] += ret_data;

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_3] ret_data=%d (9_8=%x,7_0=%x)\r\n", ret_data, temp_data_9_8, temp_data_7_0);
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[IMM_GetOneChannelValue_PMIC_3] %d\r\n", ret_data);
            }

            u4Sample_times++;

            if (Enable_BATDRV_LOG == 1) {
                get_pmic_auxadc_reg_all();
            }

        }
        while (u4Sample_times < deCount);

        /* Value averaging  */
        u4channel[3]  = u4channel[3]  / deCount;

        upmu_auxadc_spl_num(0x1);

    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] Error dwChannel number 1 (%d)!\r\n", dwChannel);
    }

    if( dwChannel == AUXADC_BATTERY_VOLTAGE_CHANNEL )
    {
        adc_result_temp = u4channel[dwChannel];
        r_val_temp = R_BAT_SENSE;
        adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
    }
    else if( dwChannel == AUXADC_REF_CURRENT_CHANNEL )
    {
        adc_result_temp = u4channel[dwChannel] + ((u4channel[dwChannel]*otp_gain_trim_data)/1024) + otp_offset_trim_data;
        r_val_temp = R_I_SENSE;
        adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
    }
    else if( dwChannel == AUXADC_CHARGER_VOLTAGE_CHANNEL )
    {
        adc_result_temp = u4channel[dwChannel] + ((u4channel[dwChannel]*otp_gain_trim_data)/1024) + otp_offset_trim_data;
        r_val_temp = (((R_CHARGER_1+R_CHARGER_2)*100)/R_CHARGER_2);
        adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
    }
    else if( dwChannel == AUXADC_TEMPERATURE_CHANNEL )
    {
        adc_result_temp = u4channel[dwChannel] + ((u4channel[dwChannel]*otp_gain_trim_data)/1024) + otp_offset_trim_data;
        r_val_temp = 1;
        adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] Error dwChannel number 2 (%d)!\r\n", dwChannel);
    }

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] adc_result_temp=%d, adc_result=%d, r_val_temp=%d\r\n",
            adc_result_temp, adc_result, r_val_temp);
    }

    upmu_auxadc_start(0);
    count=0;

    #if 0
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:ADC-After] Bank0[0xE8]=0x%x, Bank0[0xE9]=0x%x, Bank1[0x14]=0x%x\r\n",
            upmu_get_reg_value(0xE8), upmu_get_reg_value(0xE9),
            upmu_get_reg_value_bank1(0x14) );
    }
    #endif

#if 1 //                                              
    if ( ( dwChannel == AUXADC_TEMPERATURE_CHANNEL ) && ( fake_batt_mode == 1 ) )
    {
        adc_result = 486; /* force to set the temperature to 25 degree celsius */
    }
#endif //                                              

    mutex_unlock(&pmic_adc_mutex);

    return adc_result;

}
#endif

#if defined(CONFIG_POWER_VERIFY)
//
#else
int PMIC_IMM_GetOneChannelValueSleep(int dwChannel, int deCount)
{
    kal_int32 u4Sample_times = 0;
    kal_int32 u4channel[4] = {0,0,0,0};
    kal_int32 adc_result=0;
    kal_int32 adc_result_temp=0;
    kal_int32 r_val_temp=0;

    kal_uint32 count=0;
    kal_uint32 count_time_out=1000;
    kal_uint32 ret_data=0;
    kal_uint32 temp_data_7_0=0;
    kal_uint32 temp_data_9_8=0;
    //kal_uint32 i=0;

    if( dwChannel == AUXADC_BATTERY_VOLTAGE_CHANNEL )
    {
        u4Sample_times=0;
        if (g_eco_version == PMIC6329_E1_CID_CODE)
        {
        upmu_chr_adcin_vbat_en(1);
        }
        upmu_auxadc_spl_num(0x1);
        do
        {
            upmu_write_reg_value(0xE8,0x02,0xFF,0);
            upmu_write_reg_value(0xE8,0x03,0xFF,0);

            //reset parameter
            ret_data=0;
            temp_data_7_0=0;
            temp_data_9_8=0;

            count=0;
            while( upmu_auxadc_get_ch0_ready() != 1 )
            {
                if( (count++) > count_time_out)
                {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_IMM_GetOneChannelValueSleep] Time out!\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Bank0[0xE8]=0x%x\r\n", upmu_get_reg_value(0xE8));
                    break;
                }
            }
            //temp_data_7_0=upmu_auxadc_get_ch0_data_7_0();
            temp_data_7_0=upmu_auxadc_get_trimming_data_7_0();
            ret_data = temp_data_7_0;
            //temp_data_9_8=upmu_auxadc_get_ch0_data_9_8();
            temp_data_9_8=upmu_auxadc_get_trimming_data_9_8();
            ret_data |= (temp_data_9_8 << 8);
            //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[CH0] ret_data=%d (9_8=%x,7_0=%x)\r\n", ret_data, temp_data_9_8, temp_data_7_0);
            u4channel[0] += ret_data;

            u4Sample_times++;

        }
        while (u4Sample_times < deCount);

        /* Value averaging  */
        u4channel[0]  = u4channel[0]  / deCount;

    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_IMM_GetOneChannelValueSleep] Error dwChannel number 1 (%d)!\r\n", dwChannel);
    }

    if( dwChannel == AUXADC_BATTERY_VOLTAGE_CHANNEL )
    {
        adc_result_temp = u4channel[dwChannel];
        r_val_temp = R_BAT_SENSE;
        adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_IMM_GetOneChannelValueSleep] Error dwChannel number 2 (%d)!\r\n", dwChannel);
    }

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_IMM_GetOneChannelValueSleep] adc_result_temp=%d, adc_result=%d, r_val_temp=%d\r\n",
            adc_result_temp, adc_result, r_val_temp);
    }

    upmu_auxadc_start(0);
    count=0;

    return adc_result;

}
#endif

#if defined(CONFIG_POWER_VERIFY)
//
#else
///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC PCHR Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
kal_bool upmu_is_chr_det(void)
{
    kal_uint32 tmp32;
    tmp32=upmu_get_PCHR_CHRDET();
    if(tmp32 == 0)
    {
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[upmu_is_chr_det] No charger\n");
        return KAL_FALSE;
    }
    else
    {
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[upmu_is_chr_det] Charger exist\n");
        if( mt_usb_is_device() )
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[upmu_is_chr_det] Charger exist and USB is not host\n");
            }
            return KAL_TRUE;
        }
        else
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[upmu_is_chr_det] Charger exist but USB is host\n");
            }
            return KAL_FALSE;
        }

        //return KAL_TRUE;
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//// Smart Battery Structure
///////////////////////////////////////////////////////////////////////////////////////////
#define UINT32 unsigned long
#define UINT16 unsigned short
#define UINT8 unsigned char

typedef struct
{
    kal_bool       bat_exist;
    kal_bool       bat_full;
    kal_bool       bat_low;
    UINT32      bat_charging_state;
    UINT32      bat_vol;
    kal_bool     charger_exist;
    UINT32      pre_charging_current;
    UINT32      charging_current;
    INT32      charger_vol;
    UINT32       charger_protect_status;
    UINT32      ISENSE;
    UINT32      ICharging;
    INT32       temperature;
    UINT32      total_charging_time;
    UINT32      PRE_charging_time;
    UINT32      CC_charging_time;
    UINT32      TOPOFF_charging_time;
    UINT32      POSTFULL_charging_time;
    UINT32       charger_type;
    UINT32       PWR_SRC;
    UINT32       SOC;
    UINT32       ADC_BAT_SENSE;
    UINT32       ADC_I_SENSE;
} PMU_ChargerStruct;

typedef enum
{
    PMU_STATUS_OK = 0,
    PMU_STATUS_FAIL = 1,
}PMU_STATUS;

///////////////////////////////////////////////////////////////////////////////////////////
//// Global Variable
///////////////////////////////////////////////////////////////////////////////////////////
#if defined(CONFIG_POWER_VERIFY)
//
#else
static CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;
static unsigned short batteryVoltageBuffer[BATTERY_AVERAGE_SIZE];
static unsigned short batteryCurrentBuffer[BATTERY_AVERAGE_SIZE];
static unsigned short batterySOCBuffer[BATTERY_AVERAGE_SIZE];
static int batteryTempBuffer[BATTERY_AVERAGE_SIZE];
static int batteryIndex = 0;
static int batteryVoltageSum = 0;
static int batteryCurrentSum = 0;
static int batterySOCSum = 0;
static int batteryTempSum = 0;
#endif
PMU_ChargerStruct BMT_status;
kal_bool g_bat_full_user_view = KAL_FALSE;
kal_bool g_Battery_Fail = KAL_FALSE;
kal_bool batteryBufferFirst = KAL_FALSE;

struct wake_lock battery_suspend_lock;

int g_BatTempProtectEn = 0; /*0:temperature measuring default off*/
//int g_BatTempProtectEn = 1; /*1:temperature measuring default on*/

//int g_PMIC_CC_VTH = PMIC_ADPT_VOLT_03_300000_V;
//int g_PMIC_CV_VTH = PMIC_ADPT_VOLT_04_000000_V;
//                                                                   
int V_PRE2CC_THRES = 3160;
#if defined(FEATURE_LGE_V4_HW)
int V_CC2TOPOFF_THRES = 4100;
#else  // V5
int V_CC2TOPOFF_THRES = 4270;
#endif
//                                                                   
int V_compensate_EVB = 80;

int g_HW_Charging_Done = 0;
int g_Charging_Over_Time = 0;

int g_SW_CHR_OUT_EN = 0;
int g_HW_stop_charging = 0;

//                                                           
int g_SW_Charging_On = 0;
int g_Charging_Over_Temp = 0;
unsigned char EOC_status = 0;
unsigned char EOC_counts = 0;
#define EOC_CHECK_DELAY_COUNT    3

static kal_bool at_charging_mode = KAL_FALSE;

//                                                           
// HW CV algorithm
//int g_sw_cv_enable=0;

int CHARGING_FULL_CURRENT=220;    // mA

int gForceADCsolution=0;
//int gForceADCsolution=1;

int gSyncPercentage=0;

unsigned int g_BatteryNotifyCode=0x0000;
unsigned int g_BN_TestMode=0x0000;

kal_uint32 gFGsyncTimer=0;
kal_uint32 DEFAULT_SYNC_TIME_OUT=60; //1mins

int g_Calibration_FG=0;

int g_XGPT_restart_flag=0;

#define CHR_OUT_CURRENT    50

int gSW_CV_prepare_flag=0;

int getVoltFlag = 0;
int g_bat_temperature_pre=0;

int gADC_BAT_SENSE_temp=0;
int gADC_I_SENSE_temp=0;
int gADC_I_SENSE_offset=0;

int g_battery_flag_resume=0;

int gBAT_counter_15=1;

int gFG_can_reset_flag = 1;

//cut off to full
#define POST_CHARGING_TIME 30 * 60 // 30mins
int post_charging_time=0;

#if defined ( LGE_BSP_LGBM ) //                                              
#else
//                                                                
USB_ID_TYPE usb_id_type_num = DEVICE_NONE;

#define FACTORY_CABLE_ADC_THRESHOLD         (3440)      // about 2.10V

// 56k cable, center : 0.47V, 0.37V ~ 0.57V
#define FACTORY_CABLE_56K_MIN_VOLTAGE       (37)
#define FACTORY_CABLE_56K_MAX_VOLTAGE      (57)

// 130k cable, center : 0.88V, 0.78V ~ 0.98V
#define FACTORY_CABLE_130K_MIN_VOLTAGE     (78)
#define FACTORY_CABLE_130K_MAX_VOLTAGE    (110)

// 180k cable, center : 1.07V, 0.99V ~ 1.17V
#define FACTORY_CABLE_180K_MIN_VOLTAGE     (111)
#define FACTORY_CABLE_180K_MAX_VOLTAGE    (130)

// 910k cable, center : 1.98V, 1.88V ~ 2.08V
#define FACTORY_CABLE_910K_MIN_VOLTAGE     (188)
#define FACTORY_CABLE_910K_MAX_VOLTAGE    (208)

// USB cable over 2.40V
#define OPEN_CABLE_MIN_VOLTAGE                  (240)
//                                                                
#endif //                                              


#ifdef LGE_RF_TEMP
static RF_TYPE rf_temp_state = RF_NORMAL_CHARGING_STATE;
static int rf_temperature = 0;

void rf_temperature_ADC()
{
    static int res = 0;
    static unsigned int rf_adc_value = 0;
    static int data[4] = {0,0,0,0};

    res = IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL, data, &rf_adc_value );
    rf_temperature = rf_adc_value;
}

void rf_temperature_check(INT32 rf_temp_adc)
{
    if( BMT_status.bat_exist == KAL_FALSE )  // booting without Battery
    {
        rf_temp_state = RF_NORMAL_CHARGING_STATE;
    }
    else
    {
        if(rf_temp_adc <= RF_ADC_HIGH)
        {
            rf_temp_state = RF_LOW_CHARGING_STATE;
        }
        #if 0
        else if(rf_temp_adc < RF_ADC_LOW)
        {
            rf_temp_state = RF_LOW_CHARGING_STATE;
        }
        #endif
        else
        {
            rf_temp_state = RF_NORMAL_CHARGING_STATE;
        }
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] rf_temp_adc : %d, rf state : %d\n", rf_temp_adc, rf_temp_state);
}
#endif

#if defined ( LGE_BSP_LGBM ) //                                              
#else
//                                                         
#if defined( LGE_FW_OTP )
static BAT_OTP_TYPE otp_state = OTP_NORMAL_CHARGING_STATE;

void otp_state_check(INT32 temperature, UINT32 bat_vol)
{
    if( BMT_status.bat_exist == KAL_FALSE || at_charging_mode == KAL_TRUE )  // booting without Battery
    {
        otp_state = OTP_NORMAL_CHARGING_STATE;
    }
    else
    {
        if((OTP_CHARGING_RESUME_MIN_TEMP < temperature) && (temperature < OTP_CHARGING_RESUME_MAX_TEMP))
        {  // Resume Area
        otp_state = OTP_NORMAL_CHARGING_STATE;
    }
        else if((temperature < OTP_CHARGING_STOP_MIN_TEMP) || (OTP_CHARGING_STOP_MAX_TEMP < temperature))
        {  // Stop Area
            if(temperature < OTP_CHARGING_STOP_MIN_TEMP)  // To show Charging Icon
            {
                otp_state = OTP_DECREASE_STOP_CHARGING_STATE;
            }
            else
            {
            otp_state = OTP_STOP_CHARGING_STATE;
        }
        }
        else if((otp_state != OTP_STOP_CHARGING_STATE) &&
            (OTP_CHARGING_DECREASE_TEMP < temperature) && (temperature <= OTP_CHARGING_STOP_MAX_TEMP))
        {  // Decrease Area
            if(OTP_DECREASE_STOP_VOLTAGE < bat_vol)  // over 4.0V
                {
                otp_state = OTP_DECREASE_STOP_CHARGING_STATE;
                }
            else if(otp_state == OTP_NORMAL_CHARGING_STATE)
                {
                    otp_state = OTP_DECREASE_CHARGING_STATE;
                }
            else if((otp_state == OTP_DECREASE_STOP_CHARGING_STATE) && (bat_vol <= OTP_DECREASE_CHARGING_THRESHOLD))
            {
                otp_state = OTP_DECREASE_CHARGING_STATE;
            }
        }
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] OTP temperature : %d, otp_state : %d, bat_vol : %d\r\n", temperature, otp_state, bat_vol);
}
#endif
//                                                         

/*                                                                                                */
#if defined (LGE_FW_OTP)
int Is_Not_FactoryCable_PowerOn()
{
/*                                                                                            */
    if(usb_id_type_num == DEVICE_NONE)
    {
        usb_id_type_num = readUSB_ID_Value();
    }
/*                                                                                            */

    if (usb_id_type_num == DEVICE_FACTORY_UART_CABLE       ||
        usb_id_type_num == DEVICE_FACTORY_USB_CABLE        ||
        usb_id_type_num == DEVICE_FACTORY_DOWNLOAD_CABLE)
    {
        return 0;
    }
/*                                                                                            */
#if 0
    else if (usb_id_type_num == DEVICE_NONE)
    {
        return -1;
    }
#endif
/*                                                                                            */
    return 1;
}
#endif
/*                                                                                                */
#endif //                                              

////////////////////////////////////////////////////////////////////////////////
// EM
////////////////////////////////////////////////////////////////////////////////
int g_BAT_TemperatureR = 0;
int g_TempBattVoltage = 0;
int g_InstatVolt = 0;
int g_BatteryAverageCurrent = 0;
int g_BAT_BatterySenseVoltage = 0;
int g_BAT_ISenseVoltage = 0;
int g_BAT_ChargerVoltage = 0;

#if defined(CONFIG_POWER_VERIFY)
//
#else
////////////////////////////////////////////////////////////////////////////////
// Definition For GPT
////////////////////////////////////////////////////////////////////////////////
static int bat_thread_timeout = 0;

static DEFINE_MUTEX(bat_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);

////////////////////////////////////////////////////////////////////////////////
//Logging System
////////////////////////////////////////////////////////////////////////////////
int g_chr_event = 0;
int bat_volt_cp_flag = 0;
int bat_volt_check_point = 0;
int g_wake_up_bat=0;

#if defined ( LGE_BSP_LGBM ) //                                              
#else
//                                                                              
#ifdef CONFIG_LGE_NVRAM_ALGORITHM
int bat_soc_tracker = 0;

kal_bool battery_info_write(int battery_data)
{
        /*
            This is the prototype of f_op->read/write
            ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
            ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
        */
        struct file *fd = NULL;
        int ret_val = 0;

        fd = filp_open("/data/battery_data", O_CREAT|O_TRUNC|O_WRONLY , 0666);

        if (IS_ERR(fd))
        {
            ret_val = PTR_ERR(fd);
            printk("Open /data/battery_data file fail! errno=%d\n", ret_val);
            return KAL_FALSE;
        }
        else
        {
            fd->f_op->write(fd, (unsigned char *)&battery_data, sizeof(int), &(fd->f_pos));
            printk("[battery_info_write] f_pos : 0x%X\n", &(fd->f_pos));
            printk("[battery_info_write] write data : %X\n", battery_data);
            filp_close(fd, NULL);
            return KAL_TRUE;
        }
}

int battery_info_read(void)
{
    /*
        This is the prototype of f_op->read/write
        ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
        ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
    */
    struct file *fd = NULL;
    int read_val=0;
    int ret_val = 0;

    fd = filp_open("/data/battery_data", O_RDONLY , 0666);

    if (IS_ERR(fd))
    {
        ret_val = PTR_ERR(fd);
        printk("Open /data/battery_data file fail! errno=%d\n", ret_val);
        return -1;
    }
    else
    {
            fd->f_op->read(fd, (unsigned char *)&read_val, sizeof(int), &(fd->f_pos));
        printk("[battery_info_read] f_pos : 0x%X\n", &(fd->f_pos));
        printk("[battery_info_read] read data : %X\n", read_val);
        filp_close(fd, NULL);
        return read_val;
    }
}



#endif
//                                                                              
#endif //                                              


void wake_up_bat (void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] wake_up_bat. \r\n");
    }

    g_wake_up_bat=1;

#if defined ( LGE_BSP_LGBM )
    g_LGBM_wakeup_by_charger = 1;
#endif

    bat_thread_timeout = 1;
    wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);

////////////////////////////////////////////////////////////////////////////////
// USB-IF
////////////////////////////////////////////////////////////////////////////////
int g_usb_state = USB_UNCONFIGURED;
//int g_usb_state = USB_SUSPEND;

int g_temp_CC_value = Cust_CC_0MA;
int g_soft_start_delay = 1;

#if (CONFIG_USB_IF == 0)
int g_Support_USBIF = 0;
#else
int g_Support_USBIF = 1;
#endif

////////////////////////////////////////////////////////////////////////////////
// Integrate with NVRAM
////////////////////////////////////////////////////////////////////////////////
#define ADC_CALI_DEVNAME "MT_pmic_adc_cali"

#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal _IOW('k', 3, int)
#define ADC_CHANNEL_READ _IOW('k', 4, int)
#define BAT_STATUS_READ _IOW('k', 5, int)
#define Set_Charger_Current _IOW('k', 6, int)
//add for meta tool-----------------------------------------
#define Get_META_BAT_VOL _IOW('k', 10, int) 
#define Get_META_BAT_SOC _IOW('k', 11, int) 
//add for meta tool-----------------------------------------

static struct class *adc_cali_class = NULL;
static int adc_cali_major = 0;
static dev_t adc_cali_devno;
static struct cdev *adc_cali_cdev;

int adc_cali_slop[14] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
int adc_cali_offset[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int adc_cali_cal[1] = {0};

int adc_in_data[2] = {1,1};
int adc_out_data[2] = {1,1};

int battery_in_data[1] = {0};
int battery_out_data[1] = {0};

int charging_level_data[1] = {0};

kal_bool g_ADC_Cali = KAL_FALSE;
kal_bool g_ftm_battery_flag = KAL_FALSE;

////////////////////////////////////////////////////////////////////////////////
// Battery Logging Entry
////////////////////////////////////////////////////////////////////////////////
static struct proc_dir_entry *proc_entry;
static char proc_bat_data[32];

ssize_t bat_log_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
    if (copy_from_user( &proc_bat_data, buff, len )) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "bat_log_write error.\n");
        return -EFAULT;
    }

    if (proc_bat_data[0] == '1') {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "enable battery driver log system\n");
        Enable_BATDRV_LOG = 1;
    } else if (proc_bat_data[0] == '2') {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "enable battery driver log system:2\n");
        Enable_BATDRV_LOG = 2;
    } else {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Disable battery driver log system\n");
        Enable_BATDRV_LOG = 0;
    }

    return len;
}

int init_proc_log(void)
{
    int ret=0;
    proc_entry = create_proc_entry( "batdrv_log", 0644, NULL );

    if (proc_entry == NULL) {
        ret = -ENOMEM;
          xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "init_proc_log: Couldn't create proc entry\n");
    } else {
        proc_entry->write_proc = bat_log_write;
        //proc_entry->owner = THIS_MODULE;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "init_proc_log loaded.\n");
    }

    return ret;
}
#endif

////////////////////////////////////////////////////////////////////////////////
// FOR ANDROID BATTERY SERVICE
////////////////////////////////////////////////////////////////////////////////
struct mt6329_ac_data {
    struct power_supply psy;
    int AC_ONLINE;
};

struct mt6329_usb_data {
    struct power_supply psy;
    int USB_ONLINE;
};

struct mt6329_battery_data {
    struct power_supply psy;
    int BAT_STATUS;
    int BAT_HEALTH;
    int BAT_PRESENT;
    int BAT_TECHNOLOGY;
    int BAT_CAPACITY;
    /* Add for Battery Service*/
    int BAT_batt_vol;
    int BAT_batt_temp;
    /* Add for EM */
    int BAT_TemperatureR;
    int BAT_TempBattVoltage;
    int BAT_InstatVolt;
    int BAT_BatteryAverageCurrent;
    int BAT_BatterySenseVoltage;
    int BAT_ISenseVoltage;
    int BAT_ChargerVoltage;
};

static enum power_supply_property mt6329_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt6329_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt6329_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
    /* Add for Battery Service */
    POWER_SUPPLY_PROP_batt_vol,
    POWER_SUPPLY_PROP_batt_temp,
    /* Add for EM */
    POWER_SUPPLY_PROP_TemperatureR,
    POWER_SUPPLY_PROP_TempBattVoltage,
    POWER_SUPPLY_PROP_InstatVolt,
    POWER_SUPPLY_PROP_BatteryAverageCurrent,
    POWER_SUPPLY_PROP_BatterySenseVoltage,
    POWER_SUPPLY_PROP_ISenseVoltage,
    POWER_SUPPLY_PROP_ChargerVoltage,
};

static int mt6329_ac_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct mt6329_ac_data *data = container_of(psy, struct mt6329_ac_data, psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = data->AC_ONLINE;
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int mt6329_usb_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct mt6329_usb_data *data = container_of(psy, struct mt6329_usb_data, psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        #if defined(CONFIG_POWER_EXT)
        //#if 0
        data->USB_ONLINE = 1;
        val->intval = data->USB_ONLINE;
        #else
        val->intval = data->USB_ONLINE;
        #endif
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int mt6329_battery_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct mt6329_battery_data *data = container_of(psy, struct mt6329_battery_data, psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = data->BAT_STATUS;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = data->BAT_HEALTH;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = data->BAT_PRESENT;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = data->BAT_TECHNOLOGY;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = data->BAT_CAPACITY;
        break;
    case POWER_SUPPLY_PROP_batt_vol:
        val->intval = data->BAT_batt_vol;
        break;
    case POWER_SUPPLY_PROP_batt_temp:
        val->intval = data->BAT_batt_temp;
        break;
    case POWER_SUPPLY_PROP_TemperatureR:
        val->intval = data->BAT_TemperatureR;
        break;
    case POWER_SUPPLY_PROP_TempBattVoltage:
        val->intval = data->BAT_TempBattVoltage;
        break;
    case POWER_SUPPLY_PROP_InstatVolt:
        val->intval = data->BAT_InstatVolt;
        break;
    case POWER_SUPPLY_PROP_BatteryAverageCurrent:
        val->intval = data->BAT_BatteryAverageCurrent;
        break;
    case POWER_SUPPLY_PROP_BatterySenseVoltage:
        val->intval = data->BAT_BatterySenseVoltage;
        break;
    case POWER_SUPPLY_PROP_ISenseVoltage:
        val->intval = data->BAT_ISenseVoltage;
        break;
    case POWER_SUPPLY_PROP_ChargerVoltage:
        val->intval = data->BAT_ChargerVoltage;
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

/* mt6329_ac_data initialization */
static struct mt6329_ac_data mt6329_ac_main = {
    .psy = {
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .properties = mt6329_ac_props,
    .num_properties = ARRAY_SIZE(mt6329_ac_props),
    .get_property = mt6329_ac_get_property,
    },
    .AC_ONLINE = 0,
};

/* mt6329_usb_data initialization */
static struct mt6329_usb_data mt6329_usb_main = {
    .psy = {
    .name = "usb",
    .type = POWER_SUPPLY_TYPE_USB,
    .properties = mt6329_usb_props,
    .num_properties = ARRAY_SIZE(mt6329_usb_props),
    .get_property = mt6329_usb_get_property,
    },
    .USB_ONLINE = 0,
};

/* mt6329_battery_data initialization */
static struct mt6329_battery_data mt6329_battery_main = {
    .psy = {
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = mt6329_battery_props,
    .num_properties = ARRAY_SIZE(mt6329_battery_props),
    .get_property = mt6329_battery_get_property,
    },
/* CC: modify to have a full power supply status */
#if defined(CONFIG_POWER_EXT)
//#if 0
    .BAT_STATUS = POWER_SUPPLY_STATUS_FULL,
    .BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
    .BAT_PRESENT = 1,
    .BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
    .BAT_CAPACITY = 100,
    .BAT_batt_vol = 4200,
    .BAT_batt_temp = 22,
#else
    .BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING,
    .BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
    .BAT_PRESENT = 1,
    .BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
    .BAT_CAPACITY = 50,
    .BAT_batt_vol = 0,
    .BAT_batt_temp = 0,
#endif
};

#if defined ( LGE_BSP_LGBM ) //                                              
#else
#if defined(CONFIG_POWER_EXT)
#else
static void mt6329_ac_update(struct mt6329_ac_data *ac_data)
{
    struct power_supply *ac_psy = &ac_data->psy;

    if( upmu_is_chr_det() == KAL_TRUE )
    {
        if ( (BMT_status.charger_type == NONSTANDARD_CHARGER) ||
             (BMT_status.charger_type == STANDARD_CHARGER)        )
        {
        /*                                                                  */
        #if defined(LGE_FW_OTP)
            #if 1
            if(otp_state == OTP_STOP_CHARGING_STATE || (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE && bat_volt_check_point == 0))
            #else
            if(otp_state == OTP_STOP_CHARGING_STATE)
            #endif
            {
                ac_data->AC_ONLINE = 0;
                ac_psy->type = POWER_SUPPLY_TYPE_BATTERY;
        }
            else
            {
                ac_data->AC_ONLINE = 1;
                ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
            }
        #else
            ac_data->AC_ONLINE = 1;
            ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
        #endif
        /*                                                                  */
        }
    }
    else
    {
        ac_data->AC_ONLINE = 0;
    }

    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[battery] ac_data->AC_ONLINE :%d, ac_psy->type : %d", ac_data->AC_ONLINE, ac_psy->type);

    power_supply_changed(ac_psy);
}

static void mt6329_usb_update(struct mt6329_usb_data *usb_data)
{
    struct power_supply *usb_psy = &usb_data->psy;

    if( upmu_is_chr_det() == KAL_TRUE )
    {
        if ( (BMT_status.charger_type == STANDARD_HOST) ||
             (BMT_status.charger_type == CHARGING_HOST)        )
        {
        /*                                                                  */
            usb_data->USB_ONLINE = 1;
        #if defined(LGE_FW_OTP)
#if 1  // to get power off sequence in OTP not or decrease charging
            if(otp_state == OTP_STOP_CHARGING_STATE || (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE && bat_volt_check_point == 0))
#else
            if(otp_state == OTP_STOP_CHARGING_STATE)
#endif
            {
                usb_psy->type = POWER_SUPPLY_TYPE_BATTERY;
                if(BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE && bat_volt_check_point == 0){
                    usb_data->USB_ONLINE = 0;
                }

        }
            else
            {
                usb_psy->type = POWER_SUPPLY_TYPE_USB;
            }
        #else
            usb_data->USB_ONLINE = 1;
            usb_psy->type = POWER_SUPPLY_TYPE_USB;
        #endif
        /*                                                                  */
        }
    }
    else
    {
        usb_data->USB_ONLINE = 0;
    }

    power_supply_changed(usb_psy);
}

//                                                                                    
kal_bool bootingFullCheck = KAL_FALSE;
extern kal_bool initChargingFull;
kal_bool one_percent_in_charing = KAL_FALSE;
//                                                                                    

static void mt6329_battery_update(struct mt6329_battery_data *bat_data)
{
    struct power_supply *bat_psy = &bat_data->psy;
    int i;

    bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
    bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
    bat_data->BAT_batt_vol = BMT_status.bat_vol;
    bat_data->BAT_batt_temp= BMT_status.temperature * 10;

    if (BMT_status.bat_exist)
        bat_data->BAT_PRESENT = 1;
    else
        bat_data->BAT_PRESENT = 0;

    /* Charger and Battery Exist */
    //if( (upmu_is_chr_det(CHR)==KAL_TRUE) && (!g_Battery_Fail) )
    if( (upmu_is_chr_det()==KAL_TRUE) && (!g_Battery_Fail) && (g_Charging_Over_Time==0))
    {
        if ( BMT_status.bat_exist )
        {
            /*                                                                  */
            #if defined(LGE_FW_OTP)
            if(otp_state != OTP_NORMAL_CHARGING_STATE  && BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE)
            {
                gSyncPercentage=1;
                bat_volt_check_point--;
                if(bat_volt_check_point <= 0)
                {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    bat_volt_check_point=0;
                }
                g_Calibration_FG = 0;
                FGADC_Reset_SW_Parameter();
                gFG_DOD0=100-bat_volt_check_point;
                gFG_DOD1=gFG_DOD0;
                BMT_status.SOC=bat_volt_check_point;
                bat_data->BAT_CAPACITY = bat_volt_check_point;

                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] No Charging By OTP, under System Off Voltage, bat_data->BAT_STATUS :%d, Bat_level(%d)\n", bat_data->BAT_STATUS, bat_volt_check_point);
            }
            else
            {
            #endif
            /*                                                                  */

            /* Battery Full */
#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if ( (BMT_status.bat_vol >= g_jeita_recharging_voltage) && (BMT_status.bat_full == KAL_TRUE) )
#else
            if ( (BMT_status.bat_vol >= RECHARGING_VOLTAGE) && (BMT_status.bat_full == KAL_TRUE) )
#endif
            {
                /*Use no gas gauge*/
                if( gForceADCsolution == 1 )
                {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
                    bat_data->BAT_CAPACITY = Battery_Percent_100;

                    /* For user view */
                    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                        batterySOCBuffer[i] = 100;
                        batterySOCSum = 100 * BATTERY_AVERAGE_SIZE; /* for user view */
                    }
                    bat_volt_check_point = 100;
                }
                /*Use gas gauge*/
                else
                {
                    gSyncPercentage=1;

#if defined(MTK_JEITA_STANDARD_SUPPORT)
                    //increase after xxs
                    if(gFGsyncTimer_jeita >= g_default_sync_time_out_jeita)
                    {
                        gFGsyncTimer_jeita=0;
                        bat_volt_check_point++;
                    }
                    else
                    {
                        gFGsyncTimer_jeita+=10;
                        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] In JEITA (%d on %d)\r\n",
                            bat_volt_check_point, gFGsyncTimer_jeita);
                    }
#else
                    bat_volt_check_point++;
#endif

                    if(bat_volt_check_point>=100)
                    {
                        bat_volt_check_point=100;
                        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
                    }
                    //                                                                                                 
                    if(otp_state == OTP_STOP_CHARGING_STATE || otp_state == OTP_DECREASE_STOP_CHARGING_STATE)
                    {
                        //bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
                        BMT_status.bat_full = KAL_FALSE;
                        g_bat_full_user_view = KAL_TRUE;
                        BMT_status.bat_charging_state = CHR_CC;
                        g_HW_Charging_Done = 0;
                    }
                    //                                                                                                 

                    bat_data->BAT_CAPACITY = bat_volt_check_point;

                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] In FULL Range (%d)\r\n", bat_volt_check_point);
                    }

                        gSyncPercentage=1;

                        if (Enable_BATDRV_LOG == 1) {
                            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_SyncRecharge] In recharging state, do not sync FG\r\n");
                        }
                }
            }
            /* battery charging */
            else
            {
                /* Do re-charging for keep battery soc */
                if (g_bat_full_user_view)
                {
                    //                                                                                                 
                    #if 1
                    if(otp_state == OTP_STOP_CHARGING_STATE)
                    {
                        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    }
                    else
                    {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
                    }
                    #else
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
                    #endif
                    //                                                                                                 
                    bat_data->BAT_CAPACITY = Battery_Percent_100;

                    /* For user view */
                    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                        batterySOCBuffer[i] = 100;
                        batterySOCSum = 100 * BATTERY_AVERAGE_SIZE; /* for user view */
                    }
                    bat_volt_check_point = 100;

                    gSyncPercentage=1;
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_Recharging] Keep UI as 100. bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n",
                        bat_volt_check_point, BMT_status.SOC);
                    }
                    //                                                                                                 
                    if(otp_state == OTP_DECREASE_STOP_CHARGING_STATE || otp_state == OTP_STOP_CHARGING_STATE)  // in stop charging state, for recharging, level down
                    {
                        g_bat_full_user_view = KAL_FALSE;
                    }
                    //                                                                                                 
                }
                //                                                                                    
                else if(bootingFullCheck == KAL_FALSE && initChargingFull == KAL_TRUE)
                {
                    g_bat_full_user_view = KAL_TRUE;
                }
                //                                                                                    
                else
                {
                //                                                                                                 
                #if 1
                    if(otp_state == OTP_STOP_CHARGING_STATE)  // to show not charging image
                    {
                        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    }
                    else
                    {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
                    }
                #else
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
                #endif
                //                                                                                                 

                    /*Use no gas gauge*/
                    if( gForceADCsolution == 1 )
                    {
                        /* SOC only UP when charging */
                        if ( BMT_status.SOC > bat_volt_check_point ) {
                            bat_volt_check_point = BMT_status.SOC;
                        }
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                    }
                    /*Use gas gauge*/
                    else
                    {
                        if(bat_volt_check_point >= 100 )
                        {
                            bat_volt_check_point=99;
                            //BMT_status.SOC=99;
                            gSyncPercentage=1;

                            //if (Enable_BATDRV_LOG == 1) {
                                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Use gas gauge : gas gague get 100 first (%d)\r\n", bat_volt_check_point);
                            //}
                        }
                        /*                                                                     */
                        else if(bat_volt_check_point < 1)
                        {
                            bat_volt_check_point = 1;
                            gSyncPercentage=1;
                            //                                                                                                               
                            one_percent_in_charing = KAL_TRUE;
                            //                                                                                                               
                        }
                        /*                                                                     */
                        else
                        {
                            if(bat_volt_check_point == BMT_status.SOC)
                            {
                                gSyncPercentage=0;

                                //                                                                                                               
                                if(bat_volt_check_point == 0 || bat_volt_check_point == 1) {
                                    one_percent_in_charing = KAL_TRUE;
                                }
                                else if(one_percent_in_charing == KAL_TRUE && bat_volt_check_point >= 2) {
                                    one_percent_in_charing = KAL_FALSE;
                                }
                                //                                                                                                               

                                if (Enable_BATDRV_LOG == 1) {
                                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] Can sync due to bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n",
                                    bat_volt_check_point, BMT_status.SOC);
                                }
                            }
                            else
                            {
                                //                                                                                                 
                                #if 1
                                if((bat_volt_check_point > BMT_status.SOC) &&(g_bat_full_user_view == KAL_FALSE) && (otp_state == OTP_DECREASE_STOP_CHARGING_STATE || otp_state == OTP_STOP_CHARGING_STATE))
                                {  // for UI is higher than SOC, but not charging state by OTP, battery level down
                                    //reduce after 3 mins
                                    if(gFGsyncTimer >= DEFAULT_SYNC_TIME_OUT*3)
                                    {
                                        gFGsyncTimer=0;
                                        bat_volt_check_point--;
                                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                                    }
                                    else
                                    {
                                        gFGsyncTimer+=10;
                                    }
                                    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Not Charging State by OTP, bat_volt_check_point=%d,BMT_status.SOC=%ld,gFGsyncTimer=%d(on %d)\r\n",
                                            bat_volt_check_point, BMT_status.SOC, gFGsyncTimer, DEFAULT_SYNC_TIME_OUT*3);
                                }
                                else
                                {
                                    if (Enable_BATDRV_LOG == 1) {
                                        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Keep UI due to bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n",
                                        bat_volt_check_point, BMT_status.SOC);
                                    }
                                }
                                #else
                                if (Enable_BATDRV_LOG == 1) {
                                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] Keep UI due to bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n",
                                    bat_volt_check_point, BMT_status.SOC);
                                }
                                #endif
                                //                                                                                                 
                            }
                        }
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                    }
                }
            }
            /*                                                                  */
            #if defined(LGE_FW_OTP)
            }
            #endif
            /*                                                                  */
        }
        /* No Battery, Only Charger */
        else
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
            bat_data->BAT_CAPACITY = 70;
            bat_data->BAT_batt_temp = 25 * 10;
        }

    }
    /* Only Battery */
    else
    {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;

        /* If VBAT < CLV, then shutdown */
        if (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE)
        {
            /*Use no gas gauge*/
            if( gForceADCsolution == 1 )
            {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT BATTERY] VBAT < %d mV : Android will Power Off System !!\r\n", SYSTEM_OFF_VOLTAGE);
                bat_data->BAT_CAPACITY = 0;
            }
            /*Use gas gauge*/
            else
            {
                gSyncPercentage=1;
                bat_volt_check_point--;
                if(bat_volt_check_point <= 0)
                {
                //                                                                             
                #if 1
                    bat_volt_check_point=1;
                #else
                    bat_volt_check_point=0;
                #endif
                //                                                                             
                }
                g_Calibration_FG = 0;
                FGADC_Reset_SW_Parameter();
                gFG_DOD0=100-bat_volt_check_point;
                gFG_DOD1=gFG_DOD0;
                BMT_status.SOC=bat_volt_check_point;
                bat_data->BAT_CAPACITY = bat_volt_check_point;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] VBAT < %d mV (%d, gFG_DOD0=%d)\r\n", SYSTEM_OFF_VOLTAGE, bat_volt_check_point,gFG_DOD0);
            }
        }
        /* If FG_VBAT <= gFG_15_vlot, then run to 15% */
        else if ( (gFG_voltage <= gFG_15_vlot)&&(gForceADCsolution==0)&&(bat_volt_check_point>=15) )
        {
            /*Use gas gauge*/
            gSyncPercentage=1;
            if(gBAT_counter_15==0)
            {
                bat_volt_check_point--;
                gBAT_counter_15=1;
            }
            else
            {
                gBAT_counter_15=0;
            }
            g_Calibration_FG = 0;
            FGADC_Reset_SW_Parameter();
            gFG_DOD0=100-bat_volt_check_point;
            gFG_DOD1=gFG_DOD0;
            BMT_status.SOC=bat_volt_check_point;
            bat_data->BAT_CAPACITY = bat_volt_check_point;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] FG_VBAT <= %d, then SOC run to 15. (SOC=%ld,Point=%d,D1=%d,D0=%d)\r\n",
                gFG_15_vlot, BMT_status.SOC, bat_volt_check_point, gFG_DOD1, gFG_DOD0);
        }
        /* If "FG_VBAT > gFG_15_vlot" and "FG_report=15%" , then keep 15% till FG_VBAT <= gFG_15_vlot */
        else if ( (gFG_voltage > gFG_15_vlot)&&(gForceADCsolution==0)&&(bat_volt_check_point==15) )
        {
            /*Use gas gauge*/
            gSyncPercentage=1;
            gBAT_counter_15=1;
            g_Calibration_FG = 0;
            FGADC_Reset_SW_Parameter();
            gFG_DOD0=100-bat_volt_check_point;
            gFG_DOD1=gFG_DOD0;
            BMT_status.SOC=bat_volt_check_point;
            bat_data->BAT_CAPACITY = bat_volt_check_point;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] FG_VBAT(%d) > gFG_15_vlot(%d) and FG_report=15, then UI(%d) keep 15. (D1=%d,D0=%d)\r\n",
                gFG_voltage, gFG_15_vlot, bat_volt_check_point, gFG_DOD1, gFG_DOD0);
        }
        else
        {
            gBAT_counter_15=1;
            /*Use no gas gauge*/
            if( gForceADCsolution == 1 )
            {
                /* SOC only Done when dis-charging */
                if ( BMT_status.SOC < bat_volt_check_point ) {
                    bat_volt_check_point = BMT_status.SOC;
                }
                bat_data->BAT_CAPACITY = bat_volt_check_point;
            }
            /*Use gas gauge : gas gague get 0% fist*/
            else
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_OnlyBattery!] bat_volt_check_point=%d,BMT_status.SOC=%ld\r\n",
                    bat_volt_check_point, BMT_status.SOC);
                }

                //If current display level is higher than gFG_capacity_by_v, we will decrease it every 30s.
                if( (bat_volt_check_point > gFG_capacity_by_v) && ((bat_volt_check_point >= 3)) && (gFG_capacity_by_v <= 10) &&(gForceADCsolution==0))
                {
                    //reduce after 30s
                    if(gFGsyncTimer >= 30)
                    {
                        gFGsyncTimer=0;

                        if(bat_volt_check_point >= 3) {
                            bat_volt_check_point--;
                        }

                        /*Update Fuel gauge value.*/
                        g_Calibration_FG = 0;
                        FGADC_Reset_SW_Parameter();
                        gFG_DOD0=100-bat_volt_check_point;
                        gFG_DOD1=gFG_DOD0;
                        BMT_status.SOC=bat_volt_check_point;
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] gFG_capacity_by_v renew. check_point = %d, capacity_by_v = %d\r\n", bat_volt_check_point,gFG_capacity_by_v);
                    }
                    else
                    {
                        gFGsyncTimer+=10;
                    }
                }
                else if( (bat_volt_check_point>BMT_status.SOC) && ((bat_volt_check_point>2)) )
                {
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery_OnlyBattery] bat_volt_check_point=%d,BMT_status.SOC=%ld,gFGsyncTimer=%d(on %d)\r\n",
                        bat_volt_check_point, BMT_status.SOC, gFGsyncTimer, DEFAULT_SYNC_TIME_OUT);
                    }

                    //reduce after xxs
                    if(gFGsyncTimer >= DEFAULT_SYNC_TIME_OUT)
                    {
                        gFGsyncTimer=0;
                        bat_volt_check_point--;
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                    }
                    else
                    {
                        gFGsyncTimer+=10;
                    }
                }
                else
                {
                    //                                                                                                          
                    if(bat_volt_check_point <= 1 )
                    {
                    //                                                                                                               
                    #if 1
                        if(one_percent_in_charing == KAL_TRUE) {
                            bat_volt_check_point = 1;
                        }
                        else {
                        bat_volt_check_point=2;
                        }

                    #else
                        bat_volt_check_point=2;
                    #endif
                    //                                                                                                               
                        //BMT_status.SOC=1;
                        gSyncPercentage=1;

                        //if (Enable_BATDRV_LOG == 1) {
                            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Use gas gauge : gas gague get 0 first (%d)\r\n", bat_volt_check_point);
                        //}
                    }
                    //                                                                                                          
                    else
                    {
                        gSyncPercentage=0;
                    }

                    if(bat_volt_check_point > 100)
                    {
                        bat_volt_check_point=100;
                    }

                    bat_data->BAT_CAPACITY = bat_volt_check_point;
                }

                if(bat_volt_check_point == 100) {
                    g_bat_full_user_view = KAL_TRUE;
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_Only] Set g_bat_full_user_view=KAL_TRUE\r\n");
                    }
                }
            }
        }
    }

        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:IntegrationFG:point,per_C,per_V,count,vbat_charger,CSDAC_DAT] %d,%ld,%d,%d,%ld,ADC_Solution=%d\r\n",
        bat_volt_check_point, BMT_status.SOC, FGADC_Get_BatteryCapacity_VoltageMothod(),
        BATTERY_AVERAGE_SIZE, BMT_status.bat_vol, gForceADCsolution);


    //                                                                                    
    if(bootingFullCheck == KAL_FALSE)
    {
        bootingFullCheck = KAL_TRUE;
    }
    //                                                                                    

    /* Update for EM */
    bat_data->BAT_TemperatureR=g_BAT_TemperatureR;
    bat_data->BAT_TempBattVoltage=g_TempBattVoltage;
    bat_data->BAT_InstatVolt=g_InstatVolt;
    bat_data->BAT_BatteryAverageCurrent=g_BatteryAverageCurrent;
    bat_data->BAT_BatterySenseVoltage=g_BAT_BatterySenseVoltage;
    bat_data->BAT_ISenseVoltage=g_BAT_ISenseVoltage;
    bat_data->BAT_ChargerVoltage=g_BAT_ChargerVoltage;

//                                                                              
#if 1
if( bat_soc_tracker == 15 && bat_data->BAT_CAPACITY >= 15 )
{
    /* do not save */
}
else
{
    if ( bat_soc_tracker != bat_data->BAT_CAPACITY )
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Saving check point bat_soc_tracker = %d, user BAT_SOC=%d \r\n", bat_soc_tracker, bat_data->BAT_CAPACITY);
        if( bat_data->BAT_CAPACITY >= 15 )
        {
            bat_soc_tracker =  15;
        }
        else
        {
            if(bat_data->BAT_CAPACITY <= 0)
            {
                bat_soc_tracker = 1;
            }
            else
            {
                bat_soc_tracker =  bat_data->BAT_CAPACITY;
            }
        }

        /*Customer save point every soc update.*/
        battery_info_write(bat_soc_tracker);
    }
}
#endif
//                                                                              

    power_supply_changed(bat_psy);
}

static void mt6329_battery_update_power_down(struct mt6329_battery_data *bat_data)
{
    struct power_supply *bat_psy = &bat_data->psy;

    bat_data->BAT_CAPACITY = 0;

    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] mt6329_battery_update_power_down\r\n");

    power_supply_changed(bat_psy);
}
#endif
#endif //                                              

#if defined(CONFIG_POWER_VERIFY)
//
void charger_hv_detect_sw_workaround_init(void)
{
}
#else
///////////////////////////////////////////////////////////////////////////////////////////
//// Battery Temprature Parameters and functions
///////////////////////////////////////////////////////////////////////////////////////////
#if defined ( LGE_BSP_LGBM ) //                                              
unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
    unsigned long ret_val = 0;

    if(polling_mode == 1)
        g_switch_to_i2c_polling_mode=1;

    if(polling_mode == 1)
        ret_val=PMIC_IMM_GetOneChannelValueSleep(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    else
        ret_val=PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);

    if(polling_mode == 1)
        g_switch_to_i2c_polling_mode=0;

    return ret_val;
}

void BATTERY_SetUSBState(int usb_state_value)
{
    if ( (usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))){
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
        usb_state_value = USB_UNCONFIGURED;
    } else {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAT_SetUSBState Success! Set %d\r\n", usb_state_value);
        g_usb_state = usb_state_value;
    }
}
EXPORT_SYMBOL(BATTERY_SetUSBState);

kal_bool pmic_chrdet_status(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )
    {
        return KAL_TRUE;
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[pmic_chrdet_status] No charger\r\n");
        return KAL_FALSE;
    }
}
#endif //                                              

#if defined ( LGE_BSP_LGBM )

void LGBM_PrintBmOTPState ( LGBmOTPState bmOTPState )
{
    switch( bmOTPState )
    {
        case LGBM_OTP_NORMAL_CHARGING:
            LGBM_LOG("OTP_STATE = LGBM_OTP_NORMAL_CHARGING\n");
            break;
        case LGBM_OTP_DECREASE_CHARGING:
            LGBM_LOG("OTP_STATE = LGBM_OTP_DECREASE_CHARGING\n");
            break;
        case LGBM_OTP_STOP_CHARGING:
            LGBM_LOG("OTP_STATE = LGBM_OTP_STOP_CHARGING\n");
            break;
        default:
            LGBM_ERR("Invalid OTP_STATE ( %d )\n", bmOTPState);
            break;
    }

}

int LGBM_GetLinearInterpolation(int x1, int x2, int y1, int y2, int x )
{
    int retVal = 0;

    if( x1 == x2 )
    {
        if( y1 == y2 )
        {
            retVal = y1;
        }
        else
        {
            LGBM_ERR("Can't interpolate ( x1=%d, x2=%d, y1=%d, y2=%d )\n", x1, x2, y1, y2 );
        }
    }
    else
    {
        retVal = (y2-y1)*(x-x1)/(x2-x1)+y1;
    }

    return retVal;

}

kal_uint32 LGBM_ReadPmicEcoVersion ( void )
{
    kal_uint32 regVal = 0;
    kal_uint32 ecoVersion = 0;

    /* check PMIC chip version */
    regVal = upmu_get_cid0();
    ecoVersion |= regVal;
    regVal = upmu_get_cid1();
    ecoVersion |= (regVal << 8);

    if (ecoVersion == PMIC6329_E1_CID_CODE)
    {
        LGBM_LOG("PMIC Version = E1\n");
        upmu_chr_vcdt_lv_vth(0); /* code from MTK original */
    }
    else
    {
        LGBM_LOG("PMIC Version > E1\n");
    }

    get_pmic_flag = 1;

    return ecoVersion;

}

kal_int32 LGBM_ReadFuelGaugeTrimFactor( void )
{
    kal_int32 regVal = 0;
    kal_int32 trimIndex = 0;
    kal_int32 trimFactor = 1000; /* 1000 means no effect */

    LGBM_ENTRY();

    regVal = upmu_get_reg_value_bank1(0x8C);
    trimIndex = (regVal & 0x3E) >> 1;

    if ( ( regVal & 0x01 ) == 0 ) /* trim value is available */
    {
        switch(trimIndex)
        {
            case 0:    trimFactor = 1000; break;
            case 1:    trimFactor = 1005; break;
            case 2:    trimFactor = 1010; break;
            case 3:    trimFactor = 1015; break;
            case 4:    trimFactor = 1020; break;
            case 5:    trimFactor = 1025; break;
            case 6:    trimFactor = 1030; break;
            case 7:    trimFactor = 1035; break;
            case 8:    trimFactor = 1040; break;
            case 9:    trimFactor = 1045; break;
            case 10:   trimFactor = 1050; break;
            case 11:   trimFactor = 1055; break;
            case 12:   trimFactor = 1060; break;
            case 13:   trimFactor = 1065; break;
            case 14:   trimFactor = 1070; break;
            case 15:   trimFactor = 1075; break;
            case 31:   trimFactor = 995; break;
            case 30:   trimFactor = 990; break;
            case 29:   trimFactor = 985; break;
            case 28:   trimFactor = 980; break;
            case 27:   trimFactor = 975; break;
            case 26:   trimFactor = 970; break;
            case 25:   trimFactor = 965; break;
            case 24:   trimFactor = 960; break;
            case 23:   trimFactor = 955; break;
            case 22:   trimFactor = 950; break;
            case 21:   trimFactor = 945; break;
            case 20:   trimFactor = 940; break;
            case 19:   trimFactor = 935; break;
            case 18:   trimFactor = 930; break;
            case 17:   trimFactor = 925; break;
            default:
                LGBM_ERR("Invalid Trim Index ( %d )\n", trimIndex);
                break;
        }
    }
    else
    {
        LGBM_ERR("Trim is not available on this chip\n");
    }

    LGBM_LOG("Fuel Gauge Trim Factor = %d\n", trimFactor);

    LGBM_EXIT();

    return trimFactor ;

}

kal_int32 LGBM_GetTrimmedValue(kal_int32 oriValue)
{
    kal_int32 trimmedValue = 0;

    //LGBM_ENTRY();

    trimmedValue = ((oriValue*g_FuelGaugeTrimFactor)/1000);

    //LGBM_DBG("oriValue = %d, trimmedValue = %d\n", oriValue, trimmedValue);

    return trimmedValue;
}

kal_int32 LGBM_GetSocByOcv ( int batId, int batTemp, int batOcv )
{
    BATTERY_PROFILE_STRUC *pHighTempTbl = NULL;
    BATTERY_PROFILE_STRUC *pLowTempTbl = NULL;
    kal_int32 highTempBatDod = 0;
    kal_int32 lowTempBatDod = 0;
    kal_int32 highTempTblSize = 0;
    kal_int32 lowTempTblSize = 0;
    kal_int32 highTemp = 0;
    kal_int32 lowTemp = 0;
    kal_int32 batDod = 0;
    kal_int32 batSoc = 0;

    int index = 0;
    int x1=0, x2=0, y1=0, y2=0;

    LGBM_ENTRY();

    /* make sure that the temperature value is on the proper range */
    if( batTemp < LGBM_TEMPERATURE_M10 )
    {
        batTemp = LGBM_TEMPERATURE_M10;
    }
    else if ( batTemp > LGBM_TEMPERATURE_50 )
    {
        batTemp = LGBM_TEMPERATURE_50;
    }

    /* select table to use */
    if( batId == 1 )
    {
        if( batTemp < LGBM_TEMPERATURE_0 )
        {
            highTemp = LGBM_TEMPERATURE_0;
            lowTemp = LGBM_TEMPERATURE_M10;
            pHighTempTbl = battery_profile_t1;
            pLowTempTbl = battery_profile_t0;
            highTempTblSize = sizeof(battery_profile_t1)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_t0)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else if( batTemp < LGBM_TEMPERATURE_25 )
        {
            highTemp = LGBM_TEMPERATURE_25;
            lowTemp = LGBM_TEMPERATURE_0;
            pHighTempTbl = battery_profile_t2;
            pLowTempTbl = battery_profile_t1;
            highTempTblSize = sizeof(battery_profile_t2)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_t1)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else
        {
            highTemp = LGBM_TEMPERATURE_50;
            lowTemp = LGBM_TEMPERATURE_25;
            pHighTempTbl = battery_profile_t3;
            pLowTempTbl = battery_profile_t2;
            highTempTblSize = sizeof(battery_profile_t3)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_t2)/sizeof(BATTERY_PROFILE_STRUC);
        }
    }
    else
    {
        if( batTemp < LGBM_TEMPERATURE_0 )
        {
            highTemp = LGBM_TEMPERATURE_0;
            lowTemp = LGBM_TEMPERATURE_M10;
            pHighTempTbl = battery_profile_2nd_t1;
            pLowTempTbl = battery_profile_2nd_t0;
            highTempTblSize = sizeof(battery_profile_2nd_t1)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_2nd_t0)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else if( batTemp < LGBM_TEMPERATURE_25 )
        {
            highTemp = LGBM_TEMPERATURE_25;
            lowTemp = LGBM_TEMPERATURE_0;
            pHighTempTbl = battery_profile_2nd_t2;
            pLowTempTbl = battery_profile_2nd_t1;
            highTempTblSize = sizeof(battery_profile_2nd_t2)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_2nd_t1)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else
        {
            highTemp = LGBM_TEMPERATURE_50;
            lowTemp = LGBM_TEMPERATURE_25;
            pHighTempTbl = battery_profile_2nd_t3;
            pLowTempTbl = battery_profile_2nd_t2;
            highTempTblSize = sizeof(battery_profile_2nd_t3)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_2nd_t2)/sizeof(BATTERY_PROFILE_STRUC);
        }
    }

    /* get interpolated value of battery resistance at high temperature */
    if( batOcv > pHighTempTbl[0].voltage )
    {
        highTempBatDod = pHighTempTbl[0].percentage;
    }
    else
    {
        for ( index = 1 ; index < highTempTblSize ; index++ )
        {
            if( batOcv > pHighTempTbl[index].voltage )
            {
                x1 = pHighTempTbl[index].voltage;
                x2 = pHighTempTbl[index-1].voltage;
                y1 = pHighTempTbl[index].percentage;
                y2 = pHighTempTbl[index-1].percentage;
                highTempBatDod = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batOcv);
                //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, batOcv, highTempBatDod);
                break;
            }
        }

        if ( index == highTempTblSize )
        {
            highTempBatDod = pHighTempTbl[highTempTblSize-1].percentage;
        }
    }

    if( batOcv > pLowTempTbl[0].voltage )
    {
        lowTempBatDod = pLowTempTbl[0].percentage;
    }
    else
    {
        for ( index = 1 ; index < lowTempTblSize ; index++ )
        {
            if( batOcv > pLowTempTbl[index].voltage )
            {
                x1 = pLowTempTbl[index].voltage;
                x2 = pLowTempTbl[index-1].voltage;
                y1 = pLowTempTbl[index].percentage;
                y2 = pLowTempTbl[index-1].percentage;
                lowTempBatDod = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batOcv);
                //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, batOcv, lowTempBatDod);
                break;
            }
        }

        if ( index == lowTempTblSize )
        {
            lowTempBatDod = pLowTempTbl[lowTempTblSize-1].percentage;
        }
    }


    /* get interpolated value of battery resistance at low temperature */

    x1 = highTemp;
    x2 = lowTemp;
    y1 = highTempBatDod;
    y2 = lowTempBatDod;
    batDod = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batTemp);

    if( batDod < 0 )
    {
        batSoc = 100;
    }
    else if( batDod > 99 )
    {
        batSoc = 1;
    }
    else
    {
        batSoc = 100 - batDod;
    }

    //LGBM_DBG("highTemp=%d, highTempBatDod=%d, lowTemp=%d, lowTempBatDod=%d, batTemp=%d, batDod=%d, batSoc=%d\n", highTemp, highTempBatDod, lowTemp, lowTempBatDod, batTemp, batDod, batSoc);

    LGBM_LOG("Battery SOC by OCV = %d[%]\n", batSoc);

    LGBM_EXIT();

    return batSoc;

}

int LGBM_ReadChgVoltAdc ( void )
{
    static int lastAdcValue = -99;
    int adcValue = 0;

    LGBM_ENTRY();

    adcValue = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL, 5); /* unit : mV */

    //LGBM_DBG("Charger ADC = %d[mV]\n", adcValue);

    if( adcValue <= 0 )
    {
        LGBM_ERR("Battery ADC is out of range ( %d ), so use previous value\n", adcValue);
        adcValue = lastAdcValue;
    }
    else
    {
        lastAdcValue = adcValue;
    }

    LGBM_EXIT();

    return adcValue;

}

int LGBM_ReadBatVoltAdc ( void )
{
    static int lastAdcValue = -99;
    int adcValue = 0;

    LGBM_ENTRY();

    adcValue = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL, 5); /* unit : mV */

    //LGBM_DBG("Battery ADC = %d[mV]\n", adcValue);

    if( adcValue <= 0 )
    {
        LGBM_ERR("Battery ADC is out of range ( %d ), so use previous value\n", adcValue);
        adcValue = lastAdcValue;
    }
    else
    {
        lastAdcValue = adcValue;
    }

    LGBM_EXIT();

    return adcValue;

}

int LGBM_ReadBatTempAdc ( void )
{
    static int lastAdcValue = -99;
    int adcValue = 0;

    LGBM_ENTRY();

    adcValue = PMIC_IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL, 5); /* unit : mV */

    //LGBM_DBG("Battery Temp ADC = %d[mV]\n", adcValue);

    if( adcValue <= 0 )
    {
        LGBM_ERR("Battery Temperature ADC is out of range ( %d ), so use previous value\n", adcValue);
        adcValue = lastAdcValue;
    }
    else
    {
        lastAdcValue = adcValue;
    }

    LGBM_EXIT();

    return adcValue;

}

int LGBM_ReadCumulatedColumb ( void )
{
    kal_uint32 regVal = 0;
    kal_uint32 regVal_High = 0;
    kal_int16 tmpRegVal = 0;
    kal_int32 readColumb = 0;
    kal_int32 tmpColumb = 0;
    kal_int32 compensatedColumb = 0;

    int m = 0;
    kal_uint32 ret = 0;

    LGBM_ENTRY();

    // HW Init // Enable VA2
    ret=pmic_config_interface(0xC8, 0x1, 0xFF, 0x0);

    //Read HW Raw Data // Set READ command
    ret=pmic_bank1_config_interface(0x6A, 0x02, 0xFF, 0x0);

    // Keep i2c read when status = 1 (0x06)
    m=0;
    while ( fg_get_data_ready_status() == 0 )
    {
        m++;
        if(m>1000)
        {
            break;
        }
    }

    // Read FG_CURRENT_OUT
    regVal = (upmu_fgadc_car_15_08())>>6;
    regVal |= (upmu_fgadc_car_23_16())<<2;
    regVal |= (upmu_fgadc_car_31_24())<<10;
    regVal_High = (upmu_fgadc_car_35_32() & 0x0F)>>3;

    // Clear status to 0
    ret=pmic_bank1_config_interface(0x6A, 0x08, 0xFF, 0x0);

    // Keep i2c read when status = 0 (0x08)
    m=0;
    while ( fg_get_data_ready_status() != 0 )
    {
        m++;
        if(m>1000)
        {
            break;
        }
    }

    // Recover original settings
    ret=pmic_bank1_config_interface(0x6A, 0x00, 0xFF, 0x0);

    tmpRegVal = (kal_int16)(regVal & 0xFFFF);

    //calculate thereal world data
    readColumb = ( ((tmpRegVal*35986)/10) + (5) )/10; //[28:14]'s LSB=359.86 uAh
    readColumb = readColumb / 1000; //mAh

    tmpColumb = ((readColumb*CAR_TUNE_VALUE)/100);

    compensatedColumb = LGBM_GetTrimmedValue(tmpColumb);

    //LGBM_DBG("regVal = %d, readColumb = %d[mAh], compensatedColumb = %d[mAh]\n", (kal_int16)(regVal & 0xFFFF), readColumb, compensatedColumb);

    LGBM_EXIT();

    return compensatedColumb;

}

int LGBM_ReadBatCurrent ( void )
{
    int m = 0;
    kal_uint32 ret = 0;

    LGBM_ENTRY();

    kal_int16 regVal = 0;
    kal_int32 readCurrent = 0;
    kal_int32 tmpCurrent = 0;
    kal_int32 compensatedCurrent = 0;

    // HW Init // Enable VA2
    ret=pmic_config_interface(0xC8, 0x1, 0xFF, 0x0);

    //Read HW Raw Data  // Set READ command
    ret=pmic_bank1_config_interface(0x6A, 0x02, 0xFF, 0x0);

    // Keep i2c read when status = 1 (0x06)
    m=0;
    while ( fg_get_data_ready_status() == 0 )
    {
        m++;
        if(m>1000)
        {
            break;
        }
    }

    // Read FG_CURRENT_OUT
    regVal = upmu_fgadc_current_out_07_00();
    regVal |= (upmu_fgadc_current_out_15_08())<<8;

    // Clear status to 0
    ret=pmic_bank1_config_interface(0x6A, 0x08, 0xFF, 0x0);

    // Keep i2c read when status = 0 (0x08)
    m=0;
    while ( fg_get_data_ready_status() != 0 )
    {
        m++;
        if(m>1000)
        {
            break;
        }
    }
    // Recover original settings
    ret=pmic_bank1_config_interface(0x6A, 0x00, 0xFF, 0x0);

    readCurrent = (kal_int32) ((regVal * UNIT_FGCURRENT) / 100000);

    tmpCurrent = ((readCurrent*CAR_TUNE_VALUE)/100);

    compensatedCurrent = LGBM_GetTrimmedValue(tmpCurrent);

    /* unit convertion from mini-Amp to milli-Amp */
    compensatedCurrent = compensatedCurrent/10;

    /* if negative value of current means dis-charging current */
    //LGBM_DBG("regVal = %d, readCurrent = %d[mA], compensatedCurrent = %d[mA]\n", regVal, readCurrent/10, compensatedCurrent);

    LGBM_EXIT();

    return compensatedCurrent;

}

kal_int32 LGBM_GetBatVoltFromSoc ( int batId, int batTemp, int batSoc )
{
    BATTERY_PROFILE_STRUC *pHighTempTbl = NULL;
    BATTERY_PROFILE_STRUC *pLowTempTbl = NULL;
    kal_int32 highTempBatVolt = 0;
    kal_int32 lowTempBatVolt = 0;
    kal_int32 highTempTblSize = 0;
    kal_int32 lowTempTblSize = 0;
    kal_int32 highTemp = 0;
    kal_int32 lowTemp = 0;
    kal_int32 batVolt = 0;
    kal_int32 batDod = 100 - batSoc;

    int index = 0;
    int x1=0, x2=0, y1=0, y2=0;

    //LGBM_ENTRY();

    /* make sure that the temperature value is on the proper range */
    if( batTemp < LGBM_TEMPERATURE_M10 )
    {
        batTemp = LGBM_TEMPERATURE_M10;
    }
    else if ( batTemp > LGBM_TEMPERATURE_50 )
    {
        batTemp = LGBM_TEMPERATURE_50;
    }

    /* select table to use */
    if( batId == 1 )
    {
        if( batTemp < LGBM_TEMPERATURE_0 )
        {
            highTemp = LGBM_TEMPERATURE_0;
            lowTemp = LGBM_TEMPERATURE_M10;
            pHighTempTbl = battery_profile_t1;
            pLowTempTbl = battery_profile_t0;
            highTempTblSize = sizeof(battery_profile_t1)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_t0)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else if( batTemp < LGBM_TEMPERATURE_25 )
        {
            highTemp = LGBM_TEMPERATURE_25;
            lowTemp = LGBM_TEMPERATURE_0;
            pHighTempTbl = battery_profile_t2;
            pLowTempTbl = battery_profile_t1;
            highTempTblSize = sizeof(battery_profile_t2)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_t1)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else
        {
            highTemp = LGBM_TEMPERATURE_50;
            lowTemp = LGBM_TEMPERATURE_25;
            pHighTempTbl = battery_profile_t3;
            pLowTempTbl = battery_profile_t2;
            highTempTblSize = sizeof(battery_profile_t3)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_t2)/sizeof(BATTERY_PROFILE_STRUC);
        }
    }
    else
    {
        if( batTemp < LGBM_TEMPERATURE_0 )
        {
            highTemp = LGBM_TEMPERATURE_0;
            lowTemp = LGBM_TEMPERATURE_M10;
            pHighTempTbl = battery_profile_2nd_t1;
            pLowTempTbl = battery_profile_2nd_t0;
            highTempTblSize = sizeof(battery_profile_2nd_t1)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_2nd_t0)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else if( batTemp < LGBM_TEMPERATURE_25 )
        {
            highTemp = LGBM_TEMPERATURE_25;
            lowTemp = LGBM_TEMPERATURE_0;
            pHighTempTbl = battery_profile_2nd_t2;
            pLowTempTbl = battery_profile_2nd_t1;
            highTempTblSize = sizeof(battery_profile_2nd_t2)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_2nd_t1)/sizeof(BATTERY_PROFILE_STRUC);
        }
        else
        {
            highTemp = LGBM_TEMPERATURE_50;
            lowTemp = LGBM_TEMPERATURE_25;
            pHighTempTbl = battery_profile_2nd_t3;
            pLowTempTbl = battery_profile_2nd_t2;
            highTempTblSize = sizeof(battery_profile_2nd_t3)/sizeof(BATTERY_PROFILE_STRUC);
            lowTempTblSize = sizeof(battery_profile_2nd_t2)/sizeof(BATTERY_PROFILE_STRUC);
        }
    }

    /* get interpolated value of battery resistance at high temperature */
    if( batDod < pHighTempTbl[0].percentage )
    {
        highTempBatVolt = pHighTempTbl[0].voltage;
    }
    else
    {
        for ( index = 1 ; index < highTempTblSize ; index++ )
        {
            if( batDod < pHighTempTbl[index].percentage )
            {
                x1 = pHighTempTbl[index].percentage;
                x2 = pHighTempTbl[index-1].percentage;
                y1 = pHighTempTbl[index].voltage;
                y2 = pHighTempTbl[index-1].voltage;
                highTempBatVolt = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batDod);
                //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, batDod, highTempBatVolt);
                break;
            }
        }

        if ( index == highTempTblSize )
        {
            highTempBatVolt = pHighTempTbl[highTempTblSize-1].voltage;
        }
    }

    if( batDod < pLowTempTbl[0].percentage )
    {
        lowTempBatVolt = pLowTempTbl[0].voltage;
    }
    else
    {
        for ( index = 1 ; index < lowTempTblSize ; index++ )
        {
            if( batDod < pLowTempTbl[index].percentage )
            {
                x1 = pLowTempTbl[index].percentage;
                x2 = pLowTempTbl[index-1].percentage;
                y1 = pLowTempTbl[index].voltage;
                y2 = pLowTempTbl[index-1].voltage;
                lowTempBatVolt = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batDod);
                //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, batDod, lowTempBatVolt);
                break;
            }
        }

        if ( index == lowTempTblSize )
        {
            lowTempBatVolt = pLowTempTbl[lowTempTblSize-1].voltage;
        }
    }


    /* get interpolated value of battery resistance at low temperature */

    x1 = highTemp;
    x2 = lowTemp;
    y1 = highTempBatVolt;
    y2 = lowTempBatVolt;
    batVolt = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batTemp);

    //LGBM_DBG("highTemp=%d, highTempBatVolt=%d, lowTemp=%d, lowTempBatVolt=%d, batTemp=%d, batVolt=%d\n", highTemp, highTempBatVolt, lowTemp, lowTempBatVolt, batTemp, batVolt);

    return batVolt;

}

int LGBM_GetBatTemp ( int batId, int batTempAdc, int batCurrent )
{
    int tmpAdc = 0;
    int batTempVolt = 0;
    int batTemp = 0;

    LGBM_ENTRY();

    tmpAdc = batTempAdc;

    if( tmpAdc < 0 )
    {
        tmpAdc = 0;
    }

    batTempVolt = tmpAdc - ((batCurrent*g_FuelGaugeResistance)/1000);

    //batTemp = LGBM_GetTemperatureFromVoltage(batId, batTempVolt);
    {
        BATT_TEMPERATURE *pTable = NULL;
        int tableSize = 0;
        int index = 0;
        int thermistorValue = 0;
        //int batTemp = 0;
        int x1=0, x2=0, y1=0, y2=0;

        //LGBM_ENTRY();

        if( batId == 1 )
        {
            pTable = lgbmBatTempTbl;
            tableSize = sizeof(lgbmBatTempTbl)/sizeof(BATT_TEMPERATURE);
        }
        else
        {
            pTable = lgbmBatTempTbl;
            tableSize = sizeof(lgbmBatTempTbl)/sizeof(BATT_TEMPERATURE);
        }

        thermistorValue = ( batTempVolt * RBAT_PULL_UP_R ) / ( RBAT_PULL_UP_VOLT - batTempVolt ) ;

        g_BAT_TemperatureR = thermistorValue; /* TBD */

        if( thermistorValue > pTable[0].TemperatureR )
        {
            batTemp = pTable[0].BatteryTemp;
        }
        else
        {
            for ( index = 1 ; index < tableSize ; index++ )
            {
                if( thermistorValue > pTable[index].TemperatureR )
                {
                    x1 = pTable[index].TemperatureR;
                    x2 = pTable[index-1].TemperatureR;
                    y1 = pTable[index].BatteryTemp;
                    y2 = pTable[index-1].BatteryTemp;
                    batTemp = LGBM_GetLinearInterpolation(x1, x2, y1, y2, thermistorValue);
                    //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, thermistorValue, batTemp);
                    break;
                }
            }

            if ( index == tableSize )
            {
                batTemp = pTable[tableSize-1].BatteryTemp;
            }
        }

    }

    LGBM_EXIT();

    return batTemp;

}

kal_int32 LGBM_GetBatResistance ( int batId, int batTemp, int batVolt )
{
    R_PROFILE_STRUC *pHighTempTbl = NULL;
    R_PROFILE_STRUC *pLowTempTbl = NULL;
    kal_int32 highTempBatRes = 0;
    kal_int32 lowTempBatRes = 0;
    kal_int32 highTempTblSize = 0;
    kal_int32 lowTempTblSize = 0;
    kal_int32 highTemp = 0;
    kal_int32 lowTemp = 0;
    kal_int32 batRes = 0;

    int index = 0;
    int x1=0, x2=0, y1=0, y2=0;

    //LGBM_ENTRY();

    /* make sure that the temperature value is on the proper range */
    if( batTemp < LGBM_TEMPERATURE_M10 )
    {
        batTemp = LGBM_TEMPERATURE_M10;
    }
    else if ( batTemp > LGBM_TEMPERATURE_50 )
    {
        batTemp = LGBM_TEMPERATURE_50;
    }

    /* select table to use */
    if( batId == 1 )
    {
        if( batTemp < LGBM_TEMPERATURE_0 )
        {
            highTemp = LGBM_TEMPERATURE_0;
            lowTemp = LGBM_TEMPERATURE_M10;
            pHighTempTbl = r_profile_t1;
            pLowTempTbl = r_profile_t0;
            highTempTblSize = sizeof(r_profile_t1)/sizeof(R_PROFILE_STRUC);
            lowTempTblSize = sizeof(r_profile_t0)/sizeof(R_PROFILE_STRUC);
        }
        else if( batTemp < LGBM_TEMPERATURE_25 )
        {
            highTemp = LGBM_TEMPERATURE_25;
            lowTemp = LGBM_TEMPERATURE_0;
            pHighTempTbl = r_profile_t2;
            pLowTempTbl = r_profile_t1;
            highTempTblSize = sizeof(r_profile_t2)/sizeof(R_PROFILE_STRUC);
            lowTempTblSize = sizeof(r_profile_t1)/sizeof(R_PROFILE_STRUC);
        }
        else
        {
            highTemp = LGBM_TEMPERATURE_50;
            lowTemp = LGBM_TEMPERATURE_25;
            pHighTempTbl = r_profile_t3;
            pLowTempTbl = r_profile_t2;
            highTempTblSize = sizeof(r_profile_t3)/sizeof(R_PROFILE_STRUC);
            lowTempTblSize = sizeof(r_profile_t2)/sizeof(R_PROFILE_STRUC);
        }
    }
    else
    {
        if( batTemp < LGBM_TEMPERATURE_0 )
        {
            highTemp = LGBM_TEMPERATURE_0;
            lowTemp = LGBM_TEMPERATURE_M10;
            pHighTempTbl = r_profile_2nd_t1;
            pLowTempTbl = r_profile_2nd_t0;
            highTempTblSize = sizeof(r_profile_2nd_t1)/sizeof(R_PROFILE_STRUC);
            lowTempTblSize = sizeof(r_profile_2nd_t0)/sizeof(R_PROFILE_STRUC);
        }
        else if( batTemp < LGBM_TEMPERATURE_25 )
        {
            highTemp = LGBM_TEMPERATURE_25;
            lowTemp = LGBM_TEMPERATURE_0;
            pHighTempTbl = r_profile_2nd_t2;
            pLowTempTbl = r_profile_2nd_t1;
            highTempTblSize = sizeof(r_profile_2nd_t2)/sizeof(R_PROFILE_STRUC);
            lowTempTblSize = sizeof(r_profile_2nd_t1)/sizeof(R_PROFILE_STRUC);
        }
        else
        {
            highTemp = LGBM_TEMPERATURE_50;
            lowTemp = LGBM_TEMPERATURE_25;
            pHighTempTbl = r_profile_2nd_t3;
            pLowTempTbl = r_profile_2nd_t2;
            highTempTblSize = sizeof(r_profile_2nd_t3)/sizeof(R_PROFILE_STRUC);
            lowTempTblSize = sizeof(r_profile_2nd_t2)/sizeof(R_PROFILE_STRUC);
        }
    }

    /* get interpolated value of battery resistance at high temperature */
    if( batVolt > pHighTempTbl[0].voltage )
    {
        highTempBatRes = pHighTempTbl[0].resistance;
    }
    else
    {
        for ( index = 1 ; index < highTempTblSize ; index++ )
        {
            if( batVolt > pHighTempTbl[index].voltage )
            {
                x1 = pHighTempTbl[index].voltage;
                x2 = pHighTempTbl[index-1].voltage;
                y1 = pHighTempTbl[index].resistance;
                y2 = pHighTempTbl[index-1].resistance;
                highTempBatRes = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batVolt);
                //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, batVolt, highTempBatRes);
                break;
            }
        }

        if ( index == highTempTblSize )
        {
            highTempBatRes = pHighTempTbl[highTempTblSize-1].resistance;
        }
    }

    if( batVolt > pLowTempTbl[0].voltage )
    {
        lowTempBatRes = pLowTempTbl[0].resistance;
    }
    else
    {
        for ( index = 1 ; index < lowTempTblSize ; index++ )
        {
            if( batVolt > pLowTempTbl[index].voltage )
            {
                x1 = pLowTempTbl[index].voltage;
                x2 = pLowTempTbl[index-1].voltage;
                y1 = pLowTempTbl[index].resistance;
                y2 = pLowTempTbl[index-1].resistance;
                lowTempBatRes = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batVolt);
                //LGBM_DBG("x1=%d, x2=%d, y1=%d, y2=%d, x=%d, y=%d\n", x1, x2, y1, y2, batVolt, lowTempBatRes);
                break;
            }
        }

        if ( index == lowTempTblSize )
        {
            lowTempBatRes = pLowTempTbl[lowTempTblSize-1].resistance;
        }
    }


    /* get interpolated value of battery resistance at low temperature */

    x1 = highTemp;
    x2 = lowTemp;
    y1 = highTempBatRes;
    y2 = lowTempBatRes;
    batRes = LGBM_GetLinearInterpolation(x1, x2, y1, y2, batTemp);

    //LGBM_DBG("highTemp=%d, highTempBatRes=%d, lowTemp=%d, lowTempBatRes=%d, batTemp=%d, batRes=%d\n", highTemp, highTempBatRes, lowTemp, lowTempBatRes, batTemp, batRes);

    return batRes;

}

int LGBM_GetBatVoltage ( int batVoltAdc, int batId, int batTemp, int batCurrent )
{
    kal_int32 batVolt = 0;
    kal_int32 batRes = 0;
    kal_int32 recursion = 4; /* 4 is enough from testing */
    kal_int32 count = 0;
    kal_int32 compensatedValue = 0;
    static kal_int32 bufIndex = 0xFF;
    static kal_int32 bufBatVolt[LGBM_BAT_VOLT_BUF_SIZE] = {0};
    static kal_int32 bufBatVoltSum = 0;

    LGBM_ENTRY();

    batVolt = batVoltAdc;

    for( count=0 ; count < recursion ; count++)
    {
        batRes = LGBM_GetBatResistance ( batId, batTemp, batVolt );
        compensatedValue = (batCurrent * (batRes + R_FG_VALUE)) / 1000;
        batVolt = batVoltAdc - compensatedValue;
    }

    if( bufIndex == 0xFF )
    {
        bufBatVoltSum = 0;
        for( count = 0; count < LGBM_BAT_VOLT_BUF_SIZE ; count++ )
        {
            bufBatVolt[count] = batVolt;
            bufBatVoltSum += batVolt;
        }

        bufIndex = 0;
    }
    else
    {
        bufBatVoltSum -= bufBatVolt[bufIndex];
        bufBatVolt[bufIndex++] = batVolt;
        bufBatVoltSum += batVolt;
        bufIndex %= LGBM_BAT_VOLT_BUF_SIZE;
    }

    batVolt = bufBatVoltSum/LGBM_BAT_VOLT_BUF_SIZE;

//    LGBM_DBG("batVoltAdc=%d, compensatedValue=%d, batVolt=%d\n", batVoltAdc, compensatedValue, batVolt);

    LGBM_EXIT();

    return batVolt;

}

LGBmTempState LGBM_GetTempState( LGBmTempState prevState, kal_int32 batTemp )
{
    LGBmTempState newState = LGBM_TEMP_UNKNOWN;

    kal_int32 lowToMid = 0;
    kal_int32 midToHigh = 0;
    kal_int32 highToUltra = 0;

    if( prevState == LGBM_TEMP_UNKNOWN || prevState == LGBM_TEMP_MIDDLE )
    {
        lowToMid = LGBM_OTP_STOP_MIN_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TEMP;
        highToUltra = LGBM_OTP_STOP_MAX_TEMP;
    }
    else if( prevState == LGBM_TEMP_LOW )
    {
        lowToMid = LGBM_OTP_STOP_TO_NORMAL_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TEMP;
        highToUltra = LGBM_OTP_STOP_MAX_TEMP;
    }
    else if( prevState == LGBM_TEMP_HIGH )
    {
        lowToMid = LGBM_OTP_STOP_MIN_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TO_NORMAL_TEMP;
        highToUltra = LGBM_OTP_STOP_MAX_TEMP;
    }
    else /* prevState == LGBM_TEMP_ULTRA_HIGH */
    {
        lowToMid = LGBM_OTP_STOP_MIN_TEMP;
        #if 0 /* OTP Spec. Ver1.6 */
        midToHigh = LGBM_OTP_DECREASE_TO_NORMAL_TEMP;
        highToUltra = LGBM_OTP_DECREASE_TO_NORMAL_TEMP;
        #else
        midToHigh = LGBM_OTP_DECREASE_TEMP;
        highToUltra = LGBM_OTP_STOP_TO_DECREASE_TEMP;
        #endif
    }

    if( batTemp < lowToMid )
    {
        newState = LGBM_TEMP_LOW;
    }
    else if( batTemp < midToHigh )
    {
        newState = LGBM_TEMP_MIDDLE;
    }
    else if( batTemp < highToUltra )
    {
        newState = LGBM_TEMP_HIGH;
    }
    else
    {
        newState = LGBM_TEMP_ULTRA_HIGH;
    }

    if( prevState != newState )
    {
        switch( newState )
        {
            case LGBM_TEMP_LOW:
                LGBM_LOG("TEMP = LGBM_TEMP_LOW\n");
                break;
            case LGBM_TEMP_MIDDLE:
                LGBM_LOG("TEMP = LGBM_TEMP_MIDDLE\n");
                break;
            case LGBM_TEMP_HIGH:
                LGBM_LOG("TEMP = LGBM_TEMP_HIGH\n");
                break;
            case LGBM_TEMP_ULTRA_HIGH:
                LGBM_LOG("TEMP = LGBM_TEMP_ULTRA_HIGH\n");
                break;
            default:
                LGBM_ERR("Invalid Temp State ( %d )\n", newState);
                break;
        }

    }

    return newState;

}

void LGBM_ReadBatVital(LGBmVital *pVital)
{
    kal_int32 batTempAdc = 0;
    kal_int32 batVoltAdc = 0;
    kal_int32 chgVoltAdc = 0;
    kal_int32 accColumb = 0;
    kal_int32 batCurrent = 0;
    kal_int32 batTemp = 0;
    kal_int32 batVolt = 0;
    kal_int32 chgVolt = 0;

    /* read battery ADC */
    batVoltAdc = LGBM_ReadBatVoltAdc();

    /* read charger ADC */
    chgVoltAdc = LGBM_ReadChgVoltAdc();
    chgVolt = chgVoltAdc / 100;

    /* read battery temperature ADC */
    batTempAdc = LGBM_ReadBatTempAdc();
    if( batTempAdc > LGBM_BAT_REMOVE_ADC_TH )
    {
        /* impossible temperature so set to default embient temperature */
        /* it is needed to process battery removal, if not high temperature protection will be processed */
        batTempAdc = 450; /* 450[mV] means 25 degree celsius */
    }

    /* read ( accumulated ) columb */
    accColumb = LGBM_ReadCumulatedColumb();

    /* read current ( charging(+) / dis-charging(-) ) */
    gFG_current = batCurrent = LGBM_ReadBatCurrent();
	if(gFG_current<0)
		gFG_current=gFG_current*-1;
		

    /* read battery temperature */
    batTemp = LGBM_GetBatTemp(batt_id_check, batTempAdc, batCurrent);

    /* read battery voltage */
    batVolt = LGBM_GetBatVoltage(batVoltAdc, batt_id_check, batTemp, batCurrent);

    LGBM_LOG("[Vital] batVoltAdc=%d[mV], batTempAdc=%d[mV], chgVoltAdc=%d\n", batVoltAdc, batTempAdc, chgVoltAdc);
    LGBM_LOG("[Vital] accColumb=%d[mAh], batCurrent=%d[mA], batTemp=%d[C], batVolt=%d[mV], chgVolt=%d[mV]\n", accColumb, batCurrent, batTemp, batVolt, chgVolt);
	#ifndef MT6575
	if(upmu_is_chr_det()==KAL_TRUE)
		max8971_register_info(1);
	#endif
    pVital->batTempAdc = batTempAdc;
    pVital->batVoltAdc = batVoltAdc;
    pVital->chgVoltAdc = chgVoltAdc;
    pVital->accColumb = accColumb;
    pVital->batCurrent = batCurrent;
    pVital->batTemp = batTemp;
    pVital->batVolt = batVolt;
    pVital->chgVolt = chgVolt;

}

#if 1 //                                              
int LGBM_ReadEocState ( LGBmVital *pVital )
{
    kal_int32 hwEocState = 0;
    kal_int32 swEocState = 0;
    kal_int32 eocState = 0;

    int decisionCount = 3; /* 30ms duration */

    LGBM_ENTRY();

	#if defined(CONFIG_MAX8971_CHARGER)
	hwEocState=max8971_eoc_status();
    #elif defined(CONFIG_SINGLE_CHARGER)
    hwEocState = mt_get_gpio_in(GPIO_EOC_PIN);
    #elif defined(CONFIG_MINIABB_CHARGER)
    hwEocState = check_EOC_status();
    #endif

    #if defined(CONFIG_SINGLE_CHARGER)
    while ( hwEocState == 1 && decisionCount > 0 )
    {
        msleep(10);
        hwEocState = mt_get_gpio_in(GPIO_EOC_PIN);
        decisionCount--;
    }
    #endif

    if( 0 < pVital->batCurrent && pVital->batCurrent < LGBM_SW_EOC_CURRENT_TH )
    {
        swEocState = 1;
    }
	
    if( hwEocState == 1 || swEocState == 1 )
    {
        if( pVital->batVoltAdc > LGBM_SW_EOC_VOLTAGE_TH )
        {
            eocState = 1;
            LGBM_LOG("EOC = 1 ( HW = %d, SW = %d, batVoltAdc = %d[mV] )\n", hwEocState, swEocState, pVital->batVoltAdc);
        }
        else
        {
            LGBM_LOG("EOC = 0 ( HW = %d, SW = %d, batVoltAdc = %d[mV] )\n", hwEocState, swEocState, pVital->batVoltAdc);
        }
    }

    LGBM_EXIT();

    return eocState;

}
#else
int LGBM_ReadEocState ( void )
{
    kal_int32 eocState = 0;
    int decisionCount = 3; /* 30ms duration */

    LGBM_ENTRY();

    #if defined(CONFIG_SINGLE_CHARGER)
    eocState = mt_get_gpio_in(GPIO_EOC_PIN);
    #elif defined(CONFIG_MINIABB_CHARGER)
    eocState = check_EOC_status();
    #endif

    #if defined(CONFIG_SINGLE_CHARGER)
    while ( eocState == 1 && decisionCount > 0 )
    {
        msleep(10);
        eocState = mt_get_gpio_in(GPIO_EOC_PIN);
        decisionCount--;
    }
    #endif

    if( eocState == 1 )
    {
        LGBM_LOG("EOC = 1\n");
    }

    LGBM_EXIT();

    return eocState;

}
#endif //                                              

int LGBM_ReadBatExistance( void )
{
    int batExist = 1;
    int batTempAdc = 0;
    int loopCount = 0;

    batTempAdc = LGBM_ReadBatTempAdc();
    if( batTempAdc > LGBM_BAT_REMOVE_ADC_TH )
    {
        for( loopCount=0 ; loopCount < 5 ; loopCount++ )
        {
            mdelay(100);
            batTempAdc = LGBM_ReadBatTempAdc();
            if( batTempAdc < LGBM_BAT_REMOVE_ADC_TH )
            {
                break;
            }
        }

        if( loopCount >= 5 )
        {
            batExist = 0;
        }
    }

    return batExist;

}



kal_bool LGBM_ReadChargerExistance(void)
{
    kal_bool chargerExist = KAL_FALSE;

#if 0 //                                              
    chargerExist = upmu_get_PCHR_CHRDET();
#else
    chargerExist = upmu_is_chr_det();
#endif //                                              

    LGBM_LOG("CHARGER = %d\n", chargerExist);

    return chargerExist;
}

CHARGER_TYPE LGBM_ReadChargerType ( void )
{
    CHARGER_TYPE charger = CHARGER_UNKNOWN;

    LGBM_ENTRY();

    charger = mt_charger_type_detection();

    switch ( charger )
    {
        case STANDARD_HOST:
            LGBM_LOG("STANDARD_HOST was detected\n");
            break;
        case CHARGING_HOST:
            LGBM_LOG("CHARGING_HOST was detected\n");
            break;
        case NONSTANDARD_CHARGER:
            LGBM_LOG("NONSTANDARD_CHARGER was detected\n");
            break;
        case STANDARD_CHARGER:
            LGBM_LOG("STANDARD_CHARGER was detected\n");
            break;
        default:
            LGBM_ERR("Unknown type of charger ( %d ) was detected, and treat is as STANDARD_CHARGER\n", charger);
            charger = STANDARD_CHARGER;
            break;
    }

    LGBM_EXIT();

    return charger ;

}

LGBmCableId LGBM_ReadUsbCableId( void )
{
    LGBmCableId cableId = LGBM_CABLE_ID_UNKNOWN;
    int resVal = 0;
    int adcValue[5] = {0,0,0,0,0};
    int adcVolt = 0;
    int data[4] = {0,0,0,0};
	int i,adc_avr=0;
	int adc_fail_count=0;

    LGBM_ENTRY();

	for(i=0;i<5;i++)
	{
    	resVal = IMM_GetOneChannelValue(AUXADC_USB_ID_CHANNEL, data, adcValue+i); /* unit : 4096 = 2500mV */
	    if( resVal < 0 )
	    {
    	    LGBM_ERR("ADC Read Fail on AUXADC_USB_ID_CHANNEL ( error = %d )\n", resVal);
			adc_fail_count++;
	    }
		else
		{
			adc_avr+=adcValue[i];
			LGBM_DBG("ReadUsbCableId ( adcValue[%d] = %d ) adc_sum = %d\n", i, adcValue[i],adc_avr);		
		}

	}
	if(adc_fail_count==5)
	{
		LGBM_DBG(" ADC read fail occur 5 times, exit LGBM_ReadUsbCableId\n");		
		return LGBM_CABLE_ID_UNKNOWN;
	}
	else
	{
		adc_avr=adc_avr/(i-adc_fail_count);
	}		


    adcVolt = adc_avr*2500/4096; /* unit : 4096 = 2500mV */

    LGBM_DBG("ReadUsbCableId ( adcValue = %d, adcVolt = %d[mV] )\n", adc_avr, adcVolt);

    if( LGBM_CABLE_ID_56K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_56K_ADC_MAX )
    {
        cableId = LGBM_CABLE_ID_56K;
        LGBM_LOG("cableId = LGBM_CABLE_ID_56K\n");
    }
    else if( LGBM_CABLE_ID_130K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_130K_ADC_MAX )
    {
        cableId = LGBM_CABLE_ID_130K;
        LGBM_LOG("cableId = LGBM_CABLE_ID_130K\n");
    }
    else if( LGBM_CABLE_ID_180K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_180K_ADC_MAX )
    {
        cableId = LGBM_CABLE_ID_180K;
        LGBM_LOG("cableId = LGBM_CABLE_ID_180K\n");
    }
    else if( LGBM_CABLE_ID_910K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_910K_ADC_MAX )
    {
        cableId = LGBM_CABLE_ID_910K;
        LGBM_LOG("cableId = LGBM_CABLE_ID_910K\n");
    }
    else if( LGBM_CABLE_ID_OPEN_ADC_MIN <= adcVolt )
    {
        cableId = LGBM_CABLE_ID_OPEN;
        LGBM_LOG("cableId = LGBM_CABLE_ID_OPEN\n");
    }
    else
    {
        cableId = LGBM_CABLE_ID_UNKNOWN;
        LGBM_LOG("cableId = LGBM_CABLE_ID_UNKNOWN ( adcValue = %d, adcVolt = %d[mV] )\n", adc_avr, adcVolt);
    }

        //LGBM_LOG("cableId = %d\n", cableId);


    LGBM_EXIT();

    return cableId;

}

CHARGER_TYPE LGBM_GetCharger ( void )
{
    LGBmCableId cableId = LGBM_CABLE_ID_UNKNOWN;

    CHARGER_TYPE charger = CHARGER_UNKNOWN;

    LGBM_ENTRY();

    cableId = LGBM_ReadUsbCableId();
    if( cableId == LGBM_CABLE_ID_56K || cableId == LGBM_CABLE_ID_130K || cableId == LGBM_CABLE_ID_910K )
    {
        charger = FACTORY_CHARGER;
    }
    else
    {
        charger = LGBM_ReadChargerType();
    }

    return charger;

}

kal_int32 LGBM_GetNewSoc( kal_int32 initSoc, kal_int32 columb, kal_int32 maxBatCap )
{
    kal_int32 newSoc = 0;

    LGBM_ENTRY();

    newSoc = initSoc + columb*100/maxBatCap;

    LGBM_DBG("newSoc=%d[%]\n", newSoc);

    LGBM_EXIT();

    return newSoc;
}

void LGBM_SetChargingCurrent( LGBmChargingCurrent chgCurrent )
{
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else
    RT9536_ChargingMode mode = RT9536_CM_UNKNOWN;
#endif
    if( prevChgCurrent != chgCurrent )
    {
        prevChgCurrent = chgCurrent;

        switch ( chgCurrent )
        {
            case LGBM_CC_OFF:
                LGBM_LOG("Charging Current = LGBM_CC_OFF\n");
#if defined(CONFIG_MAX8971_CHARGER)
				max8971_stop_charging();
#elif (CONFIG_MINIABB_CHARGER)
				set_charger_stop_mode();
#else
                mode = RT9536_CM_OFF;
#endif
                break;
            case LGBM_CC_USB_100:
                LGBM_LOG("Charging Current = LGBM_CC_USB_100\n");
#if defined(CONFIG_MAX8971_CHARGER)
				max8971_start_charging(250);
#elif (CONFIG_MINIABB_CHARGER)
				set_charger_start_mode(CHG_100);
#else
				mode = RT9536_CM_USB_100;
#endif
                break;
            case LGBM_CC_USB_500:
                LGBM_LOG("Charging Current = LGBM_CC_USB_500\n");
#if defined(CONFIG_MAX8971_CHARGER)
				max8971_start_charging(500);
#elif (CONFIG_MINIABB_CHARGER)
				set_charger_start_mode(CHG_500);
#else
				mode = RT9536_CM_USB_500;
#endif
                break;
            case LGBM_CC_I_SET:
                LGBM_LOG("Charging Current = LGBM_CC_I_SET\n");
#if defined(CONFIG_MAX8971_CHARGER)
				max8971_start_charging(1200);
#elif (CONFIG_MINIABB_CHARGER)
				set_charger_start_mode(CHG_TA);
#else
				mode = RT9536_CM_I_SET;
#endif
                break;
            case LGBM_CC_FACTORY:
                LGBM_LOG("Charging Current = LGBM_CC_FACTORY\n");
#if defined(CONFIG_MAX8971_CHARGER)
				max8971_start_charging(500);
#elif (CONFIG_MINIABB_CHARGER)

				set_charger_start_mode(CHG_500);
#else
				mode = RT9536_CM_FACTORY;
#endif
                break;
            default:
                LGBM_ERR("Invalid Charging Current ( %d )\n", chgCurrent);
                break;
        }
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else
        if( mode != RT9536_CM_UNKNOWN )
        {
            RT9536_SetChargingMode(mode);
        }
#endif
    }
}

int LGBM_ResetColumbCounter ( void )
{
    kal_uint32 regVal = 0;
    kal_uint32 regVal_High = 0;
    kal_int16 tmpRegVal = 0;
    kal_int32 readColumb = 0;
    kal_int32 tmpColumb = 0;
    kal_int32 compensatedColumb = 0;

    int m = 0;
    kal_uint32 ret = 0;

    LGBM_ENTRY();

    ret=pmic_bank1_config_interface(0x6A, 0x71, 0xFF, 0x0);

    // HW Init // Enable VA2
    ret=pmic_config_interface(0xC8, 0x1, 0xFF, 0x0);

    //Read HW Raw Data // Set READ command
    ret=pmic_bank1_config_interface(0x6A, 0x73, 0xFF, 0x0);

    // Keep i2c read when status = 1 (0x06)
    m=0;
    while ( fg_get_data_ready_status() == 0 )
    {
        m++;
        if(m>1000)
        {
            break;
        }
    }

    // Read FG_CURRENT_OUT
    regVal = (upmu_fgadc_car_15_08())>>6;
    regVal |= (upmu_fgadc_car_23_16())<<2;
    regVal |= (upmu_fgadc_car_31_24())<<10;
    regVal_High = (upmu_fgadc_car_35_32() & 0x0F)>>3;

    // Clear status to 0
    ret=pmic_bank1_config_interface(0x6A, 0x08, 0xFF, 0x0);

    // Keep i2c read when status = 0 (0x08)
    m=0;
    while ( fg_get_data_ready_status() != 0 )
    {
        m++;
        if(m>1000)
        {
            break;
        }
    }

    // Recover original settings
    ret=pmic_bank1_config_interface(0x6A, 0x00, 0xFF, 0x0);

    tmpRegVal = (kal_int16)(regVal & 0xFFFF);

    //calculate thereal world data
    readColumb = ( ((tmpRegVal*35986)/10) + (5) )/10; //[28:14]'s LSB=359.86 uAh
    readColumb = readColumb / 1000; //mAh

    tmpColumb = ((readColumb*CAR_TUNE_VALUE)/100);

    compensatedColumb = LGBM_GetTrimmedValue(tmpColumb);

    LGBM_DBG("regVal = %d, readColumb = %d[mAh], compensatedColumb = %d[mAh]\n", (kal_int16)(regVal & 0xFFFF), readColumb, compensatedColumb);

    LGBM_EXIT();

    return compensatedColumb;

}

void LGBM_SetWakeLock( kal_bool isLock )
{
    if( isLock == KAL_TRUE )
    {
        wake_lock(&battery_suspend_lock);
        LGBM_LOG("WAKE LOCK = TRUE\n");
    }
    else
    {
        #if 1 //                                                                                                                                                        
        wake_lock_timeout(&battery_suspend_lock, 5*HZ);
        #else
        wake_unlock(&battery_suspend_lock);
        #endif //                                              
        LGBM_LOG("WAKE LOCK = FALSE\n");
    }
}

void LGBM_SetUsbConnection( kal_bool isConnect )
{
    if( isConnect == KAL_TRUE )
    {
        mt_usb_connect();
        LGBM_LOG("USB CONNECTION = 1\n");
    }
    else
    {
        mt_usb_disconnect();
        LGBM_LOG("USB CONNECTION = 0\n");
    }
}

void LGBM_UpdateAcStatus( kal_bool isConnect )
{
    if( isConnect == KAL_TRUE )
    {
        mt6329_ac_main.AC_ONLINE = 1;
        power_supply_changed(&mt6329_ac_main);
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else		
        ist30xx_set_ta_mode ( KAL_TRUE );
#endif		
    }
    else
    {
        mt6329_ac_main.AC_ONLINE = 0;
        power_supply_changed(&mt6329_ac_main);
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else			
        ist30xx_set_ta_mode ( KAL_FALSE );
#endif	
    }

    LGBM_LOG("UI_UPDATE ( AC =  %d )\n", mt6329_ac_main.AC_ONLINE);
}

void LGBM_UpdateUsbStatus( kal_bool isConnect )
{
    if( isConnect == KAL_TRUE )
    {
        mt6329_usb_main.USB_ONLINE = 1;
        power_supply_changed(&mt6329_usb_main);
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else			
        ist30xx_set_ta_mode ( KAL_TRUE );
#endif		
    }
    else
    {
        mt6329_usb_main.USB_ONLINE = 0;
        power_supply_changed(&mt6329_usb_main);
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else			
        ist30xx_set_ta_mode ( KAL_FALSE );
#endif		
    }

    LGBM_LOG("UI_UPDATE ( USB =  %d )\n", mt6329_usb_main.USB_ONLINE);
}


void LGBM_UpdateBatStatus( LGBmData *pBmData, LGBmVital *pVital, struct mt6329_battery_data *bat_data )
{
    struct power_supply *bat_psy = &bat_data->psy;

    //bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;

    if( pBmData->tempState == LGBM_TEMP_LOW )
    {
        bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_COLD;
    }
    else if( pBmData->tempState == LGBM_TEMP_HIGH ||pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
    {
        bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_OVERHEAT;
    }
    else
    {
        bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
    }

    bat_data->BAT_batt_vol = pVital->batVolt ;
    bat_data->BAT_batt_temp= pVital->batTemp * 10 ;
    bat_data->BAT_PRESENT = pBmData->batExist ;
    bat_data->BAT_CAPACITY = pBmData->curSoc ;
    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;

    if( pBmData->bmState == LGBM_TS_NO_CHARGER )
    {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_DISCHARGING;
		if(prevBmState==LGBM_TS_CHARGING && bat_data->BAT_CAPACITY>=100)
		{
			bat_data->BAT_CAPACITY=99;
			LGBM_LOG("UI show battery capacity 99% because prevBmState is LGBM_TS_CHARGING");
		}
		else if( bat_data->BAT_CAPACITY > 100 )
        {
            bat_data->BAT_CAPACITY = 100;
        }
        else if( bat_data->BAT_CAPACITY < 1 )
        {
            bat_data->BAT_CAPACITY = 1;
        }
    }
    else if( pBmData->bmState == LGBM_TS_CHARGING )
    {
        if( pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        else
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
        }

        if( bat_data->BAT_CAPACITY > 99 )
        {
            bat_data->BAT_CAPACITY = 99;
        }
        else if( bat_data->BAT_CAPACITY < 1 )
        {
            bat_data->BAT_CAPACITY = 1;
        }
    }
    else if( pBmData->bmState == LGBM_TS_CHARGING_FULL )
    {
        if( pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        else
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
        }

        bat_data->BAT_CAPACITY = 100;
    }
    else if( pBmData->bmState == LGBM_TS_FACTORY )
    {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
        if( bat_data->BAT_CAPACITY > 100 )
        {
            bat_data->BAT_CAPACITY = 100;
        }
        else if( bat_data->BAT_CAPACITY < 1 )
        {
            bat_data->BAT_CAPACITY = 1;
        }
    }

    bat_data->BAT_TemperatureR = g_BAT_TemperatureR;
    bat_data->BAT_TempBattVoltage = pVital->batTempAdc ;
    bat_data->BAT_InstatVolt = pVital->batVolt ;
    bat_data->BAT_BatteryAverageCurrent = pVital->batCurrent ;
    bat_data->BAT_BatterySenseVoltage = pVital->batVoltAdc ;
    bat_data->BAT_ISenseVoltage = 0 ;
    bat_data->BAT_ChargerVoltage = 0; /* TBD */

    /* for facotry AT command */
    g_AtCmdBatSocUI = bat_data->BAT_CAPACITY;
    g_AtCmdBatFullUI = bat_data->BAT_STATUS;

    power_supply_changed(bat_psy);

    LGBM_LOG("UI_UPDATE ( BATTERY ) : batVolt=%d, batTemp=%d, batSoc=%d, driverSoc=%d\n", bat_data->BAT_batt_vol, bat_data->BAT_batt_temp, bat_data->BAT_CAPACITY, pBmData->curSoc);

}

kal_bool LGBM_WriteSocToFile( int batSoc )
{
    struct file *fd = NULL;
    int retVal = 0;
    kal_bool result = KAL_TRUE;

    fd = filp_open("/data/battery_data", O_CREAT|O_TRUNC|O_WRONLY , 0666);
    if (IS_ERR(fd))
    {
        retVal = PTR_ERR(fd);
        LGBM_ERR("Soc Write ( File Open ) Fail ( errno = %d )\n", retVal);
        result = KAL_FALSE;
    }
    else
    {
        retVal = fd->f_op->write(fd, (unsigned char *)&batSoc, 1, &(fd->f_pos));
        if( retVal < 0 )
        {
            LGBM_ERR("Soc Write ( File Write ) Fail ( errno = %d )\n", retVal);
            result = KAL_FALSE;
        }
        else
        {
            LGBM_LOG("Soc Write Success ( batSoc = %d )\n", batSoc);
        }

        filp_close(fd, NULL);
    }

    return result;

}

int LGBM_ReadSocFromFile( void )
{
    struct file *fd = NULL;
    int retVal = 0;
    int batSoc=0;
    int waitCount = 0;

    do {
        fd = filp_open("/data/battery_data", O_RDONLY , 0666);
        if(!IS_ERR(fd))
        {
            retVal = fd->f_op->read(fd, (unsigned char *)&batSoc, 1, &(fd->f_pos));
            LGBM_LOG("Soc File Open Success!\n");
            break;
        }
        else
        {
            waitCount++;
            retVal = PTR_ERR(fd);
            LGBM_ERR("Wait for mount /data/ file system: %dtime, retVal: %d\n", waitCount, retVal);
            msleep(200);
        }
    }while(waitCount < 5);

    if( retVal < 0 )
    {
        LGBM_ERR("Soc Read ( File Read ) Fail ( errno = %d )\n", retVal);
        batSoc = -1;
    }
    else
    {
        LGBM_LOG("Soc Read Success ( batSoc = %d )\n", batSoc);
        filp_close(fd, NULL);
    }

    return batSoc;

}


#if 1 //                                              
#else
void LGBM_DelayedInitialize( LGBmData *pBmData )
{
    LGBmCableId cableId = LGBM_CABLE_ID_UNKNOWN;

    LGBM_ENTRY();

    /* CAUTION : factory cable detection will be failed if you move this job to LGBM_Initialize() */
    if( LGBM_ReadChargerExistance() == KAL_TRUE )
    {
        cableId = LGBM_ReadUsbCableId();
        if( cableId == LGBM_CABLE_ID_56K || cableId == LGBM_CABLE_ID_130K || cableId == LGBM_CABLE_ID_910K )
        {
            LGBM_LOG("Factory Cable was detected\n");

            pBmData->charger = FACTORY_CHARGER;

            //newState = LGBM_TS_FACTORY;

            //LGBM_ChangeState( pBmData, LGBM_TS_FACTORY );

#if 1 //                                              
#else
            /* charging current was already set on LK, so do not set here again */

            LGBM_SetUsbConnection(KAL_TRUE);

            /* TBD : Need to verify it's ok or not for flight mode current consumption */
            //LGBM_SetWakeLock(KAL_TRUE);
#endif //                                              
        }
        else
        {
            pBmData->charger = LGBM_ReadChargerType();

#if 1 //                                              
#else
            LGBM_ChangeState( pBmData, LGBM_TS_FACTORY );
            LGBM_SetChargingCurrent( pBmData->charger );

            if( ( pBmData->charger == STANDARD_HOST ) || ( pBmData->charger == CHARGING_HOST ) )
            {
                LGBM_SetUsbConnection(KAL_TRUE);
            }
            else
            {
                mt6329_ac_main.AC_ONLINE = 1;
                power_supply_changed(&mt6329_ac_main);

                LGBM_LOG("UI_UPDATE ( AC =  %d )\n", mt6329_ac_main.AC_ONLINE);
            }

            LGBM_SetWakeLock(KAL_TRUE);
#endif //                                              

        }

    }
    else
    {
        pBmData->charger = CHARGER_UNKNOWN;
    }

    LGBM_EXIT();

}
#endif //                                              

#if 1 //                                              
#else
kal_bool LGBM_ProcessChargerEvent ( LGBmData *pBmData )
{
    LGBmCableId cableId = LGBM_CABLE_ID_UNKNOWN;
    CHARGER_TYPE charger = CHARGER_UNKNOWN;
    kal_bool chargerChanged = KAL_TRUE;
    kal_bool chargerInserted = 0;

    chargerInserted = LGBM_ReadChargerExistance();

    if( chargerInserted == KAL_TRUE )
    {
        LGBM_LOG("Charger insertion was detected\n");

        if( pBmData->charger == CHARGER_UNKNOWN )
        {
            msleep(200); /* CAUTION : Need to optimize */

            cableId = LGBM_ReadUsbCableId();
            if( cableId == LGBM_CABLE_ID_56K || cableId == LGBM_CABLE_ID_130K || cableId == LGBM_CABLE_ID_910K )
            {
                LGBM_LOG("Factory Cable was inserted\n");

                pBmData->charger = FACTORY_CHARGER;
                LGBM_SetChargingCurrent( pBmData->charger );

                LGBM_SetUsbConnection(KAL_TRUE);
            }
            else
            {
                pBmData->charger = LGBM_ReadChargerType();
                LGBM_SetChargingCurrent( pBmData->charger );

                if( ( pBmData->charger == STANDARD_HOST ) || ( pBmData->charger == CHARGING_HOST ) )
                {
                    LGBM_SetUsbConnection(KAL_TRUE);
                }
                else
                {
                    mt6329_ac_main.AC_ONLINE = 1;
                    power_supply_changed(&mt6329_ac_main);

                    LGBM_LOG("UI_UPDATE ( AC =  %d )\n", mt6329_ac_main.AC_ONLINE);
                }

                LGBM_SetWakeLock(KAL_TRUE);

            }

        }
        else
        {
            LGBM_LOG("Charger was already inserted so ignore it\n");
            chargerChanged = KAL_FALSE;
        }
    }
    else
    {
        LGBM_LOG("Charger removal was detected\n");

        if( pBmData->charger == FACTORY_CHARGER )
        {
            msleep(200); /* CAUTION : Need to optimize */

            cableId = LGBM_ReadUsbCableId();
            if( !( cableId == LGBM_CABLE_ID_56K || cableId == LGBM_CABLE_ID_130K || cableId == LGBM_CABLE_ID_910K ) )
            {
                LGBM_LOG("Factory Cable was removed\n");

                LGBM_SetUsbConnection(KAL_FALSE);

                pBmData->charger = CHARGER_UNKNOWN;
                LGBM_SetChargingCurrent( pBmData->charger );
            }
            else
            {
                LGBM_LOG("Charger is absent but USB cable ID still shows factory cable, so just ignore it\n");
                chargerChanged = KAL_FALSE;
            }
        }
        else if ( pBmData->charger != CHARGER_UNKNOWN )
        {
            if( ( pBmData->charger == STANDARD_HOST ) || ( pBmData->charger == CHARGING_HOST ) )
            {
                LGBM_SetUsbConnection(KAL_FALSE);
            }
            else
            {
                mt6329_ac_main.AC_ONLINE = 0;
                power_supply_changed(&mt6329_ac_main);

                LGBM_LOG("UI_UPDATE ( AC =  %d )\n", mt6329_ac_main.AC_ONLINE);
            }

            LGBM_SetWakeLock(KAL_FALSE);

            pBmData->charger = CHARGER_UNKNOWN;
            LGBM_SetChargingCurrent( pBmData->charger );
        }
        else
        {
            LGBM_LOG("Charger was already removed so ignore it\n");
            chargerChanged = KAL_FALSE;
        }
    }

    return chargerChanged;

}
#endif //                                              


void LGBM_Initialize ( void )
{
    LGBM_ENTRY();

    /* check PMIC chip version */
    g_eco_version = LGBM_ReadPmicEcoVersion();

    /* get trim factor for fuel gauge. it will be used to compensate the value of columb counting and battery current */
    g_FuelGaugeTrimFactor = LGBM_ReadFuelGaugeTrimFactor();

    /* fuel gauge resistance */
    g_FuelGaugeResistance = R_FG_VALUE + R_FG_OFFSET;

#if 1 //                                               
    if ( batt_id_check == 1 )
    {
#ifdef CONFIG_MAX8971_CHARGER
   	    max8971_stop_factory_charging();
#elif (CONFIG_MINIABB_CHARGER)
#else

#endif
        LGBM_LOG("LG Chemical/TOCAD Battery Cell was detected ( Battery ID = %d )\n", batt_id_check);
    }
    else
    {
        LGBM_LOG("BYD Battery Cell was detected ( Battery ID = %d )\n", batt_id_check);
    }
#else
    if ( batt_id_check == 1 ) /* LG Chemical Battery Cell */
    {
        LGBM_LOG("LG Chemical Battery Cell was detected ( Battery ID = %d )\n", batt_id_check);
    }
    else /* BYD Battery Cell */
    {
        LGBM_LOG("BYD Battery Cell was detected ( Battery ID = %d )\n", batt_id_check);
    }
#endif //                                               

    LGBM_LOG("Battery Capacity ( Qmax = %d )\n", g_lgbmQmax);

    g_lgbmBootUsbCableId = LGBM_ReadUsbCableId();
    LGBM_LOG("g_lgbmBootUsbCableId = %d\n", g_lgbmBootUsbCableId);

#if 1 //                                              
#else
    #if ( PRODUCT_DEVICE == vee4ss )
    #error "PRODUCT_DEVICE = vee4ss"
    #elif ( PRODUCT_DEVICE == vee4ds )
    #error "PRODUCT_DEVICE = vee4ds"
    #elif ( PRODUCT_DEVICE == vee4ts )
    #error "PRODUCT_DEVICE = vee4ts"
    #else
    #error "PRODUCT_DEVICE = UNKNOWN"
    #endif
#endif //                                              

    fgauge_initialization();

}

void LGBM_PrintBmStateChange ( LGBmState prevState, LGBmState newState )
{
    char strPrevState[30] = {0};
    char strNewState[30] = {0};

    switch ( prevState )
    {
        case LGBM_TS_INIT:
            strcpy( strPrevState, "LGBM_TS_INIT" );
            break;

        case LGBM_TS_NO_CHARGER:
            strcpy( strPrevState, "LGBM_TS_NO_CHARGER" );
            break;

        case LGBM_TS_CHARGING:
            strcpy( strPrevState, "LGBM_TS_CHARGING" );
            break;

        case LGBM_TS_CHARGING_FULL:
            strcpy( strPrevState, "LGBM_TS_CHARGING_FULL" );
            break;

        case LGBM_TS_FACTORY:
            strcpy( strPrevState, "LGBM_TS_FACTORY" );
            break;

        case LGBM_TS_INSERT_BATTERY:
            strcpy( strPrevState, "LGBM_TS_INSERT_BATTERY" );
            break;

        case LGBM_TS_UNKNOWN:
            strcpy( strPrevState, "LGBM_TS_UNKNOWN" );
            break;

        default :
            strcpy( strPrevState, "LGBM_TS_INVALID" );
            break;

    }

    switch ( newState )
    {
        case LGBM_TS_INIT:
            strcpy( strNewState, "LGBM_TS_INIT" );
            break;

        case LGBM_TS_NO_CHARGER:
            strcpy( strNewState, "LGBM_TS_NO_CHARGER" );
            break;

        case LGBM_TS_CHARGING:
            strcpy( strNewState, "LGBM_TS_CHARGING" );
            break;

        case LGBM_TS_CHARGING_FULL:
            strcpy( strNewState, "LGBM_TS_CHARGING_FULL" );
            break;

        case LGBM_TS_FACTORY:
            strcpy( strNewState, "LGBM_TS_FACTORY" );
            break;

        case LGBM_TS_INSERT_BATTERY:
            strcpy( strNewState, "LGBM_TS_INSERT_BATTERY" );
            break;

        case LGBM_TS_UNKNOWN:
            strcpy( strNewState, "LGBM_TS_UNKNOWN" );
            break;

        default :
            strcpy( strNewState, "LGBM_TS_INVALID" );
            break;

    }

    if( prevState != newState )
    {
        LGBM_LOG("BM_STATE : %s => %s\n", strPrevState, strNewState);
    }
    else
    {
        LGBM_ERR("BM_STATE : %s => %s\n", strPrevState, strNewState);
    }

}


void LGBM_ChangeState( LGBmData *pBmData, LGBmState newState )
{
    kal_bool result = KAL_TRUE;

    prevBmState = pBmData->bmState;

    if( prevBmState != newState )
    {
        if( prevBmState == LGBM_TS_INIT || prevBmState == LGBM_TS_NO_CHARGER )
        {
            if( newState == LGBM_TS_CHARGING || newState == LGBM_TS_CHARGING_FULL )
            {
                pBmData->bmOTPState = LGBM_OTP_UNKNOWN;
//                                                                                                              
#if defined(CONFIG_MAX8971_CHARGER)
#else
                if( newState == LGBM_TS_CHARGING )
               {
#endif
//                                                                                                             
                    if( pBmData->charger == NONSTANDARD_CHARGER || pBmData->charger == STANDARD_CHARGER )
                    {
                        LGBM_SetChargingCurrent(LGBM_CC_I_SET);
                    }
                    else
                    {
                        LGBM_SetChargingCurrent(LGBM_CC_USB_500);
                    }
//                                                                                                             
#if defined(CONFIG_MAX8971_CHARGER)
#else					
               }
#endif
//                                                                                                             
            }
            else if( newState == LGBM_TS_FACTORY )
            {
                if( prevBmState == LGBM_TS_NO_CHARGER )
                {
                    LGBM_SetChargingCurrent(LGBM_CC_FACTORY);
                }
            }
            else if( newState == LGBM_TS_NO_CHARGER )
            {
                pBmData->socTrackState = LGBM_SOC_UN_TRACKED;
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
            }
            else if( newState == LGBM_TS_INSERT_BATTERY )
            {
                /* do nothing */
            }
            else
            {
                result = KAL_FALSE;
            }
        }
        else if( prevBmState == LGBM_TS_CHARGING || prevBmState == LGBM_TS_CHARGING_FULL )
        {
            if( newState == LGBM_TS_NO_CHARGER )
            {
                pBmData->socTrackState = LGBM_SOC_UN_TRACKED;
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
            }
            else if( newState == LGBM_TS_CHARGING || newState == LGBM_TS_CHARGING_FULL )
            {
                pBmData->bmOTPState = LGBM_OTP_UNKNOWN;

                if( newState == LGBM_TS_CHARGING_FULL )
                {
//                                                                                                                        
#if defined(CONFIG_MAX8971_CHARGER)
//					LGBM_LOG("Charging state full MAX8971 charging stop itself, AICL ON for recharging \n");
//		   			max8971_aicl_on();  
#else					
                    LGBM_SetChargingCurrent(LGBM_CC_OFF);
#endif
//                                                                                                                       
                }
            }
            else
            {
                result = KAL_FALSE;
            }
        }
        else if( prevBmState == LGBM_TS_FACTORY )
        {
            if( newState == LGBM_TS_NO_CHARGER )
            {
                pBmData->socTrackState = LGBM_SOC_UN_TRACKED;
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
            }
            else
            {
                result = KAL_FALSE;
            }
        }
        else
        {
            result = KAL_FALSE;
        }

    }

    if( result == KAL_TRUE )
    {
        if( pBmData->bmState != newState )
        {
            LGBM_PrintBmStateChange( prevBmState, newState );

            pBmData->bmState = newState ;

            switch( newState )
            {
                case LGBM_TS_NO_CHARGER:
                    LGBM_LOG("BM_STATE = LGBM_TS_NO_CHARGER\n");
                    break;
                case LGBM_TS_CHARGING:
                    LGBM_LOG("BM_STATE = LGBM_TS_CHARGING\n");
                    break;
                case LGBM_TS_CHARGING_FULL:
                    LGBM_LOG("BM_STATE = LGBM_TS_CHARGING_FULL\n");
                    break;
                case LGBM_TS_FACTORY:
                    LGBM_LOG("BM_STATE = LGBM_TS_FACTORY\n");
                    break;
                case LGBM_TS_INSERT_BATTERY:
                    LGBM_LOG("BM_STATE = LGBM_TS_INSERT_BATTERY\n");
                    break;
                default:
                    LGBM_ERR("Invalid BM_STATE ( %d )\n", newState);
                    break;
            }
        }
        else
        {
            LGBM_DBG("No State Change\n");
        }
    }
    else
    {
        LGBM_ERR("Invalid State Change ( %d => %d )\n", prevBmState, newState);
    }

}

#if 1 //                                              
#if 1 //                                              
#else
void LGBM_UpdateCurSoc( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 newSoc = 0;
    kal_int32 eocState = 0;
    kal_int32 batVolt = pVital->batVolt;
    static kal_int32 trackingMethod = 0; /* 0=No tracking, 1=Running, 2=Keeping */
    static kal_int32 trackingVolt = 0;
    static kal_int32 trackingSoc = 0;
    static kal_int32 trackingCount = 0;

    eocState = LGBM_ReadEocState();
    newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, 1597);

    if( trackingMethod == 0 ) /* No tracking */
    {
        if( pVital->batCurrent > 0 )
        {
            if( eocState == 1 && newSoc < 100 )
            {
                trackingMethod = 1;
                trackingVolt = 0;
                trackingSoc = 100;
            }
            else if( eocState == 0 && newSoc >= 100 )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 99;
            }
            else if( newSoc == 15 && batVolt < g_lgbmSoc15Volt )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 15;
            }
            else if( newSoc < 15 && batVolt > g_lgbmSoc15Volt )
            {
                trackingMethod = 1;
                trackingVolt = batVolt;
                trackingSoc = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, batVolt);
                if( trackingSoc <= newSoc )
                {
                    trackingMethod = 0;
                    LGBM_ERR("Invalid Tracking Condition ( newSoc = %d, trackingSoc = %d )\n", newSoc, trackingSoc);
                }
            }
            else
            {
                pBmData->curSoc = newSoc;
            }

            if( trackingMethod != 0 )
            {
                trackingCount = 0;
                LGBM_LOG("SOC TRACKING : Charging : Method = %d, trackingSoc = %d, trackingVolt = %d\n", trackingMethod, trackingSoc, trackingVolt);
            }

        }
        else
        {
            if( newSoc > 1 && batVolt < 3300 )
            {
                trackingMethod = 1;
                trackingVolt = 0;
                trackingSoc = 1;
            }
            else if( newSoc == 2 && batVolt > 3300 )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 2;
            }
            else if( newSoc == 15 && batVolt > g_lgbmSoc15Volt )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 15;
            }
            else if( newSoc > 15 && batVolt < g_lgbmSoc15Volt )
            {
                trackingMethod = 1;
                trackingVolt = batVolt;
                trackingSoc = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, batVolt);
                if( trackingSoc >= newSoc )
                {
                    trackingMethod = 0;
                    LGBM_ERR("Invalid Tracking Condition ( newSoc = %d, trackingSoc = %d )\n", newSoc, trackingSoc);
                }
            }
            else
            {
                pBmData->curSoc = newSoc;
            }

            if( trackingMethod != 0 )
            {
                trackingCount = 0;
                LGBM_LOG("SOC TRACKING : Dis-Charging : Method = %d, trackingSoc = %d, trackingVolt = %d\n", trackingMethod, trackingSoc, trackingVolt);
            }

        }

    }
    else if( trackingMethod == 1 ) /* Running */
    {
        trackingCount++;
        if( trackingCount == 6 ) /* WakeLock ==> 6 = 6*10sec, WakeUnlock ==> 6 = ?? ( TBD )  */
        {
            trackingCount = 0;

            if( pVital->batCurrent > 0 )
            {
                pBmData->curSoc++;
            }
            else
            {
                pBmData->curSoc--;
            }

            if( pBmData->curSoc == trackingSoc )
            {
                trackingMethod = 0;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
    }
    else if( trackingMethod == 2 ) /* Keeping */
    {
        if( pVital->batCurrent > 0 )
        {
            if( trackingSoc == 99 )
            {
                if( eocState == 1 )
                {
                    trackingMethod = 0;
                    pBmData->curSoc++;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else if( trackingSoc == 15 )
            {
                if( batVolt > 3750 )
                {
                    trackingMethod = 0;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else
            {
                LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
            }
        }
        else
        {
            if( trackingSoc == 2 )
            {
                if( batVolt < 3300 )
                {
                    trackingMethod = 0;
                    pBmData->curSoc--;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else if( trackingSoc == 15 )
            {
                if( batVolt < 3750 )
                {
                    trackingMethod = 0;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else
            {
                LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
            }
        }
    }
    else
    {
        LGBM_ERR("Invalid Tracking Method ( %d )\n", trackingMethod);
    }

}
#endif //                                              
#else
void LGBM_UpdateCurSoc( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 newSoc = 0;
    kal_int32 eocState = 0;
    kal_int32 batVolt = pVital->batVolt;
    static kal_int32 trackingMethod = 0; /* 0=No tracking, 1=Running, 2=Keeping */
    static kal_int32 trackingVolt = 0;
    static kal_int32 trackingSoc = 0;
    static kal_int32 trackingCount = 0;

    eocState = LGBM_ReadEocState();
    newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, 1597);

    if( trackingMethod == 0 ) /* No tracking */
    {
        if( pVital->batCurrent > 0 )
        {
            if( eocState == 1 && newSoc < 100 )
            {
                trackingMethod = 1;
                trackingVolt = 0;
                trackingSoc = 100;
            }
            else if( eocState == 0 && newSoc >= 100 )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 99;
            }
            else if( newSoc == 15 && batVolt < g_lgbmSoc15Volt )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 15;
            }
            else if( newSoc < 15 && batVolt > g_lgbmSoc15Volt )
            {
                trackingMethod = 1;
                trackingVolt = batVolt;
                trackingSoc = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, batVolt);
                if( trackingSoc <= newSoc )
                {
                    trackingMethod = 0;
                    LGBM_ERR("Invalid Tracking Condition ( newSoc = %d, trackingSoc = %d )\n", newSoc, trackingSoc);
                }
            }
            else
            {
                pBmData->curSoc = newSoc;
            }

            if( trackingMethod != 0 )
            {
                trackingCount = 0;
                LGBM_LOG("SOC TRACKING : Charging : Method = %d, trackingSoc = %d, trackingVolt = %d\n", trackingMethod, trackingSoc, trackingVolt);
            }

        }
        else
        {
            if( newSoc > 1 && batVolt < 3300 )
            {
                trackingMethod = 1;
                trackingVolt = 0;
                trackingSoc = 1;
            }
            else if( newSoc == 2 && batVolt > 3300 )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 2;
            }
            else if( newSoc == 15 && batVolt > g_lgbmSoc15Volt )
            {
                trackingMethod = 2;
                trackingVolt = 0;
                trackingSoc = 15;
            }
            else if( newSoc > 15 && batVolt < g_lgbmSoc15Volt )
            {
                trackingMethod = 1;
                trackingVolt = batVolt;
                trackingSoc = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, batVolt);
                if( trackingSoc >= newSoc )
                {
                    trackingMethod = 0;
                    LGBM_ERR("Invalid Tracking Condition ( newSoc = %d, trackingSoc = %d )\n", newSoc, trackingSoc);
                }
            }
            else
            {
                pBmData->curSoc = newSoc;
            }

            if( trackingMethod != 0 )
            {
                trackingCount = 0;
                LGBM_LOG("SOC TRACKING : Dis-Charging : Method = %d, trackingSoc = %d, trackingVolt = %d\n", trackingMethod, trackingSoc, trackingVolt);
            }

        }

    }
    else if( trackingMethod == 1 ) /* Running */
    {
        trackingCount++;
        if( trackingCount == 6 ) /* WakeLock ==> 6 = 6*10sec, WakeUnlock ==> 6 = ?? ( TBD )  */
        {
            trackingCount = 0;

            if( pVital->batCurrent > 0 )
            {
                pBmData->curSoc++;
            }
            else
            {
                pBmData->curSoc--;
            }

            if( pBmData->curSoc == trackingSoc )
            {
                trackingMethod = 0;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
    }
    else if( trackingMethod == 2 ) /* Keeping */
    {
        if( pVital->batCurrent > 0 )
        {
            if( trackingSoc == 99 )
            {
                if( eocState == 1 )
                {
                    trackingMethod = 0;
                    pBmData->curSoc++;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else if( trackingSoc == 15 )
            {
                if( batVolt > 3750 )
                {
                    trackingMethod = 0;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else
            {
                LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
            }
        }
        else
        {
            if( trackingSoc == 2 )
            {
                if( batVolt < 3300 )
                {
                    trackingMethod = 0;
                    pBmData->curSoc--;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else if( trackingSoc == 15 )
            {
                if( batVolt < 3750 )
                {
                    trackingMethod = 0;
                    LGBM_ResetColumbCounter();
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
                }
            }
            else
            {
                LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
            }
        }
    }
    else
    {
        LGBM_ERR("Invalid Tracking Method ( %d )\n", trackingMethod);
    }

}
#endif //                                              

void LGBM_StateInit( LGBmData *pBmData, LGBmVital *pVital )
{
    LGBM_ENTRY();
}

void LGBM_StateNoCharger( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 newSoc = 0;
    kal_int32 batVolt = pVital->batVolt;
    static kal_int32 trackingMethod = 0; /* 0=No tracking, 1=Running, 2=Keeping */
    static kal_int32 trackingVolt = 0;
    static kal_int32 trackingSoc = 0;
    static kal_int32 trackingCount = 0;
    kal_int32 soc15Volt = 0;

    newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, g_lgbmQmax);
    soc15Volt = LGBM_GetBatVoltFromSoc(batt_id_check, pVital->batTemp, 15);

    if( pBmData->socTrackState == LGBM_SOC_UN_TRACKED )
    {
        trackingMethod = 0;
        trackingVolt = 0;
        trackingSoc = 0;
        trackingCount = 0;

        pBmData->socTrackState = LGBM_SOC_ON_TRACKING;
    }

    if( trackingMethod == 0 ) /* No tracking */
    {
        if( newSoc > 1 && batVolt < LGBM_CUT_OFF_VOLTAGE )
        {
            trackingMethod = 1;
            trackingVolt = 0;
            trackingSoc = 1;
        }
        else if( newSoc == 1 && batVolt > LGBM_CUT_OFF_VOLTAGE )
        {
            trackingMethod = 2;
            trackingVolt = 0;
            trackingSoc = 1;
        }
        else if( newSoc == 14 && batVolt > soc15Volt )
        {
            trackingMethod = 2;
            trackingVolt = soc15Volt;
            trackingSoc = 14;
        }
        else if( newSoc > 15 && batVolt < soc15Volt )
        {
            trackingMethod = 1;
            trackingVolt = batVolt;
            trackingSoc = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, batVolt);
            if( trackingSoc >= newSoc )
            {
                trackingMethod = 0;
                LGBM_ERR("Invalid Tracking Condition ( newSoc = %d, trackingSoc = %d )\n", newSoc, trackingSoc);
            }
        }
        else
        {
        	if(newSoc>=100 && prevBmState==LGBM_TS_CHARGING)
       		{
	       		pBmData->curSoc=99;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
				LGBM_LOG("New soc is over 100, reset ColumbCounter set initsoc and curSoc = %d",pBmData->initSoc);
       		
       		}
			else
	            pBmData->curSoc = newSoc;
        }

        if( trackingMethod != 0 )
        {
            trackingCount = 0;
            LGBM_ResetColumbCounter();
            pBmData->initSoc = pBmData->curSoc;
            LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d, trackingVolt = %d\n", trackingMethod, trackingSoc, trackingVolt);
        }

    }
    else if( trackingMethod == 1 ) /* Running */
    {
        trackingCount++;
        if( trackingCount == 6 ) /* WakeLock ==> 6 = 6*10sec, WakeUnlock ==> 6 = ?? ( TBD )  */
        {
            trackingCount = 0;

            pBmData->curSoc--;

            if( pBmData->curSoc == trackingSoc )
            {
                trackingMethod = 0;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
    }
    else if( trackingMethod == 2 ) /* Keeping */
    {
        if( trackingSoc == 1 )
        {
            if( batVolt < LGBM_CUT_OFF_VOLTAGE )
            {
                trackingMethod = 0;
                pBmData->curSoc--;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
        else if( trackingSoc == 14 )
        {
            if( batVolt < trackingVolt )
            {
                trackingMethod = 0;
                pBmData->curSoc--;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
        else
        {
            LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
        }
    }
    else
    {
        LGBM_ERR("Invalid Tracking Method ( %d )\n", trackingMethod);
    }

#if 1 //                                              
#else
    kal_int32 newSoc = 0;
    kal_int32 tmpSoc = 0;
    kal_int32 socByOcv = 0;
    static kal_int32 runningCount = 6;
    static kal_int32 trackingMethod = 0; /* keeping = 0, running = 1 */

    LGBM_ENTRY();

    tmpSoc = pBmData->curSoc;

    if( pBmData->curSoc == 1 )
    {
        LGBM_LOG("NO_CHARGER : SOC = 1\n");
        /* do nothing */
    }
    else if( pBmData->curSoc == 2 )
    {
        LGBM_LOG("NO_CHARGER : SOC = 2\n");
        if( pVital->batVolt < 3300 )
        {
            LGBM_LOG("NO_CHARGER : batVolt < 3300\n");
            pBmData->curSoc--;
            pBmData->initSoc = pBmData->curSoc;
            //LGBM_ResetColumbCounter();
        }
    }
    else if( pVital->batVolt < 3300 )
    {
        LGBM_LOG("NO_CHARGER : batVolt < 3300\n");
        pBmData->curSoc--;
        pBmData->initSoc = pBmData->curSoc;
        LGBM_ResetColumbCounter();
    }
    else
    {
        if( pBmData->socTrackState == LGBM_SOC_UN_TRACKED )
        {
            LGBM_LOG("NO_CHARGER : UN_TRACKED\n");
            if( pVital->batVolt > 3700 && pBmData->curSoc > 15 )
            {
                newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, 1597);
                pBmData->curSoc = newSoc;
            }
            else
            {
                pBmData->socTrackState = LGBM_SOC_ON_TRACKING;
                socByOcv = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, pVital->batVolt);
                LGBM_LOG("NO_CHARGER : socByOcv  = %d, pBmData->curSoc = %d\n", socByOcv, pBmData->curSoc);
                if( socByOcv > pBmData->curSoc )
                {
                    /* keeping */
                    trackingMethod = 0;
                    LGBM_LOG("NO_CHARGER : Track Method = Keeping\n");
                }
                else
                {
                    /* running */
                    trackingMethod = 1;
                    runningCount = 6;
                    LGBM_LOG("NO_CHARGER : Track Method = Running\n");
                }
            }
        }
        else if( pBmData->socTrackState == LGBM_SOC_ON_TRACKING )
        {
            LGBM_LOG("NO_CHARGER : ON_TRACKING\n");
            socByOcv = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, pVital->batVolt);

            if( trackingMethod == 0 )
            {
                if( socByOcv > pBmData->curSoc )
                {
                    /* do nothing */
                }
                else
                {
                    pBmData->socTrackState = LGBM_SOC_TRACKED;
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_ResetColumbCounter();
                    LGBM_LOG("NO_CHARGER : TRACKED : socByOcv  = %d, pBmData->curSoc = %d\n", socByOcv, pBmData->curSoc);
                }
            }
            else
            {
                if( socByOcv < pBmData->curSoc )
                {
                    runningCount--;
                    if( runningCount == 0 )
                    {
                        runningCount = 6;
                        pBmData->curSoc--;
                    }
                }
                else
                {
                    pBmData->socTrackState = LGBM_SOC_TRACKED;
                    pBmData->initSoc = pBmData->curSoc;
                    LGBM_ResetColumbCounter();
                    LGBM_LOG("NO_CHARGER : TRACKED : socByOcv  = %d, pBmData->curSoc = %d\n", socByOcv, pBmData->curSoc);
                }
            }
        }
        else
        {
            LGBM_LOG("NO_CHARGER : TRACKED\n");
            newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, 1597);
            pBmData->curSoc = newSoc;
        }
    }

    if( tmpSoc > pBmData->curSoc )
    {
        LGBM_UpdateBatStatus( pBmData, pVital, &mt6329_battery_main );
    }
#endif //                                              

}

void LGBM_StateCharging( LGBmData *pBmData, LGBmVital *pVital )
{
    LGBmOTPState newOtpState = LGBM_OTP_UNKNOWN;
    kal_int32 stopChargingVolt = 0;
    kal_int32 batVolt = pVital->batVolt;
    kal_int32 newSoc = 0;
    kal_int32 eocState = 0;
    int tmpQmax = 0;

    static kal_int32 trackingMethod = 0; /* 0=No tracking, 1=Running, 2=Keeping */
    static kal_int32 trackingSoc = 0;
    static kal_int32 trackingCount = 0;

    newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, g_lgbmQmax);

    if( pBmData->bmOTPState == LGBM_OTP_NORMAL_CHARGING )
    {
        eocState = LGBM_ReadEocState(pVital);
    }

    if( pBmData->bmOTPState == LGBM_OTP_UNKNOWN )
    {
        trackingMethod = 0;
        trackingSoc = 0;
        trackingCount = 0;
    }

    if( trackingMethod == 0 ) /* No tracking */
    {
        if( eocState == 1 )
        {
            trackingMethod = 1;
            trackingSoc = 100;
            if( pVital->accColumb >= LGBM_Q_MAX_UPDATE_TH )
            {
                tmpQmax = ( pVital->accColumb * 100 / ( 100 - pBmData->initSoc ) );
                if( tmpQmax >= LGBM_BAT_CAPACITY_MIN && tmpQmax <= LGBM_BAT_CAPACITY_MAX )
                {
                    g_lgbmQmax = tmpQmax;
                    LGBM_LOG("Qmax was re-calculated ( Qmax = %d )\n", g_lgbmQmax);
                }
            }
        }
        else if( eocState == 0 && newSoc >= 100 )
        {
            trackingMethod = 2;
            trackingSoc = 100;
            if( pVital->accColumb >= LGBM_Q_MAX_UPDATE_TH )
            {
                tmpQmax = ( pVital->accColumb * 100 / ( 100 - pBmData->initSoc ) );
                if( tmpQmax >= LGBM_BAT_CAPACITY_MIN && tmpQmax <= LGBM_BAT_CAPACITY_MAX )
                {
                    g_lgbmQmax = tmpQmax;
                    LGBM_LOG("Qmax was re-calculated ( Qmax = %d )\n", g_lgbmQmax);
                }
            }
        }
        else
        {
            pBmData->curSoc = newSoc;
        }

        if( trackingMethod != 0 )
        {
            trackingCount = 0;
            LGBM_ResetColumbCounter();
            pBmData->initSoc = pBmData->curSoc;
            LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d\n", trackingMethod, trackingSoc);
        }

    }
    else if( trackingMethod == 1 ) /* Running */
    {
        trackingCount++;
        if( trackingCount == 6 ) /* WakeLock ==> 6 = 6*10sec, WakeUnlock ==> 6 = ?? ( TBD )  */
        {
            trackingCount = 0;

            pBmData->curSoc++;

            if( pBmData->curSoc == trackingSoc )
            {
                trackingMethod = 0;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
    }
    else if( trackingMethod == 2 ) /* Keeping */
    {
        if( trackingSoc == 100 )
        {
            if( eocState == 1 )
            {
                trackingMethod = 0;
                pBmData->curSoc++;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
        else
        {
            LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
        }
    }
    else
    {
        LGBM_ERR("Invalid Tracking Method ( %d )\n", trackingMethod);
    }

    if( pBmData->tempState == LGBM_TEMP_LOW || pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
    {
        newOtpState = LGBM_OTP_STOP_CHARGING;
    }
    else if( pBmData->tempState == LGBM_TEMP_HIGH )
    {
        if( pBmData->bmOTPState == LGBM_OTP_DECREASE_CHARGING )
        {
            stopChargingVolt = LGBM_OTP_STOP_VOLTAGE;
        }
        else
        {
            stopChargingVolt = LGBM_OTP_HYST_VOLTAGE;
        }

        if( pVital->batVolt > stopChargingVolt )
        {
            newOtpState = LGBM_OTP_STOP_CHARGING;
        }
        else
        {
            newOtpState = LGBM_OTP_DECREASE_CHARGING;
        }
    }
    else
    {
        newOtpState = LGBM_OTP_NORMAL_CHARGING;
    }

    if( pBmData->bmOTPState != newOtpState )
    {
        pBmData->bmOTPState = newOtpState;

        if( newOtpState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( pBmData->charger == NONSTANDARD_CHARGER || pBmData->charger == STANDARD_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_I_SET);
            }
            else
            {
                LGBM_SetChargingCurrent(LGBM_CC_USB_500);
            }
        }
        else if ( newOtpState == LGBM_OTP_DECREASE_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_USB_500);
        }
        else if( newOtpState == LGBM_OTP_STOP_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_OFF);
        }
        else
        {
            LGBM_ERR("Invalid OtpState ( %d )\n", newOtpState);
        }

        switch( newOtpState )
        {
            case LGBM_OTP_NORMAL_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_NORMAL_CHARGING\n");
                break;
            case LGBM_OTP_DECREASE_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_DECREASE_CHARGING\n");
                break;
            case LGBM_OTP_STOP_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_STOP_CHARGING\n");
                break;
            default:
                LGBM_ERR("Invalid OPT_STATE ( %d )\n", newOtpState);
                break;
        }

    }
    else
    {
        if( pBmData->bmOTPState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( eocState == 1 && pBmData->curSoc >= 100 )
            {
                if( pVital->accColumb >= LGBM_Q_MAX_UPDATE_TH )
                {
                    tmpQmax = ( pVital->accColumb * 100 / ( 100 - pBmData->initSoc ) );
                    if( tmpQmax >= LGBM_BAT_CAPACITY_MIN && tmpQmax <= LGBM_BAT_CAPACITY_MAX )
                    {
                        g_lgbmQmax = tmpQmax;
                        LGBM_LOG("Qmax was re-calculated ( Qmax = %d )\n", g_lgbmQmax);
                    }
                }
                LGBM_ChangeState( pBmData, LGBM_TS_CHARGING_FULL );
                pBmData->curSoc = 102;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
            }
        }
    }

}

void LGBM_StateChargingFull( LGBmData *pBmData, LGBmVital *pVital )
{
    LGBmOTPState newOtpState = LGBM_OTP_UNKNOWN;
    kal_int32 stopChargingVolt = 0;
    kal_int32 newSoc = 0;

    newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, g_lgbmQmax);
    pBmData->curSoc = newSoc;

    if( pBmData->tempState == LGBM_TEMP_LOW || pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
    {
        newOtpState = LGBM_OTP_STOP_CHARGING;
    }
    else if( pBmData->tempState == LGBM_TEMP_HIGH )
    {
        if( pBmData->bmOTPState == LGBM_OTP_DECREASE_CHARGING )
        {
            stopChargingVolt = LGBM_OTP_STOP_VOLTAGE;
        }
        else
        {
            stopChargingVolt = LGBM_OTP_HYST_VOLTAGE;
        }

        if( pVital->batVolt > stopChargingVolt )
        {
            newOtpState = LGBM_OTP_STOP_CHARGING;
        }
        else
        {
            newOtpState = LGBM_OTP_DECREASE_CHARGING;
        }
    }
    else /* pBmData->tempState == LGBM_TEMP_MIDDLE */
    {
        if( pBmData->bmOTPState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( LGBM_ReadEocState(pVital) == 1 )
            {
                newOtpState = LGBM_OTP_STOP_CHARGING;
                pBmData->curSoc = 102;
                LGBM_ResetColumbCounter();
                pBmData->initSoc = pBmData->curSoc;
            }
            else
            {
                newOtpState = LGBM_OTP_NORMAL_CHARGING;
            }
        }
        else
        {
            //if( pVital->batVolt > RECHARGING_VOLTAGE )
//                                                                                            
#if defined(CONFIG_MAX8971_CHARGER)
	        if(max8971_eoc_status()== 1)
#else					
            if(pBmData->curSoc > 100)
#endif
//                                                                                            
            {
                newOtpState = LGBM_OTP_STOP_CHARGING;
            }
            else
            {
                newOtpState = LGBM_OTP_NORMAL_CHARGING;
            }
        }
    }

    if( pBmData->bmOTPState != newOtpState )
    {
        pBmData->bmOTPState = newOtpState;

        if( newOtpState == LGBM_OTP_NORMAL_CHARGING )
        {
//                                                                                                
#if defined(CONFIG_MAX8971_CHARGER)	
			LGBM_LOG("MAX8971 eoc register changed... Recharging_Start  \n");			
#else
            if( pBmData->charger == NONSTANDARD_CHARGER || pBmData->charger == STANDARD_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_I_SET);
            }
            else
            {
                LGBM_SetChargingCurrent(LGBM_CC_USB_500);
            }
#endif
//                                                                                                
        }
        else if ( pBmData->bmOTPState == LGBM_OTP_DECREASE_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_USB_500);
        }
        else if( pBmData->bmOTPState == LGBM_OTP_STOP_CHARGING )
        {
//                                                                                                  
#if defined(CONFIG_MAX8971_CHARGER)
//			max8971_aicl_on();
//			LGBM_LOG("MAX8971 EOC register OK Charging_Stop AICL on \n");
#else			
            LGBM_SetChargingCurrent(LGBM_CC_OFF);
#endif
//                                                                                                   
        }
        else
        {
            LGBM_ERR("Invalid OtpState ( %d )\n", newOtpState);
        }

        switch( newOtpState )
        {
            case LGBM_OTP_NORMAL_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_NORMAL_CHARGING\n");
                break;
            case LGBM_OTP_DECREASE_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_DECREASE_CHARGING\n");
                break;
            case LGBM_OTP_STOP_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_STOP_CHARGING\n");
                break;
            default:
                LGBM_ERR("Invalid OPT_STATE ( %d )\n", newOtpState);
                break;
        }

    }
	
	if(pBmData->curSoc  <= LGBM_RECHARGING_SOC && max8971_eoc_status() )
	{
		pBmData->curSoc = 100;
		LGBM_ResetColumbCounter();
		pBmData->initSoc = pBmData->curSoc;

	}
    if( pBmData->curSoc  <= LGBM_CHARGING_FULL_TO_CHARGING_SOC)
    {
        if((prevChgCurrent == LGBM_CC_I_SET) 
#ifdef CONFIG_MAX8971_CHARGER
		&& (max8971_aicl_check()==1)
#elif (CONFIG_MINIABB_CHARGER)
		
#else

#endif
	)
   		{
            LGBM_SetChargingCurrent(LGBM_CC_OFF);
			mdelay(20);
            LGBM_SetChargingCurrent(LGBM_CC_I_SET);	
            LGBM_LOG("AICL Recovery code operate\n");

   		}
		else if((prevChgCurrent == LGBM_CC_USB_500) 
#ifdef CONFIG_MAX8971_CHARGER
		&& (max8971_aicl_check()==1)
#elif (CONFIG_MINIABB_CHARGER)
#else
	
#endif
			)
		{
            LGBM_SetChargingCurrent(LGBM_CC_OFF);
			mdelay(20);
            LGBM_SetChargingCurrent(LGBM_CC_USB_500);	
            LGBM_LOG("AICL Recovery code operate\n");			
		}
        LGBM_ChangeState( pBmData, LGBM_TS_CHARGING );
    }

}

void LGBM_StateFactory( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 batExist = 0;
    kal_int32 newSoc = 0;

    LGBM_ENTRY();

    batExist = LGBM_ReadBatExistance();
    if( pBmData->batExist != batExist )
    {
        pBmData->batExist = batExist;

        LGBM_ResetColumbCounter();

        LGBM_LOG("BATTERY = %d\n", batExist);

        if( batExist == 1 )
        {
            /* Need to check battery ID */

            pBmData->initSoc = LGBM_GetSocByOcv(batt_id_check, pVital->batTemp, pVital->batVolt);
            pBmData->curSoc = pBmData->initSoc;
        }
        else
        {
            pBmData->initSoc = 100;
            pBmData->curSoc = 100;
        }

        LGBM_UpdateBatStatus( pBmData, pVital, &mt6329_battery_main );

    }
    else
    {
        if( batExist == 1 )
        {
            newSoc = LGBM_GetNewSoc(pBmData->initSoc, pVital->accColumb, g_lgbmQmax);
        }
    }

}

void LGBM_StateInsertBattery( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 batExist = 0;

    LGBM_ENTRY();

    batExist = LGBM_ReadBatExistance();
    if( batExist == 1 )
    {
        LGBM_LOG("BATTERY = %d so do power down\n", batExist);

        if( LGBM_WriteSocToFile(255) )
        {
            LGBM_LOG("Success to clear the stored SOC value\n");
        }
        else
        {
            LGBM_ERR("Fail to clear the stored SOC value\n");
        }

        msleep(100);

        mt_power_off();
    }
    else
    {
        pBmData->initSoc = 0;
        pBmData->curSoc = 0;
    }
}

extern int g_switch_to_i2c_polling_mode;
kal_int32 get_dynamic_period(int first_use, int first_wakeup_time, int battery_capacity_level)
{
    kal_int32 sleepCurrent = 0;
    kal_int32 columbCount = 0;
    static kal_int32 sleepDuration = 0;
    static kal_int32 sleepColumb = 0;
    static kal_int32 awakeColumb = 0;

    LGBM_ENTRY();

    LGBM_DBG("PreCondition : sleepColumb=%d[mAh], awakeColumb=%d[mAh], sleepDuration=%d[sec]\n", sleepColumb, awakeColumb, sleepDuration);

    LGBM_DBG("Input : first_use=%d, first_wakeup_time=%d[sec], battery_capacity_level=%d[%]\n", first_use, first_wakeup_time, battery_capacity_level);

    g_switch_to_i2c_polling_mode=1;
    columbCount = LGBM_ReadCumulatedColumb();
    g_switch_to_i2c_polling_mode=0;

    if( first_use == 1 || sleepDuration == 0 )
    {
        sleepDuration = first_wakeup_time;
        sleepColumb = columbCount;
    }
    else
    {
        awakeColumb = columbCount;

        if( awakeColumb == sleepColumb )
        {
            sleepDuration = first_wakeup_time;
        }
        else
        {
            if( awakeColumb > sleepColumb )
            {
                LGBM_DBG("awakeColumb is bigger than sleepColumb and it means charging case\n");
                sleepCurrent = ( awakeColumb - sleepColumb )*3600 / sleepDuration;
            }
            else
            {
                LGBM_DBG("awakeColumb is smaller than sleepColumb and it means discharging case\n");
                sleepCurrent = ( sleepColumb - awakeColumb )*3600 / sleepDuration;
            }

            if( sleepCurrent == 0 )
            {
                LGBM_LOG("sleepDuration is zero, so set to default duration(%d[sec])\n", first_wakeup_time);
                sleepDuration = first_wakeup_time;
            }
            else
            {
                sleepDuration = ((1597*battery_capacity_level*3600)/100)/sleepCurrent; /* NUGUYU_TBD : should use Qmax */
            }
        }

        if( sleepDuration < first_wakeup_time )
        {
            LGBM_LOG("sleepDuration is smaller than default duration, so set to default duration(%d[sec])\n", first_wakeup_time);
            sleepDuration = first_wakeup_time;
        }

        LGBM_DBG("sleepColumb=%d[mAh], awakeColumb=%d[mAh], sleepCurrent=%d[mA], sleepDuration=%d[sec]\n", sleepColumb, awakeColumb, sleepCurrent, sleepDuration);
        LGBM_LOG("new sleepDuration=%d[sec]\n", sleepDuration);

        sleepColumb = awakeColumb;

    }

    return sleepDuration;

}

int bat_thread_kthread(void *x)
{
    kal_bool delayedInitDone = KAL_FALSE;
    LGBmVital batVital = { 0, 0, 0, 0, 0, 0 };
    LGBmData bmData;
    kal_bool chargerChanged = KAL_FALSE;
    LGBmState newBmState = LGBM_TS_UNKNOWN;
    LGBmTempState newTempState = LGBM_TEMP_UNKNOWN;
    CHARGER_TYPE prevCharger = CHARGER_UNKNOWN;
    int initSocGap = 0;
    kal_int32 readSoc = 0;

    g_pAtCmdBatVital = &batVital;
    g_pAtCmdBmData = &bmData;

    bmData.initSoc = 0;
    bmData.curSoc = 0;
    bmData.batExist = 1;
    bmData.charger = CHARGER_UNKNOWN;
    bmData.bmState = LGBM_TS_INIT;
    bmData.bmOTPState = LGBM_OTP_NORMAL_CHARGING;
    bmData.bmOTPChanged = KAL_FALSE;
    bmData.socTrackState = LGBM_SOC_UN_TRACKED;
    bmData.tempState = LGBM_TEMP_UNKNOWN;

    LGBM_Initialize();

#if 1 //                                              
#else
    LGBM_ReadBatVital(&batVital);
    newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
    bmData.tempState = newTempState;

    bmData.initSoc = LGBM_GetSocByOcv(batt_id_check, batVital.batTemp, batVital.batVolt);
    LGBM_LOG("Newly Measured Initial SOC = %d\n", bmData.initSoc);

    g_lgbmSoc15Volt = LGBM_GetBatVoltFromSoc(batt_id_check, batVital.batTemp, 15);
    LGBM_LOG("g_lgbmSoc15Volt = %d\n", g_lgbmSoc15Volt);
#endif //                                              

    /* Run on a process content */
    while (1) {

        wait_event(bat_thread_wq, bat_thread_timeout);

        bat_thread_timeout=0;

        mutex_lock(&bat_mutex);

        if( delayedInitDone == KAL_FALSE )
        {
            if( g_LGBM_wakeup_by_charger == 1 )
            {
                g_LGBM_wakeup_by_charger = 0;

                LGBM_LOG("Charger Event Happened before delayed init processing\n");
            }
            else
            {
                LGBM_LOG("Delayed init processing was started\n");

                BatThread_XGPTConfig();

                LGBM_ReadBatVital(&batVital);
                newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
                bmData.tempState = newTempState;

#if 1 //                                              
#else
                g_lgbmSoc15Volt = LGBM_GetBatVoltFromSoc(batt_id_check, batVital.batTemp, 15);
                LGBM_LOG("g_lgbmSoc15Volt = %d\n", g_lgbmSoc15Volt);
#endif //                                              

                bmData.batExist = LGBM_ReadBatExistance();
                LGBM_LOG("BATTERY = %d\n", bmData.batExist);

                bmData.initSoc = LGBM_GetSocByOcv(batt_id_check, batVital.batTemp, batVital.batVolt);
                LGBM_LOG("Newly Measured Initial SOC = %d\n", bmData.initSoc);

#if 1 //                                              
                readSoc = LGBM_ReadSocFromFile();
                if( readSoc == -1 )
                {
                    LGBM_ERR("Fail to read the stored SOC value\n");
                }
                else if( readSoc == 255 )
                {
                    LGBM_LOG("Stored SOC was not written\n");
                }
                else if( readSoc > 0 && readSoc <= 100 )
                {
                    if( readSoc > bmData.initSoc )
                    {
                        initSocGap = readSoc - bmData.initSoc;

						if( initSocGap > 5)
						{
							readSoc = readSoc - 1;
							LGBM_LOG("readSoc is tracking initSoc (readSoc= %d )\n",readSoc);
							
						}

                    }
                    else
                    {
                        initSocGap = bmData.initSoc - readSoc;
						
                    }
                    #if 0 /* rollback to fix side effect */
                    if( 5 < initSocGap && initSocGap < 10 )
                    {
                        bmData.initSoc = readSoc;
                        LGBM_LOG("Stored SOC is close to newly measured SOC, so we will use stored SOC ( %d )\n", bmData.initSoc);
                    }
                    #else
					if( initSocGap < 15 )
	                {
						bmData.initSoc = readSoc;
						LGBM_LOG("Stored SOC is close to newly measured SOC, so we will use stored SOC ( %d )\n", bmData.initSoc);
	                }

                    #endif
                }
                else
                {
                    LGBM_ERR("Stored SOC is out of range ( SOC = %d )\n", readSoc);
                }

                if( readSoc != 255 )
                {
                    if( LGBM_WriteSocToFile(255) )
                    {
                        LGBM_LOG("Success to clear the stored SOC value\n");
                    }
                    else
                    {
                        LGBM_ERR("Fail to clear the stored SOC value\n");
                    }
                }

                LGBM_LOG("Decided Initial SOC = %d\n", bmData.initSoc);

                bmData.curSoc = bmData.initSoc;
#endif //                                              

                if( LGBM_ReadChargerExistance() == KAL_TRUE )
                {
                    bmData.charger = LGBM_GetCharger();
                }
                else
                {
                    bmData.charger = CHARGER_UNKNOWN;
                }

                if( bmData.charger == FACTORY_CHARGER )
                {
                    newBmState = LGBM_TS_FACTORY;
                    LGBM_SetUsbConnection(KAL_TRUE);
                    LGBM_UpdateUsbStatus(KAL_TRUE);
                }
                else if( bmData.charger == CHARGER_UNKNOWN )
                {
                    newBmState = LGBM_TS_NO_CHARGER;
                    LGBM_SetUsbConnection(KAL_FALSE);
                    LGBM_UpdateUsbStatus(KAL_FALSE);
                    LGBM_UpdateAcStatus(KAL_FALSE);
                }
                else
                {
#ifdef CONFIG_MAX8971_CHARGER
					max8971_charger_reset();
#elif (CONFIG_MINIABB_CHARGER)
#else

#endif
	                LGBM_LOG("MAX8971 Charger reset\n");
					
                    if( bmData.batExist == 1 )
                    {
                        if( bmData.curSoc < 100 )
                        {
                            newBmState = LGBM_TS_CHARGING;
                        }
                        else
                        {
                            newBmState = LGBM_TS_CHARGING_FULL;
                        }

                    }
                    else
                    {
                        newBmState = LGBM_TS_INSERT_BATTERY;
                        bmData.initSoc = 0;
                        bmData.curSoc = 0;
                    }

                    LGBM_SetWakeLock(KAL_TRUE);

                    if( bmData.charger == NONSTANDARD_CHARGER || bmData.charger == STANDARD_CHARGER )
                    {
                        LGBM_UpdateAcStatus(KAL_TRUE);
                    }
                    else
                    {
                        LGBM_SetUsbConnection(KAL_TRUE);
                        LGBM_UpdateUsbStatus(KAL_TRUE);
                    }

                }

                LGBM_ChangeState( &bmData, newBmState );

                LGBM_UpdateBatStatus( &bmData, &batVital, &mt6329_battery_main );

                LGBM_LOG("Delayed init processing was done\n");

                delayedInitDone = KAL_TRUE;

                if(g_updateBatStateFlag == 0)
                {
                    g_updateBatStateFlag = 1;
                }

            }

        }
        else
        {
            if( g_LGBM_wakeup_by_charger == 1 )
            {
                g_LGBM_wakeup_by_charger = 0;

                LGBM_DBG("Battery Thread Was Called By Charger Event ( PMIC or USB )\n");

                prevCharger = bmData.charger;

                if( LGBM_ReadChargerExistance() == KAL_TRUE )
                {
                    if( prevCharger == CHARGER_UNKNOWN )
                    {
                        bmData.charger = LGBM_GetCharger();
                    }
                }
                else
                {
                    #if defined ( ENABLE_SUPPORT_STUPID_CHARGER )
                    LGBM_ReadBatVital(&batVital);
                    if( batVital.chgVolt > 3000 )
                    {
                        g_stupidCharger = 1 ;

                        GPT_Stop(GPT5);
                        GPT_SetCompare(GPT5, 1*512);
                        GPT_Restart(GPT5);
                    }
                    else
                    {
                        bmData.charger = CHARGER_UNKNOWN;
                    }
                    #else
                    bmData.charger = CHARGER_UNKNOWN;
                    #endif
                }

                if( prevCharger != bmData.charger )
                {
                    LGBM_ReadBatVital(&batVital);
                    newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
                    bmData.tempState = newTempState;

                    newBmState = LGBM_TS_UNKNOWN; /* why do we need ? */

                    if( bmData.charger == FACTORY_CHARGER )
                    {
                        newBmState = LGBM_TS_FACTORY;
                        LGBM_SetUsbConnection(KAL_TRUE);
                        LGBM_UpdateUsbStatus(KAL_TRUE);
                    }
                    else if( bmData.charger == CHARGER_UNKNOWN )
                    {
                        newBmState = LGBM_TS_NO_CHARGER;
                        if( prevCharger == NONSTANDARD_CHARGER || prevCharger == STANDARD_CHARGER )
                        {
                            LGBM_UpdateAcStatus(KAL_FALSE);
                        }
                        else
                        {
                            LGBM_SetUsbConnection(KAL_FALSE);
                            LGBM_UpdateUsbStatus(KAL_FALSE);
                        }

                        #if 1 //                                                                                                                                                        
                        #else
                        LGBM_SetWakeLock(KAL_FALSE);
                        #endif //                                              
                    }
                    else
                    {
                        if( bmData.curSoc < 100 )
                        {
                            newBmState = LGBM_TS_CHARGING;
                        }
                        else
                        {
                            newBmState = LGBM_TS_CHARGING_FULL;
                        }

                        LGBM_SetWakeLock(KAL_TRUE);

                        if( bmData.charger == NONSTANDARD_CHARGER || bmData.charger == STANDARD_CHARGER )
                        {
                            LGBM_UpdateAcStatus(KAL_TRUE);
                        }
                        else
                        {
                            LGBM_SetUsbConnection(KAL_TRUE);
                            LGBM_UpdateUsbStatus(KAL_TRUE);
                        }
                    }

                    LGBM_ChangeState( &bmData, newBmState );

                    LGBM_UpdateBatStatus( &bmData, &batVital, &mt6329_battery_main );

                }

            }
            else
            {
                kal_int32 eocState = 0;
                kal_int32 newSoc = 0;

                LGBM_DBG("Battery Thread Was Called By Timer\n");

                LGBM_ReadBatVital(&batVital);
                newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
                bmData.tempState = newTempState;
				
				/*max8971 charger register reset check, charging fail check*/
				if(prevChgCurrent == LGBM_CC_USB_500 || prevChgCurrent == LGBM_CC_I_SET || prevChgCurrent == LGBM_CC_FACTORY)
				{
				
#ifdef CONFIG_MAX8971_CHARGER
					if((max8971_register_reset_check()==1)||(max8971_charging_fail_check()==1))
					{
						max8971_charger_reset();
						if(max8971_uvp_check())
						{
							max8971_start_charging(500);
						}
						else
						{
							if(prevChgCurrent==LGBM_CC_USB_500)
								max8971_start_charging(500);
							else if(prevChgCurrent==LGBM_CC_I_SET)
								max8971_start_charging(1200);
							else
								max8971_start_charging(500);
						}
					}
#elif (CONFIG_MINIABB_CHARGER)
		
#else
					
#endif
				}
				
                if( bmData.bmState == LGBM_TS_CHARGING || bmData.bmState == LGBM_TS_CHARGING_FULL )
                {
                    if( bmData.bmOTPState == LGBM_OTP_NORMAL_CHARGING || bmData.bmOTPState == LGBM_OTP_DECREASE_CHARGING )
                    {
                        if( LGBM_ReadBatExistance() == 0 )
                        {
                            /* turn off charging to power shut down in case of battery removal */
			    #ifdef TARGET_S1_75
			    mt_power_off();
			    #else
			    LGBM_SetChargingCurrent(LGBM_CC_OFF);
			    #endif
                        }
                    }

                    #if defined ( ENABLE_SUPPORT_STUPID_CHARGER )
                    if( batVital.chgVolt < 3000 )
                    {
                        g_stupidCharger = 0 ;
                        GPT_Stop(GPT5);
                        GPT_SetCompare(GPT5, 10*512);
                        GPT_Restart(GPT5);
                        bmData.charger = CHARGER_UNKNOWN;
                        newBmState = LGBM_TS_NO_CHARGER;
                        if( prevCharger == NONSTANDARD_CHARGER || prevCharger == STANDARD_CHARGER )
                        {
                            LGBM_UpdateAcStatus(KAL_FALSE);
                        }
                        else
                        {
                            LGBM_SetUsbConnection(KAL_FALSE);
                            LGBM_UpdateUsbStatus(KAL_FALSE);
                        }

                        LGBM_ChangeState( &bmData, newBmState );

                        LGBM_UpdateBatStatus( &bmData, &batVital, &mt6329_battery_main );

                        msleep(100);

                        LGBM_SetWakeLock(KAL_FALSE);

                    }
                    #endif

                }
                //LGBM_UpdateCurSoc(&bmData, &batVital);
                //newSoc = LGBM_GetNewSoc(bmData.initSoc, batVital.accColumb, 1597);
                //bmData.curSoc = newSoc;

                switch( bmData.bmState )
                {
                    case LGBM_TS_INIT:
                        //LGBM_DBG("BM_STATE = LGBM_TS_INIT\n");
                        LGBM_StateInit( &bmData, &batVital );
                        break;

                    case LGBM_TS_NO_CHARGER:
                        //LGBM_DBG("BM_STATE = LGBM_TS_NO_CHARGER\n");
                        /* CAUTION : fix sleep current problem after set USB connection type, it can save about 1 ~ 2mA */
                        mt_usb_disconnect();
                        /* CAUTION : fix sleep current problem caused by wake lock ( pmic interrupt set wake lock ) */
                        if( wake_lock_active(&battery_suspend_lock) )
                        {
                            LGBM_SetWakeLock(KAL_FALSE);
                        }
                        LGBM_StateNoCharger( &bmData, &batVital );
                        break;

                    case LGBM_TS_CHARGING:
                        //LGBM_DBG("BM_STATE = LGBM_TS_CHARGING\n");
                        LGBM_StateCharging( &bmData, &batVital );
                        break;

                    case LGBM_TS_CHARGING_FULL:
                        //LGBM_DBG("BM_STATE = LGBM_TS_CHARGING_FULL\n");
                        LGBM_StateChargingFull( &bmData, &batVital );
                        break;

                    case LGBM_TS_FACTORY:
                        //LGBM_DBG("BM_STATE = LGBM_TS_FACTORY\n");
                        LGBM_StateFactory( &bmData, &batVital );
                        break;

                    case LGBM_TS_INSERT_BATTERY:
                        //LGBM_DBG("BM_STATE = LGBM_TS_INSERT_BATTERY\n");
                        LGBM_StateInsertBattery( &bmData, &batVital );
                        break;

                    default:
                        LGBM_ERR("Invalid BM_STATE ( %d )\n", bmData.bmState);
                        break;

                }

                LGBM_UpdateBatStatus( &bmData, &batVital, &mt6329_battery_main );

#if 1 //                                              
#else
                {
                    static int toggle = 1;

                    toggle = 1 - toggle;

                    if( toggle == 1 )
                    {
                        LGBM_SetChargingCurrent(LGBM_CC_I_SET);
                    }
                    else
                    {
                        LGBM_SetChargingCurrent(LGBM_CC_USB_500);
                    }
                }
#endif //                                              

            }

        }

        mutex_unlock(&bat_mutex);

    }

    return 0;
}
#if defined (CONFIG_MINIABB_CHARGER)
/*                                                                          */
int Is_Not_FactoryCable_PowerOn()
{
#if defined(LGE_BSP_LGBM)
    if(g_lgbmBootUsbCableId == LGBM_CABLE_ID_UNKNOWN)
    {
        g_lgbmBootUsbCableId = LGBM_ReadUsbCableId();
	}

    if (g_lgbmBootUsbCableId == LGBM_CABLE_ID_56K       ||
        g_lgbmBootUsbCableId == LGBM_CABLE_ID_130K      ||
        g_lgbmBootUsbCableId == LGBM_CABLE_ID_910K)
    	{
        	return 0;
    	}
#endif
    return 1;
}
/*                                                                          */
#endif

#else

typedef struct{
    INT32 BatteryTemp;
    INT32 TemperatureR;
}BATT_TEMPERATURE;

/* convert register to temperature  */
INT16 BattThermistorConverTemp(INT32 Res)
{
    int i=0;
    INT32 RES1=0,RES2=0;
    INT32 TBatt_Value=-200,TMP1=0,TMP2=0;

    //                                                                                      
    int table_size = 0;
    //                                                                                      

//                                                                                
// Murata 68k
#if (BAT_NTC_68 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
    {-20,660000},  // at -20C, Rntc, -20C = 660.000 k-ohm
    {-15,500000},
    {-10,390000},
    { -5,290000},
    {  0,225000},
    {  5,170000},
    { 10,135000},
    { 15,103000},
    { 20,84000},
    { 25,67000},
    { 30,54000},
    { 35,43000},
    { 40,34000},
    { 45,28000},
    { 50,23000},
    { 55,18000},
    { 60,15700},
    { 65,11800}
};
#endif
//                                                                                

#if defined(BAT_NTC_TSM_1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
{-20,70603},
{-15,55183},
{-10,43499},
{ -5,34569},
{  0,27680},
{  5,22316},
{ 10,18104},
{ 15,14773},
{ 20,12122},
{ 25,10000},
{ 30,8294},
{ 35,6915},
{ 40,5795},
{ 45,4882},
{ 50,4133},
{ 55,3516},
{ 60,3004}
};
#endif

#if defined(BAT_NTC_10_SEN_1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
 {-20,74354},
 {-15,57626},
 {-10,45068},
 { -5,35548},
 {  0,28267},
 {  5,22650},
 { 10,18280},
 { 15,14855},
 { 20,12151},
 { 25,10000},
 { 30,8279},
 { 35,6892},
 { 40,5768},
 { 45,4852},
 { 50,4101},
 { 55,3483},
 { 60,2970}
};
#endif

#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}
    };
#endif

    //                                                                                      
    table_size = sizeof(Batt_Temperature_Table)/sizeof(BATT_TEMPERATURE);
    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BatteryTemp Table size : %d\n", table_size);
    //                                                                                      

    if(Res>=Batt_Temperature_Table[0].TemperatureR)
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res>=%d\n", Batt_Temperature_Table[0].TemperatureR);
        #endif
        TBatt_Value = Batt_Temperature_Table[0].BatteryTemp;
    }
    //                                                                                      
    #if 1
    else if(Res <= Batt_Temperature_Table[table_size-1].TemperatureR)
    #else
    else if(Res<=Batt_Temperature_Table[16].TemperatureR)
    #endif
    //                                                                                      
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res<=%d\n", Batt_Temperature_Table[16].TemperatureR);
        #endif
        //                                                                                      
        #if 1
        TBatt_Value = Batt_Temperature_Table[table_size-1].BatteryTemp;
        #else
        TBatt_Value = 60;
        #endif
        //                                                                                      
    }
    else
    {
        RES1=Batt_Temperature_Table[0].TemperatureR;
        TMP1=Batt_Temperature_Table[0].BatteryTemp;

        //                                                                                      
        #if 1
        for(i=0;i<table_size;i++)
        #else
        for(i=0;i<=16;i++)
        #endif
        //                                                                                      
        {
            if(Res>=Batt_Temperature_Table[i].TemperatureR)
            {
                RES2=Batt_Temperature_Table[i].TemperatureR;
                TMP2=Batt_Temperature_Table[i].BatteryTemp;
                break;
            }
            else
            {
                RES1=Batt_Temperature_Table[i].TemperatureR;
                TMP1=Batt_Temperature_Table[i].BatteryTemp;
            }
        }

        TBatt_Value = (((Res-RES2)*TMP1)+((RES1-Res)*TMP2))/(RES1-RES2);
    }

    #if 0
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : TBatt_Value = %d\n",TBatt_Value);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : Res = %d\n",Res);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : RES1 = %d\n",RES1);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : RES2 = %d\n",RES2);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : TMP1 = %d\n",TMP1);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : TMP2 = %d\n",TMP2);
    #endif

    return TBatt_Value;
}

//                                                                   
/* convert register to temperature  */
INT16 BattThermistorConverTemp_2nd(INT32 Res)
{
    int i=0;
    INT32 RES1=0,RES2=0;
    INT32 TBatt_Value=-200,TMP1=0,TMP2=0;

//                                                                                      
int table_size = 0;
//                                                                                      
#if 1
#if (BAT_NTC_68 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
    {-30,738998},           // at -20C, Rntc, -20C = 738.998 k-ohm
    {-20,547455},
    {-15,445599},
    {-10,338492},
    { -5,250940},
    {  0,198590},
    {  5,152100},
    { 10,120588},
    { 15,95063},
    { 20,75350},
    { 25,61070},
    { 30,49541},
    { 35,39117},
    { 40,33965},
    { 45,27553},
    { 50,22470},
    { 55,18426},
    { 60,15186},
        { 65,12966}
    };
#endif
#else
//                                                                   
// Murata 68k
#if (BAT_NTC_68 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#endif
//                                                                   
#endif

    //                                                                                      
    table_size = sizeof(Batt_Temperature_Table)/sizeof(BATT_TEMPERATURE);
    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BatteryTemp Table size : %d\n", table_size);
    //                                                                                      

    if(Res>=Batt_Temperature_Table[0].TemperatureR)
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res>=%d\n", Batt_Temperature_Table[0].TemperatureR);
        #endif
        TBatt_Value = Batt_Temperature_Table[0].BatteryTemp;
    }
    //                                                                                      
#if 1
    else if(Res <= Batt_Temperature_Table[table_size-1].TemperatureR)
#else
    else if(Res<=Batt_Temperature_Table[16].TemperatureR)
#endif
    //                                                                                      
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res<=%d\n", Batt_Temperature_Table[16].TemperatureR);
        #endif
        //                                                                                      
    #if 1
        TBatt_Value = Batt_Temperature_Table[table_size-1].BatteryTemp;
    #else
        TBatt_Value = 60;
    #endif
        //                                                                                      
    }
    else
    {
        RES1=Batt_Temperature_Table[0].TemperatureR;
        TMP1=Batt_Temperature_Table[0].BatteryTemp;

        //                                                                                      
    #if 1
        for(i=0;i<table_size;i++)
    #else
        for(i=0;i<=16;i++)
    #endif
        //                                                                                      
        {
            if(Res>=Batt_Temperature_Table[i].TemperatureR)
            {
                RES2=Batt_Temperature_Table[i].TemperatureR;
                TMP2=Batt_Temperature_Table[i].BatteryTemp;
                break;
            }
            else
            {
                RES1=Batt_Temperature_Table[i].TemperatureR;
                TMP1=Batt_Temperature_Table[i].BatteryTemp;
            }
        }

        TBatt_Value = (((Res-RES2)*TMP1)+((RES1-Res)*TMP2))/(RES1-RES2);
    }

    return TBatt_Value;
}
//                                                                   

/*                                                                         */
typedef struct{
    INT32 RFTemp;
    INT32 TemperatureRF;
}RF_TEMPERATURE;

INT16 RF_temperatureConver(INT32 Res)
{
    int i=0;
    INT32 RES1=0,RES2=0;
    INT32 RF_TValue=-400,TMP1=0,TMP2=0;

    RF_TEMPERATURE RF_Temperature_Table[] = {
        {-40,3440},
        {-35,3429},
        {-30,3414},
        {-25,3395},
        {-20,3369},
        {-15,3336},
        {-10,3293},
        { -5,3239},
        {  0,3172},
        {  5,3091},
        { 10,2994},
        { 15,2881},
        { 20,2751},
        { 25,2605},
        { 30,2446},
        { 35,2276},
        { 40,2098},
        { 45,1918},
        { 50,1737},
        { 55,1562},
        { 60,1394},
        { 65,1237},
        { 70,1092},
        { 75, 959},
        { 80, 840},
        { 85, 734},
        { 90, 640},
        { 95, 558},
        {100, 486},
    };

    if(Res>=RF_Temperature_Table[0].TemperatureRF)
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res>=%d\n", RF_Temperature_Table[0].TemperatureRF);
        #endif
        RF_TValue = -40;
    }
    else if(Res<=RF_Temperature_Table[28].TemperatureRF)
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res<=%d\n", RF_Temperature_Table[16].TemperatureRF);
        #endif
        RF_TValue = 100;
    }
    else
    {
        RES1=RF_Temperature_Table[0].TemperatureRF;
        TMP1=RF_Temperature_Table[0].RFTemp;

        for(i=0;i<=28;i++)
        {
            if(Res>=RF_Temperature_Table[i].TemperatureRF)
            {
                RES2=RF_Temperature_Table[i].TemperatureRF;
                TMP2=RF_Temperature_Table[i].RFTemp;
                break;
            }
            else
            {
                RES1=RF_Temperature_Table[i].TemperatureRF;
                TMP1=RF_Temperature_Table[i].RFTemp;
            }
        }

        RF_TValue = (((Res-RES2)*TMP1)+((RES1-Res)*TMP2))/(RES1-RES2);
    }

    return RF_TValue;
}
/*                                                                         */

/* convert ADC_bat_temp_volt to register */
INT16 BattVoltToTemp(UINT32 dwVolt)
{
    INT32 TRes;
    INT32 dwVCriBat = (TBAT_OVER_CRITICAL_LOW*RBAT_PULL_UP_VOLT)/(TBAT_OVER_CRITICAL_LOW+RBAT_PULL_UP_R);
    INT32 sBaTTMP = -100;

    if(dwVolt > dwVCriBat)
    {
        TRes = TBAT_OVER_CRITICAL_LOW;
    }
    else
    {
        TRes = (RBAT_PULL_UP_R*dwVolt)/(RBAT_PULL_UP_VOLT-dwVolt);
    }

    g_BAT_TemperatureR = TRes;

    //                                                                   
    if(batt_id_check)  /*  LG CELL */
    {
        /* convert register to temperature */
        sBaTTMP = BattThermistorConverTemp(TRes);
    }
    else  /* Dualization */
    {
        /* convert register to temperature */
        sBaTTMP = BattThermistorConverTemp(TRes);
    }
    //                                                                   

    #if 0
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : TBAT_OVER_CRITICAL_LOW = %d\n", TBAT_OVER_CRITICAL_LOW);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : RBAT_PULL_UP_VOLT = %d\n", RBAT_PULL_UP_VOLT);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : dwVolt = %d\n", dwVolt);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : TRes = %d\n", TRes);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : sBaTTMP = %d\n", sBaTTMP);
    #endif

    return sBaTTMP;
}

//void BAT_SetUSBState(int usb_state_value)
void BATTERY_SetUSBState(int usb_state_value)
{
    if ( (usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))){
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
        usb_state_value = USB_UNCONFIGURED;
    } else {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAT_SetUSBState Success! Set %d\r\n", usb_state_value);
        g_usb_state = usb_state_value;
    }
}
//EXPORT_SYMBOL(BAT_SetUSBState);
EXPORT_SYMBOL(BATTERY_SetUSBState);

kal_bool pmic_chrdet_status(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )
    {
        return KAL_TRUE;
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[pmic_chrdet_status] No charger\r\n");
        return KAL_FALSE;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
//// Pulse Charging Algorithm
///////////////////////////////////////////////////////////////////////////////////////////
void select_charging_curret(void)
{
    if (g_ftm_battery_flag)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] FTM charging : %d\r\n", charging_level_data[0]);
        g_temp_CC_value = charging_level_data[0];
    }
    else
    {
        if ( BMT_status.charger_type == STANDARD_HOST )
        {

            if (g_Support_USBIF == 1)
            {
                if (g_usb_state == USB_SUSPEND)
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
                }
                else if (g_usb_state == USB_UNCONFIGURED)
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
                }
                else if (g_usb_state == USB_CONFIGURED)
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_CONFIGURED;
                }
                else
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
                }

                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Support BC1.1\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n", g_temp_CC_value, g_usb_state);
                }
            }
            else
            {

#if defined(MTK_JEITA_STANDARD_SUPPORT)
                if(g_temp_status == TEMP_NEG_10_TO_POS_0)
                {
                    g_temp_CC_value = Cust_CC_200MA;   //for low temp
                }
                else
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT;
                }
#else
        //                                                         
/*                                                                                                */
            if(Is_Not_FactoryCable_PowerOn() == 0)  // Factory Cable
/*                                                                                                */
            {
                g_temp_CC_value = Cust_CC_1600MA;  // Factory Test Mode, Max 2.3A
            }
            else
            {
                g_temp_CC_value = USB_CHARGER_CURRENT;  // Normal USB Charging Current
            }
        //                                                         
#endif
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Not Support BC1.1\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] STANDARD_HOST CC mode charging : %d\r\n", g_temp_CC_value);
                }
            }
        }
        else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
        {
#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                g_temp_CC_value = Cust_CC_200MA;   //for low temp
            }
            else
            {
                g_temp_CC_value = USB_CHARGER_CURRENT;
            }
#else
        //                                                         
/*                                                                                                */
            if (Is_Not_FactoryCable_PowerOn() == 0)
/*                                                                                                */
            {
                g_temp_CC_value = Cust_CC_1600MA;  // Factory Test Mode, Max 2.3A
            }
            else
            {
                g_temp_CC_value = USB_CHARGER_CURRENT;
            }
        //                                                         
            #endif

            if (Enable_BATDRV_LOG == 1) {
                   xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] NONSTANDARD_CHARGER CC mode charging : %d\r\n", g_temp_CC_value); // USB HW limitation
            }
        }
        else if (BMT_status.charger_type == STANDARD_CHARGER)
                {

#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                g_temp_CC_value = Cust_CC_200MA;   //for low temp
                }
                else
                {
                g_temp_CC_value = AC_CHARGER_CURRENT;
                }
#else
            g_temp_CC_value = AC_CHARGER_CURRENT;
#endif

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] STANDARD_CHARGER CC mode charging : %d\r\n", g_temp_CC_value);
            }
        }
        else if (BMT_status.charger_type == CHARGING_HOST)
        {

#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                g_temp_CC_value = Cust_CC_200MA;   //for low temp
            }
            else
            {
                g_temp_CC_value = AC_CHARGER_CURRENT;
            }
#else
            //                                                                      
            g_temp_CC_value = USB_CHARGER_CURRENT;
            //                                                                      
#endif
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] CHARGING_HOST CC mode charging : %d\r\n", g_temp_CC_value);
            }
        }
        else
        {
            g_temp_CC_value = USB_CHARGER_CURRENT;  // Default Charging

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Default CC mode charging : %d\r\n", g_temp_CC_value);
            }
        }

    // Charger Current Setting
        #if defined( LGE_FW_OTP )
        if(g_temp_CC_value == Cust_CC_1600MA)  // Factory Cable Charging
            {
            /*If it is factory cable, it will set PTM mode.*/
            #if defined(CONFIG_SINGLE_CHARGER)
                charging_ic_set_factory_mode();
            #elif defined(CONFIG_MINIABB_CHARGER)
                set_charger_factory_mode();
            #endif
            }
        else if(g_temp_CC_value == AC_CHARGER_CURRENT)
        {
            if(otp_state == OTP_NORMAL_CHARGING_STATE)
            {
                #if defined ( LGE_RF_TEMP )
                if (rf_temp_state == RF_LOW_CHARGING_STATE)
                {
                    /*Set USB mode current.*/
                    #if defined(CONFIG_SINGLE_CHARGER)
                        charging_ic_set_usb_mode();
                    #elif defined(CONFIG_MINIABB_CHARGER)
                      set_charger_start_mode(CHG_500);
                    #endif
                }
                else
                #endif
                {
                    /*Set TA charger mode current.*/
                    #if defined(CONFIG_SINGLE_CHARGER)
                        charging_ic_set_ta_mode();
                    #elif defined(CONFIG_MINIABB_CHARGER)
                        set_charger_start_mode(CHG_TA);
                    #endif
                }
            }
            else if(otp_state == OTP_DECREASE_CHARGING_STATE)
            {
                   /*Set USB mode current.*/
            #if defined(CONFIG_SINGLE_CHARGER)
                        charging_ic_set_usb_mode();
            #elif defined(CONFIG_MINIABB_CHARGER)
                        set_charger_start_mode(CHG_USB);
#endif
        }
            }
            else
            {
            /*Set USB mode current.*/
            #if defined(CONFIG_SINGLE_CHARGER)
                charging_ic_set_usb_mode();
            #elif defined(CONFIG_MINIABB_CHARGER)
                set_charger_start_mode(CHG_USB);
            #endif
        }
    #else
        if(g_temp_CC_value == Cust_CC_1600MA)  // Factory Cable Charging
        {
            /*If it is factory cable, it will set PTM mode.*/
            #if defined(CONFIG_SINGLE_CHARGER)
                charging_ic_set_factory_mode();
            #elif defined(CONFIG_MINIABB_CHARGER)
                set_charger_factory_mode();
#endif
            }
        else if(g_temp_CC_value == AC_CHARGER_CURRENT)
        {
                /*Set TA charger mode current.*/
                #if defined(CONFIG_SINGLE_CHARGER)
                    charging_ic_set_ta_mode();
                #elif defined(CONFIG_MINIABB_CHARGER)
                 set_charger_start_mode(CHG_TA);
                #endif
        }
        else
        {
                /*Set USB mode current.*/
            #if defined(CONFIG_SINGLE_CHARGER)
                charging_ic_set_usb_mode();
            #elif defined(CONFIG_MINIABB_CHARGER)
                set_charger_start_mode(CHG_USB);
            #endif
        }
    #endif

    }
}

void ChargerHwInit(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[MT6329 BAT_probe] ChargerHwInit\n" );
    }

    /*If you need any initialization code for External charger IC, you can add on here.*/
}

//                                                                
USB_ID_TYPE readUSB_ID_Value()
{

    int res = 0;
    unsigned int usb_id_adc_value = 0;
    int data[4] = {0,0,0,0};
    USB_ID_TYPE usb_id_type = DEVICE_NONE;

    int deCount =0;
    int validCount = 0;
    unsigned int adc_value_result = 0;
    int chr_usb_voltage = 0;


        for(deCount = 0; deCount < 10; deCount++)
        {
            res = IMM_GetOneChannelValue(AUXADC_USB_ID_CHANNEL, data, &usb_id_adc_value );
            if(res == 0)
    {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BAT_thread] ADC raw data : %d, ADC value  : %d.%d%d\n", usb_id_adc_value, data[0], data[1] / 10, data[1] % 10);
                if(usb_id_adc_value <= FACTORY_CABLE_ADC_THRESHOLD)
        {
                    adc_value_result += usb_id_adc_value;
                    validCount++;
                }
            }
        }

        if(validCount == 0)
        {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT_thread] USB_ID_TYPE : %d\n", DEVICE_OPEN_CABLE);
            return DEVICE_OPEN_CABLE;
        }
        else
        {
        adc_value_result = adc_value_result / validCount;
        data[0] = (adc_value_result * 250 / 4096 / 100);
        data[1] = ((adc_value_result * 250 / 4096) % 100);
        chr_usb_voltage = (data[0] * 100) + ((data[1] / 10) * 10) + (data[1] % 10);
        }

        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT_thread] ADC read valid count : %d, USB_ID AVR Voltage : %d\n", validCount, chr_usb_voltage);

        if( (FACTORY_CABLE_56K_MIN_VOLTAGE <= chr_usb_voltage) && (chr_usb_voltage <= FACTORY_CABLE_56K_MAX_VOLTAGE) )
                {
                    usb_id_type = DEVICE_FACTORY_USB_CABLE; // 56K
                }
        else if( (FACTORY_CABLE_130K_MIN_VOLTAGE <= chr_usb_voltage) && (chr_usb_voltage <= FACTORY_CABLE_130K_MAX_VOLTAGE) )
                {
                    usb_id_type = DEVICE_FACTORY_UART_CABLE; // 130K
                }
        else if( (FACTORY_CABLE_180K_MIN_VOLTAGE <= chr_usb_voltage) && (chr_usb_voltage <= FACTORY_CABLE_180K_MAX_VOLTAGE) )
                {
            usb_id_type = DEVICE_FACTORY_180_CABLE; // 180K
                }
        else if( (FACTORY_CABLE_910K_MIN_VOLTAGE <= chr_usb_voltage) && (chr_usb_voltage <= FACTORY_CABLE_910K_MAX_VOLTAGE) )
                {
            usb_id_type = DEVICE_FACTORY_DOWNLOAD_CABLE; // 910K
                }
        else if( OPEN_CABLE_MIN_VOLTAGE <= chr_usb_voltage )
        {
                usb_id_type = DEVICE_OPEN_CABLE; // Open Type
    }
    else
    {
            usb_id_type = DEVICE_NONE;  // Unknown Type
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT_thread] USB_ID_TYPE : %d\n", usb_id_type);
    return usb_id_type;

}
//                                                                

//                                                           
static kal_bool BMT_EOC_state(void)
{
    kal_bool eoc_state = KAL_FALSE;

    #if defined(CONFIG_SINGLE_CHARGER)
    eoc_state = mt_get_gpio_in(GPIO_EOC_PIN);
    #elif defined(CONFIG_MINIABB_CHARGER)
    eoc_state = 0; //check_EOC_status();
    #endif

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT_thread] EOC pin read : %d\n", eoc_state);

    if(eoc_state == TRUE)
    {
        if(EOC_counts < EOC_CHECK_DELAY_COUNT)
        {
            EOC_counts++;
        }
        else
        {
            return KAL_TRUE;
        }
    }
    else
    {
        EOC_counts = 0;
        return KAL_FALSE;
    }
    return KAL_FALSE;
}
//                                                           

void pchr_turn_off_charging (void)
{
    //                                                                         
    if(Is_Not_FactoryCable_PowerOn() == 0)
    {
        if (Enable_BATDRV_LOG == 1)
        {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] LG Factory Cable, No Charging OFF !!!\r\n");
        }

        return ;
    }
    //                                                                         

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] pchr_turn_off_charging !\r\n");
    }

    g_SW_Charging_On = KAL_FALSE;

    //                                                           
    /*Disable charging.*/
#if defined(CONFIG_SINGLE_CHARGER)
    charging_ic_deactive();
#elif defined(CONFIG_MINIABB_CHARGER)
    set_charger_stop_mode();
#endif
    //                                                           

}

extern kal_int32 gFG_booting_counter_I_FLAG;
int g_fg_ready_flag = 0;

void pchr_turn_on_charging (void)
{
    if ( BMT_status.bat_charging_state == CHR_ERROR )
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Charger Error, turn OFF charging !\r\n");
        }
        pchr_turn_off_charging();
    }
    //                                                         
    #if defined( LGE_FW_OTP )
    else if( (otp_state == OTP_STOP_CHARGING_STATE) || (otp_state == OTP_DECREASE_STOP_CHARGING_STATE))
    {
        pchr_turn_off_charging();
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] OTP in STOP Charging state, stop charging\r\n");
    }
    #endif
    //                                                         
    else
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] pchr_turn_on_charging !\r\n");
        }

        //                                                           
        #if 1
        if(0)
        #else
        if( g_temp_CC_value == Cust_CC_0MA)
        #endif
        {
            pchr_turn_off_charging();
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] charging current is set 0mA !\r\n");
        }
        else
        {
#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_POS_10_TO_POS_45)
            {
                upmu_chr_vbat_cv_vth(0x0); // VBAT_CV_VTH,4.2V
            }
            else if((g_temp_status == TEMP_NEG_10_TO_POS_0)||(g_temp_status == TEMP_BELOW_NEG_10))
            {
                upmu_chr_vbat_cv_vth(0x10); // VBAT_CV_VTH,4V low temp
            }
            else if ((g_temp_status == TEMP_POS_0_TO_POS_10)||(g_temp_status == TEMP_POS_45_TO_POS_60)||(g_temp_status == TEMP_ABOVE_POS_60))
            {
                upmu_chr_vbat_cv_vth(0x18); // VBAT_CV_VTH.4.1v
            }
            else
            {
                upmu_chr_vbat_cv_vth(0x0); // VBAT_CV_VTH,default 4.2v
            }
#else
            //upmu_chr_vbat_cv_vth(0x0);                // VBAT_CV_VTH,default 4.2v
#endif

            //upmu_chr_csdac_enable(1);                // CSDAC_EN
            //upmu_chr_enable(1);                        // CHR_EN

            select_charging_curret();
            g_SW_Charging_On = KAL_TRUE;

            // do charging after FG ready
            if(gFG_booting_counter_I_FLAG == 2)
            {
                //upmu_chr_enable(1);                    // CHR_EN
               /*Select charging current and enable charging here.*/
                if(g_fg_ready_flag == 0)
                {
                    FGADC_Reset_SW_Parameter();
                    g_fg_ready_flag = 1;
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] wait g_fg_ready_flag (%d)\r\n", g_fg_ready_flag);
                }
            }
            else
            {
                //                                                                     
                //pchr_turn_off_charging();
                //                                                                     
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] wait gFG_booting_counter_I_FLAG==2 (%d)\r\n", gFG_booting_counter_I_FLAG);
            }
        //                                                           
        }
    }

    //if (Enable_BATDRV_LOG == 1) {
    //    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] pchr_turn_on_charging : reset HW 30mins timer\r\n");
    //}
    // reset HW 30mins timer
    //upmu_chr_bc11_bb_ctrl(1);        //BC11_BB_CTRL, TODO
    //upmu_chr_bc11_rst(1);            //BC11_RST

    //                                                           
    #if 0
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Reg[0x21]=0x%x, Reg[0x24]=0x%x(VBAT_CV_VTH), Reg[0x25]=0x%x, Reg[0x2E]=0x%x\r\n",
            upmu_get_reg_value(0x21), upmu_get_reg_value(0x24), upmu_get_reg_value(0x25), upmu_get_reg_value(0x2E) );
    }
    #endif
    //                                                           

}

int BAT_CheckPMUStatusReg(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )
    {
        BMT_status.charger_exist = TRUE;
    }
    else
    {
        BMT_status.charger_exist = FALSE;

        BMT_status.total_charging_time = 0;
        BMT_status.PRE_charging_time = 0;
        BMT_status.CC_charging_time = 0;
        BMT_status.TOPOFF_charging_time = 0;
        BMT_status.POSTFULL_charging_time = 0;

        BMT_status.bat_charging_state = CHR_PRE;

        return PMU_STATUS_FAIL;
    }

    return PMU_STATUS_OK;
}

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
    unsigned long ret_val = 0;

    if(polling_mode == 1)
        g_switch_to_i2c_polling_mode=1;

    if(polling_mode == 1)
        ret_val=PMIC_IMM_GetOneChannelValueSleep(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    else
        ret_val=PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);

    if(polling_mode == 1)
        g_switch_to_i2c_polling_mode=0;

    return ret_val;
}

int g_Get_I_Charging(void)
{
    kal_int32 ADC_BAT_SENSE_tmp[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    kal_int32 ADC_BAT_SENSE_sum=0;
    kal_int32 ADC_BAT_SENSE=0;
    kal_int32 ADC_I_SENSE_tmp[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    kal_int32 ADC_I_SENSE_sum=0;
    kal_int32 ADC_I_SENSE=0;
    int repeat=20;
    int i=0;
    int j=0;
    kal_int32 temp=0;
    int ICharging=0;

    for(i=0 ; i<repeat ; i++)
    {
        ADC_BAT_SENSE_tmp[i] = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
        ADC_I_SENSE_tmp[i] = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);

        ADC_BAT_SENSE_sum += ADC_BAT_SENSE_tmp[i];
        ADC_I_SENSE_sum += ADC_I_SENSE_tmp[i];
    }

    //sorting    BAT_SENSE
    for(i=0 ; i<repeat ; i++)
    {
        for(j=i; j<repeat ; j++)
        {
            if( ADC_BAT_SENSE_tmp[j] < ADC_BAT_SENSE_tmp[i] )
            {
                temp = ADC_BAT_SENSE_tmp[j];
                ADC_BAT_SENSE_tmp[j] = ADC_BAT_SENSE_tmp[i];
                ADC_BAT_SENSE_tmp[i] = temp;
            }
        }
    }
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging:BAT_SENSE]\r\n");
        for(i=0 ; i<repeat ; i++ )
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "%d,", ADC_BAT_SENSE_tmp[i]);
        }
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "\r\n");
    }

    //sorting    I_SENSE
    for(i=0 ; i<repeat ; i++)
    {
        for(j=i ; j<repeat ; j++)
        {
            if( ADC_I_SENSE_tmp[j] < ADC_I_SENSE_tmp[i] )
            {
                temp = ADC_I_SENSE_tmp[j];
                ADC_I_SENSE_tmp[j] = ADC_I_SENSE_tmp[i];
                ADC_I_SENSE_tmp[i] = temp;
            }
        }
    }
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging:I_SENSE]\r\n");
        for(i=0 ; i<repeat ; i++ )
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "%d,", ADC_I_SENSE_tmp[i]);
        }
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "\r\n");
    }

    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[0];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[1];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[18];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[19];
    ADC_BAT_SENSE = ADC_BAT_SENSE_sum / (repeat-4);

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging] ADC_BAT_SENSE=%d\r\n", ADC_BAT_SENSE);
    }

    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[0];
    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[1];
    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[18];
    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[19];
    ADC_I_SENSE = ADC_I_SENSE_sum / (repeat-4);

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging] ADC_I_SENSE(Before)=%d\r\n", ADC_I_SENSE);
    }

    ADC_I_SENSE += gADC_I_SENSE_offset;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging] ADC_I_SENSE(After)=%d\r\n", ADC_I_SENSE);
    }

    BMT_status.ADC_BAT_SENSE = ADC_BAT_SENSE;
    BMT_status.ADC_I_SENSE = ADC_I_SENSE;

    if(ADC_I_SENSE > ADC_BAT_SENSE)
    {
        ICharging = (ADC_I_SENSE - ADC_BAT_SENSE)*10/R_CURRENT_SENSE;
    }
    else
    {
        ICharging = 0;
    }

    return ICharging;
}

void BAT_GetVoltage(void)
{
    int bat_temperature_volt=0;

    int fg_r_value=0;
    kal_int32 fg_current_temp=0;
    kal_bool fg_current_state;
    int bat_temperature_volt_temp=0;

    /* Get V_BAT_SENSE */
    if (g_chr_event == 0)
    {
        BMT_status.ADC_BAT_SENSE = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    }
    else
    {
        /* Just charger in/out event, same as I_sense */
        g_chr_event = 0;
        BMT_status.ADC_BAT_SENSE = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);
    }
    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE;

    if (g_eco_version == PMIC6329_E1_CID_CODE)
    {
        g_E1_vbat_sense = BMT_status.ADC_BAT_SENSE;
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Charger_E1] Get g_E1_vbat_sense = %d\r\n", g_E1_vbat_sense);
        }
    }

    /* Get V_I_SENSE */
    //BMT_status.ADC_I_SENSE = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);
    //BMT_status.ADC_I_SENSE += gADC_I_SENSE_offset;

    /* Get V_Charger */
    BMT_status.charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5);
    BMT_status.charger_vol = BMT_status.charger_vol / 100;

    /* Get V_BAT_Temperature */
    bat_temperature_volt = PMIC_IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL,5);
    if(bat_temperature_volt == 0)
    {
        BMT_status.temperature = g_bat_temperature_pre;
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Warning !! bat_temperature_volt == 0, restore temperature value\n\r");
        }
    }
    else
    {
    #if defined(CONFIG_POWER_EXT)
    //by pass
        fg_r_value=0;
        fg_current_temp=0;
        fg_current_state=KAL_FALSE;
        bat_temperature_volt_temp=0;
    #else
    //-----------------------------------------------------------------------------
        if( gForceADCsolution == 1 )
        {
            /*Use no gas gauge*/
        }
        else
        {
            fg_r_value = get_r_fg_value();
            fg_current_temp = fgauge_read_current();
            fg_current_temp = fg_current_temp/10;
            fg_current_state = get_gFG_Is_Charging();
            if(fg_current_state==KAL_TRUE)
            {
                bat_temperature_volt_temp = bat_temperature_volt;
                bat_temperature_volt = bat_temperature_volt - ((fg_current_temp*fg_r_value)/1000);
            }
            else
            {
                bat_temperature_volt_temp = bat_temperature_volt;
                bat_temperature_volt = bat_temperature_volt + ((fg_current_temp*fg_r_value)/1000);
            }
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[tbat_workaround] %d,%d,%d,%d,%d\n",
                bat_temperature_volt_temp, bat_temperature_volt, fg_current_state, fg_current_temp, fg_r_value);
        }
    //-----------------------------------------------------------------------------
    #endif

        BMT_status.temperature = BattVoltToTemp(bat_temperature_volt);
        g_bat_temperature_pre = BMT_status.temperature;
    }
    if( (g_battery_tt_check_flag==0) && (BMT_status.temperature<60) && (BMT_status.temperature>(-20)) )
    {
        g_battery_thermal_throttling_flag=3;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Enable battery TT (%d)\n\r", BMT_status.temperature);
        g_battery_tt_check_flag=1;
    }

    /* Data Calibration  */
    if (g_ADC_Cali) {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Before Cal : %ld(B) , %ld(I) \r\n", BMT_status.ADC_BAT_SENSE, BMT_status.ADC_I_SENSE);
        }

           BMT_status.ADC_I_SENSE = ((BMT_status.ADC_I_SENSE * (*(adc_cali_slop+1)))+(*(adc_cali_offset+1)))/1000;
        BMT_status.ADC_BAT_SENSE = ((BMT_status.ADC_BAT_SENSE * (*(adc_cali_slop+0)))+(*(adc_cali_offset+0)))/1000;

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "After Cal : %ld(B) , %ld(I) \r\n", BMT_status.ADC_BAT_SENSE, BMT_status.ADC_I_SENSE);
        }
    }

    /* Calculate the charging current */
    //if(BMT_status.ADC_I_SENSE > BMT_status.ADC_BAT_SENSE)
    //    BMT_status.ICharging = (BMT_status.ADC_I_SENSE - BMT_status.ADC_BAT_SENSE)*10/R_CURRENT_SENSE;
    //else
    //    BMT_status.ICharging = 0;
    BMT_status.ICharging = g_Get_I_Charging();

    if (Enable_BATDRV_LOG >= 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:ADC] VCHR:%d BAT_SENSE:%ld I_SENSE:%ld Current:%ld CAL:%d BatTT:%d\n", BMT_status.charger_vol,
            BMT_status.ADC_BAT_SENSE, BMT_status.ADC_I_SENSE, BMT_status.ICharging, g_ADC_Cali, g_battery_thermal_throttling_flag );
    }

    g_InstatVolt = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    g_BatteryAverageCurrent = BMT_status.ICharging;
    g_BAT_BatterySenseVoltage = BMT_status.ADC_BAT_SENSE;
    g_BAT_ISenseVoltage = BMT_status.ADC_I_SENSE;
    g_BAT_ChargerVoltage = BMT_status.charger_vol;

}

/*                                                                                                             */
bool hw_check_battery()
{
    kal_uint32 reg_data = 0;

    if (g_eco_version == PMIC6329_E1_CID_CODE)
    {
        return TRUE;
    }

    upmu_chr_baton_tdet_en(0x1);                /* CHR_CON18_OFFSET(0x38), bit 5 : BATON_TDET_EN */
    upmu_auxadc_buf_pwd_b(0x1);                 /* AUXADC_CON15_OFFSET(0xE9, bit 1 : RG_BUF_PWD_B */

    reg_data = upmu_chr_get_baton_undet();      /* BANK0_CHR_CON7(0x28), bit 7, BATON_UNDET */

    if (Enable_BATDRV_LOG == 1)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:CHECK] BATON_UNDET : %d\r\n", reg_data);
    }

    if (reg_data)
    {
        reg_data = upmu_get_reg_value(BANK0_CHR_CON18);
        reg_data = upmu_get_reg_value(BANK0_AUXADC_CON15);

        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:CHECK] No Battery !!! \r\n");

        reg_data = upmu_get_reg_value(BANK0_CHR_CON18);
        reg_data = upmu_get_reg_value(BANK0_AUXADC_CON15);

        return FALSE;
    }
    else
    {
        reg_data = upmu_get_reg_value(BANK0_CHR_CON18);
        reg_data = upmu_get_reg_value(BANK0_AUXADC_CON15);

        if (Enable_BATDRV_LOG == 1)
        {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:CHECK] Battery exist !!! \r\n");
        }

        reg_data = upmu_get_reg_value(BANK0_CHR_CON18);
        reg_data = upmu_get_reg_value(BANK0_AUXADC_CON15);

        upmu_chr_baton_tdet_en(0x1);                /* CHR_CON18_OFFSET(0x38), bit 5 : BATON_TDET_EN */
        upmu_chr_low_ich_db(0x1);                   /* CHR_CON18_OFFSET(0x37), RG_LOW_ICH_DB = 00001'b */

        return TRUE;
    }

    return TRUE;
}

/*                                                                                                             */

UINT32 BattVoltToPercent(UINT16 dwVoltage)
{
    UINT32 m=0;
    UINT32 VBAT1=0,VBAT2=0;
    UINT32 bPercntResult=0,bPercnt1=0,bPercnt2=0;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "###### 100 <-> voltage : %d ######\r\n", Batt_VoltToPercent_Table[10].BattVolt);
    }

    if(dwVoltage<=Batt_VoltToPercent_Table[0].BattVolt)
    {
        bPercntResult = Batt_VoltToPercent_Table[0].BattPercent;
        return bPercntResult;
    }
    else if (dwVoltage>=Batt_VoltToPercent_Table[10].BattVolt)
    {
        bPercntResult = Batt_VoltToPercent_Table[10].BattPercent;
        return bPercntResult;
    }
    else
    {
        VBAT1 = Batt_VoltToPercent_Table[0].BattVolt;
        bPercnt1 = Batt_VoltToPercent_Table[0].BattPercent;
        for(m=1;m<=10;m++)
        {
            if(dwVoltage<=Batt_VoltToPercent_Table[m].BattVolt)
            {
                VBAT2 = Batt_VoltToPercent_Table[m].BattVolt;
                bPercnt2 = Batt_VoltToPercent_Table[m].BattPercent;
                break;
            }
            else
            {
                VBAT1 = Batt_VoltToPercent_Table[m].BattVolt;
                bPercnt1 = Batt_VoltToPercent_Table[m].BattPercent;
            }
        }
    }

    bPercntResult = ( ((dwVoltage-VBAT1)*bPercnt2)+((VBAT2-dwVoltage)*bPercnt1) ) / (VBAT2-VBAT1);

    return bPercntResult;

}

#if defined(MTK_JEITA_STANDARD_SUPPORT)
int do_jeita_state_machine(void)
{
    //JEITA battery temp Standard
    if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD)
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Over high Temperature(%d) !!\n\r", TEMP_POS_60_THRESHOLD);
        g_temp_status = TEMP_ABOVE_POS_60;
        return PMU_STATUS_FAIL;
    }
    else if(BMT_status.temperature > TEMP_POS_45_THRESHOLD)
    {
        if((g_temp_status == TEMP_ABOVE_POS_60) && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE))
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
                TEMP_POS_60_THRES_MINUS_X_DEGREE,TEMP_POS_60_THRESHOLD);
            return PMU_STATUS_FAIL;
        }
        else
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_POS_45_THRESHOLD,TEMP_POS_60_THRESHOLD);
            g_temp_status = TEMP_POS_45_TO_POS_60;
            g_jeita_recharging_voltage = 4000;
            V_CC2TOPOFF_THRES = 4050;
        }
    }
    else if(BMT_status.temperature >= TEMP_POS_10_THRESHOLD)
    {
        if( ((g_temp_status == TEMP_POS_45_TO_POS_60) && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE)) ||
            ((g_temp_status == TEMP_POS_0_TO_POS_10 ) && (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE ))    )
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
        }
        else
        {
            if(Enable_BATDRV_LOG >=1)
            {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
                    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
            }
            g_temp_status = TEMP_POS_10_TO_POS_45;
            g_jeita_recharging_voltage = 4100;
            V_CC2TOPOFF_THRES = 4050;
        }
    }
    else if(BMT_status.temperature >= TEMP_POS_0_THRESHOLD)
    {
        if((g_temp_status == TEMP_NEG_10_TO_POS_0) && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE))
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_POS_0_THRES_PLUS_X_DEGREE, TEMP_POS_10_THRESHOLD);
        }else{
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_POS_0_THRESHOLD,TEMP_POS_10_THRESHOLD);
            g_temp_status = TEMP_POS_0_TO_POS_10;
            g_jeita_recharging_voltage = 4000;
            V_CC2TOPOFF_THRES = 4050;
        }
    }
    else if(BMT_status.temperature >= TEMP_NEG_10_THRESHOLD)
    {
        if((g_temp_status == TEMP_BELOW_NEG_10) && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE))
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
                TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);
            return PMU_STATUS_FAIL;
        }else{
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_NEG_10_THRESHOLD,TEMP_POS_0_THRESHOLD);
            g_temp_status = TEMP_NEG_10_TO_POS_0;
            g_jeita_recharging_voltage = 3900;
            V_CC2TOPOFF_THRES = 3950;
        }
    }
    else
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery below low Temperature(%d) !!\n\r", TEMP_NEG_10_THRESHOLD);
        g_temp_status = TEMP_BELOW_NEG_10;
        return PMU_STATUS_FAIL;
    }

    return 0;
}
#endif

int BAT_CheckBatteryStatus(void)
{
    int BAT_status = PMU_STATUS_OK;
    int i = 0;
    //int bat_temperature_volt=0;

    /* Get Battery Information */
    BAT_GetVoltage();

/*                                                                                                     */
    if (hw_check_battery() == FALSE && Is_Not_FactoryCable_PowerOn() != 0)
    {
        if (hw_check_battery() == FALSE)
        {
            mt_power_off();
        }
    }
/*                                                                                                     */

    /*                                                                                                       */
    if ( BMT_status.bat_exist == FALSE && Is_Not_FactoryCable_PowerOn() != 0)
    {
        BMT_status.bat_exist = hw_check_battery();
    }
    /*                                                                                                       */

    //                                                           
    /*In case of External charger IC, it do not use pluse charging operation.*/
#if (0)
    /*Charging 9s and discharging 1s : start*/
    if( (upmu_is_chr_det() == KAL_TRUE) &&
        //(BMT_status.bat_full == KAL_FALSE) &&
        (g_HW_Charging_Done == 0) &&
        (BMT_status.bat_charging_state != CHR_ERROR) &&
        (BMT_status.bat_charging_state != CHR_TOP_OFF))
    {
        g_HW_stop_charging = 1;

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Dis Charging 1s\n\r");
        }
        pchr_turn_off_charging();
        getVoltFlag = 1;
        msleep(1000);
    }

    BMT_status.ADC_BAT_SENSE = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    //BMT_status.ADC_BAT_SENSE += V_compensate_EVB;
    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE;

    if ( BMT_status.bat_charging_state != CHR_TOP_OFF)
    {
        gADC_BAT_SENSE_temp = (int)BMT_status.bat_vol;
        gADC_I_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);

        //workaround
        gADC_BAT_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
        gADC_I_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);
        gADC_BAT_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
        gADC_I_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);
        gADC_BAT_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
        gADC_I_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);
        gADC_BAT_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
        gADC_I_SENSE_temp = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,1);

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] gADC_BAT_SENSE_temp=%d, gADC_I_SENSE_temp=%d\n\r", gADC_BAT_SENSE_temp, gADC_I_SENSE_temp);
        }
        gADC_I_SENSE_offset = gADC_BAT_SENSE_temp - gADC_I_SENSE_temp;
    }

    g_TempBattVoltage = BMT_status.bat_vol;
    if ( getVoltFlag == 1 )
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Charging 9s\n\r");
        }
        getVoltFlag = 0;
    }
    /*Charging 9s and discharging 1s : end*/
#endif
//                                                           

    /*Use no gas gauge*/
    if( gForceADCsolution == 1 )
    {
        /* Re-calculate Battery Percentage (SOC) */
        BMT_status.SOC = BattVoltToPercent(BMT_status.bat_vol);

        /* User smooth View when discharging : start */
        if( upmu_is_chr_det() == KAL_FALSE )
        {

#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if (BMT_status.bat_vol >= g_jeita_recharging_voltage){
#else
            if (BMT_status.bat_vol >= RECHARGING_VOLTAGE) {
#endif
                BMT_status.SOC = 100;
                BMT_status.bat_full = KAL_TRUE;
            }
        }
        if (bat_volt_cp_flag == 0)
        {
            bat_volt_cp_flag = 1;
            bat_volt_check_point = BMT_status.SOC;
        }
        /* User smooth View when discharging : end */

        /**************** Averaging : START ****************/
        if (!batteryBufferFirst)
        {
            if(BMT_status.bat_vol != 0)
            {
                batteryBufferFirst = KAL_TRUE;

                for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                    batteryVoltageBuffer[i] = BMT_status.bat_vol;
                    batteryCurrentBuffer[i] = BMT_status.ICharging;
                    batterySOCBuffer[i] = BMT_status.SOC;
                }

                batteryVoltageSum = BMT_status.bat_vol * BATTERY_AVERAGE_SIZE;
                batteryCurrentSum = BMT_status.ICharging * BATTERY_AVERAGE_SIZE;
                batterySOCSum = BMT_status.SOC * BATTERY_AVERAGE_SIZE;
            }
            else
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] BMT_status.bat_vol == 0, init avg array again.\n\r");
                }
            }
        }

        batteryVoltageSum -= batteryVoltageBuffer[batteryIndex];
        batteryVoltageSum += BMT_status.bat_vol;
        batteryVoltageBuffer[batteryIndex] = BMT_status.bat_vol;

        batteryCurrentSum -= batteryCurrentBuffer[batteryIndex];
        batteryCurrentSum += BMT_status.ICharging;
        batteryCurrentBuffer[batteryIndex] = BMT_status.ICharging;

        if (BMT_status.bat_full)
            BMT_status.SOC = 100;
        if (g_bat_full_user_view)
            BMT_status.SOC = 100;

        batterySOCSum -= batterySOCBuffer[batteryIndex];
        batterySOCSum += BMT_status.SOC;
        batterySOCBuffer[batteryIndex] = BMT_status.SOC;

        BMT_status.bat_vol = batteryVoltageSum / BATTERY_AVERAGE_SIZE;
        BMT_status.ICharging = batteryCurrentSum / BATTERY_AVERAGE_SIZE;
        BMT_status.SOC = batterySOCSum / BATTERY_AVERAGE_SIZE;

        batteryIndex++;
        if (batteryIndex >= BATTERY_AVERAGE_SIZE)
            batteryIndex = 0;
        /**************** Averaging : END ****************/

        if( BMT_status.SOC == 100 ) {
            BMT_status.bat_full = KAL_TRUE;
        }
    }
    /*Use gas gauge*/
    else
    {
        /* Re-calculate Battery Percentage (SOC) */
        BMT_status.SOC = FGADC_Get_BatteryCapacity_CoulombMothod();
        //BMT_status.bat_vol = FGADC_Get_FG_Voltage();

        /* Sync FG's percentage */
        if(gSyncPercentage==0)
        {
            if( (upmu_is_chr_det()==KAL_TRUE) && (!g_Battery_Fail) && (g_Charging_Over_Time==0))
            {
                /* SOC only UP when charging */
                //                                                                              
#if 1  // to lower SOC level when not charging or low charging
                if ( BMT_status.SOC < bat_volt_check_point &&
                        (BMT_status.charger_type == STANDARD_HOST || otp_state != OTP_NORMAL_CHARGING_STATE ) ) {
                    bat_volt_check_point = BMT_status.SOC;
                }
                else if ( BMT_status.SOC > bat_volt_check_point ) {
                    bat_volt_check_point = BMT_status.SOC;
                }
#else
                if ( BMT_status.SOC > bat_volt_check_point ) {
                    bat_volt_check_point = BMT_status.SOC;
                }
#endif
                //                                                                              
            }
            else
            {
                /* SOC only Done when dis-charging */
                if ( BMT_status.SOC < bat_volt_check_point ) {
                    bat_volt_check_point = BMT_status.SOC;
                }
            }
        }

        /**************** Averaging : START ****************/
        if (!batteryBufferFirst)
        {
            batteryBufferFirst = KAL_TRUE;

            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                batteryVoltageBuffer[i] = BMT_status.bat_vol;
                batteryCurrentBuffer[i] = BMT_status.ICharging;
                batteryTempBuffer[i] = BMT_status.temperature;
            }

            batteryVoltageSum = BMT_status.bat_vol * BATTERY_AVERAGE_SIZE;
            batteryCurrentSum = BMT_status.ICharging * BATTERY_AVERAGE_SIZE;
            batteryTempSum = BMT_status.temperature * BATTERY_AVERAGE_SIZE;
        }

        if( (batteryCurrentSum==0) && (BMT_status.ICharging!=0) )
        {
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                batteryCurrentBuffer[i] = BMT_status.ICharging;
            }
            batteryCurrentSum = BMT_status.ICharging * BATTERY_AVERAGE_SIZE;
        }

        batteryVoltageSum -= batteryVoltageBuffer[batteryIndex];
        batteryVoltageSum += BMT_status.bat_vol;
        batteryVoltageBuffer[batteryIndex] = BMT_status.bat_vol;

        batteryCurrentSum -= batteryCurrentBuffer[batteryIndex];
        batteryCurrentSum += BMT_status.ICharging;
        batteryCurrentBuffer[batteryIndex] = BMT_status.ICharging;

        batteryTempSum -= batteryTempBuffer[batteryIndex];
        batteryTempSum += BMT_status.temperature;
        batteryTempBuffer[batteryIndex] = BMT_status.temperature;

        //if (g_bat_full_user_view)
        //    BMT_status.SOC = 100;

        BMT_status.bat_vol = batteryVoltageSum / BATTERY_AVERAGE_SIZE;
        BMT_status.ICharging = batteryCurrentSum / BATTERY_AVERAGE_SIZE;
        BMT_status.temperature = batteryTempSum / BATTERY_AVERAGE_SIZE;

        batteryIndex++;
        if (batteryIndex >= BATTERY_AVERAGE_SIZE)
            batteryIndex = 0;
        /**************** Averaging : END ****************/
    }

    //if (Enable_BATDRV_LOG >= 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:AVG] BatTemp:%d Vbat:%ld VBatSen:%ld SOC:%ld ChrDet:%d Vchrin:%d Icharging:%ld ChrType:%d USBstate:%d gADC_I_SENSE_offset:%d\r\n",
           BMT_status.temperature ,BMT_status.bat_vol, BMT_status.ADC_BAT_SENSE, BMT_status.SOC,
           upmu_is_chr_det(), BMT_status.charger_vol, BMT_status.ICharging, CHR_Type_num, g_usb_state, gADC_I_SENSE_offset );
    //}

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:FG] %d,%ld,%ld,%ld,%d,%d,%ld,%d,%d,%d,%d\r\n",
           BMT_status.temperature ,BMT_status.bat_vol, BMT_status.ADC_BAT_SENSE, BMT_status.SOC,
           upmu_is_chr_det(), BMT_status.charger_vol, BMT_status.ICharging, CHR_Type_num,
           FGADC_Get_BatteryCapacity_CoulombMothod(), FGADC_Get_BatteryCapacity_VoltageMothod(), BATTERY_AVERAGE_SIZE );

        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] Bank0[0x31]=0x%x, Bank0[0x38]=0x%x\r\n",
        upmu_get_reg_value(0x31), upmu_get_reg_value(0x38));
    }

    /* Protection Check : start*/
    BAT_status = BAT_CheckPMUStatusReg();
    if(BAT_status != PMU_STATUS_OK)
        return PMU_STATUS_FAIL;

    if(battery_cmd_thermal_test_mode == 1){
        BMT_status.temperature = battery_cmd_thermal_test_mode_value;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] In thermal_test_mode 1, Tbat=%d\n", BMT_status.temperature);
    }

#if defined(MTK_JEITA_STANDARD_SUPPORT)

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] support JEITA, Tbat=%d\n", BMT_status.temperature);
    }

    if( do_jeita_state_machine() == PMU_STATUS_FAIL)
    {
        return PMU_STATUS_FAIL;
    }

#else

    #if (BAT_TEMP_PROTECT_ENABLE == 1)
    if ((BMT_status.temperature <= MIN_CHARGE_TEMPERATURE) ||
        (BMT_status.temperature == ERR_CHARGE_TEMPERATURE))
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
        BMT_status.bat_charging_state = CHR_ERROR;
        return PMU_STATUS_FAIL;
    }
    #endif

    //                                                         
    #if !defined( LGE_FW_OTP )
    if (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE)
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Over Temperature !!\n\r");
        BMT_status.bat_charging_state = CHR_ERROR;
        return PMU_STATUS_FAIL;
    }
    #endif
    //                                                         

#endif

    //                                                         
    if( upmu_is_chr_det() == KAL_TRUE)
    {
        #if (V_CHARGER_ENABLE == 1)
        if (BMT_status.charger_vol <= V_CHARGER_MIN )
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY]Charger under voltage!!\r\n");
            BMT_status.bat_charging_state = CHR_ERROR;
            return PMU_STATUS_FAIL;
        }
        #endif
        if ( BMT_status.charger_vol >= V_CHARGER_MAX )
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY]Charger over voltage !!\r\n");
            BMT_status.charger_protect_status = charger_OVER_VOL;
            BMT_status.bat_charging_state = CHR_ERROR;
            return PMU_STATUS_FAIL;
        }

        #if defined( LGE_FW_OTP)
        #if defined (LGE_RF_TEMP )
        rf_temperature_ADC();
        rf_temperature_check(rf_temperature);
        #endif
        otp_state_check(BMT_status.temperature, BMT_status.bat_vol);
        #endif
    }
    #if defined( LGE_FW_OTP)
    else
    {
        otp_state = OTP_NORMAL_CHARGING_STATE;
        #if defined ( LGE_RF_TEMP )
        rf_temp_state = RF_NORMAL_CHARGING_STATE;
        #endif
    }
    #endif

    /* Protection Check : end*/

    if( upmu_is_chr_det() == KAL_TRUE)
    {

#if defined(MTK_JEITA_STANDARD_SUPPORT)
        if((BMT_status.bat_vol < g_jeita_recharging_voltage) && (BMT_status.bat_full) && (g_HW_Charging_Done == 1) && (!g_Battery_Fail) )
#else
    #if defined( LGE_FW_OTP)
        if((otp_state == OTP_STOP_CHARGING_STATE) || (otp_state == OTP_DECREASE_STOP_CHARGING_STATE))
        {
            return PMU_STATUS_FAIL;
        }
        else
        {
            if((BMT_status.bat_vol < RECHARGING_VOLTAGE) && (BMT_status.bat_full) && (g_HW_Charging_Done == 1) && (!g_Battery_Fail) )
    #else
        if((BMT_status.bat_vol < RECHARGING_VOLTAGE) && (BMT_status.bat_full) && (g_HW_Charging_Done == 1) && (!g_Battery_Fail) )
    #endif
#endif
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Re-charging !!\n\r");
            }
            BMT_status.bat_full = KAL_FALSE;
            g_bat_full_user_view = KAL_TRUE;
            BMT_status.bat_charging_state = CHR_CC;

            g_HW_Charging_Done = 0;
            g_Calibration_FG = 0;

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Re-charging. Call FGADC_Reset_SW_Parameter.\n\r");
            }
            FGADC_Reset_SW_Parameter();
        }
        #if defined(LGE_FW_OTP)
    }
        #endif
    }
    //                                                         

    return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryStatusFailAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAD Battery status... Charging Stop !!\n\r");
    }

#if defined(MTK_JEITA_STANDARD_SUPPORT)
    if((g_temp_status == TEMP_ABOVE_POS_60) ||(g_temp_status == TEMP_BELOW_NEG_10))
    {
        temp_error_recovery_chr_flag=KAL_FALSE;
    }
    if((temp_error_recovery_chr_flag==KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60) && (g_temp_status != TEMP_BELOW_NEG_10))
    {
        temp_error_recovery_chr_flag=KAL_TRUE;
        BMT_status.bat_charging_state=CHR_PRE;
    }
#endif

    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;
    post_charging_time=0;

    /*  Disable charger */
    pchr_turn_off_charging();

    //g_sw_cv_enable=0;

    return PMU_STATUS_OK;
}

PMU_STATUS BAT_ChargingOTAction(void)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Charging over %d hr stop !!\n\r", MAX_CHARGING_TIME);

    //BMT_status.bat_full = KAL_TRUE;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;

    g_HW_Charging_Done = 1;
    g_Charging_Over_Time = 1;

    /*  Disable charger*/
    pchr_turn_off_charging();
    //                                                           
    EOC_status = 0;
    EOC_counts = 0;
    //                                                           

    //g_sw_cv_enable=0;

    return PMU_STATUS_OK;
}

extern void fg_qmax_update_for_aging(void);

PMU_STATUS BAT_BatteryFullAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery full !!\n\r");
    }

    BMT_status.bat_full = KAL_TRUE;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;

    g_HW_Charging_Done = 1;
    fg_qmax_update_for_aging();
    g_Calibration_FG = 1;
    if(gFG_can_reset_flag == 1)
    {
        gFG_can_reset_flag = 0;

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full. Call FGADC_Reset_SW_Parameter.\n");
        }
        FGADC_Reset_SW_Parameter();
    }
    g_Calibration_FG = 0;

    /*  Disable charger */
    pchr_turn_off_charging();
    //                                                           
            EOC_status = 0;
            EOC_counts = 0;
    //                                                           

    //g_sw_cv_enable=0;

    gSyncPercentage=1;

    return PMU_STATUS_OK;
}


PMU_STATUS BAT_PreChargeModeAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Pre-CC mode charge, timer=%ld on %ld !!\n\r",
        BMT_status.PRE_charging_time, BMT_status.total_charging_time);
    }

    BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.total_charging_time += BAT_TASK_PERIOD;
    post_charging_time=0;

    //                                                           
    EOC_status = 0;
    EOC_counts = 0;
    //                                                           
    /*  Enable charger */
    pchr_turn_on_charging();

    if ( BMT_status.bat_vol > V_PRE2CC_THRES )
    {
        BMT_status.bat_charging_state = CHR_CC;
    }

    //g_sw_cv_enable=0;

    if (g_eco_version == PMIC6329_E1_CID_CODE)
    {
    upmu_chr_hw_cv_en(0); // RG_HWCV_EN
    }

    return PMU_STATUS_OK;
}


PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
    int i=0;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] CC mode charge, timer=%ld on %ld !!\n\r",
        BMT_status.CC_charging_time, BMT_status.total_charging_time);
    }

    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time += BAT_TASK_PERIOD;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.total_charging_time += BAT_TASK_PERIOD;
    post_charging_time=0;

    /*  Enable charger */
    pchr_turn_on_charging();

    if (g_eco_version == PMIC6329_E1_CID_CODE)
    {
        V_CC2TOPOFF_THRES = 4150;
        BMT_status.bat_vol = g_E1_vbat_sense;
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Charger_E1] Update g_E1_vbat_sense=%d, V_CC2TOPOFF_THRES=%d\r\n", g_E1_vbat_sense, V_CC2TOPOFF_THRES);
        }
    }

    if ( BMT_status.bat_vol > V_CC2TOPOFF_THRES )
    {
        BMT_status.bat_charging_state = CHR_TOP_OFF;

        #if 0
        gSW_CV_prepare_flag=1;

        SW_CV_Algo_prepare();
        #endif

        if(g_temp_CC_value == AC_CHARGER_CURRENT)
        {
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                batteryCurrentBuffer[i] = 650;
            }
            batteryCurrentSum = 650 * BATTERY_AVERAGE_SIZE;
        }
        else
        {
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                batteryCurrentBuffer[i] = 450;
            }
            batteryCurrentSum = 450 * BATTERY_AVERAGE_SIZE;
        }
    }

    //g_sw_cv_enable=0;

    if (g_eco_version == PMIC6329_E1_CID_CODE)
    {
    upmu_chr_hw_cv_en(0); // RG_HWCV_EN
    }

    return PMU_STATUS_OK;
}


PMU_STATUS BAT_TopOffModeAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Top Off mode charge, timer=%ld on %ld !!\n\r",
        BMT_status.TOPOFF_charging_time, BMT_status.total_charging_time);
    }

    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time += BAT_TASK_PERIOD;
    BMT_status.total_charging_time += BAT_TASK_PERIOD;

    /*  Enable charger */
    pchr_turn_on_charging();

    //g_sw_cv_enable=1;

    upmu_chr_hw_cv_en(1); // RG_HWCV_EN

    return PMU_STATUS_OK;
}

int POSTFULL_safety_timer=0;

PMU_STATUS BAT_PostFullModeAction(void)
{
    //g_sw_cv_enable=0;

    return PMU_STATUS_OK;
}

void mt_battery_notify_check(void)
{
    g_BatteryNotifyCode = 0x0000;

    if(g_BN_TestMode == 0x0000)
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] mt_battery_notify_check\n");
        }

#if defined(BATTERY_NOTIFY_CASE_0000)
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0000\n");
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0001)
        if(BMT_status.charger_vol > V_CHARGER_MAX)
        //if(BMT_status.charger_vol > 3000) //test
        {
            g_BatteryNotifyCode |= 0x0001;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BMT_status.charger_vol(%ld) > %d mV\n",
                BMT_status.charger_vol, V_CHARGER_MAX);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0001);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0001 (%x)\n",
                g_BatteryNotifyCode);
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0002)
        if(BMT_status.temperature >= MAX_CHARGE_TEMPERATURE)
        //if(BMT_status.temperature > 20) //test
        {
            g_BatteryNotifyCode |= 0x0002;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] bat_temp(%d) > 50'C\n", BMT_status.temperature);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0002);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0002 (%x)\n",
                g_BatteryNotifyCode);
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0003)
        //if(BMT_status.ICharging > 1000)
        if( (BMT_status.ICharging > 1000) &&
            (BMT_status.total_charging_time > 300)
            )
        //if(BMT_status.ICharging > 200) //test
        {
            g_BatteryNotifyCode |= 0x0004;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] I_charging(%ld) > 1000mA\n", BMT_status.ICharging);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0004);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0003 (%x)\n",
                g_BatteryNotifyCode);
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0004)
        if(BMT_status.bat_vol > 4350)
        //if(BMT_status.bat_vol > 3800) //test
        {
            g_BatteryNotifyCode |= 0x0008;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] bat_vlot(%ld) > 4350mV\n", BMT_status.bat_vol);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0008);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0004 (%x)\n",
                g_BatteryNotifyCode);
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0005)
        //if(g_battery_thermal_throttling_flag==2)
        if( (g_battery_thermal_throttling_flag==2) || (g_battery_thermal_throttling_flag==3) )
        {
            printk("[TestMode] Disable Safty Timer : no UI display\n");
        }
        else
        {
        if(BMT_status.total_charging_time >= MAX_CHARGING_TIME)
        //if(BMT_status.total_charging_time >= 60) //test
        {
            g_BatteryNotifyCode |= 0x0010;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Charging Over Time\n");
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0010);
        }
        }

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0005 (%x)\n",
                g_BatteryNotifyCode);
        }
#endif

    }
    else if(g_BN_TestMode == 0x0001)
    {
        g_BatteryNotifyCode = 0x0001;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0001\n");
    }
    else if(g_BN_TestMode == 0x0002)
    {
        g_BatteryNotifyCode = 0x0002;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0002\n");
    }
    else if(g_BN_TestMode == 0x0003)
    {
        g_BatteryNotifyCode = 0x0004;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0003\n");
    }
    else if(g_BN_TestMode == 0x0004)
    {
        g_BatteryNotifyCode = 0x0008;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0004\n");
    }
    else if(g_BN_TestMode == 0x0005)
    {
        g_BatteryNotifyCode = 0x0010;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0005\n");
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Unknown BN_TestMode Code : %x\n", g_BN_TestMode);
    }
}

int get_pmic_flag=0;
/*                                                                  */
#if defined (LGE_BSP_LGBM)
int touch_ta_mode_falg = 0;
#endif
/*                                                                  */

void BAT_thread(void)
{
    int i=0;
    int BAT_status = 0;
    kal_uint32 tmp32;

    g_switch_to_i2c_polling_mode = 0;
		upmu_chr_usbdl_set(0);
    // init HW
    //upmu_chr_vcdt_hv_vth(0xB);    //VCDT_HV_VTH, 7V
        upmu_chr_vcdt_hv_vth(0xA);    //VCDT_HV_VTH, 6.5V

    if (Enable_BATDRV_LOG == 1) {
#if defined(MTK_JEITA_STANDARD_SUPPORT)
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TOP] LOG. %d,%d,%d,%d,%d,%d----------------------------\n",
            BATTERY_AVERAGE_SIZE, g_jeita_recharging_voltage, RECHARGING_VOLTAGE, g_switch_to_i2c_polling_mode, gFG_15_vlot, mtk_jeita_support_flag);
#else
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TOP] LOG. %d,%d,%d,%d,%d,%d----------------------------\n",
            BATTERY_AVERAGE_SIZE, CHARGING_FULL_CURRENT, RECHARGING_VOLTAGE, g_switch_to_i2c_polling_mode, gFG_15_vlot, mtk_jeita_support_flag);
#endif
    }

    if(get_pmic_flag == 0)
    {
        /* get pmic version */
        /* Low part of CID */
        tmp32=upmu_get_cid0();
        g_eco_version |= tmp32;
        /* High part of CID */
        tmp32=upmu_get_cid1();
        g_eco_version |= (tmp32 << 8);
        if (g_eco_version == PMIC6329_E1_CID_CODE)
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Charger_E1] Get PMIC version = E1\n");
            upmu_chr_vcdt_lv_vth(0); // VCDT_LV=4.2V
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Charger_E1] Set VCDT_LV=4.2V\n");
        }
        else
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] Get PMIC version > E1\n");
        }

        get_pmic_flag = 1;
    }

    //if(g_battery_thermal_throttling_flag==1)
    if( (g_battery_thermal_throttling_flag==1) || (g_battery_thermal_throttling_flag==3) )
    {
        if(battery_cmd_thermal_test_mode == 1){
            BMT_status.temperature = battery_cmd_thermal_test_mode_value;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] In thermal_test_mode 2, Tbat=%d\n", BMT_status.temperature);
        }

#if defined(MTK_JEITA_STANDARD_SUPPORT)
        //ignore default rule
#else
    //                                                         
    #if !defined(LGE_FW_OTP)
        if(BMT_status.temperature >= 60)
        {
            #if defined(CONFIG_POWER_EXT)
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] CONFIG_POWER_EXT, no update mt6329_battery_update_power_down.\n");
            #else
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Tbat(%d)>=60, system need power down.\n", BMT_status.temperature);
            mt6329_battery_update_power_down(&mt6329_battery_main);
            if( upmu_is_chr_det() == KAL_TRUE )
            {
                // can not power down due to charger exist, so need reset system
                arch_reset(0,NULL);
            }
            //avoid SW no feedback
            mt_power_off();
            #endif
        }
    #endif
    //                                                         
#endif

    }

    /* If charger exist, then get the charger type */
    if( upmu_is_chr_det() == KAL_TRUE )
    {
        wake_lock(&battery_suspend_lock);

        if(BMT_status.charger_type == CHARGER_UNKNOWN)
        //if((BMT_status.charger_type == CHARGER_UNKNOWN) && mt_usb_is_device())
        {
               CHR_Type_num = mt_charger_type_detection();
               //CHR_Type_num = STANDARD_HOST;
               xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BAT_thread] CHR_Type_num=%d\r\n", CHR_Type_num);
            BMT_status.charger_type = CHR_Type_num;

            //                                                                
            Is_Not_FactoryCable_PowerOn();
            //                                                                

            if( (CHR_Type_num==STANDARD_HOST) || (CHR_Type_num==CHARGING_HOST) )
            {
                mt_usb_connect();
            }

            /*                                                                  */
#if defined (LGE_BSP_LGBM)
            if ( touch_ta_mode_falg == KAL_FALSE )
            {
                printk ( "[TSP] TA in!\n" );
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)

#else					
                ist30xx_set_ta_mode ( KAL_TRUE );
#endif				
                touch_ta_mode_falg = KAL_TRUE;
            }
#endif 
            /*                                                                  */
        }
    }
    else
    {
        post_charging_time=0;

        wake_unlock(&battery_suspend_lock);

        BMT_status.charger_type = CHARGER_UNKNOWN;
        BMT_status.bat_full = KAL_FALSE;

        /*Initialize USB ID type.*/
        //                                                                
        usb_id_type_num = DEVICE_NONE;
        //                                                                

        /*Use no gas gauge*/
        if( gForceADCsolution == 1 )
        {
            g_bat_full_user_view = KAL_FALSE;
        }
        /*Use gas gauge*/
        else
        {
        if(bat_volt_check_point != 100) {
            g_bat_full_user_view = KAL_FALSE;
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_Only] Set g_bat_full_user_view=KAL_FALSE\r\n");
            }
        }
        }

        g_usb_state = USB_UNCONFIGURED;

        g_HW_Charging_Done = 0;
        g_Charging_Over_Time = 0;
        g_Calibration_FG = 0;

//                                                           
        EOC_status = 0;
        EOC_counts = 0;
//                                                           

        mt_usb_disconnect();

        /*                                                                  */
#if defined (LGE_BSP_LGBM)
        if ( touch_ta_mode_falg == KAL_TRUE )
        {
            printk ( "[TSP] TA out!\n" );
#if defined(CONFIG_MAX8971_CHARGER)
#elif (CONFIG_MINIABB_CHARGER)
#else				
            ist30xx_set_ta_mode ( KAL_FALSE );
#endif			
            touch_ta_mode_falg = KAL_FALSE;
        }
#endif 
        /*                                                                  */

        //gSW_CV_prepare_flag=0;

        for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
           batteryCurrentBuffer[i] = 0;
           }
           batteryCurrentSum = 0;

    }

    /* Check Battery Status */
    BAT_status = BAT_CheckBatteryStatus();
    if( BAT_status == PMU_STATUS_FAIL )
        g_Battery_Fail = KAL_TRUE;
    else
        g_Battery_Fail = KAL_FALSE;

    if( BMT_status.bat_charging_state == CHR_ERROR )
        g_Battery_Fail = KAL_TRUE;
    else
        g_Battery_Fail = KAL_FALSE;

    if(battery_cmd_thermal_test_mode == 1){
        BMT_status.temperature = battery_cmd_thermal_test_mode_value;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] In thermal_test_mode 3, Tbat=%d\n", BMT_status.temperature);
    }

    /* Battery Notify Check */
    mt_battery_notify_check();

    #if defined(CONFIG_POWER_EXT)
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] CONFIG_POWER_EXT, no update Android.\n");
    #else
    if(gFG_booting_counter_I_FLAG == 2)
    {
//                                                                                     
#if 1
    /* AC/USB/Battery information update for Android */
    mt6329_battery_update(&mt6329_battery_main);
    mt6329_ac_update(&mt6329_ac_main);
    mt6329_usb_update(&mt6329_usb_main);
#else
    /* AC/USB/Battery information update for Android */
    mt6329_ac_update(&mt6329_ac_main);
    mt6329_usb_update(&mt6329_usb_main);
    mt6329_battery_update(&mt6329_battery_main);
#endif
//                                                                                     

    }
    else if(gFG_booting_counter_I_FLAG == 1)
    {
        /*Use no gas gauge*/
        if( gForceADCsolution == 1 )
        {
            //do nothing
        }
        else
        {
            //                                                                                                              
            #if 1
            if(offChargingBatteryFull == '1')
            {
                mt6329_battery_main.BAT_CAPACITY = 100;
            }
            else
            {
            mt6329_battery_main.BAT_CAPACITY = FGADC_Get_BatteryCapacity_CoulombMothod();
            }

            if(mt6329_battery_main.BAT_CAPACITY <= 1)
            {
                mt6329_battery_main.BAT_CAPACITY = 2;
            }
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG is 1, soc=%d\n", mt6329_battery_main.BAT_CAPACITY);
            #else
            //mt6329_battery_main.BAT_CAPACITY = fgauge_read_capacity_by_v();
            //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG is 1, soc=%d\n", fgauge_read_capacity_by_v());
            mt6329_battery_main.BAT_CAPACITY = 50;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG is 1, soc=50.\n");
            #endif
            //                                                                                                              
    }
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG!=2 (%d)\r\n", gFG_booting_counter_I_FLAG);
    }
    #endif

    /* No Charger */
    if(BAT_status == PMU_STATUS_FAIL || g_Battery_Fail)
    {
        gFG_can_reset_flag = 1;

        BAT_BatteryStatusFailAction();
    }

    /* Battery Full */
    //else if (BMT_status.bat_full)
       /* HW charging done, real stop charging */
    else if (g_HW_Charging_Done == 1)
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full. \n");
        }
        BAT_BatteryFullAction();
    }

    /* Charging Overtime, can not charging */
    else if (g_Charging_Over_Time == 1)
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Charging Over Time. \n");
        }
        pchr_turn_off_charging();

        if(gFG_can_reset_flag == 1)
        {
            gFG_can_reset_flag = 0;
        }
    }

    /* Battery Not Full and Charger exist : Do Charging */
    else
    {
        gFG_can_reset_flag = 1;

//                                                           
        EOC_status = 0;//BMT_EOC_state();
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT_thread] EOC status : %d, EOC count: %d\n", EOC_status, EOC_counts);
//                                                           

        //if(g_battery_thermal_throttling_flag==2)
        if( (g_battery_thermal_throttling_flag==2) || (g_battery_thermal_throttling_flag==3) )
        {
            printk("[TestMode] Disable Safty Timer. bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
            g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
        }
        else
        {
            /* Charging OT */
            if(BMT_status.total_charging_time >= MAX_CHARGING_TIME)
            {
            //                                                       
            #if 1
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.total_charging_time >= %d \r\n", MAX_CHARGING_TIME);
            BMT_status.bat_charging_state = CHR_BATFULL;
            BAT_BatteryFullAction();
            #else
                BAT_ChargingOTAction();
            #endif
            //                                                       
                return;
            }
        }

        if ((BMT_status.TOPOFF_charging_time >= MAX_CV_CHARGING_TIME) && (BMT_status.bat_vol >=CHARGER_THRESH_HOLD))
//        if ( BMT_status.TOPOFF_charging_time >= MAX_CV_CHARGING_TIME && ((g_battery_thermal_throttling_flag!=2) && (g_battery_thermal_throttling_flag!=3) ))
        {
            //if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.TOPOFF_charging_time >= %d \r\n", MAX_CV_CHARGING_TIME);
            //}
            BMT_status.bat_charging_state = CHR_BATFULL;
            BAT_BatteryFullAction();
            return;
        }


        if(0)
        {
            if ( (BMT_status.bat_charging_state == CHR_TOP_OFF) &&
                 (BMT_status.SOC == 100) &&
                 (BMT_status.bat_vol >= Batt_VoltToPercent_Table[10].BattVolt) )
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full(%ld,%d) and disable charging !\n",
                            BMT_status.SOC, Batt_VoltToPercent_Table[10].BattVolt);
                }
                BMT_status.bat_charging_state = CHR_BATFULL;
                BAT_BatteryFullAction();
                return;
            }
        }
        else
        {
#if defined(DISABLE_POST_CHARGE)
//                                                           
            #if 1
            if( (BMT_status.bat_charging_state == CHR_TOP_OFF) && (BMT_status.bat_exist == 1) && (BMT_status.bat_vol >= CHARGER_THRESH_HOLD)
                    && (EOC_status == KAL_TRUE) )
            #else
            /* charging full condition when charging current < CHARGING_FULL_CURRENT mA on CHR_TOP_OFF mode*/
            if ( (BMT_status.bat_charging_state == CHR_TOP_OFF )
                 && (BMT_status.TOPOFF_charging_time > 60)
                 && (BMT_status.ICharging <= CHARGING_FULL_CURRENT)
                 )
            #endif
//                                                           
            {
                BMT_status.bat_charging_state = CHR_BATFULL;
                BAT_BatteryFullAction();
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full and disable charging on %ld mA \n", BMT_status.ICharging);
                return;
            }
#else
            #if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                CHARGING_FULL_CURRENT=120;
            }
            else
            {
                CHARGING_FULL_CURRENT=220;
            }
            #endif

            if (post_charging_time >= POST_CHARGING_TIME  && ((g_battery_thermal_throttling_flag!=2) && (g_battery_thermal_throttling_flag!=3) ) )
            {
                BMT_status.bat_charging_state = CHR_BATFULL;
                BAT_BatteryFullAction();
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full and disable charging on %ld mA \n", BMT_status.ICharging);

                post_charging_time=0;
                return;
            }
            else if (post_charging_time > 0)
            {
                post_charging_time+=BAT_TASK_PERIOD;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] post_charging_time=%d,POST_CHARGING_TIME=%d\n", post_charging_time, POST_CHARGING_TIME);
            }
            else if ((BMT_status.bat_charging_state == CHR_TOP_OFF)
                && (BMT_status.TOPOFF_charging_time > 60)
                && (BMT_status.ICharging <= CHARGING_FULL_CURRENT))
            {
                post_charging_time=BAT_TASK_PERIOD;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Enter Post charge\n");

                BMT_status.bat_full=KAL_TRUE;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] UI show full first\n");
            }
            else
            {
                post_charging_time=0;
            }
#endif
        }

        /* Charging flow begin */
        switch(BMT_status.bat_charging_state)
        {
            case CHR_PRE :
                BAT_PreChargeModeAction();
                break;

            case CHR_CC :
                BAT_ConstantCurrentModeAction();
                break;

            case CHR_TOP_OFF :
                BAT_TopOffModeAction();
                break;

            case CHR_POST_FULL :
                BAT_PostFullModeAction();
                break;

            case CHR_BATFULL:
                BAT_BatteryFullAction();
                break;

            case CHR_ERROR:
                BAT_BatteryStatusFailAction();
                break;
        }
    }

    g_SW_CHR_OUT_EN = 1;
    g_HW_stop_charging = 0;

}

///////////////////////////////////////////////////////////////////////////////////////////
//// Internal API
///////////////////////////////////////////////////////////////////////////////////////////
int g_FG_init = 0;
//static int bat_thread_kthread(void *x)
int bat_thread_kthread(void *x)
{
    /* Run on a process content */
    while (1) {

        if(g_battery_flag_resume==0)
        {
        mutex_lock(&bat_mutex);
        #if defined(CONFIG_POWER_EXT)
        BAT_thread();
        #else
        if(g_FG_init == 0)
        {
            g_FG_init=1;
            fgauge_initialization();
            FGADC_thread_kthread();
        }
        else
        {
            // if plug-in/out USB, bypass once
            if(g_chr_event==0)
            {
                FGADC_thread_kthread();
            }

            BAT_thread();
        }
        #endif
        mutex_unlock(&bat_mutex);
        }
        else
        {
            g_battery_flag_resume=0;
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[bat_thread_kthread] g_battery_flag_resume=%d\r\n", g_battery_flag_resume);
        }

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6329 battery : bat_thread_kthread : 1 ********\n" );
        }

        wait_event(bat_thread_wq, bat_thread_timeout);

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6329 battery : bat_thread_kthread : 2 ********\n" );
        }
        bat_thread_timeout=0;

        if( g_wake_up_bat==1 )
        {
            g_wake_up_bat=0;
            g_Calibration_FG = 0;
            FGADC_Reset_SW_Parameter();
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Call FGADC_Reset_SW_Parameter.\r\n");
            }
        }

    }

    return 0;
}
#endif //                                              

UINT32 bat_thread_timeout_sum=0;

void bat_thread_wakeup(UINT16 i)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6329 battery : bat_thread_wakeup : 1 ********\n" );
    }
    bat_thread_timeout = 1;
    wake_up(&bat_thread_wq);
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6329 battery : bat_thread_wakeup : 2 ********\n" );
    }
}

void BatThread_XGPTConfig(void)
{
#if defined ( LGE_BSP_LGBM )
    static int isFirstCalled = 1;

    GPT_CONFIG config;
    GPT_NUM  gpt_num = GPT5;
    GPT_CLK_SRC clkSrc = GPT_CLK_SRC_RTC;
    //GPT_CLK_DIV clkDiv = GPT_CLK_DIV_128;
    GPT_CLK_DIV clkDiv = GPT_CLK_DIV_64;

    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6329 battery : BatThread_XGPTConfig !********\n" );

    GPT_Init (gpt_num, bat_thread_wakeup);
    config.num = gpt_num;
    config.clkSrc = clkSrc;
    config.clkDiv = clkDiv;

    if( isFirstCalled == 1 )
    {
        isFirstCalled = 0;
        config.mode = GPT_ONE_SHOT;
        config.u4CompareL = 9*512; // 10s : 512*64=32768
    }
    else
    {
        config.mode = GPT_REPEAT;
        config.u4CompareL = 10*512; // 10s : 512*64=32768
    }

    config.u4CompareH = 0;
    config.bIrqEnable = TRUE;

    if (GPT_Config(config) == FALSE )
        return;

    GPT_Start(gpt_num);

    return ;
#else
    GPT_CONFIG config;
    GPT_NUM  gpt_num = GPT5;
    GPT_CLK_SRC clkSrc = GPT_CLK_SRC_RTC;
    //GPT_CLK_DIV clkDiv = GPT_CLK_DIV_128;
    GPT_CLK_DIV clkDiv = GPT_CLK_DIV_64;

    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6329 battery : BatThread_XGPTConfig !********\n" );

    GPT_Init (gpt_num, bat_thread_wakeup);
    config.num = gpt_num;
    config.mode = GPT_REPEAT;
    config.clkSrc = clkSrc;
    config.clkDiv = clkDiv;
    //config.u4Timeout = 10*128;
    config.u4CompareL = 10*512; // 10s : 512*64=32768
    config.u4CompareH = 0;
    config.bIrqEnable = TRUE;

    if (GPT_Config(config) == FALSE )
        return;

    GPT_Start(gpt_num);

    return ;
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
//// fop API
///////////////////////////////////////////////////////////////////////////////////////////
static long adc_cali_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int *user_data_addr;
    int *naram_data_addr;
    int i = 0;
    int ret = 0;

    mutex_lock(&bat_mutex);

    switch(cmd)
    {
        case TEST_ADC_CALI_PRINT :
            g_ADC_Cali = KAL_FALSE;
            break;

        case SET_ADC_CALI_Slop:
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_slop, naram_data_addr, 36);
            g_ADC_Cali = KAL_FALSE; /* enable calibration after setting ADC_CALI_Cal */
            /* Protection */
            for (i=0;i<14;i++)
            {
                if ( (*(adc_cali_slop+i) == 0) || (*(adc_cali_slop+i) == 1) ) {
                    *(adc_cali_slop+i) = 1000;
                }
            }
            for (i=0;i<14;i++) xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "adc_cali_slop[%d] = %d\n",i , *(adc_cali_slop+i));
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SET_ADC_CALI_Slop Done!\n");
            break;

        case SET_ADC_CALI_Offset:
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_offset, naram_data_addr, 36);
            g_ADC_Cali = KAL_FALSE; /* enable calibration after setting ADC_CALI_Cal */
            for (i=0;i<14;i++) xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "adc_cali_offset[%d] = %d\n",i , *(adc_cali_offset+i));
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SET_ADC_CALI_Offset Done!\n");
            break;

        case SET_ADC_CALI_Cal :
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_cal, naram_data_addr, 4);
            g_ADC_Cali = KAL_TRUE;
            if ( adc_cali_cal[0] == 1 ) {
                g_ADC_Cali = KAL_TRUE;
            } else {
                g_ADC_Cali = KAL_FALSE;
            }
            for (i=0;i<1;i++) xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "adc_cali_cal[%d] = %d\n",i , *(adc_cali_cal+i));
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SET_ADC_CALI_Cal Done!\n");
            break;

        case ADC_CHANNEL_READ:
            //g_ADC_Cali = KAL_FALSE; /* 20100508 Infinity */
            user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8); /* 2*int = 2*4 */

            if( adc_in_data[0] == 0 ) // I_SENSE
            {
                adc_out_data[0] = PMIC_IMM_GetOneChannelValue(AUXADC_REF_CURRENT_CHANNEL,adc_in_data[1]) * adc_in_data[1];
            }
            else if( adc_in_data[0] == 1 ) // BAT_SENSE
            {
                adc_out_data[0] = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,adc_in_data[1]) * adc_in_data[1];
            }
            else if( adc_in_data[0] == 3 ) // V_Charger
            {
                adc_out_data[0] = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,adc_in_data[1]) * adc_in_data[1];
                adc_out_data[0] = adc_out_data[0] / 100;
            }
            else if( adc_in_data[0] == 30 ) // V_Bat_temp magic number
            {
                //adc_out_data[0] = PMIC_IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL,adc_in_data[1]) * adc_in_data[1];
                adc_out_data[0] = BMT_status.temperature;
            }
            else if( adc_in_data[0] == 66 )
            {
                adc_out_data[0] = (gFG_current)/10;

                if (gFG_Is_Charging == KAL_TRUE)
                {
                    adc_out_data[0] = 0 - adc_out_data[0]; //charging
                }
            }
            else
            {
                adc_out_data[0] = PMIC_IMM_GetOneChannelValue(adc_in_data[0],adc_in_data[1]) * adc_in_data[1];
            }

            if (adc_out_data[0]<0)
                adc_out_data[1]=1; /* failed */
            else
                adc_out_data[1]=0; /* success */

            if( adc_in_data[0] == 30 )
                adc_out_data[1]=0; /* success */

            if( adc_in_data[0] == 66 )
                adc_out_data[1]=0; /* success */

            ret = copy_to_user(user_data_addr, adc_out_data, 8);
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : Channel %d * %d times = %d\n", adc_in_data[0], adc_in_data[1], adc_out_data[0]);
            break;

        case BAT_STATUS_READ:
            user_data_addr = (int *)arg;
            ret = copy_from_user(battery_in_data, user_data_addr, 4);
            /* [0] is_CAL */
            if (g_ADC_Cali) {
                battery_out_data[0] = 1;
            } else {
                battery_out_data[0] = 0;
            }
            ret = copy_to_user(user_data_addr, battery_out_data, 4);
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : CAL:%d\n", battery_out_data[0]);
            break;

        case Set_Charger_Current: /* For Factory Mode*/
            user_data_addr = (int *)arg;
            ret = copy_from_user(charging_level_data, user_data_addr, 4);
            g_ftm_battery_flag = KAL_TRUE;
            if( charging_level_data[0] == 0 ) {                charging_level_data[0] = Cust_CC_70MA;
            } else if ( charging_level_data[0] == 1  ) {    charging_level_data[0] = Cust_CC_200MA;
            } else if ( charging_level_data[0] == 2  ) {    charging_level_data[0] = Cust_CC_400MA;
            } else if ( charging_level_data[0] == 3  ) {    charging_level_data[0] = Cust_CC_450MA;
            } else if ( charging_level_data[0] == 4  ) {    charging_level_data[0] = Cust_CC_550MA;
            } else if ( charging_level_data[0] == 5  ) {    charging_level_data[0] = Cust_CC_650MA;
            } else if ( charging_level_data[0] == 6  ) {    charging_level_data[0] = Cust_CC_700MA;
            } else if ( charging_level_data[0] == 7  ) {    charging_level_data[0] = Cust_CC_800MA;
            } else if ( charging_level_data[0] == 8  ) {    charging_level_data[0] = Cust_CC_900MA;
            } else if ( charging_level_data[0] == 9  ) {    charging_level_data[0] = Cust_CC_1000MA;
            } else if ( charging_level_data[0] == 10 ) {    charging_level_data[0] = Cust_CC_1100MA;
            } else if ( charging_level_data[0] == 11 ) {    charging_level_data[0] = Cust_CC_1200MA;
            } else if ( charging_level_data[0] == 12 ) {    charging_level_data[0] = Cust_CC_1300MA;
            } else if ( charging_level_data[0] == 13 ) {    charging_level_data[0] = Cust_CC_1400MA;
            } else if ( charging_level_data[0] == 14 ) {    charging_level_data[0] = Cust_CC_1500MA;
            } else if ( charging_level_data[0] == 15 ) {    charging_level_data[0] = Cust_CC_1600MA;
            } else {
                charging_level_data[0] = Cust_CC_450MA;
            }
            wake_up_bat();
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : set_Charger_Current:%d\n", charging_level_data[0]);
            break;

		//add for meta tool-------------------------------
		case Get_META_BAT_VOL:
			user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8);
			adc_out_data[0] = BMT_status.bat_vol;
			ret = copy_to_user(user_data_addr, adc_out_data, 8); 
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : BAT_VOL:%d\n", adc_out_data[0]);   
			break;
		case Get_META_BAT_SOC:
			user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8);
			adc_out_data[0] = bat_volt_check_point;
			ret = copy_to_user(user_data_addr, adc_out_data, 8); 
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SOC:%d\n", adc_out_data[0]);   
			break;
		//add for meta tool-------------------------------
          
        default:
            g_ADC_Cali = KAL_FALSE;
            break;
    }

    mutex_unlock(&bat_mutex);

    return 0;
}

static int adc_cali_open(struct inode *inode, struct file *file)
{
   return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations adc_cali_fops = {
    .owner        = THIS_MODULE,
    .unlocked_ioctl    = adc_cali_ioctl,
    .open        = adc_cali_open,
    .release    = adc_cali_release,
};

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Charger_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Charger_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] show_ADC_Charger_Voltage : %d\n", BMT_status.charger_vol);
    return sprintf(buf, "%u\n", BMT_status.charger_vol);
}
static ssize_t store_ADC_Charger_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Charger_Voltage, 0664, show_ADC_Charger_Voltage, store_ADC_Charger_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_0_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_0_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+0));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_0_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_0_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_0_Slope, 0664, show_ADC_Channel_0_Slope, store_ADC_Channel_0_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_1_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_1_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+1));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_1_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_1_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_1_Slope, 0664, show_ADC_Channel_1_Slope, store_ADC_Channel_1_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_2_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_2_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+2));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_2_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_2_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_2_Slope, 0664, show_ADC_Channel_2_Slope, store_ADC_Channel_2_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_3_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_3_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+3));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_3_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_3_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_3_Slope, 0664, show_ADC_Channel_3_Slope, store_ADC_Channel_3_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_4_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_4_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+4));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_4_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_4_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_4_Slope, 0664, show_ADC_Channel_4_Slope, store_ADC_Channel_4_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_5_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_5_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+5));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_5_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_5_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_5_Slope, 0664, show_ADC_Channel_5_Slope, store_ADC_Channel_5_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_6_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_6_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+6));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_6_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_6_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_6_Slope, 0664, show_ADC_Channel_6_Slope, store_ADC_Channel_6_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_7_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_7_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+7));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_7_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_7_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_7_Slope, 0664, show_ADC_Channel_7_Slope, store_ADC_Channel_7_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_8_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_8_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+8));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_8_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_8_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_8_Slope, 0664, show_ADC_Channel_8_Slope, store_ADC_Channel_8_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_9_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_9_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+9));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_9_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_9_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_9_Slope, 0664, show_ADC_Channel_9_Slope, store_ADC_Channel_9_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_10_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_10_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+10));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_10_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_10_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_10_Slope, 0664, show_ADC_Channel_10_Slope, store_ADC_Channel_10_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_11_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_11_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+11));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_11_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_11_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_11_Slope, 0664, show_ADC_Channel_11_Slope, store_ADC_Channel_11_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_12_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_12_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+12));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_12_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_12_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_12_Slope, 0664, show_ADC_Channel_12_Slope, store_ADC_Channel_12_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_13_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_13_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+13));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_13_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_13_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_13_Slope, 0664, show_ADC_Channel_13_Slope, store_ADC_Channel_13_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_0_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_0_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+0));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_0_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_0_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_0_Offset, 0664, show_ADC_Channel_0_Offset, store_ADC_Channel_0_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_1_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_1_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+1));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_1_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_1_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_1_Offset, 0664, show_ADC_Channel_1_Offset, store_ADC_Channel_1_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_2_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_2_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+2));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_2_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_2_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_2_Offset, 0664, show_ADC_Channel_2_Offset, store_ADC_Channel_2_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_3_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_3_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+3));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_3_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_3_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_3_Offset, 0664, show_ADC_Channel_3_Offset, store_ADC_Channel_3_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_4_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_4_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+4));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_4_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_4_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_4_Offset, 0664, show_ADC_Channel_4_Offset, store_ADC_Channel_4_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_5_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_5_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+5));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_5_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_5_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_5_Offset, 0664, show_ADC_Channel_5_Offset, store_ADC_Channel_5_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_6_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_6_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+6));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_6_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_6_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_6_Offset, 0664, show_ADC_Channel_6_Offset, store_ADC_Channel_6_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_7_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_7_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+7));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_7_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_7_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_7_Offset, 0664, show_ADC_Channel_7_Offset, store_ADC_Channel_7_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_8_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_8_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+8));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_8_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_8_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_8_Offset, 0664, show_ADC_Channel_8_Offset, store_ADC_Channel_8_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_9_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_9_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+9));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_9_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_9_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_9_Offset, 0664, show_ADC_Channel_9_Offset, store_ADC_Channel_9_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_10_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_10_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+10));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_10_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_10_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_10_Offset, 0664, show_ADC_Channel_10_Offset, store_ADC_Channel_10_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_11_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_11_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+11));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_11_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_11_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_11_Offset, 0664, show_ADC_Channel_11_Offset, store_ADC_Channel_11_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_12_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_12_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+12));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_12_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_12_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_12_Offset, 0664, show_ADC_Channel_12_Offset, store_ADC_Channel_12_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_13_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_13_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+13));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_13_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_13_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_13_Offset, 0664, show_ADC_Channel_13_Offset, store_ADC_Channel_13_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_Is_Calibration
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_Is_Calibration(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=2;
    ret_value = g_ADC_Cali;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_Is_Calibration : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_Is_Calibration(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(ADC_Channel_Is_Calibration, 0664, show_ADC_Channel_Is_Calibration, store_ADC_Channel_Is_Calibration);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Power_On_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Power_On_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = Batt_VoltToPercent_Table[0].BattVolt;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Power_On_Voltage : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Power_On_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(Power_On_Voltage, 0664, show_Power_On_Voltage, store_Power_On_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Power_Off_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Power_Off_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = Batt_VoltToPercent_Table[0].BattVolt;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Power_Off_Voltage : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Power_Off_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(Power_Off_Voltage, 0664, show_Power_Off_Voltage, store_Power_Off_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Charger_TopOff_Value
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Charger_TopOff_Value(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = Batt_VoltToPercent_Table[10].BattVolt;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Charger_TopOff_Value : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Charger_TopOff_Value(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(Charger_TopOff_Value, 0664, show_Charger_TopOff_Value, store_Charger_TopOff_Value);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : FG_Battery_CurrentConsumption
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_FG_Battery_CurrentConsumption(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=8888;
    ret_value = gFG_current;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] FG_Battery_CurrentConsumption : %d/10 mA\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_FG_Battery_CurrentConsumption(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(FG_Battery_CurrentConsumption, 0664, show_FG_Battery_CurrentConsumption, store_FG_Battery_CurrentConsumption);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : FG_SW_CoulombCounter
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_FG_SW_CoulombCounter(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_int32 ret_value=7777;
    ret_value = gFG_columb;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] FG_SW_CoulombCounter : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_FG_SW_CoulombCounter(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(FG_SW_CoulombCounter, 0664, show_FG_SW_CoulombCounter, store_FG_SW_CoulombCounter);

#if defined ( LGE_BSP_LGBM ) //                                              
#else
//                                                           
///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For ATCMD : Charger Control
///////////////////////////////////////////////////////////////////////////////////////////

static ssize_t store_Charging_Mode_Enable(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    if(buf != NULL && size != 0)
    {
        if(buf[0] == '1')
        {
            printk("[Battery] Charging Mode Enable\n");
            at_charging_mode = KAL_TRUE;  // to not affect by temperature
            pchr_turn_on_charging();
        }
        else
        {
            // no action
        }
    }
    return size;
}

static ssize_t show_Is_Charging_Mode(struct device *dev,struct device_attribute *attr, char *buf)
{
    int is_Charging_Mode;

    #if defined(CONFIG_SINGLE_CHARGER)
    is_Charging_Mode = is_charging_ic_enable();
    #elif defined(CONFIG_MINIABB_CHARGER)
    is_Charging_Mode = is_charging_enable();
    #endif

    printk("[Charger] Get Charging Mode : %d\n", is_Charging_Mode);

    return sprintf(buf, "%d\n", is_Charging_Mode);
}

static DEVICE_ATTR(Charger_Mode, 0664, show_Is_Charging_Mode, store_Charging_Mode_Enable);



///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For ATCMD : Charging Complete
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Charging_Complete(struct device *dev,struct device_attribute *attr, char *buf)
{
    //                                                            
    UINT32 bat_voltage = 0;
    UINT32 bat_level = 0;

    bat_voltage = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    bat_level = FGADC_Get_BatteryCapacity_CoulombMothod();
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[ATCMD:CHCOMP] bat_voltage : %d, bat_level : %d\n", bat_voltage, bat_level);

    if(bat_voltage >= CHARGER_THRESH_HOLD && bat_level >= 95)  // Full Charging State
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[ATCMD:CHCOMP] Charging Complete\n");
        return sprintf(buf, "1\n");
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[ATCMD:CHCOMP] Charging\n");
        return sprintf(buf, "0\n");
    }
    //                                                            
}
static DEVICE_ATTR(Charger_Complete, 0664, show_Charging_Complete, NULL);

//                                                            
///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For ATCMD : Battery Level
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Battery_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    UINT32 bat_voltage = 0;

    bat_voltage = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[ATCMD:BATL] bat_voltage : %d\n", bat_voltage);

    return sprintf(buf, "%d\n", bat_voltage);
}
static DEVICE_ATTR(Battery_Voltage, 0664, show_Battery_Voltage, NULL);
//                                                            
#endif //                                              

#if defined ( LGE_BSP_LGBM ) //                                              
static ssize_t show_usb_cable(struct device *dev,struct device_attribute *attr, char *buf)
{
    LGBM_LOG("SHOW_USB_CABLE : g_lgbmBootUsbCableId = %d\n", g_lgbmBootUsbCableId);
    return sprintf(buf, "%d\n", g_lgbmBootUsbCableId);
}
static DEVICE_ATTR(usb_cable, S_IRUGO, show_usb_cable, NULL);
#else
///////////////////////////////////////////////////////////////////////////////////////////
//// Cable detect : usb_cable
///////////////////////////////////////////////////////////////////////////////////////////
static unsigned char cable_type;
static ssize_t show_usb_cable(struct device *dev,struct device_attribute *attr, char *buf)
{
/*                                                                                            */
#if 1
    Is_Not_FactoryCable_PowerOn();
    cable_type = usb_id_type_num;
#else
    cable_type = readUSB_ID_Value();
#endif
/*                                                                                            */
    return sprintf(buf, "%d\n", cable_type);
}

static DEVICE_ATTR(usb_cable, S_IRUGO, show_usb_cable, NULL);
//                                                           
#endif //                                              

//                                                          
static ssize_t show_rf_temperature_ADC(struct device *dev,struct device_attribute *attr, char *buf)
{
    static int res = 0;
    static unsigned int usb_id_adc_value = 0;
    static int data[4] = {0,0,0,0};

    #if 1  // DWS define of RF Temperature ADC change for META tool
    res = IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL, data, &usb_id_adc_value );
    #else
    res = IMM_GetOneChannelValue(AUXADC_RF_TEMP_ADC_CHANNEL, data, &usb_id_adc_value );
    #endif

    return sprintf(buf, "%d\n", usb_id_adc_value);
}

static DEVICE_ATTR(rf_temperature_ADC, S_IRUGO, show_rf_temperature_ADC, NULL);
//                                                          

#if defined ( LGE_BSP_LGBM ) //                                              
#else
/*                                                                         */
static ssize_t show_rf_temperature(struct device *dev,struct device_attribute *attr, char *buf)
{
    static int res = 0;
    static unsigned int usb_id_adc_value = 0;
    static unsigned int rf_adc_temp = 0;
    static int data[4] = {0,0,0,0};

    // RF temperature conversion
    res = IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL, data, &usb_id_adc_value );
    rf_adc_temp = RF_temperatureConver(usb_id_adc_value);

    return sprintf(buf, "%d\n", rf_adc_temp);
}

static DEVICE_ATTR(rf_temperature, S_IRUGO, show_rf_temperature, NULL);
/*                                                                         */
#endif //                                              

//                                                                             
static ssize_t show_pseudo_batt(struct device *dev,struct device_attribute *attr, char *buf)
{
#if defined ( LGE_BSP_LGBM ) //                                              
    LGBM_LOG("Show Fake Battery Mode = %d\n", fake_batt_mode);
#endif
    return sprintf(buf, "%d\n", fake_batt_mode);
}

static ssize_t store_pseudo_batt(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    if(buf != NULL && size != 0)
    {
        if(buf[0] == '1')
        {
            fake_batt_mode = 1 ;
        }
        else
        {
            fake_batt_mode = 0 ;
        }
    }
#if defined ( LGE_BSP_LGBM ) //                                              
    LGBM_LOG("Store Fake Battery Mode = %d\n", fake_batt_mode);
#endif
     return size;
}

static DEVICE_ATTR(pseudo_batt, 0664, show_pseudo_batt, store_pseudo_batt);
//                                                                             


#if defined ( LGE_BSP_LGBM ) //                                              
//                                                                              
extern unsigned int get_chip_id(void);
static ssize_t show_LGBM_AtCmdDeviceID(struct device *dev,struct device_attribute *attr, char *buf)
{

    /* return factory charge mode */
	g_AtCmdDeviceID = get_chip_id();
	
	LGBM_LOG("[chulho.park] g_AtCmdDeviceID :%x\n",g_AtCmdDeviceID);
	
	return sprintf(buf, "%x\n", g_AtCmdDeviceID);
}
//                                                                              

static ssize_t show_LGBM_AtCmdCharge(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return factory charge mode */
    return sprintf(buf, "%d\n", g_AtCmdChargeMode);
}
static ssize_t store_LGBM_AtCmdCharge(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    /* set factory charge mode */
    if(buf != NULL && size != 0)
    {
        if(buf[0] == '1')
        {
            g_AtCmdChargeMode = 1;
        }
        else
        {
            g_AtCmdChargeMode = 0;
        }
    }
    return size;
}
//                                                                              
static DEVICE_ATTR(LGBM_AtCmdDeviceID, 0664, show_LGBM_AtCmdDeviceID, NULL);
//                                                                              

static DEVICE_ATTR(LGBM_AtCmdCharge, 0664, show_LGBM_AtCmdCharge, store_LGBM_AtCmdCharge);

static ssize_t show_LGBM_AtCmdBatl(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery voltage */

    int batVolt = 0;

    if( g_AtCmdChargeMode == 1 )
    {
        batVolt = LGBM_ReadBatVoltAdc();
    }
    else
    {
        batVolt = g_pAtCmdBatVital->batVolt;
    }

    return sprintf(buf, "%d\n", batVolt);
}
static DEVICE_ATTR(LGBM_AtCmdBatl, 0664, show_LGBM_AtCmdBatl, NULL);

static ssize_t show_LGBM_AtCmdBatmp(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery temperature */
    kal_int32 batTemp = 0;

    LGBmVital batVital = {0};

    if( g_AtCmdChargeMode == 1 )
    {
        LGBM_ReadBatVital(&batVital);
        batTemp = batVital.batTemp;
    }
    else
    {
        batTemp = g_pAtCmdBatVital->batTemp;
    }

    return sprintf(buf, "%d\n", batTemp);

}
static DEVICE_ATTR(LGBM_AtCmdBatmp, 0664, show_LGBM_AtCmdBatmp, NULL);

static ssize_t show_LGBM_AtCmdChcomp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int isChargeComplete = 0;
    int batVolt = {0};

    /* return charge complete state */
    if( g_AtCmdChargeMode ==1 )
    {
        batVolt = LGBM_ReadBatVoltAdc();
        if( batVolt > 4250 )
        {
            isChargeComplete = 1;
        }
    }
    else
    {
        if( g_AtCmdBatFullUI == POWER_SUPPLY_STATUS_FULL )
        {
            isChargeComplete = 1;
        }
    }

    return sprintf(buf, "%d\n", isChargeComplete);

}
static DEVICE_ATTR(LGBM_AtCmdChcomp, 0664, show_LGBM_AtCmdChcomp, NULL);

static ssize_t show_LGBM_AtCmdFuelval(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery soc */

    LGBmVital batVital = {0};
    int batSoc = 0;

    if( g_AtCmdChargeMode == 1 )
    {
        LGBM_ReadBatVital(&batVital);
        batSoc = LGBM_GetSocByOcv(batt_id_check, batVital.batTemp, batVital.batVolt);
    }
    else
    {
        batSoc = g_pAtCmdBmData->curSoc;
    }

    return sprintf(buf, "%d\n", batSoc);
}
static DEVICE_ATTR(LGBM_AtCmdFuelval, 0664, show_LGBM_AtCmdFuelval, NULL);

static ssize_t show_LGBM_AtCmdBattlevel(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery soc of UI */
    return sprintf(buf, "%d\n", g_AtCmdBatSocUI);
}
static DEVICE_ATTR(LGBM_AtCmdBattlevel, 0664, show_LGBM_AtCmdBattlevel, NULL);
#endif //                                              

static ssize_t show_LGBM_UpdateBatStateFlag(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", g_updateBatStateFlag);
}
static DEVICE_ATTR(LGBM_UpdateBatStateFlag, 0664, show_LGBM_UpdateBatStateFlag, NULL);

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API
///////////////////////////////////////////////////////////////////////////////////////////
#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static struct hrtimer charger_hv_detect_timer;
static struct task_struct *charger_hv_detect_thread = NULL;
static int charger_hv_detect_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(charger_hv_detect_waiter);

int charger_hv_detect_sw_thread_handler(void *unused)
{
    ktime_t ktime;

    do
    {
        ktime = ktime_set(0, BAT_MS_TO_NS(500));

        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[charger_hv_detect_sw_thread_handler] \n");
        //printk("[charger_hv_detect_sw_thread_handler]\n");

        wait_event_interruptible(charger_hv_detect_waiter, charger_hv_detect_flag != 0);

        charger_hv_detect_flag = 0;

        if(get_pmic_flag == 1)
        {
            if( upmu_chr_get_vcdt_hv_det() == 1)
            {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() == 1\n");
                /*  Disable charger */
#if defined ( LGE_BSP_LGBM ) //                                              
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
#else
                pchr_turn_off_charging();
#endif //                                              
            }
            else
            {
                //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");
            }
               upmu_chr_chrwdt_td(0x0);                // CHRWDT_TD, 4s, check me
           upmu_chr_chrwdt_int_en(1);                // CHRWDT_INT_EN, check me
           upmu_chr_chrwdt_en(1);                     // CHRWDT_EN, check me
           upmu_chr_chrwdt_flag_wr(1);                // CHRWDT_FLAG, check me
        }

        hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

    } while (!kthread_should_stop());

    return 0;
}

enum hrtimer_restart charger_hv_detect_sw_workaround(struct hrtimer *timer)
{
    charger_hv_detect_flag = 1;
    wake_up_interruptible(&charger_hv_detect_waiter);

    //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[charger_hv_detect_sw_workaround] \n");

    return HRTIMER_NORESTART;
}

void charger_hv_detect_sw_workaround_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(0, BAT_MS_TO_NS(500));
    hrtimer_init(&charger_hv_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    charger_hv_detect_timer.function = charger_hv_detect_sw_workaround;
    hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

    charger_hv_detect_thread = kthread_run(charger_hv_detect_sw_thread_handler, 0, "mtk charger_hv_detect_sw_workaround");
    if (IS_ERR(charger_hv_detect_thread))
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[%s]: failed to create charger_hv_detect_sw_workaround thread\n", __FUNCTION__);
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "charger_hv_detect_sw_workaround_init : done\n" );
}
#endif

#if defined(CONFIG_POWER_VERIFY)
static int mt6329_battery_probe(struct platform_device *dev)
{
    int ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6329 battery driver probe!! ********\n" );

    /* Integrate with Android Battery Service */
    ret = power_supply_register(&(dev->dev), &mt6329_ac_main.psy);
    if (ret)
    {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register AC Fail !!\n");
    return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register AC Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6329_usb_main.psy);
    if (ret)
    {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register USB Fail !!\n");
    return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register USB Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6329_battery_main.psy);
    if (ret)
    {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register Battery Fail !!\n");
    return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register Battery Success !!\n");

    /* battery kernel thread for 10s check and charger in/out event */
    //kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread"); //move to pmic driver

    return 0;
}
#else
static int mt6329_battery_probe(struct platform_device *dev)
{
    struct class_device *class_dev = NULL;
    int ret=0;
    int i=0;
    int ret_device_file=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6329 battery driver probe!! ********\n" );

    //move to pmic bank1 probe
    //charger_hv_detect_sw_workaround_init();

    /* Integrate with NVRAM */
    ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
    if (ret)
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Error: Can't Get Major number for adc_cali \n");
    adc_cali_cdev = cdev_alloc();
    adc_cali_cdev->owner = THIS_MODULE;
    adc_cali_cdev->ops = &adc_cali_fops;
    ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
    if(ret)
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "adc_cali Error: cdev_add\n");
    adc_cali_major = MAJOR(adc_cali_devno);
    adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
    class_dev = (struct class_device *)device_create(adc_cali_class,
                                                   NULL,
                                                   adc_cali_devno,
                                                   NULL,
                                                   ADC_CALI_DEVNAME);
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] NVRAM prepare : done !!\n ");

    /* Integrate with Android Battery Service */
    ret = power_supply_register(&(dev->dev), &mt6329_ac_main.psy);
    if (ret)
    {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register AC Fail !!\n");
    return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register AC Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6329_usb_main.psy);
    if (ret)
    {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register USB Fail !!\n");
    return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register USB Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6329_battery_main.psy);
    if (ret)
    {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register Battery Fail !!\n");
    return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6329 BAT_probe] power_supply_register Battery Success !!\n");

    wake_lock_init(&battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery wakelock");

    /* For EM */
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Charger_Voltage);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Slope);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Offset);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_Is_Calibration);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_On_Voltage);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_Off_Voltage);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_TopOff_Value);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Battery_CurrentConsumption);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_SW_CoulombCounter);

#if defined ( LGE_BSP_LGBM ) //                                              
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_usb_cable); /* Using */
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_rf_temperature_ADC); /* Using */
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_pseudo_batt); /* Using */
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdCharge);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatl);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatmp);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChcomp);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdFuelval);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBattlevel);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_UpdateBatStateFlag);
	
	//                                                                              
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdDeviceID);
	//                                                                              
	
#else
//                                                           
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_Mode);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_Complete);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_usb_cable);
//                                                           
//                                                            
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Battery_Voltage);
//                                                            
//                                                          
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_rf_temperature_ADC);
//                                                          
/*                                                                         */
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_rf_temperature);
/*                                                                         */

//                                                                             
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_pseudo_batt);
//                                                                             
#endif //                                              


    /* Initialization BMT Struct */
    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
        batteryCurrentBuffer[i] = 0;
        batteryVoltageBuffer[i] = 0;
        batterySOCBuffer[i] = 0;
        batteryTempBuffer[i] = 0;
    }
    batteryVoltageSum = 0;
    batteryCurrentSum = 0;
    batterySOCSum = 0;
    batteryTempSum = 0;

    //                                                                 
    BMT_status.bat_exist = 1;//(!strcmp(batt_exist, "1") ? 1 : 0);
    //BMT_status.bat_exist = 1;       /* phone must have battery */
    //                                                                 
    BMT_status.charger_exist = 0;     /* for default, no charger */
    BMT_status.bat_vol = 0;
    BMT_status.ICharging = 0;
    BMT_status.temperature = 0;
    BMT_status.charger_vol = 0;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;

    BMT_status.bat_charging_state = CHR_PRE;

#if 1 //                                               
    if(strcmp(batt_id_info, "ds2704_l") == 0)
    {
        batt_id_check = 1;  /* Dallas 1st Code */
    }
    else if(strcmp(batt_id_info, "ds2704_c") == 0)
    {
        batt_id_check = 0;  /* Dallas 2nd Code */
//        batt_id_check = 2;  /* Dallas 2nd Code */
    }
    else if(strcmp(batt_id_info, "isl6296_l") == 0)
    {
        batt_id_check = 1;  /* Intersil 1st Code */
//        batt_id_check = 3;  /* Intersil 1st Code */
    }
    else if(strcmp(batt_id_info, "isl6296_c") == 0)
    {
        batt_id_check = 0;  /* Intersil 2nd Code */
//        batt_id_check = 4;  /* Intersil 2nd Code */
    }
    else
    {
        batt_id_check = 0; /* Unknown Battery ID */
//        batt_id_check = -1; /* Unknown Battery ID */
    }
#else
    //                                                                   
    batt_id_check = (!strcmp(batt_id_info, "ds2704_l") ? 1 : 0);
    //                                                                   
#endif //                                               
    /* Run Battery Thread Use GPT timer */
    BatThread_XGPTConfig();

#if defined ( LGE_BSP_LGBM ) //                                              
#else
    //                                                           
    /*External Charger IC initialization.*/
    ChargerHwInit();
    //                                                           
#endif //                                              

    /* battery kernel thread for 10s check and charger in/out event */
    //kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread"); //move to pmic driver

    /*LOG System Set*/
    init_proc_log();

    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_BATTERY_VOLTAGE_CHANNEL=%d\r\n",AUXADC_BATTERY_VOLTAGE_CHANNEL);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_REF_CURRENT_CHANNEL=%d\r\n",AUXADC_REF_CURRENT_CHANNEL);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_CHARGER_VOLTAGE_CHANNEL=%d\r\n",AUXADC_CHARGER_VOLTAGE_CHANNEL);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_TEMPERATURE_CHANNEL=%d\r\n",AUXADC_TEMPERATURE_CHANNEL);

    return 0;
}
#endif

static int mt6329_battery_remove(struct platform_device *dev)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6329 battery driver remove!! ********\n" );

    return 0;
}

static void mt6329_battery_shutdown(struct platform_device *dev)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6329 battery driver shutdown!! ********\n" );

}

static int mt6329_battery_suspend(struct platform_device *dev, pm_message_t state)
{
    xlog_printk(ANDROID_LOG_VERBOSE, "Power/Battery", "******** MT6329 battery driver suspend!! ********\n" );

    return 0;
}

static int mt6329_battery_resume(struct platform_device *dev)
{
    xlog_printk(ANDROID_LOG_VERBOSE, "Power/Battery", "******** MT6329 battery driver resume!! ********\n" );

    //g_battery_flag_resume=1;

    return 0;
}

struct platform_device MT6329_battery_device = {
    .name   = "mt6329-battery",
    .id        = -1,
};

static struct platform_driver mt6329_battery_driver = {
    .probe        = mt6329_battery_probe,
    .remove        = mt6329_battery_remove,
    .shutdown    = mt6329_battery_shutdown,
    //#ifdef CONFIG_PM
    .suspend    = mt6329_battery_suspend,
    .resume        = mt6329_battery_resume,
    //#endif
    .driver     = {
        .name = "mt6329-battery",
    },
};

#if defined(CONFIG_POWER_VERIFY)
//
#else
///////////////////////////////////////////////////////////////////////////////////////////
//// Battery Notify API
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_BatteryNotify(struct device *dev,struct device_attribute *attr, char *buf)
{
    if (Enable_BATDRV_LOG == 1) {
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] show_BatteryNotify : %x\n", g_BatteryNotifyCode);
    }
    return sprintf(buf, "%u\n", g_BatteryNotifyCode);
}
static ssize_t store_BatteryNotify(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_BatteryNotifyCode = 0;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store_BatteryNotify\n");
    if(buf != NULL && size != 0)
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] buf is %s and size is %d \n",buf,size);
        reg_BatteryNotifyCode = simple_strtoul(buf,&pvalue,16);
        g_BatteryNotifyCode = reg_BatteryNotifyCode;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store code : %x \n",g_BatteryNotifyCode);
    }
    return size;
}
static DEVICE_ATTR(BatteryNotify, 0664, show_BatteryNotify, store_BatteryNotify);

static ssize_t show_BN_TestMode(struct device *dev,struct device_attribute *attr, char *buf)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] show_BN_TestMode : %x\n", g_BN_TestMode);
    return sprintf(buf, "%u\n", g_BN_TestMode);
}
static ssize_t store_BN_TestMode(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_BN_TestMode = 0;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store_BN_TestMode\n");
    if(buf != NULL && size != 0)
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] buf is %s and size is %d \n",buf,size);
        reg_BN_TestMode = simple_strtoul(buf,&pvalue,16);
        g_BN_TestMode = reg_BN_TestMode;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store g_BN_TestMode : %x \n",g_BN_TestMode);
    }
    return size;
}
static DEVICE_ATTR(BN_TestMode, 0664, show_BN_TestMode, store_BN_TestMode);

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API
///////////////////////////////////////////////////////////////////////////////////////////
static int battery_cmd_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n",
        g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);

    *start = buf + off;

    len = p - buf;
    if (len > off)
        len -= off;
    else
        len = 0;

    return len < count ? len  : count;
}

static ssize_t battery_cmd_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    int len = 0, bat_tt_enable=0, bat_thr_test_mode=0, bat_thr_test_value=0;
    char desc[32];

    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';

    if (sscanf(desc, "%d %d %d", &bat_tt_enable, &bat_thr_test_mode, &bat_thr_test_value) == 3)
    {
        g_battery_thermal_throttling_flag = bat_tt_enable;
        battery_cmd_thermal_test_mode = bat_thr_test_mode;
        battery_cmd_thermal_test_mode_value = bat_thr_test_value;

        xlog_printk(ANDROID_LOG_DEBUG, "Power/Thermal", "bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
            g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);

        return count;
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Thermal", "  bad argument, echo [bat_tt_enable] [bat_thr_test_mode] [bat_thr_test_value] > battery_cmd\n");
    }

    return -EINVAL;
}

static int mt_batteryNotify_probe(struct platform_device *dev)
{
    int ret_device_file = 0;
    struct proc_dir_entry *entry = NULL;
    struct proc_dir_entry *battery_dir = NULL;

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** mt_batteryNotify_probe!! ********\n" );

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BatteryNotify);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BN_TestMode);

    battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
    if (!battery_dir)
    {
        pr_err("[%s]: mkdir /proc/mtk_battery_cmd failed\n", __FUNCTION__);
    }
    else
    {
        entry = create_proc_entry("battery_cmd", S_IRUGO | S_IWUSR, battery_dir);
        if (entry)
        {
            entry->read_proc = battery_cmd_read;
            entry->write_proc = battery_cmd_write;
        }
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** mtk_battery_cmd!! ********\n" );

    return 0;
}

struct platform_device MT_batteryNotify_device = {
    .name   = "mt-battery",
    .id        = -1,
};

static struct platform_driver mt_batteryNotify_driver = {
    .probe        = mt_batteryNotify_probe,
    .driver     = {
        .name = "mt-battery",
    },
};
#endif

static int __init mt6329_battery_init(void)
{
    int ret;

    ret = platform_device_register(&MT6329_battery_device);
    if (ret) {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt6329_battery_driver] Unable to device register(%d)\n", ret);
    return ret;
    }
    ret = platform_driver_register(&mt6329_battery_driver);
    if (ret) {
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt6329_battery_driver] Unable to register driver (%d)\n", ret);
    return ret;
    }

#if defined(CONFIG_POWER_VERIFY)
//
#else
    // battery notofy UI
    ret = platform_device_register(&MT_batteryNotify_device);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt_batteryNotify] Unable to device register(%d)\n", ret);
        return ret;
    }
    ret = platform_driver_register(&mt_batteryNotify_driver);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt_batteryNotify] Unable to register driver (%d)\n", ret);
        return ret;
    }
#endif

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt6329_battery_driver] Initialization : DONE !!\n");

    return 0;
}

static void __exit mt6329_battery_exit (void)
{
}

module_init(mt6329_battery_init);
module_exit(mt6329_battery_exit);

MODULE_AUTHOR("James Lo");
MODULE_DESCRIPTION("MT6329 Battery Device Driver");
MODULE_LICENSE("GPL");
