#include <linux/kernel.h>
#include <linux/string.h>
#include <mach/mt_clock_manager.h>
#include <mach/mtk_rtc.h>
/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
#include <asm/io.h>
#include "mach/mt_devs.h"
#include "mach/mt_gpio.h"
#include <mach/fbcon.h>
#include <../drivers/video/dpi_reg.h>
#endif
/*                                                                      */

extern void wdt_arch_reset(char);
/*                                                 */
#ifdef CONFIG_LOCAL_WDT
enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE,
	WK_WDT_LOC_TYPE_NOLOCK,
	WK_WDT_EXT_TYPE_NOLOCK,
};
extern void mtk_wdt_restart(enum wk_wdt_type type);
#else
extern void mtk_wdt_restart(void);
#endif
/*                                                 */

/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
static struct fbcon_config mipi_fb_cfg =
{
    .width          = LCM_WIDTH,
    .height         = LCM_HEIGHT,
    .stride         = LCM_WIDTH,
    .format         = FB_FORMAT_RGB888,
    .bpp            = 24,
    .update_start   = NULL,
    .update_done    = NULL,
};

static PDPI_REGS const DPI_REG = (PDPI_REGS)(DPI_BASE);

struct crash_log_dump {
	unsigned int magic_key;
	unsigned int size;
	unsigned char buffer[0];
};

static struct crash_log_dump *crash_dump_log;
static unsigned long *cpu_crash_ctx = NULL;

static volatile GPIO_REGS *reg_gpio = (GPIO_REGS*) GPIO_BASE;
volatile u16 key_input_val = 0;

#endif
/*                                                                      */

/*                                                           */
#if defined(CONFIG_LGE_HIDDEN_RESET)
extern int hreset_enable;
#endif
/*                                          */

void arch_reset(char mode, const char *cmd)
{
/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
    DPI_REG_STATUS      reg_status;
    char szMsg[128];
    unsigned long i = 0;

/*                                                                                */

/*                                                                                    */
#if defined(MT6577)
   volatile u32 *lge_boot_magic_number 	= (volatile u32*)(INTERNAL_SRAM_BASE + 0x0C000 - 32);
#else
   volatile u32 *lge_boot_magic_number 	= (volatile u32*)(INTERNAL_SRAM_BASE + 0x10000 - 32);
#endif
   volatile u32 *lge_boot_reason 		= lge_boot_magic_number + 1; // next 4 byte


/*                                                 */

#endif
/*                                                                      */

#if !defined ( CONFIG_LGE_HANDLE_PANIC ) /*                                                 */
/*                                                                                     */
#ifdef MT6575
  if (cmd && !strcmp(cmd, "charger")) {
      	wdt_arch_reset(0);
    }
	else
	{
		wdt_arch_reset(1);
	}
#endif
#else
#ifndef CONFIG_MT6577_FPGA
    char reboot = 0;
    printk("arch_reset: cmd = %s\n", cmd ? : "NULL");

    if (cmd && !strcmp(cmd, "charger")) {
        /* do nothing */
    }
#if defined( LGE_BSP_LGBM )
    else if (cmd && !strcmp(cmd, "charge_reset")) {
        lge_save_boot_reason(0x776655AA, 0, 0);
    }
#endif
      else if (cmd && !strcmp(cmd, "recovery")) {
        rtc_mark_recovery();
    } else if (cmd && !strcmp(cmd, "--bnr_recovery")) {
        lge_save_boot_reason(LGE_BOOT_BNR_RECOVERY_MODE_REBOOT, 0, 0);
        /*                                                                  */
        rtc_mark_recovery();
        /*                                                                  */
    } else {
    	reboot = 1;
    }

/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
/*                                                                                */
/*                                                         */
    if (*lge_boot_magic_number == LGE_BOOT_REASON_MAGIC_CODE && *lge_boot_reason == LGE_BOOT_KERNEL_CRASH
        && hreset_enable == 0)
    {
/*                                                  */
        reg_status          = DPI_REG->STATUS;

        mipi_fb_cfg.width   = DPI_REG->SIZE.WIDTH;
        mipi_fb_cfg.height  = DPI_REG->SIZE.HEIGHT;
        mipi_fb_cfg.stride  = DPI_REG->SIZE.WIDTH;

        mipi_fb_cfg.base    = (void*) ioremap_cached((unsigned int)DPI_REG->FB[reg_status.FB_INUSE].ADDR,
                                                     (mipi_fb_cfg.width * mipi_fb_cfg.height * (mipi_fb_cfg.bpp / 3)));

        crash_dump_log      = (struct crash_log_dump *) ioremap_cached(LGE_BSP_CRASH_LOG_PHY_ADDR, LGE_BSP_CRASH_LOG_SIZE);
        cpu_crash_ctx       = (unsigned long *) ioremap_cached(LGE_CRASH_CTX_BUF_PHY_ADDR, LGE_CRASH_CTX_BUF_SIZE);

        if (mipi_fb_cfg.base && crash_dump_log && cpu_crash_ctx)
        {
            fbcon_setup(&mipi_fb_cfg);

            fbcon_puts("============================================================\n");
            fbcon_puts("Kernel Crash!\n");
            fbcon_puts("============================================================\n");
            fbcon_puts("MTK6577 Crash Handler : Kernel Crash! \n");
            fbcon_puts(" \n");
            fbcon_puts("Please do following action. \n");
            fbcon_puts("  1) Press key to Volumn Up or Down \n");
            fbcon_puts("  2) Please connect USB. \n");
            fbcon_puts("  3) Get the ram dump image using MTK's ram dump downloader \n");
            fbcon_puts("  4) Send e-mail to contact point or L10-Project@lge.com \n");
            fbcon_puts(" \n");
            fbcon_puts("============================================================\n");
/*                                                                                                                */
            memset (szMsg, 0, sizeof(szMsg));
            sprintf(szMsg, " R0 : 0x%08X  R1 : 0x%08X  R2 : 0x%08X \n", *cpu_crash_ctx, *(cpu_crash_ctx + 1), *(cpu_crash_ctx + 2));
            fbcon_puts(szMsg);
            sprintf(szMsg, " R3 : 0x%08X  R4 : 0x%08X  R5 : 0x%08X \n", *(cpu_crash_ctx + 3), *(cpu_crash_ctx + 4), *(cpu_crash_ctx + 5));
            fbcon_puts(szMsg);
            sprintf(szMsg, " R6 : 0x%08X  R7 : 0x%08X  R8 : 0x%08X \n", *(cpu_crash_ctx + 6), *(cpu_crash_ctx + 7), *(cpu_crash_ctx + 8));
            fbcon_puts(szMsg);
            sprintf(szMsg, " R9 : 0x%08X R10 : 0x%08X R11 : 0x%08X \n", *(cpu_crash_ctx + 9), *(cpu_crash_ctx + 10), *(cpu_crash_ctx + 11));
            fbcon_puts(szMsg);
            sprintf(szMsg, "R12 : 0x%08X  SP : 0x%08X  LR : 0x%08X \n", *(cpu_crash_ctx + 12), *(cpu_crash_ctx + 13), *(cpu_crash_ctx + 14));
            fbcon_puts(szMsg);
            sprintf(szMsg, " PC : 0x%08X  CPSR : 0x%08X \n", *(cpu_crash_ctx + 15), *(cpu_crash_ctx + 16));
            fbcon_puts(szMsg);
            sprintf(szMsg, "CTRL : 0x%08X TRANSBASE : 0x%08X  DAC : 0x%08X \n", *(cpu_crash_ctx + 17), *(cpu_crash_ctx + 18), *(cpu_crash_ctx + 19));
            fbcon_puts(szMsg);
/*                                                                                                                */
            fbcon_puts(" \n");
            fbcon_puts(" \n");
            fbcon_puts("------------------------------------------------------------\n");
            fbcon_puts(crash_dump_log->buffer);
			
            /* Volumn down key : KP_ROW1(GPIO_97) & KP_COL1(GPIO_108) */
            /* Volumn up key : KP_ROW0() & KP_COL1(GPIO_108) */

            do
            {
                /*                                                                                                                */
#ifdef CONFIG_LOCAL_WDT
                mtk_wdt_restart(WK_WDT_EXT_TYPE);
#else
                mtk_wdt_restart();
#endif
                /*                                                                                                                */

                key_input_val = 0;
                key_input_val = reg_gpio->din[6].val;
/*                                                                                                                                            */
                key_input_val = key_input_val & 0x2000;
/*                                                                                                                                            */
                i = 0;
                while (++i < 0x50000000)
                {
                }
            } while (key_input_val != 0);

        }
/*                                                                                */
    }
/*                                                  */
/*                                                           */
    else if (*lge_boot_magic_number == LGE_BOOT_REASON_MAGIC_CODE && *lge_boot_reason == LGE_BOOT_KERNEL_CRASH
        && hreset_enable == 1)  {
        lge_save_boot_reason(LGE_BOOT_HIDDEN_RESET_REBOOT, 0, 0);
    }
/*                                                           */

#endif
/*                                                                      */

/*                                                */
#if defined(TARGET_S1_77)
	mt_set_gpio_mode(GPIO_LCD_RESET, GPIO_LCD_RESET_M_GPIO); /* LCD ENABLE PIN -> LOW */
	mt_set_gpio_pull_enable(GPIO_LCD_RESET, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCD_RESET, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RESET,GPIO_OUT_ZERO);	
#endif
/*                                                 */

    wdt_arch_reset(reboot);

#endif
#endif
}

#ifdef CONFIG_MT6577_FPGA
void arch_idle(void)
{}
#endif
