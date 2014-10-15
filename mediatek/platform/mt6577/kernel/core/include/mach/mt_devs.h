#ifndef __MT_DEVS_H__
#define __MT_DEVS_H__

#include <board-custom.h>
#include <mach/board.h>

#define CFG_DEV_UART1
#define CFG_DEV_UART2

#ifndef CONFIG_MT6577_FPGA
#define CFG_DEV_UART3
#define CFG_DEV_UART4
#endif

/*
 * Define constants.
 */

#define MTK_UART_SIZE 0x100

 /*                              
                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC) || defined(CONFIG_LGE_HIDDEN_RESET)
#ifdef CONFIG_MACH_LGE_V5
	// RESERVED_ADDR_END	: 0x30,000,000 (= 768 mb)
	// HIDDEN_RESET			: 0x2F,F3F,400 ~ 0x2F,F3F,400 + 771 kb
	// CTX				    : 0x2F,F3F,000 ~ 0x2F,F3F,000 + 1kb
	// CRASH_LOG			: 0x2F,F3E,000 ~ 0x2F,F3E,000 + 4 kb
	// RAM_CONSOLE 			: 0x2F,F00,000 ~ 0x2F,F00,000 + 248 kb
	// RESERVED_MEM			: 0x2F,F00,000 (= 0x30,000,000 - 1 mb(0x100,000))

#ifdef TARGET_S1_75
	#define LGE_BSP_MEM_MAX_PHY_ADDR		(512 * SZ_1M)
#else
	#define LGE_BSP_MEM_MAX_PHY_ADDR		(1024 * SZ_1M)
#endif

#endif
	#define LGE_BSP_RESERVED_MEM_SIZE		(1 * SZ_1M)
	#define LGE_BSP_RESERVED_MEM_PHY_ADDR	(LGE_BSP_MEM_MAX_PHY_ADDR - LGE_BSP_RESERVED_MEM_SIZE)

	#define LGE_BSP_RAM_CONSOLE_PHY_ADDR	(LGE_BSP_RESERVED_MEM_PHY_ADDR)
	#define LGE_BSP_RAM_CONSOLE_SIZE    	(124 * 2 * SZ_1K)

	#define LGE_BSP_CRASH_LOG_PHY_ADDR		(LGE_BSP_RAM_CONSOLE_PHY_ADDR + LGE_BSP_RAM_CONSOLE_SIZE)
	#define LGE_BSP_CRASH_LOG_SIZE      	(4 * SZ_1K)

	#define LGE_CRASH_CTX_BUF_PHY_ADDR 		(LGE_BSP_CRASH_LOG_PHY_ADDR + LGE_BSP_CRASH_LOG_SIZE)
	#define	LGE_CRASH_CTX_BUF_SIZE			(1 * SZ_1K)

	#define LGE_BSP_HIDDEN_RESET_PHY_ADDR	(LGE_CRASH_CTX_BUF_PHY_ADDR + LGE_CRASH_CTX_BUF_SIZE)
	#define LGE_BSP_HIDDEN_RESET_SIZE      	(LGE_BSP_MEM_MAX_PHY_ADDR - LGE_BSP_HIDDEN_RESET_PHY_ADDR)


	// these are the standard values. used in lge_save_boot_reason(), lge_get_boot_reason()
	// use these values if need in other files
	#define LGE_BOOT_REASON_MAGIC_CODE		0x1234ABCD
	// boot reason value
	//	prefix, postfix 1 bytes (0xff) so, 0xFFxxxxFF
	#define LGE_BOOT_KERNEL_CRASH 				0xFF0001FF
	#define LGE_BOOT_HIDDEN_RESET_REBOOT		0xFF0002FF
	#define LGE_BOOT_BNR_RECOVERY_MODE_REBOOT	0x77665555
/*                                                                                          */
    #define LGE_BOOT_NORMAL_POWER_OFF           0xFF0003FF
/*                                                                                          */

	int lge_save_boot_reason(unsigned long reason, unsigned long extra1, unsigned long extra2);
	unsigned long lge_get_boot_reason(unsigned long *pExtra1, unsigned long * pExtra2);

#endif

/*
 * Define function prototype.
 */

 /*                              
                                       */
#if defined(CONFIG_LGE_HANDLE_PANIC)
void __init lge_add_panic_handler_devices(void);
#endif

#if defined(CONFIG_LGE_HIDDEN_RESET)
//void lge_set_reboot_reason(unsigned int reason); //temp block regarding for rebooting
#endif


extern int mt_board_init(void);

#endif  /* !__MT6577_DEVS_H__ */

