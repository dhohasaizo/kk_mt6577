#include "partition_define.h"
static const struct excel_info PartInfo_Private[PART_NUM]={
			{"preloader",262144,0x0, EMMC, 0,BOOT_1},
			{"dsp_bl",12320768,0x40000, EMMC, 0,BOOT_1},
			{"mbr",524288,0xc00000, EMMC, 0,USER},
			{"ebr1",524288,0xc80000, EMMC, 1,USER},
			{"ebr2",524288,0xd00000, EMMC, 0,USER},
			{"ebr3",524288,0xd80000, EMMC, 0,USER},
			{"misc2",8388608,0xe00000, EMMC, 0,USER},
			{"pmt",4194304,0x1600000, EMMC, 0,USER},
			{"nvram",3145728,0x1a00000, EMMC, 0,USER},
			{"seccfg",524288,0x1d00000, EMMC, 0,USER},
			{"uboot",524288,0x1d80000, EMMC, 0,USER},
			{"bootimg",9437184,0x1e00000, EMMC, 0,USER},
			{"recovery",10485760,0x2700000, EMMC, 0,USER},
			{"sec_ro",6291456,0x3100000, EMMC, 2,USER},
			{"misc",524288,0x3700000, EMMC, 0,USER},
			{"logo",3145728,0x3780000, EMMC, 0,USER},
			{"expdb",1048576,0x3a80000, EMMC, 0,USER},
			{"persist_lg",8388608,0x3b80000, EMMC, 3,USER},
			{"persist",6291456,0x4380000, EMMC, 4,USER},
			{"mpt",23068672,0x4980000, EMMC, 5,USER},
			{"lgfota",10485760,0x5f80000, EMMC, 0,USER},
			{"swap",10485760,0x6980000, EMMC, 6,USER},
			{"cust",78643200,0x7380000, EMMC, 7,USER},
			{"rct",1048576,0xbe80000, EMMC, 0,USER},
			{"android",1771044864,0xbf80000, EMMC, 8,USER},
			{"cache",744488960,0x75880000, EMMC, 9,USER},
			{"usrdata",5100273664,0xa1e80000, EMMC, 10,USER},
			{"fat",0,0x1d1e80000, EMMC, 11,USER},
			{"bmtpool",10485760,0xFFFF0050, EMMC, 0,USER},
 };

#ifdef  MTK_EMMC_SUPPORT
struct MBR_EBR_struct MBR_EBR_px[MBR_COUNT]={
	{"mbr", {1, 2, 3, 4, }},
	{"ebr1", {5, 6, 7, }},
	{"ebr2", {8, 9, 10, }},
	{"ebr3", {11, }},
};

EXPORT_SYMBOL(MBR_EBR_px);
#endif

