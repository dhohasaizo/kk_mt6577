#include <linux/delay.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/i2c.h>


#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#include "mms100_ISP_download.h"


#include "core_bin_V01.h"
#include "core_bin_R50.h"
#include "core_bin_R51.h"


#define UINT32 unsigned long
#define UINT16 unsigned short
#define UINT8 unsigned char

#define MFS_HEADER_		5
#define MFS_DATA_		20480

//#define DATA_SIZE 1024
#define DATA_SIZE 128


#define CLENGTH 4
#define PACKET_			(MFS_HEADER_ + MFS_DATA_)

/*
 * State Registers
 */

/*
 * Config Update Commands
 */
#define ISC_CMD_ENTER_ISC						0x5F
#define ISC_CMD_ENTER_ISC_PARA1					0x01
#define ISC_CMD_ISC_ADDR						0xD5
#define ISC_CMD_ISC_STATUS_ADDR					0xD9

/*
 * ISC Status Value
 */
#define ISC_STATUS_RET_MASS_ERASE_DONE			0X0C
#define ISC_STATUS_RET_MASS_ERASE_MODE			0X08

//typedef int

#define MFS_CHAR_2_BCD(num)	\
	(((num/10)<<4) + (num%10))
#define MFS_MAX(x, y)		( ((x) > (y))? (x) : (y) )

#define MFS_DEFAULT_SLAVE_ADDR	0x48

static unsigned char *buf;
const unsigned char mfs_i2c_slave_addr = 0x48;
unsigned char mfs_slave_addr;




extern struct i2c_client *melfas_i2c_client;
extern int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 len, u8 *rxbuf);
extern int melfas_i2c_write_data(struct i2c_client *client,int len, u8 *txbuf);
extern int melfas_i2c_read_data(struct i2c_client *client, int len, u8 *rxbuf);

extern void tpd_hw_enable(void);
extern void tpd_hw_disable(void);

mfs_bool_t MFS_I2C_set_slave_addr(unsigned char _slave_addr)
{
	mfs_slave_addr = _slave_addr << 1; /*수정하지 마십시오.*/

	/* TODO: I2C slave address를 셋팅해 주세요. */
	return MFS_TRUE;
}

mfs_bool_t MFS_I2C_read_with_addr(unsigned char* _read_buf,
		unsigned char _addr, int _length)
{
	melfas_i2c_read(melfas_i2c_client, _addr, _length, _read_buf);

	/* TODO: I2C로 1 byte address를 쓴 후 _length 갯수만큼 읽어 _read_buf에 채워 주세요. */
	return MFS_TRUE;
}

mfs_bool_t MFS_I2C_write(const unsigned char* _write_buf, int _length)
{
	/*
	 * TODO: I2C로 _write_buf의 내용을 _length 갯수만큼 써 주세요.
	 * address를 명시해야 하는 인터페이스의 경우, _write_buf[0]이 address가 되고
	 * _write_buf+1부터 _length-1개를 써 주시면 됩니다.
	 */
	melfas_i2c_write_data(melfas_i2c_client, _length, _write_buf);
	return MFS_TRUE;
}

mfs_bool_t MFS_I2C_read(unsigned char* _read_buf, int _length)
{
	/* TODO: I2C로 _length 갯수만큼 읽어 _read_buf에 채워 주세요. */
	melfas_i2c_read_data(melfas_i2c_client, _length, _read_buf);
	return MFS_TRUE;
}


void MFS_debug_msg(const char* fmt, int a, int b, int c)
{
	//TODO: message logging을 위한 함수를 연결해 주세요.
	//printk(fmt);	
}
void MFS_ms_delay(int msec)
{
	msleep(msec);
}

void MFS_reboot(void)
{
	printk("<MELFAS> TOUCH IC REBOOT!!!\n");
	//TODO: rebooting을 위한 함수를 연결해 주세요.
	tpd_hw_disable();
	msleep(100);
	tpd_hw_enable();
	msleep(100);
}


int mass_erase(void)
{
	int i = 0;
	const unsigned char mass_erase_cmd[MFS_HEADER_] =
	{ ISC_CMD_ISC_ADDR, 0, 0xC1, 0, 0 };

	unsigned char read_buffer[4] =
	{ 0, };

	printk("<MELFAS> mass erase start\n\n");

	MFS_ms_delay(5);
	
	if (!MFS_I2C_write(mass_erase_cmd, MFS_HEADER_))
		return MRET_I2C_ERROR;

	printk("<MELFAS> Firmware Mass Erase ..........\n");

	MFS_ms_delay(5);

	while (read_buffer[2] != ISC_STATUS_RET_MASS_ERASE_DONE)
	{
		if (!MFS_I2C_read_with_addr(read_buffer, ISC_CMD_ISC_STATUS_ADDR, 4))
			return MRET_I2C_ERROR;

		if (read_buffer[2] == ISC_STATUS_RET_MASS_ERASE_DONE)
		{
			printk("<MELFAS> Firmware Mass Erase done.\n");
			return MRET_SUCCESS;
		}
		else if (read_buffer[2] == ISC_STATUS_RET_MASS_ERASE_MODE)
			printk("<MELFAS> Firmware Mass Erase enter success!!!\n");

		MFS_ms_delay(1);
		if (i > 20)
			return MRET_MASS_ERASE_ERROR;
		i++;
	}
	return MRET_SUCCESS;
}


int firmware_write(const unsigned char *_pBinary_Data)
{
	int i = 0;
	int k = 0;
	int ret = 0;
	unsigned char write_buffer[MFS_HEADER_ + DATA_SIZE];
	unsigned short int start_addr = 0;

	printk("<MELFAS> FIRMARE WRITING...\n");
	printk("<MELFAS> ");
	while (start_addr * CLENGTH < MFS_DATA_)
	{
		write_buffer[0] = ISC_CMD_ISC_ADDR;
		write_buffer[1] = (UINT8) ((start_addr) & 0X00FF);
		write_buffer[2] = (UINT8) ((start_addr >> 8) & 0X00FF);
		write_buffer[3] = 0X00;
		write_buffer[4] = 0X00;

		for (i = 0; i < DATA_SIZE; i++)
			write_buffer[MFS_HEADER_ + i] = _pBinary_Data[i	+ start_addr * CLENGTH];

		MFS_ms_delay(10);
		
		if (!MFS_I2C_write(write_buffer, MFS_HEADER_ + DATA_SIZE))
			return MRET_I2C_ERROR;
		
		printk("touch update ");

		MFS_ms_delay(5);
		k++;
		start_addr = DATA_SIZE * k / CLENGTH;
	}
	printk("\n");
	
	return MRET_SUCCESS;
}


int firmware_verify(const unsigned char *_pBinary_Data)
{

	int i, k = 0;
	unsigned char write_buffer[MFS_HEADER_], read_buffer[DATA_SIZE];
	unsigned short int start_addr = 0;

	printk("<MELFAS> FIRMARE VERIFY...\n");
	printk("<MELFAS> ");
	while (start_addr * CLENGTH < MFS_DATA_)
	{
		write_buffer[0] = ISC_CMD_ISC_ADDR;
		write_buffer[1] = (UINT8) ((start_addr) & 0X00FF);
		write_buffer[2] = 0x40 + (UINT8) ((start_addr >> 8) & 0X00FF);
		write_buffer[3] = 0X00;
		write_buffer[4] = 0X00;

		if (!MFS_I2C_write(write_buffer, MFS_HEADER_))
			return MRET_I2C_ERROR;

		MFS_ms_delay(5);
		if (!MFS_I2C_read(read_buffer, DATA_SIZE))
			return MRET_I2C_ERROR;

		for (i = 0; i < DATA_SIZE; i++)
			if (read_buffer[i] != _pBinary_Data[i + start_addr * CLENGTH])
			{
				printk("<MELFAS> VERIFY Failed\n");
				printk(
						"<MELFAS> original : 0x%2x, buffer : 0x%2x, addr : %d \n",
						_pBinary_Data[i + start_addr * CLENGTH], read_buffer[i],
						i);
				return MRET_FIRMWARE_VERIFY_ERROR;
			}
		printk("%dbyte ", k);
		k++;
		start_addr = DATA_SIZE * k / CLENGTH;

	}
	printk("\n");
	return MRET_SUCCESS;
}

int mms100_ISC_download_binary_data(int hw_ver)
{
	int nRet = 0;
	int retry_cnt = 0;

	
	MFS_reboot();
	mass_erase();
	MFS_reboot();

	if (hw_ver == 0) // 150 Ohm
	{
		printk(" +++ mms100_ISP_download_binary_data R-150 Ohm\n");
		nRet = firmware_write(MELFAS_binary);
		if (nRet != MRET_SUCCESS)
		{
			printk("<MELFAS> WRITE Failed\n");
		}

		nRet = firmware_verify(MELFAS_binary);
		if (nRet != MRET_SUCCESS)
		{
			mass_erase();
			MFS_reboot();
			nRet = firmware_write(MELFAS_binary);
			nRet = firmware_verify(MELFAS_binary);
		}
		if (nRet != MRET_SUCCESS)
			printk(" +++ TOUCH download Failed.. \n");
	} 
	else if (hw_ver == 1) // 270 Ohm
	{
		printk(" +++ mms100_ISP_download_binary_data R-270 Ohm\n");
		nRet = firmware_write(MELFAS_binary_r50);
		if (nRet != MRET_SUCCESS)
		{
			printk("<MELFAS> WRITE Failed\n");
		}

		nRet = firmware_verify(MELFAS_binary_r50);
		if (nRet != MRET_SUCCESS)
		{
			mass_erase();
			MFS_reboot();
			nRet = firmware_write(MELFAS_binary_r50);
			nRet = firmware_verify(MELFAS_binary_r50);
		}
		if (nRet != MRET_SUCCESS)
			printk(" +++ TOUCH download Failed.. \n");
	}
	else // hw_ver == 2
	{
		printk(" +++ mms100_ISP_download_binary_data R51 / R71 Ohm\n");
		nRet = firmware_write(MELFAS_binary_r51);
		if (nRet != MRET_SUCCESS)
		{
			printk("<MELFAS> WRITE Failed\n");
		}

		nRet = firmware_verify(MELFAS_binary_r51);
		if (nRet != MRET_SUCCESS)
		{
			mass_erase();
			MFS_reboot();
			nRet = firmware_write(MELFAS_binary_r51);
			nRet = firmware_verify(MELFAS_binary_r51);
		}
		if (nRet != MRET_SUCCESS)
			printk(" +++ TOUCH download Failed.. \n");
	}
	return nRet;
}

