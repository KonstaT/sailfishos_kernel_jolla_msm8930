/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR        Note
 *    1.0		  2010-01-05			WenFS    only support mulititouch	Wenfs 2010-10-01
 *    2.0         2011-09-05            Duxx     Add touch key, and project setting update, auto CLB command
 *    3.0		  2011-09-09			Luowj   
 *
 */

//Leon add
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#ifdef   CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/slab.h>
#include <linux/time.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <mach/vreg.h>
//

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/ft5x06_ts.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>   
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>

//Leon add
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

// Jonas, 20121011, add for FTD {
#include <linux/debugfs.h>
// } Jonas, 20121011, add for FTD

static unsigned int TP_DBG_LEVEL = 0;
#define DPRINTK(level, args...) if(level <= TP_DBG_LEVEL) printk("[TP], " args);
#define DPRINTK_NP(level, args...) if(level <= TP_DBG_LEVEL) printk(args);

static struct i2c_client *this_client;

#define CONFIG_FT5X0X_MULTITOUCH 1
#define CFG_SUPPORT_TOUCH_KEY 1

struct ts_event
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];              //x coordinate
	u16 au16_y[CFG_MAX_TOUCH_POINTS];              //y coordinate
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];       //touch ID
	u8 au8_touchWeight[CFG_MAX_TOUCH_POINTS];       //touch Weight
	u8 au8_touchArea[CFG_MAX_TOUCH_POINTS];       //touch Area
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data
{
	struct input_dev *input_dev;
	struct ts_event event;
	struct work_struct pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend early_suspend;
	struct mutex device_mode_mutex; /* Ensures that only one function can specify the Device Mode at a time. */
//Leon
	struct regulator *avdd_regulator; //L8 voltage
	struct regulator *i2c_regulator; //LVS4
	/* [DSPS] Power Rails         */
	/*    VDD (VREG_L9_2P85) ->   */
	/*    VLOGIC (VREG_LVS4_1P8)  */
	struct regulator *ldo9_regulator;
	int irq; //ISR number
	int gpio_irq; //GPIO number
	int gpio_rst; //GPIO reset

	// Jonas, 20121011, Add for FTD test {
	struct dentry   *dent;
	// } Jonas, 20121011, Add for FTD test
};

//u16 g_RawData[FT5x0x_TX_NUM][FT5x0x_RX_NUM];
u16 *g_RawData = 0;
unsigned char tx_num = 0;
unsigned char rx_num = 0;

// Save previous X, Y for FTD used
static int ftd_OldX = 0;
static int ftd_OldY = 0;

//Leon
//static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts);
//static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts);
//Leon

#if CFG_SUPPORT_TOUCH_KEY
// Jonas, 20121009, Set touch key {
int tsp_keycodes[CFG_NUMOFKEYS] = {
	KEY_BACK, KEY_HOMEPAGE, KEY_MENU
};

char *tsp_keyname[CFG_NUMOFKEYS] = {
	"Back", "Home", "Menu"
};
// } Jonas, 20121009, Set touch key

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif
/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata

Input	:	*rxdata
                     *length

Output	:	ret

function	:

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

	//
	buf[0] = addr;    //register address

	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = this_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
}

static int ft5x0x_read_block(u8 addr, u8 len, u8 *buf)
{
	int ret;
	//u8 buf[2];
	struct i2c_msg msgs[2];

	//
	buf[0] = addr;    //register address

	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = this_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void


Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}

#if 1  //upgrade related
typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
} E_UPGRADE_ERR_TYPE;

typedef unsigned char		FTS_BYTE;	//8 bit
typedef unsigned short		FTS_WORD;	//16 bit
typedef unsigned int		FTS_DWRD;	//16 bit
typedef unsigned char		FTS_BOOL;	//8 bit

typedef struct _FTS_CTP_PROJECT_SETTING_T
{
	unsigned char uc_i2C_addr;             //I2C slave address (8 bit address)
	unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
	unsigned char uc_panel_factory_id;     //TP panel factory ID
} FTS_CTP_PROJECT_SETTING_T;

#define FTS_NULL			0x0
#define FTS_TRUE			0x01
#define FTS_FALSE			0x0

#define I2C_CTPM_ADDRESS	0x70


void delay_qt_ms(unsigned long w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
}

/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;

	ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if (ret <= 0) {
		printk("[FTS]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
	ret = i2c_master_send(this_client, pbt_buf, dw_lenth);
	if (ret <= 0) {
		printk("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]      :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd, FTS_BYTE btPara1, FTS_BYTE btPara2, FTS_BYTE btPara3, FTS_BYTE num)
{
	FTS_BYTE write_cmd[4] = { 0 };

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]:
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[] = {
	//#include "ft_app.i"
};

E_UPGRADE_ERR_TYPE fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	FTS_BYTE reg_val[2] = { 0 };
	FTS_DWRD i = 0;

	FTS_DWRD packet_number;
	FTS_DWRD j;
	FTS_DWRD temp;
	FTS_DWRD lenght;
	FTS_BYTE packet_buf[FTS_PACKET_LENGTH + 6];
	FTS_BYTE auc_i2c_write_buf[10];
	FTS_BYTE bt_ecc;
	int i_ret;

	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	ft5x0x_write_reg(0xfc, 0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	ft5x0x_write_reg(0xfc, 0x55);
	printk("[FTS] Step 1: Reset CTPM test\n");

	delay_qt_ms(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
		delay_qt_ms(5);
	} while (i_ret <= 0 && i < 5);

	/*********Step 3:check READ-ID***********************/
	cmd_write(0x90, 0x00, 0x00, 0x00, 4);
	byte_read(reg_val, 2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x7) {
		//Leon 0x07 for FT5316
		printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	} else {
		printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x Fail !!\n", reg_val[0], reg_val[1]);
		return ERR_READID;
		//i_is_new_protocol = 1;
	}

	cmd_write(0xcd, 0x0, 0x00, 0x00, 1);
	byte_read(reg_val, 1);
	printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

	/*********Step 4:erase app and panel paramenter area ********************/
	cmd_write(0x61, 0x00, 0x00, 0x00, 1);  //erase app area
	delay_qt_ms(1500);
	cmd_write(0x63, 0x00, 0x00, 0x00, 1);  //erase panel parameter area
	delay_qt_ms(100);
	printk("[FTS] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[FTS] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE) (temp >> 8);
		packet_buf[3] = (FTS_BYTE) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE) (lenght >> 8);
		packet_buf[5] = (FTS_BYTE) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		byte_write(&packet_buf[0], FTS_PACKET_LENGTH + 6);
		delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0) {
			printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int) j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE) (temp >> 8);
		packet_buf[3] = (FTS_BYTE) temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE) (temp >> 8);
		packet_buf[5] = (FTS_BYTE) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		byte_write(&packet_buf[0], temp + 6);
		delay_qt_ms(20);
	}

	//send the last six byte
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE) (temp >> 8);
		packet_buf[3] = (FTS_BYTE) temp;
		temp = 1;
		packet_buf[4] = (FTS_BYTE) (temp >> 8);
		packet_buf[5] = (FTS_BYTE) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0], 7);
		delay_qt_ms(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc, 0x00, 0x00, 0x00, 1);
	byte_read(reg_val, 1);
	printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if (reg_val[0] != bt_ecc) {
		return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07, 0x00, 0x00, 0x00, 1);

	msleep(300);  //make sure CTP startup normally

	return ERR_OK;
}

#if 0
static int fts_Get_RawData(void)
{
	int retval = 0;
	int i = 0;
	//u16 dataval = 0x0000;
	u8 devmode = 0x00;
	u8 rownum = 0x00;
	u8 read_buffer[FT5x0x_RX_NUM * 2];
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = FT5x0x_RX_NUM * 2,
			.buf = read_buffer,
		},
	};

	struct ft5x0x_ts_data * ft5x0x_ts = i2c_get_clientdata(this_client);
	if(ft5x0x_enter_factory(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR: could not enter factory mode", __FUNCTION__);
		retval = -1;
		goto error_return;
	}
	//scan
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	devmode |= 0x80;
	if(ft5x0x_write_reg(0x00, devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	msleep(20);
	if(ft5x0x_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	if(0x00 != (devmode&0x80))
	{
		pr_err("%s %d ERROR: could not scan", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	pr_info("Read rawdata .......\n");
	for(rownum=0; rownum<FT5x0x_TX_NUM; rownum++)
	{
		memset(read_buffer, 0x00, (FT5x0x_RX_NUM * 2));

		if(ft5x0x_write_reg(0x00, rownum)<0) //Leon
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		msleep(1);

		//msleep(1);
		retval = i2c_transfer(this_client->adapter, msgs, 1);
		if (retval < 0)
		{
			pr_err("%s ERROR:Could not read row %u raw data", __FUNCTION__, rownum);
			retval = -1;
			goto error_return;
		}
		for(i=0; i<FT5x0x_RX_NUM; i++)
		{
			g_RawData[rownum][i] = (read_buffer[i<<1]<<8) + read_buffer[(i<<1)+1];
		}
	}
error_return:
	if(ft5x0x_enter_work(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR:could not enter work mode ", __FUNCTION__);
		retval = -1;
	}
	return retval;
}
#endif

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i;

	printk("[FTS] start auto CLB.\n");
	msleep(200);
	ft5x0x_write_reg(0, 0x40);
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5x0x_write_reg(2, 0x4);  //write command to start calibration
	delay_qt_ms(300);
	for (i = 0; i < 100; i++) {
		ft5x0x_read_reg(0, &uc_temp);
		if (((uc_temp & 0x70) >> 4) == 0x0)  //return to normal mode, calibration finish
			{
			break;
		}
		delay_qt_ms(200);
		printk("[FTS] waiting calibration %d\n", i);

	}
	printk("[FTS] calibration OK.\n");

	msleep(300);
	ft5x0x_write_reg(0, 0x40);  //goto factory mode
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5x0x_write_reg(2, 0x5);  //store CLB result
	delay_qt_ms(300);
	ft5x0x_write_reg(0, 0x0); //return to normal mode
	msleep(300);
	printk("[FTS] store CLB result OK.\n");
	return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE* pbt_buf = FTS_NULL;
	int i_ret;

	//=========FW upgrade========================*/
	pbt_buf = CTPM_FW;
	/*call the upgrade function*/
	i_ret = fts_ctpm_fw_upgrade(pbt_buf, sizeof(CTPM_FW));
	if (i_ret != 0) {
		printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		//error handling ...
		//TBD
	} else {
		printk("[FTS] upgrade successfully.\n");
		fts_ctpm_auto_clb();  //start auto CLB
	}

	return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
	unsigned int ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2) {
		return CTPM_FW[ui_sz - 2];
	} else {
		//TBD, error handling?
		return 0xff; //default value
	}
}

#define    FTS_SETTING_BUF_LEN        128

//update project setting
//only update these settings for COB project, or for some special case
int fts_ctpm_update_project_setting(void)
{
	unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
	unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
	unsigned char uc_panel_factory_id;     //TP panel factory ID

	unsigned char buf[FTS_SETTING_BUF_LEN];
	FTS_BYTE reg_val[2] = { 0 };
	FTS_BYTE auc_i2c_write_buf[10];
	FTS_BYTE packet_buf[FTS_SETTING_BUF_LEN + 6];
	FTS_DWRD i = 0;
	int i_ret;

	uc_i2c_addr = 0x70;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;

	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	ft5x0x_write_reg(0xfc, 0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	ft5x0x_write_reg(0xfc, 0x55);
	printk("[FTS] Step 1: Reset CTPM test\n");

	delay_qt_ms(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
		delay_qt_ms(5);
	} while (i_ret <= 0 && i < 5);

	/*********Step 3:check READ-ID***********************/
	cmd_write(0x90, 0x00, 0x00, 0x00, 4);
	byte_read(reg_val, 2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3) {
		printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	} else {
		return ERR_READID;
	}

	cmd_write(0xcd, 0x0, 0x00, 0x00, 1);
	byte_read(reg_val, 1);
	printk("bootloader version = 0x%x\n", reg_val[0]);

	/* --------- read current project setting  ---------- */
	//set read start address
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;
	byte_write(buf, 4);
	byte_read(buf, FTS_SETTING_BUF_LEN);

	printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n", buf[0], buf[2],
		buf[4]);
	for (i = 0; i < FTS_SETTING_BUF_LEN; i++) {
		if (i % 16 == 0)
			printk("\n");
		printk("0x%x, ", buf[i]);

	}
	printk("\n");

	/*--------- Step 4:erase project setting --------------*/
	cmd_write(0x62, 0x00, 0x00, 0x00, 1);
	delay_qt_ms(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_SETTING_BUF_LEN;
	for (i = 0; i < FTS_SETTING_BUF_LEN; i++) {
		packet_buf[6 + i] = buf[i];
		if (i % 16 == 0)
			printk("\n");
		printk("0x%x, ", buf[i]);
	}
	printk("\n");
	byte_write(&packet_buf[0], FTS_SETTING_BUF_LEN + 6);
	delay_qt_ms(100);

	/********* reset the new FW***********************/
	cmd_write(0x07, 0x00, 0x00, 0x00, 1);

	msleep(200);

	return 0;

}



#if CFG_SUPPORT_AUTO_UPG

int fts_ctpm_auto_upg(void)
{
	unsigned char uc_host_fm_ver;
	unsigned char uc_tp_fm_ver;
	int i_ret;

	uc_tp_fm_ver = ft5x0x_read_fw_ver();
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	if ( uc_tp_fm_ver == 0xa6 ||   //the firmware in touch panel maybe corrupted
		uc_tp_fm_ver < uc_host_fm_ver//the firmware in host flash is new, need upgrade
	)
	{
		msleep(100);
		printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
			uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file();
		if (i_ret == 0)
		{
			msleep(300);
			uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
		}
		else
		{
			printk("[FTS] upgrade failed ret=%d.\n", i_ret);
		}
	}

	return 0;
}

#endif

#endif


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}


//read touch point information
static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i;

	ret = ft5x0x_i2c_rxdata(buf, CFG_POINT_READ_BUF);
	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	if (event->touch_point > CFG_MAX_TOUCH_POINTS) {
		event->touch_point = CFG_MAX_TOUCH_POINTS;
	}

	// Jonas, Read all data until no event existing
	//for (i = 0; i < event->touch_point; i++) {
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		event->au16_x[i] = (s16) (buf[3 + 6 * i] & 0x0F) << 8 | (s16) buf[4 + 6 * i];
		event->au16_y[i] = (s16) (buf[5 + 6 * i] & 0x0F) << 8 | (s16) buf[6 + 6 * i];
		event->au8_touch_event[i] = buf[0x3 + 6 * i] >> 6;
		if (event->au8_touch_event[i] == EVENT_NO_EVENT)
			break;
		event->au8_finger_id[i] = (buf[5 + 6 * i]) >> 4;
		event->au8_touchWeight[i] = buf[7 + 6 * i];
		event->au8_touchArea[i] = (buf[8 + 6 * i]) >> 4;
	}

	event->pressure = 200;

	return 0;
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
	int i;
	int key_id;

	if (x == TOUCH_KEY0_X && y == TOUCH_KEY0_Y) {
		key_id = 0;
	} else if (x == TOUCH_KEY1_X && y == TOUCH_KEY1_Y) {
		key_id = 1;
	} else if (x == TOUCH_KEY2_X && y == TOUCH_KEY2_Y) {
		key_id = 2;
	} else {
		key_id = 0xf;
	}
	for (i = 0; i < CFG_NUMOFKEYS; i++) {
		if (tsp_keystatus[i]) {
			// report release event
			input_report_key(dev, tsp_keycodes[i], 0);
			printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
			tsp_keystatus[i] = KEY_RELEASE;
		} else if (key_id == i) {
			if (touch_event == 0) {
				// detect
				input_report_key(dev, tsp_keycodes[i], 1);
				printk("[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
				tsp_keystatus[i] = KEY_PRESS;
			}
		}
	}
	return 0;

}
#endif

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	int i;

	// Jonas, Check every touch data until invalid item found
	//for (i = 0; i < event->touch_point; i++) {
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		if (event->au8_touch_event[i] == EVENT_NO_EVENT)
			break;
		if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y) {
			// Save previous X, Y for ftd use
			if (i == 0) {
				ftd_OldX = event->au16_x[i];
				ftd_OldY = event->au16_y[i];
			}
			// LCD view area
			input_report_key(data->input_dev, BTN_TOUCH, 1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 40);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);  // Use finger id for tracking

			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);

			/*
			 input_report_abs(g_tp->input, ABS_MT_POSITION_X, (g_tp->msg[i].coord.x*720)/1100);
			 input_report_abs(g_tp->input, ABS_MT_POSITION_Y, (g_tp->msg[i].coord.y*1280)/1900);
			 input_report_abs(g_tp->input, ABS_MT_TOUCH_MAJOR, g_tp->msg[i].coord.z);
			 input_report_abs(g_tp->input, ABS_MT_WIDTH_MAJOR, g_tp->msg[i].coord.wxy);
			 input_report_abs(g_tp->input, ABS_MT_TRACKING_ID, i);
			 input_report_abs(g_tp->input, ABS_MT_PRESSURE, g_tp->msg[i].coord.z);
			 */

			if (event->au8_touch_event[i] == EVENT_PUT_DOWN || event->au8_touch_event[i] == EVENT_CONTACT) {
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			} else {
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				//printk("[Leon] touch[%d]= X= %d, Y= %d ABS_MT_TOUCH_MAJOR\n",i ,event->au16_x[i], event->au16_y[i]);
			}
			//printk("[Leon] touch[%d]= X= %d, Y= %d, Event=%d, ID =%d, Weight=%x, Area=%x, Num=%d\n", i, event->au16_x[i],
			//			event->au16_y[i], event->au8_touch_event[i], event->au8_finger_id[i], event->au8_touchWeight[i],
			//			event->au8_touchArea[i], event->touch_point);
			DPRINTK(8, "touch[%d]= X= %d, Y= %d, Event=%d, ID =%d, Num=%d\n", i, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i], event->au8_finger_id[i], event->touch_point);
		} else {
			//maybe the touch key area
#if CFG_SUPPORT_TOUCH_KEY
			if (event->au16_y[i] >= SCREEN_MAX_Y) {
				ft5x0x_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i],
					event->au8_touch_event[i]);
			}
#endif
		}

		input_mt_sync(data->input_dev);
	}
	input_sync(data->input_dev);

	if (event->touch_point == 0) {
		ft5x0x_ts_release();
		return;
	}
} /*end ft5x0x_report_value*/

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	//printk("[Leon] ft5x0x_ts_pen_irq_work !!\n");
	ret = ft5x0x_read_data();
	if (ret == 0) {
		ft5x0x_report_value();
	}

	enable_irq(MSM_GPIO_TO_INT(11));
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	//disable_irq(IRQ_EINT(6));
	disable_irq_nosync(MSM_GPIO_TO_INT(11));
	//printk("[Leon]-------------------------INT______-\n");

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	//struct ft5x0x_ts_data *ts;
	//ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk("==ft5x0x_ts_suspend=\n");
	//disable_irq(this_client->irq);
	//disable_irq(IRQ_EINT(6));
	//cancel_work_sync(&ts->pen_event_work);
	//flush_workqueue(ts->ts_workqueue);
	// ==set mode ==, 
	//ft5x0x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	printk("==ft5x0x_ts_resume=\n");
	// wake the mode
	//__gpio_as_output(GPIO_FT5X0X_WAKE);
	//__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
	// msleep(100);
	//__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
	//msleep(100);
	//enable_irq(this_client->irq);
	//enable_irq(IRQ_EINT(6));
}
#endif  //CONFIG_HAS_EARLYSUSPEND

/* sysfs */

#if 0 //Leon
static u8 ft5x0x_enter_factory(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
	flush_workqueue(ft5x0x_ts->ts_workqueue);
	disable_irq_nosync(ft5x0x_ts->irq);
	ft5x0x_write_reg(0, 0x40);  //goto factory mode
	delay_qt_ms(100);//make sure already enter factory mode
	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else {
		if((regval & 0x70) != 0x40) {
			pr_err("%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	return 0;
}


static u8 ft5x0x_enter_work(struct ft5x0x_ts_data *ft5x0x_ts)
{
	u8 regval;
	ft5x0x_write_reg(0x00, 0x00); //return to normal mode
	msleep(100);

	if(ft5x0x_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else {
		if((regval & 0x70) != 0x00) {
			pr_err("%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	enable_irq(ft5x0x_ts->irq);
	return 0;
}
#endif 

static int ft5x0x_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if (NULL == pfile) {
		pfile = filp_open("/sdcard/Ft5x16_Lib_app.bin", O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL );
	return fsize;
}
static int ft5x0x_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if (NULL == pfile) {
		pfile = filp_open("/sdcard/Ft5x16_Lib_app.bin", O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	//char * buf;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL );
	set_fs(old_fs);
	return 0;
}

int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
	FTS_BYTE* pbt_buf = FTS_NULL;
	int i_ret;
	u8 fwver;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);
	if (fwsize <= 0) {
		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
	}
	//=========FW upgrade========================*/
	pbt_buf = (unsigned char *) kmalloc(fwsize + 1, GFP_ATOMIC );
	if (ft5x0x_ReadFirmware(firmware_name, pbt_buf)) {
		pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
		kfree(pbt_buf);
		return -1;
	}
	/*call the upgrade function*/
	i_ret = fts_ctpm_fw_upgrade(pbt_buf, fwsize);
	if (i_ret != 0) {
		pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n", __FUNCTION__, i_ret);
		//error handling ...
		//TBD
	} else {
		pr_info("[FTS] upgrade successfully.\n");
		if (ft5x0x_read_reg(FT5X0X_REG_FIRMID, &fwver) >= 0)
			pr_info("the new fw ver is 0x%02x\n", fwver);
		fts_ctpm_auto_clb();  //start auto CLB
	}
	kfree(pbt_buf);
	return i_ret;
}

static ssize_t ft5x0x_tpfwver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata(client);

	mutex_lock(&data->device_mode_mutex);
	if (ft5x0x_read_reg(FT5X0X_REG_FIRMID, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x0x_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x0x_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	unsigned long wmreg = 0;
	u8 regaddr = 0xff;
	u8 regvalue = 0xff;
	u8 valbuf[5];

	data = (struct ft5x0x_ts_data *) i2c_get_clientdata(client);

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&data->device_mode_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	//pr_info("valbuf=%s wmreg=%x\n", valbuf, wmreg);
	if (0 != retval) {
		pr_err("%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n",
			__FUNCTION__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		//read register
		regaddr = wmreg;
		if (ft5x0x_read_reg(regaddr, &regvalue) < 0)
			pr_err("Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (ft5x0x_write_reg(regaddr, regvalue) < 0)
			pr_err("Could not write the register(0x%02x)\n", regaddr);
		else
			pr_err("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}


static ssize_t ft5x0x_fwupdate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from *.i
static ssize_t ft5x0x_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 uc_host_fm_ver;
	int i_ret = fts_ctpm_fw_upgrade_with_i_file();
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	ssize_t num_read_chars = 0;

	mutex_lock(&data->device_mode_mutex);

	disable_irq(MSM_GPIO_TO_INT(11));

        if (i_ret == 0)
        {
            msleep(300);
            uc_host_fm_ver = fts_ctpm_get_i_file_ver();
            pr_info("%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
        }
        else
        {
            pr_err("%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
        }
	enable_irq(MSM_GPIO_TO_INT(11));

//error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_fwupgradeapp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from app.bin
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x0x_ts_data *data = NULL;
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
//	ssize_t num_read_chars = 0;

	memset(fwname, 0, sizeof(fwname));
	fwname[count - 1] = '\0';
	sprintf(fwname, "%s", buf);

	mutex_lock(&data->device_mode_mutex);
	disable_irq(MSM_GPIO_TO_INT(11));

	fts_ctpm_fw_upgrade_with_app_file(fwname);

	enable_irq(MSM_GPIO_TO_INT(11));

//error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}
#if 0 //Leon
static ssize_t ft5x0x_rawdata_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = NULL;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;


	int i=0, j=0;
	data = (struct ft5x0x_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);
	if(fts_Get_RawData()<0)
		sprintf(buf, "%s", "could not get rawdata\n");
	else
	{
		for(i=0; i<FT5x0x_TX_NUM; i++)
		{
			for(j=0; j<FT5x0x_RX_NUM; j++)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", g_RawData[i][j]);
			}
			buf[num_read_chars-1] = '\n';
		}
	}
//error_return:
	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

//upgrade from app.bin
static ssize_t ft5x0x_rawdata_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{

	return -EPERM;
}
#endif

/* sysfs */
//static DEVICE_ATTR(rawbase, S_IRUGO|S_IWUSR, ft5x0x_rawbase_show, ft5x0x_rawbase_store);
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ft5x0x_tpfwver_show, ft5x0x_tpfwver_store);
//upgrade from *.i
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ft5x0x_fwupdate_show, ft5x0x_fwupdate_store);
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, ft5x0x_tprwreg_show, ft5x0x_tprwreg_store);
//upgrade from app.bin 
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ft5x0x_fwupgradeapp_show, ft5x0x_fwupgradeapp_store);
//static DEVICE_ATTR(ftsrawdatashow, S_IRUGO|S_IWUSR, ft5x0x_rawdata_show, ft5x0x_rawdata_store);


static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	//ev_attr_ftsrawdatashow.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};
//Leon add
/* release gpios */
static int touchpad_release_gpio(struct ft5x0x_ts_data *g_tp)
{
    int rc = 0;

	printk("releasing gpio!!!\n");
	gpio_free(g_tp->gpio_irq);
	gpio_free(g_tp->gpio_rst);
	return rc;
}

/* HW reset for Synaptics s3202 */
static int touchpad_config_gpio(struct ft5x0x_ts_data *g_tp)
{
	int rc = 0;

    /* set hw reset low */
	//gpio_set_value(g_tp->gpio_rst, 0);
	//printk("set hw reset low, delay 1ms\n");
	//msleep(1);
	/* set hw reset high */
    gpio_set_value(g_tp->gpio_rst, 1);
    printk("Tset hw reset high, delay 100ms\n");
	msleep(1);
    return rc;
}

static int touchpad_setup_gpio(struct ft5x0x_ts_data *g_tp)
{
	int rc = 0;

	printk("Set up GPIO!!!! \n");
	/* set up IRQ gpio */
	rc = gpio_request(g_tp->gpio_irq, "synaptics_ts_irq");
	if (rc) {
		printk("failed to request gpio_irq:%d (rc=%d)\n", g_tp->gpio_irq, rc);
		goto err_gpio_config;
	}

	rc = gpio_direction_input(g_tp->gpio_irq);
	if (rc) {
		printk("failed to set gpio_irq:%d mode (rc=%d)\n", g_tp->gpio_irq, rc);
		goto err_gpio_config;
	}
	/* print configure gpio num for debugging */
	printk("success to configure gpio_irq:%d gpio_name:%s\n", g_tp->gpio_irq, "synaptics_ts_irq");

	/* set up RST gpio */
	rc = gpio_request(g_tp->gpio_rst, "synaptics_ts_rst");
	if (rc) {
		printk("failed to request gpio_rst:%d (rc=%d)\n", g_tp->gpio_rst, rc);
		goto err_gpio_config;
	}

	rc = gpio_direction_output(g_tp->gpio_rst, 0);
	if (rc) {
		printk("failed to set gpio_rst:%d mode (rc=%d)\n", g_tp->gpio_rst, rc);
		goto err_gpio_config;
	}
	/* print configure gpio num for debugging */
	printk("success to configure gpio_rst:%d gpio_name:%s\n", g_tp->gpio_rst, "synaptics_ts_rst");

	return 0;

err_gpio_config:
	touchpad_release_gpio(g_tp);
	return rc;
}



static int touchpad_power_on_device(struct ft5x0x_ts_data *g_tp, int on)
{
	int rc = 0;
	static int prev_on = 0;

	if (on == prev_on) {
		return 0;
	}

	if (on) {

		/* L9 voltage for avdd touch */
		g_tp->avdd_regulator = regulator_get(NULL, "avdd_touch");
		if (IS_ERR(g_tp->avdd_regulator)) {
			pr_err("could not get avdd_touch\n");
			rc = -ENODEV;
			goto exit;
		}

		/* [DSPS] L14 voltage */
		g_tp->ldo9_regulator = regulator_get(NULL, "dsps_vdd");
		if (IS_ERR(g_tp->ldo9_regulator)) {
			rc = PTR_ERR(g_tp->ldo9_regulator);
			printk("regulator get ldo9 voltage failed\n");
			rc = -ENODEV;
			goto get_ldo9_fail;
		}

		//LPS4 voltage for i2c bus
		g_tp->i2c_regulator = regulator_get(NULL, "touch_i2c");
		if (IS_ERR(g_tp->i2c_regulator)) {
			rc = PTR_ERR(g_tp->i2c_regulator);
			printk("regulator get i2c voltage failed \n");
			//pr_err("could not get i2c bus votage, rc = %ld\n",
			//PTR_ERR(g_tp->vdd_regulator));
			rc = -ENODEV;
			goto get_i2c_fail;
		}

		rc = regulator_set_voltage(g_tp->avdd_regulator, 2850000, 2850000);
		if (rc) {
			printk("set avdd_touch failed \n");
			goto set_avdd_fail;
		}

		/* [DSPS] L9 voltage is 2.85V *//*because low temperatures (-30C) the voltage for L9 drops),
		 So, follow QC changed, setting 3.0V for 2.85 requirement */
		rc = regulator_set_voltage(g_tp->ldo9_regulator, 1800000, 1800000);
		if (rc) {
			printk("set avdd_touch failed \n");
			goto set_avdd_fail;
		}

		rc = regulator_enable(g_tp->avdd_regulator);
		if (rc) {
			printk("enable avdd_touch failed \n");
			goto set_avdd_fail;
		}
		msleep(5);

		/* [DSPS] L9 enable */
		rc = regulator_enable(g_tp->ldo9_regulator);
		if (rc) {
			printk("enable ldo9 failed\n");
			goto set_ldo9_fail;
		}
		msleep(5);

		rc = regulator_enable(g_tp->i2c_regulator);
		if (rc) {
			printk("enable i2c failed\n");
			goto set_i2c_fail;
		}
		msleep(5);

		msleep(100);
		printk("avdd_touch have been turn on\n");
/* TODO */
#if 0		
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(g_tp->pmlog_device);
		if (rc)
			printk( "pmlog_device_on fail \n");
#endif	//CONFIG_PM_LOG
#endif
		prev_on = on;
		return 0;	/* Successful, do return here. */
	} else {
		printk( "does not support turn onoff vdd_touch\n");
/* TODO */
#if 0		
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_off(g_tp->pmlog_device);
		if (rc)
			printk("pmlog_device_off fail\n");
#endif //CONFIG_PM_LOG
#endif
		prev_on = on;
	}

	/* Normal off sequence, also used if any errors */
	regulator_disable(g_tp->i2c_regulator);
set_i2c_fail:
	regulator_disable(g_tp->ldo9_regulator);
set_ldo9_fail:
	regulator_disable(g_tp->avdd_regulator);
set_avdd_fail:
	regulator_put(g_tp->i2c_regulator);
get_i2c_fail:
	regulator_put(g_tp->ldo9_regulator);
get_ldo9_fail:
	regulator_put(g_tp->avdd_regulator);
exit:
	return rc;
}

// Jonas, 20121011, FTD test related {
static int get_rawData(void)
{
	unsigned char mode = 0;
	int i, j;
	char *rxBuf = 0;

	//1) Switch to factory mode
	msleep(200);
	ft5x0x_write_reg(0, 0x40);
	delay_qt_ms(100);   //make sure already enter factory mode
	for (i = 0; i < 10; i++) {
		ft5x0x_read_reg(0, &mode);
		if (mode == 0x40)
			break;
		msleep(10);
	}
	if (mode != 0x40) {
		DPRINTK(1, "Cannot enter factory mode!\n");
		ft5x0x_write_reg(0, 0x0);
		return -1;
	}

	//2) Get TX and RX number
	ft5x0x_read_reg(3, &tx_num);
	ft5x0x_read_reg(4, &rx_num);

	// allocate memory for raw data
	if (g_RawData == 0) {
		DPRINTK(4, "Allocate memory for raw data!\n");
		g_RawData = kzalloc(tx_num * rx_num * 2, GFP_KERNEL);
		if (!g_RawData) {
			DPRINTK(1, "Cannot allocate memory for raw data!\n");
			return -1;
		}
	}
	rxBuf = kzalloc(rx_num * 2, GFP_KERNEL);
	if (!rxBuf) {
		DPRINTK(1, "Cannot allocate memory for reading raw data by rx!\n");
		return -1;
	}

	//3) Get Each Raw data according to TX
	for (i = 0; i < tx_num; i++) {
		ft5x0x_write_reg(1, i);
		msleep(5);
		ft5x0x_read_block(0x10, rx_num * 2, rxBuf);
		for (j = 0; j < rx_num; j++) {
			g_RawData[rx_num * i + j] = (rxBuf[j * 2] << 8) + rxBuf[j * 2 + 1];
		}
	}

	//4) Switch to normal mode
	ft5x0x_write_reg(0, 0x0);
	kfree(rxBuf);
	return 0;
}

#define TOUCH_DRIVER_NAME	"ft5x0x_dbg"
//DEFINE_SIMPLE_ATTRIBUTE(tp_onoff_fops, dbg_tp_onoff_get, dbg_tp_onoff_set, "%llu\n");

/* *************************************************************************
 * add debugfs for ftd test to get device id
 * ************************************************************************* */
static int tp_read_touch_point(void *data, u64 * val)
{
	unsigned long pointValue = 0;
	DPRINTK(4, "tp_read_touch_point called\n");
	DPRINTK(4, "Touch Point X: %d, Y: %d\n", ftd_OldX, ftd_OldY);

	pointValue = (ftd_OldX << 16) + ftd_OldY;

	* val = pointValue;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_read_point, tp_read_touch_point, NULL, "%lld\n");

static int tp_read_device_id(void *data, u64 * val)
{
	unsigned char uc_reg_value = 0;
	DPRINTK(8, "tp_read_device_id called\n");

	ft5x0x_read_reg(FT5X0X_REG_CIPHER, &uc_reg_value);
	DPRINTK(8, "Chip vendor ID: %d\n", uc_reg_value);
	* val = uc_reg_value;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_read_id, tp_read_device_id, NULL, "%lld\n");

static int tp_read_device_version(void *data, u64 * val)
{
	* val = ft5x0x_read_fw_ver();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_read_version, tp_read_device_version, NULL, "%lld\n");

static int tp_get_debug_level(void *data, u64 * val)
{
	DPRINTK(8, "%s called\n", __FUNCTION__);

	*val = TP_DBG_LEVEL;
	return 0;
}
static int tp_set_debug_level(void *data, u64 val)
{
	DPRINTK(8, "%s called, data:%s, value:%d\n", __FUNCTION__, (char*)data, (int)val);

	TP_DBG_LEVEL = (int)val;
	if (TP_DBG_LEVEL > 10)
		TP_DBG_LEVEL = 10;
	else if (TP_DBG_LEVEL < 0)
		TP_DBG_LEVEL = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_debug_level, tp_get_debug_level, tp_set_debug_level, "%lld\n");

static int tp_do_calibration(void *data, u64 val)
{
	DPRINTK(8, "%s called\n", __FUNCTION__);
	return fts_ctpm_auto_clb();
}
DEFINE_SIMPLE_ATTRIBUTE(tp_calibration, NULL, tp_do_calibration, "%lld\n");

#define RAW_BUF_LEN	2000
int tp_get_raw_data(struct file *fp, char __user *user_buffer, size_t count, loff_t *position)
{
	char oneLine[100];
	char *outputBuf = 0;
	int result = 0, len, dataSize = 0;
	int i, j;
	if (*position != 0)
		return 0;
	DPRINTK(8, "%s called\n", __FUNCTION__);
	//DPRINTK(8, "data length %d, count: %d\n", sizeof(outputBuf), (int)count);

	outputBuf = kzalloc(RAW_BUF_LEN, GFP_KERNEL);
	if (!outputBuf) {
		DPRINTK(1, "cannot allocate memory\n");
		return -1;
	}
	memset(oneLine, 0, 100);
	if (get_rawData() != 0) {
		DPRINTK(1, "Get raw data fail\n");
		return -1;
	}
	len = sprintf(oneLine, "Tx num: %d, Rx num: %d \n", tx_num, rx_num);
	dataSize += len;
	strncat(outputBuf, oneLine, len);
	DPRINTK(4, "%s", oneLine);
	for (i = 0; i < tx_num; i++) {
		len = sprintf(oneLine, "Col(%2d): ", i);
		dataSize += len;
		strncat(outputBuf, oneLine, len);
		DPRINTK_NP(4, "%s", oneLine);
		for (j = 0; j < rx_num; j++) {
			len = sprintf(oneLine, "%4d ", g_RawData[i * rx_num + j]);
			dataSize += len;
			strncat(outputBuf, oneLine, len);
			DPRINTK_NP(4, "%s", oneLine);
		}
		len = sprintf(oneLine, "\n");
		dataSize += len;
		strncat(outputBuf, oneLine, len);
		DPRINTK_NP(4, "%s", oneLine);
	}
	result = simple_read_from_buffer(user_buffer, count, position, outputBuf, dataSize);

	DPRINTK(4, "result: %d, Count: %d, dataSize:%d\n", result, (int)count, dataSize);
	return result;
}
static const struct file_operations tp_raw_data = {
	.read = tp_get_raw_data,
};

static void touchpad_create_debugfs_entries(struct ft5x0x_ts_data *g_tp)
{
	g_tp->dent = debugfs_create_dir(TOUCH_DRIVER_NAME, NULL);
	if (g_tp->dent) {
		debugfs_create_file("touch_point", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_read_point);
		debugfs_create_file("device_id", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_read_id);
		debugfs_create_file("device_version", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_read_version);
		debugfs_create_file("debug_level", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_debug_level);
		debugfs_create_file("calibration", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_calibration);
		debugfs_create_file("raw_data", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_raw_data);
		//debugfs_create_file("tp_onoff", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_onoff_fops);
	}
}

// } Jonas, 20121011, FTD test related

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	int result = 0;
//Leon
	char* path;

#if CFG_SUPPORT_TOUCH_KEY
	int i;
#endif

	printk("[FTS] ft5x0x_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);
	printk("[FTS] Touch resolution X:%d, Y:%d \n", SCREEN_MAX_X, SCREEN_MAX_Y);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);
	//ft5x0x_ts = kmalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);
	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	//memset(ft5x0x_ts, 0, sizeof(struct ft5x0x_ts_data));

	this_client = client;
	i2c_set_clientdata(client, ft5x0x_ts);
	ft5x0x_ts->gpio_irq = 11;
	ft5x0x_ts->gpio_rst = 52;
	ft5x0x_ts->irq = MSM_GPIO_TO_INT(11);
	mutex_init(&ft5x0x_ts->device_mode_mutex);
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	/* enable vdd & avdd voltage */
//Leon add	
	/* set up synaptics irq */
	result = touchpad_setup_gpio(ft5x0x_ts);
	if (result) {
		printk("Failed to setup gpio fail !!\n");
		goto exit_create_singlethread;
	}

	printk("set hw reset low, delay 150ms\n");
	msleep(150);
	result = touchpad_power_on_device(ft5x0x_ts, 1);
	if (result) {
		//SYNAPTICS_PRINTK(0, "Unable to power device result=%d\n", result);
		printk("[FTS] ft5x0x_ts_probe power on fail !!\n");
		goto exit_create_singlethread;
	}

	msleep(15);

	/* configure up cyttsp rst gpio */
	result = touchpad_config_gpio(ft5x0x_ts);
	if (result) {
		printk("Failed to config reset GPIO !!\n");
		goto exit_create_singlethread;
	}

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	err = request_irq(ft5x0x_ts->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ft5x0x_ts", ft5x0x_ts);
	//err = request_threaded_irq(ft5x0x_ts->gpio_irq, NULL,
	//			   ft5x0x_ts_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	//			   "ft5x0x_ts", ft5x0x_ts);	
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(ft5x0x_ts->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;
//Leon add
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);

	/*
	 set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	 set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	 set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	 set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);*/

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
//Leon
	//set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit); //fix the input issue JB only

#if CFG_SUPPORT_TOUCH_KEY
    //setup key code area
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_dev->keycode = tsp_keycodes;
	for (i = 0; i < CFG_NUMOFKEYS; i++) {
		input_set_capability(input_dev, EV_KEY, ((int*) input_dev->keycode)[i]);
		tsp_keystatus[i] = KEY_RELEASE;
	}
#endif

	input_dev->name		= FOCALTECH_TP_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	msleep(150);  //make sure CTP already finish startup process

	//get some register information
	uc_reg_value = ft5x0x_read_fw_ver();
	printk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	ft5x0x_read_reg(FT5X0X_REG_PERIODACTIVE, &uc_reg_value);
	printk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
	ft5x0x_read_reg(FT5X0X_REG_THGROUP, &uc_reg_value);
	printk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

#if CFG_SUPPORT_AUTO_UPG
    fts_ctpm_auto_upg();
#endif    

#if CFG_SUPPORT_UPDATE_PROJECT_SETTING
    fts_ctpm_update_project_setting();
#endif

	enable_irq(ft5x0x_ts->irq);
	//create sysfs

	//client_data->input->dev.kobj = *kobject_create_and_add("don", NULL);
	path = kobject_get_path(&client->dev.kobj, GFP_KERNEL); ///path=/sys/devices/i2c-3/3-0038
	printk("[Leon]%s, path=%s\n", __func__, path);

	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_attribute_group);
	if (0 != err) {
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ft5x0x_attribute_group);
	} else {
		printk("ft5x0x:%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
	}
	/*Jonas, 20120911, add debugfs for ftd test to get device id */
	touchpad_create_debugfs_entries(ft5x0x_ts);

	printk("[FTS] ==probe over =\n");
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	//free_irq(client->irq, ft5x0x_ts);
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
exit_irq_request_failed:
	//exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	printk("==ft5x0x_ts_remove=\n");
	if(g_RawData)
		kfree(g_RawData);
	ft5x0x_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	//free_irq(client->irq, ft5x0x_ts);
	mutex_destroy(&ft5x0x_ts->device_mode_mutex);
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL );
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FOCALTECH_TP_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);


static struct i2c_driver ft5x0x_ts_driver = {
	.driver	= {
		.name	= FOCALTECH_TP_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,

};


/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
	int ret;
	printk("==ft5x0x_ts_init==\n");
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	printk("ret=%d\n",ret);
	return ret;
//	return i2c_add_driver(&ft5x0x_ts_driver);
}

/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
	printk("==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
