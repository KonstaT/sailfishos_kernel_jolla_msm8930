/* 
 * drivers/input/touchscreen/focaltech_ft5316_ts.c
 * Driver for the FocalTech ft5316 Touch Panel chip.
 *
 * Copyright (c) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 *
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
 */

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
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <mach/vreg.h>
#include <linux/input/focaltech_ft5316_ts.h>
#include <linux/debugfs.h>
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif //CONFIG_PM_LOG
#include <mach/hwid.h>
#include <linux/proc_fs.h>

//DEBUG_LEVEL
extern struct dentry *kernel_debuglevel_dir;
static struct proc_dir_entry *proc_entry;
static struct proc_dir_entry *proc1_entry;
static struct proc_dir_entry *proc2_entry;
static struct proc_dir_entry *proc3_entry;
static struct proc_dir_entry *proc4_entry;
static unsigned int TOUCH_DLL=0;
static unsigned int TOUCH_EVENT_DLL=0;
static unsigned int KEY_EVENT_DLL=0;
#define TOUCH_ERR_LEVEL   0
#define TOUCH_EVENT_LEVEL 1
#define KEY_EVENT_LEVEL 1
#define TOUCH_PRINTK(level, args...) if(level <= TOUCH_DLL) printk( "[TP] " args);
#define TOUCH_EVENT_PRINTK(level, args...) if(level <= TOUCH_EVENT_DLL) printk("[TP] " args);
#define KEY_PRINTK(level, args...) if(level <= KEY_EVENT_DLL) printk("[KEY] " args);

static struct i2c_driver i2c_touchpad_driver;

struct ft5631_ts_info {
	struct i2c_client	*client;
    struct input_dev	*input;
	struct input_dev	*keyarray_input;
	struct ft5316_ts_event_t event; //report touch point strcut
	int	gpio_irq; //the GPIO number of interrupt
    int	gpio_rst; //the GPIO number of reset
	int	gpio_vendor_id; //the GPIO number of vendor id
	int	irq; //ISR number
	uint8_t vendor_id_val; //vendor id
	uint8_t fw_id_val; //fw version
	/*     ft5316 Power Rails      */
	struct regulator *ldo9_regulator; //VDD (VREG_L9_2P85)
	struct regulator *lvs2_regulator; //VLOGIC (VREG_LVS2_1P8)
	struct mutex	mutex;
	int	open_count; //count open times for touch input device
	int keyarray_open_count; //count open times for keyarray input device
	struct ft5316_touch_point_status_t msg[MAX_TS_REPORT_POINTS];
	//struct ft5316_key_point_status_t key_msg;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ts_early_suspend;
#endif /* CONFIG_HAS_EARLYSUSPEND */
	int is_suspended;
	/* add debugfs for csd and ftd testing */
	struct dentry   *dent;
	uint8_t rx_num;
	uint8_t tx_num;
	int pass_flag; //check if self-test pass or fail
	int back_key_x;
	int home_key_x;
	int menu_key_x;
	int keys_Y;
#ifdef CONFIG_PM_LOG
	struct pmlog_device *pmlog_device;
#endif //CONFIG_PM_LOG
	u16 *g_RawData;
	u16 *g_BaseLine;
	u16 *g_differ;
	struct ft5316_ts_rx_raw_data_t rx_rx;
};	

/*init synaptics IC first, if detect pass then don't init focaltech IC, if detect failed then init focaltech IC  */
extern int touch_panel_detected; //dtect touch IC flag
static void ft5316_ts_report_mt_protocol(struct ft5631_ts_info *ts);
struct  ft5631_ts_info *g_ts;
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5316_ts_early_suspend(struct early_suspend *h);
static void ft5316_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */
static int ft5316_enter_factory(void);
static int ft5316_start_scan(void);
static int tp_get_rawData(void);
static void ft5316_report_capkey(struct ft5631_ts_info *ts);

#define TOUCH_RETRY_COUNT 5
static int ft5316_write_i2c_data(struct i2c_client *client,
                        uint8_t           regBuf,
                        uint8_t           *dataBuf,
                        uint8_t           dataLen)
{
    int     result = 0;
    uint8_t *buf = NULL;
    int     retryCnt = TOUCH_RETRY_COUNT;
	
    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)buf,
            .len    = 0
        }
    };

    buf = kzalloc(dataLen+sizeof(regBuf), GFP_KERNEL);
    if( NULL == buf )
    {
        TOUCH_PRINTK(0, "alloc memory failed\n");
        return -EFAULT;
    }

    buf[0] = regBuf;
    memcpy(&buf[1], dataBuf, dataLen);
    msgs[0].buf = buf;
    msgs[0].len = dataLen+1;

    while(retryCnt)
    {
        result = i2c_transfer(client->adapter, msgs, 1);
        if(result != ARRAY_SIZE(msgs))
        {
            TOUCH_PRINTK(0, "write %Xh %d bytes return failure, %d\n", buf[0], dataLen, result);
            if(-ETIMEDOUT == result) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        TOUCH_PRINTK(0, "write %Xh %d bytes retry at %d\n", buf[0], dataLen, TOUCH_RETRY_COUNT-retryCnt);

    kfree( buf );
    return result;
}

static int ft5316_read_i2c_data(struct i2c_client *client,
                        uint8_t           regBuf,
                        uint8_t           *dataBuf,
                        uint8_t           dataLen)
{
    int     result = 0;
    int     retryCnt = TOUCH_RETRY_COUNT;

    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)&regBuf,
            .len    = 1
        },
        [1] = {
            .addr   = client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)dataBuf,
            .len    = dataLen
        }
    };

    while(retryCnt)
    {
        result = i2c_transfer(client->adapter, msgs, 2);
        if( result != ARRAY_SIZE(msgs) )
        {
            TOUCH_PRINTK(0, "read %Xh %d bytes return failure, %d\n", regBuf, dataLen, result );
            //return result;
            if( -ETIMEDOUT == result ) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        TOUCH_PRINTK(0, "read %Xh %d bytes retry at %d\n", regBuf, dataLen, TOUCH_RETRY_COUNT-retryCnt);

    return result;
}

static void delay_qt_ms(unsigned long w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
}
#if 0
static int read_raw_data(void)
{
	int rc = 0;
	uint8_t row_addr = 0x0;
	//uint8_t get_data[g_ts->tx_num*g_ts->rx_num*2];
	//uint16_t raw_data[g_ts->tx_num][g_ts->rx_num];
	uint8_t get_data[g_ts->rx_num*2];
	uint16_t raw_data[g_ts->tx_num*g_ts->rx_num*2];
	uint8_t i;
	int raw_data_size= g_ts->rx_num*2;
	int j = 0;
	//int k = 0;
	
	//read raw data
	msleep(5);
	for (i = 0 ; i < g_ts->tx_num; i++) {
		row_addr = i;
		//write tx_num to raw_addr;
		rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR+1, &row_addr, 1);
		if (rc) {
			TOUCH_PRINTK(0, "failed to read row_addr[0x%x]=0x%x (rc=%d)\n",
				START_REG_MAP_ADDR+1, row_addr, rc);
			rc = -EFAULT;
			return rc;
		}
		msleep(5);
		rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR+10, &get_data[j], raw_data_size);
		if (rc) {
			TOUCH_PRINTK(0, "failed to read get_data[0x%x]=0x%x (rc=%d)\n",
				START_REG_MAP_ADDR+10, get_data[0], rc);
			rc = -EFAULT;
			return rc;
		}
		for (j = 0; j < g_ts->rx_num; j++) {
			raw_data[g_ts->rx_num * i + j] = (get_data[j*2] << 8) + get_data[j * 2 + 1];
		}
	}
	#if 0
	for (i = 0; i < g_ts->tx_num; i++)
	{
		//TOUCH_PRINTK(0,"raw [%d]: ", i);
		for (j = 0 ; j < g_ts->rx_num; j++)
		{
			raw_data[i][j] = get_data[k] << 8 | get_data[k+1];
			//TOUCH_PRINTK(0, "get_data[%d]=0x%x\n", k, get_data[k]);
			//TOUCH_PRINTK(0, "get_data[%d]=0x%x\n", k+1, get_data[k+1]);
			//TOUCH_PRINTK(0, "raw_data[%d][%d]=0x%x\n", i, j, raw_data[i][j]);
			//TOUCH_PRINTK(0, "raw_data[%d][%d]=%d\n", i, j, (int)raw_data[i][j]);
			//printk("  %4d  ", (int)raw_data[i][j]);
			k = k + 2;	
		}
		printk("\n");
	}
	#endif
	/* print raw capacitance data */
	for (i = 0; i < g_ts->tx_num; i++)
	{
		//TOUCH_PRINTK(0,"raw [%d]: ", i);
		TOUCH_PRINTK(0,"Col(%2d): ", i);
		for (j = 0 ; j < g_ts->rx_num; j++)
		{
			printk("%5d", (int)raw_data[g_ts->rx_num * i+j]);
			
		}	
		printk("\n");
	}
#endif

static int ft5316_i2c_txdata(char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= g_ts->client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(g_ts->client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5316_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5316_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static int ft5316_read_block(u8 addr, u8 len, u8 *buf)
{
	int ret;
	//u8 buf[2];
	struct i2c_msg msgs[2];

	buf[0] = addr;    //register address

	msgs[0].addr = g_ts->client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = g_ts->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	ret = i2c_transfer(g_ts->client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int tp_check_rx_rx_raw_data(int aveg,int max,int min)
{
	int rc = 0;
	int i;
	int aveg_val[g_ts->rx_num];
	int max_val[g_ts->rx_num];
	int min_val[g_ts->rx_num];
	
	//compare aveg
	for (i = 0 ; i < g_ts->rx_num; i++) {
		aveg_val[i] = aveg - g_ts->rx_rx.aveg[i];
		if (aveg_val[i] > RX_RX_AVEG) {
			TOUCH_PRINTK(0, "RX_aveg[%d] = %d (fail)\n",i , aveg_val[i]);
		} else {
			TOUCH_PRINTK(0, "RX_aveg[%d] = %d\n",i , aveg_val[i]);
		}	
	}

	//comapre max
	for (i = 0 ; i < g_ts->rx_num; i++) {
		max_val[i] = max - g_ts->rx_rx.max[i];
		if (max_val[i] > RX_RX_MAX) {
			TOUCH_PRINTK(0, "RX_max[%d] = %d (fail)\n",i , max_val[i]);
		} else {
			TOUCH_PRINTK(0, "RX_max[%d] = %d\n",i , max_val[i]);
		}	
	}
	
	//comapre min
	for (i = 0 ; i < g_ts->rx_num; i++) {
		min_val[i] = min - g_ts->rx_rx.min[i];
		if (min_val[i] > RX_RX_MIN) {
			TOUCH_PRINTK(0, "RX_min[%d] = %d (fail)\n",i , min_val[i]);
		} else {
			TOUCH_PRINTK(0, "RX_min[%d] = %d\n",i , min_val[i]);
		}	
	}
	return rc;
}

static int tp_rx_rx_raw_data_min(void)
{
	
	int rc = 0;
	int i,j = 0;
	int min = 0;
	
	do {
		for (i = 0; i < g_ts->tx_num -1; i++) {
			if(i == 0) {
				min = g_ts->g_BaseLine[i * g_ts->rx_num + j];
				TOUCH_PRINTK(1, " RX_RX[%d]= %d, min=%d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j], min);
			} else {
				if (min > g_ts->g_BaseLine[i * g_ts->rx_num + j])
				min = g_ts->g_BaseLine[i * g_ts->rx_num + j];
				TOUCH_PRINTK(1, " g_ts->g_BaseLine[%d] = %d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j]);
				TOUCH_PRINTK(1, " min raw data value = %d\n", min);	
			}
		}
		g_ts->rx_rx.min[j] = min;
		TOUCH_PRINTK(1, "rx_rx->min[%d]=%d min= %d\n",j ,g_ts->rx_rx.min[j], min);
		j++;
	} while (j < g_ts->rx_num);
	
	for (i = 0 ; i < g_ts->rx_num; i++)
		TOUCH_PRINTK(0, "rx_rx->min[%d] = %d\n",i , g_ts->rx_rx.min[i]);
	return rc;
}

static int tp_raw_data_min(void)
{
	int i,j;
	int min = 0;
	
	for (i = 0; i < g_ts->tx_num -1; i++) {
		for (j = 0; j < g_ts->rx_num; j++) {
			if(i == 0 && j == 0) {
				min = g_ts->g_BaseLine[i * g_ts->rx_num + j];
			} else {
				if (min > g_ts->g_BaseLine[i * g_ts->rx_num + j])
				min = g_ts->g_BaseLine[i * g_ts->rx_num + j];
				TOUCH_PRINTK(1, " g_ts->g_BaseLine[%d] = %d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j]);
				TOUCH_PRINTK(1, " min raw data value = %d\n", min);	
			}	
		}
	}	
	
	TOUCH_PRINTK(1," min raw data value = %d\n", min);
	
	return min;
}


static int tp_rx_rx_raw_data_max(void)
{
	int rc = 0;
	int i,j = 0;
	int max = 0;
	
	do {
		for (i = 0; i < g_ts->tx_num -1; i++) {
			if(i == 0) {
				max = g_ts->g_BaseLine[i * g_ts->rx_num + j];
				TOUCH_PRINTK(1, " RX_RX[%d]= %d, max=%d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j], max);
			} else {
				if (max < g_ts->g_BaseLine[i * g_ts->rx_num + j])
				max = g_ts->g_BaseLine[i * g_ts->rx_num + j];
				TOUCH_PRINTK(1, " g_ts->g_BaseLine[%d] = %d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j]);
				TOUCH_PRINTK(1, " max raw data value = %d\n", max);	
			}
		}
		g_ts->rx_rx.max[j] = max;
		TOUCH_PRINTK(1, "rx_rx->max[%d]=%d max= %d\n",j ,g_ts->rx_rx.max[j], max);
		j++;
	} while (j < g_ts->rx_num);
	
	for (i = 0 ; i < g_ts->rx_num; i++)
		TOUCH_PRINTK(0, "rx_rx->max[%d] = %d\n",i , g_ts->rx_rx.max[i]);
		
	return rc;
}

static int tp_raw_data_max(void)
{
	int i,j;
	int max = 0;
	
	for (i = 0; i < g_ts->tx_num -1; i++) {
		for (j = 0; j < g_ts->rx_num; j++) {
			if(i == 0 && j == 0) {
				max = g_ts->g_BaseLine[i * g_ts->rx_num + j];
			} else {
				if (max < g_ts->g_BaseLine[i * g_ts->rx_num + j])
				max = g_ts->g_BaseLine[i * g_ts->rx_num + j];
				TOUCH_PRINTK(1, " g_ts->g_BaseLine[%d] = %d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j]);
				TOUCH_PRINTK(1, " max raw data value = %d\n", max);	
			}	
		}
	}	
	
	TOUCH_PRINTK(1," max raw data value = %d\n", max);
	
	return max;
}

static int tp_rx_rx_raw_data_aveg(int rx_size)
{
	int ret = 0;
	int i,j = 0;
	int raw_data_sum = 0;
	
	//touch area
	do {
		for (i = 0; i < g_ts->tx_num -1 ; i++) {
			raw_data_sum += g_ts->g_BaseLine[i * g_ts->rx_num + j];
			TOUCH_PRINTK(1, "raw data[%d] = %d sum = %d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j], raw_data_sum);
		}
		g_ts->rx_rx.aveg[j] = raw_data_sum / rx_size;
		TOUCH_PRINTK(1, "rx_rx->aveg[%d] %d = %d / %d\n",j , g_ts->rx_rx.aveg[j], raw_data_sum, rx_size);
		raw_data_sum = 0;
		j++;
	} while (j < g_ts->rx_num);
	
	for (i = 0 ; i < g_ts->rx_num; i++)
		TOUCH_PRINTK(0, "rx_rx->aveg[%d] = %d\n",i , g_ts->rx_rx.aveg[i]);
	
	return ret;
}

static int tp_raw_data_aveg(int data_size)
{
	int i,j;
	int raw_data_sum = 0, raw_data_aveg;
	
	//touch area
	for (i = 0; i < g_ts->tx_num -1 ; i++) {
		for (j = 0; j < g_ts->rx_num; j++) {
			raw_data_sum += g_ts->g_BaseLine[i * g_ts->rx_num + j];
			TOUCH_PRINTK(1, "raw data[%d] = %d sum = %d\n", i * g_ts->rx_num + j, g_ts->g_BaseLine[i * g_ts->rx_num + j], raw_data_sum);
		}
	}	
	
	raw_data_aveg = raw_data_sum / data_size;
	TOUCH_PRINTK(1," raw_data_aveg %d = %d/%d\n", raw_data_aveg, raw_data_sum, data_size);
	
	return raw_data_aveg;
}

//get RX_RX data
static int tp_get_RX_RX_data_analysis(void)
{
	int rc = 0;
	int aveg_raw_data = 0;
	int raw_data_size = (g_ts->tx_num - 1) * g_ts->rx_num;
	int rx_rx_size = (g_ts->tx_num - 1);
	int max_raw_val = 0;
	int min_raw_val = 0;
	
	//raw data average caculation
	aveg_raw_data = tp_raw_data_aveg(raw_data_size);
	TOUCH_PRINTK(0, " aveg_raw_data = %d\n", aveg_raw_data);
	
	//Max raw data
	max_raw_val = tp_raw_data_max();
	TOUCH_PRINTK(0, " max raw data value = %d\n", max_raw_val);
	
	//Min raw data
	min_raw_val = tp_raw_data_min();
	TOUCH_PRINTK(0, " min raw data value = %d\n", min_raw_val);
	
	//Rx_Rx raw data average caculation
	rc = tp_rx_rx_raw_data_aveg(rx_rx_size);
	if (rc) {
			TOUCH_PRINTK(0, "Failed to caculate rx_rx raw data aveg (rc=%d)\n",
				rc);
		rc = -EFAULT;
		return rc;
	}
	
	//Rx_Rx raw data average caculation
	rc = tp_rx_rx_raw_data_max();
	if (rc) {
			TOUCH_PRINTK(0, "Failed to caculate rx_rx raw data maximum (rc=%d)\n",
				rc);
		rc = -EFAULT;
		return rc;
	}
	
	//Rx_Rx raw data average caculation
	rc = tp_rx_rx_raw_data_min();
	if (rc) {
			TOUCH_PRINTK(0, "Failed to caculate rx_rx raw data minimum (rc=%d)\n",
				rc);
		rc = -EFAULT;
		return rc;
	}
	
	//compare rx_rx raw data aveg/max/min
	rc = tp_check_rx_rx_raw_data(aveg_raw_data,max_raw_val,min_raw_val);
	if (rc) {
			TOUCH_PRINTK(0, "Failed to caculate rx_rx raw data minimum (rc=%d)\n",
				rc);
		rc = -EFAULT;
		return rc;
	}
	return rc;
}

//check rawData
static int tp_check_differ(void)
{
	int i,j;
	int count = 0;
	
	if (g_ts->g_differ == 0) {
		TOUCH_PRINTK(0, "Allocate memory for differ!\n");
		g_ts->g_differ = kzalloc(g_ts->tx_num * g_ts->rx_num * 2, GFP_KERNEL);
		if (!g_ts->g_differ) {
			TOUCH_PRINTK(0, "Cannot allocate memory fordiffer!\n");
			return -1;
		}
	}
	// Check against test limits and print raw data
	TOUCH_PRINTK(1, "Tx num: %d, Rx num: %d\n", g_ts->tx_num, g_ts->rx_num);
	for (i = 0; i < g_ts->tx_num; i++) {
		TOUCH_PRINTK(0,"Col(%2d): ", i);
		for (j = 0; j < g_ts->rx_num; j++) {
			g_ts->g_differ[i * g_ts->rx_num + j] = (g_ts->g_RawData[i * g_ts->rx_num + j] - g_ts->g_BaseLine[i * g_ts->rx_num + j]);
			if (i <= g_ts->tx_num-2) {
				//touch ared
				if (g_ts->g_differ[i * g_ts->rx_num + j] >= DIFFER_MIN_LIMIT && g_ts->g_differ[i * g_ts->rx_num + j]<= DIFFER_MAX_LIMIT)
				{
					count += 0;
					printk(" %5d ", g_ts->g_differ[i * g_ts->rx_num + j]);
				} else {
					count += 1;
					printk(" %5d (fail)", g_ts->g_differ[i * g_ts->rx_num + j]);
				}
			} else if (i == g_ts->tx_num-1) {
				//keys area
				if (j == 1 || j == 5 || j == 10) {
					if (g_ts->g_differ[i * g_ts->rx_num + j] >= DIFFER_MIN_LIMIT && g_ts->g_differ[i * g_ts->rx_num + j]<= DIFFER_MAX_LIMIT)
					{
						count += 0;
						printk(" %5d ", g_ts->g_differ[i * g_ts->rx_num + j]);
					} else {
						count += 1;
						printk(" %5d (fail)", g_ts->g_differ[i * g_ts->rx_num + j]);
					}
				} else {
					//other area
					printk(" %5d ", g_ts->g_differ[i * g_ts->rx_num + j]);
				}
			}  else {
				//out of range
			}
		}
		printk("\n");
	}
	
	if (count == 0) {
		g_ts->pass_flag += 0;
		TOUCH_PRINTK(0, "differ testing Pass.\n");
	} else {
		g_ts->pass_flag  += 1;
		TOUCH_PRINTK(0, "differ testing Fail.\n");
	}
	return 0;
}
static int tp_copy_rawData(void)
{
	int i,j;
	
	TOUCH_PRINTK(0, "%s called\n", __FUNCTION__);
	// allocate memory for base line
	if (g_ts->g_BaseLine == 0) {
		TOUCH_PRINTK(1, "Allocate memory for base line!\n");
		g_ts->g_BaseLine = kzalloc(g_ts->tx_num * g_ts->rx_num * 2, GFP_KERNEL);
		if (!g_ts->g_BaseLine) {
			TOUCH_PRINTK(0, "Cannot allocate memory for base line!\n");
			return -1;
		}
	}
	memcpy(g_ts->g_BaseLine, g_ts->g_RawData, g_ts->tx_num * g_ts->rx_num * 2);
    //print baseline
	TOUCH_PRINTK(1, "print baseline\n");
	for (i = 0; i < g_ts->tx_num; i++) {
		TOUCH_PRINTK(1, "Col(%2d): ", i);
		for (j = 0; j < g_ts->rx_num; j++) {
			TOUCH_PRINTK(1, " %5d ", g_ts->g_BaseLine[i * g_ts->rx_num + j]);
		}
		TOUCH_PRINTK(1, "\n");
	}
	return 0;
}

int ft5316_auto_clb(void)
{
	unsigned char i;
	uint8_t dev_mode,calb;
	int rc = 0;
	
	TOUCH_PRINTK(0, "start auto calibration.\n");
	msleep(200);
	//ft5x0x_write_reg(0, 0x40);
	dev_mode= 0x40;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &dev_mode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write dev_mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, dev_mode, rc);
		rc = -EFAULT;
		return rc;
	}
	delay_qt_ms(100);   //make sure already enter factory mode
	//ft5x0x_write_reg(2, 0x4);  //write command to start calibration
	calb = 0x4;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR+2, &calb, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write calb[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+2, calb, rc);
		rc = -EFAULT;
		return rc;
	}
	delay_qt_ms(300);
	for (i = 0; i < 100; i++) {
		//ft5x0x_read_reg(0, &uc_temp);
		rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR, &dev_mode, 1);
		if (rc) {
			TOUCH_PRINTK(0, "Failed to read dev_mode[0x%x]=0x%x (rc=%d)\n",
				START_REG_MAP_ADDR, dev_mode, rc);
			rc = -EFAULT;
			return rc;
		}
		if (((dev_mode & 0x70) >> 4) == 0x0)  //return to normal mode, calibration finish
			{
			break;
		}
		delay_qt_ms(200);
		TOUCH_PRINTK(0, "waiting calibration %d\n", i);

	}
	TOUCH_PRINTK(0, "calibration OK.\n");
	msleep(300);
	//ft5x0x_write_reg(0, 0x40);  //goto factory mode
	dev_mode= 0x40;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &dev_mode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write dev_mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, dev_mode, rc);
		rc = -EFAULT;
		return rc;
	}
	delay_qt_ms(100);   //make sure already enter factory mode
	//ft5x0x_write_reg(2, 0x5);  //store CLB result
	calb = 0x5;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR+2, &calb, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write calb[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+2, calb, rc);
		rc = -EFAULT;
		return rc;
	}
	delay_qt_ms(300);
	//ft5x0x_write_reg(0, 0x0); //return to normal mode
	dev_mode= 0x0;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &dev_mode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write dev_mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, dev_mode, rc);
		rc = -EFAULT;
		return rc;
	}
	msleep(300);
	TOUCH_PRINTK(0, "store CLB result OK.\n");
	return rc;
}
static int tp_get_differ(void)
{
	int rc = 0,i;
	uint8_t vol = 0;
	
	TOUCH_PRINTK(0, "%s()\n", __func__);
	//copy rawData form vol 5
	rc = tp_copy_rawData();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to copy raw data. (rc=%d)\n", rc);
		rc = -EFAULT;
		return rc;
	}
	
	//write voltage level:0x0
	vol = 0x0;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR+5, &vol, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write chip_vol[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+5, vol, rc);
		rc = -EFAULT;
		return rc;
	}
	msleep(100);
	//read voltage	
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR+5, &vol, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to read chip_vol[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+5, vol, rc);
		rc = -EFAULT;
		return rc;
	}
	TOUCH_PRINTK(0, "read chip_vol[0x%x]=0x%x\n",
			START_REG_MAP_ADDR+5, vol);
	
	for (i = 0; i < 6; i++) {
		//ST_SCAN
		rc = ft5316_start_scan();
		if (rc) {
			TOUCH_PRINTK(0, "Failed to start scan. (rc=%d)\n", rc);
			rc = -EFAULT;
			return rc;
		}
		rc = ft5316_start_scan();
		if (rc) {
			TOUCH_PRINTK(0, "Failed to start scan.. (rc=%d)\n", rc);
			rc = -EFAULT;
			return rc;
		}
		//get rawData
		rc = tp_get_rawData();
		if (rc) {
			TOUCH_PRINTK(0, "Failed to read raw data. (rc=%d)\n", rc);
			rc = -EFAULT;
			return rc;
		}
	}	
	//check Differ
	rc = tp_check_differ();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to check differ. (rc=%d)\n", rc);
		rc = -EFAULT;
		return rc;
	}
	
	//write voltage level:0x5
	vol = 0x5;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR+5, &vol, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write chip_vol[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+5, vol, rc);
		rc = -EFAULT;
		return rc;
	}
	msleep(100);
	// read voltage	
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR+5, &vol, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to read chip_vol[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+5, vol, rc);
		rc = -EFAULT;
		return rc;
	}
	TOUCH_PRINTK(0, "read chip_vol[0x%x]=0x%x\n",
			START_REG_MAP_ADDR+5, vol);
	//do calibration
	rc = ft5316_auto_clb();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to auto calibration. (rc=%d)\n",rc);
		rc = -EFAULT;
		return rc;
	}

	return rc;
}
//check rawData
static int tp_check_rawData(void)
{
	int i,j;
	int count = 0;
	TOUCH_PRINTK(0, "%s()\n", __func__);
	// Check against test limits and print raw data
	TOUCH_PRINTK(0, "Tx num: %d, Rx num: %d\n", g_ts->tx_num, g_ts->rx_num);
	for (i = 0; i < g_ts->tx_num; i++) {
		TOUCH_PRINTK(0,"Col(%2d): ", i);
		for (j = 0; j < g_ts->rx_num; j++) {
			//raw data of key
			if(i == g_ts->tx_num-1) {
				printk(" %5d ", g_ts->g_RawData[i * g_ts->rx_num + j]);
				count += 0;
			} else {
				//raw data of touch panel
				if (g_ts->g_RawData[i * g_ts->rx_num + j] >= RAW_DATA_MIN_LIMIT && 
					g_ts->g_RawData[i * g_ts->rx_num + j] <= RAW_DATA_MAX_LIMIT) {
					count += 0;
					printk(" %5d ", g_ts->g_RawData[i * g_ts->rx_num + j]);
				} else {
					count += 1;
					printk(" %5d (fail)", g_ts->g_RawData[i * g_ts->rx_num + j]);
				}
			}	
		}
		printk("\n");
	}
	
	if (count == 0) {
		g_ts->pass_flag = 0;
		TOUCH_PRINTK(0, "raw data testing Pass.\n");
	} else {
		g_ts->pass_flag  = 1;
		TOUCH_PRINTK(0, "raw data testing Fail.\n");
	}
	return 0;
}
static int tp_get_rawData(void)
{
	int i, j;
	char *rxBuf = 0;
	int retval = 0;
	
	TOUCH_PRINTK(1, "%s()\n", __func__);
	// allocate memory for raw data
	if (g_ts->g_RawData == 0) {
		TOUCH_PRINTK(1, "Allocate memory for raw data!\n");
		g_ts->g_RawData = kzalloc(g_ts->tx_num * g_ts->rx_num * 2, GFP_KERNEL);
		if (!g_ts->g_RawData) {
			TOUCH_PRINTK(0, "Cannot allocate memory for raw data!\n");
			return -1;
		}
	}
	rxBuf = kzalloc(g_ts->rx_num * 2, GFP_KERNEL);
	if (!rxBuf) {
		TOUCH_PRINTK(0, "Cannot allocate memory for reading raw data by rx!\n");
		return -1;
	}

	//Get Each Raw data according to TX
	for (i = 0; i < g_ts->tx_num; i++) {
		TOUCH_PRINTK(1, "Col(%2d): ", i);
		ft5316_write_reg(1, i);
		msleep(5);
		ft5316_read_block(0x10, g_ts->rx_num * 2, rxBuf);
		for (j = 0; j < g_ts->rx_num; j++) {
			g_ts->g_RawData[g_ts->rx_num * i + j] = (rxBuf[j * 2] << 8) + rxBuf[j * 2 + 1];
			TOUCH_PRINTK(1, " %5d ", g_ts->g_RawData[i * g_ts->rx_num + j]);
		}
		TOUCH_PRINTK(1, "\n");
	}
	
	if (rxBuf)
		kfree(rxBuf);
	return retval;
}

static int ft5316_enter_work(void)
{
	int rc = 0;
	u8 devmode = 0x0;
	
	TOUCH_PRINTK(0, "%s()\n", __func__);
	//return to normal mode
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &devmode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to write device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	msleep(100);
	
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR, &devmode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to read device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	
	if((devmode & 0x70) != 0x00) {
		TOUCH_PRINTK(0, "%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode[0x%x]=0x%X\n", 
			__FUNCTION__, START_REG_MAP_ADDR, devmode);
		rc = -EFAULT;
		return rc;
	}
	
	devmode |= 0x0;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &devmode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to write device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	msleep(20);
	TOUCH_PRINTK(0, "device_mode[0x%x]=0x%x\n", START_REG_MAP_ADDR, devmode);
	//enable_irq(g_ts->irq);
	return rc;
}

static int ft5316_start_scan(void)
{
	int rc = 0;
	u8 devmode = 0;
	
	TOUCH_PRINTK(1, "%s()\n", __FUNCTION__);
	//scan
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR, &devmode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to read device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	devmode |= 0x80;
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &devmode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to write device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	msleep(20);
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR, &devmode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to read device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	
	if (0x00 != (devmode & 0x80)) {
		TOUCH_PRINTK(0, "%s() failed to read device mode[0x%x]=0x%x (rc=%d)\n",
			__func__, START_REG_MAP_ADDR, devmode, rc);
		rc = -EFAULT;
		return rc;
	}
	TOUCH_PRINTK(1, "device_mode[0x%x]=0x%x\n", START_REG_MAP_ADDR, devmode);
	return rc;
}

static int ft5316_enter_factory(void)
{
	int rc = 0;
	uint8_t device_mode = 0x40;
	
	TOUCH_PRINTK(0, "%s()\n", __FUNCTION__);
	//disable_irq_nosync(g_ts->irq);
	rc = ft5316_write_i2c_data(g_ts->client, START_REG_MAP_ADDR, &device_mode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to write device mode[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, device_mode, rc);
		rc = -EFAULT;
		return rc;
	}
	delay_qt_ms(100);//make sure already enter factory mode
	
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR, &device_mode, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to read device device[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR, device_mode, rc);
		rc = -EFAULT;
		return rc;
	}
	
	if((device_mode & 0x70) != 0x40) {
		TOUCH_PRINTK(0, "%s() - ERROR: The Touch Panel was not put in Factory Mode. The device device[0x%x]=0x%x (rc=%d)\n",
			__FUNCTION__, START_REG_MAP_ADDR, device_mode, rc);
		rc = -EFAULT;
		return rc;
	}
	TOUCH_PRINTK(0, "device_mode[0x%x]=0x%x\n", START_REG_MAP_ADDR, device_mode);
	return rc;
}

/* *************************************************************************
 * read raw data for self test
 * ************************************************************************* */
static int tp_selftest_all(void *data, u64 *val)
{
	int rc = 0;
	uint8_t tx_rx_num[2];
	
	TOUCH_PRINTK(0, "%s()\n", __FUNCTION__);
	//mutex_lock(&g_ts->mutex);
	msleep(200);
	//1) switch to factory mode
	rc = ft5316_enter_factory();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to enter factory mode(rc=%d)\n", rc);
		goto error_return;
	}
	//2. ST_SCAN
	rc = ft5316_start_scan();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to start scan. (rc=%d)\n", rc);
		goto error_return;
	}
	rc = ft5316_start_scan();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to start scan.. (rc=%d)\n", rc);
		goto error_return;
	}
	//3. read Tx_num and Rx_num
	memset(tx_rx_num,0x0,2);
	rc = ft5316_read_i2c_data(g_ts->client, START_REG_MAP_ADDR+3, &tx_rx_num[0], 2);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to read tx_rx_num[0x%x]=0x%x (rc=%d)\n",
			START_REG_MAP_ADDR+3, tx_rx_num[0], rc);
		goto error_return;
	}
	g_ts->tx_num = tx_rx_num[0];
	g_ts->rx_num = tx_rx_num[1];
	TOUCH_PRINTK(0, "tx_num[0x%x]=0x%x\n", START_REG_MAP_ADDR+3, g_ts->tx_num);
	TOUCH_PRINTK(0, "rx_num[0x%x]=0x%x\n", START_REG_MAP_ADDR+4, g_ts->rx_num);
	//4. get raw data
	rc = tp_get_rawData();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to get raw data. (rc=%d)\n", rc);
		goto error_return;
	}
	//5. read raw data
	rc = tp_check_rawData();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to check raw data. (rc=%d)\n", rc);
		goto error_return;
	}
	//6. get Differ test
	rc = tp_get_differ();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to get differ. (rc=%d)\n", rc);
		goto error_return;
	}
	//7. get Rx-Rx data analysis
	rc = tp_get_RX_RX_data_analysis();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to get RX_RX data. (rc=%d)\n", rc);
		goto error_return;
	}
	
	//8. enter work mode
	rc = ft5316_enter_work();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to enter work mode. (rc=%d)\n", rc)
		goto error_return;
	}
	
	//9. return test result
	if(g_ts->pass_flag == 0) 
	{
	    TOUCH_PRINTK(0, "self test all pass.(test result=%d)\n",g_ts->pass_flag);  
		*val = g_ts->pass_flag;
	} else {
		TOUCH_PRINTK(0, "self test all Fail.(test result=%d)\n",g_ts->pass_flag);
		*val = g_ts->pass_flag;
	}
	
	mutex_unlock(&g_ts->mutex);
	return 0;
	
error_return:
	rc = ft5316_enter_work();
	if (rc) {
		TOUCH_PRINTK(0, "Failed to enter work mode. (rc=%d)\n", rc)
	}
	mutex_unlock(&g_ts->mutex);
	rc = -EFAULT;
	return rc;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_selftest, tp_selftest_all, NULL, "%llu")

/* *************************************************************************
 * add debugfs for enabling/disabling touch panel
 * ************************************************************************* */
static int dbg_tp_onoff_get(void *data, u64 *value)
{
	int state = 0;
	TOUCH_PRINTK(0, "%s: %s touch panel\n",
		__func__, g_ts->is_suspended ? "disable":"enable");
	if (g_ts->is_suspended == 1)
		state = 0;
	else if (g_ts->is_suspended == 0)
		state = 1;
		
	*value = state;
	return 0;
}

static int dbg_tp_onoff_set(void *data, u64 value)
{
	if (value == 0) {
		ft5316_ts_early_suspend(&g_ts->ts_early_suspend);
		TOUCH_PRINTK(0, "%s, disable touch panel\n",__func__);
	} else if (value == 1) {
		ft5316_ts_late_resume(&g_ts->ts_early_suspend);
		TOUCH_PRINTK(0, "%s, enable touch panel\n",__func__);
	} else {
			TOUCH_PRINTK(0, "%s: Error command(%lld)\n",
				__func__, value);
	}
			
	TOUCH_PRINTK(0, "%s: %s touch panel\n",
		__func__, g_ts->is_suspended ? "disable":"enable");
	
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_onoff_fops, dbg_tp_onoff_get, dbg_tp_onoff_set, "%llu\n");

#define TOUCH_DRIVER_NAME	"ft5316_ts_dbg"
/* *************************************************************************
 * Description: ADD DEBUGFS FOR CSD and FTD test
 * ************************************************************************* */
static void touchpad_create_debugfs_entries(void)
{
	g_ts->dent = debugfs_create_dir(TOUCH_DRIVER_NAME, NULL);
	if (g_ts->dent) {
		debugfs_create_file("tp_onoff", S_IRUGO | S_IWUGO, g_ts->dent, g_ts, &tp_onoff_fops);
		debugfs_create_file("tp_self_test", S_IRUGO | S_IWUGO, g_ts->dent, g_ts, &tp_selftest);
	}
}

/* *************************************************************************
 * read vendor id for ftd test
 * ************************************************************************* */
static ssize_t tp_read_vendor_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	int vendor_id;
	
	if(gpio_get_value(g_ts->gpio_vendor_id) == 1)
		vendor_id = 1;
	else
		vendor_id = 0;
	return snprintf(buf, 11, "%d\n",vendor_id);
}
static DEVICE_ATTR(tp_vendor_id, 0444, tp_read_vendor_id, NULL);


/* *************************************************************************
 * read fw version for ftd test
 * ************************************************************************* */
static ssize_t tp_read_fw_ver(struct device *dev, struct device_attribute *attr,char *buf)
{
	if (g_ts->fw_id_val!=0x00)
		return snprintf(buf, 11, "%d\n",g_ts->fw_id_val);
	else
		return -EINVAL;
}
static DEVICE_ATTR(tp_fw_ver, 0444, tp_read_fw_ver, NULL);

/* *************************************************************************
 * read device id for ftd test
 * ************************************************************************* */
static ssize_t tp_read_device_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int rc;
	uint8_t dev_id_val;
    
	mutex_lock(&g_ts->mutex);
	//read chip id
	rc = ft5316_read_i2c_data(client, FT5316_REG_CIPHER, &dev_id_val, 1);
	if (rc) {
		TOUCH_PRINTK(0, "failed to read device id[0x%x]=0x%x (rc=%d)\n",
			FT5316_REG_CIPHER, dev_id_val, rc);
		mutex_unlock(&g_ts->mutex);
		return rc;
	}
	TOUCH_PRINTK(0, "device id[0x%x]=0x%x\n", FT5316_REG_CIPHER, dev_id_val);

	if(dev_id_val == 0xa)
	{
		mutex_unlock(&g_ts->mutex);
		return snprintf(buf, 11, "%d\n",dev_id_val);
	}	
	else 
	{
		mutex_unlock(&g_ts->mutex);
		return -EINVAL;
	}
}
static DEVICE_ATTR(device_id, 0444, tp_read_device_id, NULL);

/* *************************************************************************
 * Description: SYS FOR FTD TEST TO GET DEVICE ID,tp_fw_ver,vendor_id
 * ************************************************************************* */
static int touchpad_create_sys_entries(struct i2c_client *client)
{
	int ret=0;

	ret = device_create_file(&client->dev,&dev_attr_device_id);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_tp_fw_ver);
	WARN_ON(ret);   
	if (ret)  return ret;	
	
	ret = device_create_file(&client->dev,&dev_attr_tp_vendor_id);
	WARN_ON(ret); 

	return ret;
}

static int ft5316_ts_suspend(struct i2c_client *client)
{
	struct ft5631_ts_info *ts = i2c_get_clientdata(client); 
	int rc = 0;
    uint8_t pwr_mode_val;
	
	TOUCH_PRINTK(0, "ft5316_ts_suspend() +++\n");
	mutex_lock(&ts->mutex);
	if (ts->is_suspended) {
		TOUCH_PRINTK(0, "ft5316 ts cip is in deep sleep mode!!!\n");
		mutex_unlock(&ts->mutex);
		rc = -EFAULT;
	}

	/* Emily, 20121022 */
	/* fixed GDETB1A_Detroit-003747 */
	/* [FT v03.001.02][Idle] DUT touch can not work after receiving an MT call. */ 
	/* to avoid mutex deadlock when the panel is touched after suspending */
	mutex_unlock(&ts->mutex);
	/* disable worker */
	disable_irq(ts->irq);
	TOUCH_PRINTK(1,"disable irq %d\n", ts->irq);
	mutex_lock(&ts->mutex);
	/* Emily, 20121022 */
	ts->is_suspended = 1;

#if 0
    /* release cap key when device in sleep mode */
    ts->key_msg.key_coord.x = 0x0;
    ts->key_msg.key_coord.y = 0x0;
    ts->key_msg.key_coord.state = TS_PUT_UP;
    input_report_key(ts->keyarray_input, KEY_BACK, 0);
    input_report_key(ts->keyarray_input, KEY_HOMEPAGE, 0);
	input_report_key(ts->keyarray_input, KEY_MENU, 0);
#endif
		
	/* setting ft5316 ts chip into deep sleep mode */
	pwr_mode_val = TS_HIBERNATE_MODE;
	rc = ft5316_write_i2c_data(ts->client, FT5316_REG_PMODE, &pwr_mode_val, 1);
	if (rc) {
		TOUCH_PRINTK(0, "Failed to write the power contrl[0x%x]:0x%x into deep sleep mode. (rc=%d)\n",
			FT5316_REG_PMODE, pwr_mode_val, rc);
		rc = -EFAULT;
	}
	
	rc = ft5316_read_i2c_data(ts->client, FT5316_REG_PMODE, &pwr_mode_val, 1);
	if (rc < 0) {
		TOUCH_PRINTK(0, "Failed to read the power contrl[0x%x]:0x%x into deep sleep mode. (rc=%d)\n", 
			FT5316_REG_PMODE, pwr_mode_val, rc);
		rc = -EFAULT;
	}
	TOUCH_PRINTK(0, "read the power contrl[0x%x]:0x%x into deep sleep mode. (rc=%d)\n", 
			FT5316_REG_PMODE, pwr_mode_val, rc);
/* Add for PM LOG */
#ifdef CONFIG_PM_LOG
	rc = pmlog_device_off(ts->pmlog_device);
	if (rc)
		TOUCH_PRINTK(0, "[PM_LOG]touch off ,fail rc = %d\n", rc);
#endif //CONFIG_PM_LOG
	mutex_unlock(&ts->mutex);
	TOUCH_PRINTK(0, "ft5316_ts_suspend() ---\n");
	return rc;
	
}

static int ft5316_ts_resume(struct i2c_client *client)
{
	struct ft5631_ts_info *ts = i2c_get_clientdata(client);
	int rc = 0;
	int i;
	
	TOUCH_PRINTK(0, "ft5316_ts_resume() +++\n");
	mutex_lock(&ts->mutex);
    /* reset mt points coordinates after wakeup */
	for (i = 0 ; i < MAX_TS_REPORT_POINTS ; i++)
	{
		ts->msg[i].coord.pre_state = ts->msg[i].coord.state;
		ts->msg[i].coord.state = TS_PUT_UP;
		if(ts->msg[i].coord.z != -1) {
			ts->msg[i].coord.z = 0;
		} 
	}
	ft5316_ts_report_mt_protocol(ts);
	ft5316_report_capkey(ts);
	
	//set hw reset to wake up device
	gpio_set_value_cansleep(ts->gpio_rst, 0);
	msleep(20);
	gpio_set_value_cansleep(ts->gpio_rst, 1);
	msleep(100);
		
	enable_irq(ts->irq);
	ts->is_suspended = 0;	
	/* Add for PM LOG */
#ifdef CONFIG_PM_LOG
	rc = pmlog_device_on(ts->pmlog_device);
	if (rc)
		TOUCH_PRINTK(0, "[PM_LOG]touch on ,fail rc = %d\n", rc);
#endif	//CONFIG_PM_LOG
	mutex_unlock(&ts->mutex);
	TOUCH_PRINTK(0, "ft5316_ts_resume() ---\n");
	return rc;
	
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5316_ts_early_suspend(struct early_suspend *h)
{
	struct ft5631_ts_info *ts = container_of(h, 
		struct ft5631_ts_info, ts_early_suspend);
	ft5316_ts_suspend(ts->client);
}

static void ft5316_ts_late_resume(struct early_suspend *h)
{
	struct ft5631_ts_info *ts = container_of(h, 
		struct ft5631_ts_info, ts_early_suspend);
	ft5316_ts_resume(ts->client);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static void ft5316_report_capkey(struct ft5631_ts_info *ts)
{
	int i;
	struct ft5316_point_t *target_coord;
	
	for(i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		if(ts->msg[i].switched == 1) {//.coord contains touch_down coord, so .switched_coord contains fake key_up coord
			target_coord = &ts->msg[i].switched_coord;
		} else {
			target_coord = &ts->msg[i].coord;
		}
		if (target_coord->x <= DISPLAY_PANEL_MAX_X && target_coord->y <= DISPLAY_PANEL_MAX_Y)
			continue;
		if (target_coord->y < ts->keys_Y) 
			continue;
		if (target_coord->z == -1)
			continue;
		
		if (target_coord->x == ts->back_key_x) {
			if (target_coord->z > 0) {
				KEY_PRINTK(1, "id[%d] Back(x,y)=(%d,%d) keycode[%d]:press\n",
					i, target_coord->x, target_coord->y, KEY_BACK);
				input_report_key(ts->keyarray_input, KEY_BACK, 1);
			} else {
				KEY_PRINTK(1, "id[%d] Back(x,y)=(%d,%d) keycode[%d]:release\n",
					i, target_coord->x, target_coord->y, KEY_BACK);
				input_report_key(ts->keyarray_input, KEY_BACK, 0);
			}
		}else if (target_coord->x == ts->home_key_x) {
			if (target_coord->z > 0) {
				KEY_PRINTK(1, "id[%d] Home(x,y)=(%d,%d) keycode[%d]:press\n", 
					i, target_coord->x, target_coord->y, KEY_HOMEPAGE);
				input_report_key(ts->keyarray_input, KEY_HOMEPAGE, 1);
			} else {
				KEY_PRINTK(1, "id[%d] Home(x,y)=(%d,%d) keycode[%d]:release\n",
					i, target_coord->x, target_coord->y, KEY_HOMEPAGE);
				input_report_key(ts->keyarray_input, KEY_HOMEPAGE, 0);
			}
		}else if (target_coord->x == ts->menu_key_x) {
			if (target_coord->z > 0) {
				KEY_PRINTK(1, "id[%d] Menu(x,y)=(%d,%d) keycode[%d]:press\n",
					i, target_coord->x, target_coord->y, KEY_MENU);
				input_report_key(ts->keyarray_input, KEY_MENU, 1);
			} else {
				KEY_PRINTK(1, "id[%d] Menu(x,y)=(%d,%d) keycode[%d]:release\n",
					i, target_coord->x, target_coord->y, KEY_MENU);
				input_report_key(ts->keyarray_input, KEY_MENU, 0);
			}	
		}
		
		if (target_coord->z == 0)
			target_coord->z = -1;
	}		
	input_sync(ts->keyarray_input);
	return;
}

static void ft5316_ts_report_mt_protocol(struct ft5631_ts_info *ts)
{
	//struct ft5316_ts_event_t *event = &ts->event;
	int i;
	int all_up = 1;
	struct ft5316_point_t *target_coord;
	
	for(i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		if(ts->msg[i].switched == 2) {//.coord contains key_down coord, so .switched_coord contains fake touch_up coord
			target_coord = &ts->msg[i].switched_coord;
		} else {
			target_coord = &ts->msg[i].coord;
		}
		if (target_coord->z == -1)
			continue;
		if (target_coord->x > DISPLAY_PANEL_MAX_X || target_coord->y > DISPLAY_PANEL_MAX_Y)
			continue;
		input_report_abs(ts->input, ABS_MT_POSITION_X, target_coord->x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, target_coord->y);
		if(target_coord->z == 0)input_report_abs(ts->input, ABS_MT_PRESSURE, 1);
		else input_report_abs(ts->input, ABS_MT_PRESSURE, target_coord->z);
		TOUCH_EVENT_PRINTK(1, "id=%d, state=%d,(x,y)=(%d,%d),z=(%d)\n",
			i, target_coord->state, target_coord->x, target_coord->y, target_coord->z);
		if(target_coord->z != 0) all_up = 0;
		input_mt_sync(ts->input);
		
		if (target_coord->z == 0)
			target_coord->z = -1;
	}
	input_sync(ts->input);
	if(all_up) 
	{
		input_mt_sync(ts->input);
		input_sync(ts->input);
	}
}

static uint is_key_or_touch(uint x,uint y,struct ft5631_ts_info *ts)
{
	if (x <= DISPLAY_PANEL_MAX_X && y <= DISPLAY_PANEL_MAX_Y)
	{
		return 1; //(x,y) is touch
	}else {
		if (y >= ts->keys_Y) return 2; //(x,y) is key
		else {
			printk("touch: warning (%d,%d) is not touch nor key\n",x,y);
			return 2;
		}
	}
}

static uint is_switched(uint new_x,uint new_y,uint new_state,uint old_x,uint old_y,uint old_state,struct ft5631_ts_info *ts)
{
	if(old_state == TS_PUT_DOWN || old_state == TS_CONTACT)
	{
		if ( is_key_or_touch(new_x,new_y,ts) != is_key_or_touch(old_x,old_y,ts))
			return is_key_or_touch(new_x,new_y,ts);
		else
			return 0;
	}else if (old_state == TS_NO_EVENT) {
		printk("touch: warning TS_NO_EVENT\n");
		return 0;
	}else {
		return 0;
	}
}
static void ft5316_ts_report_coord(struct ft5631_ts_info *ts)
{
	struct ft5316_ts_event_t *event = &ts->event;
	int i;
	
	for (i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		//if (event->x[i] <= DISPLAY_PANEL_MAX_X && event->y[i] <= DISPLAY_PANEL_MAX_Y) {
			if (event->state[i] == TS_NO_EVENT) event->state[i] = TS_PUT_UP;
			ts->msg[i].switched = is_switched(event->x[i],event->y[i],event->state[i],ts->msg[i].coord.x,ts->msg[i].coord.y,ts->msg[i].coord.state,ts);
			if(ts->msg[i].switched){
				ts->msg[i].switched_coord.x = ts->msg[i].coord.x;
				ts->msg[i].switched_coord.y = ts->msg[i].coord.y;
				ts->msg[i].switched_coord.pre_state = ts->msg[i].coord.state;
				ts->msg[i].switched_coord.state = TS_PUT_UP;
				ts->msg[i].switched_coord.z = 0;
			}
			ts->msg[i].coord.x = (uint)event->x[i];
			ts->msg[i].coord.y = (uint)event->y[i];
			ts->msg[i].coord.pre_state = ts->msg[i].coord.state;
			ts->msg[i].coord.state = event->state[i];
			if (ts->msg[i].coord.state == TS_PUT_DOWN || ts->msg[i].coord.state == TS_CONTACT) {
				if(event->z[i] <= 0) event->z[i] = 1;
				ts->msg[i].coord.z = (uint)event->z[i];
			}else if (ts->msg[i].coord.state == TS_NO_EVENT) {
				printk("touch: warning TS_NO_EVENT\n");
			}else if (ts->msg[i].coord.state == TS_PUT_UP  && ts->msg[i].coord.z != -1) {
				ts->msg[i].coord.z = 0;
			} 
			else {
				ts->msg[i].coord.z = -1;
				//input_report_abs(ts->input, ABS_MT_PRESSURE, ts->msg[i].coord.z);
			}
#if 0
		} else {
			//key event
			if (event->y[i] >= ts->keys_Y) {
				ts->key_msg.key_coord.x = (uint)event->x[i];
				ts->key_msg.key_coord.y = (uint)event->y[i];
				ts->key_msg.key_coord.state = event->state[i];
				ft5316_report_capkey(ts, i);
				if (event->touch_point == 0x0 && ts->msg[i].coord.pre_state == TS_CONTACT) {
					input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
					ts->msg[i].coord.z = -1;
					input_sync(ts->input);
				}
					input_sync(ts->input);	
			}
		}
#endif
	}
	ft5316_ts_report_mt_protocol(ts);
	ft5316_report_capkey(ts);
}

/* request irq thread for FocalTech ft5316 chip */
static irqreturn_t ft5316_irq_thread(int irq, void *dev_id)
{
	struct ft5631_ts_info *ts = dev_id;
	struct i2c_client	*client = ts->client;
	struct ft5316_ts_event_t *event = &ts->event;
	u8 buf[READ_BUF_SIZE] = { 0 };
	int i;
    int rc = 0;
    
    mutex_lock(&ts->mutex);
	disable_irq_nosync(ts->irq);

	//read touch id and coordinates
	rc = ft5316_read_i2c_data(client, START_REG_MAP_ADDR, &buf[0], READ_BUF_SIZE);
	if (rc < 0) {
		TOUCH_PRINTK(0, "Failed to read touch id and points[0x%x]:0x%x (rc=%d)\n", 
		START_REG_MAP_ADDR, buf[0], rc);
		enable_irq(ts->irq);
		mutex_unlock(&ts->mutex);
		return IRQ_HANDLED;
	}
	event->touch_point = buf[2] & 0x07;
	TOUCH_PRINTK(1,"read number of touch points[0x%x]:%d\n", START_REG_MAP_ADDR+2, (int)event->touch_point);
	//read touch id and touch event	
	for (i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		event->x[i] = (buf[3 + 6 * i] & 0x0F) << 8 | buf[4 + 6 * i];
		event->y[i] = (buf[5 + 6 * i] & 0x0F) << 8 | buf[6 + 6 * i];
		event->state[i] = buf[3 + 6 * i] >> 6;
		event->finger_id[i] = (buf[5 + 6 * i]) >> 4;
		event->z[i] = buf[7 + 6 * i];
	}
	/* report touch coordinate */
	ft5316_ts_report_coord(ts);
	
	enable_irq(ts->irq);
	mutex_unlock(&ts->mutex);
    return IRQ_HANDLED;
}

#if 0
static int ft5316_keyarray_open(struct input_dev *dev)
{
	int rc = 0;
    struct ft5631_ts_info *kp = input_get_drvdata(dev);
	
	TOUCH_PRINTK(0, "open keyarray input class\n");
    mutex_lock(&kp->mutex);
    if(kp->keyarray_open_count == 0) {
        kp->keyarray_open_count++; //record opencount
        TOUCH_PRINTK(0, "keyarray opened %d times\n",kp->keyarray_open_count);      
    }
    mutex_unlock(&kp->mutex);
    
    return rc;
}

static void ft5316_keyarray_close(struct input_dev *dev)
{
	struct ft5631_ts_info *kp = input_get_drvdata(dev);
	
	TOUCH_PRINTK(0,"close keyarray input class\n");
	mutex_lock(&kp->mutex);
    if(kp->keyarray_open_count > 0) {
       kp->keyarray_open_count--;
        TOUCH_PRINTK(0, "still opened keyarray %d times\n", kp->keyarray_open_count);        
    }
    mutex_unlock(&kp->mutex);
}
#endif

/* register keyarray input device for FOCALTECH FT5316 chip */
static int ft5316_keyarray_register_input( struct input_dev **input,
                            struct i2c_client *client)
{
	int rc = 0;
	//int i;
	struct input_dev *input_dev;
  
	input_dev = input_allocate_device();
	if (!input_dev)
	{
		rc = -ENOMEM;
		return rc;
	}

	input_dev->name = FOCALTECH_CAPKEY_NAME;
	input_dev->phys = "ft5316_capkey/event0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &client->dev;
	//input_dev->open = ft5316_keyarray_open;
	//input_dev->close = ft5316_keyarray_close;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	set_bit(EV_SYN, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, KEY_BACK);
	input_set_capability(input_dev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(input_dev, EV_KEY, KEY_MENU);
	
	TOUCH_PRINTK(1, "%s: Register keyarray input device\n", FOCALTECH_CAPKEY_NAME);
	rc = input_register_device(input_dev);
	if (rc) {
		TOUCH_PRINTK(0, "%s: Failed to register keyarray input device\n", FOCALTECH_CAPKEY_NAME);
		input_free_device(input_dev);
	}
	else {
		*input = input_dev;
	}
  
  return rc;
}

static int ft5316_ts_open(struct input_dev *dev)
{
    int rc = 0;
    struct ft5631_ts_info *pTS = input_get_drvdata(dev);
	
    TOUCH_PRINTK(1, "open touch input class\n");
    mutex_lock(&pTS->mutex);
    if(pTS->open_count == 0)
    {
        pTS->open_count++;
        TOUCH_PRINTK(1, "opened touch %d times\n", pTS->open_count);      
    }
    mutex_unlock(&pTS->mutex);
  
    return rc;
}

static void ft5316_ts_close(struct input_dev *dev)
{
    struct ft5631_ts_info *pTS = input_get_drvdata(dev);
	
	TOUCH_PRINTK(1, "close touch input class\n");
    mutex_lock(&pTS->mutex);
	
    if(pTS->open_count > 0)
    {
        pTS->open_count--;
        TOUCH_PRINTK(1, "still opened touch %d times\n", pTS->open_count);        
    }
    mutex_unlock(&pTS->mutex);
}

/* register touch input device for FOCALTECH FT5316 chip */
static int ft5316_touch_register_input( struct input_dev **input,
                                    struct focaltech_tp_platform_data_t *pdata,
                                    struct i2c_client *client )
{
    int rc = 0;
    struct input_dev *input_dev;
	
	
    input_dev = input_allocate_device();
    if ( !input_dev ) {
        rc = -ENOMEM;
        return rc;
    }
    input_dev->name = FOCALTECH_TP_NAME;
    input_dev->phys = "ft5316_ts/input0";
    input_dev->id.bustype = BUS_I2C;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0002;
    input_dev->id.version = 0x0100;
    input_dev->dev.parent = &client->dev;
    input_dev->open = ft5316_ts_open;
    input_dev->close = ft5316_ts_close;
    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, DISPLAY_PANEL_MAX_X, 0, 0); 
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, DISPLAY_PANEL_MAX_Y, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,  0, MAX_TOUCH_MAJOR, 0, 0); 
	//input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,  0, 255, 0, 0); 
	//input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_TOUCH_MAJOR, 0, 0);	
	
	TOUCH_PRINTK(1, "%s: Register input device\n", FOCALTECH_TP_NAME);
    rc = input_register_device(input_dev);
    if (rc) {
        TOUCH_PRINTK(0, "%s: Failed to register input device\n", FOCALTECH_TP_NAME);
        input_free_device(input_dev);
    }else {
        *input = input_dev;
    }
    
    return rc;
}

/* read register map for FOCALTECH FT5316 chip */
static int ts_detect_ft5316(struct ft5631_ts_info *ts)
{
	int rc = 0;
	
	//read CTPM Vendor ID
	rc = ft5316_read_i2c_data(ts->client, FT5316_REG_FT5201ID, &ts->vendor_id_val, 1);
	if (rc < 0) {
		TOUCH_PRINTK(0, "Failed to read CTPM Vendor ID[0x%x]:0x%x (rc=%d)\n", 
			FT5316_REG_FT5201ID,ts->vendor_id_val, rc);
		return rc;
	}
	TOUCH_PRINTK(0,"read CTPM Vendor ID[0x%x]:0x%x\n",FT5316_REG_FT5201ID, ts->vendor_id_val);
	//read CTPM Vendor ID
	rc = ft5316_read_i2c_data(ts->client, FT5316_REG_FIRMID, &ts->fw_id_val, 1);
	if (rc < 0) {
		TOUCH_PRINTK(0, "Failed to read firmware version[0x%x]:0x%x (rc=%d)\n", 
			FT5316_REG_FIRMID, ts->fw_id_val, rc);
		return rc;
	}
	TOUCH_PRINTK(0,"read firmware version[0x%x]:0x%x\n",FT5316_REG_FIRMID, ts->fw_id_val);
	return rc;
}

/* HW reset for FOCALTECH FT5316 chip */
static int ft5316_config_gpio(struct ft5631_ts_info *ts)
{
	int rc = 0;

	TOUCH_PRINTK(1,"delay 1ms after power on.\n");
	msleep(20);
	/* config reset pin to pull up */
    gpio_set_value(ts->gpio_rst, 1);
    
	//time of starting to report point after resetting
	TOUCH_PRINTK(1,"delay 200ms after pulling up reset pin.\n");
	msleep(200); 
    return rc;
}

/* release IRQ and RST GPIO for FOCALTECH FT5316 chip */
static int ft5316_release_gpio(struct ft5631_ts_info *ts)
{
    int rc = 0;
	
	TOUCH_PRINTK(0,"%s, %d\n", __func__, __LINE__);
	gpio_free(ts->gpio_irq);
	gpio_free(ts->gpio_rst);
	gpio_free(ts->gpio_vendor_id);	
	return rc;
}

/* setup IRQ and RST GPIO for FOCALTECH FT5316 chip */
static int ft5316_setup_gpio(struct ft5631_ts_info *ts)
{
    int rc = 0;

    TOUCH_PRINTK(1,"%s\n", __func__);
	/* set up IRQ gpio */
    rc = gpio_request(ts->gpio_irq, "ft5316_ts_irq");
    if (rc) {
    	TOUCH_PRINTK(0, "Failed to request gpio_irq:%d (rc=%d)\n", ts->gpio_irq, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_input(ts->gpio_irq);
    if (rc) {
        TOUCH_PRINTK(0, "Failed to set gpio_irq:%d mode (rc=%d)\n", ts->gpio_irq, rc);
		goto err_gpio_config;
    }
	/* print configure gpio num for debugging */
	TOUCH_PRINTK(0, "Success to configure gpio_irq:%d gpio_name:%s\n",
			ts->gpio_irq, "ft5316_ts_irq");
			
	/* set up RST gpio */		
	rc = gpio_request(ts->gpio_rst, "ft5316_ts_rst");
	if (rc)
    {
        TOUCH_PRINTK(0, "Failed to request gpio_rst:%d (rc=%d)\n", ts->gpio_rst, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_output(ts->gpio_rst, 0);
    if ( rc )
    {
        TOUCH_PRINTK(0, "Failed to set gpio_rst:%d mode (rc=%d)\n", ts->gpio_rst, rc);
        goto err_gpio_config;
    }		
	/* print configure gpio num for debugging */
	TOUCH_PRINTK(0, "Success to configure gpio_rst:%d gpio_name:%s\n",
			ts->gpio_rst, "ft5316_ts_rst");
			
	/* set up vendor id gpio */	
	rc = gpio_request(ts->gpio_vendor_id, "ft5316_ts_id");
    if (rc) {
    	TOUCH_PRINTK(0, "Failed to request gpio_vendor_id:%d (rc=%d)\n", ts->gpio_vendor_id, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_input(ts->gpio_vendor_id);
    if (rc) {
        TOUCH_PRINTK(0, "Failed to set gpio_vendor_id:%d mode (rc=%d)\n", ts->gpio_vendor_id, rc);
		goto err_gpio_config;
    }
	/* print configure gpio num for debugging */
	TOUCH_PRINTK(0, "Success to configure gpio_vendor_id:%d gpio_name:%s\n",
			ts->gpio_vendor_id, "ft5316_ts_vendor_id");
	return 0;
	
err_gpio_config:
	ft5316_release_gpio(ts);
	return rc;
}

/* power on sequence for FocalTech ft5316 touchscreen chip */
static int ft5316_power_on_device(struct ft5631_ts_info *ts, int on)
{
	int rc = 0;
	static int prev_on = 0;

	if (on == prev_on) {
		return 0;
	}

	if(on) {
		/* LVS2 voltage for ft5316 I2C */
		ts->lvs2_regulator = regulator_get(NULL, "touch_i2c");
		if (IS_ERR(ts->lvs2_regulator)) {
			rc = PTR_ERR(ts->lvs2_regulator);
			pr_err("%s:regulator get lvs2 voltage failed rc=%d\n",
							__func__, rc);
			rc = -ENODEV;
			goto exit;
		}
		
		/* L9 voltage for ft5316 AVDD */
		ts->ldo9_regulator = regulator_get(NULL, "touch_avdd");
		if (IS_ERR(ts->ldo9_regulator)) {
			rc = PTR_ERR(ts->ldo9_regulator);
			pr_err("%s:regulator get ldo9 voltage failed rc=%d\n",
							__func__, rc);
			rc = -ENODEV;
			goto get_ldo9_fail;
		}
		
		/* L9 voltage is 2.85V */
		rc = regulator_set_voltage(ts->ldo9_regulator, 2850000, 2850000);
		if (rc) {
			TOUCH_PRINTK(0, "set ldo9_regulator failed, rc=%d\n", rc);
			goto set_ldo9_fail;
		}
	    
		/* LVS2 enable */
		rc = regulator_enable(ts->lvs2_regulator);
		if (rc) {
			TOUCH_PRINTK(0, "enable lvs2_regulator failed, rc=%d\n", rc);
			goto set_ldo9_fail;
		}
		msleep(5);
		
		/* L9 enable */
		rc = regulator_enable(ts->ldo9_regulator);
		if (rc) {
			TOUCH_PRINTK(0, "enable ldo9_regulaotr failed, rc=%d\n", rc);
			goto enable_ldo9_fail;
		}
		msleep(5);
		msleep(100);
		TOUCH_PRINTK(0, "power on device successfully.\n");			
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(ts->pmlog_device);
		if (rc)
			TOUCH_PRINTK(0, "[PM_LOG] Failed to device on . rc = %d\n", rc);
#endif	//CONFIG_PM_LOG
		prev_on = on;
		return 0;	/* Successful, do return here. */
	} else {
		TOUCH_PRINTK(0, "Power off device. %s, line=#%d \n", __func__, __LINE__);
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_off(ts->pmlog_device);
		if (rc)
			TOUCH_PRINTK(0, "[PM_LOG] Failed to device off. rc = %d\n", rc);
#endif //CONFIG_PM_LOG
		prev_on = on;
	}

/* Normal off sequence, also used if any errors */
	regulator_disable(ts->ldo9_regulator);
enable_ldo9_fail:
	regulator_disable(ts->lvs2_regulator);
set_ldo9_fail:
	regulator_put(ts->ldo9_regulator);
get_ldo9_fail:
	regulator_put(ts->lvs2_regulator);
exit:
	return rc;
}

/* ************************************************************************
 * Description: CREATE_KERNEL_DEBUGLEVEL
 * ************************************************************************ */
static void touchpad_create_kernel_debuglevel(void)
{
	TOUCH_PRINTK(1, "create kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir!=NULL) {
		debugfs_create_u32("ts_flow_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&TOUCH_DLL));
		debugfs_create_u32("ts_event_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&TOUCH_EVENT_DLL));
		debugfs_create_u32("key_event_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&KEY_EVENT_DLL));	
	} else {
		printk(KERN_ERR "failed to create SYNAPTICS mXT224E touch dll in kernel_debuglevel_dir!!!\n");
	}

}

/* **************************************************************************
 * Description: DESTROY_KERNEL_DEBUGLEVEL
 * ************************************************************************ */
static void touchpad_destroy_kernel_debuglevel(void)
{
	TOUCH_PRINTK(0, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}

static int proc_detect_tp_chip_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_ts->client->addr);
}

static int proc_tp_config_id_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_ts->vendor_id_val);
}

static int proc_tp_product_id_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_ts->vendor_id_val);
}

static int proc_tp_module_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", gpio_get_value(g_ts->gpio_vendor_id));
}

/* *************************************************************************
 * Description: PROC_FS FOR FTD TEST TO GET TP_CHIP,CONFIG_ID,PRODUCT_ID, MODULE
 * ************************************************************************* */
static int touchpad_create_procfs_entries(struct i2c_client *client)
{
	int ret=0;

	proc_entry = proc_mkdir("touchinfo", NULL);
	
	if (proc_entry == NULL) {
		pr_err("failed to create detect_panel entry\n");
		return -EPERM;
	} else {
		// create tp_chip
		proc1_entry = create_proc_entry("tp_chip", S_IRUGO, proc_entry);
		if (proc1_entry == NULL) {
			pr_err("failed to create detect_panel entry\n");
			return -EPERM;
		}	
		proc1_entry->read_proc = proc_detect_tp_chip_read;
		// create tp_config_id
		proc2_entry = create_proc_entry("tp_config_id", S_IRUGO, proc_entry);
		if (proc2_entry == NULL) {
			pr_err("failed to create tp_config_id entry\n");
			return -EPERM;
		}	
		proc2_entry->read_proc = proc_tp_config_id_read;
		// create tp_product_id
		proc3_entry = create_proc_entry("tp_product_id", S_IRUGO, proc_entry);
		if (proc3_entry == NULL) {
			pr_err("failed to create tp_product_id entry\n");
			return -EPERM;
		}	
		proc3_entry->read_proc = proc_tp_product_id_read;
		// create tp_module
		proc4_entry = create_proc_entry("tp_module", S_IRUGO, proc_entry);
		if (proc4_entry == NULL) {
			pr_err("failed to create tp_module entry\n");
			return -EPERM;
		}	
		proc4_entry->read_proc = proc_tp_module_read;
	}
					  

	return ret;
}
static int __devinit ft5316_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int	rst = 0;
	int i;
	struct	focaltech_tp_platform_data_t *pdata;
	struct  ft5631_ts_info *ts;
	
	TOUCH_PRINTK(0,"%s\n",__func__);
	/* allocate and clear memory */
	ts = kzalloc(sizeof(struct ft5631_ts_info), GFP_KERNEL);
    if(!ts) {
        rst = -ENOMEM;
        return rst;
    }
    pdata = client->dev.platform_data;
    ts->gpio_irq = pdata->gpio_irq;
    ts->gpio_rst = pdata->gpio_rst;
	ts->gpio_vendor_id = pdata->gpio_vendor_id;
    ts->irq = MSM_GPIO_TO_INT(ts->gpio_irq);
    ts->client = client;
    mutex_init(&ts->mutex);	
#ifdef CONFIG_PM_LOG
	/* Register pm log */
	ts->pmlog_device = pmlog_register_device(&client->dev);
	TOUCH_PRINTK(0, "[PM_LOG]register pm log for touch driver.\n");
#endif //CONFIG_PM_LOG
	
	/* create debugfs */
	//touchpad_create_kernel_debuglevel();
	
	/* set up gpio */
    rst = ft5316_setup_gpio(ts);
    if(rst) {
        TOUCH_PRINTK( 0, "Failed to setup gpio. (rst=%d)\n", rst);
		goto err_alloc_mem;
    }
	TOUCH_PRINTK(1,"set hw reset low, delay 5ms\n");
	msleep(5);
	
	/* power on sequence */
    rst = ft5316_power_on_device(ts, 1);
    if(rst) {
    	TOUCH_PRINTK(0, "Unable to power on device. (rst=%d)\n", rst);
		goto err_setup_gpio;
    }
	
	/* HW reset for FacalTech ft5316 chip */
    rst = ft5316_config_gpio(ts);
    if(rst) {
        TOUCH_PRINTK(0, "Failed to HW reset. (rst=%d)\n", rst);
		goto err_power_device;
    }
	
	/* raed register datas of the focaltech ft5316 chip */
	rst = ts_detect_ft5316(ts);
    if(rst) {
		TOUCH_PRINTK(0, "Failed to read register data. (rst=%d)\n", rst);
		goto err_power_device;
    } else {
		/* create debugfs */
		touchpad_create_kernel_debuglevel();
	}	
	client->driver = &i2c_touchpad_driver;
	i2c_set_clientdata(client, ts);
	
	/* Create FocalTech FT5316 touch input device and register it. */
    rst = ft5316_touch_register_input(&ts->input, pdata, client);
    if(rst) {
    	TOUCH_PRINTK(0, "Failed to register touch input device for FocalTech ft5316 chip. (rst=%d)\n", rst);
		goto err_power_device;
    }
    input_set_drvdata(ts->input, ts);
	
	/* Create FocalTech FT5316 key input device and register it. */
	rst = ft5316_keyarray_register_input(&ts->keyarray_input, client);
    if(rst) {
        TOUCH_PRINTK(0, "Failed to register keyarray input device for FocalTech ft5316 chip. (rst=%d)\n", rst);
        goto err_register_touch_input;
    }
    input_set_drvdata(ts->keyarray_input, ts);
	
	/* CHECK IF HW ID and Project ID to get 3 keys coordinates correctly */
	if ((msm_project_id <= BOSTON) && (system_rev <= EVT0)) {
		ts->back_key_x = 90;
		ts->home_key_x = 270;
		ts->menu_key_x = 450;
		ts->keys_Y = 1008;
	} else if ((msm_project_id == BOSTON) && (system_rev >= EVT1)) {
		ts->back_key_x = 90;
		ts->home_key_x = 270;
		ts->menu_key_x = 450;
		ts->keys_Y = 1344;
	} else {
		ts->back_key_x = 90;
		ts->home_key_x = 270;
		ts->menu_key_x = 450;
		ts->keys_Y = 1344;
	}
	TOUCH_PRINTK(0, "back_key_x=%d\n", ts->back_key_x);
	TOUCH_PRINTK(0, "home_key_x=%d\n", ts->home_key_x);
	TOUCH_PRINTK(0, "menu_key_x=%d\n", ts->menu_key_x);
	TOUCH_PRINTK(0, "3 keys_y=%d\n", ts->keys_Y);
	/* request a irq threaded for Synaptics S3202 */
	if (ts->fw_id_val <= 0x2) {
		rst = request_threaded_irq(ts->irq, NULL, ft5316_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ft5316_ts_irq", ts);
	} else {
		rst = request_threaded_irq(ts->irq, NULL, ft5316_irq_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ft5316_ts_irq", ts);
	}		
	if (rst < 0) {
		TOUCH_PRINTK(0, "Failed to request irq:%d. (rst=%d)\n", ts->irq, rst);
		goto err_register_keyarray_input;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->ts_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	//ts->ts_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1;
	ts->ts_early_suspend.suspend = ft5316_ts_early_suspend;
	ts->ts_early_suspend.resume = ft5316_ts_late_resume;
	register_early_suspend(&ts->ts_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */	
	
	g_ts = ts;
	// add ftd mode
	rst = touchpad_create_sys_entries(client);
	touchpad_create_debugfs_entries();
	
	//init z value
	for (i = 0 ; i < MAX_TS_REPORT_POINTS ; i++)
	{
		g_ts->msg[i].switched = 0;
		g_ts->msg[i].coord.pre_state = TS_PUT_UP;
		g_ts->msg[i].coord.state = TS_PUT_UP;
		g_ts->msg[i].coord.z = -1;
	}
		
	/* Install the proc_fs entries */
	rst = touchpad_create_procfs_entries(client);
	TOUCH_PRINTK(0, "Start Probe %s\n",
		(rst < 0) ? "FAIL" : "PASS");
	
	return 0;
	
err_register_keyarray_input:
    input_unregister_device(ts->keyarray_input);
    input_free_device(ts->keyarray_input);
    ts->keyarray_input = NULL;	
err_register_touch_input:
    input_unregister_device(ts->input);
    input_free_device(ts->input);
    ts->input = NULL;	
err_power_device:
	ft5316_power_on_device(ts, 0);
err_setup_gpio:
	ft5316_release_gpio(ts);		
err_alloc_mem:
	kfree(ts);
	rst = -EFAULT;
	return rst;
}

/* unregistere device */
static int __devexit ft5316_remove(struct i2c_client *client)
{
	TOUCH_PRINTK(0, "+++ %s +++\n", __func__);
	
	/* clientdata registered on probe */
	g_ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&g_ts->ts_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
	/* Start cleaning up by removing any delayed work and the timer */
	free_irq(g_ts->irq, g_ts);
	input_unregister_device(g_ts->keyarray_input);
    input_free_device(g_ts->keyarray_input);
    g_ts->keyarray_input = NULL;
	input_unregister_device(g_ts->input);
    input_free_device(g_ts->input);
    g_ts->input = NULL;
	ft5316_release_gpio(g_ts);
	touchpad_destroy_kernel_debuglevel();
	mutex_destroy(&g_ts->mutex); 
	g_ts = NULL;
	kfree(g_ts);
	
	TOUCH_PRINTK(0, "--- %s ---\n", __func__);
	return 0;
}

static const struct i2c_device_id ft5316_id_table[] = {
	{ FOCALTECH_TP_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ft5316_id_table);

static struct i2c_driver i2c_touchpad_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = FOCALTECH_TP_NAME,
	},
	.probe	 = ft5316_probe,
	.remove	 = ft5316_remove,

#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ft5316_ts_suspend,
	.resume  = ft5316_ts_resume,
#endif
	.id_table = ft5316_id_table,
};

static int __init ft5316_init(void)
{
	int rc = 0;
	
	TOUCH_PRINTK(0, "BootLog, %s +\n", __func__);
	TOUCH_PRINTK(0, "FocalTech ft5316 Touch Driver (Built %s @ %s)\n",
		__DATE__, __TIME__);
	//TOUCH_PRINTK(0, "%s, system_rev=0x%x\n", __func__, system_rev);
	//TOUCH_PRINTK(1, "%s, msm960_project_id=0x%x\n", __func__, msm8960_project_id);
	if(touch_panel_detected) {
		//detect focaltech IC failed, do nothing
	} else {	
		/*init synaptics IC first, if detect pass then don't init focaltech IC, if detect failed then init focaltech IC  */
		i2c_touchpad_driver.driver.name = FOCALTECH_TP_NAME;
		rc = i2c_add_driver(&i2c_touchpad_driver);
	}
	TOUCH_PRINTK(0, "BootLog, %s -, rc=%d\n", __func__,rc);
    return rc;
}
module_init(ft5316_init);

static void __exit ft5316_exit(void)
{
    i2c_del_driver(&i2c_touchpad_driver);
    TOUCH_PRINTK(0, "FocalTech ft5316 touch driver Exiting\n");
}
module_exit(ft5316_exit);

MODULE_DESCRIPTION("FOCALTECH FT5316 touchpad driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Emily Jiang");
MODULE_ALIAS("platform:FOCALTECH_FT5316_touch");
