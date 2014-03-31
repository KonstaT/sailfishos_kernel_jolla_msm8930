/* drivers/input/touchscreen/synaptics_s3202_touch.c
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

// 20111213
// Emily Jiang
// New synaptics_s3202_touch.c

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
#include <linux/input/synaptics_s3202_touch.h>
#include <linux/input/synaptics_s3202_touch_DS4_3_2.h>
#include <linux/input/synaptics_s3202_touch_DS4_3_6.h>
#include "synaptics_s3202_config_30303035.h"
#include "synaptics_s3202_config_tpk_black_30303036.h"
#include "synaptics_s3202_firmware_1200567.h"
#include "synaptics_s3202_firmware_1296077.h"
#include "synaptics_s3202_config_tpk_white_30313031.h"
#include "synaptics_s3202_config_tpk_white_30323031.h"
#include "synaptics_s3202_config_JTP_black_31303031.h"
#include "synaptics_s3202_config_JTP_white_31313031.h"
#include "synaptics_s3202_config_JTP_black_31303032.h"
#include "synaptics_s3202_config_JTP_white_31313032.h"
#include "synaptics_s3202_firmware_1365481.h"
#include "synaptics_s3202_config_OF_black_4F423230.h"
#include "synaptics_s3202_config_OF_black_4F423330.h"
#include "synaptics_s3202_config_OF_black_4F423430.h"
#include "synaptics_s3202_firmware_1421304.h"
#include "synaptics_s3202_config_OF_black_4F423530.h"
#include "synaptics_s3202_config_OF_black_4F423630.h"
#include <linux/debugfs.h>
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif //CONFIG_PM_LOG
//#define ECHOSTR_SIZE   30
//#define SPEED_DISABLETOUCH_WHEN_PSENSORACTIVE 1
#define SYNAPTICS_POR_DELAY         100 //Dealy after Power-On Reset
#define MAX_TOUCH_MAJOR		15
#include <mach/hwid.h>
#include <linux/proc_fs.h>
/* Emily jiang, abort suspending procedure if power key event comes in during freezing. 20140218 */
#include <linux/wakelock.h>

//DEBUG_LEVEL
extern struct dentry *kernel_debuglevel_dir;
static struct proc_dir_entry *proc_entry;
static struct proc_dir_entry *proc1_entry;
static struct proc_dir_entry *proc2_entry;
static struct proc_dir_entry *proc3_entry;
static struct proc_dir_entry *proc4_entry;
static unsigned int SYNAPTICS_DLL=0;
static unsigned int SYNAPTICS_TOUCH_REPORT_DLL=0;
static unsigned int SYNAPTICS_CAPKEY_REPORT_DLL=0;
#define SYNAPTICS_ERR_LEVEL   0
#define SYNAPTICS_FLOW_INFO_LEVEL  1
#define SYNAPTICS_TOUCH_REPORT_LEVEL 1
#define SYNAPTICS_CAPKEY_REPORT_LEVEL 1
#define SYNAPTICS_OTHER_LEVEL 2
#define SYNAPTICS_PRINTK(level, args...) if(level <= SYNAPTICS_DLL) printk("[TP] " args);
#define SYNAPTICS_TOUCH_PRINTK(level, args...) if(level <= SYNAPTICS_TOUCH_REPORT_DLL) printk("[TP] " args);
#define SYNAPTICS_CAPKEY_PRINTK(level, args...) if(level <= SYNAPTICS_CAPKEY_REPORT_DLL) printk("[KEY] " args);
#define IS_UNDER_TESTING 0

struct synaptics_tp_t {
    struct i2c_client        *client;
    struct input_dev         *input;
	struct input_dev         *keyarray_input;
	int                      irq; //ISR number
    int                      gpio_irq; //GPIO number
    int                      gpio_rst; //GPIO reset
    int                      open_count; //input device open count
	int                      keyarray_open_count;
	uint                  sensor_max_x;
	uint                  sensor_max_y;
	struct F11_2D_data_t     f11_2d_data;
	int is_earlysuspended;
	int is_suspended;
    struct  mutex            mutex;
	/*    Power Rails         */
	struct regulator *ldo9_regulator; //VDD (VREG_L9_2P85)
	struct regulator *lvs2_regulator; //VLOGIC (VREG_LVS2_1P8)
	uint8_t			current_page;
	struct synaptics_touch_point_status_t msg[MAX_TS_REPORT_POINTS];
	struct synaptics_cap_key_point_status_t key_msg[MAX_KEY_REPORT_POINTS];
#ifdef CONFIG_PM_LOG
	struct pmlog_device *pmlog_device;
#endif 	//CONFIG_PM_LOG
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ts_early_suspend;
#endif /* CONFIG_HAS_EARLYSUSPEND */
	/* add debugfs for ftd test to get device id */
	struct dentry   *dent;
	uint32_t fw_version;
	uint32_t config_id;
	/* Page Description Table (PDT) */
	struct synaptics_pdt_register_map_t pdt_map;
	int number_of_rx_electrodes;
	int number_of_tx_electrodes;
	/* F54 self test */
	int F54_testing_flag;
	uint8_t report_type;
	int report_size;
	uint8_t raw_cap_value[F54_RAW_CAPACITANCE_READ_BYTES];
	uint16_t raw_cap[NUM_OF_TX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint16_t rx_rx_imagearray_7[NUM_OF_TX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint16_t rx_rx_imagearray_17[NUM_OF_RX_TX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint16_t rx_rx_imagearray[NUM_OF_RX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint8_t report_type7[RX_RX_7_SIZE];
	uint8_t report_type17[RX_RX_17_SIZE];
	int f54_raw_cap_min_limit;
	int f54_raw_cap_max_limit;
	/* update FW ver and configuration file*/
	uint16_t f34_bootload_id;
	uint16_t config_block_size;
	uint16_t config_block_count;
	uint16_t config_image_size;
	uint16_t firmware_block_size;
	uint16_t firmware_block_count;
	uint16_t firmware_image_size;
	uint8_t  bootloader_version;
	int program_enable_success;
	/* update config file */
	unsigned char *puData;
	/* update firmware binnary code */
	unsigned char *puFirmwareData;
	uint8_t gpio_vendor_id; //vendor id
	/*add for double tap gesture, 20130801 */
	wait_queue_head_t wq;
	/* add a new product id to distinguish between old and new ito film for OFilm tp in EVT2, 20130910 */
	bool is_new_prodid_flag;
    /* add sysfs API for enable double tap feature, 20131018 */
	unsigned int dtap_allowed;
	int irq_pending;
	/* Emily jiang, abort suspending procedure if power key event comes in during freezing. 20140218 */
	struct wake_lock doubletap_wakelock;
};

/*init synaptics IC first, if detect pass then don't init focaltech IC, if detect failed then init focaltech IC  */
int touch_panel_detected = 0; //dtect touch IC flag
static struct synaptics_tp_t         *g_tp;
static int __devinit touchpad_probe(struct i2c_client *client,  const struct i2c_device_id *);
static int touchpad_setup_gpio(struct synaptics_tp_t *g_tp);
static int __devexit touchpad_remove(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_early_suspend(struct early_suspend *handler);
static void synaptics_late_resume(struct early_suspend *handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */
/*add for double tap gesture, 20130801 */
static int synaptics_suspend(struct device *dev);

#define TOUCH_RETRY_COUNT 5
static int touchpad_write_i2c(struct i2c_client *client,
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
        SYNAPTICS_PRINTK(0, "alloc memory failed\n");
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
            SYNAPTICS_PRINTK(0, "write %Xh %d bytes return failure, %d\n", buf[0], dataLen, result);
            if(-ETIMEDOUT == result) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        SYNAPTICS_PRINTK(0, "write %Xh %d bytes retry at %d\n", buf[0], dataLen, TOUCH_RETRY_COUNT-retryCnt);

    kfree( buf );
    return result;
}

static int touchpad_read_i2c(struct i2c_client *client,
                        uint16_t           regBuf,
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
            SYNAPTICS_PRINTK(0, "read %Xh %d bytes return failure, %d\n", regBuf, dataLen, result );
            //return result;
            if( -ETIMEDOUT == result ) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        SYNAPTICS_PRINTK(0, "read %Xh %d bytes retry at %d\n", regBuf, dataLen, TOUCH_RETRY_COUNT-retryCnt);

    return result;
}

static void touchpad_set_dtap_mode(int enable)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	u8 f11_2D_ctrl0;

	if (enable)
		f11_2D_ctrl0 = 0xc; /* double tap mode */
	else
		f11_2D_ctrl0 = 0x8; /* disable double tap mode */

	rc = touchpad_write_i2c(client, g_tp->pdt_map.F11_control_base,
							&f11_2D_ctrl0, 1);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "Failed write to f11_2D_ctrl0[0x%x]:0x%x",
			g_tp->pdt_map.F11_control_base, f11_2D_ctrl0);
		SYNAPTICS_PRINTK(0, "Error code rc=%d\n", rc);
		return;
	}
}

/* Last set sleep mode; 0x00 = Active mode is the default after reset */
static u8 touchpad_sleep_mode = 0x00;

/* Flag for: generate touch release events on resume */
static u8 generate_touch_release = 0;

static void touchpad_set_sleep_mode(int sleep)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	u8 sleep_mode;

	if (sleep)
		sleep_mode = 0x01; /* Sleep mode */
	else
		sleep_mode = 0x00; /* Active mode */

	if( touchpad_sleep_mode == sleep_mode ) {
		SYNAPTICS_PRINTK(0, "sleep mode %d already set\n", sleep_mode);
		return;
	}

	SYNAPTICS_PRINTK(0, "setting sleep mode %d\n", sleep_mode);

	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base,
							&sleep_mode, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "Write fail [0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_control_base, sleep_mode, rc);
		return;
	}

	if (sleep_mode == 0x01)
		generate_touch_release = 1;

	touchpad_sleep_mode = sleep_mode;
}

/* Disabled cap key feature, 20131018 */
#if 0
static void touchpad_report_capkey(struct synaptics_tp_t *g_tp,uint8_t F1A_0D_data)
{
	
	if (F1A_0D_data & 0x1) {
		g_tp->key_msg[0].key_state = KEY_PRESS;
		SYNAPTICS_CAPKEY_PRINTK(1," KEY_BACK[%d] is pressed\n",KEY_BACK);
		input_report_key(g_tp->keyarray_input, KEY_BACK, 1);
	} else {
		g_tp->key_msg[0].key_state = KEY_RELEASE;
		SYNAPTICS_CAPKEY_PRINTK(1," KEY_BACK[%d] is released\n",KEY_BACK);
		input_report_key(g_tp->keyarray_input, KEY_BACK, 0);
	}
	if (F1A_0D_data & 0x2) {
		g_tp->key_msg[0].key_state = KEY_PRESS;
		SYNAPTICS_CAPKEY_PRINTK(1," KEY_HOMEPAGE[%d] is pressed\n",KEY_HOMEPAGE);
		input_report_key(g_tp->keyarray_input, KEY_HOMEPAGE, 1);
	} else {
		g_tp->key_msg[0].key_state = KEY_RELEASE;
		SYNAPTICS_CAPKEY_PRINTK(1," KEY_HOMEPAGE[%d] is released\n",KEY_HOMEPAGE);
		input_report_key(g_tp->keyarray_input, KEY_HOMEPAGE, 0);
	}
	if (F1A_0D_data & 0x4) {
		g_tp->key_msg[0].key_state = KEY_PRESS;
		SYNAPTICS_CAPKEY_PRINTK(1," KEY_MENU[%d] is pressed\n",KEY_MENU);
		input_report_key(g_tp->keyarray_input, KEY_MENU, 1);
	} else {
		g_tp->key_msg[0].key_state = KEY_RELEASE;
		SYNAPTICS_CAPKEY_PRINTK(1," KEY_MENU[%d] is released\n",KEY_MENU);
		input_report_key(g_tp->keyarray_input, KEY_MENU, 0);
	}
	input_sync(g_tp->keyarray_input);
	return;
}
#endif

static void touchpad_report_mt_protocol(struct synaptics_tp_t *g_tp)
{
	int i;
	int all_up = 1;
	
	for(i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		if (g_tp->msg[i].coord.z == -1)
			continue;
		if(g_tp->msg[i].prev_state == TS_RELEASE && g_tp->msg[i].state == TS_PRESS) {
			SYNAPTICS_TOUCH_PRINTK(1, "skip <touch: id=%d, (x,y)=(%d,%d), (z,w)=(%d,%d)>\n",
				i , g_tp->msg[i].coord.x, g_tp->msg[i].coord.y, g_tp->msg[i].coord.z, g_tp->msg[i].coord.wxy);
		} else {
			//input_report_key(g_tp->input, BTN_TOUCH, 1);
			//input_report_abs(g_tp->input, ABS_MT_TOUCH_MAJOR, g_tp->msg[i].coord.z);
			//input_report_abs(g_tp->input, ABS_MT_WIDTH_MAJOR, g_tp->msg[i].coord.wxy);
			input_report_abs(g_tp->input, ABS_MT_POSITION_X, g_tp->msg[i].coord.x);
			input_report_abs(g_tp->input, ABS_MT_POSITION_Y, g_tp->msg[i].coord.y);
			//input_report_abs(g_tp->input, ABS_MT_TRACKING_ID, i);
			if(g_tp->msg[i].coord.z == 0)input_report_abs(g_tp->input, ABS_MT_PRESSURE, 1);
			else input_report_abs(g_tp->input, ABS_MT_PRESSURE, g_tp->msg[i].coord.z);
			SYNAPTICS_TOUCH_PRINTK(1, "touch: id=%d, (x,y)=(%d,%d), (z,w)=(%d,%d)\n",
				i , g_tp->msg[i].coord.x, g_tp->msg[i].coord.y, g_tp->msg[i].coord.z, g_tp->msg[i].coord.wxy);
			if(g_tp->msg[i].coord.z != 0) all_up = 0;
			input_mt_sync(g_tp->input);
		}
			if (g_tp->msg[i].coord.z == 0)
				g_tp->msg[i].coord.z = -1;
	}
	input_sync(g_tp->input);
	if(all_up) 
	{
		input_mt_sync(g_tp->input);
		input_sync(g_tp->input);
	}
}

static void touchpad_report_coord(struct synaptics_tp_t *g_tp)
{
	int i;
	uint8_t state;
	
	for (i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		g_tp->msg[i].coord.x = (uint)g_tp->f11_2d_data.x[i];
		g_tp->msg[i].coord.y = (uint)g_tp->f11_2d_data.y[i];
		state = ((g_tp->f11_2d_data.finger_state[i/4] & (0x3 << ((i%4)*2))) >> ((i%4)*2));
		if (state == 0 && g_tp->msg[i].coord.z != -1) {
			g_tp->msg[i].prev_state = g_tp->msg[i].state;
			g_tp->msg[i].state = TS_RELEASE;
			g_tp->msg[i].coord.wxy = (uint)g_tp->f11_2d_data.wxy[i];
			g_tp->msg[i].coord.z = 0;
		} else if(state == 1 || state == 2) {
			g_tp->msg[i].prev_state = g_tp->msg[i].state;
			g_tp->msg[i].state = TS_PRESS;
			g_tp->msg[i].coord.wxy = (uint)g_tp->f11_2d_data.wxy[i];
			g_tp->msg[i].coord.z = (uint)g_tp->f11_2d_data.z[i];
		} else
			if(state) SYNAPTICS_TOUCH_PRINTK(0, "touch:error finger state = 0x%x\n",state);
    }
	touchpad_report_mt_protocol(g_tp);	
}


static irqreturn_t touchpad_irq_thread(int irq, void *dev_id)
{
	struct i2c_client	*client = g_tp->client;
	struct synaptics_tp_t *g_tp = dev_id;
    int rc = 0;
    uint8_t /*page_select,*/inrt_status/*,F1A_0D_data*/;
	/*add for double tap gesture, 20130801 */
	uint8_t F11_2D_data[28],f11_2D_ctrl0;
    mutex_lock(&g_tp->mutex);
	disable_irq_nosync(g_tp->irq);
	/*add for double tap gesture, 20130801 */
	if (g_tp->is_earlysuspended == 1 && g_tp->is_suspended == 0) {
		SYNAPTICS_TOUCH_PRINTK(0, "will send power key event\n");
		/* enable double tap wake up system for reporting mode */
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F11_control_base, &f11_2D_ctrl0, 1);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read f11_2D_ctrl0[0x%x]:0x%x (rc=%d)\n", 
				g_tp->pdt_map.F11_control_base, f11_2D_ctrl0, rc);
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
		SYNAPTICS_PRINTK(1, "read f11_2D_ctrl0[0x%x]:0x%x\n", g_tp->pdt_map.F11_control_base, f11_2D_ctrl0);
		/* read interrupt status */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+1, &inrt_status, 1);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read interrupt status[0x%x]:0x%x (rc=%d)\n", 
				g_tp->pdt_map.F01_data_base+1, inrt_status, rc);
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
		SYNAPTICS_PRINTK(1, "synaptics interrupt status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, inrt_status);
		if (((f11_2D_ctrl0 == 0xc) && ((inrt_status & 0x4) == 0x4)) && g_tp->dtap_allowed) {
			SYNAPTICS_PRINTK(1, "enter double tap wake up system feature\n");
			//report power key event to notify power manager service to wake up system
			input_report_key(g_tp->keyarray_input, KEY_POWER, 1);
			input_sync(g_tp->keyarray_input);
			input_report_key(g_tp->keyarray_input, KEY_POWER, 0);
			input_sync(g_tp->keyarray_input);
			SYNAPTICS_PRINTK(0, "sent power key event\n");
			/* Emily jiang, abort suspending procedure if power key event comes in during freezing. 20140218 */
			wake_lock_timeout(&g_tp->doubletap_wakelock, 2 * HZ);
		}
	} else if (g_tp->is_suspended == 1) {
		SYNAPTICS_PRINTK(0, "IST run when is_suspended = 1\n");
		g_tp->irq_pending = 1;
		mutex_unlock(&g_tp->mutex);
		return IRQ_HANDLED;
	} else { 
		/* read interrupt status */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+1, &inrt_status, 1);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read interrupt status[0x%x]:0x%x (rc=%d)\n", 
				g_tp->pdt_map.F01_data_base, inrt_status, rc);
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
		SYNAPTICS_PRINTK(1, "synaptics interrupt status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, inrt_status);
				/* read F11 2D data */
			rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+2, &F11_2D_data[0], 28);
			if (rc < 0) {
				SYNAPTICS_PRINTK(0, "Failed to read finger state[0x%x]:0x%x (rc=%d)\n", 
					g_tp->pdt_map.F01_data_base+2, F11_2D_data[0], rc);
				enable_irq(g_tp->irq);
				mutex_unlock(&g_tp->mutex);
				return IRQ_HANDLED;
			}
			/* read finger state & read x/y/w/z values */
			g_tp->f11_2d_data.finger_state[0] = F11_2D_data[0];
			g_tp->f11_2d_data.finger_state[1] = F11_2D_data[1];
			SYNAPTICS_PRINTK(1, "finger state[0]:0x%x\n", F11_2D_data[0]);
			SYNAPTICS_PRINTK(1, "finger state[0]:0x%x\n", F11_2D_data[1]);
			g_tp->f11_2d_data.x[0] = (F11_2D_data[3] << 4) | (F11_2D_data[5] & 0x0F);
			g_tp->f11_2d_data.y[0] = (F11_2D_data[4] << 4) | (F11_2D_data[5] & 0xF0) >> 4;
			g_tp->f11_2d_data.wxy[0] = F11_2D_data[6];
			g_tp->f11_2d_data.z[0] = F11_2D_data[7];
			g_tp->f11_2d_data.x[1] = (F11_2D_data[8] << 4) | (F11_2D_data[10] & 0x0F);
			g_tp->f11_2d_data.y[1] = (F11_2D_data[9] << 4) | (F11_2D_data[10] & 0xF0) >> 4;
			g_tp->f11_2d_data.wxy[1] = F11_2D_data[11];
			g_tp->f11_2d_data.z[1] = F11_2D_data[12];
			g_tp->f11_2d_data.x[2] = (F11_2D_data[13] << 4) | (F11_2D_data[15] & 0x0F);
			g_tp->f11_2d_data.y[2] = (F11_2D_data[14] << 4) | (F11_2D_data[15] & 0xF0) >> 4;
			g_tp->f11_2d_data.wxy[2] = F11_2D_data[16];
			g_tp->f11_2d_data.z[2] = F11_2D_data[17];
			g_tp->f11_2d_data.x[3] = (F11_2D_data[18] << 4) | (F11_2D_data[20] & 0x0F);
			g_tp->f11_2d_data.y[3] = (F11_2D_data[19] << 4) | (F11_2D_data[20] & 0xF0) >> 4;
			g_tp->f11_2d_data.wxy[3] = F11_2D_data[21];
			g_tp->f11_2d_data.z[3] = F11_2D_data[22];
			g_tp->f11_2d_data.x[4] = (F11_2D_data[23] << 4) | (F11_2D_data[25] & 0x0F);
			g_tp->f11_2d_data.y[4] = (F11_2D_data[24] << 4) | (F11_2D_data[25] & 0xF0) >> 4;
			g_tp->f11_2d_data.wxy[4] = F11_2D_data[26];
			g_tp->f11_2d_data.z[4] = F11_2D_data[27];
		
		if (inrt_status == 0x4) {
			touchpad_report_coord(g_tp);
		} else if (inrt_status == 0x20 || inrt_status == 0x10) {
/* Disabled cap key feature, 20131018 */
#if 0
			page_select = 0x02;
			rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
					PAGE_SELECT_REGISTER, page_select, rc);
				enable_irq(g_tp->irq);
				mutex_unlock(&g_tp->mutex);
				return IRQ_HANDLED;
			}
			rc = touchpad_read_i2c(client, g_tp->pdt_map.F1A_data_base, &F1A_0D_data, sizeof(F1A_0D_data));
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to read synaptics F1A_0D_data[0x%x]:0x%x (rc=%d)\n",
					g_tp->pdt_map.F1A_data_base, F1A_0D_data, rc);
				enable_irq(g_tp->irq);
				mutex_unlock(&g_tp->mutex);
				return IRQ_HANDLED;
			}
			touchpad_report_capkey(g_tp,F1A_0D_data);
			page_select = 0x00;
			rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
					PAGE_SELECT_REGISTER, page_select, rc);
				enable_irq(g_tp->irq);
				mutex_unlock(&g_tp->mutex);
				return IRQ_HANDLED;
			}
#endif
		} else {
				/* TODO */
		}
    }
	enable_irq(g_tp->irq);
	mutex_unlock(&g_tp->mutex);
    return IRQ_HANDLED;
}

static int touchpad_keyarray_open(struct input_dev *dev)
{
	int rc = 0;
    
	SYNAPTICS_PRINTK(1, "open keyarray input class\n");
    mutex_lock(&g_tp->mutex);
    if(g_tp->keyarray_open_count == 0) {
        g_tp->keyarray_open_count++; //record opencount
        SYNAPTICS_PRINTK(1, "keyarray opened %d times\n", g_tp->keyarray_open_count);      
    }
    mutex_unlock(&g_tp->mutex);
    
    return rc;
}

static void touchpad_keyarray_close(struct input_dev *dev)
{
	SYNAPTICS_PRINTK(1,"close keyarray input class\n");
	mutex_lock(&g_tp->mutex);
    if(g_tp->keyarray_open_count > 0) {
        g_tp->keyarray_open_count--;
        SYNAPTICS_PRINTK(1, "still opened keyarray %d times\n", g_tp->keyarray_open_count);        
    }
    mutex_unlock(&g_tp->mutex);
}

static int keyarray_register_input( struct input_dev **input,
                              struct i2c_client *client )
{
	int rc = 0;
	struct input_dev *input_dev;
  
	input_dev = input_allocate_device();
	if (!input_dev)
	{
		rc = -ENOMEM;
		return rc;
	}
	input_dev->name = SYNAPTICS_CAPKEY_NAME;
	input_dev->phys = "synaptcis_s3202_capkey/event0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &client->dev;
	input_dev->open = touchpad_keyarray_open;
	input_dev->close = touchpad_keyarray_close;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	/* Remove check hw id and project id, we don't need to check that in sailfish. 20130820*/
	//if ( msm_project_id <= EGYPT && system_rev >= EVT0) {
		set_bit(KEY_BACK, input_dev->keybit);
		set_bit(KEY_HOMEPAGE, input_dev->keybit);
		set_bit(KEY_MENU, input_dev->keybit);
	//} else {
		/*add for double tap gesture, 20130801 */
		set_bit(KEY_POWER, input_dev->keybit);
	//}
	
	SYNAPTICS_PRINTK(1, "%s: Register input device\n", SYNAPTICS_CAPKEY_NAME);
	rc = input_register_device(input_dev);
	if (rc) {
		SYNAPTICS_PRINTK(0, "%s: Failed to register keyarray input device\n", SYNAPTICS_CAPKEY_NAME);
		input_free_device(input_dev);
	}
	else {
		*input = input_dev;
	}
  
  return rc;
}

static int touchpad_open(struct input_dev *dev)
{
    int rc = 0;
    
    SYNAPTICS_PRINTK(1, "open touch input class\n");
    mutex_lock(&g_tp->mutex);
    if(g_tp->open_count == 0)
    {
        g_tp->open_count++;
        SYNAPTICS_PRINTK(1, "opened touch %d times\n", g_tp->open_count);      
    }
    mutex_unlock(&g_tp->mutex);
    
    return rc;
}

static void touchpad_close(struct input_dev *dev)
{
    SYNAPTICS_PRINTK(1, "close touch input class\n");
    mutex_lock(&g_tp->mutex);
    if(g_tp->open_count > 0)
    {
        g_tp->open_count--;
        SYNAPTICS_PRINTK(1, "still opened touch %d times\n", g_tp->open_count);        
    }
    mutex_unlock(&g_tp->mutex);
}

/* register mXT224E touch input device */
static int touchpad_register_input( struct input_dev **input,
                                    struct synaptics_tp_platform_data_t *pdata,
                                    struct i2c_client *client )
{
    int rc = 0;
    struct input_dev *input_dev;
    int i;
    uint8_t page_select;
	uint8_t value[4];
    i = 0;
	
    input_dev = input_allocate_device();
    if ( !input_dev ) {
        rc = -ENOMEM;
        return rc;
    }
    input_dev->name = SYNAPTICS_TP_NAME;
    input_dev->phys = "synaptcis_s3202_ts/input0";
    input_dev->id.bustype = BUS_I2C;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0002;
    input_dev->id.version = 0x0100;
    input_dev->dev.parent = &client->dev;
    input_dev->open = touchpad_open;
    input_dev->close = touchpad_close;
    //input_dev->event = touchpad_event;
    
    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    //input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
    set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit); //fix the input issue JB only
	
	/* set page select to 0x00 */
	page_select = 0x00;
	rc = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "synaptics page table addr[0x%x]:0x%x\n", PAGE_SELECT_REGISTER, page_select);
	msleep(1);
	
	/* config register map dynamiclly */
	/* read max x/y position */
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F11_control_base+6, &value[0], 4);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read MAX_X_POSITION[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F11_control_base+6, value[0], rc);
		return rc;
	}
	g_tp->sensor_max_x = (value[1] & 0x0F) << 8 | (value[0] & 0xFF);
	g_tp->sensor_max_y = (value[3] & 0x0F) << 8 | (value[2] & 0xFF);
	SYNAPTICS_PRINTK(1, "g_tp->sensor_max_x:%d\n", (int)g_tp->sensor_max_x);
	SYNAPTICS_PRINTK(1, "g_tp->sensor_max_y:%d\n", (int)g_tp->sensor_max_y);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, (int)g_tp->sensor_max_x, 0, 0); 
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, (int)g_tp->sensor_max_y, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,  0, MAX_TOUCH_MAJOR, 0, 0); 
	//input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,  0, 255, 0, 0); 
	//input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	
	SYNAPTICS_PRINTK(1,"%s: Register input device\n", SYNAPTICS_TP_NAME);
    rc = input_register_device(input_dev);
    if (rc) {
        SYNAPTICS_PRINTK(0, "%s: Failed to register input device\n", SYNAPTICS_TP_NAME);
        input_free_device(input_dev);
    }else {
        *input = input_dev;
    }
    
    return rc;
}

/* constructRMI
 * Construct the RMI register map based on the Page Description
 * Tables. This will initialize the register address arrays and
 * interrupt masks
 *
 * First, finds Function 01 addresses to build 
 * interrupt masks, then builds other functions' addresses
 */
static int touchpad_pdtscan(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	int ret = 0;
	uint8_t page_select = 0x00;
	uint8_t pdt_properities_value = 0x00;
	uint8_t pdt_addr,pdt_value[PDT_SIZE];
	uint8_t i;
	
	memset(pdt_value,0,PDT_SIZE);
	/* read page Registers of DS4 */
	for (i = 0x00; i < NUM_OF_DS4_PAGES; i++) {
		page_select = i;
		/* write page number to select page */
		rc = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
				PAGE_SELECT_REGISTER, page_select, rc);
			goto out;
		}
		msleep(1);
		
		/* read PDT properties */
		rc = touchpad_read_i2c(g_tp->client, PDT_PROPERTIES_ADDR, &pdt_properities_value, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write synaptics PDT properties[0x%x]=0x%x (rc=%d)\n",
				PDT_PROPERTIES_ADDR, pdt_properities_value, rc);
			goto out;
		}
		SYNAPTICS_PRINTK(1, "touch:PDT properties[0x%x]=0x%x\n", PDT_PROPERTIES_ADDR, pdt_properities_value);
		
		if (pdt_properities_value == 0x40) {
			SYNAPTICS_PRINTK(0,"failed to support Page Description Table\n");
			rc = -EFAULT;
			goto out;
		} else if (pdt_properities_value == 0x00) {
		    SYNAPTICS_PRINTK(1,"touch:pdt properities value = 0x00\n");
			for(pdt_addr = (PDT_PROPERTIES_ADDR-PDT_SIZE); pdt_addr < 0xFF;  pdt_addr-=PDT_SIZE) {  
				/* read function registers map */
				rc = touchpad_read_i2c(g_tp->client, pdt_addr, &pdt_value[0], PDT_SIZE);
				if (rc) {
					SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
						pdt_addr, pdt_value[0], rc);
					goto out;
				}	
				if (page_select == 0x00) {
					if (pdt_value[5] == 0x00) {
						SYNAPTICS_PRINTK(1, "touch:reserved register value[0x%x]:0x%x\n", pdt_addr+5, pdt_value[5]);
						break;
					} else if (pdt_value[5] == PDT_FLASH_F34) {
						g_tp->pdt_map.F34_query_base = pdt_value[0];
						g_tp->pdt_map.F34_command_base = pdt_value[1];
						g_tp->pdt_map.F34_control_base = pdt_value[2];
						g_tp->pdt_map.F34_data_base = pdt_value[3];
						g_tp->pdt_map.F34_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F34_function_exists = pdt_value[5];
						SYNAPTICS_PRINTK(1, "touch:F34_query_base [0x%x]:0x%x\n", pdt_addr, g_tp->pdt_map.F34_query_base);
						SYNAPTICS_PRINTK(1, "touch:F34_command_base [0x%x]:0x%x\n", pdt_addr+1, g_tp->pdt_map.F34_command_base);
						SYNAPTICS_PRINTK(1, "touch:F34_control_base [0x%x]:0x%x\n", pdt_addr+2, g_tp->pdt_map.F34_control_base);
						SYNAPTICS_PRINTK(1, "touch:F34_data_base [0x%x]:0x%x\n", pdt_addr+3, g_tp->pdt_map.F34_data_base);
						SYNAPTICS_PRINTK(1, "touch:F34_version_interrupt_count [0x%x]:0x%x\n", pdt_addr+4, g_tp->pdt_map.F34_version_interrupt_count);
						SYNAPTICS_PRINTK(1, "touch:F34_function_exists [0x%x]:0x%x\n", pdt_addr+5, g_tp->pdt_map.F34_function_exists);
					} else if (pdt_value[5] == PDT_RMI_F01) {
						g_tp->pdt_map.F01_query_base = pdt_value[0];
						g_tp->pdt_map.F01_command_base = pdt_value[1];
						g_tp->pdt_map.F01_control_base = pdt_value[2];
						g_tp->pdt_map.F01_data_base = pdt_value[3];
						g_tp->pdt_map.F01_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F01_function_exists = pdt_value[5];
						SYNAPTICS_PRINTK(1, "touch:F01_query_base [0x%x]:0x%x\n", pdt_addr, g_tp->pdt_map.F01_query_base);
						SYNAPTICS_PRINTK(1, "touch:F01_command_base [0x%x]:0x%x\n", pdt_addr+1, g_tp->pdt_map.F01_command_base);
						SYNAPTICS_PRINTK(1, "touch:F01_control_base [0x%x]:0x%x\n", pdt_addr+2, g_tp->pdt_map.F01_control_base );
						SYNAPTICS_PRINTK(1, "touch:F01_data_base [0x%x]:0x%x\n", pdt_addr+3, g_tp->pdt_map.F01_data_base);
						SYNAPTICS_PRINTK(1, "touch:F01_version_interrupt_count [0x%x]:0x%x\n", pdt_addr+4, g_tp->pdt_map.F01_version_interrupt_count);
						SYNAPTICS_PRINTK(1, "touch:F01_function_exists [0x%x]:0x%x\n", pdt_addr+5, g_tp->pdt_map.F01_function_exists);
					} else if (pdt_value[5] == PDT_2D_DATA_F11) {
						g_tp->pdt_map.F11_query_base = pdt_value[0];
						g_tp->pdt_map.F11_command_base = pdt_value[1];
						g_tp->pdt_map.F11_control_base = pdt_value[2];
						g_tp->pdt_map.F11_data_base = pdt_value[3];
						g_tp->pdt_map.F11_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F11_function_exists = pdt_value[5];
						SYNAPTICS_PRINTK(1, "touch:F11_query_base [0x%x]:0x%x\n", pdt_addr, g_tp->pdt_map.F11_query_base);
						SYNAPTICS_PRINTK(1, "touch:F11_command_base [0x%x]:0x%x\n", pdt_addr+1, g_tp->pdt_map.F11_command_base);
						SYNAPTICS_PRINTK(1, "touch:F11_control_base [0x%x]:0x%x\n", pdt_addr+2, g_tp->pdt_map.F11_control_base);
						SYNAPTICS_PRINTK(1, "touch:F11_data_base [0x%x]:0x%x\n", pdt_addr+3, g_tp->pdt_map.F11_data_base);
						SYNAPTICS_PRINTK(1, "touch:F11_version_interrupt_count [0x%x]:0x%x\n", pdt_addr+4, g_tp->pdt_map.F11_version_interrupt_count);
						SYNAPTICS_PRINTK(1, "touch:F11_function_exists [0x%x]:0x%x\n", pdt_addr+5, g_tp->pdt_map.F11_function_exists);
					}
				} else if (page_select == 0x01)	{
					if (pdt_value[5] == 0x00) {
						SYNAPTICS_PRINTK(1, "touch:resvered register value[0x%x]:0x%x\n", pdt_addr+5, pdt_value[5]);
						break;
					} else if (pdt_value[5] == PDT_ANALOG_F54) {
						g_tp->pdt_map.F54_query_base = pdt_value[0];
						g_tp->pdt_map.F54_command_base = pdt_value[1];
						g_tp->pdt_map.F54_control_base = pdt_value[2];
						g_tp->pdt_map.F54_data_base = pdt_value[3];
						g_tp->pdt_map.F54_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F54_function_exists = pdt_value[5];
						SYNAPTICS_PRINTK(1, "touch:F54_query_base [0x%x]:0x%x\n", pdt_addr, g_tp->pdt_map.F54_query_base);
						SYNAPTICS_PRINTK(1, "touch:F54_command_base [0x%x]:0x%x\n", pdt_addr+1, g_tp->pdt_map.F54_command_base);
						SYNAPTICS_PRINTK(1, "touch:F54_control_base [0x%x]:0x%x\n", pdt_addr+2, g_tp->pdt_map.F54_control_base);
						SYNAPTICS_PRINTK(1, "touch:F54_data_base [0x%x]:0x%x\n", pdt_addr+3, g_tp->pdt_map.F54_data_base);
						SYNAPTICS_PRINTK(1, "touch:F54_version_interrupt_count [0x%x]:0x%x\n", pdt_addr+4, g_tp->pdt_map.F54_version_interrupt_count);
						SYNAPTICS_PRINTK(1, "touch:F54_function_exists [0x%x]:0x%x\n", pdt_addr+5, g_tp->pdt_map.F54_function_exists);
					}	
				} else if (page_select == 0x02)	{
					if (pdt_value[5] == 0x00) {
						SYNAPTICS_PRINTK(0, "touch:reserved register value[0x%x]:0x%x\n", pdt_addr+5, pdt_value[5]);
						break;
					} else if (pdt_value[5] == PDT_LED_F31) {
						g_tp->pdt_map.F31_query_base = pdt_value[0];
						g_tp->pdt_map.F31_command_base = pdt_value[1];
						g_tp->pdt_map.F31_control_base = pdt_value[2];
						g_tp->pdt_map.F31_data_base = pdt_value[3];
						g_tp->pdt_map.F31_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F31_function_exists = pdt_value[5];
						SYNAPTICS_PRINTK(0, "touch:F31_query_base [0x%x]:0x%x\n", pdt_addr, g_tp->pdt_map.F31_query_base);
						SYNAPTICS_PRINTK(0, "touch:F31_command_base [0x%x]:0x%x\n", pdt_addr+1, g_tp->pdt_map.F31_command_base);
						SYNAPTICS_PRINTK(0, "touch:F31_control_base [0x%x]:0x%x\n", pdt_addr+2, g_tp->pdt_map.F31_control_base);
						SYNAPTICS_PRINTK(0, "touch:F31_data_base [0x%x]:0x%x\n", pdt_addr+3, g_tp->pdt_map.F31_data_base);
						SYNAPTICS_PRINTK(0, "touch:F31_version_interrupt_count [0x%x]:0x%x\n", pdt_addr+4, g_tp->pdt_map.F31_version_interrupt_count);
						SYNAPTICS_PRINTK(0, "touch:F31_function_exists [0x%x]:0x%x\n", pdt_addr+5, g_tp->pdt_map.F31_function_exists);
					} else if (pdt_value[5] == PDT_0D_F1A) {
						g_tp->pdt_map.F1A_query_base = pdt_value[0];
						g_tp->pdt_map.F1A_command_base = pdt_value[1];
						g_tp->pdt_map.F1A_control_base = pdt_value[2];
						g_tp->pdt_map.F1A_data_base = pdt_value[3];
						g_tp->pdt_map.F1A_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F1A_function_exists = pdt_value[5];
						SYNAPTICS_PRINTK(0, "touch:F1A_query_base [0x%x]:0x%x\n", pdt_addr, g_tp->pdt_map.F1A_query_base);
						SYNAPTICS_PRINTK(0, "touch:F1A_command_base [0x%x]:0x%x\n", pdt_addr+1, g_tp->pdt_map.F1A_command_base);
						SYNAPTICS_PRINTK(0, "touch:FA1_control_base [0x%x]:0x%x\n", pdt_addr+2, g_tp->pdt_map.F1A_control_base );
						SYNAPTICS_PRINTK(0, "touch:F1A_data_base [0x%x]:0x%x\n", pdt_addr+3, g_tp->pdt_map.F1A_data_base);
						SYNAPTICS_PRINTK(0, "touch:F1A_version_interrupt_count [0x%x]:0x%x\n", pdt_addr+4, g_tp->pdt_map.F1A_version_interrupt_count);
						SYNAPTICS_PRINTK(0, "touch:F1A_function_exists [0x%x]:0x%x\n", pdt_addr+5, g_tp->pdt_map.F1A_function_exists);
					}	
				} else {
					//out of range
				}
			}	
		}
	}
out:
	/* Page Descriptionc Table */
	page_select = 0x00;
	ret = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
	if (ret) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (ret=%d)\n",
			PAGE_SELECT_REGISTER, page_select, ret);
		return ret;
	}
	SYNAPTICS_PRINTK(1, "synaptics page table addr[0x%x]:0x%x\n", PAGE_SELECT_REGISTER, page_select);
	msleep(1);
    return rc;
}

/* SynaWriteBootloadID writes the bootloader ID to the F34 data register to unlock the reflash process
 */
 static int SynaWriteBootloadID(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t bootid[2];
	uint8_t f34_block_data_addr;
	
	f34_block_data_addr = g_tp->pdt_map.F34_data_base + f34_flash_data02_00;
	bootid[0] = g_tp->f34_bootload_id & 0x00FF;
	bootid[1] = (g_tp->f34_bootload_id & 0xFF00) >> 8;
	SYNAPTICS_PRINTK(1, "touch:block data addr[0x%x]\n", f34_block_data_addr);
	SYNAPTICS_PRINTK(1, "touch:bootid[0]:0x%x\n", bootid[0]);
	SYNAPTICS_PRINTK(1, "touch:bootid[1]:0x%x\n", bootid[1]);

	/* write the bootloader iD to the F34 Block Data */
	rc = touchpad_write_i2c(g_tp->client, f34_block_data_addr, &bootid[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics F34_Flash Block Data[0x%x]:0x%x (rc=%d)\n",
			f34_block_data_addr, bootid[0], rc);
		return rc;
	}
	/* check if write block data is correct */
	rc = touchpad_read_i2c(g_tp->client, f34_block_data_addr, &bootid[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read F34_Flash Block Data[0x%x]:0x%x (rc=%d)\n",
			f34_block_data_addr, bootid[0], rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch: write bootid to f34_block_datao[0x%x]:0x%x\n", f34_block_data_addr, bootid[0]);
	SYNAPTICS_PRINTK(1, "touch: write bootid to f34_block_data1[0x%x]:0x%x\n", f34_block_data_addr+1, bootid[1]);
	
	return rc;
}
/* SynaReadBootloadID reads the F34 query registers and retrieves the bootloader ID of the firmware
 */
static int SynaReadBootloadID(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t bootid[2];
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F34_query_base, &bootid[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read f34_flash_BootloaderID[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F34_query_base, bootid[0], rc);
		return rc;
	}
	g_tp->f34_bootload_id = bootid[1] << 8 | bootid[0];
	SYNAPTICS_PRINTK(1, "touch:read BootID_0[0x%x]:0x%x\n", g_tp->pdt_map.F34_query_base,bootid[0]);
	SYNAPTICS_PRINTK(1, "touch:read BootID_1[0x%x]:0x%x\n", g_tp->pdt_map.F34_query_base+1,bootid[1]);
	SYNAPTICS_PRINTK(1, "touch:read BootloadID=%d\n", (int)g_tp->f34_bootload_id);
	return rc;
}
/* SynaEnableFlashing kicks off the reflash process */
static int SynaEnableFlashing(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr, program_enable;
	uint8_t irq_status;
	uint8_t f34_flash_cmd;
	int i;

	SYNAPTICS_PRINTK(0,"touch:Enable Reflash...\n");
	/* Reflash is enabled by first reading the bootloader ID from the firmware and write it back */
	rc = SynaReadBootloadID(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to read Bootload ID (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	rc = SynaWriteBootloadID(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write Bootload ID (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	/* Issue a flash program enable command */
	/* check if flash ctrl is 0 */
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read SynaF34_Flash_Control[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(0, "touch:read the F34_Flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
	
	/* write enable command to enable flash */
	f34_flash_cmd |= 0x0f;
	rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	msleep(500); /* need to wait 500ms */
		
	for(i=0;;i++)
	{
		/* check if flash interrupt status bit is 1 */
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read F01_interrupt_status[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F01_data_base+1, irq_status, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(0, "touch:F01_interrupt_status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, irq_status);
		if (irq_status & 0x1)  break;
		if(i == 500) {
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	/* Page Description Table (PDT) */
	rc = touchpad_pdtscan(g_tp);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to reads the PDT and constructs the RMI register map. (rc=%d)\n", rc);
		return rc;
	}
			
	/* check if programming enable */
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read F34_Flash_Ctrl program cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, program_enable, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:program enable[0x%x]:0x%x\n", f34_flash_cmd_addr, program_enable);
	
	if (program_enable == 0x80){
		SYNAPTICS_PRINTK(0, "touch:enabled flash programming successfully.\n");
		g_tp->program_enable_success = 1;
		rc = 0;
	}else {
		SYNAPTICS_PRINTK(0, "touch:failed to enable flash programming.\n");
		g_tp->program_enable_success = 0;
		rc = -1;
	}
	return rc;
}
/* SynaReadFirmwareInfo reads the F34 query registers and retrieves the block size and count
 * of the firmware section of the image to be reflashed 
 */
static int SynaReadFirmwareInfo(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_blocksize_addr,f34_blocksize_value[2];
	uint8_t f34_blockcount_addr,f34_blockcount_value[2];
	
	SYNAPTICS_PRINTK(1, "touch:read firmware Info.\n");
	
	/* read firmware block size */
	f34_blocksize_addr = g_tp->pdt_map.F34_query_base + f34_flash_query03;
	rc = touchpad_read_i2c(g_tp->client, f34_blocksize_addr, &f34_blocksize_value[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read f34_firmware_block_size[0x%x]:0x%x (rc=%d)\n",
			f34_blocksize_addr, f34_blocksize_value[0], rc);
		return rc;
	}
	g_tp->firmware_block_size = f34_blocksize_value[1] << 8 | f34_blocksize_value[0];
	SYNAPTICS_PRINTK(1, "touch:read f34_firmware_blocksize=0x%x\n", g_tp->firmware_block_size);
	
	/* read firmware block count */
	f34_blockcount_addr = g_tp->pdt_map.F34_query_base + f34_flash_query05;
	rc = touchpad_read_i2c(g_tp->client, f34_blockcount_addr, &f34_blockcount_value[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read f34_firmware_block_count[0x%x]:0x%x (rc=%d)\n",
			f34_blockcount_addr, f34_blockcount_value[0], rc);
		return rc;
	}
	g_tp->firmware_block_count = f34_blockcount_value[1] << 8 | f34_blockcount_value[0];
	SYNAPTICS_PRINTK(1, "touch:read f34_firmware_blockcount=0x%x\n", g_tp->firmware_block_count);
	/* caculate firmware image size */
	g_tp->firmware_image_size = g_tp->firmware_block_count * g_tp->firmware_block_size;
	SYNAPTICS_PRINTK(1, "touch:read g_tp->firmware_image_size=0x%x\n", g_tp->firmware_image_size);
	return rc;

}
/* SynaReadConfigInfo reads the F34 query registers and retrieves the block size and count
 * of the configuration section of the image to be reflashed 
 */
static int SynaReadConfigInfo(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_blocksize_addr,f34_blocksize_value[2];
	uint8_t f34_blockcount_addr,f34_blockcount_value[2];
	
	SYNAPTICS_PRINTK(1, "touch:read Config Info.\n");
	/* read config block size */
	f34_blocksize_addr = g_tp->pdt_map.F34_query_base + f34_flash_query03;
	rc = touchpad_read_i2c(g_tp->client, f34_blocksize_addr, &f34_blocksize_value[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read f34_config_block_size[0x%x]:0x%x (rc=%d)\n",
			f34_blocksize_addr, f34_blocksize_value[0], rc);
		return rc;
	}
	g_tp->config_block_size = f34_blocksize_value[1] << 8 | f34_blocksize_value[0];
	SYNAPTICS_PRINTK(1, "touch:read f34_config_blocksize=0x%x\n", g_tp->config_block_size);
	
	/* read config block count */
	f34_blockcount_addr = g_tp->pdt_map.F34_query_base + f34_flash_query07;
	rc = touchpad_read_i2c(g_tp->client, f34_blockcount_addr, &f34_blockcount_value[0], 2);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read f34_config_block_count[0x%x]:0x%x (rc=%d)\n",
			f34_blockcount_addr, f34_blockcount_value[0], rc);
		return rc;
	}
	g_tp->config_block_count = f34_blockcount_value[1] << 8 | f34_blockcount_value[0];
	SYNAPTICS_PRINTK(1, "touch:read f34_config_blockcount=0x%x\n", g_tp->config_block_count);
	/* caculate config image size */
	g_tp->config_image_size = g_tp->config_block_count * g_tp->config_block_size;
	SYNAPTICS_PRINTK(1, "touch:read g_tp->config_image_size=0x%x\n", g_tp->config_image_size);
	return rc;
}

/* SynaProgramConfiguration writes the configuration section of the image block by block
 **************************************************************************************
 */
static int SynaProgramConfiguration(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t  blockNum_value[2];
	uint16_t blockNum;
	uint8_t f34_flash_cmd,f34_flash_cmd_addr;
    uint8_t irq_status;
	int i;
	uint8_t program_enable;
	
	SYNAPTICS_PRINTK(0,"touch:Program Configuration Section...\n");
	for (blockNum = 0x000; blockNum < g_tp->config_block_count; blockNum++)
	{
	    blockNum_value[0] = blockNum & 0x00ff;
		blockNum_value[1] = (blockNum & 0xff00) >> 8;
		//Block by blcok, write the block number and data to the corresponding F34 data registers
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base, &blockNum_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to write f34_blockNum[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F34_data_base, blockNum_value[0], rc);
			return rc;
		}
		
		/* write config image data to block data */
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base+2, g_tp->puData, (int)g_tp->config_block_size);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to write f34_block_data[0x%x]:0x%s (rc=%d)\n",
				g_tp->pdt_map.F34_data_base+2, g_tp->puData, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1,"touch:puData=0x%x\n", *(g_tp->puData));
		g_tp->puData += (int)g_tp->config_block_size;
		
		f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read SynaF34_Flash_Control[0x%x]:0x%x (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "touch:read the F34_Flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
		// Issue the "Write Configuration Block" command
		f34_flash_cmd = (f34_flash_cmd & 0xF0) | 0x06;
		rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		msleep(10); /* need to wait 500ms */
		for(i=0;;i++)
		{
			/* check if flash interrupt status bit is 1 */
			rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
			if (rc) {
				SYNAPTICS_PRINTK(0, "touch:failed to read F01_interrupt_status[0x%x]:0x%x (rc=%d)\n",
					g_tp->pdt_map.F01_data_base+1, irq_status, rc);
				return rc;
			}
			SYNAPTICS_PRINTK(1, "touch:F01_interrupt_status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, irq_status);
			if (irq_status & 0x1)  break;
			if(i == 500) 
			{
				rc = -1;
				return rc;
			}
			msleep(10);
		}
		/* check if programming enable */
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read F34_Flash_Ctrl program cmd[0x%x]:0x%x (rc=%d)\n",
				f34_flash_cmd_addr, program_enable, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "touch:program enable[0x%x]:0x%x\n", f34_flash_cmd_addr, program_enable);
	
		if (program_enable == 0x80){
			SYNAPTICS_PRINTK(1, "touch:enabled flash programming successfully.\n");
			rc = 0;
		}else {
			SYNAPTICS_PRINTK(0, "touch:failed to enable flash programming.\n");
			rc = -1;
			break;
		}
	}
	return rc;
}

/* SynaFinalizeReflash finalizes the reflash process */
static int SynaFinalizeReflash(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,f34_flash_cmd;
	uint8_t f01_reset_cmd = 0x0;
	uint8_t f01_device_status = 0x0;
	int i;
	
	SYNAPTICS_PRINTK(0, "Finalizing Reflash...\n");
	// Issue the "Reset" command to F01 command register to reset the chip
	// This command will also test the new firmware image and check if its is valid
	f01_reset_cmd = 0x1;
	rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F01_command_base, &f01_reset_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:touch:failed to write f01_reset_cmd[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_command_base, f01_reset_cmd, rc);
		return rc;
	}
	else
	{
		SYNAPTICS_PRINTK(0, "touch:successfully send reset cmd\n");
	}
	msleep(500); /* need to wait 500ms */
	SYNAPTICS_PRINTK(0, "touch:write f01_reset_cmd[0x%x]:0x%x\n", g_tp->pdt_map.F01_command_base, f01_reset_cmd);
	for(i=0;;i++)
	{
		/* read device status */
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base, &f01_device_status, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read f01_device_status[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F01_data_base, f01_device_status, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(0, "touch:read f01_device_status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base, f01_device_status);
		
		// Check if the "Program Enabled" bit in F01 data register is cleared
		// Sanity check that the reflash process is still enabled
		f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read f34_flash_cmd[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_data_base, f01_device_status, rc);
				return rc;
		}
		SYNAPTICS_PRINTK(0, "touch:read f34_flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
		if( (f01_device_status & 0x40) == 0 && f34_flash_cmd == 0)
		{
			 SYNAPTICS_PRINTK(0, "autotest:successfully reset ic\n");
			 break;
		}
		if(i == 500)
		{
			SYNAPTICS_PRINTK(0, "autotest:fail to reset ic\n");
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	
	/* scan the page description table */
	rc = touchpad_pdtscan(g_tp);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to reads the PDT and constructs the RMI register map. (rc=%d)\n", rc);
		return rc;
	}
	SYNAPTICS_PRINTK(0,"touch:Reflash configuration image Completed.\n");
	
	return rc;
}

/* erases the config block */
static int SynaEraseConfigBlock(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd,f34_flash_cmd_addr,irq_status;
	uint8_t f34_flash_erase_cmd;
	int i;
	
	/* Reflash is enabled by first reading the bootloader ID from the firmware and write it back */
	rc = SynaReadBootloadID(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to read Bootload ID (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	rc = SynaWriteBootloadID(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write Bootload ID (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	/* read f34 flash cmd */
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
					SYNAPTICS_PRINTK(0, "failed to read SynaF34_Flash_Control[0x%x]:0x%x (rc=%d)\n",
						f34_flash_cmd_addr, f34_flash_cmd, rc);
					return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:read the F34_Flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
	
	/* flash Command 7 to erase config */
	f34_flash_erase_cmd = (f34_flash_cmd & 0xF0) | 0x07;
	rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_erase_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "autotest:failed to write f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_erase_cmd, rc);
		return rc;
	}
	else
	{
		SYNAPTICS_PRINTK(0, "autotest:successfully send erase config cmd\n");
	}
	msleep(500); /* need to wait 500ms */
		
	for(i=0;;i++)
	{
		/* check if flash interrupt status bit is 1 */
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read F01_interrupt_status[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F01_data_base+1, irq_status, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "touch:F01_interrupt_status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, irq_status);
		if (irq_status & 0x1)  break;
		if(i == 500) 
		{
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	
	return rc;
}

/* ConfigBlockReflash reflashes the config block only */
static int SynaConfigBlockReflash(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr, program_enable;
	
	/* erase config block */
	rc = SynaEraseConfigBlock(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to erase config block (rc=%d)\n", rc);
		return rc;
	}

	/* check if programming enable */
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read F34_Flash_Ctrl program cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, program_enable, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(0, "touch:program enable[0x%x]:0x%x\n", f34_flash_cmd_addr, program_enable);
	if (program_enable != 0x80) {
		SYNAPTICS_PRINTK(0, "autotest:failed to erase config block \n");
		rc = -1;
		return rc;
	}
	else
	{
		SYNAPTICS_PRINTK(0, "autotest:successfully erase config block \n");
	}
	
	/* write config image to config area*/
	rc = SynaProgramConfiguration(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write config image to config area (rc=%d)\n",
			rc);
		return rc;
	}
	
	/* disable flash programming */
	rc = SynaFinalizeReflash(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to disable flash programming. (rc=%d)\n",
			rc);
		return rc;
	}
	
	return rc ;
}

/* update configuration image */
static int touchpad_update_config(struct synaptics_tp_t *g_tp)
{
	int rc = 0;

	/* Flash Programming Process */
	rc = SynaEnableFlashing(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to enable flash (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}	
    
	if (g_tp->program_enable_success == 1) {
		/* read config block size and block count */
		rc = SynaReadConfigInfo(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read config block size and block count (rc=%d)\n",
				rc);
			return rc;
		}
		/* read firmware block size and block count */
		rc = SynaReadFirmwareInfo(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read firmware block size and block count (rc=%d)\n",
				rc);
			return rc;
		}
		/* update config block image */
		rc = SynaConfigBlockReflash(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to enable flash (rc=%d)\n", rc);
			rc = -EINVAL;
			return rc;
		}
	}
	
	return rc;
}
/* erases the firmware block */
static int SynaEraseFirmwareBlock(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,irq_status;
	uint8_t f34_flash_erase_cmd;
	uint8_t f34_flash_cmd;
	int i;
		
	/* Reflash is enabled by first reading the bootloader ID from the firmware and write it back */
	rc = SynaReadBootloadID(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read Bootload ID (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	rc = SynaWriteBootloadID(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write Bootload ID (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	SYNAPTICS_PRINTK(0,"touch:erase all blocks...\n");
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read SynaF34_Flash_Control[0x%x]:0x%x (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:read the F34_Flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
	
	/* flash Command 3 to erase all(firmware and config block) */
	f34_flash_erase_cmd = (f34_flash_cmd & 0xF0) | 0x03;
	rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_erase_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_erase_cmd, rc);
		return rc;
	}
	else
	{
		SYNAPTICS_PRINTK(0, "touch:successfully send erase all cmd\n");
	}
	msleep(1000); /* need to wait 1000ms */
		
	for(i=0;;i++)
	{
		/* check if flash interrupt status bit is 1 */
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read F01_interrupt_status[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F01_data_base+1, irq_status, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "touch:F01_interrupt_status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, irq_status);
		if (irq_status & 0x1)  break;
		if(i == 500) 
		{
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	
	return rc;
}
/* SynaProgramFirmware writes the Firmware section of the image block by block
 **************************************************************************************
 */
static int SynaProgramFirmware(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t  blockNum_value[2];
	uint16_t blockNum;
	uint8_t f34_flash_cmd,f34_flash_cmd_addr;
    uint8_t irq_status;
	int i;
		
	SYNAPTICS_PRINTK(0,"touch:Program Firmware Section...\n");
	for (blockNum = 0x000; blockNum < g_tp->firmware_block_count; blockNum++)
	{
		blockNum_value[0] = blockNum & 0x00ff;
		blockNum_value[1] = (blockNum & 0xff00) >> 8;
		SYNAPTICS_PRINTK(1, "touch:f34_blocknum[0x%x]:0x%x\n", blockNum,blockNum_value[0]);
		SYNAPTICS_PRINTK(1, "touch:f34_blocknum[0x%x]:0x%x\n", blockNum,blockNum_value[1]);
		//Block by blcok, write the block number and data to the corresponding F34 data registers
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base, &blockNum_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to write f34_blockNum[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F34_data_base, blockNum_value[0], rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "touch:write f34_blocknum[0x%x]:0x%x\n", g_tp->pdt_map.F34_data_base, blockNum_value[0]);
		SYNAPTICS_PRINTK(1, "touch:write f34_blocknum[0x%x]:0x%x\n", g_tp->pdt_map.F34_data_base+1, blockNum_value[1]);

		/* write firmware image data to block data */
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base+2, g_tp->puFirmwareData, (int)g_tp->firmware_block_size);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to write f34_block_data[0x%x]:0x%s (rc=%d)\n",
				g_tp->pdt_map.F34_data_base+2, g_tp->puFirmwareData, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1,"touch:puFirmwareData=0x%x\n", *(g_tp->puFirmwareData));
		g_tp->puFirmwareData += (int)g_tp->firmware_block_size;

		f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
			if (rc) {
				SYNAPTICS_PRINTK(0, "touch:failed to read SynaF34_Flash_Control[0x%x]:0x%x (rc=%d)\n",
					f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "touch:read the F34_Flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
	
		// Issue the "Write Firmware Block" command
		f34_flash_cmd = (f34_flash_cmd & 0xF0) | 0x02;
		rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to write f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		msleep(10); /* need to wait 500ms */
		
		for(i=0;;i++)
		{
			/* check if flash interrupt status bit is 1 */
			rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
			if (rc) {
				SYNAPTICS_PRINTK(0, "touch:failed to read F01_interrupt_status[0x%x]:0x%x (rc=%d)\n",
					g_tp->pdt_map.F01_data_base+1, irq_status, rc);
				return rc;
			}
			SYNAPTICS_PRINTK(1, "touch:F01_interrupt_status[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, irq_status);
			if (irq_status & 0x1)  break;
			if(i == 500) 
			{
				rc = -1;
				return rc;
			}
			msleep(10);
		}
	}	
	return rc;
}
/* FirmwareBlockReflash reflashes the firmware block only */
static int SynaFirmwareBlockReflash(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	//uint8_t irq_status;
	uint8_t f34_flash_cmd_addr,program_enable/*,f34_flash_cmd*/;

	/* erase firmware block */
	rc = SynaEraseFirmwareBlock(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to erase firmware block (rc=%d)\n", rc);
		return rc;
	}
	
	/* check if programming enable */
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to read F34_Flash_Ctrl program cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, program_enable, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(0, "touch:program enable[0x%x]:0x%x\n", f34_flash_cmd_addr, program_enable);
	if (program_enable != 0x80) {
		SYNAPTICS_PRINTK(0, "touch:fail to erase all\n");
		rc = -1;
		return rc;
	}
	else
	{
		SYNAPTICS_PRINTK(0, "touch:successfully erase all\n");
	}
	
	/* write firmware image area*/
	rc = SynaProgramFirmware(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write firmware image to firmware area (rc=%d)\n",
			rc);
		return rc;
	}

	#if 0
	/* check if programming enable */
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
				SYNAPTICS_PRINTK(0, "touch:failed to read F34_Flash_Ctrl program cmd[0x%x]:0x%x (rc=%d)\n",
					f34_flash_cmd_addr, program_enable, rc);
				return rc;
	}
	SYNAPTICS_PRINTK(0, "touch:program enable[0x%x]:0x%x\n", f34_flash_cmd_addr, program_enable);
	if (program_enable != 0x80) {
		//do {
			/* write enable command to enable flash */
			f34_flash_cmd = 0x0f;
			rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
			if (rc) {
				SYNAPTICS_PRINTK(0, "touch:failed to write f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
					f34_flash_cmd_addr, f34_flash_cmd, rc);
				return rc;
			}
			msleep(500); /* need to wait 500ms */
			/* read enable command to enable flash */
			rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
			if (rc) {
				SYNAPTICS_PRINTK(0, "touch:failed to read f34_enable_flash_cmd[0x%x]:0x%x (rc=%d)\n",
					f34_flash_cmd_addr, f34_flash_cmd, rc);
				return rc;
			}
			SYNAPTICS_PRINTK(0, "touch:read f34_enable_flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
		//} while (f34_flash_cmd == 0x80);
	}
	#endif
	
	/* write config image to config area*/
	rc = SynaProgramConfiguration(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write config image to config area (rc=%d)\n",
			rc);
		return rc;
	}
	
	/* disable flash programming */
	rc = SynaFinalizeReflash(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to disable flash programming. (rc=%d)\n",
			rc);
		return rc;
	}
	
	return rc ;
}
/* update firmware image */
static int touchpad_update_firmware_img(struct synaptics_tp_t *g_tp)
{
	int rc = 0;

	/* Flash Programming Process */
	SYNAPTICS_PRINTK(0, "touch:update firmware image\n");
	rc = SynaEnableFlashing(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to enable flash (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	if (g_tp->program_enable_success == 1) {
		/* read config block size and block count */
		rc = SynaReadConfigInfo(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read config block size and block count (rc=%d)\n",
				rc);
			return rc;
		}
		/* read firmware block size and block count */
		rc = SynaReadFirmwareInfo(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to read firmware block size and block count (rc=%d)\n",
				rc);
			return rc;
		}
		/* update firmware block image */
		rc = SynaFirmwareBlockReflash(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "touch:failed to enable flash (rc=%d)\n", rc);
			rc = -EINVAL;
			return rc;
		}
	}
    return rc;
}

/* read fw version  */
static int touchpad_read_fw_ver(void)
{
	int rc = 0;
	uint8_t fw_ver_addr,fw_value[3];
	
	fw_ver_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query18;
	rc = touchpad_read_i2c(g_tp->client, fw_ver_addr, &fw_value[0], 3);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to read F01 rmi data addr[0x%x]:0x%x (rc=%d)\n", 
			fw_ver_addr, fw_value[0], rc);
		return rc;
	}
	g_tp->fw_version = fw_value[2] << 16 | fw_value[1] << 8 | fw_value[0];
	SYNAPTICS_PRINTK(0, "touch:fw version:%d\n", (int)g_tp->fw_version);
	return 0;
}

/* read config id */
static int touchpad_read_config_id(void)
{
	int rc = 0;
	uint8_t f34_ctrl_value[4];
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F34_control_base, &f34_ctrl_value[0], 4);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to read f34_ctrl_config_id[0x%x]:0x%x (rc=%d)\n", 
			g_tp->pdt_map.F11_control_base, f34_ctrl_value[0], rc);
		return rc;
	}
	g_tp->config_id = f34_ctrl_value[0] << 24 | f34_ctrl_value[1] << 16 | f34_ctrl_value[2] << 8 | f34_ctrl_value[3];
	SYNAPTICS_PRINTK(0, "touch:config id:0x%x\n", g_tp->config_id);
	return 0;
}

/* read product id */
static uint16_t get_module_tp_in_bootloader_mode(void)
{
	int rc = 0;
	uint8_t pid_addr,pid_value[11];
	
	pid_value[10] = 0;
	pid_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query11;
	rc = touchpad_read_i2c(g_tp->client, pid_addr, &pid_value[0], 10);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to read product id in bootloader mode. addr[0x%x]:0x%x (rc=%d)\n", pid_addr, pid_value[0], rc);
		return INVALID_TP;
	}
	SYNAPTICS_PRINTK(0, "touch:product id:[%s]\n", pid_value);
	if(strncmp(pid_value,"TPKW0100", strlen("TPKW0100")) == 0) return TPK_WHITE_TP;
	else if(strncmp(pid_value,"TPKW0200", strlen("TPKW0200")) == 0) return TPK_WHITE_TP_WITH_PET;
	else if(strncmp(pid_value,"JTPB1000", strlen("JTPB1000")) == 0) return JTOUCH_BLACK_TP;
	else if(strncmp(pid_value,"JTPW1100", strlen("JTPW1100")) == 0) return JTOUCH_WHITE_TP;
	/* add a new product id to distinguish between old and new ito film for OFilm tp in EVT2, 20130910 { */
	else if(strncmp(pid_value,"SAP_OF_B01", strlen("SAP_OF_B01")) == 0) {
		g_tp->is_new_prodid_flag = false;
		return OFILM_BLACK_TP;
	}	
	else if(strncmp(pid_value,"SAP_OF_B11", strlen("SAP_OF_B11")) == 0) {
		g_tp->is_new_prodid_flag = true;
		return OFILM_BLACK_TP;
	}
	/* } add a new product id to distinguish between old and new ito film for OFilm tp in EVT2, 20130910 */
	else if(strncmp(pid_value,"SAP_DJ_B01", strlen("SAP_DJ_B01")) == 0) return DJ_BLACK_TP;
	else return INVALID_TP;
	
}

static void get_module_tp_and_config_id(uint16_t *module_tp,uint16_t *config_id, uint8_t is_ic_in_bootloader_mode)
{
	*module_tp = get_module_tp_in_bootloader_mode();
	if(is_ic_in_bootloader_mode)
	{
		SYNAPTICS_PRINTK(0, "touch: in bootloader mode, module_tp:[0x%x]\n", *module_tp);
	}
	else
	{
		//*module_tp = (g_tp->config_id & 0xFFFF0000) >> 16;
		*config_id = (g_tp->config_id & 0x0000FFFF);
		SYNAPTICS_PRINTK(0, "touch: in UI mode, module_tp:[0x%x]\n", *module_tp);
	}
}

/* check if TPK or JTouch module TP  */
static int touchpad_check_module_tp(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,f34_flash_cmd;
	uint16_t module_tp;
	uint16_t config_id = 0x0;
	uint8_t is_ic_in_bootloader_mode;
	g_tp->puData = 0x0;
	g_tp->puFirmwareData = 0x0;
	
	SYNAPTICS_PRINTK(0, "touch:check TPK or JTouch module.\n");
	/*  read firmware version */
	touchpad_read_fw_ver();
	/* read config ID */
	touchpad_read_config_id();
	/* read flash control */
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to read SynaF34_Flash_cmd[0x%x]:0x%x (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(0, "touch: the F34_Flash_cmd[0x%x]:0x%x\n", f34_flash_cmd_addr, f34_flash_cmd);
	is_ic_in_bootloader_mode = f34_flash_cmd & 0x80;
	get_module_tp_and_config_id(&module_tp,&config_id,is_ic_in_bootloader_mode);
	SYNAPTICS_PRINTK(0, "touch:get the 3rd and 4th of config id:[0x%x]\n", config_id);
	switch(module_tp) {
	case TPK_BLACK_TP:
		SYNAPTICS_PRINTK(0, "touch:TPK black TP.\n");
		/* check if update fw version and config id */
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_2_2_FW_VER || IS_UNDER_TESTING) 
		{
			SYNAPTICS_PRINTK(0, "touch:update FW version = %d\n", SYNAP_DS4_3_2_2_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1200567_img;
			g_tp->puData = S3202_Config_tpk_black_30303036_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				SYNAPTICS_PRINTK(0, "touch:Failed to update firmware (rc=%d)\n" ,rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_06_TPK_BLACK_CONFIG_ID) { 
				SYNAPTICS_PRINTK(0, "touch:update CONFIG ID=0x%x\n", SYNAP_06_TPK_BLACK_CONFIG_ID);
				g_tp->puData = S3202_Config_tpk_black_30303036_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					SYNAPTICS_PRINTK(0, "touch:Failed to update config file (rc=%d)\n" ,rc);
					return rc;
				}
			}
		}
		break;
	case TPK_WHITE_TP:
		SYNAPTICS_PRINTK(0, "touch:TPK white TP.\n");
		/* check if update fw version and config id */
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_2_2_FW_VER || IS_UNDER_TESTING) 
		{
			SYNAPTICS_PRINTK(0, "touch:update FW version = %d\n", SYNAP_DS4_3_2_2_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1200567_img;
			g_tp->puData = S3202_Config_tpk_white_30313031_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				SYNAPTICS_PRINTK(0, "touch:Failed to update firmware (rc=%d)\n" ,rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_01_TPK_WHITETP_CONFIG_ID) { 
				SYNAPTICS_PRINTK(0, "touch:update CONFIG ID=0x%x\n", SYNAP_01_TPK_WHITETP_CONFIG_ID);
				g_tp->puData = S3202_Config_tpk_white_30313031_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					SYNAPTICS_PRINTK(0, "touch:Failed to update config file (rc=%d)\n" ,rc);
					return rc;
				}
			}
		}
		break;
	case TPK_WHITE_TP_WITH_PET:
		SYNAPTICS_PRINTK(0, "touch:TPK white TP with PET.\n");
		/* check if update fw version and config id */
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_2_2_FW_VER || IS_UNDER_TESTING) 
		{
			SYNAPTICS_PRINTK(0, "touch:update FW version = %d\n", SYNAP_DS4_3_2_2_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1200567_img;
			g_tp->puData = S3202_Config_tpk_white_30323031_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				SYNAPTICS_PRINTK(0, "touch:Failed to update firmware (rc=%d)\n" ,rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_01_TPK_WHITETP_CONFIG_ID) { 
				SYNAPTICS_PRINTK(0, "touch:update CONFIG ID=0x%x\n", SYNAP_01_TPK_WHITETP_CONFIG_ID);
				g_tp->puData = S3202_Config_tpk_white_30323031_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					SYNAPTICS_PRINTK(0, "touch:Failed to update config file (rc=%d)\n" ,rc);
					return rc;
				}
			}
		}
		break;
	case JTOUCH_BLACK_TP:
		SYNAPTICS_PRINTK(0, "touch:J-Touch black TP.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_6_0_33_FW_VER || IS_UNDER_TESTING) 
		{
			SYNAPTICS_PRINTK(0, "touch:update FW version = %d\n", SYNAP_DS4_3_6_0_33_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1296077_img;
			g_tp->puData = S3202_Config_JTP_black_31303032_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				SYNAPTICS_PRINTK(0, "touch:Failed to update firmware (rc=%d)\n" ,rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_02_JTP_CONFIG_ID) { 
				SYNAPTICS_PRINTK(0, "touch:update CONFIG ID=0x%x\n", SYNAP_02_JTP_CONFIG_ID);
				g_tp->puData = S3202_Config_JTP_black_31303032_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					SYNAPTICS_PRINTK(0, "touch:Failed to update config file (rc=%d)\n" ,rc);
					return rc;
				}
			}
		}
		break;
	case JTOUCH_WHITE_TP:
		SYNAPTICS_PRINTK(0, "touch:J-Touch white TP.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_6_0_33_FW_VER || IS_UNDER_TESTING) 
		{
			SYNAPTICS_PRINTK(0, "touch:update FW version = %d\n", SYNAP_DS4_3_6_0_33_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1296077_img;
			g_tp->puData = S3202_Config_JTP_white_31313032_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				SYNAPTICS_PRINTK(0, "touch:Failed to update firmware (rc=%d)\n" ,rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_02_JTP_CONFIG_ID) { 
				SYNAPTICS_PRINTK(0, "touch:update CONFIG ID=0x%x\n", SYNAP_02_JTP_CONFIG_ID);
				g_tp->puData = S3202_Config_JTP_white_31313032_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					SYNAPTICS_PRINTK(0, "touch:Failed to update config file (rc=%d)\n" ,rc);
					return rc;
				}
			}
		}
		break;
	case OFILM_BLACK_TP:
		SYNAPTICS_PRINTK(0, "This is the O-film black TP.\n");
		if(is_ic_in_bootloader_mode != 0 || 
			(g_tp->is_new_prodid_flag == false && g_tp->fw_version != SYNAP_DS4_3_6_0_34_FW_VER) ||
/* check if FW version is 1365481 and config id is OB40 in factory image for EVT2,
   check if FW version is 1421304 and config is OB50 in release and ship image for EVT2  
*/			
#ifdef CONFIG_BUILD_FACTORY
				(g_tp->is_new_prodid_flag == true && g_tp->fw_version != SYNAP_DS4_3_6_0_34_FW_VER) ||
#else
				(g_tp->is_new_prodid_flag == true && g_tp->fw_version != SYNAP_DS4_3_6_0_35_FW_VER) ||
#endif				
				IS_UNDER_TESTING) {
				/* add a new product id to distinguish between old and new ito film for OFilm tp in EVT2, 20130910 { */
				if(g_tp->is_new_prodid_flag == false) {
					SYNAPTICS_PRINTK(0, "update FW version = %d\n", SYNAP_DS4_3_6_0_34_FW_VER);
					g_tp->puFirmwareData = S3202_Firmware_1365481_img; 
					g_tp->puData = S3202_Config_of_black_4F423330_img;
					rc = touchpad_update_firmware_img(g_tp);
					if (rc < 0) {
						SYNAPTICS_PRINTK(0, "Failed to update firmware (rc=%d)\n" ,rc);
						return rc;
					} 
				} else if(g_tp->is_new_prodid_flag == true) {
/* check if FW version is 1365481 and config id is OB40 in factory image for EVT2,
   check if FW version is 1421304 and config is OB50 in release and ship image for EVT2  
*/				
#ifdef CONFIG_BUILD_FACTORY
					SYNAPTICS_PRINTK(0, "update FW version = %d\n", SYNAP_DS4_3_6_0_34_FW_VER);
					g_tp->puFirmwareData = S3202_Firmware_1365481_img;
					g_tp->puData = S3202_Config_of_black_4F423430_img;
					rc = touchpad_update_firmware_img(g_tp);
					if (rc < 0) {
						SYNAPTICS_PRINTK(0, "Failed to update firmware (rc=%d)\n" ,rc);
						return rc;
					}
#else				
					SYNAPTICS_PRINTK(0, "update FW version = %d\n", SYNAP_DS4_3_6_0_35_FW_VER);
					g_tp->puFirmwareData = S3202_Firmware_1421304_img;
					g_tp->puData = S3202_Config_of_black_4F423630_img;
					rc = touchpad_update_firmware_img(g_tp);
					if (rc < 0) {
						SYNAPTICS_PRINTK(0, "Failed to update firmware (rc=%d)\n" ,rc);
						return rc;
					} 
#endif	
				}
		}
		else
		{
			/* add a new product id to distinguish between old and new ito film for OFilm tp in EVT2, 20130910 { */
			if(g_tp->is_new_prodid_flag == false && config_id != SYNAP_03_OF_CONFIG_ID)
			{
				SYNAPTICS_PRINTK(0, "update CONFIG ID = 0x%x\n", SYNAP_03_OF_CONFIG_ID);
				g_tp->puData = S3202_Config_of_black_4F423330_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					SYNAPTICS_PRINTK(0, "Failed to update config file (rc=%d)\n" ,rc);
					return rc;
				}
			} else if(g_tp->is_new_prodid_flag == true) {
/* check if FW version is 1365481 and config id is OB40 in factory image for EVT2,
   check if FW version is 1421304 and config is OB50 in release and ship image for EVT2  
*/
#ifdef CONFIG_BUILD_FACTORY
				if (config_id != SYNAP_04_OF_CONFIG_ID) {
					SYNAPTICS_PRINTK(0, "update CONFIG ID = 0x%x\n", SYNAP_05_OF_CONFIG_ID);
					g_tp->puData = S3202_Config_of_black_4F423430_img; 
					rc = touchpad_update_config(g_tp);
					if (rc < 0) {
					SYNAPTICS_PRINTK(0, "Failed to update config file (rc=%d)\n" ,rc);
					return rc;
					}
				}
#else
				if (config_id != SYNAP_06_OF_CONFIG_ID) {
					SYNAPTICS_PRINTK(0, "update CONFIG ID = 0x%x\n", SYNAP_06_OF_CONFIG_ID);
					g_tp->puData = S3202_Config_of_black_4F423630_img; 
					rc = touchpad_update_config(g_tp);
					if (rc < 0) {
					SYNAPTICS_PRINTK(0, "Failed to update config file (rc=%d)\n" ,rc);
					return rc;
					}
				}
#endif				
			}
			/* } add a new product id to distinguish between old and new ito film for OFilm tp in EVT2, 20130910 */
 }
		break;
	case DJ_BLACK_TP:
		SYNAPTICS_PRINTK(0, "This is the DJ black TP.\n");
		/* TOD upgrade FW and Config file */
		break;
	default:
		SYNAPTICS_PRINTK(0, "touch:unknow tp.\n");
	} //end of switch
	return rc;
}

/* ***************************************************************************
 * get raw capacitance pass spec for different module tp.
 * ***************************************************************************/
static int touchpad_get_raw_cap_pass_sepc(void)
{
	int rc = 0;
	uint16_t module_tp;
	
	module_tp = get_module_tp_in_bootloader_mode();
	SYNAPTICS_PRINTK(0, "touch: in UI mode, module_tp:[0x%x]\n", module_tp);
	switch(module_tp) {
	case TPK_BLACK_TP:
		/* set up touch self-test pass criteria */
		g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_TPK_BLACK_TP;
		g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_TPK_BLACK_TP;
		break;
	case TPK_WHITE_TP:
		/* set up touch self-test pass criteria */
		if (g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER) {
			g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_TPK_WHITE_TP_1115991;
			g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_TPK_WHITE_TP_1115991;
		} else if (g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER) {
			g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_TPK_WHITE_TP_1200567;
			g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_TPK_WHITE_TP_1200567;
		}
		break;
	case TPK_WHITE_TP_WITH_PET:
		/* set up touch self-test pass criteria */
		g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_TPK_WHITE_TP_with_PET;
		g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_TPK_WHITE_TP_with_PET;
		break;
	case JTOUCH_BLACK_TP:	
		/* set up touch self-test pass criteria */
		g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_JT_BLACK_TP;
		g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_JT_BLACK_TP;
		break;
	case JTOUCH_WHITE_TP:
		/* set up touch self-test pass criteria */
		g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_JT_WHITE_TP;
		g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_JT_WHITE_TP;
		break;
	case OFILM_BLACK_TP:
		/* set up touch self-test pass criteria */
		/* add a raw capacitance pass criteria for the new ito film, 20130910 { */
		if(g_tp->is_new_prodid_flag == false) {
			g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_OB_BLACK_TP;
			g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_OB_BLACK_TP;
		} else if (g_tp->is_new_prodid_flag == true) {
		   SYNAPTICS_PRINTK(0,"gpio(%d) == %d\n", TP_LAMITATION_DETECTION_PIN, gpio_get_value(TP_LAMITATION_DETECTION_PIN));
			if (gpio_get_value(TP_LAMITATION_DETECTION_PIN) == 1) {
				g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_OB_NEW_BLACK_TP;
				g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_OB_NEW_BLACK_TP;
			} else if (gpio_get_value(TP_LAMITATION_DETECTION_PIN) == 0) {
				g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_OB_Lamination_BLACK_TP;
				g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_OB_Lamination_BLACK_TP;
			}
		}
		/* add a raw capacitance pass criteria for the new ito film, 20130910 { */
		break;
	case DJ_BLACK_TP:
		SYNAPTICS_PRINTK(0,"gpio(%d) == %d\n", TP_LAMITATION_DETECTION_PIN, gpio_get_value(TP_LAMITATION_DETECTION_PIN));
		if (gpio_get_value(TP_LAMITATION_DETECTION_PIN) == 1) {
			g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_DJ_BLACK_TP;
			g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_DJ_BLACK_TP;
		} else if (gpio_get_value(TP_LAMITATION_DETECTION_PIN) == 0) {
			g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_DJ_Lamination_BLACK_TP;
			g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_DJ_Lamination_BLACK_TP;
		}
		break;
	default:
		SYNAPTICS_PRINTK(0, "unknow tp.\n");
	} //end of switch
	SYNAPTICS_PRINTK(0,"f54_raw_cap_pass_spec=%d ~ %d\n", g_tp->f54_raw_cap_min_limit, g_tp->f54_raw_cap_max_limit);
	return rc;		
}

/* detect Synaptics s3202 chip */
static int touchpad_detect_synaptics(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f01_value[2];
	uint8_t page_select = 0x0;
	uint8_t f11_addr,f11_value[2];
	
	SYNAPTICS_PRINTK(1, "touch:detect synaptics register map\n");
	/* Page Description Table (PDT) */
	rc = touchpad_pdtscan(g_tp);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to reads the PDT and constructs the RMI register map. (rc=%d)\n", rc);
		return rc;
	}
	/* Page Descriptionc Table */
	rc = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select)); 
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);

	/* check Touch module  */
	rc = touchpad_check_module_tp(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "touch:failed to check module tp. (rc=%d)\n",rc);
		return rc;
	}
	else
	{
		/*  read firmware version */
		touchpad_read_fw_ver();
		/* read config ID */
		touchpad_read_config_id();
		SYNAPTICS_PRINTK(0, "touch:successfully upgrade tp module. (rc=%d)\n",rc);
		/* get raw capacitance pass spec */
		//touchpad_get_raw_cap_pass_sepc();
	}
	
	/* read device status and interrupt status */
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base, &f01_value[0], 2);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to read F01 rmi data addr[0x%x]:0x%x (rc=%d)\n", 
			g_tp->pdt_map.F01_data_base, f01_value[0], rc);
		return rc;
	}
	/* print device status and interrupt status */
	SYNAPTICS_PRINTK(1, "touch:read f01 device status addr[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base, f01_value[0]);
	SYNAPTICS_PRINTK(1, "touch:read f01 interrupt status addr[0x%x]:0x%x\n", g_tp->pdt_map.F01_data_base+1, f01_value[1]);
	
	/* read Tx,Rx electrodes */
	if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
		(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER) {
		f11_addr = (g_tp->pdt_map.F11_control_base + DS4_3_2_F11_2D_CTRL77);
		rc = touchpad_read_i2c(g_tp->client, f11_addr, &f11_value[0], 2);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read F01 rmi data addr[0x%x]:0x%x (rc=%d)\n", 
				f11_addr, f11_value[0], rc);
			return rc;
		}
		g_tp->number_of_rx_electrodes = (int)f11_value[0];
		g_tp->number_of_tx_electrodes = (int)f11_value[1];
		SYNAPTICS_PRINTK(0, "total number of TX(%d) * RX(%d) channels\n", 
			g_tp->number_of_tx_electrodes, g_tp->number_of_rx_electrodes)
	}else if ((int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER) {
		f11_addr = (g_tp->pdt_map.F11_control_base + DS4_3_6_F11_2D_CTRL77);
		rc = touchpad_read_i2c(g_tp->client, f11_addr, &f11_value[0], 2);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read F01 rmi data addr[0x%x]:0x%x (rc=%d)\n", 
				f11_addr, f11_value[0], rc);
			return rc;
		}
		g_tp->number_of_rx_electrodes = (int)f11_value[0];
		g_tp->number_of_tx_electrodes = (int)f11_value[1];
		SYNAPTICS_PRINTK(0, "total number of TX(%d) * RX(%d) channels\n", 
			g_tp->number_of_tx_electrodes, g_tp->number_of_rx_electrodes);
	} else {
		SYNAPTICS_PRINTK(0, "Warning:TX-RX not in range!\n");
	}
	return 0;
}

/* HW reset for Synaptics s3202 */
static int touchpad_config_gpio(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	/* set hw reset high */
    gpio_set_value(g_tp->gpio_rst, 1);
    SYNAPTICS_PRINTK(1, "touch:Tset hw reset high, delay 100ms\n");
	msleep(30);
    gpio_set_value(g_tp->gpio_rst, 0);
    SYNAPTICS_PRINTK(1, "touch:set hw reset low, delay 1ms\n");
    msleep(1);
	/* set hw reset high */
    gpio_set_value(g_tp->gpio_rst, 1);
    SYNAPTICS_PRINTK(1, "touch:Tset hw reset high, delay 100ms\n");
	msleep(1); 
    return rc;
}

/* release gpios */
static int touchpad_release_gpio(struct synaptics_tp_t *g_tp)
{
    int rc = 0;
	
	SYNAPTICS_PRINTK(1, "touch:releasing gpio!!!\n");
	gpio_free(g_tp->gpio_irq);
	gpio_free(g_tp->gpio_rst);
	gpio_free(g_tp->gpio_vendor_id);
	return rc;
}

static int touchpad_setup_gpio(struct synaptics_tp_t *g_tp)
{
    int rc = 0;

    SYNAPTICS_PRINTK(1,"touch:Set up GPIO!!!! \n");
	/* set up IRQ gpio */
    rc = gpio_request(g_tp->gpio_irq, "synaptics_ts_irq");
    if (rc) {
    	SYNAPTICS_PRINTK(0, "touch:failed to request gpio_irq:%d (rc=%d)\n", g_tp->gpio_irq, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_input(g_tp->gpio_irq);
    if (rc) {
        SYNAPTICS_PRINTK(0, "touch:failed to set gpio_irq:%d mode (rc=%d)\n", g_tp->gpio_irq, rc);
		goto err_gpio_config;
    }
	/* print configure gpio num for debugging */
	SYNAPTICS_PRINTK(1, "touch:success to configure gpio_irq:%d gpio_name:%s\n",
			g_tp->gpio_irq, "synaptics_ts_irq");
			
	/* set up RST gpio */		
	rc = gpio_request(g_tp->gpio_rst, "synaptics_ts_rst");
	if (rc)
    {
        SYNAPTICS_PRINTK(0, "touch:failed to request gpio_rst:%d (rc=%d)\n", g_tp->gpio_rst, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_output(g_tp->gpio_rst, 0);
    if ( rc )
    {
        SYNAPTICS_PRINTK(0, "touch:failed to set gpio_rst:%d mode (rc=%d)\n", g_tp->gpio_rst, rc);
        goto err_gpio_config;
    }		
	/* print configure gpio num for debugging */
	SYNAPTICS_PRINTK(1, "touch:success to configure gpio_rst:%d gpio_name:%s\n",
			g_tp->gpio_rst, "synaptics_ts_rst");
		
	
	/* set up vendor id gpio */	
	rc = gpio_request(g_tp->gpio_vendor_id, "ft5316_ts_id");
    if (rc) {
    	SYNAPTICS_PRINTK(0, "Failed to request gpio_vendor_id:%d (rc=%d)\n", g_tp->gpio_vendor_id, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_input(g_tp->gpio_vendor_id);
    if (rc) {
        SYNAPTICS_PRINTK(0, "Failed to set gpio_vendor_id:%d mode (rc=%d)\n", g_tp->gpio_vendor_id, rc);
		goto err_gpio_config;
    }
	/* print configure gpio num for debugging */
	SYNAPTICS_PRINTK(0, "Success to configure gpio_vendor_id:%d gpio_name:%s\n",
			g_tp->gpio_vendor_id, "ft5316_ts_vendor_id");
	return 0;
	
err_gpio_config:
	touchpad_release_gpio(g_tp);
	return rc;
}


/* power on sequence for Synaptics S3202 touchscreen chip */
static int touchpad_power_on_device(struct synaptics_tp_t *g_tp, int on)
{
	int rc = 0;
	static int prev_on = 0;

	if (on == prev_on) {
		return 0;
	}

	if(on) {
		/* L9 voltage for s3202 AVDD */
		g_tp->ldo9_regulator = regulator_get(NULL, "touch_avdd");
		if (IS_ERR(g_tp->ldo9_regulator)) {
			rc = PTR_ERR(g_tp->ldo9_regulator);
			pr_err("%s:regulator get ldo9 voltage failed rc=%d\n",
							__func__, rc);
			rc = -ENODEV;
			goto exit;
		}
		
		/* LVS2 voltage for s3202 I2C(1.8V) */
		g_tp->lvs2_regulator = regulator_get(NULL, "touch_i2c");
		if (IS_ERR(g_tp->lvs2_regulator)) {
			rc = PTR_ERR(g_tp->lvs2_regulator);
			pr_err("%s:regulator get lvs2 voltage failed rc=%d\n",
							__func__, rc);
			rc = -ENODEV;
			goto get_lvs2_fail;
		}
		
		/* L9 voltage is 2.85V */
		rc = regulator_set_voltage(g_tp->ldo9_regulator, 2850000, 2850000);
		if (rc) {
			SYNAPTICS_PRINTK(0, "set ldo9_regulator failed, rc=%d\n", rc);
			goto set_ldo9_fail;
		}
	    
		/* L9 enable */
		rc = regulator_enable(g_tp->ldo9_regulator);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to enable ldo9_regulaotr, rc=%d\n", rc);
			goto set_ldo9_fail;
		}
		msleep(5);
		
		/* LVS2 enable */
		rc = regulator_enable(g_tp->lvs2_regulator);
		if (rc) {
			SYNAPTICS_PRINTK(0, "enable lvs2_regulator, rc=%d\n", rc);
			goto set_lvs2_fail;
		}
		msleep(5);
		
		//msleep(SYNAPTICS_POR_DELAY);
		SYNAPTICS_PRINTK(0, "power on device successfully.\n");			
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(g_tp->pmlog_device);
		if (rc)
			SYNAPTICS_PRINTK(0, "[PM_LOG] Failed to device on . rc = %d\n", rc);
#endif	//CONFIG_PM_LOG
		prev_on = on;
		return 0;	/* Successful, do return here. */
	} else {
		SYNAPTICS_PRINTK(0, "Power off device failed. %s, line=#%d \n", __func__, __LINE__);
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_off(g_tp->pmlog_device);
		if (rc)
			SYNAPTICS_PRINTK(0, "[PM_LOG] Failed to device off. rc = %d\n", rc);
#endif //CONFIG_PM_LOG
		prev_on = on;
	}

/* Normal off sequence, also used if any errors */
	regulator_disable(g_tp->lvs2_regulator);
set_lvs2_fail:
	regulator_disable(g_tp->ldo9_regulator);
set_ldo9_fail:
	regulator_put(g_tp->lvs2_regulator);
get_lvs2_fail:
	regulator_put(g_tp->ldo9_regulator);
exit:
	return rc;
}


/* ************************************************************************
 * Description: CREATE_KERNEL_DEBUGLEVEL
 * ************************************************************************ */
static void touchpad_create_kernel_debuglevel(void)
{
	SYNAPTICS_PRINTK(1, "create kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir!=NULL) {
		debugfs_create_u32("ts_flow_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&SYNAPTICS_DLL));
		debugfs_create_u32("ts_event_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&SYNAPTICS_TOUCH_REPORT_DLL));
		/* CHECK IF HW ID and Project ID */
		debugfs_create_u32("key_event_dll", S_IRUGO | S_IWUGO,
				kernel_debuglevel_dir, (u32 *)(&SYNAPTICS_CAPKEY_REPORT_DLL));	
	} else {
		printk(KERN_ERR "failed to create SYNAPTICS mXT224E touch dll in kernel_debuglevel_dir!!!\n");
	}

}

/* **************************************************************************
 * Description: DESTROY_KERNEL_DEBUGLEVEL
 * ************************************************************************ */
static void touchpad_destroy_kernel_debuglevel(void)
{
	SYNAPTICS_PRINTK(1, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}

/* ***************************************************************************
 * add debugfs for self test F54 set report size
 * ************************************************************************* */
static int touchpad_F54_set_report_size(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	int rx = g_tp->number_of_rx_electrodes;
    int tx = g_tp->number_of_tx_electrodes;
	
	switch (g_tp->report_type) {
	case F54_HIGH_RESISTANCE:
		g_tp->report_size = F54_HIGH_RESISTANCE_READ_BYTES / 2;
		break;
	case F54_TX_TX_SHORT:
		g_tp->report_size = F54_TX_TX_SHORT_READ_BYTES;
		break;
	case F54_RAW_CAPACITANCE:
		g_tp->report_size = 2 * rx * tx;
		break;
	case F54_RX_RX_SHORT_7:
		g_tp->report_size = 2 * rx * tx;
		break;
	case F54_RX_RX_REPORT_17:
		g_tp->report_size = 2 * rx * (rx - tx);
		break;		
	default:
		g_tp->report_size = 0;
	}
	
	SYNAPTICS_PRINTK(1, "F54 report type:%d ,test size:%d\n",
			(int)g_tp->report_type, g_tp->report_size);	
	return rc;

}

/* ***************************************************************************
 * add debugfs for self test F54 read report type 17
 * ************************************************************************* */
static int touchpad_F54_report_type_17(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j,k=0,m=0;
	uint16_t command;
	uint8_t command_value[2];

	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		// write f54 low index and high index data to 0
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		/* read report data */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &g_tp->report_type17[m],1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, g_tp->report_type17[m], rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1,"report_type17[%d]=0x%x\n", m,g_tp->report_type17[m]);
		m=m+1;
	}
	
	/* print rx rx 17data */
	for (i = 0; i < (g_tp->number_of_rx_electrodes - g_tp->number_of_tx_electrodes); i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			g_tp->rx_rx_imagearray_17[i][j] = (g_tp->report_type17[k] | (g_tp->report_type17[k+1] << 8));
			SYNAPTICS_PRINTK(1,"data17[%d][%d]= %4d", i, j, (int)g_tp->rx_rx_imagearray_17[i][j]);
			k = k + 2;
	   }
	   SYNAPTICS_PRINTK(1,"\n");
	}
#if 0
	/* print rx rx data */
	for (i = 0; i < (g_tp->number_of_rx_electrodes - g_tp->number_of_tx_electrodes); i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			SYNAPTICS_PRINTK(0,"data17[%d][%d]= %4d", i, j, (int)g_tp->rx_rx_imagearray_17[i][j]);
		}
		printk("\n");
	}
#endif	
	return rc;
}

/* ***************************************************************************
 * add debugfs for self test F54 read report type 7
 * ***************************************************************************/
static int touchpad_F54_report_type_7(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j,k=0,m=0;
	uint16_t command;
	uint8_t command_value[2];

	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		//write f54 low index and high index data to 0
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		/* read report data */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &g_tp->report_type7[m],1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, g_tp->report_type7[m], rc);
			return rc;
		}
		m=m+1;
	}

	/* print rx rx 7 data */
	for (i = 0; i < g_tp->number_of_tx_electrodes; i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			g_tp->rx_rx_imagearray_7[i][j] = (g_tp->report_type7[k] | (g_tp->report_type7[k+1] << 8));
			SYNAPTICS_PRINTK(1,"data7[%d][%d]= %4d", i, j, (int)g_tp->rx_rx_imagearray_7[i][j]);
			k = k + 2;			
	   }
	   SYNAPTICS_PRINTK(1,"\n");
	}
	#if 0
	/* print rx rx data */
	for (i = 0; i <g_tp->number_of_tx_electrodes; i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			
		}
		printk("\n");
	}
	#endif
	return rc;
}

/* ***************************************************************************
 * add debugfs for self test F54 raw capacitance
 * ***************************************************************************/
static int touchpad_F54_raw_capacitance(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j=0,k=0;
	uint16_t command;
	uint8_t command_value[2];
	int count = 0;
	
	SYNAPTICS_PRINTK(0,"f54_raw_cap_pass_spec=%d ~ %d\n", g_tp->f54_raw_cap_min_limit, g_tp->f54_raw_cap_max_limit);
	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		// write f54 low index and high index data to 0
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		/* read FIFO data */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &g_tp->raw_cap_value[j],1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, g_tp->raw_cap_value[j], rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1," raw_cap_value[%d]=0x%x\n", j,g_tp->raw_cap_value[j]);
		j=j+1;
	}
	
	/* print raw capacitance data */
	for (i = 0; i < g_tp->number_of_tx_electrodes; i++) {
		SYNAPTICS_PRINTK(0, "[%d]: ",i);
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			g_tp->raw_cap[i][j] = (g_tp->raw_cap_value[k] | (g_tp->raw_cap_value[k+1] << 8));
			k = k + 2;
			printk("  %4d  ", (int)g_tp->raw_cap[i][j]);
		}
		printk("\n");
	}
	
	/* Check against test limits */
	for (i = 0; i < g_tp->number_of_tx_electrodes; i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			/* Remove check hw id and project id, we don't need to check that in sailfish. 20130820*/
			#if 0
			if ( msm_project_id <= EGYPT && system_rev >= EVT0) {
				if (i == 0) {
					//touch and key area
					if (g_tp->raw_cap[i][j] >= g_tp->f54_raw_cap_min_limit &&
						g_tp->raw_cap[i][j] <= g_tp->f54_raw_cap_max_limit) {
						count += 0;
						SYNAPTICS_PRINTK(1,"raw_cap_value[%d][%d]= %d Pass\n", i,j,(int)g_tp->raw_cap[i][j]);
					} else {
						count += 1;
						SYNAPTICS_PRINTK(0,"raw[%d][%d]= %d (fail)\n", i,j,(int)g_tp->raw_cap[i][j]);
					}
				} else {
					if (j < (g_tp->number_of_rx_electrodes - KEY_RX_CHANNEL)) {
						//touch area
						if (g_tp->raw_cap[i][j] >= g_tp->f54_raw_cap_min_limit &&
							g_tp->raw_cap[i][j] <= g_tp->f54_raw_cap_max_limit) {
							count += 0;
							SYNAPTICS_PRINTK(1,"raw_cap_value[%d][%d]= %d Pass\n", i,j,(int)g_tp->raw_cap[i][j]);
						} else {
							count += 1;
							SYNAPTICS_PRINTK(0,"raw[%d][%d]= %d (fail)\n", i,j,(int)g_tp->raw_cap[i][j]);
						}
					} else {
						//skip other area
					}
				}
			}
			#endif	
			//else if ( msm_project_id == SAPPORO && system_rev >= EVT0) {
				// touch AA area without key in Sapporo project
				if (g_tp->raw_cap[i][j] >= g_tp->f54_raw_cap_min_limit &&
					g_tp->raw_cap[i][j] <= g_tp->f54_raw_cap_max_limit) {
					count += 0;
					SYNAPTICS_PRINTK(1,"raw_cap_value[%d][%d]= %d Pass\n", i,j,(int)g_tp->raw_cap[i][j]);
				} else {
					count += 1;
					SYNAPTICS_PRINTK(0,"raw[%d][%d]= %d (fail)\n", i,j,(int)g_tp->raw_cap[i][j]);
				}
			//} else {
				//unknown project
			//}
		}
	}

	if (count == 0) {
		g_tp->F54_testing_flag += 0;
		SYNAPTICS_PRINTK(0,"raw capacitance testing Pass.\n");
	} else {
		g_tp->F54_testing_flag += RAW_CAPACITANCE;
		SYNAPTICS_PRINTK(0,"raw capacitance testing Fail.\n");
	}
	return rc;
}

/* *************************************************************************
 * add debugfs for self test F54 TX_TX short
 * ************************************************************************* */
static int touchpad_F54_TX_TX_short(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i = 0;
	int count = 0;
	uint16_t command;
	uint8_t command_value[2],tx_tx_value[g_tp->report_size];

	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		// write f54 low index and high index data to 0
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		/* read report data */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &tx_tx_value[i],1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, tx_tx_value[i], rc);
			return rc;
		}
		i=i+1;
	}
	
	/* Check against test limits */
	for (i = 0; i < g_tp->report_size; i++) {
		if (tx_tx_value[i] == 0x0) {
			count += 0;
			SYNAPTICS_PRINTK(0,"Tx-Tx value[%d]=%d\n",i,tx_tx_value[i]);
		} else {
			count += 1;
			SYNAPTICS_PRINTK(0,"Tx-Tx[%d]=%d (fail).\n",i,tx_tx_value[i]);
		}
	}
	
	if (count == 0) {
		g_tp->F54_testing_flag += 0;
		SYNAPTICS_PRINTK(0,"Tx-Tx short testing Pass.\n");
	} else {
		g_tp->F54_testing_flag += TX_TX_SHORT_ERR;
		SYNAPTICS_PRINTK(0,"Tx-Tx short testing Fail.\n");
	}
	return rc;
}

/* *************************************************************************
 * add debugfs for self test F54 hgih_resistance
 * ************************************************************************* */
static int touchpad_F54_hgih_resistance(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j=0,k=0;
	uint16_t command,high_resistance[g_tp->report_size];
	uint8_t command_value[2],high_resistance_value[F54_HIGH_RESISTANCE_READ_BYTES];

	for (command = 0x0000; command < F54_HIGH_RESISTANCE_READ_BYTES; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		// write f54 low index and high index data to 0
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		/* read report data */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &high_resistance_value[j],1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, high_resistance_value[j], rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1,"high resistance value[%d]=0x%x\n", j,high_resistance_value[j]);
		j=j+1;
	}
	
	/* print high resistance */
	for (i = 0; i < g_tp->report_size; i++) 
	{
		high_resistance[i] = (high_resistance_value[k] | (high_resistance_value[k+1] << 8));
		SYNAPTICS_PRINTK(0,"high_resistance[%d]=%d\n", i,(int)high_resistance[i]);
		k = k + 2;
	}
	g_tp->F54_testing_flag += 0;
#if 0
	for (i = 0; i < g_tp->report_size; i++) {
		high_resistance[i] = (high_resistance_value[k] | (high_resistance_value[k+1] << 8));
		if (i <= 1) {
			if ((int)high_resistance[i] > F54_HIGH_RESISTANCE_RX_LIMIT) {
				g_tp->F54_testing_flag = HIGH_RESISTANCE_ERR;
				SYNAPTICS_PRINTK(0,"high_resistance[%d]=%d (fail)\n", i,(int)high_resistance[i]);
			}else {
				g_tp->F54_testing_flag += 0;
				SYNAPTICS_PRINTK(0,"high_resistance[%d]=%d\n", i,(int)high_resistance[i]);
			} 
		} else {
			if (high_resistance[i] < F54_HIGH_RESISTANCE_MIN_LIMIT) {
				g_tp->F54_testing_flag = HIGH_RESISTANCE_ERR;
				SYNAPTICS_PRINTK(0,"high_resistance[%d]=%d (fail)\n", i,(int)high_resistance[i]);
			} else {
				g_tp->F54_testing_flag += 0;
				SYNAPTICS_PRINTK(0,"high_resistance[%d]=%d\n", i,(int)high_resistance[i]);
			}
        }
		k = k + 2;
	}

	/* Check against test limits */
	if (g_tp->F54_testing_flag == 0)
	{
		SYNAPTICS_PRINTK(0,"high resistance testing Pass.\n");
	}	
	else
	{
		SYNAPTICS_PRINTK(0,"high resistance testing Fail.\n");
	}
#endif	
	return rc;
}

/* *************************************************************************
 * add debugfs for F54 testing
 * ************************************************************************* */
static int touchpad_F54_testing(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
    uint8_t page_select,get_report,irq_cmd,interrupt_enable;
	uint8_t force_k,force_update,noise_mitigation_ctrl_addr=0x0,noise_mitigation_ctrl;
	uint8_t cbc_2d_val = 0x0,cbc_0d_val = 0x0,cbc_0d_addr = 0x0,cbc_2d_addr = 0x0,key_cbc_val = 0x0,touch_cbc_val = 0x0;
	
	/* write page number 0x00 */
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
	/* read f01 ctrl1 */
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_control_base+1, &interrupt_enable, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read interrupt enable[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, interrupt_enable, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "read interrupt enable[0x%x]:0x%x\n", g_tp->pdt_map.F01_control_base+1, interrupt_enable);
	/* Enabling only the analog image reporting interrupt, and turn off the rest */
	irq_cmd = 0x00;
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base+1, &irq_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write interrupt enable[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, irq_cmd, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "write interrupt enable[0x%x]:0x%x\n", g_tp->pdt_map.F01_control_base+1, irq_cmd);
	/* read f01 ctrl1 */
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_control_base+1, &irq_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read interrupt enable[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, irq_cmd, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "read interrupt enable[0x%x]:0x%x\n", g_tp->pdt_map.F01_control_base+1, irq_cmd);
	/* write page number 0x01 */
	page_select = 0x01;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
    rc = touchpad_read_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	
	/* Set report mode to run the AutoScan */
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base, &g_tp->report_type, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write report type[0x%x]:%d (rc=%d)\n",
			g_tp->pdt_map.F54_data_base, (int)g_tp->report_type, rc);
		return rc;
	}
	msleep(1);
	
	/* set report size */
	rc = touchpad_F54_set_report_size(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to set report size(rc=%d)\n", rc);
		return rc;
	}
	msleep(1);
	
	if (g_tp->report_type == F54_RX_RX_SHORT_7 || g_tp->report_type == F54_RX_RX_REPORT_17)
	{
		//step1. disable CBC for 2D sensing by setting the CBC Transmitter Carrier Selection bits to '0'
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
				(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
			cbc_2d_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL07;
		rc = touchpad_read_i2c(client, cbc_2d_addr, &touch_cbc_val, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read 2D_CBC_val[0x%x]:0x%x (rc=%d)\n",
				cbc_2d_addr, touch_cbc_val, rc);
			return rc;
		}
		cbc_2d_val = 0x0;
		rc = touchpad_write_i2c(client, cbc_2d_addr, &cbc_2d_val, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write CBC_2D_val[0x%x]:0x%x (rc=%d)\n",
				cbc_2d_addr, cbc_2d_val, rc);
			return rc;
		}
		
		//step2. disable 0D_CBC sensing by setting the CBC Transmitter Carrier Selection bits to '0'
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
				(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
			cbc_0d_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL57;
		rc = touchpad_read_i2c(client, cbc_0d_addr, &key_cbc_val, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read 0D_CBC[0x%x]:0x%x (rc=%d)\n",
				cbc_0d_addr, key_cbc_val, rc);
			return rc;
		}
		cbc_0d_val = 0x0;
		rc = touchpad_write_i2c(client, cbc_0d_addr, &cbc_0d_val, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write 0D_CBC[0x%x]:0x%x (rc=%d)\n",
				cbc_0d_addr, cbc_0d_val, rc);
			return rc;
		}
		
		//step3. disable noise mitigation ctrl
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER ||
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
				(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		noise_mitigation_ctrl = 0x1;
		rc = touchpad_write_i2c(client, noise_mitigation_ctrl_addr, &noise_mitigation_ctrl, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to disable noise mitigation ctrl[0x%x]:0x%x (rc=%d)\n",
				noise_mitigation_ctrl_addr, noise_mitigation_ctrl, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "disable noise mitigation ctrl[0x%x]:0x%x\n", noise_mitigation_ctrl_addr, noise_mitigation_ctrl);
		msleep(1);
		
		//step4. force update
		force_update = 0x04;
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_command_base, &force_update, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to force update[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_command_base, force_update, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "force update[0x%x]:0x%x\n", g_tp->pdt_map.F54_command_base, force_update);
		
		// Wait until the command is completed
		do {
				rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_command_base, &force_update, 1);
				if (rc) {
					SYNAPTICS_PRINTK(0, "failed to read f54 cmd base[0x%x]:0x%x (rc=%d)\n",
					g_tp->pdt_map.F54_command_base, force_update, rc);
					return rc;
				}
				SYNAPTICS_PRINTK(0, "force_update[0x%x]:0x%x\n", g_tp->pdt_map.F54_command_base, force_update);
			} while (force_update != 0x0);
		
		//step5. force calibration
		force_k = 0x02;
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_command_base, &force_k, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to force calibration[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F54_command_base, force_k, rc);
			SYNAPTICS_PRINTK(0, "force calibration[0x%x]:0x%x\n", g_tp->pdt_map.F54_command_base, force_k);
			return rc;
		}
		// Wait until the command is completed
		do {
			rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_command_base, &force_k, 1);
			if (rc) {
				SYNAPTICS_PRINTK(0, "[touch] failed to read f54 cmd base[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_command_base, force_k, rc);
				return rc;
			}
			SYNAPTICS_PRINTK(0, "[touch] read f54 cmd base[0x%x]:0x%x\n", g_tp->pdt_map.F54_command_base, force_k);
		} while (force_k != 0x0);
	}
	
	/* set the GetReport bit to run the AutoScan */
	get_report = 0x01;
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_command_base, &get_report, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write get report[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F54_command_base, get_report, rc);
		return rc;
	}
	msleep(1);
	do {
		// Wait until the command is completed
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_command_base, &get_report, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read get report[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F54_command_base, get_report, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(0, "get report[0x%x]:0x%x\n", g_tp->pdt_map.F54_command_base, get_report);
	} while(get_report!=0x0);	
    
	if (get_report == 0x0) {
		/* read report type */
		switch (g_tp->report_type) {
		case F54_RAW_CAPACITANCE:
			rc = touchpad_F54_raw_capacitance(g_tp);
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to F54 testing report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;	
		case F54_HIGH_RESISTANCE:
			rc = touchpad_F54_hgih_resistance(g_tp);
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to F54 testing report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		case F54_TX_TX_SHORT:
			rc = touchpad_F54_TX_TX_short(g_tp);
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to F54 testing report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		case F54_RX_RX_SHORT_7:
			rc = touchpad_F54_report_type_7(g_tp);
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to F54 testing report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		case F54_RX_RX_REPORT_17:
			rc = touchpad_F54_report_type_17(g_tp);
			if (rc) {
				SYNAPTICS_PRINTK(0, "failed to F54 testing report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		default:
			SYNAPTICS_PRINTK(0, "fail to set F54 report type=%d\n",g_tp->report_type);
		} //end of switch
	}
	
	/* enable noise mitigation ctrl */
	if (g_tp->report_type == F54_RX_RX_SHORT_7 || g_tp->report_type == F54_RX_RX_REPORT_17)
	{
		//restore 2D CBC settings to its original value.
		rc = touchpad_write_i2c(client, cbc_2d_addr, &touch_cbc_val, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read 2D_CBC_val[0x%x]:0x%x (rc=%d)\n",
				cbc_2d_addr, touch_cbc_val, rc);
			return rc;
		}
		
		//restore 0D CBC settings to its original value.
		rc = touchpad_write_i2c(client, cbc_0d_addr, &key_cbc_val, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to read 0D_CBC_val[0x%x]:0x%x (rc=%d)\n",
				cbc_0d_addr, key_cbc_val, rc);
			return rc;
		}
		
		/* enable noise mitigation ctrl */
		#if 0
		if (system_rev <= EVT1_3 && (msm8960_project_id == DETROIT)) {
			if ((int)g_tp->fw_version == SYNAP_DS4_3_0_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + f54_analog_ctrl41;
			else
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		} else {
			if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		}
		#endif
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
				(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		noise_mitigation_ctrl = 0x0;
		rc = touchpad_write_i2c(client, noise_mitigation_ctrl_addr, &noise_mitigation_ctrl, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to enable noise mitigation ctrl[0x%x]:0x%x (rc=%d)\n",
				noise_mitigation_ctrl_addr, noise_mitigation_ctrl, rc);
			return rc;
		}
		SYNAPTICS_PRINTK(1, "enable noise mitigation ctrl[0x%x]:0x%x\n", noise_mitigation_ctrl_addr, noise_mitigation_ctrl);
		msleep(1);
	}
	
	/* write page number 0x00 */
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:page table addr[0x%x]:0x%x\n", PAGE_SELECT_REGISTER, page_select);
	msleep(1);
    rc = touchpad_read_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:page table addr[0x%x]:0x%x\n", PAGE_SELECT_REGISTER, page_select);
	
	/* enable UI space */
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base+1, &interrupt_enable, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write interrupt enable[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, interrupt_enable, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:write interrupt enable[0x%x]:0x%x\n", g_tp->pdt_map.F01_control_base+1, interrupt_enable);
	/* read f01 ctrl1 */
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_control_base+1, &irq_cmd, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to read interrupt enalbe[0x%x]:0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, irq_cmd, rc);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch:read interrupt enable[0x%x]:0x%x\n", g_tp->pdt_map.F01_control_base+1, irq_cmd);
	
	return rc;
}	

/* ***************************************************************************
 * add debugfs for self test F54 RX_RX short
 * ***************************************************************************/
static int touchpad_F54_RX_RX_short(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	int i,j,k=0;
	int count = 0;
	
	/* read report type 7 */
	g_tp->report_type = F54_RX_RX_SHORT_7;	
	rc = touchpad_F54_testing(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to F54 testing RX_RX_short 7(rc=%d)\n",
			rc);
		return rc;
	}
	mdelay(1);
	
	/* read report type 17 */
	g_tp->report_type = F54_RX_RX_REPORT_17;	
	rc = touchpad_F54_testing(g_tp);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to F54 testing RX_RX_short 17(rc=%d)\n",
			rc);
		return rc;
	}
	mdelay(1);
    
	/* Check against test limits */
	for (i = 0; i < g_tp->number_of_rx_electrodes; i++) {
		SYNAPTICS_PRINTK(1, "rx_rx_data[%d]",i);
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			if (i < g_tp->number_of_tx_electrodes) {
				g_tp->rx_rx_imagearray[i][j] = g_tp->rx_rx_imagearray_7[k][j];
				SYNAPTICS_PRINTK(1, " %2d",(int)g_tp->rx_rx_imagearray[i][j]);
			}else {
				if (k == g_tp->number_of_tx_electrodes)
					k = 0;
				g_tp->rx_rx_imagearray[i][j] = g_tp->rx_rx_imagearray_17[k][j];
				SYNAPTICS_PRINTK(1, "  %2d",(int)g_tp->rx_rx_imagearray[i][j]);
			}	
		}
		k = k + 1;
		SYNAPTICS_PRINTK(1,"\n");
	}

	for (i = 0; i < g_tp->number_of_rx_electrodes; i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			if (i == j){ 
			    if ((int)g_tp->rx_rx_imagearray[i][j] < F54_RX_RX_MAX_LIMIT && (int)g_tp->rx_rx_imagearray[i][j] > F54_RX_RX_MIN_LIMIT)
				{
					count += 0;
					SYNAPTICS_PRINTK(0,"rx-rx[%d][%d]=%d", i,j,(int)g_tp->rx_rx_imagearray[i][j]);
					
				}else {
					count += 1;
					SYNAPTICS_PRINTK(0,"rx-rx[%d][%d]=%d (fail)", i,j,(int)g_tp->rx_rx_imagearray[i][j]);
				}	
			}
		}
		printk("\n");
	}
	if (count == 0) {
		g_tp->F54_testing_flag += 0;
		SYNAPTICS_PRINTK(0,"Rx-Rx short testing Pass.\n");
	} else {
		g_tp->F54_testing_flag += RX_RX_SHORT_ERR;
		SYNAPTICS_PRINTK(0,"Rx-Rx short testing Fail.\n");
	}
	
	return rc;
}

/* *************************************************************************
 * add debugfs for slef test
 * ************************************************************************* */
static int tp_selftest_all(void *data, u64 *val)
{
	struct synaptics_tp_t *g_tp = (struct synaptics_tp_t *)data;
	int rc = 0;
	uint8_t cmd;

	SYNAPTICS_PRINTK(0, "self-test all.\n");
	mutex_lock(&g_tp->mutex);
	g_tp->F54_testing_flag = 0;

	SYNAPTICS_PRINTK(0, "touch_info(%d,%d)\n", (int)g_tp->fw_version, g_tp->config_id);
	SYNAPTICS_PRINTK(0, "total number of TX(%d) * Rx(%d) channels\n",
		g_tp->number_of_rx_electrodes, g_tp->number_of_tx_electrodes);
	touchpad_get_raw_cap_pass_sepc();
	if (g_tp->number_of_rx_electrodes != NUM_OF_RX_ELECTRODES || g_tp->number_of_tx_electrodes != NUM_OF_TX_ELECTRODES) 
	{
		SYNAPTICS_PRINTK(0, "the number of rx channels(= %d) or the number of tx channels(= %d) are abnormal. (rc=%d)\n",
			g_tp->number_of_rx_electrodes, g_tp->number_of_tx_electrodes, rc);
		g_tp->F54_testing_flag += TX_RX_CHANNEL_ERR;
		/* check if self test pass/fail criteria */
		if(g_tp->F54_testing_flag == 0) 
		{
			SYNAPTICS_PRINTK(0, "self test all pass.(test result=%d)\n",SELF_TEST_ALL_PASS);  
			*val = SELF_TEST_ALL_PASS;
		} else {
			SYNAPTICS_PRINTK(0, "self test all Fail.(test result=%d)\n",g_tp->F54_testing_flag);
			*val = g_tp->F54_testing_flag;
		}
		mutex_unlock(&g_tp->mutex);
		return rc;
	} else {
		/* slef test for F54 testing high resistance */
		SYNAPTICS_PRINTK(0, "high resistance testing.\n");
		g_tp->report_type = F54_HIGH_RESISTANCE;	
		rc = touchpad_F54_testing(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to F54 testing high resistance(rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
		
		/* slef test for F54 Tx Tx short */
		SYNAPTICS_PRINTK(0, "TX-TX short testing.\n");
		g_tp->report_type = F54_TX_TX_SHORT;
		rc = touchpad_F54_testing(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to F54 testing Tx-Tx short(rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
	
		/* slef test for F54 testing raw capacitance */
		SYNAPTICS_PRINTK(0, "raw capacitance testing.\n");
		g_tp->report_type = F54_RAW_CAPACITANCE;
		rc = touchpad_F54_testing(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to F54 testing raw capacitance(rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
	
		/* slef test for F54 testing RX RX short */
		SYNAPTICS_PRINTK(0, "RX-RX short testing.\n");
		rc = touchpad_F54_RX_RX_short(g_tp);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to F54 testing Rx-Rx short(rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
		/* check if self test pass/fail criteria */
		if(g_tp->F54_testing_flag == 0) 
		{
			SYNAPTICS_PRINTK(0, "self test all pass.(test result=%d)\n",SELF_TEST_ALL_PASS);  
			*val = SELF_TEST_ALL_PASS;
		} else {
			SYNAPTICS_PRINTK(0, "self test all Fail.(test result=%d)\n",g_tp->F54_testing_flag);
			*val = g_tp->F54_testing_flag;
		}
	
		/* sw reset */
		cmd = 0x1;
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F01_command_base, &cmd, 1);
		if (rc) {
			SYNAPTICS_PRINTK(0, "failed to write sw reset[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F01_command_base, cmd, rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
		msleep(500);
	}
	/* disable worker queue */
	//enable_irq(g_tp->irq);
	mutex_unlock(&g_tp->mutex);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_selftest, tp_selftest_all, NULL, "%llu")

/* *************************************************************************
 * add debugfs for ftd test to get device id
 * ************************************************************************* */
static ssize_t tp_read_device_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = container_of( dev, struct i2c_client, dev);
	int rc;
    uint8_t page_select = 0x0;
	uint8_t device_id_value;
	/* Page Descriptionc Table */
	mutex_lock(&g_tp->mutex);
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		mutex_unlock(&g_tp->mutex);	
		return rc;
	}
	SYNAPTICS_PRINTK(1, "tcouh: page table addr[0x%x]:0x%x\n", PAGE_SELECT_REGISTER, page_select);
	mdelay(1);
    
	/* config register map dynamiclly */
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_query_base, &device_id_value, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics device id[0x%x]=0x%x (rc=%d)\n",
			g_tp->pdt_map.F01_query_base, device_id_value, rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	SYNAPTICS_PRINTK(1, "touch: device id[0x%x]=0x%x\n", g_tp->pdt_map.F01_query_base, device_id_value);
	
	if(device_id_value == 0x01)
	{
		mutex_unlock(&g_tp->mutex);
		return snprintf(buf, 11, "%d\n",device_id_value);
	}	
	else 
	{
		mutex_unlock(&g_tp->mutex);
		return -EINVAL;
	}	
}
static DEVICE_ATTR(device_id, 0444, tp_read_device_id, NULL);

/* *************************************************************************
 * add debugfs for enabling/disabling touch panel
 * ************************************************************************* */
static int dbg_tp_onoff_get(void *data, u64 *value)
{
	struct synaptics_tp_t *g_tp = (struct synaptics_tp_t *)data;
	int state = 0;
	SYNAPTICS_PRINTK(1, "%s: %s touch panel\n",
		__func__, g_tp->is_earlysuspended ? "disable":"enable");
	if (g_tp->is_earlysuspended == 1)
		state = 0;
	else if (g_tp->is_earlysuspended == 0)
		state = 1;
		
	*value = state;
	return 0;
}

static int dbg_tp_onoff_set(void *data, u64 value)
{
	struct synaptics_tp_t *g_tp = (struct synaptics_tp_t *)data;

	if (value == 0) {
		synaptics_early_suspend(&g_tp->ts_early_suspend);
		SYNAPTICS_PRINTK(1,"%s, disable touch panel\n",__func__);
	} else if (value == 1) {
		synaptics_late_resume(&g_tp->ts_early_suspend);
		SYNAPTICS_PRINTK(1,"%s, enable touch panel\n",__func__);
	} else {
			SYNAPTICS_PRINTK(0, "%s: Error command(%lld)\n",
				__func__, value);
	}
			
	SYNAPTICS_PRINTK(1, "%s: %s touch panel\n",
		__func__, g_tp->is_earlysuspended ? "disable":"enable");
	
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tp_onoff_fops, dbg_tp_onoff_get, dbg_tp_onoff_set, "%llu\n");

/* *************************************************************************
 * add debugfs for reading FW version
 * ************************************************************************* */
static ssize_t tp_read_fw_ver(struct device *dev, struct device_attribute *attr,char *buf)
{
	if (g_tp->fw_version!=0x00)
		return snprintf(buf, 11, "%d\n",g_tp->fw_version);
	else
		return -EINVAL;
}
static DEVICE_ATTR(tp_fw_ver, 0444, tp_read_fw_ver, NULL);

/* *************************************************************************
 * add debugfs for reading config id
 * ************************************************************************* */
static ssize_t tp_read_config_id(struct device *dev, struct device_attribute *attr,char *buf)
{

	if(g_tp->config_id != 0x0) 
		return snprintf(buf, 11, "%d\n",g_tp->config_id);
	else
		return -EINVAL;
}
static DEVICE_ATTR(tp_config_id, 0444, tp_read_config_id, NULL);

/* *************************************************************************
 * add debugfs for reading product id
 * ************************************************************************* */
static ssize_t tp_read_product_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = container_of( dev, struct i2c_client, dev);
	int rc = 0;
    uint8_t page_select;
	uint8_t pid_addr,pid_value[11];
	
	pid_value[10] = 0;
	mutex_lock(&g_tp->mutex);
	/* Page Descriptionc Table */
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		SYNAPTICS_PRINTK(0, "failed to write synaptics page table addr[0x%x]:0x%x (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	SYNAPTICS_PRINTK(0, "tcouh: page table addr[0x%x]:0x%x\n", PAGE_SELECT_REGISTER, page_select);
	mdelay(1);
	
	
	pid_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query11;
	rc = touchpad_read_i2c(g_tp->client, pid_addr, &pid_value[0], 10);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to read product id. addr[0x%x]:0x%x (rc=%d)\n", pid_addr, pid_value[0], rc);
		mutex_unlock(&g_tp->mutex);
		return INVALID_TP;
	}
	mutex_unlock(&g_tp->mutex);
	return snprintf(buf, 11, "%s\n",pid_value);
}
static DEVICE_ATTR(tp_product_id, 0444, tp_read_product_id, NULL);
/* *************************************************************************
 * add sysfs API for enable double tap feature
 * ************************************************************************* */
static ssize_t tp_dtap_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", g_tp->dtap_allowed);
}

static ssize_t tp_dtap_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

	unsigned int new_value;
	int ret = kstrtouint(buf, 10, &new_value);
	if (ret < 0)
		return ret;

	new_value = !!new_value;

	mutex_lock(&g_tp->mutex);
	if (g_tp->dtap_allowed != new_value && g_tp->is_earlysuspended) {
		if (new_value) {
			/* Enable double tap wake up */
			touchpad_set_dtap_mode(1);
			touchpad_set_sleep_mode(0);
		} else {
			/* Disable double tap wake up and put touch to sleep */
			touchpad_set_sleep_mode(1);
			touchpad_set_dtap_mode(0);
		}
	}
	g_tp->dtap_allowed = new_value;
	mutex_unlock(&g_tp->mutex);

	return count;
}

static DEVICE_ATTR(double_tap_enable, 0644, tp_dtap_show, tp_dtap_store);
/* *************************************************************************
 * add sysfs API for palm detect
 * ************************************************************************* */
static ssize_t palm_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t rc;
	uint8_t f11_2d_data28_val;

	mutex_lock(&g_tp->mutex);
        if(g_tp->is_earlysuspended == 0)
	{
		touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F11_data_base+DS4_3_6_F11_2D_DATA28, &f11_2d_data28_val, 1);
		rc = snprintf(buf, PAGE_SIZE, "%u\n", f11_2d_data28_val);
		printk("f11_2d_data28_val = %x\n",f11_2d_data28_val);
	}
	else
	{
		rc = snprintf(buf, PAGE_SIZE, "%u\n", 0);
		printk("f11_2d_data28_val = %x\n",0);
	}
        mutex_unlock(&g_tp->mutex);
	return rc;
}

static ssize_t palm_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
        return -1;
}

static DEVICE_ATTR(palm_status, 0644, palm_show, palm_store);


#define TOUCH_DRIVER_NAME	"tp_s3202_dbg"
/* *************************************************************************
 * Description: DEBUGFS FOR FTD TEST TO GET DEVICE ID
 * ************************************************************************* */
static void touchpad_create_debugfs_entries(struct synaptics_tp_t *g_tp)
{
	g_tp->dent = debugfs_create_dir(TOUCH_DRIVER_NAME, NULL);
	if (g_tp->dent) {
		debugfs_create_file("tp_onoff", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_onoff_fops);
		/* tp F54 testing */
		debugfs_create_file("tp_self_test", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_selftest);
	}
}

/* *************************************************************************
 * Description: SYS FOR FTD TEST TO GET DEVICE ID
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

	ret = device_create_file(&client->dev,&dev_attr_tp_config_id);
	WARN_ON(ret); 
	if (ret)  return ret;
	
	ret = device_create_file(&client->dev,&dev_attr_tp_product_id);
	WARN_ON(ret); 
	if (ret)  return ret;
	
	ret = device_create_file(&client->dev,&dev_attr_double_tap_enable);
	WARN_ON(ret);
	if (ret)  return ret;
	
    ret = device_create_file(&client->dev,&dev_attr_palm_status);
    WARN_ON(ret);
	
	return ret;
}

#ifdef CONFIG_PM
/*add for double tap gesture, 20130801  { */
static int synaptics_suspend(struct device *dev)
{
	SYNAPTICS_PRINTK(0, "%s() +++\n", __func__);
	mutex_lock(&g_tp->mutex);
	g_tp->is_suspended = 1;
	/* add sysfs API for enable double tap feature */
	if (device_may_wakeup(dev) && g_tp->dtap_allowed)
		enable_irq_wake(g_tp->irq);
	mutex_unlock(&g_tp->mutex);
	SYNAPTICS_PRINTK(0, "%s() ---\n", __func__);
	return 0;
}

static int synaptics_resume(struct device *dev)
{
	int rc = 0;
	uint8_t inrt_status, f11_2D_ctrl0;
	
	SYNAPTICS_PRINTK(0, "%s() +++\n", __func__);
	mutex_lock(&g_tp->mutex);
	g_tp->is_suspended = 0;
	/* add sysfs API for enable double tap feature */
	if (device_may_wakeup(dev) && g_tp->dtap_allowed)
		disable_irq_wake(g_tp->irq);
	if(g_tp->irq_pending) {
		g_tp->irq_pending = 0;
		SYNAPTICS_PRINTK(0, "will send power key event\n");
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F11_control_base, &f11_2D_ctrl0, 1);
		/* read interrupt status */
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &inrt_status, 1);
		if (((f11_2D_ctrl0 == 0xc) && ((inrt_status & 0x4) == 0x4))&& g_tp->dtap_allowed) {
			//report power key event to notify power manager service to wake up system
			input_report_key(g_tp->keyarray_input, KEY_POWER, 1);
			input_sync(g_tp->keyarray_input);
			input_report_key(g_tp->keyarray_input, KEY_POWER, 0);
			input_sync(g_tp->keyarray_input);
			SYNAPTICS_PRINTK(0, "sent power key event\n");
		}
		enable_irq(g_tp->irq);
	}
	mutex_unlock(&g_tp->mutex);
	SYNAPTICS_PRINTK(0, "%s() ---\n", __func__);
	return 0;
}
/* } add for double tap gesture, 20130801 */

static int synaptics_suspend_noirq(struct device *dev)
{
	int rc = 0;
	SYNAPTICS_PRINTK(0, "%s() +++\n", __func__);
    	mutex_lock(&g_tp->mutex);
	if (g_tp->irq_pending) {
		SYNAPTICS_PRINTK(0, "double tap irq pending ret -1\n");
		rc = -1;
	} 
	mutex_unlock(&g_tp->mutex);
    	SYNAPTICS_PRINTK(0, "%s() ---\n", __func__);
	return rc;
}
static const struct dev_pm_ops ts_s3202_pm_ops = {
	.suspend_noirq = synaptics_suspend_noirq,
	.suspend  = synaptics_suspend,
	.resume   = synaptics_resume,
};
#endif
/* } add for double tap gesture, 20130801 */

static const struct i2c_device_id i2cSYNAPTICSTouch_idtable[] = {
	{ SYNAPTICS_TP_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, i2cSYNAPTICSTouch_idtable);

static struct i2c_driver i2c_touchpad_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SYNAPTICS_TP_NAME,
/*add for double tap gesture, 20130801 { */
#ifdef CONFIG_PM
		.pm = &ts_s3202_pm_ops,
#endif
/* } add for double tap gesture, 20130801 */
	},
	.probe	 = touchpad_probe,
	.remove	 = touchpad_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = synaptics_early_suspend,
    .resume  = synaptics_late_resume,
#endif
	.id_table = i2cSYNAPTICSTouch_idtable,
};


static int proc_detect_tp_chip_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_tp->client->addr);
}

static int proc_tp_config_id_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_tp->config_id);
}

static int proc_tp_product_id_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	
	uint8_t pid_addr,pid_value[11];
	int rc = 0;
	
	mutex_lock(&g_tp->mutex);
	pid_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query11;
	rc = touchpad_read_i2c(g_tp->client, pid_addr, &pid_value[0], 10);
	if (rc < 0) {
		SYNAPTICS_PRINTK(0, "touch:Failed to read product id. addr[0x%x]:0x%x (rc=%d)\n", pid_addr, pid_value[0], rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	mutex_unlock(&g_tp->mutex);
	*eof = 1;
	return sprintf(page, "%s\n", pid_value);
}

static int proc_tp_module_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", gpio_get_value(g_tp->gpio_vendor_id));
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

static int __devinit touchpad_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int      result = 0;
	struct   synaptics_tp_platform_data_t *pdata;
	
	SYNAPTICS_PRINTK(1,"%s %d\n",__func__,__LINE__);
	/* allocate and clear memory */
	g_tp = kzalloc(sizeof(struct synaptics_tp_t), GFP_KERNEL);
    if(!g_tp) {
        result = -ENOMEM;
        return result;
    }
    
	/* register driver_data */
    pdata = client->dev.platform_data;
    g_tp->gpio_irq = pdata->gpio_irq;
    g_tp->gpio_rst = pdata->gpio_rst;
	g_tp->gpio_vendor_id = pdata->gpio_vendor_id;
    g_tp->irq = MSM_GPIO_TO_INT(g_tp->gpio_irq);
    g_tp->client = client;
    mutex_init(&g_tp->mutex);
		
#ifdef CONFIG_PM_LOG
	/* Register pm log */
	g_tp->pmlog_device = pmlog_register_device(&client->dev);
	SYNAPTICS_PRINTK(0, "[PM_LOG]register pm log for touch driver.\n");
#endif //CONFIG_PM_LOG
	/* create debugfs */
	//touchpad_create_kernel_debuglevel();
	
	/* set up synaptics irq */
	result = touchpad_setup_gpio(g_tp);
	if(result) {
		SYNAPTICS_PRINTK( 0, "Failed to setup gpio result=%d\n", result);
		goto err_alloc_mem;
	}
	msleep(15);
	
	/* enable vdd & avdd voltage */
    result = touchpad_power_on_device(g_tp, 1);
    if(result) {
    	SYNAPTICS_PRINTK(0, "Unable to power device result=%d\n", result);
		goto err_setup_gpio;
    }
	    
	/* configure up cyttsp rst gpio */
    result = touchpad_config_gpio(g_tp);
    if(result) {
        SYNAPTICS_PRINTK(0, "Failed to config gpio\n" );
		goto err_power_device;
    }
	
	/* detect synaptics s3202 chip */
	result = touchpad_detect_synaptics(g_tp);
    if(result) {
		SYNAPTICS_PRINTK(0, "Failed to detect Synaptics S3202 chip result=%d\n", result);
		goto err_power_device;
    }	else {
		/* create debugfs */
		touchpad_create_kernel_debuglevel();
		touch_panel_detected = 1; //detect s3202 successfully
	}	
	client->driver = &i2c_touchpad_driver;
	i2c_set_clientdata(client, g_tp);
	
	/* Create Synaptics S3202 touch input device and register it. */
    result = touchpad_register_input(&g_tp->input, pdata, client);
    if(result) {
    	SYNAPTICS_PRINTK(0, "touch:Failed to register synaptics ts input device (result=%d)\n", result);
		goto err_power_device;
    }
    input_set_drvdata(g_tp->input, g_tp);
	 
	result = keyarray_register_input(&g_tp->keyarray_input, client);
	if(result) {
		SYNAPTICS_PRINTK(0, "touch:Failed to register synaptics keyarray input device (result=%d)\n", result);
        goto err_register_touch_input;
	}
	input_set_drvdata(g_tp->keyarray_input, g_tp);
	
	/* request a irq threaded for Synaptics S3202 */
	result = request_threaded_irq(g_tp->irq, NULL, touchpad_irq_thread,
	IRQF_TRIGGER_LOW | IRQF_ONESHOT, "synaptics_ts", g_tp);
	if (result < 0) {
		SYNAPTICS_PRINTK(0, "touch:failed to request irq:%d (result=%d)\n", g_tp->irq, result);
		/* touch panel with capkey */
		goto err_register_keyarray_input;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	g_tp->ts_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	g_tp->ts_early_suspend.suspend = synaptics_early_suspend;
	g_tp->ts_early_suspend.resume = synaptics_late_resume;
	register_early_suspend(&g_tp->ts_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
	
	/* add debugfs for ftd test to get device id */
	touchpad_create_debugfs_entries(g_tp);
	result = touchpad_create_sys_entries(client);
	/* 20120322 */
	
	/* Install the proc_fs entries */
	result = touchpad_create_procfs_entries(client);
	
	device_init_wakeup(&client->dev, 1);
	SYNAPTICS_PRINTK(0,"touch:Start Probe %s\n",
		(result < 0) ? "FAIL" : "PASS");
	/*add for double tap gesture, 20130801 */
	init_waitqueue_head(&g_tp->wq);

	for (i = 0 ; i < MAX_TS_REPORT_POINTS ; i++)
	{
		g_tp->msg[i].coord.z = -1;
	}
/* Enable double tap feature, 20131018 */
	g_tp->dtap_allowed = 1;
	
	/* Emily jiang, abort suspending procedure if power key event comes in during freezing. 20140218 */
	wake_lock_init(&g_tp->doubletap_wakelock, WAKE_LOCK_SUSPEND, "DoubleTap");
	return 0;

err_register_keyarray_input:
    input_unregister_device(g_tp->keyarray_input);
    input_free_device(g_tp->keyarray_input);
    g_tp->keyarray_input = NULL;
err_register_touch_input:
    input_unregister_device(g_tp->input);
    input_free_device(g_tp->input);
    g_tp->input = NULL;
err_power_device:
	touchpad_power_on_device(g_tp, 0);	
err_setup_gpio:
	touchpad_release_gpio(g_tp);
err_alloc_mem:
	kfree(g_tp);
	result = -EFAULT;
	return result;
}	

/* unregistere device */
static int __devexit touchpad_remove(struct i2c_client *client)
{
	/* clientdata registered on probe */
	struct synaptics_tp_t *g_tp = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	//unregister_early_suspend(&g_ts->ts_early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	//misc_deregister(&ts_misc_device);
	
	/* Emily jiang, abort suspending procedure if power key event comes in during freezing. 20140218 */
	wake_lock_destroy(&g_tp->doubletap_wakelock);
	
	/* Start cleaning up by removing any delayed work and the timer */
	free_irq(g_tp->irq, g_tp);

	input_unregister_device(g_tp->keyarray_input);
    input_free_device(g_tp->keyarray_input);
    g_tp->keyarray_input = NULL;
	
	input_unregister_device(g_tp->input);
    input_free_device(g_tp->input);
    g_tp->input = NULL;
	touchpad_release_gpio(g_tp);
	touchpad_destroy_kernel_debuglevel();
	mutex_destroy(&g_tp->mutex); 
	
	g_tp = NULL;
	kfree(g_tp);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_early_suspend(struct early_suspend *handler)
{
	struct synaptics_tp_t *g_tp = container_of(handler, 
	struct synaptics_tp_t, ts_early_suspend);
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	uint8_t inrt_status;
		
	SYNAPTICS_PRINTK(0, "%s() +++\n", __func__);
	mutex_lock(&g_tp->mutex);
	if (g_tp->is_earlysuspended) {
		SYNAPTICS_PRINTK(1, "Synaptics s3202 is in sleep mode!!!\n");
		mutex_unlock(&g_tp->mutex);
		SYNAPTICS_PRINTK(0, "%s()%d ---\n", __func__,__LINE__);
		return;
	}

	mutex_unlock(&g_tp->mutex);
	disable_irq(g_tp->irq);
	SYNAPTICS_PRINTK(1,"disable irq %d\n", g_tp->irq);
	mutex_lock(&g_tp->mutex);

	/* Clear the touch release events needed flag. Will be set
	 * again if touchpad_set_sleep_mode(1) gets called for any reason
	 * before the resume occurs. */
	generate_touch_release = 0;

	/*add for double tap gesture, 20130801 { */
	if (g_tp->dtap_allowed) {
		touchpad_set_dtap_mode(1);
		touchpad_set_sleep_mode(0);
	}
	else {
		/* setup synaptics tp in sleep mode */
		touchpad_set_sleep_mode(1);

		msleep(50);

		/* clear the isr status bit */
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+1, &inrt_status, 1);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read interrupt status[0x%x]:0x%x (rc=%d)\n",
					 g_tp->pdt_map.F01_data_base+1, inrt_status, rc);
			return;
		}
		SYNAPTICS_PRINTK(0, "isr status = %x\n",inrt_status);
		msleep(20);
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+1, &inrt_status, 1);
		if (rc < 0) {
			SYNAPTICS_PRINTK(0, "Failed to read interrupt status[0x%x]:0x%x (rc=%d)\n",
					 g_tp->pdt_map.F01_data_base+1, inrt_status, rc);
			return;
		}
		SYNAPTICS_PRINTK(0, "isr status = %x\n", inrt_status);
	}
	/*} add for double tap gesture, 20130801 */
	g_tp->is_earlysuspended = 1;
	enable_irq(g_tp->irq);
	/* Add for PM LOG */
#ifdef CONFIG_PM_LOG
	rc = pmlog_device_off(g_tp->pmlog_device);
	if (rc)
		SYNAPTICS_PRINTK(0, "[PM_LOG]touch off ,fail rc = %d\n", rc);
#endif //CONFIG_PM_LOG
	mutex_unlock(&g_tp->mutex);
	SYNAPTICS_PRINTK(0, "%s()%d ---\n", __func__,__LINE__);
	return;
	
}

static void synaptics_late_resume(struct early_suspend *handler)
{
	struct synaptics_tp_t *g_tp = container_of(handler, 
		struct synaptics_tp_t, ts_early_suspend);
	int rc = 0,i;
	
	SYNAPTICS_PRINTK(0, "%s() +++\n", __func__);
	mutex_lock(&g_tp->mutex);

	/* If Sleep mode was activated some time during suspend */
	if (generate_touch_release) {
		generate_touch_release = 0;

		SYNAPTICS_PRINTK(0, "emitting touch release events\n");

		/* reset mt points coordinates after wakeup */
		for (i = 0 ; i < MAX_TS_REPORT_POINTS ; i++)
		{
			g_tp->msg[i].prev_state = g_tp->msg[i].state;
			g_tp->msg[i].state = TS_RELEASE;
			if(g_tp->msg[i].coord.z != -1) {
				g_tp->msg[i].coord.z = 0;
			}
		}
		touchpad_report_mt_protocol(g_tp);
	}
	/*add for double tap gesture, 20130801 { */
	/* disable double tap wake up system for reporting mode */
	if (g_tp->dtap_allowed)
		touchpad_set_dtap_mode(0);
	else
		touchpad_set_sleep_mode(0);

	g_tp->is_earlysuspended = 0;

	/* Add for PM LOG */
#ifdef CONFIG_PM_LOG
	rc = pmlog_device_on(g_tp->pmlog_device);
	if (rc)
		SYNAPTICS_PRINTK(0, "[PM_LOG]touch on ,fail rc = %d\n", rc);
#endif	//CONFIG_PM_LOG
	mutex_unlock(&g_tp->mutex);
	SYNAPTICS_PRINTK(0, "%s()%d ---\n", __func__,__LINE__);
	return ;
	
}
#endif/* CONFIG_HAS_EARLYSUSPEND */

static int __init touchpad_init(void)
{
	int rc = 0;
	
	printk("BootLog, +%s\n", __func__);
	SYNAPTICS_PRINTK(0, "SYNAPTICS RMI4 Touchscreen Driver\n");
	SYNAPTICS_PRINTK(1, "%s, system_rev=0x%x\n", __func__, system_rev);
	SYNAPTICS_PRINTK(1, "%s, msm_project_id=0x%x\n", __func__, msm_project_id);
	if(touch_panel_detected) { 
	  //detect synatics IC failed, do nothing
	 } else { 
		/*init synaptics IC first*/
		i2c_touchpad_driver.driver.name = SYNAPTICS_TP_NAME;
		rc = i2c_add_driver(&i2c_touchpad_driver);
	}
	printk("BootLog, -%s, rc=%d\n", __func__,rc);
    return rc;
}
module_init(touchpad_init);

static void __exit touchpad_exit(void)
{
    i2c_del_driver(&i2c_touchpad_driver);
    SYNAPTICS_PRINTK(0, "SYNAPTICS RMI4 touch driver Exiting\n");
}
module_exit(touchpad_exit);

MODULE_DESCRIPTION("SYNAPTICS RMI4 touchpad driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Emily Jiang");
MODULE_ALIAS("platform:SYNAPTICS_RMI4_touch");
