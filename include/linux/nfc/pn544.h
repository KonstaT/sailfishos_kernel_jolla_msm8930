/*
 * Driver include for the PN544 NFC chip.
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Jari Vanhala <ext-jari.vanhala@nokia.com>
 * Contact: Matti Aaltoenn <matti.j.aaltonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef _PN544_H_
#define _PN544_H_

#include <linux/i2c.h>

#define PN544_DRIVER_NAME	"pn544"
/* Jen Chang add for gpio and i2c address mapping */
/* Fix me -- need to check hw board */
#define PN544_HOST_INT_GPIO		6
#define PN544_ENABLE_GPIO		7
#define PN544_FW_RESET_GPIO		10
#define PN544_I2C_ADDR			0x28
/* -- Fix me */

//add for kernel debug level
enum pn544_debug_level {
	NFC_CRITICIAL = 0,
	NFC_PROBE,
	NFC_DEBUGFS,
	NFC_TEST,
	NFC_INFO,
	NFC_DEBUG,
};

union PN544_DEBUGLEVEL {
	u32 debug_level;
	char debug_byte[4];
};

extern union PN544_DEBUGLEVEL PN544_DEBUG_DLL;

#define PN544_TEST_DLL_ON	(1 << 0)
#define	PN544_INFO_DLL_ON	(1 << 7)
#define	PN544_DEBUG_DLL_ON	(1 << 15)

#ifdef CONFIG_BUILD_FACTORY
#define pn544_printk(level, fmt, args...) \
	do { \
		if (level <= NFC_TEST) printk(fmt, ##args); \
	} while(0)
#else
#define pn544_printk(level, fmt, args...) \
	do { \
		if (PN544_DEBUG_DLL.debug_level >= PN544_DEBUG_DLL_ON) { \
			if (level <= NFC_DEBUG) printk(fmt, ##args); \
		} else if (PN544_DEBUG_DLL.debug_level >= PN544_INFO_DLL_ON) { \
			if (level <= NFC_INFO) printk(fmt, ##args); \
		} else if (PN544_DEBUG_DLL.debug_level >= PN544_TEST_DLL_ON) { \
			if (level <= NFC_TEST) printk(fmt, ##args); \
		} else { \
			if (level <= NFC_DEBUGFS) printk(fmt, ##args); } \
	} while(0)
#endif
/* Jen Chang, 20121217 */

#define PN544_MAXWINDOW_SIZE	7
#define PN544_WINDOW_SIZE	4
#define PN544_RETRIES		10
#define PN544_MAX_I2C_TRANSFER	0x0400
#define PN544_MSG_MAX_SIZE	0x21 /* at normal HCI mode */
#define PN544_MAX_BUFFER_SIZE	512

/* ioctl */
#define PN544_CHAR_BASE		'P'
#define PN544_IOR(num, dtype)	_IOR(PN544_CHAR_BASE, num, dtype)
#define PN544_IOW(num, dtype)	_IOW(PN544_CHAR_BASE, num, dtype)
#define PN544_GET_FW_MODE	PN544_IOW(1, unsigned int)
#define PN544_SET_FW_MODE	PN544_IOW(2, unsigned int)
#define PN544_GET_DEBUG		PN544_IOW(3, unsigned int)
#define PN544_SET_DEBUG		PN544_IOW(4, unsigned int)
#define PN544_GET_SH_VERSION	PN544_IOW(6, unsigned char**)
/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled */
#define PN544_MAGIC	0xE9
#define PN544_SET_PWR	_IOW(PN544_MAGIC, 0x01, unsigned int)

/* Timing restrictions (ms) */
#define PN544_RESETVEN_TIME	30 /* 7 */
#define PN544_PVDDVEN_TIME	0
#define PN544_VBATVEN_TIME	0
#define PN544_GPIO4VEN_TIME	0
#define PN544_WAKEUP_ACK	5
#define PN544_WAKEUP_GUARD	(PN544_WAKEUP_ACK + 1)
#define PN544_INACTIVITY_TIME	1000
#define PN544_INTERFRAME_DELAY	200 /* us */
#define PN544_BAUDRATE_CHANGE	150 /* us */

/* Debug bits */
#define PN544_DEBUG_BUF		0x01
#define PN544_DEBUG_READ	0x02
#define PN544_DEBUG_WRITE	0x04
#define PN544_DEBUG_IRQ		0x08
#define PN544_DEBUG_CALLS	0x10
#define PN544_DEBUG_MODE	0x20

/* Normal (HCI) mode */
#define PN544_LLC_HCI_OVERHEAD	3 /* header + crc (to length) */
#define PN544_LLC_MIN_SIZE	(1 + PN544_LLC_HCI_OVERHEAD) /* length + */
#define PN544_LLC_MAX_DATA	(PN544_MSG_MAX_SIZE - 2)
#define PN544_LLC_MAX_HCI_SIZE	(PN544_LLC_MAX_DATA - 2)

struct pn544_llc_packet {
	unsigned char length; /* of rest of packet */
	unsigned char header;
	unsigned char data[PN544_LLC_MAX_DATA]; /* includes crc-ccitt */
};

/* Firmware upgrade mode */
#define PN544_FW_HEADER_SIZE	3
/* max fw transfer is 1024bytes, but I2C limits it to 0xC0 */
#define PN544_MAX_FW_DATA	(PN544_MAX_I2C_TRANSFER - PN544_FW_HEADER_SIZE)

struct pn544_fw_packet {
	unsigned char command; /* status in answer */
	unsigned char length[2]; /* big-endian order (msf) */
	unsigned char data[PN544_MAX_FW_DATA];
};

/* Jen Chang add for reading id and selftest scripts */
#ifdef CONFIG_PN544_TEST
enum script_type {
	r_ver = 0,
	w_test,
	antenna_self_test,
	swp_self_test,
	switch_Tx_only,
	hci_TypeA_reader,
};

union pn544_value {
	u32 value;
	u8	value_byte[4];
};

typedef struct chip_data_t {
	union pn544_value sw_ver;
	union pn544_value hw_ver;
} chip_data_t;

typedef struct card_data_t {
	u8 uid_len;
	union pn544_value uid[2];
} card_data_t;

typedef struct chip_test_data_t {
	struct chip_data_t chip_version;
	union pn544_value antenna_loop_tolerance;
	union pn544_value antenna_selftest_threshold;
	union pn544_value swp_selftest_result;
	struct card_data_t card_data;
	u8 switch_Tx_onoff;
} chip_test_data_t;

extern struct chip_test_data_t pn544_test_info;
extern int PN544_NFC_TEST(int type, struct i2c_client *client,
	struct chip_test_data_t *test_info);
#endif
/* Jen Chang, 20130114 */

#ifdef __KERNEL__
/* board config */
struct pn544_nfc_platform_data {
	int (*request_resources) (struct i2c_client *client);
	void (*free_resources) (void);
	void (*enable) (int fw);
	int (*test) (void);
	void (*disable) (void);
	int irq_gpio;
};
#endif /* __KERNEL__ */

#endif /* _PN544_H_ */
