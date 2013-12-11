/*For pn5444 read version*/

#include <asm/byteorder.h>
#include <linux/completion.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/nfc/pn544.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/serial_core.h> /* for TCGETS */
#include <linux/slab.h>

#define READ_DELAY 10 //ms
#define WRITE_DELAY 10 //ms
#define READ_RETRIES 10
#define Active_on	1
#define Active_off 	0

const int order = 16;
const unsigned long polynom = 0x1021;
const int direct = 1;
const unsigned long crcinit = 0xffff;
const unsigned long crcxor = 0xffff;//0x0000;
const int refin = 1;//0;
const int refout = 1;//0;

unsigned long crcmask = 0xffff;
unsigned long crchighbit = 0x8000;
unsigned long crcinit_direct = 0xffff;
unsigned long crcinit_nondirect = 0x84cf;
struct i2c_client *pn544_client;

int iFrameCount = 0;

struct chip_test_data_t pn544_test_info;

static int pn544_i2c_write_test(struct i2c_client *client, char *buf, int count)
{
	if (count != i2c_master_send(client, buf, count)) {
		pr_err("%s: Send reg. info error\n", __func__);
		return -1;
	}

	return 0;
}

static int pn544_i2c_read_test(struct i2c_client *client, char *buf, int count)
{
  int rc;

  struct i2c_msg msgs[] = {
    {
        .addr = client->addr,
        .flags = 0,
        .len = 1,
        .buf = buf,
    },
    {
        .addr = client->addr,
        .flags = I2C_M_RD,
        .len = count,
        .buf = buf,
    },
  };

  rc = i2c_transfer(client->adapter, msgs, 2);
  if (rc < 0) {
      pr_err("%s: error %d,read addr= 0x%x,len= %d\n", __func__ ,rc ,*buf ,count);
      return -1;
  }

  return 0;
}

unsigned long reflect (unsigned long crc, int bitnum)
{
	// reflects the lower 'bitnum' bits of 'crc'
	unsigned long i, j = 1, crcout = 0;

	for (i = (unsigned long) 1 << (bitnum-1); i; i >>= 1) {
		if (crc & i) crcout |= j;
		j <<= 1;
	}

	return (crcout);
}

unsigned long crcbitbybitfast(unsigned char* p, unsigned long len)
{
	// fast bit by bit algorithm without augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.
	unsigned long i, j, c, bit;
	unsigned long crc = crcinit_direct;

	for (i = 0; i < len; i++) {

		c = (unsigned long)*p++;
		if (refin) c = reflect(c, 8);

		for (j = 0x80; j; j >>= 1) {

			bit = crc & crchighbit;
			crc <<= 1;
			if (c & j) bit ^= crchighbit;
			if (bit) crc ^= polynom;
		}
	}

	if (refout) crc = reflect(crc, order);
	crc ^= crcxor;
	crc &= crcmask;

	return(crc);
}

static void pr_pkg(const char *func, char *msg, char *buf)
{
	int i;

	pn544_printk(NFC_TEST, "--%s: %s = { ", func, msg);
	for(i = 0; i < buf[0] + 1; i++)
		pn544_printk(NFC_TEST, "0x%02X ", buf[i]);
	pn544_printk(NFC_TEST, "}--\n");
}

static int pn544_write_test(char *buf, u8 count, u16 delay)
{
	pr_pkg(__func__, "cmd write", buf);
	mdelay(delay);
	return pn544_i2c_write_test(pn544_client, buf, count);
}

static int pn544_read_test(char *buf, u8 buf_len, u16 delay)
{
	char *in, *tmp;
	int res;
	u8 in_len, irq, read_retry_count = READ_RETRIES;
	u16 crc_in, crc_gen;

	in = kzalloc(33, GFP_KERNEL);
	tmp = kzalloc(1, GFP_KERNEL);
	memset(in, 0, 33);
	memset(buf, 0, buf_len);

	// read frame length
	while (read_retry_count--) {
	mdelay(delay);
		irq = gpio_get_value(PN544_HOST_INT_GPIO);
		pn544_printk(NFC_TEST, "%s: count(%d) irq(%d)\n", __func__, read_retry_count, irq);
		if (irq)
			break;
		else
			pn544_printk(NFC_TEST, "%s: irq not ready\n", __func__);
	}

	res = pn544_i2c_read_test(pn544_client, tmp, 1);
	in_len = tmp[0];

	if (res)
		pr_err("[%s]I2C read error = %d\n", __func__, res);

	if(in_len == 0x51) {
		// no data to read
		pr_info("%s: empty frame\n", __func__);
	} else if(in_len < 3 || in_len > 32) {
		pr_err("%s: bad frame length: %d\n", __func__, in_len);
		res = -1;
	} else {
		// read frame body
		in[0] = in_len;
		res = pn544_i2c_read_test(pn544_client, in + 1, in_len);

		// check CRC
		crc_in  = (((u16)in[in_len]) << 8) | in[in_len - 1];
		crc_gen = crcbitbybitfast(in, in_len - 1);
		if(crc_in != crc_gen) {
			pr_err("%s: bad CRC\n", __func__);
			pr_pkg(__func__, "frame CRC error", in);
			// TODO: error handling
			res = -1;
		} else {
			pr_pkg(__func__, "frame read", in);

			// copy frame data
			if(buf_len > in[0])
				memcpy(buf, in, in[0] + 1);
			else
				memcpy(buf, in, buf_len);
		}
	}

	kfree(in);
	kfree(tmp);

	return res;
}

static void pn544_gen_crc(char *buf, u8 len)
{
	u16 crc = crcbitbybitfast(buf, len - 2);

	buf[len - 2] = (crc) & 0xff;
	buf[len - 1] = (crc >> 8) & 0xff;
}

static int pn544_U_RSET(char *cmd, u8 len)
{
	char *frame, *ack;
	int res = 0;

	// write reset command
	frame = kzalloc(len + 4, GFP_KERNEL);
	frame[0] = len + 3;
	frame[1] = 0xF9;
	memcpy(frame+2, cmd, len);
	pn544_gen_crc(frame, len + 4);
	pn544_write_test(frame, len + 4, WRITE_DELAY);

	// get ack
	ack = kzalloc(4, GFP_KERNEL);
	pn544_read_test(ack, 4, READ_DELAY);
	if(ack[0] != 0x03 || ack[1] != 0xE6) {
		pr_err("%s: bad ack! reset fail!\n", __func__);
		pr_pkg(__func__, "ack read", ack);
		res = -1;
	}

	// reset I-frame
	iFrameCount = 0;

	kfree(frame);
	kfree(ack);

	return res;
}

static void pn544_S_encode(u8 nextPId, u8 msg)
{
	char *buf = kzalloc(4, GFP_KERNEL);

	buf[0] = 3;
	buf[1] = msg|nextPId;

	pn544_gen_crc(buf, 4);
	pn544_write_test(buf, 4, WRITE_DELAY);

	kfree(buf);
}

static u8 pn544_S_decode(char *buf)
{
	if(buf[0] != 3 || (buf[1]&0xE0) != 0xC0) {
		// not a S-frame
		return 0xFF;
	}

	// TODO: check CRC
	return buf[1]&0xF8;
}


static int pn544_I(char *cmd, u8 len, char *buf, u8 buf_len)
{
	char header[] = {0x80, 0x89, 0x92, 0x9B, 0xA4, 0xAD, 0xB6, 0xBF};
	char res_h[]  = {0x81, 0x8A, 0x93, 0x9C, 0xA5, 0xAE, 0xB7, 0xB8};
	//char ack[]  	= {0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC0};

	char *frame, *ack_in;//, *ack_out;
	bool read_retry;
	unsigned int retry_times = 0;
	int res = 0;

	// clean data
	memset(buf, 0, buf_len);

	// write I-frame
	frame = kzalloc(len + 4, GFP_KERNEL);
	frame[0] = len + 3;
	frame[1] = header[iFrameCount];
	memcpy(frame+2, cmd, len);

	pn544_gen_crc(frame, len + 4);
	pn544_printk(NFC_TEST, "%s: writing I-frame[%d]...\n", __func__, iFrameCount);

	pn544_write_test(frame, len + 4, WRITE_DELAY);

	// read return value
	ack_in = kzalloc(33, GFP_KERNEL);
	memset(ack_in, 0, 33);
	do {
		read_retry = false;
		pn544_read_test(ack_in, 33, READ_DELAY);
		if(ack_in[0] >= 3) {
			if(ack_in[0] == 3 && (ack_in[1] & 0xE0) == 0xC0) {
				// got S-frame
				if(pn544_S_decode(ack_in) == 0XC0) {
					pn544_printk(NFC_TEST, "%s: response OK: S(RR)\n", __func__);
					read_retry = true;
					retry_times = 5;
				} else {
					pr_err("%s: response ERROR: %d\n", __func__, (ack_in[1] >> 3) & 0x03);
					//pr_pkg(__func__, "I-frame write", frame);
					//pr_pkg(__func__, "ack read", ack_in);
					res = -1;
				}
			} else if((ack_in[1] & 0xC0) == 0x80) {
				// got I-frame
				if(ack_in[1] != res_h[iFrameCount]) {
					pr_err("%s: bad ack!\n", __func__);
					//pr_pkg(__func__, "I-frame write", frame);
					//pr_pkg(__func__, "ack read", ack_in);
					res = -1;
				} else {
					if(ack_in[0] > 4) {
						// ack
						switch(ack_in[3] >> 6) {
							case 0:
								pn544_printk(NFC_TEST, "%s: type: command: %d\n", __func__, (ack_in[3] & 0x3F));
								break;
							case 1:
								pn544_printk(NFC_TEST, "%s: type: event: %d\n", __func__, (ack_in[3] & 0x3F));
								break;
							case 2:
								pn544_printk(NFC_TEST, "%s: type: response: %d\n", __func__, (ack_in[3] & 0x3F));
								break;
							case 3:
								pn544_printk(NFC_TEST, "%s: type ERROR: %d\n", __func__, ack_in[3]);
								break;
						}
					}
					// set return data
					if(buf_len > ack_in[0])
						memcpy(buf, ack_in, ack_in[0] + 1);
					else
						memcpy(buf, ack_in, buf_len);
				}
				iFrameCount = (ack_in[1] & 0x07);
				pn544_printk(NFC_TEST, "%s: next: I-frame[%d]\n", __func__, iFrameCount);
				// auto-ack (S-frame)
				//ack_out = kzalloc(4, GFP_KERNEL);
				pn544_S_encode(iFrameCount, 0xC0);
				//ack_out[0] = 3;
				////ack_out[1] = ack[iFrameCount];
				//ack_out[1] = 0xC0&iFrameCount;
				//pn544_gen_crc(ack_out, 4);
				//pn544_write(ack_out, 4);
				//kfree(ack_out);
			} else {
				pn544_printk(NFC_TEST, "%s: UNKNOW response: not I/S frame\n", __func__);
				pr_pkg(__func__, "ack read", ack_in);
			}
		} else if (ack_in[0] == 0) {
			if (retry_times) {
				pr_info("%s : No data can readed, retry once\n", __func__);
				read_retry = true;
				retry_times--;
			}
		} else {
			pr_err("%s: UNKNOW response: length = %d\n", __func__, ack_in[0]);
			pr_pkg(__func__, "ack read", ack_in);
		}
	} while(read_retry);

	// set iFrameCount
	//iFrameCount = (iFrameCount+1)&0x07;	// 0~7

	kfree(frame);
	kfree(ack_in);

	return res;
}

static void demoScript_init(void)
{
	char cmd1[] = {0x04, 0x00};	// Send U-RSET frame
	char cmd2[] = {0x81, 0x03};	// Open Pipe Admin
	char cmd3[] = {0x81, 0x14};	// Admin Clear All Pipe
	char cmd4[] = {0x81, 0x03};	// ReOpen Pipe Admin

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);
	pn544_U_RSET(cmd1, sizeof(cmd1));
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_I(cmd3, sizeof(cmd3), 0, 0);
	pn544_I(cmd4, sizeof(cmd4), 0, 0);
	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

static void demoScript_deinit(void)
{
	char cmd1[] = {0x82,0x04};
	char cmd2[] = {0x81,0x11,0x02};

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

static void Set_ActiveMode_Script(int onoff)
{
	char cmd1[] = {0x81, 0x10, 0x20, 0x00, 0x90}; // Create Pipe to the gate system MNGT
												  // {xx, xx, srcGId, dstHId, dstGId}
	char cmd2[] = {0x82, 0x03};	// Open Pipe to the gate system MNGT
	char cmd3[] = {0x82, 0x3E, 0x00, 0x9E, 0xAA};
	char cmd4[] = {0x82, 0x3F, 0x00, 0x9E, 0xAA, 0x00};
	char cmd5[] = {0x82, 0x3F, 0x00, 0x9E, 0xAA, 0x01};
	char *in;
	bool repolling;
	int retry_times = 0;

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);

	in = kzalloc(7, GFP_KERNEL);

	/*Change PN544 to active mode*/
	pn544_printk(NFC_TEST, "[NFC] set active mode: %d\n", onoff);
	demoScript_init();
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_I(cmd3, sizeof(cmd3), in, 7);
	if (in[3] == 0x80)
		pn544_printk(NFC_TEST, "[NFC] Detect nfc chip is in %s mode\n", in[4] ? "standby":"active");

	if (onoff == Active_on) {
		do {
			repolling = false;
			pn544_I(cmd4, sizeof(cmd4), in, 7);
			pn544_I(cmd3, sizeof(cmd3), in, 7);
			if (in[3] == 0x80) {
				pn544_printk(NFC_TEST, "[NFC]Active: Recheck nfc chip is in %s mode\n", in[4] ? "standby":"active");
				if (in[4] != 0x00) {
					pn544_printk(NFC_TEST, "[NFC]Active: Re-set nfc chip to active mode, retry(%d)!!\n", retry_times);
					repolling = true;
				}
			} else {
				pn544_printk(NFC_TEST, "[NFC]Active: Set nfc chip to active mode failed, retry(%d)!!\n", retry_times);
				repolling = true;
			}

			if (++retry_times > 5) {
				pn544_printk(NFC_TEST, "[NFC]Active: retry 5 times failed, break!!\n");
				break;
			}
		} while (repolling);
	} else {
		do {
			repolling = false;
			pn544_I(cmd5, sizeof(cmd5), in, 7);
			pn544_I(cmd3, sizeof(cmd3), in, 7);
			if (in[3] == 0x80) {
				pn544_printk(NFC_TEST, "[NFC]Deactive: Recheck nfc chip is in %s mode\n", in[4] ? "standby":"active");
				if (in[4] != 0x01) {
					pn544_printk(NFC_TEST, "[NFC]Deactive: Re-set nfc chip to standby mode, retry(%d)!!\n", retry_times);
					repolling = true;
				}
			} else {
				pn544_printk(NFC_TEST, "[NFC]Deactive: Set nfc chip to standby mode failed, retry(%d)!!\n", retry_times);
				repolling = true;
			}

			if (++retry_times > 5) {
				pn544_printk(NFC_TEST, "[NFC]Deactive: retry 5 times failed, break!!\n");
				break;
			}
		} while (repolling);
	}
	demoScript_deinit();
	kfree(in);

	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

/*
 *This function let nxp to read mode(Mifare)
 *@ID : Return remote card ID->UID0, UID1, UID2, UID3
*/

int demoScript_hci_TypeA_reader_writer(char *UID_LEN, unsigned char *UID)
{
	char cmd1[] = {0x81, 0x10, 0x20, 0x00, 0x94}; // Create Pipe to the gate system MNGT
												  // {xx, xx, srcGId, dstHId, dstGId}
	char cmd2[] = {0x82, 0x03};	// Open Pipe to the Gate PL MNGT
	char cmd3[] = {0x82, 0x01, 0x06,0x7F}; // SET RD_PHASE : activate all readers: 0x7F
	char cmd4[] = {0x82, 0x02, 0x06};
	char cmd5[] = {0x81, 0x10, 0x13, 0x00, 0x13}; // Host creates pipe on ReaderA gate
	char cmd6[] = {0x83, 0x03}; // Host opens pipe on ReaderA gate
	char cmd7[] = {0x81, 0x10, 0x20, 0x00, 0x90}; // Create pipe to the Gate System MNGT
	char cmd8[] = {0x84, 0x03}; // Open pipe to the Gate System MNGT
	char cmd9[] = {0x84, 0x02, 0x02}; // Get Event handling
	char cmd10[] = {0x84, 0x01, 0x02, 0xFF}; // Set Event Handling: All Events
	char cmd11[] = {0x83, 0x01, 0x10, 0x00}; // Stop Activation after SAK received (not to jump into T=CL)
	char cmd12[] = {0x83, 0x50}; // Host sends an Event "ReaderRequested"
	char cmd13[] = {0x83, 0x75}; // Restart the polling loop
	char cmd14[] = {0x83, 0x02, 0x04};
	char cmd15[] = {0x83, 0x02, 0x03};
	char cmd16[] = {0x83, 0x02, 0x02};
	char *in;
	bool repolling;
	int uid_len = 0, retry_times = 0, i;

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);

	in = kzalloc(13, GFP_KERNEL);
	memset(UID_LEN, 0, sizeof(UID_LEN));
	memset(UID, 0, 8);

	demoScript_init();
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_I(cmd3, sizeof(cmd3), 0, 0);
	pn544_I(cmd4, sizeof(cmd4), 0, 0);
	pn544_I(cmd5, sizeof(cmd5), 0, 0);
	pn544_I(cmd6, sizeof(cmd6), 0, 0);
	pn544_I(cmd7, sizeof(cmd7), 0, 0);
	pn544_I(cmd8, sizeof(cmd8), 0, 0);
	pn544_I(cmd9, sizeof(cmd9), 0, 0);
	pn544_I(cmd10, sizeof(cmd10), 0, 0);
	pn544_I(cmd11, sizeof(cmd11), 0, 0);
	do {
		repolling = false;
		pn544_I(cmd12, sizeof(cmd12), in, 13);
		if (in[2] == 0x83) {
			if (in[4] == 0x00) {
				pn544_printk(NFC_TEST, "[NFC](%d)One card detected\n",	__LINE__);
				pn544_I(cmd14, sizeof(cmd14), 0, 0);
				pn544_I(cmd15, sizeof(cmd15), 0, 0);
				pn544_I(cmd16, sizeof(cmd16), in, 13);
				uid_len = in[0] - 5;
				*UID_LEN = uid_len;
				pn544_printk(NFC_TEST, "[NFC] uid_len is %d\n", uid_len);

				memcpy(UID, in+4, uid_len);
				pn544_printk(NFC_TEST, "[NFC] The card ID is ");
				for (i = 0; i < uid_len; i++) {
					pn544_printk(NFC_TEST, "%x ", UID[i]);
				}
				pn544_printk(NFC_TEST, "\n");
			} else if (in[4] == 0x03) {
				pn544_printk(NFC_TEST, "[NFC](%d)Collision occurred\n",	__LINE__);
				repolling = true;
			} else {
				pn544_I(cmd13, sizeof(cmd13), 0, 0);
				repolling = true;
				pn544_printk(NFC_TEST, "[NFC](%d)Unknow fail(0x%02x, 0x%02x),  restart the polling \n",
					__LINE__, in[3], in[4]);
			}
		} else {
			pn544_printk(NFC_TEST, "[NFC](%d)No card detected, retry!!\n",	__LINE__);
			repolling = true;
		}

		if (++retry_times > 5) {
			pn544_printk(NFC_TEST, "[NFC](%d)retry 5 times failed, break!!\n", __LINE__);
			break;
		}
	} while (repolling);

	demoScript_deinit();
	kfree(in);

	pn544_printk(NFC_TEST, "-- %s --\n", __func__);

	return 0;
}
//EXPORT_SYMBOL(emulation_card_mode);

static void demoScript_readVersion(chip_data_t *chip_data)
{
	char cmd1[] = {0x81, 0x10, 0x05, 0x00, 0x05}; // Create Pipe to the gate system MNGT
												  // {xx, xx, srcGId, dstHId, dstGId}
	char cmd2[] = {0x82, 0x03};	// Open Pipe to the gate system MNGT
	char cmd3[] = {0x82, 0x02, 0x01};
	char cmd4[] = {0x82, 0x02, 0x03};
	char *in;

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);

	in = kzalloc(9, GFP_KERNEL);
	memset(chip_data->sw_ver.value_byte, 0, sizeof(chip_data->sw_ver.value_byte));
	memset(chip_data->hw_ver.value_byte, 0, sizeof(chip_data->hw_ver.value_byte));

	demoScript_init();
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_I(cmd3, sizeof(cmd3), in, 9);
	if(in[3] == 0x80)
		memcpy(chip_data->sw_ver.value_byte, in + 4, 3);
	pn544_I(cmd4, sizeof(cmd4), in, 9);
	if(in[3] == 0x80)
		memcpy(chip_data->hw_ver.value_byte, in + 4, 3);

	demoScript_deinit();
	kfree(in);

	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

static void antenna_selftest_failed_log(int _line, char *data)
{
	pn544_printk(NFC_TEST, "[NFC-ANT](%d)The selftets threshold is %02x,%02x,%02x,%02x\n",
		_line, data[5], data[6], data[7], data[8]);

	if (data[4] == 0x79) {
		pn544_printk(NFC_TEST, "[NFC](%d)ANT 1st failed(RF sensitivity on TX1)\n",
			_line);
	} else if (data[4] == 0x7A) {
		pn544_printk(NFC_TEST, "[NFC](%d)ANT 2nd failed(RF sensitivity on TX2)\n",
			_line);
	} else if (data[4] == 0x7B) {
		pn544_printk(NFC_TEST, "[NFC](%d)ANT 3rd failed(Current detection level on TX1 & TX2)\n",
			_line);
	} else if (data[4] == 0x7C) {
		pn544_printk(NFC_TEST, "[NFC](%d)ANT 4th failed(ANT1/ANT2 circuitry check)\n",
			_line);
	}
}

static void demoScript_Antenna_selftest(unsigned char *threshold,
	unsigned char *tolerance)
{
	char cmd1[] = {0x81, 0x10, 0x20, 0x00, 0x90}; // Create Pipe to the gate system MNGT
												  // {xx, xx, srcGId, dstHId, dstGId}
	char cmd2[] = {0x82, 0x03};	// Open Pipe to the gate system MNGT
	/** Get Tolerance Loop 1 **/
	char cmd3[] = {0x82, 0x3E, 0x0, 0x98, 0x9F}; // Get Loop 1 config
	char cmd4[] = {0x82, 0x3E, 0x0, 0x98, 0x9D}; // Get Loop 2 config
	char cmd5[] = {0x82, 0x3E, 0x0, 0x98, 0x9E}; // Get Loop 3 config
	char cmd6[] = {0x82, 0x3F, 0x0, 0x9B, 0xF5, tolerance[0]}; // Set Tolerance Loop 1
	char cmd7[] = {0x82, 0x3F, 0x0, 0x9B, 0xF6, tolerance[1]}; // Set Tolerance Loop 2
	char cmd8[] = {0x82, 0x3F, 0x0, 0x9B, 0xF7, tolerance[2]}; // Set Tolerance Loop 3
	char cmd9[] = {0x82, 0x3F, 0x0, 0x9B, 0xF8, tolerance[3]}; // Set Tolerance Loop 4
	char cmd10[] = {0x82, 0x3E, 0x0, 0x9B, 0xF5}; // Get Tolerance Loop 1
	char cmd11[] = {0x82, 0x3E, 0x0, 0x9B, 0xF6}; // Get Tolerance Loop 2
	char cmd12[] = {0x82, 0x3E, 0x0, 0x9B, 0xF7}; // Get Tolerance Loop 3
	char cmd13[] = {0x82, 0x3E, 0x0, 0x9B, 0xF8}; // Get Tolerance Loop 4
	/** Start RF self test PASS/FAIL **/
#if 0
	char cmd14[] = {0x82, 0x20, 0x07, 0x04, 0x1D, 0xFF};
	char cmd15[] = {0x82, 0x20, 0x03, 0x05, 0x0E, 0xFF};
#else
	char cmd14[] = {0x82, 0x20, 0x00, 0x00, 0x00, 0x00};
	char cmd15[] = {0x82, 0x20, 0x03, 0x05, 0x0A, 0xFF};
#endif
	char *in;

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);

	in = kzalloc(12, GFP_KERNEL);
	memset(threshold, 0, 4);

	demoScript_init();
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_I(cmd3, sizeof(cmd3), 0, 0);
	pn544_I(cmd4, sizeof(cmd4), 0, 0);
	pn544_I(cmd5, sizeof(cmd5), 0, 0);
	pn544_I(cmd6, sizeof(cmd6), 0, 0);
	pn544_I(cmd7, sizeof(cmd7), 0, 0);
	pn544_I(cmd8, sizeof(cmd8), 0, 0);
	pn544_I(cmd9, sizeof(cmd9), 0, 0);
	pn544_I(cmd10, sizeof(cmd10), 0, 0);
	pn544_I(cmd11, sizeof(cmd11), 0, 0);
	pn544_I(cmd12, sizeof(cmd12), 0, 0);
	pn544_I(cmd13, sizeof(cmd13), 0, 0);

	pn544_I(cmd14, sizeof(cmd14), in, 12);

	if (in[4] >= 0x79 && in[4] <= 0x7C) {
		antenna_selftest_failed_log(__LINE__, in);
		pn544_I(cmd15, sizeof(cmd15), in, 12);
	}

	memcpy(threshold, in + 5, 4);

	demoScript_deinit();
	kfree(in);

	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

static void demoScript_SWP_selftest(unsigned char *result)
{
	char cmd1[] = {0x81, 0x10, 0x20, 0x00, 0x90}; // Create Pipe to the gate system MNGT
												  // {xx, xx, srcGId, dstHId, dstGId}
	char cmd2[] = {0x82, 0x03};	// Open Pipe to the gate system MNGT
	/** Get Tolerance Loop 1 **/
	char cmd3[] = {0x82, 0x21}; // Perform SWP self test
	char *in;

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);

	in = kzalloc(8, GFP_KERNEL);
	memset(result, 0, 4);

	demoScript_init();
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	pn544_I(cmd3, sizeof(cmd3), in, 8);

	pn544_printk(NFC_TEST, "[NFC](%d)SWP test result (%x,%x,%x,%x,%x,%x,%x,%x)\n",
		__LINE__, in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7]);

	if (in[4] == 0x00 && in[5] == 0x01) {
		pn544_printk(NFC_TEST, "[NFC](%d)SWP test pass (0x%02x, 0x%02x)\n",
			__LINE__, in[4], in[5]);
	} else {
		pn544_printk(NFC_TEST, "[NFC](%d)SWP test failed (0x%02x, 0x%02x)\n",
			__LINE__, in[4], in[5]);
	}

	memcpy(result, in + 4, 2);

	demoScript_deinit();
	kfree(in);

	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

static void demoScript_switch_Tx_only(unsigned char *result)
{
	char cmd1[] = {0x81, 0x10, 0x20, 0x00, 0x90}; // Create Pipe to the gate system MNGT
												  // {xx, xx, srcGId, dstHId, dstGId}
	char cmd2[] = {0x82, 0x03};	// Open Pipe to the gate system MNGT
	char cmd3[] = {0x82, 0x25, 0x03, 0x00};
	char *in;
	bool repolling;
	int retry_times = 0;

	pn544_printk(NFC_TEST, "++ %s ++\n", __func__);

	in = kzalloc(9, GFP_KERNEL);
	memset(result, 0, 4);

	demoScript_init();
	pn544_I(cmd1, sizeof(cmd1), 0, 0);
	pn544_I(cmd2, sizeof(cmd2), 0, 0);
	do {
		repolling = false;
		pn544_I(cmd3, sizeof(cmd3), in, 9);
		if (in[2] == 0x82 && in[3] == 0x80) {
			pn544_printk(NFC_TEST, "[NFC](%d)Switch Tx only success!!\n", __LINE__);
			*result = 1;
		} else {
			pn544_printk(NFC_TEST, "[NFC](%d)Switch Tx only failed, retry!!\n", __LINE__);
			repolling = true;
		}

		if (++retry_times > 5) {
			pn544_printk(NFC_TEST, "[NFC](%d)retry 5 times failed, break!!\n", __LINE__);
			demoScript_deinit();
			*result = 0;
			break;
		}
	} while (repolling);

	kfree(in);

	pn544_printk(NFC_TEST, "-- %s --\n", __func__);
}

int PN544_NFC_TEST(int type, struct i2c_client *client,
	struct chip_test_data_t *test_info)
{
	int result = 0;

	pn544_printk(NFC_TEST, "[NFC_TEST] start nfc test, its type = %d\n", type);
	pn544_client = client;

	Set_ActiveMode_Script(Active_on);
	switch(type) {
	case r_ver:
		demoScript_readVersion(&test_info->chip_version);
		test_info->chip_version.sw_ver.value =
			ntohl(test_info->chip_version.sw_ver.value) >> 8;

		test_info->chip_version.hw_ver.value =
			ntohl(test_info->chip_version.hw_ver.value) >> 8;

		pn544_printk(NFC_TEST, "[NFC_TEST] The nfc HW version is [0x%06x]\n",
			test_info->chip_version.hw_ver.value);
		pn544_printk(NFC_TEST, "[NFC_TEST] The nfc FW version is [0x%06x]\n",
			test_info->chip_version.sw_ver.value);
		Set_ActiveMode_Script(Active_off);
		result = 1;
		break;
	case w_test:
		break;
	case antenna_self_test:
		demoScript_Antenna_selftest(test_info->antenna_selftest_threshold.value_byte,
			test_info->antenna_loop_tolerance.value_byte);

		pn544_printk(NFC_TEST, "[NFC_TEST] The nfc antenna self test threshold is [0x%08x]\n",
			test_info->antenna_selftest_threshold.value);
		Set_ActiveMode_Script(Active_off);
		result = 1;
		break;
	case swp_self_test:
		demoScript_SWP_selftest(test_info->swp_selftest_result.value_byte);

		pn544_printk(NFC_TEST, "[NFC_TEST] The nfc swp self test value is [0x%x,0x%x]\n",
			test_info->swp_selftest_result.value_byte[1],
			test_info->swp_selftest_result.value_byte[0]);
		Set_ActiveMode_Script(Active_off);
		result = 1;
		break;
	case switch_Tx_only:
		if (test_info->switch_Tx_onoff == 1) {
			demoScript_switch_Tx_only(&test_info->switch_Tx_onoff);
		pn544_printk(NFC_TEST, "[NFC_TEST] Switch Tx 13.56MHz On %s !!\n",
				(test_info->switch_Tx_onoff ? "Success" : "Failed"));
		} else {
			Set_ActiveMode_Script(Active_off);
		}
		result = 1;
		break;
	case hci_TypeA_reader:
		demoScript_hci_TypeA_reader_writer(&test_info->card_data.uid_len,
			test_info->card_data.uid[0].value_byte);
		pn544_printk(NFC_TEST, "[NFC_TEST] Hci TypeA reader\n");
		Set_ActiveMode_Script(Active_off);
		result = 1;
		break;
	default :
		result = 0;
		break;
	};
	pn544_printk(NFC_TEST, "[NFC_TEST] end nfc test, its result = %d\n", result);

	return result;
}
