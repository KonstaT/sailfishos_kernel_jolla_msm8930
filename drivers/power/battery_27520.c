//============================================================================//
//
//  Battery driver for 27520
//
//  Author:   Eric Liu (Qisda Corp.)
//
//============================================================================//
//============================================================================//
//  Date:     2012.12.23
//
//  HW base:  EVT1
//  SW base:  1023
//
//============================================================================//

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/seq_file.h> /* add for FTD to get signed curr value */
#include <linux/slab.h>
//#include <linux/power_supply.h>
//#include <linux/reboot.h>
//#include <linux/regulator/consumer.h>
//#include <linux/regulator/driver.h>
//#include <linux/regulator/machine.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include "battery_27520.h"

//============================================================================//
//  debug
//============================================================================//
extern struct dentry *kernel_debuglevel_dir;

//static int bat_log_on1  = 0;
//static int bat_log_on2  = 1;
//static int bat_log_on3  = 0;
//#define MSG(format, arg...)   {if(bat_log_on1)  printk(KERN_INFO "[BAT]" format "\n", ## arg);}
//#define MSG2(format, arg...)  {if(bat_log_on2)  printk(KERN_INFO "[BAT]" format "\n", ## arg);}
//#define MSG3(format, arg...)  {if(bat_log_on3)  printk(KERN_INFO "[BAT]" format "\n", ## arg);}
#define MSG(format, arg...)   {}
#define MSG2(format, arg...)  printk(KERN_INFO "[BAT]" format "\n", ## arg)

//#define BAT_ALL_FEATURE

//============================================================================//
//  qsd bat driver
//============================================================================//
//static DEFINE_SPINLOCK(bat_irq_lock);
static struct bat_drv_data *bat_drv;

//============================================================================//
//  i2c access function
//============================================================================//
static int bat_read_i2c(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  struct i2c_msg msgs[] = {
    [0] = {
      .addr   = addr,
      .flags  = 0,
      .buf    = (void *)&reg,
      .len    = 1
    },
    [1] = {
      .addr   = addr,
      .flags  = I2C_M_RD,
      .buf    = (void *)buf,
      .len    = len
    }
  };
  int ret;
  if(!bat_drv->client)
  {
    MSG2("%s, status = -ENODEV",__func__);
    return -ENODEV;
  }
  ret = i2c_transfer(bat_drv->client->adapter, msgs, 2);
  if(ret == 2)
  {
    if(bat_drv->i2c_err)
      MSG2("%s, status = 2, i2c_err = 0",__func__);
    bat_drv->i2c_err = 0;
  }
  else
  {
    bat_drv->i2c_err ++;
    if(bat_drv->i2c_err < 20)
      MSG2("%s, ret = %d, i2c_err = %d",__func__,ret,bat_drv->i2c_err);
  }
  return ret;
}
static int bat_read_i2c_retry(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  int i,ret;
  for(i=0; i<BAT_I2C_RETRY_MAX; i++)
  {
    ret = bat_read_i2c(addr,reg,buf,len);
    if(ret == 2)
      return ret;
    else if(ret == -ENODEV)
      return ret;
    else
      msleep(10);
  }
  return ret;
}
static int bat_write_i2c(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  int i;
  unsigned char buf_w[128];
  struct i2c_msg msgs[] = {
    [0] = {
      .addr   = addr,
      .flags  = 0,
      .buf    = (void *)buf_w,
      .len    = len+1
    }
  };
  int ret;
  if(len >= sizeof(buf_w))  //invalid len
  {
    MSG2("%s, status = -ENOMEM, len = %d",__func__, len);
    return -ENOMEM;
  }
  if(!bat_drv->client)
  {
    MSG2("%s, status = -ENODEV",__func__);
    return -ENODEV;
  }
  buf_w[0] = reg;
  for(i=0; i<len; i++)
    buf_w[i+1] = buf[i];
  ret = i2c_transfer(bat_drv->client->adapter, msgs, 1);

  if(ret == 1)
  {
    if(bat_drv->i2c_err)
      MSG2("%s, status = 2, i2c_err = 0",__func__);
    bat_drv->i2c_err = 0;
  }
  else
  {
    bat_drv->i2c_err ++;
    if(bat_drv->i2c_err < 20)
      MSG2("%s, ret = %d, i2c_err = %d",__func__,ret,bat_drv->i2c_err);
  }
  return ret;
}
static int bat_write_i2c_retry(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  int i,ret;
  for(i=0; i<BAT_I2C_RETRY_MAX; i++)
  {
    ret = bat_write_i2c(addr,reg,buf,len);
    if(ret == 1)
      return ret;
    else if(ret == -ENODEV)
      return ret;
    else
      msleep(10);
  }
  return ret;
}
static int bat_read_i2c_cmd(unsigned short *val)
{
  int ret;
  unsigned char data[2] = {0,0};
  ret = bat_read_i2c_retry(GAUGE_ADDR, 0x00, &data[0], sizeof(data));
  *val = data[0] + (data[1] << 8);
  return ret;
}
static int bat_write_i2c_cmd(unsigned short subcmd)
{
  unsigned char data[2];
  data[0] = subcmd & 0x00FF;
  data[1] = (subcmd & 0xFF00) >> 8;
  return bat_write_i2c_retry(GAUGE_ADDR, 0x00, &data[0], sizeof(data));
}

//============================================================================//
//  DFI
//============================================================================//
#define BQ275250_DFI_SUPPORT

#ifdef BQ275250_DFI_SUPPORT

#define BQ27520_ID          0x0520
#define BQ27520_FIRMWARE    0x0302
#define BQ27520_HARDWARE    0x00a3

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static int bq27520_dfi_update_ongoing = 0;

static unsigned char bq27520_rom_mode_download = 0;

static unsigned char *yDataFlashImage;

static bool bq27520_enable_rom_mode(bool enable)
{
  u8 exit_rom[2] = {0x0F, 0x00};
  u8 enter_rom[2] = {0x00, 0x0F};
  int ret;

  if (enable) {
    ret = bat_write_i2c_retry(GAUGE_ADDR, 0x00, enter_rom, sizeof(enter_rom));
    if (!ret) {
      MSG2("%s: write 0x0f00 with command 0x00 failed\n", __func__);
      return FALSE;
    }
  } else {
    ret = bat_write_i2c_retry(ROM_ADDR, 0x00, &exit_rom[0], 1);
    if (!ret) {
      MSG2("%s: write 0x0f with command 0x00 failed\n", __func__);
      return FALSE;
    }

    ret = bat_write_i2c_retry(ROM_ADDR, 0x64, &exit_rom[0], 1);
    if (!ret) {
      MSG2("%s: write 0x0f with command 0x64 failed\n", __func__);
      return FALSE;
    }

    ret = bat_write_i2c_retry(ROM_ADDR, 0x65, &exit_rom[1], 1);
    if(!ret) {
      MSG2("%s: write 0x00 with command 0x65 failed\n", __func__);
      return FALSE;
    }
  }

  msleep(20);

  return TRUE;
}

static bool bq27520_erase_instruction_flash(void)
{
  u8 write_buf[2] = {0x03, 0x00}, *write_buf_ptr = write_buf;
  u8  read_buf = 0xFF, *read_buf_ptr = &read_buf;
  int i, ret;

  MSG2("%s: --start--\n", __func__);

  for(i = 0; i < 3; i++) {
    // Erase the first two rows by writing 0x00 to cmd 0x00
    ret = bat_write_i2c_retry(ROM_ADDR, 0x00, write_buf_ptr, 1);
    if (!ret) {
      MSG2("%s: write data 0x00 with command 0x03 failed\n",
        __func__);
      return ret;
    }
    // Send checksum data 0x03 and 0x00 to cmd 0x64 and 0x65
     ret = bat_write_i2c_retry(ROM_ADDR, 0x64, write_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: write data 0x03,0x00 with command 0x64 failed\n",
        __func__);
      return ret;
    }

    /*flow char needs delay 20ms*/
    msleep(20);

    ret = bat_read_i2c_retry(ROM_ADDR, 0x66, read_buf_ptr, 1);
    if (!ret) {
      MSG2("%s: read  data with command 0x66 failed\n",
        __func__);
      return ret;
    }

    if(*read_buf_ptr == 0x00){
      break;
    }
  }

  MSG2("%s: --end--\n", __func__);

  return TRUE;
}

static bool bq27520_erase_data_flash(void)
{
  u8 write_buf[2], *write_buf_ptr = write_buf;
  u8 read_buf = 0xFF, *read_buf_ptr = &read_buf;
  int i, ret;

  MSG2("%s: --start--\n", __func__);

  for(i = 0; i < 3; i++) {
    // Send Mass Erase cmd by sending cmd 0x00 with 0x0C
    *write_buf_ptr = 0x0C;

    ret = bat_write_i2c_retry(ROM_ADDR, 0x00, write_buf_ptr, 1);
    if (!ret) {
      MSG2("%s: write data 0x0c with command 0x00 failed\n",
        __func__);
      return ret;
    }

    // Setup Mass Erase by sending cmd 0x04 and 0x05 with 0x83 and 0xDE
    *write_buf_ptr = 0x83;
    *(write_buf_ptr + 1) = 0xDE;

    ret = bat_write_i2c_retry(ROM_ADDR, 0x04, write_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: write data 0x83, 0xde with command 0x04 failed\n",
        __func__);
      return ret;
    }

    // Send checksum 0x6D (= 0x0C +0x83 + 0xDE)
    *write_buf_ptr = 0x6D;
    // Send checksum 0x01 for Mass Erase cmd
    *(write_buf_ptr + 1) = 0x01;

    ret = bat_write_i2c_retry(ROM_ADDR, 0x64, write_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: write data 0x6d, 0x01 with command 0x64 failed\n",
        __func__);
      return ret;
    }

    /*flow char needs delay 200ms*/
    msleep(200);

    // Check for Mass Erase success
    ret = bat_read_i2c_retry(ROM_ADDR, 0x66, read_buf_ptr, 1);
    if (!ret) {
      MSG2("%s: read  data  with command 0x66 failed\n",
        __func__);
      return ret;
    }

    if(read_buf == 0x00) {
      break;
    }
  }

  MSG2("%s: --end--\n", __func__);

  return TRUE;
}

static bool bq27520_dfi_write_data_flash(u16 *DFI_checksum)
{
  u8 read_buf, *read_buf_ptr = &read_buf;
  u8 write_buf[2], *write_buf_ptr = write_buf;
  u8 row, i, num, *dfi_ptr = (u8 *)yDataFlashImage;
  u16 checksum;
  u32 sum_of_dfi_buf;
  bool ret;

  MSG2("%s: --start--\n", __func__);

  for(i = 0; i < 3; i++){
    for(row = 0; row < 0x400 / 32; row++) {
      // Send Program Row cmd 0xA and Data iRow
      *write_buf_ptr = 0x0A;
      *(write_buf_ptr + 1) = row;

      ret = bat_write_i2c_retry(ROM_ADDR, 0x00, write_buf_ptr, 2);
      if (!ret) {
        MSG2("%s: write data 0x0a,row with command 0x00 failed\n",
          __func__);
        return ret;
      }

      // Send Write Row cmd 0x04
      ret = bat_write_i2c_retry(ROM_ADDR, 0x04, (dfi_ptr + row * 32), 32);
      if (!ret) {
        MSG2("%s: write data DFI with command 0x04 failed\n",
          __func__);
        return ret;
      }
      sum_of_dfi_buf = 0;

      for(num = 0; num < 32; num++) {
        sum_of_dfi_buf += *(dfi_ptr + row * 32 + num);
      }

      checksum = (u16)((0x0A + row + sum_of_dfi_buf) % 0x10000);
      *DFI_checksum = (u16)((*DFI_checksum + sum_of_dfi_buf) % 0x10000);

      // Send checksum to cmd 0x64 and 0x65
      *write_buf_ptr = checksum & 0x00FF;
      *(write_buf_ptr + 1) = (checksum & 0xFF00) >> 8;

      ret = bat_write_i2c_retry(ROM_ADDR, 0x64,  write_buf_ptr, 2);
      if (!ret) {
        MSG2("%s: write data LSB, MSB, row with command 0x64 failed\n",
          __func__);
        return ret;
      }

      // Check for data programming success
      /*flow char needs delay 2ms*/
      msleep(2);

      *read_buf_ptr = 0xFF;

      ret = bat_read_i2c_retry(ROM_ADDR, 0x66, read_buf_ptr, 1);
      if (!ret) {
        MSG2("%s: read data with command 0x66 failed\n", __func__);
        return ret;
      }

      if(*read_buf_ptr != 0x00) {
        MSG2("%s: start writing image, read 0x66\n", __func__);
        break;
      }
    }

    if(row < 0x400 / 32) {
      if(!bq27520_erase_data_flash()) {
        return FALSE;
      }
    }
    else {
      return TRUE;
    }
  }

  MSG2("%s: --end--\n", __func__);

  return FALSE;
}


static u8 bq27520_update_DFI(void)
{
  u8 instruction_bak[2][96], *instruction_bak_ptr = (u8 *)instruction_bak;
  u8 write_buf[2], *write_buf_ptr = write_buf;
  u8 read_buf[2] = {0xFF, 0xFF}, *read_buf_ptr = read_buf;
  u16 DFI_checksum, DFI_checksum_RB, checksum;
  u32 sum_of_dfi_bak;
  int row, i, num;
  bool ret;

  MSG2("%s: --read and erase instruction flash--\n", __func__);

  if(bq27520_rom_mode_download)
  {
    MSG2("%s: bypass enable rom mode",__func__);
    goto rom_mode;
  }

  if (!bq27520_enable_rom_mode(TRUE)) {
    MSG2("%s: Can't enter rom mode\n", __func__);
    return FALSE;
  }

rom_mode:
   /* Backup Instruction Flash */
  for (row = 0; row < 2; row++) {
    MSG2("%s: Backup Instruction Flash (%d)\n", __func__, row);

    *write_buf_ptr = 0x00;

    ret = bat_write_i2c_retry(ROM_ADDR, 0x00, write_buf_ptr, 1);
    if (!ret) {
      MSG2("%s: write data 0x00 with command 0x00 failed\n",
        __func__);
      return ret;
    }

    *write_buf_ptr = row;  // data = 0x00 + row for cmd 0x01
    *(write_buf_ptr + 1) = 0x00; // data = 0x00 for cmd 0x02

    // Write two bytes of data for cmd 0x01 and 0x02
    // Set IF row address to 0x0000 and column address to 0
    ret = bat_write_i2c_retry(ROM_ADDR, 0x01, write_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: write data row,0x00 with command 0x01 failed\n",
        __func__);
      return ret;
    }

      //Write checksum to cmd 0x64 and 0x65, and data are as same as cmd 0x01 and 0x02
    ret = bat_write_i2c_retry(ROM_ADDR, 0x64, write_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: write data row,0x00 with command 0x64 failed\n",
        __func__);
      return ret;
    }

      //Read 96 bytes from data register 0x04 to instruction_bak arrary
    MSG2("%s: read instruction flash into instruction_bak\n",
      __func__);

    ret = bat_read_i2c_retry(ROM_ADDR, 0x04,
      instruction_bak_ptr + row * sizeof(instruction_bak[row]),
      sizeof(instruction_bak[row]));
    if (!ret) {
      MSG2("%s: read instruction flash failed\n", __func__);
      return ret;
    }

    /*flow char needs delay 20ms*/
    msleep(20);
  }

  if (!bq27520_erase_instruction_flash()) {
    MSG2("%s: erase instruction flash failed\n", __func__);
    return FALSE;
  }

  MSG2("%s: --start writing image--\n", __func__);

  for (i = 0; i < 3; i++) {
    MSG2("%s: Update DFI (%d)\n", __func__, i);

    DFI_checksum = 0;

    if (!bq27520_erase_data_flash()) {
      MSG2("%s: erase data flash failed\n", __func__);
      return FALSE;
    }

    if (!bq27520_dfi_write_data_flash(&DFI_checksum)) {
      MSG2("%s: write dfi data flash failed\n", __func__);
      return FALSE;
    }

    // Setup for Data Flash Checksum cmd
    MSG2("%s: --setup data flash checksum--\n", __func__);

    *write_buf_ptr = 0x08;
    *(write_buf_ptr + 1) = 0x00;

    ret = bat_write_i2c_retry(ROM_ADDR, 0x00, write_buf_ptr, 1);
    if (!ret) {
      MSG2("%s: write data 0x08 with command 0x00 failed\n",
        __func__);
      return ret;
    }

    ret = bat_write_i2c_retry(ROM_ADDR, 0x64, write_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: write data 0x08,0x00 with command 0x64 failed\n",
        __func__);
      return ret;
    }

    /*flow char needs delay 20ms*/
    msleep(20);

    // Read back gauge IC DFI checksum
    ret = bat_read_i2c_retry(ROM_ADDR, 0x04, read_buf_ptr, 2);
    if (!ret) {
      MSG2("%s: read data with command 0x04 failed\n", __func__);
      return ret;
    }

    DFI_checksum_RB = ((read_buf[1] << 8) & 0xFF00) | read_buf[0];

    // DFI checksum verification
    if (DFI_checksum == DFI_checksum_RB) {
      MSG2("%s: DFI checksum verify correct\n", __func__);
      break;
    } else {
      MSG2("%s: DFI_checksum(%d) != DFI_checksum_RB(%d)\n",
        __func__, DFI_checksum, DFI_checksum_RB);
    }

  }

  if (i >= 3) {
    MSG2("%s: Update DFI 3 times failed\n", __func__);
    return FALSE;
  }

    /* Restore Instruction Flash */
  MSG2("%s: --end writing image--\n", __func__);
  MSG2("%s: --restore instruction flash--\n", __func__);

  for (row = 1; row >= 0; row--) {
    for (i = 0; i < 3; i++) {
      // Send Program Row cmd
      *write_buf_ptr = 0x02;

      ret = bat_write_i2c_retry(ROM_ADDR, 0x00, write_buf_ptr, 1);
      if (!ret) {
        MSG2("*%s: write data 0x02 with command 0x00 failed\n",
          __func__);
        return ret;
      }

      // Write Data Row
      *write_buf_ptr = row;
      *(write_buf_ptr + 1) = 0x00;

      ret = bat_write_i2c_retry(ROM_ADDR, 0x01, write_buf_ptr, 2);
      if (!ret) {
        MSG2("%s: write data row,0x00 with command 0x01 failed\n",
          __func__);
        return ret;
      }
      // Start restore IF data
      ret = bat_write_i2c_retry(ROM_ADDR, 0x04, instruction_bak_ptr + row * sizeof(instruction_bak[row]), sizeof(instruction_bak[row]));
      if (!ret) {
        MSG2("%s: write data IFrowdata with command 0x04 failed\n",
          __func__);
        return ret;
      }

      // Calculate IF data checksum
      sum_of_dfi_bak = 0;
      for(num = 0; num < 96; num++) {
        sum_of_dfi_bak += *(instruction_bak_ptr + row * sizeof(instruction_bak[row]) + num);
      }

      // Send checksum
      checksum = (u16)((0x02 + row + sum_of_dfi_bak) % 0x10000);

      *write_buf_ptr = checksum & 0x00FF;
      *(write_buf_ptr + 1) = (checksum & 0xFF00) >> 8;

      ret = bat_write_i2c_retry(ROM_ADDR, 0x64, write_buf_ptr, 2);
      if (!ret) {
        MSG2("%s: write data LSB, MSB with command 0x64 failed\n",
          __func__);
        return ret;
      }

      /*flow char needs delay 20ms*/
      msleep(20);

      // Check for Program IF success
      *read_buf_ptr = 0xFF;

      ret = bat_read_i2c_retry(ROM_ADDR, 0x66, read_buf_ptr, 1);
      if (!ret) {
        MSG2("%s: read data with command 0x66 failed\n", __func__);
        return ret;
      }

      if (read_buf[0] == 0x00){
        break;
      }
    }//for(i = 0; i < 3; i++)

    // Check program IF data row number
    if (i >= 3) {
      return FALSE;
    }
  }

  return TRUE;
}

u8 bq27520_version_DFI(void)
{
  bool ret = FALSE;
  u8 version;
  u8 block_data_control = 0x00, data_flash_class = 0x39, data_flash_block = 0x00;

  ret = bat_write_i2c_retry(GAUGE_ADDR, 0x61, &block_data_control, 1);
  if (!ret) {
    MSG2("%s: write data 0x00 with command 0x61 failed\n", __func__);
    return ret;
  }

  ret = bat_write_i2c_retry(GAUGE_ADDR, 0x3E, &data_flash_class, 1);
  if (!ret) {
    MSG2("%s: write data 0x39 with command 0x3e failed\n", __func__);
    return ret;
  }

  ret = bat_write_i2c_retry(GAUGE_ADDR, 0x3F, &data_flash_block, 1);
  if (!ret) {
    MSG2("%s: write data 0x00 with command 0x3f failed\n", __func__);
    return ret;
  }
//todo
  ret = bat_read_i2c_retry(GAUGE_ADDR, 0x40, &version, 1);
  if (!ret) {
    MSG2("%s: read data with command 0x40 failed\n", __func__);
    return ret;
  } else
    MSG2("%s: ---bq27520 DFI version=0x%x---\n",__func__, version);

  return version;
}

static u8 bq27520_calibration_DFI(void)
{
  int ret, cmd = 0;
  unsigned short val = 0;

  MSG2("--%s--\n", __func__);

  if(bq27520_rom_mode_download)
  {
    MSG2("%s: bypass device check",__func__);
    goto update_dfi;
  }
  
  //check the gauge id
  ret = bat_write_i2c_cmd(BQ27520_SUBCMD_DEVCIE_TYPE);
  if (ret < 0) {
    MSG2("error %d writing subcmd device id failed\n", ret);
    return FALSE;
  }
  msleep(20);

  ret = bat_read_i2c_cmd(&val);
  if (ret < 0) {
    MSG2("error %d reading device id failed\n", ret);
    return FALSE;
  }

  if ( val != BQ27520_ID) {
    MSG2("%s: check gauge id(0x%04x) error!!\n", __func__, val);
    return FALSE;
  } else {
    MSG2("%s: check gauge id(0x%04x) success!!\n", __func__, val);
  }

  //check the gauge firmware version
  ret = bat_write_i2c_cmd(BQ27520_SUBCMD_FW_VER);
  if (ret < 0) {
    MSG2("error %d writing subcmd firmware version failed\n", ret);
    return FALSE;
  }
  msleep(20);

  ret = bat_read_i2c_cmd(&val);
  if (ret < 0) {
    MSG2("error %d reading firmware version failed\n", ret);
    return FALSE;
  }

  if ( val != BQ27520_FIRMWARE) {
    MSG2("%s: check firmware version(0x%04x) error!!\n",
      __func__, val);
    return FALSE;
  } else {
    MSG2("%s: check firmware version(0x%04x) success!!\n",
      __func__, val);
  }

  //check the gauge hardware version
  ret = bat_write_i2c_cmd(BQ27520_SUBCMD_HW_VER);
  if (ret < 0) {
    MSG2("error %d writing subcmd hardware version failed\n", ret);
    return FALSE;
  }
  msleep(20);

  ret = bat_read_i2c_cmd(&val);
  if (ret < 0) {
    MSG2("error %d reading hardware version failed\n", ret);
  return FALSE;
  }

  if ( val != BQ27520_HARDWARE) {
    MSG2("%s: check hardware version(0x%04x) error!!\n",
      __func__, val);
    return FALSE;
  } else {
    MSG2("%s: check hardware version(0x%04x) success!!\n",
      __func__, val);
  }

update_dfi:
  ret = bq27520_update_DFI();
  if (!ret) {
    MSG2("%s: ****** write Gauge data flash failed.******\n",
      __func__);
    return FALSE;
  }

  //exit rom mode
  if (!bq27520_enable_rom_mode(FALSE)) {
    return FALSE;
  }

  msleep(20);

  // Gauge IC Reset
  cmd = 0x0041;

  if (!bat_write_i2c_retry(GAUGE_ADDR, 0x00, (u8 *)&cmd, 2)) {
    return FALSE;
  }

  msleep(50);

  /* force the gauge to begin the impedance track algorithm
   * do not implement impedance track algorithm here,
   * the dummy battery can effect on gauge.
   * IT ENABLE can be write when all the whole work completed.*/
/* Jen Chang add for enable_it on ship os */
#ifdef CONFIG_BUILDTYPE_SHIP
  ret = bat_write_i2c_cmd(BQ27520_SUBCMD_ENABLE_IT);
  if (ret < 0) {
    MSG2("error %d writing subcmd enable it failed\n", ret);
    return FALSE;
  }

  msleep(20);
#endif
/* Jen Chang, 20111122 */
  bq27520_version_DFI();

  return TRUE;

}

u8 bq27520_force_calibration_DFI(void)
{
  int ret, cmd = 0;

  bq27520_update_DFI();

  //exit rom mode
  if (!bq27520_enable_rom_mode(FALSE)) {
    return FALSE;
  }

  msleep(20);

  // Gauge IC Reset
  cmd = 0x0041;

  if (!bat_write_i2c_retry(GAUGE_ADDR, 0x00, (u8 *)&cmd, 2)) {
    return FALSE;
  }

  msleep(50);

  /*force the gauge to begin the impedance track algorithm*/
  ret = bat_write_i2c_cmd(BQ27520_SUBCMD_ENABLE_IT);
  if (ret < 0) {
    MSG2("error %d writing cntl command failed\n", ret);
    return FALSE;
  }

  bq27520_version_DFI();

  return TRUE;
}

static int bq27520_copy_dfi_user_to_kernel(const char *val, u16 size)
{
  const char *dfi_ptr = val;
  int i = 0, ret = 0;

  MSG2("--%s--\n", __func__);

  yDataFlashImage = kmalloc(sizeof(u8)*size, GFP_KERNEL);
  if (yDataFlashImage == NULL) {
    MSG2("%s: could'nt request enough memory!\n", __func__);
    return FALSE;
  }

  memset(yDataFlashImage, 0, sizeof(u8) *size);
  memcpy(yDataFlashImage, dfi_ptr, sizeof(u8) *size);

  while (i < size) {
    printk("%02x ", *(yDataFlashImage + i));
    i++;
    if ((i % 16) == 0)
      printk("\n");
  }
  MSG2("\nDFI size is %d\n", (size - 1));

  /* we get the information of dfi file,such as size,address stored.
   * so we can request enough memory, copy DFI data from user space to kernel space.*/

  bq27520_dfi_update_ongoing = 1;
  ret = bq27520_calibration_DFI();
  bq27520_dfi_update_ongoing = 0;

  if (!ret) {
    MSG2("%s: ******calibration DFI failed.******\n", __func__);
    kfree(yDataFlashImage);
    return FALSE;
  } else
    MSG2("%s: === DFI write completed ===\n", __func__);

  msleep(20);

  kfree(yDataFlashImage);

  return TRUE;
}
#endif




//============================================================================//
//  i2c test
//============================================================================//
static void a2h(char *in, char *out) //ascii to hex, in[0]=MSB, in[1]=LSB
{
  int i;
  char a, h[2];
  for(i=0; i<2; i++)
  {
    a = *in++;
    if(a <= '9')        h[i] = a - '0';
    else if (a <= 'F')  h[i] = a - 'A' + 10;
    else if (a <= 'f')  h[i] = a - 'a' + 10;
    else                h[i] = 0;
  }
  *out = (h[0]<<4) + h[1];
}
static void h2a(char *in, char *out) //hex to ascii, out[0]=MSB, out[1]=LSB
{
  static const char my_ascii[] = "0123456789ABCDEF";
  char c = *in;
  *out++ =  my_ascii[c >> 4];
  *out =    my_ascii[c & 0xF];
}
static void bat_27520_i2c_test(unsigned char *bufLocal, int count, struct i2c_client *client)
{
  struct i2c_msg msgs[2];
  int i2c_ret, i, j;
  char id, reg[2], len, dat[BAT_BUF_LENGTH/4];

  printk(KERN_INFO "\n");

  //==========================
  // read (addr = 1 byte)
  //==========================
  if(bufLocal[1]=='r' && count>=9)
  {
    a2h(&bufLocal[2], &id);     //device id
    a2h(&bufLocal[4], &reg[0]); //reg addr
    a2h(&bufLocal[6], &len);    //data len
    if(len >= sizeof(dat))
    {
      MSG2("R %02X:%02X(%02d) Fail: max length=%d", id,reg[0],len,sizeof(dat));
      return;
    }
    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &reg[0];
    msgs[0].len = 1;
    msgs[1].addr = id;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = &dat[0];
    msgs[1].len = len;
    i2c_ret = i2c_transfer(client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      MSG2("R %02X:%02X(%02d) Fail: ret=%d", id,reg[0],len,i2c_ret);
      return;
    }
    j = 0;
    for(i=0; i<len; i++)
    {
      h2a(&dat[i], &bufLocal[j]);
      bufLocal[j+2] = ' ';
      j = j + 3;
    }
    bufLocal[j] = '\0';
    MSG2("R %02X:%02X(%02d) = %s", id,reg[0],len,bufLocal);
  }
  //==========================
  // read (addr = 2 byte)
  //==========================
  else if(bufLocal[1]=='R' && count>=11)
  {
    a2h(&bufLocal[2], &id);     //device id
    a2h(&bufLocal[4], &reg[0]); //reg addr
    a2h(&bufLocal[6], &reg[1]); //reg addr
    a2h(&bufLocal[8], &len);    //data len
    if(len >= sizeof(dat))
    {
      MSG2("R %02X:%02X%02X(%02d) Fail (max length=%d)", id,reg[0],reg[1],len,sizeof(dat));
      return;
    }
    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &reg[0];
    msgs[0].len = 2;
    msgs[1].addr = id;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = &dat[0];
    msgs[1].len = len;
    i2c_ret = i2c_transfer(client->adapter, msgs,2);
    if(i2c_ret != 2)
    {
      MSG2("R %02X:%02X%02X(%02d) Fail (ret=%d)", id,reg[0],reg[1],len,i2c_ret);
      return;
    }
    j = 0;
    for(i=0; i<len; i++)
    {
      h2a(&dat[i], &bufLocal[j]);
      bufLocal[j+2] = ' ';
      j = j + 3;
    }
    bufLocal[j] = '\0';
    MSG2("R %02X:%02X%02X(%02d) = %s", id,reg[0],reg[1],len,bufLocal);
  }
  //==========================
  // write
  //==========================
  else if(bufLocal[1]=='w' && count>=9)
  {
    a2h(&bufLocal[2], &id);     //device id
    len = count - 5;
    if(len & 1)
    {
      MSG2("W %02X Fail (invalid data) len=%d", id,len);
      return;
    }
    len = len/2;
    if(len >= sizeof(dat))
    {
      MSG2("W %02X Fail (too many data)", id);
      return;
    }
    j = 4;
    for(i=0; i<len; i++)
    {
      a2h(&bufLocal[j], &dat[i]);
      j = j + 2;
    }
    msgs[0].addr = id;
    msgs[0].flags = 0;
    msgs[0].buf = &dat[0];
    msgs[0].len = len;
    i2c_ret = i2c_transfer(client->adapter, msgs,1);
    //MSG2("W %s %s", &bufLocal[2], i2c_ret==1 ? "Pass":"Fail");
    MSG2("W %02X = %s", id, i2c_ret==1 ? "Pass":"Fail");
  }
  else
  {
    MSG2("rd: r40000B   (addr=40(7bit), reg=00, read count=11");
    MSG2("Rd: R2C010902 (addr=2C(7bit), reg=0109, read count=2");
    MSG2("wr: w40009265CA (addr=40(7bit), reg & data=00,92,65,CA...");
  }
}

//============================================================================//
static ssize_t bat_27520_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  /*if(bat_drv->inited && time_after(jiffies, bat_drv->jiff_property_valid_time))
  {
    queue_work(bat_drv->wqueue, &bat_drv->work);
  }*/
  return 0;
}
static ssize_t bat_27520_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned char bufLocal[BAT_BUF_LENGTH];

  printk(KERN_INFO "\n");
  if(count >= sizeof(bufLocal))
  {
    MSG2("%s input invalid, count = %d", __func__, count);
    return count;
  }
  memcpy(bufLocal,buf,count);

  switch(bufLocal[0])
  {
    //=================
    //  dynamic log
/*    case 'z':
      if(bufLocal[1]=='0')
      {
        MSG2("Dynamic Log All Off");
        bat_log_on1 = 0;
        bat_log_on2 = 1;
        bat_log_on3 = 0;
      }
      else if(bufLocal[1]=='1')
      {
        MSG2("Dynamic Log 1 On");
        bat_log_on1 = 1;
      }
      else if(bufLocal[1]=='3')
      {
        MSG2("Dynamic Log 3 On");
        bat_log_on3 = 1;
      }
      break;*/

    //=================
    //  gauge test
    case 'g':
      if(bufLocal[1]=='r' && bufLocal[2]=='1')
      {
        #ifdef BQ275250_DFI_SUPPORT
        MSG2("Gauge Rom mode = 1");
        bq27520_rom_mode_download = 1;
        #endif
      }
      else if(bufLocal[1]=='r')
      {
        #ifdef BQ275250_DFI_SUPPORT
        MSG2("Gauge Rom mode = 0");
        bq27520_rom_mode_download = 0;
        #endif
      }
      else if(bufLocal[1]=='v')
      {
      }
      else if(bufLocal[1]=='r')
      {
      }
      else
      {
        MSG2("Test Commands:");
        #ifdef BQ275250_DFI_SUPPORT
        MSG2("gr1: Gauge Rom mode = 1");
        MSG2("gr0: Gauge Rom mode = 0");
        #endif
      }
      break;

    //=================
    //  i2c test
    case 'i':
      bat_27520_i2c_test(bufLocal, count, bat_drv->client);
      break;
  }
  return count;
}
static struct device_attribute bat_ctrl_attrs[] = {
  __ATTR(ctrl, 0664, bat_27520_ctrl_show, bat_27520_ctrl_store),
};

//============================================================================//
int bat_27520_get_info(struct bat_gauge_info* binfo)
{
  int ret_gag1, ret_gag2;
  char gag_reg_00_15[22];
  char gag_reg_2c_2d[2];

  if(bq27520_dfi_update_ongoing)
    return -EBUSY;

  if(!bat_drv || !bat_drv->inited)  //driver not inited
    return -EIO;

  ret_gag1 = bat_read_i2c_retry(GAUGE_ADDR,0x00,&gag_reg_00_15[0],sizeof(gag_reg_00_15));
  ret_gag2 = bat_read_i2c_retry(GAUGE_ADDR,0x2c,&gag_reg_2c_2d[0],sizeof(gag_reg_2c_2d));
  if(ret_gag1 == 2 && ret_gag2 == 2)
  {
    binfo->data[GAG_CTRL]  = gag_reg_00_15[0x00] + (gag_reg_00_15[0x01]<<8);
    binfo->data[GAG_TEMP]  = gag_reg_00_15[0x06] + (gag_reg_00_15[0x07]<<8); //0.1K degree
    binfo->data[GAG_TEMP]  = binfo->data[GAG_TEMP]-2731;  // 0.1C degree
    binfo->data[GAG_VOLT]  = gag_reg_00_15[0x08] + (gag_reg_00_15[0x09]<<8);
    binfo->data[GAG_FLAG]  = gag_reg_00_15[0x0a] + (gag_reg_00_15[0x0b]<<8);
    binfo->data[GAG_RM]    = gag_reg_00_15[0x10] + (gag_reg_00_15[0x11]<<8);
    binfo->data[GAG_FCC]   = gag_reg_00_15[0x12] + (gag_reg_00_15[0x13]<<8);
    binfo->data[GAG_AI]    = gag_reg_00_15[0x14] + (gag_reg_00_15[0x15]<<8);
    binfo->data[GAG_SOC]   = gag_reg_2c_2d[0x00] + (gag_reg_2c_2d[0x01]<<8);
    return 0;
  }
/*  else
  {
    binfo->data[GAG_CTRL]  = 0;
    binfo->data[GAG_TEMP]  = 0;
    binfo->data[GAG_TEMP]  = 0;
    binfo->data[GAG_VOLT]  = 0;
    binfo->data[GAG_FLAG]  = 0;
    binfo->data[GAG_RM]    = 0;
    binfo->data[GAG_FCC]   = 0;
    binfo->data[GAG_AI]    = 0;
    binfo->data[GAG_SOC]   = 0;
  }*/
  return -EIO;
}
EXPORT_SYMBOL(bat_27520_get_info);

//============================================================================//
//  Work Function
//============================================================================//
/*#ifdef BAT_ALL_FEATURE
static void bat_27520_work_func(struct work_struct *work)
{
  MSG("%s+, read_again=%d",__func__,bat_drv->read_again);

  if(!bat_drv->inited) //exit if non-inited or suspended
  {
    MSG2("## Cancel Work, driver not inited! ##");
    return;
  }

  //mutex_lock(&bat_drv->work_lock);

  //===============
  //  battery info
  //===============

  //====================
  //  update bat_present
  //====================

  //===================
  //  update bat_status
  //===================

  //===================
  //  update bat_health
  //===================

  //=========================
  //  update irq
  //=========================
  //spin_lock_irqsave(&bat_27520_irq_lock, flags);
  //spin_unlock_irqrestore(&bat_27520_irq_lock, flags);

  bat_drv->jiff_property_valid_time = jiffies + bat_drv->jiff_property_valid_interval;

  //=========================
  //  assign next work time
  if(bat_drv->suspend_flag)
  {
    //don't schedule next timer function if in suspend
    //but if ac or usb still have voltage, keep timer registered
    del_timer_sync(&bat_drv->timer);
  }
  else
  {
    if(bat_drv->read_again > 0)
    {
      bat_drv->read_again --;
      mod_timer(&bat_drv->timer, jiffies + 1*HZ);
    }
    else
    {
      mod_timer(&bat_drv->timer, jiffies + bat_drv->jiff_polling_interval);
    }
  }

  //mutex_unlock(&bat_drv->work_lock);
  MSG("%s-, read_again=%d",__func__,bat_drv->read_again);
}
static void bat_27520_timer_func(unsigned long temp)
{
  MSG("%s",__func__);
  if(bat_drv->inited)
  {
    if(bat_drv->suspend_flag)
    {
      int ret;
      ret = queue_work(bat_drv->wqueue, &bat_drv->work);
      if(!ret)
        MSG2("%s, ## queue_work already ##", __func__);
    }
  }
}

//============================================================================//
//  irq handler
//============================================================================//
static irqreturn_t bat_27520_gag_int_irq_handler(int irq, void *args)
{
  if(bat_drv->inited)
  {
    disable_irq_nosync(bat_drv->irq_gag_int);
    bat_drv->read_again = 3;
    if(bat_drv->suspend_flag)  //in suspend?
    {
      MSG2("%s ## [GAG_INT] (wake_lock=2)", __func__);
      //wake_lock_timeout(&bat_drv->wlock, HZ*2);
    }
    else
    {
      int ret;
      MSG2("%s ## [GAG_INT] (queue work)", __func__);
      ret = queue_work(bat_drv->wqueue, &bat_drv->work);
      if(!ret)
        MSG2("%s, ## queue_work already ##", __func__);
    }
  }
  else
  {
    MSG2("%s ## [GAG_INT] Cancelled!", __func__);
  }
  return IRQ_HANDLED;
}
static irqreturn_t bat_27520_bat_low_irq_handler(int irq, void *args)
{
  if(bat_drv->inited)
  {
    disable_irq_nosync(bat_drv->irq_bat_low);
    bat_drv->read_again = 3;
    if(bat_drv->suspend_flag)  //in suspend?
    {
      MSG2("%s ## [BAT_LOW] (wake_lock=2)", __func__);
      //wake_lock_timeout(&bat_drv->wlock, HZ*2);
    }
    else
    {
      int ret;
      MSG2("%s ## [BAT_LOW] (queue work)", __func__);
      ret = queue_work(bat_drv->wqueue, &bat_drv->work);
      if(!ret)
        MSG2("%s, ## queue_work already ##", __func__);
    }
  }
  else
  {
    MSG2("%s ## [BAT_LOW] Cancelled!", __func__);
  }
  return IRQ_HANDLED;
}
#endif*/

//============================================================================//
//  debugfs
//============================================================================//
#ifdef BQ275250_DFI_SUPPORT
static int msm_update_gauge_dfi(struct seq_file *s, void *data)
{
  MSG2("--%s--", __func__);
  return 0;
}

static int msm_update_gauge_dfi_open(struct inode *inode, struct file *file)
{
  return single_open(file, msm_update_gauge_dfi, NULL);
}

static ssize_t msm_update_gauge_dfi_write(struct file *file,
  const char __user *buffer, size_t count, loff_t *pos)
{
  int ret, i;
  char *buf = (char *) __get_free_page(GFP_USER);

  MSG2("--%s--", __func__);

  if (!buf)
    return -ENOMEM;

  ret = -EINVAL;
  if (count >= PAGE_SIZE)
    goto out;

  ret = -EFAULT;
  if (copy_from_user(buf, buffer, count))
    goto out;

  buf[count] = '\0';
  for (i = 0; i < count; i++) {
    printk("%02x ", buf[i]);
    if (((i + 1) % 16) == 0)
      printk("\n");
  }

  ret = bq27520_copy_dfi_user_to_kernel((const char *)buf, count);
  if (ret)
    ret = count;
  else {
    ret = -EFAULT;
    MSG2("%s: copy dif from user to kernel failed", __func__);
  }

out:
  free_page((unsigned long)buf);
  return ret;
}

static const struct file_operations msm_update_gauge_dfi_operations = {
  .open     = msm_update_gauge_dfi_open,
  .read     = seq_read,
  .llseek   = seq_lseek,
  .release  = single_release,
  .write    = msm_update_gauge_dfi_write,
};
#endif

//============================================================================//
//  debugfs
//============================================================================//
#if defined(CONFIG_DEBUG_FS)
static int bat_27520_read_id(void *data, u64 * val)
{
  int ret;
  unsigned short dd;
  ret = bat_write_i2c_cmd(BQ27520_SUBCMD_DEVCIE_TYPE);
  msleep(5);
  ret = bat_read_i2c_cmd(&dd);
  if(ret == 2)
    *val = dd;
  else
    *val = -EIO;
  return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bat_read_id, bat_27520_read_id, NULL, "%lld\n");

/*static int bat_27520_read_info(void *data, u64 * val)
{
  int cmd = (int)data;
  int ret, temp;
  unsigned char read[2];

	switch(cmd)
	{
  	case BQ27520_REG_TEMP:
  	  ret = bat_read_i2c_retry(GAUGE_ADDR, cmd, &read[0], sizeof(read));
  	  temp = (int)(read[0] + (read[1] << 8)); //0.1K degree
  	  *val = (int)(temp - 2731);              //0.1C degree
  		break;

  	case BQ27520_REG_VOLT:
  	  ret = bat_read_i2c_retry(GAUGE_ADDR, cmd, &read[0], sizeof(read));
  	  *val = (int)(read[0] + (read[1] << 8));
  		break;

  	case BQ27520_REG_AI:
  	  ret = bat_read_i2c_retry(GAUGE_ADDR, cmd, &read[0], sizeof(read));
  	  *val = (int)(read[0] + (read[1] << 8));
  		break;
  	default:
  	  *val = 0;
  		ret = -EINVAL;
  		goto exit;
	}
  if(ret == 2)  //read success
  {
    ret = 0;
  }
  else
  {
    *val = 0;
    ret = -EIO;
  }

exit:
	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(bat_read_info, bat_27520_read_info, NULL, "%lld\n");*/

static int bat_27520_debugfs_ftd_open(struct inode *inode, struct file *file)
{
  //MSG2("%s",__func__);
  //MSG2("%s, inode = %X, inode->i_private   = %X",__func__,(int)inode,(int)inode->i_private);
  //MSG2("%s, file =  %X, file->private_data = %X",__func__,(int)file,(int)file->private_data);
  //non-seekable
  file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
  file->private_data = inode->i_private;
  return 0;
}
static int bat_27520_debugfs_ftd_release(struct inode *inode, struct file *file)
{
  //MSG2("%s",__func__);
  file->private_data = 0;
  return 0;
}
static ssize_t bat_27520_debugfs_ftd_read(
    struct file *file,
    char __user *buff,
    size_t count,
    loff_t *ppos)
{
  int ret = 0, size;
  int ret_i2c, cmd;
  int tmp32;
  short tmp16;
  char local[64];
  char read[2];

  //MSG2("%s+, count=%d, ppos=%d, file->private_data = %X",__func__,count,(int)*ppos,(int)file->private_data);
  if(*ppos)
  {
    size = 0;
    goto exit;
  }
  if(sizeof(local) > count)
  {
    ret = -EFBIG;
    goto exit;
  }

  cmd = (int)file->private_data;
	switch(cmd)
	{
  	case BQ27520_REG_TEMP:
  	  ret_i2c = bat_read_i2c_retry(GAUGE_ADDR, cmd, &read[0], sizeof(read));
  	  tmp16 = read[0] + (read[1] << 8);   //0.1K degree
  	  tmp32 = (int)tmp16 - 2731;          //0.1C degree
  		break;
  	case BQ27520_REG_VOLT:
  	  ret_i2c = bat_read_i2c_retry(GAUGE_ADDR, cmd, &read[0], sizeof(read));
  	  tmp16 = read[0] + (read[1] << 8);   //mV
  	  tmp32 = (int)tmp16 * 1000;          //uV
  		break;
  	case BQ27520_REG_AI:
  	  ret_i2c = bat_read_i2c_retry(GAUGE_ADDR, cmd, &read[0], sizeof(read));
  	  tmp16 = read[0] + (read[1] << 8);   //mA
  	  tmp32 = (int)tmp16 * 1000;          //uA
  	  tmp32 = -tmp32;                     //follow BMS define, negative = charge, positive = discharge
  		break;
  	default:
  	  MSG2("%s, invalid command = %d",__func__,cmd);
  	  ret = -EINVAL;
  		goto exit;
	}
  if(ret_i2c == 2)  //read success
  {
    memset(local, 0, sizeof(local));
    size = scnprintf(local, sizeof(local), "%d\n", tmp32);
  }
  else  //read fail
  {
    MSG2("%s, i2c read fail!",__func__);
    ret = -EIO;
    goto exit;
  }
  if(copy_to_user(buff, local, size))
  {
    ret = -EFAULT;
    //goto exit;
  }
exit:
  if(!ret)
  {
    *ppos += size;
    ret = size;
  }
  //MSG2("%s-, ret=%d",__func__,ret);
  return ret;
}
static const struct file_operations bat_27520_debugfs_ftd_fops = {
  .open     = bat_27520_debugfs_ftd_open,
  .release  = bat_27520_debugfs_ftd_release,
  .read     = bat_27520_debugfs_ftd_read,
};

//============================================================================//
//  ctrl test
//============================================================================//
static int bat_27520_debugfs_open(struct inode *inode, struct file *file)
{
  //non-seekable
  file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
  return 0;
}
static int bat_27520_debugfs_release(struct inode *inode, struct file *file)
{
  return 0;
}
static ssize_t bat_27520_debugfs_read(
    struct file *file,
    char __user *buff,
    size_t count,
    loff_t *ppos)
{
  int ret = 0, size;
  //MSG2("%s+, count=%d, ppos=%d",__func__,count,(unsigned int)*ppos);
  if(*ppos)
  {
    size = 0;
    goto exit;
  }
  size = bat_27520_ctrl_show(NULL, NULL, bat_drv->debug_read_buf);
  if(size > count)
  {
    ret = -EFAULT;
    goto exit;
  }
  if(copy_to_user(buff, bat_drv->debug_read_buf, size))
  {
    ret = -EFAULT;
    goto exit;
  }
exit:
  if(!ret)
  {
    *ppos += size;
    ret = size;
  }
  //MSG2("%s-, ret=%d",__func__,ret);
  return ret;
}
static ssize_t bat_27520_debugfs_write(
    struct file *file,
    const char __user *buff,
    size_t count,
    loff_t *ppos)
{
  int ret = 0;
  //MSG2("%s+, count=%d, ppos=%d",__func__,count,(unsigned int)*ppos);
  if(count > sizeof(bat_drv->debug_write_buf))
  {
    ret = -EFAULT;
    goto exit;
  }
  if(copy_from_user(bat_drv->debug_write_buf, buff, count))
  {
    MSG2("%s, failed to copy from user",__func__);
    ret = -EFAULT;
    goto exit;
  }
  bat_drv->debug_write_buf[count] = '\0';
  bat_27520_ctrl_store(NULL, NULL, bat_drv->debug_write_buf, count);
exit:
  if(!ret)
    ret = count;
  //MSG2("%s-, ret=%d",__func__,ret);
  return ret;
}
static const struct file_operations bat_27520_debugfs_fops = {
  .open     = bat_27520_debugfs_open,
  .release  = bat_27520_debugfs_release,
  .read     = bat_27520_debugfs_read,
  .write    = bat_27520_debugfs_write,
};
//============================================================================//
//  init
//============================================================================//
static void bat_27520_debug_init(void)
{
  bat_drv->dent = debugfs_create_dir("bat_27520", 0);
  if (IS_ERR(bat_drv->dent))
    return;
  debugfs_create_file("ctrl",         0664, bat_drv->dent, NULL, &bat_27520_debugfs_fops);
  debugfs_create_file("gauge_id",     0444, bat_drv->dent, NULL, &bat_read_id);
  debugfs_create_file("gauge_temp",   0444, bat_drv->dent, (void *)BQ27520_REG_TEMP, &bat_27520_debugfs_ftd_fops);
  debugfs_create_file("gauge_volt",   0444, bat_drv->dent, (void *)BQ27520_REG_VOLT, &bat_27520_debugfs_ftd_fops);
  debugfs_create_file("gauge_curr",   0444, bat_drv->dent, (void *)BQ27520_REG_AI,   &bat_27520_debugfs_ftd_fops);
  #ifdef BQ275250_DFI_SUPPORT
  debugfs_create_file("gauge_update_dfi", S_IWUGO, bat_drv->dent, NULL, &msm_update_gauge_dfi_operations);
  #endif
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bat_27520_early_suspend(struct early_suspend *h)
{
  //MSG2("%s+", __func__);
  bat_drv->early_suspend_flag = 1;
  //MSG2("%s-", __func__);
}
static void bat_27520_late_resume(struct early_suspend *h)
{
  //MSG2("%s+", __func__);
  bat_drv->early_suspend_flag = 0;
  /*#ifdef BAT_ALL_FEATURE
    if(bat_drv->inited)
    {
      bat_drv->read_again ++;
      queue_work(bat_drv->wqueue, &bat_drv->work);
    }
  #endif*/
  //MSG2("%s-", __func__);
}
#endif


#ifdef CONFIG_PM
static int bat_27520_i2c_suspend(struct device *dev)
{
  MSG("%s", __func__);
  bat_drv->suspend_flag = 1;
  /*#ifdef BAT_ALL_FEATURE
    if(bat_drv->inited)
    {
      del_timer_sync(&bat_drv->timer);
      cancel_work_sync(&bat_drv->work);
      flush_workqueue(bat_drv->wqueue);
    }
  #endif*/
  return 0;
}
static int bat_27520_i2c_resume(struct device *dev)
{
  MSG("%s", __func__);
  bat_drv->suspend_flag = 0;
  /*#ifdef BAT_ALL_FEATURE
    if(bat_drv->inited)
    {
      int ret;
      ret = queue_work(bat_drv->wqueue, &bat_drv->work);
      if(!ret)
        MSG2("%s, ## queue_work already ##", __func__);
    }
  #endif*/
  return 0;
}
static const struct dev_pm_ops bat_27520_pm_ops = {
  .suspend  = bat_27520_i2c_suspend,
  .resume   = bat_27520_i2c_resume,
};
#endif
static void bat_27520_i2c_shutdown(struct i2c_client *client)
{
  MSG2("%s",__func__);
}
#define GAG_GPIO_BAT_LOW  109
#define GAG_GPIO_GAG_INT  110
static int bat_27520_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int ret, i;
  char gag_ctrl[2];
  char gag_cmd[2];

  //MSG2("%s+, client=%s, id=%s, adapter=%s, system_rev = %d", __func__,
  //  client->name, id->name, client->adapter->name, system_rev);
  MSG2("%s+", __func__);

  //============================================================================//
  //  init
  //============================================================================//
  bat_drv = kzalloc(sizeof(struct bat_drv_data), GFP_KERNEL);
  if(bat_drv == NULL)
  {
    MSG2("%s, kmalloc fail!",__func__);
    ret = -ENOMEM;
    goto exit_err;
  }

  //interval
  bat_drv->jiff_property_valid_interval = 1*HZ/2;
  bat_drv->jiff_polling_interval = 10*HZ;
  //gpio,irq
  bat_drv->gpio_gag_int  = GAG_GPIO_GAG_INT;
  bat_drv->gpio_bat_low  = GAG_GPIO_BAT_LOW;
  bat_drv->irq_gag_int   = MSM_GPIO_TO_INT(GAG_GPIO_GAG_INT);
  bat_drv->irq_bat_low   = MSM_GPIO_TO_INT(GAG_GPIO_BAT_LOW);

  //============================================================================//
  //  gpio irq config
  //============================================================================//
  //gag_int
  //gpio
  bat_drv->gpio_gag_int_ret =
    gpio_request(bat_drv->gpio_gag_int, "bat_gag_int");
  if(bat_drv->gpio_gag_int_ret)
    MSG2("%s, %d gpio_request, FAIL = %d", __func__, bat_drv->gpio_gag_int, bat_drv->gpio_gag_int_ret);
  /*#ifdef BAT_ALL_FEATURE
    //irq
    //set_irq_flags(bat_drv->irq_gag_int, IRQF_VALID | IRQF_NOAUTOEN);
    bat_drv->irq_gag_int_ret =
      request_irq(bat_drv->irq_gag_int, &bat_27520_gag_int_irq_handler,
      IRQF_TRIGGER_FALLING, "bat_gag_int", NULL);
    if(bat_drv->irq_gag_int_ret)
      MSG2("%s, %d request_irq, FAIL = %d", __func__, bat_drv->irq_gag_int, bat_drv->irq_gag_int_ret);
    ret = irq_set_irq_wake(bat_drv->irq_bat_low, 1);
    if(ret)
      MSG2("%s, %d irq_set_irq_wake, FAIL = %d", __func__, bat_drv->irq_gag_int, ret);
    //disable_irq(bat_drv->irq_gag_int);
  #endif*/

  //bat_low
  //gpio
  bat_drv->gpio_bat_low_ret =
    gpio_request(bat_drv->gpio_bat_low, "bat_bat_low");
  if(bat_drv->gpio_bat_low_ret)
    MSG2("%s, %d gpio_request, FAIL = %d", __func__, bat_drv->gpio_bat_low, bat_drv->gpio_bat_low_ret);
  /*#ifdef BAT_ALL_FEATURE
    //irq
    //set_irq_flags(bat_drv->irq_bat_low, IRQF_VALID | IRQF_NOAUTOEN);
    bat_drv->irq_bat_low_ret =
      request_irq(bat_drv->irq_bat_low, &bat_27520_bat_low_irq_handler,
      IRQF_TRIGGER_FALLING, "bat_bat_low", NULL);
    if(bat_drv->irq_bat_low_ret)
      MSG2("%s, %d request_irq, FAIL = %d", __func__, bat_drv->irq_bat_low, bat_drv->irq_bat_low_ret);
    ret = irq_set_irq_wake(bat_drv->irq_bat_low, 1);
    if(ret)
      MSG2("%s, %d irq_set_irq_wake, FAIL = %d", __func__, bat_drv->irq_bat_low, ret);
    //disable_irq(bat_drv->irq_bat_low);
  #endif*/

  //============================================================================//
  //  Regulator
  //============================================================================//
  /*//MSG2("%s, client->dev->init_name = %s",__func__,client->dev.init_name);
  //bat_drv->vio = regulator_get(&client->dev, "vcc_i2c_bat_27520");
  bat_drv->vio = regulator_get(NULL, "vcc_i2c_bat_27520");
  if(IS_ERR(bat_drv->vio))
  {
    MSG2("%s, regulator_get vcc_i2c_bat, FAIL = %d", __func__, (int)bat_drv->vio);
    bat_drv->vio = NULL;
  }
  else
  {
    ret = regulator_enable(bat_drv->vio);
    if(!ret)
      MSG2("%s, regulator_enable vcc_i2c_bat_27520, PASS", __func__);
    else
      MSG2("%s, regulator_enable vcc_i2c_bat_27520, FAIL = %d", __func__, ret);
  }*/

  bat_drv->client = client;

  //============================================================================//
  //  gagic i2c verify
  //    set IT_ENALBE, enable impedence track algorithm
  //    set sleep+, for avoid i2c SCL being pull-down by gauge
  //============================================================================//
  {
    //set "read device type"
    gag_cmd[0] = 0x01;  gag_cmd[1] = 0x00;
    ret = bat_write_i2c_retry(GAUGE_ADDR,0x00,&gag_cmd[0],sizeof(gag_cmd));
    if((ret != 1) || (ret < 0))
      goto exit_i2c_fail;
    msleep(5);
    ret = bat_read_i2c_retry(GAUGE_ADDR,0x00,&gag_ctrl[0],sizeof(gag_ctrl));
    MSG2("%s, DEVICE_TYPE %04X",__func__,gag_ctrl[0]+(gag_ctrl[1]<<8));

    //set "read firmware version"
    gag_cmd[0] = 0x02;  gag_cmd[1] = 0x00;
    ret = bat_write_i2c_retry(GAUGE_ADDR,0x00,&gag_cmd[0],sizeof(gag_cmd));
    msleep(5);
    ret = bat_read_i2c_retry(GAUGE_ADDR,0x00,&gag_ctrl[0],sizeof(gag_ctrl));
    MSG2("%s, FW_VERSION  %04X",__func__,gag_ctrl[0]+(gag_ctrl[1]<<8));

    //set "read control status"
    gag_cmd[0] = 0x00;  gag_cmd[1] = 0x00;
    ret = bat_write_i2c_retry(GAUGE_ADDR,0x00,&gag_cmd[0],sizeof(gag_cmd));
    msleep(5);
    ret = bat_read_i2c_retry(GAUGE_ADDR,0x00,&gag_ctrl[0],sizeof(gag_ctrl));
    MSG2("%s, CTRL_STATUS %04X",__func__,gag_ctrl[0]+(gag_ctrl[1]<<8));

    // 20101028, enable Qmax if not enabled, for Austin and Toucan
    /*{
      if(!TST_BIT(gag_ctrl[0],0) && !bat_drv->i2c_err) //QEN is not 1?
      {
        gag_cmd[0] = 0x21;        //IT_ENABLE, refer to "bq27500-v120.pdf page 10"
        gag_cmd[1] = 0x00;
        ret = bat_write_i2c_retry(GAUGE_ADDR,0x00,&gag_cmd[0],sizeof(gag_cmd));
        MSG2("%s Gauge IC  QMax Enable %s!",__func__,ret==1?"Pass":"Fail");
      }
      else if(TST_BIT(gag_ctrl[0],0))
      {
        MSG2("%s Gauge IC  QMax Enabled!",__func__);
      }
    }*/

    /*if(!TST_BIT(gag_ctrl[0],5) && !bat_drv->i2c_err) //SNOOZE is not 1?
    {
      gag_cmd[0] = 0x13;        //SET_SLEEP+, refer to "bq27500-v120.pdf page 10"
      gag_cmd[1] = 0x00;
      ret = bat_write_i2c_retry(GAUGE_ADDR,0x00,&gag_cmd[0],sizeof(gag_cmd));
      MSG2("%s Gauge IC  set SLEEP+ %s!",__func__,ret==1?"Pass":"Fail");
    }*/

    //set as "read control status", will be used in engineer mode 2
    gag_cmd[0] = 0x00;  gag_cmd[1] = 0x00;
    ret = bat_write_i2c_retry(GAUGE_ADDR,0x00,&gag_cmd[0],sizeof(gag_cmd));
    MSG2("%s, (%02X) access %s!",__func__,GAUGE_ADDR<<1,!bat_drv->i2c_err?"Pass":"Fail");
  }

  //================//
  //  driver function
  //================//
  /*#ifdef BAT_ALL_FEATURE
    //mutex
    mutex_init(&bat_drv->mutex);

    //timer
    init_timer(&bat_drv->timer);
    bat_drv->timer.function = bat_27520_timer_func;
    bat_drv->timer.expires = jiffies + 10*HZ;

    //workqueue
    INIT_WORK(&bat_drv->work, bat_27520_work_func);
    bat_drv->wqueue = create_singlethread_workqueue("bat_27520_workqueue");
    if(bat_drv->wqueue) //work queue success
    {
      MSG("%s bat_27520_workqueue created PASS!",__func__);
    }
    else  //fail
    {
      MSG2("%s bat_27520_workqueue created FAIL!",__func__);
      goto exit_err;
    }
    queue_work(bat_drv->wqueue, &bat_drv->work);
  #endif*/

  #ifdef CONFIG_HAS_EARLYSUSPEND
    bat_drv->early_suspend.level   = 150;
    bat_drv->early_suspend.suspend = bat_27520_early_suspend;
    bat_drv->early_suspend.resume  = bat_27520_late_resume;
    register_early_suspend(&bat_drv->early_suspend);
  #endif

  #if defined(CONFIG_DEBUG_FS)
    bat_27520_debug_init();
  #endif

  //create attr file
  for(i=0; i<ARRAY_SIZE(bat_ctrl_attrs); i++)
  {
    ret = device_create_file(&client->dev, &bat_ctrl_attrs[i]);
    if(ret) MSG2("%s: create FAIL, ret=%d",bat_ctrl_attrs[i].attr.name,ret);
  }

  bat_drv->inited = 1;

  MSG2("%s-, ret=0",__func__);
  return 0;

exit_i2c_fail:
  bat_drv->client = NULL;
  if(!bat_drv->gpio_bat_low_ret)
  {
    gpio_free(bat_drv->gpio_bat_low);
  }
  if(!bat_drv->gpio_gag_int_ret)
  {
    gpio_free(bat_drv->gpio_gag_int);
  }
  kfree(bat_drv);

exit_err:
  MSG2("%s-, ret=-1",__func__);
  return -1;
}

static const struct i2c_device_id bat_27520_id[] = { { "bat_27520", 0 } };
static struct i2c_driver bat_27520_driver = {
  .driver.owner = THIS_MODULE,
  .driver.name  = "bat_27520",
  #ifdef CONFIG_PM
    .driver.pm = &bat_27520_pm_ops,
  #endif
  .id_table = bat_27520_id,
  .shutdown = bat_27520_i2c_shutdown,
  .probe    = bat_27520_i2c_probe,
};

static int __init bat_27520_init(void)
{
  int ret;
  printk("BootLog, +%s\n", __func__);
  ret = i2c_add_driver(&bat_27520_driver);
  printk("BootLog, -%s, ret=%d\n", __func__,ret);
  return ret;
}

module_init(bat_27520_init);
MODULE_DESCRIPTION("Battery 27520 Driver");


