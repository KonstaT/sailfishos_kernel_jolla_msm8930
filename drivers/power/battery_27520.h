//============================================================================//
//
//  Battery driver for Detroit project
//
//  Author:   Eric Liu (Qisda Corp.)
//
//============================================================================//
//============================================================================//
//  Date:     2011.12.23
//
//  HW base:  EVT1
//  SW base:  1023
//
//============================================================================//

#ifndef __BATTERY_27520_H__
#define __BATTERY_27520_H__

#include <linux/earlysuspend.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>

//============================================================================//
//  my macro
//============================================================================//
//#define MYBIT(b)        (1<<b)
#define TST_BIT(x,b)    ((x & (1<<b)) ? 1 : 0)
#define CLR_BIT(x,b)    (x &= (~(1<<b)))
#define SET_BIT(x,b)    (x |= (1<<b))

//============================================================================//
//  i2c addr
//============================================================================//
#define GAUGE_ADDR    (0xAA >> 1)
#define ROM_ADDR      (0x16 >> 1)

//============================================================================//
//  gpio pin define
//============================================================================//
#define BAT_GPIO_BAT_LOW    40
#define BAT_GPIO_CHG_INT    153

#define BAT_I2C_RETRY_MAX   5

#define BAT_BUF_LENGTH      256

//============================================================================//
//  standard data commands
//============================================================================//
#define BQ27520_REG_CNTL    0x00
#define BQ27520_REG_AR      0x02
#define BQ27520_REG_ARTTE   0x04
#define BQ27520_REG_TEMP    0x06
#define BQ27520_REG_VOLT    0x08
#define BQ27520_REG_FLAGS   0x0A
#define BQ27520_REG_NAC     0x0C
#define BQ27520_REG_FAC     0x0e
#define BQ27520_REG_RM      0x10
#define BQ27520_REG_FCC     0x12
#define BQ27520_REG_AI      0x14
#define BQ27520_REG_TTE     0x16
#define BQ27520_REG_TTF     0x18
#define BQ27520_REG_SI      0x1a
#define BQ27520_REG_STTE    0x1c
#define BQ27520_REG_MLI     0x1e
#define BQ27520_REG_MLTTE   0x20
#define BQ27520_REG_AE      0x22
#define BQ27520_REG_AP      0x24
#define BQ27520_REG_TTECP   0x26
#define BQ27520_REG_SOH     0x28
#define BQ27520_REG_SOC     0x2c
#define BQ27520_REG_NIC     0x2e
#define BQ27520_REG_ICR     0x30
#define BQ27520_REG_LOGIDX  0x32
#define BQ27520_REG_LOGBUF  0x34

//============================================================================//
//  Control subcommands
//============================================================================//
#define BQ27520_SUBCMD_CTNL_STATUS  0x0000
#define BQ27520_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27520_SUBCMD_FW_VER       0x0002
#define BQ27520_SUBCMD_HW_VER       0x0003
#define BQ27520_SUBCMD_DF_CSUM      0x0004
#define BQ27520_SUBCMD_PREV_MACW    0x0007
#define BQ27520_SUBCMD_CHEM_ID      0x0008
#define BQ27520_SUBCMD_BD_OFFSET    0x0009
#define BQ27520_SUBCMD_INT_OFFSET   0x000a
#define BQ27520_SUBCMD_CC_VER       0x000b
#define BQ27520_SUBCMD_OCV          0x000c
#define BQ27520_SUBCMD_BAT_INS      0x000d
#define BQ27520_SUBCMD_BAT_REM      0x000e
#define BQ27520_SUBCMD_SET_HIB      0x0011
#define BQ27520_SUBCMD_CLR_HIB      0x0012
#define BQ27520_SUBCMD_SET_SLP      0x0013
#define BQ27520_SUBCMD_CLR_SLP      0x0014
#define BQ27520_SUBCMD_FCT_RES      0x0015
#define BQ27520_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27520_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27520_SUBCMD_SEALED       0x0020
#define BQ27520_SUBCMD_ENABLE_IT    0x0021
#define BQ27520_SUBCMD_DISABLE_IT   0x0023
#define BQ27520_SUBCMD_CAL_MODE     0x0040
#define BQ27520_SUBCMD_RESET        0x0041


//============================================================================//
//  declaration
//============================================================================//
struct bat_drv_data
{
  struct mutex        mutex;
  struct i2c_client   *client;
  struct wake_lock    wlock;
  struct timer_list   timer;
  struct work_struct  work;
  struct workqueue_struct *wqueue;
  #ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
  #endif
  //struct regulator    *vio;   //i2c bus pull high voltage

  //flag of init
  char  inited;             // 1: inited,     0: not inited,
  char  suspend_flag;       // 1: suspend,    0: active
  char  early_suspend_flag; // 1: early_suspend, 0: normal
  char  wake_flag;          // 1: wake lock,  0: no wake lock

  int   read_again;     //gauge ic need some time to enter stable state
                        //after charger in/out, we need to read again to get correct state

  //i2c access error
  int   i2c_err;

  //gpio and irq
  int gpio_gag_int;
  int gpio_bat_low;
  //int gpio_bat_god;
  int irq_gag_int;
  int irq_bat_low;
  //int irq_bat_god;

  //gpio and irq, return value, 0 means success
  int gpio_gag_int_ret;
  int gpio_bat_low_ret;
  //int gpio_bat_god_ret;
  int irq_gag_int_ret;
  int irq_bat_low_ret;
  //int irq_bat_god_ret;

  //if the jiff is bigger than this, the property will be invalid
  //we must get new gag and chg info
  unsigned long jiff_property_valid_time;

  //the interval of valid property
  unsigned long jiff_property_valid_interval;

  //the interval of update gauge & charge info
  unsigned long jiff_polling_interval;


  //for low bat power off
  int bat_low_count;  //if polling 10 times and still bat low, sending capacity 0 to AP
  unsigned long jiff_bat_low_count_wait_time; //low bat count wait time

  //gauge info
  short gag_ctrl;       //control status
  short gag_flag;       //gag flags
  short gag_rm;         //gag rm
  short gag_fcc;        //gag fcc
  short gag_ai;         //current (ma)

  //debugfs
  struct dentry   *dent;
  char debug_write_buf[128];
  char debug_read_buf[128];
};

//============================================================================//
//  gauge info
//============================================================================//
enum {GAG_CTRL=0, GAG_TEMP, GAG_VOLT, GAG_FLAG, GAG_RM, GAG_FCC, GAG_AI, GAG_SOC, GAG_MAX_NUM,};
struct bat_gauge_info
{
  short data[GAG_MAX_NUM];
};

#ifdef CONFIG_BATTERY_27520
int bat_27520_get_info(struct bat_gauge_info* binfo);
#else
static inline int bat_27520_get_info(struct bat_gauge_info* binfo)  { return -ENODEV; }
#endif

#endif  //__BATTERY_27520_H__

