//============================================================================//
//
//  Flash Light driver for ADP1650 (Detroit project)
//
//  Author:   Eric Liu (Qisda Corp.)
//
//============================================================================//
//============================================================================//
//  Date:     2012.01.02
//
//  HW base:  EVT1
//  SW base:  1023
//
//============================================================================//

#ifndef __FLASH_ADP1650_H__
#define __FLASH_ADP1650_H__

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
//#include <linux/wakelock.h>
#include <mach/camera.h>

/* Sophia, 20120525 , add pm log*/
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif

//============================================================================//
//  my macro
//============================================================================//
//#define MYBIT(b)        (1<<b)
#define TST_BIT(x,b)    ((x & (1<<b)) ? 1 : 0)
#define CLR_BIT(x,b)    (x &= (~(1<<b)))
#define SET_BIT(x,b)    (x |= (1<<b))

#define I2C_RETRY_MAX   5
#define I2C_BUF_LENGTH  256

//============================================================================//
//  declaration
//============================================================================//
struct adp1650_drv_data
{
  struct mutex        mutex;
  struct i2c_client   *client;
  struct regulator    *vio;   //i2c bus pull high voltage
  //struct wake_lock    wlock;

  //flag of init
  //char  inited;             // 1: inited,     0: not inited,
  //char  suspend_flag;       // 1: suspend,    0: active
  //char  wake_flag;          // 1: wake lock,  0: no wake lock

  char  i2c_addr;
  char  chipid;
  char  i2c_bus_enabled;
  int   i2c_err;

  int   i2c_gpio20_requested; //request gpio to enable i2c bus
  int   i2c_gpio21_requested;
  int   gpio_8m_cam_analog_request; // sophia, 201301111, add for fix ftd flash 

  //gpio
  int gpio_led_en;
  int gpio_strobe;
  int gpio_gpio1;
  int gpio_gpio2;

  // 20130711, sophia
  // if the flash i2c bus is not the same as camera sensor, we need to provide power for flash independently
  // boston: the same in i2c 4bus
  // casper: flash in bus 5, camea is bus4
  // sapporo: flash in bus 5, camea is bus4 
  int  in_the_same_i2c_bus_with_camera;

  #ifdef CONFIG_PM_LOG
  struct pmlog_device *pmlog_device;
  #endif
};

int adp1650_flash_control(struct msm_camera_sensor_flash_external *external, unsigned led_state);

#endif  //__FLASH_ADP1650_H__

