//============================================================================//
//
//  Flash Light driver for ADP1650 (Detroit project)
//
//  Author:   Eric Liu (Qisda Corp.)
//
//============================================================================//
//============================================================================//
//  Date:     2012.01.03
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
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include "flash_adp1650.h"

// sophia wang++, 20130702, adp1650 may in different bus in differt project
// need hwid information
#include <mach/hwid.h>
// sophia wang--

//static int adp1650_log_on1  = 0;
//static int adp1650_log_on2  = 1;
//static int adp1650_log_on3  = 0;
//#define MSG(format, arg...)   {if(adp1650_log_on1)  printk(KERN_INFO "[FLASH]" format "\n", ## arg);}
//#define MSG2(format, arg...)  {if(adp1650_log_on2)  printk(KERN_INFO "[FLASH]" format "\n", ## arg);}
//#define MSG3(format, arg...)  {if(adp1650_log_on3)  printk(KERN_INFO "[FLASH]" format "\n", ## arg);}

//#define MSG2(format, arg...)  {printk(KERN_INFO "[FLASH]" format "\n", ## arg);}
#define MSG2(format, arg...)  {printk(KERN_ERR "[FLASH]" format "\n", ## arg);}

//#define DEBUG
// #define PERF
uint flash_current = 0xE;
uint assist_current = 0x3;
uint flash_duration = 0x2;
	
//#define STROBE

unsigned int preMode;
#ifdef PERF
struct timespec start = {0, 0};
#endif
//============================================================================//
//  driver
//============================================================================//
static struct adp1650_drv_data *adp1650_drv;

//============================================================================//
//  debugfs
//============================================================================//
static struct dentry *dent;
unsigned short adp1650_chipID = 0;
//============================================================================//
//  i2c access function
//============================================================================//
static int adp1650_read_i2c(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
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
  if(!adp1650_drv->client)
    return -ENODEV;
  ret = i2c_transfer(adp1650_drv->client->adapter, msgs, 2);
  if(ret == 2)
  {
    if(adp1650_drv->i2c_err)
      MSG2("%s, status = 2, i2c_err = 0",__func__);
    adp1650_drv->i2c_err = 0;
  }
  else
  {
    adp1650_drv->i2c_err ++;
    if(adp1650_drv->i2c_err < 20)
      MSG2("%s, ret = %d, i2c_err = %d",__func__,ret,adp1650_drv->i2c_err);
  }
  return ret;
}
static int adp1650_read_i2c_retry(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len)
{
  int i,ret;
  for(i=0; i<I2C_RETRY_MAX; i++)
  {
    ret = adp1650_read_i2c(addr,reg,buf,len);
    if(ret == 2)
      return ret;
    else
      msleep(10);
  }
  return ret;
}
static int adp1650_write_i2c(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  int i;
  unsigned char buf_w[64];
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
    return -ENOMEM;
  if(!adp1650_drv->client)
    return -ENODEV;
  buf_w[0] = reg;
  for(i=0; i<len; i++)
    buf_w[i+1] = buf[i];
  ret = i2c_transfer(adp1650_drv->client->adapter, msgs, 1);

  if(ret == 1)
  {
    if(adp1650_drv->i2c_err)
      MSG2("%s, status = 2, i2c_err = 0",__func__);
    adp1650_drv->i2c_err = 0;
  }
  else
  {
    adp1650_drv->i2c_err ++;
    if(adp1650_drv->i2c_err < 20)
      MSG2("%s, ret = %d, i2c_err = %d",__func__,ret,adp1650_drv->i2c_err);
  }
  return ret;
}
static int adp1650_write_i2c_retry(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len)
{
  int i,ret;
  for(i=0; i<I2C_RETRY_MAX; i++)
  {
    ret = adp1650_write_i2c(addr,reg,buf,len);
    if(ret == 1)
      return ret;
    else
      msleep(10);
  }
  return ret;
}

static void adp1650_i2c_bus_onOff(int onOff)
{
  
  if(!adp1650_drv->vio) //no regulator found
  {
    MSG2("%s, vio get FAIL",__func__);
    return;
  }
  if(onOff && adp1650_drv->i2c_bus_enabled)   //already on?
    return;
  if(!onOff && !adp1650_drv->i2c_bus_enabled) //already off?
    return;

  if(onOff) //on
  {
    if(regulator_enable(adp1650_drv->vio))
      MSG2("%s, vio enable FAIL",__func__);

       // sophia wang ++, 20130703
       // dynamic support boston(bus4) and casper hw (bus5)
       // 1. we don't need to apply previous wordaround of FTD for casper hw since the power of flash is not related to camera
       // 2. gpio 20, 21 is just for bus 4
       if( (adp1650_drv->in_the_same_i2c_bus_with_camera) == true) 
       {
     		// Sophia wang ++, 20130111, lvs1 is to supply i2c bus and 8m camera digital power, for ftd flash test, we need to enable i2c bus power
     		// however, for some camera module, if they just have digital power without analog power, this camera will act abnormal and affect the flash test
     		adp1650_drv->gpio_8m_cam_analog_request = gpio_request(35, "8m_analoge_ftd_flash_test");
     		if( adp1650_drv->gpio_8m_cam_analog_request)
	  	MSG2("%s, 35 gpio_request, FAIL = %d", __func__, adp1650_drv->gpio_8m_cam_analog_request);

      		adp1650_drv->gpio_8m_cam_analog_request = gpio_direction_output(35, 1);
     		if( adp1650_drv->gpio_8m_cam_analog_request)
	  	MSG2("%s, 35 gpio_direction_output 0, FAIL = %d", __func__,  adp1650_drv->gpio_8m_cam_analog_request);
     		// Sophia wang --, 20130111, lvs1 is to supply i2c bus and 8m camera digital power, for ftd flash test, we need to enable i2c bus power
     		// however, for some camera module, if they just have digital power without analog power, this camera will act abnormal and affect the flash test
     
    		adp1650_drv->i2c_gpio20_requested = gpio_request(20,"adp1650");
    		adp1650_drv->i2c_gpio21_requested = gpio_request(21,"adp1650");
       }
       //sophia wang--, 20130703
       
    adp1650_drv->i2c_bus_enabled = 1;
  }
  else      //off
  {

       // sophia wang ++, 20130703
       // dynamic support boston(bus4) and casper hw (bus5)
       // 1. we don't need to apply previous wordaround of FTD for casper hw since the power of flash is not related to camera
       // 2. gpio 20, 21 is just for bus 4
     if( (adp1650_drv->in_the_same_i2c_bus_with_camera) == true) 
     {
    		if(!adp1650_drv->i2c_gpio20_requested)
    		{
      		gpio_free(20);
      		adp1650_drv->i2c_gpio20_requested = -1;
    		}
    		if(!adp1650_drv->i2c_gpio21_requested)
    		{
      		gpio_free(21);
      		adp1650_drv->i2c_gpio21_requested = -1;
    		}
     	

    		// Sophia wang ++, 20130111, lvs1 is to supply i2c bus and 8m camera digital power, for ftd flash test, we need to enable i2c bus power
    		// however, for some camera module, if they just have digital power without analog power, this camera will act abnormal and affect the flash test
    		if( !adp1650_drv->gpio_8m_cam_analog_request)
    		{
       		gpio_direction_output(35, 0);
    			gpio_free(35);
    			adp1650_drv->gpio_8m_cam_analog_request = -1;
    		}
    		// Sophia Wang --, 20130111
     }
     
    if(regulator_disable(adp1650_drv->vio))
      MSG2("%s, vio disable FAIL",__func__);
    adp1650_drv->i2c_bus_enabled = 0;
  }
}

//============================================================================//
//  flash control
//============================================================================//
int adp1650_flash_control(struct msm_camera_sensor_flash_external *fdata, unsigned led_state)
{
	int ret = 0;
	char r03 = 0x00, r05 = 0x00, r02 = 0x00;
	static char r04 = 0x00;
    #ifdef PERF
    struct timespec temp;
    #endif

     #ifdef DEBUG
    int i = 0;
    char r00_09[10];
     #endif
     
  MSG2("%s, %s (%d)",__func__,
    led_state == MSM_CAMERA_LED_OFF     ? "OFF" :
    led_state == MSM_CAMERA_LED_LOW     ? "LOW" :
    led_state == MSM_CAMERA_LED_HIGH    ? "HIGH" :
    led_state == MSM_CAMERA_LED_INIT    ? "INIT" :
    led_state == MSM_CAMERA_LED_RELEASE ? "RELEASE" : "ERROR!!!",
    led_state);

  switch(led_state)
  {
    case MSM_CAMERA_LED_INIT:
   #if 1	
       //adp1650_i2c_bus_onOff(1);

   #ifdef STROBE
      gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 0);
      udelay(10); //todo
   #endif  

      // sophia wang , 20130703
      // For casper project, the power of flash is not the same as cam_vio of sensor
      // we need to enable the regulator in flash driver 
      if( adp1650_drv->in_the_same_i2c_bus_with_camera == false)
      {
           if(regulator_enable(adp1650_drv->vio))
           MSG2("%s, vio enable FAIL",__func__);
      	}
      
      gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 1);
     // msleep(10); //todo


#if 0
      	//Flash = 300mA, Torch = 25mA (set current before choose mode)
      	// r03 = 0x00;

     	//Flash = 1000mA, Torch = 200mA (set current before choose mode);
     	if(flash_current == 0xE)
     	{
         	 r03 = 0x73; // default assist mode is 100mA
     	}
     	else
     	{
           	r03 = ((flash_current <<3)|0x7);
      	}
      	ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x03,&r03,sizeof(r03));

      	if(ret < 0)
      	{
		MSG2("%s, INIT: write 0x%02X to r03 failed",__func__,r03);
		return ret;
       }

      	//set flash duration
      	r02 = 0x02; //300ms
      	ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x02,&r02,sizeof(r02));
      	if(ret < 0)
      	{

		MSG2("%s, INIT: write 0x%02X to r02 failed",__func__,r02);
		return ret;
      	}

    
      	//default + o/p EN + HW + flash mode
      	r04 = 0xA4;
      	ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x04,&r04,sizeof(r04));
      	if(ret < 0)
      	{

		MSG2("%s, INIT: write 0x%02X to r04 failed",__func__,r04);
		return ret;
      	}
      	#endif
#endif

       printk("%s, Sophia, init for debug\n ", __func__);
      	break;

    case MSM_CAMERA_LED_RELEASE:
      //read fault info
      adp1650_read_i2c_retry(adp1650_drv->i2c_addr,0x05,&r05,sizeof(r05));
      if(r05)   MSG2("%s, RELE: fault = 0x%02X",__func__,r05);

      //clear r04
      r04 = 0x00;
      #ifdef STROBE
      gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 0);
      #endif
      gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 0);


       // sophia wang , 20130703
      // For casper project, the power of flash is no the same as cam_vio of sensor
      // we need to set the regulator in flash driver 
      if( adp1650_drv->in_the_same_i2c_bus_with_camera == false)
      	{

           if(regulator_disable(adp1650_drv->vio))
           MSG2("%s, vio enable FAIL",__func__);
      }
      
      //adp1650_i2c_bus_onOff(0);
      break;

    case MSM_CAMERA_LED_OFF:
      //read fault info
      adp1650_read_i2c_retry(adp1650_drv->i2c_addr,0x05,&r05,sizeof(r05));
      if(r05)   MSG2("%s, OFF:  fault = 0x%02X",__func__,r05);


      if(preMode == MSM_CAMERA_LED_HIGH)
      	{
      	     printk("%s, delay 60ms\n", __func__);
            mdelay(60);
      }

      #ifdef STROBE
      gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 0);
      #endif

      //default (must set to standby mode first, or strobe sometimes fail)
      r04 = 0xA4;
      ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x04,&r04,sizeof(r04));
      if (ret < 0)
      {
		MSG2("%s, LED_OFF: write 0x%02X to r04 failed",__func__,r04);
		return ret;
      }

#ifdef CONFIG_PM_LOG
      ret = pmlog_device_off(adp1650_drv->pmlog_device);
      if(ret <  0)
      	{
           MSG2("%s, pmlog device off fail ret:%d\n", __func__, ret);
      }
#endif

	#if 0
      //default + o/p EN + HW + flash mode
      r04 = 0xAF;
      ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x04,&r04,sizeof(r04));
	if (ret < 0)
      {
		MSG2("%s, LED_OFF: write 0x%02X to r04 failed",__func__,r04);
		return ret;
      }
	#endif

    	#ifdef PERF
    	if( preMode == MSM_CAMERA_LED_LOW)
    	{
    		temp = timespec_sub(CURRENT_TIME, start);
    		MSG2("%s: the pre flash duration is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    	}
    	else if( preMode == MSM_CAMERA_LED_HIGH)
    	{
    		temp = timespec_sub(CURRENT_TIME, start);
    		MSG2("%s: the flash duration is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    	}

    	start = CURRENT_TIME;



    	
    	#endif
    
      	break;

    case MSM_CAMERA_LED_LOW:
      //deafult + o/p en + hw + assist mode
      r04 = 0xAE;
      ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x04,&r04,sizeof(r04));
      if (ret < 0)
      {
		MSG2("%s, LED_LOW: write 0x%02X to r04 failed",__func__,r04);
		return ret;
      }

    	//Flash = 1000mA, Torch = 200mA (set current before choose mode);
     	if(flash_current == 0xE && assist_current == 0x3)
     	{
          r03 = 0x73;
     	}
     	else
     	{
           r03 = ((flash_current <<3)|assist_current);
      }

      MSG2("%s, LED_LOW: write 0x%02X to r03",__func__,r03);

      ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x03,&r03,sizeof(r03));

//Terry Cheng, 20121119, log flash led component using time
#ifdef CONFIG_PM_LOG
      ret = pmlog_device_on(adp1650_drv->pmlog_device);
      if(ret <  0)
      	{
           MSG2("%s, pmlog device on fail ret:%d\n", __func__, ret);
      }
#endif
     
      #ifdef PERF
      if( preMode == MSM_CAMERA_LED_OFF)
    	{
    		temp = timespec_sub(CURRENT_TIME, start);
    		MSG2("%s: the flash off to pre flash on is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    	}
    	start = CURRENT_TIME;
    	#endif
    
       //gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 1);
       break;

    case MSM_CAMERA_LED_HIGH:

      //set flash duration
      r02 = 0x0F & flash_duration; //300ms
      //MSG2("%s,  write 0x%02X to r02 ",__func__,r02);

      ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x02,&r02,sizeof(r02));
      if(ret < 0)
      {

		MSG2("%s, INIT: write 0x%02X to r02 failed",__func__,r02);
		return ret;
      }

    	//Flash = 1000mA, Torch = 100mA (set current before choose mode);
     	if(flash_current == 0xE)
     	{
          r03 = 0x73;
     	}
     	else
     	{
           r03 = ((flash_current <<3)|0x3);
      }

      MSG2("%s, LED_HIGH: write 0x%02X to r03",__func__,r03);

      ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x03,&r03,sizeof(r03));
     
      if(r04 != 0xAB) //check r04 is flash mode?
      {
        //default + o/p EN + HW + flash mode
        r04 = 0xAB;
       // r04 = 0xAF;
        ret = adp1650_write_i2c_retry(adp1650_drv->i2c_addr,0x04,&r04,sizeof(r04));
        if (ret < 0)
        {
		MSG2("%s, LED_HIGH: write 0x%02X to r04 failed",__func__,r04);
		return ret;
        }

      }

#ifdef CONFIG_PM_LOG
      ret = pmlog_device_on(adp1650_drv->pmlog_device);
      if(ret <  0)
      	{
           MSG2("%s, pmlog device on fail ret:%d\n", __func__, ret);
      }
#endif

     #ifdef DEBUG
     for(i=0; i<ARRAY_SIZE(r00_09); i++)
      adp1650_read_i2c_retry(adp1650_drv->i2c_addr,i,&r00_09[i],sizeof(r00_09[i]));
        MSG2("%s R00_09 = %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X",__func__,
          r00_09[0],r00_09[1],r00_09[2],r00_09[3],
          r00_09[4],r00_09[5],r00_09[6],r00_09[7],
          r00_09[8],r00_09[9]);

     #endif
    
    #ifdef PERF
    if( preMode == MSM_CAMERA_LED_OFF)
    {
    	temp = timespec_sub(CURRENT_TIME, start);
    	MSG2("%s: the pre flash off to flash on is :%ld ms\n",__func__, ((long)temp.tv_sec*1000)+((long)temp.tv_nsec/1000000));
    }
    start = CURRENT_TIME;
    #endif

    #ifdef STROBE
      gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 1);
    #endif
      break;
  }

//#ifdef PERF
   preMode = led_state;
//#endif
  //MSG2("%s-, ret=%d",__func__,ret);
  return ret;
 

}

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
static void adp1650_i2c_test(unsigned char *bufLocal, int count, struct i2c_client *client)
{
  struct i2c_msg msgs[2];
  int i2c_ret, i, j;
  char id, reg[2], len, dat[I2C_BUF_LENGTH/4];

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
//
int adp1650_FTM_test(void *parm, u64 mode)
{
    struct msm_camera_sensor_flash_external* data = NULL;
    int rc = 0, i = 0;
    char r00_09[10] = {0};

    MSG2("%s, mode:%d\n", __func__, (int)mode);
    switch (mode)
    {
        case 0:
	 rc = adp1650_flash_control(data, MSM_CAMERA_LED_OFF);
        adp1650_flash_control(data, MSM_CAMERA_LED_RELEASE);
        adp1650_i2c_bus_onOff(0);

        assist_current = 0x3; // // sophia, 20130111, hw request  for 25 mA for ftd test, but for os, it use 100mA for assist mode, resotre the ori value
        break;

        // assist light, 100mA
        case 1:
        adp1650_i2c_bus_onOff(1);
       // sophia++, 20130111, remove the below workaround
       // msleep(100); // sophia, 20121225, add to wait i2c stable
      // sophia--, 20130111, remove the below workaround
       rc = adp1650_flash_control(data, MSM_CAMERA_LED_INIT);

       msleep(10); //10ms is removed in MSM_CAMERA_LED_INIT
     	// sophia++, 20130111, remove the below workaround, this issue seems related to unstable camera module
	// msleep(1000); //sophia, 201225, add to wait led enable to avoid some led will turn off right away even in assist mode
	// sophia++, 20130111, remove the below workaround, this issue seems related to unstable camera module

       assist_current = 0x0; // sophia, 20130111, hw request, for boston flash ftd test, just use 25mA for assist mode 
     	
	rc = adp1650_flash_control(data, MSM_CAMERA_LED_LOW);

        adp1650_read_i2c_retry(adp1650_drv->i2c_addr,0x00,&r00_09[0],sizeof(r00_09[0]));
    	for(i=1; i<ARRAY_SIZE(r00_09); i++)
    	adp1650_read_i2c_retry(adp1650_drv->i2c_addr,i,&r00_09[i],sizeof(r00_09[i]));
   	 MSG2("%s R00_09 = %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X",__func__,
      	r00_09[0],r00_09[1],r00_09[2],r00_09[3],
      	r00_09[4],r00_09[5],r00_09[6],r00_09[7],
      	r00_09[8],r00_09[9]);
      
        break;

        default:
        printk("invalid command\n");
        rc = -1;

    }

     return rc;
}

static int  read_adp1650_device_id_s(void *data, u64 *val)
{
   *val =(int)adp1650_chipID;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(adp1650_flash_enable_fops, NULL , adp1650_FTM_test, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(adp1650_flash_read_deviceID,  read_adp1650_device_id_s, NULL, "%lld\n");
static void adp1650_create_debugfs_entries(void)
{
	MSG2("%s++\n", __func__);

	dent = debugfs_create_dir("flash_adp1650", NULL);
	if (dent) {
		debugfs_create_file("mode", S_IRUGO | S_IWUGO, dent, NULL, &adp1650_flash_enable_fops);
		debugfs_create_file("deviceID", 0444, dent, NULL, &adp1650_flash_read_deviceID);
	}else{
              printk("%s, failed\n", __func__);
	}
	
}

//============================================================================//
static ssize_t adp1650_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  return 0;
}
static ssize_t adp1650_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned char bufLocal[I2C_BUF_LENGTH];
  //int ret;

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
        adp1650_log_on1 = 0;
        adp1650_log_on2 = 1;
        adp1650_log_on3 = 0;
      }
      else if(bufLocal[1]=='1')
      {
        MSG2("Dynamic Log 1 On");
        adp1650_log_on1 = 1;
      }
      else if(bufLocal[1]=='3')
      {
        MSG2("Dynamic Log 3 On");
        adp1650_log_on3 = 1;
      }
      break;*/

    case 'y':
      {
        int i;
        char r00_09[10];
        for(i=0; i<ARRAY_SIZE(r00_09); i++)
          adp1650_read_i2c_retry(adp1650_drv->i2c_addr,i,&r00_09[i],sizeof(r00_09[i]));
        MSG2("%s R00_09 = %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X",__func__,
          r00_09[0],r00_09[1],r00_09[2],r00_09[3],
          r00_09[4],r00_09[5],r00_09[6],r00_09[7],
          r00_09[8],r00_09[9]);
      }
      break;

    //=================
    //  pin test
    case 'p':
      if(bufLocal[1]=='e' && bufLocal[2]=='0')  gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 0);
      else
      if(bufLocal[1]=='e' && bufLocal[2]=='1')  gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 1);
 #ifdef STROBE     
      else	
      if(bufLocal[1]=='s' && bufLocal[2]=='0')  gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 0);
     
      else
      if(bufLocal[1]=='s' && bufLocal[2]=='1')  gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 1);
 #endif
 #if 0
      else
      if(bufLocal[1]=='1' && bufLocal[2]=='0')  gpio_set_value_cansleep(adp1650_drv->gpio_gpio1, 0);
      else
      if(bufLocal[1]=='1' && bufLocal[2]=='1')  gpio_set_value_cansleep(adp1650_drv->gpio_gpio1, 1);
      else
      if(bufLocal[1]=='2' && bufLocal[2]=='0')  gpio_set_value_cansleep(adp1650_drv->gpio_gpio2, 0);
      else
      if(bufLocal[1]=='2' && bufLocal[2]=='1')  gpio_set_value_cansleep(adp1650_drv->gpio_gpio2, 1);
 #endif     
      else
      {
        int en,st,g1,g2;
        en = gpio_get_value(adp1650_drv->gpio_led_en);
  #ifdef STROBE      
        st = gpio_get_value(adp1650_drv->gpio_strobe);
 #endif 
        st = -1;
        g1 = gpio_get_value(adp1650_drv->gpio_gpio1);
        g2 = gpio_get_value(adp1650_drv->gpio_gpio2);
        MSG2("en=%d, strobe=%d, g1=%d, g2=%d",en,st,g1,g2);
      }
      break;

    //=================
    //  enable/disable i2c bus
    case 'r':
      if(bufLocal[1]=='1')
      {
        adp1650_i2c_bus_onOff(1);
      }
      else if(bufLocal[1]=='0')
      {
        adp1650_i2c_bus_onOff(0);
      }
      else
      {
        MSG2("i2c_bus_enabled = %d, i2c_gpio20_requested = %d, i2c_gpio21_requested = %d",
          adp1650_drv->i2c_bus_enabled, adp1650_drv->i2c_gpio20_requested, adp1650_drv->i2c_gpio21_requested);
      }
      break;

    //=================
    //  i2c test
    case 'i':
      adp1650_i2c_test(bufLocal, count, adp1650_drv->client);
      break;

  }
  return count;
}
static struct device_attribute adp1650_ctrl_attrs[] = {
  __ATTR(ctrl, 0664, adp1650_ctrl_show, adp1650_ctrl_store),
};

/*
#ifdef CONFIG_PM
static int adp1650_i2c_suspend(struct device *dev)
{
  MSG("%s", __func__);
  adp1650_drv->suspend_flag = 1;

  return 0;
}
static int adp1650_i2c_resume(struct device *dev)
{
  MSG("%s", __func__);
  adp1650_drv->suspend_flag = 0;
  return 0;
}
static const struct dev_pm_ops adp1650_pm_ops = {
	.suspend  = adp1650_i2c_suspend,
	.resume   = adp1650_i2c_resume,
};
#endif
*/

static void adp1650_i2c_shutdown(struct i2c_client *client)
{
  MSG2("%s",__func__);
  #ifdef STROBE
  gpio_set_value_cansleep(adp1650_drv->gpio_strobe, 0);
  #endif
  
  gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 0);
}
static int adp1650_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int ret, i;
  char r00_09[10];

  printk("%s+, client=%s, id=%s, adapter=%s", __func__,
    client->name, id->name, client->adapter->name);

  //============================================================================//
  //  init
  //============================================================================//
  adp1650_drv = kzalloc(sizeof(struct adp1650_drv_data), GFP_KERNEL);
  if(adp1650_drv == NULL)
  {
    MSG2("%s, kmalloc fail!",__func__);
    ret = -ENOMEM;
    goto exit_err;
  }
  adp1650_drv->i2c_addr    = 0x30;
  adp1650_drv->gpio_led_en = 2;
  #ifdef STROBE
  adp1650_drv->gpio_strobe = 3;
  #endif
  #if 0
  adp1650_drv->gpio_gpio1  = 42;
  adp1650_drv->gpio_gpio2  = 43;
  #endif
	adp1650_drv->i2c_gpio20_requested = -1;
	adp1650_drv->i2c_gpio21_requested = -1;

  //============================================================================//
  //  gpio config
  //============================================================================//
  //gpio_led_en
	ret = gpio_request(adp1650_drv->gpio_led_en, "adp1650_led_en");
	if(ret)
	  MSG2("%s, %d gpio_request, FAIL = %d", __func__, adp1650_drv->gpio_led_en, ret);
	ret = gpio_direction_output(adp1650_drv->gpio_led_en, 0);
	if(ret)
	  MSG2("%s, %d gpio_direction_output 0, FAIL = %d", __func__, adp1650_drv->gpio_led_en, ret);
#ifdef STROBE	
  //gpio_strobe
	ret = gpio_request(adp1650_drv->gpio_strobe, "adp1650_strobe");
	if(ret)
	  MSG2("%s, %d gpio_request, FAIL = %d", __func__, adp1650_drv->gpio_strobe, ret);
	ret = gpio_direction_output(adp1650_drv->gpio_strobe, 0);
	if(ret)
	  MSG2("%s, %d gpio_direction_output 0, FAIL = %d", __func__, adp1650_drv->gpio_strobe, ret);
#endif
#if 0
  //gpio_gpio1
	ret = gpio_request(adp1650_drv->gpio_gpio1, "adp1650_gpio1");
	if(ret)
	  MSG2("%s, %d gpio_request, FAIL = %d", __func__, adp1650_drv->gpio_gpio1, ret);
	ret = gpio_direction_output(adp1650_drv->gpio_gpio1, 0);
	if(ret)
	  MSG2("%s, %d gpio_direction_output 0, FAIL = %d", __func__, adp1650_drv->gpio_gpio1, ret);
  //gpio_gpio2
	ret = gpio_request(adp1650_drv->gpio_gpio2, "adp1650_gpio2");
	if(ret)
	  MSG2("%s, %d gpio_request, FAIL = %d", __func__, adp1650_drv->gpio_gpio2, ret);
	ret = gpio_direction_output(adp1650_drv->gpio_gpio2, 0);
	if(ret)
	  MSG2("%s, %d gpio_direction_output 0, FAIL = %d", __func__, adp1650_drv->gpio_gpio2, ret);
#endif
  //============================================================================//
  //  Regulator
  //============================================================================//
  //MSG2("%s, client->dev->init_name = %s",__func__,client->dev.init_name);
  //adp1650_drv->vio = regulator_get(&client->dev, "cam_vio");

 // sophia wang, 20130711
 // sappora don't need to consider dynamic proble 1650 in different i2c issue
 #if 0
  if( msm_project_id == BOSTON && system_rev < CASPER_EVT1) 
  {
  	adp1650_drv->vio = regulator_get(&client->dev, "cam_vio_adp1650");
  	adp1650_drv->in_the_same_i2c_bus_with_camera = true;
  }
  else
  #endif
  {
  	adp1650_drv->vio = regulator_get(&client->dev, "cam_vio_adp1650_l11");// casper, adp1650 will in bus9
  	adp1650_drv->in_the_same_i2c_bus_with_camera = false;
  }
  if(IS_ERR(adp1650_drv->vio))
  {
	  MSG2("%s, regulator_get cam_vio, FAIL = %d", __func__, (int)adp1650_drv->vio);
	  adp1650_drv->vio = NULL;
	}

  adp1650_drv->client = client;

  //============================================================================//
  //  Read Chip Id
  //============================================================================//
  adp1650_i2c_bus_onOff(1);
  gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 1);
  msleep(10); //todo
  ret = adp1650_read_i2c_retry(adp1650_drv->i2c_addr,0x00,&r00_09[0],sizeof(r00_09[0]));
  if(ret == 2)
  {
    adp1650_drv->chipid = r00_09[0];
    adp1650_chipID = r00_09[0];
    
    for(i=1; i<ARRAY_SIZE(r00_09); i++)
    adp1650_read_i2c_retry(adp1650_drv->i2c_addr,i,&r00_09[i],sizeof(r00_09[i]));
    MSG2("%s R00_09 = %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X",__func__,
      r00_09[0],r00_09[1],r00_09[2],r00_09[3],
      r00_09[4],r00_09[5],r00_09[6],r00_09[7],
      r00_09[8],r00_09[9]);
  }
  else
  {
    MSG2("%s Chip read fail, ret = %d",__func__,ret);
  }
  gpio_set_value_cansleep(adp1650_drv->gpio_led_en, 0);
  adp1650_i2c_bus_onOff(0);

  //================//
  //  driver function
  //================//
  for(i=0; i<ARRAY_SIZE(adp1650_ctrl_attrs); i++)
  {
    ret = device_create_file(&client->dev, &adp1650_ctrl_attrs[i]);
    if(ret) MSG2("%s: create FAIL, ret=%d",adp1650_ctrl_attrs[i].attr.name,ret);
  }

#ifdef CONFIG_PM_LOG
   adp1650_drv->pmlog_device = pmlog_register_device(&client->dev);
#endif

  //adp1650_drv->inited = 1;

  MSG2("%s-, ret=0",__func__);
  return 0;

exit_err:
  MSG2("%s-, ret=%d",__func__,ret);
  return ret;
}

static const struct i2c_device_id adp1650_id[] = { { "adp1650", 0 } };
static struct i2c_driver adp1650_driver = {
  .driver.owner = THIS_MODULE,
  .driver.name  = "adp1650",
  #if 0
  #ifdef CONFIG_PM
    .driver.pm = &adp1650_pm_ops,
  #endif
  #endif
  .id_table = adp1650_id,
  .shutdown = adp1650_i2c_shutdown,
  .probe    = adp1650_i2c_probe,
};

static int __init adp1650_init(void)
{
  int ret;
  printk("BootLog, +%s\n", __func__);
  ret = i2c_add_driver(&adp1650_driver);
  adp1650_create_debugfs_entries();
  printk("BootLog, -%s, ret=%d\n", __func__,ret);
  return ret;
};

//Sophia wang++, 20120815 close these node, it is just for debugging
//module_param(flash_current, uint, 0644);
//module_param(assist_current, uint, 0644);
//module_param(flash_duration, uint, 0644);
//Sophia wang++, 20120815 close these node, it is just for debugging

module_init(adp1650_init);
MODULE_DESCRIPTION("ADP1650 Flash Light Driver");

