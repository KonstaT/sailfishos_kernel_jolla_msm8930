// 20121108
// Emily Jiang created
//

#ifndef _FOCALTECH_FT5316_TS_H_
#define _FOCALTECH_FT5316_TS_H_

#define FOCALTECH_TP_NAME	"ft5316_ts"
#define FOCALTECH_CAPKEY_NAME "ft5316_capkey"
#define TS_RST_GPIO_NUM 52
#define TS_IRQ_GPIO_NUM 11
#define TS_VENDOR_ID_GPIO_NUM	93
#define TS_I2C_ADDR 0x38
#define DISPLAY_PANEL_MAX_X    540
#define DISPLAY_PANEL_MAX_Y    960
#define MAX_TOUCH_MAJOR        128
#define MAX_TS_REPORT_POINTS  5
#define MAX_KEYS_SIZE 3
#define START_REG_MAP_ADDR	0x00 
#define READ_BUF_SIZE  (3 + 6 * (MAX_TS_REPORT_POINTS))
//============================================================================//
// define 3 keys coordinates
//============================================================================//
#define TOUCH_BACK_KEY_X	90
#define TOUCH_BACK_KEY_Y	1008
#define TOUCH_HOME_KEY_X	270
#define TOUCH_HOME_KEY_Y	1008
#define TOUCH_MENU_X	450
#define TOUCH_MENU_Y	1008
#define TOUCH_DEFAULE_VALUE FFF
//============================================================================//
// the power consumption mode
//============================================================================//
#define TS_ACTIVE_MODE	0       /* active mode */
#define TS_MONITOR_MODE	1      /* idle mode */
#define	TS_HIBERNATE_MODE	3 /* deep sleep mode */
#define FT_RESET_DLY		20
#define TOUCH_DEFAULE_VALUE FFF
//============================================================================//
// get raw data pass criteria
//============================================================================//
#define RAW_DATA_MIN_LIMIT 7000
#define RAW_DATA_MAX_LIMIT 11000
#define DIFFER_MIN_LIMIT 200
#define DIFFER_MAX_LIMIT 800
#define RX_SIZE 12
#define RX_RX_AVEG 500
#define RX_RX_MAX 500
#define RX_RX_MIN 500
	
struct focaltech_tp_platform_data_t {
    unsigned int gpio_irq;
    unsigned int gpio_rst;
	unsigned int gpio_vendor_id;
};

// enum registers map for FocalTech FT5316 touchscreen chip
enum ft5316_ts_regs {
	FT5316_REG_THGROUP = 0x80,	/* touch threshold, related to sensitivity */
	FT5316_REG_THPEAK,					
	FT5316_REG_THCAL,					
	FT5316_REG_THWATER,					
	FT5316_REG_THTEMP,					
	FT5316_REG_THDIFF,					
	FT5316_REG_CTRL, //power control mode,0:active,1:monitor(idle), hibernate(deep sleep) mode						
	FT5316_REG_TIMEENTERMONITOR,			
	FT5316_REG_PERIODACTIVE,	/* report rate */
	FT5316_REG_PERIODMONITOR,			
	FT5316_REG_HEIGHT_B	,				
	FT5316_REG_MAX_FRAME,				
	FT5316_REG_DIST_MOVE,				
	FT5316_REG_DIST_POINT,				
	FT5316_REG_FEG_FRAME,				
	FT5316_REG_SINGLE_CLICK_OFFSET,
	FT5316_REG_DOUBLE_CLICK_TIME_MIN,
	FT5316_REG_SINGLE_CLICK_TIME,
	FT5316_REG_LEFT_RIGHT_OFFSET,
	FT5316_REG_UP_DOWN_OFFSET,
	FT5316_REG_DISTANCE_LEFT_RIGHT,
	FT5316_REG_DISTANCE_UP_DOWN,
	FT5316_REG_ZOOM_DIS_SQR,
	FT5316_REG_RADIAN_VALUE,
	FT5316_REG_MAX_X_HIGH,
	FT5316_REG_MAX_X_LOW,
	FT5316_REG_MAX_Y_HIGH,
	FT5316_REG_MAX_Y_LOW,
	FT5316_REG_K_X_HIGH,
	FT5316_REG_K_X_LOW,
	FT5316_REG_K_Y_HIGH,
	FT5316_REG_K_Y_LOW,
	FT5316_REG_AUTO_CLB_MODE,
	FT5316_REG_LIB_VERSION_H,
	FT5316_REG_LIB_VERSION_L,
	FT5316_REG_CIPHER,	/* Chip vendor ID */
	FT5316_REG_MODE	,
	FT5316_REG_PMODE,	/* Power Consume Mode */
	FT5316_REG_FIRMID,	/* Firmware version */
	FT5316_REG_STATE,
	FT5316_REG_FT5201ID,	/* CTPM Vendor ID */
	FT5316_REG_ERR,
	FT5316_REG_CLB,
};

enum ft5316_ts_state_t {
	TS_PUT_DOWN = 0, /* press touch */
    TS_PUT_UP, /* release touch */
    TS_CONTACT, /* contact touch */
	TS_NO_EVENT, /* no touch event */
};

struct ft5316_point_t {
	uint	x;
    uint	y;
	int	z;
	uint    state;
	uint    pre_state;
	uint    touch_id;
};

//record each of ts point status
struct ft5316_touch_point_status_t 
{
    struct ft5316_point_t  coord;
    uint switched;
    struct ft5316_point_t  switched_coord; // check if the coord is touch or key area
};

struct ft5316_ts_event_t
{
	u16 x[MAX_TS_REPORT_POINTS];	//x coordinate
	u16 y[MAX_TS_REPORT_POINTS];	//y coordinate
	u8 z[MAX_TS_REPORT_POINTS];    //pressure
	u8 state[MAX_TS_REPORT_POINTS];	//touch event flag:  0 -- down; 1-- contact; 2 -- contact
	u8 finger_id[MAX_TS_REPORT_POINTS];	//touch ID
	u8 touchWeight[MAX_TS_REPORT_POINTS];	//touch Weight
	u8 touchArea[MAX_TS_REPORT_POINTS];	//touch Area
	u8 touch_point;
};

struct ft5316_key_point_t {
    uint    x;
    uint    y;
	uint    state;
};

//record each of key point status
struct ft5316_key_point_status_t 
{
    struct ft5316_key_point_t  key_coord;
};

struct ft5316_ts_rx_raw_data_t
{
	int aveg[RX_SIZE];	//average RX-RX raw data 
	int max[RX_SIZE];	//max RX-RX  raw data
	int min[RX_SIZE];	//min RX-RX raw data

};

#endif
