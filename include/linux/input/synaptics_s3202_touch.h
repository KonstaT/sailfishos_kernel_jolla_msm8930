// 20111213
// Emily Jiang created
//

#ifndef _SYANPTICS_S3202_TOUCH_H_
#define _SYNAPTICS_S3202_TOUCH_H_

#define SYNAPTICS_TP_NAME "touch_synaptics"
#define SYNAPTICS_CAPKEY_NAME "capkey_synaptics"

struct synaptics_tp_platform_data_t {
    unsigned int gpio_irq;
    unsigned int gpio_rst;
	unsigned int gpio_vendor_id;
};

#define MAX_TS_REPORT_POINTS  5
#define MAX_KEY_REPORT_POINTS  3
#define MAX_FINEGER_STATE      3
#define DISPLAY_PANEL_MAX_X    540
#define DISPLAY_PANEL_MAX_Y    960
#define PAGE_SELECT_REGISTER   0xFF
#define INTERRUPT_STATUS       0x14
#define FINGER_STATE        0x15
#define TOTAL_CONFIG_SIZE   245
#define MAX_X_POSITION      0x69
#define SYNAPTICS_PANEL_MAX_Y 1740
#define PDT_PROPERTIES_ADDR		0xEF
#define PDT_SIZE		  6
#define NUM_RMI_FUNCS	  4
#define PDT_FLASH_F34           0x34
#define PDT_RMI_F01		0x01
#define PDT_2D_DATA_F11		0x11
#define PDT_ANALOG_F54		0x54
#define PDT_LED_F31		0x31
#define PDT_0D_F1A		0x1A
#define NUM_OF_DS4_PAGES		0x03
#define NUM_OF_F34_FLASH_DATA_SIZE		19
#define NUM_OF_F34_FLASH_CTRL_SIZE		4
#define NUM_OF_F34_FLASH_QUERY_SIZE		9
#define NUM_OF_F01_RMI_DATA_SIZE		2
#define NUM_OF_F01_RMI_CTRL_SIZE		7
#define NUM_OF_F01_RMI_QUERY_SIZE		26
#define NUM_OF_F11_2D_DATA_SIZE		67 
#define NUM_OF_F11_2D_CTRL_SIZE		62
#define NUM_OF_F11_2D_QUERY_SIZE		9
#define FIRMWARE_IMAGE_START_ADDR		0x0100
#define TS_RST_GPIO_NUM 52
#define TS_IRQ_GPIO_NUM 11
#define TS_VENDOR_ID_GPIO_NUM	93
#define TP_LAMITATION_DETECTION_PIN 107
#define TS_I2C_ADDR_SYNA 0x20
#define KEY_RX_CHANNEL 3
//============================================================================//
// synaptics s3202 frimware version
//============================================================================//
#define SYNAP_DS4_3_0_FW_VER 1089762
#define SYNAP_DS4_3_2_1_FW_VER 1115991
#define SYNAP_DS4_3_2_2_FW_VER 1200567
#define SYNAP_DS4_3_6_0_33_FW_VER 1296077
#define SYNAP_DS4_3_6_0_34_FW_VER 1365481
#define SYNAP_DS4_3_6_0_35_FW_VER 1421304
//============================================================================//
// synaptics s3202 config id for TPK black TP
//============================================================================//
#define SYNAP_04_TPK_BLACK_CONFIG_ID 0x3034
#define SYNAP_05_TPK_BLACK_CONFIG_ID 0x3035
#define SYNAP_06_TPK_BLACK_CONFIG_ID 0x3036
//============================================================================//
// synaptics s3202 config id for TPK white TP
//============================================================================//
#define SYNAP_00_TPK_WHITETP_CONFIG_ID 0x3030
#define SYNAP_01_TPK_WHITETP_CONFIG_ID 0x3031
//============================================================================//
// synaptics s3202 define moduel TP
//============================================================================//
#define TPK_BLACK_TP 0x3030
#define TPK_WHITE_TP 0x3031
#define TPK_WHITE_TP_WITH_PET 0x3032
#define JTOUCH_BLACK_TP 0x3130
#define JTOUCH_WHITE_TP 0x3131
#define OFILM_BLACK_TP 0x4F42
#define DJ_BLACK_TP 0x444A
#define INVALID_TP 0xFFFF
//============================================================================//
// add debugfs for self test F54 command
//============================================================================//
 /* report type */
#define F54_RAW_CAPACITANCE 0x3
#define F54_HIGH_RESISTANCE 0x4
#define F54_TX_TX_SHORT 0x5
#define F54_RX_RX_SHORT_7 0x7
#define F54_RX_RX_REPORT_17 0x11
/* report size */
#define F54_HIGH_RESISTANCE_READ_BYTES 6
#define F54_TX_TX_SHORT_READ_BYTES 4
#define F54_RAW_CAPACITANCE_READ_BYTES 576
/* pass criteria */
#define F54_HIGH_RESISTANCE_RX_LIMIT 450
#define F54_HIGH_RESISTANCE_TX_LIMIT 450
#define F54_HIGH_RESISTANCE_MIN_LIMIT 0xFE70
#define F54_RAW_CAP_MAX_LIMIT_TPK_BLACK_TP 4000
#define F54_RAW_CAP_MIN_LIMIT_TPK_BLACK_TP 1600
#define F54_RAW_CAP_MAX_LIMIT_TPK_WHITE_TP_1115991 4000
#define F54_RAW_CAP_MIN_LIMIT_TPK_WHITE_TP_1115991 1600
#define F54_RAW_CAP_MAX_LIMIT_TPK_WHITE_TP_1200567 3520
#define F54_RAW_CAP_MIN_LIMIT_TPK_WHITE_TP_1200567 1330
#define F54_RAW_CAP_MAX_LIMIT_TPK_WHITE_TP_with_PET 4000
#define F54_RAW_CAP_MIN_LIMIT_TPK_WHITE_TP_with_PET 1480
#define F54_RAW_CAP_MAX_LIMIT_JT_BLACK_TP 4400
#define F54_RAW_CAP_MIN_LIMIT_JT_BLACK_TP 1000
#define F54_RAW_CAP_MAX_LIMIT_JT_WHITE_TP 4400
#define F54_RAW_CAP_MIN_LIMIT_JT_WHITE_TP 1000
#define F54_RAW_CAP_MAX_LIMIT_OB_BLACK_TP 2430
#define F54_RAW_CAP_MIN_LIMIT_OB_BLACK_TP 1020
#define F54_RAW_CAP_MAX_LIMIT_OB_NEW_BLACK_TP 2880
#define F54_RAW_CAP_MIN_LIMIT_OB_NEW_BLACK_TP 1200
#define F54_RAW_CAP_MAX_LIMIT_OB_Lamination_BLACK_TP 2750
#define F54_RAW_CAP_MIN_LIMIT_OB_Lamination_BLACK_TP 1320
#define F54_RAW_CAP_MAX_LIMIT_DJ_BLACK_TP 4000
#define F54_RAW_CAP_MIN_LIMIT_DJ_BLACK_TP 1000
#define F54_RAW_CAP_MAX_LIMIT_DJ_Lamination_BLACK_TP 4000
#define F54_RAW_CAP_MIN_LIMIT_DJ_Lamination_BLACK_TP 1000
#define F54_RX_RX_MAX_LIMIT 1100
#define F54_RX_RX_MIN_LIMIT 900
#define NUM_OF_RX_ELECTRODES 21
#define NUM_OF_TX_ELECTRODES  12
#define NUM_OF_RX_TX_ELECTRODES 12
#define RX_RX_7_SIZE (NUM_OF_RX_ELECTRODES * NUM_OF_TX_ELECTRODES)*2
#define RX_RX_17_SIZE (NUM_OF_RX_ELECTRODES * NUM_OF_TX_ELECTRODES)*2
#define SELF_TEST_ALL_PASS 0x0
/*F54 testing failed error code */
#define TX_RX_CHANNEL_ERR 1
#define RAW_CAPACITANCE 3
#define HIGH_RESISTANCE_ERR 4
#define TX_TX_SHORT_ERR 5
#define RX_RX_SHORT_ERR 24
#define FULL_RAW_CAPACITANCE_MAX_MIN 13


enum synaptics_ts_state_t {
	TS_RELEASE = 0,
    TS_PRESS,
    TS_MOVE
};

enum synaptics_key_state_t {
	KEY_RELEASE = 0,
    KEY_PRESS,
};

struct synaptics_point_t {
	uint	x;
    uint    y;
	uint8_t wxy;
	int	z;
};

struct synaptics_key_point_t {
    uint    x;
    uint    y;
};

//record each of ts point status
struct synaptics_touch_point_status_t 
{
    struct synaptics_point_t  coord;
    enum synaptics_ts_state_t state;
    enum synaptics_ts_state_t prev_state;
};

//record each of key point status
struct synaptics_cap_key_point_status_t 
{
    struct synaptics_key_point_t  key_coord;
    enum synaptics_key_state_t key_state;
};

/* Synaptcis F11 2D DATA Info */
struct F11_2D_data_t {
	u8 finger_state[MAX_FINEGER_STATE];
	u16 x[MAX_TS_REPORT_POINTS];
	u16 y[MAX_TS_REPORT_POINTS];
	u8 z[MAX_TS_REPORT_POINTS];
	u8 wxy[MAX_TS_REPORT_POINTS];
};

//define config version for J-Touch
enum syn_config_id_jtp {
	SYNAP_01_JTP_CONFIG_ID = 0x3031,
	SYNAP_02_JTP_CONFIG_ID,
	SYNAP_03_JTP_CONFIG_ID,
	SYNAP_04_JTP_CONFIG_ID,
	SYNAP_05_JTP_CONFIG_ID,
	SYNAP_06_JTP_CONFIG_ID = 0x3136,
};

//define config version for O-film
enum syn_config_id_ofilm {
	SYNAP_01_OF_CONFIG_ID = 0x3130,
	SYNAP_02_OF_CONFIG_ID = 0x3230,
	SYNAP_03_OF_CONFIG_ID = 0x3330,
	SYNAP_04_OF_CONFIG_ID = 0x3430,
	SYNAP_05_OF_CONFIG_ID = 0x3530,
	SYNAP_06_OF_CONFIG_ID = 0x3630,
};

//define config version for DJ
enum syn_config_id_dj {
	SYNAP_01_DJ_CONFIG_ID = 0x3130,
	SYNAP_02_DJ_CONFIG_ID = 0x3230,
	SYNAP_03_DJ_CONFIG_ID = 0x3330,
	SYNAP_04_DJ_CONFIG_ID = 0x3430,
	SYNAP_05_DJ_CONFIG_ID = 0x3530,
	SYNAP_06_DJ_CONFIG_ID = 0x3630,
};

//============================================================================//
// SYNAPTICS_RMI_DS4_3_0_2
// SYNAPTICS S3202 FW Ver:1089762
// Register Map
//============================================================================//
enum synaptics_f01_rmi_query_t {
	f01_rmi_query00 = 0x00,
    f01_rmi_query01,
    f01_rmi_query02,
	f01_rmi_query03,
    f01_rmi_query04,
	f01_rmi_query05,
    f01_rmi_query06,
	f01_rmi_query07,
    f01_rmi_query08,
	f01_rmi_query09,
    f01_rmi_query10,
	f01_rmi_query11,
    f01_rmi_query12,
	f01_rmi_query13,
    f01_rmi_query14,
	f01_rmi_query15,
    f01_rmi_query16,
	f01_rmi_query17,
    f01_rmi_query18,
	f01_rmi_query19,
    f01_rmi_query20,
	f01_rmi_query42,
    f01_rmi_query43_00,
	f01_rmi_query43_01,
    f01_rmi_query43_02,
	f01_rmi_query44,
};

enum synaptics_f11_2d_data_t {
	f11_rmi_data00_00 = 0x00,
    f11_rmi_data00_01,
    f11_rmi_data00_02,
	f11_rmi_data01_00,
    f11_rmi_data02_00,
	f11_rmi_data03_00,
    f11_rmi_data04_00,
	f11_rmi_data05_00,
    f11_rmi_data01_01,
	f11_rmi_data02_01,
    f11_rmi_data03_01,
	f11_rmi_data04_01,
    f11_rmi_data05_01,
	f11_rmi_data01_02,
    f11_rmi_data02_02,
	f11_rmi_data03_02,
    f11_rmi_data04_02,
	f11_rmi_data05_02,
    f11_rmi_data01_03,
	f11_rmi_data02_03,
    f11_rmi_data03_03,
	f11_rmi_data04_03,
    f11_rmi_data05_03,
	f11_rmi_data01_04,
    f11_rmi_data02_04,
	f11_rmi_data03_04,
	f11_rmi_data04_04,
	f11_rmi_data05_04,
	f11_rmi_data01_05,
    f11_rmi_data02_05,
	f11_rmi_data03_05,
	f11_rmi_data04_05,
	f11_rmi_data05_05,
	f11_rmi_data01_06,
    f11_rmi_data02_06,
	f11_rmi_data03_06,
	f11_rmi_data04_06,
	f11_rmi_data05_06,
	f11_rmi_data01_07,
    f11_rmi_data02_07,
	f11_rmi_data03_07,
	f11_rmi_data04_07,
	f11_rmi_data05_07,
	f11_rmi_data01_08,
    f11_rmi_data02_08,
	f11_rmi_data03_08,
	f11_rmi_data04_08,
	f11_rmi_data05_08,
	f11_rmi_data01_09,
    f11_rmi_data02_09,
	f11_rmi_data03_09,
	f11_rmi_data04_09,
	f11_rmi_data05_09,
	f11_rmi_data28,
	f11_rmi_data29,
    f11_rmi_data30,
	f11_rmi_data31,
	f11_rmi_data35_00,
	f11_rmi_data35_01,
	f11_rmi_data35_02,
	f11_rmi_data35_03,
	f11_rmi_data35_04,
	f11_rmi_data35_05,
	f11_rmi_data35_06,
	f11_rmi_data35_07,
	f11_rmi_data35_08,
	f11_rmi_data35_09,
};

enum synaptics_f11_2d_ctrl_t {
	f11_2d_ctrl00 = 0x00,
    f11_2d_ctrl01,
    f11_2d_ctrl02,
	f11_2d_ctrl03,
    f11_2d_ctrl04,
	f11_2d_ctrl05,
    f11_2d_ctrl06,
	f11_2d_ctrl07,
    f11_2d_ctrl08,
	f11_2d_ctrl09,
    f11_2d_ctrl20,
	f11_2d_ctrl21,
    f11_2d_ctrl29,
	f11_2d_ctrl30,
    f11_2d_ctrl31,
	f11_2d_ctrl32_00,
    f11_2d_ctrl32_01,
	f11_2d_ctrl33_00,
    f11_2d_ctrl33_01,
	f11_2d_ctrl34,
    f11_2d_ctrl35,
	f11_2d_ctrl36,
    f11_2d_ctrl37,
	f11_2d_ctrl38,
    f11_2d_ctrl39,
	f11_2d_ctrl40_00,
	f11_2d_ctrl40_01,
	f11_2d_ctrl41_00,
	f11_2d_ctrl41_01,
    f11_2d_ctrl42_00,
	f11_2d_ctrl42_01,
	f11_2d_ctrl43_00,
	f11_2d_ctrl43_01,
	f11_2d_ctrl44,
    f11_2d_ctrl45,
	f11_2d_ctrl46,
	f11_2d_ctrl47,
	f11_2d_ctrl48,
	f11_2d_ctrl49,
    f11_2d_ctrl50,
	f11_2d_ctrl51,
	f11_2d_ctrl58,
	f11_2d_ctrl68_00,
	f11_2d_ctrl68_01,
    f11_2d_ctrl68_02,
	f11_2d_ctrl68_03,
	f11_2d_ctrl68_04,
	f11_2d_ctrl68_05,
	f11_2d_ctrl69,
    f11_2d_ctrl70,
	f11_2d_ctrl71,
	f11_2d_ctrl72,
	f11_2d_ctrl73,
	f11_2d_ctrl74,
    f11_2d_ctrl75_00,
	f11_2d_ctrl75_01,
	f11_2d_ctrl75_02,
	f11_2d_ctrl76_00,
	f11_2d_ctrl76_01,
	f11_2d_ctrl76_02,
	f11_2d_ctrl77,
	f11_2d_ctrl78,
};

enum synaptics_f34_flash_data_t {
	f34_flash_data00 = 0x00,
    f34_flash_data01,
    f34_flash_data02_00,
	f34_flash_data02_01,
    f34_flash_data02_02,
	f34_flash_data02_03,
    f34_flash_data02_04,
	f34_flash_data02_05,
    f34_flash_data02_06,
	f34_flash_data02_07,
    f34_flash_data02_08,
	f34_flash_data02_09,
    f34_flash_data02_10,
	f34_flash_data02_11,
    f34_flash_data02_12,
	f34_flash_data02_13,
    f34_flash_data02_14,
	f34_flash_data02_15,
    f34_flash_data03,
};	

enum synaptics_f34_flash_query_t {
	f34_flash_query00 = 0x00,
    f34_flash_query01,
    f34_flash_query02,
	f34_flash_query03,
    f34_flash_query04,
	f34_flash_query05,
    f34_flash_query06,
	f34_flash_query07,
    f34_flash_query08,
};

enum synaptics_f54_flash_query_t {
	f54_analog_ctrl00 = 0x00,
	f54_analog_ctrl01,   
	f54_analog_ctrl02_00,
	f54_analog_ctrl02_01,
	f54_analog_ctrl03,   
	f54_analog_ctrl04,   
	f54_analog_ctrl05,   
	f54_analog_ctrl06,   
	f54_analog_ctrl07,   
	f54_analog_ctrl08_00,
	f54_analog_ctrl08_01,
	f54_analog_ctrl09,   
	f54_analog_ctrl10,   
	f54_analog_ctrl11_00,
	f54_analog_ctrl11_01,
	f54_analog_ctrl12,   
	f54_analog_ctrl13,   
	f54_analog_ctrl14,   
	f54_analog_ctrl15_00,
	f54_analog_ctrl15_01,
	f54_analog_ctrl15_02,
	f54_analog_ctrl15_03,
	f54_analog_ctrl15_04,
	f54_analog_ctrl15_05,
	f54_analog_ctrl15_06,
	f54_analog_ctrl15_07,
	f54_analog_ctrl15_08,
	f54_analog_ctrl15_09,
	f54_analog_ctrl15_10,
	f54_analog_ctrl15_11,
	f54_analog_ctrl15_12,
	f54_analog_ctrl15_13,
	f54_analog_ctrl15_14,
	f54_analog_ctrl15_15,
	f54_analog_ctrl15_16,
	f54_analog_ctrl15_17,
	f54_analog_ctrl15_18,
	f54_analog_ctrl15_19,
	f54_analog_ctrl15_20,
	f54_analog_ctrl15_21,
	f54_analog_ctrl15_22,
	f54_analog_ctrl15_23,
	f54_analog_ctrl15_24,
	f54_analog_ctrl15_25,
	f54_analog_ctrl15_26,
	f54_analog_ctrl16_00,
	f54_analog_ctrl16_01,
	f54_analog_ctrl16_02,
	f54_analog_ctrl16_03,
	f54_analog_ctrl16_04,
	f54_analog_ctrl16_05,
	f54_analog_ctrl16_06,
	f54_analog_ctrl16_07,
	f54_analog_ctrl16_08,
	f54_analog_ctrl16_09,
	f54_analog_ctrl16_10,
	f54_analog_ctrl16_11,
	f54_analog_ctrl16_12,
	f54_analog_ctrl16_13,
	f54_analog_ctrl16_14,
	f54_analog_ctrl16_15,
	f54_analog_ctrl17_00,
	f54_analog_ctrl17_01,
	f54_analog_ctrl17_02,
	f54_analog_ctrl17_03,
	f54_analog_ctrl17_04,
	f54_analog_ctrl17_05,
	f54_analog_ctrl17_06,
	f54_analog_ctrl17_07,
	f54_analog_ctrl18_00,
	f54_analog_ctrl18_01,
	f54_analog_ctrl18_02,
	f54_analog_ctrl18_03,
	f54_analog_ctrl18_04,
	f54_analog_ctrl18_05,
	f54_analog_ctrl18_06,
	f54_analog_ctrl18_07,
	f54_analog_ctrl19_00,
	f54_analog_ctrl19_01,
	f54_analog_ctrl19_02,
	f54_analog_ctrl19_03,
	f54_analog_ctrl19_04,
	f54_analog_ctrl19_05,
	f54_analog_ctrl19_06,
	f54_analog_ctrl19_07,
	f54_analog_ctrl20,   
	f54_analog_ctrl21_00,
	f54_analog_ctrl21_01,
	f54_analog_ctrl22,   
	f54_analog_ctrl23_00,
	f54_analog_ctrl23_01,
	f54_analog_ctrl24_00,
	f54_analog_ctrl24_01,
	f54_analog_ctrl25,   
	f54_analog_ctrl26,   
	f54_analog_ctrl27,   
	f54_analog_ctrl28_00,
	f54_analog_ctrl28_01,
	f54_analog_ctrl29,   
	f54_analog_ctrl30,   
	f54_analog_ctrl31,   
	f54_analog_ctrl32_00,
	f54_analog_ctrl32_01,
	f54_analog_ctrl33_00,
	f54_analog_ctrl33_01,
	f54_analog_ctrl34_00,
	f54_analog_ctrl34_01,
	f54_analog_ctrl35_00,
	f54_analog_ctrl35_01,
	f54_analog_ctrl36_00,
	f54_analog_ctrl36_01,
	f54_analog_ctrl36_02,
	f54_analog_ctrl36_03,
	f54_analog_ctrl36_04,
	f54_analog_ctrl36_05,
	f54_analog_ctrl36_06,
	f54_analog_ctrl36_07,
	f54_analog_ctrl36_08,
	f54_analog_ctrl36_09,
	f54_analog_ctrl36_10,
	f54_analog_ctrl36_11,
	f54_analog_ctrl36_12,
	f54_analog_ctrl36_13,
	f54_analog_ctrl36_14,
	f54_analog_ctrl36_15,
	f54_analog_ctrl36_16,
	f54_analog_ctrl36_17,
	f54_analog_ctrl36_18,
	f54_analog_ctrl36_19,
	f54_analog_ctrl36_20,
	f54_analog_ctrl36_21,
	f54_analog_ctrl36_22,
	f54_analog_ctrl36_23,
	f54_analog_ctrl36_24,
	f54_analog_ctrl36_25,
	f54_analog_ctrl36_26,
	f54_analog_ctrl38_00,
	f54_analog_ctrl38_01,
	f54_analog_ctrl38_02,
	f54_analog_ctrl38_03,
	f54_analog_ctrl38_04,
	f54_analog_ctrl38_05,
	f54_analog_ctrl38_06,
	f54_analog_ctrl38_07,
	f54_analog_ctrl39_00,
	f54_analog_ctrl39_01,
	f54_analog_ctrl39_02,
	f54_analog_ctrl39_03,
	f54_analog_ctrl39_04,
	f54_analog_ctrl39_05,
	f54_analog_ctrl39_06,
	f54_analog_ctrl39_07,
	f54_analog_ctrl40_00,
	f54_analog_ctrl40_01,
	f54_analog_ctrl40_02,
	f54_analog_ctrl40_03,
	f54_analog_ctrl40_04,
	f54_analog_ctrl40_05,
	f54_analog_ctrl40_06,
	f54_analog_ctrl40_07,
	f54_analog_ctrl41,   
	f54_analog_ctrl42_00,
	f54_analog_ctrl42_01,
	f54_analog_ctrl55,   
	f54_analog_ctrl56,   
	f54_analog_ctrl57,   
	f54_analog_ctrl58,   
};

/* Page Description Table (PDT) */
struct synaptics_pdt_register_map_t {
	u8 F11_query_base;
	u8 F11_command_base;
	u8 F11_control_base;
	u8 F11_data_base;
	u8 F11_version_interrupt_count;
	u8 F11_function_exists;
	u8 F01_query_base;
	u8 F01_command_base;
	u8 F01_control_base;
	u8 F01_data_base;
	u8 F01_version_interrupt_count;
	u8 F01_function_exists;
	u8 F34_query_base;
	u8 F34_command_base;
	u8 F34_control_base;
	u8 F34_data_base;
	u8 F34_version_interrupt_count;
	u8 F34_function_exists;
	u8 F54_query_base;
	u8 F54_command_base;
	u8 F54_control_base;
	u8 F54_data_base;
	u8 F54_version_interrupt_count;
	u8 F54_function_exists;
	u8 F31_query_base;
	u8 F31_command_base;
	u8 F31_control_base;
	u8 F31_data_base;
	u8 F31_version_interrupt_count;
	u8 F31_function_exists;
	u8 F1A_query_base;
	u8 F1A_command_base;
	u8 F1A_control_base;
	u8 F1A_data_base;
	u8 F1A_version_interrupt_count;
	u8 F1A_function_exists;
};

#endif
