#ifndef _QCFG_H_
#define _QCFG_H_


#include "partition_softlink.h"
// access QCFG_LINK_STR or PART_CFG

#define FA_CHECK_PATTERN        61
#define FA_CHECK_PATTERN_LENGTH 5
#define FA_TLINEMODE            66
#define FA_TLINEMODE_LENGTH     4
#define FA_TLINEMODE_KEY        "TLNM"  // t-line mode


#define FA_STATION_LENGTH       4
#define FA_STATION_OFFSET_DAY   0
#define FA_STATION_OFFSET_MON   1
#define FA_STATION_OFFSET_YEAR  2
#define FA_STATION_OFFSET_NUM   3

#define FA_STATION1         5
#define FA_STATION2         9
#define FA_STATION3         13
#define FA_STATION4         17
#define FA_STATION5         21
#define FA_STATION6         25
#define FA_STATION7         29
#define FA_STATION8         33
#define FA_STATION9         37
#define FA_STATION10        41

#define FA_STATION11        128
#define FA_STATION12        132
#define FA_STATION13        136
#define FA_STATION14        140
#define FA_STATION15        144
#define FA_STATION16        148
#define FA_STATION17        152
#define FA_STATION18        156
#define FA_STATION19        160
#define FA_STATION20        164


#define FA_MMI_ERRORCODE_OFFSET     504
#define FA_MMI_ERRORCODE_LENGTH     8
#define FA_FLAG_301                 512
#define FA_FLAG_301_LENGTH          48
#define FA_FLAG_GSENSOR             531
#define FA_FLAG_GSENSOR_LEN         1
#define FA_FLAG_304                 560
#define FA_FLAG_304_LENGTH          12

#define FA_RUNIN_ERRORCODE_OFFSET   572
#define FA_RUNIN_ERRORCODE_LENGTH   6
#define FA_FLAG_RUNIN               578
#define FA_FLAG_RUNIN_LEN           20


#define FA_FLAG_303             1152
#define FA_FLAG_303_LENGTH      48

#define FA_FLAG_ACTIVATION      895
#define FA_FLAG_ACTIVATION_LEN  128

#define FA_SERVICE_TAG_LENGTH   32
#define FA_SERVICE_TAG1         1024
#define FA_SERVICE_TAG2         1056
#define FA_SERVICE_TAG3         1088
#define FA_SERVICE_TAG4         1120


//============================================================================//
//  P-Sensor and L-Sensor Calibration
//============================================================================//
struct lpsensor_cal_data {
  struct {
    unsigned short als_factor;
    unsigned short als_ratio;
  } als;
  struct {
    unsigned short prox_far;
    unsigned short prox_near;
  } prox;
};
#define FA_LPSENSOR_DATA        1200
#define FA_LPSENSOR_DATA_LENGTH sizeof(struct lpsensor_cal_data)

//============================================================================//
//  G-Sensor Calibration
//============================================================================//
struct gsensor_cal_data {
  struct {
    signed int x;
    signed int y;
    signed int z;
  } bias;
};
#define FA_GSENSOR_DATA        1208
#define FA_GSENSOR_DATA_LENGTH sizeof(struct gsensor_cal_data)

//============================================================================//
//  P-Sensor and L-Sensor Reserved Data
//============================================================================//
struct lpsensor_csd_data {
#define FA_LPSENSOR_MAGIC_NUM	0x2771 //ISOK
    unsigned short magic_num;
};
#define FA_LPSENSOR_CSD_DATA	1256
#define FA_LPSENSOR_CSD_DATA_LENGTH sizeof(struct lpsensor_csd_data)

/* 8930 Boston CR #: XXX, WH Lee, 20130131 */
struct psensor_offset_cal_data {
	signed char prox_offset;
};
#define FA_PSENSOR_OFFSET      1320
#define FA_PSENSOR_OFFSET_LENGTH sizeof(struct psensor_offset_cal_data)
/* WH Lee, 20130131 */

#define FA_TIME_ZONE            1352
#define FA_TIME_ZONE_LENGTH     32
#define FA_TIME_ZONE_MAGIC_NUM 0x00005A54
struct fa_time_zone {
    unsigned int magic_num;
    signed int time_zone;
};

/* Jen Chang add for adc and thershold calibration */
//============================================================================//
//  Audio Headset Calibration
//============================================================================//
struct audio_mic_cal_data {
	unsigned int magic_num;
	signed int dce_z;
	signed int dce_mb;
	signed int sta_z;
	signed int sta_mb;
	signed int offset;
};

#define FA_AUDIOHS_DATA		1224
#define FA_AUDIOHS_DATA_LENGTH sizeof(struct audio_mic_cal_data)

#define FA_AUDIOHS_MAGIC_NUM		0x49534F4B //ISOK

#define FA_AUDIOHS_MAGIC_NUM_DATA			1224
#define FA_AUDIOHS_MAGIC_NUM_DATA_LENGTH	4

#define FA_AUDIOHS_INIT_CAL_DATA			1228
#define FA_AUDIOHS_INIT_CAL_DATA_LENGTH		16

#define FA_AUDIOHS_THRESHOLD_DATA			1244
#define FA_AUDIOHS_THRESHOLD_DATA_LENGTH	4
/* Jen Chang, 20120906 */

//Eric Liu+
//============================================================================//
//  BMS FCC data (CC uAh per soc)
//============================================================================//
#define BMS_CC_DATA         1288
#define BMS_CC_DATA_LENGTH  4
//Eric Liu-
//Carl Chang+
//============================================================================//
//  BMS r_sense_uohm data
//============================================================================//
#define BMS_R_SENSE_DATA         1292
#define BMS_R_SENSE_DATA_LENGTH  4
//Carl Chang-

#define FA_WIFI_GOLDBIN_INDEX   1220
#define FA_WIFI_GOLDBIN_INDEX_LENGTH    4
#define FA_WIFI_GOLDBIN_INDEX_MAX   2

#define FA_OTA_INFO         1536
#define FA_OTA_INFO_LENGTH  1024

#define FA_WIFI_DATA		8192
#define FA_WIFI_LENGTH_MAX	16384


#define FA_STATION1_STR     "STATION1"
#define FA_STATION2_STR     "STATION2"
#define FA_STATION3_STR     "STATION3"
#define FA_STATION4_STR     "STATION4"
#define FA_STATION5_STR     "STATION5"
#define FA_STATION6_STR     "STATION6"
#define FA_STATION7_STR     "STATION7"
#define FA_STATION8_STR     "STATION8"
#define FA_STATION9_STR     "STATION9"
#define FA_STATION10_STR    "STATION10"

#define FA_STATION11_STR    "STATION11"
#define FA_STATION12_STR    "STATION12"
#define FA_STATION13_STR    "STATION13"
#define FA_STATION14_STR    "STATION14"
#define FA_STATION15_STR    "STATION15"
#define FA_STATION16_STR    "STATION16"
#define FA_STATION17_STR    "STATION17"
#define FA_STATION18_STR    "STATION18"
#define FA_STATION19_STR    "STATION19"
#define FA_STATION20_STR    "STATION20"


#define FA_CHECK_PATTERN_STR    "check_pattern"
#define FA_TLINEMODE_STR        "t_line_mode"


#define FA_MMI_ERRORCODE_STR    "mmi_err_code"
#define FA_FLAG_301_STR         "*#301"
#define FA_FLAG_304_STR         "*#304"

#define FA_RUNIN_ERRORCODE_STR  "runin_err_code"
#define FA_FLAG_RUNIN_STR       "runin"


#define FA_FLAG_303_STR         "*#303"

#define FA_SERVICE_TAG1_STR     "service_tag1"
#define FA_SERVICE_TAG2_STR     "service_tag2"
#define FA_SERVICE_TAG3_STR     "service_tag3"
#define FA_SERVICE_TAG4_STR     "service_tag4"


#define FA_LPSENSOR_DATA_STR    "LP_sensor_data"
#define FA_GSENSOR_DATA_STR     "G_sensor_data"

/* Jen Chang add for adc and thershold calibration */
#define FA_AUDIOHS_DATA_STR		"audio_mic_data"
/* Jen Chang, 20120906 */

#define FA_LPSENSOR_CSD_DATA_STR    "LP_csd_data"

#define FA_WIFI_GOLDBIN_INDEX_STR   "wifi_goldbin_index"

/* 8930 Boston CR #: XXX, WH Lee, 20130131 */
#define FA_PSENSOR_OFFSET_STR     "P_sensor_offset"
/* WH Lee, 20130131 */

#define FA_OTA_INFO_STR         "OTA_update_info"

#define FA_WIFI_DATA_STR        "wifi_data"


// name, offset, and length pair array
#define CFG_INFO_TABLE \
    static const struct { \
        char *name; \
        unsigned int offset; \
        unsigned int length; \
    } const cfg_info_table[] = { \
        {FA_STATION1_STR,   FA_STATION1,    FA_STATION_LENGTH}, \
        {FA_STATION2_STR,   FA_STATION2,    FA_STATION_LENGTH}, \
        {FA_STATION3_STR,   FA_STATION3,    FA_STATION_LENGTH}, \
        {FA_STATION4_STR,   FA_STATION4,    FA_STATION_LENGTH}, \
        {FA_STATION5_STR,   FA_STATION5,    FA_STATION_LENGTH}, \
        {FA_STATION6_STR,   FA_STATION6,    FA_STATION_LENGTH}, \
        {FA_STATION7_STR,   FA_STATION7,    FA_STATION_LENGTH}, \
        {FA_STATION8_STR,   FA_STATION8,    FA_STATION_LENGTH}, \
        {FA_STATION9_STR,   FA_STATION9,    FA_STATION_LENGTH}, \
        {FA_STATION10_STR,  FA_STATION10,   FA_STATION_LENGTH}, \
                                                                \
        {FA_STATION11_STR,  FA_STATION11,   FA_STATION_LENGTH}, \
        {FA_STATION12_STR,  FA_STATION12,   FA_STATION_LENGTH}, \
        {FA_STATION13_STR,  FA_STATION13,   FA_STATION_LENGTH}, \
        {FA_STATION14_STR,  FA_STATION14,   FA_STATION_LENGTH}, \
        {FA_STATION15_STR,  FA_STATION15,   FA_STATION_LENGTH}, \
        {FA_STATION16_STR,  FA_STATION16,   FA_STATION_LENGTH}, \
        {FA_STATION17_STR,  FA_STATION17,   FA_STATION_LENGTH}, \
        {FA_STATION18_STR,  FA_STATION18,   FA_STATION_LENGTH}, \
        {FA_STATION19_STR,  FA_STATION19,   FA_STATION_LENGTH}, \
        {FA_STATION20_STR,  FA_STATION20,   FA_STATION_LENGTH}, \
                                                                \
        {FA_CHECK_PATTERN_STR,  FA_CHECK_PATTERN,   FA_CHECK_PATTERN_LENGTH},  \
        {FA_TLINEMODE_STR,      FA_TLINEMODE,       FA_TLINEMODE_LENGTH},      \
                                                                               \
        {FA_MMI_ERRORCODE_STR,  FA_MMI_ERRORCODE_OFFSET,    FA_MMI_ERRORCODE_LENGTH}, \
        {FA_FLAG_301_STR,       FA_FLAG_301,                FA_FLAG_301_LENGTH},      \
        {FA_FLAG_304_STR,       FA_FLAG_304,                FA_FLAG_304_LENGTH},      \
                                                                                      \
        {FA_RUNIN_ERRORCODE_STR,    FA_RUNIN_ERRORCODE_OFFSET, FA_RUNIN_ERRORCODE_LENGTH}, \
        {FA_FLAG_RUNIN_STR,     FA_FLAG_RUNIN,      FA_FLAG_RUNIN_LEN}, \
                                                             \
        {FA_FLAG_303_STR,       FA_FLAG_303,        FA_FLAG_303_LENGTH}, \
                                                                         \
        {FA_SERVICE_TAG1_STR,   FA_SERVICE_TAG1,    FA_SERVICE_TAG_LENGTH}, \
        {FA_SERVICE_TAG2_STR,   FA_SERVICE_TAG2,    FA_SERVICE_TAG_LENGTH}, \
        {FA_SERVICE_TAG3_STR,   FA_SERVICE_TAG3,    FA_SERVICE_TAG_LENGTH}, \
        {FA_SERVICE_TAG4_STR,   FA_SERVICE_TAG4,    FA_SERVICE_TAG_LENGTH}, \
                                                                            \
        {FA_LPSENSOR_DATA_STR,  FA_LPSENSOR_DATA,   FA_LPSENSOR_DATA_LENGTH}, \
        {FA_GSENSOR_DATA_STR,   FA_GSENSOR_DATA,    FA_GSENSOR_DATA_LENGTH}, \
                                                                          \
        {FA_WIFI_GOLDBIN_INDEX_STR, FA_WIFI_GOLDBIN_INDEX,  FA_WIFI_GOLDBIN_INDEX_LENGTH}, \
                                                                              \
        {FA_AUDIOHS_DATA_STR, FA_AUDIOHS_DATA, FA_AUDIOHS_DATA_LENGTH}, \
																	 	\
        {FA_LPSENSOR_CSD_DATA_STR, FA_LPSENSOR_CSD_DATA, FA_LPSENSOR_CSD_DATA_LENGTH}, \
																	 	\
        {FA_PSENSOR_OFFSET_STR, FA_PSENSOR_OFFSET, FA_PSENSOR_OFFSET_LENGTH}, \
																	 	\
        {FA_OTA_INFO_STR,       FA_OTA_INFO,        FA_OTA_INFO_LENGTH}, \
                                                                         \
        {FA_WIFI_DATA_STR,      FA_WIFI_DATA,       FA_WIFI_LENGTH_MAX}, \
    }


#endif


