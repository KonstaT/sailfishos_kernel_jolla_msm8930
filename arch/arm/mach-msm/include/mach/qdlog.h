#ifndef _DOWNLOAD_LOG_H_
#define _DOWNLOAD_LOG_H_

#include "partition_softlink.h"
// access QDLOG_LINK_STR or PART_DLOG

#define DLOG_MAGIC_NUM  0x474F4C44  // 'DLOG'

#define DLOG_OFF_MAGIC_NUM  0
#define DLOG_LEN_MAGIC_NUM  4
#define DLOG_OFF_COUNTER    4
#define DLOG_LEN_COUNTER    4

#define DLOG_LEN_TYPE       16
#define DLOG_LEN_PARTITION  16
#define DLOG_LEN_MD5_PC     32
#define DLOG_LEN_MD5_TARGET 32
#define DLOG_LEN_DATE_TIME  32
#define DLOG_LEN_RTC        24
#define DLOG_LEN_PASSPORT   16
#define DLOG_LEN_BEGIN      8
#define DLOG_LEN_END        8
#define DLOG_LEN            256

#define MAX_DLOG_NO         1000

struct download_log {
	char type[DLOG_LEN_TYPE];
	char pad_0[8];
	char partition[DLOG_LEN_PARTITION];
	char pad_1[8];
	char MD5_PC[DLOG_LEN_MD5_PC];
	char pad_2[8];
	char MD5_target[DLOG_LEN_MD5_TARGET];
	char pad_3[8];
	char date_time[DLOG_LEN_DATE_TIME];
	char pad_4[8];
	char rtc[DLOG_LEN_RTC];
	char pad_5[8];
	char passport[DLOG_LEN_PASSPORT];
	char pad_6[8];
	char begin[DLOG_LEN_BEGIN];
	char pad_7[8];
	char end[DLOG_LEN_END];
	char pad_8[8];     // 256 bytes per download log
};


#endif