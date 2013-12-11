#ifndef _QOTP_H_
#define _QOTP_H_


#include "partition_softlink.h"
// access QOTP_LINK_STR or PART_OTP

#define MAX_OTP_NUM     1
#define OTP_SIZE        0x00400000  // 4MB

/*
static inline int OTP_BASE(unsigned int n)
{
    return (n < MAX_OTP_NUM) ? (int)(n*OTP_SIZE) : -1;
}
*/

#define OTP_MAGIC_NUM_OFFSET    0x0
#define OTP_MAGIC_NUM_LENGTH    4

#define OTP_MAGIC_NUM_0     0x16024958
#define OTP_MAGIC_NUM_1     0x648009F9
#define OTP_MAGIC_NUM_2     0xAB419598
#define OTP_MAGIC_NUM_3     0xEC479A90

#define OTP_MAGIC_NUM \
    static const unsigned int otp_magic_num[4] = {OTP_MAGIC_NUM_0, OTP_MAGIC_NUM_1, OTP_MAGIC_NUM_2, OTP_MAGIC_NUM_3}


#define OTP_IMEI_STRING     "IMEI"
#define OTP_IMEI_OFFSET     0x10
#define OTP_IMEI_LENGTH     15


#define OTP_SERIAL_NO_STRING    "SerialNo"
#define OTP_SERIAL_NO_OFFSET    0x20
#define OTP_SERIAL_NO_LENGTH    16


#define OTP_BT_STRING       "BT"
#define OTP_BT_OFFSET       0x30
#define OTP_BT_LENGTH       12


#define OTP_WIFI_STRING     "Wifi"
#define OTP_WIFI_OFFSET     0x40
#define OTP_WIFI_LENGTH     12


#define OTP_QLOCK_STRING    "Qlock"
#define OTP_QLOCK_OFFSET    0x50
#define OTP_QLOCK_LENGTH    128


#define OTP_MEID_STRING     "MEID"
#define OTP_MEID_OFFSET     0xD0
#define OTP_MEID_LENGTH     15


// name, offset, and length pair
#define OTP_INFO_ITEM(x)    {OTP_##x##_STRING, OTP_##x##_OFFSET, OTP_##x##_LENGTH}

// name, offset, and length pair array
#define OTP_INFO_TABLE \
    static const struct { \
        char *name; \
        unsigned int offset; \
        unsigned int length; \
    } otp_info_table[] = { \
        OTP_INFO_ITEM(IMEI), \
        OTP_INFO_ITEM(SERIAL_NO), \
        OTP_INFO_ITEM(BT), \
        OTP_INFO_ITEM(WIFI), \
        OTP_INFO_ITEM(QLOCK), \
        OTP_INFO_ITEM(MEID), \
    }

#endif
