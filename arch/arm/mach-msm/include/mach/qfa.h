#ifndef _QFA_H_
#define _QFA_H_


#include "partition_softlink.h"
// access QFA_LINK_STR or PART_FA

#define FA_SECTOR_SZ    256
#define FA_BASE_OFFSET  0

#define FA_HEADER_1_OFFSET  (FA_BASE_OFFSET)
#define FA_HEADER_2_OFFSET  (FA_HEADER_1_OFFSET + FA_SECTOR_SZ)

#define FA_COMMA_LENGTH     1
#define FA_OFFSET(n)        (n) + FA_COMMA_LENGTH


// Header 1
#define FA_MAGIC_NUM    "Dan6673Dan"

#define FA_MAGIC_NUM_LENGTH     10
#define FA_VER_LENGTH           2
#define FA_PICASSO_LENGTH       10
#define FA_IMEI_LENGTH          15

#define FA_MAGIC_NUM_OFFSET     (FA_HEADER_1_OFFSET)
#define FA_VER_OFFSET           FA_OFFSET(FA_MAGIC_NUM_OFFSET + FA_MAGIC_NUM_LENGTH)
#define FA_PICASSO_OFFSET       FA_OFFSET(FA_VER_OFFSET + FA_VER_LENGTH)
#define FA_IMEI_OFFSET          FA_OFFSET(FA_PICASSO_OFFSET + FA_PICASSO_LENGTH)
#define FA_SERVICE_ID_OFFSET    FA_OFFSET(FA_IMEI_OFFSET + FA_IMEI_LENGTH)

#define FA_MAGIC_NUM_STRING    "Magic_Number"
#define FA_VER_STRING          "FA_version"
#define FA_PICASSO_STRING      "picasso"
#define FA_IMEI_STRING         "IMEI"


// Header 2
#define FA_FIRST_RECORD_POINTER_LENGTH  4
#define FA_TAIL_RECORD_POINTER_LENGTH   4

#define FA_FIRST_RECORD_POINTER_OFFSET  (FA_HEADER_2_OFFSET)
#define FA_TAIL_RECORD_POINTER_OFFSET   FA_OFFSET(FA_FIRST_RECORD_POINTER_OFFSET + FA_FIRST_RECORD_POINTER_LENGTH)

#define FA_FIRST_RECORD_POINTER_STRING  "First_Record_Pointer"
#define FA_TAIL_RECORD_POINTER_STRING   "Tail_Record_Pointer"


// Record
#define FA_RECORD_SZ                    (FA_SECTOR_SZ)
#define FA_RECORD_TYPE_LENGTH           1
#define FA_RECORD_PART_NUMBER_LENGTH    15
#define FA_RECORD_SO_LENGTH             10
#define FA_RECORD_SO_TYPE_LENGTH        1

#define FA_RECORD_TYPE_OFFSET(base)         (base)
#define FA_RECORD_PART_NUMBER_OFFSET(base)  FA_OFFSET(FA_RECORD_TYPE_OFFSET(base) + FA_RECORD_TYPE_LENGTH)
#define FA_RECORD_SO_OFFSET(base)           FA_OFFSET(FA_RECORD_PART_NUMBER_OFFSET(base) + FA_RECORD_PART_NUMBER_LENGTH)
#define FA_RECORD_SO_TYPE_OFFSET(base)      FA_OFFSET(FA_RECORD_SO_OFFSET(base) + FA_RECORD_SO_LENGTH)


// FQC Record
#define FA_FQC_RECORD_SZ            (FA_SECTOR_SZ)
#define FA_FQC_RECORD_TYPE_LENGTH   3
#define FA_FQC_TEST_DATE_LENGTH     8
#define FA_FQC_TEST_TIME_LENGTH     6
#define FA_FQC_TEST_RESULT_LENGTH   1

#define FA_FQC_RECORD_TYPE_OFFSET(base)     (base)
#define FA_FQC_TEST_DATE_OFFSET(base)       FA_OFFSET(FA_FQC_RECORD_TYPE_OFFSET(base) + FA_FQC_RECORD_TYPE_LENGTH)
#define FA_FQC_TEST_TIME_OFFSET(base)       FA_OFFSET(FA_FQC_TEST_DATE_OFFSET(base) + FA_FQC_TEST_DATE_LENGTH)
#define FA_FQC_TEST_RESULT_OFFSET(base)     FA_OFFSET(FA_FQC_TEST_TIME_OFFSET(base) + FA_FQC_TEST_TIME_LENGTH)
#define FA_FQC_ERROR_CODE_OFFSET(base)      FA_OFFSET(FA_FQC_TEST_RESULT_OFFSET(base) + FA_FQC_TEST_RESULT_LENGTH)


// OOB Record
#define FA_OOB_RECORD_SZ            (FA_SECTOR_SZ)
#define FA_OOB_RECORD_TYPE_LENGTH   3
#define FA_OOB_TEST_DATE_LENGTH     8
#define FA_OOB_TEST_TIME_LENGTH     6

#define FA_OOB_RECORD_TYPE_OFFSET(base)    (base)
#define FA_OOB_TEST_DATE_OFFSET(base)      (FA_OOB_RECORD_TYPE_OFFSET(base) + FA_OOB_RECORD_TYPE_LENGTH)
#define FA_OOB_TEST_TIME_OFFSET(base)      (FA_OOB_TEST_DATE_OFFSET(base) + FA_OOB_TEST_DATE_LENGTH)


// name, offset, and length pair
#define FA_INFO_ITEM(x)     {FA_##x##_STRING, FA_##x##_OFFSET, FA_##x##_LENGTH}

// name, offset, and length pair array
#define FA_INFO_TABLE \
    static const struct { \
        char *name; \
        unsigned int offset; \
        unsigned int length; \
    } const fa_info_table[] = { \
        FA_INFO_ITEM(MAGIC_NUM), \
        FA_INFO_ITEM(VER), \
        FA_INFO_ITEM(PICASSO), \
    }


#endif


