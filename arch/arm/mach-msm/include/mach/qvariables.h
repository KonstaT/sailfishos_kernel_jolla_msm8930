#ifndef __QVARIABLE_H
#define __QVARIABLE_H

#define VAR_LEN     5

#define VAR_ROOT_STR        "root"
#define VAR_ROOT_OFFSET     0x100
#define VAR_ROOT_OnSTR      "R1oOt"

#define VAR_ADB_STR         "adb"
#define VAR_ADB_OFFSET      0x252
#define VAR_ADB_OnSTR       "qa2Db"

#define VAR_DIAG_STR        "diag"
#define VAR_DIAG_OFFSET     0x3C3
#define VAR_DIAG_OnSTR      "DIa3g"

#define VAR_FTD_STR         "ftd"
#define VAR_FTD_OFFSET      0x507
#define VAR_FTD_OnSTR       "ftdQ4"

#define VAR_UART_STR        "uart"
#define VAR_UART_OFFSET     0x66D
#define VAR_UART_OnSTR      "U5ARt"

#define VAR_FIXUSBID_STR    "fixusbid"
#define VAR_FIXUSBID_OFFSET 0x7C2
#define VAR_FIXUSBID_OnSTR  "Us6bQ"

#define VAR_DEBUGFS_STR     "debugfs"
#define VAR_DEBUGFS_OFFSET  0x917
#define VAR_DEBUGFS_OnSTR   "DbG7q"

#define VAR_LOGNOTIFY_STR       "lognotify"
#define VAR_LOGNOTIFY_OFFSET    0xA5B
#define VAR_LOGNOTIFY_OnSTR     "LoGN8"

#define VAR_NOOFFCHARGE_STR     "nooffcharge"
#define VAR_NOOFFCHARGE_OFFSET  0xBA7
#define VAR_NOOFFCHARGE_OnSTR   "9nChG"

#define VAR_NONFCUPGRADE_STR    "nonfcupgrade"
#define VAR_NONFCUPGRADE_OFFSET 0xC81
#define VAR_NONFCUPGRADE_OnSTR  "n0NFC"

#define VAR_DEBUGLEVEL_STR          "debuglevel"
#define VAR_DEBUGLEVEL_OFFSET       0xD00
#define VAR_DEBUGLEVEL_OnSTR        "D36ug"
#define VAR_DEBUGLEVEL_VALUE_OFFSET 0xD10

#define VAR_SUBSYS_RAMDUMP_STR      "subsys_ramdump"
#define VAR_SUBSYS_RAMDUMP_OFFSET   0xEBA
#define VAR_SUBSYS_RAMDUMP_OnSTR    "ssRdp"

#define VAR_QDLMODE_STR     "qdlmode"
#define VAR_QDLMODE_OFFSET  0xF42
#define VAR_QDLMODE_OnSTR   "9d1md"

#define VAR_DO_FSCK_STR     "do_fsck"
#define VAR_DO_FSCK_OFFSET  0x10A4
#define VAR_DO_FSCK_OnSTR   "dfsck"

#define VAR_SU_STR          "superuser"
#define VAR_SU_OFFSET       0x11B7
#define VAR_SU_OnSTR        "su_5U"

#define VAR_UNLOCK_STR      "unlock"
#define VAR_UNLOCK_OFFSET   0x4F45
#define VAR_UNLOCK_LEN      11
#define VAR_UNLOCK_OnVal    {0x4B, 0x31, 0x32, 0x43, 0x6F, 0x4C, 0x4E, 0x75, 0x4F, 0x65, 0x4D}


// name, offset, length, and on value pair
#define VAR_INFO_ITEM(x)    {VAR_##x##_STR, VAR_##x##_OFFSET, VAR_LEN, VAR_##x##_OnSTR}

// name, offset, length, and on value pair array
#define VAR_INFO_TABLE \
    static const struct { \
        char *name; \
        unsigned int offset; \
        unsigned int length; \
        unsigned char on_value[16]; \
    } const variables_info_table[] = { \
        VAR_INFO_ITEM(ROOT), \
        VAR_INFO_ITEM(ADB), \
        VAR_INFO_ITEM(DIAG), \
        VAR_INFO_ITEM(FTD), \
        VAR_INFO_ITEM(UART), \
        VAR_INFO_ITEM(DEBUGFS), \
        VAR_INFO_ITEM(LOGNOTIFY), \
        VAR_INFO_ITEM(NOOFFCHARGE), \
        VAR_INFO_ITEM(NONFCUPGRADE), \
        VAR_INFO_ITEM(SUBSYS_RAMDUMP), \
        VAR_INFO_ITEM(QDLMODE), \
        VAR_INFO_ITEM(SU), \
    }


#endif
