#ifndef __PARTITION_SOFTLINK_H
#define __PARTITION_SOFTLINK_H


// symbolic links used in android
#define QFA_LINK_STR		"/dev/block/platform/msm_sdcc.1/by-name/Qfa"
#define QCFG_LINK_STR		"/dev/block/platform/msm_sdcc.1/by-name/Qcfg"
#define QOTP_LINK_STR		"/dev/block/platform/msm_sdcc.1/by-name/QOTP"
#define QDLOG_LINK_STR		"/dev/block/platform/msm_sdcc.1/by-name/Qdlog"
#define QVARIABLE_LINK_STR	"/dev/block/platform/msm_sdcc.1/by-name/Qvariables"
#define QLOGFILTER_LINK_STR	"/dev/block/platform/msm_sdcc.1/by-name/Qlogfilter"
#define QGLOG_LINK_STR		"/dev/block/platform/msm_sdcc.1/by-name/Qglog"
#define FSG_LINK_STR		"/dev/block/platform/msm_sdcc.1/by-name/fsg"

// device nodes used in android, not suggest to use device nodes
/*
#define QFA_DEV_NODE		"/dev/block/mmcblk0p3"  
#define QCFG_DEV_NODE		"/dev/block/mmcblk0p4"  
#define QVARIABLE_DEV_NODE	"/dev/block/mmcblk0p6"
#define QLOGFILTER_DEV_NODE	"/dev/block/mmcblk0p7"
*/

// partition names used in aboot
#define PART_NAME_LEN_MAX   16
#define PART_FA     "Qfa"
#define PART_CFG    "Qcfg"
#define PART_OTP    "QOTP"
#define PART_VAR    "Qvariables"
#define PART_DLOG   "Qdlog"

#endif
