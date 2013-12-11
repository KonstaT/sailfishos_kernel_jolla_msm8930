/*******************************************************************************
                                                                                
                      Qisda Handset Project                                     
                                                                                
                     Copyright (c) 2011 Qisda Corpration                        
                                                                                
File Description                                                                
----------------                                                                
                                                                                
when        who          what, where, why                                     
                                                                                
----------  -----------	 ----------------------------------------------------
2013/08/29  JZ Hou       Boston 1.0:[NON_HLOS][L1] read auth_enabled status here
2013/08/23  Tina Hsu	      Boston 1.0:[NON_HLOS] Record Diag Sleep Vote caller and Diag Ballot Mask Value
2013/07/16  Tina Hsu      Boston 1.0:[NON_HLOS][SYS] Modem PM Log record sleep vote status
2013/06/18  Jerry Tseng  Boston 1.0:[NON_HLOS][L1] Expand Board ID for Casper Project
2013/04/09  JoshuaYeh    Boston 1.0:[NON_HLOS]Add function to statistic values
2013/04/09  Tina Hsu     Boston 1.0:[NON_HLOS][SYS] Delete Log Size And Version From Modem PM Log body
2013/04/08  Tina Hsu     Boston 1.0:[NON_HLOS][SYS] Add Log Size And Version In Modem PM Log Structure And Modify Log 
2013/04/01  MarkHsieh    Boston 1.0:[NON_HLOS][L1] Boston T-Mobile porting
2013/03/28  Tina Hsu     Boston 1.0:[NON_HLOS][SYS] Add More PM Log
2013/03/26  Jerry Tseng  Boston 1.0:[NON_HLOS][L1] Refine CPU sample type category
2013/03/22  Tina Hsu     Boston 1.0:[NON_HLOS][SYS] Fix Modem LPM PM log and add log size and version
2013/02/21  Tina Hsu     Boston 1.0:[NON_HLOS][SYS] Add Modem LPM PM Log
2013/01/15  Jerry Tseng  Boston 1.0:[NON_HLOS][L1] Porting FSG backup check mechanism
2012/12/26  JZHou        Boston 1.0:[NON_HLOS][L1] Add new board id
2012/12/08  Mark Hsieh	 Boston 1.0:[NON_HLOS][L1] Write SBLx log to RAM buffer
2012/11/05  JZHou        [NON_HLOS][L1] Merge from Detroit
********************************************************************************/ 
#ifndef _OEM_SMEM_STRUCT_H_
#define _OEM_SMEM_STRUCT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "qotp.h"


/* QISDA  JZ Hou, 2012/05/10, add uint8/uint16/uint32/uint64 define for HLOS { */
//typedef  unsigned long long uint64;
//typedef  unsigned long int  uint32;  
//typedef  unsigned short  uint16;  
//typedef  unsigned char  uint8;  
/* } QISDA  JZ Hou, 2012/05/10 */

/* This enum indicates OEM band type*/
typedef enum
{
  EU      = 0x11,	
  JP      = 0x10,
  NONLTE  = 0x01,
  NA717   = 0x00, /* Qisda Mark Hsieh, 2013/04/01, Boston T-Mobile porting */
  BAND_NOT_SUPPORTED = 0x7fffffff
}BAND_TYPE;
	
/* This enum indicates OEM board id */
typedef enum
{
  EVB0        = 0x100000,
  EVB1        = 0x101000,
  EVB2        = 0x102000,
  EVT0        = 0x200000,
  EVT1        = 0x201000,
  EVT1_1      = 0x201010,
  EVT1_2      = 0x201020,
  EVT1_3      = 0x201030,
  EVT2        = 0x202000,
  EVT2_1      = 0x202010,
  EVT2_2      = 0x202020,
  EVT2_3      = 0x202030,
  DVT0        = 0x300000,
  DVT1        = 0x301000,
  DVT1_1      = 0x301010,
  PVT0        = 0x400000,
  /* Qisda, Jerry Tseng, 2013/06/18, Expand Board ID for Casper Project { */
  CASPER_EVT1 = 0x01201000,
  CASPER_EVT2 = 0x01202000,
  CASPER_DVT0 = 0x01300000,
  /* } Qisda, Jerry Tseng, 2013/06/18 */
  UNKNOWN_BOARD_ID = 0x7fffffff
} BOARD_ID_TYPE;

typedef enum
{
  HOUSTON    = 0x0,
  BOSTON     = 0x1,
  EGYPT      = 0x2,
  SAPPORO    = 0x3,
  UNKNOWN_PROJECT_ID = 0x7fffffff
} PROJECT_ID_TYPE;
	
/* This is real DDR vendor number from mode register */
typedef enum
{
  SAMSUNG_DDR = 0x1,
  ELPIDA_DDR  = 0x3,
  HYNIX_DDR   = 0x6,
  MICRON_DDR  = 0xff,
  UNKNOWN_DDR_VENDOR = 0x7fffffff
} DDR_VENDOR_TYPE;

/*Qisda Tina Hsu, 2013/01/25, Add Modem LPM PM log { */
typedef enum
{
  CLEAR_MODEM_STATUS_FLAG =0x0,
  READED_MODEM_STATUS_FLAG =0x1
}Modem_PM_Log_Flag_Status;

typedef enum
{
 Modem_PM_LOG_LPM_STATUS_CHECKAMOUNT=0,
 Modem_PM_LOG_LPM_STATUS_RPMMQueueIsEmpty_FIRST, /*1*/
 Modem_PM_LOG_LPM_STATUS_LowPowerMode_waitforactive, /*2*/
 Modem_PM_LOG_LPM_STATUS_SleepIdleEntry_waitforactive, /*3*/
 Modem_PM_LOG_LPM_STATUS_BlastPowerShutdown_waitforactive, /*4*/
 Modem_PM_LOG_LPM_STATUS_RPMMQueueIsEmpty_SECOND, /*5*/
 Modem_PM_LOG_LPM_STATUS_TIMEREXPIRY, /*6*/
 Modem_PM_LOG_LPM_STATUS_pendinginterrupt_FIRST, /*7*/
 Modem_PM_LOG_LPM_STATUS_noenoughtime,  /*8*/
 Modem_PM_LOG_LPM_STATUS_setcpufrequency, /*9*/
 Modem_PM_LOG_LPM_STATUS_nosleepmodechosen,      /*0x0A*/
 Modem_PM_LOG_LPM_STATUS_pendinginterrupt_SECOND, /*0x0B*/
 Modem_PM_LOG_LPM_STATUS_noenoughtime_TIMETILEDEADLINE, /*0x0C*/
 Modem_PM_LOG_LPM_STATUS_entersleepmode, /*0x0D*/
 Modem_PM_LOG_LPM_STATUS_exitsleepmode, /*0x0E*/
 Modem_PM_LOG_LPM_STATUS_pendinginterruptwakeup_SCHEDULED, /*0x0F*/
 Modem_PM_LOG_LPM_STATUS_pendinginterruptwakeup_UNSCHEDULED, /*0x10*/
 Modem_PM_LOG_LPM_STATUS_latewakeup, /*0x11*/
 Modem_PM_LOG_LPM_STATUS_CHECKAMOUNT_SLEEPEXITPOINT, /*0x12*/
 Modem_PM_LOG_LPM_STATUS_Legacy_Solver_Entry, /*0x13*/
 Modem_PM_LOG_LPM_STATUS_Out_of_Time, /*0x14*/
 Modem_PM_LOG_LPM_STATUS_Sleep_Swfi_Entry, /*0x15*/
 Modem_PM_LOG_LPM_STATUS_Legacy_Solver_Scan_Lpr, /*0x16*/
 Modem_PM_LOG_LPM_STATUS_Legacy_Solver_Scan_Lprm, /*0x17*/
 Modem_PM_LOG_LPMSTATUS_NUM			
}ModemPMloglpmstatus;

typedef enum
{ 
  ID_LATENCY_BUDGET, /*0*/
  ID_HARD_DURATION, /*1*/
  ID_SOFT_DURATION, /*2*/
  ID_START_TO_CURRENT_DURATION, /*3*/
  ID_SOLVER_LATENCY, /*4*/
  PMLogValue_ID_NUM
}ModemPMlogValueId;


typedef struct
{
  unsigned long int count;
  unsigned long int total;
  unsigned long int max;
}ModemPMLogValueStatistics;

/*used to records ok/nok to sleep requests from clients*/
typedef enum
{ 
  PMLogSleepLprClient_ID_DIAG, /*0, name: "DIAG_NPA_CLIENT", this is the client of cpu_vdd node*/
  PMLogSleepLprClient_ID_GL1_HW, /*1, name: "gl1_hw_latency_client", this is the client of cpu_vdd node*/
  PMLogSleepLprClient_ID_SLEEP, /*2, Name: "core/power/sleep", most lprs use this name*/
  PMLogSleepLprClient_ID_NPA_SCHEDULER, /*3, Name: "/npa/scheduler/lpr"*/
  PMLogSleepLprClient_ID_XO_CXO, /*4, Name: "/node/xo/cxo", used by MCPM, no source code*/
  PMLogSleepLprClient_ID_CORE_POWER_RPM, /*5, Name: "core/power/rpm"*/
  PMLogSleepLprClient_ID_XO_PXO, /*6, Name: "/node/xo/pxo"*/
  PMLogSleepLprClient_ID_CORE_CPU, /*7, Name: "/node/core/cpu/"*/
  PMLogSleepLprClient_ID_MCPM_SLEEP_LPR, /*8, Name: "mcpm_sleep_lpr_client"*/
  PMLogSleepLprClient_ID_CORE_CPU_VDD, /*9, Name: "/node/core/cpu/vdd", sleep lpr node is dependency of cpu_vdd node*/
  PMLogSleepLprClient_ID_VDD_DIG, /*0xA, Name: "/node/rail/vdd_dig"*/
  PMLogSleepLprClient_ID_OTHERS, /*0xB, otehrs*/
  PMLogSleepLprClient_ID_NUM
}MPMlogSleepLprClientId;

typedef enum
{ 
  MPMLog_OK_SLEEP, /*0, ok to sleep*/
  MPMLog_NOK_SLEEP, /*1, disable sleep*/
  MPMLog_LATEST_REQUEST, /*2, latest/current state*/
  MPMLog_Sleep_Lpr_State_NUM
}MPMlogSleepLprState;

/*This structure will reset to 0, whenever HLOS read it. Please put variable which need reset.
    Otherwise, put at ModemPMLog*/ 
typedef struct
{
  unsigned long int Modem_PM_log_lpm_status[Modem_PM_LOG_LPMSTATUS_NUM];
  ModemPMLogValueStatistics value_statictics[PMLogValue_ID_NUM];
  unsigned long int lpr_client_state[PMLogSleepLprClient_ID_NUM][MPMLog_Sleep_Lpr_State_NUM];
  unsigned long int   *caller_of_diag_sleep_vote;    
  unsigned long int   value_of_diag_ballot_mask;
}ModemPMLogbody;
      
typedef struct
{
  unsigned long int   Modem_PM_Log_reset_count_flag;
  unsigned long int   Modem_PM_Log_size;
  unsigned long int   Modem_PM_Log_version;
  unsigned long int   Modem_PM_Log_reset_time;
  ModemPMLogbody Modem_PM_Log_body;
}ModemPMLog;
/* } Qisda Tina Hsu, 2013/02/21, Add Modem LPM PM log */


/*-------------------------------------------
  Reference document: 80-N7379-4 Rev. F
--------------------------------------------*/
/* This is real reversion number */
typedef enum
{
  ES = 0x1,  // engineering sample
  CS = 0x2,  // commercial sample
  UNKNOWN_CPU_VERSION = 0x7fffffff
} CPU_VERSION_TYPE;
/* } Qisda, Jerry Tseng, 2013/03/26 */

typedef enum
{
  NORMAL_CABLE  = 0x0,
  FACTORY_CABLE = 0x1,
  UNKNOWN_Q_CABLE = 0x7fffffff
} CABLE_TYPE;

typedef enum
{
  BIST_MODE   = 0x0,
  FT_MODE     = 0x1,
  NORMAL_MODE = 0x2,
  UNKNOWN_FACTORY_MODE = 0x7fffffff
} FACTORY_MODE_TYPE;

/* This is real eMMC's vendor ID */
typedef enum
{
  SANDISK_EMMC  = 0x45,
  KINGSTON_EMMC = 0x70,
  SAMSUNG_EMMC  = 0x15,
  MICRON_EMMC   = 0xFE,
  UNKNOWN_EMMC_VENDOR = 0x7fffffff
} EMMC_VENDOR_TYPE;
		
typedef struct
{
  unsigned int     card_size_in_sectors;
  unsigned int     write_protect_group_size_in_sectors;
  EMMC_VENDOR_TYPE vendor_id;
}EMMC_INFO_TYPE;

typedef struct
{
  unsigned char rf_id_flag;          // rf chip version;
  unsigned char rf_id_ver;           // rf chip version;
}RF_VERSION_TYPE;

typedef struct 
{
  BAND_TYPE         band_type;
  BOARD_ID_TYPE     board_id;
  PROJECT_ID_TYPE   project_id;
  int               hw_id_adc_value;
  unsigned int      lpddr2_size_in_MB;
  DDR_VENDOR_TYPE   ddr_vendor;
  CPU_VERSION_TYPE  cpu_version;
  CABLE_TYPE        cable_type;
  FACTORY_MODE_TYPE factory_mode;
  EMMC_INFO_TYPE    emmc_info;
  unsigned char     uart_over_uim1; // 1 means uart over uim1;
  unsigned char     auth_enabled; // 1 means enable secure boot;
}HW_INFO;
   
/* Qisda Mark Hsieh, 2012/12/08, Write SBLx log to RAM buffer { */
typedef struct 
{
  unsigned long int ram_log_buf_addr;
  unsigned long int ram_log_buf_len;
}RAM_LOG_INFO;
/* } Qisda Mark Hsieh, 2012/12/08 */
 
typedef enum 
{
  DEFAULT_NV_RESTORE_STATUS_NOT_READY = 0,
  DEFAULT_NV_RESTORE_STATUS_READY     = 1,
  DEFAULT_NV_RESTORE_STATUS_MAX       = 0x7fffffff,
} default_nv_restore_status_type;

/* Qisda, Jerry Tseng, 2013/01/15, Porting FSG backup check mechanism { */
typedef enum 
{
  FSG_BACKUP_STATUS_NOT_DONE = 0,
  FSG_BACKUP_STATUS_DONE     = 1,
} fsg_backup_status_type;
/* } Qisda, Jerry Tseng, 2013/01/15 */

/* structure definition for 
  SMEM_ID_VENDOR0,
  SMEM_ID_VENDOR1,
  SMEM_ID_VENDOR2,  
  
  Notice!! SMEM space is limited.
  Please remove structure definition not use anymore!!   
*/

/* Vendor ID 0 : data written by AMSS */
typedef struct
{
  unsigned long int modem_reboot_reason; //Qisda JZHou, 2012/05/10, for modem reboot reason
  RF_VERSION_TYPE   rf_id_info;
  int time_zone;
/*Qisda Tina Hsu, 2013/01/25, Add Modem LPM PM log { */ 
   ModemPMLog Modem_PM_Log;   
/* } Qisda Tina Hsu, 2013/01/25, Add Modem LPM PM log */
} smem_vendor_id0_amss_data;
  
/* Vendor ID 1 : data written by APPS */
typedef struct
{
  unsigned char imei[OTP_IMEI_LENGTH];
  unsigned char serial_no[OTP_SERIAL_NO_LENGTH];
  unsigned char bt[OTP_BT_LENGTH];
  unsigned char wifi[OTP_WIFI_LENGTH];
  unsigned char qlock[OTP_QLOCK_LENGTH];
  unsigned char meid[OTP_MEID_LENGTH-1];
  unsigned char tcpports[8192]; // Bright Lee, 20120323, tcp/udp port monitor 
  unsigned char udpports[8192]; // Bright Lee, 20120323, tcp/udp port monitor 
  unsigned char ipforwarding; // Bright Lee, 20120429, tcp/udp port monitor	
} smem_vendor_id1_apps_data;
  
/* Vendor ID 2 : data written by Boot loader */
typedef struct
{    
  HW_INFO  hw_info;
  default_nv_restore_status_type default_nv_restore_status; //Qisda JZHou, 2011/11/14, for aboot continue condition
  /* Qisda Mark Hsieh, 2012/12/08, Write SBLx log to RAM buffer { */
  RAM_LOG_INFO ram_log_info;
  /* } Qisda Mark Hsieh, 2012/12/08 */
  /* Qisda, Jerry Tseng, 2013/01/15, Porting FSG backup check mechanism { */
  fsg_backup_status_type FSG_backup_flag;
  /* } Qisda, Jerry Tseng, 2013/01/15 */
}smem_vendor_id2_bl_data;

#ifdef __cplusplus
}
#endif  
#endif /* _OEM_SMEM_STRUCT_H_ */

