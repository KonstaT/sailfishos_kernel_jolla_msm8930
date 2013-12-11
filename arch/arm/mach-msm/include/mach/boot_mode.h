/*******************************************************************************
                                                                                
                      Qisda Handset Project                                     
                                                                                
                     Copyright (c) 2009 Qisda Corpration                        
                                                                                
File Description                                                                
----------------                                                                
                                                                                
when        who            what, where, why                                     
                                                                                
----------  ------------   ----------------------------------------------------
2013/11/01  PhoebeChen  [NON_HLOS][SYS]save board-id info for RPM.
2013/05/13  MarkHsieh   [NON_HLOS][L1] Define DDR address for keeping modem error information
2013/03/01  MarkHsieh   [NON_HLOS][L1] Enter PBL eDL mode by fastboot command
2013/02/26  MarkHsieh   [NON_HLOS][L1] Output OEM boot-up log according to eMMC flag
2013/02/18  MarkHsieh   [NON_HLOS][L1] Define IRAM address for SBLx error information
2013/02/08  MarkHsieh   [NON_HLOS][L1] Append SBL error info to eMMC sector and IRAM
2013/02/01  MarkHsieh  [NON_HLOS][L1] Define sector for keeping eMMC multi-download flag
2013/01/31  MarkHsieh   [NON_HLOS][L1] Add feature to write log to eMMC according to restart reason
2013/01/23  MarkHsieh   [NON_HLOS][L1] Define starting sector for eMMC log
2013/01/22  MarkHsieh   [NON_HLOS][L1] Add feature to write boot-up log to eMMC
2013/01/17  JZ Hou            Boston 1.0:[NON_HLOS][L1] Add boot mode flag for reboot to blow HWIO_OVERRIDE_4_ADDR to reenable JTAG port
2013/01/17  MarkHsieh   Boston 1.0:[NON_HLOS][L1] Add function to do IRAM verificatoin
2013/01/03  JZ Hou            Boston 1.0:[NON_HLOS][L1] Add boot mode flag for reboot to SBL1 charging
2011/01/04  Mark Hsieh        Chicago ICS:[Modem][L1] Add boot mode flag for multi-download
********************************************************************************/ 

#ifndef __BOOT_MODE_H
#define __BOOT_MODE_H

#define FASTBOOT_MODE       0x77665500
#define NONE_MODE           0x77665501		//nothing, so reboot normal
#define RECOVERY_MODE       0x77665502
#define LOW_BATT_MODE       0x77665503
#define USER_REBOOT         0x77665510
#define REBOOT_PWRKEY_MODE  0x7766aa04		//offcharge, then long press power key
#define REBOOT_CHARGE_MODE  0x7766aa05		//reboot to poweroff charge
#define FASTBOOT_REBOOT_MODE    0x7766aa06		//reboot when stop at fastboot
#define RECOVERY_REBOOT_MODE    0x7766aa08		//add this for factory do factory reset reboot check

#define EUU_HEX_2_FASTBOOT  0x7766aa0e		//hex jump to fastboot for EUU
#define RD_HEX_2_FASTBOOT   0x7766aa0f		//hex jump to fastboot for RD/Factory download
#define SD_RECOVERY_MODE    0x7766aa09		//SD download for recovery
#define MENU_RECOVERY_MODE  0x7766aa0A		//recovery menu
#define ABNORMAL_REBOOT     0x7766aa12		//abnormal reboot
#define KERNEL_REBOOT_FOR_CPO   0x7766aa16      // the reboot is one from kernel for CPO test.
#define REBOOT_FOR_DDRTEST  0x7766aa18      // need this reboot reason let reduce memory size for kernel use
#define REBOOT_FOR_KMEMLEAK_DETECTION   0x7766aa1a // enable the kmemleak detection for memory leakage detectin in kernel
/* Qisda Mark Hsieh, 2011/01/04, Add boot mode flag 0x7766aa20~0x7766aa2Fffor multi-download { */
#define MULTI_DL_FASTBOOT_MODE_HS1      0x7766aa20
#define MULTI_DL_FASTBOOT_MODE_HS2      0x7766aa21
#define MULTI_DL_FASTBOOT_MODE_HS3      0x7766aa22
#define MULTI_DL_FASTBOOT_MODE_HS4      0x7766aa23
#define MULTI_DL_FASTBOOT_MODE_HS5      0x7766aa24
#define MULTI_DL_FASTBOOT_MODE_HS6      0x7766aa25
#define MULTI_DL_FASTBOOT_MODE_HS7      0x7766aa26
#define MULTI_DL_FASTBOOT_MODE_HS8      0x7766aa27
/* } Qisda Mark Hsieh, 2011/01/04 */

/* Qisda JZHou, 2012/12/27, Add boot mode flag for reboot to SBL1 charging { */
#define REBOOT_SBL1_CHARGING   	0x7766aa28
/* } Qisda JZHou, 2012/12/27 */
/* Qisda JZHou, 2013/01/17, add boot mode for sblx usage { */
#define REBOOT_SBL3_BLOW_OVERRIDE_4   	0x7766aa29  //reenable JTAG port. need this feature after EVT2-1 board id if need JTAG debug.
/* } Qisda JZHou, 2013/01/17 */

#define ENABLE_SECURE_BOOT      0x7766aa30
#define ENABLE_SBL2_DDRTEST     0x7766aa31
#define ENABLE_BIST_MODE        0x7766aa32
#define ENTER_MASS_STORAGE_MODE 0x7766aa33
/* Qisda Mark Hsieh, 2013/01/17, Add function to do IRAM verificatoin { */
#define ENABLE_IRAM_VERIFICATION        0x7766aa34
/* } Qisda Mark Hsieh, 2013/01/17 */
/* Qisda Mark Hsieh, 2013/01/22, Add feature to write boot-up log to eMMC { */
#define ENABLE_EMMC_LOG         0x7766aa35
/* } Qisda Mark Hsieh, 2013/01/22 */
/* Qisda Mark Hsieh, 2013/03/01, Enter PBL eDL mode by fastboot command { */
#define ENTER_PBL_EDL           0x7766aa36
/* } Qisda Mark Hsieh, 2013/03/01 */
/* Qisda Okpa, 2013/10/22, workaround for jolla CSD. Not enter FTD mode when factory cable connected. { */
#define NO_FACTORY_MODE         0x7766aa37
/* } Qisda Okpa */
/* Bright Lee, 20120207, work around for unknown bootup reason, backup restart_reason in ddrram { */
#define RESTART_REASON_MAGIC    0xA4BC385F
/* } Bright Lee, 20120207 */
#define DLOAD_MAGIC_NUM1	0xE47B337D
#define DLOAD_MAGIC_NUM2	0xCE14091A
#define DLOAD_MAGIC_NUM3	0xAABBCCDD

#define RECOVERY_MODE_STR       "recovery"
#define FASTBOOT_MODE_STR       "fastboot"
#define USER_REBOOT_STR         "user-reboot"
#define FASTBOOT_REBOOT_MODE_STR    "fastboot-reboot"
#define RECOVERY_REBOOT_MODE_STR    "rcvyReboot"
#define REBOOT_PWRKEY_MODE_STR  "pwrkey-on"			//offcharge, then long press power key
#define REBOOT_CHARGE_MODE_STR  "charge"			//reboot to poweroff charge
#define NONE_MODE_STR           "normal"			//nothing, so reboot normal
#define EUU_HEX_2_FASTBOOT_STR  "euuhex2fastboot"		//hex jump to fastboot for euu
#define RD_HEX_2_FASTBOOT_STR   "rdhex2fastboot"	//hex jump to fastboot for rd/factory
#define SD_RECOVERY_MODE_STR    "sd-recovery"		//SD download for recovery
#define MENU_RECOVERY_MODE_STR  "menu-recovery"		//recovery menu
#define ABNORMAL_REBOOT_STR     "abnormal"		//abnormal reboot
#define KERNEL_REBOOT_FOR_CPO_STR   "CPO-test"
#define REBOOT_FOR_DDRTEST_STR  "ddrtest"			// need this reboot reason let reduce memory size for kernel use
#define REBOOT_FOR_KMEMLEAK_DETECTION_STR   "kmemleak"
#define ENABLE_SECURE_BOOT_STR      "enable_secure_boot"
#define ENABLE_SBL2_DDRTEST_STR     "enable_sbl2_ddrtest"
#define ENABLE_BIST_MODE_STR        "enable_bist_mode"
#define ENTER_MASS_STORAGE_MODE_STR "enter_mass_storage_mode"
/* Qisda Mark Hsieh, 2013/03/01, Enter PBL eDL mode by fastboot command { */
#define ENTER_PBL_EDL_STR           "enter_pbl_edl" // Enter PBL emergency download mode
/* } Qisda Mark Hsieh, 2013/03/01 */
#define MULTI_DL_FASTBOOT_MODE_HS1_STR      "mbootloader1"
#define MULTI_DL_FASTBOOT_MODE_HS2_STR      "mbootloader2"
#define MULTI_DL_FASTBOOT_MODE_HS3_STR      "mbootloader3"      
#define MULTI_DL_FASTBOOT_MODE_HS4_STR      "mbootloader4"     
#define MULTI_DL_FASTBOOT_MODE_HS5_STR      "mbootloader5"    
#define MULTI_DL_FASTBOOT_MODE_HS6_STR      "mbootloader6"   
#define MULTI_DL_FASTBOOT_MODE_HS7_STR      "mbootloader7"  
#define MULTI_DL_FASTBOOT_MODE_HS8_STR      "mbootloader8" 
/* Qisda Okpa, 2013/10/22, workaround for jolla CSD. Not enter FTD mode when factory cable connected. { */
#define NO_FACTORY_MODE_STR         "no_factory_mode"
/* } Qisda Okpa */

// string and boot-reason pair
#define BOOT_INFO_ITEM(x)   {x##_STR, x}

// string and boot-reason pair array
#define BOOT_INFO_TABLE \
    static struct { \
        char *str_reason; \
        unsigned int reason; \
    } bootup_reasons[] = { \
        BOOT_INFO_ITEM(RECOVERY_MODE), \
        BOOT_INFO_ITEM(FASTBOOT_MODE), \
        BOOT_INFO_ITEM(USER_REBOOT), \
        BOOT_INFO_ITEM(FASTBOOT_REBOOT_MODE), \
        BOOT_INFO_ITEM(REBOOT_PWRKEY_MODE), \
        BOOT_INFO_ITEM(REBOOT_CHARGE_MODE), \
        BOOT_INFO_ITEM(NONE_MODE), \
        BOOT_INFO_ITEM(EUU_HEX_2_FASTBOOT), \
        BOOT_INFO_ITEM(RD_HEX_2_FASTBOOT), \
        BOOT_INFO_ITEM(SD_RECOVERY_MODE), \
        BOOT_INFO_ITEM(MENU_RECOVERY_MODE), \
        BOOT_INFO_ITEM(ABNORMAL_REBOOT), \
        BOOT_INFO_ITEM(KERNEL_REBOOT_FOR_CPO), \
        BOOT_INFO_ITEM(REBOOT_FOR_DDRTEST), \
        BOOT_INFO_ITEM(REBOOT_FOR_KMEMLEAK_DETECTION), \
        BOOT_INFO_ITEM(RECOVERY_REBOOT_MODE), \
        BOOT_INFO_ITEM(ENABLE_SECURE_BOOT), \
        BOOT_INFO_ITEM(ENABLE_SBL2_DDRTEST), \
        BOOT_INFO_ITEM(ENABLE_BIST_MODE), \
        BOOT_INFO_ITEM(ENTER_MASS_STORAGE_MODE), \
        BOOT_INFO_ITEM(ENTER_PBL_EDL), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS1), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS2), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS3), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS4), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS5), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS6), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS7), \
	BOOT_INFO_ITEM(MULTI_DL_FASTBOOT_MODE_HS8), \
	BOOT_INFO_ITEM(NO_FACTORY_MODE), \
    }

/* Qisda Mark Hsieh, 2013/02/26, Output OEM boot-up log according to eMMC flag { */
#define EMMC_FLAG_UART_VERBOSE  "VERBOSE" 
#define EMMC_FLAG_UART_SILENT  "SILENT" 

#define EMMC_FLAG_SECTOR  8099
/* } Qisda Mark Hsieh, 2013/02/26 */
/* Qisda Mark Hsieh, 2013/02/08, Append SBL error info to eMMC sector and IRAM { */    
/* Qisda Mark Hsieh, 2013/01/31, Add feature to write log to eMMC according to restart reason { */
#define EMMC_ERR_INFO_SECTOR 8100          // Keep boot-up failure log written by SW error handler
#define EMMC_ERR_LOG_STARTING_SECTOR 8101  // Keep boot-up failure log written by SW error handler
#define EMMC_ERR_LOG_END_SECTOR 8169       // Keep boot-up failure log written by SW error handler
/* Qisda Mark Hsieh, 2013/01/23, Define starting sector for eMMC log { */
#define EMMC_LOG_STARTING_SECTOR 8170      // Keep boot-up log, triggerred by restart reason 'ENABLE_EMMC_LOG'
#define EMMC_LOG_END_SECTOR 8190           // Keep 8KB for storing eMMC log
/* } Qisda Mark Hsieh, 2013/01/23 */
/* } Qisda Mark Hsieh, 2013/01/31 */

/* Qisda Mark Hsieh, 2013/02/01, Define sector for keeping eMMC multi-download flag { */
#define EMMC_MULTI_DL_FLAG_SECTOR 8191
/* } Qisda Mark Hsieh, 2013/02/01 */
/* } Qisda Mark Hsieh, 2013/02/08 */

/* Qisda Mark Hsieh, 2013/02/18, Define IRAM address for SBLx error information { */    
#define RPM_CODE_RAM_SBL_ERR_INFO_ADDR (volatile uint32 *)(0x0002F000)
/* } Qisda Mark Hsieh, 2013/02/18 */    

/* Qisda Phoebe.Chen 20131101, save board-id info for RPM{ */    
#define SS_DEBUG_TRIG_LPASS  ((volatile uint32 *)(0x2a03f93C))
/* } Qisda Phoebe.Chen, 20131101 */ 
/* Qisda Mark Hsieh, 2013/05/13, Define DDR address for keeping modem error information { */    
#define DDR_MODEM_ERR_INFO_ADDR ((volatile uint32 *)(0xBFEFE000))
/* } Qisda Mark Hsieh, 2013/05/13 */    

#endif
