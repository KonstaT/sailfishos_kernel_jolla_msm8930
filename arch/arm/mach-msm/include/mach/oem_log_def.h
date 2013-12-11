/*******************************************************************************
                                                                                
                      Qisda Handset Project                                     
                                                                                
                     Copyright (c) 2011 Qisda Corpration                        
                                                                                
File Description                                                                
----------------                                                                
                                                                                
when        who            what, where, why                                     
                                                                                
----------  ------------   ----------------------------------------------------                                                                                
2013/04/17  NoahYang        Boston 1.0:   [Modem][SYS] Add qmi autotest (modem part)
2013/04/16  NoahYang        Boston 1.0:   [Modem][SYS] Fix EFS buffer mapping base and size
2013/04/15  NoahYang        Boston 1.0:   [Modem][SYS] Move EFS buffer mapping base because Qualcomm release
2012/12/10  NoahYang        Boston 1.0:   [Modem][SYS] Dump EFS log API for HLOS (move to current loation of Boston)
2012/05/17  NoahYang        Detroit LA1P5:[Modem][SYS] Dump EFS log API for HLOS
2012/05/14  NoahYang        Detroit LA1P5:[Modem][SYS] add sys command interface
2012/05/18  TerryCheng     Detroit LA1P5:[HLOS][BSP] Add log buffer map size define
********************************************************************************/ 

#ifndef OEM_LOG_DEF_H
#define OEM_LOG_DEF_H

/* QISDA  Noah Yang, 2011/11/10, add uint32/64 define for HLOS { */ 
#ifndef _UINT64_DEFINED
typedef  unsigned long long uint64;
#define _UINT64_DEFINED
#endif
#ifndef _UINT32_DEFINED
typedef  unsigned long int  uint32;
#define _UINT32_DEFINED
#endif
/* } QISDA  Noah Yang, 2011/11/10 */


/* QISDA  Noah Yang, 2013/4/16, Fix EFS buffer mapping base and size { */
#define LOG_BUFFER_MAPPING_BASE (0x8af00000)

/* Terry Cheng, 20120518, Add LOG buffer size define {*/
#define LOG_BUFFER_MAPPING_SIZE (0x00400000)
/* } Terry Cheng, 20120518 */
/* } QISDA  Noah Yang, 2013/4/16 */

/* structure for EFS file saving buffer */
/* QISDA  Noah Yang, 2012/2/17, Redmine# 1880 ; Extend f3 trace buffer size and exp log buffer size { */ 
#define MAX_EFS_FILE_SIZE 300000  
/* } QISDA  Noah Yang, 2012/2/17 */

/* max filename, path length */
#define MAX_EFS_NAME_LENGTH  50
#define MAX_EFS_PATH_LENGTH  50

/* max name list, each separated by '#' */
#define MAX_EFS_LIST_LEN 1000

/* EFS file content */
typedef struct {
	char 	   name[MAX_EFS_NAME_LENGTH];
	uint32 	 size;                     /* size in bytes */
	char     buffer[MAX_EFS_FILE_SIZE+1];
}LogBufferInfo;

/* EFS directory content */
typedef struct {
	char 	    path[MAX_EFS_PATH_LENGTH];
	uint32 	  fileNum; 
	char 	    fileList[MAX_EFS_LIST_LEN+1];	    
	uint32 	  dirNum; 
	char 	    dirList[MAX_EFS_LIST_LEN+1];	
}LogFolderInfo;

/* Qisda Noah Yang, 2013/4/17, Add qmi autotest (modem part)  { */
enum 
{
	SYS_ENG_CMD,          /* eng mode command,  0: force phone crash (9999)  1: test cmd  */	      
	SYS_UPDATE_TIME_ZONE, /* timezone update,   0: get                       1: set */  
        SYS_UPDATE_RESET_OPT, /* amss reboot option 0: get                       1: set */
	SYS_QMI_ASYNC_TEST        =10,       /* qmi async. test */  	
        SYS_QMI_ASYNC_IND_TEST    =11,       /* qmi indication test */	
        SYS_DUMP_AMSS_DATA_TEST   =12,       /* get amss data test */
	SYS_LAST_CMD
};
/* } Qisda Noah Yang, 2013/4/17 */

#endif /* ! OEM_LOG_DEF_H */
