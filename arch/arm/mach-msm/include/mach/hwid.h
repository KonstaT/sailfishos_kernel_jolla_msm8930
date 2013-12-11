#ifndef __HWID_H_
#define __HWID_H_

#include <mach/oem_smem_struct.h>
/* Terry Cheng, 20120831, Port system_rev when porting JB {*/
#include <asm/system_info.h>
/* } Terry Cheng, 20120831, Port system_rev when porting JB */

/* Terry Cheng, 20120215,  Add for Qisda Project id got share memory {*/
extern int msm_project_id;
/* }Terry Cheng, 20120215,  Add for Qisda Project id got share memory */

extern unsigned QcableVar;
extern unsigned QfactoryVar;
#define QcableFACTORY  0x66616374
#define QfactoryBIST   0x62697374
#define QfactoryFT     0x20206674

#endif
