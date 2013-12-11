#ifndef _ASM_GENERIC_EMERGENCY_RESTART_H
#define _ASM_GENERIC_EMERGENCY_RESTART_H

static inline void machine_emergency_restart(void)
{
	/* Bright Lee, 20120517, reboot log { */
	#ifdef CONFIG_PANIC_LASTLOG
	machine_restart("abnormal");
	#else
	machine_restart(NULL);
	#endif
	/* } Bright Lee, 20120517 */
}

#endif /* _ASM_GENERIC_EMERGENCY_RESTART_H */
