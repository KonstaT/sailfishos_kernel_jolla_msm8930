#ifndef __KEVENT_H_
#define	__KEVENT_H_

enum {
	KEVENT_OOPS = 0,
	KEVENT_MODEM_CRASH,
	KEVENT_ADSP_CRASH,
	KEVENT_DUMP_WAKELOCKS,
	KEVENT_RIVA_CRASH,	//Terry Cheng, 20120807, notify logmaster riva crash.
	KEVENT_MODEM_SMEM_LOG,  // Bright Lee, 20130322, add uevent for modem smem log without ramdump
	KEVENT_ABNORMAL_SUBSYTEM,		//Terry Cheng, 20121031, Add abnormal subsystem event
	KEVENT_ALARM_COUNT,	//Vincet Ying, 20130516, For Abnormal component usage
	KEVENT_COMP_CAMERA,	//Terry Cheng, 20121023, For Abnormal component usage
	KEVENT_COMP_BACKLIGHT,	//Terry Cheng, 20121023, For Abnormal component usage
	KEVENT_COMP_LCD,		//Terry Cheng, 20121023, For Abnormal component usage
	KEVENT_COMP_TOUCH,	//Terry Cheng, 20121023, For Abnormal component usage
	KEVENT_COUNT
};

#ifdef CONFIG_ANDROID_KERNEL_EVENT_DRIVER
int kevent_trigger(unsigned event_id);
#else
#define kevent_trigger
#endif

#endif
