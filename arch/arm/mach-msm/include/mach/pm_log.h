#ifndef __PM_LOG_H_
#define __PM_LOG_H_

struct pmlog_device {
	struct list_head pmlog_list;
	struct device *dev;
	struct timespec start_time;
	struct timespec run_time;
	unsigned long count;
	unsigned int index;	//Terry Cheng, 20120525, Support multiple register
};
/* Terry Cheng, 20120119, Refine pmlog driver {*/
enum pm_log_debug_level {
	PMLOG_DBG_ERR 	= 0,
	PMLOG_DBG_TRACE 	= 1,
};
enum pm_move_path_type {
	PMLOG 	= 0,
	POWEROFF 	= 1,
};
typedef enum {
	ALIGN_LEFT = 0,
	ALIGN_CENTER,
	ALIGN_RIGHT,
} ALIGNMENT;
/* } Terry Cheng, 20120119, Refine pmlog driver */

#ifdef CONFIG_PM_LOG
//For PM API to update wake time and deep time statistics
void pmlog_update_suspend(unsigned long);
void pmlog_update_wakeup(unsigned long);
void pmlog_update_status(int, unsigned long);
int pmlog_device_on(struct pmlog_device *node);
int pmlog_device_off(struct pmlog_device *node);
struct pmlog_device* pmlog_register_device(struct device *dev);
void pmlog_unregister_device(struct pmlog_device *dev);
void pmlog_trigger_abnormal_earlysuspend_components(void);//20121102, Terry Cheng, Check whether abnormal compoents usage
void pmlog_check_camera_components(void);//20121102, Terry Cheng, Check whether abnormal camera compoents usage
int pmlog_check_audio_components(void);	//20121102, Terry Cheng, Check audio devices whether using 
#else
static inline void pmlog_update_suspend(unsigned long){ }
static inline void pmlog_update_wakeup(unsigned long){}
static inline void pmlog_update_status(unsigned long){ }
static inline int pmlog_device_on(struct pmlog_device *dev) { return 0; }
static inline int pmlog_device_off(struct pmlog_device *dev) { return 0; }
static inline struct pmlog_device* pmlog_register_device(struct device *dev) { return NULL; }
static inline void pmlog_unregister_device(struct pmlog_device *dev) { }
static inline void pmlog_trigger_abnormal_earlysuspend_components(void){}//20121102, Terry Cheng, Check whether abnormal compoents usage
void pmlog_check_camera_components(void){}//20121102, Terry Cheng, Check whether abnormal camera compoents usage
int pmlog_check_audio_components(void){}	//20121102, Terry Cheng, Check audio devices whether using 
#endif	//CONFIG_PM_LOG

/* Terry Cheng, 20120822, Improve pmlog to save cpu pm stats {*/
#ifdef CONFIG_MSM_IDLE_STATS
int msm_pm_stats_dump(char *buffer, int bufer_size, int index);
#endif	//CONFIG_MSM_IDLE_STATS
/* }Terry Cheng, 20120822, Improve pmlog to save cpu pm stats */

/* Terry Cheng, 20121011, Log backlight using time {*/
#if defined(CONFIG_FB_MSM_MIPI_DSI_TRULY_OTM9608A) || defined(CONFIG_FB_MSM_MIPI_DSI_TRUST_NT35516)
ssize_t dump_bl_using_time(char *buf, int bufer_size);
#endif		//CONFIG_FB_MSM_MIPI_DSI_TRULY_OTM9608A
/* } Terry Cheng, 20121011, Log backlight using time */

#endif	//__PM_LOG_H_
