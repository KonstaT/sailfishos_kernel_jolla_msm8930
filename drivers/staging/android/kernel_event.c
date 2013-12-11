#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/kevent.h>
//Terry Cheng, provide active wakelock for logservice to handle
#include <linux/wakelock.h>
#include <linux/debugfs.h>

#define MAX_LEN_BUFFER	256
#define UEVENT_WAKELOCK_NAME	"WAKELOCKS="

static DEFINE_SPINLOCK(kevent_lock);
static unsigned	events_map = 0;
static bool initialized = 0;
static unsigned long event_counts[KEVENT_COUNT] = {0};
static char* wakelock_buffer = NULL; //Terry Cheng, provide active wakelock for logservice to handle

//Terry Cheng, 20130603, Add debug mode for test 
static char* wakelock_test_name=NULL;

static char *event_oops[] = {
	"EVENT=Oops",0
};
static char *event_modem_crash[] = {
	"EVENT=ModemCrash",0
};
/* Terry Cheng, 20100818, Add ADSP crash event {*/
static char *event_adsp_crash[] = {
	"EVENT=ADSPCrash",0
};
static char *event_dump_wakelock[] = {
	"EVENT=WakelockDump",0
};
//Terry Cheng, 20120807, notify logmaster riva crash.
static char *event_riva_crash[] = {
	"EVENT=RIVACrash",0
};
/* Bright Lee, 20130322, add uevent for modem smem log without ramdump { */
static char *event_modem_smem_log[] = {
	"EVENT=ModemCrashNoRamdump",0
};
/* } Bright Lee, 20130322 */
/* Terry Cheng, 20121019, detect abnormal current consumption behavior { */
static char *event_pm_abnormal[] = {
	"EVENT=Resource",0
};
//Terry Cheng, 20121031, Add abnormal subsystem event
static char *event_subsystem_abnormal[] = {
	"EVENT=Subsystem",0
};
//Vincent Ying, 20130517, Add abnormal subsystem event
static char *evnet_pm_alarm_count[] = {
	"EVENT=AlarmCount",0
};


static char **event_string[] = {
	event_oops,
	event_modem_crash, 
	event_adsp_crash,
	event_dump_wakelock,
	event_riva_crash,	//Terry Cheng, 20120807, notify logmaster riva crash.
	event_modem_smem_log, // Bright Lee, 20130322, add uevent for modem smem log without ramdump
	event_subsystem_abnormal,	//Terry Cheng, 20121031, Add abnormal subsystem event
	evnet_pm_alarm_count,//Vincent Ying, 20130517, add alarm event.
	event_pm_abnormal, 
	event_pm_abnormal, 
	event_pm_abnormal, 
	event_pm_abnormal, 
};

static char *component_camera[] = {
	"COMPONENT=Camera",0
};
static char *component_backlight[] = {
	"COMPONENT=Backlight",0
};
static char *component_lcd[] = {
	"COMPONENT=LCD",0
};
static char *component_touch[] = {
	"COMPONENT=Touch",0
};
static char **component_string[] = {
	component_camera,
	component_backlight, 
	component_lcd,
	component_touch,
};
/* }Terry Cheng, 20121019, detect abnormal current consumption behavior */

struct kevent_device{
        struct miscdevice       miscdevice;
        struct workqueue_struct *workqueue;
        struct work_struct      work;
	struct proc_dir_entry	*proc;
};

static int kevent_proc_read(char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	int len=0;
	int i;
	for(i=0;i<KEVENT_COUNT;i++) {
		len += sprintf(page+len, "%lu ", event_counts[i]);
	}
	*(page+len-1)='\n';

	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}

void kevent_work(struct work_struct *work)
{
	struct kevent_device	*device =
		container_of(work, struct kevent_device, work);
	int	i;
	unsigned long 	flags;
	unsigned	events;
	ssize_t size = 0;
	ssize_t size_active_lock = 0;//Terry Cheng, 20130103, Check whether need to ignore empty wacklok 
	char *uevent_env[3] = { NULL, NULL, NULL};

repeat:
	spin_lock_irqsave(&kevent_lock,flags);
	events = events_map;
	events_map = 0;
	spin_unlock_irqrestore(&kevent_lock,flags);

	if (events) {
		for(i=0;events;i++) {
			if (events & 1) {

				printk(KERN_INFO "%s: send kobject uevent, %s\n",__func__, event_string[i][0]);
				uevent_env[0] = event_string[i][0];	
				//Terry Cheng, provide active wakelock for logservice to handle
				if (i == KEVENT_DUMP_WAKELOCKS)
				{
					if(!wakelock_buffer)
						wakelock_buffer = kmalloc(MAX_LEN_BUFFER, GFP_KERNEL);
					if (!wakelock_buffer) {
						pr_err("Cannot alloc wakelock_buffer\n");
					}
					else
					{
						size = snprintf(wakelock_buffer, MAX_LEN_BUFFER, "%s", UEVENT_WAKELOCK_NAME); 
						//Terry Cheng, 20130103, Check whether need to ignore empty wacklok 
						if(!wakelock_test_name)
						{
							size_active_lock = get_active_suspend_locks(wakelock_buffer+size, MAX_LEN_BUFFER-size);
							//Ignore empty wakelock. 
							if(!size_active_lock ){
								events >>= 1;
								continue;
							}
						}
						else
						{
							size += snprintf(wakelock_buffer+size, MAX_LEN_BUFFER-size, "%s", wakelock_test_name); 
						}	
						wakelock_buffer[size+size_active_lock] =  '\0';
						uevent_env[1] = wakelock_buffer;
					}
				}
				//Terry Cheng, provide abnormal component for logservice to handle
				else if ( i >=  KEVENT_COMP_CAMERA)
				{
					uevent_env[1] = component_string[i -KEVENT_COMP_CAMERA][0];
				}
				if(uevent_env[1])
					printk(KERN_INFO "uevent_env[1], %s\n", uevent_env[1]);
				
				kobject_uevent_env(
					&device->miscdevice.this_device->kobj,
					KOBJ_CHANGE,
					uevent_env
				);
			}
			events >>= 1;
		}
		if (wakelock_buffer) 
		{
			kfree(wakelock_buffer);
			wakelock_buffer = NULL;
		}
		goto repeat;
	}

}

static int kevent_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int kevent_release(struct inode *inode , struct file *file)
{
	return 0;
}

static struct file_operations kevent_fops = {
	.owner = THIS_MODULE,
	.open = kevent_open,
	.release = kevent_release,
};

static struct kevent_device	kevent_device = {
	.miscdevice = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "kevent",
		.fops = &kevent_fops,
		.parent = NULL,
	},
};

int kevent_trigger(unsigned event_id)
{
	unsigned long	flags;

	if (event_id > KEVENT_COUNT) return -1;

	spin_lock_irqsave(&kevent_lock,flags);
	if (!initialized) {
		spin_unlock_irqrestore(&kevent_lock,flags);
		return -1;
	}
	/* Terry Cheng, 20130508, Filter out abnormal power usage with same resaon since this reason need to reboot to recover {*/
	if( (event_counts[event_id] <= 2) || event_id <  KEVENT_ABNORMAL_SUBSYTEM){
		events_map |= (1<<event_id);
		event_counts[event_id]++;
	}	

	printk(KERN_INFO "%s: queue event work event id is %d\n", __func__, event_id);
	if( event_counts[event_id] >= 1)
		queue_work(kevent_device.workqueue, &kevent_device.work);
	/* } Terry Cheng, 20130508, Filter out abnormal power usage with same resaon since this reason need to reboot to recover */

	spin_unlock_irqrestore(&kevent_lock,flags);

	return 0;
}
EXPORT_SYMBOL(kevent_trigger);

/* Terry Cheng, Add debugfs to test kevent trigger { */
#if defined(CONFIG_DEBUG_FS)
//Config 
static int kevent_trigger_set(void *data, u64 val)
{
	unsigned int trigger_type = val;

	pr_info("kevent_trigger_set trigger_type = %d\n", trigger_type);

	if (trigger_type < KEVENT_COUNT)
		kevent_trigger(trigger_type);
	
	return 0;
}
static int kevent_trigger_get(void *data, u64 *val)
{
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(kevent_trigger_fops, kevent_trigger_get,
						kevent_trigger_set, "%llu\n");
/* Terry Cheng, 20130603, Add debug mode for test {*/
static ssize_t wakelock_read_file(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	int ret = 0;
	if(wakelock_test_name)
		ret = simple_read_from_buffer(user_buf, count, ppos, wakelock_test_name, strlen(wakelock_test_name));
	return ret;
}

static ssize_t wakelock_write_file(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *ppos)
{

	ssize_t buf_size;
	if(!wakelock_test_name)
		wakelock_test_name = kmalloc(MAX_LEN_BUFFER, GFP_KERNEL);
	if (!wakelock_test_name) {
		pr_err("Cannot alloc wakelock_test_name\n");
	}
	buf_size = min(count, (size_t)(MAX_LEN_BUFFER-1));
	if (copy_from_user(wakelock_test_name, user_buf, buf_size)) {
		pr_err("Failed to copy from user\n");
		return -EFAULT;
	}
	wakelock_test_name[buf_size] = 0;
	pr_err("wakelock_buffer %s\n", wakelock_test_name);

	return buf_size;
}
static const struct file_operations kevent_wakelock_fops = {
	.open = simple_open,
	.read = wakelock_read_file,
	.write = wakelock_write_file,
};
/* } Terry Cheng, 20130603, Add debug mode for test */

#endif 	//CONFIG_DEBUG_FS
/* } Terry Cheng, Add debugfs to test kevent trigger  */

static int __init kevent_init(void)
{
	int		ret;
	unsigned long	flags;
	
/* Terry Cheng, Add debugfs to test kevent trigger { */
#if defined(CONFIG_DEBUG_FS)

	struct dentry *dent;
	dent = debugfs_create_dir("kevent", 0);
	if (IS_ERR(dent))
		return 0;

	//Create debugfs file
	debugfs_create_file("trigger", 0644, dent, 0, &kevent_trigger_fops);
	//Terry Cheng, 20130603, Add debug mode for test 
	debugfs_create_file("wakelock", 0644, dent, 0, &kevent_wakelock_fops);
#endif 	//CONFIG_DEBUG_FS
/* } Terry Cheng, Add debugfs to test kevent trigger  */

	ret = misc_register(&kevent_device.miscdevice);
	if( unlikely(ret)) {
		printk(KERN_ERR "%s: failed to register misc \n",__func__);
		goto failed_register;
	}

	kevent_device.proc = create_proc_entry("kevent_counts", S_IRUSR|S_IRGRP,NULL);
	if (!kevent_device.proc) {
		printk(KERN_ERR "%s: failed to create proc entry\n",__func__);
		ret = -ENOMEM;
		goto failed_proc;
	}
	kevent_device.proc->read_proc = kevent_proc_read;

	kevent_device.workqueue = create_singlethread_workqueue("keventd");
	if (!kevent_device.workqueue) {
		printk(KERN_ERR "%s: failed to create workqueue\n",__func__);
		ret = -ENOMEM;
		goto failed_workqueue;
	}

	INIT_WORK(&kevent_device.work,kevent_work);

	spin_lock_irqsave(&kevent_lock,flags);
	initialized = 1;
	spin_unlock_irqrestore(&kevent_lock,flags);

	return 0;

failed_workqueue:
	remove_proc_entry("kevent_counts",NULL);
failed_proc:
	misc_deregister(&kevent_device.miscdevice);
failed_register:

	return ret;
}

static void __exit kevent_exit(void)
{
	unsigned long	flags;
	spin_lock_irqsave(&kevent_lock,flags);
	initialized = 1;
	spin_unlock_irqrestore(&kevent_lock,flags);

	misc_deregister(&kevent_device.miscdevice);

	flush_workqueue(kevent_device.workqueue);

	destroy_workqueue(kevent_device.workqueue);
	//Terry Cheng, 20130603, Add debug mode for test 
	if(wakelock_test_name)
		kfree(wakelock_test_name);
}

module_init(kevent_init);
module_exit(kevent_exit);
