#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/poll.h>

#include <mach/oem_log_def.h>

#define	IS_PAGE_ALIGNED(addr)	(!((addr) & (~PAGE_MASK)))

#define MSM_MODEM_MEM_PHYS LOG_BUFFER_MAPPING_BASE
#define MSM_MODEM_MEM_SIZE LOG_BUFFER_MAPPING_SIZE


static int modem_logger_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long	addr;
	unsigned long	length = vma->vm_end - vma->vm_start;

	printk ("%s vm_start:%lX vm_pgoff%lX size:%lX\n", __func__, vma->vm_start, vma->vm_pgoff, length);

	if (!IS_PAGE_ALIGNED(length) || length > MSM_MODEM_MEM_SIZE) {
		printk(KERN_ERR "%s: Invalid length.\n", __func__);
		return -EINVAL;
	}

	if (vma->vm_flags & VM_WRITE) {
		printk(KERN_ERR "%s: Permission denied.\n",__func__);
		return -EPERM;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	addr = MSM_MODEM_MEM_PHYS;
	addr &= ~(PAGE_SIZE - 1);
        addr &= 0xffffffffUL;

	if (io_remap_pfn_range(vma, vma->vm_start, addr >> PAGE_SHIFT,
				length, vma->vm_page_prot)) {
		printk(KERN_ERR "remap_pfn_range failed in %s\n",__FILE__);
		return -EAGAIN;
	}

	return 0;
}

static int modem_logger_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int modem_logger_release(struct inode *inode , struct file *file)
{
	return 0;
}

static struct file_operations modem_logger_fops = {
	.owner = THIS_MODULE,
	.mmap = modem_logger_mmap,
	.open = modem_logger_open,
	.release = modem_logger_release,
};

static struct miscdevice modem_logger_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "log_modem",
	.fops = &modem_logger_fops,
	.parent = NULL,
};

static int __init modem_logger_init(void)
{
	int ret;

	ret = misc_register(&modem_logger_device);
	if( unlikely(ret)) {
		printk(KERN_ERR "%s: failed to register misc \n",__func__);
		return ret;
	}

	return 0;
}

static void __exit modem_logger_exit(void)
{
	misc_deregister(&modem_logger_device);
	return ;
}

module_init(modem_logger_init);
module_exit(modem_logger_exit);
