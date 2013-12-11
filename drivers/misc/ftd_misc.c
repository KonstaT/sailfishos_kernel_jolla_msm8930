#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/oem_smem_struct.h>
#include <mach/msm_smsm.h>

void show_ddr_size_in_debugfs(void); // rk.chen 2012/1/18
void show_ddr_manufacturerID_in_debugfs(void);


static int __init ftd_debugfs_misc_init(void)
{
    printk("+BootLog, %s\n", __func__);
    show_ddr_size_in_debugfs();	
    show_ddr_manufacturerID_in_debugfs();
    printk("-BootLog, %s\n", __func__);
    return 0;
}

static void __exit ftd_debugfs_misc_exit(void)
{
}


// {
// rk.chen , 2012/1/18
// show ddr size in debugfs.
void show_ddr_size_in_debugfs() {
    smem_vendor_id2_bl_data *vendor2_data;
    vendor2_data = (smem_vendor_id2_bl_data *) smem_alloc(SMEM_ID_VENDOR2, sizeof(smem_vendor_id2_bl_data));
    if (!vendor2_data) {
        printk("smem error! failed to get smem: SMEM_ID_VENDOR2.\n");
    } else {
        printk("[rk] ddr size=%d MB \n", vendor2_data->hw_info.lpddr2_size_in_MB);
        debugfs_create_u32("ddr_size_in_MB", 0444, NULL, &(vendor2_data->hw_info.lpddr2_size_in_MB));
    }
}
// }

// {
// rk.chen , 2012/2/6
// show the manufacturer id of DDR in debugfs.
void show_ddr_manufacturerID_in_debugfs() {
    smem_vendor_id2_bl_data *vendor2_data;
    vendor2_data = (smem_vendor_id2_bl_data *) smem_alloc(SMEM_ID_VENDOR2, sizeof(smem_vendor_id2_bl_data));
    if (!vendor2_data) {
        printk("smem error! failed to get smem: SMEM_ID_VENDOR2.\n");
    } else {
        printk("[rk] manufacturer id of ddr=%d \n", vendor2_data->hw_info.ddr_vendor);
        debugfs_create_u32("ddr_manufacturer_id", 0444, NULL, &(vendor2_data->hw_info.ddr_vendor));
    }
}
// }


module_init(ftd_debugfs_misc_init);
module_exit(ftd_debugfs_misc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("misc data used in debugfs");
