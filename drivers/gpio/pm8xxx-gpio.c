/*
 * Qualcomm PMIC8XXX GPIO driver
 *
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

//Terry Add debugfs to config and control PMIC GPIO 
#include <linux/debugfs.h>
#define PM89XX_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + NR_GPIO_IRQS)

/* GPIO registers */
#define	SSBI_REG_ADDR_GPIO_BASE		0x150
#define	SSBI_REG_ADDR_GPIO(n)		(SSBI_REG_ADDR_GPIO_BASE + n)

/* GPIO */
#define	PM_GPIO_BANK_MASK		0x70
#define	PM_GPIO_BANK_SHIFT		4
#define	PM_GPIO_WRITE			0x80

/* Bank 0 */
#define	PM_GPIO_VIN_MASK		0x0E
#define	PM_GPIO_VIN_SHIFT		1
#define	PM_GPIO_MODE_ENABLE		0x01

/* Bank 1 */
#define	PM_GPIO_MODE_MASK		0x0C
#define	PM_GPIO_MODE_SHIFT		2
#define	PM_GPIO_OUT_BUFFER		0x02
#define	PM_GPIO_OUT_INVERT		0x01

#define	PM_GPIO_MODE_OFF		3
#define	PM_GPIO_MODE_OUTPUT		2
#define	PM_GPIO_MODE_INPUT		0
#define	PM_GPIO_MODE_BOTH		1

/* Bank 2 */
#define	PM_GPIO_PULL_MASK		0x0E
#define	PM_GPIO_PULL_SHIFT		1

/* Bank 3 */
#define	PM_GPIO_OUT_STRENGTH_MASK	0x0C
#define	PM_GPIO_OUT_STRENGTH_SHIFT	2
#define PM_GPIO_PIN_ENABLE		0x00
#define	PM_GPIO_PIN_DISABLE		0x01

/* Bank 4 */
#define	PM_GPIO_FUNC_MASK		0x0E
#define	PM_GPIO_FUNC_SHIFT		1

/* Bank 5 */
#define	PM_GPIO_NON_INT_POL_INV	0x08
#define PM_GPIO_BANKS		6

struct pm_gpio_chip {
	struct list_head	link;
	struct gpio_chip	gpio_chip;
	spinlock_t		pm_lock;
	u8			*bank1;
	int			irq_base;
};

static LIST_HEAD(pm_gpio_chips);
static DEFINE_MUTEX(pm_gpio_chips_lock);

static int pm_gpio_get(struct pm_gpio_chip *pm_gpio_chip, unsigned gpio)
{
	int	mode;

	if (gpio >= pm_gpio_chip->gpio_chip.ngpio || pm_gpio_chip == NULL)
		return -EINVAL;

	/* Get gpio value from config bank 1 if output gpio.
	   Get gpio value from IRQ RT status register for all other gpio modes.
	 */
	mode = (pm_gpio_chip->bank1[gpio] & PM_GPIO_MODE_MASK) >>
		PM_GPIO_MODE_SHIFT;
	if (mode == PM_GPIO_MODE_OUTPUT)
		return pm_gpio_chip->bank1[gpio] & PM_GPIO_OUT_INVERT;
	else
		return pm8xxx_read_irq_stat(pm_gpio_chip->gpio_chip.dev->parent,
				pm_gpio_chip->irq_base + gpio);
}

static int pm_gpio_set(struct pm_gpio_chip *pm_gpio_chip,
		unsigned gpio, int value)
{
	int	rc;
	u8	bank1;
	unsigned long flags;

	if (gpio >= pm_gpio_chip->gpio_chip.ngpio || pm_gpio_chip == NULL)
		return -EINVAL;

	spin_lock_irqsave(&pm_gpio_chip->pm_lock, flags);
	bank1 = PM_GPIO_WRITE
			| (pm_gpio_chip->bank1[gpio] & ~PM_GPIO_OUT_INVERT);

	if (value)
		bank1 |= PM_GPIO_OUT_INVERT;

	pm_gpio_chip->bank1[gpio] = bank1;
	rc = pm8xxx_writeb(pm_gpio_chip->gpio_chip.dev->parent,
				SSBI_REG_ADDR_GPIO(gpio), bank1);
	spin_unlock_irqrestore(&pm_gpio_chip->pm_lock, flags);

	if (rc)
		pr_err("FAIL pm8xxx_writeb(): rc=%d. "
		       "(gpio=%d, value=%d)\n",
		       rc, gpio, value);

	return rc;
}

static int dir_map[] = {
	PM_GPIO_MODE_OFF,
	PM_GPIO_MODE_OUTPUT,
	PM_GPIO_MODE_INPUT,
	PM_GPIO_MODE_BOTH,
};
/* Terry cheng, 20110829, Implement GPIO debug fs  {*/
static int decode_dir_map[] = {
	PM_GPIO_MODE_OUTPUT,
	PM_GPIO_MODE_OFF,
	PM_GPIO_MODE_BOTH,	
	PM_GPIO_MODE_INPUT,
};
/* }Terry cheng, 20110829, Implement GPIO debug fs  */

static int pm_gpio_set_direction(struct pm_gpio_chip *pm_gpio_chip,
			      unsigned gpio, int direction)
{
	int	rc;
	u8	bank1;
	unsigned long flags;

	if (!direction || pm_gpio_chip == NULL)
		return -EINVAL;

	spin_lock_irqsave(&pm_gpio_chip->pm_lock, flags);
	bank1 = PM_GPIO_WRITE
			| (pm_gpio_chip->bank1[gpio] & ~PM_GPIO_MODE_MASK);

	bank1 |= ((dir_map[direction] << PM_GPIO_MODE_SHIFT)
		  & PM_GPIO_MODE_MASK);

	pm_gpio_chip->bank1[gpio] = bank1;
	rc = pm8xxx_writeb(pm_gpio_chip->gpio_chip.dev->parent,
				SSBI_REG_ADDR_GPIO(gpio), bank1);
	spin_unlock_irqrestore(&pm_gpio_chip->pm_lock, flags);

	if (rc)
		pr_err("Failed on pm8xxx_writeb(): rc=%d (GPIO config)\n",
			rc);

	return rc;
}

static int pm_gpio_init_bank1(struct pm_gpio_chip *pm_gpio_chip)
{
	int i, rc;
	u8 bank;

	for (i = 0; i < pm_gpio_chip->gpio_chip.ngpio; i++) {
		bank = 1 << PM_GPIO_BANK_SHIFT;
		rc = pm8xxx_writeb(pm_gpio_chip->gpio_chip.dev->parent,
				SSBI_REG_ADDR_GPIO(i),
				bank);
		if (rc) {
			pr_err("error setting bank rc=%d\n", rc);
			return rc;
		}

		rc = pm8xxx_readb(pm_gpio_chip->gpio_chip.dev->parent,
				SSBI_REG_ADDR_GPIO(i),
				&pm_gpio_chip->bank1[i]);
		if (rc) {
			pr_err("error reading bank 1 rc=%d\n", rc);
			return rc;
		}
	}
	return 0;
}

static int pm_gpio_to_irq(struct gpio_chip *gpio_chip, unsigned offset)
{
	struct pm_gpio_chip *pm_gpio_chip = dev_get_drvdata(gpio_chip->dev);

	return pm_gpio_chip->irq_base + offset;
}

static int pm_gpio_read(struct gpio_chip *gpio_chip, unsigned offset)
{
	struct pm_gpio_chip *pm_gpio_chip = dev_get_drvdata(gpio_chip->dev);

	return pm_gpio_get(pm_gpio_chip, offset);
}

static void pm_gpio_write(struct gpio_chip *gpio_chip,
		unsigned offset, int val)
{
	struct pm_gpio_chip *pm_gpio_chip = dev_get_drvdata(gpio_chip->dev);

	pm_gpio_set(pm_gpio_chip, offset, val);
}

static int pm_gpio_direction_input(struct gpio_chip *gpio_chip,
		unsigned offset)
{
	struct pm_gpio_chip *pm_gpio_chip = dev_get_drvdata(gpio_chip->dev);

	return pm_gpio_set_direction(pm_gpio_chip, offset, PM_GPIO_DIR_IN);
}

static int pm_gpio_direction_output(struct gpio_chip *gpio_chip,
		unsigned offset,
		int val)
{
	int ret;
	struct pm_gpio_chip *pm_gpio_chip = dev_get_drvdata(gpio_chip->dev);

	ret = pm_gpio_set_direction(pm_gpio_chip, offset, PM_GPIO_DIR_OUT);
	if (!ret)
		ret = pm_gpio_set(pm_gpio_chip, offset, val);

	return ret;
}

static void pm_gpio_dbg_show(struct seq_file *s, struct gpio_chip *gpio_chip)
{
	static const char * const cmode[] = { "in", "in/out", "out", "off" };
	struct pm_gpio_chip *pm_gpio_chip = dev_get_drvdata(gpio_chip->dev);
	u8 mode, state, bank;
	const char *label;
	int i, j;
	/* Terry Cheng, 20111026, fine tune PMIC 8921 debugfs output data {*/
	int vin = 0, out_buff=0, out_val=0, pull=0, out_str=0, disable=0, function=0, inv_int_pol=0;
	static const char * const vin_str[] = { "VPH", "BB", "S4", "L15", "L4", "L3", "L17" };
	static const char * const out_stregn_str[] = { "No", "High", "Med", "Low" };
	static const char * const pull_str[] = { "30", "1P5", "31P5", "1P5_30", "DN", "No" };
	seq_printf(s, "GPIO-num  requested     state vin mode outbuf outval pull str  dis func inv\n");

	for (i = 0; i < gpio_chip->ngpio; i++) {
		label = gpiochip_is_requested(gpio_chip, i);
		mode = (pm_gpio_chip->bank1[i] & PM_GPIO_MODE_MASK) >>
			PM_GPIO_MODE_SHIFT;
		state = pm_gpio_get(pm_gpio_chip, i);
#if 0		
		seq_printf(s, "gpio-%-3d (%-12.12s) %-10.10s"
				" %s",
				gpio_chip->base + i,
				label ? label : "--",
				cmode[mode],
				state ? "hi" : "lo");
#endif // if 0
		for (j = 0; j < PM_GPIO_BANKS; j++) {
			bank = j << PM_GPIO_BANK_SHIFT;
			pm8xxx_writeb(gpio_chip->dev->parent,
					SSBI_REG_ADDR_GPIO(i),
					bank);
			pm8xxx_readb(gpio_chip->dev->parent,
					SSBI_REG_ADDR_GPIO(i),
					&bank);
			//seq_printf(s, " 0x%02x", bank);
			switch (j) {
				case 0:
					vin = (bank & PM_GPIO_VIN_MASK) >>PM_GPIO_VIN_SHIFT;
					break;
				case 1:
					out_buff = (bank & 0x02) >> 1;
					out_val = (bank & 0x01) ;					
					mode = (bank & 0x0c)>>2;
					break;
				case 2:
					pull = (bank & PM_GPIO_PULL_MASK) >> PM_GPIO_PULL_SHIFT;
					break;
				case 3:
					out_str = (bank & PM_GPIO_OUT_STRENGTH_MASK) >>PM_GPIO_OUT_STRENGTH_SHIFT;
					disable = (bank & 0x01);					
					break;
				case 4:
					
					function = (bank & PM_GPIO_FUNC_MASK)>>PM_GPIO_FUNC_SHIFT;
					break;
				case 5:
					if (bank & PM_GPIO_NON_INT_POL_INV)
						inv_int_pol= 0;
					else
						inv_int_pol= 1;
					break;
				default:
					break;
		}
		}
		//seq_printf(s, "\n");
		seq_printf(s, "gpio-%-3d (%-12.12s)"
				" %s   %3s %3s %5d %4d %8s  %2s  %3s %3d %4s",
				gpio_chip->base + i,
				label ? label : "--",
				state ? "hi" : "lo", 
				vin_str[vin], cmode[mode], out_buff, out_val, pull_str[pull], out_stregn_str[out_str], disable ? "Yes": "No", function, inv_int_pol ? "Yes" : "No");
		seq_printf(s, "\n");
		/* } Terry Cheng, 20111026, fine tune PMIC 8921 debugfs output data */		

	}
}

/* Terry Cheng, 20120220, dump soc gpio, pmic gpio, and pmic mpp {*/
void pm_gpio_dbg_show_suspend_resume_dump(void)
{
	static const char * const cmode[] = { "in", "in/out", "out", "off" };
	struct pm_gpio_chip *pm_gpio_chip = NULL;
	struct gpio_chip *gpio_chip = NULL;
	u8 mode, state, bank;
	const char *label;
	int i, j;
	int vin = 0, out_buff=0, out_val=0, pull=0, out_str=0, disable=0, function=0, inv_int_pol=0;
	static const char * const vin_str[] = { "VPH", "BB", "S4", "L15", "L4", "L3", "L17" };
	static const char * const out_stregn_str[] = { "No", "High", "Med", "Low" };
	static const char * const pull_str[] = { "30", "1P5", "31P5", "1P5_30", "DN", "No" };

	list_for_each_entry(pm_gpio_chip, &pm_gpio_chips, link) {
		if(pm_gpio_chip)
		{
			gpio_chip = &pm_gpio_chip->gpio_chip;
			break;
		}	
	}

	if(!pm_gpio_chip)
	{
		pr_err("Cannot find PMIC GPIO chip\n");
		return;
	}	
	
	printk("GPIO-num  requested     state vin mode outbuf outval pull str  dis func inv\n");

	for (i = 0; i < gpio_chip->ngpio; i++) {
		label = gpiochip_is_requested(gpio_chip, i);
		mode = (pm_gpio_chip->bank1[i] & PM_GPIO_MODE_MASK) >>
			PM_GPIO_MODE_SHIFT;
		state = pm_gpio_get(pm_gpio_chip, i);
			for (j = 0; j < PM_GPIO_BANKS; j++) {
				bank = j << PM_GPIO_BANK_SHIFT;
				pm8xxx_writeb(gpio_chip->dev->parent,
						SSBI_REG_ADDR_GPIO(i),
						bank);
				pm8xxx_readb(gpio_chip->dev->parent,
						SSBI_REG_ADDR_GPIO(i),
						&bank);
				switch (j) {
					case 0:
						vin = (bank & PM_GPIO_VIN_MASK) >>PM_GPIO_VIN_SHIFT;
						break;
					case 1:
						out_buff = (bank & 0x02) >> 1;
						out_val = (bank & 0x01) ;					
						mode = (bank & 0x0c)>>2;
						break;
					case 2:
						pull = (bank & PM_GPIO_PULL_MASK) >> PM_GPIO_PULL_SHIFT;
						break;
					case 3:
						out_str = (bank & PM_GPIO_OUT_STRENGTH_MASK) >>PM_GPIO_OUT_STRENGTH_SHIFT;
						disable = (bank & 0x01);					
						break;
					case 4:
						
						function = (bank & PM_GPIO_FUNC_MASK)>>PM_GPIO_FUNC_SHIFT;
						break;
					case 5:
						if (bank & PM_GPIO_NON_INT_POL_INV)
							inv_int_pol= 0;
						else
							inv_int_pol= 1;
						break;
					default:
						break;
				}
			}
		printk("gpio-%-3d (%-12.12s)"
				" %s   %3s %3s %5d %4d %8s  %2s  %3s %3d %4s",
				gpio_chip->base + i,
				label ? label : "--",
				state ? "hi" : "lo", 
				vin_str[vin], cmode[mode], out_buff, out_val, pull_str[pull], out_stregn_str[out_str], disable ? "Yes": "No", function, inv_int_pol ? "Yes" : "No");
		printk("\n");
	}
}
EXPORT_SYMBOL_GPL(pm_gpio_dbg_show_suspend_resume_dump);
/* } Terry Cheng, 20120220, dump soc gpio, pmic gpio, and pmic mpp */

static int __devinit pm_gpio_probe(struct platform_device *pdev)
{
	int ret;
	const struct pm8xxx_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct pm_gpio_chip *pm_gpio_chip;

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	pm_gpio_chip = kzalloc(sizeof(struct pm_gpio_chip), GFP_KERNEL);
	if (!pm_gpio_chip) {
		pr_err("Cannot allocate pm_gpio_chip\n");
		return -ENOMEM;
	}

	pm_gpio_chip->bank1 = kzalloc(sizeof(u8) * pdata->gpio_cdata.ngpios,
					GFP_KERNEL);
	if (!pm_gpio_chip->bank1) {
		pr_err("Cannot allocate pm_gpio_chip->bank1\n");
		ret = -ENOMEM;
		goto free_chip;
	}

	spin_lock_init(&pm_gpio_chip->pm_lock);
	pm_gpio_chip->gpio_chip.label = "pm-gpio";
	pm_gpio_chip->gpio_chip.direction_input	= pm_gpio_direction_input;
	pm_gpio_chip->gpio_chip.direction_output = pm_gpio_direction_output;
	pm_gpio_chip->gpio_chip.to_irq = pm_gpio_to_irq;
	pm_gpio_chip->gpio_chip.get = pm_gpio_read;
	pm_gpio_chip->gpio_chip.set = pm_gpio_write;
	pm_gpio_chip->gpio_chip.dbg_show = pm_gpio_dbg_show;
	pm_gpio_chip->gpio_chip.ngpio = pdata->gpio_cdata.ngpios;
	pm_gpio_chip->gpio_chip.can_sleep = 0;
	pm_gpio_chip->gpio_chip.dev = &pdev->dev;
	pm_gpio_chip->gpio_chip.base = pdata->gpio_base;
	pm_gpio_chip->irq_base = platform_get_irq(pdev, 0);
	mutex_lock(&pm_gpio_chips_lock);
	list_add(&pm_gpio_chip->link, &pm_gpio_chips);
	mutex_unlock(&pm_gpio_chips_lock);
	platform_set_drvdata(pdev, pm_gpio_chip);

	ret = gpiochip_add(&pm_gpio_chip->gpio_chip);
	if (ret) {
		pr_err("gpiochip_add failed ret = %d\n", ret);
		goto reset_drvdata;
	}

	ret = pm_gpio_init_bank1(pm_gpio_chip);
	if (ret) {
		pr_err("gpio init bank failed ret = %d\n", ret);
		goto remove_chip;
	}

	pr_info("OK: base=%d, ngpio=%d\n", pm_gpio_chip->gpio_chip.base,
		pm_gpio_chip->gpio_chip.ngpio);

	return 0;

remove_chip:
	if (gpiochip_remove(&pm_gpio_chip->gpio_chip))
		pr_err("failed to remove gpio chip\n");
reset_drvdata:
	platform_set_drvdata(pdev, NULL);
	kfree(pm_gpio_chip->bank1);
free_chip:
	kfree(pm_gpio_chip);
	return ret;
}

static int __devexit pm_gpio_remove(struct platform_device *pdev)
{
	struct pm_gpio_chip *pm_gpio_chip
		= platform_get_drvdata(pdev);

	mutex_lock(&pm_gpio_chips_lock);
	list_del(&pm_gpio_chip->link);
	mutex_unlock(&pm_gpio_chips_lock);
	platform_set_drvdata(pdev, NULL);
	if (gpiochip_remove(&pm_gpio_chip->gpio_chip))
		pr_err("failed to remove gpio chip\n");
	kfree(pm_gpio_chip->bank1);
	kfree(pm_gpio_chip);
	return 0;
}

int pm8xxx_gpio_config(int gpio, struct pm_gpio *param)
{
	int	rc, pm_gpio = -EINVAL;
	u8	bank[8];
	unsigned long flags;
	struct pm_gpio_chip *pm_gpio_chip;
	struct gpio_chip *gpio_chip;

	if (param == NULL)
		return -EINVAL;

	mutex_lock(&pm_gpio_chips_lock);
	list_for_each_entry(pm_gpio_chip, &pm_gpio_chips, link) {
		gpio_chip = &pm_gpio_chip->gpio_chip;
		if (gpio >= gpio_chip->base
			&& gpio < gpio_chip->base + gpio_chip->ngpio) {
			pm_gpio = gpio - gpio_chip->base;
			break;
		}
	}
	mutex_unlock(&pm_gpio_chips_lock);
	printk("GPIO %d dir = %d out buf = %d out value = %d pull = %d vol sel %d stren = %d func = %d inv = %d dis = %d\n",
		pm_gpio, param->direction, param->output_buffer, param->output_value, param->pull, param->vin_sel, param->out_strength, param->function, param->inv_int_pol, param->disable_pin);
	
	if (pm_gpio < 0) {
		pr_err("called on gpio %d not handled by any pmic\n", gpio);
		return -EINVAL;
	}

	/* Select banks and configure the gpio */
	bank[0] = PM_GPIO_WRITE |
		((param->vin_sel << PM_GPIO_VIN_SHIFT) &
			PM_GPIO_VIN_MASK) |
		PM_GPIO_MODE_ENABLE;
	bank[1] = PM_GPIO_WRITE |
		((1 << PM_GPIO_BANK_SHIFT) &
			PM_GPIO_BANK_MASK) |
		((dir_map[param->direction] <<
			PM_GPIO_MODE_SHIFT) &
			PM_GPIO_MODE_MASK) |
		((param->direction & PM_GPIO_DIR_OUT) ?
			((param->output_buffer & 1) ?
			 PM_GPIO_OUT_BUFFER : 0) : 0) |
		((param->direction & PM_GPIO_DIR_OUT) ?
			param->output_value & 0x01 : 0);
	bank[2] = PM_GPIO_WRITE |
		((2 << PM_GPIO_BANK_SHIFT) &
			PM_GPIO_BANK_MASK) |
		((param->pull << PM_GPIO_PULL_SHIFT) &
			PM_GPIO_PULL_MASK);
	bank[3] = PM_GPIO_WRITE |
		((3 << PM_GPIO_BANK_SHIFT) &
			PM_GPIO_BANK_MASK) |
		((param->out_strength <<
			PM_GPIO_OUT_STRENGTH_SHIFT) &
			PM_GPIO_OUT_STRENGTH_MASK) |
		(param->disable_pin ?
			PM_GPIO_PIN_DISABLE : PM_GPIO_PIN_ENABLE);
	bank[4] = PM_GPIO_WRITE |
		((4 << PM_GPIO_BANK_SHIFT) &
			PM_GPIO_BANK_MASK) |
		((param->function << PM_GPIO_FUNC_SHIFT) &
			PM_GPIO_FUNC_MASK);
	bank[5] = PM_GPIO_WRITE |
		((5 << PM_GPIO_BANK_SHIFT) & PM_GPIO_BANK_MASK) |
		(param->inv_int_pol ? 0 : PM_GPIO_NON_INT_POL_INV);

	spin_lock_irqsave(&pm_gpio_chip->pm_lock, flags);
	/* Remember bank1 for later use */
	pm_gpio_chip->bank1[pm_gpio] = bank[1];
	rc = pm8xxx_write_buf(pm_gpio_chip->gpio_chip.dev->parent,
			SSBI_REG_ADDR_GPIO(pm_gpio), bank, 6);
	spin_unlock_irqrestore(&pm_gpio_chip->pm_lock, flags);

	if (rc)
		pr_err("Failed on pm8xxx_write_buf() rc=%d (GPIO config)\n",
			rc);

	return rc;
}
EXPORT_SYMBOL(pm8xxx_gpio_config);

static struct platform_driver pm_gpio_driver = {
	.probe		= pm_gpio_probe,
	.remove		= __devexit_p(pm_gpio_remove),
	.driver		= {
		.name	= PM8XXX_GPIO_DEV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init pm_gpio_init(void)
{
	return platform_driver_register(&pm_gpio_driver);
}
postcore_initcall(pm_gpio_init);

static void __exit pm_gpio_exit(void)
{
	platform_driver_unregister(&pm_gpio_driver);
}
module_exit(pm_gpio_exit);
/* Terry cheng, 20110829, Implement GPIO debug fs { */
#if defined(CONFIG_DEBUG_FS)
static unsigned int pmic_gpio_num = 0;
static unsigned int pmic_gpio_debugfs_results = 0;

//Config 
static int pmic_gpio_config_set(void *data, u64 val)
{
	unsigned int config = val;
	struct pm_gpio param;
	pmic_gpio_num = (unsigned int)PMIC_GPIO_PIN(config);
	param.direction = PMIC_GPIO_DIR(config);
	param.output_buffer = PMIC_GPIO_OUT_BUF(config);
	param.output_value = PMIC_GPIO_OUT_VAL(config);
	param.pull = PMIC_GPIO_PULL(config);
	param.vin_sel = PMIC_GPIO_VIN(config);
	param.out_strength= PMIC_GPIO_DRVSTR(config);
	param.function= PMIC_GPIO_FUNC(config);
	param.inv_int_pol= PMIC_GPIO_INV(config);
	param.disable_pin= PMIC_GPIO_DIS(config);
	
	pmic_gpio_debugfs_results= pm8xxx_gpio_config(PM89XX_GPIO_PM_TO_SYS(pmic_gpio_num+1), &param);
	return 0;
}
static int pmic_gpio_config_get(void *data, u64 *val)
{

	struct pm_gpio_chip *pm_gpio_chip = NULL;
	struct gpio_chip *gpio_chip = NULL;
	u8 mode=0;
	u8 bank=0;
	int vin = 0, out_buff=0, out_val=0, pull=0, out_str=0, disable=0, function=0, inv_int_pol=0;
	int j;
	
	//20120418, Terry Cheng, Fix get wrong pm_gpio_chip +
	mutex_lock(&pm_gpio_chips_lock);
	list_for_each_entry(pm_gpio_chip, &pm_gpio_chips, link) {
		if(pm_gpio_chip)
		{
		gpio_chip = &pm_gpio_chip->gpio_chip;
			break;
		}	
	}
	mutex_unlock(&pm_gpio_chips_lock);

	if(!pm_gpio_chip)
	{
		pr_err("Cannot find PMIC GPIO chip\n");
		return 1;
	}
	//20120418, Terry Cheng, Fix get wrong pm_gpio_chip -	
	//Direction
	mode = (pm_gpio_chip->bank1[pmic_gpio_num] & PM_GPIO_MODE_MASK) >> PM_GPIO_MODE_SHIFT;
	for (j = 0; j < PM_GPIO_BANKS; j++) {
		bank = j << PM_GPIO_BANK_SHIFT;
		pm8xxx_writeb(gpio_chip->dev->parent,
				SSBI_REG_ADDR_GPIO(pmic_gpio_num),
				bank);
		pm8xxx_readb(gpio_chip->dev->parent,
				SSBI_REG_ADDR_GPIO(pmic_gpio_num),
				&bank);
		switch (j) {
			case 0:
				vin = (bank & PM_GPIO_VIN_MASK) >>PM_GPIO_VIN_SHIFT;
				break;
			case 1:
				out_buff = (bank & 0x02) >> 1;
				out_val = (bank & 0x01) ;					
				mode = decode_dir_map[(bank & 0x0c)>>2];
				break;
			case 2:
				pull = (bank & PM_GPIO_PULL_MASK) >> PM_GPIO_PULL_SHIFT;
				break;
			case 3:
				out_str = (bank & PM_GPIO_OUT_STRENGTH_MASK) >>PM_GPIO_OUT_STRENGTH_SHIFT;
				disable = (bank & 0x01);					
				break;
			case 4:
				
				function = (bank & PM_GPIO_FUNC_MASK)>>PM_GPIO_FUNC_SHIFT;
				break;
			case 5:
				if (bank & PM_GPIO_NON_INT_POL_INV)
					inv_int_pol= 0;
				else
					inv_int_pol= 1;
				break;
			default:
				break;
		}				
	}
	/* } Terry Cheng, 20111026, fine tune PMIC 8921 debugfs output data */		


	printk("PMIC GPIO: pmic_gpio_num %d,  dir = %d, out_buff = %d , out_val =%d, pull = %d, vin = %d, out_str = %d, func = %d, inv = %d, dis = %d\n"
			, pmic_gpio_num, mode, out_buff, out_val, pull, vin, out_str, function, inv_int_pol, disable);  
	//Pack
	*val = ((pmic_gpio_num <<17)|mode |(out_buff<<2)|(out_val<<3)|(pull<<4)|(vin<<7)
			|(out_str<<10)|(function<<12)
			|(inv_int_pol<<15)| (disable << 16));
	return 0;
}
//Value
static int pmic_gpio_value_get(void *data, u64 *val)
{
	u8  mode, state; //mode,bank;
	unsigned int gpio = pmic_gpio_num;

	struct pm_gpio_chip *pm_gpio_chip = NULL;
	struct gpio_chip *gpio_chip = NULL;

	//20120418, Terry Cheng, Fix get wrong pm_gpio_chip +
	mutex_lock(&pm_gpio_chips_lock);
	list_for_each_entry(pm_gpio_chip, &pm_gpio_chips, link) {
		if(pm_gpio_chip)
		{
		gpio_chip = &pm_gpio_chip->gpio_chip;
			break;
		}
	}
	mutex_unlock(&pm_gpio_chips_lock);

	if(!pm_gpio_chip)
	{
		pr_err("Cannot find PMIC GPIO chip\n");
		return 1;
	}	
	//20120418, Terry Cheng, Fix get wrong pm_gpio_chip -
	mode = (pm_gpio_chip->bank1[gpio] & PM_GPIO_MODE_MASK) >> PM_GPIO_MODE_SHIFT;
	state = pm_gpio_get(pm_gpio_chip, pmic_gpio_num);

	printk("pmic_gpio_value_get = %d\n", state);
	*val = state;

	return 0;
}
static int pmic_gpio_value_set(void *data, u64 val)
{

	struct pm_gpio_chip *pm_gpio_chip = NULL;
	struct gpio_chip *gpio_chip = NULL;

	//20120418, Terry Cheng, Fix get wrong pm_gpio_chip +
	mutex_lock(&pm_gpio_chips_lock);
	list_for_each_entry(pm_gpio_chip, &pm_gpio_chips, link) {
		if(pm_gpio_chip)
		{
		gpio_chip = &pm_gpio_chip->gpio_chip;
			break;
		}
	}
	mutex_unlock(&pm_gpio_chips_lock);

	if(!pm_gpio_chip)
	{
		pr_err("Cannot find PMIC GPIO chip\n");
		return 1;
	}	
	//20120418, Terry Cheng, Fix get wrong pm_gpio_chip -
	pm_gpio_write(gpio_chip, pmic_gpio_num, (int)val);
	printk("pmic gpio_value_set = %d\n", (int)val);

	return 0;
}

//Results
static int pmic_gpio_results_get(void *data, u64 *val)
{
	unsigned int result = pmic_gpio_debugfs_results;
	pmic_gpio_debugfs_results = 0;
	if (result)
		*val = 1;
	else
		*val = 0;

	return 0;
}

static int pmic_gpio_results_set(void *data, u64 val)
{
	return 0;
}
//GPIO number
static int pmic_gpio_num_get(void *data, u64 *val)
{
	*val = (u64)pmic_gpio_num;
	return 0;
}

static int pmic_gpio_num_set(void *data, u64 val)
{
	pmic_gpio_num = (unsigned int)val;
	printk("pmic_gpio_num = %d\n", pmic_gpio_num);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gpio_config_fops, pmic_gpio_config_get,
						pmic_gpio_config_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(gpio_value_fops, pmic_gpio_value_get,
						pmic_gpio_value_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(gpio_results_fops, pmic_gpio_results_get,
						pmic_gpio_results_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(gpio_num_fops, pmic_gpio_num_get,
						pmic_gpio_num_set, "%llu\n");

static int __init pmic_gpio_debug_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("pmic_gpio", 0);
	if (IS_ERR(dent))
		return 0;

	//Create debugfs file
	debugfs_create_file("config", 0644, dent, 0, &gpio_config_fops);
	debugfs_create_file("value", 0644, dent, 0, &gpio_value_fops);
	debugfs_create_file("results", 0644, dent, 0, &gpio_results_fops);
	debugfs_create_file("num", 0644, dent, 0, &gpio_num_fops);

	return 0;
}
device_initcall(pmic_gpio_debug_init);
#endif	//CONFIG_DEBUG_FS
/* }Terry cheng, 20110829, Implement GPIO debug fs  */


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC GPIO driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8XXX_GPIO_DEV_NAME);
