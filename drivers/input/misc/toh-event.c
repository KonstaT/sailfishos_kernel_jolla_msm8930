/*
 *  The Other Half smart cover event driver
 *
 *  Copyright (c) 2013 Jolla ltd.
 *  Contact: Kalle Jokiniemi <kalle.jokiniemi@jollamobile.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mfd/toh.h>
#include <linux/i2c.h>
#include <linux/mutex.h>

#define TOH_DEVICE_NAME		"toh-event"

struct toh_event_devinfo {
	struct input_dev	*idev;
	struct platform_device	*core_pdev;
	int			gpio_cover;
	int			irq_cover;	/* Cover attach/detach event */
	int			gpio_int;
	int			gpio_cover_val; /* Last gpio_cover value */
	int			irq_int;	/* Interrupt event from cover */
	int			i2c_bus;
	int			toh_enabled;
	struct mutex		event_mutex;
};

static void toh_event_register_devices(struct toh_event_devinfo *toh,
							int enable)
{
	int ret;
	struct toh_platform_data toh_pdata = {
		.i2c_bus = toh->i2c_bus,
		.gpio_int = toh->gpio_int,
		.irq_int = toh->irq_int,
	};

	if (enable && !toh->toh_enabled) {
		toh->core_pdev = platform_device_alloc("toh-core", 0);
		if (toh->core_pdev == NULL) {
			dev_err(&toh->idev->dev, "Not enough memory, core\n");
			goto toh_register_exit;
		}
		if (platform_device_add_data(toh->core_pdev, &toh_pdata,
					sizeof(struct toh_platform_data))) {
			dev_err(&toh->idev->dev, "Not enough memory, pdata\n");
			goto toh_register_alloc;
		}
		ret = platform_device_add(toh->core_pdev);
		if (ret) {
			dev_err(&toh->idev->dev, "Error %d adding pdev\n", ret);
			goto toh_register_alloc;
		}
		toh->toh_enabled = 1;
	} else if (!enable && toh->toh_enabled) {
		platform_device_unregister(toh->core_pdev);
		toh->toh_enabled = 0;
	}

toh_register_exit:

	return;

toh_register_alloc:
	platform_device_put(toh->core_pdev);
	goto toh_register_exit;
}

static irqreturn_t toh_event_irq(int irq, void *dev_id)
{
	int val;
	struct toh_event_devinfo *toh = dev_id;

	if (irq == toh->irq_cover) {
		mutex_lock(&toh->event_mutex);
		val = gpio_get_value(toh->gpio_cover);
		/* The GPIO line is bit shaky, report only if value changed */
		if (val != toh->gpio_cover_val) {
			toh_event_register_devices(toh, !val);
			input_report_switch(toh->idev, SW_DOCK, !val);
			input_sync(toh->idev);
			toh->gpio_cover_val = val;
		}
		mutex_unlock(&toh->event_mutex);
	} else {
		dev_err(&toh->idev->dev, "Unexpected irq %d received\n", irq);
	}

	return IRQ_HANDLED;
}

static int __devinit toh_event_probe(struct platform_device *pdev)
{
	struct resource *gpio_cover;
	struct resource *gpio_int;
	struct resource *cover_det;
	struct resource *cover_int;
	struct resource *cover_i2c;
	struct toh_event_devinfo *toh;
	int ret = 0;

	toh = kzalloc(sizeof(struct toh_event_devinfo), GFP_KERNEL);
	toh->idev = input_allocate_device();
	if (!toh || !toh->idev) {
		dev_err(&pdev->dev, "Not enough memory\n");
		ret = -ENOMEM;
		goto toh_probe_err1;
	}

	mutex_init(&toh->event_mutex);

	input_set_capability(toh->idev, EV_SW, SW_DOCK);
	toh->idev->name = pdev->name;

	input_set_drvdata(toh->idev, toh);

	ret = input_register_device(toh->idev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register device\n");
		goto toh_probe_err1;
	}

	gpio_cover = platform_get_resource(pdev, IORESOURCE_IO, 0);
	gpio_int = platform_get_resource(pdev, IORESOURCE_IO, 1);
	cover_det = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	cover_int = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	cover_i2c = platform_get_resource(pdev, IORESOURCE_BUS, 0);

	if (cover_det == NULL || cover_int == NULL || cover_i2c == NULL ||
				gpio_cover == NULL || gpio_int == NULL) {
		dev_err(&pdev->dev, "Incomplete platform resources\n");
		ret = -ENODEV;
		goto toh_probe_err2;
	}

	toh->gpio_cover= gpio_cover->start;
	toh->gpio_int= gpio_int->start;
	toh->irq_cover = cover_det->start;
	toh->irq_int = cover_int->start;
	toh->i2c_bus = cover_i2c->start;
	toh->toh_enabled = 0;

	ret = gpio_request_one(toh->gpio_cover, GPIOF_IN, "toh-cover");

	if (ret) {
		dev_err(&pdev->dev, "Invalid cover GPIO %d\n", toh->gpio_cover);
		goto toh_probe_err2;
	}

	ret = request_threaded_irq(toh->irq_cover, NULL, toh_event_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT, TOH_DEVICE_NAME, toh);

	if (ret) {
		dev_err(&pdev->dev, "Can't allocate cover attach irq %d\n",
							toh->irq_cover);
		goto toh_probe_err3;
	}

	mutex_lock(&toh->event_mutex);

	toh->gpio_cover_val = gpio_get_value(toh->gpio_cover);
	/* Cover switch is active-low, register the core if cover attached */
	if (toh->gpio_cover_val == 0)
		toh_event_register_devices(toh, 1);

	/* Report initial status */
	input_report_switch(toh->idev, SW_DOCK, !toh->gpio_cover_val);
	input_sync(toh->idev);
	mutex_unlock(&toh->event_mutex);

	platform_set_drvdata(pdev, toh);
	device_init_wakeup(&pdev->dev, 1);
	enable_irq_wake(toh->irq_cover);

	return ret;

toh_probe_err3:
	gpio_free(toh->gpio_cover);
toh_probe_err2:
	input_unregister_device(toh->idev);
toh_probe_err1:
	input_free_device(toh->idev);
	kfree(toh);
	return ret;
}

static int __devexit toh_event_remove(struct platform_device *pdev)
{
	struct toh_event_devinfo *toh = platform_get_drvdata(pdev);

	disable_irq_wake(toh->irq_cover);
	device_init_wakeup(&pdev->dev, 0);
	free_irq(toh->irq_cover, toh);
	gpio_free(toh->gpio_cover);
	input_unregister_device(toh->idev);
	input_free_device(toh->idev);
	kfree(toh);

	return 0;
}

static struct platform_driver toh_driver = {
	.probe		= toh_event_probe,
	.remove		= toh_event_remove,
	.driver = {
		.name = TOH_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(toh_driver);

MODULE_DESCRIPTION("Jolla Other Half event detection driver");
MODULE_AUTHOR("Kalle Jokiniemi <kalle.jokiniemi@jollamobile.com>");
MODULE_LICENSE("GPL v2");
