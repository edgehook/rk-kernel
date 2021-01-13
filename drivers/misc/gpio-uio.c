/*
 * Driver for GPIO timing control.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#define UIO_GPIO_MODNAME		"uio-gpio"

static char uio_name_string[64];

static ssize_t uio_show_name(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
	return sprintf(buf, "%s\n", uio_name_string);
}

static DEVICE_ATTR(name, S_IRUGO, uio_show_name, NULL);


static int uio_gpio_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np;
    const char *strings;
    int ret;

    enum of_gpio_flags flags;

    int  lan_pwren_gpio;
    int  lan_reset_gpio;

    bool  lan_pwren_active;
    bool  lan_reset_active;

    int timing_interval = 0;
    int num_input_gpios, num_output_gpios;
    int i, gpio, err;
    bool active;

    np = dev->of_node;

    //gpios
    num_input_gpios = of_gpio_named_count(np, "input-gpios");
    num_output_gpios = of_gpio_named_count(np, "output-gpios");

    for (i = 0; i < num_input_gpios; i++) {
        gpio = of_get_named_gpio(np, "input-gpios", i);
	if (gpio_is_valid(gpio)) {
            err = gpio_request(gpio, "uio_input_gpio");
            if (!err) {
                gpio_direction_input(gpio);
                gpio_export(gpio, false);
            }
        }
    }

    for (i = 0; i < num_output_gpios; i++) {
        gpio = of_get_named_gpio_flags(np, "output-gpios", i, &flags);
	if (gpio_is_valid(gpio)) {
            err = gpio_request(gpio, "uio_output_gpio");
            if (!err) {
                active = !(flags & OF_GPIO_ACTIVE_LOW);
                if(active)
                    gpio_direction_output(gpio, GPIOF_OUT_INIT_HIGH);
                else
                    gpio_direction_output(gpio, GPIOF_OUT_INIT_LOW);
                gpio_export(gpio, false);
            }
        }
    }

    //power
    lan_pwren_gpio = of_get_named_gpio_flags(np, "lan-pwren-gpio", 0, &flags);
    if (gpio_is_valid(lan_pwren_gpio))
    {
        lan_pwren_active = !(flags & OF_GPIO_ACTIVE_LOW);
        if(lan_pwren_active)
            gpio_request_one(lan_pwren_gpio, GPIOF_OUT_INIT_HIGH, "uio lan pwren gpio");
	else
            gpio_request_one(lan_pwren_gpio, GPIOF_OUT_INIT_LOW, "uio lan pwren gpio");
    }

    //reset disable
    lan_reset_gpio = of_get_named_gpio_flags(np, "lan-reset-gpio", 0, &flags);
    if (gpio_is_valid(lan_reset_gpio))
    {
        lan_reset_active = !(flags & OF_GPIO_ACTIVE_LOW);
        if(lan_reset_active)
            gpio_request_one(lan_reset_gpio, GPIOF_OUT_INIT_HIGH, "uio lan reset gpio");
	else
            gpio_request_one(lan_reset_gpio, GPIOF_OUT_INIT_LOW, "uio lan reset gpio");
    }

    //delay
    if (of_property_read_u32(np,"timing-interval",&timing_interval))
        timing_interval = 50;

    if(timing_interval)
        mdelay(timing_interval);

    //reset enable
    if (gpio_is_valid(lan_reset_gpio))
        gpio_direction_output(lan_reset_gpio, !lan_reset_active);

    //sysfs
    ret = of_property_read_string(np, "uio-name", &strings);
    if (ret) {
        strcpy(uio_name_string, "unknown");
    } else {
        strcpy(uio_name_string, strings);
    }
    device_create_file(dev, &dev_attr_name);

    return 0;
}

static const struct of_device_id uio_gpio_of_match[] = {
	{ .compatible = UIO_GPIO_MODNAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, uio_gpio_of_match);

static struct platform_driver uio_gpio_driver = {
	.probe		= uio_gpio_probe,
	.driver		= {
		.name	= UIO_GPIO_MODNAME,
		.of_match_table	=  of_match_ptr(uio_gpio_of_match),
	}
};

static int __init uio_gpio_init(void)
{
	return platform_driver_register(&uio_gpio_driver);
}

static void __exit uio_gpio_exit(void)
{
	platform_driver_unregister(&uio_gpio_driver);
}

arch_initcall(uio_gpio_init);
module_exit(uio_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("gpio uio driver for advantech");
