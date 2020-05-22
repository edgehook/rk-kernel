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

#define MISC_ADV_GPIO_MODNAME		"misc-adv-gpio"
static int pm_reset_gpio;
static bool pm_reset_gpio_active;

void pm_adv_reboot(void)
{
	if(gpio_is_valid(pm_reset_gpio)) {
		gpio_direction_output(pm_reset_gpio,!pm_reset_gpio_active);
		mdelay(5);
		gpio_direction_output(pm_reset_gpio,pm_reset_gpio_active);
		mdelay(500);
	}
}

static int misc_adv_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    struct device_node *np;
    enum of_gpio_flags flags;
    int  minipcie_pwr_gpio;
    int  m2_reset_gpio;
	int  minipcie_reset_gpio;
	int  m2_pwr_gpio;
	bool  minipcie_pwr_active;
	bool  m2_reset_active;
	bool  minipcie_reset_active;
	bool  m2_pwr_active;
    int  timing_interval = 0;
	
    np = dev->of_node;
	minipcie_reset_gpio = of_get_named_gpio_flags(np, "minipcie-reset-gpio", 0, &flags);
	if (gpio_is_valid(minipcie_reset_gpio))
	{
		minipcie_reset_active = !(flags & OF_GPIO_ACTIVE_LOW);
		if(minipcie_reset_active)
			gpio_request_one(minipcie_reset_gpio, 
                        GPIOF_OUT_INIT_HIGH, "minipcie 4g reset gpio");
		else
			gpio_request_one(minipcie_reset_gpio, 
                        GPIOF_OUT_INIT_LOW, "minipcie 4g reset gpio");
	}

	minipcie_pwr_gpio = of_get_named_gpio_flags(np, "minipcie-pwr-gpio", 0, &flags);
	if (gpio_is_valid(minipcie_pwr_gpio))
	{
		minipcie_pwr_active = !(flags & OF_GPIO_ACTIVE_LOW);
		if(minipcie_pwr_active)
			gpio_request_one(minipcie_pwr_gpio, 
                        GPIOF_OUT_INIT_HIGH, "minipcie pwr gpio");
		else
			gpio_request_one(minipcie_pwr_gpio, 
                        GPIOF_OUT_INIT_LOW, "minipcie pwr gpio");
	}

	if (!of_property_read_u32(np,"timing-interval",&timing_interval))
		timing_interval = 50;
	if(timing_interval)
	    mdelay(timing_interval);
	gpio_direction_output(minipcie_reset_gpio, !minipcie_reset_active);

	m2_reset_gpio = of_get_named_gpio_flags(np, "m2-reset-gpio", 0, &flags);
	if (gpio_is_valid(m2_reset_gpio))
	{
		m2_reset_active = !(flags & OF_GPIO_ACTIVE_LOW);
		if(m2_reset_active)
			gpio_request_one(m2_reset_gpio, 
                        GPIOF_OUT_INIT_HIGH, "minipcie reset gpio");
		else
			gpio_request_one(m2_reset_gpio, 
                        GPIOF_OUT_INIT_LOW, "minipcie reset gpio");
	}

	m2_pwr_gpio = of_get_named_gpio_flags(np, "m2-pwr-gpio", 0, &flags);
	if (gpio_is_valid(m2_pwr_gpio))
	{
		m2_pwr_active = !(flags & OF_GPIO_ACTIVE_LOW);
		if(m2_pwr_active)
			gpio_request_one(m2_pwr_gpio, 
                        GPIOF_OUT_INIT_HIGH, "m2 pwr gpio");
		else
			gpio_request_one(m2_pwr_gpio, 
                        GPIOF_OUT_INIT_LOW, "m2 pwr gpio");
	}

	if(timing_interval)
	    	mdelay(timing_interval);
	gpio_direction_output(m2_reset_gpio, !m2_reset_active);

	pm_reset_gpio = of_get_named_gpio_flags(np,"system-reset-gpio",0,&flags);
	if(gpio_is_valid(pm_reset_gpio)) {
		pm_reset_gpio_active = flags & OF_GPIO_ACTIVE_LOW;
		if(!pm_reset_gpio_active)
			gpio_request_one(pm_reset_gpio, 
                        GPIOF_OUT_INIT_LOW, "system reset gpio");
		else
			gpio_request_one(pm_reset_gpio, 
                        GPIOF_OUT_INIT_HIGH, "system reset gpio");
	}

	return 0;
}

static const struct of_device_id misc_adv_gpio_of_match[] = {
	{ .compatible = MISC_ADV_GPIO_MODNAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, misc_adv_gpio_of_match);

static struct platform_driver misc_adv_gpio_driver = {
	.probe		= misc_adv_gpio_probe,
	.driver		= {
		.name	= MISC_ADV_GPIO_MODNAME,
		.of_match_table	=  of_match_ptr(misc_adv_gpio_of_match),
	}
};

static int __init misc_adv_gpio_init(void)
{
	return platform_driver_register(&misc_adv_gpio_driver);
}

static void __exit misc_adv_gpio_exit(void)
{
	platform_driver_unregister(&misc_adv_gpio_driver);
}

arch_initcall(misc_adv_gpio_init);
module_exit(misc_adv_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("misc gpio driver for advantech");
