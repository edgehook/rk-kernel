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
#include <linux/device.h>

#define CP210X_232_485_MODNAME		"cp210x-232-485"
#define CP210X_232_485_MAXPORT		4
#define CP210X_232_485_GPIO_NUM		3

enum cp210x_uart_type {
	CP210X_RS232,
	CP210X_RS485
};

static int g_port_type[CP210X_232_485_MAXPORT]  = {0xFF, 0xFF, 0xFF, 0xFF};

static int __init setup_uart_mode(char *buf)
{
	u32 val;
	int i=0;
	
	if (!buf)
		return -EINVAL;
	
	val = simple_strtol(buf, NULL, 0);
	for(i=0;i<CP210X_232_485_MAXPORT;i++)
		g_port_type[i] = (val&(1<<i)) ? CP210X_RS485 : CP210X_RS232;
	
	return 0;
}

early_param("uart_mode", setup_uart_mode);

static ssize_t cp210x_232_485_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int i = 0;
	int len = 0;
	for(i = 0; i < CP210X_232_485_MAXPORT; i++)
	{
		len += sprintf(buf+len, "%d ", g_port_type[i]);
	}		
	return len;
}

static DEVICE_ATTR(mode, S_IRUGO|S_IRUSR,cp210x_232_485_mode_show, NULL);

static int cp210x_232_485_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    struct device_node *np;
	struct device_node *port_np=NULL;
    enum of_gpio_flags flags;
	int i = 0;
	char port_name[20];
	u32 val;
	int sel0_gpio, sel1_gpio;
	int term_gpio;
	char *gpio_name[3];
	u32 rs232_mode_table[CP210X_232_485_GPIO_NUM];
	u32 rs485_mode_table[CP210X_232_485_GPIO_NUM];
	u32 *mode_table;

	np = dev->of_node;
	
	of_property_read_u32_array(np, "rs232_mode_table", rs232_mode_table, ARRAY_SIZE(rs232_mode_table)); 
	of_property_read_u32_array(np, "rs485_mode_table", rs485_mode_table, ARRAY_SIZE(rs485_mode_table));
	
	for(i = 0; i < CP210X_232_485_MAXPORT; i++)
	{
		sprintf(port_name, "port%d", i);
		port_np = of_get_child_by_name(np, port_name);
		if(port_np)
		{
			if(0xff == g_port_type[i])
			{
				if (!of_property_read_u32(port_np, "mode", &val))
				{
					if(val == CP210X_RS485)
					{
						g_port_type[i] = CP210X_RS485;
						mode_table = rs485_mode_table;	
					}
					else
					{
						g_port_type[i] = CP210X_RS232;
						mode_table = rs232_mode_table;
					}
				}
				else
				{
					printk("cp210x %s dts do not set mode !!!\n", port_name);
					return -1;
				}
			}
			else
			{
				if(g_port_type[i] == CP210X_RS485)
				{
					mode_table = rs485_mode_table;	
				}
				else
				{
					g_port_type[i] = CP210X_RS232;
					mode_table = rs232_mode_table;
				}
			}

			printk("cp210x %s uart_type %d \n", port_name, g_port_type[i]);
			sel0_gpio = of_get_named_gpio_flags(port_np, "sel0_gpio", 0, &flags);
			sel1_gpio = of_get_named_gpio_flags(port_np, "sel1_gpio", 0, &flags);
			term_gpio = of_get_named_gpio_flags(port_np, "term_gpio", 0, &flags);
			
			if(gpio_is_valid(sel0_gpio) && gpio_is_valid(sel1_gpio))
			{
				gpio_name[0] = devm_kasprintf(dev, GFP_KERNEL, "cp210x port%d sel0", i);
				if(mode_table[0] == 0)
					gpio_request_one(sel0_gpio, GPIOF_OUT_INIT_LOW, gpio_name[0]);
				else
					gpio_request_one(sel0_gpio, GPIOF_OUT_INIT_HIGH, gpio_name[0]);

				gpio_name[1] = devm_kasprintf(dev, GFP_KERNEL, "cp210x port%d sel1", i);
				if(mode_table[1] == 0)
					gpio_request_one(sel1_gpio, GPIOF_OUT_INIT_LOW, gpio_name[1]);
				else
					gpio_request_one(sel1_gpio, GPIOF_OUT_INIT_HIGH, gpio_name[1]);
				
				if(gpio_is_valid(term_gpio))
				{
					gpio_name[2] = devm_kasprintf(dev, GFP_KERNEL, "cp210x port%d term", i);
					if(mode_table[2] == 0)
						gpio_request_one(term_gpio, GPIOF_OUT_INIT_LOW, gpio_name[2]);
					else
						gpio_request_one(term_gpio, GPIOF_OUT_INIT_HIGH, gpio_name[2]);
				}
			}
			else
			{
				printk("cp210x get sel0 or sel1 gpio error!!!\n");
				return -1;
			}
		}
		else
		{
			printk("cp210x %s do not config in dts\n", port_name);
		}
	}

	if (device_create_file(dev, &dev_attr_mode))
	{
        dev_err(dev, "sys file creation failed\n");
        return -ENODEV;
	}

	return 0;
}

static const struct of_device_id cp210x_232_485_of_match[] = {
	{ .compatible = CP210X_232_485_MODNAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, cp210x_232_485_of_match);

static struct platform_driver cp210x_232_485_driver = {
	.probe		= cp210x_232_485_probe,
	.driver		= {
		.name	= CP210X_232_485_MODNAME,
		.owner	= THIS_MODULE,
		.of_match_table	=  of_match_ptr(cp210x_232_485_of_match),
	}
};

static int __init cp210x_232_485_init(void)
{
	return platform_driver_register(&cp210x_232_485_driver);
}

static void __exit cp210x_232_485_exit(void)
{
	platform_driver_unregister(&cp210x_232_485_driver);
}

subsys_initcall(cp210x_232_485_init);
module_exit(cp210x_232_485_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("cp210x set rs232 or rs485 driver for advantech");
