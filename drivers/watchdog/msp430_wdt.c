/*
 * Advantech MSP430 Watchdog driver
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

#define ADV_WDT_MAX_TIME	7200		/* in seconds */
#define ADV_WDT_DEFAULT_TIME	60		/* in seconds */

#define WDOG_SEC_TO_COUNT(s)	(s * 10)	/* Time unit for register: 100ms */

#define ADV_WDT_STATUS_OPEN	0
#define ADV_WDT_STATUS_STARTED	1
#define ADV_WDT_EXPECT_CLOSE	2

#define DRIVER_NAME "msp430-wdt-i2c"

#define REG_WDT_WATCHDOG_TIME_OUT	0x15
#define REG_WDT_POWER_OFF_TIME 		0x16
#define REG_WDT_INT_PRE_TIME 		0x17
#define REG_WDT_REMAIN_TIME_OUT		0x25
#define REG_WDT_REMAIN_PRE_TIME 	0x26
#define REG_WDT_VERSION 			0x27
#define REG_WDT_POWER_BTN_MODE 		0x28
#define REG_WDT_HANDSHAKE 			0x30

static struct i2c_client *msp430_client;

static struct {
	unsigned int timeout;
	unsigned int remain_time;
	unsigned long status;
	int gpio_wdt_ping;
	int wdt_ping_status;
	int gpio_wdt_en;
	int wdt_en_off;
	unsigned char version[2];
} msp430_wdt;

static struct miscdevice msp430_wdt_miscdev;

static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(nowayout, bool, 0);

MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");


static unsigned int timeout = ADV_WDT_DEFAULT_TIME;

module_param(timeout, uint, 0);

MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
				__MODULE_STRING(ADV_WDT_DEFAULT_TIME) ")");

static struct watchdog_info msp430_wdt_info = {
	.identity = "Advantech watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.firmware_version = 0,
};

static int msp430_wdt_i2c_write_reg(struct i2c_client *client, u8 reg, void *buf, size_t len)
{
	u8 val[1 + len];
	struct i2c_msg msg[1] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(val),
			.buf = val,
		}
	};

	val[0] = reg;
	memcpy(&val[1], buf, len);

	if (i2c_transfer(client->adapter, msg, 1) != 1) {
		dev_err(&client->dev, "msp430_wdt_i2c_write: i2c transfer failed\n");
		return -EIO;
	}
	
	msleep(10);
	return 0;
}

static int msp430_wdt_i2c_read_reg(struct i2c_client *client, u8 reg, void *buf, size_t len)
{
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(&client->dev, "msp430_wdt_i2c_read: i2c transfer failed\n");
		return -EIO;
	}
	
	msleep(10);
	return 0;
}

static int msp430_wdt_i2c_set_timeout(struct i2c_client *client, unsigned int val)
{
	int ret = 0;
	val = WDOG_SEC_TO_COUNT(val) & 0x0000FFFF;
	ret = msp430_wdt_i2c_write_reg(client, REG_WDT_WATCHDOG_TIME_OUT, &val, sizeof(val));
	if (ret)
		return -EIO;
	return 0;
}

static int msp430_wdt_i2c_read_timeout(struct i2c_client *client, unsigned int *val)
{
	int ret = 0;
	
	ret = msp430_wdt_i2c_read_reg(client, REG_WDT_WATCHDOG_TIME_OUT, val, sizeof(val));
	if (ret)
		return -EIO;
	return 0;
}

static int msp430_wdt_i2c_read_remain_time(struct i2c_client *client, unsigned int *val)
{
	int ret = 0;
	
	ret = msp430_wdt_i2c_read_reg(client, REG_WDT_REMAIN_TIME_OUT, val, sizeof(val));
	if (ret)
		return -EIO;
	return 0;
}

static int msp430_wdt_i2c_read_version(struct i2c_client *client, unsigned int *val)
{
	int ret = 0;
	
	ret = msp430_wdt_i2c_read_reg(client, REG_WDT_VERSION, val, sizeof(val));
	if (ret)
		return -EIO;
	return 0;
}

static inline void msp430_wdt_ping(void)
{
	msp430_wdt.wdt_ping_status= !msp430_wdt.wdt_ping_status;
	gpio_set_value(msp430_wdt.gpio_wdt_ping, msp430_wdt.wdt_ping_status);
}

static void msp430_wdt_start(void)
{
	if (!test_and_set_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status)) 
	{
		/* at our first start we enable clock and do initialisations */
		gpio_set_value(msp430_wdt.gpio_wdt_en, !msp430_wdt.wdt_en_off);
	} 

	/* Watchdog is enabled - time to reload the timeout value */
	msp430_wdt_ping();
}

static void msp430_wdt_stop(void)
{
	msp430_wdt_ping();

	/* we don't need a clk_disable, it cannot be disabled once started.
	 * We use a timer to ping the watchdog while /dev/watchdog is closed */
	gpio_set_value(msp430_wdt.gpio_wdt_en, msp430_wdt.wdt_en_off);
}

static int msp430_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(ADV_WDT_STATUS_OPEN, &msp430_wdt.status))
		return -EBUSY;
	msp430_wdt_start();
	return nonseekable_open(inode, file);
}

static int msp430_wdt_close(struct inode *inode, struct file *file)
{
	if (test_bit(ADV_WDT_EXPECT_CLOSE, &msp430_wdt.status) && !nowayout)
		msp430_wdt_stop();
	else {
		dev_crit(msp430_wdt_miscdev.parent,"Unexpected close: Expect reboot!\n");
		msp430_wdt_ping();
	}

	clear_bit(ADV_WDT_EXPECT_CLOSE, &msp430_wdt.status);
	clear_bit(ADV_WDT_STATUS_OPEN, &msp430_wdt.status);
	clear_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status);
	return 0;
}

static long msp430_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	unsigned int new_value = 0;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		msp430_wdt_ping();
		return copy_to_user(argp, &msp430_wdt_info,
			sizeof(struct watchdog_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
		return put_user(msp430_wdt.status, p);

	case WDIOC_KEEPALIVE:
		msp430_wdt_ping();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;

		if ((new_value < 1) || (new_value > ADV_WDT_MAX_TIME))
		{
			pr_err("timeout value must be between 1 and %d\n", ADV_WDT_MAX_TIME);
			return -EINVAL;
		}
		msp430_wdt.timeout = new_value;
		msp430_wdt_i2c_set_timeout(msp430_client, msp430_wdt.timeout);
		msp430_wdt_ping();
	
		/* Fallthrough to return current value */
	case WDIOC_GETTIMEOUT:
		msp430_wdt_i2c_read_timeout(msp430_client, &msp430_wdt.timeout);
		return put_user(msp430_wdt.timeout & 0xFFFF, p);

	case WDIOC_GETTIMELEFT:
		msp430_wdt_i2c_read_remain_time(msp430_client, &msp430_wdt.remain_time);
		return put_user(msp430_wdt.remain_time & 0xFFFF, p);
	
	default:
		return -ENOTTY;
	}
}

static ssize_t msp430_wdt_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	size_t i;
	char c;

	if (len == 0)	/* Can we see this even ? */
		return 0;

	clear_bit(ADV_WDT_EXPECT_CLOSE, &msp430_wdt.status);
	/* scan to see whether or not we got the magic character */
	for (i = 0; i != len; i++) {
		if (get_user(c, data + i))
			return -EFAULT;
		if (c == 'V')
			set_bit(ADV_WDT_EXPECT_CLOSE, &msp430_wdt.status);
	}

	msp430_wdt_ping();
	
	return len;
}

static const struct file_operations msp430_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = msp430_wdt_ioctl,
	.open = msp430_wdt_open,
	.release = msp430_wdt_close,
	.write = msp430_wdt_write,
};

static struct miscdevice msp430_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &msp430_wdt_fops,
};

void msp430_wdt_restart(void)
{
	if(msp430_client) {
		msp430_wdt_i2c_set_timeout(msp430_client, 1);
		/* don't sleep in restart API */
		msp430_wdt_start();

		/* wait for wdog to fire */
		mdelay(2000);
	}
}

static int msp430_wdt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	unsigned int tmp_version;
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags flags;
	
	if (!np)
	{
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		return -ENODEV;
	}

	i2c_set_clientdata(client, &msp430_wdt);

	//Setting GPIO
	msp430_wdt.gpio_wdt_en = of_get_named_gpio_flags(np, "wdt-en", 0, &flags);
	if (!gpio_is_valid(msp430_wdt.gpio_wdt_en))
		return -ENODEV;	
	msp430_wdt.wdt_en_off = flags & OF_GPIO_ACTIVE_LOW;
	ret = devm_gpio_request_one(&client->dev, msp430_wdt.gpio_wdt_en,
				GPIOF_OUT_INIT_LOW, "msp430_wdt.wdt_en");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed: %d\n", ret);
		return ret;
	}
	gpio_direction_output(msp430_wdt.gpio_wdt_en, msp430_wdt.wdt_en_off);
	
	msp430_wdt.gpio_wdt_ping = of_get_named_gpio_flags(np, "wdt-ping", 0, &flags);
	if (!gpio_is_valid(msp430_wdt.gpio_wdt_ping))
		return -ENODEV;	

	ret = devm_gpio_request_one(&client->dev, msp430_wdt.gpio_wdt_ping, 
				GPIOF_OUT_INIT_LOW, "msp430_wdt.wdt_ping");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed: %d\n", ret);
		return ret;
	}
	msp430_wdt.wdt_ping_status=flags & OF_GPIO_ACTIVE_LOW;
	gpio_direction_output(msp430_wdt.gpio_wdt_ping, !msp430_wdt.wdt_ping_status);
	msleep(10);
	gpio_direction_output(msp430_wdt.gpio_wdt_ping, msp430_wdt.wdt_ping_status);

	msp430_wdt.timeout = clamp_t(unsigned, timeout, 1, ADV_WDT_MAX_TIME);
	if (msp430_wdt.timeout != timeout)
		dev_warn(&client->dev, "Initial timeout out of range! "
			"Clamped from %u to %u\n", timeout, msp430_wdt.timeout);

	ret = msp430_wdt_i2c_set_timeout(client, msp430_wdt.timeout);
	if (ret)
	{
		pr_err("Set watchdog timeout err=%d\n", ret);
		goto fail;
	}

	ret = msp430_wdt_i2c_read_version(client, &tmp_version);
	if (ret == 0 )
	{
		msp430_wdt.version[0]= (tmp_version & 0xFF00) >> 8;
		msp430_wdt.version[1]= tmp_version & 0xFF;
		msp430_wdt_info.firmware_version = (unsigned int)(msp430_wdt.version[1] - '0') * 10 + (unsigned int)(msp430_wdt.version[0] - '0');
	} else {
		pr_err("Read watchdog version err=%d\n", ret);
		goto fail;
	}

	msp430_client = client;
	msp430_wdt_miscdev.parent = &client->dev;
	ret = misc_register(&msp430_wdt_miscdev);
	if (ret)
	{
		pr_err("cannot register miscdev on minor=%d (err=%d)\n",
	     WATCHDOG_MINOR, ret);
		goto fail;
	}

	dev_info(&client->dev,
						"Advantech MSP430 Watchdog Timer enabled. timeout=%ds (nowayout=%d), Ver.%d\n",
						msp430_wdt.timeout, nowayout, msp430_wdt_info.firmware_version);

	return 0;

fail:
	msp430_wdt_miscdev.parent = NULL;
	return ret;
}

static int __exit msp430_wdt_i2c_remove(struct i2c_client *client)
{
	misc_deregister(&msp430_wdt_miscdev);

	if (test_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status))
	{	
		gpio_set_value(msp430_wdt.gpio_wdt_en, msp430_wdt.wdt_en_off);
		dev_crit(msp430_wdt_miscdev.parent, "Device removed: Expect reboot!\n");
	}
	clear_bit(ADV_WDT_EXPECT_CLOSE, &msp430_wdt.status);
	clear_bit(ADV_WDT_STATUS_OPEN, &msp430_wdt.status);
	clear_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status);
	msp430_wdt_miscdev.parent = NULL;

	return 0;
}

static int msp430_wdt_i2c_resume(struct device *dev)
{
	if (test_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status))
	{
		gpio_set_value(msp430_wdt.gpio_wdt_en, !msp430_wdt.wdt_en_off);
		msp430_wdt_i2c_set_timeout(msp430_client, msp430_wdt.timeout);
		msp430_wdt_ping();
	}
	return 0;
}

static int msp430_wdt_i2c_suspend(struct device *dev)
{
	if (test_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status)) {
		msp430_wdt_stop();
	}
	return 0;
}

static void msp430_wdt_i2c_shutdown(struct i2c_client *client)
{
	if (test_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status)) {
		/* we are running, we need to delete the timer but will give
		 * max timeout before reboot will take place */
		gpio_set_value(msp430_wdt.gpio_wdt_en, msp430_wdt.wdt_en_off);

		dev_crit(msp430_wdt_miscdev.parent,"Device shutdown: Expect reboot!\n");
	}
	clear_bit(ADV_WDT_EXPECT_CLOSE, &msp430_wdt.status);
	clear_bit(ADV_WDT_STATUS_OPEN, &msp430_wdt.status);
	clear_bit(ADV_WDT_STATUS_STARTED, &msp430_wdt.status);
}

static const struct i2c_device_id msp430_wdt_i2c_id[] = {
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, msp430_wdt_i2c_id);

static const struct of_device_id msp430_wdt_i2c_dt_ids[] = {
	{ .compatible = "msp430-wdt-i2c", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, msp430_wdt_i2c_dt_ids);

static const struct dev_pm_ops msp430_wdt_device_pm_ops = {
	.resume = msp430_wdt_i2c_resume,
	.suspend = msp430_wdt_i2c_suspend,
};

static struct i2c_driver msp430_wdt_i2c_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = msp430_wdt_i2c_dt_ids,
		   .pm = &msp430_wdt_device_pm_ops,
		   },
	.probe = msp430_wdt_i2c_probe,
	.remove = msp430_wdt_i2c_remove,
	.shutdown	= msp430_wdt_i2c_shutdown,
	.id_table = msp430_wdt_i2c_id,
};

static int __init msp430_wdt_i2c_init(void)
{
	return i2c_add_driver(&msp430_wdt_i2c_driver);
}

static void __exit msp430_wdt_i2c_exit(void)
{
	i2c_del_driver(&msp430_wdt_i2c_driver);
}

module_init(msp430_wdt_i2c_init);
module_exit(msp430_wdt_i2c_exit);

MODULE_DESCRIPTION("Advantech MSP430 Watchdog Driver");
MODULE_LICENSE("GPL");
