/*
 * Seiko Instruments S-35390A RTC Driver
 *
 * Copyright (c) 2007 Byron Bradley
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/i2c.h>
#include <linux/bitrev.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define PT7C4337_BYTE_SECS	0
#define PT7C4337_BYTE_MINS	1
#define PT7C4337_BYTE_HOURS	2
#define PT7C4337_BYTE_WDAY	3
#define PT7C4337_BYTE_DAY	4
#define PT7C4337_BYTE_MONTH	5
#define PT7C4337_BYTE_YEAR	6

#define PT7C4337_CMD_CONTROL	0x0e
#define PT7C4337_CMD_STATUS		0x0f

/* flags for STATUS */
#define PT7C4337_HOURS_24H		0x40
#define PT7C4337_STATUS_OSF		0x80
#define PT7C4337_CONTROL_ETIME	0x80

static const struct i2c_device_id pt7c4337_id[] = {
	{ "pt7c4337", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pt7c4337_id);

struct pt7c4337 {
	struct i2c_client *client;
	struct rtc_device *rtc;
	int twentyfourhour;
};

static int pt7c4337_set_reg(struct pt7c4337 *pt7c4337, char *buf, int len)
{
	struct i2c_client *client = pt7c4337->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.len = len,
			.buf = buf
		},
	};

	if ((i2c_transfer(client->adapter, msg, 1)) != 1)
		return -EIO;

	return 0;
}

static int pt7c4337_get_reg(struct pt7c4337 *pt7c4337, char reg, char *buf, int len)
{
	struct i2c_client *client = pt7c4337->client;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		}
	};

	if ((i2c_transfer(client->adapter, msg, 2)) != 2)
		return -EIO;

	return 0;
}

/*
 * Returns <0 on error, 0 if rtc is setup fine and 1 if the chip was reset.
 * To keep the information if an irq is pending, pass the value read from
 * STATUS1 to the caller.
 */
static int pt7c4337_init(struct pt7c4337 *pt7c4337, char *status)
{
	char buf[2];
	int ret;

	//Enable oscillator and time count chain
	buf[0] = PT7C4337_CMD_CONTROL;
	buf[1] = 0x18;
	ret = pt7c4337_set_reg(pt7c4337, buf, 2);
	if (ret < 0)
		return ret;

	//clear Oscillator Stop Flag
	buf[0] = PT7C4337_CMD_STATUS;
	buf[1] = 0;
	ret = pt7c4337_set_reg(pt7c4337, buf, 2);
	if (ret < 0)
		return ret;

	//set 24 hour mode
	*status = 0;
	ret = pt7c4337_get_reg(pt7c4337, PT7C4337_BYTE_HOURS, status, 1);
	if (ret < 0)
		return ret;
	*status &= ~PT7C4337_HOURS_24H;
	buf[0] = PT7C4337_BYTE_HOURS;
	buf[1] = *status;
	ret = pt7c4337_set_reg(pt7c4337, buf, 2);
	if (ret < 0)
		return ret;

	return 1;
}

static char pt7c4337_hr2reg(struct pt7c4337 *pt7c4337, int hour)
{
	if (pt7c4337->twentyfourhour)
		return bin2bcd(hour);

	if (hour < 12)
		return bin2bcd(hour);

	return 0x20 | bin2bcd(hour - 12);
}

static int pt7c4337_reg2hr(struct pt7c4337 *pt7c4337, char reg)
{
	unsigned hour;

	if (pt7c4337->twentyfourhour)
		return bcd2bin(reg & 0x3f);

	hour = bcd2bin(reg & 0x1f);
	if (reg & 0x20)
		hour += 12;

	return hour;
}

static int pt7c4337_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct pt7c4337	*pt7c4337 = i2c_get_clientdata(client);
	int err;
	char buf[8];

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d mday=%d, "
		"mon=%d, year=%d, wday=%d\n", __func__, tm->tm_sec,
		tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_wday);

	buf[0] = PT7C4337_BYTE_SECS;
	buf[1+PT7C4337_BYTE_YEAR] = bin2bcd(tm->tm_year - 100);
	buf[1+PT7C4337_BYTE_MONTH] = bin2bcd(tm->tm_mon + 1);
	buf[1+PT7C4337_BYTE_DAY] = bin2bcd(tm->tm_mday);
	buf[1+PT7C4337_BYTE_WDAY] = bin2bcd(tm->tm_wday + 1);
	buf[1+PT7C4337_BYTE_HOURS] = pt7c4337_hr2reg(pt7c4337, tm->tm_hour);
	buf[1+PT7C4337_BYTE_MINS] = bin2bcd(tm->tm_min);
	buf[1+PT7C4337_BYTE_SECS] = bin2bcd(tm->tm_sec);

	err = pt7c4337_set_reg(pt7c4337, buf, sizeof(buf));

	return err;
}

static int pt7c4337_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct pt7c4337 *pt7c4337 = i2c_get_clientdata(client);
	char buf[7];
	int err;

	err = pt7c4337_get_reg(pt7c4337, PT7C4337_BYTE_SECS, buf, sizeof(buf));
	if (err < 0)
		return err;

	tm->tm_sec = bcd2bin(buf[PT7C4337_BYTE_SECS]);
	tm->tm_min = bcd2bin(buf[PT7C4337_BYTE_MINS]);
	tm->tm_hour = pt7c4337_reg2hr(pt7c4337, buf[PT7C4337_BYTE_HOURS]);
	tm->tm_wday = bcd2bin(buf[PT7C4337_BYTE_WDAY]) - 1;
	tm->tm_mday = bcd2bin(buf[PT7C4337_BYTE_DAY]);
	tm->tm_mon = bcd2bin(buf[PT7C4337_BYTE_MONTH]) - 1;
	tm->tm_year = bcd2bin(buf[PT7C4337_BYTE_YEAR]) + 100;

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, mday=%d, "
		"mon=%d, year=%d, wday=%d\n", __func__, tm->tm_sec,
		tm->tm_min, tm->tm_hour, tm->tm_mday, tm->tm_mon, tm->tm_year,
		tm->tm_wday);

	return rtc_valid_tm(tm);
}

static int pt7c4337_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return pt7c4337_get_datetime(to_i2c_client(dev), tm);
}

static int pt7c4337_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return pt7c4337_set_datetime(to_i2c_client(dev), tm);
}

static const struct rtc_class_ops pt7c4337_rtc_ops = {
	.read_time	= pt7c4337_rtc_read_time,
	.set_time	= pt7c4337_rtc_set_time,
};

static struct i2c_driver pt7c4337_driver;

static int pt7c4337_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err;
	struct pt7c4337 *pt7c4337;
	char status;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	pt7c4337 = devm_kzalloc(&client->dev, sizeof(struct pt7c4337),
				GFP_KERNEL);
	if (!pt7c4337) {
		err = -ENOMEM;
		goto exit;
	}

	pt7c4337->client = client;
	i2c_set_clientdata(client, pt7c4337);

	err = pt7c4337_init(pt7c4337, &status);
	if (err < 0) {
		dev_err(&client->dev, "error initting chip\n");
		goto exit;
	}

	if (status & PT7C4337_HOURS_24H)
		pt7c4337->twentyfourhour = 1;
	else
		pt7c4337->twentyfourhour = 0;

	pt7c4337->rtc = devm_rtc_device_register(&client->dev,
					pt7c4337_driver.driver.name,
					&pt7c4337_rtc_ops, THIS_MODULE);

	if (IS_ERR(pt7c4337->rtc)) {
		err = PTR_ERR(pt7c4337->rtc);
		goto exit;
	}

	return 0;

exit:
	return err;
}

static int pt7c4337_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver pt7c4337_driver = {
	.driver		= {
		.name	= "rtc-pt7c4337",
	},
	.probe		= pt7c4337_probe,
	.remove		= pt7c4337_remove,
	.id_table	= pt7c4337_id,
};

module_i2c_driver(pt7c4337_driver);

MODULE_DESCRIPTION("PT7C4337 RTC driver");
MODULE_LICENSE("GPL");
