// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * fm24.c - handle most I2C EEPROMs
 *
 * Copyright (C) 2005-2008 David Brownell & Wolfram Sang
 * Copyright (C) 2021  Chang.QIng
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/* Address pointer is 16 bit. */
#define FM24_FLAG_ADDR16	BIT(7)
/* sysfs-entry will be read-only. */
#define FM24_FLAG_READONLY	BIT(6)
/* sysfs-entry will be world-readable. */
#define FM24_FLAG_IRUGO		BIT(5)
/* Take always 8 addresses (24c00). */
#define FM24_FLAG_TAKE8ADDR	BIT(4)
/* Does not auto-rollover reads to the next slave address. */
#define FM24_FLAG_NO_RDROL		BIT(1)

/*
 * I2C EEPROMs from most vendors are inexpensive and mostly interchangeable.
 * Differences between different vendor product lines (like Atmel AT24C or
 * MicroChip 24LC, etc) won't much matter for typical read/write access.
 * There are also I2C RAM chips, likewise interchangeable. One example
 * would be the PCF8570, which acts like a 24c02 EEPROM (256 bytes).
 *
 * However, misconfiguration can lose data. "Set 16-bit memory address"
 * to a part with 8-bit addressing will overwrite data. Writing with too
 * big a page size also loses data. And it's not safe to assume that the
 * conventional addresses 0x50..0x57 only hold eeproms; a PCF8563 RTC
 * uses 0x51, for just one example.
 *
 * Accordingly, explicit board-specific configuration data should be used
 * in almost all cases. (One partial exception is an SMBus used to access
 * "SPD" data for DRAM sticks. Those only use 24c02 EEPROMs.)
 *
 * So this driver uses "new style" I2C driver binding, expecting to be
 * told what devices exist. That may be in arch/X/mach-Y/board-Z.c or
 * similar kernel-resident tables; or, configuration data coming from
 * a bootloader.
 *
 * Other than binding model, current differences from "eeprom" driver are
 * that this one handles write access and isn't restricted to 24c02 devices.
 * It also handles larger devices (32 kbit and up) with two-byte addresses,
 * which won't work on pure SMBus systems.
 */

struct fm24_client {
	struct i2c_client *client;
	struct regmap *regmap;
};

struct fm24_data {
	/*
	 * Lock protects against activities from other Linux tasks,
	 * but not from changes by other I2C masters.
	 */
	struct mutex lock;

	unsigned int write_max;
	unsigned int num_addresses;
	unsigned int offset_adj;

	u32 byte_len;
	u16 page_size;
	u8 flags;

	struct nvmem_device *nvmem;
	struct regulator *vcc_reg;
	void (*read_post)(unsigned int off, char *buf, size_t count);

	/*
	 * Some chips tie up multiple I2C addresses; dummy devices reserve
	 * them for us, and we'll use them with SMBus calls.
	 */
	struct fm24_client client[];
};

/*
 * This parameter is to help this driver avoid blocking other drivers out
 * of I2C for potentially troublesome amounts of time. With a 100 kHz I2C
 * clock, one 256 byte read takes about 1/43 second which is excessive;
 * but the 1/170 second it takes at 400 kHz may be quite reasonable; and
 * at 1 MHz (Fm+) a 1/430 second delay could easily be invisible.
 *
 * This value is forced to be a power of two so that writes align on pages.
 */
static unsigned int fm24_io_limit = 128;
module_param_named(io_limit, fm24_io_limit, uint, 0);
MODULE_PARM_DESC(fm24_io_limit, "Maximum bytes per I/O (default 128)");

/*
 * Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned int fm24_write_timeout = 25;
module_param_named(write_timeout, fm24_write_timeout, uint, 0);
MODULE_PARM_DESC(fm24_write_timeout, "Time (in ms) to try writes (default 25)");

struct fm24_chip_data {
	u32 byte_len;
	u8 flags;
	void (*read_post)(unsigned int off, char *buf, size_t count);
};

#define FM24_CHIP_DATA(_name, _len, _flags)				\
	static const struct fm24_chip_data _name = {			\
		.byte_len = _len, .flags = _flags,			\
	}

#define FM24_CHIP_DATA_CB(_name, _len, _flags, _read_post)		\
	static const struct fm24_chip_data _name = {			\
		.byte_len = _len, .flags = _flags,			\
		.read_post = _read_post,				\
	}

/*
 * This routine supports chips which consume multiple I2C addresses. It
 * computes the addressing information to be used for a given r/w request.
 * Assumes that sanity checks for offset happened at sysfs-layer.
 *
 * Slave address and byte offset derive from the offset. Always
 * set the byte address; on a multi-master board, another master
 * may have changed the chip's "current" address pointer.
 */
static struct fm24_client *fm24_translate_offset(struct fm24_data *fm24,
						 unsigned int *offset)
{
	unsigned int i;

	if (fm24->flags & FM24_FLAG_ADDR16) {
		i = *offset >> 16;
		*offset &= 0xffff;
	} else {
		i = *offset >> 8;
		*offset &= 0xff;
	}

	return &fm24->client[i];
}

static struct device *fm24_base_client_dev(struct fm24_data *fm24)
{
	return &fm24->client[0].client->dev;
}

static size_t fm24_adjust_read_count(struct fm24_data *fm24,
				      unsigned int offset, size_t count)
{
	unsigned int bits;
	size_t remainder;

	/*
	 * In case of multi-address chips that don't rollover reads to
	 * the next slave address: truncate the count to the slave boundary,
	 * so that the read never straddles slaves.
	 */
	if (fm24->flags & FM24_FLAG_NO_RDROL) {
		bits = (fm24->flags & FM24_FLAG_ADDR16) ? 16 : 8;
		remainder = BIT(bits) - offset;
		if (count > remainder)
			count = remainder;
	}

	if (count > fm24_io_limit)
		count = fm24_io_limit;

	return count;
}

static ssize_t fm24_regmap_read(struct fm24_data *fm24, char *buf,
				unsigned int offset, size_t count)
{
	unsigned long timeout, read_time;
	struct fm24_client *fm24_client;
	struct i2c_client *client;
	struct regmap *regmap;
	int ret;

	fm24_client = fm24_translate_offset(fm24, &offset);
	regmap = fm24_client->regmap;
	client = fm24_client->client;
	count = fm24_adjust_read_count(fm24, offset, count);

	/* adjust offset for mac and serial read ops */
	offset += fm24->offset_adj;

	timeout = jiffies + msecs_to_jiffies(fm24_write_timeout);
	do {
		/*
		 * The timestamp shall be taken before the actual operation
		 * to avoid a premature timeout in case of high CPU load.
		 */
		read_time = jiffies;

		ret = regmap_bulk_read(regmap, offset, buf, count);
		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
			count, offset, ret, jiffies);
		if (!ret)
			return count;

		usleep_range(1000, 1500);
	} while (time_before(read_time, timeout));

	return -ETIMEDOUT;
}

/*
 * Note that if the hardware write-protect pin is pulled high, the whole
 * chip is normally write protected. But there are plenty of product
 * variants here, including OTP fuses and partial chip protect.
 *
 * We only use page mode writes; the alternative is sloooow. These routines
 * write at most one page.
 */

static size_t fm24_adjust_write_count(struct fm24_data *fm24,
				      unsigned int offset, size_t count)
{
	unsigned int next_page;

	/* write_max is at most a page */
	if (count > fm24->write_max)
		count = fm24->write_max;

	/* Never roll over backwards, to the start of this page */
	next_page = roundup(offset + 1, fm24->page_size);
	if (offset + count > next_page)
		count = next_page - offset;

	return count;
}

static ssize_t fm24_regmap_write(struct fm24_data *fm24, const char *buf,
				 unsigned int offset, size_t count)
{
	unsigned long timeout, write_time;
	struct fm24_client *fm24_client;
	struct i2c_client *client;
	struct regmap *regmap;
	int ret;

	fm24_client = fm24_translate_offset(fm24, &offset);
	regmap = fm24_client->regmap;
	client = fm24_client->client;
	count = fm24_adjust_write_count(fm24, offset, count);
	timeout = jiffies + msecs_to_jiffies(fm24_write_timeout);

	do {
		/*
		 * The timestamp shall be taken before the actual operation
		 * to avoid a premature timeout in case of high CPU load.
		 */
		write_time = jiffies;

		ret = regmap_bulk_write(regmap, offset, buf, count);
		dev_dbg(&client->dev, "write %zu@%d --> %d (%ld)\n",
			count, offset, ret, jiffies);
		if (!ret)
			return count;

		usleep_range(1000, 1500);
	} while (time_before(write_time, timeout));

	return -ETIMEDOUT;
}

static int fm24_read(void *priv, unsigned int off, void *val, size_t count)
{
	struct fm24_data *fm24;
	struct device *dev;
	char *buf = val;
	int i, ret;

	fm24 = priv;
	dev = fm24_base_client_dev(fm24);

	if (unlikely(!count))
		return count;

	if (off + count > fm24->byte_len)
		return -EINVAL;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	/*
	 * Read data from chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&fm24->lock);

	for (i = 0; count; i += ret, count -= ret) {
		ret = fm24_regmap_read(fm24, buf + i, off + i, count);
		if (ret < 0) {
			mutex_unlock(&fm24->lock);
			pm_runtime_put(dev);
			return ret;
		}
	}

	mutex_unlock(&fm24->lock);
	pm_runtime_put(dev);

	if (unlikely(fm24->read_post))
		fm24->read_post(off, buf, i);

	return 0;
}

static int fm24_write(void *priv, unsigned int off, void *val, size_t count)
{
	struct fm24_data *fm24;
	struct device *dev;
	char *buf = val;
	int ret;

	fm24 = priv;
	dev = fm24_base_client_dev(fm24);

	if (unlikely(!count))
		return -EINVAL;

	if (off + count > fm24->byte_len)
		return -EINVAL;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	/*
	 * Write data to chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&fm24->lock);

	while (count) {
		ret = fm24_regmap_write(fm24, buf, off, count);
		if (ret < 0) {
			mutex_unlock(&fm24->lock);
			pm_runtime_put(dev);
			return ret;
		}
		buf += ret;
		off += ret;
		count -= ret;
	}

	mutex_unlock(&fm24->lock);

	pm_runtime_put(dev);

	return 0;
}

FM24_CHIP_DATA(fm24_data_24v10, 1048576 / 8, FM24_FLAG_ADDR16);

static const struct i2c_device_id fm24_ids[] = {
	{ "fm24v10",	(kernel_ulong_t)&fm24_data_24v10 },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, fm24_ids);

static const struct of_device_id fm24_of_match[] = {
	{ .compatible = "cypress,fm24v10",		.data = &fm24_data_24v10 },
	{ /* END OF LIST */ },
};
MODULE_DEVICE_TABLE(of, fm24_of_match);

static const struct acpi_device_id __maybe_unused fm24_acpi_ids[] = {
	{ "INT3499",	(kernel_ulong_t)&fm24_data_24v10 },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(acpi, fm24_acpi_ids);

static const struct fm24_chip_data *fm24_get_chip_data(struct device *dev, const struct i2c_device_id *id)
{
	struct device_node *of_node = dev->of_node;
	const struct fm24_chip_data *cdata;

	/*
	 * The I2C core allows OF nodes compatibles to match against the
	 * I2C device ID table as a fallback, so check not only if an OF
	 * node is present but also if it matches an OF device ID entry.
	 */
	if (of_node && of_match_device(fm24_of_match, dev))
		cdata = of_device_get_match_data(dev);
	else if (id)
		cdata = (void *)id->driver_data;
	else{
		const struct acpi_device_id *aid;

		aid = acpi_match_device(fm24_acpi_ids, dev);
		if (aid)
			cdata = (void *)aid->driver_data;
	}
		

	if (!cdata)
		return ERR_PTR(-ENODEV);

	return cdata;
}

static int fm24_make_dummy_client(struct fm24_data *fm24, unsigned int index,
				  struct regmap_config *regmap_config)
{
	struct regmap *regmap;
	struct i2c_client *base_client, *dummy_client;

	base_client = fm24->client[0].client;
	dummy_client = i2c_new_dummy(base_client->adapter, base_client->addr + index);
	if (IS_ERR(dummy_client))
		return PTR_ERR(dummy_client);

	regmap = devm_regmap_init_i2c(dummy_client, regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	fm24->client[index].client = dummy_client;
	fm24->client[index].regmap = regmap;

	return 0;
}

static unsigned int fm24_get_offset_adj(u8 flags, unsigned int byte_len)
{
	return 0;
}

static int fm24_probe(struct i2c_client *client, const struct i2c_device_id * device_id)
{
	struct regmap_config regmap_config = { };
	struct nvmem_config nvmem_config = { };
	u32 byte_len, page_size, flags;
	const struct fm24_chip_data *cdata;
	struct device *dev = &client->dev;
	bool i2c_fn_i2c;
	unsigned int i, num_addresses;
	struct fm24_data *fm24;
	struct regmap *regmap;
	bool writable;
	u8 test_byte;
	int err;

	i2c_fn_i2c = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	cdata = fm24_get_chip_data(dev, device_id);
	if (IS_ERR(cdata))
		return PTR_ERR(cdata);

	err = device_property_read_u32(dev, "pagesize", &page_size);
	if (err)
		/*
		 * This is slow, but we can't know all eeproms, so we better
		 * play safe. Specifying custom eeprom-types via device tree
		 * or properties is recommended anyhow.
		 */
		page_size = 128;

	flags = cdata->flags;
	if (device_property_present(dev, "read-only"))
		flags |= FM24_FLAG_READONLY;
	if (device_property_present(dev, "no-read-rollover"))
		flags |= FM24_FLAG_NO_RDROL;

	//total size of the eeprom.
	byte_len = cdata->byte_len;
	//no page size concept for fm24. 
	page_size = 128;

	err = device_property_read_u32(dev, "num-addresses", &num_addresses);
	if (err) {
		num_addresses =	DIV_ROUND_UP(byte_len,
			(flags & FM24_FLAG_ADDR16) ? 65536 : 256);
	}

	regmap_config.val_bits = 8;
	regmap_config.reg_bits = (flags & FM24_FLAG_ADDR16) ? 16 : 8;

	regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	fm24 = devm_kzalloc(dev, sizeof(*fm24) + sizeof(struct fm24_client) * num_addresses, GFP_KERNEL);
	if (!fm24)
		return -ENOMEM;

	mutex_init(&fm24->lock);
	fm24->byte_len = byte_len;
	fm24->page_size = page_size;
	fm24->flags = flags;
	fm24->read_post = cdata->read_post;
	fm24->num_addresses = num_addresses;
	fm24->offset_adj = fm24_get_offset_adj(flags, byte_len);
	fm24->client[0].client = client;
	fm24->client[0].regmap = regmap;

	writable = !(flags & FM24_FLAG_READONLY);
	if (writable) {
		fm24->write_max = min_t(unsigned int,
					page_size, fm24_io_limit);
		if (!i2c_fn_i2c && fm24->write_max > I2C_SMBUS_BLOCK_MAX)
			fm24->write_max = I2C_SMBUS_BLOCK_MAX;
	}

	/* use dummy devices for multiple-address chips */
	for (i = 1; i < num_addresses; i++) {
		err = fm24_make_dummy_client(fm24, i, &regmap_config);
		if (err)
			return err;
	}

	if (device_property_read_string(dev, "label", &nvmem_config.name))
		nvmem_config.name = dev_name(dev);
	
	nvmem_config.dev = dev;
	nvmem_config.read_only = !writable;
	nvmem_config.root_only = !(flags & FM24_FLAG_IRUGO);
	nvmem_config.owner = THIS_MODULE;
	nvmem_config.compat = true;
	nvmem_config.base_dev = dev;
	nvmem_config.reg_read = fm24_read;
	nvmem_config.reg_write = fm24_write;
	nvmem_config.priv = fm24;
	nvmem_config.stride = 1;
	nvmem_config.word_size = 1;
	nvmem_config.size = byte_len;

	i2c_set_clientdata(client, fm24);
	
	/* enable runtime pm */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	fm24->nvmem = nvmem_register(&nvmem_config);
	if (IS_ERR(fm24->nvmem)) {
		pm_runtime_disable(dev);
		return PTR_ERR(fm24->nvmem);
	}

	/*
	 * Perform a one-byte test read to verify that the
	 * chip is functional.
	 */
	err = fm24_read(fm24, 0, &test_byte, 1);
	if (err) {
		pm_runtime_disable(dev);
		nvmem_unregister(fm24->nvmem);
		dev_err(dev, "%s is not exists\n", nvmem_config.name);
		return -ENODEV;
	}

	pm_runtime_idle(dev);

	if (writable)
		dev_info(dev, "%u byte %s EEPROM, writable, %u bytes/write\n",
			 byte_len, client->name, fm24->write_max);
	else
		dev_info(dev, "%u byte %s EEPROM, read-only\n",
			 byte_len, client->name);

	return 0;
}

static int fm24_remove(struct i2c_client *client)
{
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static struct i2c_driver fm24_driver = {
	.driver = {
		.name = "fm24",
		.of_match_table = fm24_of_match,
		.acpi_match_table = ACPI_PTR(fm24_acpi_ids),
	},
	.probe = fm24_probe,
	.remove = fm24_remove,
	.id_table = fm24_ids,
};

static int __init fm24_init(void)
{
	return i2c_add_driver(&fm24_driver);
}
module_init(fm24_init);

static void __exit fm24_exit(void)
{
	i2c_del_driver(&fm24_driver);
}
module_exit(fm24_exit);

MODULE_DESCRIPTION("Driver for most I2C FM24 EEPROM");
MODULE_AUTHOR("Qing <cnj1990@sina.com>");
MODULE_LICENSE("GPL");
