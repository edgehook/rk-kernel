#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/module.h>

#include <linux/reboot.h>


static bool early_enable = 0;
module_param(early_enable, bool, 0);
MODULE_PARM_DESC(early_enable,
        "adv_bootprocess is started on module insertion (default=0)");

static int timer_flag = 0;
struct delayed_work adv_delay_work;

static void adv_delay_work_handler(struct work_struct *work)
{
	if(timer_flag == 0)
	{
		kernel_restart("backup");
	}
}

static ssize_t timer_flag_show(struct device *dev,
                              struct device_attribute *attr, char *buf)        
{
	size_t status;
	status = sprintf(buf, "%d\n", timer_flag);

	return status;
}
 
static ssize_t timer_flag_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf, size_t len)        
{
	long value;
	size_t status;

	status = kstrtol(buf, 0, &value);
	timer_flag = value;
	status = len;

	return status;
}

static DEVICE_ATTR_RW(timer_flag);

struct file_operations adv_timer_ops={
    .owner  = THIS_MODULE,
};
 
static int major;
static struct class *cls;
static int adv_timer_init(void)
{
	struct device *mydev;

	if (early_enable)
	{
		major=register_chrdev(0,"timer_flag", &adv_timer_ops);
		cls=class_create(THIS_MODULE, "adv_bootprocess_class");

		mydev = device_create(cls, 0, MKDEV(major,0),NULL,"adv_bootprocess_device");    

		if(sysfs_create_file(&(mydev->kobj), &dev_attr_timer_flag.attr)) {
			return -1;
		}

		printk(" Enabled delay work\n");
		INIT_DELAYED_WORK(&adv_delay_work, adv_delay_work_handler );
		mod_delayed_work(system_wq, &adv_delay_work, msecs_to_jiffies(60000));
	}
	else
		printk(" Disabled delay work\n");

    return 0;
}
 
static void adv_timer_exit(void)
{
	if (early_enable)
	{
		device_destroy(cls, MKDEV(major,0));
		class_destroy(cls);
		unregister_chrdev(major, "timer_flag");
	}
}
 
late_initcall_sync(adv_timer_init);
module_exit(adv_timer_exit);
MODULE_LICENSE("GPL");

