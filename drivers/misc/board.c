/*
 * Board Procfs information.
 *
 * Copyright (c) 2017 Chang.Qing <chang.qing@advantech.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 */
#include <linux/atomic.h>
#include <linux/init.h>
#include <linux/module.h>	/* for module_name() */
#include <linux/rwsem.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <asm/uaccess.h>

static const char *machine=NULL;  /* board name */


static const char * of_get_machine_name(void)
{
	struct device_node *root=NULL;
	const char *model;
	int err;

	if (IS_ENABLED(CONFIG_OF)){  /* Devcice tree is enabled ?  */
		root =of_find_node_by_path("/");
		if(!root)
			return NULL;

		err = of_property_read_string(root, "model", &model);
		if (err == 0) {
			return model;
		}
	}

	return NULL;
}

static int proc_board_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", machine);
	return 0;
}

static int proc_board_open(struct inode *inode, struct file *file)
{

	machine = of_get_machine_name();
	if(!machine){
		/* then, the device tree is not supported! you can use
			such as CONFIG_BOARD_NAME to machine. and the following
			just as a example.*/
		machine="unknow"; 		
	}

	return single_open(file, proc_board_show, NULL);
}

int proc_board_release(struct inode *inode, struct file *file)
{
	machine=NULL;
	return single_release(inode, file);
}

static const struct proc_ops proc_board_ops = {
	.proc_open		= proc_board_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= proc_board_release
};


static int __init proc_board_init(void)
{
	proc_create("board", 0, NULL, &proc_board_ops);
	return 0;
}
module_init(proc_board_init);

static void __exit proc_board_exit(void)
{
	remove_proc_entry("board", NULL);
}
module_exit(proc_board_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Board Information");
MODULE_AUTHOR("Chang Qing <chang.qing@advantech.com.cn>");
