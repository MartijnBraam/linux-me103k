/* ASUS 2013 for vcm driver, AD5823 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/proc_fs.h>
#include <asm/mach-types.h>

enum {
	ASUS_CUSTOM_IOCTL_NUMBASE = 100,
	ASUS_CUSTOM_IOCTL_SET_DAC,
	ASUS_CUSTOM_IOCTL_GET_DAC,
};

#define VCM_NAME "ad5823"
#define SENSOR_MAX_RETRIES 3
#define VCM_SET_DAC _IOWR('o', ASUS_CUSTOM_IOCTL_SET_DAC, __u32)
#define VCM_GET_DAC _IOWR('o', ASUS_CUSTOM_IOCTL_GET_DAC, __u32)

typedef struct
{
	unsigned int addr;
	unsigned int value;
} custom_vcm_package;

struct vcm_info {
	struct i2c_client *client;
};
static struct vcm_info *info;

static int vcm_open(struct inode *inode, struct file *file)
{
	pr_debug("open vcm_dev_ctrl\n");
	file->private_data = info;
	return 0;
}

static int vcm_release(struct inode *inode, struct file *file)
{
	pr_debug("release vcm_dev_ctrl\n");
	file->private_data = NULL;
	return 0;
}

static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = 0x0C;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr & 0xff);

	msg[1].addr = 0x0C;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 1;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter){
		pr_info("%s client->adapter is null",__func__);
		return -ENODEV;
	}

//	pr_info("%s reg(0x%x)=0x%x",__func__,addr,val);

	data[0] = (u8) (addr & 0xff);
	data[1] = (u8) (val & 0xff);

	msg.addr = 0x0C;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
			addr, val);
	} while (retry <= SENSOR_MAX_RETRIES);

	if (err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}

static long vcm_ioctl(struct file *file,
                                 unsigned int cmd, unsigned long arg)
{
	struct vcm_info *info = file->private_data;
	custom_vcm_package vcm_dac;
	u16 addr, value;
	switch (cmd) {
		case VCM_SET_DAC:
			if (copy_from_user(&vcm_dac,(void *)arg, sizeof(custom_vcm_package)))
				return -EFAULT;
			pr_info("set_dac: 0x%x", vcm_dac.value);
			addr = vcm_dac.addr;
			value = (vcm_dac.value & 0xFF00) >> 8;
			pr_info("i2c addr: 0x%x value: 0x%x", addr, value);
			sensor_write_reg(info->client, addr, value);
			addr = vcm_dac.addr + 1;
			value = vcm_dac.value & 0xFF;
			pr_info("i2c addr: 0x%x value: 0x%x", addr, value);
			sensor_write_reg(info->client, addr, value);
			break;
		case VCM_GET_DAC:
			if (copy_from_user(&vcm_dac,(void *)arg, sizeof(custom_vcm_package)))
				return -EFAULT;
			addr = vcm_dac.addr;
			sensor_read_reg(info->client, addr, &value);
			vcm_dac.value = value;
			if (copy_to_user((void *)arg, &vcm_dac, sizeof(custom_vcm_package)))
				return -EFAULT;
			break;
		default:
			pr_info("ioctl fail");
			break;
	}
	return 0;
}

static const struct file_operations vcm_fileops = {
	.owner = THIS_MODULE,
	.open = vcm_open,
	.release = vcm_release,
	.unlocked_ioctl = vcm_ioctl,
};

struct miscdevice vcm_device = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = VCM_NAME,
	.fops   = &vcm_fileops,
};

int32_t vcm_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	pr_info("%s +++ \n",__func__);

	info = kzalloc(sizeof(struct vcm_info), GFP_KERNEL);
	if (!info) {
		pr_err("vcm : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&vcm_device);
	if (err) {
		pr_err("vcm : Unable to register misc device!\n");
		kfree(info);
		return err;
	}
	info->client = client;

	i2c_set_clientdata(client, info);

	pr_info("%s --- \n",__func__);
	return 0;
}

static int vcm_remove(struct i2c_client *client)
{
	struct vcm_info *info;

	pr_info("yuv %s\n",__func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&vcm_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ad5823_i2c_id[] = {
	{VCM_NAME, 0},
	{ }
};

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = VCM_NAME,
		.owner = THIS_MODULE,
	},
	.id_table = ad5823_i2c_id,
	.probe  = vcm_probe,
	.remove = vcm_remove,
};

static int __init vcm_init(void)
{
	pr_info("%s()\n", __func__);
	return  i2c_add_driver(&ad5823_i2c_driver);
}

static void __exit vcm_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&ad5823_i2c_driver);
}

module_init(vcm_init);
module_exit(vcm_exit);
