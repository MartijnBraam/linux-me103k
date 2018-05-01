/*
 * drivers/power/bq27541_battery.c
 *
 * Gas Gauge driver for TI's BQ27541
 *
 * Copyright (c) 2012, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#define SMBUS_RETRY                                     (0)
#define GPIOPIN_LOW_BATTERY_DETECT	  29
#define BATTERY_POLLING_RATE	(60)
#define DELAY_FOR_CORRECT_CHARGER_STATUS	(5)
#define TEMP_KELVIN_TO_CELCIUS		(2731)
#define MAXIMAL_VALID_BATTERY_TEMP	(1200)
#define BATTERY_PROTECTED_VOLT	(2800)
#define USB_NO_Cable	0
#define USB_DETECT_CABLE	1
#define USB_SHIFT	0
#define AC_SHIFT	1
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* Battery flags bit definitions */
#define BATT_REG_CNTL           0x00
#define BATT_STS_DSG		0x0001
#define BATT_STS_SOCF		0x0002
#define BATT_STS_FC		0x0200

/* Battery Control Subcommands */
#define CONTROL_STATUS          0x0000
#define DEVICE_TYPE             0x0001
#define FW_VERSION              0x0002
#define DF_VERSION              0x001F

#define THERMAL_RULE1 1
#define THERMAL_RULE2 2

/* Debug Message */
#define BAT_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "%s " format , __FUNCTION__ , ## arg)

#define BAT_ERR(format, arg...)		\
	printk(KERN_ERR format , ## arg)

/* Global variable */
unsigned bq27541_battery_cable_status = 0;
unsigned bq27541_battery_driver_ready = 0;
int ac_on ;
int usb_on ;
extern bool otg_on;
static unsigned int 	battery_current;
static unsigned int  battery_remaining_capacity;
struct workqueue_struct *bq27541_battery_work_queue = NULL;
static unsigned int battery_check_interval = BATTERY_POLLING_RATE;
static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

/* Functions declaration */
static int bq27541_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val);
static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
extern int smb345_charger_enable(bool enable);
extern unsigned  get_usb_cable_status(void);
extern int check_charging_status(void);
//extern int smb345_config_thermal_charging(int temp, int rule);
extern void smb345_recheck_charging_type(void);

module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);

#define BQ27541_DATA(_psp, _addr, _min_value, _max_value)	\
	{								\
		.psp = POWER_SUPPLY_PROP_##_psp,	\
		.addr = _addr,				\
		.min_value = _min_value,		\
		.max_value = _max_value,	\
	}

enum gaugeIC_update_ret_value{
        error_sleep = -4,
        error_check = -3,
        error_write = -2,
        error_read = -1,
        update_success = 0,
};

enum gaugeIC_mode_enum{
        may_broken = 0,
        ROM_mode = 1,
        normal_mode = 2,
};

enum gaugeIC_update_command{
        update_command_read = 0x00,
        update_command_write = 0x01,
        update_command_sleep = 0x02,
};

enum {
       REG_MANUFACTURER_DATA,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};

typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_WIRELESS,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

static struct bq27541_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq27541_data[] = {
       [REG_MANUFACTURER_DATA]	= BQ27541_DATA(PRESENT, 0, 0, 65535),
       [REG_STATE_OF_HEALTH]		= BQ27541_DATA(HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE]			= BQ27541_DATA(TEMP, 0x06, 0, 65535),
	[REG_VOLTAGE]				= BQ27541_DATA(VOLTAGE_NOW, 0x08, 0, 6000),
	[REG_CURRENT]				= BQ27541_DATA(CURRENT_NOW, 0x14, -32768, 32767),
	[REG_TIME_TO_EMPTY]			= BQ27541_DATA(TIME_TO_EMPTY_AVG, 0x16, 0, 65535),
	[REG_TIME_TO_FULL]			= BQ27541_DATA(TIME_TO_FULL_AVG, 0x18, 0, 65535),
	[REG_STATUS]				= BQ27541_DATA(STATUS, 0x0a, 0, 65535),
	[REG_CAPACITY]				= BQ27541_DATA(CAPACITY, 0x20, 0, 100),
};

static enum power_supply_property bq27541_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

void bq27541_check_cable_type(void)
{
      if(bq27541_battery_cable_status == USB_AC_Adapter) {
		 ac_on = 1;
	        usb_on = 0;
	}
	else if(bq27541_battery_cable_status  == USB_Cable) {
		usb_on = 1;
		ac_on = 0;
	}
	else {
		ac_on = 0;
		usb_on = 0;
	}
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret=0;
	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  ac_on)
			val->intval =  1;
		else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
			val->intval =  1;
		else
			val->intval = 0;
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static char *supply_list[] = {
	"battery",
	"ac",
#ifndef REMOVE_USB_POWER_SUPPLY
	"usb",
#endif
};

static struct power_supply bq27541_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq27541_properties,
		.num_properties = ARRAY_SIZE(bq27541_properties),
		.get_property	= bq27541_get_property,
       },
	{
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property	= power_get_property,
	},
#ifndef REMOVE_USB_POWER_SUPPLY
	{
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties =power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#endif
};

static DEFINE_MUTEX(lock);

static struct bq27541_device_info {
	struct i2c_client *client;
	struct i2c_client *ROM_mode_client;
	struct delayed_work status_poll_work;
	struct delayed_work low_low_bat_work;
	struct miscdevice battery_misc;
	struct wake_lock low_battery_wake_lock;
	struct wake_lock cable_wake_lock;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_low_battery_detect;
	int irq_battery_detect;
	int bat_status;
	int bat_temp;
	int bat_vol;
	int bat_current;
	int bat_capacity;
	int bat_limit_off;
	int bat_capacity_zero_count;
	unsigned int old_capacity;
	unsigned int cap_err;
	int old_temperature;
	bool temp_err;
	unsigned int prj_id;
	spinlock_t lock;
} *bq27541_device;

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single)
{
	struct i2c_client *client = bq27541_device->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

int bq27541_smbus_read_data(int reg_offset,int byte,int *rt_value)
{
     s32 ret=-EINVAL;
     int count=0;

	do {
		ret = bq27541_read_i2c(bq27541_data[reg_offset].addr, rt_value, 0);
	} while((ret<0)&&(++count<=SMBUS_RETRY));
	return ret;
}

int bq27541_smbus_write_data(int reg_offset,int byte, unsigned int value)
{
     s32 ret = -EINVAL;
     int count=0;

	do{
		if(byte){
			ret = i2c_smbus_write_byte_data(bq27541_device->client,bq27541_data[reg_offset].addr,value&0xFF);
		}
		else{
			ret = i2c_smbus_write_word_data(bq27541_device->client,bq27541_data[reg_offset].addr,value&0xFFFF);
		}
	}while((ret<0) && (++count<=SMBUS_RETRY));
	return ret;
}

static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int status=!bq27541_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}

static ssize_t show_battery_INFO(struct device *dev, struct device_attribute *devattr, char *buf)
{

        char tmp_buf[64];
        int ret;
        int temp;

        sprintf(tmp_buf, "--HPA02254 INFO--\n");
        strcpy(buf, tmp_buf);

        if (!bq27541_device->client)
                return sprintf(buf, "device does not probe!\n");

        i2c_smbus_write_word_data(bq27541_device->client, BATT_REG_CNTL, CONTROL_STATUS&0xFFFF);
	msleep(50);
        ret = bq27541_smbus_read_data(BATT_REG_CNTL, 0, &temp);
        if (ret < 0) {
                return sprintf(buf, "CONTROL_STATUS\n");
        }
        sprintf(tmp_buf, "CONTROL_STATUS: %x\n", temp);
        strcat(buf, tmp_buf);

        i2c_smbus_write_word_data(bq27541_device->client, BATT_REG_CNTL, DEVICE_TYPE&0xFFFF);
	msleep(50);
        ret = bq27541_smbus_read_data(BATT_REG_CNTL, 0, &temp);
        if (ret < 0) {
                return sprintf(buf, "DEVICE_TYPE\n");
        }
        sprintf(tmp_buf, "DEVICE_TYPE: %x\n", temp);
        strcat(buf, tmp_buf);

        i2c_smbus_write_word_data(bq27541_device->client, BATT_REG_CNTL, FW_VERSION&0xFFFF);
	msleep(50);
        ret = bq27541_smbus_read_data(BATT_REG_CNTL, 0, &temp);
        if (ret < 0) {
                return sprintf(buf, "FW_VERSION fails\n");
        }
        sprintf(tmp_buf, "FW_VERSION: %x\n", temp);
        strcat(buf, tmp_buf);

        i2c_smbus_write_word_data(bq27541_device->client, BATT_REG_CNTL, DF_VERSION&0xFFFF);
	msleep(50);
        ret = bq27541_smbus_read_data(BATT_REG_CNTL, 0, &temp);
        if (ret < 0) {
                return sprintf(buf, "DF_VERSION fails\n");
        }
        sprintf(tmp_buf, "DF_VERSION: %x\n", temp);
        strcat(buf, tmp_buf);

        return strlen(buf);

}

#ifdef NOT_USER
static int close_file(struct file *fp)
{
        filp_close(fp,NULL);

        return 0;
}

static struct file *open_file(char *path,int flag,int mode)
{
        struct file *fp;

        fp = filp_open(path, flag, 0);

        if(fp) return fp;
        else return NULL;
}

static void exit_kernel_to_read_file(mm_segment_t origin_fs)
{
        set_fs(origin_fs);
}

static mm_segment_t enable_kernel_to_read_file_and_return_origin_fs(void)
{
        mm_segment_t old_fs;

        old_fs = get_fs();

        set_fs(KERNEL_DS);

        return old_fs;
}

static int __write_or_read_or_sleep_gauge_firmware_data(u8 dffs_command, u8 buff[], u8 buff_length)
{
        struct i2c_client *client = NULL;
        u8 data_length = buff_length - 2;               // 16 00 13 (address reg data) data_length = 3 - 2
        u8 read_buff[data_length];
        int i;
        int sleep_time;

        if(dffs_command != update_command_sleep){
                if(buff[0] == 0xAA)             // normail mode address
                        client = bq27541_device->client;
                else if(buff[0] == 0x16)        // rom mode address
                        client = bq27541_device->ROM_mode_client;
                else                                            // invalid address
                        return -EINVAL;
        }

        switch(dffs_command){
                case update_command_read:       //read command
                        if(i2c_smbus_read_i2c_block_data(client, buff[1], data_length, read_buff) < 0){
                                BAT_NOTICE("error readr: i2c_smbus_read_i2c_block_data return error\n");
                                return -EINVAL;
                        }

                        BAT_NOTICE("gauge check return data: ");
                        for(i = 0; i < data_length; i++)
                                BAT_NOTICE("%02x", read_buff[i]);
                        BAT_NOTICE("\n");

                        for(i = 0; i < data_length; i++){               // check the data of the check sum
                                if(read_buff[i] != buff[2+i]){
                                        BAT_NOTICE("error check: i = %d, command data = %02x, gauge return data = %02x", i, buff[2+i], read_buff[i]);
                                        return -EINVAL;
                                }
                        }

                        break;
                case update_command_write:      // write command
                        if(i2c_smbus_write_i2c_block_data(client, buff[1], data_length, &buff[2]) < 0){
                                BAT_NOTICE("error write: i2c_smbus_write_i2c_block_data error\n");
                                return -EINVAL;
                        }

                        break;
                case update_command_sleep:      // sleep command
                        if(buff_length == 2){
                                sleep_time = buff[0] * 256 + buff[1];
                                BAT_NOTICE("sleep time = %d\n", sleep_time);
                                msleep(sleep_time);
                        }else{
                                BAT_NOTICE("error: invalid sleep buff length\n");
                                return -EINVAL;
                        }

                        break;
                default:
                        BAT_NOTICE("error: invalid dffs_command\n");
                        return -EINVAL;
                        break;
        }

        return 0;
}

static enum gaugeIC_update_ret_value analyze_dffs_string_to_hexadecimal_and_send_data(
        char dffs_string[], int string_length)
{
        char temp_dffs_string[string_length];
        char *temp_dffs_string_for_strsep_use;
        char *delim_token = " ";
        char *temp_char;
        u8 temp_result_array[string_length];
        int send_byte_count = 0;
        int j = 0;
        int wait_time = 0;
        int EC_return_vaule = 0;
        int re_send_to_EC_count = 0;
        enum gaugeIC_update_ret_value gaugeIC_update_return_value;
        #define RE_SEND_TO_EC_TIMES (3)

       memcpy(temp_dffs_string, dffs_string, sizeof(temp_dffs_string));

       switch(temp_dffs_string[0]){
               case '#':
                        // dffs's comment format is like:: # Date_2013_6_11
                        // abstract the string Date_2013_6_11

                        temp_dffs_string_for_strsep_use = temp_dffs_string;

                        temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
                        temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);

                        BAT_NOTICE("dffs_string: comment: %s\n", temp_char);

                        break;

                case 'W':
                        // dffs's write format is like:: W: 16 00 08
                        temp_dffs_string_for_strsep_use = &temp_dffs_string[3];

                        send_byte_count = 0;
                        for(temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
                                temp_char != NULL; temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token)){
                                temp_result_array[send_byte_count] = (u8)simple_strtol(temp_char, NULL, 16);

                                send_byte_count++;
                        }

                        BAT_NOTICE("dffs_string: write %d bytes: ", send_byte_count);
                        j = 0;
                        for(j = 0; j < send_byte_count; j++)
                                printk("%02x ", temp_result_array[j]);

                        BAT_NOTICE("\n");

                        do{
                                EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_write, temp_result_array, send_byte_count);
                                re_send_to_EC_count++;
                        } while((EC_return_vaule <0) && (re_send_to_EC_count < RE_SEND_TO_EC_TIMES));

                        gaugeIC_update_return_value = (EC_return_vaule <0) ? error_write : update_success;

                        return gaugeIC_update_return_value;

                        //gaugeIC_update_return_value = change_EC_ret_value_to_gaugeIC_update_ret_value(EC_return_vaule);
                        //if(gaugeIC_update_return_value != 0)
                        //      return EC_return_vaule;

                        break;

                case 'C':

                        // dffs's check format is like:: C: 16 04 34 11 3F A7

                        temp_dffs_string_for_strsep_use = &temp_dffs_string[3];

                        send_byte_count = 0;
                        for(temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
                                temp_char != NULL; temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token)){
                                temp_result_array[send_byte_count] = (u8)simple_strtol(temp_char, NULL, 16);

                                send_byte_count++;
                        }

                        BAT_NOTICE("dffs_string: check %d bytes: ", send_byte_count);
                        j = 0;
                        for(j = 0; j < send_byte_count; j++)
                                BAT_NOTICE("%02x ", temp_result_array[j]);

                        BAT_NOTICE("\n");

                        do{
                                EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_read, temp_result_array,send_byte_count);
                                re_send_to_EC_count++;
                        }while((EC_return_vaule <0) && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

                        gaugeIC_update_return_value = (EC_return_vaule <0) ? error_read : update_success;

                        return gaugeIC_update_return_value;

                        break;

                case 'X':
                        // dffs's check format is like:: X: 170

                        temp_dffs_string_for_strsep_use = temp_dffs_string;


                        temp_char = strsep(&temp_dffs_string_for_strsep_use,delim_token);
                        temp_char = strsep(&temp_dffs_string_for_strsep_use,delim_token);

                        wait_time = (int)simple_strtol(temp_char, NULL, 10);
                        temp_result_array[0] = wait_time / 256;
                        temp_result_array[1] = wait_time % 256;
                        BAT_NOTICE("dffs_string: wait: %02x, %02x\n", temp_result_array[0], temp_result_array[1]);

                        do{
                                EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_sleep, temp_result_array, 2);
                                re_send_to_EC_count++;
                        }while(EC_return_vaule < 0 && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

                        gaugeIC_update_return_value = (EC_return_vaule <0) ? error_sleep : update_success;

                        return gaugeIC_update_return_value;

                        break;
                default:
                        BAT_NOTICE("error\n");

                        break;
        }

        return 0;
}

static int gaugeIC_exit_ROM_mode(void)
{
        int max_dffs_string_length = 256;
        char dffs_string[max_dffs_string_length];

        BAT_NOTICE("gaugeIC: exit ROM mode...\n");

        strcpy(dffs_string, "W: 16 00 0F");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: 16 00 0F update success\n");

        strcpy(dffs_string, "W: 16 64 0F 00");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: 16 64 0F 00 update success\n");

        strcpy(dffs_string, "X: 4000");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: X: 4000 update success\n");

        strcpy(dffs_string, "W: AA 00 20 00");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: AA 00 20 00 update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: X: 20 update success\n");

        strcpy(dffs_string, "W: AA 00 20 00");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: AA 00 20 00 update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: X: 20 update success\n");

        BAT_NOTICE("gaugeIC: exit ROM mode success\n");

        return 0;
}

static int gaugeIC_enter_ROM_mode(void)
{
        int max_dffs_string_length = 256;
        char dffs_string[max_dffs_string_length];

        BAT_NOTICE("gaugeIC: enter ROM mode...\n");

        strcpy(dffs_string, "W: AA 00 14 04");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 14 04 update success\n");

        strcpy(dffs_string, "W: AA 00 72 16");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 72 16 update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: X: 20 update success\n");

        strcpy(dffs_string, "W: AA 00 FF FF");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 FF FF update success\n");

        strcpy(dffs_string, "W: AA 00 FF FF");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 FF FF update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: X: 20 update success\n");

        strcpy(dffs_string, "W: AA 00 00 0F");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 00 0F update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: X: 20 update success\n");

        BAT_NOTICE("gaugeIC: enter ROM mode success\n");

        return 0;
}

static int get_gaugeIC_mode(void)
{
        // normal mode: if we can get battery information(such as capacity).
        // ROM mode:    if we can get ROM mode data(we choose 0x66, since we found some check sum address is 0x66).
        // may broken:   we can't get battery information and can't get ROM mode data.
        int return_value;

        u8 ROM_mode_read_buff[1];
        int retry_times = 0;
        #define MAX_RETRY_TIME 3

        do{
                if(bq27541_smbus_read_data(REG_CAPACITY, 0, &return_value) >= 0)
                return normal_mode;

                if(i2c_smbus_read_i2c_block_data(bq27541_device->ROM_mode_client, 0x66, 1, ROM_mode_read_buff) >= 0)
                return ROM_mode;

                retry_times++;
        }while(retry_times <= MAX_RETRY_TIME);

        return may_broken;
}

static bool check_if_gaugeIC_in_ROM_mode(void)
{
        enum gaugeIC_mode_enum gaugeIC_mode = may_broken;

        gaugeIC_mode = get_gaugeIC_mode();

        return (gaugeIC_mode == normal_mode) ? false : true;  // since sometimes it will check error, so if not normal, feedback it is in rom mode
}

static int __update_gaugeIC_firmware_data_flash_blocks(struct file *fp)
{
        int max_dffs_string_length = 256;
        int i = 0;
        int dffs_line_count = 0;
        int ret;
        int gaugeIC_check_error_count = 0;
        char temp_char;
        char dffs_string[max_dffs_string_length];
        enum gaugeIC_update_ret_value gaugeIC_update_return_value;
        #define F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR (0)
        #define GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES (5)

        BAT_NOTICE("gaugeIC: update data flash blocks...\n");

        if (!(fp->f_op) || !(fp->f_op->read)){
                BAT_NOTICE("gaugeIC: update data flash blocks error: no dffs file operation\n");
                return -EINVAL;
        }

        do{
                ret = fp->f_op->read(fp,&temp_char,1, &fp->f_pos);
                if(ret > 0){
                        if(temp_char != '\n'){
                                dffs_string[i] = temp_char;

                                i++;
                        }else{
                                dffs_string[i] = '\0';

                                BAT_NOTICE("line %d: %s\n", dffs_line_count, dffs_string);

                                gaugeIC_update_return_value =analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
                                                                                                max_dffs_string_length);

                                switch(gaugeIC_update_return_value){
                                        case error_sleep:
                                        case error_check:
                                        case error_write:
                                        case error_read:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: error = %d , error time = %d\n", gaugeIC_update_return_value, gaugeIC_check_error_count);
                                                fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
                                                gaugeIC_check_error_count++;
                                                i = 0;
                                                dffs_line_count = 0;

                                                if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
                                                        return -EINVAL;

                                                break;

                                        //case EC_error_unknow:
                                        //      printk("gauge_bq27520: gaugeIC firmware update: Major errors\n");
                                        //      return -EINVAL;
                                        //      break;

                                        case update_success:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: success\n");
                                                i = 0;
                                                dffs_line_count++;
                                                break;

                                        default:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: no this case\n");
                                                return -EINVAL;
                                                break;
                                }

                        }
                }else{  // last line
                        if(i != 0){
                                dffs_string[i] = '\0';

                                BAT_NOTICE("line %d: %s\n", dffs_line_count, dffs_string);

                                gaugeIC_update_return_value = analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
                                                                                                max_dffs_string_length);

                                switch(gaugeIC_update_return_value){
                                        case error_sleep:
                                        case error_check:
                                        case error_write:
                                        case error_read:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: error = %d , error time = %d\n", gaugeIC_update_return_value, gaugeIC_check_error_count);
                                                fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
                                                gaugeIC_check_error_count++;
                                                i = 0;
                                                dffs_line_count = 0;
                                                ret = 1;

                                                if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
                                                        return -EINVAL;

                                                break;

                                        //case EC_error_unknow:
                                        //      printk("gauge_bq27520: gaugeIC firmware update: Major errors\n");
                                        //      return -EINVAL;
                                        //      break;
                                        case update_success:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: success\n");
                                                i = 0;
                                                dffs_line_count++;
                                                break;

                                        default:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: no this case\n");
                                                return -EINVAL;
                                                break;
                                }

                        }
                }
        }while(ret > 0);


        BAT_NOTICE("gaugeIC: update data flash blocks success\n");

        return 0;
}

static int update_gaugeIC_firmware(struct file *fp)
{
        int ret = 0;

        if(!check_if_gaugeIC_in_ROM_mode())
                ret = gaugeIC_enter_ROM_mode();

        if(ret !=0)
                return -EINVAL;

        ret = __update_gaugeIC_firmware_data_flash_blocks(fp);

        if(ret != 0)
                return -EINVAL;

        return ret;
}

static ssize_t show_battery_BSOC(struct device *dev, struct device_attribute *devattr, char *buf)
{
        int ret;
        int value;
	mutex_lock(&lock);
        ret = bq27541_read_i2c(0x74, &value, 0);
	mutex_unlock(&lock);
        if (ret) {
                BAT_ERR("error in reading battery RSOC= %x\n", ret);
                return 0;
        }

        return sprintf(buf, "%d\n", value);
}

static ssize_t show_battery_RSOC(struct device *dev, struct device_attribute *devattr, char *buf)
{
        int ret;
        int value;
	mutex_lock(&lock);
        ret = bq27541_read_i2c(0x20, &value, 0);
	mutex_unlock(&lock);
        if (ret) {
                BAT_ERR("error in reading battery RSOC= %x\n", ret);
                return 0;
        }

        return sprintf(buf, "%d\n", value);
}

static ssize_t show_battery_RM(struct device *dev, struct device_attribute *devattr, char *buf)
{
        int ret;
        int value;
	mutex_lock(&lock);
        ret = bq27541_read_i2c(0x6c, &value, 0);
	mutex_unlock(&lock);
        if (ret) {
                BAT_ERR("error in reading battery RM = %x\n", ret);
                return 0;
        }

        return sprintf(buf, "%d\n", value);
}

static ssize_t show_battery_FCC(struct device *dev, struct device_attribute *devattr, char *buf)
{
        int ret;
        int value;
	mutex_lock(&lock);
        ret = bq27541_read_i2c(0x70, &value, 0);
	mutex_unlock(&lock);
        if (ret) {
                BAT_ERR("error in reading battery FCC = %x\n", ret);
                return 0;
        }

        return sprintf(buf, "%d\n", value);
}

static ssize_t gauge_mode_show(struct device *dev,
        struct device_attribute *devattr, char *buf)
{
        // 0: not normal mode and not rom mode. the gauge may broken.
        // 1: gaugeIC in rom mode.
        // 2: gaugeIC in normal mode.

        enum gaugeIC_mode_enum gaugeIC_mode = may_broken;
        char gaugeIC_mode_in_sentence[15];

        gaugeIC_mode = get_gaugeIC_mode();
        BAT_NOTICE("gaugeIC mode: %d\n", gaugeIC_mode);
        switch(gaugeIC_mode){
                case may_broken:                // not normal mode and not rom mode. the gauge may broken.
                        strcpy(gaugeIC_mode_in_sentence, "may broken");
                        break;
                case ROM_mode:          // gaugeIC in rom mode.
                        strcpy(gaugeIC_mode_in_sentence, "rom mode");
                        break;
                case normal_mode:       // gaugeIC in normal mode.
                        strcpy(gaugeIC_mode_in_sentence, "normal mode");
                        break;
                default:                                // no this case
                        strcpy(gaugeIC_mode_in_sentence, "no this case");
        }

        return sprintf(buf, "gaugeIC mode: %s\n", gaugeIC_mode_in_sentence);
}

static ssize_t gauge_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int temp,ret;
        struct file *fp;
        mm_segment_t old_fs;

        if(buf[0] == '1'){
                temp = get_gaugeIC_mode();
                BAT_NOTICE("gaugeIC_mode = %d\n", temp);
        }else if(buf[0] == '2'){
                gaugeIC_enter_ROM_mode();
                BAT_NOTICE("enter ROM mode\n");
        }else if(buf[0] == '3'){
                gaugeIC_exit_ROM_mode();
                BAT_NOTICE("exit ROM mode\n");
        }else if(buf[0] == '4'){
                BAT_NOTICE("update\n");
                old_fs = enable_kernel_to_read_file_and_return_origin_fs();
                fp = open_file("/system/etc/firmware/TF303Kv01.dffs", O_RDONLY, 0);

                if(fp!=NULL) {
                        ret = update_gaugeIC_firmware(fp);
                        if (ret == 0) {
                                printk("gaugeIC firware update success\n");
                        }
                        else{
                                printk("gaugeIC firmware update fail\n");
                        }
                        close_file(fp);
                } else {
			printk("gaugeIC firmware update fail: no file\n");
                }
                exit_kernel_to_read_file(old_fs);
        }

        return count;
}
#endif

#ifdef FACTORY_IMAGE
static ssize_t battery_limit_off_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	if (buf[0] == '0') {
		bq27541_device->bat_limit_off = 0;
		BAT_NOTICE("bq27541_device->bat_limit_off = 0\n");
	}
	if (buf[0] == '1') {
		bq27541_device->bat_limit_off = 1;
		BAT_NOTICE("bq27541_device->bat_limit_off = 1\n");
	}
	return count;
}

static ssize_t battery_limit_off_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "bat_limit_off = %d, echo 1(off) or 0(on) > battery_limit_off\n", bq27541_device->bat_limit_off);
}
#endif

static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status, NULL);
static DEVICE_ATTR(battery_INFO, S_IWUSR | S_IRUGO, show_battery_INFO, NULL);
#ifdef FACTORY_IMAGE
static DEVICE_ATTR(battery_limit_off, S_IWUSR | S_IRUGO, battery_limit_off_show, battery_limit_off_store);
#endif
#ifdef NOT_USER
static DEVICE_ATTR(battery_FCC, S_IWUSR | S_IRUGO, show_battery_FCC, NULL);
static DEVICE_ATTR(battery_RM, S_IWUSR | S_IRUGO, show_battery_RM, NULL);
static DEVICE_ATTR(battery_RSOC, S_IWUSR | S_IRUGO, show_battery_RSOC, NULL);
static DEVICE_ATTR(battery_BSOC, S_IWUSR | S_IRUGO, show_battery_BSOC, NULL);
static DEVICE_ATTR(gauge_mode, S_IWUSR | S_IRUGO, gauge_mode_show, NULL);
static DEVICE_ATTR(gauge_debug, S_IWUSR | S_IRUGO, NULL, gauge_store);
#endif

static struct attribute *battery_smbus_attributes[] = {

	&dev_attr_battery_smbus.attr,
	&dev_attr_battery_INFO.attr,
#ifdef FACTORY_IMAGE
	&dev_attr_battery_limit_off.attr,
#endif
#ifdef NOT_USER
        &dev_attr_battery_FCC.attr,
        &dev_attr_battery_RM.attr,
        &dev_attr_battery_RSOC.attr,
        &dev_attr_battery_BSOC.attr,
        &dev_attr_gauge_debug.attr,
        &dev_attr_gauge_mode.attr,
#endif
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};

int bq27541_battery_current(void)
{
	int ret;
	int curr = 0;

	mutex_lock(&lock);
	ret = bq27541_read_i2c(bq27541_data[REG_CURRENT].addr, &curr, 0);
	mutex_unlock(&lock);
	if (ret) {
		BAT_NOTICE("error reading current ret = %x\n",ret);
		return 0;
	}

	curr = (int)(s16)curr;

	if (curr >= bq27541_data[REG_CURRENT].min_value &&
		curr <= bq27541_data[REG_CURRENT].max_value) {
		return curr;
	} else
		return 0;
}
EXPORT_SYMBOL(bq27541_battery_current);

static void battery_status_poll(struct work_struct *work)
{
       struct bq27541_device_info *batt_dev = container_of(work, struct bq27541_device_info, status_poll_work.work);

	if(!bq27541_battery_driver_ready)
		BAT_NOTICE("battery driver not ready\n");

	power_supply_changed(&bq27541_supply[Charger_Type_Battery]);

	BAT_NOTICE("charging_status = %d\n", check_charging_status());

	if(machine_is_apq8064_macaque())
		smb345_recheck_charging_type();

#ifndef FACTORY_IMAGE
/*
	if (!bq27541_device->temp_err) {
		if (ac_on)
			smb345_config_thermal_charging(bq27541_device->old_temperature/10, THERMAL_RULE1);
		else if(usb_on)
			smb345_config_thermal_charging(bq27541_device->old_temperature/10, THERMAL_RULE1);
	}
*/
#endif
	/* Schedule next polling */
	queue_delayed_work(bq27541_battery_work_queue, &batt_dev->status_poll_work, battery_check_interval*HZ);
}

static void low_low_battery_check(struct work_struct *work)
{
	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(bq27541_battery_work_queue,&bq27541_device->status_poll_work, 0.1*HZ);
	msleep(2000);
	enable_irq(bq27541_device->irq_low_battery_detect);
}

static irqreturn_t low_battery_detect_isr(int irq, void *dev_id)
{
	disable_irq_nosync(bq27541_device->irq_low_battery_detect);
	bq27541_device->low_battery_present = gpio_get_value(bq27541_device->gpio_low_battery_detect);
	BAT_NOTICE("gpio LL_BAT_T30=%d\n", bq27541_device->low_battery_present);
	wake_lock_timeout(&bq27541_device->low_battery_wake_lock, 10*HZ);
	queue_delayed_work(bq27541_battery_work_queue, &bq27541_device->low_low_bat_work, 0.1*HZ);
	return IRQ_HANDLED;
}

static int setup_low_battery_irq(void)
{
	unsigned gpio = GPIOPIN_LOW_BATTERY_DETECT;
	int ret, irq = gpio_to_irq(gpio);

	ret = gpio_request(gpio, "low_battery_detect");
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 request failed\n");
		goto fail;
	}

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 unavaliable for input\n");
		goto fail_gpio;
	}

	ret = request_irq(irq, low_battery_detect_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"bq27541-battery (low battery)", NULL);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 irq request failed\n");
		goto fail_irq;
	}

	bq27541_device->low_battery_present = gpio_get_value(gpio);
	bq27541_device->irq_low_battery_detect = gpio_to_irq(gpio);
	BAT_NOTICE("irq=%d, LL_BAT_T30=%d\n", irq, bq27541_device->low_battery_present);

	enable_irq_wake(bq27541_device->irq_low_battery_detect);
	return ret;
fail_irq:
fail_gpio:
	gpio_free(gpio);
fail:
	return ret;
}

int bq27541_battery_callback(unsigned usb_cable_state)
{
	int old_cable_status;

	if(!bq27541_battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		return 0;
	}

	old_cable_status = bq27541_battery_cable_status;
	bq27541_battery_cable_status = usb_cable_state;

       printk("========================================================\n");
	printk("bq27541_battery_callback  usb_cable_state = %x\n", usb_cable_state) ;
       printk("========================================================\n");

	if (old_cable_status != bq27541_battery_cable_status) {
		printk("battery_callback cable_wake_lock 5 sec...\n ");
		wake_lock_timeout(&bq27541_device->cable_wake_lock, 5*HZ);
	}

	bq27541_check_cable_type();

	if(!bq27541_battery_cable_status) {
		if (old_cable_status == USB_AC_Adapter) {
			power_supply_changed(&bq27541_supply[Charger_Type_AC]);
		}
		#ifndef REMOVE_USB_POWER_SUPPLY
		else if ( old_cable_status == USB_Cable) {
			power_supply_changed(&bq27541_supply[Charger_Type_USB]);
		}
		#endif
	}
	#ifndef REMOVE_USB_POWER_SUPPLY
	else if (bq27541_battery_cable_status == USB_Cable) {
		power_supply_changed(&bq27541_supply[Charger_Type_USB]);
	}
	#endif
	else if (bq27541_battery_cable_status == USB_AC_Adapter) {
		power_supply_changed(&bq27541_supply[Charger_Type_AC]);
	}

	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(bq27541_battery_work_queue, &bq27541_device->status_poll_work, 2*HZ);

	return 1;
}
EXPORT_SYMBOL(bq27541_battery_callback);

static int bq27541_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		val->intval = 1;
	}
	else if (psp == POWER_SUPPLY_PROP_HEALTH) {
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}
	return 0;
}

static int bq27541_get_psp(int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
	int rt_value=0;

	mutex_lock(&lock);
	bq27541_device->smbus_status = bq27541_smbus_read_data(reg_offset, 0, &rt_value);
	mutex_unlock(&lock);

	if ((bq27541_device->smbus_status < 0) && (psp != POWER_SUPPLY_PROP_TEMP)) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d failed\n", __func__, reg_offset);
		return -EINVAL;
	}

	if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
		if (rt_value >= bq27541_data[REG_VOLTAGE].min_value &&
			rt_value <= bq27541_data[REG_VOLTAGE].max_value) {
			if (rt_value > BATTERY_PROTECTED_VOLT) {
				val->intval = bq27541_device->bat_vol = rt_value*1000;
			} else {
				val->intval = bq27541_device->bat_vol;
			}
		} else {
			val->intval = bq27541_device->bat_vol;
		}
		BAT_NOTICE("voltage_now= %u uV\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_STATUS) {
		ret = bq27541_device->bat_status = rt_value;

		if ((ac_on || usb_on) && !otg_on) {		/* Charging detected */
			if (bq27541_device->old_capacity == 100) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
			} else {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else if (ret & BATT_STS_SOCF) {		/* Fully Discharged detected */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		BAT_NOTICE("status: %s ret= 0x%04x\n", status_text[val->intval], ret);

	} else if (psp == POWER_SUPPLY_PROP_TEMP) {
		ret = bq27541_device->bat_temp = rt_value;

		if (bq27541_device->smbus_status >=0) {
			if (rt_value >= bq27541_data[REG_TEMPERATURE].min_value &&
				rt_value <= bq27541_data[REG_TEMPERATURE].max_value) {
				if (rt_value >=2330  && rt_value <=3730) {
					ret = rt_value -2730;
					if(bq27541_device->temp_err)
						bq27541_device->temp_err = false;
				} else
					bq27541_device->temp_err = true;
			} else
				bq27541_device->temp_err = true;
		} else
			bq27541_device->temp_err = true;

		if (bq27541_device->temp_err) {
			BAT_NOTICE("error: temperature ret=%d, old_temp=%d \n",
				rt_value, bq27541_device->old_temperature);
			if (bq27541_device->old_temperature != 0xFF)
				ret = bq27541_device->old_temperature;
			else
				return -EINVAL;
		}
		bq27541_device->old_temperature = val->intval = ret;
		BAT_NOTICE("temperature= %d (0.1¢XC)\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_CURRENT_NOW) {
		val->intval = bq27541_device->bat_current = bq27541_battery_current();
		BAT_NOTICE("current = %d mA\n", val->intval);
	}
        return 0;
}

static int bq27541_get_capacity(union power_supply_propval *val)
{
	s32 ret;
	s32 temp_capacity;
	bool check_cap = false;
	int smb_retry=0;
	int smb_retry_max = (SMBUS_RETRY + 2);
	int temp;
	int df_version;

	bq27541_device->bat_capacity = 0;

	do {
		mutex_lock(&lock);
		bq27541_device->smbus_status = bq27541_smbus_read_data(REG_CAPACITY, 0 ,&bq27541_device->bat_capacity);
		mutex_unlock(&lock);
		if ((bq27541_device->bat_capacity <= 0) || (bq27541_device->bat_capacity > 100)) {
			check_cap = true;
			BAT_NOTICE("check capacity, cap = %d, smb_retry = %d\n", bq27541_device->bat_capacity, smb_retry);
		} else
			check_cap = false;
	} while(((bq27541_device->smbus_status < 0) || check_cap) && ( ++smb_retry <= smb_retry_max));

	if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d "
			"failed bq27541_device->cap_err=%u\n", __func__, REG_CAPACITY, bq27541_device->cap_err);

		if(bq27541_device->cap_err>5 || (bq27541_device->old_capacity == 0xFF)) {
			return -EINVAL;
		} else {
			val->intval = bq27541_device->old_capacity;
			bq27541_device->cap_err++;
			BAT_NOTICE("cap_err=%u use old capacity=%u\n", bq27541_device->cap_err, val->intval);
			return 0;
		}
	}

	temp_capacity = ret = bq27541_device->bat_capacity;

	if (!(bq27541_device->bat_capacity >= bq27541_data[REG_CAPACITY].min_value &&
	                bq27541_device->bat_capacity <= bq27541_data[REG_CAPACITY].max_value)) {
	        val->intval = bq27541_device->old_capacity;
	        BAT_NOTICE("use old capacity=%u\n", bq27541_device->old_capacity);
	        return 0;
	}

	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);

	mutex_lock(&lock);
        i2c_smbus_write_word_data(bq27541_device->client, BATT_REG_CNTL, DF_VERSION&0xFFFF);
	mutex_unlock(&lock);
	msleep(10);
	mutex_lock(&lock);
        temp = bq27541_smbus_read_data(BATT_REG_CNTL, 0, &df_version);
	mutex_unlock(&lock);
        if (temp < 0) {
                return BAT_NOTICE("dfversion check fail\n");
        }


	if(df_version != 1){
		BAT_NOTICE("Gauge workaround, due to DF_VERSION: %x\n", df_version);
	        /* for suddenly capacity dropping workaround */
	        if (temp_capacity == 0 && bq27541_device->bat_vol >= 4200000)
	                temp_capacity = 100;
	        else if (temp_capacity == 0 && bq27541_device->bat_vol >= 4000000)
	                temp_capacity = 80;
	        else if (temp_capacity == 0 && bq27541_device->bat_vol >= 3850000)
	                temp_capacity = 60;
	        else if (temp_capacity == 0 && bq27541_device->bat_vol >= 3800000)
	                temp_capacity = 40;
	        else if (temp_capacity == 0 && bq27541_device->bat_vol >= 3770000)
	                temp_capacity = 20;
	}

	val->intval = temp_capacity;

	if ((temp_capacity == 0) && (bq27541_battery_cable_status)) {
		bq27541_device->bat_capacity_zero_count++;
		if (bq27541_device->bat_capacity_zero_count >= 6)
		{
			bq27541_device->bat_capacity_zero_count = 0;
			BAT_NOTICE("pretend no charging type to shutdown system !!!\n");
			bq27541_battery_callback(0);
		}
		else
			BAT_NOTICE("bat_capacity_zero_count = %d",
				bq27541_device->bat_capacity_zero_count);
	}
	else
		bq27541_device->bat_capacity_zero_count = 0;

	if (temp_capacity < 5 && battery_check_interval != 10) {
		battery_check_interval = 10;
		BAT_NOTICE("battery_check_interval = %d \n", battery_check_interval);
	}
	else if(temp_capacity >= 5 && battery_check_interval != BATTERY_POLLING_RATE)
	{
		battery_check_interval = BATTERY_POLLING_RATE;
		BAT_NOTICE("battery_check_interval = %d \n", battery_check_interval);
	}

	bq27541_device->old_capacity = val->intval;
	bq27541_device->cap_err=0;

#ifdef FACTORY_IMAGE
	if(bq27541_device->bat_limit_off == 0)
	{
		if(val->intval <= 40)
		{
			BAT_NOTICE("smb345 charger enable\n");
			smb345_charger_enable(true);
		}
		else if(val->intval >= 60)
		{
			BAT_NOTICE("smb345 charger disable\n");
			smb345_charger_enable(false);
		}
	}
	else
	{
		BAT_NOTICE("smb345 charger enable\n");
		smb345_charger_enable(true);
	}
#endif

	BAT_NOTICE("= %u%% ret= %u\n", val->intval, ret);
	return 0;
}

static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	u8 count;
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (bq27541_get_health(psp, val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (bq27541_get_capacity(val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_TEMP:
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			for (count = 0; count < REG_MAX; count++) {
				if (psp == bq27541_data[count].psp)
					break;
			}

			if (bq27541_get_psp(count, psp, val))
				return -EINVAL;
			break;

		default:
			dev_err(&bq27541_device->client->dev,
				"%s: INVALID property psp=%u\n", __func__,psp);
			return -EINVAL;
	}

	return 0;

error:

	return -EINVAL;
}

static int bq27541_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
        int ret, i=0;

	printk("hpa02254 probe init\n");

	BAT_NOTICE("+ client->addr= %02x\n", client->addr);
	bq27541_device = kzalloc(sizeof(*bq27541_device), GFP_KERNEL);
	if (!bq27541_device)
		return -ENOMEM;

	memset(bq27541_device, 0, sizeof(*bq27541_device));
	bq27541_device->client = client;
	i2c_set_clientdata(client, bq27541_device);
	bq27541_device->smbus_status = 0;
	bq27541_device->cap_err = 0;
	bq27541_device->temp_err = false;
	bq27541_device->old_capacity = 0xFF;
	bq27541_device->old_temperature = 0xFF;
	bq27541_device->gpio_low_battery_detect = GPIOPIN_LOW_BATTERY_DETECT;
	bq27541_device->bat_limit_off = 0;
	bq27541_device->bat_capacity_zero_count = 0;

	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		ret = power_supply_register(&client->dev, &bq27541_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply\n");
			while (i--)
				power_supply_unregister(&bq27541_supply[i]);
				kfree(bq27541_device);
			return ret;
		}
	}

	bq27541_battery_work_queue = create_singlethread_workqueue("bq27541_battery_work_queue");
	INIT_DELAYED_WORK(&bq27541_device->status_poll_work, battery_status_poll);
	INIT_DELAYED_WORK(&bq27541_device->low_low_bat_work, low_low_battery_check);
	cancel_delayed_work(&bq27541_device->status_poll_work);

	mutex_init(&lock);
	spin_lock_init(&bq27541_device->lock);
	wake_lock_init(&bq27541_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");
	wake_lock_init(&bq27541_device->cable_wake_lock, WAKE_LOCK_SUSPEND, "cable_state_changed");

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_smbus_group);
	if (ret) {
		dev_err(&client->dev, "bq27541_probe: unable to create the sysfs\n");
	}
 
	/* Misc device registration */
	bq27541_device->battery_misc.minor = MISC_DYNAMIC_MINOR;
	bq27541_device->battery_misc.name = "battery";
//	bq27541_device->battery_misc.fops   = &bq27541_battery_fops;
	ret = misc_register(&bq27541_device->battery_misc);
	if(ret) {
		pr_info("Cannot register bq27541 miscdev (err=%d)\n", ret);
	}

	/* gauge FW update */
        bq27541_device->ROM_mode_client = kzalloc(sizeof *client, GFP_KERNEL);
        *bq27541_device->ROM_mode_client = *bq27541_device->client;
        bq27541_device->ROM_mode_client->addr = 0x0B;

	setup_low_battery_irq();

	bq27541_battery_driver_ready = 1;

	queue_delayed_work(bq27541_battery_work_queue, &bq27541_device->status_poll_work, 15*HZ);

	BAT_NOTICE("- %s driver registered\n", client->name);

	return 0;
}

static int bq27541_remove(struct i2c_client *client)
{
	struct bq27541_device_info *bq27541_device;
	int i=0;
	bq27541_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		power_supply_unregister(&bq27541_supply[i]);
	}
	if (bq27541_device) {
		wake_lock_destroy(&bq27541_device->low_battery_wake_lock);
		wake_lock_destroy(&bq27541_device->cable_wake_lock);
		kfree(bq27541_device);
		bq27541_device = NULL;
	}
	return 0;
}

#if defined (CONFIG_PM)
static int bq27541_suspend(struct i2c_client *client, pm_message_t state)
{
	cancel_delayed_work_sync(&bq27541_device->status_poll_work);
	flush_workqueue(bq27541_battery_work_queue);
	return 0;
}

/* any smbus transaction will wake up pad */
static int bq27541_resume(struct i2c_client *client)
{
	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(bq27541_battery_work_queue,&bq27541_device->status_poll_work, 5*HZ);
	return 0;
}
#endif

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.probe	= bq27541_probe,
	.remove 	= bq27541_remove,
#if defined (CONFIG_PM)
	.suspend	= bq27541_suspend,
	.resume 	= bq27541_resume,
#endif
	.id_table	= bq27541_id,
	.driver = {
		.name = "bq27541-battery",
	},
};
static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		dev_err(&bq27541_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);
	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("bq27541 battery monitor driver");
MODULE_LICENSE("GPL");
