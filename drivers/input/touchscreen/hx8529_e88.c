/* Himax Android Driver Sample Code Ver 2.2 for ME301
*
* Copyright (C) 2012 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/*
* ==============================================================================
* Segment list :
* Include Header file
* Himax Define Options
* Himax Define Variable
* Himax Include Header file / Data Structure
* Himax Variable/Pre Declation Function
* Himax Normal Function
* Himax SYS Debug Function
* Himax Touch Work Function
* Himax Linux Driver Probe Function
* Other Function
* ==============================================================================
*/
/*
* ==============================================================================
*		Segment : Include Header file
* ==============================================================================
*/
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/time.h>

/* for linux 2.6.36.3 */
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>
#include <linux/hx8529_e88.h>
#include <mach/board_asustek.h>
/*
* ==============================================================================
*		Segment : Himax Define Options
* ==============================================================================
*/
/* TODO START : Select the function you need! */
/* Support Function Enable: */
#define HX_TP_PROC_DIAG        /*Support Proc: Diag func, default open*/
#define HX_TP_SYS_REGISTER     /*Support Proc: Register func, default open*/
#define HX_TP_SYS_DEBUG_LEVEL  /*Support Proc: Debug Level func, default open*/
#define HX_TP_SYS_FLASH_DUMP   /*Support Proc: Flash dump func, default open*/
#define HX_TP_SYS_SELF_TEST    /*Support Proc: Self Test func, default open*/
#define HX_TP_SYS_FS
#define HX_RST_PIN_FUNC        /*Support HW Reset, default open*/
/*#define HX_PORTING_DEB_MSG*/ /*Support Driver Porting Message, default close*/
#define HX_FW_UPDATE_BY_I_FILE /*Support Update FW by i file, default close*/
/* TODO END */

/* Support Different IC. Select one at one time. */
#define HX_85XX_E_SERIES_PWON 1

/* Supoort ESD Issue */
/*#define HX_85XX_E_nWATCH_DOG*/

#ifdef HX_RST_PIN_FUNC
#define HX_ESD_WORKAROUND         /*Support ESD Workaround, default close*/
#define ENABLE_CHIP_RESET_MACHINE /*Support Chip Reset Workqueue, default open*/
/*#define HX_ESD_WORKAROUND_HANDSHAKING*/
#endif

#ifdef ENABLE_CHIP_RESET_MACHINE
#define HX_TP_SYS_RESET        /*Support Proc: HW Reset function, default open*/
/*#define ENABLE_CHIP_STATUS_MONITOR*/ /*Support Polling ic status, default close*/
#endif

/* Support FW Bin checksum method,mapping with Hitouch *.bin */
#define HX_TP_BIN_CHECKSUM_CRC 1

/*
* ==============================================================================
*		Segment : Himax Define Variable
* ==============================================================================
*/

/* TODO START : Modify follows deinfe variable */
#define HX_KEY_MAX_COUNT 4    /* Max virtual keys */
#define DEFAULT_RETRY_CNT 3   /* For I2C Retry count */
/* TODO END */

/* TODO START : Modify follows power gpio/ interrupt gpio/ reset gpio */
#define HIMAX_INT_GPIO 6
#define HIMAX_RST_GPIO 31
/* TODO END */

/* TODO START : Modify the I2C address */
#define HIMAX_I2C_ADDR 0x48
#define HIMAX_TS_NAME "himax-ts"
/* TODO END */

/* Input Device */
#define INPUT_DEV_NAME "himax-touchscreen"

/* Flash dump file */
#define FLASH_DUMP_FILE "/sdcard/Flash_Dump.bin"

/* Diag Coordinate dump file */
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

/* Virtual key */
#define HX_VKEY_0  KEY_MENU
#define HX_VKEY_1  KEY_BACK
#define HX_VKEY_2  KEY_HOMEPAGE
#define HX_VKEY_3  104
#define HX_KEY_ARRAY {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

#define HX8529_IOC_MAGIC 0xF3
#define HX8529_IOC_MAXNR 2
#define HX8529_POLL_DATA _IOR(HX8529_IOC_MAGIC, 2, int)

#define HX8529_IOCTL_START_HEAVY 2
#define HX8529_IOCTL_START_NORMAL 1
#define HX8529_IOCTL_END 0

#define START_NORMAL (HZ)
#define START_HEAVY  (HZ)

/* Himax TP COMMANDS -> Do not modify the below definition */
#define HX_CMD_NOP                   0x00   /* no operation */
#define HX_CMD_SETMICROOFF           0x35   /* set micro on */
#define HX_CMD_SETROMRDY             0x36   /* set flash ready */
#define HX_CMD_TSSLPIN               0x80   /* set sleep in */
#define HX_CMD_TSSLPOUT              0x81   /* set sleep out */
#define HX_CMD_TSSOFF                0x82   /* sense off */
#define HX_CMD_TSSON                 0x83   /* sense on */
#define HX_CMD_ROE                   0x85   /* read one event */
#define HX_CMD_RAE                   0x86   /* read all events */
#define HX_CMD_RLE                   0x87   /* read latest event */
#define HX_CMD_CLRES                 0x88   /* clear event stack */
#define HX_CMD_TSSWRESET             0x9E   /* TS software reset */
#define HX_CMD_SETDEEPSTB            0xD7   /* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN         0xDD   /* set cache function */
#define HX_CMD_SETIDLE               0xF2   /* set idle mode */
#define HX_CMD_SETIDLEDELAY          0xF3   /* set idle delay */
#define HX_CMD_SELFTEST_BUFFER       0x8D   /* Self-test return buffer */
#define HX_CMD_MANUALMODE            0x42
#define HX_CMD_FLASH_ENABLE          0x43
#define HX_CMD_FLASH_SET_ADDRESS     0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND     0x47
#define HX_CMD_FLASH_WRITE_BUFFER    0x48
#define HX_CMD_FLASH_PAGE_ERASE      0x4D
#define HX_CMD_FLASH_SECTOR_ERASE    0x4E
#define HX_CMD_CB                    0xCB
#define HX_CMD_EA                    0xEA
#define HX_CMD_4A                    0x4A
#define HX_CMD_4F                    0x4F
#define HX_CMD_B9                    0xB9
#define HX_CMD_76                    0x76
/* for firmware update */
#define HX_UPDATE_SUCCESS    1
#define HX_UPDATE_FAIL       0

/*
* ==============================================================================
*		Segment : Himax Include Header file / Data Structure
* ==============================================================================
*/
struct himax_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	int (*power)(int on);
	struct early_suspend early_suspend;
	int intr_gpio;
	/* Firmware Information */
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
	/* For Firmare Update */
	struct miscdevice firmware;
	struct attribute_group attrs;
	struct switch_dev touch_sdev;
	int abs_x_max;
	int abs_y_max;
	int rst_gpio;
	struct regulator *vdd;
	int init_success;

	/* i2c stress test */
	struct miscdevice misc_dev;
	struct wake_lock wake_lock;
	struct mutex mutex_lock;

#ifdef HX_TP_SYS_FLASH_DUMP
	struct workqueue_struct *flash_wq;
	struct work_struct flash_work;
#endif

#ifdef ENABLE_CHIP_RESET_MACHINE
	int retry_time;
	struct delayed_work himax_chip_reset_work;
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
	struct delayed_work himax_chip_monitor;
	int running_status;
#endif
};

/*
* ====================================================================
*		Segment : Himax Variable/Pre Declation Function
* ====================================================================
*/

static struct himax_ts_data *private_ts;
static struct kobject *android_touch_kobj;
/* for Hand shaking to check IC status */
static uint8_t IC_STATUS_CHECK = 0xAA;
/* for Virtual key array */
static int tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY;
struct i2c_client *touch_i2c;

static unsigned char IC_CHECKSUM;
static unsigned char IC_TYPE;

static int HX_TOUCH_INFO_POINT_CNT;

static int HX_RX_NUM;
static int HX_TX_NUM;
static int HX_BT_NUM;
static int HX_X_RES;
static int HX_Y_RES;
static int HX_MAX_PT;
static bool HX_INT_IS_EDGE;
static unsigned int FW_VER_MAJ_FLASH_ADDR;
static unsigned int FW_VER_MAJ_FLASH_LENG;
static unsigned int FW_VER_MIN_FLASH_ADDR;
static unsigned int FW_VER_MIN_FLASH_LENG;
static unsigned int CFG_VER_MAJ_FLASH_ADDR;
static unsigned int CFG_VER_MAJ_FLASH_LENG;
static unsigned int CFG_VER_MIN_FLASH_ADDR;
static unsigned int CFG_VER_MIN_FLASH_LENG;
/* for Firmware Version */
static u16 FW_VER_MAJ_buff[1];
static u16 FW_VER_MIN_buff[1];
static u16 CFG_VER_MAJ_buff[12];
static u16 CFG_VER_MIN_buff[12];

static bool is_suspend;

/* for himax_ts_work_func use */
static int hx_point_num;
static int p_point_num = 0xFFFF;
static int tpd_key;
static int tpd_key_old = 0xFF;

static unsigned int gPrint_point;

/* i2c stress test */
static int poll_mode;
struct delayed_work hx8529_poll_data_work;
static struct workqueue_struct *touch_work_queue;
struct i2c_client *hx8529_client;

static int i2c_himax_read(struct i2c_client *client, uint8_t command,
				uint8_t *data, uint8_t length, uint8_t toRetry);
static int i2c_himax_write(struct i2c_client *client, uint8_t command,
				uint8_t *data, uint8_t length, uint8_t toRetry);
static int i2c_himax_master_write(struct i2c_client *client, uint8_t *data,
				uint8_t length, uint8_t toRetry);
static int i2c_himax_write_command(struct i2c_client *client,
				uint8_t command, uint8_t toRetry);

static int himax_lock_flash(void);
static int himax_unlock_flash(void);

static int himax_hang_shaking(void);
static int himax_ts_poweron(struct himax_ts_data *ts_modify);

static int i_update_func(void);
static bool i_Check_FW_Version(void);

#ifdef HX_FW_UPDATE_BY_I_FILE
static bool i_Needupdate = true;
static unsigned char i_isTP_Updated;
static unsigned char i_CTPM_FW[] = {
#ifdef	TOUCH_SECOND_SOURCE
#include "ME103K_E0120_00_06.h"
#else
#include "TF303K_E0111_00_1F_20141030.h"
#endif
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#ifdef HX_ESD_WORKAROUND
static u8 ESD_RESET_ACTIVATE = 2;
static u8 ESD_COUNTER;
static int ESD_COUNTER_SETTING = 3;
unsigned char TOUCH_UP_COUNTER;
void ESD_HW_REST(void);
#endif

static int himax_chip_self_test(void);
static uint8_t rFE96_setting[8] = {0xB4, 0x64, 0x3F, 0x3F,
				   0x3C, 0x00, 0x3C, 0x00};
static int self_test_delay_time = 3;
static ssize_t himax_self_test_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

/*----[HX_TP_SYS_DEBUG_LEVEL]----------------------------------start*/
static uint8_t debug_log_level;
static bool fw_update_complete;
static bool fw_update_OK;
static bool irq_enable;
static int handshaking_result;
static unsigned char debug_level_cmd;
static unsigned char upgrade_fw[32*1024];
static uint8_t getDebugLevel(void);
/*----[HX_TP_SYS_DEBUG_LEVEL]------------------------------------end*/

/*----[HX_TP_SYS_REGISTER]-------------------------------------start*/
static uint8_t register_command;
static uint8_t multi_register_command;
static uint8_t multi_register[8] = {0x00};
static uint8_t multi_cfg_bank[8] = {0x00};
static uint8_t multi_value[1024] = {0x00};
static bool config_bank_reg;
/*----[HX_TP_SYS_REGISTER]---------------------------------------end*/

/*----[HX_TP_PROC_DIAG]----------------------------------------start*/
#define HIMAX_PROC_DIAG_FILE "himax_diag"
static uint8_t x_channel;
static uint8_t y_channel;
static uint8_t *diag_mutual;
static uint8_t diag_command;
static uint8_t diag_coor[128];
static uint8_t diag_self[100] = {0};

static uint8_t *getMutualBuffer(void);
static uint8_t *getSelfBuffer(void);
static uint8_t getDiagCommand(void);
static uint8_t getXChannel(void);
static uint8_t getYChannel(void);

static void setMutualBuffer(void);
static void setXChannel(uint8_t x);
static void setYChannel(uint8_t y);

static uint8_t coordinate_dump_enable;
struct file *coordinate_fn;
/*----[HX_TP_PROC_DIAG]------------------------------------------end*/

/*----[HX_TP_SYS_FLASH_DUMP]-----------------------------------start*/
#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t *flash_buffer;
static uint8_t flash_command;
static uint8_t flash_read_step;
static uint8_t flash_progress;
static uint8_t flash_dump_complete;
static uint8_t flash_dump_fail;
static uint8_t sys_operation;
static uint8_t flash_dump_sector;
static uint8_t flash_dump_page;
static bool flash_dump_going;

static uint8_t getFlashCommand(void);
static uint8_t getFlashDumpComplete(void);
static uint8_t getFlashDumpFail(void);
static uint8_t getFlashDumpProgress(void);
static uint8_t getFlashReadStep(void);
static uint8_t getSysOperation(void);
static uint8_t getFlashDumpSector(void);
static uint8_t getFlashDumpPage(void);
static bool getFlashDumpGoing(void);
static int himax_touch_sysfs_init(void);
static void himax_touch_sysfs_deinit(void);

static void setFlashBuffer(void);
static void setFlashCommand(uint8_t command);
static void setFlashReadStep(uint8_t step);
static void setFlashDumpComplete(uint8_t complete);
static void setFlashDumpFail(uint8_t fail);
static void setFlashDumpProgress(uint8_t progress);
static void setSysOperation(uint8_t operation);
static void setFlashDumpSector(uint8_t sector);
static void setFlashDumpPage(uint8_t page);
static void setFlashDumpGoing(bool going);
#endif
/*----[HX_TP_SYS_FLASH_DUMP]-------------------------------------end*/

static int cable_status = 2;

/*
* ====================================================================
*		Segment : Himax Normal Function
* ====================================================================
*/
/*----[ normal function]---------------------------------------start*/

/*[status]2:usb no plug-in. 0,1: usb plug in*/
int himax_cable_status(int status)
{
	uint8_t buf0[2] = {0};

	pr_info("[Himax] %s: cable_status=%d init_success=%d\n",
				__func__, status, private_ts->init_success);
	cable_status = status;
	if (private_ts->init_success == 1) {
		if (status == 0x02) {
			buf0[0] = 0x00;
			i2c_himax_write(touch_i2c, 0xF0, &buf0[0], 1, DEFAULT_RETRY_CNT);
		} else if ((status == 0x00) || (status == 0x01)) {
			buf0[0] = 0x01;
			i2c_himax_write(touch_i2c, 0xF0 , &buf0[0], 1, DEFAULT_RETRY_CNT);
		}
		return 0;
	}
	return 0;
}
EXPORT_SYMBOL(himax_cable_status);

void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4;

	if ((HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4;
	else
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) + 1) * 4;
}

void himax_ic_package_check(struct himax_ts_data *ts_modify)
{
	IC_TYPE = HX_85XX_E_SERIES_PWON;
	IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	FW_VER_MAJ_FLASH_ADDR = 133;  /*0x0085*/
	FW_VER_MAJ_FLASH_LENG = 1;
	FW_VER_MIN_FLASH_ADDR = 134;  /*0x0086*/
	FW_VER_MIN_FLASH_LENG = 1;
	CFG_VER_MAJ_FLASH_ADDR = 160; /*0x00A0*/
	CFG_VER_MAJ_FLASH_LENG = 12;
	CFG_VER_MIN_FLASH_ADDR = 172; /*0x00AC*/
	CFG_VER_MIN_FLASH_LENG = 12;
	pr_info("Himax IC package 8530 E\n");
}

static int himax_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct himax_ts_data *ts_modify = i2c_get_clientdata(client);
	uint8_t buf[2] = {0};
	int ret = 0;

	is_suspend = false;

#ifdef HX_TP_SYS_FLASH_DUMP
	if (getFlashDumpGoing()) {
		pr_info("[himax] %s: Flash dump is going, reject suspend\n", __func__);
		return 0;
	}
#endif

	pr_info("[himax] %s: TS suspend\n", __func__);

	wake_lock(&ts_modify->wake_lock);
	mutex_lock(&ts_modify->mutex_lock);

	buf[0] = HX_CMD_TSSOFF; /* MCU off */
	ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);
	if (ret < 0)
		pr_info("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
	msleep(120);

	buf[0] = HX_CMD_TSSLPIN; /* Sleep In */
	ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);
	if (ret < 0)
		pr_info("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
	msleep(120);

	mutex_unlock(&ts_modify->mutex_lock);
	wake_unlock(&ts_modify->wake_lock);

#ifdef ENABLE_CHIP_STATUS_MONITOR
	ts_modify->running_status = 1;
	cancel_delayed_work_sync(&ts_modify->himax_chip_monitor);
#endif

	disable_irq(client->irq);

	ret = cancel_work_sync(&ts_modify->work);
	if (ret)
		enable_irq(client->irq);

	is_suspend = true;

	return 0;
}

static int himax_ts_resume(struct i2c_client *client)
{
	struct himax_ts_data *ts_modify = i2c_get_clientdata(client);
	uint8_t buf[11] = {0};
	int ret = 0;
	pr_info("[himax] %s: TS resume\n", __func__);

	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = 0x00;
	buf[9] = 0x00;
	if (i2c_himax_write(touch_i2c, 0xF6 , &buf[0], 10, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (!is_suspend) {
		pr_info("[himax] %s TP never enter suspend , reject the resume action\n", __func__);
		return 0;
	}

	wake_lock(&ts_modify->wake_lock);

	buf[0] = HX_CMD_TSSON; /* 0x83 MCU on */
	ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);
	if (ret < 0)
		pr_info("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
	msleep(120);

	buf[0] = HX_CMD_TSSLPOUT; /* 0x81 Sleep Out */
	ret = i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT);
	if (ret < 0)
		pr_info("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts_modify->client->addr);
	msleep(120);

	wake_unlock(&ts_modify->wake_lock);

#ifdef HX_ESD_WORKAROUND_HANDSHAKING
	ret = himax_hang_shaking(); /* 0:Running, 1:Stop, 2:I2C Fail */
	if (ret == 2) {
		queue_delayed_work(ts_modify->himax_wq, &ts_modify->himax_chip_reset_work, 0);
		pr_info("[Himax] %s: I2C Fail\n", __func__);
	}
	if (ret == 1) {
		pr_info("[Himax] %s: MCU Stop\n", __func__);
		ESD_HW_REST();
	} else
		pr_info("[Himax] %s: MCU Running\n", __func__);
#endif

	enable_irq(client->irq);

#ifdef ENABLE_CHIP_STATUS_MONITOR
	/*for ESD solution */
	queue_delayed_work(ts_modify->himax_wq, &ts_modify->himax_chip_monitor, 10*HZ);
#endif

#ifdef HX_ESD_WORKAROUND
	ESD_COUNTER = 0;
#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax_ts_resume(ts->client);
}
#endif
/*----[ normal function]---------------------------------------------------end*/

/*----[ i2c read/write function]-----------------------------------------start*/
static int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;

		msleep(10);
	}
	if (retry == toRetry) {
		pr_info("[TP] %s: i2c_read_block retry over %d\n", __func__, toRetry);
		return -EIO;
	}
	return 0;
}

static int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t)*(length+1), GFP_KERNEL);

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;

		msleep(10);
	}

	if (retry == toRetry) {
		pr_info("[TP] %s: i2c_write_block retry over %d\n", __func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;
}

static int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t)*length, GFP_KERNEL);

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i] = data[loop_i];
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;

		msleep(10);
	}

	if (retry == toRetry) {
		pr_info("[TP] %s: i2c_write_block retry over %d\n", __func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;
}
/*----[ i2c read/write function]-------------------------------------------end*/

/*----[ register flow function]------------------------------------------start*/
static int himax_ts_poweron(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[20];
	int ret = 0;

	wake_lock(&ts_modify->wake_lock);
	mutex_lock(&ts_modify->mutex_lock);

	/*[ HX_85XX_E_SERIES_PWON]*/
	buf0[0] = 0x35; /*reload disable*/
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);

	buf0[0] = 0x42;
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);

#ifdef HX_85XX_E_nWATCH_DOG
	buf0[0] = 0xCA; /*Disable WDT*/
	buf0[1] = 0xA5;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);

	buf0[0] = 0xCE; /*Disable WDT*/
	buf0[1] = 0xCA;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);
#endif

	buf0[0] = 0x36; /*ROMRDY*/
	buf0[1] = 0x0F;
	buf0[2] = 0x53;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);

	buf0[0] = 0xDD; /*prefetch*/
	buf0[1] = 0x05; /*wait 6.5T*/
	buf0[2] = 0x03; /*enable prefetch and cache*/
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);

	buf0[0] = 0xCB;
	buf0[1] = 0x01;
	buf0[2] = 0xF5;
	buf0[3] = 0xFF;
	buf0[4] = 0xFF;
	buf0[5] = 0x01;
	buf0[6] = 0x00;
	buf0[7] = 0x05;
	buf0[8] = 0x00;
	buf0[9] = 0x0F;
	buf0[10] = 0x00;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 11, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	udelay(100);

	buf0[0] = 0x83;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	msleep(120);

	buf0[0] = 0x81;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("i2c_master_send failed addr = 0x%x\n", ts_modify->client->addr);
		goto send_i2c_msg_fail;
	}
	msleep(120);

	mutex_unlock(&ts_modify->mutex_lock);
	wake_unlock(&ts_modify->wake_lock);

	return ret;

send_i2c_msg_fail:
	pr_info("[Himax]:send_i2c_msg_failline: %d\n", __LINE__);

	mutex_unlock(&ts_modify->mutex_lock);
	wake_unlock(&ts_modify->wake_lock);

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
	return ret;
}

int himax_ManualMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;

	if (i2c_himax_write(touch_i2c, 0x42, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

int himax_FlashMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(void)
{
	uint8_t cmd[5];

	/* lock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x7D;
	cmd[3] = 0x03;
	if (i2c_himax_write(touch_i2c, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(touch_i2c, 0x4A, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(50);
	return 0;
	/* lock sequence stop */
}

static int himax_unlock_flash(void)
{
	uint8_t cmd[5];

	/* unlock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x3D;
	cmd[3] = 0x03;
	if (i2c_himax_write(touch_i2c, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(touch_i2c, 0x4A, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(50);

	return 0;
	/* unlock sequence stop */
}

static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)
{
	/*----[ HX_TP_BIN_CHECKSUM_CRC]----*/
	uint8_t cmd[5];

	/*Set Flash Clock Rate*/
	if (i2c_himax_read(touch_i2c, 0xED, cmd, 5, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	cmd[3] = 0x04;

	if (i2c_himax_write(touch_i2c, 0xED, &cmd[0], 5, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	/*Enable Flash*/
	/*himax_FlashMode(1);*/
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x02;
	if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	/*Select CRC Mode*/
	cmd[0] = 0x05;
	if (i2c_himax_write(touch_i2c, 0xD2, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	/*Enable CRC Function*/
	cmd[0] = 0x01;
	if (i2c_himax_write(touch_i2c, 0x53, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	/*Must delay 30 ms*/
	msleep(30);

	/*Read HW CRC*/
	if (i2c_himax_read(touch_i2c, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0) {
		himax_FlashMode(0);
		pr_info("[Himax]%s: checksum pass\n", __func__);
		return 1;
	} else {
		himax_FlashMode(0);
		pr_info("[Himax]%s: checksum fail\n", __func__);
		return 0;
	}
	return 0;
}

#ifdef HX_RST_PIN_FUNC
void himax_HW_reset(void)
{
#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 2;
#endif
	gpio_set_value(private_ts->rst_gpio, 0);
	msleep(100);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(100);
}
#endif
/*----[ register flow function]--------------------------------------------end*/

int himax_hang_shaking(void)    /* 0:Running, 1:Stop, 2:I2C Fail */
{
	int ret, result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];

	mutex_lock(&private_ts->mutex_lock);

	buf0[0] = 0x92;
	if (IC_STATUS_CHECK == 0xAA) {
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	} else {
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}

	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("[Himax]:write 0x92 failed line: %d\n", __LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(15);	/* Must more than 1 frame */

	buf0[0] = 0x92;
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("[Himax]:write 0x92 failed line: %d\n", __LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	msleep(2);

	ret = i2c_himax_read(private_ts->client, 0xDA, hw_reset_check, 1, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		pr_info("[Himax]:i2c_himax_read 0xDA failed line: %d\n", __LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	/*pr_info("[Himax]: ESD 0xDA - 0x%x.\n", hw_reset_check[0]);*/

	if ((IC_STATUS_CHECK != hw_reset_check[0])) {
		msleep(2);
		ret = i2c_himax_read(private_ts->client, 0xDA, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			pr_info("[Himax]:i2c_himax_read 0xDA failed line: %d\n", __LINE__);
			goto work_func_send_i2c_msg_fail;
		}
		/*pr_info("[Himax]: ESD check 2 0xDA - 0x%x.\n", hw_reset_check_2[0]);*/

		if (hw_reset_check[0] == hw_reset_check_2[0])
			result = 1; /*MCU Stop*/
		else
			result = 0; /*MCU Running*/
	} else
		result = 0; /*MCU Running*/

	mutex_unlock(&private_ts->mutex_lock);
	return result;

work_func_send_i2c_msg_fail:
	mutex_unlock(&private_ts->mutex_lock);
	return 2;
}

#ifdef ENABLE_CHIP_RESET_MACHINE
static void himax_chip_reset_function(struct work_struct *dat)
{
	pr_info("[Himax]:Enter %s\n", __func__);

	wake_lock(&private_ts->wake_lock);
	mutex_lock(&private_ts->mutex_lock);

#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 2;
#endif
	gpio_set_value(private_ts->rst_gpio, 0);
	msleep(30);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(30);

	mutex_unlock(&private_ts->mutex_lock);

	himax_ts_poweron(private_ts);

	wake_unlock(&private_ts->wake_lock);
	if (private_ts->init_success)
		himax_cable_status(cable_status);
}
#endif

#ifdef HX_ESD_WORKAROUND
void ESD_HW_REST(void)
{
	ESD_RESET_ACTIVATE = 2;
	ESD_COUNTER = 0;

	pr_info("Himax TP: ESD - Reset\n");

	wake_lock(&private_ts->wake_lock);
	mutex_lock(&private_ts->mutex_lock);

	gpio_set_value(private_ts->rst_gpio, 0);
	msleep(30);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(30);

	mutex_unlock(&private_ts->mutex_lock);

	himax_ts_poweron(private_ts);

	wake_unlock(&private_ts->wake_lock);

	if (gpio_get_value(private_ts->intr_gpio) == 0) {
		pr_info("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
		enable_irq(private_ts->client->irq);
	}

	if (private_ts->init_success)
		himax_cable_status(cable_status);
}
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
static int himax_chip_monitor_function(struct work_struct *dat) /*for ESD solution*/
{
	int ret;

	if (private_ts->running_status == 0) {
		if (gpio_get_value(private_ts->intr_gpio) == 0) {
			pr_info("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
			enable_irq(private_ts->client->irq);
		}

		ret = himax_hang_shaking(); /*0:Running, 1:Stop, 2:I2C Fail*/
		if (ret == 2) {
			queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
			pr_info("[Himax] %s: I2C Fail\n", __func__);
		}
		if (ret == 1) {
			pr_info("[Himax] %s: MCU Stop\n", __func__);
			private_ts->retry_time = 0;
			ESD_HW_REST();
		}

		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 10*HZ);
	}
	return 0;
}
#endif

/*
#ifdef HX_PORTING_DEB_MSG
static int himax_i2c_test_function(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[5];
	int ret = 0;

	buf0[0] = 0xE9;
	buf0[1] = 0x01;
	buf0[2] = 0x01;

	while (1) {
		ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);
		if (ret < 0)
			pr_info("*****HIMAX PORTING: i2c_master_send failed addr = 0x%x\n",ts_modify->client->addr);
		else
			pr_info("*****HIMAX PORTING: OK addr = 0x%x\n",ts_modify->client->addr);
		msleep(200);
	}
	return ret;
}
#endif
*/

u32 fw_version_int(void)
{
	u32 version = 0;
	int i = 0;
	for (i = 0; i < FW_VER_MAJ_FLASH_LENG; i++) {
		if (i != 0)
			version = version << 8;
		version += FW_VER_MAJ_buff[i];
	}
	for (i = 0; i < FW_VER_MIN_FLASH_LENG; i++) {
		version = version << 8;
		version += FW_VER_MIN_buff[i];
	}
	return version;
}

u8 himax_read_FW_ver(void)
{
	u16 fw_ver_maj_start_addr;
	u16 fw_ver_maj_end_addr;
	u16 fw_ver_maj_addr;
	u16 fw_ver_maj_length;

	u16 fw_ver_min_start_addr;
	u16 fw_ver_min_end_addr;
	u16 fw_ver_min_addr;
	u16 fw_ver_min_length;

	u16 cfg_ver_maj_start_addr;
	u16 cfg_ver_maj_end_addr;
	u16 cfg_ver_maj_addr;
	u16 cfg_ver_maj_length;

	u16 cfg_ver_min_start_addr;
	u16 cfg_ver_min_end_addr;
	u16 cfg_ver_min_addr;
	u16 cfg_ver_min_length;

	uint8_t cmd[4];
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;

	if (fw_version_int() > 0 && !fw_update_OK)
		return 0;

	fw_ver_maj_start_addr = FW_VER_MAJ_FLASH_ADDR / 4;				/* start addr = 133 / 4 = 33 */
	fw_ver_maj_length = FW_VER_MAJ_FLASH_LENG;					/* length = 1 */
	fw_ver_maj_end_addr = (FW_VER_MAJ_FLASH_ADDR + fw_ver_maj_length) / 4 + 1;	/* end addr = 134 / 4 = 33 */
	fw_ver_maj_addr = FW_VER_MAJ_FLASH_ADDR % 4;					/* 133 mod 4 = 1 */

	fw_ver_min_start_addr = FW_VER_MIN_FLASH_ADDR / 4;				/* start addr = 134 / 4 = 33 */
	fw_ver_min_length = FW_VER_MIN_FLASH_LENG;					/* length = 1 */
	fw_ver_min_end_addr = (FW_VER_MIN_FLASH_ADDR + fw_ver_min_length) / 4 + 1;	/* end addr = 135 / 4 = 33 */
	fw_ver_min_addr = FW_VER_MIN_FLASH_ADDR % 4;					/* 134 mod 4 = 2 */

	cfg_ver_maj_start_addr = CFG_VER_MAJ_FLASH_ADDR / 4;				/* start addr = 160 / 4 = 40 */
	cfg_ver_maj_length = CFG_VER_MAJ_FLASH_LENG;					/* length = 12 */
	cfg_ver_maj_end_addr = (CFG_VER_MAJ_FLASH_ADDR + cfg_ver_maj_length) / 4 + 1;	/* end addr = (160 + 12) / 4 = 43 */
	cfg_ver_maj_addr = CFG_VER_MAJ_FLASH_ADDR % 4;					/* 160 mod 4 = 0 */

	cfg_ver_min_start_addr = CFG_VER_MIN_FLASH_ADDR / 4;				/* start addr = 172 / 4 = 43 */
	cfg_ver_min_length = CFG_VER_MIN_FLASH_LENG;					/* length = 12 */
	cfg_ver_min_end_addr = (CFG_VER_MIN_FLASH_ADDR + cfg_ver_min_length) / 4 + 1;	/* end addr = (172 + 12) / 4 = 46 */
	cfg_ver_min_addr = CFG_VER_MIN_FLASH_ADDR % 4;					/* 172 mod 4 = 0 */

	pr_info("[Himax]:%s\n", __func__);
	disable_irq(private_ts->client->irq);
#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif
	/*Sleep out*/
	if (i2c_himax_write(touch_i2c, 0x81, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
		pr_info("[TP] %s: i2c access fail!\n", __func__);
		goto firmware_read_fail;
	}
	mdelay(120);

	/*Enter flash mode*/
	himax_FlashMode(1);

	/*Read Flash Start*/
	/*FW Version MAJ*/
	i = fw_ver_maj_start_addr;
	do {
		cmd[0] = i & 0x1F;	   /* column = 33 mod 32 = 1 */
		cmd[1] = (i >> 5) & 0x1F;  /* page = 33 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F; /* sector = 33 / 1024 = 0 */

		if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_write(touch_i2c, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}

		if (i == fw_ver_maj_start_addr) {
			/*first page*/
			j = 0;
			for (k = fw_ver_maj_addr; k < 4 && j < fw_ver_maj_length; k++)
				FW_VER_MAJ_buff[j++] = cmd[k];
		} else {
			/*other page*/
			for (k = 0; k < 4 && j < fw_ver_maj_length; k++)
				FW_VER_MAJ_buff[j++] = cmd[k];
		}
		i++;
	} while (i < fw_ver_maj_end_addr);

	/*FW Version MIN*/
	i = fw_ver_min_start_addr;
	do {
		cmd[0] = i & 0x1F;         /* column = 33 mod 32 = 1 */
		cmd[1] = (i >> 5) & 0x1F;  /* page = 33 / 32  = 1 */
		cmd[2] = (i >> 10) & 0x1F; /* sector= 33 / 1024 = 0 */

		if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_write(touch_i2c, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}

		if (i == fw_ver_min_start_addr) {
			/* first page */
			j = 0;
			for (k = fw_ver_min_addr; k < 4 && j < fw_ver_min_length; k++)
				FW_VER_MIN_buff[j++] = cmd[k];
		} else {
			/* other page */
			for (k = 0; k < 4 && j < fw_ver_min_length; k++)
				FW_VER_MIN_buff[j++] = cmd[k];
		}
		i++;
	} while (i < fw_ver_min_end_addr);


	/* CFG Version MAJ */
	i = cfg_ver_maj_start_addr;
	do {
		cmd[0] = i & 0x1F;         /* column = 40 mod 32 = 8 */
		cmd[1] = (i >> 5) & 0x1F;  /* page = 40 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F; /* sector = 40 / 1024 = 0 */

		if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_write(touch_i2c, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}

		if (i == cfg_ver_maj_start_addr) {
			/*first page*/
			j = 0;
			for (k = cfg_ver_maj_addr; k < 4 && j < cfg_ver_maj_length; k++)
				CFG_VER_MAJ_buff[j++] = cmd[k];

		} else {
			/*other page*/
			for (k = 0; k < 4 && j < cfg_ver_maj_length; k++)
				CFG_VER_MAJ_buff[j++] = cmd[k];
		}
		i++;
	} while (i < cfg_ver_maj_end_addr);

	/*CFG Version MIN*/
	i = cfg_ver_min_start_addr;
	do {
		cmd[0] = i & 0x1F;         /* column = 43 mod 32 = 11 */
		cmd[1] = (i >> 5) & 0x1F;  /* page = 43 / 32 = 1 */
		cmd[2] = (i >> 10) & 0x1F; /* sector = 43 / 1024 = 0 */

		if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_write(touch_i2c, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}
		if (i2c_himax_read(touch_i2c, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			goto firmware_read_fail;
		}

		if (i == cfg_ver_min_start_addr) {
			/* first page */
			j = 0;
			for (k = cfg_ver_min_addr; k < 4 && j < cfg_ver_min_length; k++)
				CFG_VER_MIN_buff[j++] = cmd[k];

		} else {
			/* other page */
			for (k = 0; k < 4 && j < cfg_ver_min_length; k++)
				CFG_VER_MIN_buff[j++] = cmd[k];
		}
		i++;
	} while (i < cfg_ver_min_end_addr);

	/* Exit flash mode */
	himax_FlashMode(0);

	printk("FW_VER_MAJ_buff : %d\n", FW_VER_MAJ_buff[0]);
	printk("FW_VER_MIN_buff : %d\n", FW_VER_MIN_buff[0]);

	printk("CFG_VER_MAJ_buff : ");
	for (i = 0; i < 12; i++)
		printk(" %d,", CFG_VER_MAJ_buff[i]);
	printk("\n");

	printk("CFG_VER_MIN_buff : ");
	for (i = 0; i < 12; i++)
		printk(" %d,", CFG_VER_MIN_buff[i]);
	printk("\n");

	fw_update_OK = false;
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
	enable_irq(private_ts->client->irq);
	return 0;

firmware_read_fail:
	memset(FW_VER_MAJ_buff, 0x00, FW_VER_MAJ_FLASH_LENG);
	memset(FW_VER_MIN_buff, 0x00, FW_VER_MIN_FLASH_LENG);
	memset(CFG_VER_MAJ_buff, 0x00, CFG_VER_MAJ_FLASH_LENG);
	memset(CFG_VER_MIN_buff, 0x00, CFG_VER_MIN_FLASH_LENG);
	enable_irq(private_ts->client->irq);
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
	return 1;
}

void himax_touch_information(struct i2c_client *client)
{
	struct touch_platform_data *pd;

	/* For TF303K/ME103K */
	if (asustek_get_tp_type() == TP_IC_TYPE_B) {
		pd = &(((struct touch_platform_data *)
			client->dev.platform_data)[1]);
	} else {
		pd = &(((struct touch_platform_data *)
			client->dev.platform_data)[0]);
	}

	HX_RX_NUM = pd->hx_rx_num;
	HX_TX_NUM = pd->hx_tx_num;
	HX_MAX_PT = 10;
	HX_BT_NUM = 0;
	HX_X_RES = pd->hx_x_res;
	HX_Y_RES = pd->hx_y_res;
	HX_INT_IS_EDGE = false;
}

#ifdef HX_FW_UPDATE_BY_I_FILE
int fts_ctpm_fw_upgrade_with_i_file(void)
{
	unsigned char *ImageBuffer = i_CTPM_FW;
	int fullFileLength = sizeof(i_CTPM_FW);

	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	/* Try 3 Times */
	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength;
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
#endif
		if (i2c_himax_write(touch_i2c, 0x81, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}

		mdelay(120);

		himax_unlock_flash();

		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i2c_himax_write(touch_i2c, 0x4F, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}
		mdelay(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) {
			last_byte = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;

			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];

				cmd[0] = 0x01;
				cmd[1] = 0x09;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x0D;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x09;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);
			if (i2c_himax_write(touch_i2c, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x0D;
			if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x09;
			if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (last_byte == 1) {
				cmd[0] = 0x01;
				cmd[1] = 0x01;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x05;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x01;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x00;
				if (i2c_himax_write(touch_i2c, 0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				mdelay(10);
				if (i == (FileLength - 1)) {
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);

					himax_lock_flash();

					if (checksumResult)
						return 1;	/*Success*/
					else
						return 0;	/*Fail*/
				}
			}
		}
	}
	return 0;
}

static bool i_Check_FW_Version()
{
	himax_read_FW_ver();

	/*TODO modify the condition by your project*/
	/*Here Only check fw_ver_min*/
	pr_info("[Himax] FW_VER_MIN_buff[0]: %d\n", FW_VER_MIN_buff[0]);
	pr_info("[Himax] i_CTPM_FW[%d]: %d\n", FW_VER_MIN_FLASH_ADDR, i_CTPM_FW[FW_VER_MIN_FLASH_ADDR]);
	pr_info("[Himax] CFG_VER_MIN_buff[11]: %d\n", CFG_VER_MIN_buff[11]);
	pr_info("[Himax] i_CTPM_FW[%d]: %d\n", CFG_VER_MIN_FLASH_ADDR+11, i_CTPM_FW[CFG_VER_MIN_FLASH_ADDR + 11]);

	if (FW_VER_MAJ_buff[0] == 0xFF || FW_VER_MIN_buff[0] == 0xFF ||
			CFG_VER_MAJ_buff[11] == 0xFF || CFG_VER_MIN_buff[11] == 0xFF) {
		pr_info("[Himax] i_Check_FW_Version return true. FW version:0xff\n");
		return true;
	}

	if (FW_VER_MIN_buff[0] < i_CTPM_FW[FW_VER_MIN_FLASH_ADDR]) {
		pr_info("[Himax] i_Check_FW_Version return true\n");
		return true;
	} else if (FW_VER_MIN_buff[0] == i_CTPM_FW[FW_VER_MIN_FLASH_ADDR]) {
		if (CFG_VER_MIN_buff[11] < i_CTPM_FW[CFG_VER_MIN_FLASH_ADDR + 11]) {
			pr_info("[Himax] i_Check_FW_Version return true\n");
			return true;
		}
	}
	pr_info("[Himax] i_Check_FW_Version return false\n");
	return false;
}

static int i_update_func()
{
	unsigned char *ImageBuffer = i_CTPM_FW;
	int fullFileLength = sizeof(i_CTPM_FW);

	if (i_Check_FW_Version() > 0 || himax_calculateChecksum(ImageBuffer, fullFileLength) == 0) {
		if (fts_ctpm_fw_upgrade_with_i_file() == 0)
			pr_info("TP upgrade error, line: %d\n", __LINE__);
		else {
			pr_info("TP upgrade OK, line: %d\n", __LINE__);
			fw_update_OK = true;
		}
	}
	return 0;
}
#endif
/*
* ==============================================================================
*		Segment : Himax SYS Debug Function
* ==============================================================================
*/
/*----[HX_TP_SYS_REGISTER]-----------------------------------------------start*/
#ifdef HX_TP_SYS_FS
static ssize_t himax_register_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i, loop_j;
	uint8_t inData[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(inData, 0x00, sizeof(inData));

	pr_info("Himax multi_register_command = %d\n", multi_register_command);

	if (multi_register_command == 1) {
		base = 0;

		for (loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				/*config bank register*/
				if (multi_cfg_bank[loop_i] == 1) {
					outData[0] = 0x15;
					i2c_himax_write(touch_i2c, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
					msleep(10);

					outData[0] = 0x00;
					outData[1] = multi_register[loop_i];
					i2c_himax_write(touch_i2c, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);
					msleep(10);

					i2c_himax_read(touch_i2c, 0x5A, inData, 128, DEFAULT_RETRY_CNT);

					outData[0] = 0x00;
					i2c_himax_write(touch_i2c, 0x8C , &outData[0], 1, DEFAULT_RETRY_CNT);

					for (loop_j = 0; loop_j < 128; loop_j++)
						multi_value[base++] = inData[loop_j];

				} else { /*normal register*/
					i2c_himax_read(touch_i2c, multi_register[loop_i], inData, 128, DEFAULT_RETRY_CNT);

					for (loop_j = 0; loop_j < 128; loop_j++)
						multi_value[base++] = inData[loop_j];
				}
			}
		}

		base = 0;
		for (loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1)
					ret += sprintf(buf + ret, "Register: FE(%x)\n", multi_register[loop_i]);
				else
					ret += sprintf(buf + ret, "Register: %x\n", multi_register[loop_i]);

				for (loop_j = 0; loop_j < 128; loop_j++) {
					ret += sprintf(buf + ret, "0x%2.2X ", multi_value[base++]);
					if ((loop_j % 16) == 15)
						ret += sprintf(buf + ret, "\n");
				}
			}
		}
		return ret;
	}

	if (config_bank_reg) {
		pr_info("[TP] %s: register_command = FE(%x)\n", __func__, register_command);

		/*Config bank register read flow.*/
		outData[0] = 0x15;
		i2c_himax_write(touch_i2c, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		outData[1] = register_command;
		i2c_himax_write(touch_i2c, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);

		msleep(10);

		i2c_himax_read(touch_i2c, 0x5A, inData, 128, DEFAULT_RETRY_CNT);

		msleep(10);

		outData[0] = 0x00;
		i2c_himax_write(touch_i2c, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);
	} else {
		if (i2c_himax_read(touch_i2c, register_command, inData, 128, DEFAULT_RETRY_CNT) < 0)
			return ret;
	}

	if (config_bank_reg)
		ret += sprintf(buf, "command: FE(%x)\n", register_command);
	else
		ret += sprintf(buf, "command: %x\n", register_command);

	for (loop_i = 0; loop_i < 128; loop_i++) {
		ret += sprintf(buf + ret, "0x%2.2X ", inData[loop_i]);
		if ((loop_i % 16) == 15)
			ret += sprintf(buf + ret, "\n");
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_register_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	unsigned long result = 0;
	uint8_t loop_i = 0;
	uint16_t base = 5;
	uint8_t write_da[128];
	uint8_t outData[5];
	if (count >= 80) {
		pr_info("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	pr_info("himax %s\n", buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		pr_info("himax multi register enter\n");

		multi_register_command = 1;

		base = 2;
		loop_i = 0;

		while (true) {
			if (buf[base] == '\n')
				break;

			if (loop_i >= 6)
				break;

			if (buf[base] == ':' && buf[base+1] == 'x' && buf[base+2] == 'F' && buf[base+3] == 'E' && buf[base+4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		pr_info("==========================\n");
		for (loop_i = 0; loop_i < 6; loop_i++)
			printk("%d,%d:", multi_register[loop_i], multi_cfg_bank[loop_i]);

		printk("\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			/*Config bank register*/
			if (buf[3] == 'F' && buf[4] == 'E') {
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
					register_command = result;

				base = 7;

				pr_info("CMD: FE(%x)\n", register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
					register_command = result;

				base = 5;
				pr_info("CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x15;
							i2c_himax_write(touch_i2c, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);

							msleep(10);

							outData[0] = 0x00;
							outData[1] = register_command;
							i2c_himax_write(touch_i2c, 0x8B, &outData[0], 2, DEFAULT_RETRY_CNT);

							msleep(10);
							i2c_himax_write(touch_i2c, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);

							msleep(10);

							outData[0] = 0x00;
							i2c_himax_write(touch_i2c, 0x8C, &outData[0], 1, DEFAULT_RETRY_CNT);

							pr_info("CMD: FE(%x), %x, %d\n", register_command, write_da[0], length);
						} else {
							i2c_himax_write(touch_i2c, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
							pr_info("CMD: %x, %x, %d\n", register_command, write_da[0], length);
						}
					}

					pr_info("\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul(buf_tmp, 16, &result))
						write_da[loop_i] = result;

					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO|S_IWGRP),
				himax_register_read, himax_register_write);
/*----[HX_TP_SYS_REGISTER]-------------------------------------------------end*/

/*----[HX_TP_SYS_DEBUG_LEVEL]--------------------------------------------start*/
static uint8_t getDebugLevel(void)
{
	return debug_log_level;
}

int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
	unsigned char *ImageBuffer = fw;
	int fullFileLength = len;
	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	/*Try 3 Times*/
	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength;
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
#endif
		if (i2c_himax_write(touch_i2c, 0x81, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}

		mdelay(120);

		himax_unlock_flash();

		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i2c_himax_write(touch_i2c, 0x4F, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}
		mdelay(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) {
			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;

			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(touch_i2c, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];
				cmd[0] = 0x01;
				cmd[1] = 0x09;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x0D;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x09;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);
			if (i2c_himax_write(touch_i2c, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x0D;
			if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x09;
			if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (last_byte == 1) {
				cmd[0] = 0x01;
				cmd[1] = 0x01;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x05;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x01;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;
				cmd[1] = 0x00;
				if (i2c_himax_write(touch_i2c, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_info("[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				mdelay(10);
				if (i == (FileLength - 1)) {
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);
					himax_lock_flash();

					if (checksumResult)
						return 1;   /*Success*/
					else
						return 0;   /*Fail*/

				}
			}
		}
	}
	return 0;
}

static ssize_t himax_debug_level_read(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i = 0;
	pr_info("[Himax]:himax_debug_level_read\n");

	if (debug_level_cmd == 't') {
		if (fw_update_complete)
			ret += sprintf(buf, "FW Update Complete\n");
		else
			ret += sprintf(buf, "FW Update Fail\n");

	} else if (debug_level_cmd == 'i') {
		if (irq_enable)
			ret += sprintf(buf, "IRQ is enable\n");
		else
			ret += sprintf(buf, "IRQ is disable\n");

	} else if (debug_level_cmd == 'h') {
		if (handshaking_result == 0)
			ret += sprintf(buf, "Handshaking Result = %d (MCU Running)\n", handshaking_result);
		else if (handshaking_result == 1)
			ret += sprintf(buf, "Handshaking Result = %d (MCU Stop)\n", handshaking_result);
		else if (handshaking_result == 2)
			ret += sprintf(buf, "Handshaking Result = %d (I2C Error)\n", handshaking_result);
		else
			ret += sprintf(buf, "Handshaking Result = error\n");

	} else if (debug_level_cmd == 'v') {
		pr_info("[Himax]:himax_debug_level_read v\n");
		ret += sprintf(buf + ret, "FW_VER_MAJ_buff = ");
		ret += sprintf(buf + ret, "0x%2.2X\n", FW_VER_MAJ_buff[0]);

		ret += sprintf(buf + ret, "FW_VER_MIN_buff = ");
		ret += sprintf(buf + ret, "0x%2.2X\n", FW_VER_MIN_buff[0]);

		ret += sprintf(buf + ret, "CFG_VER_MAJ_buff = ");
		for (i = 0; i < 12; i++)
			ret += sprintf(buf + ret, "0x%2.2X ", CFG_VER_MAJ_buff[i]);

		ret += sprintf(buf + ret, "\n");

		ret += sprintf(buf + ret, "CFG_VER_MIN_buff = ");
		for (i = 0; i < 12; i++)
			ret += sprintf(buf + ret, "0x%2.2X ", CFG_VER_MIN_buff[i]);

		ret += sprintf(buf + ret, "\n");
	} else if (debug_level_cmd == 'd') {
		ret += sprintf(buf + ret, "Himax Touch IC Information :\n");
		ret += sprintf(buf + ret, "IC Type : E\n");
		ret += sprintf(buf + ret, "IC Checksum : CRC\n");
		if (HX_INT_IS_EDGE)
			ret += sprintf(buf + ret, "Interrupt : EDGE TIRGGER\n");
		else
			ret += sprintf(buf + ret, "Interrupt : LEVEL TRIGGER\n");
		ret += sprintf(buf + ret, "RX Num : %d\n", HX_RX_NUM);
		ret += sprintf(buf + ret, "TX Num : %d\n", HX_TX_NUM);
		ret += sprintf(buf + ret, "BT Num : %d\n", HX_BT_NUM);
		ret += sprintf(buf + ret, "X Resolution : %d\n", HX_X_RES);
		ret += sprintf(buf + ret, "Y Resolution : %d\n", HX_Y_RES);
		ret += sprintf(buf + ret, "Max Point : %d\n", HX_MAX_PT);
	} else if (debug_level_cmd == 'p') {
		if (gPrint_point)
			ret += sprintf(buf, "Enable report data log\n");
		else
			ret += sprintf(buf, "Disable report data log\n");
	} else {
		ret += sprintf(buf, "%d\n", debug_log_level);
	}
	return ret;
}

static ssize_t himax_debug_level_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct file *local_filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];

	if (count >= 80) {
		pr_info("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n') {
		debug_log_level = buf[0] - '0';
		return count;
	}

	/*irq*/
	if (buf[0] == 'i') {
		debug_level_cmd = buf[0];

		if (buf[2] == '1') {
			/*enable irq*/
			enable_irq(private_ts->client->irq);
			irq_enable = true;
		} else if (buf[2] == '0') {
			/*disable irq*/
			disable_irq(private_ts->client->irq);
			irq_enable = false;
		} else {
			pr_info("[TP] %s: debug_level command = 'i' , parameter error.\n", __func__);
		}
		return count;
	}
	/*handshaking*/
	if (buf[0] == 'h') {
		debug_level_cmd = buf[0];

		disable_irq(private_ts->client->irq);

		handshaking_result = himax_hang_shaking(); /*0:Running, 1:Stop, 2:I2C Fail*/

		enable_irq(private_ts->client->irq);

		return count;
	}
	/*firmware version*/
	if (buf[0] == 'v')  {
		debug_level_cmd = buf[0];
		himax_read_FW_ver();
		return count;
	}
	/*test*/
	if (buf[0] == 'd') {
		debug_level_cmd = buf[0];
		return count;
	}

	if (buf[0] == 'p') {
		/*open report data log*/
		debug_level_cmd = buf[0];
		pr_info("[HIMAX]himax_debug_level_write_P\n");
		if (buf[2] == '1') {
			/*enable report data log*/
			gPrint_point = 1;
			pr_info("gPrint_point = %d\n", gPrint_point);
		} else if (buf[2] == '0') {
			/*disable report data log*/
			gPrint_point = 0;
			pr_info("gPrint_point = %d\n", gPrint_point);
		} else {
			pr_info("[TP] %s: debug_level command = 'p' , parameter error.\n", __func__);
		}
		return count;
	}

#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

	wake_lock(&private_ts->wake_lock);
	mutex_lock(&private_ts->mutex_lock);

	if (buf[0] == 't') {
		debug_level_cmd = buf[0];
		fw_update_complete = false;

		memset(fileName, 0, 128);
		/* parse the file name */
		snprintf(fileName, count-2, "%s", &buf[2]);
		pr_info("[TP] %s: upgrade from file(%s) start!\n", __func__, fileName);
		/* open file */
		local_filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(local_filp)) {
			pr_info("[TP] %s: open firmware file failed\n", __func__);
			goto firmware_upgrade_done;
		}
		oldfs = get_fs();
		set_fs(get_ds());

		/* read the latest firmware binary file */
		result = local_filp->f_op->read(local_filp, upgrade_fw, sizeof(upgrade_fw), &local_filp->f_pos);
		if (result < 0) {
			pr_info("[TP] %s: read firmware file failed\n", __func__);
			goto firmware_upgrade_done;
		}

		set_fs(oldfs);
		filp_close(local_filp, NULL);

		pr_info("[TP] %s: upgrade start,count %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

		if (result > 0) {
			/* start to upgrade */
			disable_irq(private_ts->client->irq);
			if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0) {
				pr_info("[TP] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				pr_info("[TP] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
				fw_update_OK = true;
			}
			enable_irq(private_ts->client->irq);
			goto firmware_upgrade_done;
		}
	}

#ifdef HX_FW_UPDATE_BY_I_FILE
	if (buf[0] == 'f') {
		pr_info("[TP] %s: upgrade firmware from kernel image start!\n", __func__);
		pr_info("himax touch isTP_Updated: %d\n", i_isTP_Updated);

		disable_irq(private_ts->client->irq);
		pr_info("himax touch firmware upgrade: %d\n", i_isTP_Updated);
		if (fts_ctpm_fw_upgrade_with_i_file() == 0) {
			pr_info("himax_marked TP upgrade error, line: %d\n", __LINE__);
			fw_update_complete = false;
		} else {
			pr_info("himax_marked TP upgrade OK, line: %d\n", __LINE__);
			fw_update_complete = true;
			fw_update_OK = true;
		}
		enable_irq(private_ts->client->irq);
		i_isTP_Updated = 1;
		goto firmware_upgrade_done;
	}
#endif

firmware_upgrade_done:

	mutex_unlock(&private_ts->mutex_lock);

#ifdef ENABLE_CHIP_RESET_MACHINE
	queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif

	wake_unlock(&private_ts->wake_lock);

#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 10*HZ);
#endif
	return count;
}
static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO|S_IWGRP),
				himax_debug_level_read, himax_debug_level_write);
/*----[HX_TP_SYS_DEBUG_LEVEL]----------------------------------------------end*/

#ifdef HX_TP_PROC_DIAG
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}

static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}

static uint8_t getXChannel(void)
{
	return x_channel;
}

static uint8_t getYChannel(void)
{
	return y_channel;
}

static uint8_t getDiagCommand(void)
{
	return diag_command;
}

static void setXChannel(uint8_t x)
{
	x_channel = x;
}

static void setYChannel(uint8_t y)
{
	y_channel = y;
}

static void setMutualBuffer(void)
{
	diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}
static void *himax_diag_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;
	return (void *)((unsigned long) *pos+1);
}

static void *himax_diag_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}
static void himax_diag_seq_stop(struct seq_file *s, void *v)
{
}
static int himax_diag_seq_read(struct seq_file *s, void *v)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel; /*don't add KEY_COUNT*/

	width = x_channel;

	seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);

	/* start to show out the raw data in adb shell */
	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command <= 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, " %3d\n", diag_self[width + loop_i/width]);
			}
			seq_printf(s, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					seq_printf(s, "\n");
			}

		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				seq_printf(s, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
					seq_printf(s, "\n");
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				seq_printf(s, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					seq_printf(s, "\n");
			}
		}
		seq_printf(s, "ChannelEnd");
		seq_printf(s, "\n");
	} else if (diag_command == 7) {
		for (loop_i = 0; loop_i < 128 ; loop_i++) {
			if ((loop_i % 16) == 0)
				seq_printf(s, "LineStart:");

			seq_printf(s, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15)
				seq_printf(s, "\n");
		}
	}
	return count;
}

static struct seq_operations himax_diag_seq_ops = {
	.start = himax_diag_seq_start,
	.next = himax_diag_seq_next,
	.stop = himax_diag_seq_stop,
	.show = himax_diag_seq_read
};
static int himax_diag_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_diag_seq_ops);
};
static ssize_t himax_diag_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0x00;

	uint8_t command_ec_128_raw_baseline_flag = 0x02;
	uint8_t command_ec_128_raw_bank_flag = 0x03;
	uint8_t command_F1h[2] = {0xF1, 0x00};

	char messages[80] = {0};

	if (len >= 80) {
		pr_info("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	command_ec_128_raw_baseline_flag = 0x02;
	command_ec_128_raw_bank_flag = 0x03;

	if (messages[0] == '1') {
		/*IIR*/
		command_F1h[1] = command_ec_128_raw_baseline_flag; /*E:0x02*/
		i2c_himax_write(touch_i2c, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = messages[0] - '0';
		pr_info("[Himax]diag_command=0x%x\n", diag_command);
	} else if (messages[0] == '2') {
		/*DC*/
		command_F1h[1] = command_ec_128_raw_flag; /*0x01*/
		i2c_himax_write(touch_i2c, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = messages[0] - '0';
		pr_info("[Himax]diag_command=0x%x\n", diag_command);
	} else if (messages[0] == '3') {
		/*BANK*/
		command_F1h[1] = command_ec_128_raw_bank_flag; /*0x03*/
		i2c_himax_write(touch_i2c, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = messages[0] - '0';
		pr_info("[Himax]diag_command=0x%x\n", diag_command);

	} else if (messages[0] == '7') {
		diag_command = messages[0] - '0';
	} else if (messages[0] == '8') {
		/*coordinate dump start*/
		diag_command = messages[0] - '0';

		coordinate_fn = filp_open(DIAG_COORDINATE_FILE, O_CREAT | O_WRONLY | O_APPEND | O_TRUNC, 0666);
		if (IS_ERR(coordinate_fn)) {
			pr_info("[HIMAX TP ERROR]%s: coordinate_dump_file_create error\n", __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn, NULL);
		}
		coordinate_dump_enable = 1;
	} else if (messages[0] == '9') {
		coordinate_dump_enable = 0;
		diag_command = messages[0] - '0';

		if (!IS_ERR(coordinate_fn))
			filp_close(coordinate_fn, NULL);
		/*coordinate dump end*/
	} else {
		command_F1h[1] = command_ec_24_normal_flag;
		i2c_himax_write(touch_i2c, command_F1h[0] , &command_F1h[1], 1, DEFAULT_RETRY_CNT);
		diag_command = 0;
		pr_info("[Himax]diag_command=0x%x\n", diag_command);
	}
	return len;
}

static struct file_operations himax_diag_ops = {
	.owner = THIS_MODULE,
	.open = himax_diag_proc_open,
	.read = seq_read,
	.write = himax_diag_write,
};
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t getFlashCommand(void)
{
	return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
	return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
	return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
	return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
	return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
	return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
	return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
	return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
	return flash_dump_going;
}

static void setFlashBuffer(void)
{
	int i = 0;
	flash_buffer = kzalloc(32768*sizeof(uint8_t), GFP_KERNEL);
	for (i = 0; i < 32768; i++)
		flash_buffer[i] = 0x00;
}

static void setSysOperation(uint8_t operation)
{
	sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
	flash_progress = progress;
	pr_info("TPPPP setFlashDumpProgress : progress = %d ,flash_progress = %d\n", progress, flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
	flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
	flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
	flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
	flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
	flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
	flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
	flash_dump_going = going;
}

static ssize_t himax_flash_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step = 0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	pr_info("TPPPP flash_progress = %d\n", local_flash_progress);

	if (local_flash_fail) {
		ret += sprintf(buf+ret, "FlashStart:Fail\n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (!local_flash_complete) {
		ret += sprintf(buf+ret, "FlashStart:Ongoing:0x%2.2x\n", flash_progress);
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 1 && local_flash_complete) {
		ret += sprintf(buf+ret, "FlashStart:Complete\n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 3 && local_flash_complete) {
		ret += sprintf(buf+ret, "FlashStart:\n");
		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
			if ((loop_i % 16) == 15)
				ret += sprintf(buf + ret, "\n");
		}
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	local_flash_read_step = getFlashReadStep();

	ret += sprintf(buf+ret, "FlashStart:%2.2x\n", local_flash_read_step);

	for (loop_i = 0; loop_i < 1024; loop_i++) {
		ret += sprintf(buf + ret, "x%2.2X", flash_buffer[local_flash_read_step*1024 + loop_i]);

		if ((loop_i % 16) == 15)
			ret += sprintf(buf + ret, "\n");
	}

	ret += sprintf(buf + ret, "FlashEnd");
	ret += sprintf(buf + ret, "\n");
	return ret;
}

/*
himax_flash_store
command 0 : Read the page by step number
command 1 : driver start to dump flash data, save it to mem
command 2 : driver start to dump flash data, save it to sdcard/Flash_Dump.bin
*/
static ssize_t himax_flash_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	if (count >= 80) {
		pr_info("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	pr_info("[TP] %s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1) {
		pr_info("[TP] %s: SYS is busy , return!\n", __func__);
		return count;
	}

	if (buf[0] == '0') {
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x') {
			memcpy(buf_tmp, buf + 3, 2);
			pr_info("[TP] %s: read_Step = %s\n", __func__, buf_tmp);
			if (!strict_strtoul(buf_tmp, 16, &result)) {
				pr_info("[TP] %s: read_Step = %lu\n", __func__, result);
				setFlashReadStep(result);
			}
		}
	} else if (buf[0] == '1') {
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '2') {
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '3') {
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result))
			setFlashDumpSector(result);

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result))
			setFlashDumpPage(result);

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	} else if (buf[0] == '4') {
		pr_info("[TP] %s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpSector(result);
		} else {
			pr_info("[TP] %s: command 4 , sector error.\n", __func__);
			return count;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result)) {
			setFlashDumpPage(result);
		} else {
			pr_info("[TP] %s: command 4 , page error.\n", __func__);
			return count;
		}

		base = 11;

		pr_info("=========Himax flash page buffer start=========\n");
		for (loop_i = 0; loop_i < 128; loop_i++) {
			memcpy(buf_tmp, buf + base, 2);
			if (!strict_strtoul(buf_tmp, 16, &result)) {
				flash_buffer[loop_i] = result;
				printk(" %d ", flash_buffer[loop_i]);
				if (loop_i % 16 == 15)
					printk("\n");
			}
			base += 3;
		}
		pr_info("=========Himax flash page buffer end=========\n");

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	return count;
}

static void himax_ts_flash_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);

	uint8_t page_tmp[128];
	uint8_t x59_tmp[4] = {0, 0, 0, 0};
	int i = 0, j = 0, k = 0, l = 0, buffer_ptr = 0;
	uint8_t local_flash_command = 0;
	uint8_t sector = 0;
	uint8_t page = 0;

	uint8_t x81_command[2] = {0x81, 0x00};
	uint8_t x82_command[2] = {0x82, 0x00};
	uint8_t x35_command[2] = {0x35, 0x00};
	uint8_t x43_command[4] = {0x43, 0x00, 0x00, 0x00};
	uint8_t x44_command[4] = {0x44, 0x00, 0x00, 0x00};
	uint8_t x45_command[5] = {0x45, 0x00, 0x00, 0x00, 0x00};
	uint8_t x46_command[2] = {0x46, 0x00};
	uint8_t x4A_command[2] = {0x4A, 0x00};
	uint8_t x4D_command[2] = {0x4D, 0x00};

	disable_irq(ts->client->irq);
	setFlashDumpGoing(true);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif

	sector = getFlashDumpSector();
	page = getFlashDumpPage();

	local_flash_command = getFlashCommand();

	/*sleep out*/
	if (i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0) {
		pr_info("[TP]TOUCH_ERR: %s i2c write 81 fail.\n", __func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(120);

	if (i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0) {
		pr_info("[TP]TOUCH_ERR: %s i2c write 82 fail.\n", __func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(100);

	pr_info("[TP] %s: local_flash_command = %d enter.\n", __func__, local_flash_command);
	pr_info("[TP] %s: flash buffer start.\n", __func__);
	for (i = 0; i < 128; i++) {
		printk(" %2.2x ", flash_buffer[i]);
		if (i % 16 == 15)
			printk("\n");
	}
	pr_info("[TP] %s: flash buffer end.\n", __func__);

	if (local_flash_command == 1 || local_flash_command == 2) {
		x43_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
			goto Flash_Dump_i2c_transfer_error;

		msleep(100);

		for (i = 0; i < 8; i++) {
			for (j = 0; j < 32; j++) {
				pr_info("TPPPP Step 2 i=%d, j=%d %s\n", i, j, __func__);
				/*read page start*/
				for (k = 0; k < 128; k++)
					page_tmp[k] = 0x00;

				for (k = 0; k < 32; k++) {
					x44_command[1] = k;
					x44_command[2] = j;
					x44_command[3] = i;
					if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
						pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
						goto Flash_Dump_i2c_transfer_error;
					}

					if (i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0) {
						pr_info("[TP]TOUCH_ERR: %s i2c write 46 fail.\n", __func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					if (i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0) {
						pr_info("[TP]TOUCH_ERR: %s i2c write 59 fail.\n", __func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					for (l = 0; l < 4; l++)
						page_tmp[k*4+l] = x59_tmp[l];
				}
				/*read page end*/

				for (k = 0; k < 128; k++)
					flash_buffer[buffer_ptr++] = page_tmp[k];

				setFlashDumpProgress(i*32 + j);
			}
		}
	} else if (local_flash_command == 3) {
		x43_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for (i = 0; i < 128; i++)
			page_tmp[i] = 0x00;

		for (i = 0; i < 32; i++) {
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;

			if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}

			if (i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 46 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			if (i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 59 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			for (j = 0; j < 4; j++)
				page_tmp[i*4+j] = x59_tmp[j];
		}
		/*read page end*/
		for (i = 0; i < 128; i++)
			flash_buffer[buffer_ptr++] = page_tmp[i];
	} else if (local_flash_command == 4) {
		/*page write flow.*/
		pr_info("[TP] %s: local_flash_command = 4, enter.\n", __func__);

		/*unlock flash*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x06;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x03;
		x44_command[2] = 0x00;
		x44_command[3] = 0x00;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x45_command[1] = 0x00;
		x45_command[2] = 0x00;
		x45_command[3] = 0x3D;
		x45_command[4] = 0x03;
		if (i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 45 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 4A fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(50);

		/*page erase*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x02;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 4D fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		/*enter manual mode*/
		x35_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x35_command[0], &x35_command[1], 1, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 35 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		/*flash enable*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*set flash address*/
		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*manual mode command:*/
		/*47 to latch the flash address when page address change.*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x0D;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		for (i = 0; i < 32; i++) {
			pr_info("himax :i=%d\n", i);
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;
			if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x45_command[1] = flash_buffer[i*4 + 0];
			x45_command[2] = flash_buffer[i*4 + 1];
			x45_command[3] = flash_buffer[i*4 + 2];
			x45_command[4] = flash_buffer[i*4 + 3];
			if (i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 45 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			/*manual mode command : 48 ,data will be written into flash buffer*/
			x43_command[1] = 0x01;
			x43_command[2] = 0x0D;
			if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);
		}

		/*manual mode command : 49 ,program data from flash buffer to this page*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x05;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*flash disable*/
		x43_command[1] = 0x00;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		/*leave manual mode*/
		x35_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x35_command[0], &x35_command[1], 1, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 35 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}

		/*lock flash*/
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x06;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x03;
		x44_command[2] = 0x00;
		x44_command[3] = 0x00;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x45_command[1] = 0x00;
		x45_command[2] = 0x00;
		x45_command[3] = 0x7D;
		x45_command[4] = 0x03;
		if (i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 45 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0) {
			pr_info("[TP]TOUCH_ERR: %s i2c write 4D fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}

		msleep(50);

		buffer_ptr = 128;
		pr_info("Himax: Flash page write Complete.\n");
	}

	pr_info("Himax: Complete.\n");

	pr_info(" buffer_ptr = %d\n", buffer_ptr);

	for (i = 0; i < buffer_ptr; i++) {
		printk("%2.2X ", flash_buffer[i]);
		if ((i % 16) == 15)
			printk("\n");
	}
	pr_info("Himax: End.\n");

	i2c_himax_master_write(ts->client, x43_command, 1, 3);
	msleep(50);

	if (local_flash_command == 2) {
		struct file *fn;

		fn = filp_open(FLASH_DUMP_FILE, O_CREAT | O_WRONLY , 0);
		if (!IS_ERR(fn)) {
			fn->f_op->write(fn, flash_buffer, buffer_ptr*sizeof(uint8_t), &fn->f_pos);
			filp_close(fn, NULL);
		}
	}

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif

	enable_irq(ts->client->irq);
	setFlashDumpGoing(false);

	setFlashDumpComplete(1);
	setSysOperation(0);
	return;

Flash_Dump_i2c_transfer_error:

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
	enable_irq(ts->client->irq);
	setFlashDumpGoing(false);
	setFlashDumpComplete(0);
	setFlashDumpFail(1);
	setSysOperation(0);
	return;
}
static DEVICE_ATTR(flash_dump, (S_IWUSR|S_IRUGO|S_IWGRP),
				   himax_flash_read, himax_flash_write);
#endif

/*----[HX_TP_SYS_SELF_TEST]----------------------------------------------start*/
static ssize_t himax_self_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0x00;
	val = himax_chip_self_test();
	return sprintf(buf, "%d\n", val);
	if (val == 0)
		return sprintf(buf, "PASS\n");
	else
		return sprintf(buf, "FAIL\n");
}

static int himax_chip_self_test(void)
{
	uint8_t cmdbuf[11];
	uint8_t valuebuf[16];
	int i = 0, pf_value = 0x00;

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif
	himax_ts_poweron(private_ts);

	/*Step 0 : sensor off*/
	i2c_himax_write(private_ts->client, 0x82, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);
	i2c_himax_write(private_ts->client, 0x80, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);
	cmdbuf[0] = 0xA5;
	i2c_himax_write(private_ts->client, 0xCA, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(100);
	cmdbuf[0] = 0xCA;
	i2c_himax_write(private_ts->client, 0xCE, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(100);

	/*Step 1 : Close Re-Calibration FE02*/
	/*-->Read 0xFE02*/
	cmdbuf[0] = 0x15;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	cmdbuf[1] = 0x02;
	i2c_himax_write(private_ts->client, 0x8B, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
	msleep(10);
	i2c_himax_read(private_ts->client, 0x5A, valuebuf, 2, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(30);

	pr_info("[Himax]:0xFE02_0 = 0x%x\n", valuebuf[0]);
	pr_info("[Himax]:0xFE02_1 = 0x%x\n", valuebuf[1]);
	/* close re-calibration, shift first byte of config bank register read issue.*/
	valuebuf[0] = valuebuf[0] & 0xFD;
	pr_info("[Himax]:0xFE02_valuebuf = 0x%x\n", valuebuf[0]);

	/*-->Write 0xFE02*/
	cmdbuf[0] = 0x15;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	cmdbuf[1] = 0x02;
	i2c_himax_write(private_ts->client, 0x8B, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = valuebuf[0];
	i2c_himax_write(private_ts->client, 0x40, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(30);
	/*0xFE02 Read Back*/

	/*-->Read 0xFE02*/
	cmdbuf[0] = 0x15;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	cmdbuf[1] = 0x02;
	i2c_himax_write(private_ts->client, 0x8B, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
	msleep(10);
	i2c_himax_read(private_ts->client, 0x5A, valuebuf, 2, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(30);

	pr_info("[Himax]:0xFE02_0_back = 0x%x\n", valuebuf[0]);
	pr_info("[Himax]:0xFE02_1_back = 0x%x\n", valuebuf[1]);

	/*Step 2 : Close Flash-Reload*/
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0xE3, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(30);
	i2c_himax_read(private_ts->client, 0xE3, valuebuf, 1, DEFAULT_RETRY_CNT);

	pr_info("[Himax]:0xE3_back = 0x%x\n", valuebuf[0]);

	/*Step 4 : Write self_test parameter to FE96~FE9D*/
	cmdbuf[0] = 0x15;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	cmdbuf[1] = 0x96;
	i2c_himax_write(private_ts->client, 0x8B, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
	msleep(10);

	/*-->Modify the initial value of self_test.*/
	cmdbuf[0] = rFE96_setting[0];
	cmdbuf[1] = rFE96_setting[1];
	cmdbuf[2] = rFE96_setting[2];
	cmdbuf[3] = rFE96_setting[3];
	cmdbuf[4] = rFE96_setting[4];
	cmdbuf[5] = rFE96_setting[5];
	cmdbuf[6] = rFE96_setting[6];
	cmdbuf[7] = rFE96_setting[7];
	i2c_himax_write(private_ts->client, 0x40, &cmdbuf[0], 8, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(30);
	/*Read back*/
	cmdbuf[0] = 0x15;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	cmdbuf[1] = 0x96;
	i2c_himax_write(private_ts->client, 0x8B, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
	msleep(10);
	i2c_himax_read(private_ts->client, 0x5A, valuebuf, 16, DEFAULT_RETRY_CNT);
	msleep(10);
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);

	for (i = 0; i < 15; i++)
		pr_info("[Himax]:0xFE96 buff_back[%d] = 0x%x\n", i, valuebuf[i]);

	msleep(30);

	/*Step 5 : Enter self_test mode*/
	cmdbuf[0] = 0x06;
	i2c_himax_write(private_ts->client, 0xF1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);

	i2c_himax_read(private_ts->client, 0xF1, valuebuf, 1, DEFAULT_RETRY_CNT);

	pr_info("[Himax]:0x91_back = 0x%x\n", valuebuf[0]);
	msleep(10);

	/*Step 6 : Sensor On*/
	i2c_himax_write(private_ts->client, 0x83, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	msleep(120);

	i2c_himax_write(private_ts->client, 0x81, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);

	mdelay(self_test_delay_time * 1000);

	/*Step 7 : Sensor Off*/
	i2c_himax_write(private_ts->client, 0x82, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);

	msleep(30);

	/*Step 8 : Get self_test result*/
	cmdbuf[0] = 0x15;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	msleep(10);

	cmdbuf[0] = 0x00;
	cmdbuf[1] = 0x96;
	i2c_himax_write(private_ts->client, 0x8B, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
	msleep(10);

	i2c_himax_read(private_ts->client, 0x5A, valuebuf, 16, DEFAULT_RETRY_CNT);
	msleep(10);

	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0x8C, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);

	/*Final : Leave self_test mode*/
	cmdbuf[0] = 0x00;
	i2c_himax_write(private_ts->client, 0xF1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);

	if (valuebuf[0] == 0xAA) {
		pr_info("[Himax]: self-test pass\n");
		pf_value = 0x0;
		for (i = 0; i < 7; i++)
			pr_info("[Himax]:0xFE96 buff[%d] = 0x%x\n", i, valuebuf[i]);
	} else {
		pr_info("[Himax]: self-test fail\n");
		pf_value = 0x1;
		for (i = 0; i < 15; i++)
			pr_info("[Himax]:0xFE96 buff[%d] = 0x%x\n", i, valuebuf[i]);
	}
	/*HW reset and power on again.*/
#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif
	himax_ts_poweron(private_ts);
	return pf_value;
}

/*
* time : t:x12 (12s)
* fe96 : w:x96:x03:x02......
*/
static ssize_t himax_self_test_setting(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;
	static uint8_t himax_command;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	if (buf[0] == 't' && buf[1] == ':' && buf[2] == 'x') {
		if (buf[3] > 47 && buf[3] < 58 && buf[4] > 47 && buf[4] < 58) {
			self_test_delay_time = (buf[3] - 48) * 10 + buf[4] - 48;
			pr_info("self_test_delay_time: %d\n", self_test_delay_time);
		}
		return count;
	}

	if (buf[0] == 'w' && buf[1] == ':') {
		if (buf[2] == 'x') {
			uint8_t loop_i;
			uint16_t base = 5;
			memcpy(buf_tmp, buf + 3, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
				himax_command = result;
			for (loop_i = 0; loop_i < 100; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w')
						printk("CMD: %x, %x, %d\n", himax_command, write_da[0], length);
					for (veriLen = 0; veriLen < length; veriLen++) {
						printk("%x ", *((&write_da[0])+veriLen));
						rFE96_setting[veriLen] = *((&write_da[0])+veriLen);
						printk("rFE96_setting[%d] : %x\n", veriLen , rFE96_setting[veriLen]);
					}
					printk("\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul(buf_tmp, 16, &result))
						write_da[loop_i] = result;
					length++;
				}
				base += 4;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(tp_self_test, (S_IWUSR|S_IRUGO|S_IWGRP),
				himax_self_test_read, himax_self_test_setting);
#ifdef HX_TP_SYS_RESET
static ssize_t himax_reset_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
	return count;
}
static DEVICE_ATTR(reset, (S_IWUSR|S_IRUGO|S_IWGRP),
				   NULL, himax_reset_write);
#endif

static ssize_t himax_get_touch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t buf0[3];
	int ret = -1;
	int test_count = 5;
	int i;

	buf0[0] = 0xE9;
	buf0[1] = 0x01;
	buf0[2] = 0x01;

	for (i = 0; i < test_count; i++) {
		ret = i2c_himax_master_write(private_ts->client, buf0, 3, DEFAULT_RETRY_CNT);
		if (ret < 0)
			break;
		msleep(50);
	}

	return sprintf(buf, "%d\n", (ret < 0) ? 0 : 1);
}
static DEVICE_ATTR(touch_status, (S_IRUGO), himax_get_touch_status, NULL);

static ssize_t himax_touch_switch_name(struct switch_dev *sdev, char *buf)
{
	int count = 0;
	himax_read_FW_ver();
	count += sprintf(buf + count, "Himax:MAJ");
	count += sprintf(buf + count, "-0x%2.2X", FW_VER_MAJ_buff[0]);
	count += sprintf(buf + count, ":MIN");
	count += sprintf(buf + count, "-0x%2.2X", FW_VER_MIN_buff[0]);
	count += sprintf(buf + count, ":CFG");
	count += sprintf(buf + count, "-0x%2.2X", CFG_VER_MIN_buff[CFG_VER_MIN_FLASH_LENG-1]);
	count += sprintf(buf + count, ":TP");
	count += sprintf(buf + count, "-0x%2.2X\n", CFG_VER_MAJ_buff[CFG_VER_MAJ_FLASH_LENG-1]);
	return count;
}

static int himax_touch_proc_init(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create(HIMAX_PROC_DIAG_FILE, 0664, NULL, &himax_diag_ops);
	if (!entry) {
		pr_info("[Himax] %s: proc diag file create failed!\n", __func__);
		return -EINVAL;
	}
	return 0;
}
/*
* ==============================================================================
*	Segment : Himax Touch Work Function
* ==============================================================================
*/
static struct attribute *himax_attr[] = {
	&dev_attr_register.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_tp_self_test.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_flash_dump.attr,
	&dev_attr_reset.attr,
	NULL
};

static int himax_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		pr_info("[TP]TOUCH_ERR: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}

#ifdef HX_TP_SYS_DEBUG_LEVEL
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		pr_info("[TP]TOUCH_ERR: create_file debug_level failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_REGISTER
	register_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		pr_info("[TP]TOUCH_ERR: create_file register failed\n");
		return ret;
	}
#endif
	/*
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		pr_info("[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	*/

#ifdef HX_TP_SYS_SELF_TEST
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
	if (ret) {
		pr_info("[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_self_test failed\n");
		return ret;
	}
#endif

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr);
	if (ret) {
		pr_info("[Himax]TOUCH_ERR: sysfs_create_file dev_attr_touch_status failed\n");
		return ret;
	}

#ifdef HX_TP_SYS_FLASH_DUMP
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr);
	if (ret) {
		pr_info("[TP]TOUCH_ERR: sysfs_create_file dev_attr_flash_dump failed\n");
		return ret;
	}
#endif

#ifdef HX_TP_SYS_RESET
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
	if (ret) {
		pr_info("[TP]TOUCH_ERR: sysfs_create_file dev_attr_reset failed\n");
		return ret;
	}
#endif
	return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);

#ifdef HX_TP_SYS_FLASH_DUMP
	sysfs_remove_file(android_touch_kobj, &dev_attr_flash_dump.attr);
#endif
#ifdef HX_TP_SYS_RESET
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
#endif

	sysfs_remove_file(android_touch_kobj, &dev_attr_touch_status.attr);

	kobject_del(android_touch_kobj);
}
#endif

static void himax_ts_work_func(struct work_struct *work)
{
	int ret, i, temp1, temp2;
	unsigned int x = 0, y = 0, area = 0, press = 0;
	const unsigned int x_res = HX_X_RES;
	const unsigned int y_res = HX_Y_RES;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
	unsigned char check_sum_cal = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[128] = {0};
	int RawDataLen = 0;
	unsigned int temp_x[HX_MAX_PT], temp_y[HX_MAX_PT];

#ifdef HX_TP_PROC_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int mul_num;
	int self_num;
	int index = 0;

	/*coordinate dump start*/
	char coordinate_char[15+(HX_MAX_PT+5)*2*5+2];
	struct timeval t;
	struct tm broken;
	/*coordinate dump end*/
#endif

	/*Calculate the raw data length*/
	/*Bizzy added for common RawData*/
	int raw_cnt_max = HX_MAX_PT/4;
	int raw_cnt_rmd = HX_MAX_PT%4;
	int hx_touch_info_size;
	/*more than 4 fingers*/
	if (raw_cnt_rmd != 0x00) {
		RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+3)*4) - 1;
		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+2)*4;
	} else { /*less than 4 fingers*/
		RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+2)*4) - 1;
		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+1)*4;
	}

#ifdef ENABLE_CHIP_STATUS_MONITOR
	ts->running_status = 1;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif

	start_reg = HX_CMD_RAE;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;

#ifdef HX_TP_PROC_DIAG
	/*count the i2c read length*/
	if (diag_command)
#else
	if (false)
#endif
	{
#ifdef HX_TP_PROC_DIAG
		msg[1].len = 128; /*hx_touch_info_size + RawDataLen + 4 + 1; 4: RawData Headear*/
#else
		msg[1].len = hx_touch_info_size;
#endif
	} else {
#ifdef HX_ESD_WORKAROUND
		/*Bizzy modify for E version*/
		if (ESD_RESET_ACTIVATE) {
			msg[1].len = 128;
			pr_info("[HIMAX]:ESD_RESET_ACTIVATE = %d, 0x86 128 bytes.\n", ESD_RESET_ACTIVATE);
		} else
#endif
			msg[1].len = hx_touch_info_size;
	}
	msg[1].buf = buf;

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s Touch Controller Trigger ISR enter.\n", __func__);
#endif

	mutex_lock(&ts->mutex_lock);

	/*read 0x86 all event*/
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		pr_info("[HIMAX TP ERROR]:%s:i2c_transfer fail.\n", __func__);
		memset(buf, 0xff , 128);

#ifdef ENABLE_CHIP_RESET_MACHINE
		mutex_unlock(&ts->mutex_lock);
		enable_irq(ts->client->irq);
		goto work_func_send_i2c_msg_fail;
#endif
	}

#ifdef HX_ESD_WORKAROUND
	if (ESD_RESET_ACTIVATE)
		pr_info("[HIMAX]:ESD_RESET_ACTIVATE=%d, Resd %d bytes.\n", ESD_RESET_ACTIVATE, msg[1].len);
#endif
	/*----[HX_ESD_WORKAROUND]----------------------------------------start*/
#ifdef HX_ESD_WORKAROUND
	for (i = 0; i < hx_touch_info_size; i++) {
		if (buf[i] == 0x00) {
			check_sum_cal = 1;
		} else if (buf[i] == 0xED) {
			check_sum_cal = 2;
		} else {
			check_sum_cal = 0;
			i = hx_touch_info_size;
		}
	}

	/*IC status is abnormal ,do hand shaking*/
	/*----[HX_TP_PROC_DIAG]------------------------------------------start*/
#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
	/*ESD Check*/
	if ((check_sum_cal != 0 || TOUCH_UP_COUNTER > 10) && ESD_RESET_ACTIVATE == 0 && diag_cmd == 0)
#else
	if (check_sum_cal != 0 && diag_cmd == 0)
#endif
#else
#ifdef HX_ESD_WORKAROUND
	/*ESD Check*/
	if ((check_sum_cal != 0 || TOUCH_UP_COUNTER > 10) && ESD_RESET_ACTIVATE == 0)
#else
	if (check_sum_cal != 0)
#endif
#endif
	/*----[HX_TP_PROC_DIAG]--------------------------------------end*/
	{
		mutex_unlock(&ts->mutex_lock);

#ifdef HX_ESD_WORKAROUND_HANDSHAKING
		ret = himax_hang_shaking(); /* 0:Running, 1:Stop, 2:I2C Fail */
#else
		ret = 1; /* return STOP */
#endif
		enable_irq(ts->client->irq);

		if (ret == 2)
			goto work_func_send_i2c_msg_fail;

		if ((ret == 1) && (check_sum_cal == 1)) {
			pr_info("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
		} else if (check_sum_cal == 2) {
			pr_info("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
			ESD_HW_REST();
		} else if (TOUCH_UP_COUNTER > 10) {
			pr_info("[HIMAX TP MSG]: TOUCH UP COUNTER > 10.\n");
			ESD_HW_REST();
		}

		if (TOUCH_UP_COUNTER > 10)
			TOUCH_UP_COUNTER = 0;

#ifdef ENABLE_CHIP_STATUS_MONITOR
		ts->running_status = 0;
		queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
		return;
	} else if (ESD_RESET_ACTIVATE) {
		ESD_RESET_ACTIVATE--;
		pr_info("[HIMAX TP MSG]:%s: Back from ESD reset, ready to serve., ESD_RESET_ACTIVATE=%d\n", __func__, ESD_RESET_ACTIVATE);
		mutex_unlock(&ts->mutex_lock);
		enable_irq(ts->client->irq);

#ifdef ENABLE_CHIP_STATUS_MONITOR
		ts->running_status = 0;
		queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
		return;
	}
#endif
	/*----[HX_ESD_WORKAROUND]------------------------------------------end*/

	/* calculate the checksum */
	for (i = 0; i < hx_touch_info_size; i++)
		check_sum_cal += buf[i];

	/* check sum fail */
	if ((check_sum_cal != 0x00) || (buf[HX_TOUCH_INFO_POINT_CNT] & 0xF0) != 0xF0) {
		pr_info("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);

		mutex_unlock(&ts->mutex_lock);

		enable_irq(ts->client->irq);

#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER++;
		pr_info("[HIMAX TP MSG]: ESD event checked - check_sum_cal, ESD_COUNTER = %d.\n", ESD_COUNTER);
		if (ESD_COUNTER > ESD_COUNTER_SETTING)
			ESD_HW_REST();
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
		ts->running_status = 0;
		queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif

		return;
	}

#ifdef HX_TP_SYS_DEBUG_LEVEL
	if (getDebugLevel() & 0x1) {
		pr_info("[HIMAX TP MSG]%s: raw data:\n", __func__);
		for (i = 0; i < 128; i = i+8)
			pr_info("%d: 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X\n", i, buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
	}
#endif

#ifdef HX_TP_PROC_DIAG
	/*touch monitor raw data fetch */
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6) {
		/* Check 128th byte CRC */
		for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++)
			check_sum_cal += buf[i];

		if (check_sum_cal % 0x100 != 0)
			goto bypass_checksum_failed_packet;

		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();

		/* initiallize the block number of mutual and self */
		mul_num = getXChannel() * getYChannel();
		self_num = getXChannel() + getYChannel();

		/* Himax: Check Raw-Data Header */
		if (buf[hx_touch_info_size] == buf[hx_touch_info_size+1] && buf[hx_touch_info_size+1] == buf[hx_touch_info_size+2]
				&& buf[hx_touch_info_size+2] == buf[hx_touch_info_size+3] && buf[hx_touch_info_size] > 0) {
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;
			pr_debug("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
			for (i = 0; i < RawDataLen; i++) {
				temp1 = index + i;

				if (temp1 < mul_num) {
					/* mutual */
					mutual_data[index + i] = buf[i + hx_touch_info_size+4]; /* 4: RawData Header */
				} else {
					/*self */
					temp1 = i + index;
					temp2 = self_num + mul_num;

					if (temp1 >= temp2)
						break;

					self_data[i+index-mul_num] = buf[i + hx_touch_info_size+4]; /* 4: RawData Header */
				}
			}
		} else {
			pr_info("[HIMAX TP MSG]%s: header format is wrong!\n", __func__);
		}
	} else if (diag_cmd == 7) {
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	/* coordinate dump start */
	if (coordinate_dump_enable == 1) {
		for (i = 0; i < (15 + (HX_MAX_PT+5)*2*5); i++)
			coordinate_char[i] = 0x20;

		coordinate_char[15 + (HX_MAX_PT+5)*2*5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT+5)*2*5 + 1] = 0xA;
	}
	/* coordinate dump end */
#endif

bypass_checksum_failed_packet:

	tpd_key = 0xFF;
	p_point_num = hx_point_num;

	if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
		hx_point_num = 0;
	else
		hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

	/* Touch Point information */
	if (hx_point_num != 0 && tpd_key == 0xFF) {
		/* parse the point information */
		for (i = 0; i < HX_MAX_PT; i++) {
			if (buf[4*i] != 0xFF) {
				/* x and y axis */
				x = buf[4 * i + 1] | (buf[4 * i] << 8) ;
				y = buf[4 * i + 3] | (buf[4 * i + 2] << 8);

				temp_x[i] = x;
				temp_y[i] = y;

				if ((x <= x_res) && (y <= y_res)) {
					/* caculate the pressure and area */
					press = buf[4*HX_MAX_PT+i];
					area = press;
					if (area > 31)
						area = (area >> 3);

					/* kernel call for report point area, pressure and x-y axis */
					input_report_key(ts->input_dev, BTN_TOUCH, 1);			/* touch down */
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);		/* ID of touched point */
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, area);	/* Finger Size */
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, press);	/* Pressure */
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);		/* X axis */
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);		/* Y axis */

					input_mt_sync(ts->input_dev);


					if (gPrint_point)
						pr_info("[HIMAX PORTING MSG]%s Touch DOWN x = %d, y = %d, area = %d, press = %d.\n", __func__, x, y, area, press);

					#ifdef HX_TP_PROC_DIAG
					if (coordinate_dump_enable == 1) {
						do_gettimeofday(&t);
						time_to_tm(t.tv_sec, 0, &broken);

						sprintf(&coordinate_char[0], "%2d:%2d:%2d:%3li,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);

						sprintf(&coordinate_char[15 + (i*2)*5], "%4d,", x);
						sprintf(&coordinate_char[15 + (i*2)*5 + 5], "%4d,", y);

						coordinate_fn->f_op->write(coordinate_fn, &coordinate_char[0], 15+(HX_MAX_PT+5)*2*sizeof(char)*5+2, &coordinate_fn->f_pos);
					}
					#endif
				}
			} else {
				temp_x[i] = 0xFFFF;
				temp_y[i] = 0xFFFF;
				input_mt_sync(ts->input_dev);
			}
		}
		input_sync(ts->input_dev);

#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
		TOUCH_UP_COUNTER = 0;
#endif
	} else if (hx_point_num == 0 && tpd_key != 0xFF) {
		temp_x[0] = 0xFFFF;
		temp_y[0] = 0xFFFF;
		temp_x[1] = 0xFFFF;
		temp_y[1] = 0xFFFF;

		if (tpd_key == 1) {
			input_report_key(ts->input_dev, tpd_keys_local[0], 1);
			input_sync(ts->input_dev);
			pr_debug("Press BT1***\r\n");
		}
		if (tpd_key == 2) {
			input_report_key(ts->input_dev, tpd_keys_local[1], 1);
			input_sync(ts->input_dev);
			pr_debug("Press BT2***\r\n");
		}
		if (tpd_key == 3) {
			input_report_key(ts->input_dev, tpd_keys_local[2], 1);
			input_sync(ts->input_dev);
			pr_debug("Press BT3***\r\n");
		}
		if (tpd_key == 4) {
			input_report_key(ts->input_dev, tpd_keys_local[3], 1);
			input_sync(ts->input_dev);
			pr_debug("Press BT4***\r\n");
		}

#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
		TOUCH_UP_COUNTER = 0;
#endif
	} else if (hx_point_num == 0 && tpd_key == 0xFF) {
		temp_x[0] = 0xFFFF;
		temp_y[0] = 0xFFFF;
		temp_x[1] = 0xFFFF;
		temp_y[1] = 0xFFFF;

		if (tpd_key_old != 0xFF) {
			input_report_key(ts->input_dev, tpd_keys_local[tpd_key_old-1], 0);
			input_sync(ts->input_dev);
		} else {
#ifdef HX_ESD_WORKAROUND
			TOUCH_UP_COUNTER++;
#endif

			/* leave event */
			input_report_key(ts->input_dev, BTN_TOUCH, 0);  /* touch up */
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);

#ifdef HX_PORTING_DEB_MSG
			pr_info("[HIMAX PORTING MSG]%s Touch UP.\n", __func__);
#endif

#ifdef HX_TP_PROC_DIAG
			/*coordinate dump start*/
			if (coordinate_dump_enable == 1) {
				do_gettimeofday(&t);
				time_to_tm(t.tv_sec, 0, &broken);

				sprintf(&coordinate_char[0], "%2d:%2d:%2d:%lu,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);
				sprintf(&coordinate_char[15], "Touch up!");
				coordinate_fn->f_op->write(coordinate_fn, &coordinate_char[0], 15+(HX_MAX_PT+5)*2*sizeof(char)*5+2, &coordinate_fn->f_pos);
			}
			/*coordinate dump end*/
#endif
		}
#ifdef HX_ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
	}
	tpd_key_old = tpd_key;

	mutex_unlock(&ts->mutex_lock);

	enable_irq(ts->client->irq);

#ifdef ENABLE_CHIP_STATUS_MONITOR
	ts->running_status = 0;
	queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif

	return;

work_func_send_i2c_msg_fail:

	pr_info("[HIMAX TP ERROR]:work_func_send_i2c_msg_fail: %d\n", __LINE__);

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
		queue_delayed_work(ts->himax_wq, &ts->himax_chip_reset_work, 0);
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
	ts->running_status = 0;
	queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
#endif
	return;
}

/*
* ==============================================================================
*	Segment : Himax Linux Driver Probe Function
* ==============================================================================
*/

/*----[ interrupt ]------------------------------------------------------start*/
static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->himax_wq, &ts->work);

	return IRQ_HANDLED;
}

static int himax_ts_register_interrupt(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	if (HX_INT_IS_EDGE)
		err = request_irq(client->irq, himax_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);
	else
		err = request_irq(client->irq, himax_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);

	if (err)
		dev_err(&client->dev, "[himax] %s: request_irq %d failed\n", __func__, client->irq);
	else
		pr_info("[himax]%s request_irq ok\r\n", __func__);
	return err;
}
/*----[ interrupt ]--------------------------------------------------------end*/

int hx8529_open(struct inode *inode, struct file *filp)
{
	pr_info("[himax] %s\n", __func__);
	return 0;
}

int hx8529_release(struct inode *inode, struct file *filp)
{
	pr_info("[himax] %s\n", __func__);
	return 0;
}

long hx8529_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	if (_IOC_TYPE(cmd) != HX8529_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HX8529_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case HX8529_POLL_DATA:
		if (arg == HX8529_IOCTL_START_HEAVY) {
			pr_info("[himax] ioctl heavy\n");
			poll_mode = START_HEAVY;
			queue_delayed_work(touch_work_queue, &hx8529_poll_data_work, poll_mode);
		} else if (arg == HX8529_IOCTL_START_NORMAL) {
			pr_info("[himax] ioctl normal\n");
			queue_delayed_work(touch_work_queue, &hx8529_poll_data_work, poll_mode);
		} else if (arg == HX8529_IOCTL_END) {
			pr_info("[himax] ioctl end\n");
			cancel_delayed_work_sync(&hx8529_poll_data_work);
		} else
			return -ENOTTY;
		break;
	default: /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}

	return 0;
}

struct file_operations hx8529_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hx8529_ioctl,
	.open = hx8529_open,
	.release = hx8529_release,
};

static void hx8529_poll_data(struct work_struct *work)
{

	uint8_t buf0[3];
	int ret = -1;
	int test_count = 5;
	int i;

	buf0[0] = 0xE9;
	buf0[1] = 0x01;
	buf0[2] = 0x01;

	for (i = 0; i < test_count; i++) {
		ret = i2c_himax_master_write(private_ts->client, buf0, 3, DEFAULT_RETRY_CNT);
		if (ret < 0)
			break;
		msleep(50);
	}

	pr_info("[himax] hx8529_poll_data\n");

	if (poll_mode == 0)
		msleep(5);

	queue_delayed_work(touch_work_queue, &hx8529_poll_data_work, poll_mode);
}

static int himax_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s enter\n", __func__);
#endif

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S1 : point number position = %d\n", __func__, HX_TOUCH_INFO_POINT_CNT);
#endif

	/* Allocate the himax_ts_data */
	private_ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (private_ts == NULL) {
		pr_info("[HIMAX TP ERROR] %s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	client->addr = HIMAX_I2C_ADDR;
	private_ts->intr_gpio = HIMAX_INT_GPIO;
	private_ts->rst_gpio = HIMAX_RST_GPIO;

	client->irq = gpio_to_irq(private_ts->intr_gpio);
	private_ts->client = client;
	private_ts->init_success = 0;
	i2c_set_clientdata(client, private_ts);
	touch_i2c = client; /*global variable*/

#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 2;
#endif

	/* interrupt gpio */
	if (gpio_request(private_ts->intr_gpio, "himax-irq") != 0) {
		pr_info("[HIMAX PORTING ERROR] interrupt gpio %d request fail.\n", private_ts->intr_gpio);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}
	gpio_direction_input(private_ts->intr_gpio);

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S2 : Interrupt GPIO ok.\n", __func__);
#endif

	/* reset gpio */
	if (gpio_request(private_ts->rst_gpio, "himax-reset") != 0) {
		pr_info("[HIMAX PORTING ERROR] reset gpio %d request fail.\n", private_ts->rst_gpio);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}
	gpio_direction_output(private_ts->rst_gpio, 0);
	msleep(100);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(100);

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S3 : Reset GPIO ok.\n", __func__);
#endif

	/* check i2c capability */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("[HIMAX TP ERROR] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S4 : i2c functionality check pass.\n", __func__);
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	private_ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
	if (!private_ts->flash_wq) {
		pr_info("[HIMAX TP ERROR] %s: create flash workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}
#endif

	/*---Create himax work queue---*/
	private_ts->himax_wq = create_singlethread_workqueue("himax_wq");
	if (!private_ts->himax_wq) {
		pr_info("[HIMAX TP ERROR] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S5 :himax_wq create complete.\n", __func__);
#endif

	/*Init the work queue function*/
#ifdef ENABLE_CHIP_RESET_MACHINE
	INIT_DELAYED_WORK(&private_ts->himax_chip_reset_work, himax_chip_reset_function);
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
	INIT_DELAYED_WORK(&private_ts->himax_chip_monitor, himax_chip_monitor_function); /*for ESD solution*/
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	INIT_WORK(&private_ts->flash_work, himax_ts_flash_work_func);
#endif

	INIT_WORK(&private_ts->work, himax_ts_work_func);

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S7 : INIT_WORK ok.\n", __func__);
#endif

	/*setup the i2c client data*/
	private_ts->client = client;
	i2c_set_clientdata(client, private_ts);

	touch_i2c = client;
	himax_ic_package_check(private_ts);

#ifdef HX_FW_UPDATE_BY_I_FILE
	if (i_Needupdate)
		i_update_func();
#endif

	himax_touch_information(client);
	calculate_point_number();
	private_ts->init_success = 0;

#ifdef ENABLE_CHIP_RESET_MACHINE
	private_ts->retry_time = 0;
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
	private_ts->running_status = 0;
#endif

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S8 : interrupt/reset gpio/irq OK.\n", __func__);
#endif

	mutex_init(&private_ts->mutex_lock);

	wake_lock_init(&private_ts->wake_lock, WAKE_LOCK_SUSPEND, "himax_touch_wake_lock");

	/*allocate the input device*/
	private_ts->input_dev = input_allocate_device();

	if (private_ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "[HIMAX TP ERROR] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	private_ts->input_dev->name = INPUT_DEV_NAME;
	private_ts->abs_x_max = HX_X_RES;
	private_ts->abs_y_max = HX_Y_RES;

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S9 :x_max = %d, y_max = %d\n", __func__, private_ts->abs_x_max, private_ts->abs_y_max);
#endif

	__set_bit(EV_KEY, private_ts->input_dev->evbit);
	__set_bit(EV_ABS, private_ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, private_ts->input_dev->keybit);

	input_set_abs_params(private_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(private_ts->input_dev, ABS_MT_POSITION_X, 0, HX_X_RES, 0, 0);
	input_set_abs_params(private_ts->input_dev, ABS_MT_POSITION_Y, 0, HX_Y_RES, 0, 0);
	input_set_abs_params(private_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0); /*Finger Size*/
	input_set_abs_params(private_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 31, 0, 0); /*Touch Size*/
	input_set_abs_params(private_ts->input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);

	err = input_register_device(private_ts->input_dev);
	if (err) {
		dev_err(&client->dev, "[HIMAX TP ERROR]%s: unable to register %s input device\n", __func__, private_ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S10 : Input Device Reigster ok.\n", __func__);
#endif

#ifdef HX_RST_PIN_FUNC
	err = gpio_direction_output(private_ts->rst_gpio, 1);
	if (err)
		pr_info("Failed to set reset direction, error=%d\n", err);

#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 2;
#endif

	gpio_set_value(private_ts->rst_gpio, 0);
	msleep(100);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(100);
#endif

#ifdef HX_PORTING_DEB_MSG
	/* default is close. Only for test i2c. */
	/* himax_i2c_test_function(ts); */
#endif

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S11 : Check IC_TYPE = %d.\n", __func__, IC_TYPE);
#endif

	himax_ts_poweron(private_ts);

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S12 : Power on complete.\n", __func__);
#endif

#ifdef HX_TP_PROC_DIAG
	setXChannel(HX_RX_NUM); /* X channel */
	setYChannel(HX_TX_NUM); /* Y channel */
#endif

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s X_Channel = %d, Y_Channel = %d\n", __func__, HX_RX_NUM, HX_TX_NUM);
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	setSysOperation(0);
	setFlashBuffer();
#endif

#ifdef HX_TP_PROC_DIAG
	setMutualBuffer();
	if (getMutualBuffer() == NULL) {
		pr_info("[HIMAX TP ERROR] %s: mutual buffer allocate fail failed\n", __func__);
		return -1;
	}
#endif

	/*TODO START : check the interrupt is level or edge trigger*/
	himax_ts_register_interrupt(private_ts->client);
	/*TODO END*/

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S13 :Interrupt Request ok.\n", __func__);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	private_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	private_ts->early_suspend.suspend = himax_ts_early_suspend;
	private_ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&private_ts->early_suspend);
#endif

	/* Register Switch file */
	private_ts->touch_sdev.name = "touch";
	private_ts->touch_sdev.print_name = himax_touch_switch_name;
	if (switch_dev_register(&private_ts->touch_sdev) < 0)
		dev_info(&client->dev, "switch_dev_register for dock failed!\n");
	switch_set_state(&private_ts->touch_sdev, 0);

#ifdef HX_TP_SYS_FS
	#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S14 : Proc Initial Start.\n", __func__);
	#endif
	himax_touch_proc_init();

	#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S15 : Sysfs Initial Start.\n", __func__);
	#endif
	himax_touch_sysfs_init();
	/* register sysfs node for debug APK such as raw-count and fw-upgrade */
	private_ts->attrs.attrs = himax_attr;
	err = sysfs_create_group(&private_ts->client->dev.kobj, &private_ts->attrs);
	if (err)
		dev_err(&client->dev, "[TP] %s: Not able to create the sysfs\n", __func__);
	#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s S16 : Sysfs Initial OK.\n", __func__);
	#endif
#endif

	private_ts->init_success = 1;

#ifdef ENABLE_CHIP_RESET_MACHINE
	private_ts->retry_time = 0;
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
	/*for ESD solution*/
	queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 60*HZ);
#endif

	dev_info(&client->dev, "[HIMAX MSG] Start touchscreen %s in interrupt mode\n", private_ts->input_dev->name);

	/* init for i2c stress test */
	touch_work_queue = create_singlethread_workqueue("i2c_touch_wq");
	if (!touch_work_queue)
		pr_info("hx8529_probe: Unable to create workqueue\n");

	INIT_DELAYED_WORK(&hx8529_poll_data_work, hx8529_poll_data);
	hx8529_client = client;
	private_ts->misc_dev.minor = MISC_DYNAMIC_MINOR;
	private_ts->misc_dev.name = "touch";
	private_ts->misc_dev.fops = &hx8529_fops;
	err = misc_register(&private_ts->misc_dev);
	if (err)
		pr_info("touch err : Unable to register %s misc device\n", private_ts->misc_dev.name);

#ifdef HX_PORTING_DEB_MSG
	pr_info("[HIMAX PORTING MSG]%s complete.\n", __func__);
#endif
	himax_cable_status(cable_status);
	return 0;

err_input_register_device_failed:
	if (private_ts->input_dev)
		input_free_device(private_ts->input_dev);

err_input_dev_alloc_failed:
	mutex_destroy(&private_ts->mutex_lock);
	wake_lock_destroy(&private_ts->wake_lock);

#ifdef ENABLE_CHIP_RESET_MACHINE
	cancel_delayed_work(&private_ts->himax_chip_reset_work);
#endif

#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work(&private_ts->himax_chip_monitor);
#endif

	if (private_ts->himax_wq)
		destroy_workqueue(private_ts->himax_wq);

err_create_wq_failed:
	kfree(private_ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int himax_ts_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

#ifdef HX_TP_SYS_FS
	himax_touch_sysfs_deinit();
#endif

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	mutex_destroy(&ts->mutex_lock);

	if (ts->himax_wq)
		destroy_workqueue(ts->himax_wq);

	input_unregister_device(ts->input_dev);

	wake_lock_destroy(&ts->wake_lock);

	kfree(ts);

	return 0;
}

static const struct i2c_device_id himax_ts_id[] = {
	{ HIMAX_TS_NAME, 0 },
	{ }
};

static struct i2c_driver himax_ts_driver = {
	.probe = himax_ts_probe,
	.remove = himax_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = himax_ts_suspend,
	.resume = himax_ts_resume,
#endif
	.id_table = himax_ts_id,
	.driver = {
		.name = HIMAX_TS_NAME,
		.owner = THIS_MODULE,
	},
};

static int himax_ts_init(void)
{
	if (asustek_get_tp_type() == TP_IC_TYPE_A) {
		return 0;
	} else {
		pr_info("[himax] %s\n", __func__);
		return i2c_add_driver(&himax_ts_driver);
	}
}

static void __exit himax_ts_exit(void)
{
	i2c_del_driver(&himax_ts_driver);
	return;
}

module_init(himax_ts_init);
module_exit(himax_ts_exit);

MODULE_DESCRIPTION("Himax Touchscreen Driver");
MODULE_LICENSE("GPL");
