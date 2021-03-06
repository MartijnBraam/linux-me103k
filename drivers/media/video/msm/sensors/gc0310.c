/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm.h"
#include "msm_ispif.h"
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/proc_fs.h>
#include <asm/mach-types.h>
#include "gc0310.h"


//#define KEEP_CSI_PARMS

int machine_is_apq8064_flo(void);
int machine_is_apq8064_deb(void);
#ifdef CONFIG_PROC_FS
void create_gc0310_proc_file(void);
#endif

#define SENSOR_NAME "gc0310"
#define PLATFORM_DRIVER_NAME "msm_camera_gc0310"
#define gc0310_obj gc0310_##obj
/* Macros assume PMIC GPIOs and MPPs start at 1 */
#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)
#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */
#define CAM_RST PM8921_GPIO_PM_TO_SYS(43)
#define CAM_RST_DUMA PM8921_GPIO_PM_TO_SYS(12)
#define CAM_PWDN PM8921_GPIO_PM_TO_SYS(10)
#define CAM_VENDOR_1 PM8921_GPIO_PM_TO_SYS(11)
#define CAM_VENDOR_0 PM8921_GPIO_PM_TO_SYS(10)
#define CAM_LENS PM8921_GPIO_PM_TO_SYS(24)

static struct msm_sensor_ctrl_t gc0310_s_ctrl;

DEFINE_MUTEX(gc0310_mut);

static struct msm_camera_i2c_conf_array gc0310_raw_init_conf[] = {
	{gc0310_recommend_settings,
	ARRAY_SIZE(gc0310_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array gc0310_confs[] = {
	{gc0310_vga_settings,
	ARRAY_SIZE(gc0310_vga_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct v4l2_subdev_info gc0310_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,//workaround to get more support format
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static struct msm_sensor_output_info_t gc0310_dimensions[] = {
	{
		.x_output = 640,
		.y_output = 480,
		.line_length_pclk = 800,
		.frame_length_lines = 596,
		.vt_pixel_clk = 15000000,
		.op_pixel_clk = 128000000,
		.binning_factor = 1,
	},
};
#ifdef KEEP_CSI_PARMS
static struct msm_camera_csid_vc_cfg gc0310_cid_cfg[] = {
	{0, CSI_RAW8, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params gc0310_csi_params = {
	.csid_params = {
		.lane_cnt = 1,
		.lut_params = {
			.num_cid = ARRAY_SIZE(gc0310_cid_cfg),
			.vc_cfg = gc0310_cid_cfg,
		},
	},
		.csiphy_params = {
		.lane_cnt = 1,
		.settle_cnt = 0x14,
	},
};

static struct msm_camera_csi2_params *gc0310_csi_params_array[] = {
	&gc0310_csi_params,
	&gc0310_csi_params,
};
#endif
static struct msm_sensor_output_reg_addr_t gc0310_reg_addr = {
};

static struct msm_sensor_id_info_t gc0310_id_info = {
	.sensor_id_reg_addr = 0xf0,
	.sensor_id = 0xa3,
};

static struct msm_sensor_exp_gain_info_t gc0310_exp_gain_info = {
	.coarse_int_time_addr = 0x03,
};

static int32_t gc0310_write_pict_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line, int32_t luma_avg, uint16_t fgain)
{
	u8 intg_time_hsb, intg_time_lsb;
	uint8_t a_gain, d_gain;

	CDBG(KERN_ERR "picture exposure setting 0x%x, 0x%x, %d",
		 gain, line, line);

	a_gain = (uint8_t) ((gain & 0x0700)>>8);
	d_gain = (uint8_t)(gain & 0x00FF);

	if (line < 1)
		line = 1;
	if (line > 4095)
		line = 4095;
	intg_time_hsb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line & 0x00FF);

	/* Coarse Integration Time */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
		intg_time_hsb,
		MSM_CAMERA_I2C_BYTE_DATA);

	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr + 1,
		intg_time_lsb,
		MSM_CAMERA_I2C_BYTE_DATA);

	/* analog gain */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x48,
		a_gain,
		MSM_CAMERA_I2C_BYTE_DATA);
	/* digital gain */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x71,
		d_gain,
		MSM_CAMERA_I2C_BYTE_DATA);

	return 0;
}

static int32_t gc0310_write_prev_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line, int32_t luma_avg, uint16_t fgain)
{
	u8 intg_time_hsb, intg_time_lsb;
	uint8_t a_gain, d_gain;

	CDBG(KERN_ERR "picture exposure setting 0x%x, 0x%x, %d",
		 gain, line, line);

	a_gain = (uint8_t) ((gain & 0x0700)>>8); //P0:0x48 [2:0] Analog gain
	d_gain = (uint8_t)(gain & 0x00FF);

	if (line < 1)
		line = 1;
	if (line > 4095)
		line = 4095;
	intg_time_hsb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line & 0x00FF);

	/* Coarse Integration Time */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
		intg_time_hsb,
		MSM_CAMERA_I2C_BYTE_DATA);

	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr + 1,
		intg_time_lsb,
		MSM_CAMERA_I2C_BYTE_DATA);

	/* analog gain */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x48,
		a_gain,
		MSM_CAMERA_I2C_BYTE_DATA);
	/* digital gain */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		0x71,
		d_gain,
		MSM_CAMERA_I2C_BYTE_DATA);

	return 0;
}
static struct pm_gpio pm_isp_gpio_high = {
	.direction		  = PM_GPIO_DIR_OUT,
	.output_buffer	  = PM_GPIO_OUT_BUF_CMOS,
	.output_value	  = 1,
	.pull			  = PM_GPIO_PULL_NO,
	.vin_sel		  = PM_GPIO_VIN_S4,
	.out_strength	  = PM_GPIO_STRENGTH_HIGH,
	.function		  = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	  = 0,
	.disable_pin	  = 0,
};

static struct pm_gpio pm_isp_gpio_low = {
	.direction		  = PM_GPIO_DIR_OUT,
	.output_buffer	  = PM_GPIO_OUT_BUF_CMOS,
	.output_value	  = 0,
	.pull			  = PM_GPIO_PULL_NO,
	.vin_sel		  = PM_GPIO_VIN_S4,
	.out_strength	  = PM_GPIO_STRENGTH_HIGH,
	.function		  = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	  = 0,
	.disable_pin	  = 0,
};

static int gc0310_regulator_init(bool on)
{
	static struct regulator *reg_8921_dvdd, *reg_8921_l8;
	int rc;

	pr_info("%s +++\n", __func__);

	if (on) {
		pr_info("Turn on the regulators\n");
		if (!reg_8921_dvdd) {
			reg_8921_dvdd = regulator_get(NULL, "8921_l23");
			if (!IS_ERR(reg_8921_dvdd)) {
				rc = regulator_set_voltage(reg_8921_dvdd,
					1800000, 1800000);
				if (rc) {
					pr_err("DVDD: reg_8921_dvdd regulator set_voltage failed\n");
					goto reg_put_dvdd;
				}
			}
			if (IS_ERR(reg_8921_dvdd)) {
				pr_info("PTR_ERR(DVDD)=%ld\n",
					PTR_ERR(reg_8921_dvdd));
				return -ENODEV;
			}
		}

		rc = regulator_enable(reg_8921_dvdd);
		if (rc) {
			pr_err("DVDD regulator enable failed(%d)\n", rc);
			goto reg_put_dvdd;
		}
		pr_info("DVDD enable(%d)\n",
			regulator_is_enabled(reg_8921_dvdd));

		if (!reg_8921_l8) {
			reg_8921_l8 = regulator_get(NULL, "8921_l8");
			if (IS_ERR(reg_8921_l8)) {
				pr_err("PTR_ERR(reg_8921_l8)=%ld\n",
					PTR_ERR(reg_8921_l8));
				goto reg_put_l8;
			}
		}
		rc = regulator_set_voltage(reg_8921_l8, 2800000, 2800000);
		if (rc) {
			pr_err("AVDD: reg_8921_l8 regulator set_voltage failed\n");
			goto reg_put_l8;
		}
		rc = regulator_enable(reg_8921_l8);
		if (rc) {
			pr_err("AVDD: reg_8921_l8 regulator enable failed\n");
			goto reg_put_l8;
		}
		pr_info("AVDD: reg_8921_l8(%d) enable(%d)\n",
			regulator_get_voltage(reg_8921_l8),
			regulator_is_enabled(reg_8921_l8));

		return 0;
	} else {
		pr_info("Turn off the regulators\n");

		if (reg_8921_l8) {
			pr_info("Turn off the regulators AVDD:reg_8921_l8\n");
			regulator_disable(reg_8921_l8);
			regulator_put(reg_8921_l8);
			reg_8921_l8 = NULL;
		}
		if (reg_8921_dvdd) {
			pr_info("Turn off the regulators DOVDD:reg_8921_dvdd\n");
			regulator_disable(reg_8921_dvdd);
			regulator_put(reg_8921_dvdd);
			reg_8921_dvdd = NULL;
		}
		return 0;
	}

reg_put_l8:
	regulator_put(reg_8921_l8);
	regulator_disable(reg_8921_dvdd);
	reg_8921_l8 = NULL;

reg_put_dvdd:
	regulator_put(reg_8921_dvdd);
	reg_8921_dvdd = NULL;

	return rc;
}

int32_t gc0310_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t rdata = 0;
	pr_info("%s +++\n", __func__);

	msm_camera_i2c_read(gc0310_s_ctrl.sensor_i2c_client, 0xf0,
		&rdata, MSM_CAMERA_I2C_BYTE_DATA);
	if (rdata == gc0310_id_info.sensor_id)
		rc = 0;
	else
		rc = -EFAULT;

	pr_info("Sensor id: 0x%x\n", rdata);

	pr_info("%s ---\n", __func__);
	return rc;
}

static int gc0310_gpio_request(void)
{
	int32_t rc = 0;

	pr_info("%s +++\n", __func__);

	rc = gpio_request(CAM_PWDN, "gc0310");
	if (rc) {
		pr_err("%s: gpio CAM_PWDN %d, rc(%d)fail\n", __func__, CAM_PWDN, rc);
		goto init_probe_fail;
	}

	pr_info("%s ---\n", __func__);
	return rc;

init_probe_fail:
	pr_info("%s fail---\n", __func__);
	return rc;
}

int32_t gc0310_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	pr_info("%s +++\n", __func__);

	if (!s_ctrl->sensordata || !gc0310_s_ctrl.sensordata) {
		pr_err("gc0310_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n", __func__);
		return -EFAULT;
	}

	rc = gc0310_gpio_request();
	if (rc < 0)	{
		pr_err("0.3M Camera GPIO request fail!!\n");
		pr_info("%s ---\n", __func__);
		return -EFAULT;
	}

	rc = pm8xxx_gpio_config(CAM_PWDN, &pm_isp_gpio_low);
	if (rc != 0)
		pr_err("PMIC gpio 10 CAM_PWDN config low fail\n");
	else
		pr_info("PMIC gpio 10 CAM_PWDN(%d)\n", gpio_get_value(CAM_PWDN));

	/* PMIC regulator - DOVDD 1.8V and AVDD 2.8V ON */
	rc = gc0310_regulator_init(true);
	if (rc < 0) {
		pr_err("0.3M Camera regulator init fail!!\n");
		pr_info("%s ---\n", __func__);
		return -EFAULT;
	}

	/* enable MCLK */
	msm_sensor_power_up(&gc0310_s_ctrl);

	usleep_range(10, 20);

	rc = pm8xxx_gpio_config(CAM_PWDN, &pm_isp_gpio_high);
	if (rc != 0)
		pr_err("PMIC gpio 10 CAM_PWDN config high fail\n");
	else
		pr_info("PMIC gpio 10 CAM_PWDN(%d)\n", gpio_get_value(CAM_PWDN));

	usleep_range(10, 20);
	rc = pm8xxx_gpio_config(CAM_PWDN, &pm_isp_gpio_low);
	if (rc != 0)
		pr_err("PMIC gpio 10 CAM_PWDN config high fail\n");
	else
		pr_info("PMIC gpio 10 CAM_PWDN(%d)\n", gpio_get_value(CAM_PWDN));

	usleep_range(10, 20);

	pr_info("%s ---\n", __func__);
	return 0;
}

int32_t gc0310_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	pr_info("%s +++\n", __func__);

	if (!s_ctrl->sensordata || !gc0310_s_ctrl.sensordata) {
		pr_info("gc0310_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n", __func__);
		return -EFAULT;
	}

	rc = pm8xxx_gpio_config(CAM_PWDN, &pm_isp_gpio_high);
	if (rc != 0)
		pr_err("%s: CAM_PWDN config high fail\n", __func__);

	/* disable MCLK */
	msm_sensor_power_down(&gc0310_s_ctrl);

	/* PMIC regulator - DOVDD 1.8V and AVDD 2.8V OFF */
	gc0310_regulator_init(false);

	usleep_range(4000, 5000);
	rc = pm8xxx_gpio_config(CAM_PWDN, &pm_isp_gpio_low);
	if (rc != 0)
		pr_err("%s: CAM_PWDN config low fail\n", __func__);

	gpio_free(CAM_PWDN);

	pr_info("%s ---\n", __func__);
	return 0;
}


int32_t gc0310_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	unsigned short rdata = 0;

	pr_info("%s +++\n", __func__);

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	s_ctrl->sensor_device_type = MSM_SENSOR_I2C_DEVICE;
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		rc = -EFAULT;
		pr_info("%s --- n", __func__);
		return rc;
	}

	if (client->dev.platform_data == NULL) {
		pr_err("%s: NULL sensor data\n", __func__);
		return -EFAULT;
	}

	s_ctrl->sensordata = client->dev.platform_data;
	gc0310_s_ctrl.sensordata = client->dev.platform_data;

	rc = s_ctrl->func_tbl->sensor_power_up(&gc0310_s_ctrl);
	if (rc < 0)
		goto probe_fail;

	msm_camera_i2c_read(gc0310_s_ctrl.sensor_i2c_client, 0xf0, &rdata, MSM_CAMERA_I2C_BYTE_DATA);
	pr_info("Sensor id: 0x%x\n", rdata);
	if (rdata != 0xa3)
		goto probe_fail;
	create_gc0310_proc_file();

	if (!s_ctrl->wait_num_frames)
		s_ctrl->wait_num_frames = 1 * Q10;
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	s_ctrl->sensor_v4l2_subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->sensor_v4l2_subdev.entity, 0, NULL, 0);
	s_ctrl->sensor_v4l2_subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->sensor_v4l2_subdev.entity.group_id = SENSOR_DEV;
	s_ctrl->sensor_v4l2_subdev.entity.name =
		s_ctrl->sensor_v4l2_subdev.name;

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	s_ctrl->sensor_v4l2_subdev.entity.revision =
		s_ctrl->sensor_v4l2_subdev.devnode->num;

	goto power_down;

probe_fail:
	pr_info("Sensor power on fail\n");

power_down:
	s_ctrl->func_tbl->sensor_power_down(&gc0310_s_ctrl);
	s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	pr_info("%s ---\n", __func__);
	return rc;
}


static const struct i2c_device_id gc0310_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&gc0310_s_ctrl},
	{ }
};

static struct i2c_driver gc0310_i2c_driver = {
	.id_table = gc0310_i2c_id,
	.probe  = gc0310_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc0310_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

#ifdef	CONFIG_PROC_FS
#define	GC0310_PROC_CAMERA_STATUS	"driver/vga_status"
static ssize_t gc0310_proc_read_vga_status(char *page,
	char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0, rc = 0;

	gc0310_s_ctrl.func_tbl->sensor_power_up(&gc0310_s_ctrl);
	rc = gc0310_s_ctrl.func_tbl->sensor_match_id(&gc0310_s_ctrl);
	gc0310_s_ctrl.func_tbl->sensor_power_down(&gc0310_s_ctrl);

	if (*eof == 0) {
		if (!rc)
			len += snprintf(page+len, 10, "1\n");
		else
			len += snprintf(page+len, 10, "0\n");
		*eof = 1;
		pr_info("%s: string:%s", __func__, (char *)page);
	}
	return len;
}
void create_gc0310_proc_file(void)
{
	if (create_proc_read_entry(GC0310_PROC_CAMERA_STATUS, 0666, NULL,
			gc0310_proc_read_vga_status, NULL) == NULL) {
		pr_err("[Camera]proc file create failed!\n");
	}
}

void remove_gc0310_proc_file(void)
{
	pr_info("gc0310_s_ctrl_proc_file\n");
	remove_proc_entry(GC0310_PROC_CAMERA_STATUS, &proc_root);
}
#endif

static int __init gc0310_sensor_init_module(void)
{
	//create_gc0310_proc_file();
	return  i2c_add_driver(&gc0310_i2c_driver);
}

static struct v4l2_subdev_core_ops gc0310_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops gc0310_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops gc0310_subdev_ops = {
	.core = &gc0310_subdev_core_ops,
	.video  = &gc0310_subdev_video_ops,
};


static struct msm_sensor_fn_t gc0310_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = gc0310_write_prev_exp_gain,
	.sensor_write_snapshot_exp_gain = gc0310_write_pict_exp_gain,
	.sensor_csi_setting = msm_sensor_setting,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_match_id = gc0310_sensor_match_id,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = gc0310_sensor_power_up,
	.sensor_power_down = gc0310_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t gc0310_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = gc0310_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(gc0310_start_settings),
	.stop_stream_conf = gc0310_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(gc0310_stop_settings),
	.group_hold_on_conf = gc0310_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(gc0310_groupon_settings),
	.group_hold_off_conf = gc0310_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(gc0310_groupoff_settings),
	.init_settings = &gc0310_raw_init_conf[0],
	.init_size = ARRAY_SIZE(gc0310_raw_init_conf),
	.mode_settings = &gc0310_confs[0],
	.output_settings = &gc0310_dimensions[0],
	.num_conf = ARRAY_SIZE(gc0310_confs),
};

static struct msm_sensor_ctrl_t gc0310_s_ctrl = {
	.msm_sensor_reg = &gc0310_regs,
	.sensor_i2c_client = &gc0310_sensor_i2c_client,
	.sensor_i2c_addr = 0x42,
	.sensor_output_reg_addr = &gc0310_reg_addr,
	.sensor_id_info = &gc0310_id_info,
	.sensor_exp_gain_info = &gc0310_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
#ifdef KEEP_CSI_PARMS
	.csi_params = &gc0310_csi_params_array[0],
#endif
	.msm_sensor_mutex = &gc0310_mut,
	.sensor_i2c_driver = &gc0310_i2c_driver,
	.sensor_v4l2_subdev_info = gc0310_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc0310_subdev_info),
	.sensor_v4l2_subdev_ops = &gc0310_subdev_ops,
	.func_tbl = &gc0310_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
	.min_delay = 180,
};


module_init(gc0310_sensor_init_module);
MODULE_DESCRIPTION("Galaxy Core 0.3MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

/*Create debugfs for gc0310 */
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
static unsigned int register_value = 0xffffffff;
static int i2c_set_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t i2c_camera(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len;
	int arg[2];
	int err;

	if (*ppos)
		return 0;

	len = (count > DBG_TXT_BUF_SIZE-1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;

	debugTxtBuf[len] = 0;

	sscanf(debugTxtBuf, "%x", &arg[0]);
	pr_info("1 is open_camera 0 is close_camera\n");
	pr_info("command is arg1=%x\n", arg[0]);

	*ppos = len;

	switch (arg[0]) {
	case 0:
			pr_info("gc0310 power_off\n");
			err = gc0310_sensor_power_down(&gc0310_s_ctrl);
			if (err)
				return -ENOMEM;
			break;
	case 1:
			pr_info("gc0310 power_on\n");
			err = gc0310_sensor_power_up(&gc0310_s_ctrl);
			if (err)
				return -ENOMEM;
			break;

	default:
			break;
	}

	return len;
}

static ssize_t i2c_camera_read(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	int len;
	u32 arg[1];
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];
	unsigned int val = 0;

	if (!gc0310_s_ctrl.sensor_i2c_client->client->adapter)
		return -ENODEV;

	if (*ppos)
		return 0;

	len = (count > DBG_TXT_BUF_SIZE-1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;

	debugTxtBuf[len] = 0;

	sscanf(debugTxtBuf, "%x", &arg[0]);
	pr_info("command is reg_addr=%x\n", arg[0]);

	msg[0].addr = 0x21;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (arg[0] & 0xff);

	msg[1].addr = 0x21;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 1;

	err = i2c_transfer(gc0310_s_ctrl.
		sensor_i2c_client->client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	val = data[1] & 0xff;

	register_value = val;
	pr_info("register value: 0x%x\n", val);

	*ppos = len;
	return len;    /* the end */
}

static ssize_t i2c_camera_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	int len;
	u32 arg[2];
	int err;
	struct i2c_msg msg;
	unsigned char data[2];
	int retry = 0;

	if (*ppos)
		return 0;

	len = (count > DBG_TXT_BUF_SIZE-1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;

	debugTxtBuf[len] = 0;

	sscanf(debugTxtBuf, "%x %x", &arg[0], &arg[1]);
	pr_info("command is reg_addr=%x value=%x\n",
		arg[0], arg[1]);

	if (!gc0310_s_ctrl.sensor_i2c_client->client->adapter) {
		pr_info("%s client->adapter is null", __func__);
		return -ENODEV;
	}

	data[0] = (u8) (arg[0] & 0xff);

	data[1] = (u8) (arg[1] & 0xff);
	msg.len = 2;

	msg.addr = 0x21;
	msg.flags = 0;
	msg.buf = data;

	do {
		err = i2c_transfer(gc0310_s_ctrl.
			sensor_i2c_client->client->adapter, &msg, 1);
		if (err == 1) {
			*ppos = len;
			return len;    /* the end */
		}
		retry++;
		pr_err("bayer_sensor : i2c transfer failed, retrying %x\n",
		arg[1]);
	} while (retry <= SENSOR_MAX_RETRIES);

	*ppos = len;
	return len;    /* the end */
}

static int i2c_read_value(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	int len = 0;
	char *bp = debugTxtBuf;

	if (*ppos)
		return 0;    /* the end */

	len = snprintf(bp, DBG_TXT_BUF_SIZE, "Register value is 0x%X\n",
		register_value);

	if (copy_to_user(buf, debugTxtBuf, len))
		return -EFAULT;

	*ppos += len;
	return len;
}

static const struct file_operations i2c_open_camera = {
	.open = i2c_set_open,
	.write = i2c_camera,
};

static const struct file_operations i2c_read_register = {
	.open = i2c_set_open,
	.write = i2c_camera_read,
};
static const struct file_operations i2c_write_register = {
	.open = i2c_set_open,
	.write = i2c_camera_write,
};
static const struct file_operations read_register_value = {
	.open = i2c_set_open,
	.read = i2c_read_value,
};

static int __init qualcomm_i2c_debuginit(void)
{
	struct dentry *dent = debugfs_create_dir("gc0310", NULL);

	(void) debugfs_create_file("i2c_open_camera", S_IRUGO | S_IWUSR,
			dent, NULL, &i2c_open_camera);
	(void) debugfs_create_file("i2c_read", S_IRUGO | S_IWUSR,
			dent, NULL, &i2c_read_register);
	(void) debugfs_create_file("i2c_write", S_IRUGO | S_IWUSR,
			dent, NULL, &i2c_write_register);
	(void) debugfs_create_file("read_register_value", S_IRUGO | S_IWUSR,
			dent, NULL, &read_register_value);


	return 0;
}

late_initcall(qualcomm_i2c_debuginit);

