/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/*
[SENSOR]
Sensor Model:   GC0310
Camera Module:
Lens Model:
Driver IC:
PV Size         = 640 x 480
Cap Size        = 640 x 480
Output Format   = RGGB
MCLK Speed      = 24M
PV Frame Rate   = 30fps
Cap Frame Rate  = 30fps
I2C Slave ID    = 0x21
I2C Mode        = 8Addr, 8Data
*/

#ifndef CAMSENSOR_GC0310
#define CAMSENSOR_GC0310

extern struct proc_dir_entry proc_root;

enum Vendor_ID {
	A,
	B,
};

static struct msm_camera_i2c_reg_conf gc0310_start_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x94},
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc0310_stop_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x84},
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc0310_groupon_settings[] = {
};

static struct msm_camera_i2c_reg_conf gc0310_groupoff_settings[] = {
};

static struct msm_camera_i2c_reg_conf gc0310_recommend_settings[] = {
/////////////////////////////////////////////////
/////////////////   system reg  /////////////////
/////////////////////////////////////////////////
{0xfe, 0xf0},
{0xfe, 0xf0},
{0xfe, 0x00},

{0xfc, 0x0e}, //4e
{0xfc, 0x0e}, //16//4e // [0]apwd [6]regf_clk_gate
{0xf2, 0x80}, //sync output
{0xf3, 0x00}, //1f//01 data output
{0xf7, 0x33}, //f9
{0xf8, 0x03}, //00
{0xf9, 0x8e}, //0f
{0xfa, 0x11},

/////////////////////////////////////////////////
///////////////////   MIPI   ////////////////////
/////////////////////////////////////////////////
{0xfe, 0x03},
{0x01, 0x03}, ///mipi 1lane
{0x02, 0x22},
{0x03, 0x94},
{0x04, 0x10}, // fifo_prog
{0x05, 0x00}, //fifo_prog
{0x06, 0x80}, //b0  //YUV ISP data
//{0x10, 0x94}, //94 // last bit  lane num
{0x10, 0x04}, //94 // last bit  lane num
{0x11, 0x2a},//1e //LDI set YUV422
{0x12, 0x80},//00 //04 //00 //04//00 //LWC[7:0]  //
{0x13, 0x02},//05 //05 //LWC[15:8]
{0x15, 0x12}, //DPHYY_MODE read_ready
{0x17, 0x01},
{0x21, 0x02},
{0x22, 0x02},
{0x23, 0x01},
{0x26, 0x02},
{0x29, 0x00},
{0x2a, 0x01},
{0x2b, 0x02},
{0x40, 0x08},
{0x41, 0x00},
{0x42, 0x00},
{0x43, 0x00},

{0xfe, 0x00},

/////////////////////////////////////////////////
/////////////////   CISCTL reg  /////////////////
/////////////////////////////////////////////////
{0x00, 0x2f}, //2f//0f//02//01
{0x01, 0x06}, //06
{0x02, 0x04},
{0x03, 0x02},//04
{0x04, 0x40},//58
{0x05, 0x01},
{0x06, 0x26}, //HB
{0x07, 0x00},
{0x08, 0x4e}, //VB
{0x09, 0x00}, //row start
{0x0a, 0x00}, //
{0x0b, 0x00}, //col start
{0x0c, 0x06},
{0x0d, 0x01}, //height
{0x0e, 0xe8}, //height
{0x0f, 0x02}, //width
{0x10, 0x88}, //height
{0x17, 0x14},
{0x18, 0x1a}, //0a//[4]double reset
{0x19, 0x14}, //AD pipeline
{0x1b, 0x48},
{0x1e, 0x6b}, //3b//col bias
{0x1f, 0x28}, //20//00//08//txlow
{0x20, 0x8b}, //88//0c//[3:2]DA15
{0x21, 0x49}, //48//[3] txhigh
{0x22, 0xb0},
{0x23, 0x04}, //[1:0]vcm_r
{0x24, 0x16}, //15
{0x34, 0x20}, //[6:4] rsg high//range

/////////////////////////////////////////////////
////////////////////   BLK   ////////////////////
/////////////////////////////////////////////////
{0x26, 0x23}, //[1]dark_current_en [0]offset_en
{0x28, 0xff}, //BLK_limie_value
{0x29, 0x00}, //global offset
{0x32, 0x06},
{0x33, 0x18}, //offset_ratio
{0x37, 0x20}, //dark_current_ratio
{0x47, 0x80}, //a7
{0x4e, 0x66}, //select_row
{0xa8, 0x02}, //win_width_dark, same with crop_win_width
{0xa9, 0x80},

/////////////////////////////////////////////////
//////////////////   ISP reg  ///////////////////
/////////////////////////////////////////////////
{0x40, 0x1e},//ff //48
{0x41, 0x01}, //00//[0]curve_en
{0x42, 0x00}, //0a//[1]awn_en
{0x44, 0x17},//02
{0x45, 0xaf},
{0x46, 0x33}, //sync
{0x4a, 0x11},
{0x4b, 0x01},
{0x4c, 0xa0}, //00[5]pretect exp
{0x4d, 0x05}, //update gain mode
{0x4f, 0x00}, //AEC off
{0x50, 0x01}, //crop enable
{0x55, 0x01}, //crop window height
{0x56, 0xe0},
{0x57, 0x02}, //crop window width
{0x58, 0x80},

/////////////////////////////////////////////////
///////////////////   GAIN   ////////////////////
/////////////////////////////////////////////////
{0x70, 0x70}, //70 //80//global gain
{0x5a, 0x84}, //84//analog gain 0
{0x5b, 0xc9}, //c9
{0x5c, 0xed}, //ed//not use pga gain highest level
{0x77, 0x74}, //awb gain
{0x78, 0x40},
{0x79, 0x5f},

{0x48, 0x00}, //[2:0] col_code
{0xfe, 0x01},
{0x0a, 0x45}, //[7]col gain mode

{0x3e, 0x40},
{0x3f, 0x57},
{0x40, 0x7d},
{0x41, 0xbd},
{0x42, 0xf6},
{0x43, 0x63},
{0x03, 0x60},
{0x44, 0x03},
{0xfe, 0x00},
/////////////////////////////////////////////////
///////////////////   DNDD  /////////////////////
/////////////////////////////////////////////////
{0x82, 0x05},//08 //05
{0x83, 0x0b},
{0x89, 0xf0},

/////////////////////////////////////////////////
//////////////////   EEINTP  ////////////////////
/////////////////////////////////////////////////
{0x8f, 0xaa},
{0x90, 0x8c},
{0x91, 0x90},
{0x92, 0x03},
{0x93, 0x03},
{0x94, 0x05},
{0x95, 0x65},
{0x96, 0xf0},

/////////////////////////////////////////////////
/////////////////   dark sun   //////////////////
/////////////////////////////////////////////////
{0xfe, 0x01},
{0x45, 0xf7},
{0x46, 0xff}, //f0//sun vaule th
{0x47, 0x15},
{0x48, 0x03}, //sun mode
{0x4f, 0x60}, //sun_clamp

};

static struct msm_camera_i2c_reg_conf gc0310_vga_settings[] = {
};

#endif
