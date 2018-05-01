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
Sensor Model:   hm2056
Camera Module:
Lens Model:
Driver IC:
PV Size         = 1280 x 960
Cap Size        = 2592 x 1944
Output Format   = BGGR
MCLK Speed      = 24M
PV Frame Rate   = 30fps
Cap Frame Rate  = 30fps
I2C Slave ID    = 0x24
I2C Mode        = 16Addr, 8Data
*/

#ifndef CAMSENSOR_HM2056
#define CAMSENSOR_HM2056

/* HM SENSOR SCCB */
struct hm2056_sensor {
	uint16_t addr;
	uint8_t data;
	uint8_t mask;
};

/* Sensor ID */
#define HM2056_SENSOR_ID 0x2056

#define capture_framerate 3000     /* 30fps capture frame rate */
#define g_preview_frameRate 3000  /* 30fps preview frame rate */

extern struct proc_dir_entry proc_root;

static struct msm_camera_i2c_reg_conf hm2056_start_settings[] = {
	{0x0000, 0x01},
	{0x0100, 0x01},
	{0x0101, 0x01},

	{0x0005, 0x01},	/// Turn on rolling shutter
};

static struct msm_camera_i2c_reg_conf hm2056_stop_settings[] = {
	{0x0005, 0x00},  /* streaming off*/
};

static struct msm_camera_i2c_reg_conf hm2056_groupon_settings[] = {
	//{0x3208, 0x0},
};

static struct msm_camera_i2c_reg_conf hm2056_groupoff_settings[] = {
	//{0x3208, 0x10},
	//{0x3208, 0xa0},
};

static struct msm_camera_i2c_reg_conf hm2056_res0_settings[] = {
#if 1
	{0x0024, 0x00},  /// Mipi Disable
//	{0x0006, 0x00},	///
	{0x0006, 0x00},	///
	{0x000D, 0x00},	/// 1600 x 1200
	{0x000E, 0x00},	///
	{0x0012, 0x04},	///
	{0x0013, 0x00},	///
	{0x002A, 0x2F},	/// PLL Setting For 15fps under VGA @ 24MHz MCLK, MIPI 576bps
	{0x0071, 0xAB},	///
	{0x0082, 0xE2},	///
	{0x011F, 0xFF},	///
	{0x0131, 0xBC},	/// simle bpc enable[4]
	{0x0144, 0x04},	/// Sort bpc hot pixel ratio
	{0x0190, 0x87},	/// A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{0x0192, 0x50},	/// A13 Strength[4:0],hot pixel detect for cluster[6]

#if 1
	{0x05E4, 0x00},
	{0x05E5, 0x00},
	{0x05E6, 0x53},
	{0x05E7, 0x06},
	{0x05E8, 0x00},
	{0x05E9, 0x00},
	{0x05EA, 0xC3},
	{0x05EB, 0x04},
#endif

#if 0
	{0x05E4, 0x04},	/// Windowing
	{0x05E5, 0x00},	///
	{0x05E6, 0x83},
	{0x05E7, 0x02},
	{0x05E8, 0x06},
	{0x05E9, 0x00},	///
	{0x05EA, 0xE5},
	{0x05EB, 0x01},
#endif
	///////////////////////////////////////////////////////////////////////////
	/// MIPI Setting          ///
	///////////////////////////////////////////////////////////////////////////
	/// TX pseudo-buffer setting
	{0x007C, 0x37},	/// pre-hsync setting
	{0x0024, 0x40},  /// Mipi Enable
#endif
};

static struct msm_camera_i2c_reg_conf hm2056_res1_settings[] = {
	{0x0024, 0x00},  /// Mipi Disable
//	{0x0006, 0x00},	///
	{0x0006, 0x00},	///
	{0x000D, 0x11},	/// 800 x 600
	{0x000E, 0x11},	///
	{0x0012, 0x1C},	///
	{0x0013, 0x01},	///
	{0x002A, 0x1F},	/// PLL
	{0x0071, 0xFF},	///
	{0x0082, 0xA2},	///
	{0x011F, 0xF7},	///
	{0x0131, 0xBD},	/// simle bpc enable[4]
	{0x0144, 0x06},	/// Sort bpc hot pixel ratio
	{0x0190, 0x80},	/// A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{0x0192, 0x48},	/// A13 Strength[4:0],hot pixel detect for cluster[6]

#if 1
	{0x05E4, 0x00},
	{0x05E5, 0x00},
	{0x05E6, 0x29},
	{0x05E7, 0x03},
	{0x05E8, 0x00},
	{0x05E9, 0x00},
	{0x05EA, 0x61},
	{0x05EB, 0x02},
#endif

#if 0
	{0x05E4, 0x04},	/// Windowing
	{0x05E5, 0x00},	///
	{0x05E6, 0x83},
	{0x05E7, 0x02},
	{0x05E8, 0x06},
	{0x05E9, 0x00},	///
	{0x05EA, 0xE5},
	{0x05EB, 0x01},
#endif
	///////////////////////////////////////////////////////////////////////////
	/// MIPI Setting          ///
	///////////////////////////////////////////////////////////////////////////
	/// TX pseudo-buffer setting
	{0x007C, 0x37},	/// pre-hsync setting
	{0x0024, 0x40},  /// Mipi Enable
};

static struct msm_camera_i2c_reg_conf hm2056_res2_settings[] = {
	{0x0024, 0x00}, /// Mipi Disable
	///S 10	/// Delay 10ms
	{0x0006, 0x10},	/// 1392 x 780
	{0x000D, 0x00},	///
	{0x000E, 0x00},	///
	{0x0012, 0x1C},	/// For 1620 x 912 (1620 x 1220)
	{0x0013, 0x01},	///
	{0x002A, 0x2F},	/// PLL Setting For 15fps under VGA @ 24MHz MCLK, MIPI 576bps
	{0x0071, 0xFF},	///
	{0x0082, 0xA2},	///
	{0x011F, 0xFF},	///
	{0x0131, 0xB8},	/// simle bpc enable[4]
	{0x0144, 0x06},	/// Sort bpc hot pixel ratio
	{0x0190, 0x80},	/// A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{0x0192, 0x48},	/// A13 Strength[4:0],hot pixel detect for cluster[6]
#if 1
	{0x05E4, 0x72},	/// Windowing X:1392
	{0x05E5, 0x00},	///
	{0x05E6, 0xE2},
	{0x05E7, 0x05},

	{0x05E8, 0xDC},	/// Y:780
	{0x05E9, 0x00},	///
	{0x05EA, 0xE7},
	{0x05EB, 0x03},
#endif

#if 0
	{0x05E4, 0x04},	/// Windowing
	{0x05E5, 0x00},	///
	{0x05E6, 0x83},
	{0x05E7, 0x02},
	{0x05E8, 0x06},	///
	{0x05E9, 0x00},	///
	{0x05EA, 0xE5},
	{0x05EB, 0x01},
#endif
	///////////////////////////////////////////////////////////////////////////
	/// MIPI Setting          ///
	///////////////////////////////////////////////////////////////////////////
	/// TX pseudo-buffer setting
	{0x007C, 0x37},	/// pre-hsync setting
	///S 100
	{0x0024, 0x40},  /// Mipi Enable
	///S 10              /// Delay 10ms
};

static struct msm_camera_i2c_reg_conf hm2056_res3_settings[] = {
	{0x0024, 0x00},  /// Mipi Disable
//	{0x0006, 0x00},	///
	{0x0006, 0x00},	///
	{0x000D, 0x11},	/// 800 x 600
	{0x000E, 0x11},	///
	{0x0012, 0x1C},	///
	{0x0013, 0x01},	///
	{0x002A, 0x1F},	/// PLL
	{0x0071, 0xFF},	///
	{0x0082, 0xA2},	///
	{0x011F, 0xF7},	///
	{0x0131, 0xBD},	/// simle bpc enable[4]
	{0x0144, 0x06},	/// Sort bpc hot pixel ratio
	{0x0190, 0x80},	/// A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{0x0192, 0x48},	/// A13 Strength[4:0],hot pixel detect for cluster[6]
#if 0
	{0x05E4, 0x04},	/// Windowing
	{0x05E5, 0x00},	///
	{0x05E6, 0x83},
	{0x05E7, 0x02},
	{0x05E8, 0x06},
	{0x05E9, 0x00},	///
	{0x05EA, 0xE5},
	{0x05EB, 0x01},
#endif
	///////////////////////////////////////////////////////////////////////////
	/// MIPI Setting          ///
	///////////////////////////////////////////////////////////////////////////
	/// TX pseudo-buffer setting
	{0x007C, 0x37},	/// pre-hsync setting
	{0x0024, 0x40},  /// Mipi Enable
};

static struct msm_camera_i2c_reg_conf hm2056_recommend_settings[] = {
	{0x0022, 0x00},	/// Reset

	{0x0020, 0x00},	///
	{0x0025, 0x00},	/// CKCFG 80 from system clock, 00 from PLL
	{0x0026, 0x87},	/// For 15fps @ 24MHz MCLK, MIPI 144bps
	{0x0027, 0x03},	/// RAW8 output with Bayer Denoise
	{0x002A, 0x2F},	///
	{0x002B, 0x04},	/// sys_div = 00, 1/4 /// op_div = 01, 1/2
	{0x002C, 0x0A},	/// Set default vaule for CP and resistance of LPF to 1010

	{0x0004, 0x10},	///
//	{0x0006, 0x00},	/// Flip/Mirror
	{0x0006, 0x00},	/// Flip/Mirror
	{0x000D, 0x00},	/// FullFrame
	{0x000E, 0x00},	/// FullFrame
	{0x000F, 0x00},	/// IMGCFG
	//{0x0010, 0x00},
	{0x0011, 0x14},	///
	{0x0012, 0x1C},	/// 2012.02.08
	{0x0013, 0x01},	///

	{0x0040, 0x20},	/// 20120224 for BLC stable
	{0x0053, 0x0A},	///
	{0x0044, 0x06},	/// enable BLC_phase2
	{0x0046, 0xD8},	/// enable BLC_phase1, disable BLC_phase2 dithering
	{0x004A, 0x0A},	/// disable BLC_phase2 hot pixel filter
	{0x004B, 0x72},	///

	{0x0075, 0x01},	/// in OMUX data swap for debug usage

	{0x0070, 0x5F},	///
	{0x0071, 0xFF},	///
	{0x0072, 0x55},	///
	{0x0073, 0x50},	///
	{0x0080, 0xC8},	/// 2012.02.08
	{0x0082, 0xA2},	///
	{0x0083, 0xF0},	///
	{0x0085, 0x11},	/// Enable Thin-Oxide Case (Kwangoh kim), Set ADC power to 100% Enable thermal sensor control bit[7] 0:on 1:off 2012 02 13 (YL)
	{0x0086, 0x02},	/// K.Kim 2011.12.09
	{0x0087, 0x80},	/// K.Kim 2011.12.09
	{0x0088, 0x6C},	///
	{0x0089, 0x2E},	///
	{0x008A, 0x7D},	/// 20120224 for BLC stable
	{0x008D, 0x20},	///
	{0x0090, 0x00},	/// 1.5x(Change Gain Table )
	{0x0091, 0x10},	/// 3x  (3x CTIA)
	{0x0092, 0x11},	/// 6x  (3x CTIA + 2x PGA)
	{0x0093, 0x12},	/// 12x (3x CTIA + 4x PGA)
	{0x0094, 0x16},	/// 24x (3x CTIA + 8x PGA)
	{0x0095, 0x08},	/// 1.5x  20120217 for color shift
	{0x0096, 0x00},	/// 3x    20120217 for color shift
	{0x0097, 0x10},	/// 6x    20120217 for color shift
	{0x0098, 0x11},	/// 12x   20120217 for color shift
	{0x0099, 0x12},	/// 24x   20120217 for color shift
	{0x009A, 0x16},	/// 48x
	{0x009B, 0x34},	///
	{0x00A0, 0x00},	///
	{0x00A1, 0x04},	/// 2012.02.06(for Ver.C)

	///////////////////////////////////////////////////////////////////////////

	{0x011F, 0xFF},	/// simple bpc P31 & P33[4] P40 P42 P44[5]

	{0x0120, 0x13},	/// 36:50Hz, 37:60Hz, BV_Win_Weight_En=1
	{0x0121, 0x01},	/// NSatScale_En=0, NSatScale=0
	{0x0122, 0x39},
	{0x0123, 0xC2},
	{0x0124, 0xCE},
	{0x0125, 0x20},
	{0x0126, 0x50},
	{0x0128, 0x1F},
	{0x0132, 0x10},
	{0x0136, 0x0A},	/// BLI Target

	{0x0131, 0xB8},	/// simle bpc enable[4]
	{0x0140, 0x14},
	{0x0141, 0x0A},
	{0x0142, 0x14},
	{0x0143, 0x0A},
	{0x0144, 0x06},	/// Sort bpc hot pixel ratio
	{0x0145, 0x00},
	{0x0146, 0x20},
	{0x0147, 0x0A},
	{0x0148, 0x10},
	{0x0149, 0x0C},
	{0x014A, 0x80},
	{0x014B, 0x80},
	{0x014C, 0x2E},
	{0x014D, 0x2E},
	{0x014E, 0x05},
	{0x014F, 0x05},
	{0x0150, 0x0D},
	{0x0155, 0x00},
	{0x0156, 0x10},
	{0x0157, 0x0A},
	{0x0158, 0x0A},
	{0x0159, 0x0A},
	{0x015A, 0x05},
	{0x015B, 0x05},
	{0x015C, 0x05},
	{0x015D, 0x05},

	{0x015E, 0x08},
	{0x015F, 0xFF},
	{0x0160, 0x50},	/// OTP BPC 2line & 4line enable
	{0x0161, 0x20},
	{0x0162, 0x14},
	{0x0163, 0x0A},
	{0x0164, 0x10},	/// OTP 4line Strength
	{0x0165, 0x08},
	{0x0166, 0x0A},

	{0x018C, 0x24},
	{0x018D, 0x04},	/// Cluster correction enable singal from thermal sensor (YL 2012 02 13)
	{0x018E, 0x00},	/// Enable Thermal sensor control bit[7] (YL 2012 02 13)
	{0x018F, 0x11},	/// Cluster Pulse enable T1[0] T2[1] T3[2] T4[3]
	{0x0190, 0x80},	/// A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{0x0191, 0x47},	/// A11[0],A7[1],Sort[3],A13 AVG[6]
	{0x0192, 0x48},	/// A13 Strength[4:0],hot pixel detect for cluster[6]
	{0x0193, 0x64},
	{0x0194, 0x32},
	{0x0195, 0xc8},
	{0x0196, 0x96},
	{0x0197, 0x64},
	{0x0198, 0x32},
	{0x0199, 0x14},	/// A13 hot pixel th
	{0x019A, 0x20},	/// A13 edge detect th
	{0x019B, 0x14},

	{0x01BA, 0x10},	/// BD
	{0x01BB, 0x04},
	{0x01D8, 0x40},
	{0x01DE, 0x60},
	{0x01E4, 0x04},
	{0x01E5, 0x04},
	{0x01E6, 0x04},
	{0x01F2, 0x0C},
	{0x01F3, 0x14},
	{0x01F8, 0x04},
	{0x01F9, 0x0C},
	{0x01FE, 0x02},
	{0x01FF, 0x04},

	{0x0380, 0xFC},	/// Turn off AEAWB
	{0x0381, 0x4A},
	{0x0382, 0x36},
	{0x038A, 0x40},
	{0x038B, 0x08},
	{0x038C, 0xC1},
	{0x038E, 0x40},
	{0x038F, 0x09},
	{0x0390, 0xD0},
	{0x0391, 0x05},
	{0x0393, 0x80},
	{0x0395, 0x21},	/// AEAWB skip count

	{0x0420, 0x84},	/// Digital Gain offset
	{0x0421, 0x00},
	{0x0422, 0x00},
	{0x0423, 0x83},

	{0x0466, 0x14},

	{0x0460, 0x01},	/// Alpha MinBV
	{0x0461, 0xFF},	/// Alpha Outdoor
	{0x0462, 0xFF},	/// Alpha CT
	{0x0478, 0x01},	/// Alpha TG

	{0x047A, 0x00},	/// ELOFFNRB
	{0x047B, 0x00},	/// ELOFFNRY

	{0x0540, 0x00},	///
	{0x0541, 0x9D},	/// 60Hz Flicker
	{0x0542, 0x00},	///
	{0x0543, 0xBC},	/// 50Hz Flicker



	{0x05E4, 0x00},	/// Windowing
	{0x05E5, 0x00},
	{0x05E6, 0x53},
	{0x05E7, 0x06},

	{0x05E8, 0x00},	/// Windowing
	{0x05E9, 0x00},
	{0x05EA, 0xC3},
	{0x05EB, 0x04},



	{0x0698, 0x00},
	{0x0699, 0x00},

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	///      MIPI Setting     ///
	/// Skip if parallel only ///
	///////////////////////////////////////////////////////////////////////////

	/// TX pseudo-buffer setting
	{0x007C, 0x37},	/// pre-hsync setting
	{0x007D, 0x3E},	/// pre-vsync setting

	/// TX Digital Core initial
	{0x0B02, 0x01},	/// TLPX Width
//	{0x0B03, 0x01},	/// HS ZERO WIDTH
	{0x0B03, 0x03},	/// HS ZERO WIDTH
	{0x0B04, 0x02},	/// HS TRAIL WIDTH
	{0x0B05, 0x08},  	/// CLK ZERO WIDTH
	{0x0B06, 0x02},	/// CLK TRIAL WIDTH
	{0x0B07, 0x28},	/// MARK1 WIDTH

	{0x0B0E, 0x0B}, 	/// CLK FRONT PORCH WIDTH
	{0x0B0F, 0x04}, 	/// CLK BACK PORCH WIDTH

	{0x0B22, 0x02},	/// HS_EXIT Eanble
	{0x0B39, 0x03},  	/// Clock Lane HS_EXIT WIDTH(at least 100ns)


	{0x0B11, 0x7F}, 	/// Clock Lane LP Driving Strength
	{0x0B12, 0x7F}, 	/// Data Lane LP Driving Strength
	{0x0B17, 0xE0}, 	/// D-PHY Power Down Control

	{0x0B20, 0xBE},	/// Set clock lane is on at sending packet

	{0x0B22, 0x02},	/// HS_EXIT Eanble
	{0x0B30, 0x0F},	/// D-PHY Reset, set to 1 for normal operation
	{0x0B31, 0x02}, 	/// [1]: PHASE_SEL = 1 First Data at rising edge
	{0x0B32, 0x00},	/// [4]: DBG_ULPM
	{0x0B33, 0x00}, 	/// DBG_SEL

	{0x0B35, 0x00}, 	/// Data Lane WIDTH for LP to HS(LP-00)
//	{0x0B35, 0x01}, 	/// Data Lane WIDTH for LP to HS(LP-00)
	{0x0B36, 0x00}, 	/// Data Lane WIDTH for HS to LP(HS-Trail)
	{0x0B37, 0x00}, 	/// Clock Lane WIDTH for LP to HS(LP-00)
	{0x0B38, 0x00}, 	/// Clock Lane WIDTH for HS to LP(HS-Trail)
	{0x0B39, 0x03}, 	/// Clock Lane HS_EXIT WIDTH(at least 100ns)
	{0x0B3A, 0x00}, 	/// Data Lane HS_EXIT WIDTH(at least 100ns)
	{0x0B3B, 0x12},	/// Turn on PHY LDO

	{0x0B3F, 0x01},	/// MIPI reg delay, Add by Wilson, 20111114
	//{0x0028, 0x83},

	/// MIPI TX Enable
	{0x0024, 0x40}, 	/// [6]: MIPI Enable
};
#endif

