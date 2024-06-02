// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the AR0144 Image Sensor Processor
 *
 * Copyright (C) 2018 eyeSight Technologies Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "ccs-pll.h"

#define AR0144_CHIP_VERSION_REG					CCI_REG16(0x3000)
#define		AR0144_CHIP_VERSION					0x0356
#define AR0144_Y_ADDR_START					CCI_REG16(0x3002)
#define AR0144_X_ADDR_START					CCI_REG16(0x3004)
#define AR0144_Y_ADDR_END					CCI_REG16(0x3006)
#define AR0144_X_ADDR_END					CCI_REG16(0x3008)
#define AR0144_FRAME_LENGTH_LINES				CCI_REG16(0x300a)
#define AR0144_LINE_LENGTH_PCK					CCI_REG16(0x300c)
#define AR0144_REVISION_NUMBER					CCI_REG16(0x300e)
#define		AR0144_REVISION_NUMBER_CREV(n)				(((n) >> 12) & 0xf)
#define		AR0144_REVISION_NUMBER_SILICON(n)			(((n) >> 4) & 0xf)
#define		AR0144_REVISION_NUMBER_OTPM(n)				(((n) >> 0) & 0xf)
#define AR0144_LOCK_CONTROL					CCI_REG16(0x3010)
#define		AR0144_LOCK_CONTROL_VALUE				0xbeef
#define AR0144_COARSE_INTEGRATION_TIME				CCI_REG16(0x3012)
#define AR0144_FINE_INTEGRATION_TIME				CCI_REG16(0x3014)
#define AR0144_COARSE_INTEGRATION_TIME_CB			CCI_REG16(0x3016)
#define AR0144_FINE_INTEGRATION_TIME_CB				CCI_REG16(0x3018)
#define AR0144_RESET_REGISTER					CCI_REG16(0x301a)
#define		AR0144_GROUP_PARAMETER_HOLD				BIT(15)
#define		AR0144_SMIA_SERIALIZER_DIS				BIT(12)
#define		AR0144_FORCED_PLL_ON					BIT(11)
#define		AR0144_RESTART_BAD					BIT(10)
#define		AR0144_MASK_BAD						BIT(9)
#define		AR0144_GPI_EN						BIT(8)
#define		AR0144_PARALLEL_EN					BIT(7)
#define		AR0144_DRIVE_PINS					BIT(6)
#define		AR0144_LOCK_REG						BIT(3)
#define		AR0144_STREAM						BIT(2)
#define		AR0144_RESTART						BIT(1)
#define		AR0144_RESET						BIT(0)
#define AR0144_MODE_SELECT					CCI_REG8(0x301c)
#define		AR0144_MODE_STREAM					BIT(0)
#define AR0144_IMAGE_ORIENTATION				CCI_REG8(0x301d)
#define		AR0144_ORIENTATION_VERT_FLIP				BIT(1)
#define		AR0144_ORIENTATION_HORIZ_MIRROR				BIT(0)
#define AR0144_DATA_PEDESTAL					CCI_REG16(0x301e)
#define AR0144_SOFTWARE_RESET					CCI_REG8(0x3021)
#define AR0144_GROUPED_PARAMETER_HOLD				CCI_REG8(0x3022)
#define AR0144_MASK_CORRUPTED_FRAMES				CCI_REG8(0x3023)
#define AR0144_PIXEL_ORDER					CCI_REG8(0x3024)
#define AR0144_GPI_STATUS					CCI_REG16(0x3026)
#define		AR0144_TRIGGER						BIT(2)
#define		AR0144_OUTPUT_ENABLE_N					BIT(1)
#define		AR0144_SADDR						BIT(0)
#define AR0144_ROW_SPEED					CCI_REG16(0x3028)
#define		AR0144_ROW_SPEED_HALF_CLK				BIT(4)
#define AR0144_VT_PIX_CLK_DIV					CCI_REG16(0x302a)
#define AR0144_VT_SYS_CLK_DIV					CCI_REG16(0x302c)
#define AR0144_PRE_PLL_CLK_DIV					CCI_REG16(0x302e)
#define AR0144_PLL_MULTIPLIER					CCI_REG16(0x3030)
#define AR0144_CTX_CONTROL_REG					CCI_REG16(0x3034)
#define AR0144_OP_PIX_CLK_DIV					CCI_REG16(0x3036)
#define AR0144_OP_SYS_CLK_DIV					CCI_REG16(0x3038)
#define AR0144_FRAME_COUNT					CCI_REG16(0x303a)
#define AR0144_FRAME_STATUS					CCI_REG16(0x303c)
#define		AR0144_PLL_LOCKED					BIT(3)
#define		AR0144_FRAME_START_DURING_GPH				BIT(2)
#define		AR0144_STANDBY_STATUS					BIT(1)
#define		AR0144_FRAMESYNC					BIT(0)
#define AR0144_LINE_LENGTH_PCK_CB				CCI_REG16(0x303e)
#define AR0144_READ_MODE					CCI_REG16(0x3040)
#define		AR0144_VERT_FLIP					BIT(15)
#define		AR0144_HORZ_MIRROR					BIT(14)
#define		AR0144_READ_MODE_COL_BIN				BIT(13)
#define		AR0144_READ_MODE_ROW_BIN				BIT(12)
#define		AR0144_READ_MODE_COL_BIN_CB				BIT(11)
#define		AR0144_READ_MODE_ROW_BIN_CB				BIT(10)
#define		AR0144_READ_MODE_COL_SF_BIN_EN				BIT(9)
#define		AR0144_READ_MODE_COL_SF_BIN_EN_CB			BIT(8)
#define		AR0144_READ_MODE_COL_SF_BIN_MONO_EN			BIT(7)
#define		AR0144_READ_MODE_COL_SF_BIN_MONO_EN_CB			BIT(6)
#define		AR0144_READ_MODE_COL_SUM				BIT(5)
#define AR0144_EXTRA_DELAY					CCI_REG16(0x3042)
#define AR0144_GREEN1_GAIN					CCI_REG16(0x3056)
#define AR0144_BLUE_GAIN					CCI_REG16(0x3058)
#define AR0144_RED_GAIN						CCI_REG16(0x305a)
#define AR0144_GREEN2_GAIN					CCI_REG16(0x305c)
#define AR0144_GLOBAL_GAIN					CCI_REG16(0x305e)
#define AR0144_ANALOG_GAIN					CCI_REG16(0x3060)
#define		AR0144_COARSE_GAIN_CB(n)				((n) << 12)
#define		AR0144_FINE_GAIN_CB(n)					((n) << 8)
#define		AR0144_COARSE_GAIN(n)					((n) << 4)
#define		AR0144_FINE_GAIN(n)					((n) << 0)
#define AR0144_SMIA_TEST					CCI_REG16(0x3064)
#define		AR0144_EMBEDDED_DATA					BIT(8)
#define		AR0144_STATS_EN						BIT(7)
#define AR0144_CTX_WR_DATA_REG					CCI_REG16(0x3066)
#define AR0144_CTX_RD_DATA_REG					CCI_REG16(0x3068)
#define AR0144_DATAPATH_SELECT					CCI_REG16(0x306e)
#define		AR0144_SLEW_RATE_CTRL_PARALLEL(n)			(((n) & 7) << 13)
#define		AR0144_SLEW_RATE_CTRL_PIXCLK(n)				(((n) & 7) << 10)
#define		AR0144_SPECIAL_LINE_VALID_NORMAL			(0U << 0)
#define		AR0144_SPECIAL_LINE_VALID_CONTINUOUS			(1U << 0)
#define		AR0144_SPECIAL_LINE_VALID_CONTINUOUS_XOR		(2U << 0)
#define AR0144_TEST_PATTERN_MODE				CCI_REG16(0x3070)
#define		AR0144_TEST_PATTERN_NONE				0
#define		AR0144_TEST_PATTERN_SOLID				1
#define		AR0144_TEST_PATTERN_BARS				2
#define		AR0144_TEST_PATTERN_BARS_FADE				3
#define		AR0144_TEST_PATTERN_WALKING_1S				256
#define AR0144_TEST_DATA_RED					CCI_REG16(0x3072)
#define AR0144_TEST_DATA_GREENR					CCI_REG16(0x3074)
#define AR0144_TEST_DATA_BLUE					CCI_REG16(0x3076)
#define AR0144_TEST_DATA_GREENB					CCI_REG16(0x3078)
#define AR0144_OPERATION_MODE_CTRL				CCI_REG16(0x3082)
#define AR0144_OPERATION_MODE_CTRL_CB				CCI_REG16(0x3084)
#define AR0144_SEQ_DATA_PORT					CCI_REG16(0x3086)
#define AR0144_SEQ_CTRL_PORT					CCI_REG16(0x3088)
#define		AR0144_SEQUENCER_STOPPED				BIT(15)
#define		AR0144_AUTO_INC_ON_READ					BIT(14)
#define		AR0144_ACCESS_ADDRESS(n)				((n) & 0x3ff)
#define AR0144_X_ADDR_START_CB					CCI_REG16(0x308a)
#define AR0144_Y_ADDR_START_CB					CCI_REG16(0x308c)
#define AR0144_X_ADDR_END_CB					CCI_REG16(0x308e)
#define AR0144_Y_ADDR_END_CB					CCI_REG16(0x3090)
#define AR0144_X_EVEN_INC					CCI_REG16(0x30a0)
#define AR0144_X_ODD_INC					CCI_REG16(0x30a2)
#define		AR0144_X_ODD_INC_SKIP(n)				(((n) << 1) - 1)
#define AR0144_Y_EVEN_INC					CCI_REG16(0x30a4)
#define AR0144_Y_ODD_INC					CCI_REG16(0x30a6)
#define		AR0144_Y_ODD_INC_SKIP(n)				(((n) << 1) - 1)
#define AR0144_Y_ODD_INC_CB					CCI_REG16(0x30a8)
#define AR0144_FRAME_LINE_LENGTH_CB				CCI_REG16(0x30aa)
#define AR0144_X_ODD_INC_CB					CCI_REG16(0x30ae)
#define AR0144_DIGITAL_TEST					CCI_REG16(0x30b0)
#define AR0144_TEMPSENS_DATA_REG				CCI_REG16(0x30b2)
#define AR0144_TEMPSENS_CTRL_REG				CCI_REG16(0x30b4)
#define AR0144_GREEN1_GAIN_CB					CCI_REG16(0x30bc)
#define AR0144_BLUE_GAIN_CB					CCI_REG16(0x30be)
#define AR0144_RED_GAIN_CB					CCI_REG16(0x30c0)
#define AR0144_GREEN2_GAIN_CB					CCI_REG16(0x30c2)
#define AR0144_GLOBAL_GAIN_CB					CCI_REG16(0x30c4)
#define AR0144_TEMPSENS_CALIB1					CCI_REG16(0x30c6)
#define AR0144_TEMPSENS_CALIB2					CCI_REG16(0x30c8)
#define AR0144_GRR_CONTROL1					CCI_REG16(0x30ce)
#define AR0144_NOISE_PEDESTAL					CCI_REG16(0x30fe)
#define AR0144_DELTA_DK_CONTROL					CCI_REG16(0x3180)
#define		AR0144_DELTA_DK_SUB_EN					BIT(15)
#define		AR0144_DELTA_DK_EVERY_FRAME				BIT(14)
#define		AR0144_DELTA_DK_RECALC					BIT(13)
#define		AR0144_DELTA_DK_GRADIENT_REMOVAL			BIT(10)
#define AR0144_DATA_FORMAT_BITS					CCI_REG16(0x31ac)
#define		AR0144_DATA_FORMAT_IN(n)				((n) << 8)
#define		AR0144_DATA_FORMAT_OUT(n)				((n) << 0)
#define AR0144_SERIAL_FORMAT					CCI_REG16(0x31ae)
#define		AR0144_NUM_LANES(n)					(n)
#define AR0144_FRAME_PREAMBLE					CCI_REG16(0x31b0)
#define AR0144_LINE_PREAMBLE					CCI_REG16(0x31b2)
#define AR0144_MIPI_TIMING_0					CCI_REG16(0x31b4)
#define		AR0144_T_HS_PREPARE(n)					((n) << 12)
#define		AR0144_T_HS_ZERO(n)					((n) << 8)
#define		AR0144_T_HS_TRAIL(n)					((n) << 4)
#define		AR0144_T_CLK_TRAIL(n)					((n) << 0)
#define AR0144_MIPI_TIMING_1					CCI_REG16(0x31b6)
#define		AR0144_T_CLK_PREPARE(n)					((n) << 12)
#define		AR0144_T_HS_EXIT(n)					((n) << 6)
#define		AR0144_T_CLK_ZERO(n)					((n) << 0)
#define AR0144_MIPI_TIMING_2					CCI_REG16(0x31b8)
#define		AR0144_T_BGAP(n)					((n) << 12)
#define		AR0144_T_CLK_PRE(n)					((n) << 6)
#define		AR0144_T_CLK_POST(n)					((n) << 0)
#define AR0144_MIPI_TIMING_3					CCI_REG16(0x31ba)
#define		AR0144_T_LPX(n)						((n) << 7)
#define		AR0144_T_WAKE_UP(n)					((n) << 0)
#define AR0144_MIPI_TIMING_4					CCI_REG16(0x31bc)
#define		AR0144_CONT_TX_CLK					BIT(15)
#define		AR0144_HEAVY_LP_LOAD					BIT(14)
#define		AR0144_T_INIT(n)					((n) << 0)
#define AR0144_SERIAL_CONFIG_STATUS				CCI_REG16(0x31be)
#define AR0144_SERIAL_CONTROL_STATUS				CCI_REG16(0x31c6)
#define AR0144_SERIAL_CRC_0					CCI_REG16(0x31c8)
#define AR0144_COMPANDING					CCI_REG16(0x31d0)
#define		AR0144_COMPAND_EN					BIT(0)
#define AR0144_STAT_FRAME_ID					CCI_REG16(0x31d2)
#define AR0144_I2C_WRT_CHEKCSUM					CCI_REG16(0x31d6)
#define AR0144_SERIAL_TEST					CCI_REG16(0x31d8)
#define AR0144_PIX_DEF_1D					CCI_REG16(0x31e0)
#define		AR0144_PIX_DEF_1D_DDC_EN				BIT(3)
#define		AR0144_PIX_DEF_CORRECTION_MODE				BIT(1)
#define		AR0144_PIX_DEF_ENABLE					BIT(0)
#define AR0144_HORIZONTAL_CURSOR_POSITION			CCI_REG16(0x31e8)
#define AR0144_VERTICAL_CURSOR_POSITION				CCI_REG16(0x31ea)
#define AR0144_HORIZONTAL_CURSOR_WIDTH				CCI_REG16(0x31ec)
#define AR0144_VERTICAL_CURSOR_WIDTH				CCI_REG16(0x31ee)
#define AR0144_CCI_IDS						CCI_REG16(0x31fc)
#define AR0144_CUSTOMER_REV					CCI_REG16(0x31fe)
#define AR0144_LED_FLASH_CONTROL				CCI_REG16(0x3270)
#define		AR0144_LED_FLASH_EN					BIT(8)
#define		AR0144_LED_DELAY(n)					((n) << 0)
#define AR0144_MIPI_TEST_CNTRL					CCI_REG16(0x3338)
#define AR0144_MIPI_COMPRESS_8_DATA_TYPE			CCI_REG16(0x333a)
#define AR0144_MIPI_COMPRESS_7_DATA_TYPE			CCI_REG16(0x333c)
#define AR0144_MIPI_COMPRESS_6_DATA_TYPE			CCI_REG16(0x333e)
#define AR0144_MIPI_JPEG_PN9_DATA_TYPE				CCI_REG16(0x3340)
#define AR0144_MIPI_CNTRL					CCI_REG16(0x3354)
#define		AR0144_CHAN_NUM(n)					((n) << 6)
#define		AR0144_DATA_TYPE_XMIT(n)				((n) << 0)
#define AR0144_MIPI_TEST_PATTERN_CNTRL				CCI_REG16(0x3356)
#define AR0144_MIPI_TEST_PATTERN_STATUS				CCI_REG16(0x3358)
#define AR0144_DIGITAL_CTRL_1					CCI_REG16(0x3786)
#define		AR0144_LSB_ALIGN_PARA_OUT				BIT(5)
#define		AR0144_USE_1FRAME_SYNCED				BIT(4)
#define		AR0144_PLL_TEST_MODE					BIT(3)
#define		AR0144_DRIVE_PIX_CLK					BIT(0)

#define AR0144_DEF_WIDTH			1280
#define AR0144_DEF_HEIGHT			800

#define AR0144_NUM_SUPPLIES			3

struct ar0144 {
	struct device *dev;

	struct regmap *regmap;
	struct clk *clk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[AR0144_NUM_SUPPLIES];

	ktime_t off_time;

	struct v4l2_fwnode_endpoint bus_cfg;

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};

	bool streaming;
};

static inline struct ar0144 *to_ar0144(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0144, sd);
}

static const u16 ar0144at_rev4_optimized_sequencer[] = {
	0x327f, 0x5780, 0x2730, 0x7e13, 0x8000, 0x157e, 0x1380, 0x000f,
	0x8190, 0x1643, 0x163e, 0x4522, 0x0937, 0x8190, 0x1643, 0x167f,
	0x9080, 0x0038, 0x7f13, 0x8023, 0x3b7f, 0x9345, 0x0280, 0x007f,
	0xb08d, 0x667f, 0x9081, 0x923c, 0x1635, 0x7f93, 0x4502, 0x8000,
	0x7fb0, 0x8d66, 0x7f90, 0x8182, 0x3745, 0x0236, 0x8180, 0x4416,
	0x3143, 0x7416, 0x787b, 0x7d45, 0x023d, 0x6445, 0x0a3d, 0x647e,
	0x1281, 0x8037, 0x7f10, 0x450a, 0x3f74, 0x7e10, 0x7e12, 0x0f3d,
	0xd27f, 0xd480, 0x2482, 0x9c03, 0x430d, 0x2d46, 0x4316, 0x5f16,
	0x532d, 0x1660, 0x404c, 0x2904, 0x2984, 0x81e7, 0x816f, 0x170a,
	0x81e7, 0x7f81, 0x5c0d, 0x5749, 0x5f53, 0x2553, 0x274d, 0x2bf8,
	0x1016, 0x4c09, 0x2bb8, 0x2b98, 0x4e11, 0x5367, 0x4001, 0x605c,
	0x095c, 0x1b40, 0x0245, 0x0045, 0x8029, 0xb67f, 0x8040, 0x047f,
	0x8841, 0x095c, 0x0b29, 0xb241, 0x0c40, 0x0340, 0x135c, 0x0341,
	0x1117, 0x125f, 0x2b90, 0x2b80, 0x816f, 0x4010, 0x4101, 0x5327,
	0x4001, 0x6029, 0xa35f, 0x4d1c, 0x1702, 0x81e7, 0x2983, 0x4588,
	0x4021, 0x7f8a, 0x4039, 0x4580, 0x2440, 0x087f, 0x885d, 0x5367,
	0x2992, 0x8810, 0x2b04, 0x8916, 0x5c43, 0x8617, 0x0b5c, 0x038a,
	0x484d, 0x4e2b, 0x804c, 0x0b41, 0x9f81, 0x6f41, 0x1040, 0x0153,
	0x2740, 0x0160, 0x2983, 0x2943, 0x5c05, 0x5f4d, 0x1c81, 0xe745,
	0x0281, 0x807f, 0x8041, 0x0a91, 0x4416, 0x092f, 0x7e37, 0x8020,
	0x307e, 0x3780, 0x2015, 0x7e37, 0x8020, 0x0343, 0x164a, 0x0a43,
	0x160b, 0x4316, 0x8f43, 0x1690, 0x4316, 0x7f81, 0x450a, 0x4130,
	0x7f83, 0x5d29, 0x4488, 0x102b, 0x0453, 0x2d40, 0x3045, 0x0240,
	0x087f, 0x8053, 0x2d89, 0x165c, 0x4586, 0x170b, 0x5c05, 0x8a60,
	0x4b91, 0x4416, 0x0915, 0x3dff, 0x3d87, 0x7e3d, 0x7e19, 0x8000,
	0x8b1f, 0x2a1f, 0x83a2, 0x7e11, 0x7516, 0x3345, 0x0a7f, 0x5380,
	0x238c, 0x667f, 0x1381, 0x8414, 0x8180, 0x313d, 0x6445, 0x2a3d,
	0xd27f, 0x4480, 0x2494, 0x3dff, 0x3d4d, 0x4502, 0x7fd0, 0x8000,
	0x8c66, 0x7f90, 0x8194, 0x3f44, 0x1681, 0x8416, 0x2c2c, 0x2c2c,
};

static const struct cci_reg_sequence ar0144at_rev4_recommended_setting[] = {
	{ CCI_REG16(0x3ed6), 0x3cb5 },
	{ CCI_REG16(0x3ed8), 0x8765 },
	{ CCI_REG16(0x3eda), 0x8888 },
	{ CCI_REG16(0x3edc), 0x97ff },
	{ CCI_REG16(0x3ef8), 0x6522 },
	{ CCI_REG16(0x3efa), 0x2222 },
	{ CCI_REG16(0x3efc), 0x6666 },
	{ CCI_REG16(0x3f00), 0xaa05 },
	{ CCI_REG16(0x3ee2), 0x180e },
	{ CCI_REG16(0x3ee4), 0x0808 },
	{ CCI_REG16(0x3eea), 0x2a09 },
	{ AR0144_ANALOG_GAIN, AR0144_COARSE_GAIN(0) | AR0144_FINE_GAIN(13) },
	{ CCI_REG16(0x3092), 0x00cf },
	{ CCI_REG16(0x3268), 0x0030 },
	{ AR0144_DIGITAL_CTRL_1, AR0144_LSB_ALIGN_PARA_OUT | 0x0020 },
	{ CCI_REG16(0x3f4a), 0x0f70 },
	{ AR0144_DATAPATH_SELECT, AR0144_SLEW_RATE_CTRL_PARALLEL(2) |
				  AR0144_SLEW_RATE_CTRL_PIXCLK(2) |
				  AR0144_SPECIAL_LINE_VALID_NORMAL | 0x0010 },
	{ AR0144_SMIA_TEST, 0x1802 },
	{ CCI_REG16(0x3ef6), 0x804d },
	{ AR0144_DELTA_DK_CONTROL, AR0144_DELTA_DK_SUB_EN |
				   AR0144_DELTA_DK_EVERY_FRAME | 0x008f },
	{ CCI_REG16(0x30ba), 0x7623 },
	{ CCI_REG16(0x3176), 0x0480 },
	{ CCI_REG16(0x3178), 0x0480 },
	{ CCI_REG16(0x317a), 0x0480 },
	{ CCI_REG16(0x317c), 0x0480 },
};

static const struct cci_reg_sequence ar0144at_pll_27mhz[] = {
	{ AR0144_VT_PIX_CLK_DIV, 6 },
	{ AR0144_VT_SYS_CLK_DIV, 1 },
	{ AR0144_PRE_PLL_CLK_DIV, 4 },
	{ AR0144_PLL_MULTIPLIER, 74 /* 66 */ },
	{ AR0144_OP_PIX_CLK_DIV, 12 },
	{ AR0144_OP_SYS_CLK_DIV, 1 },
	/* Addition settings from Oren */
	{ CCI_REG16(0x3080), 0x0000 },
	{ AR0144_DELTA_DK_CONTROL, 0x0042 },
	{ CCI_REG16(0x3182), 0x002e },
	{ CCI_REG16(0x3184), 0x1665 },
	{ CCI_REG16(0x3186), 0x110e },
	{ CCI_REG16(0x3188), 0x2047 },
	{ CCI_REG16(0x318a), 0x0105 },
	{ CCI_REG16(0x318c), 0x0004 },
};

static const struct cci_reg_sequence ar0144at_mipi_2lane_12bit[] = {
	{ AR0144_FRAME_PREAMBLE, 66 },
	{ AR0144_LINE_PREAMBLE, 46 },
	{ AR0144_MIPI_TIMING_0, AR0144_T_HS_PREPARE(1) | AR0144_T_HS_ZERO(6) |
				AR0144_T_HS_TRAIL(6) | AR0144_T_CLK_TRAIL(5) },
	{ AR0144_MIPI_TIMING_1, AR0144_T_CLK_PREPARE(1) | AR0144_T_HS_EXIT(4) |
				AR0144_T_CLK_ZERO(14) },
	{ AR0144_MIPI_TIMING_2, AR0144_T_BGAP(2) | AR0144_T_CLK_PRE(1) |
				AR0144_T_CLK_POST(7) },
	{ AR0144_MIPI_TIMING_3, AR0144_T_LPX(2) | AR0144_T_WAKE_UP(5) },
	{ AR0144_MIPI_TIMING_4, AR0144_T_INIT(4) },
};

static const struct cci_reg_sequence ar0144at_1280x800_60fps[] = {
	{ AR0144_Y_ADDR_START, 0 },
	{ AR0144_X_ADDR_START, 4 },
	{ AR0144_Y_ADDR_END, 799 },
	{ AR0144_X_ADDR_END, 1283 },
	{ AR0144_FRAME_LENGTH_LINES, 825 },
	{ AR0144_LINE_LENGTH_PCK, 1488 },
	{ AR0144_COARSE_INTEGRATION_TIME, 100 },
	{ AR0144_X_ODD_INC, AR0144_X_ODD_INC_SKIP(1) },
	{ AR0144_Y_ODD_INC, AR0144_Y_ODD_INC_SKIP(1) },
	{ AR0144_READ_MODE, 0 },
};

static const struct cci_reg_sequence ar0144at_context_b_2x2_binning[] = {
	{ AR0144_READ_MODE, AR0144_READ_MODE_ROW_BIN },
	{ AR0144_Y_ODD_INC_CB, AR0144_Y_ODD_INC_SKIP(2) },
	{ AR0144_READ_MODE, AR0144_READ_MODE_COL_BIN | AR0144_READ_MODE_ROW_BIN },
	{ AR0144_X_ODD_INC_CB, AR0144_X_ODD_INC_SKIP(2) },
};

static const struct cci_reg_sequence ar0144at_embedded_data_stats[] = {
	{ AR0144_SMIA_TEST, AR0144_EMBEDDED_DATA | AR0144_STATS_EN | 0x1802 },
};

static const struct cci_reg_sequence ar0144at_start_stream[] = {
	{ AR0144_ROW_SPEED, AR0144_ROW_SPEED_HALF_CLK },
	{ AR0144_RESET_REGISTER, AR0144_DRIVE_PINS | AR0144_LOCK_REG |
				 AR0144_STREAM | 0x10},
};

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

static void ar0144_log_status(struct ar0144 *sensor)
{
	u64 count = 0;
	u64 status = 0;

	if (!IS_ENABLED(CONFIG_DYNAMIC_DEBUG) && !IS_ENABLED(DEBUG))
		return;

	cci_read(sensor->regmap, AR0144_FRAME_COUNT, &count, NULL);
	cci_read(sensor->regmap, AR0144_FRAME_STATUS, &status, NULL);

	dev_dbg(sensor->dev, "FRAME_COUNT: %u, FRAME_STATUS 0x%04x\n",
		(u16)count, (u16)status);
}

static int ar0144_setup_pll(struct ar0144 *sensor)
{
	static const struct ccs_pll_limits limits = {
		.min_ext_clk_freq_hz = 6000000,
		.max_ext_clk_freq_hz = 64000000,

		.vt_fr = {
			.min_pre_pll_clk_div = 1,
			.max_pre_pll_clk_div = 63,
			.min_pll_ip_clk_freq_hz = 1000000,	/* min_pll_op_clk_freq_hz / max_pll_multiplier */
			.max_pll_ip_clk_freq_hz = 24000000,	/* max_pll_op_clk_freq_hz / min_pll_multiplier */
			.min_pll_multiplier = 32,
			.max_pll_multiplier = 384,
			.min_pll_op_clk_freq_hz = 384000000,
			.max_pll_op_clk_freq_hz = 768000000,
		},
		.vt_bk = {
			.min_sys_clk_div = 1,
			.max_sys_clk_div = 16,
			.min_sys_clk_freq_hz = 24000000,	/* min_pll_op_clk_freq_hz / max_pix_clk_div */
			.max_sys_clk_freq_hz = 297000000,	/* max_pix_clk_freq_hz / min_pix_clk_div */
			.min_pix_clk_div = 4,
			.max_pix_clk_div = 16,
			.min_pix_clk_freq_hz = 6000000,
			.max_pix_clk_freq_hz = 74250000,
		},
		.op_bk = {
			.min_sys_clk_div = 1,
			.max_sys_clk_div = 16,
			.min_sys_clk_freq_hz = 24000000,	/* min_pll_op_clk_freq_hz / max_pix_clk_div */
			.max_sys_clk_freq_hz = 297000000,	/* max_pix_clk_freq_hz / min_pix_clk_div */
			.min_pix_clk_div = 8,
			.max_pix_clk_div = 12,
			.min_pix_clk_freq_hz = 6000000,		/* Should be halved for 2 lanes ? */
			.max_pix_clk_freq_hz = 74250000,	/* Should be halved for 2 lanes ? */
		},

		.min_line_length_pck_bin = 1488,		/* To be checked */
		.min_line_length_pck = 1488,
	};
	struct ccs_pll pll = {
		.bus_type = CCS_PLL_BUS_TYPE_CSI2_DPHY,
		.binning_horizontal = 1,
		.binning_vertical = 1,
		.scale_m = 1,
		.scale_n = 1,
		.bits_per_pixel = 12,
		.flags = 0,
	};
	int ret;

	pll.csi2.lanes = sensor->bus_cfg.bus.mipi_csi2.num_data_lanes;
	pll.ext_clk_freq_hz = clk_get_rate(sensor->clk);
	pll.link_freq = 222750000;

	ret = ccs_pll_calculate(sensor->dev, &limits, &pll);
	if (ret) {
		dev_err(sensor->dev, "PLL calculations failed: %d\n", ret);
		return ret;
	}

	cci_write(sensor->regmap, AR0144_PRE_PLL_CLK_DIV,
		  pll.vt_fr.pre_pll_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_PLL_MULTIPLIER,
		  pll.vt_fr.pll_multiplier, &ret);
	cci_write(sensor->regmap, AR0144_VT_PIX_CLK_DIV,
		  pll.vt_bk.pix_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_VT_SYS_CLK_DIV,
		  pll.vt_bk.sys_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_OP_PIX_CLK_DIV,
		  pll.op_bk.pix_clk_div, &ret);
	cci_write(sensor->regmap, AR0144_OP_SYS_CLK_DIV,
		  pll.op_bk.sys_clk_div, &ret);

	/* Wait 1ms for the PLL to lock. */
	fsleep(1000);

	return ret;
}

static int ar0144_start_streaming(struct ar0144 *sensor,
				  struct v4l2_subdev_state *state)
{
	unsigned int nlanes = sensor->bus_cfg.bus.mipi_csi2.num_data_lanes;
	unsigned int i;
	int ret = 0;

	/* Program the sequencer. */
	cci_write(sensor->regmap, AR0144_SEQ_CTRL_PORT,
		  AR0144_SEQUENCER_STOPPED | AR0144_ACCESS_ADDRESS(0), &ret);
	for (i = 0; i < ARRAY_SIZE(ar0144at_rev4_optimized_sequencer); ++i)
		cci_write(sensor->regmap, AR0144_SEQ_DATA_PORT,
			  ar0144at_rev4_optimized_sequencer[i], &ret);

	cci_multi_reg_write(sensor->regmap, ar0144at_rev4_recommended_setting,
			    ARRAY_SIZE(ar0144at_rev4_recommended_setting),
			    &ret);
	cci_multi_reg_write(sensor->regmap, ar0144at_pll_27mhz,
			    ARRAY_SIZE(ar0144at_pll_27mhz), &ret);

	if (ret)
		return ret;

	msleep(100);

	ret = ar0144_setup_pll(sensor);
	if (ret)
		return ret;

	cci_multi_reg_write(sensor->regmap, ar0144at_mipi_2lane_12bit,
			    ARRAY_SIZE(ar0144at_mipi_2lane_12bit), &ret);

	cci_multi_reg_write(sensor->regmap, ar0144at_1280x800_60fps,
			    ARRAY_SIZE(ar0144at_1280x800_60fps), &ret);
	cci_multi_reg_write(sensor->regmap, ar0144at_context_b_2x2_binning,
			    ARRAY_SIZE(ar0144at_context_b_2x2_binning), &ret);
	cci_multi_reg_write(sensor->regmap, ar0144at_embedded_data_stats,
			    ARRAY_SIZE(ar0144at_embedded_data_stats), &ret);
	cci_multi_reg_write(sensor->regmap, ar0144at_start_stream,
			    ARRAY_SIZE(ar0144at_start_stream), &ret);

	cci_write(sensor->regmap, AR0144_SERIAL_FORMAT,
		  AR0144_NUM_LANES(nlanes) | 0x0200, &ret);

	cci_write(sensor->regmap, AR0144_DATA_FORMAT_BITS,
		  AR0144_DATA_FORMAT_IN(12) | AR0144_DATA_FORMAT_OUT(12), &ret);

	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(sensor->sd.ctrl_handler);
	if (ret) {
		dev_err(sensor->dev, "Failed to apply controls: %d\n", ret);
		return ret;
	}

	msleep(100);

	ar0144_log_status(sensor);

	return 0;
}

static int ar0144_stop_streaming(struct ar0144 *sensor)
{
	ar0144_log_status(sensor);

	return cci_write(sensor->regmap, AR0144_RESET_REGISTER,
			 AR0144_DRIVE_PINS | AR0144_LOCK_REG | 0x10, NULL);
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static const char * const ar0144_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"Full Color Bars",
	"Fade to Gray Color Bars",
	"Walking 1s",
};

static const u32 ar0144_test_pattern_values[] = {
	AR0144_TEST_PATTERN_NONE,
	AR0144_TEST_PATTERN_SOLID,
	AR0144_TEST_PATTERN_BARS,
	AR0144_TEST_PATTERN_BARS_FADE,
	AR0144_TEST_PATTERN_WALKING_1S,
};

static int ar0144_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0144 *sensor = container_of(ctrl->handler,
					     struct ar0144, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	int ret = 0;

	/*
	 * Return immediately for controls that don't need to be applied to the
	 * device.
	 */
	if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
		return 0;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	format = v4l2_subdev_state_get_format(state, 0);

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, AR0144_ANALOG_GAIN, ctrl->val, &ret);
		break;

	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, AR0144_COARSE_INTEGRATION_TIME,
			  ctrl->val, &ret);
		break;

	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, AR0144_TEST_PATTERN_MODE,
			  ar0144_test_pattern_values[ctrl->val], &ret);
		/*
		 * Register 0x3044 is not documented, but mentioned in the test
		 * pattern configuration. Bits [5:4] should be set to 0 to
		 * avoid clipping pixel values to 0xf70.
		 */
		cci_write(sensor->regmap, CCI_REG16(0x3044),
			  ctrl->val ? 0x0400 : 0x0410, &ret);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	{
		u32 reg = 0;

		if (sensor->hflip->val)
			reg |= AR0144_ORIENTATION_HORIZ_MIRROR;
		if (sensor->vflip->val)
			reg |= AR0144_ORIENTATION_VERT_FLIP;
		cci_write(sensor->regmap, AR0144_IMAGE_ORIENTATION, reg, &ret);
		break;
	}

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0144_ctrl_ops = {
	.s_ctrl = ar0144_set_ctrl,
};

static int ar0144_ctrl_init(struct ar0144 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 7);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &ar0144_ctrl_ops,
					&props);

	/*
	 * The sensor analogue gain is split in an exponential coarse gain and
	 * a fine gain. The minimum recommended gain is 1.6842, which maps to a
	 * gain code of 13. Set the minimum to 0 to expose the whole range of
	 * possible values, and the default to the recommended minimum.
	 */
	v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 79, 1, 13);

	v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
			  V4L2_CID_EXPOSURE, 1, 65535, 1, 16);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &ar0144_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ar0144_test_pattern_menu) - 1,
				     0, 0, ar0144_test_pattern_menu);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &ar0144_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &sensor->hflip);

	sensor->sd.ctrl_handler = &sensor->ctrls;

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static int ar0144_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ar0144 *sensor = to_ar0144(subdev);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(subdev);

	if (enable) {
		ret = pm_runtime_resume_and_get(sensor->dev);
		if (ret < 0)
			goto unlock;

		ret = ar0144_start_streaming(sensor, state);
		if (ret) {
			dev_err(sensor->dev, "Failed to start streaming: %d\n",
				ret);
			pm_runtime_put_sync(sensor->dev);
		}
	} else {
		ar0144_stop_streaming(sensor);
		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);
	}

unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB12_1X12;

	return 0;
}

static int ar0144_enum_frame_size(struct v4l2_subdev *subdev,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SRGGB12_1X12)
		return -EINVAL;

	if (fse->index >= 1)
		return -EINVAL;

	fse->min_width = AR0144_DEF_WIDTH;
	fse->max_width = AR0144_DEF_WIDTH;
	fse->min_height = AR0144_DEF_HEIGHT;
	fse->max_height = AR0144_DEF_HEIGHT;

	return 0;
}

static int ar0144_set_format(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_format *format)
{
	const struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_state_get_format(state, 0);
	format->format = *fmt;

	return 0;
}

static int ar0144_get_selection(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *state,
		struct v4l2_subdev_selection *sel)
{
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *v4l2_subdev_state_get_crop(state, 0);

	return 0;
}

static int ar0144_entity_init_state(struct v4l2_subdev *subdev,
		struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_state_get_format(state, 0);
	fmt->width = AR0144_DEF_WIDTH;
	fmt->height = AR0144_DEF_HEIGHT;
	fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = AR0144_DEF_WIDTH;
	crop->height = AR0144_DEF_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_video_ops ar0144_video_ops = {
	.s_stream = ar0144_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0144_subdev_pad_ops = {
	.enum_mbus_code = ar0144_enum_mbus_code,
	.enum_frame_size = ar0144_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar0144_set_format,
	.get_selection = ar0144_get_selection,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.video = &ar0144_video_ops,
	.pad = &ar0144_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0144_subdev_internal_ops = {
	.init_state = ar0144_entity_init_state,
};

/* -----------------------------------------------------------------------------
 * Power management
 */

static int ar0144_power_on(struct ar0144 *sensor)
{
	u64 reset_delay;
	long rate;
	int ret;

	/*
	 * The sensor must be powered off for at least 100ms before being
	 * powered on again.
	 */
	if (sensor->off_time) {
		u64 off_duration;

		off_duration = ktime_us_delta(ktime_get_boottime(),
					      sensor->off_time);
		if (off_duration < 100000)
			fsleep(100000 - off_duration);
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret) {
		dev_err(sensor->dev, "Failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(sensor->clk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(sensor->supplies),
				       sensor->supplies);
		dev_err(sensor->dev, "Failed to enable clock\n");
		return ret;
	}

	/*
	 * The internal initialization time after hard reset is 160000 EXTCLK
	 * cycles.
	 */
	rate = clk_get_rate(sensor->clk);
	reset_delay = DIV_ROUND_UP_ULL(160000ULL * USEC_PER_SEC, rate);

	gpiod_set_value_cansleep(sensor->reset, 1);
	fsleep(1000);
	gpiod_set_value_cansleep(sensor->reset, 0);
	fsleep(reset_delay);

	return 0;
}

static void ar0144_power_off(struct ar0144 *sensor)
{
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	sensor->off_time = ktime_get_boottime();

	clk_disable_unprepare(sensor->clk);
	gpiod_set_value_cansleep(sensor->reset, 1);
}

static int ar0144_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0144 *sensor = to_ar0144(sd);

	return ar0144_power_on(sensor);
}

static int ar0144_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0144 *sensor = to_ar0144(sd);

	ar0144_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops ar0144_pm_ops = {
	RUNTIME_PM_OPS(ar0144_runtime_suspend, ar0144_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & remove
 */

static const char * const ar0144_supply_name[AR0144_NUM_SUPPLIES] = {
	"vaa",
	"vdd_io",
	"vdd",
};

static int ar0144_parse_dt(struct ar0144 *sensor)
{
	struct v4l2_fwnode_endpoint *ep = &sensor->bus_cfg;
	struct fwnode_handle *endpoint;
	unsigned int nlanes;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!endpoint) {
		dev_err(sensor->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	/* Only CSI-2 is supported for now. */
	ep->bus_type = V4L2_MBUS_CSI2_DPHY;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, ep);
	fwnode_handle_put(endpoint);
	if (ret == -ENXIO) {
		dev_err(sensor->dev, "Unsupported bus type, should be CSI2\n");
		goto error;
	} else if (ret) {
		dev_err(sensor->dev, "Parsing endpoint node failed\n");
		goto error;
	}

	/* Validate the number of data lanes. */
	nlanes = ep->bus.mipi_csi2.num_data_lanes;
	if (nlanes != 1 && nlanes != 2) {
		dev_err(sensor->dev, "Invalid data lanes: %d\n", nlanes);
		ret = -EINVAL;
		goto error;
	}

	dev_dbg(sensor->dev, "Using %u data lanes\n", nlanes);

	if (!ep->nr_of_link_frequencies) {
		dev_err(sensor->dev, "link-frequency property not found in DT\n");
		ret = -EINVAL;
		goto error;
	}

	/*
	 * TODO: Validate the link frequencies, make sure the PLL can produce
	 * them.
	 */

	return 0;

error:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static int ar0144_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar0144 *sensor;
	unsigned int i;
	u64 chip_id;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;

	/* Acquire resources. */
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "Unable to initialize I2C\n");

	sensor->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->clk))
		return dev_err_probe(dev, PTR_ERR(sensor->clk),
				     "Cannot get clock\n");

	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "Cannot get reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = ar0144_supply_name[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot get supplies\n");

	/* Parse the DT properties. */
	ret = ar0144_parse_dt(sensor);
	if (ret)
		return ret;

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = ar0144_power_on(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Could not power on the device\n");
		goto err_dt;
	}

	ret = cci_read(sensor->regmap, AR0144_CHIP_VERSION_REG, &chip_id, NULL);
	if (ret) {
		dev_err(dev, "Could not read chip ID: %d\n", ret);
		goto err_power;
	}

	if (chip_id != AR0144_CHIP_VERSION) {
		dev_err(dev, "Wrong chip version 0x%04x (expected 0x%04x)\n",
			(u16)chip_id, AR0144_CHIP_VERSION);
		ret = -ENODEV;
		goto err_power;
	}

	if (IS_ENABLED(CONFIG_DYNAMIC_DEBUG) || IS_ENABLED(DEBUG)) {
		cci_read(sensor->regmap, AR0144_REVISION_NUMBER, &chip_id, NULL);
		dev_dbg(dev, "Sensor detected, OTPM r%u, silicon r%u, CREV r%u\n",
			(u32)AR0144_REVISION_NUMBER_OTPM(chip_id),
			(u32)AR0144_REVISION_NUMBER_SILICON(chip_id),
			(u32)AR0144_REVISION_NUMBER_CREV(chip_id));
	}

	/*
	 * Enable runtime PM with autosuspend. As the device has been powered
	 * manually, mark it as active, and increase the usage count without
	 * resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	/* Initialize the subdev. */
	v4l2_i2c_subdev_init(&sensor->sd, client, &ar0144_subdev_ops);
	sensor->sd.internal_ops = &ar0144_subdev_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.dev = &client->dev;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0) {
		dev_err(dev, "Could not register media entity\n");
		goto err_pm;
	}

	ret = ar0144_ctrl_init(sensor);
	if (ret < 0) {
		dev_err(dev, "Control initialization error %d\n", ret);
		goto err_entity;
	}

	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret < 0) {
		dev_err(dev, "Subdev initialization error %d\n", ret);
		goto err_ctrls;
	}

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret < 0) {
		dev_err(dev, "Could not register v4l2 device\n");
		goto err_subdev;
	}

	/*
	 * Decrease the PM usage count. The device will get suspended after the
	 * autosuspend delay, turning the power off.
	 */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_subdev:
	v4l2_subdev_cleanup(&sensor->sd);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
err_power:
	ar0144_power_off(sensor);
err_dt:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static void ar0144_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144 *sensor = to_ar0144(sd);

	v4l2_subdev_cleanup(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		ar0144_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct of_device_id ar0144_of_match[] = {
	{ .compatible = "onnn,ar0144" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);

static struct i2c_driver ar0144_i2c_driver = {
	.driver = {
		.name  = "ar0144",
		.of_match_table = of_match_ptr(ar0144_of_match),
		.pm = pm_ptr(&ar0144_pm_ops),
	},
	.probe  = ar0144_probe,
	.remove = ar0144_remove,
};

module_i2c_driver(ar0144_i2c_driver);

MODULE_DESCRIPTION("onsemi AR0144 Camera Sensor");
MODULE_LICENSE("GPL");
