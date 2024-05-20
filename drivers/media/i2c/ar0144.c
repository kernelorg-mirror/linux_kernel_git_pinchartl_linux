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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AR0144_ID_VAL        0x1356

#define AR0144_CHIP_VERSION_REG					CCI_REG16(0x3000)
#define AR0144_Y_ADDR_START					CCI_REG16(0x3002)
#define AR0144_X_ADDR_START					CCI_REG16(0x3004)
#define AR0144_Y_ADDR_END					CCI_REG16(0x3006)
#define AR0144_X_ADDR_END					CCI_REG16(0x3008)
#define AR0144_FRAME_LENGTH_LINES				CCI_REG16(0x300a)
#define AR0144_LINE_LENGTH_PCK					CCI_REG16(0x300c)
#define AR0144_REVISION_NUMBER					CCI_REG16(0x300e)
#define AR0144_LOCK_CONTROL					CCI_REG16(0x3010)
#define AR0144_COARSE_INTEGRATION_TIME				CCI_REG16(0x3012)
#define AR0144_FINE_INTEGRATION_TIME				CCI_REG16(0x3014)
#define AR0144_COARSE_INTEGRATION_TIME_CB			CCI_REG16(0x3016)
#define AR0144_FINE_INTEGRATION_TIME_CB				CCI_REG16(0x3018)
#define AR0144_RESET_REGISTER					CCI_REG16(0x301a)
#define AR0144_MODE_SELECT					CCI_REG8(0x301c)
#define AR0144_IMAGE_ORIENTATION				CCI_REG8(0x301d)
#define AR0144_DATA_PEDESTAL					CCI_REG16(0x301e)
#define AR0144_SOFTWARE_RESET					CCI_REG8(0x3021)
#define AR0144_GROUPED_PARAMETER_HOLD				CCI_REG8(0x3022)
#define AR0144_MASK_CORRUPTED_FRAMES				CCI_REG8(0x3023)
#define AR0144_PIXEL_ORDER					CCI_REG8(0x3024)
#define AR0144_GPI_STATUS					CCI_REG16(0x3026)
#define AR0144_ROW_SPEED					CCI_REG16(0x3028)
#define AR0144_VT_PIX_CLK_DIV					CCI_REG16(0x302a)
#define AR0144_VT_SYS_CLK_DIV					CCI_REG16(0x302c)
#define AR0144_PRE_PLL_CLK_DIV					CCI_REG16(0x302e)
#define AR0144_PLL_MULTIPLIER					CCI_REG16(0x3030)
#define AR0144_CTX_CONTROL_REG					CCI_REG16(0x3034)
#define AR0144_OP_PIX_CLK_DIV					CCI_REG16(0x3036)
#define AR0144_OP_SYS_CLK_DIV					CCI_REG16(0x3038)
#define AR0144_FRAME_COUNT					CCI_REG16(0x303a)
#define AR0144_FRAME_STATUS					CCI_REG16(0x303c)
#define AR0144_LINE_LENGTH_PCK_CB				CCI_REG16(0x303e)
#define AR0144_READ_MODE					CCI_REG16(0x3040)
#define AR0144_EXTRA_DELAY					CCI_REG16(0x3042)
#define AR0144_GREEN1_GAIN					CCI_REG16(0x3056)
#define AR0144_BLUE_GAIN					CCI_REG16(0x3058)
#define AR0144_RED_GAIN						CCI_REG16(0x305a)
#define AR0144_GREEN2_GAIN					CCI_REG16(0x305c)
#define AR0144_GLOBAL_GAIN					CCI_REG16(0x305e)
#define AR0144_ANALOG_GAIN					CCI_REG16(0x3060)
#define AR0144_SMIA_TEST					CCI_REG16(0x3064)
#define AR0144_CTX_WR_DATA_REG					CCI_REG16(0x3066)
#define AR0144_CTX_RD_DATA_REG					CCI_REG16(0x3068)
#define AR0144_DATAPATH_SELECT					CCI_REG16(0x306e)
#define AR0144_TEST_PATTERN_MODE				CCI_REG16(0x3070)
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
#define AR0144_Y_EVEN_INC					CCI_REG16(0x30a4)
#define AR0144_Y_ODD_INC					CCI_REG16(0x30a6)
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
#define AR0144_DATA_FORMAT_BITS					CCI_REG16(0x31ac)
#define AR0144_SERIAL_FORMAT					CCI_REG16(0x31ae)
#define AR0144_FRAME_PREAMBLE					CCI_REG16(0x31b0)
#define AR0144_LINE_PREAMBLE					CCI_REG16(0x31b2)
#define AR0144_MIPI_TIMING_0					CCI_REG16(0x31b4)
#define AR0144_MIPI_TIMING_1					CCI_REG16(0x31b6)
#define AR0144_MIPI_TIMING_2					CCI_REG16(0x31b8)
#define AR0144_MIPI_TIMING_3					CCI_REG16(0x31ba)
#define AR0144_MIPI_TIMING_4					CCI_REG16(0x31bc)
#define AR0144_SERIAL_CONFIG_STATUS				CCI_REG16(0x31be)
#define AR0144_SERIAL_CONTROL_STATUS				CCI_REG16(0x31c6)
#define AR0144_SERIAL_CRC_0					CCI_REG16(0x31c8)
#define AR0144_COMPANDING					CCI_REG16(0x31d0)
#define AR0144_STAT_FRAME_ID					CCI_REG16(0x31d2)
#define AR0144_I2C_WRT_CHEKCSUM					CCI_REG16(0x31d6)
#define AR0144_SERIAL_TEST					CCI_REG16(0x31d8)
#define AR0144_PIX_DEF_ID					CCI_REG16(0x31e0)
#define AR0144_HORIZONTAL_CURSOR_POSITION			CCI_REG16(0x31e8)
#define AR0144_VERTICAL_CURSOR_POSITION				CCI_REG16(0x31ea)
#define AR0144_HORIZONTAL_CURSOR_WIDTH				CCI_REG16(0x31ec)
#define AR0144_VERTICAL_CURSOR_WIDTH				CCI_REG16(0x31ee)
#define AR0144_CCI_IDS						CCI_REG16(0x31fc)
#define AR0144_CUSTOMER_REV					CCI_REG16(0x31fe)
#define AR0144_LED_FLASH_CONTROL				CCI_REG16(0x3270)
#define AR0144_MIPI_TEST_CNTRL					CCI_REG16(0x3338)
#define AR0144_MIPI_COMPRESS_8_DATA_TYPE			CCI_REG16(0x333a)
#define AR0144_MIPI_COMPRESS_7_DATA_TYPE			CCI_REG16(0x333c)
#define AR0144_MIPI_COMPRESS_6_DATA_TYPE			CCI_REG16(0x333e)
#define AR0144_MIPI_JPEG_PN9_DATA_TYPE				CCI_REG16(0x3340)
#define AR0144_MIPI_CNTRL					CCI_REG16(0x3354)
#define AR0144_MIPI_TEST_PATTERN_CNTRL				CCI_REG16(0x3356)
#define AR0144_MIPI_TEST_PATTERN_STATUS				CCI_REG16(0x3358)
#define AR0144_DIGITAL_CTRL_1					CCI_REG16(0x3786)

struct ar0144_reg_value {
	u32 reg;
	u32 val;
};

struct ar0144 {
	struct device *dev;
	struct regmap *regmap;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	struct gpio_desc *rst_gpio;
	struct mutex lock;

	bool streaming;
};

static inline struct ar0144 *to_ar0144(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0144, sd);
}

static const struct ar0144_reg_value ar0144at_rev4_optimized_sequencer[] = {
	{ AR0144_SEQ_CTRL_PORT, AR0144_SEQUENCER_STOPPED | AR0144_ACCESS_ADDRESS(0) },
	{ AR0144_SEQ_DATA_PORT, 0x327f },
	{ AR0144_SEQ_DATA_PORT, 0x5780 },
	{ AR0144_SEQ_DATA_PORT, 0x2730 },
	{ AR0144_SEQ_DATA_PORT, 0x7e13 },
	{ AR0144_SEQ_DATA_PORT, 0x8000 },
	{ AR0144_SEQ_DATA_PORT, 0x157e },
	{ AR0144_SEQ_DATA_PORT, 0x1380 },
	{ AR0144_SEQ_DATA_PORT, 0x000f },
	{ AR0144_SEQ_DATA_PORT, 0x8190 },
	{ AR0144_SEQ_DATA_PORT, 0x1643 },
	{ AR0144_SEQ_DATA_PORT, 0x163e },
	{ AR0144_SEQ_DATA_PORT, 0x4522 },
	{ AR0144_SEQ_DATA_PORT, 0x0937 },
	{ AR0144_SEQ_DATA_PORT, 0x8190 },
	{ AR0144_SEQ_DATA_PORT, 0x1643 },
	{ AR0144_SEQ_DATA_PORT, 0x167f },
	{ AR0144_SEQ_DATA_PORT, 0x9080 },
	{ AR0144_SEQ_DATA_PORT, 0x0038 },
	{ AR0144_SEQ_DATA_PORT, 0x7f13 },
	{ AR0144_SEQ_DATA_PORT, 0x8023 },
	{ AR0144_SEQ_DATA_PORT, 0x3b7f },
	{ AR0144_SEQ_DATA_PORT, 0x9345 },
	{ AR0144_SEQ_DATA_PORT, 0x0280 },
	{ AR0144_SEQ_DATA_PORT, 0x007f },
	{ AR0144_SEQ_DATA_PORT, 0xb08d },
	{ AR0144_SEQ_DATA_PORT, 0x667f },
	{ AR0144_SEQ_DATA_PORT, 0x9081 },
	{ AR0144_SEQ_DATA_PORT, 0x923c },
	{ AR0144_SEQ_DATA_PORT, 0x1635 },
	{ AR0144_SEQ_DATA_PORT, 0x7f93 },
	{ AR0144_SEQ_DATA_PORT, 0x4502 },
	{ AR0144_SEQ_DATA_PORT, 0x8000 },
	{ AR0144_SEQ_DATA_PORT, 0x7fb0 },
	{ AR0144_SEQ_DATA_PORT, 0x8d66 },
	{ AR0144_SEQ_DATA_PORT, 0x7f90 },
	{ AR0144_SEQ_DATA_PORT, 0x8182 },
	{ AR0144_SEQ_DATA_PORT, 0x3745 },
	{ AR0144_SEQ_DATA_PORT, 0x0236 },
	{ AR0144_SEQ_DATA_PORT, 0x8180 },
	{ AR0144_SEQ_DATA_PORT, 0x4416 },
	{ AR0144_SEQ_DATA_PORT, 0x3143 },
	{ AR0144_SEQ_DATA_PORT, 0x7416 },
	{ AR0144_SEQ_DATA_PORT, 0x787b },
	{ AR0144_SEQ_DATA_PORT, 0x7d45 },
	{ AR0144_SEQ_DATA_PORT, 0x023d },
	{ AR0144_SEQ_DATA_PORT, 0x6445 },
	{ AR0144_SEQ_DATA_PORT, 0x0a3d },
	{ AR0144_SEQ_DATA_PORT, 0x647e },
	{ AR0144_SEQ_DATA_PORT, 0x1281 },
	{ AR0144_SEQ_DATA_PORT, 0x8037 },
	{ AR0144_SEQ_DATA_PORT, 0x7f10 },
	{ AR0144_SEQ_DATA_PORT, 0x450a },
	{ AR0144_SEQ_DATA_PORT, 0x3f74 },
	{ AR0144_SEQ_DATA_PORT, 0x7e10 },
	{ AR0144_SEQ_DATA_PORT, 0x7e12 },
	{ AR0144_SEQ_DATA_PORT, 0x0f3d },
	{ AR0144_SEQ_DATA_PORT, 0xd27f },
	{ AR0144_SEQ_DATA_PORT, 0xd480 },
	{ AR0144_SEQ_DATA_PORT, 0x2482 },
	{ AR0144_SEQ_DATA_PORT, 0x9c03 },
	{ AR0144_SEQ_DATA_PORT, 0x430d },
	{ AR0144_SEQ_DATA_PORT, 0x2d46 },
	{ AR0144_SEQ_DATA_PORT, 0x4316 },
	{ AR0144_SEQ_DATA_PORT, 0x5f16 },
	{ AR0144_SEQ_DATA_PORT, 0x532d },
	{ AR0144_SEQ_DATA_PORT, 0x1660 },
	{ AR0144_SEQ_DATA_PORT, 0x404c },
	{ AR0144_SEQ_DATA_PORT, 0x2904 },
	{ AR0144_SEQ_DATA_PORT, 0x2984 },
	{ AR0144_SEQ_DATA_PORT, 0x81e7 },
	{ AR0144_SEQ_DATA_PORT, 0x816f },
	{ AR0144_SEQ_DATA_PORT, 0x170a },
	{ AR0144_SEQ_DATA_PORT, 0x81e7 },
	{ AR0144_SEQ_DATA_PORT, 0x7f81 },
	{ AR0144_SEQ_DATA_PORT, 0x5c0d },
	{ AR0144_SEQ_DATA_PORT, 0x5749 },
	{ AR0144_SEQ_DATA_PORT, 0x5f53 },
	{ AR0144_SEQ_DATA_PORT, 0x2553 },
	{ AR0144_SEQ_DATA_PORT, 0x274d },
	{ AR0144_SEQ_DATA_PORT, 0x2bf8 },
	{ AR0144_SEQ_DATA_PORT, 0x1016 },
	{ AR0144_SEQ_DATA_PORT, 0x4c09 },
	{ AR0144_SEQ_DATA_PORT, 0x2bb8 },
	{ AR0144_SEQ_DATA_PORT, 0x2b98 },
	{ AR0144_SEQ_DATA_PORT, 0x4e11 },
	{ AR0144_SEQ_DATA_PORT, 0x5367 },
	{ AR0144_SEQ_DATA_PORT, 0x4001 },
	{ AR0144_SEQ_DATA_PORT, 0x605c },
	{ AR0144_SEQ_DATA_PORT, 0x095c },
	{ AR0144_SEQ_DATA_PORT, 0x1b40 },
	{ AR0144_SEQ_DATA_PORT, 0x0245 },
	{ AR0144_SEQ_DATA_PORT, 0x0045 },
	{ AR0144_SEQ_DATA_PORT, 0x8029 },
	{ AR0144_SEQ_DATA_PORT, 0xb67f },
	{ AR0144_SEQ_DATA_PORT, 0x8040 },
	{ AR0144_SEQ_DATA_PORT, 0x047f },
	{ AR0144_SEQ_DATA_PORT, 0x8841 },
	{ AR0144_SEQ_DATA_PORT, 0x095c },
	{ AR0144_SEQ_DATA_PORT, 0x0b29 },
	{ AR0144_SEQ_DATA_PORT, 0xb241 },
	{ AR0144_SEQ_DATA_PORT, 0x0c40 },
	{ AR0144_SEQ_DATA_PORT, 0x0340 },
	{ AR0144_SEQ_DATA_PORT, 0x135c },
	{ AR0144_SEQ_DATA_PORT, 0x0341 },
	{ AR0144_SEQ_DATA_PORT, 0x1117 },
	{ AR0144_SEQ_DATA_PORT, 0x125f },
	{ AR0144_SEQ_DATA_PORT, 0x2b90 },
	{ AR0144_SEQ_DATA_PORT, 0x2b80 },
	{ AR0144_SEQ_DATA_PORT, 0x816f },
	{ AR0144_SEQ_DATA_PORT, 0x4010 },
	{ AR0144_SEQ_DATA_PORT, 0x4101 },
	{ AR0144_SEQ_DATA_PORT, 0x5327 },
	{ AR0144_SEQ_DATA_PORT, 0x4001 },
	{ AR0144_SEQ_DATA_PORT, 0x6029 },
	{ AR0144_SEQ_DATA_PORT, 0xa35f },
	{ AR0144_SEQ_DATA_PORT, 0x4d1c },
	{ AR0144_SEQ_DATA_PORT, 0x1702 },
	{ AR0144_SEQ_DATA_PORT, 0x81e7 },
	{ AR0144_SEQ_DATA_PORT, 0x2983 },
	{ AR0144_SEQ_DATA_PORT, 0x4588 },
	{ AR0144_SEQ_DATA_PORT, 0x4021 },
	{ AR0144_SEQ_DATA_PORT, 0x7f8a },
	{ AR0144_SEQ_DATA_PORT, 0x4039 },
	{ AR0144_SEQ_DATA_PORT, 0x4580 },
	{ AR0144_SEQ_DATA_PORT, 0x2440 },
	{ AR0144_SEQ_DATA_PORT, 0x087f },
	{ AR0144_SEQ_DATA_PORT, 0x885d },
	{ AR0144_SEQ_DATA_PORT, 0x5367 },
	{ AR0144_SEQ_DATA_PORT, 0x2992 },
	{ AR0144_SEQ_DATA_PORT, 0x8810 },
	{ AR0144_SEQ_DATA_PORT, 0x2b04 },
	{ AR0144_SEQ_DATA_PORT, 0x8916 },
	{ AR0144_SEQ_DATA_PORT, 0x5c43 },
	{ AR0144_SEQ_DATA_PORT, 0x8617 },
	{ AR0144_SEQ_DATA_PORT, 0x0b5c },
	{ AR0144_SEQ_DATA_PORT, 0x038a },
	{ AR0144_SEQ_DATA_PORT, 0x484d },
	{ AR0144_SEQ_DATA_PORT, 0x4e2b },
	{ AR0144_SEQ_DATA_PORT, 0x804c },
	{ AR0144_SEQ_DATA_PORT, 0x0b41 },
	{ AR0144_SEQ_DATA_PORT, 0x9f81 },
	{ AR0144_SEQ_DATA_PORT, 0x6f41 },
	{ AR0144_SEQ_DATA_PORT, 0x1040 },
	{ AR0144_SEQ_DATA_PORT, 0x0153 },
	{ AR0144_SEQ_DATA_PORT, 0x2740 },
	{ AR0144_SEQ_DATA_PORT, 0x0160 },
	{ AR0144_SEQ_DATA_PORT, 0x2983 },
	{ AR0144_SEQ_DATA_PORT, 0x2943 },
	{ AR0144_SEQ_DATA_PORT, 0x5c05 },
	{ AR0144_SEQ_DATA_PORT, 0x5f4d },
	{ AR0144_SEQ_DATA_PORT, 0x1c81 },
	{ AR0144_SEQ_DATA_PORT, 0xe745 },
	{ AR0144_SEQ_DATA_PORT, 0x0281 },
	{ AR0144_SEQ_DATA_PORT, 0x807f },
	{ AR0144_SEQ_DATA_PORT, 0x8041 },
	{ AR0144_SEQ_DATA_PORT, 0x0a91 },
	{ AR0144_SEQ_DATA_PORT, 0x4416 },
	{ AR0144_SEQ_DATA_PORT, 0x092f },
	{ AR0144_SEQ_DATA_PORT, 0x7e37 },
	{ AR0144_SEQ_DATA_PORT, 0x8020 },
	{ AR0144_SEQ_DATA_PORT, 0x307e },
	{ AR0144_SEQ_DATA_PORT, 0x3780 },
	{ AR0144_SEQ_DATA_PORT, 0x2015 },
	{ AR0144_SEQ_DATA_PORT, 0x7e37 },
	{ AR0144_SEQ_DATA_PORT, 0x8020 },
	{ AR0144_SEQ_DATA_PORT, 0x0343 },
	{ AR0144_SEQ_DATA_PORT, 0x164a },
	{ AR0144_SEQ_DATA_PORT, 0x0a43 },
	{ AR0144_SEQ_DATA_PORT, 0x160b },
	{ AR0144_SEQ_DATA_PORT, 0x4316 },
	{ AR0144_SEQ_DATA_PORT, 0x8f43 },
	{ AR0144_SEQ_DATA_PORT, 0x1690 },
	{ AR0144_SEQ_DATA_PORT, 0x4316 },
	{ AR0144_SEQ_DATA_PORT, 0x7f81 },
	{ AR0144_SEQ_DATA_PORT, 0x450a },
	{ AR0144_SEQ_DATA_PORT, 0x4130 },
	{ AR0144_SEQ_DATA_PORT, 0x7f83 },
	{ AR0144_SEQ_DATA_PORT, 0x5d29 },
	{ AR0144_SEQ_DATA_PORT, 0x4488 },
	{ AR0144_SEQ_DATA_PORT, 0x102b },
	{ AR0144_SEQ_DATA_PORT, 0x0453 },
	{ AR0144_SEQ_DATA_PORT, 0x2d40 },
	{ AR0144_SEQ_DATA_PORT, 0x3045 },
	{ AR0144_SEQ_DATA_PORT, 0x0240 },
	{ AR0144_SEQ_DATA_PORT, 0x087f },
	{ AR0144_SEQ_DATA_PORT, 0x8053 },
	{ AR0144_SEQ_DATA_PORT, 0x2d89 },
	{ AR0144_SEQ_DATA_PORT, 0x165c },
	{ AR0144_SEQ_DATA_PORT, 0x4586 },
	{ AR0144_SEQ_DATA_PORT, 0x170b },
	{ AR0144_SEQ_DATA_PORT, 0x5c05 },
	{ AR0144_SEQ_DATA_PORT, 0x8a60 },
	{ AR0144_SEQ_DATA_PORT, 0x4b91 },
	{ AR0144_SEQ_DATA_PORT, 0x4416 },
	{ AR0144_SEQ_DATA_PORT, 0x0915 },
	{ AR0144_SEQ_DATA_PORT, 0x3dff },
	{ AR0144_SEQ_DATA_PORT, 0x3d87 },
	{ AR0144_SEQ_DATA_PORT, 0x7e3d },
	{ AR0144_SEQ_DATA_PORT, 0x7e19 },
	{ AR0144_SEQ_DATA_PORT, 0x8000 },
	{ AR0144_SEQ_DATA_PORT, 0x8b1f },
	{ AR0144_SEQ_DATA_PORT, 0x2a1f },
	{ AR0144_SEQ_DATA_PORT, 0x83a2 },
	{ AR0144_SEQ_DATA_PORT, 0x7e11 },
	{ AR0144_SEQ_DATA_PORT, 0x7516 },
	{ AR0144_SEQ_DATA_PORT, 0x3345 },
	{ AR0144_SEQ_DATA_PORT, 0x0a7f },
	{ AR0144_SEQ_DATA_PORT, 0x5380 },
	{ AR0144_SEQ_DATA_PORT, 0x238c },
	{ AR0144_SEQ_DATA_PORT, 0x667f },
	{ AR0144_SEQ_DATA_PORT, 0x1381 },
	{ AR0144_SEQ_DATA_PORT, 0x8414 },
	{ AR0144_SEQ_DATA_PORT, 0x8180 },
	{ AR0144_SEQ_DATA_PORT, 0x313d },
	{ AR0144_SEQ_DATA_PORT, 0x6445 },
	{ AR0144_SEQ_DATA_PORT, 0x2a3d },
	{ AR0144_SEQ_DATA_PORT, 0xd27f },
	{ AR0144_SEQ_DATA_PORT, 0x4480 },
	{ AR0144_SEQ_DATA_PORT, 0x2494 },
	{ AR0144_SEQ_DATA_PORT, 0x3dff },
	{ AR0144_SEQ_DATA_PORT, 0x3d4d },
	{ AR0144_SEQ_DATA_PORT, 0x4502 },
	{ AR0144_SEQ_DATA_PORT, 0x7fd0 },
	{ AR0144_SEQ_DATA_PORT, 0x8000 },
	{ AR0144_SEQ_DATA_PORT, 0x8c66 },
	{ AR0144_SEQ_DATA_PORT, 0x7f90 },
	{ AR0144_SEQ_DATA_PORT, 0x8194 },
	{ AR0144_SEQ_DATA_PORT, 0x3f44 },
	{ AR0144_SEQ_DATA_PORT, 0x1681 },
	{ AR0144_SEQ_DATA_PORT, 0x8416 },
	{ AR0144_SEQ_DATA_PORT, 0x2c2c },
	{ AR0144_SEQ_DATA_PORT, 0x2c2c },
};

static const struct ar0144_reg_value ar0144at_rev4_recommended_setting[] = {
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
	{ AR0144_ANALOG_GAIN, 0x000d },
	{ CCI_REG16(0x3092), 0x00cf },
	{ CCI_REG16(0x3268), 0x0030 },
	{ AR0144_DIGITAL_CTRL_1, 0x0060 },
	{ CCI_REG16(0x3f4a), 0x0f70 },
	{ AR0144_DATAPATH_SELECT, 0x4810 },
	{ AR0144_SMIA_TEST, 0x1802 },
	{ CCI_REG16(0x3ef6), 0x804d },
	{ AR0144_DELTA_DK_CONTROL, 0xc08f },
	{ CCI_REG16(0x30ba), 0x7623 },
	{ CCI_REG16(0x3176), 0x0480 },
	{ CCI_REG16(0x3178), 0x0480 },
	{ CCI_REG16(0x317a), 0x0480 },
	{ CCI_REG16(0x317c), 0x0480 },
};

static const struct ar0144_reg_value ar0144at_pll_27mhz[] = {
	{ AR0144_VT_PIX_CLK_DIV, 0x0006 },
	{ AR0144_VT_SYS_CLK_DIV, 0x0001 },
	{ AR0144_PRE_PLL_CLK_DIV, 0x0004 },
	{ AR0144_PLL_MULTIPLIER, 0x4a /*0x0042*/ },
	{ AR0144_OP_PIX_CLK_DIV, 0x000c },
	{ AR0144_OP_SYS_CLK_DIV, 0x0001 },
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

static const struct ar0144_reg_value ar0144at_mipi_2lane_12bit[] = {
	{ AR0144_SERIAL_FORMAT, 0x0202 },
	{ AR0144_DATA_FORMAT_BITS, 0x0c0c },
	{ AR0144_FRAME_PREAMBLE, 0x0042 },
	{ AR0144_LINE_PREAMBLE, 0x002e },
	{ AR0144_MIPI_TIMING_0, 0x1665 },
	{ AR0144_MIPI_TIMING_1, 0x110e },
	{ AR0144_MIPI_TIMING_2, 0x2047 },
	{ AR0144_MIPI_TIMING_3, 0x0105 },
	{ AR0144_MIPI_TIMING_4, 0x0004 },
};

static const struct ar0144_reg_value ar0144at_1280x800_60fps[] = {
	{ AR0144_Y_ADDR_START, 0x0000 },
	{ AR0144_X_ADDR_START, 0x0004 },
	{ AR0144_Y_ADDR_END, 0x031f },
	{ AR0144_X_ADDR_END, 0x0503 },
	{ AR0144_FRAME_LENGTH_LINES, 0x0339 },
	{ AR0144_LINE_LENGTH_PCK, 0x05d0 },
	{ AR0144_COARSE_INTEGRATION_TIME, 0x0064 },
	{ AR0144_X_ODD_INC, 0x0001 },
	{ AR0144_Y_ODD_INC, 0x0001 },
	{ AR0144_READ_MODE, 0x0000 },
};

static const struct ar0144_reg_value ar0144at_context_b_2x2_binning[] = {
	{ AR0144_READ_MODE, 0x1000 },
	{ AR0144_Y_ODD_INC_CB, 0x0003 },
	{ AR0144_READ_MODE, 0x3000 },
	{ AR0144_X_ODD_INC_CB, 0x0003 },
};

static const struct ar0144_reg_value ar0144at_embedded_data_stats[] = {
	{ AR0144_SMIA_TEST, 0x1982 },
};

static const struct ar0144_reg_value ar0144at_start_stream[] = {
	{ AR0144_ROW_SPEED, 0x0010 },
	{ AR0144_RESET_REGISTER, 0x005c },
};

static const struct ar0144_reg_value ar0144at_stop_stream[] = {
	{ AR0144_RESET_REGISTER, 0x0058 },
};

static int ar0144_set_register_array(struct ar0144 *ar0144,
				     const struct ar0144_reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < num_settings; ++i, ++settings)
		cci_write(ar0144->regmap, settings->reg, settings->val, &ret);

	return ret;
}

static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	u64 reg_val;
	int ret = 0;

	mutex_lock(&ar0144->lock);

	gpiod_direction_output(ar0144->rst_gpio, 1);
	if (!on)
		goto out;
	msleep(2); /* more than 1ms */
	gpiod_set_value_cansleep(ar0144->rst_gpio, 0);
	msleep(10); /* more than 160000 clocks at 24MHz; FIXME: use clk rate */

	ret = cci_read(ar0144->regmap, AR0144_CHIP_VERSION_REG, &reg_val, NULL);
	if (ret < 0)
		goto out;
	if (reg_val != AR0144_ID_VAL) {
		dev_err(ar0144->dev,
			"wrong chip id (0x%04x), expected 0x%04x\n", (u16)reg_val,
			AR0144_ID_VAL);
		ret = -ENODEV;
		goto out;
	}

	ret = ar0144_set_register_array(
		ar0144, ar0144at_rev4_optimized_sequencer,
		ARRAY_SIZE(ar0144at_rev4_optimized_sequencer));
	if (ret < 0)
		goto out;
	ret = ar0144_set_register_array(
		ar0144, ar0144at_rev4_recommended_setting,
		ARRAY_SIZE(ar0144at_rev4_recommended_setting));

out:
	mutex_unlock(&ar0144->lock);
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

	fse->min_width = 1280;
	fse->max_width = 1280;
	fse->min_height = 800;
	fse->max_height = 800;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar0144_get_pad_format(struct ar0144 *ar0144,
			struct v4l2_subdev_state *state,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0144->fmt;
	default:
		return NULL;
	}
}

static int ar0144_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct ar0144 *ar0144 = to_ar0144(sd);

	mutex_lock(&ar0144->lock);
	format->format = *__ar0144_get_pad_format(ar0144, state, format->pad,
						  format->which);
	mutex_unlock(&ar0144->lock);
	return 0;
}

static struct v4l2_rect *
__ar0144_get_pad_crop(struct ar0144 *ar0144, struct v4l2_subdev_state *state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ar0144->crop;
	default:
		return NULL;
	}
}

static int ar0144_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	struct v4l2_mbus_framefmt *__format;

	mutex_lock(&ar0144->lock);

	__format = __ar0144_get_pad_format(ar0144, state, format->pad,
			format->which);
	__format->width = 1280;
	__format->height = 800;
	__format->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	mutex_unlock(&ar0144->lock);
	return 0;
}

static int ar0144_entity_init_state(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;

	ar0144_set_format(subdev, state, &fmt);

	return 0;
}

static int ar0144_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_selection *sel)
{
	struct ar0144 *ar0144 = to_ar0144(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__ar0144_get_pad_crop(ar0144, state, sel->pad,
					sel->which);
	return 0;
}

static int ar0144_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ar0144 *ar0144 = to_ar0144(subdev);
	int ret;
	u64 reg_val;

	mutex_lock(&ar0144->lock);

	if (enable == 0) {
		ret = cci_read(ar0144->regmap, AR0144_FRAME_COUNT, &reg_val, NULL);
		if (ret == 0)
			printk("%s: FRAME_COUNT: %u\n", __func__, (u16)reg_val);
		ret = cci_read(ar0144->regmap, AR0144_FRAME_STATUS, &reg_val, NULL);
		if (ret == 0)
			printk("%s: FRAME_STATUS: %u\n", __func__, (u16)reg_val);
		ret = ar0144_set_register_array(
			ar0144, ar0144at_stop_stream,
			ARRAY_SIZE(ar0144at_stop_stream));
		goto out;
	}

	ret = ar0144_set_register_array(ar0144, ar0144at_pll_27mhz,
					ARRAY_SIZE(ar0144at_pll_27mhz));
	if (ret < 0)
		goto out;
	msleep(100);

	ret = ar0144_set_register_array(ar0144, ar0144at_mipi_2lane_12bit,
					ARRAY_SIZE(ar0144at_mipi_2lane_12bit));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(ar0144, ar0144at_1280x800_60fps,
					ARRAY_SIZE(ar0144at_1280x800_60fps));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(
		ar0144, ar0144at_context_b_2x2_binning,
		ARRAY_SIZE(ar0144at_context_b_2x2_binning));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(
		ar0144, ar0144at_embedded_data_stats,
		ARRAY_SIZE(ar0144at_embedded_data_stats));
	if (ret < 0)
		goto out;

	ret = ar0144_set_register_array(ar0144, ar0144at_start_stream,
					ARRAY_SIZE(ar0144at_start_stream));

	msleep(100);
	ret = cci_read(ar0144->regmap, AR0144_FRAME_COUNT, &reg_val, NULL);
	if (ret == 0)
		printk("%s: FRAME_COUNT: %u\n", __func__, (u16)reg_val);
	ret = cci_read(ar0144->regmap, AR0144_FRAME_STATUS, &reg_val, NULL);
	if (ret == 0)
		printk("%s: FRAME_STATUS: %u\n", __func__, (u16)reg_val);

out:
	mutex_unlock(&ar0144->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops ar0144_core_ops = {
	.s_power = ar0144_s_power,
};

static const struct v4l2_subdev_video_ops ar0144_video_ops = {
	.s_stream = ar0144_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0144_subdev_pad_ops = {
	.enum_mbus_code = ar0144_enum_mbus_code,
	.enum_frame_size = ar0144_enum_frame_size,
	.get_fmt = ar0144_get_format,
	.set_fmt = ar0144_set_format,
	.get_selection = ar0144_get_selection,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.core = &ar0144_core_ops,
	.video = &ar0144_video_ops,
	.pad = &ar0144_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar0144_subdev_internal_ops = {
	.init_state = ar0144_entity_init_state,
};

static int ar0144_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct ar0144 *ar0144;
	int ret;

	ar0144 = devm_kzalloc(dev, sizeof(struct ar0144), GFP_KERNEL);
	if (!ar0144)
		return -ENOMEM;

	ar0144->dev = dev;
	mutex_init(&ar0144->lock);

	ar0144->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar0144->regmap)) {
		dev_err(dev, "Unable to initialize I2C\n");
		return -ENODEV;
	}

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ar0144->ep);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	of_node_put(endpoint);

	if (ar0144->ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "invalid bus type, must be parallel\n");
		return -EINVAL;
	}

	ar0144->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ar0144->rst_gpio)) {
		if (PTR_ERR(ar0144->rst_gpio) != -EPROBE_DEFER)
			dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(ar0144->rst_gpio);
	}

	v4l2_i2c_subdev_init(&ar0144->sd, client, &ar0144_subdev_ops);
	ar0144->sd.internal_ops = &ar0144_subdev_internal_ops;
	ar0144->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0144->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ar0144->pad.flags = MEDIA_PAD_FL_SOURCE;
	ar0144->sd.dev = &client->dev;

	ret = media_entity_pads_init(&ar0144->sd.entity, 1, &ar0144->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		return ret;
	}

	ret = ar0144_s_power(&ar0144->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up AR0144\n");
		goto free_entity;
	}

	dev_info(dev, "AR0144 detected at address 0x%02x\n", client->addr);

	ret = v4l2_async_register_subdev(&ar0144->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	ar0144_entity_init_state(&ar0144->sd, NULL);

	return 0;

free_entity:
	media_entity_cleanup(&ar0144->sd.entity);

	return ret;
}

static void ar0144_remove(struct i2c_client *client)
{
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
	},
	.probe  = ar0144_probe,
	.remove = ar0144_remove,
};

module_i2c_driver(ar0144_i2c_driver);

MODULE_DESCRIPTION("onsemi AR0144 Camera Sensor");
MODULE_LICENSE("GPL");
