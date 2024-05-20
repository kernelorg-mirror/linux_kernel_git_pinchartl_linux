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
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define AR0144_I2C_ADDR      0x10
#define AR0144_ID_REG        0x3000
#define AR0144_ID_VAL        0x1356

struct ar0144_reg_value {
	u32 reg;
	u32 val;
};

struct ar0144 {
	struct i2c_client *i2c_client;
	struct device *dev;
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
	{0x3088, 0x8000},
	{0x3086, 0x327f},
	{0x3086, 0x5780},
	{0x3086, 0x2730},
	{0x3086, 0x7e13},
	{0x3086, 0x8000},
	{0x3086, 0x157e},
	{0x3086, 0x1380},
	{0x3086, 0x000f},
	{0x3086, 0x8190},
	{0x3086, 0x1643},
	{0x3086, 0x163e},
	{0x3086, 0x4522},
	{0x3086, 0x0937},
	{0x3086, 0x8190},
	{0x3086, 0x1643},
	{0x3086, 0x167f},
	{0x3086, 0x9080},
	{0x3086, 0x0038},
	{0x3086, 0x7f13},
	{0x3086, 0x8023},
	{0x3086, 0x3b7f},
	{0x3086, 0x9345},
	{0x3086, 0x0280},
	{0x3086, 0x007f},
	{0x3086, 0xb08d},
	{0x3086, 0x667f},
	{0x3086, 0x9081},
	{0x3086, 0x923c},
	{0x3086, 0x1635},
	{0x3086, 0x7f93},
	{0x3086, 0x4502},
	{0x3086, 0x8000},
	{0x3086, 0x7fb0},
	{0x3086, 0x8d66},
	{0x3086, 0x7f90},
	{0x3086, 0x8182},
	{0x3086, 0x3745},
	{0x3086, 0x0236},
	{0x3086, 0x8180},
	{0x3086, 0x4416},
	{0x3086, 0x3143},
	{0x3086, 0x7416},
	{0x3086, 0x787b},
	{0x3086, 0x7d45},
	{0x3086, 0x023d},
	{0x3086, 0x6445},
	{0x3086, 0x0a3d},
	{0x3086, 0x647e},
	{0x3086, 0x1281},
	{0x3086, 0x8037},
	{0x3086, 0x7f10},
	{0x3086, 0x450a},
	{0x3086, 0x3f74},
	{0x3086, 0x7e10},
	{0x3086, 0x7e12},
	{0x3086, 0x0f3d},
	{0x3086, 0xd27f},
	{0x3086, 0xd480},
	{0x3086, 0x2482},
	{0x3086, 0x9c03},
	{0x3086, 0x430d},
	{0x3086, 0x2d46},
	{0x3086, 0x4316},
	{0x3086, 0x5f16},
	{0x3086, 0x532d},
	{0x3086, 0x1660},
	{0x3086, 0x404c},
	{0x3086, 0x2904},
	{0x3086, 0x2984},
	{0x3086, 0x81e7},
	{0x3086, 0x816f},
	{0x3086, 0x170a},
	{0x3086, 0x81e7},
	{0x3086, 0x7f81},
	{0x3086, 0x5c0d},
	{0x3086, 0x5749},
	{0x3086, 0x5f53},
	{0x3086, 0x2553},
	{0x3086, 0x274d},
	{0x3086, 0x2bf8},
	{0x3086, 0x1016},
	{0x3086, 0x4c09},
	{0x3086, 0x2bb8},
	{0x3086, 0x2b98},
	{0x3086, 0x4e11},
	{0x3086, 0x5367},
	{0x3086, 0x4001},
	{0x3086, 0x605c},
	{0x3086, 0x095c},
	{0x3086, 0x1b40},
	{0x3086, 0x0245},
	{0x3086, 0x0045},
	{0x3086, 0x8029},
	{0x3086, 0xb67f},
	{0x3086, 0x8040},
	{0x3086, 0x047f},
	{0x3086, 0x8841},
	{0x3086, 0x095c},
	{0x3086, 0x0b29},
	{0x3086, 0xb241},
	{0x3086, 0x0c40},
	{0x3086, 0x0340},
	{0x3086, 0x135c},
	{0x3086, 0x0341},
	{0x3086, 0x1117},
	{0x3086, 0x125f},
	{0x3086, 0x2b90},
	{0x3086, 0x2b80},
	{0x3086, 0x816f},
	{0x3086, 0x4010},
	{0x3086, 0x4101},
	{0x3086, 0x5327},
	{0x3086, 0x4001},
	{0x3086, 0x6029},
	{0x3086, 0xa35f},
	{0x3086, 0x4d1c},
	{0x3086, 0x1702},
	{0x3086, 0x81e7},
	{0x3086, 0x2983},
	{0x3086, 0x4588},
	{0x3086, 0x4021},
	{0x3086, 0x7f8a},
	{0x3086, 0x4039},
	{0x3086, 0x4580},
	{0x3086, 0x2440},
	{0x3086, 0x087f},
	{0x3086, 0x885d},
	{0x3086, 0x5367},
	{0x3086, 0x2992},
	{0x3086, 0x8810},
	{0x3086, 0x2b04},
	{0x3086, 0x8916},
	{0x3086, 0x5c43},
	{0x3086, 0x8617},
	{0x3086, 0x0b5c},
	{0x3086, 0x038a},
	{0x3086, 0x484d},
	{0x3086, 0x4e2b},
	{0x3086, 0x804c},
	{0x3086, 0x0b41},
	{0x3086, 0x9f81},
	{0x3086, 0x6f41},
	{0x3086, 0x1040},
	{0x3086, 0x0153},
	{0x3086, 0x2740},
	{0x3086, 0x0160},
	{0x3086, 0x2983},
	{0x3086, 0x2943},
	{0x3086, 0x5c05},
	{0x3086, 0x5f4d},
	{0x3086, 0x1c81},
	{0x3086, 0xe745},
	{0x3086, 0x0281},
	{0x3086, 0x807f},
	{0x3086, 0x8041},
	{0x3086, 0x0a91},
	{0x3086, 0x4416},
	{0x3086, 0x092f},
	{0x3086, 0x7e37},
	{0x3086, 0x8020},
	{0x3086, 0x307e},
	{0x3086, 0x3780},
	{0x3086, 0x2015},
	{0x3086, 0x7e37},
	{0x3086, 0x8020},
	{0x3086, 0x0343},
	{0x3086, 0x164a},
	{0x3086, 0x0a43},
	{0x3086, 0x160b},
	{0x3086, 0x4316},
	{0x3086, 0x8f43},
	{0x3086, 0x1690},
	{0x3086, 0x4316},
	{0x3086, 0x7f81},
	{0x3086, 0x450a},
	{0x3086, 0x4130},
	{0x3086, 0x7f83},
	{0x3086, 0x5d29},
	{0x3086, 0x4488},
	{0x3086, 0x102b},
	{0x3086, 0x0453},
	{0x3086, 0x2d40},
	{0x3086, 0x3045},
	{0x3086, 0x0240},
	{0x3086, 0x087f},
	{0x3086, 0x8053},
	{0x3086, 0x2d89},
	{0x3086, 0x165c},
	{0x3086, 0x4586},
	{0x3086, 0x170b},
	{0x3086, 0x5c05},
	{0x3086, 0x8a60},
	{0x3086, 0x4b91},
	{0x3086, 0x4416},
	{0x3086, 0x0915},
	{0x3086, 0x3dff},
	{0x3086, 0x3d87},
	{0x3086, 0x7e3d},
	{0x3086, 0x7e19},
	{0x3086, 0x8000},
	{0x3086, 0x8b1f},
	{0x3086, 0x2a1f},
	{0x3086, 0x83a2},
	{0x3086, 0x7e11},
	{0x3086, 0x7516},
	{0x3086, 0x3345},
	{0x3086, 0x0a7f},
	{0x3086, 0x5380},
	{0x3086, 0x238c},
	{0x3086, 0x667f},
	{0x3086, 0x1381},
	{0x3086, 0x8414},
	{0x3086, 0x8180},
	{0x3086, 0x313d},
	{0x3086, 0x6445},
	{0x3086, 0x2a3d},
	{0x3086, 0xd27f},
	{0x3086, 0x4480},
	{0x3086, 0x2494},
	{0x3086, 0x3dff},
	{0x3086, 0x3d4d},
	{0x3086, 0x4502},
	{0x3086, 0x7fd0},
	{0x3086, 0x8000},
	{0x3086, 0x8c66},
	{0x3086, 0x7f90},
	{0x3086, 0x8194},
	{0x3086, 0x3f44},
	{0x3086, 0x1681},
	{0x3086, 0x8416},
	{0x3086, 0x2c2c},
	{0x3086, 0x2c2c},
};

static const struct ar0144_reg_value ar0144at_rev4_recommended_setting[] = {
	{0x3ed6, 0x3cb5},
	{0x3ed8, 0x8765},
	{0x3eda, 0x8888},
	{0x3edc, 0x97ff},
	{0x3ef8, 0x6522},
	{0x3efa, 0x2222},
	{0x3efc, 0x6666},
	{0x3f00, 0xaa05},
	{0x3ee2, 0x180e},
	{0x3ee4, 0x0808},
	{0x3eea, 0x2a09},
	{0x3060, 0x000d},
	{0x3092, 0x00cf},
	{0x3268, 0x0030},
	{0x3786, 0x0060},
	{0x3f4a, 0x0f70},
	{0x306e, 0x4810},
	{0x3064, 0x1802},
	{0x3ef6, 0x804d},
	{0x3180, 0xc08f},
	{0x30ba, 0x7623},
	{0x3176, 0x0480},
	{0x3178, 0x0480},
	{0x317a, 0x0480},
	{0x317c, 0x0480},
};

static const struct ar0144_reg_value ar0144at_pll_27mhz[] = {
	{0x302a, 0x0006},
	{0x302c, 0x0001},
	{0x302e, 0x0004},
	{0x3030, 0x4a /*0x0042*/},
	{0x3036, 0x000c},
	{0x3038, 0x0001},
	/* Addition settings from Oren */
	{0x3080, 0x0000},
	{0x3180, 0x0042},
	{0x3182, 0x002e},
	{0x3184, 0x1665},
	{0x3186, 0x110e},
	{0x3188, 0x2047},
	{0x318a, 0x0105},
	{0x318c, 0x0004},
};

static const struct ar0144_reg_value ar0144at_mipi_2lane_12bit[] = {
	{0x31ae, 0x0202},
	{0x31ac, 0x0c0c},
	{0x31b0, 0x0042},
	{0x31b2, 0x002e},
	{0x31b4, 0x1665},
	{0x31b6, 0x110e},
	{0x31b8, 0x2047},
	{0x31ba, 0x0105},
	{0x31bc, 0x0004},
};

static const struct ar0144_reg_value ar0144at_1280x800_60fps[] = {
	{0x3002, 0x0000},
	{0x3004, 0x0004},
	{0x3006, 0x031f},
	{0x3008, 0x0503},
	{0x300a, 0x0339},
	{0x300c, 0x05d0},
	{0x3012, 0x0064},
	{0x30a2, 0x0001},
	{0x30a6, 0x0001},
	{0x3040, 0x0000},
};

static const struct ar0144_reg_value ar0144at_context_b_2x2_binning[] = {
	{0x3040, 0x1000},
	{0x30a8, 0x0003},
	{0x3040, 0x3000},
	{0x30ae, 0x0003},
};

static const struct ar0144_reg_value ar0144at_embedded_data_stats[] = {
	{0x3064, 0x1982},
};

static const struct ar0144_reg_value ar0144at_start_stream[] = {
	{0x3028, 0x0010},
	{0x301a, 0x005c},
};

static const struct ar0144_reg_value ar0144at_stop_stream[] = {
	{0x301a, 0x0058},
};

static int ar0144_write_reg(struct ar0144 *ar0144, u32 reg, u32 val)
{
	u8 regbuf[4];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val >> 8;
	regbuf[3] = val & 0xff;

	ret = i2c_master_send(ar0144->i2c_client, regbuf, 4);
	if (ret < 0)
		dev_err(&ar0144->i2c_client->dev,
			"%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);

	return ret;
}

static int ar0144_read_reg(struct ar0144 *ar0144, u32 reg, u32 *val)
{
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	ret = i2c_master_send(ar0144->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0144->i2c_client->dev,
			"%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(ar0144->i2c_client, buf, 2);
	if (ret < 0) {
		dev_err(&ar0144->i2c_client->dev,
			"%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}
	*val = buf[0] << 8;
	*val |= (buf[1] & 0xff);

	return 0;
}

static int ar0144_set_register_array(struct ar0144 *ar0144,
				     const struct ar0144_reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = ar0144_write_reg(ar0144, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	u32 reg_val;
	int ret = 0;

	mutex_lock(&ar0144->lock);

	gpiod_direction_output(ar0144->rst_gpio, 1);
	if (!on)
		goto out;
	msleep(2); /* more than 1ms */
	gpiod_set_value_cansleep(ar0144->rst_gpio, 0);
	msleep(10); /* more than 160000 clocks at 24MHz; FIXME: use clk rate */

	ret = ar0144_read_reg(ar0144, AR0144_ID_REG, &reg_val);
	if (ret < 0)
		goto out;
	if (reg_val != AR0144_ID_VAL) {
		dev_err(&ar0144->i2c_client->dev,
			"wrong chip id (%u), expected %u\n", reg_val,
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
	u32 reg_val;

	mutex_lock(&ar0144->lock);

	if (enable == 0) {
		ret = ar0144_read_reg(ar0144, 0x303a, &reg_val);
		if (ret == 0)
			printk("%s: FRAME_COUNT: %u\n", __func__, reg_val);
		ret = ar0144_read_reg(ar0144, 0x303c, &reg_val);
		if (ret == 0)
			printk("%s: FRAME_STATUS: %u\n", __func__, reg_val);
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
	ret = ar0144_read_reg(ar0144, 0x303a, &reg_val);
	if (ret == 0)
		printk("%s: FRAME_COUNT: %u\n", __func__, reg_val);
	ret = ar0144_read_reg(ar0144, 0x303c, &reg_val);
	if (ret == 0)
		printk("%s: FRAME_STATUS: %u\n", __func__, reg_val);

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

	ar0144->i2c_client = client;
	ar0144->dev = dev;
	mutex_init(&ar0144->lock);

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
