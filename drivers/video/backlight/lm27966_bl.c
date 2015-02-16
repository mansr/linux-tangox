/*
 * Copyright (C) 2015 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>

#define LM27966_GPR		0x10
#define LM27966_MAIN_BRIGHTNESS	0xa0
#define LM27966_AUX_BRIGHTNESS	0xc0

struct lm27966_bl {
	struct i2c_client *i2c;
	struct backlight_device *bl;
};

static int lm27966_bl_read(struct lm27966_bl *lmbl, u8 addr)
{
	u8 data;
	int ret;

	ret = i2c_master_send(lmbl->i2c, &addr, 1);
	if (ret <= 0)
		return ret;

	ret = i2c_master_recv(lmbl->i2c, &data, 1);
	if (ret <= 0)
		return ret;

	return data;
}

static int lm27966_bl_write(struct lm27966_bl *lmbl, u8 addr, u8 val)
{
	u8 buf[2] = { addr, val };

	return i2c_master_send(lmbl->i2c, buf, 2);
}

static int lm27966_bl_update(struct backlight_device *bl)
{
	struct lm27966_bl *lmbl = bl_get_data(bl);
	int brightness = bl->props.brightness;
	int power = 1;

	if (bl->props.power != FB_BLANK_UNBLANK)
		power = 0;
	if (bl->props.state & (BL_CORE_FBBLANK | BL_CORE_SUSPENDED))
		power = 0;

	lm27966_bl_write(lmbl, LM27966_GPR, power);
	lm27966_bl_write(lmbl, LM27966_MAIN_BRIGHTNESS, brightness);

	return 0;
}

static int lm27966_bl_get_brightness(struct backlight_device *bl)
{
	struct lm27966_bl *lmbl = bl_get_data(bl);
	int brightness = 0;

	brightness = lm27966_bl_read(lmbl, LM27966_MAIN_BRIGHTNESS) & 31;

	return brightness;
}

static struct backlight_ops lm27966_bl_ops = {
	.update_status	= lm27966_bl_update,
	.get_brightness	= lm27966_bl_get_brightness,
};

static int lm27966_bl_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct device_node *of_node = i2c->dev.of_node;
	struct lm27966_bl *lmbl;
	struct backlight_device *bl;
	struct backlight_properties props;
	char name[32];
	u32 power;
	u32 brt;

	lmbl = devm_kzalloc(&i2c->dev, sizeof(&lmbl), GFP_KERNEL);
	if (!lmbl)
		return -ENOMEM;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 31;

	snprintf(name, sizeof(name), "%s-%s", "lm27966", dev_name(&i2c->dev));

	bl = devm_backlight_device_register(&i2c->dev, name, &i2c->dev, lmbl,
					    &lm27966_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	lmbl->i2c = i2c;
	lmbl->bl = bl;

	power = lm27966_bl_read(lmbl, LM27966_GPR) & 1;
	brt = lm27966_bl_get_brightness(bl) & 31;

	if (!of_property_read_u32(of_node, "default-brightness", &brt))
		brt = min(brt, 31u);

	if (!of_property_read_u32(of_node, "default-power", &power))
		power = !!power;

	bl->props.power = power ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
	bl->props.brightness = brt;

	lm27966_bl_update(bl);

	dev_info(&bl->dev, "LM27966 backlight at I2C 0x%02x\n", i2c->addr);

	return 0;
}

static const struct of_device_id lm27966_bl_dt_ids[] = {
	{ .compatible = "ti,lm27966" },
	{ }
};

static const struct i2c_device_id lm27966_bl_ids[] = {
	{ .name = "lm27966" },
	{ }
};

static struct i2c_driver lm27966_bl_driver = {
	.probe		= lm27966_bl_probe,
	.driver		= {
		.name		= "lm27966_bl",
		.of_match_table	= lm27966_bl_dt_ids,
	},
	.id_table	= lm27966_bl_ids,
};
module_i2c_driver(lm27966_bl_driver);

MODULE_DESCRIPTION("LM27966 backlight driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
