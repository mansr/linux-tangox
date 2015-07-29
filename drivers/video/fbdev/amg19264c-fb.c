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
#include <linux/fb.h>
#include <linux/mutex.h>

#define LCD_WIDTH	192
#define LCD_HEIGHT	64
#define LCD_MEM_SIZE	(LCD_WIDTH * LCD_HEIGHT / 8)

struct amg19264c {
	struct fb_info *fb;
	u8 *mem;
	const struct amg19264c_ops *ops;
	struct mutex lock;
};

struct amg19264c_ops {
	void (*write)(struct amg19264c *lcd, int cs, int rs, int val);
	void (*flush)(struct amg19264c *lcd);
};

static const struct fb_fix_screeninfo amg19264c_fix = {
	.id		= "amg19264c",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.ywrapstep	= 1,
	.line_length	= LCD_WIDTH / 8,
};

static const struct fb_var_screeninfo amg19264c_var = {
	.xres		= LCD_WIDTH,
	.yres		= LCD_HEIGHT,
	.xres_virtual	= LCD_WIDTH,
	.yres_virtual	= LCD_HEIGHT,
	.bits_per_pixel	= 1,
	.red		= { .length = 1 },
	.green		= { .length = 1 },
	.blue		= { .length = 1 },
	.vmode		= FB_VMODE_YWRAP,
};

static void amg19264c_write_cmd(struct amg19264c *lcd, int cs, int cmd)
{
	lcd->ops->write(lcd, cs, 0, cmd);
}

static void amg19264c_write_data(struct amg19264c *lcd, int cs, int val)
{
	lcd->ops->write(lcd, cs, 1, val);
}

static void amg19264c_flush(struct amg19264c *lcd)
{
	if (lcd->ops->flush)
		lcd->ops->flush(lcd);
}

static int amg19264c_blank(int blank, struct fb_info *info)
{
	struct amg19264c *lcd = info->par;
	int on = blank == FB_BLANK_UNBLANK;

	amg19264c_write_cmd(lcd, 7, 0x3e | on);
	amg19264c_flush(lcd);

	return 0;
}

static int amg19264c_pan(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct amg19264c *lcd = fb->par;

	mutex_lock(&lcd->lock);

	amg19264c_write_cmd(lcd, 7, 0xc0 | var->yoffset);
	amg19264c_flush(lcd);

	mutex_unlock(&lcd->lock);

	return 0;
}

static int amg19264c_get_data(struct amg19264c *lcd, int x, int y)
{
	u8 *p = &lcd->mem[y * LCD_WIDTH / 8 + x / 8];
	int sh = x & 7;
	int val = 0;
	int i;

	for (i = 0; i < 8; i++) {
		val |= ((*p >> sh) & 1) << i;
		p += LCD_WIDTH / 8;
	}

	return val;
}

static int amg19264c_update(struct fb_info *fb)
{
	struct amg19264c *lcd = fb->par;
	int i, j;
	int data;
	int cs;

	mutex_lock(&lcd->lock);

	amg19264c_write_cmd(lcd, 7, 0x40);

	for (i = 0; i < 8; i++) {
		amg19264c_write_cmd(lcd, 7, 0xb8 | i);
		for (j = 0; j < LCD_WIDTH; j++) {
			cs = 1 << (2 - j / 64);
			data = amg19264c_get_data(lcd, j, 8 * i);
			amg19264c_write_data(lcd, cs, data);
		}
	}

	amg19264c_flush(lcd);

	mutex_unlock(&lcd->lock);

	return 0;
}

static void amg19264c_update_row(struct amg19264c *lcd, int row, int x, int w)
{
	int csx, sx, ex;
	int cs, data;
	int i, j;

	amg19264c_write_cmd(lcd, 7, 0xb8 | row);

	for (i = 0; i < 3; i++) {
		csx = 64 * i;

		if (x >= csx + 64 || x + w < csx)
			continue;

		sx = max(x, csx);
		ex = min(x + w, csx + 64);
		cs = 1 << (2 - i);

		amg19264c_write_cmd(lcd, cs, 0x40 | (sx & 63));

		for (j = sx; j < ex; j++) {
			data = amg19264c_get_data(lcd, j, 8 * row);
			amg19264c_write_data(lcd, cs, data);
		}
	}
}

static void amg19264c_update_rect(struct fb_info *fb, int x, int y,
				  int width, int height)
{
	struct amg19264c *lcd = fb->par;
	int i;

	mutex_lock(&lcd->lock);

	for (i = y >> 3; i < (y + height + 7) >> 3; i++)
		amg19264c_update_row(lcd, i, x, width);

	amg19264c_flush(lcd);

	mutex_unlock(&lcd->lock);
}

static ssize_t amg19264c_fb_write(struct fb_info *fb, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	ssize_t ret;

	ret = fb_sys_write(fb, buf, count, ppos);
	if (ret < 0)
		return ret;

	amg19264c_update(fb);

	return ret;
}

static void amg19264c_fillrect(struct fb_info *fb, const struct fb_fillrect *r)
{
	sys_fillrect(fb, r);
	amg19264c_update_rect(fb, r->dx, r->dy, r->width, r->height);
}

static void amg19264c_copyarea(struct fb_info *fb, const struct fb_copyarea *a)
{
	sys_copyarea(fb, a);
	amg19264c_update_rect(fb, a->dx, a->dy, a->width, a->height);
}

static void amg19264c_imageblit(struct fb_info *fb, const struct fb_image *img)
{
	sys_imageblit(fb, img);
	amg19264c_update_rect(fb, img->dx, img->dy, img->width, img->height);
}

static struct fb_ops amg19264c_ops = {
	.fb_read	= fb_sys_read,
	.fb_write	= amg19264c_fb_write,
	.fb_blank	= amg19264c_blank,
	.fb_pan_display	= amg19264c_pan,
	.fb_fillrect	= amg19264c_fillrect,
	.fb_copyarea	= amg19264c_copyarea,
	.fb_imageblit	= amg19264c_imageblit,
};

static void amg19264c_do_io(struct fb_info *fb, struct list_head *pages)
{
	amg19264c_update(fb);
}

static struct fb_deferred_io amg19264c_defio = {
	.delay		= HZ / 4,
	.deferred_io	= amg19264c_do_io,
};

static int amg19264c_setup(struct amg19264c *lcd, struct device *dev)
{
	struct fb_info *fb;
	int err;

	fb = framebuffer_alloc(0, dev);
	if (!fb)
		return -ENOMEM;

	lcd->fb = fb;
	mutex_init(&lcd->lock);

	lcd->mem = devm_kzalloc(dev, LCD_MEM_SIZE, GFP_KERNEL);
	if (!lcd->mem) {
		err = -ENOMEM;
		goto err_fb;
	}

	fb->flags = FBINFO_DEFAULT | FBINFO_VIRTFB | FBINFO_READS_FAST |
		FBINFO_HWACCEL_YWRAP;
	fb->fix = amg19264c_fix;
	fb->var = amg19264c_var;
	fb->par = lcd;

	fb->screen_base = lcd->mem;
	fb->screen_size = LCD_MEM_SIZE;
	fb->fix.smem_start = (unsigned long)lcd->mem;
	fb->fix.smem_len = LCD_MEM_SIZE;

	fb->fbops = &amg19264c_ops;
	fb->fbdefio = &amg19264c_defio;

	fb_deferred_io_init(fb);

	amg19264c_update(fb);
	amg19264c_blank(FB_BLANK_UNBLANK, fb);

	err = register_framebuffer(fb);
	if (err)
		goto err_defio;

	return 0;

err_defio:
	fb_deferred_io_cleanup(fb);
err_fb:
	framebuffer_release(fb);

	return err;
}

static void amg19264c_remove(struct amg19264c *lcd)
{
	struct fb_info *fb = lcd->fb;

	unregister_framebuffer(fb);
	fb_deferred_io_cleanup(fb);
	framebuffer_release(fb);

	mutex_destroy(&lcd->lock);
}

#define PIN_E	0
#define PIN_RS	1
#define PIN_CS1	2
#define PIN_CS2	3
#define PIN_CS3	4

static const char *amg19264c_pins[] = {
	"pin-e",
	"pin-rs",
	"pin-cs1",
	"pin-cs2",
	"pin-cs3",
};

#ifdef CONFIG_FB_AMG19264C_MAX732X

#include <linux/i2c.h>

#define I2C_BUF		256

struct amg19264c_max732x {
	struct amg19264c lcd;
	struct i2c_client *i2c_ctrl;
	struct i2c_client *i2c_data;
	u8 i2c_buf[I2C_BUF];
	int i2c_len;
	u8 data;
	u8 pins;
	u8 pinmap[5];
};

#define to_amg19264c_max732x(p) container_of(p, struct amg19264c_max732x, lcd)

static void amg19264c_max732x_flush(struct amg19264c *lcd)
{
	struct amg19264c_max732x *io = to_amg19264c_max732x(lcd);

	if (io->i2c_len) {
		i2c_master_send(io->i2c_ctrl, io->i2c_buf, io->i2c_len);
		io->i2c_len = 0;
	}
}

static void amg19264c_max732x_queue(struct amg19264c *lcd, int cs, int rs)
{
	struct amg19264c_max732x *io = to_amg19264c_max732x(lcd);
	int pins = rs ? io->pinmap[PIN_RS] : 0;
	int i;

	for (i = 0; i < 3; i++)
		if (~cs & (1 << i))
			pins |= io->pinmap[PIN_CS1 + i];

	if (pins != io->pins)
		io->i2c_buf[io->i2c_len++] = pins;
	io->i2c_buf[io->i2c_len++] = pins | io->pinmap[PIN_E];
	io->i2c_buf[io->i2c_len++] = pins;

	io->pins = pins;

	if (io->i2c_len > I2C_BUF - 3)
		amg19264c_max732x_flush(lcd);
}

static void amg19264c_max732x_write(struct amg19264c *lcd, int cs, int rs,
				    int val)
{
	struct amg19264c_max732x *io = to_amg19264c_max732x(lcd);

	if (val != io->data) {
		amg19264c_max732x_flush(lcd);
		io->data = val;
		i2c_master_send(io->i2c_data, &io->data, 1);
	}

	amg19264c_max732x_queue(lcd, cs, rs);
}

static const struct amg19264c_ops amg19264c_max732x_ops = {
	.write	= amg19264c_max732x_write,
	.flush	= amg19264c_max732x_flush,
};

static int amg19264c_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct device_node *of_node = i2c->dev.of_node;
	struct amg19264c_max732x *io;
	struct i2c_client *i2c2;
	struct amg19264c *lcd;
	int err;
	int i;

	io = devm_kzalloc(&i2c->dev, sizeof(*io), GFP_KERNEL);
	if (!io)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(io->pinmap); i++) {
		const char *name = amg19264c_pins[i];
		u32 pin;

		err = of_property_read_u32(of_node, name, &pin);
		if (err)
			return err;

		if (pin < 8 || pin > 15) {
			dev_err(&i2c->dev, "unsupported %s %u\n", name, pin);
			return -EINVAL;
		}

		io->pinmap[i] = 1 << (pin & 7);
	}

	i2c_set_clientdata(i2c, io);

	i2c2 = i2c_new_dummy(i2c->adapter, i2c->addr ^ 0x30);
	if (!i2c2)
		return -EINVAL;

	if (i2c->addr & 0x10) {
		io->i2c_ctrl = i2c;
		io->i2c_data = i2c2;
	} else {
		io->i2c_ctrl = i2c2;
		io->i2c_data = i2c;
	}

	i2c_master_send(io->i2c_data, &io->data, 1);
	i2c_master_send(io->i2c_ctrl, &io->pins, 1);

	lcd = &io->lcd;
	lcd->ops = &amg19264c_max732x_ops;

	err = amg19264c_setup(lcd, &i2c->dev);
	if (err)
		goto err_i2c;

	fb_info(lcd->fb, "AMG19264C framebuffer at I2C 0x%02x/0x%02x\n",
		i2c->addr, i2c2->addr);

	return 0;

err_i2c:
	i2c_unregister_device(i2c2);

	return err;
}

static int amg19264c_i2c_remove(struct i2c_client *i2c)
{
	struct amg19264c_max732x *io = i2c_get_clientdata(i2c);

	amg19264c_remove(&io->lcd);

	if (i2c == io->i2c_ctrl)
		i2c_unregister_device(io->i2c_data);
	else
		i2c_unregister_device(io->i2c_ctrl);

	return 0;
}

static const struct of_device_id amg19264c_i2c_dt_ids[] = {
	{ .compatible = "orient,amg19264c-max732x" },
	{ }
};
MODULE_DEVICE_TABLE(of, amg19264c_i2c_dt_ids);

static const struct i2c_device_id amg19264c_i2c_ids[] = {
	{ .name = "amg19264c-max732x" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, amg19264c_i2c_ids);

static struct i2c_driver amg19264c_i2c_driver = {
	.probe		= amg19264c_i2c_probe,
	.remove		= amg19264c_i2c_remove,
	.driver		= {
		.name		= "amg19264c_i2c",
		.of_match_table	= amg19264c_i2c_dt_ids,
	},
	.id_table	= amg19264c_i2c_ids,
};

#endif /* CONFIG_FB_AMG19264C_MAX732X */

#ifdef CONFIG_FB_AMG19264C_GPIO

#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

struct amg19264c_gpio {
	struct amg19264c lcd;
	struct gpio_desc *gpio_ctrl[5];
	struct gpio_desc *gpio_data[8];
	int ctrl[6];
	int data[8];
};

#define to_amg19264c_gpio(p) container_of(p, struct amg19264c_gpio, lcd)

static void amg19264c_gpio_set(struct gpio_desc **gpio, int *data, int num,
			       int val)
{
	struct gpio_desc *g[8];
	int d[8];
	int i, n;

	for (i = 0, n = 0; i < num; i++) {
		int v = (val >> i) & 1;
		if (v != data[i]) {
			g[n] = gpio[i];
			d[n] = v;
			n++;
		}
		data[i] = v;
	}

	if (n)
		gpiod_set_raw_array_value_cansleep(n, g, d);
}

static void amg19264c_gpio_write(struct amg19264c *lcd, int cs, int rs,
				 int val)
{
	struct amg19264c_gpio *io = to_amg19264c_gpio(lcd);
	int pins = (cs ^ 7) << 2 | rs << 1;

	amg19264c_gpio_set(io->gpio_data, io->data, 8, val);
	amg19264c_gpio_set(io->gpio_ctrl, io->ctrl, 5, pins);

	gpiod_set_raw_value_cansleep(io->gpio_ctrl[0], 1);
	gpiod_set_raw_value_cansleep(io->gpio_ctrl[0], 0);
}

static const struct amg19264c_ops amg19264c_gpio_ops = {
	.write	= amg19264c_gpio_write,
};

static int amg19264c_gpio_get_group(struct device *dev, const char *group,
				    struct gpio_desc **gpio, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		gpio[i] = devm_gpiod_get_index(dev, group, i, GPIOD_OUT_LOW);
		if (IS_ERR(gpio[i])) {
			dev_err(dev, "failed to get %s[%d] gpio\n", group, i);
			return PTR_ERR(gpio[i]);
		}
	}

	return 0;
}

static int amg19264c_gpio_get_named(struct device *dev, const char **names,
				    struct gpio_desc **gpio, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		gpio[i] = devm_gpiod_get(dev, names[i], GPIOD_OUT_LOW);
		if (IS_ERR(gpio[i])) {
			dev_err(dev, "failed to get %s gpio\n", names[i]);
			return PTR_ERR(gpio[i]);
		}
	}

	return 0;
}

static int amg19264c_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct amg19264c_gpio *io;
	struct amg19264c *lcd;
	struct gpio_desc *grw;
	int err;

	io = devm_kzalloc(dev, sizeof(*io), GFP_KERNEL);
	if (!io)
		return -ENOMEM;

	platform_set_drvdata(pdev, io);

	err = amg19264c_gpio_get_group(dev, "data", io->gpio_data, 8);
	if (err)
		return err;

	err = amg19264c_gpio_get_named(dev, amg19264c_pins, io->gpio_ctrl, 5);
	if (err)
		return err;

	grw = devm_gpiod_get_optional(dev, "pin-rw", GPIOD_OUT_LOW);
	if (IS_ERR(grw))
		return PTR_ERR(grw);

	lcd = &io->lcd;
	lcd->ops = &amg19264c_gpio_ops;

	err = amg19264c_setup(lcd, dev);
	if (err)
		return err;

	fb_info(lcd->fb, "AMG19264C framebuffer using GPIO\n");

	return 0;
}

static int amg19264c_gpio_remove(struct platform_device *pdev)
{
	struct amg19264c_gpio *io = platform_get_drvdata(pdev);

	amg19264c_remove(&io->lcd);

	return 0;
}

static const struct of_device_id amg19264c_gpio_dt_ids[] = {
	{ .compatible = "orient,amg19264c-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, amg19264c_gpio_dt_ids);

static struct platform_driver amg19264c_gpio_driver = {
	.probe		= amg19264c_gpio_probe,
	.remove		= amg19264c_gpio_remove,
	.driver		= {
		.name		= "amg19264c_gpio",
		.of_match_table	= amg19264c_gpio_dt_ids,
	},
};

#endif /* CONFIG_FB_AMG19264C_GPIO */

static int __init amg19264c_init(void)
{
#ifdef CONFIG_FB_AMG19264C_MAX732X
	i2c_add_driver(&amg19264c_i2c_driver);
#endif
#ifdef CONFIG_FB_AMG19264C_GPIO
	platform_driver_register(&amg19264c_gpio_driver);
#endif
	return 0;
}
module_init(amg19264c_init);

static void __exit amg19264c_exit(void)
{
#ifdef CONFIG_FB_AMG19264C_MAX732X
	i2c_del_driver(&amg19264c_i2c_driver);
#endif
#ifdef CONFIG_FB_AMG19264C_GPIO
	platform_driver_unregister(&amg19264c_gpio_driver);
#endif
}
module_exit(amg19264c_exit);

MODULE_DESCRIPTION("AMG19264C framebuffer driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
