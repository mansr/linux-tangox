/*
 * Copyright (C) 2014 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#define GPIO_DIR	0x00
#define GPIO_DATA	0x04
#define GPIO_INT	0x08

static int tangox_gpio_read(struct gpio_chip *chip, unsigned int reg,
			    unsigned int bit)
{
	struct of_mm_gpio_chip *mmchip = to_of_mm_gpio_chip(chip);
	unsigned int val;

	val = readl(mmchip->regs + reg);

	return val >> bit & 1;
}

static void tangox_gpio_write(struct gpio_chip *chip, unsigned int reg,
			      unsigned int bit, int val)
{
	struct of_mm_gpio_chip *mmchip = to_of_mm_gpio_chip(chip);
	unsigned int cmd = 1 << (16 + bit) | val << bit;

	writel(cmd, mmchip->regs + reg);
}

static int tangox_gpio_get_dir(struct gpio_chip *chip, unsigned int offs)
{
	return !tangox_gpio_read(chip, GPIO_DIR, offs);
}

static int tangox_gpio_dir_in(struct gpio_chip *chip, unsigned int offs)
{
	tangox_gpio_write(chip, GPIO_DIR, offs, 0);

	return 0;
}

static int tangox_gpio_dir_out(struct gpio_chip *chip, unsigned int offs,
			       int val)
{
	tangox_gpio_write(chip, GPIO_DATA, offs, val);
	tangox_gpio_write(chip, GPIO_DIR, offs, 1);

	return 0;
}

static int tangox_gpio_get(struct gpio_chip *chip, unsigned int offs)
{
	return tangox_gpio_read(chip, GPIO_DATA, offs);
}

static void tangox_gpio_set(struct gpio_chip *chip, unsigned int offs, int val)
{
	tangox_gpio_write(chip, GPIO_DATA, offs, val);
}

static int tangox_gpio_probe(struct platform_device *pdev)
{
	struct of_mm_gpio_chip *mmchip;
	struct gpio_chip *gc;

	mmchip = devm_kzalloc(&pdev->dev, sizeof(*mmchip), GFP_KERNEL);
	if (!mmchip)
		return -ENOMEM;

	gc = &mmchip->gc;

	gc->get_direction = tangox_gpio_get_dir;
	gc->direction_input = tangox_gpio_dir_in;
	gc->direction_output = tangox_gpio_dir_out;
	gc->get = tangox_gpio_get;
	gc->set = tangox_gpio_set;
	gc->ngpio = 16;

	return of_mm_gpiochip_add(pdev->dev.of_node, mmchip);
}

static const struct of_device_id tangox_gpio_dt_ids[] = {
	{ .compatible = "sigma,smp8640-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, tangox_gpio_dt_ids);

static struct platform_driver tangox_gpio_driver = {
	.driver = {
		.name		= "tangox-gpio",
		.of_match_table	= tangox_gpio_dt_ids,
	},
};
module_platform_driver_probe(tangox_gpio_driver, tangox_gpio_probe);

MODULE_DESCRIPTION("SMP86xx GPIO driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
