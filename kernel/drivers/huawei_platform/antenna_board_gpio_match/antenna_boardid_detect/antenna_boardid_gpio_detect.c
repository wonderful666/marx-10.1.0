/*
 * antenna_boardid_gpio_detect.c
 *
 * Antenna boardid detect driver,detect the antenna board id by gpio.
 *
 * Copyright (c) 2012-2019 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include "antenna_boardid_gpio_detect.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <huawei_platform/log/hw_log.h>

#define HWLOG_TAG antenna_boardid_detect
HWLOG_REGIST();

#define ANTENNA_DETECT(_name, n, m, store) {	\
	.attr = __ATTR(_name, m, antenna_detect_show, store),	\
	.name = ANTENNA_##n,	\
}

#define ANTENNA_DETECT_RO(_name, n)	\
		ANTENNA_DETECT(_name, n, 0444, NULL)

struct pinctrl_state *pinctrl_def;
struct pinctrl_state *pinctrl_idle;
struct pinctrl *pinctrl;

static ssize_t antenna_detect_show(struct device *dev,
		struct device_attribute *attr, char *buf);

struct antenna_detect_info {
	struct device_attribute attr;
	u8 name;
};

#define MAX_GPIO_NUM    3
#define CODEC_GPIO_BASE 224
#define DTS_PARSE_ERROR_TO_DI    (-1)
#define DTS_PARSE_ERROR_TO_GPIO  (-2)
#define DTS_PARSE_SUCCES    0
static int g_gpio_count;
static int g_gpio[MAX_GPIO_NUM] = {0};
static int g_gpio_expect = -1;

static struct antenna_detect_info antenna_detect_tb[] = {
	ANTENNA_DETECT_RO(antenna_board_match, BOARDID_GPIO_DETECT),
	ANTENNA_DETECT_RO(antenna_boardid_status, BOARDID_GPIO_STATUS),
};

static struct attribute *antenna_sysfs_attrs[ARRAY_SIZE(antenna_detect_tb) + 1];
static const struct attribute_group antenna_sysfs_attr_group = {
	.attrs = antenna_sysfs_attrs,
};

static void antenna_sysfs_init_attrs(void)
{
	int i;
	int limit = ARRAY_SIZE(antenna_detect_tb);

	for (i = 0; i < limit; i++)
		antenna_sysfs_attrs[i] = &antenna_detect_tb[i].attr.attr;
	antenna_sysfs_attrs[limit] = NULL;
}

static struct antenna_detect_info *antenna_detect_lookup(const char *name)
{
	int i;
	int limit = ARRAY_SIZE(antenna_detect_tb);

	for (i = 0; i < limit; i++) {
		if (!strncmp(name, antenna_detect_tb[i].attr.attr.name,
			strlen(name)))
			break;
	}
	if (i >= limit)
		return NULL;
	return &antenna_detect_tb[i];
}

static int antenna_detect_sysfs_create_group(struct antenna_device_info *di)
{
	antenna_sysfs_init_attrs();
	return sysfs_create_group(&di->dev->kobj, &antenna_sysfs_attr_group);
}

static inline void antenna_detect_sysfs_remove_group(
	struct antenna_device_info *di)
{
	sysfs_remove_group(&di->dev->kobj, &antenna_sysfs_attr_group);
}

static ssize_t antenna_detect_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int ret;
	int gpio_value = 0;
	int match = 0;
	int temp_value = 0;
	struct antenna_detect_info *info = NULL;

	info = antenna_detect_lookup(attr->attr.name);
	if (!info)
		return -EINVAL;

	ret = pinctrl_select_state(pinctrl, pinctrl_def);
	if (ret)
		dev_err(dev, "could not set pins to default state\n");

	if (g_gpio_count > MAX_GPIO_NUM) {
		hwlog_err("%s: The detect GPIO number over MAX!\n", __func__);
		return 0;
	}

	switch (info->name) {
	case ANTENNA_BOARDID_GPIO_DETECT:
		for (i = 0; i < g_gpio_count; i++) {
			if (g_gpio[i] >= CODEC_GPIO_BASE)
				temp_value = gpio_get_value_cansleep(g_gpio[i]);
			else
				temp_value = gpio_get_value(g_gpio[i]);
			gpio_value += (temp_value << i);
		}
		if (g_gpio_expect == gpio_value)
			match = ANATENNA_DETECT_SUCCEED;
		else
			match = ANATENNA_DETECT_FAIL;

		hwlog_info("%s: gpio read_value= %d, expect_value = %d\n",
			__func__, gpio_value, g_gpio_expect);
		return snprintf(buf, PAGE_SIZE, "%d\n", match);
	case ANTENNA_BOARDID_GPIO_STATUS:
		for (i = 0; i < g_gpio_count; i++) {
			if (g_gpio[i] >= CODEC_GPIO_BASE)
				temp_value = gpio_get_value_cansleep(g_gpio[i]);
			else
				temp_value = gpio_get_value(g_gpio[i]);
			gpio_value += (temp_value << i);
		}
		hwlog_info("%s: get gpio value is %d\n", __func__, gpio_value);
		return snprintf(buf, PAGE_SIZE, "%d\n", gpio_value);

	default:
		hwlog_err("%s: NODE ERR, HAVE NO THIS NODE: %d\n",
			__func__, info->name);
		break;
	}
	return 0;
}

static struct class *hw_antenna_detect_class;
static struct class *antenna_detect_class;
struct device *antenna_dev;

/* get new class */
struct class *hw_antenna_detect_get_class(void)
{
	if (hw_antenna_detect_class == NULL) {
		hw_antenna_detect_class = class_create(THIS_MODULE,
			"hw_antenna");
		if (hw_antenna_detect_class == NULL) {
			hwlog_err("hw_antenna_detect_class create fail");
			return NULL;
		}
	}
	return hw_antenna_detect_class;
}

/* static function, the input parameter does not need to be checked */
static int antenna_dts_parse(struct device_node *antenna_node, int *gpio_sub)
{
	int i;
	int ret;
	int array_len;
	const char *gpio_data_string = NULL;

	array_len = of_property_count_strings(antenna_node, "temp_gpio");
	g_gpio_count = array_len;
	if (array_len <= 0) {
		hwlog_err("temp_gpio is error, please check it!\n");
		return DTS_PARSE_ERROR_TO_DI;
	}

	if (of_property_read_u32(antenna_node, "expect_value",
		&g_gpio_expect)) {
		g_gpio_expect = -1;
	}

	for (i = 0; i < array_len; i++) {
		ret = of_property_read_string_index(antenna_node, "temp_gpio",
			i, &gpio_data_string);
		if (ret) {
			hwlog_err("get temp_gpio failed\n");
			return DTS_PARSE_ERROR_TO_DI;
		}
		g_gpio[i] = of_get_named_gpio(antenna_node,
			gpio_data_string, 0);
		hwlog_info("gpio = %d\n", g_gpio[i]);

		if (!gpio_is_valid(g_gpio[i])) {
			hwlog_err("gpio is not valid\n");
			return DTS_PARSE_ERROR_TO_DI;
		}

		ret = gpio_request(g_gpio[i], "antenna_boardid_detect");
		if (ret < 0) {
			hwlog_err("%s: gpio_request error, ret = %d, gpio = %d\n",
				__func__, ret, g_gpio[i]);
			return DTS_PARSE_ERROR_TO_DI;
		}

		ret = gpio_direction_input(g_gpio[i]);
		if (ret < 0) {
			hwlog_err("%s: gpio_direction_input error, ret = %d, gpio = %d\n",
				__func__, ret, g_gpio[i]);
			*gpio_sub = i;
			return DTS_PARSE_ERROR_TO_GPIO;
		}
	}

	*gpio_sub = i;
	return DTS_PARSE_SUCCES;
}

static int antenna_boardid_detect_probe(struct platform_device *pdev)
{
	int ret;
	int gpio_sub = 0;
	int result;
	struct antenna_device_info *di = NULL;
	struct device_node *antenna_node = pdev->dev.of_node;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->dev = &pdev->dev;

	pinctrl = pinctrl_get(di->dev);
	if (IS_ERR(pinctrl))
		dev_err(di->dev, "could not get pinctrl\n");

	pinctrl_def = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(pinctrl_def))
		dev_err(di->dev, "could not get defstate %li\n",
			PTR_ERR(pinctrl_def));
	pinctrl_idle = pinctrl_lookup_state(pinctrl, "idle");
	if (IS_ERR(pinctrl_idle))
		dev_err(di->dev, "could not get idlestate %li\n",
			PTR_ERR(pinctrl_idle));

	result = antenna_dts_parse(antenna_node, &gpio_sub);
	if (result == DTS_PARSE_ERROR_TO_DI)
		goto free_di;
	if (result == DTS_PARSE_ERROR_TO_GPIO)
		goto free_gpio;

	ret = antenna_detect_sysfs_create_group(di);
	if (ret) {
		hwlog_err("can't create antenna_detect sysfs entries\n");
		goto free_gpio;
	}
	antenna_detect_class = hw_antenna_detect_get_class();
	if (antenna_detect_class) {
		if (antenna_dev == NULL)
			antenna_dev = device_create(antenna_detect_class,
				NULL, 0, NULL, "antenna_board");
		ret = sysfs_create_link(&antenna_dev->kobj, &di->dev->kobj,
			"antenna_board_data");
		if (ret) {
			hwlog_err("create link to boardid_detect fail\n");
			goto free_gpio;
		}
	}
	hwlog_info("huawei antenna boardid detect probe ok!\n");
	return 0;

free_gpio:
	gpio_free(g_gpio[gpio_sub]);
free_di:
	kfree(di);
	di = NULL;
	return -1;
}

/* probe match table */
static const struct of_device_id antenna_boardid_detect_table[] = {
	{
		.compatible = "huawei,antenna_boardid_detect",
		.data = NULL,
	},
	{},
};

/* antenna boardid detect driver */
static struct platform_driver antenna_boardid_detect_driver = {
	.probe = antenna_boardid_detect_probe,
	.driver = {
		.name = "huawei,antenna_boardid_detect",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(antenna_boardid_detect_table),
	},
};

static int __init antenna_boardid_detect_init(void)
{
	hwlog_info("into init");
	return platform_driver_register(&antenna_boardid_detect_driver);
}

static void __exit antenna_boardid_detect_exit(void)
{
	platform_driver_unregister(&antenna_boardid_detect_driver);
}

module_init(antenna_boardid_detect_init);
module_exit(antenna_boardid_detect_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("huawei antenna boardid detect driver");
MODULE_AUTHOR("Huawei Technologies Co., Ltd.");

