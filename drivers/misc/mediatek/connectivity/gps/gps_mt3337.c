/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define DEVICE_NAME "mt3337_gpsonly"

//Begin Neostra huangxiaohui add for gps power up 20170726
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>
#include <linux/delay.h>

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0
static unsigned int GPIO_GPS_LDO_EN;
static unsigned int GPIO_GPS_RST_EN;
static unsigned int GPIO_GPS_STB_EN;

void gps_ldo_en_get_gpio_infor(void)
{
  static struct device_node *node;

  node = of_find_compatible_node(NULL, NULL, "mediatek,mt3337");
  if (node == NULL) {
    printk("gps gpio info - get node failed\n");
   }
   else
   {  
          //printk(" gps ldo en gpio info - get node success\n"); 
          GPIO_GPS_LDO_EN = of_get_named_gpio(node, "gps_ldo_en", 0);
		  GPIO_GPS_RST_EN = of_get_named_gpio(node, "gps_rst_en", 0);
		  GPIO_GPS_STB_EN = of_get_named_gpio(node, "gps_stb_en", 0);
		  //printk("===hxh  GPIO_GPS_LDO_EN=%d!GPIO_GPS_RST_EN=%d!GPIO_GPS_STB_EN=%d!\n",GPIO_GPS_LDO_EN,GPIO_GPS_RST_EN,GPIO_GPS_STB_EN); 
   }
}

static void gps_ldo_en_set_gpio_output(unsigned int GPIO, unsigned int output)
{
      gpio_direction_output(GPIO, output);
      gpio_set_value(GPIO, output);
}
//End Neostra huangxiaohui add for gps power up 20170726

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#define GPS_POWER_IOCTL _IOW('G', 0, int)

static long gps_ioctrl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev;
	struct regulator *reg;
	int ret;

	switch (cmd) {
	case GPS_POWER_IOCTL:
		dev = (struct miscdevice *)filp->private_data;
		reg = dev_get_drvdata(dev->this_device);
		if (arg)
		{
			ret = regulator_enable(reg);
			gps_ldo_en_set_gpio_output(GPIO_GPS_LDO_EN,GPIO_OUT_ONE);
			mdelay(20);	
            gps_ldo_en_set_gpio_output(GPIO_GPS_RST_EN,GPIO_OUT_ONE);			
		}
		else
		{
			ret = regulator_disable(reg);
			gps_ldo_en_set_gpio_output(GPIO_GPS_LDO_EN,GPIO_OUT_ZERO);
			mdelay(20);
            gps_ldo_en_set_gpio_output(GPIO_GPS_RST_EN,GPIO_OUT_ZERO);			
		}
		return ret;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations gps_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gps_ioctrl,
	.compat_ioctl = gps_ioctrl,
};

static struct miscdevice mt3337_gps_driver = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &gps_fops,
};

static int mt3337_gps_probe(struct platform_device *pdev)
{
	int ret;
	struct regulator *regulator;

    //Begin Neostra huangxiaohui add for gps power up 20170726
    gps_ldo_en_get_gpio_infor();
	gps_ldo_en_set_gpio_output(GPIO_GPS_LDO_EN,GPIO_OUT_ONE);
	mdelay(20);
	gps_ldo_en_set_gpio_output(GPIO_GPS_RST_EN,GPIO_OUT_ONE);
	mdelay(20);
	gps_ldo_en_set_gpio_output(GPIO_GPS_LDO_EN,GPIO_OUT_ZERO);
	mdelay(20);
	gps_ldo_en_set_gpio_output(GPIO_GPS_RST_EN,GPIO_OUT_ZERO);
	mdelay(20);
	//End Neostra huangxiaohui add for gps power up 20170726
	
	regulator = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR(regulator))
		return PTR_ERR(regulator);

	ret = misc_register(&mt3337_gps_driver);
	if (ret < 0)
		return ret;

	dev_set_drvdata(mt3337_gps_driver.this_device, regulator);

	return 0;
}

static const struct of_device_id mt3337_gps_of_match[] = {
	{ .compatible = "mediatek,mt3337" },
	{},
};

static struct platform_driver mt3337_gps_pdriver = {
	.probe = mt3337_gps_probe,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(mt3337_gps_of_match),
	},
};

module_platform_driver(mt3337_gps_pdriver);

MODULE_AUTHOR("Hao Dong <hao.dong@mediatek.com>");
MODULE_DESCRIPTION("MT3337 GPS Driver");
MODULE_LICENSE("GPL");
