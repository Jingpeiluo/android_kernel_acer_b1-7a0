#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif
#include "lcm_drv.h"

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE	0xFF   /* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE	0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

/**
 * Local Variables
 */
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))



/**
 * Local Functions
 */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg				lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifndef ASSERT
#define ASSERT(expr)					\
	do {						\
		if (expr)				\
			break;				\
		pr_debug("DDP ASSERT FAILED %s, %d\n",	\
		       __FILE__, __LINE__);		\
		BUG();					\
	} while (0)
#endif

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


#if 1
static struct LCM_setting_table lcm_initialization_setting[] = {
//JD9365+AUO10.1 initial code
//Page0
{0xE0,	1	,{0x00}},

//--- PASSWORD  ----//
{0xE1,	1	,{0x93}},
{0xE2,	1	,{0x65}},
{0xE3,	1	,{0xF8}},
{0x80,	1	,{0x03}},  //4lanes

//Lane select by internal reg
{0xE0,	1	,{0x04}},
{0x09,	1	,{0x10}},
{0x2B,	1	,{0x2B}},  //DSI ESD protect option
{0x2D,	1	,{0x03}},  //4lanes
{0x2E,	1	,{0x44}},  //DSI ESD protect option	

//--- Page1  ----//
{0xE0,	1	,{0x01}},

//Set VCOM
{0x00,	1	,{0x00}},
{0x01,	1	,{0x6A}},   //0x9A

//Set Gamma Power,	1	,{ VGMP,	1	,{VGMN,	1	,{VGSP,	1	,{VGSN
{0x17,	1	,{0x00}},  
{0x18,	1	,{0xC0}},  //VGMP
{0x19,	1	,{0x01}},  //VGSP
{0x1A,	1	,{0x00}},   
{0x1B,	1	,{0xC0}},  //VGMN
{0x1C,	1	,{0x01}},  //VGSN

//Set Gate Power
{0x1F,	1	,{0x6A}},	//VGH_REG=15V,	1	,{0x6B(15.2)
{0x20,	1	,{0x23}},	//VGL_REG=-10V,	1	,{0x33(-13.2)
{0x21,	1	,{0x23}},	//VGL_REG2=-10V,	1	,{0x33(-13.2)
{0x22,	1	,{0x0E}},	//0x3E

{0x35,	1	,{0x28}},

//SET RGBCYC
{0x37,	1	,{0x59}},	  //SS=1,	1	,{ BGR=1,	1	,{ REV=0
{0x38,	1	,{0x05}},	  //JDT=100 column
{0x39,	1	,{0x04}},	  //RGB_N_EQ1,	1	,{
{0x3A,	1	,{0x08}},	  //RGB_N_EQ2,	1	,{
{0x3B,	1	,{0x08}},	  //RGB_N_EQ2,	1	,{
{0x3C,	1	,{0x7C}},	  //SET EQ3 for TE_H
{0x3D,	1	,{0xFF}},	  //SET CHGEN_ON,	1	,{ 
{0x3E,	1	,{0xFF}},	  //SET CHGEN_OFF,	1	,{ 
{0x3F,	1	,{0xFF}},	  //SET CHGEN_OFF2,	1	,{

//Set TCON
{0x40,	1	,{0x06}},	//RSO 06h=800,	1	,{
{0x41,	1	,{0xA0}},	//LN=640->1280 line

{0x43,	1	,{0x08}},	//VFP
{0x44,	1	,{0x0B}},	//VBP
{0x45,	1	,{0x88}},	//HBP

//{0x4A,	1	,{0x35}},  //BIST

//--- power voltage  ----//
{0x55,	1	,{0x01}},	//DCDCM=1111,	1	,{ No output
{0x56,	1	,{0x01}},
{0x57,	1	,{0xAD}},	//VGH,	1	,{0xAC
{0x58,	1	,{0x0A}},	//AVDD
{0x59,	1	,{0x0A}},	//VCL &AVEE,	1	,{0xFA
{0x5A,	1	,{0x28}},	//VGH ,	1	,{15V
{0x5B,	1	,{0x1E}},	//VGL,	1	,{-10V 

//--- Gamma  ----//
{0x5D,	1	,{0x5C}},  //255
{0x5E,	1	,{0x40}},  //251
{0x5F,	1	,{0x30}},  //247
{0x60,	1	,{0x24}},  //243
{0x61,	1	,{0x21}},  //235
{0x62,	1	,{0x13}},  //227
{0x63,	1	,{0x1A}},  //211
{0x64,	1	,{0x07}},  //191
{0x65,	1	,{0x23}},  //159
{0x66,	1	,{0x25}},  //128
{0x67,	1	,{0x27}},  //96 
{0x68,	1	,{0x47}},  //64 
{0x69,	1	,{0x37}},  //44 
{0x6A,	1	,{0x3F}},  //28 
{0x6B,	1	,{0x32}},  //20 
{0x6C,	1	,{0x2E}},  //12 
{0x6D,	1	,{0x22}},  //8  
{0x6E,	1	,{0x11}},  //4  
{0x6F,	1	,{0x00}},  //0  
{0x70,	1	,{0x5C}},  //255 
{0x71,	1	,{0x40}},  //251 
{0x72,	1	,{0x30}},  //247 
{0x73,	1	,{0x24}},  //243 
{0x74,	1	,{0x21}},  //235 
{0x75,	1	,{0x13}},  //227 
{0x76,	1	,{0x1A}},  //211 
{0x77,	1	,{0x07}},  //191 
{0x78,	1	,{0x23}},  //159 
{0x79,	1	,{0x25}},  //128 
{0x7A,	1	,{0x27}},  //96  
{0x7B,	1	,{0x47}},  //64  
{0x7C,	1	,{0x37}},  //44  
{0x7D,	1	,{0x3F}},  //28  
{0x7E,	1	,{0x32}},  //20  
{0x7F,	1	,{0x2E}},  //12  
{0x80,	1	,{0x22}},  //8   
{0x81,	1	,{0x11}},  //4   
{0x82,	1	,{0x00}},  //0   

//Page2,	1	,{ for GIP
{0xE0,	1	,{0x02}},

//GIP_L Pin mapping
{0x00,	1	,{0x44}},
{0x01,	1	,{0x44}},
{0x02,	1	,{0x45}},
{0x03,	1	,{0x45}},
{0x04,	1	,{0x46}},
{0x05,	1	,{0x46}},
{0x06,	1	,{0x47}},
{0x07,	1	,{0x47}},
{0x08,	1	,{0x1D}},
{0x09,	1	,{0x1D}},
{0x0A,	1	,{0x1D}},
{0x0B,	1	,{0x1D}},
{0x0C,	1	,{0x1D}},
{0x0D,	1	,{0x1D}},
{0x0E,	1	,{0x1D}},
{0x0F,	1	,{0x57}},
{0x10,	1	,{0x57}},
{0x11,	1	,{0x58}},
{0x12,	1	,{0x58}},
{0x13,	1	,{0x40}},
{0x14,	1	,{0x55}},
{0x15,	1	,{0x55}},

//GIP_R Pin mapping   
{0x16,	1	,{0x44}},
{0x17,	1	,{0x44}},
{0x18,	1	,{0x45}},
{0x19,	1	,{0x45}},
{0x1A,	1	,{0x46}},
{0x1B,	1	,{0x46}},
{0x1C,	1	,{0x47}},
{0x1D,	1	,{0x47}},
{0x1E,	1	,{0x1D}},
{0x1F,	1	,{0x1D}},
{0x20,	1	,{0x1D}},
{0x21,	1	,{0x1D}},
{0x22,	1	,{0x1D}},
{0x23,	1	,{0x1D}},
{0x24,	1	,{0x1D}},
{0x25,	1	,{0x57}},
{0x26,	1	,{0x57}},
{0x27,	1	,{0x58}},
{0x28,	1	,{0x58}},
{0x29,	1	,{0x40}},
{0x2A,	1	,{0x55}},
{0x2B,	1	,{0x55}},

//GIP Timing	      
{0x58,	1	,{0x40}},
{0x59,	1	,{0x00}},
{0x5A,	1	,{0x00}},
{0x5B,	1	,{0x00}},
{0x5C,	1	,{0x0A}},
{0x5D,	1	,{0x10}},
{0x5E,	1	,{0x01}},
{0x5F,	1	,{0x02}},
{0x60,	1	,{0x10}},
{0x61,	1	,{0x01}},
{0x62,	1	,{0x02}},
{0x63,	1	,{0x0B}},
{0x64,	1	,{0x54}},
{0x65,	1	,{0x45}},
{0x66,	1	,{0x07}},
{0x67,	1	,{0x31}},
{0x68,	1	,{0x0B}},
{0x69,	1	,{0x1E}},
{0x6A,	1	,{0x54}},
{0x6B,	1	,{0x04}},
{0x6C,	1	,{0x00}},
{0x6D,	1	,{0x04}},
{0x6E,	1	,{0x00}},
{0x6F,	1	,{0x88}},
{0x70,	1	,{0x00}},
{0x71,	1	,{0x00}},
{0x72,	1	,{0x06}},
{0x73,	1	,{0x7B}},
{0x74,	1	,{0x00}},
{0x75,	1	,{0xF8}}, 
{0x76,	1	,{0x00}}, 
{0x77,	1	,{0x0D}}, 
{0x78,	1	,{0x14}}, 
{0x79,	1	,{0x00}}, 
{0x7A,	1	,{0x00}}, 
{0x7B,	1	,{0x00}}, 
{0x7C,	1	,{0x00}}, 
{0x7D,	1	,{0x03}}, 
{0x7E,	1	,{0x7B}},                                                                                                    

//Page0
{0xE0,	1	,{0x00}},
{0xE6,	1	,{0x02}},  //enable watch dog timer
{0xE7,	1	,{0x06}},  //setting watch doe timer

// Sleep Out				
{0x11, 0, {}},
{REGFLAG_DELAY, 200, {}},

// Display ON
{0x29,0,{}},
{REGFLAG_DELAY, 20,{}},

{REGFLAG_END_OF_TABLE, 0x00, {}}

//--- TE----//
//{0x35,	1	,{0x00}};

};
#endif





static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
    {REGFLAG_DELAY, 10, {} },
    {0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;
		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}

#ifndef BUILD_LK
static struct regulator *lcm_vgp;

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	printk("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	printk("LCM: lcm get supply ok.\n");

	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	printk("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

static int lcm_jd_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	printk("LCM: lcm_jd_vgp_supply_enable\n");

	if (lcm_vgp == NULL)
		return 0;

	printk("LCM: set regulator voltage lcm_vgp voltage to 3.3V\n");
	
	ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 3300000)
		printk("LCM: check regulator voltage=3300000 pass!\n");
	else
		pr_debug("LCM: check regulator voltage=3300000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

static int lcm_jd_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (lcm_vgp == NULL)
		return 0;


	isenable = regulator_is_enabled(lcm_vgp);

	printk("LCM: lcm query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
	
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
    printk("LCM: lcm_driver_probe\n");
	lcm_get_vgp_supply(dev);
	lcm_jd_vgp_supply_enable();

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "otm,otm1287_wxga_dsi_vdo_auo_guoxian",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "jd9365_wxga_dsi_vdo_auo_guoxian",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_init(void)
{
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
#endif
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
     memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = BURST_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;
#endif



	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;
	

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//video mode timing
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=FRAME_WIDTH*3;

	params->dsi.vertical_sync_active				= 4;//4;//2;
	params->dsi.vertical_backporch					= 8;//12;//16;
	params->dsi.vertical_frontporch					= 8;//20;//9;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 4;//20;//42;
	params->dsi.horizontal_backporch				= 132;//20;//42;
	params->dsi.horizontal_frontporch				= 24;//30;//69;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
    params->dsi.ssc_disable	= 1;
	params->dsi.cont_clock 	= 1;
	params->dsi.PLL_CLOCK   = 230; 
	
	params->dsi.clk_lp_per_line_enable   = 1;
//	params->dsi.esd_check_enable = 1;
//	params->dsi.customization_esd_check_enable = 1;
//	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
//	params->dsi.lcm_esd_check_table[0].count = 1;
//	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
}

static void lcm_init_lcm(void)
{
	
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
	
	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	

}

static void lcm_suspend(void)
{
	
	push_table(lcm_deep_sleep_mode_in_setting,
		   sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
    lcm_jd_vgp_supply_disable();
	MDELAY(20);
}

static void lcm_resume(void)
{
    lcm_jd_vgp_supply_enable();
	MDELAY(20);
	lcm_init_lcm();

}

static unsigned int lcm_esd_check(void)
{

	unsigned char buffer[1]= {0};
    unsigned int array[16];
	printk("[cabc] otm1287a: lcm_esd_check enter\n");

   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0A, buffer, 1);
    printk("lcm_esd_check  0x0A = %x\n",buffer[0]);

   if(buffer[0] != 0x9C)
   {
      return 1;
   }

   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0D, buffer, 1);
    printk("lcm_esd_check 0x0D =%x\n",buffer[0]);

   if(buffer[0] != 0x00)
   {
      return 1;
   }
   
   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0E, buffer, 1);
    printk("lcm_esd_check  0x0E = %x\n",buffer[0]);
   if(buffer[0] != 0x80)
   {
      return 1;
   }
   
	printk("[cabc] otm1287a: lcm_esd_check exit\n");
	return 0;
}

static unsigned int lcm_esd_recover(void)
{
         lcm_init_lcm();

	return TRUE;
}



LCM_DRIVER jd9365_wxga_dsi_vdo_auo_guoxian_lcm_drv = {
	.name		= "jd9365_wxga_dsi_vdo_auo_guoxian",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.esd_check		= lcm_esd_check,
	.esd_recover	= lcm_esd_recover,	
#if (LCM_DSI_CMD_MODE)
	/*.set_backlight	= lcm_setbacklight,*/
	/* .set_pwm        = lcm_setpwm, */
	/* .get_pwm        = lcm_getpwm, */
	/*.update         = lcm_update, */
#endif
};
