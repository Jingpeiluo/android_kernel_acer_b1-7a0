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

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0
static unsigned int GPIO_LCM_RST;
static unsigned int GPIO_BIAS_EN;
static unsigned int GPIO_LCM_STB;
//static unsigned int GPIO_LCD_BL_EN;



void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mtk,mt8167-lcm");
	if (node == NULL) {
		//printk("ldk lcm gpio info - get node failed\n");
	 }
	 else
	 {	
       //printk("ldk lcm gpio info - get node success\n"); 
	   GPIO_LCM_RST = of_get_named_gpio(node, "lcm_reset_gpio", 0);
	   GPIO_BIAS_EN = of_get_named_gpio(node, "lcm_bias_en", 0);
	   GPIO_LCM_STB = of_get_named_gpio(node, "lcm_stand_by", 0);
	   //GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_backlight_en", 0);
	 }
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
    gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
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

static int lcm_gx_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	printk("LCM: lcm_gx_vgp_supply_enable\n");

	if (lcm_vgp == NULL)
		return 0;

	printk("LCM: set regulator voltage lcm_vgp voltage to 3.3V\n");
	
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		printk("LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_debug("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

static int lcm_gx_vgp_supply_disable(void)
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
	lcm_get_gpio_infor();
	lcm_get_vgp_supply(dev);
	lcm_gx_vgp_supply_enable();

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "mtk,mt8167-lcm",
		.data = 0,
	}, {
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
		   .name = "KD070D33-30NC-A79",
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


#define FRAME_WIDTH  (1024)
#define FRAME_HEIGHT (600)

#define REGFLAG_DELAY                                                                   0xFE
#define REGFLAG_END_OF_TABLE                                                            0xFD   // END OF REGISTE


#define LCM_DSI_CMD_MODE	0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 
#define MDELAY(n) 

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg				lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------



struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x80,1,{0x58}},
    {0x81,1,{0x47}},
    {0x82,1,{0xd4}},
    {0x83,1,{0x88}},
    {0x84,1,{0xA9}},
    {0x85,1,{0xC3}},
    {0x86,1,{0x82}},
	{0x11,0,{0x00}}, // Sleep-Out
	{REGFLAG_DELAY, 120, {}},
	{0x29,0,{0x00}}, // Display On 
	//{0x2C,1,{0x00}},
	{REGFLAG_DELAY, 30, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 0, {} },
	{REGFLAG_DELAY, 30, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
	{0x28, 0, {} },	
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static __inline void send_ctrl_cmd(unsigned int cmd)
{

}

static __inline void send_data_cmd(unsigned int data)
{

}

static __inline void set_lcm_register(unsigned int regIndex,
                                      unsigned int regData)
{

}


void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
        unsigned int i;
    
    for(i = 0; i < count; i++) {
                
        unsigned cmd;
        cmd = table[i].cmd;
                        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
                                MDELAY(10);
        }
    }
}



// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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
        params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
   #endif

  
                params->dsi.LANE_NUM                            = LCM_FOUR_LANE;
                //The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
                params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//video mode timing
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

                params->dsi.vertical_sync_active                                = 1;// 3    2
                params->dsi.vertical_backporch                                  = 32;// 20   1
                params->dsi.vertical_frontporch                                 = 12; // 1  12
                params->dsi.vertical_active_line                                = FRAME_HEIGHT;
                
                params->dsi.horizontal_sync_active                              = 1;// 50  2
                params->dsi.horizontal_backporch                                = 160;
                params->dsi.horizontal_frontporch                               = 160;
                params->dsi.horizontal_active_pixel                             = FRAME_WIDTH;

   
				params->dsi.PLL_CLOCK   = 170;
	params->dsi.clk_lp_per_line_enable   = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x81;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x47;
}


static void lcm_init_lcm(void)
{ 

    printk("[LCM] lcm_init() enter\n");
    lcm_gx_vgp_supply_enable();
    MDELAY(10);
    lcm_set_gpio_output(GPIO_LCM_STB,GPIO_OUT_ONE);
    MDELAY(20);
    lcm_set_gpio_output(GPIO_LCM_RST,GPIO_OUT_ONE);
    MDELAY(30);        	
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    lcm_set_gpio_output(GPIO_BIAS_EN, GPIO_OUT_ONE);	
    MDELAY(200);
   //lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);

}


static void lcm_suspend(void)
{

    printk("[LCM] lcm_suspend() enter\n");

    //lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
     push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(20);

    lcm_set_gpio_output(GPIO_LCM_RST,0);
	//printk("ldk set lcm rst low\n");
	MDELAY(10);
    lcm_set_gpio_output(GPIO_LCM_STB,0);
	//printk("ldk set lcm stand by low\n");
	MDELAY(20);
    lcm_set_gpio_output(GPIO_BIAS_EN, 0);
	//printk("ldk set lcm bais low\n");
	MDELAY(20);
    lcm_gx_vgp_supply_disable();

    MDELAY(20);     


  
}


static void lcm_resume(void)
{
    printk("[LCM] lcm_resume() enter\n");

    lcm_gx_vgp_supply_enable();
    MDELAY(10);	
    lcm_set_gpio_output(GPIO_BIAS_EN, 1);
	//printk("ldk set lcm bais high\n");
    lcm_set_gpio_output(GPIO_LCM_STB, 1);
	//printk("ldk set lcm stand by high\n");
	MDELAY(10);	
    lcm_set_gpio_output(GPIO_LCM_RST, 1);
	//printk("ldk set lcm rst high\n");
    MDELAY(30);
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);	

    //lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);

}

static unsigned int lcm_esd_check(void)
{

#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x81, buffer, 1);

	if (buffer[0] != 0x47) {
		printk("[LCM ERROR] [0x81]=0x%02x\n", buffer[0]);
		return TRUE;
	} else {
		printk("[LCM NORMAL] [0x81]=0x%02x\n", buffer[0]);
		return FALSE;
	}
#else
	return FALSE;
#endif
}


static unsigned int lcm_esd_recover(void)
{
	lcm_init_lcm();

	return TRUE;
}

LCM_DRIVER k706_gx_ek79007_wsvga_ips_cpt_lcm_drv = 
{
    .name		= "KD070D33-30NC-A79",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.esd_check		= lcm_esd_check,
	.esd_recover	= lcm_esd_recover,
};

