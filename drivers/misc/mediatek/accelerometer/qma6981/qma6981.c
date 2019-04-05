/* qma6981 motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#include <accel.h>
#include <cust_acc.h>
#include <hwmsensor.h>
#include "qma6981.h"


//#define QMA6981_STEP_COUNTER
#ifdef QMA6981_STEP_COUNTER
#define QMA6981_STEP_COUNTER_USE_INT
#define QMA6981_CHECK_ABNORMAL_DATA
//#define QMA6981_STEP_TO_ZERO
//#define QMA6981_SELECT_AXIS
#endif

#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/pm.h>

 //step counter
#include <asm/io.h>
#include <linux/sched.h>
#ifdef QMA6981_STEP_COUNTER
#include <step_counter.h>
#endif

#define QMA6981_RETRY_COUNT 3

#define qma6981_BUFSIZE		256

#define GSE_TAG                  "[QMA-Gsensor] "
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_FUN(f)               pr_debug(GSE_TAG "%s\n", __FUNCTION__)
#define GSE_DEBUG
#ifdef GSE_DEBUG
#define GSE_LOG(fmt, args...)    pr_debug(GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define GSE_LOG(fmt, args...)    
#endif

#define kal_bool	bool
#define KAL_TRUE	true
#define KAL_FALSE	false

static struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;
static int qma6981_init_flag =0;

static DEFINE_MUTEX(qma6981_init_mutex);
static DEFINE_MUTEX(qma6981_mutex);

#if defined(QMA6981_STEP_COUNTER_USE_INT)
//extern struct platform_device *get_acc_platformdev(void);
//struct platform_device *accPltFmDev;
#endif


#if defined(CONFIG_HAS_EARLYSUSPEND)
static void qma6981_early_suspend(struct early_suspend *h);
static void qma6981_late_resume(struct early_suspend *h);
#endif


static struct acc_init_info qma6981_init_info;
#ifdef QMA6981_STEP_COUNTER
static struct step_c_init_info qma6981_step_c_init_info;
static struct wake_lock sc_wakelock;
static kal_bool wake_lock_status = KAL_FALSE;
static int sc_reset_flag = KAL_TRUE;// store last data	
static int qma6981_step_c_read_stepCounter(int *data);
static void qma6981_set_wakelock(int en);

#if 1//defined(QMA6981_STEP_TO_ZERO)
static char qma6981_reg13_value = 0x10;
#endif
#define QMA6981_REG13_LOW_VALUE			0x0c	//0x0a
#define QMA6981_REG13_HIGH_VALUE		0x11	//0x10

#endif


struct qma6981_data{
	struct 	acc_hw *hw;
	struct 	i2c_client *client;
	struct 	hwmsen_convert   cvt;	
	int 					 sensitivity;
    atomic_t   				 layout;
	atomic_t                 trace;
	atomic_t                 suspend;

	s16                      cali_sw[QMA6981_AXES_NUM+1];
	/*data*/
	s8                       offset[QMA6981_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	s16                      data[QMA6981_AXES_NUM+1];
	u8                       bandwidth;
	
	/*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif 
#ifdef QMA6981_STEP_COUNTER_USE_INT
	struct 	work_struct  eint_work;
	struct device_node *irq_node;
	int 	irq;
#endif
};

#ifdef QMA6981_STEP_COUNTER_USE_INT
#define STEP_INT_START_VLUE	4
//#define STEP_DUMMY_VLUE		6
static int STEP_DUMMY_VLUE = 8;
#define STEP_END	KAL_FALSE
#define STEP_START	KAL_TRUE

struct qma6981_stepcount{
	//int stepcounter_pre_start;
	int stepcounter_pre_end;  
	int stepcounter_next_start;
	int stepcounter_next_end;  
	int stepcounter_pre;
	int stepcounter_pre_fix;
	kal_bool stepcounter_statu;
	kal_bool stepcounter_start_int_update_diff;
	int back;
	int step_diff;
};
kal_bool int_statu_flag = KAL_FALSE;
static struct qma6981_stepcount step_count_index;
static struct qma6981_data *g_QMA6981_ptr = NULL;
struct platform_device  *qma6981_dev;
#endif


#if defined(QMA6981_SELECT_AXIS)

#if 1
#define QMA6981_SELECT_AXIS_WHEN_INT
#define QMA6981_ACC_SIZE	15

#else
//for weixin test define ,start
#define QMA6981_SELECT_AXIS_WHEN_TIMER
#define QMA6981_ACC_SIZE	10 
//for weixin test define ,end
#endif

#define QMA6981_USE_AXIS_Z_DEFAULT	KAL_TRUE

#if defined(QMA6981_SELECT_AXIS_WHEN_TIMER)||defined(QMA6981_SELECT_AXIS_WHEN_INT)
static int qma6981_acc_z_index = 0;
static int qma6981_acc_data[QMA6981_AXES_NUM]={0, 0, 0};
static int qma6981_acc_x_min;
static int qma6981_acc_x_max;
static int qma6981_acc_y_min;
static int qma6981_acc_y_max;
static int qma6981_acc_z_min;
static int qma6981_acc_z_max;
#endif

static kal_bool qma6981_use_axis_z = QMA6981_USE_AXIS_Z_DEFAULT;//KAL_FALSE;
static int qma6981_enable_axis_select = 1;
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
static kal_bool axis_select_timer_running = KAL_FALSE;
#elif defined(QMA6981_SELECT_AXIS_WHEN_TIMER)
static kal_bool axis_select_timer_running = KAL_TRUE;
#endif
static void qm6981_step_c_use_axis_z(kal_bool is_axis_z);
extern void step_c_set_delay(int new_mdelay);
#endif

static struct qma6981_data *qma6981 = NULL;
struct i2c_client *qma6981_i2c_client = NULL;
static bool sensor_power = true;
static bool enable_status = false;
int QMA6981_Chip_id = 0;
//static unsigned long long int_top_time = 0;
static u8 qm6981_248g_offset = 8;
#if defined(QMA6981_CHECK_ABNORMAL_DATA)
static struct timespec last_read_time = {0, 0};
static int last_read_data = -1;
#define CHECK_IN_TIME		2
#define CHECK_IN_DIFF_STEP	30
#endif


static int I2C_RxData(char *rxData, int len)
{
	int err = 0;
	u8 beg = *rxData;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&qma6981_mutex);
	msgs[0].addr = qma6981_i2c_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = qma6981_i2c_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = rxData;

	if (!qma6981_i2c_client) {
		mutex_unlock(&qma6981_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&qma6981_mutex);
		GSE_ERR("length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(qma6981_i2c_client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n", qma6981_i2c_client->addr, rxData, len, err);
		err = -EIO;
	} else
		err = 0;
	mutex_unlock(&qma6981_mutex);
	return err;

}



static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;

	mutex_lock(&qma6981_mutex);
	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		mutex_unlock(&qma6981_mutex);
		return -EINVAL;
	}
#ifdef CONFIG_MTK_I2C_EXTENSION
	qma6981_i2c_client->addr = qma6981_i2c_client->addr & I2C_MASK_FLAG;
#endif	
	for(loop_i = 0; loop_i < QMA6981_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(qma6981_i2c_client, (const char*)txData, length) > 0)
			break;

		GSE_ERR("I2C_TxData delay!\n");
		mdelay(10);
	}

	if(loop_i >= QMA6981_RETRY_COUNT)
	{
		GSE_ERR( "%s retry over %d\n", __func__, QMA6981_RETRY_COUNT);	
		mutex_unlock(&qma6981_mutex);
		return -EIO;
	}
	mutex_unlock(&qma6981_mutex);
	return 0;
}

/* X,Y and Z-axis accelerometer data readout
*/
static int qma6981_read_raw_xyz(int *data){
	//int res;	
	unsigned char databuf[6] = {0};		
	unsigned char i;	

	databuf[0] = QMA6981_XOUTL;		
	if(0 != I2C_RxData(databuf, 6)){
		GSE_ERR("read xyz error!!!");
		return -EFAULT;	
	}	

 	data[0]  = (s16)((databuf[1]<<2) |( databuf[0]>>6));
	data[1]  = (s16)((databuf[3]<<2) |( databuf[2]>>6));
	data[2]  = (s16)((databuf[5]<<2) |( databuf[4]>>6));

#if 1
	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x0200 )	//so we want to calculate actual number here
			data[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x0200 )	//transfor format
		{					//printk("data 0 step %x \n",data[i]);
			data[i] -= 0x1;			//printk("data 1 step %x \n",data[i]);
			data[i] = ~data[i];		//printk("data 2 step %x \n",data[i]);
			data[i] &= 0x01ff;		//printk("data 3 step %x \n\n",data[i]);
			data[i] = -data[i];		
		}
		#ifdef QMA6981_STEP_COUNTER
		data[i] -= QMA6981_OFS;
		#endif
	}
	//GSE_LOG("qma6981 raw LBS data  %d,%d,%d\n",data[0],data[1],data[2]);
#endif
	//printk("yzqaccraw	%d	%d	%d\n", data[0], data[1], data[2]);

	return 0;
}

static int qma6981_read_acc_xyz(int *data){
	
	int raw[3]={0};
	int acc[3]={0};
	int ret = 0;

	ret = qma6981_read_raw_xyz(raw);
	if(0 != ret ){
		GSE_ERR("qma6981_read_acc_xyz error\n");
	}
	
	//GSE_LOG("qma6981 AFTER x4:%d,y:%d,z:%d\n",raw[0],raw[1],raw[2]);

	raw[QMA6981_AXIS_X] += qma6981->cali_sw[QMA6981_AXIS_X];
	raw[QMA6981_AXIS_Y] += qma6981->cali_sw[QMA6981_AXIS_Y];
	raw[QMA6981_AXIS_Z] += qma6981->cali_sw[QMA6981_AXIS_Z];
	
	//remap coordinate
	acc[qma6981->cvt.map[QMA6981_AXIS_X]] = qma6981->cvt.sign[QMA6981_AXIS_X]*raw[QMA6981_AXIS_X];
	acc[qma6981->cvt.map[QMA6981_AXIS_Y]] = qma6981->cvt.sign[QMA6981_AXIS_Y]*raw[QMA6981_AXIS_Y];
	acc[qma6981->cvt.map[QMA6981_AXIS_Z]] = qma6981->cvt.sign[QMA6981_AXIS_Z]*raw[QMA6981_AXIS_Z];
	//GSE_LOG("qma6981 AFTER x1:%d,y:%d,z:%d\n",data[0],data[1],data[2]);

	data[QMA6981_AXIS_X] = (acc[QMA6981_AXIS_X]*GRAVITY_EARTH_1000)>>(qm6981_248g_offset);
	data[QMA6981_AXIS_Y] = (acc[QMA6981_AXIS_Y]*GRAVITY_EARTH_1000)>>(qm6981_248g_offset);
	data[QMA6981_AXIS_Z] = (acc[QMA6981_AXIS_Z]*GRAVITY_EARTH_1000)>>(qm6981_248g_offset);

	//printk("yzqacc	%d	%d	%d\n", data[0], data[1], data[2]);

	return ret;
}


/*----------------------------------------------------------------------------*/
static int qma6981_ReadOffset(struct i2c_client *client, s8 ofs[QMA6981_AXES_NUM])
{    
	int err = 0;

	ofs[1]=ofs[2]=ofs[0]=0x00;

	//GSE_LOG("qma6981 offesx=%x, y=%x, z=%x\n",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}

/*----------------------------------------------------------------------------*/
static int qma6981_ResetCalibration(struct i2c_client *client)
{
	struct qma6981_data *obj = i2c_get_clientdata(client);
	int err = 0;
	//GSE_LOG("qma6981 qma6981_ResetCalibration\n");
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}

/*----------------------------------------------------------------------------*/
static int qma6981_ReadCalibrationEx(struct i2c_client *client, int act[QMA6981_AXES_NUM], int raw[QMA6981_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct qma6981_data *obj = i2c_get_clientdata(client);
	int mul;

	mul = 0;//only SW Calibration, disable HW Calibration
	//GSE_LOG("qma6981 qma6981_ReadCalibrationEx\n");
	raw[QMA6981_AXIS_X] = obj->offset[QMA6981_AXIS_X]*mul + obj->cali_sw[QMA6981_AXIS_X];
	raw[QMA6981_AXIS_Y] = obj->offset[QMA6981_AXIS_Y]*mul + obj->cali_sw[QMA6981_AXIS_Y];
	raw[QMA6981_AXIS_Z] = obj->offset[QMA6981_AXIS_Z]*mul + obj->cali_sw[QMA6981_AXIS_Z];

	act[obj->cvt.map[QMA6981_AXIS_X]] = obj->cvt.sign[QMA6981_AXIS_X]*raw[QMA6981_AXIS_X];
	act[obj->cvt.map[QMA6981_AXIS_Y]] = obj->cvt.sign[QMA6981_AXIS_Y]*raw[QMA6981_AXIS_Y];
	act[obj->cvt.map[QMA6981_AXIS_Z]] = obj->cvt.sign[QMA6981_AXIS_Z]*raw[QMA6981_AXIS_Z];                        
	                       
	return 0;
}

/*----------------------------------------------------------------------------*/
static int qma6981_WriteCalibration(struct i2c_client *client, int dat[QMA6981_AXES_NUM])
{
	struct qma6981_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[QMA6981_AXES_NUM], raw[QMA6981_AXES_NUM];

	if(0 != qma6981_ReadCalibrationEx(client, cali, raw))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

		/*calculate the real offset expected by caller*/
	cali[QMA6981_AXIS_X] += dat[QMA6981_AXIS_X];
	cali[QMA6981_AXIS_Y] += dat[QMA6981_AXIS_Y];
	cali[QMA6981_AXIS_Z] += dat[QMA6981_AXIS_Z];

	//GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
	//	dat[QMA6981_AXIS_X], dat[QMA6981_AXIS_Y], dat[QMA6981_AXIS_Z]);

	obj->cali_sw[QMA6981_AXIS_X] = obj->cvt.sign[QMA6981_AXIS_X]*(cali[obj->cvt.map[QMA6981_AXIS_X]]);
	obj->cali_sw[QMA6981_AXIS_Y] = obj->cvt.sign[QMA6981_AXIS_Y]*(cali[obj->cvt.map[QMA6981_AXIS_Y]]);
	obj->cali_sw[QMA6981_AXIS_Z] = obj->cvt.sign[QMA6981_AXIS_Z]*(cali[obj->cvt.map[QMA6981_AXIS_Z]]);	


	return err;
}
/*----------------------------------------------------------------------------*/
static int qma6981_ReadCalibration(struct i2c_client *client, int dat[QMA6981_AXES_NUM])
{
    struct qma6981_data *obj = i2c_get_clientdata(client);
    int mul;
	
	mul = 0;//only SW Calibration, disable HW Calibration
	
    dat[obj->cvt.map[QMA6981_AXIS_X]] = obj->cvt.sign[QMA6981_AXIS_X]*(obj->offset[QMA6981_AXIS_X]*mul + obj->cali_sw[QMA6981_AXIS_X]);
    dat[obj->cvt.map[QMA6981_AXIS_Y]] = obj->cvt.sign[QMA6981_AXIS_Y]*(obj->offset[QMA6981_AXIS_Y]*mul + obj->cali_sw[QMA6981_AXIS_Y]);
    dat[obj->cvt.map[QMA6981_AXIS_Z]] = obj->cvt.sign[QMA6981_AXIS_Z]*(obj->offset[QMA6981_AXIS_Z]*mul + obj->cali_sw[QMA6981_AXIS_Z]);                    
    return 0;

	
}/*-----------------------------------------------------------------------------*/

#ifdef QMA6981_STEP_COUNTER_USE_INT

void QMA6981_eint_func(void)
{
	struct qma6981_data *obj = g_QMA6981_ptr;
	//GSE_LOG("step__QMA6981_eint_func\n");
	if(!obj)
	{
		return;
	}	
	//int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}


/*----------------------------------------------------------------------------*/

static irqreturn_t QMA6981_eint_handler(int irq, void *desc)
{
	//struct qma6981_data *obj = g_QMA6981_ptr;
	//printk("QMA6981_eint_handler");
	QMA6981_eint_func();
	//disable_irq_nosync(obj->irq);		// add by yangzhiqiang 2017/03/02

	return IRQ_HANDLED;
}

int QMA6981_setup_eint(struct i2c_client *client)
{
	struct qma6981_data *obj = i2c_get_clientdata(client);        
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};
	
	g_QMA6981_ptr = obj;

	memset(&step_count_index, 0, sizeof(step_count_index));

	//accPltFmDev = get_acc_platformdev();
/* gpio setting */
	pinctrl = devm_pinctrl_get(&qma6981_dev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		GSE_ERR("Cannot find gsensor pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		GSE_ERR("Cannot find gsensor pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "qma6981_eint_as_int");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		GSE_LOG("Cannot find gsensor pinctrl pin_cfg!\n");

	}
//	GSE_LOG("pins_cfg = %d\n", pins_cfg);
	pinctrl_select_state(pinctrl, pins_cfg);
	/* eint request */
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");
	if (obj->irq_node) {
		of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_request(ints[0], "gsensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		GSE_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
		GSE_LOG("obj->irq = %d\n", obj->irq);
		if (!obj->irq) {
			GSE_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(obj->irq, QMA6981_eint_handler, IRQF_TRIGGER_RISING, "gsensor-eint", NULL)) {
			GSE_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		disable_irq(obj->irq);
		enable_irq(obj->irq);
	} else {
		GSE_ERR("null irq node!!\n");
		return -EINVAL;
	}

    return 0;
}

static void QMA6981_eint_work(struct work_struct *work)
{
	//struct qma6981_data *obj = (struct qma6981_data *)container_of(work, struct qma6981_data, eint_work);

	int res = 0,temp = 0;
	char databuf[2];
	//u16 *data;
	int data = 0;
	
	//disable_irq_nosync(obj->irq);


	qma6981_set_wakelock(1);
	databuf[0] = QMA6981_STEP_CNT_L;
	
	if((res = I2C_RxData(databuf, 2))){
		if((res = I2C_RxData(databuf, 2)))	// if error read again
		{
			qma6981_set_wakelock(0);
			//enable_irq(g_QMA6981_ptr->irq);
			GSE_ERR("QMA6981_SC_stepcounter error!!!");
			return;
		}
	}
	data = (databuf[1]<<8)|databuf[0];
	//qma6981_set_wakelock(0);
	GSE_LOG("int	reg data=	%d\n", data);
	
	if (int_statu_flag)
	{	
		int_statu_flag = KAL_FALSE;
		//mutex_lock(&qma6981_mutex);
		step_count_index.stepcounter_next_end = data;
		step_count_index.stepcounter_statu = STEP_END;
		GSE_LOG("step_int stepcounter_next_end = %d stepcounter_next_start = %d\n", 					   step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start);
		if (step_count_index.stepcounter_next_end < step_count_index.stepcounter_next_start)
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start;
		}
		GSE_LOG("step_int_end  temp =%d\n" ,temp);
		if (temp < STEP_DUMMY_VLUE)
		{
			step_count_index.step_diff += (temp+STEP_INT_START_VLUE);
			// add by yangzhiqiang for step_diff
			if(step_count_index.step_diff > data)
			{
				step_count_index.step_diff = data;
			}
			// yangzhiqiang for step_diff
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
			
		}
		else
		{
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
		}
		//mutex_unlock(&qma6981_mutex);
		
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
		databuf[0] = 0x13;
		databuf[1] = QMA6981_REG13_LOW_VALUE;	//0x09; 	// yzq when int end set 140mg
#if defined(QMA6981_STEP_TO_ZERO)
		qma6981_reg13_value = databuf[1];
#endif
		res = I2C_TxData(databuf,2);
		if(res < 0)
		{
			res = I2C_TxData(databuf,2);
			if(res < 0)
			{
				GSE_LOG("int	write 0x13=0x08 error\n");
			}
		}

		if(1)
		{
			int acc_data[3];
			
			if((res = qma6981_read_acc_xyz(acc_data)))
			{
				if((res = qma6981_read_acc_xyz(acc_data)))
				{
					GSE_LOG("int qma6981_read_acc_xyz error \n");
				}
			}
			if(QMA6981_ABS(acc_data[0]) > 8000)
			{
				qm6981_step_c_use_axis_z(KAL_FALSE);
			}
			else
			{			
				qm6981_step_c_use_axis_z(QMA6981_USE_AXIS_Z_DEFAULT);
			}
		}
		
		step_c_set_delay(200);
		axis_select_timer_running = KAL_FALSE;
#endif
		//res = request_irq(g_QMA6981_ptr->irq, QMA6981_eint_handler, IRQF_TRIGGER_RISING, "gsensor-eint", NULL);
		irq_set_irq_type(g_QMA6981_ptr->irq,IRQF_TRIGGER_RISING);
		printk("step_int_end\n" );
	}
	else
	{
		int_statu_flag = KAL_TRUE;
		//mutex_lock(&qma6981_mutex);
		step_count_index.stepcounter_next_start= data;
		step_count_index.stepcounter_statu = STEP_START;
		GSE_LOG("step_int stepcounter_next_start = %d stepcounter_pre_end = %d\n", step_count_index.stepcounter_next_start,step_count_index.stepcounter_pre_end);
		if (step_count_index.stepcounter_next_start < step_count_index.stepcounter_pre_end)
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end;
		}
		GSE_LOG("step_int_start temp =%d\n" ,temp);
		if (temp >STEP_INT_START_VLUE)
		{
			step_count_index.step_diff += (temp - STEP_INT_START_VLUE);
		}
		//mutex_unlock(&qma6981_mutex);
// add by yangzhiqiang
		//step_notify(TYPE_STEP_DETECTOR);
// yangzhiqiang
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
		databuf[0] = 0x13;
		databuf[1] = QMA6981_REG13_HIGH_VALUE;		// yzq when int start set 250mg
#if defined(QMA6981_STEP_TO_ZERO)
		qma6981_reg13_value = databuf[1];
#endif
		res = I2C_TxData(databuf,2);
	   	if(res < 0)
	   	{
	   		res = I2C_TxData(databuf,2);
			if(res < 0)
			{
				GSE_LOG("int	write 0x13=0x10	error\n");
			}
	   	}

		// use timer to read acc data to change axis
		step_c_set_delay(100);
		qma6981_acc_z_index = 0;
		axis_select_timer_running = KAL_TRUE;
		// use timer to read acc data to change axis
#endif
		//res = request_irq(g_QMA6981_ptr->irq, QMA6981_eint_handler, IRQF_TRIGGER_FALLING, "gsensor-eint", NULL);
		irq_set_irq_type(g_QMA6981_ptr->irq,IRQF_TRIGGER_FALLING);
		printk("step_int_start\n" );
	}
	qma6981_set_wakelock(0);	// move wake lock here!
	enable_irq(g_QMA6981_ptr->irq);		// add by yangzhiqiang 2017/03/02

	return;
#if 0
	EXIT_ERR:
	enable_irq(obj->irq);

	GSE_LOG("i2c_transfer error = %d\n", res);
	return;
#endif
}
#endif

static ssize_t show_dumpallreg_value(struct device_driver *ddri, char *buf)
{
	int res;
	int i =0;
	char strbuf[1024];
	char tempstrbuf[24];
	unsigned char databuf[2];
	int length=0;

	GSE_FUN();

	/* Check status register for data availability */
	for(i =0;i<64;i++)
	{
		databuf[0] = i;
		res = I2C_RxData(databuf, 1);
		if(res < 0)
			GSE_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "reg[0x%2x] =  0x%2x \n",i, databuf[0]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s \n",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}


/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[qma6981_BUFSIZE];
	int output;
	unsigned char databuf;


	databuf = QMA6981_CHIP_ID;
	if(0 != I2C_RxData(&databuf, 1))
	{
		GSE_LOG("000000000  show_chipinfo_value read chip id =%x!!!\n", databuf);
		GSE_LOG("read chip id error!!!");
		return -EFAULT;
	}
	GSE_LOG("show_chipinfo_value read chip id =%x!!!\n", databuf);
	output = (int)databuf;

	sprintf(strbuf, "chipid:%d \n", output);

	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_waferid_value(struct device_driver *ddri, char *buf)
{
	int res;
	
	unsigned int chipid;
	unsigned char chipidh;
	unsigned char chipidl;
	
	unsigned char waferid;
	unsigned char waferid1;
	unsigned char waferid2;
	unsigned char waferid3;

	
	chipidh = 0x48;
	if((res = I2C_RxData(&chipidh, 1)))
	{
		GSE_LOG("read wafer chip h error!!!\n");
		return -EFAULT;
	}
	chipidl = 0x47;
	if((res = I2C_RxData(&chipidl, 1)))
	{
		GSE_LOG("read wafer chip l error!!!\n");
		return -EFAULT;
	}
	GSE_LOG("read wafer chip H:0x%x L:0x%x", chipidh, chipidl);
	chipid = (chipidh<<8)|chipidl;
	
	waferid1 = 0x59;
	if((res = I2C_RxData(&waferid1, 1)))
	{
		GSE_LOG("read wafer id 1 error!!!\n");
		return -EFAULT;
	}
	waferid2 = 0x41;
	if((res = I2C_RxData(&waferid2, 1)))
	{
		GSE_LOG("read wafer id 2 error!!!\n");
		return -EFAULT;
	}
	waferid3 = 0x40;
	if((res = I2C_RxData(&waferid3, 1)))
	{
		GSE_LOG("read wafer id 3 error!!!\n");
		return -EFAULT;
	}
	
	GSE_LOG("wafer ID: 0x%x 0x%x 0x%x\n", waferid1, waferid2, waferid3);
	
	waferid = (waferid1&0x10)|((waferid2>>4)&0x0c)|((waferid3>>6)&0x03);

	return sprintf(buf, " Chip id:0x%x \n Wafer ID 0x%02x\n", chipid, waferid);
}


/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct qma6981_data *obj = i2c_get_clientdata(qma6981_i2c_client);
	if(NULL == obj)
	{
		GSE_LOG("qma6981_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct qma6981_data *obj = i2c_get_clientdata(qma6981_i2c_client);
	int trace;
	if(NULL == obj)
	{
		GSE_LOG("qma6981_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		GSE_LOG("invalid content: '%s', length = %d\n", buf, (int)count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = qma6981_i2c_client;
	struct qma6981_data *obj;
	int err, len = 0, mul;
	int tmp[QMA6981_AXES_NUM];

	if(NULL == client)
	{
		GSE_LOG("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	
	err = qma6981_ReadOffset(client, obj->offset);
	if(0 != err)
	{
		return -EINVAL;
	}
	else if(0 != (err = qma6981_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = 256/64;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[QMA6981_AXIS_X], obj->offset[QMA6981_AXIS_Y], obj->offset[QMA6981_AXIS_Z],
			obj->offset[QMA6981_AXIS_X], obj->offset[QMA6981_AXIS_Y], obj->offset[QMA6981_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[QMA6981_AXIS_X], obj->cali_sw[QMA6981_AXIS_Y], obj->cali_sw[QMA6981_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[QMA6981_AXIS_X]*mul + obj->cali_sw[QMA6981_AXIS_X],
			obj->offset[QMA6981_AXIS_Y]*mul + obj->cali_sw[QMA6981_AXIS_Y],
			obj->offset[QMA6981_AXIS_Z]*mul + obj->cali_sw[QMA6981_AXIS_Z],
			tmp[QMA6981_AXIS_X], tmp[QMA6981_AXIS_Y], tmp[QMA6981_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = qma6981_i2c_client;  
	int err, x, y, z;
	int dat[QMA6981_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = qma6981_ResetCalibration(client)))
		{
			GSE_LOG("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X,0x%02X,0x%02X", &x, &y, &z))
	{
		dat[QMA6981_AXIS_X] = x;
		dat[QMA6981_AXIS_Y] = y;
		dat[QMA6981_AXIS_Z] = z;
		if(0 != (err = qma6981_WriteCalibration(client, dat)))
		{
			GSE_LOG("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_LOG("invalid format\n");
	}
	
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[qma6981_BUFSIZE];

	qma6981_read_acc_xyz(sensordata);
	//qma6981_read_acc_xyz((int*)&sensordata); //cxg 0209

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}



/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = qma6981_i2c_client;
	struct qma6981_data *data = i2c_get_clientdata(client);
	int err;
	
	data->hw = hw;
	
	if(0 != (err = hwmsen_get_convert(data->hw->direction, &data->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", data->hw->direction);
	}
	
	return scnprintf(buf, PAGE_SIZE, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = qma6981_i2c_client;
    struct qma6981_data *data = i2c_get_clientdata(client);

	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			GSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			GSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			GSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}
	return count;
}

// add by yangzhiqiang for debug reg use adb
static u8 qma6981_debug_reg_addr=0x00;
static u8 qma6981_debug_read_len=0x01;

static ssize_t store_setreg(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int addr, value;
	u8 data[C_I2C_FIFO_SIZE];
	int res = 0;
	
	GSE_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))
	{
		GSE_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);
		qma6981_debug_reg_addr = (u8)addr;
		qma6981_debug_read_len = 1;
		data[0] = (u8)addr;
		data[1] = (u8)value;
		res = I2C_TxData(data,2);
		if(res)
		{
			GSE_ERR("write reg 0x%02x fail\n", addr);
		}
	}
	else
	{
		GSE_ERR("store_reg get para error\n");
	}

	return count;
}

static ssize_t show_getreg(struct device_driver *ddri, char *buf)
{
	u8 data[C_I2C_FIFO_SIZE];
	int write_offset=0, icount=0;
	int res = 0;

	if(qma6981->client)
	{
		memset(data, 0, sizeof(data));

		data[0] = qma6981_debug_reg_addr;
		res = I2C_RxData(data, qma6981_debug_read_len);
		if(res)
		{
			GSE_ERR("read reg 0x%02x fail", qma6981_debug_reg_addr);
		}
		write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%02x=", qma6981_debug_reg_addr);
		for(icount=0; icount<qma6981_debug_read_len; icount++)
		{
			write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "0x%02x ", data[icount]);
		}
		write_offset += snprintf(buf+write_offset, PAGE_SIZE-write_offset, "\n");

		return write_offset;
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "error!");
	}
}

static ssize_t store_getreg(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int addr, value;
	
	
	//GSE_LOG("store_getreg buf=%s count=%d\n", buf, count);
	if(2 == sscanf(buf, "0x%x %d", &addr, &value))
	{
		GSE_LOG("get para OK addr=0x%x len=%d\n", qma6981_debug_reg_addr, qma6981_debug_read_len);
		qma6981_debug_reg_addr = (u8)addr;
		qma6981_debug_read_len = (u8)value;
		if(qma6981_debug_read_len >= C_I2C_FIFO_SIZE)
		{
			qma6981_debug_read_len = C_I2C_FIFO_SIZE - 1;
			GSE_ERR("len must < %d\n", C_I2C_FIFO_SIZE);
		}
	}
	else
	{
		GSE_ERR("store_reg get para error\n");
	}
	return count;
}
// add by yangzhiqiang for debug


#ifdef QMA6981_STEP_COUNTER
/*----------------------------------------------------------------------------*/
static ssize_t show_sc_power_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	databuf[0] = 0x11;
	ret = I2C_RxData(databuf, 1);
	if(0 != ret)
		return -EFAULT;

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter power status 0x%2x\n", output[0]);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_sc_power_value(struct device_driver *ddri, const char *buf, size_t count)
{

	int value = 0;
	int ret = 0;
	unsigned char data[2] = {0};

	if(1 == sscanf(buf, "%x", &value))
	{
		data[0] = 0x11;
		data[1] = value;
		ret = I2C_TxData(data,2);
		if(ret < 0)
			return -EFAULT;
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_count_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	databuf[0] = 0x12;
	ret = I2C_RxData(databuf, 1);
	if(0 != ret)
		return -EFAULT;

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter sample count 0x%2x\n", output[0]);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_sc_count_value(struct device_driver *ddri, const char *buf, size_t count)
{

	int value = 0,ret;
	unsigned char data[2] = {0};

	if((1 == sscanf(buf, "%x", &value)))
	{
		data[0] = 0x12;
		data[1] = value;
		ret = I2C_TxData(data,2);
		if(ret < 0)
			return ret;
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_recision_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	databuf[0] = 0x13;
	ret = I2C_RxData(databuf, 1);
	if(0 != ret)
		return -EFAULT;


	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter precision 0x%2x\n", output[0]);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_sc_recision_value(struct device_driver *ddri, const char *buf, size_t count)
{

	int value = 0,ret;
	unsigned char data[2] = {0};

	if(1 == sscanf(buf, "%x", &value))
	{
		data[0] = 0x13;
		data[1] = value;
#if defined(QMA6981_STEP_TO_ZERO)
		qma6981_reg13_value = value;
#endif
		ret = I2C_TxData(data,2);
		if(ret < 0)
			return -EFAULT;
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_timelow_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret=0;

	databuf[0] = 0x14;
	ret = I2C_RxData(databuf, 1);
	if(0 != ret)
		return -EFAULT;

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter time low 0x%2x\n", output[0]);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_sc_timelow_value(struct device_driver *ddri, const char *buf, size_t count)
{

	int value = 0,ret;
	unsigned char data[2] = {0};

	if(1 == sscanf(buf, "%x", &value))
	{
		data[0] = 0x14;
		data[1] = value;
		ret = I2C_TxData(data,2);
		if(ret < 0)
			return -EFAULT;
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_timeup_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret =0;

	databuf[0] = 0x15;
	ret = I2C_RxData(databuf, 1);
	if(0 != ret)
		return -EFAULT;

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter time up 0x%2x\n", output[0]);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_sc_timeup_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data[2] = {0};

	if(1 == sscanf(buf, "%x", &value))
	{
		data[0] = 0x15;
		data[1] = value;
		ret = I2C_TxData(data,2);
		if(ret < 0)
			return ret;
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sc_axis_value(struct device_driver *ddri, char *buf)
{
	unsigned char databuf[1];
	int output[1]={ 0 };
	int ret=0;

	databuf[0] = 0x32;
	ret = I2C_RxData(databuf, 1);
	if(0 != ret)
		return -EFAULT;

	output[0] = (int)databuf[0];

	return sprintf(buf, "Stepcounter saxis is 0x%2x\n", output[0]);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_sc_axis_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int value = 0,ret;
	unsigned char data[2] = {0};

	if(1 == sscanf(buf, "%x", &value))
	{
		data[0] = 0x32;
		data[1] = value;
		ret = I2C_TxData(data,2);
		if(ret < 0)
			return ret;
	}
	else
	{
		GSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

#if defined(QMA6981_STEP_COUNTER_USE_INT)
static ssize_t show_dummystep_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "DumyStep is %d\n", STEP_DUMMY_VLUE);
}

static ssize_t store_dummysteps_value(struct device_driver *ddri, const char *buf, size_t count)
{
	if(1 == sscanf(buf, "%d", &STEP_DUMMY_VLUE))
	{
		GSE_LOG("yzq dummy value=%d", STEP_DUMMY_VLUE);
	}
	else
	{
		GSE_ERR("yzq store_dummysteps_value error! \n");
	}

	return count;
}
#endif


#if defined(QMA6981_SELECT_AXIS)
static ssize_t show_axis_select_value(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "qma6981_enable_axis_select is %d\n", qma6981_enable_axis_select);
}

static ssize_t store_axis_select_value(struct device_driver *ddri, const char *buf, size_t count)
{
	if(1 == sscanf(buf, "%d", &qma6981_enable_axis_select))
	{
		GSE_LOG("yzq qma6981_enable_axis_select=%d \n", qma6981_enable_axis_select);
	}
	else
	{
		GSE_ERR("yzq qma6981_enable_axis_select error! \n");
	}

	return count;
}

#endif


static DRIVER_ATTR(spower,      S_IRUGO | S_IWUSR, show_sc_power_value, store_sc_power_value);
static DRIVER_ATTR(sspcount,      S_IRUGO | S_IWUSR, show_sc_count_value, store_sc_count_value);
static DRIVER_ATTR(sprecision,      S_IRUGO | S_IWUSR, show_sc_recision_value, store_sc_recision_value);
static DRIVER_ATTR(stimelow,      S_IRUGO | S_IWUSR, show_sc_timelow_value, store_sc_timelow_value);
static DRIVER_ATTR(stimeup,      S_IRUGO | S_IWUSR, show_sc_timeup_value, store_sc_timeup_value);
static DRIVER_ATTR(saxis,      S_IRUGO | S_IWUSR, show_sc_axis_value, store_sc_axis_value);	
#if defined(QMA6981_STEP_COUNTER_USE_INT)
static DRIVER_ATTR(dummystep,      S_IRUGO | S_IWUSR, show_dummystep_value, store_dummysteps_value);	
#endif
#if defined(QMA6981_SELECT_AXIS)
static DRIVER_ATTR(axis_select,      S_IRUGO | S_IWUSR, show_axis_select_value, store_axis_select_value);	
#endif
#endif

static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(waferid,    S_IRUGO, show_waferid_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(dumpallreg,  S_IRUGO , show_dumpallreg_value, NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(setreg,   	S_IWUSR | S_IRUGO, NULL,        store_setreg);
static DRIVER_ATTR(getreg,   	S_IWUSR | S_IRUGO, show_getreg,        store_getreg);


static struct driver_attribute *qma6981_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_waferid,
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_dumpallreg,
	&driver_attr_layout,
	&driver_attr_trace,
#ifdef QMA6981_STEP_COUNTER
	&driver_attr_spower,
	&driver_attr_sspcount,
	&driver_attr_sprecision,
	&driver_attr_stimelow,
	&driver_attr_stimeup,
	&driver_attr_saxis,
#if defined(QMA6981_STEP_COUNTER_USE_INT)
	&driver_attr_dummystep,	
#endif
#if defined(QMA6981_SELECT_AXIS)
	&driver_attr_axis_select,
#endif
#endif
	&driver_attr_setreg,
	&driver_attr_getreg
};

static int qma6981_initialize(struct i2c_client *client)
{
	int ret = 0;
	int direction = 0;
	unsigned char data[2] = {0};

	direction = qma6981->hw->direction;
	GSE_LOG("%s: direction %d\n",__FUNCTION__,direction);

	//0x0c lower output data rate ,Bandwidth = 62.5Hz ODR = 2*BW = 125Hz @2g
	//0x0d lower output data rate ,Bandwidth = 125Hz ODR = 2*BW = 250Hz @8g
	//0x0b lower output data rate ,Bandwidth = 31.2Hz ODR = 2*BW = 62.5Hz @8g
	//0x2a high output data rate ,Bandwidth = 15.6Hz ODR = 4*BW =  62.5Hz @8g
	//0x2b high output data rate ,Bandwidth = 31.25Hz ODR = 4*BW = 125Hz @8g	

	data[0] = 0x10;
	data[1] = 0x2a;	//0x2a;	0x0b
	ret = I2C_TxData(data,2);
   	if(ret < 0)
	  goto exit_i2c_err;
  	GSE_LOG("%s: 11111111111 ret %d\n",__FUNCTION__,ret);

	//0x01 range  2g , 3.9mg/LSB  @2g
	//0x04 range  8g , 15.6mg/LSB @8g
	data[0] = 0x0F;
	data[1] = 0x04;
// add by yangzhiqiang
	if(data[1] == 0x04)
		qm6981_248g_offset = 6;
	else if(data[1] == 0x02)
		qm6981_248g_offset = 7;
	else if(data[1] == 0x01)
		qm6981_248g_offset = 8;
	else
		qm6981_248g_offset = 8;
// yangzhiqiang
	ret = I2C_TxData(data,2);
   	if(ret < 0)
   		goto exit_i2c_err;
  	GSE_LOG("%s: 22222222222 ret %d\n",__FUNCTION__,ret);

#ifdef QMA6981_STEP_COUNTER
	//0x12 = 0x8c,start step_count, sample counts 48  @2g@50hz
	//0x12 = 0x9f,start step_count, sample counts 124 @8g@125hz
	//0x12 = 0x8e,start step_count, sample counts 56 @8g@55hz
	//0x12 = 0x8f,start step_count, sample counts 60 @8g@62.5hz
	//0x12 = 0x88,start step_count, sample counts 32 @8g@31.2hz
	
	data[0] = 0x12;
	data[1] = 0x8f;
	ret = I2C_TxData(data,2);
   	if(ret < 0)
   		goto exit_i2c_err;
	
	//0x13 = 0x26, dynamic precision 150mg @2g
	//0x13 = 0x33, dynamic precision 200mg @2g
	//0x13 = 0x40, dynamic precision 250mg @2g

	//0x13 = 0x0a, dynamic precision 156mg @8g
	//0x13 = 0x0c, dynamic precision 187mg @8g
	//0x13 = 0x0d, dynamic precision 203mg @8g
	//0x13 = 0x0e, dynamic precision 218mg @8g  	
	//0x13 = 0x10, dynamic precision 250mg @8g 
	//0x13 = 0x12, dynamic precision 282mg @8g  
	//0x13 = 0x13, dynamic precision 296mg @8g 
	//0x13 = 0x14, dynamic precision 312mg @8g 	
	//0x13 = 0x17, dynamic precision 350mg @8g  
	data[0] = 0x13;
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
	data[1] = QMA6981_REG13_LOW_VALUE; 
#else
	data[1] = QMA6981_REG13_HIGH_VALUE;		// yzq 250mg, old 0x10 
#endif
#if 1//defined(QMA6981_STEP_TO_ZERO)
	qma6981_reg13_value = data[1];
#endif
	ret = I2C_TxData(data,2);
   	if(ret < 0)
   		goto exit_i2c_err;
	
	//0x14 = 0x12,time window low 0.324s @2g@18ms
	//0x14 = 0x28,time window low 0.32s @8g@8ms
	//0x14 = 0x20,time window low 0.25s @8g@8ms
	//0x14 = 0x2c,time window low 0.35s @8g@8ms
	
	//0x14 = 0x12,time window low 0.324s @8g@18ms
	//0x14 = 0x13,time window low 0.342s @8g@18ms
	//0x14 = 0x14,time window low 0.36s @8g@18ms
	
	//0x14 = 0x12,time window low 0.28s @8g@16ms
	//0x14 = 0x14,time window low 0.32s @8g@16ms	
	
	//0x14 = 0x0a,time window low 0.32s @8g@32ms
	//0x14 = 0x0b,time window low 0.35s @8g@32ms
	data[0] = 0x14;
	data[1] = 0x14;		// odr 16ms, 320ms
	ret = I2C_TxData(data,2);
   	if(ret < 0)
   		goto exit_i2c_err;
	
	//0x15 = 0x0f,time window up 2.16s @2g@18ms
	//0x15 = 0x21,time window up 2.11s @8g@8ms
	//0x15 = 0x0f,time window up 2.16s @8g@18ms
	//0x15 = 0x10,time window up 2.0s @8g@16ms
	//0x15 = 0x08,time window up 2.0s @8g@32ms
	data[0] = 0x15;
	data[1] = 0x10;  // old 0x21@8ms
	ret = I2C_TxData(data,2);
   	if(ret < 0)
   		goto exit_i2c_err;

	
	//0x32,0x00 xy;0x01 yz; 0x02 xz;
	data[0] = 0x32;
	if(direction%2)
		data[1] = 0x02;
	else
		data[1] = 0x01;

	ret = I2C_TxData(data,2);
	if(ret < 0)
		goto exit_i2c_err;
	GSE_LOG("%s: 0x32[0x%x]\n",__FUNCTION__,data[1]);

	//CUSTOFFSET 0x27 0x28 0x29 = 0x64, 400mg @2g
	//CUSTOFFSET 0x27 0x28 0x29 = 0x60, 1500mg @8g

	data[0] = 0x27;
	data[1] = 0x60;
	ret = I2C_TxData(data,2);

	data[0] = 0x28;
	data[1] = 0x60;
	ret = I2C_TxData(data,2);

	data[0] = 0x29;
	data[1] = 0x60;
	ret = I2C_TxData(data,2);

	// int1_step  int1_step_quit
	data[0] = 0x19;  //eint1: 0x19 eint2:0x1b
	data[1] = 0x08;  // step_en ,step_quit_en
	ret = I2C_TxData(data,2);

	data[0] = 0x16;
	data[1] = 0x0c;  // step_en ,step_quit_en
	ret = I2C_TxData(data,2);

	
	//0x11 = 0x8a active mode, sleep time 10ms @2g
	
	//0x11 = 0x8a active mode, sleep time 10ms @8g
	//0x11 = 0x88 active mode, sleep time 4ms  @8g
	//0x11 = 0x80 active mode, sleep time 0ms  @8g
#else
	data[0] = 0x27;
	data[1] = 0x00;
	ret = I2C_TxData(data,2);

	data[0] = 0x28;
	data[1] = 0x00;
	ret = I2C_TxData(data,2);

	data[0] = 0x29;
	data[1] = 0x00;
	ret = I2C_TxData(data,2);

	// int1_step  int1_step_quit
	data[0] = 0x19;  //eint1: 0x19 eint2:0x1b
	data[1] = 0x00;  // step_en ,step_quit_en
	ret = I2C_TxData(data,2);

	data[0] = 0x16;
	data[1] = 0x00;  // step_en ,step_quit_en
	ret = I2C_TxData(data,2);
#endif
	data[0] = 0x11;
	data[1] = 0x80;		// yzq old 0x88
	ret = I2C_TxData(data,2);
  	GSE_LOG("%s: 33333333333 ret %d\n",__FUNCTION__,ret);

   	return 0;
exit_i2c_err:
	GSE_ERR("qma6981_initialize fail: %d\n",ret);
	return ret;
}


/*  ioctl command for qma6981 device file */
static long qma6981_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int err = 0;
	void __user *val;
	char strbuf[qma6981_BUFSIZE];
	int vec[3] = {0};
	struct SENSOR_DATA sensor_data= {0};
	int cali[3];

	struct i2c_client *client = (struct i2c_client*)file->private_data;

	struct qma6981_data *obj = (struct qma6981_data*)i2c_get_clientdata(client);	

	GSE_LOG("qma6981_unlocked_ioctl - cmd=%u, arg = %lu\n" , cmd, arg);
	//GSE_LOG("SET CAL = %d , CLR CAL = %d",GSENSOR_IOCTL_SET_CALI , GSENSOR_IOCTL_CLR_CALI);
	/* check qma6981_client */
	if (&qma6981->client == NULL) {
		GSE_LOG( "I2C driver not install\n");
		return -EFAULT;
	}

	switch (cmd) {
	
	case GSENSOR_IOCTL_INIT:
		qma6981_initialize(client);
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:

		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		
		//GSE_LOG("qma6981_unlocked_ioctl - cmd=GSENSOR_IOCTL_READ_SENSORDATA\n");
		qma6981_read_acc_xyz(vec);
		sprintf(strbuf, "%04x %04x %04x", vec[0], vec[1], vec[2]);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}	
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		qma6981_read_raw_xyz(vec);
		
		sprintf(strbuf, "%04x %04x %04x", vec[0], vec[1], vec[2]);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}
		break;	  

	case GSENSOR_IOCTL_SET_CALI:
		GSE_LOG("qma6981 ioctl GSENSOR_IOCTL_SET_CALI\n");
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		if(copy_from_user(&sensor_data, val, sizeof(sensor_data)))
		{
			err = -EFAULT;
			break;	  
		}
		if(atomic_read(&obj->suspend))
		{
			GSE_LOG("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		}
		else
		{
			cali[QMA6981_AXIS_X] = (sensor_data.x<<(qm6981_248g_offset))/GRAVITY_EARTH_1000;
			cali[QMA6981_AXIS_Y] = (sensor_data.y<<(qm6981_248g_offset))/GRAVITY_EARTH_1000;
			cali[QMA6981_AXIS_Z] = (sensor_data.z<<(qm6981_248g_offset))/GRAVITY_EARTH_1000;
			//GSE_LOG("QST qma6981 x:%d y:%d z:%d\n", sensor_data.x, sensor_data.y, sensor_data.z);
			//GSE_LOG("QST qma6981 cali[0]:%d cali[1]:%d cali[2]:%d\n", cali[0], cali[1], cali[2]);
			err = qma6981_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		//GSE_LOG("qma6981 ioctl GSENSOR_IOCTL_CLR_CALI\n");
		err = qma6981_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		//GSE_LOG("qma6981 ioctl GSENSOR_IOCTL_GET_CALI\n");
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		if(0 != (err = qma6981_ReadCalibration(client, cali)))
		{
			break;
		}
		//GSE_LOG("qma6981 get cali.x,y,z[%d,%d,%d]\n",cali[QMA6981_AXIS_X],cali[QMA6981_AXIS_Y],cali[QMA6981_AXIS_Z]);
		sensor_data.x = (cali[QMA6981_AXIS_X] * GRAVITY_EARTH_1000)>>(qm6981_248g_offset);
		sensor_data.y = (cali[QMA6981_AXIS_Y] * GRAVITY_EARTH_1000)>>(qm6981_248g_offset);
		sensor_data.z = (cali[QMA6981_AXIS_Z] * GRAVITY_EARTH_1000)>>(qm6981_248g_offset);
		//GSE_LOG("QST qma6981 get sensor_data.x,y,z[%d,%d,%d]\n",sensor_data.x,sensor_data.y,sensor_data.z);
		if(copy_to_user(val, &sensor_data, sizeof(sensor_data)))
		{
			err = -EFAULT;
			break;
		}		
		break;
	default:
		break;
	}
	return err;
}

#ifdef CONFIG_COMPAT
static long qma6981_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
        break;
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
        break;		
        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif


/*----------------------------------------------------------------------------*/
static int qma6981_open(struct inode *inode, struct file *file)
{
	file->private_data = qma6981_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_LOG("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int qma6981_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static const struct file_operations qma6981_fops = {
	.owner = THIS_MODULE,
	.open = qma6981_open,
	.release = qma6981_release,
	.unlocked_ioctl = qma6981_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = qma6981_compat_ioctl,
#endif
};

static struct miscdevice qma6981_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &qma6981_fops,
};

static int qma6981_SetPowerCTRL(struct i2c_client *client, bool enable)
{
    int res = 0;
    //struct qma6981_data *obj = i2c_get_clientdata(client);
	
    u8 databuf[2];

    if (enable == sensor_power)
    {
		GSE_LOG("Sensor power status is newest!\n");
        return 0;
    }
#ifdef QMA6981_STEP_COUNTER		// if stepcounter enable do not into standby mode
    sensor_power = enable;
	return 0;
#endif
    if (enable == false){
		databuf[1]=0x00;
    }else{
		databuf[1]=0x80;
		}
		
		
		//databuf[1]=0x80;
    databuf[0] = QMA6981_REG_POWER_CTL;
    res = I2C_TxData(databuf,2);
    sensor_power = enable;
    mdelay(5);
    return res;    
}

static int qma6981_open_report_data(int open)
{
	return 0;
}

static int qma6981_enable_nodata(int en)
{
    int err = 0;
	GSE_LOG("qma6981_enable_nodata en:%d!\n", en);
	if(((en == 0) && (sensor_power == false)) ||((en == 1) && (sensor_power == true)))
	{
		enable_status = sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
	}
	else
	{
		enable_status = !sensor_power;
		if (atomic_read(&qma6981->suspend) == 0)
		{

			err = qma6981_SetPowerCTRL(qma6981->client, enable_status);
			GSE_LOG("Gsensor not in suspend gsensor_SetPowerMode!, enable_status = %d\n",enable_status);
		}
		else
		{
			GSE_LOG("Gsensor in suspend and can not enable or disable!enable_status = %d\n",enable_status);
		}
	}

    if(err != 0)
	{
		GSE_LOG("gsensor_enable_nodata fail!\n");
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifndef QMA6981_STEP_COUNTER
static int qma6981_SetBWRate(struct i2c_client *client, u8 bwrate)
{
    struct qma6981_data *obj = i2c_get_clientdata(client);
    u8 databuf[2] = {0};    
    int res = 0;

    if( (obj->bandwidth != bwrate) || !(atomic_read(&obj->suspend)) )
    {    
        memset(databuf, 0, sizeof(databuf));    
   
        /* write */
        databuf[1] = databuf[0] | bwrate;
        databuf[0] = QMA6981_REG_BW_ODR;    
    
        res = I2C_TxData(databuf,2);
        if (res < 0)
            return QMA6981_ERR_I2C;

        obj->bandwidth = bwrate;
    }

    return 0;    
}
#endif

static int qma6981_set_delay(u64 ns)
{
    int err = 0;
#ifndef QMA6981_STEP_COUNTER
    int value;
	int sample_delay;
#endif

#ifndef QMA6981_STEP_COUNTER
    value = (int)ns/1000/1000;
	if(value <= 5){
		sample_delay = QMA6981_ODR_250HZ;
	}
	else if(value <= 10){
		sample_delay = QMA6981_ODR_125HZ;
	}
	else if(value <= 20){
		sample_delay = QMA6981_ODR_62HZ;
	}
	else if(value <= 50){
		sample_delay = QMA6981_ODR_31HZ;
	}
	else if(value <= 100){
		sample_delay = QMA6981_ODR_16HZ;
	}
	else {
		sample_delay = QMA6981_ODR_16HZ;
	}

	err = qma6981_SetBWRate(qma6981->client, sample_delay);

	if(err != 0 )
	{
		GSE_LOG("Set delay parameter error!\n");
        return err;
	}
#endif

	return err;
}

static int qma6981_get_data(int* x ,int* y,int* z, int* status)
{
	int vec[3]={0};
	int ret=0;

	ret = qma6981_read_acc_xyz(vec);

	*x = vec[0];
	*y = vec[1];
	*z = vec[2];
	
	*status = SENSOR_STATUS_ACCURACY_HIGH;
	return ret;
}


#ifdef QMA6981_STEP_COUNTER
static void qma6981_set_wakelock(int en)
{
#if defined(QMA6981_SELECT_AXIS_WHEN_TIMER)
	return;		// yangzhiqiang remove this wakelock
#endif
	if((en == 1)&&(wake_lock_status == KAL_FALSE))
	{
		wake_lock(&sc_wakelock);
		wake_lock_status = KAL_TRUE;
		GSE_LOG("yzqlock enable wakelock\n");
	}
	else if((en == 0)&&(wake_lock_status == KAL_TRUE))
	{
		wake_unlock(&sc_wakelock);		
		wake_lock_status = KAL_FALSE;
		GSE_LOG("yzqlock disable wakelock\n");
	}
}


#if defined(QMA6981_STEP_TO_ZERO)
static void qma6981_sc_reset_step(int en)
{
	int err;
	char data[2] ={0};

	GSE_ERR("reset step to :%d\n", en);	
#if defined(QMA6981_STEP_COUNTER_USE_INT)
	//disable_irq_nosync(g_QMA6981_ptr->irq);
	int_statu_flag = KAL_FALSE;
	memset(&step_count_index, 0, sizeof(step_count_index));
	
	/*if(en)
	{
		enable_irq(g_QMA6981_ptr->irq);
	}
	else
	{
		disable_irq_nosync(g_QMA6981_ptr->irq);
	}*/

#endif
	if(en)	// when open reset stepcounter
	{
		data[0] = 0x13;
		data[1] = 0x80; //clean data

		err = I2C_TxData(data,2);
		if(err < 0)
		{			
			err = I2C_TxData(data,2);
			if(err < 0)
			{		
				err = I2C_TxData(data,2);			
				if(err < 0)
				{
					GSE_ERR("write 0x13 0x80 error! \n");
					return;
				}
			}
		}
#if defined(QMA6981_CHECK_ABNORMAL_DATA)
		last_read_data = 0;
		last_read_time = get_monotonic_coarse();
#endif
		sc_reset_flag = KAL_TRUE;
		//udelay(50);
		
		data[0] = 0x13;
		data[1] = qma6981_reg13_value;
		//data[1] = 0x10;
		err = I2C_TxData(data,2);
		if(err < 0)
		{
			err = I2C_TxData(data,2);
			if(err < 0)
			{		
				err = I2C_TxData(data,2);
				if(err < 0)
				{
					GSE_ERR("write 0x13 0x10 error! \n");
				}
			}
		}
	}
}
#endif


static int qma6981_step_c_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

static int qma6981_step_c_enable_nodata(int en)
{
// add by yangzhiqiang
#if defined(QMA6981_SELECT_AXIS_WHEN_TIMER)
	if(en)
	{		
		wake_lock(&sc_wakelock);
	}
	else
	{
		wake_unlock(&sc_wakelock);
	}
#endif
// yangzhiqiang
#if defined(QMA6981_STEP_TO_ZERO)
	qma6981_set_wakelock(1);
	qma6981_sc_reset_step(en);
	qma6981_set_wakelock(0);
#endif

	if(en)
	{
	}
	else
	{
#if defined(QMA6981_SELECT_AXIS)	// when disable stepcounter 0x32 set 0x00 use xy axis
		qma6981_set_wakelock(1);
		qm6981_step_c_use_axis_z(QMA6981_USE_AXIS_Z_DEFAULT);
		qma6981_set_wakelock(0);
#endif
	}
	return 0;
}

static int qma6981_step_c_enable_step_detect(int en)
{
	return 0;
}

static int qma6981_step_c_enable_significant(int en)
{
	return 0;
}

static int qma6981_step_c_set_delay(u64 ns)
{
    return 0;
}

static int qma6981_step_d_set_delay(u64 ns)
{
    return 0;
}

static int qma6981_step_c_get_data_significant(unsigned int *value,int *status)
{
	return 0;
}
static int qma6981_step_c_get_data_step_d(unsigned int *value,int *status)
{
	return 0;
}

static int qma6981_step_c_read_stepCounter(int *data)
{
	int res;
	char databuf[2];
	int resut;
#ifdef QMA6981_STEP_COUNTER_USE_INT
	int tempp = 0;
#endif
#if defined(QMA6981_CHECK_ABNORMAL_DATA)
	struct timespec read_time;
	int	diff_step;
#endif

	//wake_lock(&sc_wakelock);	// add by yangzhiqiang
	databuf[0] = QMA6981_STEP_CNT_L;
	if((res = I2C_RxData(databuf, 2))){
		GSE_ERR("QMA6981_SC_stepcounter error!!!");
		return -EFAULT;
	}
	resut = (databuf[1]<<8)|databuf[0];
	//GSE_LOG("yzq	reg data=	%d\n", resut);
	//wake_unlock(&sc_wakelock);

#if defined(QMA6981_CHECK_ABNORMAL_DATA)
	read_time = get_monotonic_coarse();

	if(last_read_data >= 0)
	{
		//GSE_LOG("yzqcheck time(%ld, %ld)  lasttime(%ld, %ld)\n", read_time.tv_sec,read_time.tv_nsec, last_read_time.tv_sec,last_read_time.tv_nsec);
		if((read_time.tv_sec - last_read_time.tv_sec) <= CHECK_IN_TIME)
		{
			diff_step = (resut>=last_read_data) ? (resut-last_read_data) : (65535-last_read_data+resut);
			if(diff_step > CHECK_IN_DIFF_STEP)
			{
				int read_data_1, read_data_2, read_data_3;
				// if read error, read again, test
				read_data_1 = resut;		// store data 1
				
				databuf[0] = QMA6981_STEP_CNT_L;
				if((res = I2C_RxData(databuf, 2))){
					GSE_ERR("QMA6981_SC_stepcounter error!!!");
					return -EFAULT;
				}
				read_data_2 = (databuf[1]<<8)|databuf[0];
				GSE_LOG("yzqread_data_2	reg data=	%d\n", read_data_2);

				databuf[0] = QMA6981_STEP_CNT_L;
				if((res = I2C_RxData(databuf, 2))){
					GSE_ERR("QMA6981_SC_stepcounter error!!!");
					return -EFAULT;
				}
				read_data_3 = (databuf[1]<<8)|databuf[0];
				GSE_LOG("yzqread_data_2	reg data=	%d\n", read_data_3);
				// if read error, read again
				if((QMA6981_ABS(read_data_1-read_data_2)>CHECK_IN_DIFF_STEP)
				||(QMA6981_ABS(read_data_2-read_data_3)>CHECK_IN_DIFF_STEP)
				||(QMA6981_ABS(read_data_1-read_data_3)>CHECK_IN_DIFF_STEP))
				{
					GSE_LOG("yzqcheck data abnormal reg data=%d %d %d	lastdata=%d\n", read_data_1,read_data_2,read_data_3, last_read_data);
					resut = last_read_data; //data abnormal use last value.			
					//last_read_time.tv_sec = read_time.tv_sec;
					//last_read_time.tv_nsec = read_time.tv_nsec;
				}
				else if(QMA6981_ABS(resut-last_read_data) <= CHECK_IN_DIFF_STEP)
				{				
					resut = last_read_data;				
					last_read_time.tv_sec = read_time.tv_sec;
					last_read_time.tv_nsec = read_time.tv_nsec;
				}
				else
				{
					last_read_data = resut;
					last_read_time.tv_sec = read_time.tv_sec;
					last_read_time.tv_nsec = read_time.tv_nsec;
				}
			}
			else
			{					
				//GSE_LOG("yzqcheck time in, not abnormal \n");
				last_read_data = resut;
				last_read_time.tv_sec = read_time.tv_sec;
				last_read_time.tv_nsec = read_time.tv_nsec;
			}
		}
		else
		{				
			//GSE_LOG("yzqcheck time not in\n");
			last_read_data = resut;
			last_read_time.tv_sec = read_time.tv_sec;
			last_read_time.tv_nsec = read_time.tv_nsec;
		}
	}
	else
	{
		last_read_data = resut;
		last_read_time.tv_sec = read_time.tv_sec;
		last_read_time.tv_nsec = read_time.tv_nsec;
	}
#endif
	//GSE_LOG("step_read_count= %d\n",resut);
	
#ifdef QMA6981_STEP_COUNTER_USE_INT
	//mutex_lock(&qma6981_mutex);		// removed by yangzhiqiang
	if (resut < step_count_index.stepcounter_pre)
	{
		step_count_index.back++;
		*data = resut- step_count_index.stepcounter_pre + 65536;
		 step_count_index.stepcounter_pre = resut;
	}
	else
	{
		//nothing
		step_count_index.stepcounter_pre = resut;
		*data = resut;
	}

	if (step_count_index.stepcounter_statu == STEP_START)
	{
	/*
		if (*data >= step_count_index.stepcounter_pre_end)
		{
			tempp = *data - step_count_index.stepcounter_pre_end;
		}
		else
		{
			tempp = *data - step_count_index.stepcounter_pre_end +65536;
		}
*/
		
		if (*data >= step_count_index.stepcounter_next_start)
		{
			tempp = *data - step_count_index.stepcounter_next_start + 4;
		}
		else
		{
			tempp = *data - step_count_index.stepcounter_next_start +65540;
		}

		GSE_LOG("ReadStepCounter_running *data= %d,stepcounter_next_start = %d,tempp = %d,stepcounter_pre_end =%d \n",*data,step_count_index.stepcounter_next_start,tempp,step_count_index.stepcounter_pre_end);
		
		if (tempp < (STEP_INT_START_VLUE+STEP_DUMMY_VLUE))
		{
			*data = step_count_index.stepcounter_pre_fix;
			GSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
		}
		else
		{
			if (step_count_index.step_diff >*data)
			{
				step_count_index.step_diff = 0;
			}
			else
			{
				*data = *data -  step_count_index.step_diff;
				 step_count_index.stepcounter_pre_fix = *data ;
				 GSE_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
			}
		}
	
	}
	else 
	{
		// add by yangzhiqiang for step_diff
		if(step_count_index.step_diff > *data)
		{
			step_count_index.step_diff = *data;
		}
		// yangzhiqiang for step_diff		
#if 1//defined(QMA6981_STEP_TO_ZERO)
		step_count_index.stepcounter_pre_end = *data;
#endif
		*data  = *data -  step_count_index.step_diff;
		 step_count_index.stepcounter_pre_fix = *data;
		 GSE_LOG("ReadStepCounter_end stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
	}
	//mutex_unlock(&qma6981_mutex);	// removed by yangzhiqiang
	GSE_LOG("ReadStepCounter=%d, step_diff= %d\n",*data,step_count_index.step_diff );
#else
	*data = resut;
#endif	
	//GSE_LOG("step_diff = %d\n",step_count_index.step_diff);

	return 0;
}


static void qma6981_set_precision(char value)
{
	int err;
	char data[2] ={0};

	if(qma6981_reg13_value == value)
	{
		GSE_LOG("qma6981_set_precision return \n");
	}
	else
	{
		qma6981_reg13_value = value;
		GSE_LOG("qma6981_set_precision write 0x13 = 0x%x \n", qma6981_reg13_value);
	}
	data[0] = 0x13;
	data[1] = value;	// 0x0a
	//data[1] = 0x10;
	err = I2C_TxData(data,2);
	if(err < 0)
	{
		err = I2C_TxData(data,2);
		if(err < 0)
		{		
			err = I2C_TxData(data,2);
			if(err < 0)
			{
				GSE_ERR("qma6981_set_precision write 0x13 0x10 error! \n");
			}
		}
	}

}


#if defined(QMA6981_SELECT_AXIS)	//add by yangzhiqiang for select axis
static void qm6981_step_c_use_axis_z(kal_bool is_axis_z)
{
	char data[2];
	int ret;

	if(qma6981_enable_axis_select == 0)
	{
		GSE_LOG("yzqselect select axis disable\n");
		return;
	}

// yangzhiqiang
	if((is_axis_z == KAL_TRUE)&&(qma6981_use_axis_z == KAL_FALSE))
	{
		// send config to ic				
		//0x32,0x00 xy;0x01 yz; 0x02 xz;
		data[0] = 0x32;
		if(qma6981->hw->direction%2)
			data[1] = 0x02;
		else
			data[1] = 0x01;
		
		ret = I2C_TxData(data,2);
		if(ret < 0)
		{
			GSE_LOG("%s: err 0x32[0x%x]\n",__FUNCTION__,data[1]);
			ret = I2C_TxData(data,2);
			if(ret < 0)
			{					
				GSE_LOG("%s: err2 0x32[0x%x]\n",__FUNCTION__,data[1]);
			}
		}
		// send config to ic
		qma6981_use_axis_z = KAL_TRUE;
		GSE_LOG("yzqselect use z axis \n");
	}
	else if((is_axis_z == KAL_FALSE)&&(qma6981_use_axis_z == KAL_TRUE))
	{
		// send config to ic
		//0x32,0x00 xy;0x01 yz; 0x02 xz;
		data[0] = 0x32;
		data[1] = 0x00;

		ret = I2C_TxData(data,2);
		if(ret < 0)
		{
			GSE_LOG("%s: err 0x32[0x%x]\n",__FUNCTION__,data[1]);
			ret = I2C_TxData(data,2);
			if(ret < 0)
			{					
				GSE_LOG("%s: err2 0x32[0x%x]\n",__FUNCTION__,data[1]);
			}
		}
		// send config to ic

		qma6981_use_axis_z = KAL_FALSE;		
		GSE_LOG("yzqselect not use z axis \n");
	}
	else
	{
		//GSE_LOG("yzqselect choose axis ingor\n");
	}
	return;
}
#endif

static int qma6981_step_c_get_sc(unsigned int *value, int *status)
{
	int err;
	int pedo_data = 0;
#if defined(QMA6981_SELECT_AXIS_WHEN_TIMER)||defined(QMA6981_SELECT_AXIS_WHEN_INT)	//defined(QMA6981_SELECT_AXIS)
	int acc_data[3];
	int res;
#endif

#if defined(QMA6981_SELECT_AXIS_WHEN_TIMER)||defined(QMA6981_SELECT_AXIS_WHEN_INT)	//defined(QMA6981_SELECT_AXIS)	//add by yangzhiqiang for select axis
	if(axis_select_timer_running == KAL_FALSE)
	{
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
		if(qma6981_acc_z_index)
		{
			qma6981_set_wakelock(0);
			qma6981_acc_z_index = 0;
		}
#endif
	}
	else if(qma6981_acc_z_index < QMA6981_ACC_SIZE)
	{
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
		if(qma6981_acc_z_index == 0)
		{
			qma6981_set_wakelock(1);
		}
#endif
		if((res = qma6981_read_acc_xyz(acc_data)))
		{
			if((res = qma6981_read_acc_xyz(acc_data)))
			{
				GSE_LOG("yzqselect qma6981_read_acc_xyz error:%d \n", res);
				acc_data[0] = 0;
				acc_data[1] = 0;
				acc_data[2] = 0;
			}
		}

		if(qma6981_acc_z_index == 0)
		{		
			qma6981_acc_data[QMA6981_AXIS_X] = QMA6981_ABS(acc_data[QMA6981_AXIS_X]);
			qma6981_acc_data[QMA6981_AXIS_Y] = QMA6981_ABS(acc_data[QMA6981_AXIS_Y]);
			qma6981_acc_data[QMA6981_AXIS_Z] = QMA6981_ABS(acc_data[QMA6981_AXIS_Z]);

			qma6981_acc_x_min = qma6981_acc_x_max = acc_data[QMA6981_AXIS_X];
			qma6981_acc_y_min = qma6981_acc_y_max = acc_data[QMA6981_AXIS_Y];
			qma6981_acc_z_min = qma6981_acc_z_max = acc_data[QMA6981_AXIS_Z];
		}
		else
		{		
			qma6981_acc_data[QMA6981_AXIS_X] += acc_data[QMA6981_AXIS_X];
			qma6981_acc_data[QMA6981_AXIS_Y] += acc_data[QMA6981_AXIS_Y];
			qma6981_acc_data[QMA6981_AXIS_Z] += acc_data[QMA6981_AXIS_Z];

			qma6981_acc_x_min = ((qma6981_acc_x_min > acc_data[QMA6981_AXIS_X]) ? (acc_data[QMA6981_AXIS_X]) : (qma6981_acc_x_min));
			qma6981_acc_x_max = ((qma6981_acc_x_max < acc_data[QMA6981_AXIS_X]) ? (acc_data[QMA6981_AXIS_X]) : (qma6981_acc_x_max));
	
			qma6981_acc_y_min = ((qma6981_acc_y_min > acc_data[QMA6981_AXIS_Y]) ? (acc_data[QMA6981_AXIS_Y]) : (qma6981_acc_y_min));
			qma6981_acc_y_max = ((qma6981_acc_y_max < acc_data[QMA6981_AXIS_Y]) ? (acc_data[QMA6981_AXIS_Y]) : (qma6981_acc_y_max));
	
			qma6981_acc_z_min = ((qma6981_acc_z_min > acc_data[QMA6981_AXIS_Z]) ? (acc_data[QMA6981_AXIS_Z]) : (qma6981_acc_z_min));
			qma6981_acc_z_max = ((qma6981_acc_z_max < acc_data[QMA6981_AXIS_Z]) ? (acc_data[QMA6981_AXIS_Z]) : (qma6981_acc_z_max));
		}

		//GSE_LOG("yzqselect acc index:%d   x:%d y:%d z : %d \n", qma6981_acc_z_index, acc_data[0], acc_data[1], acc_data[2]);
		qma6981_acc_z_index++;
	}
	else
	{
		int acc_x_average, acc_y_average, acc_z_average;
		int acc_x_diff, acc_y_diff, acc_z_diff;

		acc_x_average = qma6981_acc_data[QMA6981_AXIS_X]/QMA6981_ACC_SIZE;
		acc_y_average = qma6981_acc_data[QMA6981_AXIS_Y]/QMA6981_ACC_SIZE;
		acc_z_average = qma6981_acc_data[QMA6981_AXIS_Z]/QMA6981_ACC_SIZE;
		GSE_LOG("yzqselect average:	X:%d 	Y:%d	Z:%d\n", acc_x_average, acc_y_average, acc_z_average);
		acc_x_diff = qma6981_acc_x_max-qma6981_acc_x_min;
		acc_y_diff = qma6981_acc_y_max-qma6981_acc_y_min;
		acc_z_diff = qma6981_acc_z_max-qma6981_acc_z_min;
		GSE_LOG("yzqselect average:	X(diff):%d	Y(diff):%d	Z(diff):%d\n", acc_x_diff, acc_y_diff, acc_z_diff);

		//if((QMA6981_ABS(acc_z_diff) < QMA6981_ABS(acc_x_diff))&&(QMA6981_ABS(acc_y_average)<7000)&&(QMA6981_ABS(acc_x_average) > 7000))
		if((QMA6981_ABS(acc_z_diff) < QMA6981_ABS(acc_x_diff))/*&&(QMA6981_ABS(acc_y_diff)<7000)*/&&(QMA6981_ABS(acc_x_average) > 7000))
		{
			if(QMA6981_ABS(acc_y_average)<2000)
			{
				qm6981_step_c_use_axis_z(KAL_FALSE);
				qma6981_set_precision(0x0f);
			}
			else
			{
				qm6981_step_c_use_axis_z(KAL_TRUE);
				qma6981_set_precision(0x09);
			}
		}
		else
		{
			qm6981_step_c_use_axis_z(KAL_TRUE);			
			qma6981_set_precision(QMA6981_REG13_HIGH_VALUE);
		}

		qma6981_acc_z_index = 0;
		qma6981_acc_data[QMA6981_AXIS_X] = 0;
		qma6981_acc_data[QMA6981_AXIS_Y] = 0;
		qma6981_acc_data[QMA6981_AXIS_Z] = 0;
#if defined(QMA6981_SELECT_AXIS_WHEN_INT)
		qma6981_set_wakelock(0);
		step_c_set_delay(200);
		axis_select_timer_running = KAL_FALSE;
#endif
	}
#endif	//add by yangzhiqiang for select axis
	err = qma6981_step_c_read_stepCounter(&pedo_data);
	if((pedo_data > 0)&&(sc_reset_flag))
	{
		*value = 1;
		sc_reset_flag = KAL_FALSE;
	}
	else
	{
		*value = (unsigned int)pedo_data;
	}
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	
	return err;
}
#endif

static int qma6981_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(qma6981_attr_list)/sizeof(qma6981_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qma6981_attr_list[idx]);
	}
	return err;
}

static int qma6981_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(qma6981_attr_list)/sizeof(qma6981_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, qma6981_attr_list[idx])))
		{
			GSE_LOG("attributes (%s) = %d\n", qma6981_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int qma6981_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	int err = 0;	
	static struct i2c_client *new_client;
	struct qma6981_data *obj;
	struct acc_control_path acc_ctl = {0};
	struct acc_data_path acc_data = {0};
#ifdef QMA6981_STEP_COUNTER
	char data[2] ={0};
#endif
			
	GSE_FUN();
	obj = kzalloc(sizeof(struct qma6981_data), GFP_KERNEL);
	if (obj == NULL)
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct qma6981_data));
	client->addr = 0x12;
	obj->hw = hw;
	atomic_set(&obj->layout,obj->hw->direction);
	if(0 != (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}
	GSE_LOG("%s: --------  111111111111 ----------\n",__FUNCTION__);

	qma6981 = obj;
	
#ifdef QMA6981_STEP_COUNTER_USE_INT
	INIT_WORK(&obj->eint_work, QMA6981_eint_work);
#endif


	obj->client = client;
	new_client = obj->client;	 
	i2c_set_clientdata(new_client, obj);
	
	qma6981_i2c_client = new_client;

#ifdef QMA6981_STEP_COUNTER //step counter
	data[0] = 0x13;
	data[1] = 0x80; //clean data
	err = I2C_TxData(data,2);
   	if(err < 0)
   		goto exit_kfree;
#endif
#if 0 //test
	data[0] = 0x13;
	data[1] = 0x80; //clean data
	err = I2C_TxData(data,2);
   	if(err < 0)
   		goto exit_kfree;

	err = I2C_RxData(data,1);
	
	GSE_LOG("%s: err:%d, data:%x--------  22222222222 ---read i2c test-------\n",__FUNCTION__,err,data[0]);
#endif	
	
	
	
	if(0 > qma6981_initialize(new_client))
		goto exit_kfree;
	GSE_LOG("%s: --------  22222222222 ----------\n",__FUNCTION__);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
	err = misc_register(&qma6981_miscdevice);
	if(err)
	{
		GSE_ERR("%s: misc register failed\n",__FUNCTION__);
		goto exit_kfree;
	}
		
	if(0 != (err = qma6981_create_attr(&qma6981_init_info.platform_diver_addr->driver)))
	{	
		GSE_ERR("%s: create attribute\n",__FUNCTION__);
		goto qma6981_create_attribute_failed;
	}
	GSE_LOG("%s: --------  333333333333 ----------\n",__FUNCTION__);

	acc_ctl.open_report_data= qma6981_open_report_data;
	acc_ctl.enable_nodata =qma6981_enable_nodata;
	acc_ctl.set_delay  = qma6981_set_delay;
	acc_ctl.is_report_input_direct = false;
	acc_ctl.is_support_batch = false;

	err = acc_register_control_path(&acc_ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto qma6981_local_init_failed;
	}

	acc_data.get_data = qma6981_get_data;
	acc_data.vender_div = 1000;
	err = acc_register_data_path(&acc_data);
	if(err)
	{
		GSE_ERR("register acc data path err\n");
		goto qma6981_local_init_failed;
	}
	
	err = batch_register_support_info(ID_ACCELEROMETER,acc_ctl.is_support_batch, 1000,0);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	qma6981->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,	
	qma6981->early_drv.suspend  = qma6981_early_suspend,	
	qma6981->early_drv.resume   = qma6981_late_resume,    	
	register_early_suspend(&qma6981->early_drv);
#endif

#ifdef QMA6981_STEP_COUNTER_USE_INT	
	QMA6981_setup_eint(qma6981_i2c_client);
#endif

	GSE_LOG("qma6981 device created successfully\n");
	
	qma6981_init_flag = 0;
	return 0;

qma6981_local_init_failed:
	qma6981_delete_attr(&qma6981_init_info.platform_diver_addr->driver);
qma6981_create_attribute_failed:
exit_kfree:
	kfree(obj);
exit:
	qma6981_init_flag = -1;
	return err;
}


static int qma6981_i2c_remove(struct i2c_client *client)
{
	//int err = 0;
	
#ifdef QMA6981_STEP_COUNTER
	qma6981_delete_attr(&qma6981_step_c_init_info.platform_diver_addr->driver);
#endif
    qma6981_delete_attr(&qma6981_init_info.platform_diver_addr->driver);
	/*err = */misc_deregister(&qma6981_miscdevice);
	/*if(err)
	{
		GSE_ERR("%s failed\n",__FUNCTION__);
	}*/
	qma6981_i2c_client = NULL;
	kfree(i2c_get_clientdata(client));
	qma6981 = NULL;
	i2c_unregister_device(client);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void qma6981_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;
#if 0
	//GSE_LOG("qma6981_power id:%d on:%d power_on:%d\n",hw->power_id,on,power_on);
	if(hw->power_id != MT65XX_POWER_NONE)
	{
		GSE_LOG("qma6981_power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			GSE_LOG("qma6981_ignore power control: %d\n", on);
		}
		else if(on)
		{
			GSE_LOG("qma6981_power_A %s\n", on ? "on" : "off");
			if(!hwPowerOn(hw->power_id, hw->power_vol, "qma6981"))
			{
				GSE_LOG( "qma6981_power on fails!!\n");
			}
		}
		else
		{	GSE_LOG("qma6981_power_B %s\n", on ? "on" : "off");
			if(!hwPowerDown(hw->power_id, "qma6981"))
			{
				GSE_LOG( "qma6981_power off fail!!\n");
			}
		}
	}
#endif
	power_on = on;
}


#if 1
#if !defined(CONFIG_HAS_EARLYSUSPEND)
/*----------------------------------------------------------------------------*/
//static int qma6981_suspend(struct i2c_client *client, pm_message_t msg)
static int qma6981_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);	
	struct qma6981_data *obj = i2c_get_clientdata(client);
	GSE_FUN();
	
	if(NULL == obj)
	{
		GSE_LOG("null pointer!!\n");
		return 0;
	}
	//if(msg.event == PM_EVENT_SUSPEND)
	if (enable_status == 0)
	{
		GSE_FUN();
		atomic_set(&obj->suspend, 1);
#ifndef QMA6981_STEP_COUNTER
		qma6981_power(obj->hw, 0); //don't power down if enable stepcounter
		qma6981_SetPowerCTRL(obj->client,false);
#endif
	}

	return 0;
}

//static int qma6981_resume(struct i2c_client *client)
static int qma6981_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);	
	struct qma6981_data *obj = i2c_get_clientdata(client);
#ifndef QMA6981_STEP_COUNTER
	int err;
#endif
	GSE_FUN();
	
	if(NULL == obj)
	{
		GSE_LOG("null pointer!!\n");
		return 0;
	}
#ifndef QMA6981_STEP_COUNTER
	qma6981_power(obj->hw, 1);
	qma6981_SetPowerCTRL(obj->client,true);
	err = qma6981_initialize(client);
	if(0 > err)
	{
		GSE_ERR("failed to init qma6981\n");
	}
#endif
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void qma6981_early_suspend(struct early_suspend *h)
{
	struct qma6981_data *obj = container_of(h, struct qma6981_data, early_drv);
	GSE_FUN();
	if(NULL == obj)
	{
		GSE_LOG("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend,1);
#ifndef QMA6981_STEP_COUNTER
	qma6981_power(obj->hw, 0); //don't power down if enable stepcounter
	qma6981_SetPowerCTRL(obj->client,false);
#endif
}
/*----------------------------------------------------------------------------*/
static void qma6981_late_resume(struct early_suspend *h)
{
	struct qma6981_data *obj = container_of(h, struct qma6981_data, early_drv);
#ifndef QMA6981_STEP_COUNTER
	int err;
#endif

	GSE_FUN();
	if(NULL == obj)
	{
		GSE_LOG("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend,0);
#ifndef QMA6981_STEP_COUNTER
	qma6981_power(obj->hw, 1);
	qma6981_SetPowerCTRL(obj->client,true);
	err = qma6981_initialize(obj->client);
	if(err)
	{
		GSE_ERR("failed to init qma6981\n");
	}
#endif
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
#endif


static const struct i2c_device_id qma6981_id[] = {{QMA6981_ACC_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,gsensor"},
	{},
};
#endif
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops qma6981_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(qma6981_suspend, qma6981_resume)
};
#endif
static struct i2c_driver qma6981_driver = {
	.probe = qma6981_i2c_probe,
	.remove = qma6981_i2c_remove,
	.id_table = qma6981_id,
#if 0//!defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = qma6981_suspend,
	.resume = qma6981_resume,
#endif

	.driver = {
		.owner = THIS_MODULE,
		.name = QMA6981_ACC_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &qma6981_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif
	},
};

static int qma6981_local_init(void)
{
	GSE_FUN();
	GSE_LOG("++++++++\n");

	qma6981_power(hw, 1);
	
	if(i2c_add_driver(&qma6981_driver))
	{
		GSE_ERR("add i2c driver error\n");
		return -1;
	}
	if(-1 == qma6981_init_flag)
	{
		GSE_ERR("qma6981 init error\n");
		return -1;
	}		
	
	GSE_LOG("--------\n");
	return 0;
}

static int qma6981_local_uninit(void)
{
    GSE_FUN();
	
    qma6981_power(hw, 0);    
	
    i2c_del_driver(&qma6981_driver);
	
    return 0;
}

#ifdef QMA6981_STEP_COUNTER
static int qma6981_step_c_local_init(void)
{
	int err = 0;
	struct step_c_control_path step_ctl = {0};
	struct step_c_data_path step_data = {0};
	
	GSE_FUN();
	GSE_LOG("++++++++\n");
	mutex_lock(&qma6981_init_mutex);
	//TODO IF ACC NOT REGISTER I2C
	// add by yangzhiqiang 2016/11/30
	wake_lock_init(&sc_wakelock,WAKE_LOCK_SUSPEND,"sc wakelock");
	// yangzhiqiang
	if(0 != (err = qma6981_create_attr(&qma6981_step_c_init_info.platform_diver_addr->driver)))
	{
		GSE_ERR("%s: create attribute\n",__FUNCTION__);
		goto qma6981_step_c_create_attribute_failed;
	}

#if 1		//android N
	step_ctl.open_report_data = qma6981_step_c_open_report_data;
	step_ctl.enable_nodata=qma6981_step_c_enable_nodata;
	step_ctl.enable_significant= qma6981_step_c_enable_significant;
	step_ctl.enable_step_detect = qma6981_step_c_enable_step_detect;
	step_ctl.step_c_set_delay =  qma6981_step_c_set_delay;
	step_ctl.step_d_set_delay = qma6981_step_d_set_delay;
	step_ctl.is_report_input_direct= false;
#else		//android M
	step_ctl.open_report_data = qma6981_step_c_open_report_data;
	step_ctl.enable_nodata=qma6981_step_c_enable_nodata;
	step_ctl.enable_significant= qma6981_step_c_enable_significant;
	step_ctl.enable_step_detect = qma6981_step_c_enable_step_detect;
	step_ctl.set_delay =  qma6981_step_c_set_delay;
	step_ctl.is_report_input_direct= false;
#endif
	
	err=step_c_register_control_path(&step_ctl);
	if(err)
	{
	 	GSE_ERR("register step_counter control path err\n");
		goto qma6981_step_c_local_init_failed;
	}

	step_data.get_data=qma6981_step_c_get_sc;
	step_data.get_data_significant = qma6981_step_c_get_data_significant;
	step_data.get_data_step_d = qma6981_step_c_get_data_step_d;
	step_data.vender_div = 1;
	step_c_register_data_path(&step_data);

	if(err)
	{
	 	GSE_ERR("register step_counter data path err\n");
		goto qma6981_step_c_local_init_failed;
	}
	mutex_unlock(&qma6981_init_mutex);
	GSE_LOG("--------\n");
	return 0;

qma6981_step_c_local_init_failed:
	qma6981_delete_attr(&qma6981_step_c_init_info.platform_diver_addr->driver);
qma6981_step_c_create_attribute_failed:
//	misc_deregister(&qma6981_step_c_miscdevice);
	mutex_unlock(&qma6981_init_mutex);
	return err;
}


static int qma6981_step_c_local_uninit(void)
{
//    misc_deregister(&qma6981_step_c_miscdevice);
    return 0;
}	

#endif

static struct acc_init_info qma6981_init_info = {
	.name = QMA6981_ACC_DEV_NAME,
	.init = qma6981_local_init,
	.uninit = qma6981_local_uninit,	
};

#ifdef QMA6981_STEP_COUNTER
static struct step_c_init_info qma6981_step_c_init_info = {
	.name = QMA6981_STEP_C_DEV_NAME,
	.init = qma6981_step_c_local_init,
	.uninit = qma6981_step_c_local_uninit,
};
#endif
#ifdef QMA6981_STEP_COUNTER_USE_INT
static int qma6981_remove(struct platform_device *pdev)
{
	printk("qma6981_remove\n");
	return 0;
}

static int qma6981_probe(struct platform_device *pdev)
{
	printk("qma6981_probe\n");
	qma6981_dev = pdev;
	return 0;
}

static struct platform_driver qma6981_int_driver = {
	.probe	  = qma6981_probe,
	.remove	  = qma6981_remove,
	.driver = {

		.name  = "qma6981",
	#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
		#endif
	}
};
#endif
static int __init qma6981_init(void)
{
	const char *name = "mediatek,qma6981";
	hw = get_accel_dts_func(name,hw);
	if(!hw){
		GSE_ERR("get cust_accel dts info fail\n");	
	}
	#ifdef QMA6981_STEP_COUNTER_USE_INT
	platform_driver_register(&qma6981_int_driver);
	#endif
	GSE_LOG("%s:i2c_num %d, direction %d, i2c_addr 0x%x\n",__FUNCTION__,hw->i2c_num,hw->direction,hw->i2c_addr[0]);
	acc_driver_add(&qma6981_init_info);

#ifdef QMA6981_STEP_COUNTER
	step_c_driver_add(&qma6981_step_c_init_info);
#endif
	return 0;
}

static void __exit qma6981_exit(void)
{
	#ifdef QMA6981_STEP_COUNTER_USE_INT
	platform_driver_unregister(&qma6981_int_driver);
	#endif
	GSE_FUN();
}

module_init(qma6981_init);
module_exit(qma6981_exit);

MODULE_DESCRIPTION("QST qma6981 Acc driver");
MODULE_AUTHOR("QST");
MODULE_LICENSE("GPL");
