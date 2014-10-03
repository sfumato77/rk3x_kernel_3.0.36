/* drivers/i2c/chips/mc3230.c - mc3230 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/mc3230.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define MC32X0_AXIS_X		   0
#define MC32X0_AXIS_Y		   1
#define MC32X0_AXIS_Z		   2
#define MC32X0_AXES_NUM 	   3
#define MC32X0_DATA_LEN 	   6
#define MC32X0_DEV_NAME 	   "MC32X0"
#define GRAVITY_EARTH_1000 		9807
#define IS_MC3230 1
#define IS_MC3210 2

#define FIRST_TRY_TIME		3     //此为没有校正之前，需要读取文件的次数，尽量少，以免影响开机速度
/*----------------------------------------------------------------------------*/

#define CALIB_PATH				"/data/data/com.mcube.acc/files/mcube-calib.txt"
#define DATA_PATH			   "/sdcard/mcube-register-map.txt"

static GSENSOR_VECTOR3D gsensor_gain;

static int fd_file = -1;
static int load_cali_flg = 0;
static mm_segment_t oldfs;
//add by Liang for storage offset data
static unsigned char offset_buf[9]; 
static signed int offset_data[3];
s16 G_RAW_DATA[3];
static signed int gain_data[3];
static signed int enable_RBM_calibration = 0;
static unsigned char mc32x0_type;


#if 0
#define mcprintkreg(x...) printk(x)
#else
#define mcprintkreg(x...)
#endif

#if 0
#define mcprintkfunc(x...) printk(x)
#else
#define mcprintkfunc(x...)
#endif

#if 1
#define GSE_ERR(x...) 	printk(x)
#define GSE_LOG(x...) 	printk(x)
#else
#define GSE_ERR(x...) 	
#define GSE_LOG(x...) 	
#endif

#if 0
#if 1
#define GSE_TAG 				 "[Gsensor] "
#define GSE_FUN(f)				 printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)	 printk(KERN_INFO GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)	 printk(KERN_INFO GSE_TAG fmt, ##args)
#else
#define GSE_TAG
#define GSE_FUN(f)				 do {} while (0)
#define GSE_ERR(fmt, args...)	 do {} while (0)
#define GSE_LOG(fmt, args...)	 do {} while (0)
#endif
#endif
static int  mc3230_probe(struct i2c_client *client, const struct i2c_device_id *id);

#define MC3230_SPEED		300 * 1000
#define MC3230_DEVID		0x01
/* Addresses to scan -- protected by sense_data_mutex */
//static char sense_data[RBUFF_SIZE + 1];
static struct i2c_client *this_client;
static struct miscdevice mc3230_device;

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend mc3230_early_suspend;
#endif
//static int revision = -1;
static const char* vendor = "Mcube";


typedef char status_t;
/*status*/
#define MC3230_OPEN           1
#define MC3230_CLOSE          0

//by zwx
struct hwmsen_convert {
	s8 sign[3];
	u8 map[3];
};

struct mc3230_data {
    status_t status;
    char  curr_rate;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct work_struct work;
	struct delayed_work delaywork;	/*report second event*/
    
    struct mc3230_axis sense_data;
    struct mutex sense_data_mutex;
    atomic_t data_ready;
    wait_queue_head_t data_ready_wq;

    int start_count;
    struct mutex operation_mutex;

//zwx
	s16 					offset[MC32X0_AXES_NUM+1];	/*+1: for 4-byte alignment*/
	s16 					data[MC32X0_AXES_NUM+1]; 
	s16                     cali_sw[MC32X0_AXES_NUM+1];

	struct hwmsen_convert   cvt;
};
static int MC32X0_WriteCalibration(struct i2c_client *client, int dat[MC32X0_AXES_NUM]);

static int mc3230_write_reg(struct i2c_client *client,int addr,int value);
static char mc3230_read_reg(struct i2c_client *client,int addr);
static int mc3230_rx_data(struct i2c_client *client, char *rxData, int length);
static int mc3230_tx_data(struct i2c_client *client, char *txData, int length);
static int mc3230_read_block(struct i2c_client *client, char reg, char *rxData, int length);
static int mc3230_write_block(struct i2c_client *client, char reg, char *txData, int length);


struct file *openFile(char *path,int flag,int mode) 
{ 
	struct file *fp; 
	 
	fp=filp_open(path, flag, mode); 
	if (IS_ERR(fp) || !fp->f_op) 
	{
		GSE_LOG("Calibration File filp_open return NULL\n");
		return NULL; 
	}
	else 
	{

		return fp; 
	}
} 
 
int readFile(struct file *fp,char *buf,int readlen) 
{ 
	if (fp->f_op && fp->f_op->read) 
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
	else 
		return -1; 
} 

int writeFile(struct file *fp,char *buf,int writelen) 
{ 
	if (fp->f_op && fp->f_op->write) 
		return fp->f_op->write(fp,buf,writelen, &fp->f_pos); 
	else 
		return -1; 
}
 
int closeFile(struct file *fp) 
{ 
	filp_close(fp,NULL); 
	return 0; 
} 

void initKernelEnv(void) 
{ 
	oldfs = get_fs(); 
	set_fs(KERNEL_DS);
	printk(KERN_INFO "initKernelEnv\n");
} 

static int mcube_read_cali_file(struct i2c_client *client)
{
	int cali_data[3];
	int err =0;
	char buf[64];
	printk("%s %d\n",__func__,__LINE__);
	initKernelEnv();
	fd_file = openFile(CALIB_PATH,O_RDONLY,0); 
	if (fd_file == NULL) 
	{
		GSE_LOG("fail to open\n");
		cali_data[0] = 0;
		cali_data[1] = 0;
		cali_data[2] = 0;
		return 1;
	}
	else
	{
		printk("%s %d\n",__func__,__LINE__);
		memset(buf,0,64); 
		if ((err = readFile(fd_file,buf,128))>0) 
			GSE_LOG("buf:%s\n",buf); 
		else 
			GSE_LOG("read file error %d\n",err); 
		printk("%s %d\n",__func__,__LINE__);

		set_fs(oldfs); 
		closeFile(fd_file); 

		sscanf(buf, "%d %d %d",&cali_data[MC32X0_AXIS_X], &cali_data[MC32X0_AXIS_Y], &cali_data[MC32X0_AXIS_Z]);
		GSE_LOG("cali_data: %d %d %d\n", cali_data[MC32X0_AXIS_X], cali_data[MC32X0_AXIS_Y], cali_data[MC32X0_AXIS_Z]); 	
				
		//cali_data1[MC32X0_AXIS_X] = cali_data[MC32X0_AXIS_X] * gsensor_gain.x / GRAVITY_EARTH_1000;
		//cali_data1[MC32X0_AXIS_Y] = cali_data[MC32X0_AXIS_Y] * gsensor_gain.y / GRAVITY_EARTH_1000;
		//cali_data1[MC32X0_AXIS_Z] = cali_data[MC32X0_AXIS_Z] * gsensor_gain.z / GRAVITY_EARTH_1000;
		//cali_data[MC32X0_AXIS_X]=-cali_data[MC32X0_AXIS_X];
		//cali_data[MC32X0_AXIS_Y]=-cali_data[MC32X0_AXIS_Y];
		//cali_data[MC32X0_AXIS_Z]=-cali_data[MC32X0_AXIS_Z];

		//GSE_LOG("cali_data1: %d %d %d\n", cali_data1[MC32X0_AXIS_X], cali_data1[MC32X0_AXIS_Y], cali_data1[MC32X0_AXIS_Z]); 	
		printk("%s %d\n",__func__,__LINE__);	  
		MC32X0_WriteCalibration(client, cali_data);
	}
	return 0;
}


static void MC32X0_rbm(struct i2c_client *client, int enable)
{
	int err; 

	if(enable == 1 )
	{
#if 0
		buf1[0] = 0x43; 
		err = mc3230_write_block(client, 0x07, buf1, 0x01);

		buf1[0] = 0x02; 
		err = mc3230_write_block(client, 0x14, buf1, 0x01);

		buf1[0] = 0x41; 
		err = mc3230_write_block(client, 0x07, buf1, 0x01);
#endif
		err = mc3230_write_reg(client,0x07,0x43);
		err = mc3230_write_reg(client,0x14,0x02);
		err = mc3230_write_reg(client,0x07,0x41);

		enable_RBM_calibration =1;
		
		GSE_LOG("set rbm!!\n");

		msleep(10);
	}
	else if(enable == 0 )  
	{
#if 0
		buf1[0] = 0x43; 
		err = mc3230_write_block(client, 0x07, buf1, 0x01);

		buf1[0] = 0x00; 
		err = mc3230_write_block(client, 0x14, buf1, 0x01);

		buf1[0] = 0x41; 
		err = mc3230_write_block(client, 0x07, buf1, 0x01);
#endif	
		err = mc3230_write_reg(client,0x07,0x43);
		err = mc3230_write_reg(client,0x14,0x00);
		err = mc3230_write_reg(client,0x07,0x41);
		enable_RBM_calibration =0;

		GSE_LOG("clear rbm!!\n");

		msleep(10);
	}
}

/*----------------------------------------------------------------------------*/

static int MC32X0_ReadData_RBM(struct i2c_client *client, int data[MC32X0_AXES_NUM])
{   
	//u8 uData;
	u8 addr = 0x0d;
	u8 rbm_buf[MC32X0_DATA_LEN] = {0};
	int err = 0;
	if(NULL == client)
	{
		err = -EINVAL;
		return err;
	}

	err = mc3230_read_block(client, addr, rbm_buf, 0x06);

	data[MC32X0_AXIS_X] = (s16)((rbm_buf[0]) | (rbm_buf[1] << 8));
	data[MC32X0_AXIS_Y] = (s16)((rbm_buf[2]) | (rbm_buf[3] << 8));
	data[MC32X0_AXIS_Z] = (s16)((rbm_buf[4]) | (rbm_buf[5] << 8));

	GSE_LOG("rbm_buf<<<<<[%02x %02x %02x %02x %02x %02x]\n",rbm_buf[0], rbm_buf[2], rbm_buf[2], rbm_buf[3], rbm_buf[4], rbm_buf[5]);
	GSE_LOG("RBM<<<<<[%04x %04x %04x]\n", data[MC32X0_AXIS_X], data[MC32X0_AXIS_Y], data[MC32X0_AXIS_Z]);
	GSE_LOG("RBM<<<<<[%04d %04d %04d]\n", data[MC32X0_AXIS_X], data[MC32X0_AXIS_Y], data[MC32X0_AXIS_Z]);		
	return err;
}

/* AKM HW info */
static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	// sprintf(buf, "%#x\n", revision);
	sprintf(buf, "%s.\n", vendor);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static struct kobject *android_gsensor_kobj;

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		printk(KERN_ERR
		       "MC3230 gsensor_sysfs_init:"\
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);   // "vendor"
	if (ret) {
		printk(KERN_ERR
		       "MC3230 gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

static int mc3230_read_block(struct i2c_client *client, char reg, char *rxData, int length)
{
	int ret = 0;
	ret = i2c_master_reg8_recv(client, reg, rxData, length, MC3230_SPEED);
	return (ret > 0)? 0 : ret;
}

static int mc3230_write_block(struct i2c_client *client, char reg, char *txData, int length)
{
	int ret = 0;
	ret = i2c_master_reg8_send(client, reg, &txData[0], length, MC3230_SPEED);
	return (ret > 0)? 0 : ret;
}

static int mc3230_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = i2c_master_reg8_recv(client, reg, rxData, length, MC3230_SPEED);
	return (ret > 0)? 0 : ret;
}

static int mc3230_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	char reg = txData[0];
	ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, MC3230_SPEED);
	return (ret > 0)? 0 : ret;
}

static char mc3230_read_reg(struct i2c_client *client,int addr)
{
	char tmp;
	int ret = 0;

	tmp = addr;
	ret = mc3230_rx_data(client, &tmp, 1);
	return tmp;
}

static int mc3230_write_reg(struct i2c_client *client,int addr,int value)
{
	char buffer[3];
	int ret = 0;

	buffer[0] = addr;
	buffer[1] = value;
	ret = mc3230_tx_data(client, &buffer[0], 2);
	return ret;
}


static char mc3230_get_devid(struct i2c_client *client)
{
	mcprintkreg("mc3230 devid:%x\n",mc3230_read_reg(client,MC3230_REG_CHIP_ID));
	return mc3230_read_reg(client,MC3230_REG_CHIP_ID);
}

static int mc3230_active(struct i2c_client *client,int enable)
{
	int tmp;
	int ret = 0;
	if(enable)
		tmp = 0x01;
	else
		tmp = 0x03;
	mcprintkreg("mc3230_active %s (0x%x)\n",enable?"active":"standby",tmp);	
	ret = mc3230_write_reg(client,MC3230_REG_SYSMOD,tmp);
	return ret;
}

static int mc3230_reg_init(struct i2c_client *client)
{
	int ret = 0;
	int pcode = 0;

	mcprintkfunc("-------------------------mc3230 init------------------------\n");	
	

	mc3230_active(client,0);  // 1:awake  0:standby   中断io相关设置
	mcprintkreg("mc3230 MC3230_REG_SYSMOD:%x\n",mc3230_read_reg(client,MC3230_REG_SYSMOD));

#if 0//zwx	
	 ret = mc3230_read_block(client, 0x3b, databuf, 1);
#endif

	pcode = mc3230_read_reg(client,MC3230_REG_PRODUCT_CODE);
	if( pcode=0x19 )
	{
		mc32x0_type = IS_MC3230;
	}
	else if ( pcode=0x90 )
	{
		mc32x0_type = IS_MC3210;
	}
	
	ret = mc3230_write_reg(client,MC3230_REG_RATE_SAMP,0x0);
	mcprintkreg("mc3230 MC3230_REG_RATE_SAMP:%x\n",mc3230_read_reg(client,MC3230_REG_RATE_SAMP));
	
	/* set  Generate Interrupt */
	ret = mc3230_write_reg(client,MC3230_REG_INTMOD,0x11);
	mcprintkreg("mc3230 MC3230_REG_INTMOD:%x\n",mc3230_read_reg(client,MC3230_REG_INTMOD));

	ret = mc3230_write_reg(client,MC3230_REG_SLEEP_COUNTER,0xff);
	mcprintkreg("mc3230 MC3230_REG_SLEEP_COUNTER:%x\n",mc3230_read_reg(client,MC3230_REG_SLEEP_COUNTER));

	if ( mc32x0_type == IS_MC3230 )
	{
		ret = mc3230_write_reg(client, 0x20, 0x32);
	}
	else if ( mc32x0_type == IS_MC3210 )
	{
		ret = mc3230_write_reg(client, 0x20, 0x3F);
	}


	if ( mc32x0_type == IS_MC3230 )
	{
		gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 86;
	}
	else if ( mc32x0_type == IS_MC3210 )
	{
		gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;
	}

	MC32X0_rbm(client,0);



//	mc3230_active(client,1); 
//	mcprintkreg("mc3230 0x07:%x\n",mc3230_read_reg(client,MC3230_REG_SYSMOD));
//	enable_irq(client->irq);
//	msleep(50);
	return ret;
}

static int mc3230_start_dev(struct i2c_client *client, char rate)
{
	int ret = 0;
	struct mc3230_data *mc3230 = (struct mc3230_data *)i2c_get_clientdata(client);   // mc3230_data 定义在 mc3230.h 中. 

	mcprintkfunc("-------------------------mc3230 start dev------------------------\n");	
	/* standby */
	mc3230_active(client,0);
	mcprintkreg("mc3230 MC3230_REG_SYSMOD:%x\n",mc3230_read_reg(client,MC3230_REG_SYSMOD));

	/*data rate*/
	ret = mc3230_write_reg(client,MC3230_REG_RATE_SAMP,rate);
	mc3230->curr_rate = rate;
	mcprintkreg("mc3230 MC3230_REG_RATE_SAMP:%x  rate=%d\n",mc3230_read_reg(client,MC3230_REG_RATE_SAMP),rate);
	/*wake*/
	mc3230_active(client,1);
	mcprintkreg("mc3230 MC3230_REG_SYSMOD:%x\n",mc3230_read_reg(client,MC3230_REG_SYSMOD));
	
	enable_irq(client->irq);
	return ret;

}

static int mc3230_start(struct i2c_client *client, char rate)
{ 
	struct mc3230_data *mc3230 = (struct mc3230_data *)i2c_get_clientdata(client);

	mcprintkfunc("%s::enter\n",__FUNCTION__); 
	if (mc3230->status == MC3230_OPEN) {
		return 0;      
	}
	mc3230->status = MC3230_OPEN;
	rate = 0;
	return mc3230_start_dev(client, rate);
}

static int mc3230_close_dev(struct i2c_client *client)
{    	
	disable_irq_nosync(client->irq);
	return mc3230_active(client,0);
}

static int mc3230_close(struct i2c_client *client)
{
	struct mc3230_data *mc3230 = (struct mc3230_data *)i2c_get_clientdata(client);
	mcprintkfunc("%s::enter\n",__FUNCTION__); 
	mc3230->status = MC3230_CLOSE;

	return mc3230_close_dev(client);
}

static int mc3230_reset_rate(struct i2c_client *client, char rate)
{
	int ret = 0;
	
	mcprintkfunc("\n----------------------------mc3230_reset_rate------------------------\n");
	rate = (rate & 0x07);
	disable_irq_nosync(client->irq);
    	ret = mc3230_start_dev(client, rate);
  
	return ret ;
}

static inline int mc3230_convert_to_int(s16 value)
{
    int result;


    if (value < MC3230_BOUNDARY) {
       result = value * MC3230_GRAVITY_STEP;
    } else {
       result = ~(((~value & 0x7f) + 1)* MC3230_GRAVITY_STEP) + 1;
    }
		

    return result;
}



static void mc3230_report_value(struct i2c_client *client, struct mc3230_axis *axis)
{
	struct mc3230_data *mc3230 = i2c_get_clientdata(client);
    //struct mc3230_axis *axis = (struct mc3230_axis *)rbuf;

	/* Report acceleration sensor information */
    input_report_abs(mc3230->input_dev, ABS_X, axis->x);
    input_report_abs(mc3230->input_dev, ABS_Y, axis->y);
    input_report_abs(mc3230->input_dev, ABS_Z, axis->z);
    input_sync(mc3230->input_dev);
    mcprintkreg("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);
}

static int MC32X0_ReadData(struct i2c_client *client, s16 buffer[MC32X0_AXES_NUM]);

/** 在 底半部执行, 具体获取 g sensor 数据. */
static int mc3230_get_data(struct i2c_client *client)
{
    struct mc3230_data* mc3230 = i2c_get_clientdata(client);
	s16 buffer[6];
	int ret;
	int x,y,z;
    struct mc3230_axis axis;
    struct mc3230_platform_data *pdata = pdata = client->dev.platform_data;
	//printk("%d\n==========",load_cali_flg);
 	if( load_cali_flg > 0)
	{
		ret =mcube_read_cali_file(client);
		if(ret == 0)
			load_cali_flg = ret;
		else 
			load_cali_flg --;
		GSE_LOG("load_cali %d\n",ret); 
	}  	
		
	if(ret = MC32X0_ReadData(client, buffer))
	{    
		
		GSE_ERR("%s I2C error: ret value=%d", __func__,ret);
		return EIO;
	}
	mcprintkreg("%s %x %x %x \n",__func__,buffer[0],buffer[1],buffer[2]);
	
	x = mc3230_convert_to_int(buffer[0]);
	y = mc3230_convert_to_int(buffer[1]);
	z = mc3230_convert_to_int(buffer[2]);

	if (pdata->swap_xyz) {
		axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
		axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;	
		axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;
	}
	else {
		axis.x = x;
		axis.y = y;	
		axis.z = z;
	}

	if(pdata->swap_xy)
	{
             axis.x = -axis.x;
             swap(axis.x,axis.y); 
	}
	
    mcprintkreg( "%s: ------------------mc3230_GetData axis = %d  %d  %d--------------\n",
            __func__, axis.x, axis.y, axis.z); 
     
    //memcpy(sense_data, &axis, sizeof(axis));
    mc3230_report_value(client, &axis);
	//atomic_set(&data_ready, 0);
	//wake_up(&data_ready_wq);

    /* 互斥地缓存数据. */
    mutex_lock(&(mc3230->sense_data_mutex) );
    mc3230->sense_data = axis;
    mutex_unlock(&(mc3230->sense_data_mutex) );

    /* 置位 data_ready */
    atomic_set(&(mc3230->data_ready), 1);
    /* 唤醒 data_ready 等待队列头. */
	wake_up(&(mc3230->data_ready_wq) );

	return 0;
}
static int MC32X0_ReadRBMData(struct i2c_client *client, char *buf)
{
	struct mc3230_data *mc3230 = (struct mc3230_data*)i2c_get_clientdata(client);
	int res = 0;
	int data[3];

	if (!buf || !client)
	{
		return EINVAL;
	}

	if(mc3230->status == MC3230_CLOSE)
	{
		res = mc3230_start(client, 0);
		if(res)
		{
			GSE_ERR("Power on mc32x0 error %d!\n", res);
		}
	}
	
	if(res = MC32X0_ReadData_RBM(client, data))
	{        
		GSE_ERR("%s I2C error: ret value=%d",__func__, res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", data[MC32X0_AXIS_X], 
			data[MC32X0_AXIS_Y], data[MC32X0_AXIS_Z]);
	
	}
	
	return 0;
}
static int MC32X0_ReadOffset(struct i2c_client *client, s16 ofs[MC32X0_AXES_NUM])
{    
	int err;
	u8 off_data[6];
	

	if ( mc32x0_type == IS_MC3210 )
	{
		if ((err = mc3230_read_block(client, MC32X0_XOUT_EX_L_REG, off_data, MC32X0_DATA_LEN))) 
    		{
    			GSE_ERR("error: %d\n", err);
    			return err;
    		}
		ofs[MC32X0_AXIS_X] = ((s16)(off_data[0]))|((s16)(off_data[1])<<8);
		ofs[MC32X0_AXIS_Y] = ((s16)(off_data[2]))|((s16)(off_data[3])<<8);
		ofs[MC32X0_AXIS_Z] = ((s16)(off_data[4]))|((s16)(off_data[5])<<8);
	}
	else if (mc32x0_type == IS_MC3230) 
	{
		if ((err = mc3230_read_block(client, 0, off_data, 3))) 
    		{
    			GSE_ERR("error: %d\n", err);
    			return err;
    		}
		ofs[MC32X0_AXIS_X] = (s8)off_data[0];
		ofs[MC32X0_AXIS_Y] = (s8)off_data[1];
		ofs[MC32X0_AXIS_Z] = (s8)off_data[2];			
	}

	GSE_LOG("MC32X0_ReadOffset %d %d %d \n",ofs[MC32X0_AXIS_X] ,ofs[MC32X0_AXIS_Y],ofs[MC32X0_AXIS_Z]);

    return 0;  
}
/*----------------------------------------------------------------------------*/
static int MC32X0_ResetCalibration(struct i2c_client *client)
{
	struct mc3230_data *mc3230 = i2c_get_clientdata(client);
	s16 tmp;
#if 0   //zwx	
		buf[0] = 0x43;
		if(err = mc3230_write_block(client, 0x07, buf, 1))
		{
			GSE_ERR("error 0x07: %d\n", err);
		}


		if(err = mc3230_write_block(client, 0x21, offset_buf, 6)) // add by liang for writing offset register as OTP value 
		{
			GSE_ERR("error: %d\n", err);
		}
	
		buf[0] = 0x41;
		if(err = mc3230_write_block(client, 0x07, buf, 1))
		{
			GSE_ERR("error: %d\n", err);
		}
#endif
		mc3230_write_reg(client,0x07,0x43);

		mc3230_write_block(client, 0x21, offset_buf, 6);
		
		mc3230_write_reg(client,0x07,0x41);

		msleep(20);

		tmp = ((offset_buf[1] & 0x3f) << 8) + offset_buf[0];  // add by Liang for set offset_buf as OTP value 
		if (tmp & 0x2000)
			tmp |= 0xc000;
		offset_data[0] = tmp;
					
		tmp = ((offset_buf[3] & 0x3f) << 8) + offset_buf[2];  // add by Liang for set offset_buf as OTP value 
			if (tmp & 0x2000)
				tmp |= 0xc000;
		offset_data[1] = tmp;
					
		tmp = ((offset_buf[5] & 0x3f) << 8) + offset_buf[4];  // add by Liang for set offset_buf as OTP value 
		if (tmp & 0x2000)
			tmp |= 0xc000;
		offset_data[2] = tmp;	

	memset(mc3230->cali_sw, 0x00, sizeof(mc3230->cali_sw));
	return 0;  

}
/*----------------------------------------------------------------------------*/
static int MC32X0_ReadCalibration(struct i2c_client *client, int dat[MC32X0_AXES_NUM])
{
	
    struct mc3230_data *mc3230 = i2c_get_clientdata(client);
    int err;
	
    if ((err = MC32X0_ReadOffset(client, mc3230->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    dat[MC32X0_AXIS_X] = mc3230->offset[MC32X0_AXIS_X];
    dat[MC32X0_AXIS_Y] = mc3230->offset[MC32X0_AXIS_Y];
    dat[MC32X0_AXIS_Z] = mc3230->offset[MC32X0_AXIS_Z];  
	//modify by zwx
	//GSE_LOG("MC32X0_ReadCalibration %d %d %d \n",dat[mc3230->cvt.map[MC32X0_AXIS_X]] ,dat[mc3230->cvt.map[MC32X0_AXIS_Y]],dat[mc3230->cvt.map[MC32X0_AXIS_Z]]);
                                      
    return 0;
}

/*----------------------------------------------------------------------------*/
static int MC32X0_WriteCalibration(struct i2c_client *client, int dat[MC32X0_AXES_NUM])
{
	int err;
	u8 buf[9];
	s16 tmp, x_gain, y_gain, z_gain ;
	s32 x_off, y_off, z_off;
#if 1  //modify by zwx

	GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", 
	dat[MC32X0_AXIS_X], dat[MC32X0_AXIS_Y], dat[MC32X0_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	//cali_temp[MC32X0_AXIS_X] = dat[MC32X0_AXIS_X];
	//cali_temp[MC32X0_AXIS_Y] = dat[MC32X0_AXIS_Y];
	//cali_temp[MC32X0_AXIS_Z] = dat[MC32X0_AXIS_Z];
	//cali[MC32X0_AXIS_Z]= cali[MC32X0_AXIS_Z]-gsensor_gain.z;


#endif	
// read register 0x21~0x28
#if 0 //zwx
	if ((err = mc3230_read_block(client, 0x21, buf, 3))) 
	{
		GSE_ERR("error: %d\n", err);
		return err;
	}
	if ((err = mc3230_read_block(client, 0x24, &buf[3], 3))) 
	{
		GSE_ERR("error: %d\n", err);
		return err;
	}
	if ((err = mc3230_read_block(client, 0x27, &buf[6], 3))) 
	{
		GSE_ERR("error: %d\n", err);
		return err;

	}
#endif
	buf[0] = 0x21;
	err = mc3230_rx_data(client, &buf[0], 3);
	buf[3] = 0x24;
	err = mc3230_rx_data(client, &buf[3], 3);
	buf[6] = 0x27;
	err = mc3230_rx_data(client, &buf[6], 3);
#if 1
	// get x,y,z offset
	tmp = ((buf[1] & 0x3f) << 8) + buf[0];
		if (tmp & 0x2000)
			tmp |= 0xc000;
		x_off = tmp;
					
	tmp = ((buf[3] & 0x3f) << 8) + buf[2];
		if (tmp & 0x2000)
			tmp |= 0xc000;
		y_off = tmp;
					
	tmp = ((buf[5] & 0x3f) << 8) + buf[4];
		if (tmp & 0x2000)
			tmp |= 0xc000;
		z_off = tmp;
					
	// get x,y,z gain
	x_gain = ((buf[1] >> 7) << 8) + buf[6];
	y_gain = ((buf[3] >> 7) << 8) + buf[7];
	z_gain = ((buf[5] >> 7) << 8) + buf[8];
								
	// prepare new offset
	x_off = x_off + 16 * dat[MC32X0_AXIS_X] * 256 * 128 / 3 / gsensor_gain.x / (40 + x_gain);
	y_off = y_off + 16 * dat[MC32X0_AXIS_Y] * 256 * 128 / 3 / gsensor_gain.y / (40 + y_gain);
	z_off = z_off + 16 * dat[MC32X0_AXIS_Z] * 256 * 128 / 3 / gsensor_gain.z / (40 + z_gain);

	//storege the cerrunt offset data with DOT format
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	//storege the cerrunt Gain data with GOT format
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);
	printk("%d %d ======================\n\n ",gain_data[0],x_gain);
#endif
	//buf[0]=0x43;
	//mc3230_write_block(client, 0x07, buf, 1);
	mc3230_write_reg(client,0x07,0x43);
					
	buf[0] = x_off & 0xff;
	buf[1] = ((x_off >> 8) & 0x3f) | (x_gain & 0x0100 ? 0x80 : 0);
	buf[2] = y_off & 0xff;
	buf[3] = ((y_off >> 8) & 0x3f) | (y_gain & 0x0100 ? 0x80 : 0);
	buf[4] = z_off & 0xff;
	buf[5] = ((z_off >> 8) & 0x3f) | (z_gain & 0x0100 ? 0x80 : 0);


	mc3230_write_block(client, 0x21, buf, 6);

	//buf[0]=0x41;
	//mc3230_write_block(client, 0x07, buf, 1);	

	mc3230_write_reg(client,0x07,0x41);

    return err;

}
/*
static int mc3230_trans_buff(char *rbuf, int size)
{
	//wait_event_interruptible_timeout(data_ready_wq,
	//				 atomic_read(&data_ready), 1000);
	wait_event_interruptible(data_ready_wq,
					 atomic_read(&data_ready));

	atomic_set(&data_ready, 0);
	memcpy(rbuf, &sense_data[0], size);

	return 0;
}
*/

static int mc3230_get_cached_data(struct i2c_client* client, struct mc3230_axis* sense_data)
{
    struct mc3230_data* this = (struct mc3230_data *)i2c_get_clientdata(client);
    printk("mc3230_get_cached_data\n");
    wait_event_interruptible_timeout(this->data_ready_wq, 
                                     atomic_read(&(this->data_ready) ),
                                     msecs_to_jiffies(1000) );
    if ( 0 == atomic_read(&(this->data_ready) ) ) {
        printk("waiting 'data_ready_wq' timed out.");
        return -1;
    }
    mutex_lock(&(this->sense_data_mutex) );
    *sense_data = this->sense_data;
    mutex_unlock(&(this->sense_data_mutex) );
    return 0;
}

static int mc3230_open(struct inode *inode, struct file *file)
{
	return 0;//nonseekable_open(inode, file);
}

static int mc3230_release(struct inode *inode, struct file *file)
{
	return 0;
}
static int MC32X0_ReadData(struct i2c_client *client, s16 buffer[MC32X0_AXES_NUM])
{
	s8 buf[3];
	char rbm_buf[6];
	int ret;
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
		return err;
	}

	if ( enable_RBM_calibration == 0)
	{
		//err = hwmsen_read_block(client, addr, buf, 0x06);
	}
	else if (enable_RBM_calibration == 1)
	{		
		memset(rbm_buf, 0, 3);
        	rbm_buf[0] = MC3230_REG_RBM_DATA;
        	ret = mc3230_rx_data(client, &rbm_buf[0], 6);
	}

	if ( enable_RBM_calibration == 0)
	{
	    do {
	        memset(buf, 0, 3);
	        buf[0] = MC3230_REG_X_OUT;
	        ret = mc3230_rx_data(client, &buf[0], 3);
	        if (ret < 0)
	            return ret;
	    	} while (0);
		
		buffer[0]=(s16)buf[0];
		buffer[1]=(s16)buf[1];
		buffer[2]=(s16)buf[2];
		
		mcprintkreg("0x%02x 0x%02x 0x%02x \n",buffer[0],buffer[1],buffer[2]);
	}
	else if (enable_RBM_calibration == 1)
	{
		buffer[MC32X0_AXIS_X] = (s16)((rbm_buf[0]) | (rbm_buf[1] << 8));
		buffer[MC32X0_AXIS_Y] = (s16)((rbm_buf[2]) | (rbm_buf[3] << 8));
		buffer[MC32X0_AXIS_Z] = (s16)((rbm_buf[4]) | (rbm_buf[5] << 8));

		GSE_LOG("%s RBM<<<<<[%08d %08d %08d]\n", __func__,buffer[MC32X0_AXIS_X], buffer[MC32X0_AXIS_Y], buffer[MC32X0_AXIS_Z]);
if(gain_data[0] == 0)
{
		buffer[MC32X0_AXIS_X] = 0;
		buffer[MC32X0_AXIS_Y] = 0;
		buffer[MC32X0_AXIS_Z] = 0;
	return 0;
}
		buffer[MC32X0_AXIS_X] = (buffer[MC32X0_AXIS_X] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
		buffer[MC32X0_AXIS_Y] = (buffer[MC32X0_AXIS_Y] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
		buffer[MC32X0_AXIS_Z] = (buffer[MC32X0_AXIS_Z] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];
		
		GSE_LOG("%s offset_data <<<<<[%d %d %d]\n", __func__,offset_data[0], offset_data[1], offset_data[2]);

		GSE_LOG("%s gsensor_gain <<<<<[%d %d %d]\n", __func__,gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
		
		GSE_LOG("%s gain_data <<<<<[%d %d %d]\n", __func__,gain_data[0], gain_data[1], gain_data[2]);

		GSE_LOG("%s RBM->RAW <<<<<[%d %d %d]\n", __func__,buffer[MC32X0_AXIS_X], buffer[MC32X0_AXIS_Y], buffer[MC32X0_AXIS_Z]);
	}
	
	return 0;
}
static int MC32X0_ReadRawData(struct i2c_client *client, char * buf)
{
	struct mc3230_data *obj = (struct mc3230_data*)i2c_get_clientdata(client);
	int res = 0;
	s16 raw_buf[3];

	if (!buf || !client)
	{
		return EINVAL;
	}

	if(obj->status == MC3230_CLOSE)
	{
		res = mc3230_start(client, 0);
		if(res)
		{
			GSE_ERR("Power on mc32x0 error %d!\n", res);
		}
	}
	
	if(res = MC32X0_ReadData(client, &raw_buf[0]))
	{     
	printk("%s %d\n",__FUNCTION__, __LINE__);
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
	
	GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", 
	raw_buf[MC32X0_AXIS_X], raw_buf[MC32X0_AXIS_Y], raw_buf[MC32X0_AXIS_Z]);

	G_RAW_DATA[MC32X0_AXIS_X] = raw_buf[0];
	G_RAW_DATA[MC32X0_AXIS_Y] = raw_buf[1];
	G_RAW_DATA[MC32X0_AXIS_Z] = raw_buf[2];
	G_RAW_DATA[MC32X0_AXIS_Z]= G_RAW_DATA[MC32X0_AXIS_Z]+gsensor_gain.z;
	
	//printk("%s %d\n",__FUNCTION__, __LINE__);
		sprintf(buf, "%04x %04x %04x", G_RAW_DATA[MC32X0_AXIS_X], 
			G_RAW_DATA[MC32X0_AXIS_Y], G_RAW_DATA[MC32X0_AXIS_Z]);
		GSE_LOG("G_RAW_DATA: (%+3d %+3d %+3d)\n", 
	G_RAW_DATA[MC32X0_AXIS_X], G_RAW_DATA[MC32X0_AXIS_Y], G_RAW_DATA[MC32X0_AXIS_Z]);
	}
	return 0;
}

static long mc3230_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{

	void __user *argp = (void __user *)arg;
		
	char strbuf[256];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];
	
	// char msg[RBUFF_SIZE + 1];
    struct mc3230_axis sense_data = {0};
	int ret = -1;
	char rate;
	struct i2c_client *client = container_of(mc3230_device.parent, struct i2c_client, dev);
    struct mc3230_data* this = (struct mc3230_data *)i2c_get_clientdata(client);  /* 设备数据实例的指针. */

	switch (cmd) {
	case MC_IOCTL_APP_SET_RATE:
		if (copy_from_user(&rate, argp, sizeof(rate)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case MC_IOCTL_START:
        mutex_lock(&(this->operation_mutex) );
        mcprintkreg("to perform 'MMA_IOCTL_START', former 'start_count' is %d.", this->start_count);
        (this->start_count)++;
        if ( 1 == this->start_count ) {
            atomic_set(&(this->data_ready), 0);
            if ( (ret = mc3230_start(client, this->curr_rate) < 0 )) {
                mutex_unlock(&(this->operation_mutex) );
                return ret;
            }
        }
        mutex_unlock(&(this->operation_mutex) );
        mcprintkreg("finish 'MMA_IOCTL_START', ret = %d.", ret);
        return 0;

	case MC_IOCTL_CLOSE:
        mutex_lock(&(this->operation_mutex) );
        mcprintkreg("to perform 'MMA_IOCTL_CLOSE', former 'start_count' is %d, PID : %d", this->start_count, get_current()->pid);
        if ( 0 == (--(this->start_count) ) ) {
            atomic_set(&(this->data_ready), 0);
            if ( (ret = mc3230_close(client) ) < 0 ) {
                mutex_unlock(&(this->operation_mutex) );
                return ret;
            }
        }
        mutex_unlock(&(this->operation_mutex) );
        return 0;

	case MC_IOCTL_APP_SET_RATE:
		ret = mc3230_reset_rate(client, rate);
		if (ret < 0)
			return ret;
		break;		
	case MC_IOCTL_GETDATA:
		//ret = mc3230_trans_buff(msg, RBUFF_SIZE);
		if ( (ret = mc3230_get_cached_data(client, &sense_data) ) < 0 ) {
            printk("failed to get cached sense data, ret = %d.", ret);
			return ret;
        }
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:	
	case GSENSOR_IOCTL_READ_RAW_DATA:
		GSE_LOG("fwq GSENSOR_IOCTL_READ_RAW_DATA\n");

		MC32X0_ReadRawData(client, &strbuf);
		
		break;
	case GSENSOR_IOCTL_SET_CALI:
		
			GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!\n");

			break;

	case GSENSOR_MCUBE_IOCTL_SET_CALI:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_CALI!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			//if(atomic_read(&this->suspend))
			//{
			//	GSE_ERR("Perform calibration in suspend state!!\n");
			//	err = -EINVAL;
			//}
			else
			{
				//this->cali_sw[MC32X0_AXIS_X] += sensor_data.x;
				//this->cali_sw[MC32X0_AXIS_Y] += sensor_data.y;
				//this->cali_sw[MC32X0_AXIS_Z] += sensor_data.z;
				
				cali[MC32X0_AXIS_X] = sensor_data.x;
				cali[MC32X0_AXIS_Y] = sensor_data.y;
				cali[MC32X0_AXIS_Z] = sensor_data.z;	

			  	GSE_LOG("GSENSOR_MCUBE_IOCTL_SET_CALI %d  %d  %d  %d  %d  %d!!\n", cali[MC32X0_AXIS_X], cali[MC32X0_AXIS_Y],cali[MC32X0_AXIS_Z] ,sensor_data.x, sensor_data.y ,sensor_data.z);
				
				err = MC32X0_WriteCalibration(client, cali);			 
			}
			break;
		case GSENSOR_IOCTL_CLR_CALI:
			GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
			err = MC32X0_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			GSE_LOG("fwq mc32x0 GSENSOR_IOCTL_GET_CALI\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = MC32X0_ReadCalibration(client, cali)))
			{
				GSE_LOG("fwq mc32x0 MC32X0_ReadCalibration error!!!!\n");
				break;
			}

			sensor_data.x = this->cali_sw[MC32X0_AXIS_X];
			sensor_data.y = this->cali_sw[MC32X0_AXIS_Y];
			sensor_data.z = this->cali_sw[MC32X0_AXIS_Z];
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;	
		// add by liang ****
		//add in Sensors_io.h
		//#define GSENSOR_IOCTL_SET_CALI_MODE   _IOW(GSENSOR, 0x0e, int)
		case GSENSOR_IOCTL_SET_CALI_MODE:
			GSE_LOG("fwq mc32x0 GSENSOR_IOCTL_SET_CALI_MODE\n");
			break;

		case GSENSOR_MCUBE_IOCTL_READ_RBM_DATA:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_RBM_DATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MC32X0_ReadRBMData(client, (char *)&strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;

		case GSENSOR_MCUBE_IOCTL_SET_RBM_MODE:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_RBM_MODE\n");

			MC32X0_rbm(client,1);

			break;

		case GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_RBM_MODE\n");

			MC32X0_rbm(client,0);

			break;

		case GSENSOR_MCUBE_IOCTL_REGISTER_MAP:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_REGISTER_MAP\n");

			//MC32X0_Read_Reg_Map(client);

			break;
			
	default:
		return -ENOTTY;
	}

	switch (cmd) {

	case MC_IOCTL_GETDATA:
        /*
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
        */
        if ( copy_to_user(argp, &sense_data, sizeof(sense_data) ) ) {
            printk("failed to copy sense data to user space.");
			return -EFAULT;
        }
		break;
	case GSENSOR_IOCTL_READ_RAW_DATA:
	case GSENSOR_IOCTL_READ_SENSORDATA:
		if (copy_to_user(argp, &strbuf, strlen(strbuf)+1)) {
			printk("failed to copy sense data to user space.");
			return -EFAULT;
		}
		
		break;

	default:
		break;
	}

	return 0;
}

static void mc3230_work_func(struct work_struct *work)
{
	struct mc3230_data *mc3230 = container_of(work, struct mc3230_data, work);
	struct i2c_client *client = mc3230->client;
	
	if (mc3230_get_data(client) < 0) 
		mcprintkreg(KERN_ERR "MC3230 mma_work_func: Get data failed\n");
		
	enable_irq(client->irq);		
}

static void  mc3230_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct mc3230_data *mc3230 = container_of(delaywork, struct mc3230_data, delaywork);
	struct i2c_client *client = mc3230->client;

	if (mc3230_get_data(client) < 0) 
		printk(KERN_ERR "MC3230 mma_work_func: Get data failed\n");
	mcprintkreg("%s \n",__FUNCTION__);	
	enable_irq(client->irq);		
}

static irqreturn_t mc3230_interrupt(int irq, void *dev_id)
{
	struct mc3230_data *mc3230 = (struct mc3230_data *)dev_id;
	
	disable_irq_nosync(irq);
	schedule_delayed_work(&mc3230->delaywork, msecs_to_jiffies(30));
	mcprintkfunc("%s :enter\n",__FUNCTION__);	
	return IRQ_HANDLED;
}

static struct file_operations mc3230_fops = {
	.owner = THIS_MODULE,
	.open = mc3230_open,
	.release = mc3230_release,
	.unlocked_ioctl = mc3230_ioctl,
};

static struct miscdevice mc3230_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma8452_daemon",
	.fops = &mc3230_fops,
};


static int mc3230_remove(struct i2c_client *client)
{
	struct mc3230_data *mc3230 = i2c_get_clientdata(client);
	
    misc_deregister(&mc3230_device);
    input_unregister_device(mc3230->input_dev);
    input_free_device(mc3230->input_dev);
    free_irq(client->irq, mc3230);
    kfree(mc3230); 
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&mc3230_early_suspend);
#endif      
    this_client = NULL;
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3230_suspend(struct early_suspend *h)
{
#if 0
	struct i2c_client *client = container_of(mc3230_device.parent, struct i2c_client, dev);
	struct mc3230_data *mc3230 = (struct mc3230_data *)i2c_get_clientdata(client);
	mcprintkreg("Gsensor mc3230 enter suspend mc3230->status %d\n",mc3230->status);
	if(mc3230->status == MC3230_OPEN)
	{
		mc3230_close(client);
	}
#endif
}

static void mc3230_resume(struct early_suspend *h)
{
#if 0
	struct i2c_client *client = container_of(mc3230_device.parent, struct i2c_client, dev);
   	struct mc3230_data *mc3230 = (struct mc3230_data *)i2c_get_clientdata(client);
	mcprintkreg("Gsensor mc3230 resume!! mc3230->status %d\n",mc3230->status);
	if (mc3230->status == MC3230_CLOSE)
	{
		mc3230_start(client,mc3230->curr_rate);
	}
#endif
}
#else
static int mc3230_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	return ret;
}
static int mc3230_resume(struct i2c_client *client)
{
	int ret;
	return ret;
}
#endif

static const struct i2c_device_id mc3230_id[] = {
		{"gs_mc3230", 0},
		{ }
};

static struct i2c_driver mc3230_driver = {
	.driver = {
		.name = "gs_mc3230",
	    },
	.id_table 	= mc3230_id,
	.probe		= mc3230_probe,           
	.remove		= __devexit_p(mc3230_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND	
	.suspend = &mc3230_suspend,
	.resume = &mc3230_resume,
#endif	
};


static int mc3230_init_client(struct i2c_client *client)
{
	struct mc3230_data *mc3230;
	int ret,irq;
	mc3230 = i2c_get_clientdata(client);
	//printk("gpio_to_irq(%d) is %d\n",client->irq,gpio_to_irq(client->irq));
	if ( !gpio_is_valid(client->irq)) {
		printk("+++++++++++gpio_is_invalid\n");
		return -EINVAL;
	}
	ret = gpio_request(client->irq, "mc3230_int");
	if (ret) {
		printk( "failed to request mma7990_trig GPIO%d\n",gpio_to_irq(client->irq));
		return ret;
	}
    ret = gpio_direction_input(client->irq);
    if (ret) {
        printk("failed to set mma7990_trig GPIO gpio input\n");
		gpio_free(client->irq);
		return ret;
    }
	gpio_pull_updown(client->irq, GPIOPullUp);
	irq = gpio_to_irq(client->irq);
	ret = request_irq(irq, mc3230_interrupt, IRQF_TRIGGER_LOW, client->dev.driver->name, mc3230);
	//printk("request irq is %d,ret is  0x%x\n",irq,ret);
	if (ret ) {
		gpio_free(client->irq);
		printk(KERN_ERR "mc3230_init_client: request irq failed,ret is %d\n",ret);
        return ret;
	}
	client->irq = irq;
	disable_irq(client->irq);
	init_waitqueue_head(&data_ready_wq);
 
	return 0;
}

static int  mc3230_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mc3230_data *mc3230;
	struct mc3230_platform_data *pdata = pdata = client->dev.platform_data;
	int err;
	char devid;
	s16 tmp, x_gain, y_gain, z_gain ;
	s32 x_off, y_off, z_off;
	
	load_cali_flg = FIRST_TRY_TIME;
	
	mcprintkfunc("%s enter\n",__FUNCTION__);

	mc3230 = kzalloc(sizeof(struct mc3230_data), GFP_KERNEL);
	if (!mc3230) {
		printk("[mc3230]:alloc data failed.\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
    
	INIT_WORK(&mc3230->work, mc3230_work_func);
	INIT_DELAYED_WORK(&mc3230->delaywork, mc3230_delaywork_func);

	memset(&(mc3230->sense_data), 0, sizeof(struct mc3230_axis) );
	mutex_init(&(mc3230->sense_data_mutex) );

	atomic_set(&(mc3230->data_ready), 0);
	init_waitqueue_head(&(mc3230->data_ready_wq) );

	mc3230->start_count = 0;
	mutex_init(&(mc3230->operation_mutex) );
	mc3230->curr_rate = MC3230_RATE_16;
	mc3230->status = MC3230_CLOSE;

	mc3230->client = client;
	i2c_set_clientdata(client, mc3230);

	this_client = client;

	mc3230->cvt.sign[MC32X0_AXIS_X] = 1;
	mc3230->cvt.sign[MC32X0_AXIS_Y] = 1;
	mc3230->cvt.sign[MC32X0_AXIS_Z] = 1;
	mc3230->cvt.map[MC32X0_AXIS_X]= 0;
	mc3230->cvt.map[MC32X0_AXIS_Y]= 1;
	mc3230->cvt.map[MC32X0_AXIS_Z]= 2;
	/*
	// add by Liang for reset sensor: Fix software system reset issue!!!!!!!!!
	unsigned char buf[2];
	buf[0]=0x43;
	mc3230_write_block(client, 0x07, buf, 1);	

	buf[0]=0x80;
	mc3230_write_block(client, 0x1C, buf, 1);	
	buf[0]=0x80;
	mc3230_write_block(client, 0x17, buf, 1);	
	msleep(5);
	
	buf[0]=0x00;
	mc3230_write_block(client, 0x1C, buf, 1);	
	buf[0]=0x00;
	mc3230_write_block(client, 0x17, buf, 1);	
	*/
	mc3230_write_reg(client,0x1b,0x6d);
	mc3230_write_reg(client,0x1b,0x43);
	msleep(5);
	
	mc3230_write_reg(client,0x07,0x43);
	mc3230_write_reg(client,0x1C,0x80);
	mc3230_write_reg(client,0x17,0x80);
	msleep(5);
	mc3230_write_reg(client,0x1C,0x00);
	mc3230_write_reg(client,0x17,0x00);
	msleep(5);


/*
	if ((err = mc3230_read_block(new_client, 0x21, offset_buf, 6))) //add by Liang for storeage OTP offsef register value
	{
		GSE_ERR("error: %d\n", err);
		return err;
	}
*/
	memset(offset_buf, 0, 9);
	offset_buf[0] = 0x21;
	err = mc3230_rx_data(client, offset_buf, 9);
	if(err)
	{
		GSE_ERR("error: %d\n", err);
		return err;
	}

	tmp = ((offset_buf[1] & 0x3f) << 8) + offset_buf[0];
		if (tmp & 0x2000)
			tmp |= 0xc000;
		x_off = tmp;
					
	tmp = ((offset_buf[3] & 0x3f) << 8) + offset_buf[2];
		if (tmp & 0x2000)
			tmp |= 0xc000;
		y_off = tmp;
					
	tmp = ((offset_buf[5] & 0x3f) << 8) + offset_buf[4];
		if (tmp & 0x2000)
			tmp |= 0xc000;
		z_off = tmp;
					
	// get x,y,z gain
	x_gain = ((offset_buf[1] >> 7) << 8) + offset_buf[6];
	y_gain = ((offset_buf[3] >> 7) << 8) + offset_buf[7];
	z_gain = ((offset_buf[5] >> 7) << 8) + offset_buf[8];
							

	//storege the cerrunt offset data with DOT format
	offset_data[0] = x_off;
	offset_data[1] = y_off;
	offset_data[2] = z_off;

	//storege the cerrunt Gain data with GOT format
	gain_data[0] = 256*8*128/3/(40+x_gain);
	gain_data[1] = 256*8*128/3/(40+y_gain);
	gain_data[2] = 256*8*128/3/(40+z_gain);
	printk("offser gain = %d %d %d %d %d %d======================\n\n ",
		gain_data[0],gain_data[1],gain_data[2],offset_data[0],offset_data[1],offset_data[2]);

	devid = mc3230_get_devid(this_client);

	if (MC3230_DEVID != devid) {
		pr_info("mc3230: invalid devid\n");
		goto exit_invalid_devid;
	}
	err = mc3230_init_client(client);
	if (err < 0) {
		printk(KERN_ERR
		       "mc3230_probe: mc3230_init_client failed\n");
		goto exit_request_gpio_irq_failed;
	}

	mc3230->input_dev = input_allocate_device();
	if (!mc3230->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "mc3230_probe: Failed to allocate input device\n");
		goto exit_input_allocate_device_failed;
	}

	set_bit(EV_ABS, mc3230->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(mc3230->input_dev, ABS_X, -MC3230_RANGE, MC3230_RANGE, 0, 0); //2g full scale range
	/* y-axis acceleration */
	input_set_abs_params(mc3230->input_dev, ABS_Y, -MC3230_RANGE, MC3230_RANGE, 0, 0); //2g full scale range
	/* z-axis acceleration */
	input_set_abs_params(mc3230->input_dev, ABS_Z, -MC3230_RANGE, MC3230_RANGE, 0, 0); //2g full scale range

	// mc3230->input_dev->name = "compass";
	mc3230->input_dev->name = "gsensor";
	mc3230->input_dev->dev.parent = &client->dev;

	err = input_register_device(mc3230->input_dev);
	if (err < 0) {
		printk(KERN_ERR
		       "mc3230_probe: Unable to register input device: %s\n",
		       mc3230->input_dev->name);
		goto exit_input_register_device_failed;
	}

    mc3230_device.parent = &client->dev;
	err = misc_register(&mc3230_device);
	if (err < 0) {
		printk(KERN_ERR
		       "mc3230_probe: mmad_device register failed\n");
		goto exit_misc_device_register_mc3230_device_failed;
	}

	err = gsensor_sysfs_init();
	if (err < 0) {
		printk(KERN_ERR
            "mc3230_probe: gsensor sysfs init failed\n");
		goto exit_gsensor_sysfs_init_failed;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    mc3230_early_suspend.suspend = mc3230_suspend;
    mc3230_early_suspend.resume = mc3230_resume;
    mc3230_early_suspend.level = 0x2;
    register_early_suspend(&mc3230_early_suspend);
#endif
   mc3230_reg_init(this_client);
   printk(KERN_INFO "mc3230 probe ok\n");
#if  0	
//	mc3230_start(client, MC3230_RATE_12P5);
#endif
	return 0;

exit_gsensor_sysfs_init_failed:
    misc_deregister(&mc3230_device);
exit_misc_device_register_mc3230_device_failed:
    input_unregister_device(mc3230->input_dev);
exit_input_register_device_failed:
	input_free_device(mc3230->input_dev);
exit_input_allocate_device_failed:
	free_irq(client->irq, mc3230);
exit_request_gpio_irq_failed:
	cancel_delayed_work_sync(&mc3230->delaywork);
	cancel_work_sync(&mc3230->work);
exit_invalid_devid:
	kfree(mc3230);	
exit_alloc_data_failed:
    ;
	printk("%s error\n",__FUNCTION__);
	return -1;
}


static int __init mc3230_i2c_init(void)
{
	return i2c_add_driver(&mc3230_driver);
}

static void __exit mc3230_i2c_exit(void)
{
	i2c_del_driver(&mc3230_driver);
}

module_init(mc3230_i2c_init);
module_exit(mc3230_i2c_exit);

