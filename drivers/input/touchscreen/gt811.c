/* drivers/input/touchscreen/gt811.c
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 *Any problem,please contact andrew@goodix.com,+86 755-33338828
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/proc_fs.h>
#include "gt811.h"
#include "gt811_firmware.h"

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <mach/board.h>

static struct workqueue_struct *goodix_wq;
static const char *gt811_ts_name = "Goodix Capacitive TouchScreen";
//static struct point_queue finger_list;
struct i2c_client * i2c_connect_client = NULL;
//EXPORT_SYMBOL(i2c_connect_client);
static struct proc_dir_entry *goodix_proc_entry;
static short goodix_read_version(struct goodix_ts_data *ts);
//static int tpd_button(struct goodix_ts_data *ts, unsigned int x, unsigned int y, unsigned int down);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
int gt811_downloader( struct goodix_ts_data *ts, unsigned char * data);
#endif
//used by firmware update CRC
unsigned int oldcrc32 = 0xFFFFFFFF;
unsigned int crc32_table[256];
unsigned int ulPolynomial = 0x04c11db7;

unsigned int raw_data_ready = RAW_DATA_NON_ACTIVE;

#ifdef DEBUG
int sum = 0;
int access_count = 0;
int int_count = 0;
#endif

int isGt801 = 0;

/*******************************************************	
 Function:
 Read data from the slave
 Each read operation with two i2c_msg composition, for the first message sent from the machine address,
 Article 2 reads the address used to send and retrieve data; each message sent before the start signal
 Parameters:
 client: i2c devices, including device address
 buf [0]: The first byte to read Address
 buf [1] ~ buf [len]: data buffer
 len: the length of read data
 return:
 Execution messages
 *********************************************************/
/*Function as i2c_master_send */
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = 2 - isGt801;		//gt801时，msgs.len = 1,否则为2
	msgs[0].buf = &buf[0];
	msgs[0].scl_rate = GT811_I2C_SCL;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - 2;
	msgs[1].buf = &buf[2];
	msgs[1].scl_rate = GT811_I2C_SCL;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret;
}

/*******************************************************	
 Function:
 Write data to a slave
 Parameters:
 client: i2c devices, including device address
 buf [0]: The first byte of the write address
 buf [1] ~ buf [len]: data buffer
 len: data length
 return:
 Execution messages
 *******************************************************/
/*Function as i2c_master_send */
static int i2c_write_bytes(struct i2c_client *client, uint8_t *data, int len)
{
	struct i2c_msg msg;
	int ret = -1;
	//发送设备地址
	msg.flags = !I2C_M_RD; //写消息
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;
	msg.scl_rate = GT811_I2C_SCL;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret;
}

/*******************************************************
 Function:
 Send a prefix command

 Parameters:
 ts: client private data structure

 return:
 Results of the implementation code, 0 for normal execution
 *******************************************************/
static int i2c_pre_cmd(struct goodix_ts_data *ts)
{
	int ret;
	uint8_t pre_cmd_data[2] =
	{ 0 };
	pre_cmd_data[0] = 0x0f;
	pre_cmd_data[1] = 0xff;
	ret = i2c_write_bytes(ts->client, pre_cmd_data, 2);
	//msleep(2);
	return ret;
}

/*******************************************************
 Function:
 Send a suffix command

 Parameters:
 ts: client private data structure

 return:
 Results of the implementation code, 0 for normal execution
 *******************************************************/
static int i2c_end_cmd(struct goodix_ts_data *ts)
{
	int ret;
	uint8_t end_cmd_data[2] =
	{ 0 };
	end_cmd_data[0] = 0x80;
	end_cmd_data[1] = 0x00;
	ret = i2c_write_bytes(ts->client, end_cmd_data, 2);
	//msleep(2);
	return ret;
}

/********************************************************************

 *********************************************************************/
#ifdef COOR_TO_KEY
static int list_key(s32 x_value, s32 y_value, u8* key)
{
	s32 i;

#ifdef AREA_Y
	if (y_value <= AREA_Y)
#else
	if (x_value <= AREA_X)
#endif
	{
		return 0;
	}

	for (i = 0; i < MAX_KEY_NUM; i++)
	{
		if (abs(key_center[i][x] - x_value) < KEY_X
				&& abs(key_center[i][y] - y_value) < KEY_Y)
		{
			(*key) |= (0x01<<i);
		}
	}

	return 1;
}
#endif 
static int goodix_init_panel_gt801(struct goodix_ts_data *ts)
{
	int ret=-1;
	//瑞深思内嵌
	u8 config_info[] = 	{
			0x30,0x19,0x05,0x03,0x28,0x02,0x14,0x14,0x10,0x50,
			0xB8,0x14,0x00,0x1E,0x00,0xED,0xCB,0xA9,0x87,0x65,
			0x43,0x21,0x01,0x00,0x00,0x00,0x00,0x4D,0xC1,0x20,
			0x01,0x01,0x83,0x50,0x3C,0x1E,0xB4,0x00,0x0A,0x50,
			0x82,0x1E,0x00,0x6E,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x01,
		};

	/*
	//多点平波内嵌
	u8 config_info[] = 	{
			0x30,0x14,0x05,0x07,0x28,0x02,0x14,0x14,0x10,0x2d,
			0xba,0x14,0x00,0x1e,0x00,0x01,0x23,0x45,0x67,0x89,
			0xab,0xcd,0xe1,0x00,0x00,0x36,0x2e,0x4f,0xcf,0x20,
			0x01,0x01,0x81,0x64,0x3c,0x1e,0x28,0x00,0x34,0x2d,
			0x01,0xf7,0x50,0x3c,0x32,0x71,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x01,
		};
*/
	config_info[11] = (TOUCH_MAX_HEIGHT >> 8) & 0xFF;
	config_info[12] = TOUCH_MAX_HEIGHT & 0xFF;
	config_info[13] = (TOUCH_MAX_WIDTH >> 8) & 0xFF;
	config_info[14] = TOUCH_MAX_WIDTH  & 0xFF;

	ret=i2c_write_bytes(ts->client,config_info,ARRAY_SIZE(config_info));

	if (ret < 0)
		goto error_i2c_transfer;
	msleep(30);
	return 0;

error_i2c_transfer:
	return ret;
}
/*******************************************************
 Function:
 Guitar initialization function, used to send configuration information, access to version information
 Parameters:
 ts: client private data structure
 return:
 Results of the implementation code, 0 for normal execution
 *******************************************************/
static int goodix_init_panel(struct goodix_ts_data *ts)
{
	if(isGt801 == 1)
	{
		ts->int_trigger_type = 0;
		return goodix_init_panel_gt801(ts);
	}

	short ret = -1;
	struct gt811_platform_data *pdata = ts->client->dev.platform_data;
	u8 *config_info = (u8 *) pdata->config_info;

	config_info[57] &= 0xF7;
	ret = i2c_write_bytes(ts->client, config_info, pdata->config_info_len);
	if (ret < 0)
	{
		dev_info(&ts->client->dev, "GT811 Send config failed!\n");
		return ret;
	}
	ts->abs_x_max = (config_info[62] << 8) + config_info[61];
	ts->abs_y_max = (config_info[64] << 8) + config_info[63];
	ts->max_touch_num = config_info[60];
	ts->int_trigger_type = ((config_info[57] >> 3) & 0x01);
	dev_info(
			&ts->client->dev,
			"GT811 init info:X_MAX=%d,Y_MAX=%d,TRIG_MODE=%s\n",
			ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type ? "RISING EDGE" : "FALLING EDGE");
	msleep(10);
	return 0;
}

/*******************************************************
 FUNCTION:
 Read gt811 IC Version
 Argument:
 ts:	client
 return:
 0:success
 -1:error
 *******************************************************/
static int goodix_read_chiptype(struct goodix_ts_data *ts)
{
	short ret;
	uint8_t version_data[4] =
	{ 0x40, 0x11, 0, 0 }; //store touchscreen version infomation

	ret = i2c_read_bytes(ts->client, version_data, 3);
	if (ret < 0)
		return ret;

	printk("gt811 chiptype is %d \n", version_data[2]);
	return version_data[2];
}

static short goodix_read_version(struct goodix_ts_data *ts)
{
	short ret;
	uint8_t version_data[5] =
	{ 0x07, 0x17, 0, 0 }; //store touchscreen version infomation
	uint8_t version_data2[5] =
	{ 0x07, 0x17, 0, 0 }; //store touchscreen version infomation

	char i = 0;
	char cpf = 0;
	memset(version_data, 0, 5);
	version_data[0] = 0x07;
	version_data[1] = 0x17;

	ret = i2c_read_bytes(ts->client, version_data, 4);
	if (ret < 0)
		return ret;

	for (i = 0; i < 10; i++)
	{
		i2c_read_bytes(ts->client, version_data2, 4);
		if ((version_data[2] != version_data2[2])
				|| (version_data[3] != version_data2[3]))
		{
			version_data[2] = version_data2[2];
			version_data[3] = version_data2[3];
			msleep(5);
			break;
		}
		msleep(5);
		cpf++;
	}

	if (cpf == 10)
	{
		ts->version = (version_data[2] << 8) + version_data[3];
		dev_info(&ts->client->dev, "GT811 Verion:0x%04x\n", ts->version);
		ret = 0;
	}
	else
	{
		dev_info(&ts->client->dev, " Guitar Version Read Error: %d.%d\n",
				version_data[3], version_data[2]);
		ts->version = 0xffff;
		ret = -1;
	}

	return ret;

}
/******************start add by kuuga*******************/
static void gt811_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable)
	{
		enable_irq(ts->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

static void gt811_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable)
	{
		disable_irq_nosync(ts->irq);
		ts->irq_is_disable = 1;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

static void goodix_ts_work_func_gt818(struct work_struct *work)
{
	u8  touch_status[8*MAX_FINGER_NUM + 18] = {0x07, 0x12, 0};
	u8  *key_value = NULL;
	u8  *point_data = NULL;
	static u8 finger_last[MAX_FINGER_NUM + 1]={0};
	u8  finger_current[MAX_FINGER_NUM + 1] = {0};
	u8  coor_data[6*MAX_FINGER_NUM] = {0};
	static u8  last_key = 0;

	u8  finger = 0;
	u8  key = 0;
	unsigned int  count = 0;
	unsigned int position = 0;
	int temp = 0;
	int x = 0, y = 0 , pressure;

	u16 *coor_point;

	int syn_flag = 0;

	struct goodix_ts_data *ts = container_of(work, struct goodix_ts_data, work);
	struct gt811_platform_data *pdata;
	pdata = ts->client->dev.platform_data;

	i2c_pre_cmd(ts);
	i2c_read_bytes(ts->client, touch_status, sizeof(touch_status)/sizeof(touch_status[0]));
	i2c_end_cmd(ts);

	//judge whether the data is ready
	if((touch_status[2] & 0x30) != 0x20)
	{
		printk("%s:DATA_NO_READY\n", __func__);
		goto DATA_NO_READY;
	}
	//judge whether it is large area touch
	if(touch_status[13] & 0x0f)
	{
		goto DATA_NO_READY;
	}

	ts->bad_data = 0;
	finger = touch_status[2] & 0x07;
	key_value = touch_status + 15;
	key = key_value[2] & 0x0f;

	if(finger > 0)
	{
		point_data = key_value + 3;

		for(position = 0; position < (finger*8); position += 8)
		{
			temp = point_data[position];
			//printk("track:%d\n", temp);
			if(temp < (MAX_FINGER_NUM + 1))
			{
				finger_current[temp] = 1;
				for(count = 0; count < 6; count++)
				{
					coor_data[(temp - 1) * 6 + count] = point_data[position+1+count];
				}
			}
			else
			{
				//dev_err(&(ts->client->dev),"Track Id error:%d\n ",);
				ts->bad_data = 1;
				ts->retry++;
				goto XFER_ERROR;
			}
		}
	}
	else
	{
		for(position = 1; position < MAX_FINGER_NUM+1; position++)
		{
			finger_current[position] = 0;
		}
	}

	coor_point = (u16 *)coor_data;

	for(position = 1; position < MAX_FINGER_NUM + 1; position++)
	{
		if((finger_current[position] == 0) && (finger_last[position] != 0))
		{
			input_mt_slot(ts->input_dev, position-1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			syn_flag = 1;
		}
		else if(finger_current[position])
		{
			x = (*(coor_point+3*(position-1)));//*SCREEN_MAX_WIDTH/(TOUCH_MAX_WIDTH);
			y = (*(coor_point+3*(position-1)+1));//*SCREEN_MAX_HEIGHT/(TOUCH_MAX_HEIGHT);
			pressure = (*(coor_point+3*(position-1)+2));

			if (pdata->swap_xy)
				swap(x, y);

/////////修改代码////////////////////////////////
			#define DE 10
			if(x > 780)
				x = x - DE;
			if(x < 20)
				x = x + DE;
			if(y > 460)
				y = y - DE;
			if(y < 20)
				y = y + DE;

			//printk(">>>>>>>>>>>>>>>>>%s:positon:%d (%d,%d)\n", __func__, position,x,y);
			input_mt_slot(ts->input_dev, position-1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

			//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure);
			//input_mt_sync(ts->input_dev);
			syn_flag = 1;
		}
	}
input_sync(ts->input_dev);

NO_KEY_PRESS:
	for(position = 1; position < MAX_FINGER_NUM + 1; position++)
	{
		finger_last[position] = finger_current[position];
	}

DATA_NO_READY:
XFER_ERROR:
#ifndef STOP_IRQ_TYPE
	if (ts->use_irq)
		gt811_irq_enable(ts); //KT ADD 1202
#endif

}

/*****************end add by kuuga****************/

/*******************************************************	
 Function:
 Touch-screen work function
 Triggered by the interruption, to accept a set of coordinate data,
 and then analyze the output parity
 Parameters:
 ts: client private data structure
 return:
 Results of the implementation code, 0 for normal execution
 ********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	uint8_t point_data[READ_BYTES_NUM] =
	{ READ_TOUCH_ADDR_H, READ_TOUCH_ADDR_L, 0 }; //point_data[8*MAX_FINGER_NUM+2]={ 0 };

	if(isGt801)
		point_data[0] = point_data[1] = 0;

	uint8_t check_sum = 0;
	uint8_t read_position = 0;

	uint8_t point_index = 0;
	uint8_t point_tmp = 0;
	uint8_t point_count = 0;
	uint8_t positionID = 0;

	uint16_t input_x = 0;
	uint16_t input_y = 0;
	uint8_t input_w = 0;
	unsigned int count = 0;
	unsigned int position = 0;

	int ret = -1;
	int tmp = 0;
	static int last_x = 0;
	static int last_y = 0;

	struct goodix_ts_data	*ts = container_of(work, struct goodix_ts_data, work);
	struct gt811_platform_data *pdata;
	pdata = ts->client->dev.platform_data;

#ifndef INT_PORT
	COORDINATE_POLL:
#endif
	if (tmp > 9)
	{
		dev_info(&(ts->client->dev),
				"Because of transfer error,touchscreen stop working.\n");
		goto XFER_ERROR;
	}

	ret = i2c_read_bytes(ts->client, point_data,
			sizeof(point_data) / sizeof(point_data[0]));
	if (ret <= 0)
	{
		dev_err(&(ts->client->dev), "I2C transfer error. Number:%d\n ", ret);
		ts->bad_data = 1;
		tmp++;
		ts->retry++;
#ifndef INT_PORT
		goto COORDINATE_POLL;
#else
		goto XFER_ERROR;
#endif
	}
#if 0
	for(count=0;count<(sizeof(point_data)/sizeof(point_data[0])); count++)
	{
		printk("[%2d]:0x%2x", count, point_data[count]);
		if((count+1)%10==0)printk("\n");
	}
	printk("\n");
#endif	

	if (point_data[2] & 0x20)
	{
		if (point_data[3] == 0xF0)
		{
			gpio_direction_output(SHUTDOWN_PORT, 0);
			msleep(1);
			gpio_direction_input(SHUTDOWN_PORT);
			goodix_init_panel(ts);
			goto WORK_FUNC_END;
		}
	}
#if 0
	switch(point_data[2]& 0x1f)
	{
		case 0:
		read_position = 3;
		break;
		case 1:
		for(count=2; count<9; count++)
		check_sum += (int)point_data[count];
		read_position = 9;
		break;
		case 2:
		case 3:
		for(count=2; count<14;count++)
		check_sum += (int)point_data[count];
		read_position = 14;
		break;
		default: //touch finger larger than 3
		for(count=2; count<35;count++)
		check_sum += (int)point_data[count];
		read_position = 35;
	}
	if(check_sum != point_data[read_position])
	{
		dev_info(&ts->client->dev, "coor chksum error!\n");
		goto XFER_ERROR;
	}
#endif

	point_index = point_data[2] & 0x1f;
	point_tmp = point_index;
	for (positionID = 0; positionID < MAX_FINGER_NUM/*&&point_tmp*/;positionID++, point_tmp >>= 1)
	{
		if ((point_tmp & 0x01) == 0)
		{
			if (positionID == 0)
				last_x = last_y = 0;
			input_mt_slot(ts->input_dev, positionID);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
//			track_id[point_count++] = position;
		}
		else
		{
			if (positionID != 3)
			{
				if (positionID < 3)
					position = 4 + positionID * 5;
				else
					position = 30;
				input_x = (uint16_t)(point_data[position] << 8)
						+ (uint16_t) point_data[position + 1];
				input_y = (uint16_t)(point_data[position + 2] << 8)
						+ (uint16_t) point_data[position + 3];
				input_w = point_data[position + 4];
			}
			else
			{
				input_x = (uint16_t)(point_data[19] << 8)
						+ (uint16_t) point_data[26];
				input_y = (uint16_t)(point_data[27] << 8)
						+ (uint16_t) point_data[28];
				input_w = point_data[29];
			}

			if (pdata->swap_xy)
				swap(input_x, input_y);

			if ((point_tmp == 0x01) && (positionID == 0) && (last_x != 0)
					&& (last_y != 0)) //针对单个手指触摸,做过滤
			{
				if ((abs(input_x - last_x) < 20)
						&& (abs(input_y - last_y) < 20))
				{
					input_x = last_x;
					input_x = last_y;
					break;
				}
			}

			if(ts->abs_x_max == 490)
			{
				if(input_y >= 20)
					input_y -= 10;
				else
					input_y = 10;
			}
			//智伟
			//if(input_y<30)  input_y =input_y+6;

			input_mt_slot(ts->input_dev, positionID);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
		}
	}
	input_sync(ts->input_dev);

	XFER_ERROR: WORK_FUNC_END:
#ifndef STOP_IRQ_TYPE
	if (ts->use_irq)
		gt811_irq_enable(ts); //KT ADD 1202
#endif
}

/*******************************************************	
 Function:
 Response function timer
 Triggered by a timer, scheduling the work function of the touch screen operation; after re-timing
 Parameters:
 timer: the timer function is associated
 return:
 Timer mode, HRTIMER_NORESTART that do not automatically restart
 ********************************************************/
static enum hrtimer_restart goodix_ts_timer_func(struct hrtimer *timer)
{
	struct goodix_ts_data
	*ts = container_of(timer, struct goodix_ts_data, timer);
	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME + 6) * 1000000),
			HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************	
 Function:
 Interrupt response function
 Triggered by an interrupt, the scheduler runs the touch screen handler
 ********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	//printk(KERN_INFO"-------------------ts_irq_handler------------------\n");
#ifndef STOP_IRQ_TYPE
	gt811_irq_disable(ts); //KT ADD 1202
#endif
	//disable_irq_nosync(ts->client->irq);
	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}

/*******************************************************	
 Function:
 Power management gt811, gt811 allowed to sleep or to wake up
 Parameters:
 on: 0 that enable sleep, wake up 1
 return:
 Is set successfully, 0 for success
 Error code: -1 for the i2c error, -2 for the GPIO error;-EINVAL on error as a parameter
 ********************************************************/

static int goodix_ts_power(struct goodix_ts_data * ts, int on)
{
	int ret = -1;

	unsigned char i2c_control_buf[3] =
	{ 0x06, 0x92, 0x01 }; //suspend cmd

#ifdef INT_PORT	
	if (ts != NULL && !ts->use_irq)
		return -2;
#endif		
	switch (on)
	{
	case 0:
		ret = i2c_write_bytes(ts->client, i2c_control_buf, 3);
		dev_info(&ts->client->dev, "Send suspend cmd\n");
		if (ret > 0) //failed
			ret = 0;
		return ret;

	case 1:
#ifdef INT_PORT	                     //suggest use INT PORT to wake up !!!
		gpio_direction_output(INT_PORT, 0);
		msleep(10);
		gpio_direction_output(INT_PORT, 1);
		msleep(10);
		gpio_direction_output(INT_PORT, 0);
		gpio_pull_updown(INT_PORT, 0);
		if (ts->use_irq)
			gpio_direction_input(INT_PORT);
		else
			gpio_direction_input(INT_PORT);

#else
		gpio_direction_output(SHUTDOWN_PORT, 0);
		msleep(10);
		gpio_direction_input(SHUTDOWN_PORT);
#endif
		msleep(40);
		ret = 0;
		return ret;

	default:
		/*			dev_info(&ts->client->dev, "%s: Cant't support this command.", s3c_ts_name);*/
		return -EINVAL;
	}

}
/*******************************************************	
 Function:
 Touch-screen detection function
 Called when the registration drive (required for a corresponding client);
 For IO, interrupts and other resources to apply; equipment registration; touch screen initialization, etc.
 Parameters:
 client: the device structure to be driven
 id: device ID
 return:
 Results of the implementation code, 0 for normal execution
 ********************************************************/
int gt811_probe_success = 0;
static int goodix_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	int retry = 0;
	char test_data = 1;
	const char irq_table[2] =
	{ IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_EDGE_RISING };
	struct goodix_ts_data *ts;

	struct gt811_platform_data *pdata;
	dev_info(&client->dev, "Install gt811 driver.\n");
	dev_info(&client->dev, "Driver Release Date:2012-02-08\n");

	pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	i2c_connect_client = client;

	ret = gpio_request(pdata->gpio_reset, "RESET_INT");
	if (ret < 0)
	{
		dev_err(&client->dev, "Failed to request RESET GPIO:%d, ERRNO:%d\n",
				(int) SHUTDOWN_PORT, ret);
		goto err_gpio_request;
	}
//	gpio_pull_updown(pdata->gpio_reset, 1);		//set GPIO pull-up

	for (retry = 0; retry <= 10; retry++)
	{
		client->addr = 0x5D;
		gpio_direction_output(pdata->gpio_reset, 1);
		msleep(20);
		gpio_direction_output(pdata->gpio_reset, 0);
		msleep(100);
		gpio_direction_output(pdata->gpio_reset, 1);
		msleep(200);

		ret = i2c_write_bytes(client, &test_data, 1); //Test I2C connection.
		if (ret > 0)
			break;
		client->addr = 0x55;
//		printk("err!!!\n");
		gpio_direction_output(pdata->gpio_reset, 0);
		msleep(100);
		ret = i2c_write_bytes(client, &test_data, 1); //Test I2C connection.
		if(ret > 0)
		{
			ret = 0x801;
			printk("801===============\n");
			break;
		}
		dev_info(&client->dev,
				"GT811 I2C TEST FAILED!Please check the HARDWARE connect\n");
		goto err_input_dev_alloc_failed;
	}

	gt811_probe_success = 1; 
	//init work_struct
	ts->client = client;
	i2c_set_clientdata(client, ts);

	if(ret != 0x801)
	{
		ret = goodix_read_chiptype(ts);
		if(ret > 0)
		{
			if(ret == 0x09)
			{
				printk("811=============\n");
				INIT_WORK(&ts->work, goodix_ts_work_func);
			}
			else
			{
				printk("818++++++++++++++\n");
				INIT_WORK(&ts->work, goodix_ts_work_func_gt818);
			}
		}
	}
	else
	{
		isGt801 = 1;
		INIT_WORK(&ts->work, goodix_ts_work_func);
	}

/////////////////////////////// UPDATE STEP 1 START/////////////////////////////////////////////////////////////////
#ifdef AUTO_UPDATE_GT811		//modify by andrew
	msleep(20);
	goodix_read_version(ts);

	ret = gt811_downloader( ts, goodix_gt811_firmware);
	if(ret < 0)
	{
		dev_err(&client->dev, "Warnning: gt811 update might be ERROR!\n");
		//goto err_input_dev_alloc_failed;
	}
#endif
///////////////////////////////UPDATE STEP 1 END////////////////////////////////////////////////////////////////      
#ifdef INT_PORT	
//	client->irq=TS_INT;		//If not defined in client
	if (client->irq)
	{
		ret = gpio_request(client->irq, "TS_INT"); //Request IO
		if (ret < 0)
		{
			dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",
					(int) INT_PORT, ret);
			goto err_gpio_request_failed;
		}
		gpio_direction_input(client->irq);

#ifndef STOP_IRQ_TYPE
		ts->irq = client->irq; //KT ADD 1202
		ts->irq_is_disable = 0; // enable irq
#endif
	}
#endif	

	err_gpio_request_failed:
	for (retry = 0; retry < 3; retry++)
	{
		ret = goodix_init_panel(ts);
		msleep(20);
		if (ret != 0) //Initiall failed
			continue;
		else
			break;
	}
	if (ret != 0)
	{
		ts->bad_data = 1;
		goto err_init_godix_ts;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		dev_dbg(&client->dev,
				"goodix_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	/*	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	 ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	 ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);*/
#ifdef HAVE_TOUCH_KEY
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[retry]);
	}
#endif

	/*
	 input_set_abs_params(ts->input_dev, ABS_X, 0,  ts->abs_x_max, 0, 0);
	 input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	 input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	 */

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));
//	snprintf(ts->name, sizeof(ts->name), "gt811-touchscreen");

//	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = "gt811_ts";
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427; //screen firmware version

	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);

#ifdef GOODIX_MULTI_TOUCH
	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);
	//input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TOUCH_MAX_WIDTH,
			0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TOUCH_MAX_HEIGHT,
			0, 0);

	/*	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	 input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	 input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,  ts->abs_x_max, 0, 0);
	 input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	 */
#endif
	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		dev_err(&client->dev, "Probe: Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;

#ifdef INT_PORT		
	ret = request_irq(client->irq, goodix_ts_irq_handler,
			irq_table[ts->int_trigger_type], client->name, ts);
	if (ret != 0)
	{
		dev_err(&client->dev, "Cannot allocate ts INT!ERRNO:%d\n", ret);
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
		goto err_init_godix_ts;
	}
	else
	{
#ifndef STOP_IRQ_TYPE
		gt811_irq_disable(ts); //KT ADD 1202
#else
				disable_irq(client->irq);
#endif
		ts->use_irq = 1;
//		dev_dbg(&client->dev,"Reques EIRQ %d succesd on GPIO:%d\n",TS_INT,INT_PORT);
	}
#endif	

	if (!ts->use_irq)
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	if (ts->use_irq)
#ifndef STOP_IRQ_TYPE
		gt811_irq_enable(ts); //KT ADD 1202
#else
				enable_irq(client->irq);
#endif

	ts->power = goodix_ts_power;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB; //EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

/////////////////////////////// UPDATE STEP 2 START /////////////////////////////////////////////////////////////////
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	goodix_proc_entry = create_proc_entry("goodix-update", 0666, NULL);
	if(goodix_proc_entry == NULL)
	{
		dev_info(&client->dev, "Couldn't create proc entry!\n");
		ret = -ENOMEM;
		goto err_create_proc_entry;
	}
	else
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		goodix_proc_entry->write_proc = goodix_update_write;
		goodix_proc_entry->read_proc = goodix_update_read;
	}
#endif
///////////////////////////////UPDATE STEP 2 END /////////////////////////////////////////////////////////////////
	dev_info(&client->dev,
			"Start %s in %s mode,Driver Modify Date:2012-01-05\n",
			ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

	err_init_godix_ts:
	i2c_end_cmd(ts);
	if (ts->use_irq)
	{
		ts->use_irq = 0;
		free_irq(client->irq, ts);
#ifdef INT_PORT
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
#endif
	}
	else
		hrtimer_cancel(&ts->timer);

	err_input_register_device_failed:
		input_free_device(ts->input_dev);

	err_input_dev_alloc_failed:
	if(pdata->gpio_reset != INVALID_GPIO)
		gpio_free(pdata->gpio_reset);
	if(pdata->gpio_pendown != INVALID_GPIO)
		gpio_free(pdata->gpio_pendown);
		i2c_set_clientdata(client, NULL);
	err_gpio_request:
		err_i2c_failed: kfree(ts);
	err_alloc_data_failed:
	err_check_functionality_failed:
	err_create_proc_entry:
	return ret;
}

/*******************************************************	
 Function:
 Drive the release of resources
 Parameters:
 client: the device structure
 return:
 Results of the implementation code, 0 for normal execution
 ********************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
/////////////////////////////// UPDATE STEP 3 START/////////////////////////////////////////////////////////////////
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	remove_proc_entry("goodix-update", NULL);
#endif
/////////////////////////////////UPDATE STEP 3 END///////////////////////////////////////////////////////////////

	if (ts && ts->use_irq)
	{
#ifdef INT_PORT
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
#endif
		free_irq(client->irq, ts);
	}
	else if (ts)
		hrtimer_cancel(&ts->timer);

	dev_notice(&client->dev, "The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

//停用设备
static int goodix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		//disable_irq(client->irq);
#ifndef STOP_IRQ_TYPE
		gt811_irq_disable(ts); //KT ADD 1202
#else
				disable_irq(client->irq);
#endif

	else
		hrtimer_cancel(&ts->timer);
#ifndef STOP_IRQ_TYPE
	cancel_work_sync(&ts->work);
#endif
	//ret = cancel_work_sync(&ts->work);
	//if(ret && ts->use_irq)	
	//enable_irq(client->irq);
	if (ts->power)
	{ /* 必须在取消work后再执行，避免因GPIO导致坐标处理代码死循环	*/
		ret = ts->power(ts, 0);
if	(ret < 0)
	printk(KERN_ERR "goodix_ts_resume power off failed\n");
}
	return 0;
}

static int goodix_ts_resume(struct i2c_client *client)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power)
	{
		ret = ts->power(ts, 1);
if	(ret < 0)
	printk(KERN_ERR "goodix_ts_resume power on failed\n");
}

	if (ts->use_irq)
		//enable_irq(client->irq);
#ifndef STOP_IRQ_TYPE
		gt811_irq_enable(ts); //KT ADD 1202
#else
				enable_irq(client->irq);
#endif
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(ts->client);
}
#endif

//可用于该驱动的 设备名—设备ID 列表
//only one client
static const struct i2c_device_id goodix_ts_id[] =
{
	{ GT811_I2C_NAME, 0 },
	{ }
};

//设备驱动结构体
static struct i2c_driver goodix_ts_driver =
{
		.probe = goodix_ts_probe,
		.remove = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
		.suspend = goodix_ts_suspend,
		.resume = goodix_ts_resume,
#endif
		.id_table = goodix_ts_id,
		.driver ={
				.name = GT811_I2C_NAME,
			    .owner = THIS_MODULE,
		         },
};

/*******************************************************	
 功能：
 驱动加载函数
 return：
 执行结果码，0表示正常执行
 ********************************************************/
static int __devinit goodix_ts_init(void)
{
	int ret;

	goodix_wq = create_workqueue("goodix_wq");
//create a work queue and worker thread
if (!goodix_wq)
	{
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;

	}
	ret=i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/*******************************************************	
 功能：
 驱动卸载函数
 参数：
 client：设备结构体
 ********************************************************/
static void __exit goodix_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
	destroy_workqueue(goodix_wq); //release our work queue
}

late_initcall( goodix_ts_init);
//最后初始化驱动felix
module_exit( goodix_ts_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");

