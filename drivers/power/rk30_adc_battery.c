/* drivers/power/rk30_adc_battery.c
 *
 * battery detect driver for the rk30 
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/wakelock.h>


//////////////////////////////////////////////////////////////////////////////
//debug
#if 0
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

#if defined(CONFIG_POWER_ON_CHARGER_DISPLAY)
//mia extern int charge_discharge;
int charge_discharge;
#endif/* #if defined(CONFIG_POWER_ON_CHARGER_DISPLAY) */
static int rk30_battery_dbg_level = 1;
static bool b_rk30_charged = false;
module_param_named(dbg_level, rk30_battery_dbg_level, int, 0644);

/*******************以下参数可以修改******************************/
#define	TIMER_MS_COUNTS		 				50 	//定时器的长度ms
//以下参数需要根据实际测试调整
#define	SLOPE_SECOND_COUNTS					15	//统计电压斜率的时间间隔s
#define	DISCHARGE_MIN_SECOND				60	//最快放电电1%时间
#define	DISCHARGE_MID_SECOND				110	//普通放电电1%时间
#define	DISCHARGE_MAX_SECOND				250	//最长放电电1%时间
#define	CHARGE_MIN_SECOND					60	//最快充电电1%时间
#define	CHARGE_MID_SECOND					110	//普通充电电1%时间
#define	CHARGE_MAX_SECOND					250	//最长充电电1%时间
#define CHARGE_FULL_DELAY_TIMES				10	//充电满检测防抖时间
#define USBCHARGE_IDENTIFY_TIMES			5	//插入USB混流，pc识别检测时间

#define	NUM_VOLTAGE_SAMPLE					((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	 
#define	NUM_DISCHARGE_MIN_SAMPLE			((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_DISCHARGE_MID_SAMPLE			((DISCHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	     
#define	NUM_DISCHARGE_MAX_SAMPLE			((DISCHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	 
#define	NUM_CHARGE_MIN_SAMPLE				((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE				((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	     
#define	NUM_CHARGE_MAX_SAMPLE				((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	  
#define NUM_CHARGE_FULL_DELAY_TIMES			((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度
#define NUM_USBCHARGE_IDENTIFY_TIMES		((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	//充电满状态持续时间长度        

//定义ADC采样分压电阻，以实际值为准，单位K
#define BAT_2V5_VALUE						2500
#define BAT_PULL_UP_R						200//320//200
#define BAT_PULL_DOWN_R						200//200//200
#define adc_to_voltage(adc_val)				((adc_val * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R))

#define DEBUG_MAX_COUNTS_NUMBER				(100)

#define AD_DIFFERENCE_BACKLIGHT_ON_OFF		(110)  //AD读数在背光亮暗的差值
#define AD_DIFFERENCE_DC_IN_OUT_LOW		(110)  //AD读数在电池电压较低,背光最亮时DC插入拔出的差值（压降较大）分水岭大概在3.95V左右
#define AD_DIFFERENCE_DC_IN_OUT_MIDDLE		(110/2)  //AD读数在电池电压较高时DC插入拔出的差值
#define AD_DIFFERENCE_DC_IN_OUT_HIGH		(100)  //AD读数在电池电压高时DC插入拔出的差值

#define CHARGE_LED_CONDITION  (4120)   //充电转灯的一个条件


#define RAW_HUNDRED_PERCENT_VOL_VALUE	    (4080)  //100%时的电池电压
#define RAW_NINETY_PERCENT_VOL_VALUE	    (3950)  //90%时的电池电压
#define RAW_EIGHTY_PERCENT_VOL_VALUE	    (3860)  //80%时的电池电压
#define RAW_FIFTY_PERCENT_VOL_VALUE		    (3760)  //50%时的电池电压 
#define RAW_THIRTY_PERCENT_VOL_VALUE	    (3680)  //30%时的电池电压 
#define RAW_FIFTEEN_PERCENT_VOL_VALUE	    (3620)  //15%时的电池电压
#define RAW_TEN_PERCENT_VOL_VALUE		    (3580)  //10%时的电池电压
#define RAW_FIVE_PERCENT_VOL_VALUE		    (3560)  //5%时的电池电压  
#define RAW_ZERO_PERCENT_VOL_VALUE		    (3550)  //0%时的电池电压     
#define RAW_POWEROFF_VOL_VALUE			    (3500)  //关机时的电池电压

#define DC_HUNDRED_PERCENT_VOL_VALUE	    (4080+AD_DIFFERENCE_DC_IN_OUT_HIGH)	//100%时的电池电压
#define DC_NINETY_PERCENT_VOL_VALUE		    (3950+AD_DIFFERENCE_DC_IN_OUT_HIGH)	//90%时的电池电压
#define DC_EIGHTY_PERCENT_VOL_VALUE		    (3860+AD_DIFFERENCE_DC_IN_OUT_LOW)	//80%时的电池电压
#define DC_FIFTY_PERCENT_VOL_VALUE		    (3760+AD_DIFFERENCE_DC_IN_OUT_LOW)	//50%时的电池电压 
#define DC_THIRTY_PERCENT_VOL_VALUE		    (3680+AD_DIFFERENCE_DC_IN_OUT_LOW)	//30%时的电池电压 
#define DC_FIFTEEN_PERCENT_VOL_VALUE	    (3620+AD_DIFFERENCE_DC_IN_OUT_LOW)	//15%时的电池电压
#define DC_TEN_PERCENT_VOL_VALUE		    (3580+AD_DIFFERENCE_DC_IN_OUT_LOW)	//10%时的电池电压
#define DC_FIVE_PERCENT_VOL_VALUE		    (3560+AD_DIFFERENCE_DC_IN_OUT_LOW)	//5%时的电池电压  
#define DC_ZERO_PERCENT_VOL_VALUE		    (3550+AD_DIFFERENCE_DC_IN_OUT_LOW)	//0%时的电池电压     
#define DC_POWEROFF_VOL_VALUE			    (3500+AD_DIFFERENCE_DC_IN_OUT_LOW)	//关机时的电池电压


#define BATT_FILENAME "/data/bat_last_capacity.dat"
#define BATT_NUM							(22)

struct batt_vol_cal{
	u32 disp_cal;
	u32 dis_charge_vol;
	u32 charge_vol;
};

static struct batt_vol_cal  adc_battery_table[BATT_NUM] = {
	{0,RAW_POWEROFF_VOL_VALUE,DC_POWEROFF_VOL_VALUE},
	
	{1,RAW_ZERO_PERCENT_VOL_VALUE,DC_ZERO_PERCENT_VOL_VALUE},
	
	{5,RAW_FIVE_PERCENT_VOL_VALUE,DC_FIVE_PERCENT_VOL_VALUE},
	
	{10,RAW_TEN_PERCENT_VOL_VALUE,DC_TEN_PERCENT_VOL_VALUE},
	
	{15,RAW_FIFTEEN_PERCENT_VOL_VALUE,DC_FIFTEEN_PERCENT_VOL_VALUE},
	
	{20,(RAW_FIFTEEN_PERCENT_VOL_VALUE+((RAW_THIRTY_PERCENT_VOL_VALUE-RAW_FIFTEEN_PERCENT_VOL_VALUE)/3)),(DC_FIFTEEN_PERCENT_VOL_VALUE+((DC_THIRTY_PERCENT_VOL_VALUE-DC_FIFTEEN_PERCENT_VOL_VALUE)/3))},
	{25,(RAW_FIFTEEN_PERCENT_VOL_VALUE+((RAW_THIRTY_PERCENT_VOL_VALUE-RAW_FIFTEEN_PERCENT_VOL_VALUE)*2/3)),(DC_FIFTEEN_PERCENT_VOL_VALUE+((DC_THIRTY_PERCENT_VOL_VALUE-DC_FIFTEEN_PERCENT_VOL_VALUE)*2/3))},
	{30,RAW_THIRTY_PERCENT_VOL_VALUE,DC_THIRTY_PERCENT_VOL_VALUE},
	
	{35,(RAW_THIRTY_PERCENT_VOL_VALUE+((RAW_FIFTY_PERCENT_VOL_VALUE-RAW_THIRTY_PERCENT_VOL_VALUE)/4)),(DC_THIRTY_PERCENT_VOL_VALUE+((DC_FIFTY_PERCENT_VOL_VALUE-DC_THIRTY_PERCENT_VOL_VALUE)/4))},	
	{40,(RAW_THIRTY_PERCENT_VOL_VALUE+((RAW_FIFTY_PERCENT_VOL_VALUE-RAW_THIRTY_PERCENT_VOL_VALUE)*2/4)),(DC_THIRTY_PERCENT_VOL_VALUE+((DC_FIFTY_PERCENT_VOL_VALUE-DC_THIRTY_PERCENT_VOL_VALUE)*2/4))},
	{45,(RAW_THIRTY_PERCENT_VOL_VALUE+((RAW_FIFTY_PERCENT_VOL_VALUE-RAW_THIRTY_PERCENT_VOL_VALUE)*3/4)),(DC_THIRTY_PERCENT_VOL_VALUE+((DC_FIFTY_PERCENT_VOL_VALUE-DC_THIRTY_PERCENT_VOL_VALUE)*3/4))},	
	{50,RAW_FIFTY_PERCENT_VOL_VALUE,DC_FIFTY_PERCENT_VOL_VALUE},
	
	{55,(RAW_FIFTY_PERCENT_VOL_VALUE+((RAW_EIGHTY_PERCENT_VOL_VALUE-RAW_FIFTY_PERCENT_VOL_VALUE)/6)),(DC_FIFTY_PERCENT_VOL_VALUE+((DC_EIGHTY_PERCENT_VOL_VALUE-DC_FIFTY_PERCENT_VOL_VALUE)/6))},
	{60,(RAW_FIFTY_PERCENT_VOL_VALUE+((RAW_EIGHTY_PERCENT_VOL_VALUE-RAW_FIFTY_PERCENT_VOL_VALUE)*2/6)),(DC_FIFTY_PERCENT_VOL_VALUE+((DC_EIGHTY_PERCENT_VOL_VALUE-DC_FIFTY_PERCENT_VOL_VALUE)*2/6))},
	{65,(RAW_FIFTY_PERCENT_VOL_VALUE+((RAW_EIGHTY_PERCENT_VOL_VALUE-RAW_FIFTY_PERCENT_VOL_VALUE)*3/6)),(DC_FIFTY_PERCENT_VOL_VALUE+((DC_EIGHTY_PERCENT_VOL_VALUE-DC_FIFTY_PERCENT_VOL_VALUE)*3/6))},
	{70,(RAW_FIFTY_PERCENT_VOL_VALUE+((RAW_EIGHTY_PERCENT_VOL_VALUE-RAW_FIFTY_PERCENT_VOL_VALUE)*4/6)),(DC_FIFTY_PERCENT_VOL_VALUE+((DC_EIGHTY_PERCENT_VOL_VALUE-DC_FIFTY_PERCENT_VOL_VALUE)*4/6))},	
	{75,(RAW_FIFTY_PERCENT_VOL_VALUE+((RAW_EIGHTY_PERCENT_VOL_VALUE-RAW_FIFTY_PERCENT_VOL_VALUE)*5/6)),(DC_FIFTY_PERCENT_VOL_VALUE+((DC_EIGHTY_PERCENT_VOL_VALUE-DC_FIFTY_PERCENT_VOL_VALUE)*5/6))},
	{80,RAW_EIGHTY_PERCENT_VOL_VALUE,DC_EIGHTY_PERCENT_VOL_VALUE},
	
	{85,(RAW_EIGHTY_PERCENT_VOL_VALUE+((RAW_NINETY_PERCENT_VOL_VALUE-RAW_EIGHTY_PERCENT_VOL_VALUE)/2)),(DC_EIGHTY_PERCENT_VOL_VALUE+((DC_NINETY_PERCENT_VOL_VALUE-DC_EIGHTY_PERCENT_VOL_VALUE)/2))},	
	{90,RAW_NINETY_PERCENT_VOL_VALUE,DC_NINETY_PERCENT_VOL_VALUE},
	
	{95,(RAW_NINETY_PERCENT_VOL_VALUE+((RAW_HUNDRED_PERCENT_VOL_VALUE-RAW_NINETY_PERCENT_VOL_VALUE)/2)),(DC_NINETY_PERCENT_VOL_VALUE+((DC_HUNDRED_PERCENT_VOL_VALUE-DC_NINETY_PERCENT_VOL_VALUE)/2))},	
	{100,RAW_HUNDRED_PERCENT_VOL_VALUE,DC_HUNDRED_PERCENT_VOL_VALUE},	
												
};

/********************************************************************************/
static struct wake_lock batt_wake_lock;

extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);

///////////////////////////////////////////////////////////////////////////////////

struct rk30_adc_battery_data {
	int irq;
	//struct timer_list       timer;
	struct workqueue_struct *wq;
	struct delayed_work     delay_work;
	struct work_struct 	    dcwakeup_work;
	struct work_struct      lowerpower_work;
	bool                    resume;
	
	struct rk30_adc_battery_platform_data *pdata;

	int                     full_times;
	
	struct adc_client       *client; 
	int                     adc_val;
	int                     adc_samples[NUM_VOLTAGE_SAMPLE+2];
	
	int                     bat_status;
	int                     bat_status_cnt;
	int                     bat_health;
	int                     bat_present;
	int                     bat_voltage;
	int                     bat_capacity;
	int                     bat_change;
	
	int                     old_charge_level;
	int                    *pSamples;
	int                     gBatCapacityDisChargeCnt;
	int                     gBatCapacityChargeCnt;
	int                     capacitytmp;
	int                     poweron_check;
	int                     suspend_capacity;
};
enum {
	BATTERY_STATUS          = 0,
	BATTERY_HEALTH          = 1,
	BATTERY_PRESENT         = 2,
	BATTERY_CAPACITY        = 3,
	BATTERY_AC_ONLINE       = 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED   	= 6,
	BATTERY_INT_STATUS	    = 7,
	BATTERY_INT_ENABLE	    = 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

static int DeBUG_CountA,DeBUG_CountB,DeBUG_CountC,DeBUG_CountD=0;	//debug printk out count times
static struct rk30_adc_battery_data *gBatteryData;

static int rk30_adc_battery_load_capacity(void)
{
	char value[4];
	int* p = (int *)value;
	long fd = sys_open((char __user *)BATT_FILENAME,O_RDONLY,0);

	if(fd < 0){
		DBG("%s: open file %s failed\n",__func__,BATT_FILENAME);
		return -1;
	}

	sys_read(fd,(char __user *)value,4);
	sys_close(fd);
	
	DBG("zpp-->trace %s read file %s =%d-- ok!\n",__func__,BATT_FILENAME,(*p));

	return (*p);
}

static void rk30_adc_battery_put_capacity(int loadcapacity)
{
	char value[4];
	int* p = (int *)value;
	long fd = sys_open((char __user *)BATT_FILENAME,O_WRONLY,0);
	
	if(fd < 0){
		DBG("zpp-->trace %s open file %s failed\n",__func__,BATT_FILENAME);
		return;
	}
	
	*p = loadcapacity;
	sys_write(fd, (const char __user *)value, 4);
	DBG("zpp-->trace %s write file %s =%d-- ok!\n",__func__,BATT_FILENAME,loadcapacity);

	sys_close(fd);
}

static void rk30_adc_battery_charge_enable(struct rk30_adc_battery_data *bat)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO){
		gpio_direction_output(pdata->charge_set_pin, pdata->charge_set_level);
	}
}

static void rk30_adc_battery_charge_disable(struct rk30_adc_battery_data *bat)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	if (pdata->charge_set_pin != INVALID_GPIO){
		gpio_direction_output(pdata->charge_set_pin, !(pdata->charge_set_level));
	}
}

static int rk30_adc_set_charge_led_status(struct rk30_adc_battery_data *bat,int value)
{
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	
	if(value) //led turn red
	{
		gpio_direction_output(pdata->charge_led_pin,pdata->charge_led_level);
		gpio_set_value(pdata->charge_led_pin,pdata->charge_led_level);
		//gpio_direction_output(pdata->charge_led_pin,!(pdata->charge_led_level));
		//gpio_set_value(pdata->charge_led_pin,!(pdata->charge_led_level));	
	}
	else   //led turn green
	{
		gpio_direction_output(pdata->charge_led_pin,!(pdata->charge_led_level));
		gpio_set_value(pdata->charge_led_pin,!(pdata->charge_led_level));	
		//gpio_direction_output(pdata->charge_led_pin,pdata->charge_led_level);
		//gpio_set_value(pdata->charge_led_pin,pdata->charge_led_level);	
	}
}

static int rk30_adc_battery_get_charge_level(struct rk30_adc_battery_data *bat)
{
	int charge_on = 0;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

#if defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	if (pdata->dc_det_pin != INVALID_GPIO){
		if (gpio_get_value (pdata->dc_det_pin) == pdata->dc_det_level){
			charge_on = 1;
		}
	}
#endif

#if defined  (CONFIG_BATTERY_RK30_USB_CHARGE)
	if (charge_on == 0){
		if (suspend_flag)
			return;
		if (1 == dwc_vbus_status()) {          //检测到USB插入，但是无法识别是否是充电器
		                                 //通过延时检测PC识别标志，如果超时检测不到，说明是充电
			if (0 == get_msc_connect_flag()){                               //插入充电器时间大于一定时间之后，开始进入充电状态
				if (++gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES){
					gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
					charge_on = 1;
				}
			}                               //否则，不进入充电模式
		}                   
		else{
			gBatUsbChargeCnt = 0;
			if (2 == dwc_vbus_status()) {
				charge_on = 1;
			}
		}
	}
#endif

#if defined(CONFIG_POWER_ON_CHARGER_DISPLAY)
	charge_discharge = charge_on;
#endif /* #if defined(CONFIG_POWER_ON_CHARGER_DISPLAY) */	

	
    if((charge_on !=0))
    {
        wake_lock(&batt_wake_lock);
        b_rk30_charged = true;
    }
    else
    {
        wake_unlock(&batt_wake_lock);        
	}	

	return charge_on;
}

//int old_charge_level;
static int rk30_adc_battery_status_samples(struct rk30_adc_battery_data *bat)
{
	int charge_level;
	
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	charge_level = rk30_adc_battery_get_charge_level(bat);

	//检测充电状态变化情况
	if (charge_level != bat->old_charge_level){
		bat->old_charge_level = charge_level;
		bat->bat_change  = 1;
		
		if(charge_level) {            
			rk30_adc_battery_charge_enable(bat);
		}
		else{
			rk30_adc_battery_charge_disable(bat);
		}
		bat->bat_status_cnt = 0;        //状态变化开始计数
	}

	if(charge_level == 0){   
	//discharge
		bat->full_times = 0;
		bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	else{
	//CHARGE	    
		if (pdata->charge_ok_pin == INVALID_GPIO){  //no charge_ok_pin

			if (bat->bat_capacity == 100){
				if (bat->bat_status != POWER_SUPPLY_STATUS_FULL){
					bat->bat_status = POWER_SUPPLY_STATUS_FULL;
					bat->bat_change  = 1;
				}
			}
			else{
				bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		}
		else{  // pin of charge_ok_pin
			if (gpio_get_value(pdata->charge_ok_pin) != pdata->charge_ok_level){

				bat->full_times = 0;
				bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
			}
			else{
				//检测到充电满电平标志
				bat->full_times++;

				if (bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) {
					bat->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
				}

				if ((bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) && (bat->bat_capacity==100)){
					if (bat->bat_status != POWER_SUPPLY_STATUS_FULL){
						bat->bat_status = POWER_SUPPLY_STATUS_FULL;
						bat->bat_change  = 1;
					}
				}
				else{
					bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
				}
			}
		}
	}

	return charge_level;
}

static int *pSamples;
static void rk30_adc_battery_voltage_samples(struct rk30_adc_battery_data *bat)
{
	int value;
	int i,*pStart = bat->adc_samples, num = 0;
	//int level = rk30_adc_battery_get_charge_level(bat);

	value = bat->adc_val;
	adc_async_read(bat->client);

	*pSamples++ = adc_to_voltage(value);

	bat->bat_status_cnt++;
	if (bat->bat_status_cnt > NUM_VOLTAGE_SAMPLE)  bat->bat_status_cnt = NUM_VOLTAGE_SAMPLE + 1;

	num = pSamples - pStart;
	
	if (num >= NUM_VOLTAGE_SAMPLE){
		pSamples = pStart;
		num = NUM_VOLTAGE_SAMPLE;
	}

	value = 0;
	for (i = 0; i < num; i++){
		value += bat->adc_samples[i];
	}
	bat->bat_voltage = value / num;
	
	/*消除毛刺电压*/
	/* 这一段是所有充电的bug所在
	level = rk30_adc_battery_get_charge_level(bat);
	if(1 == level){
		if(bat->bat_voltage >= adc_battery_table[BATT_NUM-1].charge_vol+ 10)
			bat->bat_voltage = adc_battery_table[BATT_NUM-1].charge_vol  + 10;
		else if(bat->bat_voltage <= adc_battery_table[0].charge_vol  - 10)
			bat->bat_voltage =  adc_battery_table[0].charge_vol - 10;
	}
	else{
		if(bat->bat_voltage >= adc_battery_table[BATT_NUM-1].dis_charge_vol+ 10)
			bat->bat_voltage = adc_battery_table[BATT_NUM-1].dis_charge_vol  + 10;
		else if(bat->bat_voltage <= adc_battery_table[0].dis_charge_vol  - 10)
			bat->bat_voltage =  adc_battery_table[0].dis_charge_vol - 10;
	}*/
}

static bool b_to_change_adc_table = true;
static int rk30_adc_battery_voltage_to_capacity(struct rk30_adc_battery_data *bat, int BatVoltage)
{
	int i = 0;
	int capacity = 0;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
	struct batt_vol_cal *p;
	int current_real_voltage;
	
	p = adc_battery_table;
	
	//current_real_voltage = adc_to_voltage(gBatteryData->adc_val) ;
	//printk("zpp->traced->F:%s,L:%d: ajust adc_battery_table when charge p[BATT_NUM - 1].charge_vol=%d\n",__FUNCTION__,__LINE__,current_real_voltage);
	
	if (rk30_adc_battery_get_charge_level(bat))	//charge ok and ajust the adc_battery_table it is very valueable.
	{  
		if( (BatVoltage  >= CHARGE_LED_CONDITION) && b_to_change_adc_table )
		{
			if (pdata->charge_ok_pin != INVALID_GPIO){
				if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level){
					p[BATT_NUM - 1].charge_vol = BatVoltage-60 ; //这里要减去60，考虑AD读数 "悬飘"
					p[BATT_NUM - 1].dis_charge_vol = BatVoltage - 60 - AD_DIFFERENCE_DC_IN_OUT_HIGH;
					p[BATT_NUM - 1].disp_cal = 100;		
					//printk("zpp->traced-> ajust adc_battery_table when charge BatVoltage=%d current_real_voltage=%d\n",BatVoltage,current_real_voltage);
				}
			}
		}
		
		if(BatVoltage >= (p[BATT_NUM - 1].charge_vol))
		{
			if (pdata->charge_ok_pin != INVALID_GPIO){
				if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level){
					capacity = 100;
					rk30_adc_set_charge_led_status(bat,0);//green
					b_to_change_adc_table = false;
					//printk("zpp->traced->F:%s,L:%d: true capacity=%d green\n",__FUNCTION__,__LINE__,capacity);
				}
				else
				{
					capacity = 99;
					rk30_adc_set_charge_led_status(bat,1);//red
					//printk("zpp->traced->F:%s,L:%d: true capacity=%d  red\n",__FUNCTION__,__LINE__,capacity);
				}
			}
			else
			{
				capacity = 100;
				rk30_adc_set_charge_led_status(bat,0);//green
			}		
			
		}	
		else
		{
			rk30_adc_set_charge_led_status(bat,1);//red
			if(BatVoltage <= (p[0].charge_vol))
			{
				capacity = 0;
			}
			else
			{
				for(i = 0; i < BATT_NUM - 1; i++)
				{
					if(((p[i].charge_vol) <= BatVoltage) && (BatVoltage < (p[i+1].charge_vol)))
					{
						capacity = p[i].disp_cal + ((BatVoltage - p[i].charge_vol) * (p[i+1].disp_cal - p[i].disp_cal)) / (p[i+1].charge_vol - p[i].charge_vol);
						break;
					}
				}
			} 
			//printk("zpp->traced->F:%s,L:%d: true capacity=%d\n",__FUNCTION__,__LINE__,capacity);			 
		}
	}
	else	//discharge
	{
		rk30_adc_set_charge_led_status(bat,0);//green and default low
		b_to_change_adc_table = true;
		if(BatVoltage >= (p[BATT_NUM - 1].dis_charge_vol))
		{
			capacity = 100;
		}	
		else
		{
			if(BatVoltage <= (p[0].dis_charge_vol))
			{
				capacity = 0;
			}
			else
			{
				for(i = 0; i < BATT_NUM - 1; i++)
				{
					if(((p[i].dis_charge_vol) <= BatVoltage) && (BatVoltage < (p[i+1].dis_charge_vol)))
					{
						capacity = p[i].disp_cal + ((BatVoltage - p[i].dis_charge_vol) * (p[i+1].disp_cal - p[i].disp_cal)) / (p[i+1].dis_charge_vol - p[i].dis_charge_vol);
						break;
					}
				}
			}  

		}
	}

	if( ++DeBUG_CountD > DEBUG_MAX_COUNTS_NUMBER){
		DeBUG_CountD=0;
		DBG("zpp->traced->F:%s,L:%d:BatVoltage=%d true capacity=%d\n",__FUNCTION__,__LINE__,BatVoltage,capacity);
	}
	
    return capacity;
}

static int base_var_capacity=1;
static int step_var_capacity = 1;
static int runtime_capacity,total_time,zpp_var_tire = 0;
static void rk30_adc_battery_capacity_samples(struct rk30_adc_battery_data *bat)
{
	int capacity = 0;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;

	//充放电状态变化后，Buffer填满之前，不更新
    if(++DeBUG_CountA > DEBUG_MAX_COUNTS_NUMBER)
    {
        DeBUG_CountA  = 0;
        DBG("zpp->traced->F:%s,L:%d: bat->bat_status_cnt=%d,base_var_capacity = %d\n",__FUNCTION__,__LINE__,bat->bat_status_cnt,base_var_capacity);
    }
    
	if (bat->bat_status_cnt < NUM_VOLTAGE_SAMPLE)  {
		bat->gBatCapacityDisChargeCnt = 0;
		bat->gBatCapacityChargeCnt    = 0;
		return;
	}
	
	capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
	runtime_capacity = rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val));
	
	if (rk30_adc_battery_get_charge_level(bat))	//充放电控制,要实现一个追击问题，一个追一个跑
	{	
		bat->gBatCapacityDisChargeCnt = 0; //充电时，放电计时器必为0
		
		if(rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)) >= 90)
		{
			if (pdata->charge_ok_pin != INVALID_GPIO){
				if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level){
					total_time = NUM_CHARGE_MAX_SAMPLE/10;
				}
				else
				{
					total_time = NUM_CHARGE_MAX_SAMPLE;
				}
			}
		}
		else if(rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)) >= 40)
		{
			total_time = NUM_CHARGE_MID_SAMPLE;
		}
		else
		{
			total_time = NUM_CHARGE_MIN_SAMPLE;
		}
		
		if( abs(base_var_capacity-step_var_capacity) == 0)
		{
			zpp_var_tire = total_time;
		}
		else
		{
			zpp_var_tire = total_time *abs(base_var_capacity-step_var_capacity)/base_var_capacity;
		}

		if( ++(bat->gBatCapacityChargeCnt) >= zpp_var_tire)
		{
			base_var_capacity = ((abs(runtime_capacity - gBatteryData->bat_capacity) ==0 ) ? 1 : abs(runtime_capacity - gBatteryData->bat_capacity));//注意不能为0!!!
			if(step_var_capacity < base_var_capacity)
			{
				step_var_capacity = step_var_capacity+1;
			}
			else if(step_var_capacity == base_var_capacity)
			{
				step_var_capacity = base_var_capacity;
			}
			else
			{
				step_var_capacity = 1;
			}
			bat->bat_capacity =(( capacity > bat->bat_capacity ) ? (bat->bat_capacity+1) : bat->bat_capacity);
			bat->bat_change  = 1;
			bat->gBatCapacityChargeCnt = 0;
		}				
	}
	else
	{
		bat->gBatCapacityChargeCnt = 0;//放电时，充电计时器必为0
		
		if(rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)) >= 90)
		{
			total_time = NUM_DISCHARGE_MAX_SAMPLE;
		}
		else if(rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)) >= 40)
		{
			total_time = NUM_DISCHARGE_MID_SAMPLE;
		}
		else
		{
			total_time = NUM_DISCHARGE_MIN_SAMPLE;
			if (pdata->batt_low_pin != INVALID_GPIO){
				if (gpio_get_value(pdata->batt_low_pin) == GPIO_LOW){
					total_time = NUM_DISCHARGE_MIN_SAMPLE/10;
				}
				else
				{
					total_time = NUM_DISCHARGE_MIN_SAMPLE;
				}
			}			
		}		
		
		if( abs(base_var_capacity-step_var_capacity) == 0)
		{
			zpp_var_tire = total_time;
		}
		else
		{
			zpp_var_tire = total_time *abs(base_var_capacity-step_var_capacity)/base_var_capacity;
		}
		
		if( bat->bat_capacity ==100 )  // 电量为100时，如果放电，快速降到99
		{
			zpp_var_tire = NUM_DISCHARGE_MIN_SAMPLE/2;
		}
		
		
		if( ++(bat->gBatCapacityDisChargeCnt) >= zpp_var_tire )
		{
			if( bat->bat_capacity ==100 )
			{
				bat->bat_capacity = bat->bat_capacity -1;
			}
			else
			{
				bat->bat_capacity =(( capacity < bat->bat_capacity ) ? (bat->bat_capacity-1) : bat->bat_capacity);
			}
			bat->bat_change  = 1;
			bat->gBatCapacityDisChargeCnt = 0;
		}					
	}
		
	
	bat->capacitytmp = capacity;
}

static void rk30_adc_battery_poweron_capacity_check(void)
{

	int new_capacity, old_capacity;

	new_capacity = gBatteryData->bat_capacity;
	old_capacity = rk30_adc_battery_load_capacity();
	if ((old_capacity <= 0) || (old_capacity >= 100)){
		old_capacity = new_capacity;
	}    

	if (gBatteryData->bat_status == POWER_SUPPLY_STATUS_FULL){
		if (new_capacity > 80){
			gBatteryData->bat_capacity = 100;
		}
	}
	else if (gBatteryData->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING){
	//chargeing state
	//问题：
	//1）长时间关机放置后，开机后读取的容量远远大于实际容量怎么办？
	//2）如果不这样做，短时间关机再开机，前后容量不一致又该怎么办？
	//3）一下那种方式合适？
	//gBatteryData->bat_capacity = new_capacity;
		gBatteryData->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
	}else{

		if(new_capacity > old_capacity + 50 )
			gBatteryData->bat_capacity = new_capacity;
		else
			gBatteryData->bat_capacity = ( old_capacity > new_capacity ) ? old_capacity : new_capacity;  //avoid the value of capacity increase 
	}

	gBatteryData->bat_change = 1;
}



#if defined(CONFIG_BATTERY_RK30_USB_CHARGE)
static int rk30_adc_battery_get_usb_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;

	default:
		return -EINVAL;
	}
	
	return 0;

}

static enum power_supply_property rk30_adc_battery_usb_props[] = {
    
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk30_usb_supply = 
{
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,

	.get_property   = rk30_adc_battery_get_usb_property,

	.properties     = rk30_adc_battery_usb_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_usb_props),
};
#endif

#if defined(CONFIG_BATTERY_RK30_AC_CHARGE)
static irqreturn_t rk30_adc_battery_dc_wakeup(int irq, void *dev_id)
{   
	queue_work(gBatteryData->wq, &gBatteryData->dcwakeup_work);
	return IRQ_HANDLED;
}


static int rk30_adc_battery_get_ac_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			if (rk30_adc_battery_get_charge_level(gBatteryData))
			{
				val->intval = 1;
			}
			else
			{
				val->intval = 0;	
			}
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk30_adc_battery_ac_props[] = 
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk30_ac_supply = 
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,

	.get_property   = rk30_adc_battery_get_ac_property,

	.properties     = rk30_adc_battery_ac_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_ac_props),
};

static void rk30_adc_battery_dcdet_delaywork(struct work_struct *work)
{
	int ret;
	struct rk30_adc_battery_platform_data *pdata;
	int irq;
	int irq_flag;
	//DBG("DC_WAKEUP\n");
	pdata    = gBatteryData->pdata;
	irq        = gpio_to_irq(pdata->dc_det_pin);
	irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	rk28_send_wakeup_key(); // wake up the system
	rk30_adc_set_charge_led_status(gBatteryData,1);//red

	free_irq(irq, NULL);
	ret = request_irq(irq, rk30_adc_battery_dc_wakeup, irq_flag, "ac_charge_irq", NULL);// reinitialize the DC irq 
	if (ret) {
		free_irq(irq, NULL);
	}

	power_supply_changed(&rk30_ac_supply);
	
	gBatteryData->bat_status_cnt = 0;        //the state of battery is change
	//wake_lock_timeout(&batt_wake_lock, 30 * HZ);
}


#endif

static int rk30_adc_battery_get_status(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_status);
}

static int rk30_adc_battery_get_health(struct rk30_adc_battery_data *bat)
{
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int rk30_adc_battery_get_present(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_voltage < RAW_HUNDRED_PERCENT_VOL_VALUE) ? 0 : 1;
}

static int rk30_adc_battery_get_voltage(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_voltage *1000);//mia volt * 1000
}

static int rk30_adc_battery_get_capacity(struct rk30_adc_battery_data *bat)
{
	return (bat->bat_capacity);
}

static int rk30_adc_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{		
	int ret = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = rk30_adc_battery_get_status(gBatteryData);
			DBG("gBatStatus=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = rk30_adc_battery_get_health(gBatteryData);
			DBG("gBatHealth=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = rk30_adc_battery_get_present(gBatteryData);
			DBG("gBatPresent=%d\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val ->intval = rk30_adc_battery_get_voltage(gBatteryData);
			DBG("gBatVoltage=%d\n",val->intval);
			break;
		//	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//		val->intval = 1100;
		//		break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = rk30_adc_battery_get_capacity(gBatteryData);
			DBG("gBatCapacity=%d%%\n",val->intval);
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = RAW_HUNDRED_PERCENT_VOL_VALUE;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = RAW_POWEROFF_VOL_VALUE;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static enum power_supply_property rk30_adc_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static struct power_supply rk30_battery_supply = 
{
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,

	.get_property   = rk30_adc_battery_get_property,

	.properties     = rk30_adc_battery_props,
	.num_properties = ARRAY_SIZE(rk30_adc_battery_props),
};

#ifdef CONFIG_PM
//int suspend_capacity = 0;
static void rk30_adc_battery_resume_check(void)
{
	int i;
	int level,oldlevel;
	int new_capacity, old_capacity;
	struct rk30_adc_battery_data *bat = gBatteryData;

	bat->old_charge_level = -1;
	pSamples = bat->adc_samples;

	adc_sync_read(bat->client);                             //start adc sample
	level = oldlevel = rk30_adc_battery_status_samples(bat);//init charge status

	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++) {               //0.3 s   
	
		mdelay(1);
		rk30_adc_battery_voltage_samples(bat);              //get voltage
		level = rk30_adc_battery_status_samples(bat);       //check charge status
		if (oldlevel != level){		
		    oldlevel = level;                               //if charge status changed, reset sample
		    i = 0;
		}        
	}
	new_capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
	old_capacity =gBatteryData-> suspend_capacity;

	if (bat->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING){
	//chargeing state
		bat->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
	}
	else{
		bat->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;  // aviod the value of capacity increase    dicharge
	}
}

static int rk30_adc_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	gBatteryData->suspend_capacity = gBatteryData->bat_capacity;
	return 0;
}

static int rk30_adc_battery_resume(struct platform_device *dev)
{
	gBatteryData->resume = true;
	return 0;
}
#else
#define rk30_adc_battery_suspend NULL
#define rk30_adc_battery_resume NULL
#endif


unsigned long AdcTestCnt = 0;
static bool android_allow_to_write = false;
static bool launch_update_batt_in_boot = false;
static int m_load_capacity = 0;
static bool b_first_to_show_battery = true;
static void rk30_adc_battery_timer_work(struct work_struct *work)
{
#ifdef CONFIG_PM
	if (gBatteryData->resume) {
		rk30_adc_battery_resume_check();
		gBatteryData->resume = false;
	}
#endif

	rk30_adc_battery_status_samples(gBatteryData);

	if (gBatteryData->poweron_check){   
		gBatteryData->poweron_check = 0;
		rk30_adc_battery_poweron_capacity_check();
	}

	rk30_adc_battery_voltage_samples(gBatteryData);
	
	rk30_adc_battery_capacity_samples(gBatteryData);
	
	if(launch_update_batt_in_boot)
	{
		m_load_capacity = rk30_adc_battery_load_capacity();
		DBG("m_load_capacity =%d, launch_update_batt_in_boot=%d,b_rk30_charged=%d \n",m_load_capacity,launch_update_batt_in_boot,b_rk30_charged);
		if(b_rk30_charged)
		{
			gBatteryData->bat_capacity =rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val));	
			launch_update_batt_in_boot = false;
			gBatteryData->bat_change  = 1;
			android_allow_to_write = true;		
		}
		else
		{
			DBG("m_load_capacity (%d),Real_capacity(%d)\n",m_load_capacity,rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)));
			if( m_load_capacity >= 0 ) 
			{
				if(m_load_capacity == 0)
				{
					if(rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)) == 0)
					{
						m_load_capacity = 0;
					}
					else
					{
						m_load_capacity = rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val));
					}
				}
				else if((abs(m_load_capacity - rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val))) > 5) || (m_load_capacity > 100))
				{
					m_load_capacity = rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val));
				}
				else if( rk30_adc_battery_voltage_to_capacity(gBatteryData, adc_to_voltage(gBatteryData->adc_val)) == 100 )
				{
					m_load_capacity =100;
				}
	
				gBatteryData->bat_capacity = m_load_capacity;
				launch_update_batt_in_boot = false;
				gBatteryData->bat_change  = 1;
				android_allow_to_write = true;
			}
		}
	}
	
	/*update battery parameter after adc and capacity has been changed*/
	if(gBatteryData->bat_change){
		gBatteryData->bat_change = 0;
		if(android_allow_to_write) //when boot in kenerl,do not record the capacity. enter into android system record it 
		{
			rk30_adc_battery_put_capacity(gBatteryData->bat_capacity);
		}
		DeBUG_CountB = ((gBatteryData->bat_capacity % 5) == 0) ? 0 : DeBUG_CountB;
		power_supply_changed(&rk30_battery_supply);
	}

//static int base_var_capacity;
//static int step_var_capacity = 1;
//static int runtime_capacity,total_time,zpp_var_tire = 0;
	if (rk30_battery_dbg_level){
		if( (((gBatteryData->bat_capacity % 5) == 0) && ( DeBUG_CountB == 0)) || (b_first_to_show_battery) )
		{
			b_first_to_show_battery = false;
			DeBUG_CountB =1;
//			DBG_Trace("Battery_status( %d ), Real_AD_value( %d ), Real_vol( %dV),Disp_vol( %dV ), \n\t\t\t  Disp_capactiy( %d ), Real_capacity( %d ), discharge counts( %d ), charge counts( %d ), \n\t\t\t b_rk30_charged(%d),base_var_capacity(%d),step_var_capacity(%d),total_time(%d),zpp_var_tire(%d),DeBUG_CountB(%d)", 
			DBG("Battery_status( %d ), Real_AD_value( %d ), Real_vol( %dV),Disp_vol( %dV ), \n\t\t\t  Disp_capactiy( %d ), Real_capacity( %d ), discharge counts( %d ), charge counts( %d ), \n\t\t\t b_rk30_charged(%d),base_var_capacity(%d),step_var_capacity(%d),total_time(%d),zpp_var_tire(%d),DeBUG_CountB(%d)", 
			gBatteryData->bat_status, gBatteryData->adc_val, adc_to_voltage(gBatteryData->adc_val), 
			gBatteryData->bat_voltage, gBatteryData->bat_capacity, gBatteryData->capacitytmp, 
			gBatteryData->gBatCapacityDisChargeCnt,gBatteryData->gBatCapacityChargeCnt,
			b_rk30_charged,base_var_capacity,step_var_capacity,total_time,zpp_var_tire,DeBUG_CountB);
//mia
		}
	}
	queue_delayed_work(gBatteryData->wq, &gBatteryData->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS));

}


static int rk30_adc_battery_io_init(struct rk30_adc_battery_platform_data *pdata)
{
	int ret = 0;
	
	if (pdata->io_init) {
		pdata->io_init();
	}
	
	//charge control pin
	if (pdata->charge_set_pin != INVALID_GPIO){
    	ret = gpio_request(pdata->charge_set_pin, NULL);
    	if (ret) {
    		DBG("failed to request dc_det gpio\n");
    		goto error;
	    }
    	gpio_direction_output(pdata->charge_set_pin, !pdata->charge_set_level);
	}
	
	//dc charge detect pin
	if (pdata->dc_det_pin != INVALID_GPIO){
		ret = gpio_request(pdata->dc_det_pin, NULL);
		if (ret) {
			DBG("failed to request dc_det gpio\n");
			goto error;
		}
	
		gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
		ret = gpio_direction_input(pdata->dc_det_pin);
		if (ret) {
			DBG("failed to set gpio dc_det input\n");
			goto error;
		}
	}
	
	//charge ok detect
	if (pdata->charge_ok_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->charge_ok_pin, NULL);
    	if (ret) {
    		DBG("failed to request charge_ok gpio\n");
    		goto error;
    	}

    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->charge_ok_pin);
    	if (ret) {
    		DBG("failed to set gpio charge_ok input\n");
    		goto error;
    	}
	}
	//batt low pin
	if( pdata->batt_low_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->batt_low_pin, NULL);
	    	if (ret) {
	    		DBG("failed to request batt_low_pin gpio\n");
	    		goto error;
	    	}
	
	    	gpio_pull_updown(pdata->batt_low_pin, GPIOPullUp); 
	    	ret = gpio_direction_input(pdata->batt_low_pin);
	    	if (ret) {
	    		DBG("failed to set gpio batt_low_pin input\n");
	    		goto error;
	    	}
	}	
	
	//charge led pin
	if( pdata->charge_led_pin != INVALID_GPIO){
 		ret = gpio_request(pdata->charge_led_pin, NULL);
	    	if (ret) {
	    		DBG("failed to request charge_led_pin gpio\n");
	    		goto error;
	    	}
	
	    	//gpio_pull_updown(pdata->charge_led_pin, GPIOPullUp); 
	    	gpio_direction_output(pdata->charge_led_pin,!(pdata->charge_led_level));
	    	gpio_set_value(pdata->charge_led_pin,!(pdata->charge_led_level));
	}	
    
	return 0;
error:
	return -1;
}

//extern void kernel_power_off(void);
static void rk30_adc_battery_check(struct rk30_adc_battery_data *bat)
{
	int i;
	int level,oldlevel;
	struct rk30_adc_battery_platform_data *pdata = bat->pdata;
    //DBG("%s--%d:\n",__FUNCTION__,__LINE__);

	bat->old_charge_level = -1;
	bat->capacitytmp = 0;
	bat->suspend_capacity = 0;
	
	pSamples = bat->adc_samples;

	adc_sync_read(bat->client);                             //start adc sample
	level = oldlevel = rk30_adc_battery_status_samples(bat);//init charge status

	bat->full_times = 0;
	for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++){                //0.3 s
		mdelay(1);
		rk30_adc_battery_voltage_samples(bat);              //get voltage
		level = rk30_adc_battery_get_charge_level(bat);

		if (oldlevel != level){
			oldlevel = level;                               //if charge status changed, reset sample
			i = 0;
		}        
	}

	bat->bat_capacity = rk30_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);  //init bat_capacity
	
	bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (rk30_adc_battery_get_charge_level(bat)){
		bat->bat_status =(bat->bat_capacity < 100) ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_FULL;
	}

#if 1
	rk30_adc_battery_poweron_capacity_check();
#else
	gBatteryData->poweron_check = 1;
#endif
	gBatteryData->poweron_check = 0;

/*******************************************
//开机采样到的电压和上次关机保存电压相差较大，怎么处理？
if (bat->bat_capacity > old_capacity)
{
if ((bat->bat_capacity - old_capacity) > 20)
{

}
}
else if (bat->bat_capacity < old_capacity)
{
if ((old_capacity > bat->bat_capacity) > 20)
{

}
}
*********************************************/
	if (bat->bat_capacity == 0) bat->bat_capacity = 1;


//DBG("bat->bat_voltage =%d=bat->bat_status = %d \n",bat->bat_voltage,bat->bat_status );
#if 0
	if ((bat->bat_voltage <= batt_table[0].dis_charge_vol+ 50)&&(bat->bat_status != POWER_SUPPLY_STATUS_CHARGING)){
		kernel_power_off();
	}
#endif
}

static void rk30_adc_battery_callback(struct adc_client *client, void *param, int result)
{
#if 0
	struct rk30_adc_battery_data  *info = container_of(client, struct rk30_adc_battery_data,
		client);
	info->adc_val = result;
#endif
	gBatteryData->adc_val = result;
	return;
}

#if 1
static irqreturn_t rk30_adc_battery_low_wakeup(int irq,void *dev_id)
{

	schedule_work(&gBatteryData->lowerpower_work);	
	return IRQ_HANDLED;
}

static void rk30_adc_battery_lowerpower_delaywork(struct work_struct *work)
{
	int ret;
	struct rk30_adc_battery_platform_data *pdata;
	struct batt_vol_cal *p;
	
	int irq;
	int irq_flag;
	
	p = adc_battery_table;

	pdata    = gBatteryData->pdata;
	irq        = gpio_to_irq(pdata->batt_low_pin);
	irq_flag = gpio_get_value (pdata->batt_low_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
		
	if (rk30_adc_battery_get_charge_level(gBatteryData))	//charge ok and ajust the adc_battery_table it is very valueable.
	{ 		
		return;	
	}
	else
	{
	    if(adc_to_voltage(gBatteryData->adc_val) > adc_battery_table[1].dis_charge_vol )
	    {
	    		return;	
	    }
	    else
	    {
			gBatteryData->bat_capacity = 0;
			gBatteryData->bat_change  = 1;			    	
	    }
	}
	
	rk28_send_wakeup_key(); // wake up the system

	free_irq(irq, NULL);
	ret = request_irq(irq, rk30_adc_battery_low_wakeup, irq_flag, "batt_low_irq", NULL);
	if (ret) {
		free_irq(irq, NULL);
	}
	
	return;	
}


#endif

static void rk30_adc_battery_update_capcity_on_android_boot()
{
	launch_update_batt_in_boot = true;
}

static int only_one = 0;
static ssize_t sc_zpplication_startget(struct device *dev,struct device_attribute *attr, const char *buf, size_t len)
{
	printk("\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$-->Androd Start To Get Battery Capacity Ready\n");
	if (only_one != 0)
	{
		return len;
	}
	
	only_one = 1;

	if((*buf=='A'))
	{
		printk("update_capcity_on_android_boot\n");
		rk30_adc_battery_update_capcity_on_android_boot();
	}
	printk("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$-->Androd Start To Get Battery Capacity End\n");
	return len;

}
static DEVICE_ATTR(startget,0777,NULL,sc_zpplication_startget);

static int rk30_adc_battery_probe(struct platform_device *pdev)
{
	int    ret;
	int    irq;
	int    irq_flag;
	struct adc_client                   *client;
	struct rk30_adc_battery_data          *data;
	struct rk30_adc_battery_platform_data *pdata = pdev->dev.platform_data;
	
//mia	DBG_Trace("Battery-charge-->%s--Ready!!",__func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	gBatteryData = data;
	
	platform_set_drvdata(pdev, data);
	
	device_create_file(&(pdev->dev), &dev_attr_startget);
	b_rk30_charged = false;

	data->pdata = pdata;
	 
	ret = rk30_adc_battery_io_init(pdata);
	 if (ret) {
	 	goto err_io_init;
	}
    
	memset(data->adc_samples, 0, sizeof(int)*(NUM_VOLTAGE_SAMPLE + 2));

	 //register adc for battery sample
	client = adc_register(0, rk30_adc_battery_callback, NULL);  //pdata->adc_channel = ani0
	if(!client)
		goto err_adc_register_failed;
	    
	 //variable init
	data->client  = client;
	data->adc_val = adc_sync_read(client);

	ret = power_supply_register(&pdev->dev, &rk30_battery_supply);
	if (ret){
		DBG("fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
		

#if defined (CONFIG_BATTERY_RK30_USB_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk30_usb_supply);
	if (ret){
		DBG("fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
 	wake_lock_init(&batt_wake_lock, WAKE_LOCK_SUSPEND, "batt_lock");	

	data->wq = create_singlethread_workqueue("adc_battd");
	INIT_DELAYED_WORK(&data->delay_work, rk30_adc_battery_timer_work);
	//Power on Battery detect
	rk30_adc_battery_check(data);
	queue_delayed_work(data->wq, &data->delay_work, msecs_to_jiffies(TIMER_MS_COUNTS));

#if  defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk30_ac_supply);
	if (ret) {
		DBG("fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
	//init dc dectet irq & delay work
	if (pdata->dc_det_pin != INVALID_GPIO){
		INIT_WORK(&data->dcwakeup_work, rk30_adc_battery_dcdet_delaywork);
		
		irq = gpio_to_irq(pdata->dc_det_pin);	        
		irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    	ret = request_irq(irq, rk30_adc_battery_dc_wakeup, irq_flag, "ac_charge_irq", NULL);
    	if (ret) {
    		DBG("failed to request dc det irq\n");
    		goto err_dcirq_failed;
    	}
    	enable_irq_wake(irq);  
	}
#endif

	// batt low irq lowerpower_work
	if( pdata->batt_low_pin != INVALID_GPIO){
		INIT_WORK(&data->lowerpower_work, rk30_adc_battery_lowerpower_delaywork);
		
		irq = gpio_to_irq(pdata->batt_low_pin);
	    	ret = request_irq(irq, rk30_adc_battery_low_wakeup, IRQF_TRIGGER_LOW, "batt_low_irq", NULL);

	    	if (ret) {
	    		DBG("failed to request batt_low_irq irq\n");
	    		goto err_lowpowerirq_failed;
	    	}
	    	enable_irq_wake(irq);
    	}


//mia	DBG_Trace("Battery-charge-->%s--OK!!",__func__);
	
	return 0;
	
#if defined (CONFIG_BATTERY_RK30_USB_CHARGE)
err_usb_failed:
	power_supply_unregister(&rk30_usb_supply);
#endif

err_ac_failed:
#if defined (CONFIG_BATTERY_RK30_AC_CHARGE)
	power_supply_unregister(&rk30_ac_supply);
#endif

err_battery_failed:
	power_supply_unregister(&rk30_battery_supply);
    
err_dcirq_failed:
	free_irq(gpio_to_irq(pdata->dc_det_pin), data);
 err_lowpowerirq_failed:
	if( pdata->batt_low_pin != INVALID_GPIO){ 	
		free_irq(gpio_to_irq(pdata->batt_low_pin), data);
	}

err_adc_register_failed:
err_io_init:    
err_data_alloc_failed:
	kfree(data);

	DBG("rk30_adc_battery: error!\n");
    
	return ret;
}

static int rk30_adc_battery_remove(struct platform_device *pdev)
{
	struct rk30_adc_battery_data *data = platform_get_drvdata(pdev);
	struct rk30_adc_battery_platform_data *pdata = pdev->dev.platform_data;

	cancel_delayed_work(&gBatteryData->delay_work);	
#if defined(CONFIG_BATTERY_RK30_USB_CHARGE)
	power_supply_unregister(&rk30_usb_supply);
#endif
#if defined(CONFIG_BATTERY_RK30_AC_CHARGE)
	power_supply_unregister(&rk30_ac_supply);
#endif
	power_supply_unregister(&rk30_battery_supply);
	
	device_remove_file(&(pdev->dev),&dev_attr_startget);

    //free gpio
	free_irq(gpio_to_irq(pdata->dc_det_pin), data);

	kfree(data);
	
	return 0;
}

static struct platform_driver rk30_adc_battery_driver = {
	.probe		= rk30_adc_battery_probe,
	.remove		= rk30_adc_battery_remove,
	.suspend    = rk30_adc_battery_suspend,
	.resume		= rk30_adc_battery_resume,
	.driver = {
		.name = "rk30-battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk30_adc_battery_init(void)
{
	return platform_driver_register(&rk30_adc_battery_driver);
	
}

static void __exit rk30_adc_battery_exit(void)
{
	platform_driver_unregister(&rk30_adc_battery_driver);
}

subsys_initcall(rk30_adc_battery_init);
//fs_initcall(rk30_adc_battery_init);
module_exit(rk30_adc_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk30");
MODULE_AUTHOR("jeki.zeng@sc-digital.cn");
MODULE_LICENSE("GPL");
