/*
 * tchip_devices.c
 *
 *  Created on: 2012-2-17
 *      Author: zhansb
 */

struct tchip_device {
	char		name[20];
	unsigned short	active;
};

#define TCSI_GET_GROUP_INDEX(pos, group) ((pos) - TCSI_##group##_PRESTART - 1)

#define TCSI_GET_CODEC_INDEX(pos) (TCSI_GET_GROUP_INDEX(pos, CODEC))
#define TCSI_GET_WIFI_INDEX(pos) (TCSI_GET_GROUP_INDEX(pos, WIFI))
#define TCSI_GET_HDMI_INDEX(pos) (TCSI_GET_GROUP_INDEX(pos, HDMI))
#define TCSI_GET_MODEM_INDEX(pos) (TCSI_GET_GROUP_INDEX(pos, MODEM))

#define GET_CUR_DEVICE(list)	get_cur_device(list, (sizeof(list)/sizeof(list[0])))
#define GET_DEVICE_LIST(Info, list)	get_device_list(Info, list, (sizeof(list)/sizeof(list[0])))

/*
 * 	board support list
 */
static const struct tchip_device tchip_boards[] =
{
#if defined(CONFIG_TCHIP_MACH_DEFAULT)
	{.name = "TCHIP",.active = 1},
#elif defined(CONFIG_TCHIP_MACH_TR736)
        {   .name = "TR736_HM",.active = 1},
#elif defined(CONFIG_TCHIP_MACH_TR976Q)
        {   .name = "TR976Q",.active = 1},
#elif defined(CONFIG_TCHIP_MACH_TR785)
#if defined(CONFIG_TCHIP_MACH_TR785_V10)
        {   .name = "TR785V10",.active = 1},
#else
        { .name = "TR785", .active = 1 },
#endif  /* CONFIG_TCHIP_MACH_TR785 */
#endif
};

/*
 * 	touch support list
 */
static const struct tchip_device tchip_touchs[] =
{
#ifdef	CONFIG_GOODIX_CAPACITIVE_SIGNAL_IC
	{.name = "GT801",.active = 1},
#elif defined(CONFIG_TOUCHSCREEN_CT36X)
	{.name = "CT365",.active = 1},
#elif defined(CONFIG_GSL1680) && defined(CONFIG_TOUCHSCREEN_GT811)
	{.name = "TP1",.active = 1}, //GT811&GSL1680
#elif defined(CONFIG_TOUCHSCREEN_GT811)
	{.name = "GT811",.active = 1},
#elif defined(CONFIG_GSL1680)
	{.name = "GSL1680",.active = 1},
#endif
};

/*
 * 	encrypt support list
 */
/* not need anymore
static const struct tchip_device tchip_encrypts[] =
{
#ifdef	CONFIG_AT18_DEVICE
	{.name = "AT18",.active = 1},
#elif defined(CONFIG_AT28_DEVICE)
	{.name = "AT28",.active = 1},
#elif defined(CONFIG_AT38_DEVICE)
	{.name = "AT38",.active = 1},
#endif
};
*/

/*
 * 	tp support list
 */
static const struct tchip_device tchip_tps[] =
{
#ifdef	CONFIG_TCHIP_TP_01
	{.name = "TP01",.active = 1},
#elif defined(CONFIG_TCHIP_TP_02)
	{.name = "TP02",.active = 1},
#endif
};

/*
 *  gsensor support list
 */
static const struct tchip_device tchip_misc[] =
{
#if defined(CONFIG_GS_DMARD06) && defined(CONFIG_GS_MMA8452) && defined(CONFIG_GS_MC3230) && defined(CONFIG_GS_MMA7660)
    {.name = "GS1",.active = 1},
#endif
#if defined(CONFIG_MFD_WM831X_I2C) && defined(CONFIG_PMU_AXP152) && defined (CONFIG_MFD_TPS65910)
    {.name = "PMU1",.active = 1},
#endif
};

/*
 * 	modem support list
 */
static const struct tchip_device tchip_modems[] =
{
#ifdef	CONFIG_MODEM_ROCKCHIP_DEMO
	[TCSI_GET_MODEM_INDEX(TCSI_MODEM_OTHERS)] =
	{
		.name = "MODEM",
#ifdef	CONFIG_TCHIP_MACH_MODEM_OTHERS
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_TDM330
	[TCSI_GET_MODEM_INDEX(TCSI_MODEM_TDM330)] =
	{
		.name = "TDM330",
#ifdef	CONFIG_TCHIP_MACH_MODEM_TDM330
		.active = 1,
#endif
	},
#endif
};

/*
 * 	hdmi support list
 */
static const struct tchip_device tchip_hdmis[] =
{
#if	defined(CONFIG_HDMI_RK610)
	[TCSI_GET_HDMI_INDEX(TCSI_HDMI_RK610)] =
	{
		.name = "RK610HDMI",
#if defined(CONFIG_TCHIP_MACH_HDMI_RK610)
		.active = 1,
#endif
	},
#endif
#if	defined(CONFIG_ANX7150) || defined(CONFIG_ANX7150_NEW)
	[TCSI_GET_HDMI_INDEX(TCSI_HDMI_ANX7150)] =
	{
		.name = "ANX7150",
#ifdef	CONFIG_TCHIP_MACH_ANX7150
		.active = 1,
#endif
	},
#endif
#if	defined(CONFIG_CAT6611) || defined(CONFIG_CAT6611_NEW)
	[TCSI_GET_HDMI_INDEX(TCSI_HDMI_CAT6611)] =
	{
		.name = "CAT6611",
#if defined(CONFIG_TCHIP_MACH_CAT6611)
		.active = 1,
#endif
	},
#endif
#if	defined(CONFIG_HDMI_RK30)
	[TCSI_GET_HDMI_INDEX(TCSI_HDMI_RK30)] =
	{
		.name = "RK30HDMI",
#if defined(CONFIG_TCHIP_MACH_HDMI_RK30)
		.active = 1,
#endif
	},
#endif
};

/*
 * 	wifi support list
 */
static const struct tchip_device tchip_wifis[] =
{
#ifdef CONFIG_MT5931_MT6622
     {
         .name = "CDTK25931",
         .active = 1,
     }
#endif
#ifdef	CONFIG_AR6003
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_AR6003)] =
	{
		.name = "AR6302",
#ifdef	CONFIG_TCHIP_MACH_AR6003
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_BCM4329
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_BCM4329)] =
	{
		.name = "B23",
#ifdef	CONFIG_TCHIP_MACH_BCM4329
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_MV8686
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_MV8686)] =
	{
		.name = "MV8686",
#ifdef	CONFIG_TCHIP_MACH_MV8686
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_RTL8192CU
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_RTL8192)] =
	{
		.name = "RTL8188",
#ifdef	CONFIG_TCHIP_MACH_RTL8192
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_RK903
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_RK903)] =
	{
		.name = "RK903",
		.active = 1,
	},
#endif
#ifdef	CONFIG_RT5370V2_STA
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_RT5370)] =
	{
		.name = "RT5370",
		.active = 1,
	},
#endif
#ifdef	CONFIG_RT5370
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_RT5370V2)] =
	{
		.name = "RT5370V2",
		.active = 1,
	},
#endif
#ifdef	CONFIG_MT7601
	[TCSI_GET_WIFI_INDEX(TCSI_WIFI_MT7601)] =
	{
		.name = "MT7601",
		.active = 1,
	},
#endif
};

/*
 * 	codec support list
 */
static const struct tchip_device tchip_codecs[] =
{
#ifdef CONFIG_SND_RK29_SOC_ES8323
    [TCSI_GET_CODEC_INDEX(TCSI_CODEC_ES8323)] =
    {
        .name = "ES8323",
#ifdef  CONFIG_TCHIP_MACH_SND_RK29_SOC_ES8323
        .active = 1,
#endif
    },
#endif
#ifdef	CONFIG_SND_RK29_SOC_RK610
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_RK610)] =
	{
		.name = "RK610CODEC",
#ifdef	CONFIG_TCHIP_MACH_SND_RK29_SOC_RK610
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_SND_RK29_SOC_WM8988
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_WM8988)] =
	{
		.name = "WM8988",
#ifdef	CONFIG_TCHIP_MACH_SND_RK29_SOC_WM8988
		.active = 1,
#endif
	},
#endif

#ifdef	CONFIG_SND_RK29_SOC_WM8900
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_WM8900)] =
	{
		.name = "WM8900",
#ifdef	CONFIG_TCHIP_MACH_SND_RK29_SOC_WM8900
		.active = 1,
#endif
	},
#endif

#ifdef	CONFIG_SND_RK29_SOC_RT5621
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_RT5621)] =
	{
		.name = "RT5621",
#ifdef	CONFIG_TCHIP_MACH_SND_RK29_SOC_RT5621
		.active = 1,
#endif
	},
#endif

#ifdef	CONFIG_SND_RK29_SOC_WM8994
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_WM8994)] =
	{
		.name = "WM8994",
#ifdef	CONFIG_TCHIP_MACH_SND_RK29_SOC_WM8994
		.active = 1,
#endif
	},
#endif

#ifdef	CONFIG_SND_RK29_SOC_RT5631
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_RT5631)] =
	{
		.name = "RT5631",
#ifdef	CONFIG_TCHIP_MACH_SND_RK29_SOC_RT5631
		.active = 1,
#endif
	},
#endif
#ifdef	CONFIG_SND_RK29_SOC_ES8323
	[TCSI_GET_CODEC_INDEX(TCSI_CODEC_ES8323)] =
	{
		.name = "ES8323",
		.active = 1,
	},
#endif
};



struct tchip_device *get_cur_device(struct tchip_device *devices, int size)
{
	int i;

	for(i = 0; i < size; devices++,i++)
	{
		if(devices->active)
		{
			return devices;
		}
	}

	return 0;
}

static char * strupper(char * dst, char *src)
{
	char * start = dst;

	while(*src!='\0')
		*dst++ = toupper(*src++);
	*dst = '\0';

	return start;
}

static void add2versionex(char *version, struct tchip_device *dev, char *prefix)
{
	char str[20];

	strcpy(str, prefix);

	strupper(&str[strlen(prefix)], dev->name);
	strcat(version, str);
}

static void add2version(char *version, struct tchip_device *dev)
{
	add2versionex(version,dev,"_");
}

struct tchip_device *set_all_active_device_version(struct tchip_device *devices, int size, char *version)
{
	int i;

	for(i = 0; i < size; devices++,i++)
	{
		if(devices->active)
		{
			add2version(version, devices);
		}
	}
}

#if 0
void cur_sensor_init(void)
{
	int i;

	cur_sensor[SENSOR_BACK] = cur_sensor[SENSOR_FRONT] = &no_sensor;

	for(i = 0; i < (TCSI_GET_SENSOR_INDEX(TCSI_CAMERA_END)); i++)
	{
		if(sensors[i].active)
		{
			if(TCSI_SENSOR_POS_BACK == sensors[i].pos)
				cur_sensor[SENSOR_BACK] = &sensors[i];
			else if(TCSI_SENSOR_POS_FRONT == sensors[i].pos)
				cur_sensor[SENSOR_FRONT] = &sensors[i];
		}
	}
}
#endif
int get_device_list(char * Info, struct tchip_device *devices, int size)
{
	int i;

	for (i = 0; i < size; devices++, i++)
	{
		if (devices->active)
			Info[i] = 1;

		strncpy(&Info[10 + i * 9], devices->name, 8);

		//printf("-->%9s:%d\n", &Info[10 + i * 9], Info[i]);
	}

	return i;
}
