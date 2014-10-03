#include <linux/hdmi.h>
#include <linux/tchip_sysinf.h>


void codec_set_spk(bool on) 
{
	/* please add sound switching-related code here or on your codec driver
	   parameter: on=1 ==> open spk 
				  on=0 ==> close spk
	*/
}

void hdmi_set_spk(int on)
{
#if defined(CONFIG_SND_RK29_SOC_RT5621)
	if (tcsi_get_value(TCSI_CODEC_RT5621)){			// rt5621
		extern void rt5621_codec_set_spk(bool on);
		rt5621_codec_set_spk(on);
	}else
#endif
#if defined(CONFIG_SND_RK29_SOC_RT5631)
	if (tcsi_get_value(TCSI_CODEC_RT5631)){			// rt5632
		extern void rt5631_codec_set_spk(bool on);
		rt5631_codec_set_spk(on);
	}else
#endif
#if defined(CONFIG_SND_RK29_SOC_WM8900)
	if (tcsi_get_value(TCSI_CODEC_WM8900)){	// wm8900
		extern void wm8900_codec_set_spk(bool on);
		wm8900_codec_set_spk(on);
	}else
#endif
		codec_set_spk(on);
}
