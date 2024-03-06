/**************************************************************************
*  AW2028_LED.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2019/09/18
**************************************************************************/

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>

#include "cust_gpio_usage.h"
#include <cust_eint.h>
#include <mach/mt_gpio.h>

#define AW2028_LED_NAME                "AW2028_LED"
#define AW2028_I2C_BUS		2
#define AW2028_I2C_ADDR		0x65

typedef enum
{
	AW2028_LED_ALWAYS_OFF,// 0
	AW2028_LED_ALWAYS_ON,// 1
	AW2028_LED_BLINK_BREATH,// 2
	AW2028_LED_BRIGHT_FIX_FREQ_FIX,// 3
	AW2028_LED_BRIGHT_FIX_FREQ_MUSIC,// 4
	AW2028_LED_BRIGHT_MUSIC_FREQ_FIX,// 5
	AW2028_LED_BRIGHT_AP_TIMER_MODE,// 6
	AW2028_LED_BRIGHT_IMAX_SET,// 7
	AW2028_LED_BRIGHT_AP_RUNNING_MODE,// 8
}LED_MODE;

static int g_imax = 0x3;//0x0:9MA 0x1:18MA 0x2:37MA 0x3:75MA

static int isGpioSet=0;

static unsigned int g_databuf[20]={0};

static ssize_t AW2028_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW2028_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t AW2028_Audio_led_get_by_node(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t AW2028_Audio_led_set_by_node(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DEVICE_ATTR(reg, 0660, AW2028_get_reg,  AW2028_set_reg);
static DEVICE_ATTR(led, 0660, AW2028_Audio_led_get_by_node,  AW2028_Audio_led_set_by_node);

struct i2c_client *AW2028_led_client;

static struct work_struct rgb_work;
static unsigned int sleep_timer = 1000;
static bool rgb_work_test_stop = true;
static bool rgb_work_is_stop = true;
static struct mutex led_mutex;
static wait_queue_head_t wait_rgb_work_stop;

//////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////
unsigned char I2C_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};
        
	wdbuf[0] = addr;
	wdbuf[1] = reg_data;
        if(NULL==AW2028_led_client)
          {
          pr_err("AW2028_led_client is NULL");
          return -1 ;
          }
	struct i2c_msg msgs[] = {
		{
			.addr	= AW2028_led_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	ret = i2c_transfer(AW2028_led_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

unsigned char I2C_read_reg(unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf[512] = {0};

	rdbuf[0] = addr;
        if(NULL==AW2028_led_client)
          {
          pr_err("AW2028_led_client is NULL");
          return -1 ;
          }
	struct i2c_msg msgs[] = {
		{
			.addr	= AW2028_led_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= AW2028_led_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	ret = i2c_transfer(AW2028_led_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AW2028 LED 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AW2028_LED_Test(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, g_imax);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	
	I2C_write_reg(0x10, 0xFF);		// ILED1
	I2C_write_reg(0x11, 0xFF);		// ILED2
	I2C_write_reg(0x12, 0xFF);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
}

// AW2028 LED Mode: True Color Mode
void AW2028_LED_Breath(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, g_imax);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, 0xFF);		// ILED1
	I2C_write_reg(0x11, 0xFF);		// ILED2
	I2C_write_reg(0x12, 0xFF);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, 0x80);		// PAT_T1		Trise & Ton
	I2C_write_reg(0x31, 0x86);		// PAT_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x00);		// PAT_T3				Tdelay
	I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN

}

unsigned char ms2timer(unsigned int time)
{
	unsigned char i, ret;
	unsigned int ref[16] = {4, 128, 256, 384, 512, 762, 1024, 1524, 2048, 2560, 3072, 4096, 5120, 6144, 7168, 8192};
	
	for(i=0; i<15; i++)
	{
		if(time <= ref[0])
		{
			return 0;
		}
		else if(time > ref[15])
		{
			return 15;
		}
		else if((time>ref[i]) && (time<=ref[i+1]))
		{
			if((time-ref[i]) <= (ref[i+1]-time))
			{
				return i;
			}
			else
			{
				return (i+1);
			}
		}
	}
}	

unsigned char AW2028_LED_ON(unsigned char r, unsigned char g, unsigned char b)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, g_imax);		// IMAX
	I2C_write_reg(0x04, 0x00);		// LCFG1
	I2C_write_reg(0x05, 0x00);		// LCFG2
	I2C_write_reg(0x06, 0x00);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	
	I2C_write_reg(0x10, r);		// ILED1 green
	I2C_write_reg(0x11, g);		// ILED2 blue
	I2C_write_reg(0x12, b);		// ILED3 red
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	return 0;
}

unsigned char AW2028_LED_OFF(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset	
	return 0;
}

unsigned char AW2028_LED_GPIO_ON(void)
{
	if (isGpioSet){
		mt_set_gpio_mode(GPIO_FLASH_LED_EN, GPIO_FLASH_LED_EN_M_GPIO);
		mt_set_gpio_dir(GPIO_FLASH_LED_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_FLASH_LED_EN, 0);
		isGpioSet = 0;
	}
	return 0;
}

unsigned char AW2028_LED_GPIO_OFF(void)
{
	if (!isGpioSet){
		mt_set_gpio_mode(GPIO_FLASH_LED_EN, GPIO_FLASH_LED_EN_M_GPIO);
		mt_set_gpio_dir(GPIO_FLASH_LED_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_FLASH_LED_EN, 1);
		isGpioSet = 1;
	}
	return 0;
}

unsigned char AW2028_Blink(unsigned char r, unsigned char g, unsigned char b, unsigned int trise_ms, unsigned int ton_ms, unsigned int tfall_ms, unsigned int toff_ms)
{
	unsigned char trise, ton, tfall, toff;
	
	trise = ms2timer(trise_ms);
	ton   = ms2timer(ton_ms);
	tfall = ms2timer(tfall_ms);
	toff  = ms2timer(toff_ms);

	printk("debug %d %d %d %d\n", trise, ton, tfall, toff);
	
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, g_imax);		// IMAX
	I2C_write_reg(0x04, 0x01);		// LCFG1
	I2C_write_reg(0x05, 0x01);		// LCFG2
	I2C_write_reg(0x06, 0x01);		// LCFG3
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x08);		// LEDCTR
	
	I2C_write_reg(0x10, r);		// ILED1
	I2C_write_reg(0x11, g);		// ILED2
	I2C_write_reg(0x12, b);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3	
	
	I2C_write_reg(0x30, (trise<<4)|ton);		// PAT_T1		Trise & Ton
	I2C_write_reg(0x31, (tfall<<4)|toff);		// PAT_T2		Tfall & Toff
	I2C_write_reg(0x32, 0x00);		// PAT_T3				Tdelay
	I2C_write_reg(0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
	I2C_write_reg(0x34, 0x00);		// PAT_T5		    Timer
	
	I2C_write_reg(0x09, 0x07);		// PAT_RIN	
	return 0;
}

unsigned char AW2028_Audio_Corss_Zero(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, g_imax);		// IMAX
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x10, 0xFF);		// ILED1
	I2C_write_reg(0x11, 0xFF);		// ILED2
	I2C_write_reg(0x12, 0xFF);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3
	
	I2C_write_reg(0x40, 0x11);		// AUDIO_CTR
	I2C_write_reg(0x41, 0x07);		// AUDIO_LEDEN
	I2C_write_reg(0x42, 0x00);		// AUDIO_FLT
	I2C_write_reg(0x43, 0x1A);		// AGC_GAIN
	I2C_write_reg(0x44, 0x1F);		// GAIN_MAX
	I2C_write_reg(0x45, 0x3D);		// AGC_CFG
	I2C_write_reg(0x46, 0x14);		// ATTH
	I2C_write_reg(0x47, 0x0A);		// RLTH
	I2C_write_reg(0x48, 0x00);		// NOISE
	I2C_write_reg(0x49, 0x02);		// TIMER
	I2C_write_reg(0x40, 0x13);		// AUDIO_CTR

	return 0;
}

unsigned char AW2028_Audio_Timer(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, g_imax);		// IMAX
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x10, 0xFF);		// ILED1
	I2C_write_reg(0x11, 0xFF);		// ILED2
	I2C_write_reg(0x12, 0xFF);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3
	
	I2C_write_reg(0x40, 0x11);		// AUDIO_CTR
	I2C_write_reg(0x41, 0x07);		// AUDIO_LEDEN
	I2C_write_reg(0x42, 0x00);		// AUDIO_FLT
	I2C_write_reg(0x43, 0x1A);		// AGC_GAIN
	I2C_write_reg(0x44, 0x1F);		// GAIN_MAX
	I2C_write_reg(0x45, 0x3D);		// AGC_CFG
	I2C_write_reg(0x46, 0x14);		// ATTH
	I2C_write_reg(0x47, 0x0A);		// RLTH
	I2C_write_reg(0x48, 0x00);		// NOISE
	I2C_write_reg(0x49, 0x00);		// TIMER
	I2C_write_reg(0x40, 0x0B);		// AUDIO_CTR
	
	return 0;
}


unsigned char AW2028_Audio_Single_Color(unsigned char r, unsigned char g, unsigned char b)
{
	I2C_write_reg(0x00, 0x55);                // software reset
	I2C_write_reg(0x01, 0x01);                // GCR
	I2C_write_reg(0x03, g_imax);                // IMAX
	I2C_write_reg(0x07, 0x07);                // LEDEN
	I2C_write_reg(0x10, 0xFF);                // ILED1
	I2C_write_reg(0x11, 0xFF);                // ILED2
	I2C_write_reg(0x12, 0xFF);                // ILED3                                                   
	I2C_write_reg(0x1C, r   );           // PWM1
	I2C_write_reg(0x1D, g   );           // PWM2
	I2C_write_reg(0x1E, b   );           // PWM3
	I2C_write_reg(0x40, 0x01);                // AUDIO_CTR
	I2C_write_reg(0x41, 0x07);                // AUDIO_LEDEN
	I2C_write_reg(0x42, 0x00);                // AUDIO_FLT
	I2C_write_reg(0x43, 0x1A);                // AGC_GAIN
	I2C_write_reg(0x44, 0x1F);                // GAIN_MAX
	I2C_write_reg(0x45, 0x3D);                // AGC_CFG
	I2C_write_reg(0x46, 0x14);                // ATTH
	I2C_write_reg(0x47, 0x0A);                // RLTH
	I2C_write_reg(0x48, 0x00);                // NOISE
	I2C_write_reg(0x49, 0x00);                // TIMER
	I2C_write_reg(0x40, 0x03);                // AUDIO_CTR
	return 0;
}

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
#include <mach/mt_boot_common.h>
extern BOOTMODE g_boot_mode;
#endif

int AW2028_led_set(unsigned char r, unsigned char g, unsigned char b, unsigned int ton_ms, unsigned int toff_ms)
{
	if (ton_ms || toff_ms){
		printk("AW2028_Blink: %d %d\n", ton_ms, toff_ms);
		g_imax = 0x0;
		if (((g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) ||\
			(g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)) && \
			(r)) {
			AW2028_Blink(r, g, b, ton_ms, ton_ms, ton_ms, toff_ms);
		} else {
			AW2028_Blink(r, g, b, 0, ton_ms, 0, toff_ms);
		}
		g_imax = 0x3;
	} else if (r || g || b) {
		printk("AW2028_LED_ON r g b : %d %d %d\n", r, g, b);
		g_imax = 0x0;
		AW2028_LED_ON(r, g, b);
		g_imax = 0x3;
	} else {
		printk("AW2028_LED_OFF r g b\n");
		AW2028_LED_OFF();
	}
	AW2028_LED_GPIO_OFF();
	return 0;
}

static void rgb_work_test(struct work_struct *work)
{
	unsigned int index = 1;
	unsigned char r = 0, g = 0, b = 0;

	I2C_write_reg(0x00, 0x55);                // software reset
	I2C_write_reg(0x01, 0x01);                // GCR
	I2C_write_reg(0x03, g_imax);                // IMAX
	I2C_write_reg(0x07, 0x07);                // LEDEN
	I2C_write_reg(0x10, 0xFF);                // ILED1
	I2C_write_reg(0x11, 0xFF);                // ILED2
	I2C_write_reg(0x12, 0xFF);                // ILED3                                                   
	I2C_write_reg(0x40, 0x01);                // AUDIO_CTR
	I2C_write_reg(0x41, 0x07);                // AUDIO_LEDEN
	I2C_write_reg(0x42, 0x00);                // AUDIO_FLT
	I2C_write_reg(0x43, 0x1A);                // AGC_GAIN
	I2C_write_reg(0x44, 0x1F);                // GAIN_MAX
	I2C_write_reg(0x45, 0x3D);                // AGC_CFG
	I2C_write_reg(0x46, 0x14);                // ATTH
	I2C_write_reg(0x47, 0x0A);                // RLTH
	I2C_write_reg(0x48, 0x00);                // NOISE
	I2C_write_reg(0x49, 0x00);                // TIMER
	I2C_write_reg(0x40, 0x03);                // AUDIO_CTR

	printk("%s: AW2028 start requested: %d\n", __func__,rgb_work_test_stop);

	while (1) {
		if (rgb_work_test_stop) {
			printk("%s: AW2028 stop requested: %d\n", __func__,rgb_work_test_stop);
			I2C_write_reg(0x00, 0x55);		// software reset
			rgb_work_is_stop = true;
			wake_up_interruptible(&wait_rgb_work_stop);
			break;
		}
		switch (index) {
			case 1:
				r = 255;
				g = 0;
				b = 0;
				break;
			case 2:
				r = 0;
				g = 255;
				b = 255;
				break;
			case 3:
				r = 255;
				g = 0;
				b = 255;
				break;
			case 4:
				r = 0;
				g = 255;
				b = 0;
				break;
			case 5:
				r = 0;
				g = 0;
				b = 255;
				index = 0;
				break;
		}
		index++;
		//printk("%s: AW2028 [%d][%d][%d][%d][%d]\n", __func__, index, r, g, b, sleep_timer);

		I2C_write_reg(0x1C, r);			// PWM1
		I2C_write_reg(0x1D, g); 		  // PWM2
		I2C_write_reg(0x1E, b); 		  // PWM3
		msleep(sleep_timer);
	}

}

unsigned char AW2028_Test(void)
{
	I2C_write_reg(0x00, 0x55);		// software reset

	I2C_write_reg(0x01, 0x01);		// GCR
	I2C_write_reg(0x03, 0x01);		// IMAX
	
	I2C_write_reg(0x04, 0x01);		// BREATH
	I2C_write_reg(0x05, 0x01);		// BREATH
	I2C_write_reg(0x06, 0x01);		// BREATH 
	I2C_write_reg(0x07, 0x07);		// LEDEN
	I2C_write_reg(0x08, 0x00);		// ASYNC

	I2C_write_reg(0x10, 0xFF);		// ILED1
	I2C_write_reg(0x11, 0xFF);		// ILED2
	I2C_write_reg(0x12, 0xFF);		// ILED3
	I2C_write_reg(0x1C, 0xFF);		// PWM1
	I2C_write_reg(0x1D, 0xFF);		// PWM2
	I2C_write_reg(0x1E, 0xFF);		// PWM3
	
	I2C_write_reg(0x30, 0x80);		// PAT1_T1
	I2C_write_reg(0x31, 0x86);		// PAT1_T2
	I2C_write_reg(0x32, 0x00);		// PAT1_T3
	I2C_write_reg(0x33, 0x00);		// PAT1_T4
	I2C_write_reg(0x34, 0x00);		// PAT1_T5

	I2C_write_reg(0x35, 0x80);		// PAT2_T1
	I2C_write_reg(0x36, 0x86);		// PAT2_T2
	I2C_write_reg(0x37, 0x00);		// PAT2_T3
	I2C_write_reg(0x38, 0x00);		// PAT2_T4
	I2C_write_reg(0x39, 0x00);		// PAT2_T5
	
	I2C_write_reg(0x3A, 0x80);		// PAT3_T1
	I2C_write_reg(0x3B, 0x86);		// PAT3_T2
	I2C_write_reg(0x3C, 0x00);		// PAT3_T3
	I2C_write_reg(0x3D, 0x00);		// PAT3_T4
	I2C_write_reg(0x3E, 0x00);		// PAT3_T5
	
	I2C_write_reg(0x09, 0x07);		// PAT RUN
	
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t AW2028_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	ssize_t len = 0;
	u8 i;
	for(i=0;i<0x4C;i++)
	{
		reg_val = I2C_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i,reg_val);
	}

	return len;
}

static ssize_t AW2028_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		I2C_write_reg((u8)databuf[0],databuf[1]);
	}
	return len;
}

static ssize_t AW2028_Audio_led_get_by_node(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t len = 0;

	len = sprintf(buf, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", g_databuf[0], g_databuf[1], g_databuf[2], g_databuf[3], g_databuf[4], g_databuf[5], g_databuf[6], g_databuf[7]);

	return len;
}

static ssize_t AW2028_Audio_led_set_by_node(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[20] = {0};
	int timeout_result;
	long timeout = msecs_to_jiffies(1500);   /* 1500ms */

	sscanf(buf, "%d %d %d %d %d %d %d %d", &databuf[0], &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5], &databuf[6], &databuf[7]);

	memcpy(g_databuf, databuf, sizeof(databuf));

	printk("AW2028 led set is %d %d %d %d %d %d %d %d\n", databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5], databuf[6], databuf[7]);

	if (databuf[0] == AW2028_LED_ALWAYS_OFF) {//OFF
		if (databuf[1] == 0) {
			AW2028_LED_OFF();
		} else if (databuf[1] == 1) {
			AW2028_LED_GPIO_OFF();
		}
	} else if (databuf[0] == AW2028_LED_ALWAYS_ON) {//ON
		if (databuf[1] == 0) {
			AW2028_LED_ON(databuf[2],databuf[3],databuf[4]);
		} else if (databuf[1] == 1) {
			AW2028_LED_GPIO_ON();
		}
	} else if (databuf[0] == AW2028_LED_BLINK_BREATH) {//BLINK
		AW2028_Blink(databuf[1], databuf[2], databuf[3], databuf[4], databuf[5], databuf[6], databuf[7]);
	} else if (databuf[0] == AW2028_LED_BRIGHT_FIX_FREQ_FIX) {//AUDIO ZERO
		AW2028_Audio_Corss_Zero();
	} else if (databuf[0] == AW2028_LED_BRIGHT_FIX_FREQ_MUSIC) {//AUDIO TIME
		AW2028_Audio_Timer();
	} else if (databuf[0] == AW2028_LED_BRIGHT_MUSIC_FREQ_FIX) {// single led blink with music !!!
		AW2028_Audio_Single_Color(databuf[1], databuf[2], databuf[3]);
	} else if (databuf[0] == AW2028_LED_BRIGHT_AP_TIMER_MODE) {
		if (databuf[1]  == 0) {
			if (!rgb_work_test_stop) {
				rgb_work_test_stop = true;
				rgb_work_is_stop = false;
				timeout_result = wait_event_interruptible_timeout(wait_rgb_work_stop, rgb_work_is_stop, timeout);
				printk("%s: AW2028 timeout_result: %d\n", __func__,timeout_result);
			}
		} else if (databuf[1]  == 1) {
			if (rgb_work_test_stop) {
				rgb_work_test_stop = false;
				schedule_work(&rgb_work);
			}
		}
	} else if (databuf[0] == AW2028_LED_BRIGHT_IMAX_SET) {
		g_imax = databuf[1];
	} else if (databuf[0] == AW2028_LED_BRIGHT_AP_RUNNING_MODE) {
		AW2028_Test();
	}
	return len;
}

static int AW2028_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err = device_create_file(dev, &dev_attr_led);
	return err;
}

static void AW2028_shutdown(struct platform_device *dev)
{
	AW2028_LED_OFF();
	printk("******** AW2028 driver shutdown!! ********\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int AW2028_LED_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char reg_value;
	int err = 0;
        int i =5;//workround
	printk("AW2028_LED_Probe\n");
          
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	AW2028_led_client = client;
//brian-add-workround-start
        while (i > 0)
        {
             msleep(20);
             reg_value = I2C_read_reg(0x00);
             if(reg_value == 0xB1)
	     {
	       break;
	     }
             printk("AW2028 CHIPID=0x%2x\n", reg_value);
             i--;
        }
          if(!i)
            {   err = -ENODEV;
		goto exit_create_singlethread;
            }
//brian-add-workround-end
	AW2028_create_sysfs(client);	

	INIT_WORK(&rgb_work, rgb_work_test);

	mutex_init(&led_mutex);

	init_waitqueue_head(&wait_rgb_work_stop);

	//AW2028_LED_Test();
	
	return 0;

exit_create_singlethread:
	AW2028_led_client = NULL;
exit_check_functionality_failed:
	return err;	
}

static int AW2028_LED_remove(struct i2c_client *client)
{
	AW2028_led_client = NULL;
	return 0;
}

static const struct i2c_device_id AW2028_i2c_id[] = {
	{ AW2028_LED_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, AW2028_i2c_id);

static struct i2c_board_info __initdata AW2028_i2c_led={ I2C_BOARD_INFO(AW2028_LED_NAME, AW2028_I2C_ADDR)};

static struct i2c_driver AW2028_led_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = AW2028_LED_NAME,
        },

        .probe          = AW2028_LED_probe,
        .remove         = AW2028_LED_remove,
        .id_table       = AW2028_i2c_id,
};

static int __init AW2028_LED_init(void) {
	int ret;
	printk("AW2028 LED Init\n");
	
	i2c_register_board_info(AW2028_I2C_BUS, &AW2028_i2c_led, 1);
		 
	ret = i2c_add_driver(&AW2028_led_driver);
	return 0;
}

static void __exit AW2028_LED_exit(void) {
	printk("AW2028 LED Exit\n");
	i2c_del_driver(&AW2028_led_driver);
}

module_init(AW2028_LED_init);
module_exit(AW2028_LED_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW2028 LED driver");
MODULE_LICENSE("GPL");


