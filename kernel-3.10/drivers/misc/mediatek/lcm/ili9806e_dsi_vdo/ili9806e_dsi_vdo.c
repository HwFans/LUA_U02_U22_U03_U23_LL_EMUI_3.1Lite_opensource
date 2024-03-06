/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/


#ifdef BUILD_LK
#include "platform/mt_gpio.h"
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0
#define LCM_ID_ILI9806    0x9806

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static int bl_Cust_Max = 1023;
static int bl_Cust_Min = 28;

static int setting_max = 1023;
static int setting_min = 40;
#define LCD_ID_ADC_CHANNEL    1
#define LCD_ID_READ_TIMES     3
#define LCD_ID_VALUE_OFFSET    300
#define ADC_ILI9806E_TCL_VALUE    	40
#define  ADC_ILI9806E_TXD_VALUE   4095
static unsigned int lcm_type = 0xFFFF;
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
typedef enum
{
    LCM_TYPE_TCL,
    LCM_TYPE_TXD,

    LCM_TYPE_MAX,
} LCM_TYPE_E;
static void lcm_adc_id(void)
{
    int lcmid_adc = 0, ret_temp = 0, i = 0;
    int data[4] = {0,0,0,0};
    int res =0;
    
    i = LCD_ID_READ_TIMES;
    
    while (i--)
    {
        res = IMM_GetOneChannelValue(LCD_ID_ADC_CHANNEL,data,&ret_temp);
        lcmid_adc += ret_temp;
#ifdef BUILD_LK
        printf("YYYYY 111[%d] = temp:%d,val:%d\n", i, ret_temp, lcmid_adc);
#else
        printk("YYYYY 222[%d] = temp:%d,val:%d\n", i, ret_temp, lcmid_adc);
#endif
    }
    
    lcmid_adc = lcmid_adc/LCD_ID_READ_TIMES;

    if((lcmid_adc > ((int)(ADC_ILI9806E_TCL_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (ADC_ILI9806E_TCL_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TCL;
    }
    else if((lcmid_adc > ((int)(ADC_ILI9806E_TXD_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (ADC_ILI9806E_TXD_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TXD;
    }
    else
    {
        lcm_type = LCM_TYPE_MAX;
    }
    
#ifdef BUILD_LK
    printf("YYYYY 333 = lcm_type:%d\n", lcm_type);
#else
    printk("YYYYY 444 = lcm_type:%d\n", lcm_type);
#endif

    return ;
}
static int lcm_brightness_mapping(int level)
{
   int mapped_level;

   if (level < 20) {
        level = 20;
   }

   if(level >= setting_min){
	 mapped_level = ((bl_Cust_Max-bl_Cust_Min)*level+(setting_max*bl_Cust_Min)-(setting_min*bl_Cust_Max))/(setting_max-setting_min);
   }else{
	 mapped_level = (bl_Cust_Min*level)/setting_min;
   } 

   #ifdef BUILD_LK
   printf("level= %d, lcm_brightness_mapping= %d\n", level, mapped_level);
   #else
   printk("level= %d, lcm_brightness_mapping= %d\n", level, mapped_level);
   #endif

   return mapped_level;
}


static struct LCM_setting_table lcm_initialization_setting_tcl[] = {
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
	
	{0x08,1,{0x10}},
	
	{0x21,1,{0x01}},
	
	{0x30,1,{0x01}},
	
	{0x31,1,{0x02}},
	
	{0x40,1,{0x18}},
	
	{0x41,1,{0x33}},
	
	{0x42,1,{0x02}},
	
	{0x43,1,{0x09}},
	
	{0x44,1,{0x09}},
	
	{0x50,1,{0x80}},
	
	{0x51,1,{0x80}},
	
	{0x52,1,{0x00}},
	
	{0x53,1,{0x48}},
	
	{0x57,1,{0x50}},
	
	{0x60,1,{0x0a}},
	
	{0x61,1,{0x00}},
	
	{0x62,1,{0x08}},
	
	{0x63,1,{0x00}},
	
	{0xa0,1,{0x00}},
	
	{0xa1,1,{0x06}},
	
	{0xa2,1,{0x10}},
	
	{0xa3,1,{0x11}},
	
	{0xa4,1,{0x0a}},
	
	{0xa5,1,{0x19}},
	
	{0xa6,1,{0x09}},
	
	{0xa7,1,{0x08}},
	
	{0xa8,1,{0x04}},
	
	{0xa9,1,{0x0a}},
	
	{0xaa,1,{0x04}},
	
	{0xab,1,{0x06}},
	
	{0xac,1,{0x0b}},
	
	{0xad,1,{0x35}},
	
	{0xae,1,{0x32}},
	
	{0xaf,1,{0x1f}},
	
	{0xc0,1,{0x00}},
	
	{0xc1,1,{0x05}},
	
	{0xc2,1,{0x10}},
	
	{0xc3,1,{0x10}},
	
	{0xc4,1,{0x08}},
	
	{0xc5,1,{0x15}},
	
	{0xc6,1,{0x0b}},
	
	{0xc7,1,{0x08}},
	
	{0xc8,1,{0x04}},
	
	{0xc9,1,{0x07}},
	
	{0xca,1,{0x08}},
	
	{0xcb,1,{0x03}},
	
	{0xcc,1,{0x0a}},
	
	{0xcd,1,{0x29}},
	
	{0xce,1,{0x24}},
	
	{0xcf,1,{0x1f}},
	
	{0xff,5,{0xff,0x98,0x06,0x04,0x05}},
	
	{0x00,1,{0x01}},
	
	{0xff,5,{0xff,0x98,0x06,0x04,0x06}},
	
	{0x00,1,{0x21}},
	
	{0x01,1,{0x0a}},
	
	{0x02,1,{0x00}},
	
	{0x03,1,{0x00}},
	
	{0x04,1,{0x01}},
	
	{0x05,1,{0x01}},
	
	{0x06,1,{0x80}},
	
	{0x07,1,{0x06}},
	
	{0x08,1,{0x01}},
	
	{0x09,1,{0x80}},
	
	{0x0a,1,{0x00}},
	
	{0x0b,1,{0x00}},
	
	{0x0c,1,{0x0a}},
	
	{0x0d,1,{0x0a}},
	
	{0x0e,1,{0x00}},
	
	{0x0f,1,{0x00}},
	
	{0x10,1,{0xf0}},
	
	{0x11,1,{0xf4}},
	
	{0x12,1,{0x04}},
	
	{0x13,1,{0x00}},
	
	{0x14,1,{0x00}},
	
	{0x15,1,{0xc0}},
	
	{0x16,1,{0x08}},
	
	{0x17,1,{0x00}},
	
	{0x18,1,{0x00}},
	
	{0x19,1,{0x00}},
	
	{0x1a,1,{0x00}},
	
	{0x1b,1,{0x00}},
	
	{0x1c,1,{0x00}},
	
	{0x1d,1,{0x00}},
	
	{0x20,1,{0x01}},
	
	{0x21,1,{0x23}},
	
	{0x22,1,{0x45}},
	
	{0x23,1,{0x67}},
	
	{0x24,1,{0x01}},
	
	{0x25,1,{0x23}},
	
	{0x26,1,{0x45}},
	
	{0x27,1,{0x67}},
	
	{0x30,1,{0x01}},
	
	{0x31,1,{0x11}},
	
	{0x32,1,{0x00}},
	
	{0x33,1,{0xee}},
	
	{0x34,1,{0xff}},
	
	{0x35,1,{0xbb}},
	
	{0x36,1,{0xca}},
	
	{0x37,1,{0xdd}},
	
	{0x38,1,{0xac}},
	
	{0x39,1,{0x76}},
	
	{0x3a,1,{0x67}},
	
	{0x3b,1,{0x22}},
	
	{0x3c,1,{0x22}},
	
	{0x3d,1,{0x22}},
	
	{0x3e,1,{0x22}},
	
	{0x3f,1,{0x22}},
	
	{0x40,1,{0x22}},
	
	{0x52,1,{0x10}},
	
	{0x53,1,{0x10}},
	
	{0xff,5,{0xff,0x98,0x06,0x04,0x07}},
	
	{0x17,1,{0x22}},
	
	{0x02,1,{0x77}},
	
	{0xe1,1,{0x79}},
	{0x06,1,{0x12}},
	
	{0x26,1,{0xb2}},
	
	//****************************** Page 0 Command ******************************//
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x35, 1, {0X00}}, 
	{0x51, 1, {0X00}}, 
	{0x53, 1, {0X24}}, 
	{0x55, 1, {0X00}}, 
	{0x11, 1, {0X00}}, 
	{REGFLAG_DELAY, 120, {}},
	{0x29, 1, {0X00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};
static struct LCM_setting_table lcm_initialization_setting_txd[] = {
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},     // Change to Page 
{0x08,1,{0x10}},
{0x21,1,{0x01}},
{0x30,1,{0x01}},
{0x31,1,{0x02}},
{0x40,1,{0x11}},// BT  +2.5/-2.5 
{0x41,1,{0x77}},// DVDDH DVDDL 
{0x42,1,{0x03}},
{0x43,1,{0x09}},
{0x44,1,{0x07}},// VGH/VGL
{0x50,1,{0x78}},// VGL_REG  -10V 
{0x51,1,{0x80}},                // VGMP
{0x52,1,{0x00}},                // VGMN
{0x53,1,{0x49}},
//{0x56,1,{0x01}},             //OTP loaded remove
{0x57,1,{0x50}},//Flicker4F		73
//{0x57,1,{0x50}},	
{0x60,1,{0x07}},// SDTI
{0x61,1,{0x00}},// CRTI
{0x62,1,{0x08}},// EQTI
{0x63,1,{0x00}},// PCTI
//{0x57,1,{0x50}},//Flicker4F		73	
//++++{++++++++++++++ Gamma Setting ++++++++++++++++++//
{0xA0,1,{0x00}},  // Gamma 0 /255
{0xA1,1,{0x05}},  // Gamma 4 /251
{0xA2,1,{0x0e}}, // Gamma 8 /247
{0xA3,1,{0x0f}},  // Gamma 16	/239
{0xA4,1,{0x08}},  // Gamma 24 /231
{0xA5,1,{0x15}},  // Gamma 52 / 203
{0xA6,1,{0x08}},  // Gamma 80 / 175
{0xA7,1,{0x07}},  // Gamma 108 /147
{0xA8,1,{0x03}},  // Gamma 147 /108
{0xA9,1,{0x07}},  // Gamma 175 / 80
{0xAA,1,{0x0e}},  // Gamma 203 / 52
{0xAB,1,{0x06}},  // Gamma 231 / 24
{0xAC,1,{0x0d}},  // Gamma 239 / 16
{0xAD,1,{0x2b}},  // Gamma 247 / 8
{0xAE,1,{0x25}},  // Gamma 251 / 4
{0xAF,1,{0x00}},  // Gamma 255 / 0
//==={0x==}},=========Nagitive
{0xC0,1,{0x00}},  // Gamma 0 
{0xC1,1,{0x05}},  // Gamma 4
{0xC2,1,{0x0f}},  // Gamma 8
{0xC3,1,{0x10}},  // Gamma 16
{0xC4,1,{0x0a}},  // Gamma 24
{0xC5,1,{0x16}},  // Gamma 52
{0xC6,1,{0x0a}},  // Gamma 80
{0xC7,1,{0x09}},  // Gamma 108
{0xC8,1,{0x04}},  // Gamma 147
{0xC9,1,{0x09}},  // Gamma 175
{0xCA,1,{0x02}},  // Gamma 203
{0xCB,1,{0x04}},  // Gamma 231
{0xCC,1,{0x0c}},  // Gamma 239
{0xCD,1,{0x2c}},  // Gamma 247
{0xCE,1,{0x27}},  // Gamma 251
{0xCF,1,{0x00}},  // Gamma 255
//{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
//{0x18,1,{0x1D}},
//{0x17,1,{0x12}},
//****{************************** Page 6 Command ******************************//
{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
{0x00,1,{0x21}},
{0x01,1,{0x0A}},
{0x02,1,{0x00}},    
{0x03,1,{0x00}},
{0x04,1,{0x01}},
{0x05,1,{0x01}},
{0x06,1,{0x80}},    
{0x07,1,{0x06}},
{0x08,1,{0x01}},
{0x09,1,{0x80}},    
{0x0A,1,{0x00}},    
{0x0B,1,{0x00}},    
{0x0C,1,{0x0A}},
{0x0D,1,{0x0A}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0xF0}},
{0x11,1,{0xF4}},
{0x12,1,{0x04}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0xC0}},
{0x16,1,{0x08}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x20,1,{0x01}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x01}},
{0x25,1,{0x23}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x01}},  //02
{0x31,1,{0x11}},
{0x32,1,{0x00}},
{0x33,1,{0xEE}},
{0x34,1,{0xFF}},
{0x35,1,{0xBB}},
{0x36,1,{0xCA}},
{0x37,1,{0xDD}},
{0x38,1,{0xAC}},
{0x39,1,{0x76}},
{0x3A,1,{0x67}},
{0x3B,1,{0x22}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x3F,1,{0x22}},
{0x40,1,{0x22}},
{0x52,1,{0x10}},
{0x53,1,{0x10}},  //VGLO refer VGL_REG
{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
//{0x18,1,{0x1d}},
{0x17,1,{0x22}},
{0x02,1,{0x77}},
{0xE1,1,{0x79}},
//****************************** Page 0 Command ******************************//
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
{0x21, 1, {0X00}}, 
{0x11, 1, {0X00}}, 
{REGFLAG_DELAY, 120, {}},
{0x29, 1, {0X00}},
{REGFLAG_DELAY, 10, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
     {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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

    params->physical_width  = 62.06;
    params->physical_height = 110.42;
    
    // enable tearing-free
    params->dbi.te_mode				= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   =SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE ;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 0;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=FRAME_HEIGHT;

    params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 16;
    params->dsi.vertical_frontporch					= 20;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 10;
    params->dsi.horizontal_backporch				= 80;
    params->dsi.horizontal_frontporch				= 80;
    params->dsi.horizontal_blanking_pixel			= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    
   // params->dsi.HS_TRAIL = 1;
   // params->dsi.LPX = 3;
	//params->dsi.CLK_TRAIL =1;

#if 0
    // Bit rate calculation
#if 0//def CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2; 	// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2; 	// div2=0,1,2,3;div1_real=1,2,4,4
    params->dsi.fbk_sel=0;		// fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =8; 	// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
    params->dsi.pll_div1=1; 	// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=0; 	// div2=0,1,2,3;div2_real=1,2,4,4
    //params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =16;		// fref=26MHz, fvco=fref*fbk_div*2/(div1_real*div2_real)		
#endif
#endif
       // params->dsi.pll_select=1;
	params->dsi.PLL_CLOCK=200;
 	params->dsi.ssc_range = 8;
	params->dsi.ssc_disable = 0;


}

static void lcm_init_tcl(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
 #ifdef BUILD_LK
   printf("  zz lcm_init_tcl\n");
   #else
   printk("  zz lcm_init_tcl\n");;
   #endif
//   lcm_adc_id();
    push_table(lcm_initialization_setting_tcl, sizeof(lcm_initialization_setting_tcl) / sizeof(struct LCM_setting_table), 1);
}
static void lcm_init_txd(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
//   lcm_adc_id();
 #ifdef BUILD_LK
   printf("  zz lcm_init_txd\n");
   #else
   printk("  zz lcm_init_txd\n");;
   #endif
    push_table(lcm_initialization_setting_txd, sizeof(lcm_initialization_setting_txd) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{


   
  push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
  
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(100);
}


static void lcm_resume_tcl(void)
{
    lcm_init_tcl();
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
   

}
static void lcm_resume_txd(void)
{
    lcm_init_txd();
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
   

}
static unsigned int lcm_compare_id_tcl(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;

        SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	lcm_adc_id();

	array[0]=0x00063902;
	array[1]=0x0698FFFF;// page enable
	array[2]=0x00000104;
	dsi_set_cmdq(array, 3, 1);
	MDELAY(10); 
	array[0]=0x00023700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0x00, buffer, 1);
	id_high = buffer[0];	
	read_reg_v2(0x01, buffer, 1);
	id_low = buffer[0];
	
    id = (id_high<<8) | id_low;
	if ((LCM_ID_ILI9806==id)?1:0)
	  {
			if(LCM_TYPE_TCL== lcm_type)
				  return 1;
			 else
			  return 0;
	  }
	   return 0;
}
static unsigned int lcm_compare_id_txd(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_low=0;
    int id=0;

        SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	lcm_adc_id();

	array[0]=0x00063902;
	array[1]=0x0698FFFF;// page enable
	array[2]=0x00000104;
	dsi_set_cmdq(array, 3, 1);
	MDELAY(10); 
	array[0]=0x00023700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0x00, buffer, 1);
	id_high = buffer[0];	
	read_reg_v2(0x01, buffer, 1);
	id_low = buffer[0];
	
    id = (id_high<<8) | id_low;
	if ((LCM_ID_ILI9806==id)?1:0)
	  {
			if(LCM_TYPE_TXD== lcm_type)
				  return 1;
			 else
			  return 0;
	  }
	   return 0;
}
LCM_DRIVER ili9806e_dsi_vdo_lcm_drv_tcl = 
{
    .name			= "ili9806e_dsi_vdo_tcl",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init_tcl,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume_tcl,
    .compare_id    = lcm_compare_id_tcl,
  //  .esd_check		= lcm_esd_check,
  //  .esd_recover = lcm_esd_recover,
#ifdef CONFIG_WT_BRIGHTNESS_MAPPING_WITH_LCM
    .cust_mapping = lcm_brightness_mapping,
#endif
#if (LCM_DSI_CMD_MODE)
    //.set_backlight	= lcm_setbacklight,
    //.update         = lcm_update,
#endif
};
LCM_DRIVER ili9806e_dsi_vdo_lcm_drv_txd = 
{
    .name			= "ili9806e_dsi_vdo_txd",
      
   .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init_txd,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume_txd,
    .compare_id    = lcm_compare_id_txd,
  //  .esd_check		= lcm_esd_check,
  //  .esd_recover = lcm_esd_recover,
#ifdef CONFIG_WT_BRIGHTNESS_MAPPING_WITH_LCM
    .cust_mapping = lcm_brightness_mapping,
#endif
#if (LCM_DSI_CMD_MODE)
    //.set_backlight	= lcm_setbacklight,
    //.update         = lcm_update,
#endif
};
