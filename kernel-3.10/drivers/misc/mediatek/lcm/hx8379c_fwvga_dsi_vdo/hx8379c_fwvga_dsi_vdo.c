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

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
	#define LCM_DEBUG  printf
	#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
	#define LCM_DEBUG  printk
	#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFE   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_HX8379C                                      0x79

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#define LCD_ID_ADC_CHANNEL     12
#define LCD_ID_READ_TIMES      3
#define LCD_ID_VALUE_OFFSET    300

#define ADC_TCL_VALUE    		0
#define ADC_TONGXINGDA_VALUE    3740

typedef enum
{
    LCM_TYPE_TCL = 1,
    LCM_TYPE_TONGXINGDA,
    LCM_TYPE_MAX,
} LCM_TYPE_E;

static unsigned int lcm_type = 0xFFFF;
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

/***********************************************************/

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	{0xB9,3,{0xFF,0x83,0x79}},
	{0xB1,31,{0x00,0x50,0x44,0xea,0x90,0x80,0x11,0x11,0x71,0x2f,0x37,0x9a,0x1a,0x42,0x1b,0x6e,0xf1,0x00,0xe6,0xe6,0xe6,0xe6,0xe6,0x00,0x04,0x05,0x0a,0x0b,0x04,0x05,0x6f}},// Interface Mode Control

    {0xB2,13,{0x00,0x00,0xfe,0x08,0x0c,0x19,0x22,0x00,0xff,0x08,0x0c,0x19,0x20}},
	//old{0xB4,31,{0x80,0x00,0x00,0x32,0x10,0x03,0x32,0x13,0x5f,0x32,0x10,0x08,0x35,0x01,0x28,0x07,0x37,0x00,0x3f,0x08,0x30,0x30,0x04,0x00,0x40,0x08,0x28,0x08,0x30,0x30,0x04}}, 
	
	{0xB4,31,{0x80,0x00,0x00,0x32,0x10,0x03,0x32,0x13,0x5f,0x32,0x10,0x08,0x35,0x01,0x28,0x07,0x37,0x00,0x3f,0x08,0x38,0x30,0x04,0x00,0x40,0x08,0x28,0x08,0x30,0x30,0x04}}, 
	{0xD5,47,{0x00,0x00,0x0A,0x00,0x01,0x05,0x00,0x00,0x18,0x88,0x99,0x88,0x01,0x45,0x88,0x88,0x01,0x45,0x23,0x67,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x99,0x54,0x10,0x88,0x88,0x76,0x32,0x54,0x10,0x88,0x88,0x88,0x88,0x88,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0xDE,3, {0x05,0x70,0x04}}, 

	{0xE0,35,{0x79,0x07,0x12,0x14,0x34,0x36,0x3f,0x42,0x57,0x06,0x12,0x12,0x16,0x18,0x16,0x16,0x12,0x16,0x07,0x12,0x14,0x34,0x36,0x3f,0x42,0x57,0x06,0x12,0x12,0x16,0x18,0x16,0x16,0x12,0x16}},

	{0xB6,5,{0x00,0x74,0x00,0x71,0x00}},

	{0xCC,1,{0x02}},

	{0x11,1,{0x00}},       
	{REGFLAG_DELAY, 120, {}}, 
	{0x29,1,{0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}

};
static void lcm_init_setting_TongXingDa() 
{ 
	unsigned int data_array[16];

	data_array[0] = 0x00043902;                          
	data_array[1] = 0x7983FFB9;                 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(5); 
	data_array[0] = 0x00163902;                           
	data_array[1] = 0x1B1B45B1;
	data_array[2] = 0xD0503131;  
	data_array[3] = 0x388052Ec;
	data_array[4] = 0x2222F838;
	data_array[5] = 0x30800022;
	data_array[6] = 0x00000000;
	dsi_set_cmdq(data_array, 7, 1);    

	data_array[0] = 0x000a3902;          
	data_array[1] = 0x0afe80B2; 
	data_array[2] = 0x11500003;
	data_array[3] = 0x00001d42;                   
	dsi_set_cmdq(data_array, 4, 1); 
	
	data_array[0] = 0x000b3902;         
	data_array[1] = 0x026a02B4; 
	data_array[2] = 0x226a026a;
	data_array[3] = 0x00802380;                   
	dsi_set_cmdq(data_array, 4, 1);
 
	data_array[0] = 0x00023902;                          
	data_array[1] = 0x000002cc;                 
	dsi_set_cmdq(data_array, 2, 1); 
	MDELAY(5); 
	data_array[0] = 0x00023902;                          
	data_array[1] = 0x000077d2;                 
	dsi_set_cmdq(data_array, 2, 1); 
	
	data_array[0] = 0x001e3902;
	data_array[1] = 0x000700D3; 
	data_array[2] = 0x06060000; 
	data_array[3] = 0x00031032; 
	data_array[4] = 0x035f0303;  
	data_array[5] = 0x0008005f; 
	data_array[6] = 0x07333508;
	data_array[7] = 0x07073707;
	data_array[8] = 0x00000837;                
	dsi_set_cmdq(data_array, 9, 1);

	data_array[0] = 0x00213902;
	data_array[1] = 0x191818D5; 
	data_array[2] = 0x20181819; 
	data_array[3] = 0x18252421; 
	data_array[4] = 0x00181818;  
	data_array[5] = 0x02050401; 
	data_array[6] = 0x18070603;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818; 
	data_array[9] = 0x00000018;               
	dsi_set_cmdq(data_array, 10, 1);
	
	data_array[0] = 0x00213902;
	data_array[1] = 0x181818D6;
	data_array[2] = 0x25191918; 
	data_array[3] = 0x18202124; 
	data_array[4] = 0x05181818;  
	data_array[5] = 0x03000104; 
	data_array[6] = 0x18060702;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818; 
	data_array[9] = 0x00000018;               
	dsi_set_cmdq(data_array, 10, 1);
	
	data_array[0] = 0x002b3902;
	data_array[1] = 0x150E00E0; 
	data_array[2] = 0x35281515;
	data_array[3] = 0x100f083d; 
	data_array[4] = 0x1815111b; 
	data_array[5] = 0x150a1616;  
	data_array[6] = 0x0e001815; 
	data_array[7] = 0x28151515;
	data_array[8] = 0x0f083d35;
	data_array[9] = 0x15111b10;  
	data_array[10] = 0x0a161618;  
	data_array[11] = 0x00181515;              
	dsi_set_cmdq(data_array, 12, 1);
	MDELAY(1); 
	
	data_array[0] = 0x00033902;
	data_array[1] = 0x003636b6;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00023902;                          
	data_array[1] = 0x00000036;                 
	dsi_set_cmdq(data_array, 2, 1); 
	
	data_array[0] = 0x00110500;           
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(120);
	
	data_array[0] = 0x00290500;  
	dsi_set_cmdq(data_array, 1, 1);            
	MDELAY(10); 

} 

static void lcm_init_setting() 
{ 

unsigned int data_array[16];

data_array[0] = 0x00043902;
data_array[1] = 0x7983ffb9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(5); 
 
data_array[0] = 0x00153902;
data_array[1] = 0x161644b1;
data_array[2] = 0xd0503131;
data_array[3] = 0x388054ee;
data_array[4] = 0x1111f838;//0x2222f838
data_array[5] = 0x33fc0011;//0x33fc0022
data_array[6] = 0x00000001;
dsi_set_cmdq(data_array, 7, 1);
 
data_array[0] = 0x000a3902;
data_array[1] = 0x0dfe82b2;
data_array[2] = 0x1150200a;
data_array[3] = 0x00001d42;
dsi_set_cmdq(data_array, 4, 1);


data_array[0] = 0x000b3902;
data_array[1] = 0x027c02b4;
data_array[2] = 0x227c027c;
data_array[3] = 0x00862386;
dsi_set_cmdq(data_array, 4, 1);



data_array[0] = 0x00053902;
data_array[1] = 0x000000c7;
data_array[2] = 0x000000c0;
dsi_set_cmdq(data_array, 3, 1);
 
data_array[0] = 0x2cc1502;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0x11d21502;
dsi_set_cmdq(data_array, 1, 1);


data_array[0] = 0x001e3902;
data_array[1] = 0x000700d3;
data_array[2] = 0x08081c3c;
data_array[3] = 0x00021032;
data_array[4] = 0x03700302;
data_array[5] = 0x00080070;
data_array[6] = 0x06333708;
data_array[7] = 0x06063706;
data_array[8] = 0x00000b37;
dsi_set_cmdq(data_array, 9, 1);

data_array[0] = 0x00233902;
data_array[1] = 0x181919d5;
data_array[2] = 0x1b1a1a18;
data_array[3] = 0x0003021b;
data_array[4] = 0x04070601;
data_array[5] = 0x22212005;
data_array[6] = 0x18181823;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(data_array, 10, 1);

 

data_array[0] = 0x00213902;
data_array[1] = 0x191818d6;
data_array[2] = 0x1b1a1a19;
data_array[3] = 0x0502031b;
data_array[4] = 0x01060704;
data_array[5] = 0x21222300;
data_array[6] = 0x18181820;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(data_array, 10, 1);
 
/*
data_array[0] = 0x002b3902;
data_array[1] = 0x040000e0;
data_array[2] = 0x1e3f0f0e;
data_array[3] = 0x0e0c0830;
data_array[4] = 0x17141018;
data_array[5] = 0x13081615;
data_array[6] = 0x00001814;
data_array[7] = 0x3f0f0e04;
data_array[8] = 0x0c07301e;
data_array[9] = 0x140f190f;
data_array[10] = 0x07161417;
data_array[11] = 0x00171413;
dsi_set_cmdq(data_array, 12, 1);
*/

data_array[0] = 0x002b3902;
data_array[1] = 0x040000e0;
data_array[2] = 0x1d150c0b;
data_array[3] = 0x0e0c082e;
data_array[4] = 0x17141018;
data_array[5] = 0x13081615;
data_array[6] = 0x00001815;
data_array[7] = 0x150c0b04;
data_array[8] = 0x0c072e1d;
data_array[9] = 0x140f190f;
data_array[10] = 0x09161517;
data_array[11] = 0x00171514;
dsi_set_cmdq(data_array, 12, 1);


data_array[0] = 0x00033902;
data_array[1] = 0x005858b6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00351502;
dsi_set_cmdq(data_array, 1, 1);

data_array[0] = 0x00110500;           
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(120);

data_array[0] = 0x00290500;  
dsi_set_cmdq(data_array, 1, 1);            
MDELAY(10); 

}

static void lcm_init_setting_TCL() 
{ 
    unsigned int data_array[16]; 
	
} 

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},

    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 200, {}},
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
				MDELAY(2);
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

static void lcm_adc_id(void)
{
    int lcmid_adc = 0, ret_temp = 0, i = 0;
    int data[4] = {0,0,0,0};
    int res =0;
    
    i = LCD_ID_READ_TIMES;
    
    while (i--) {
        res = IMM_GetOneChannelValue(LCD_ID_ADC_CHANNEL,data,&ret_temp);
        lcmid_adc += ret_temp;
#ifdef BUILD_LK
        printf("Qlw 111[%d] = temp:%d,val:%d\n", i, ret_temp, lcmid_adc);
#else
        printk("Qlw 222[%d] = temp:%d,val:%d\n", i, ret_temp, lcmid_adc);
#endif
    }
    
    lcmid_adc = lcmid_adc/LCD_ID_READ_TIMES;

    if((lcmid_adc > ((int)(ADC_TCL_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (ADC_TCL_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TCL;
    }
    else if((lcmid_adc > ((int)(ADC_TONGXINGDA_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (ADC_TONGXINGDA_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TONGXINGDA;
    }
    else
    {
        lcm_type = LCM_TYPE_MAX;
    }
    
#ifdef BUILD_LK
    printf("Qlw 333 = lcm_type:%d \n", lcm_type);
#else
    printk("Qlw 444 = lcm_type:%d \n", lcm_type);
#endif

    return ;
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
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (0)///(LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; 
#endif

    // DSI
    /* Command mode setting */
    // 1 Three lane or Four lane
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting		
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count=480*3;	

    params->dsi.vertical_sync_active				= 5; // 3    2
    params->dsi.vertical_backporch					= 8; // 20   1 6
    params->dsi.vertical_frontporch					= 12; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 40;// 50  2
    params->dsi.horizontal_backporch				= 40;//51
    params->dsi.horizontal_frontporch				= 40;//51
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    //params->dsi.HS_TRAIL = 10;
    //params->dsi.HS_ZERO = 7;
    params->dsi.HS_PRPR = 4;
    //params->dsi.LPX = 3;
    //params->dsi.TA_SACK = 1;
    //params->dsi.TA_GET = 15;
    //params->dsi.TA_SURE = 3;
    //params->dsi.TA_GO = 12;
    //params->dsi.CLK_TRAIL = 10;
    //params->dsi.CLK_ZERO = 16;
    //params->dsi.LPX_WAIT = 10;
    //params->dsi.CONT_DET = 0;
    params->dsi.CLK_HS_PRPR = 6;
    //params->dsi.LPX=8; 

    // Bit rate calculation
    // 1 Every lane speed
	params->dsi.PLL_CLOCK 	= 200;//182
	params->dsi.ssc_disable = 1;	// ssc disable control (1: disable, 0: enable, default: 0)
	params->dsi.ssc_range 	= 5;	// ssc range control (1:min, 8:max, default: 5)
	
   // params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
   // params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
   // params->dsi.fbk_div =12 ;    // fref=26MHz, fvco = fref*(fbk_div)*2/(div1_real*div2_real) 
}

#if !defined(BUILD_LK)
extern char disp_lcm_name[];
static int set_lcm_name_once = 1;
static void lcm_set_name(void)
{		
	if(0xFFFF == lcm_type){
        lcm_adc_id();
	}
	if(LCM_TYPE_TCL == lcm_type) {
		sprintf(disp_lcm_name,"hx8379c_fwvga_dsi_vdo TCL");
	}
	else if(LCM_TYPE_TONGXINGDA == lcm_type) {
		sprintf(disp_lcm_name,"hx8379c_fwvga_dsi_vdo TongXingDa");
	}
	else {
		sprintf(disp_lcm_name,"hx8379c_fwvga_dsi_vdo TongXingDa");
	}
}
#endif

static int bl_Cust_Max = 1023;
static int bl_Cust_Min = 28;

static int setting_max = 1023;
static int setting_min = 40;

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

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10); 
    SET_RESET_PIN(0);
    MDELAY(10); 

    SET_RESET_PIN(1);
    MDELAY(120); 

#if 0
    if(0xFFFF == lcm_type) {
        lcm_adc_id();
	}
    if(LCM_TYPE_TCL == lcm_type) {
        lcm_init_setting_TCL();
    }
    else if(LCM_TYPE_TONGXINGDA == lcm_type) {
    	lcm_init_setting_TongXingDa();
	}
    else {
        lcm_init_setting_TongXingDa();
    }
#else
	lcm_init_setting();
#endif
}

static void lcm_suspend(void)
{
    unsigned int data_array[2];

 //   SET_RESET_PIN(1);
 //   MDELAY(10); 
 //   SET_RESET_PIN(0);
 //   MDELAY(10); 

 //   SET_RESET_PIN(1);
 //   MDELAY(120); 
	
 //   data_array[0] = 0x00280500; // Display Off
  //  dsi_set_cmdq(data_array, 1, 1);
  //  MDELAY(30); //35


    data_array[0] = 0x00100500; // Sleep In
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);//100

}

static void lcm_resume(void)
{

       unsigned int data_array[2];

       data_array[0] = 0x00110500;           
       dsi_set_cmdq(data_array, 1, 1); 
       MDELAY(120);

       data_array[0] = 0x00290500;  
       dsi_set_cmdq(data_array, 1, 1);            
       MDELAY(10); 


  //  lcm_init();
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    
#if !defined(BUILD_LK)
	if(set_lcm_name_once == 1)
	{
		lcm_set_name();
		set_lcm_name_once = 0;
	}
#endif
}
         
//#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
//#endif

static unsigned int lcm_compare_id(void)
{
    unsigned int data_array[16]; 
    unsigned char buffer[5];
    unsigned char id_high=0;
    unsigned char id_low=0;
    unsigned int id=0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);

    data_array[0]= 0x00043902; 
    data_array[1]= 0x7983FFB9; 
    dsi_set_cmdq(data_array, 2, 1); 
    MDELAY(10);
// hx8379c read id --- start
    data_array[0]= 0x00033902; 
    data_array[1]= 0x009341BA; 
    dsi_set_cmdq(data_array, 2, 1); 
    MDELAY(10);
//hx8379c read id --- end
    data_array[0] = 0x00023700;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    read_reg_v2(0x04, buffer, 3);
	/**  83\79\b1  **/
	
	id = buffer[1];

#if defined(BUILD_LK)
    printf("HX8379C uboot/lk %s \n", __func__);
    printf("%s id = 0x%08x \n", __func__, id);
#else
    printk("HX8379C kernel %s \n", __func__);
    printk("%s id = 0x%08x \n", __func__, id);
#endif
    //return (id == LCM_ID_HX8379C)?1:0;
	
    if(LCM_ID_HX8379C == id)
    {
        lcm_adc_id();
		
	#if defined(BUILD_LK)
		printf("%s LK lcm_type = %d \n", __func__, lcm_type);
	#else
		printk("%s Kernel lcm_type = %d \n", __func__, lcm_type);
	#endif

		return 1;
    }
    else
    {
         return 0;
    }
}

/*
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}
*/

LCM_DRIVER hx8379c_fwvga_dsi_vdo_lcm_drv= 
{
    .name			= "hx8379c_fwvga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
#ifdef CONFIG_WT_BRIGHTNESS_MAPPING_WITH_LCM
	.cust_mapping = lcm_brightness_mapping,
#endif
    };

