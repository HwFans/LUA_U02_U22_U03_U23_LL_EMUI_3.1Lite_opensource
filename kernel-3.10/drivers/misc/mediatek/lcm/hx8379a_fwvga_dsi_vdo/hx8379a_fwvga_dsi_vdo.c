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

#define LCM_ID_HX8379A                                      0x79

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

#define   LCM_DSI_CMD_MODE							0

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#define LCD_ID_ADC_CHANNEL    1
#define LCD_ID_READ_TIMES    3
#define LCD_ID_VALUE_OFFSET    300

#define ADC_TCL_VALUE    	2450
#define ADC_TIANMA_VALUE    4095

typedef enum
{
    LCM_TYPE_TCL,
    LCM_TYPE_TIANMA,

    LCM_TYPE_MAX,
} LCM_TYPE_E;
int id=0;
static unsigned int lcm_type = 0xFFFF;
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

//////////////////////////////////////////////////////////////////////////////////////////////////////////

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
static void lcm_init_setting_TCL() 
{ 
    unsigned int data_array[16]; 
#if 0
    data_array[0]=0x00043902;
    data_array[1]=0x7983ffb9;
    dsi_set_cmdq(data_array, 2, 1);  
    MDELAY(1);

	//@hfs add
	#if 1
	data_array[0]=0x00073902;
    data_array[1]=0x00a351ba;
	data_array[2]=0x0010a416;
    dsi_set_cmdq(data_array, 3, 1);  
	#endif

    data_array[0]=0x00203902;
    data_array[1]=0x345000b1;//ap2
    data_array[2]=0x110890ea;
    data_array[3]=0x372f7111;
    data_array[4]=0x1b421a9a;
    data_array[5]=0xe600f16e;
    data_array[6]=0xe6e6e6e6;
    data_array[7]=0x0a050400;
    data_array[8]=0x6f05040b;
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);

    data_array[0]=0x000e3902;
    data_array[1]=0xfe0000b2;
    data_array[2]=0x22190c08;
    data_array[3]=0x0c08ff00;
    data_array[4]=0x00002019;
    dsi_set_cmdq(data_array, 5, 1);
    MDELAY(1);

    data_array[0]=0x00203902;
    data_array[1]=0x000080b4;
    data_array[2]=0x32031032;
    data_array[3]=0x10325f13;
    data_array[4]=0x28013508;
    data_array[5]=0x3f003707;
    data_array[6]=0x04303008;
    data_array[7]=0x28084000;
    data_array[8]=0x04303008;	
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);

    data_array[0]=0x00303902;
    data_array[1]=0x0a0000d5;
    data_array[2]=0x00050100;
    data_array[3]=0x99881800;
    data_array[4]=0x88450188;
    data_array[5]=0x23450188;
    data_array[6]=0x88888867;
    data_array[7]=0x88888888;
    data_array[8]=0x88105499;	
    data_array[9]=0x54327688;
    data_array[10]=0x88888810;
    data_array[11]=0x00008888;
    data_array[12]=0x00000000;		
    dsi_set_cmdq(data_array, 13, 1);
    MDELAY(1);	

    data_array[0]=0x00043902;
    data_array[1]=0x047005de;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0]=0x00243902;
    data_array[1]=0x120779e0;
    data_array[2]=0x3f363414;
    data_array[3]=0x12065742;
    data_array[4]=0x16181612;
    data_array[5]=0x07161216;
    data_array[6]=0x36341412;
    data_array[7]=0x0657423f;
    data_array[8]=0x18161212;
    data_array[9]=0x16121616;	
    dsi_set_cmdq(data_array, 10, 1);
    MDELAY(1);	

    data_array[0]=0x00063902;
    data_array[1]=0x007200b6;//0x007600b6
    data_array[2]=0x00000072; //0x00000076
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(1);

    data_array[0]=0x00023902;
    data_array[1]=0x000002cc;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);	

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(data_array, 1, 1);

	#endif

    data_array[0]=0x00043902;
    data_array[1]=0x7983ffb9;
    dsi_set_cmdq(data_array, 2, 1);  
    MDELAY(1);

	//@hfs add
#if 1
	data_array[0]=0x00033902;
    data_array[1]=0x009351ba;
	data_array[2]=0x00000000;
    dsi_set_cmdq(data_array, 3, 1);  
#endif

    data_array[0]=0x00203902;
    data_array[1]=0x375000b1;//ap2
    data_array[2]=0x110851ee;
    data_array[3]=0x2c247414;
    data_array[4]=0x0c321a9a;
    data_array[5]=0xe600f176;
    data_array[6]=0xe6e6e6e6;
    data_array[7]=0x0a050400;
    data_array[8]=0x6f05040b;
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);

    data_array[0]=0x000e3902;
    data_array[1]=0xfe0000b2;
    data_array[2]=0x22190c08;
    data_array[3]=0x0c08ff00;
    data_array[4]=0x00002019;
    dsi_set_cmdq(data_array, 5, 1);
    MDELAY(1);

    data_array[0]=0x00203902;
    data_array[1]=0x000682b4;
    data_array[2]=0x32031032;
    data_array[3]=0x10325f13;
    data_array[4]=0x28013508;
    data_array[5]=0x3c003707;
    data_array[6]=0x0f3d3d0f;
    data_array[7]=0x28084000;
    data_array[8]=0x04303008;	
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);

    data_array[0]=0x00303902;
    data_array[1]=0x0a0000d5;
    data_array[2]=0x00050100;
    data_array[3]=0x99881800;
    data_array[4]=0x88450188;
    data_array[5]=0x23450188;
    data_array[6]=0x88888867;
    data_array[7]=0x88888888;
    data_array[8]=0x88105499;	
    data_array[9]=0x54327688;
    data_array[10]=0x88888810;
    data_array[11]=0x00008888;
    data_array[12]=0x00000000;		
    dsi_set_cmdq(data_array, 13, 1);
    MDELAY(1);	

    data_array[0]=0x00043902;
    data_array[1]=0x047005de;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0]=0x00243902;
    data_array[1]=0x121279e0;
    data_array[2]=0x3f252414;
    data_array[3]=0x12064636;
    data_array[4]=0x15161410;
    data_array[5]=0x12161216;
    data_array[6]=0x25241412;
    data_array[7]=0x0646363f;
    data_array[8]=0x16141012;
    data_array[9]=0x16121615;	
    dsi_set_cmdq(data_array, 10, 1);
    MDELAY(1);	

    data_array[0]=0x00063902;
    data_array[1]=0x006d00b6;//0x007600b6
    data_array[2]=0x00000077; //0x00000076
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(1);

    data_array[0]=0x00023902;
    data_array[1]=0x000002cc;//0a
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);	

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(data_array, 1, 1);

	
} 

static void lcm_init_setting_TIANMA() 
{ 
    unsigned int data_array[16]; 

    //TM045YDZA24_HX8379A_130517
    data_array[0]=0x00043902;
    data_array[1]=0x7983ffb9;
    dsi_set_cmdq(data_array, 2, 1);  
    MDELAY(1);
	
	
    data_array[0]=0x00023902;
    data_array[1]=0x000051ba;
    dsi_set_cmdq(data_array, 2, 1);  


#if 0
    data_array[0]=0x00203902;
    data_array[1]=0x445000b1;
    data_array[2]=0x110890de;
    data_array[3]=0x2b2b1111;
    data_array[4]=0x18421d9d;
    data_array[5]=0xe600f16e;
    data_array[6]=0xe6e6e6e6;
    data_array[7]=0x0a050400;
    data_array[8]=0x6f05040b;
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);
#else

    data_array[0]=0x00203902;
    data_array[1]=0x445000b1;
    data_array[2]=0x11089afe;
    data_array[3]=0x2b2b1111;
    data_array[4]=0x18421d9d;
    data_array[5]=0xe600f16e;
    data_array[6]=0xe6e6e6e6;
    data_array[7]=0x0a050400;
    data_array[8]=0x6f05040b;
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);
#endif
    data_array[0]=0x000e3902;
    data_array[1]=0xfe0000b2;
    data_array[2]=0x2219110b;
    data_array[3]=0x0004ff00;
    data_array[4]=0x00002019;
    dsi_set_cmdq(data_array, 5, 1);
    MDELAY(1);

    data_array[0]=0x00203902;
    data_array[1]=0x002802b4; //92--2dot
    data_array[2]=0x320b1032;
    data_array[3]=0x10325f13;
    data_array[4]=0x27011128;
    data_array[5]=0x44031309; //0x30
    data_array[6]=0x02444408;
    data_array[7]=0x28084000;
    data_array[8]=0x04303008;	
    dsi_set_cmdq(data_array, 9, 1);
    MDELAY(1);

    data_array[0]=0x00023902;
    data_array[1]=0x000002cc;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

	data_array[0]=0x00053902;
    data_array[1]=0x007c00b6; //VCOM
    data_array[2]=0x0000007c;	
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(1);

    data_array[0]=0x00303902;
    data_array[1]=0x0a0000d5;
    data_array[2]=0x00000100;
    data_array[3]=0x88990002;
    data_array[4]=0x01238888;
    data_array[5]=0x01888888;
    data_array[6]=0x88888888;
    data_array[7]=0x99888888;
    data_array[8]=0x32108888;	
    data_array[9]=0x10888888;
    data_array[10]=0x88888888;
    data_array[11]=0x00008888;
    data_array[12]=0x00000000;		
    dsi_set_cmdq(data_array, 13, 1);
    MDELAY(1);	

    data_array[0]=0x00043902;
    data_array[1]=0x047005de;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    data_array[0]=0x00243902;
    data_array[1]=0x070079e0;
    data_array[2]=0x3f252114;
    data_array[3]=0x1108412d;
    data_array[4]=0x1516140f;
    data_array[5]=0x00151116;
    data_array[6]=0x25211407;
    data_array[7]=0x08412d3F;
    data_array[8]=0x16140f11;
    data_array[9]=0x15111615;	
    dsi_set_cmdq(data_array, 10, 1);
    MDELAY(1);	


	
    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);	

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(50);
	
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
    else if((lcmid_adc > ((int)(ADC_TIANMA_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (ADC_TIANMA_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TIANMA;
    }
    else
    {
        lcm_type = LCM_TYPE_MAX;
    }
    
#ifdef BUILD_LK
    printf("Qlw 333 = lcm_type:%d,id:0x%x\n", lcm_type,id);
#else
    printk("Qlw 444 = lcm_type:%d,id:0x%x\n", lcm_type,id);
#endif

    return ;
}

static void lcm_get_params_tcl(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (0)///(LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
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

    /*
    params->dsi.vertical_sync_active				= 4;// 3    2
    params->dsi.vertical_backporch					= 6;// 20   1
    params->dsi.vertical_frontporch					= 14; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 40;// 50  2
    params->dsi.horizontal_backporch				= 40;
    params->dsi.horizontal_frontporch				= 40;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    //params->dsi.LPX=8; 

    // Bit rate calculation
    //1 Every lane speed
    params->dsi.pll_div1=1;		    // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=0;		    // div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =14;           // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real) 17
    //params->dsi.pll_div1=1;	    // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    //params->dsi.pll_div2=0;	    // div2=0,1,2,3;div1_real=1,2,4,4	//1
    //params->dsi.fbk_div =17;       // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	//20		

    */

    params->dsi.vertical_sync_active				= 4;// 3    2
    params->dsi.vertical_backporch					= 7;// 20   1 6
    params->dsi.vertical_frontporch					= 12; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 40;// 50  2
    params->dsi.horizontal_backporch				= 40;
    params->dsi.horizontal_frontporch				= 40;
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
    params->dsi.CLK_HS_PRPR = 3;
    params->dsi.HS_TRAIL = 1;
	params->dsi.CLK_TRAIL =1;
    //params->dsi.LPX=8; 

    // Bit rate calculation
    // 1 Every lane speed
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =27 ;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
    params->dsi.ssc_range = 2;
	params->dsi.ssc_disable = 0;
}

static void lcm_get_params_tianma(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (0)///(LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
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

    /*
    params->dsi.vertical_sync_active				= 4;// 3    2
    params->dsi.vertical_backporch					= 6;// 20   1
    params->dsi.vertical_frontporch					= 14; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 40;// 50  2
    params->dsi.horizontal_backporch				= 40;
    params->dsi.horizontal_frontporch				= 40;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    //params->dsi.LPX=8; 

    // Bit rate calculation
    //1 Every lane speed
    params->dsi.pll_div1=1;		    // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=0;		    // div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =14;           // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real) 17
    //params->dsi.pll_div1=1;	    // div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    //params->dsi.pll_div2=0;	    // div2=0,1,2,3;div1_real=1,2,4,4	//1
    //params->dsi.fbk_div =17;       // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	//20		

    */
    params->dsi.vertical_sync_active				= 4;// 3    2
    params->dsi.vertical_backporch					= 11;// 20   1 6
    params->dsi.vertical_frontporch					= 6; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 20;// 50  2
    params->dsi.horizontal_backporch				= 30;
    params->dsi.horizontal_frontporch				= 30;
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
    params->dsi.CLK_HS_PRPR = 3;
    params->dsi.HS_TRAIL = 1;
	params->dsi.CLK_TRAIL =1;
    //params->dsi.LPX=8; 

    // Bit rate calculation
    // 1 Every lane speed
    params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4	
    params->dsi.fbk_div =26 ;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}

//add by hyde for debug
static void lcm_init(void)
{
    //lcm_util.set_gpio_mode(GPIO_DISP_LRSTB_PIN, 0);
    //lcm_util.set_gpio_dir(GPIO_DISP_LRSTB_PIN, GPIO_DIR_OUT);

    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10); 
    SET_RESET_PIN(0);
    MDELAY(50); 

    SET_RESET_PIN(1);
    MDELAY(120); 

#if 1
    if(0xFFFF == lcm_type) {
        lcm_adc_id();
}
    if(LCM_TYPE_TCL == lcm_type) {
        lcm_init_setting_TCL();
    }
    else if(LCM_TYPE_TIANMA == lcm_type) {
    lcm_init_setting_TIANMA();
}
    else {
        lcm_init_setting_TCL();
    }
#else
    lcm_type = LCM_TYPE_TCL;
	lcm_init_setting_TCL();
#endif
}

static void lcm_suspend(void)
{
    unsigned int data_array[3];

	data_array[0]=0x00073902;
    data_array[1]=0x009341ba; 
    data_array[2]=0x0010ac16;	
    dsi_set_cmdq(data_array, 3, 1);
   // MDELAY(1);

	data_array[0]=0x00043902;
    data_array[1]=0xb80100b0; 
    dsi_set_cmdq(data_array, 2, 1);
   // MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0xa80100b0; 
	dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);

	data_array[0]=0x00043902;
    data_array[1]=0x080100b0; 
    dsi_set_cmdq(data_array, 2, 1);
   MDELAY(10);

	data_array[0]=0x00043902;
    data_array[1]=0x000100b0; 
    dsi_set_cmdq(data_array, 2, 1);
   // MDELAY(10);

	data_array[0] = 0x00100500; // Sleep Out
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);	


/*
    SET_RESET_PIN(1);
    MDELAY(10); 
    SET_RESET_PIN(0);
    MDELAY(10); 

    SET_RESET_PIN(1);
    MDELAY(120); 
	*/
	#if 0
    //push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    data_array[0] = 0x00280500; // Display Off
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(100); //35
    data_array[0] = 0x00100500; // Sleep In
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(200);//100
	#endif

/*
	data_array[0]=0x00043902;
    data_array[1]=0x7983ffb9;
    dsi_set_cmdq(data_array, 2, 1);  

	data_array[0]=0x00073902;
    data_array[1]=0x00a351ba;
	data_array[2]=0x0010a416;
    dsi_set_cmdq(data_array, 3, 1); 
    */
	
}

static void lcm_resume(void)
{
    lcm_init();
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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

static unsigned int lcm_compare_id_tcl(void)
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
// hx8379a read id --- start
    data_array[0]= 0x00033902; 
    data_array[1]= 0x009341BA; 
    dsi_set_cmdq(data_array, 2, 1); 
    MDELAY(10);

//hx8379a read id --- end
    data_array[0] = 0x00023700;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    read_reg_v2(0xF4, buffer, 2);
    id = buffer[0];

#if defined(BUILD_LK)
    printf("HX8379A uboot hyde compare_id%s \n", __func__);
    printf("%s id = 0x%08x \n", __func__, id);
    printf("hyde id is %08x \n",id);
#else
    printk("HX8379A kernel hyde compare_id%s \n", __func__);
    printk("%s id = 0x%08x \n", __func__, id);
    printk("hyde id is %08x \n",id);
#endif

    
    if(LCM_ID_HX8379A == id)
    {
        lcm_adc_id();
        return (lcm_type == LCM_TYPE_TCL)?1:0;
    }
    else
    {
         return 0;
    }
}

static unsigned int lcm_compare_id_tianma(void)
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
	
    data_array[0]= 0x00033902; 
    data_array[1]= 0x009341BA; 
    dsi_set_cmdq(data_array, 2, 1); 
    MDELAY(10);
	
    data_array[0] = 0x00023700;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);
    read_reg_v2(0xF4, buffer, 2);
    id = buffer[0];

#if defined(BUILD_LK)
    printf("HX8379A uboot hyde compare_id%s \n", __func__);
    printf("%s id = 0x%08x \n", __func__, id);
    printf("hyde id is %08x \n",id);
#else
    printk("HX8379A kernel hyde compare_id%s \n", __func__);
    printk("%s id = 0x%08x \n", __func__, id);
    printk("hyde id is %08x \n",id);
#endif

    if(LCM_ID_HX8379A == id)
    {
        lcm_adc_id();
        return (lcm_type == LCM_TYPE_TIANMA)?1:0;
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
#ifdef WT_LCM_DEBUG
void lcm_debug_func(unsigned char *buffer, int len)
{
    int i = 0;
	unsigned int data_array[16];
	
	if(len/4 == 0 || len%4 != 0)
	{
	   printk("lcm_debug:data format error\n");
	   return;
	}
	for(i = 0; i < len/4; i++)
	{
	   data_array[i] = buffer[4*i];
	   data_array[i] = (data_array[i]<<8)|buffer[4*i+1];
	   data_array[i] = (data_array[i]<<8)|buffer[4*i+2];
	   data_array[i] = (data_array[i]<<8)|buffer[4*i+3];
	   printk("lcm_debug:data_array[%d] = %x\n", i, data_array[i]);
	}
	
	dsi_set_cmdq((unsigned int*)data_array, i, 1);
	MDELAY(10);
		
    return;
}
#endif

#ifdef WT_PQ_WITH_MULTI_LCM
DISP_PQ_PARAM_LCD lcm_pq_tcl = 
{
u4SHPGain:4,
u4SatGain:0,
u4HueAdj:{9,4,17,15},
u4SatAdj:{0,5,3,3}
};

void lcm_get_pqparam_tcl(DISP_PQ_PARAM_LCD *pq_data)
{
	if(pq_data != NULL)
		memcpy(pq_data, &lcm_pq_tcl, sizeof(DISP_PQ_PARAM_LCD));
}
#endif

LCM_DRIVER hx8379a_dsi_vdo_lcm_drv_tcl= 
{
    .name			= "hx8379a_dsi_vdo_tcl",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params_tcl,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id_tcl,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
#ifdef WT_LCM_DEBUG
    .lcm_debug      = lcm_debug_func,
#endif
#ifdef WT_PQ_WITH_MULTI_LCM
	.get_pqparam    = lcm_get_pqparam_tcl,
#endif
    };

LCM_DRIVER hx8379a_dsi_vdo_lcm_drv_tianma= 
{
    .name			= "hx8379a_dsi_vdo_tianma",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params_tianma,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id_tianma,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
#ifdef WT_LCM_DEBUG  
    .lcm_debug      = lcm_debug_func,
#endif
    };

