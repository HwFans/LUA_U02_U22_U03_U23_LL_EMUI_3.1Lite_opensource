/*******************************************************************************************/
// schedule
//   getsensorid ok
//   open ok
//   setting(pv,cap,video) ok

/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi545mipiraw_bilu_Sensor.h"
#include "hi545mipiraw_bilu_Camera_Sensor_para.h"
#include "hi545mipiraw_bilu_CameraCustomized.h"
static DEFINE_SPINLOCK(HI545mipiraw_drv_lock);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

// Hynix burst mode Start
static struct Hynix_Sensor_reg { 
	u16 addr;
	u16 val;
};


static struct imgsensor_diff{
   u16 ModuleHouseID;
   u16   VCAM;
};
#define HYNIX_TABLE_END 65535
#define HYNIX_SIZEOF_I2C_BUF 8  
static char Hynix_i2c_buf[HYNIX_SIZEOF_I2C_BUF];
static void hynix_i2c_burst_mode(struct Hynix_Sensor_reg table[]);
// Hynix burst mode End
static void HI545_Sensor_update_wb_gain(kal_uint32 r_gain, kal_uint32 b_gain, kal_uint32 g_gain);

static kal_uint16 HI545_Sensor_OTP_read(kal_uint16 otp_addr,kal_uint16 otp_addr_one,kal_uint16 otp_addr_two);

static kal_uint16 HI545_Sensor_calc_wbdata();

#define HI545_TEST_PATTERN_CHECKSUM (0x9FC8832A)//do rotate will change this value

#define HI545_DEBUG


//OTP mode on/off
#define HI545_OTP_FUNCTION


//#define HI545_DEBUG_SOFIA

#ifdef HI545_DEBUG
	#define HI545DB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[HI545Rawbilu] ",  fmt, ##arg)
#else
	#define HI545DB(fmt, arg...)
#endif

#ifdef HI545_DEBUG_SOFIA
	#define HI545DBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[HI545Rawbilu] ",  fmt, ##arg)
#else
	#define HI545DBSOFIA(fmt, arg...)
#endif

#define mDELAY(ms)  mdelay(ms)

static kal_uint32 HI545_FeatureControl_PERIOD_PixelNum=HI545_PV_PERIOD_PIXEL_NUMS;
static kal_uint32 HI545_FeatureControl_PERIOD_LineNum=HI545_PV_PERIOD_LINE_NUMS;

static UINT16 VIDEO_MODE_TARGET_FPS = 30;
static BOOL ReEnteyCamera = KAL_FALSE;


static MSDK_SENSOR_CONFIG_STRUCT HI545SensorConfigData;

static kal_uint32 HI545_FAC_SENSOR_REG;

static MSDK_SCENARIO_ID_ENUM HI545CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
static SENSOR_REG_STRUCT HI545SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
static SENSOR_REG_STRUCT HI545SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static HI545_PARA_STRUCT HI545;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define HI545_OTP_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HI545MIPI_WRITE_ID)
// modify by yfx
//extern int iMultiWriteReg(u8 *pData, u16 lens);
#if 1
#define HI545_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 2, HI545MIPI_WRITE_ID)
#else
static void HI545_write_cmos_sensor(u16 addr, u16 para) 
{
	u8 senddata[4];
	senddata[0] = (addr >> 8) & 0xff;
	senddata[1] = addr & 0xff;
	senddata[2] = (para >> 8) & 0xff;
	senddata[3] = para & 0xff;
	iMultiWriteReg(senddata, 4)
}
#endif
// end

static kal_uint16 HI545_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HI545MIPI_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

static void HI545_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = HI545_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 extra_lines = 0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	HI545DBSOFIA("!!shutter=%d!!!!!\n", shutter);

	if(HI545.HI545AutoFlickerMode == KAL_TRUE)
	{

		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )  //(g_iHI545_Mode == HI545_MODE_PREVIEW)	//SXGA size output
		{
			line_length = HI545_PV_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			max_shutter = HI545_PV_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI545.sensorMode ) //add for video_6M setting
		{
			line_length = HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			max_shutter = HI545_VIDEO_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else
		{
			line_length = HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			max_shutter = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}

		switch(HI545CurrentScenarioId)
		{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				HI545DBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n");
				min_framelength = max_shutter;// capture max_fps 24,no need calculate
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				if(VIDEO_MODE_TARGET_FPS==30)
				{
					min_framelength = (HI545.videoPclk*10000) /(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/304*10 ;
				}
				else if(VIDEO_MODE_TARGET_FPS==15)
				{
					min_framelength = (HI545.videoPclk*10000) /(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/148*10 ;
				}
				else
				{
					min_framelength = max_shutter;
				}
				break;
			default:
				min_framelength = (HI545.pvPclk*10000) /(HI545_PV_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/296*10 ;
    			break;
		}

		HI545DBSOFIA("AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n",min_framelength,min_framelength);
		HI545DBSOFIA("max framerate(10 base) autofilker = %d\n",(HI545.pvPclk*10000)*10 /line_length/min_framelength);

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )	//SXGA size output
		{
			frame_length = HI545_PV_PERIOD_LINE_NUMS+ HI545.DummyLines + extra_lines ;
		}
		else if(SENSOR_MODE_VIDEO == HI545.sensorMode)
		{
			frame_length = HI545_VIDEO_PERIOD_LINE_NUMS+ HI545.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			frame_length = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines + extra_lines ;
		}
		HI545DBSOFIA("frame_length 0= %d\n",frame_length);

		if (frame_length < min_framelength)
		{
			//shutter = min_framelength - 4;

			switch(HI545CurrentScenarioId)
			{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				extra_lines = min_framelength- (HI545_FULL_PERIOD_LINE_NUMS+ HI545.DummyLines);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				extra_lines = min_framelength- (HI545_VIDEO_PERIOD_LINE_NUMS+ HI545.DummyLines);
				break;
			default:
				extra_lines = min_framelength- (HI545_PV_PERIOD_LINE_NUMS+ HI545.DummyLines);
    			break;
			}
			frame_length = min_framelength;
		}

		HI545DBSOFIA("frame_length 1= %d\n",frame_length);

		ASSERT(line_length < HI545_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI545_MAX_FRAME_LENGTH); 	//0xFFFF
		
		spin_lock_irqsave(&HI545mipiraw_drv_lock,flags);
		HI545.maxExposureLines = frame_length - 4;
		HI545_FeatureControl_PERIOD_PixelNum = line_length;
		HI545_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI545mipiraw_drv_lock,flags);


		HI545_write_cmos_sensor(0x0046, 0x0100);
		//Set total frame length
		HI545_write_cmos_sensor(0x0006, frame_length);
//		HI545_write_cmos_sensor(0x0007, frame_length & 0xFF);

		//Set shutter (Coarse integration time, uint: lines.)
		HI545_write_cmos_sensor(0x0004, shutter);
		//HI545_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		
		HI545_write_cmos_sensor(0x0046, 0x0000);

		HI545DBSOFIA("frame_length 2= %d\n",frame_length);
		HI545DB("framerate(10 base) = %d\n",(HI545.pvPclk*10000)*10 /line_length/frame_length);

		HI545DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);

	}
	else
	{
		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )  //(g_iHI545_Mode == HI545_MODE_PREVIEW)	//SXGA size output
		{
			max_shutter = HI545_PV_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI545.sensorMode ) //add for video_6M setting
		{
			max_shutter = HI545_VIDEO_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}
		else
		{
			max_shutter = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines ;
		}

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )	//SXGA size output
		{
			line_length = HI545_PV_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			frame_length = HI545_PV_PERIOD_LINE_NUMS+ HI545.DummyLines + extra_lines ;
		}
		else if( SENSOR_MODE_VIDEO == HI545.sensorMode )
		{
			line_length = HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			frame_length = HI545_VIDEO_PERIOD_LINE_NUMS + HI545.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			line_length = HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
			frame_length = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines + extra_lines ;
		}

		ASSERT(line_length < HI545_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI545_MAX_FRAME_LENGTH); 	//0xFFFF

		//Set total frame length
		HI545_write_cmos_sensor(0x0046, 0x0100);		
		HI545_write_cmos_sensor(0x0006, frame_length);
		//HI545_write_cmos_sensor(0x0007, frame_length & 0xFF);
		HI545_write_cmos_sensor(0x0046, 0x0000);		

		spin_lock_irqsave(&HI545mipiraw_drv_lock,flags);
		HI545.maxExposureLines = frame_length -4;
		HI545_FeatureControl_PERIOD_PixelNum = line_length;
		HI545_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI545mipiraw_drv_lock,flags);


		//Set shutter (Coarse integration time, uint: lines.)
		HI545_write_cmos_sensor(0x0046, 0x0100);		
		HI545_write_cmos_sensor(0x0004, shutter);
		//HI545_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		HI545_write_cmos_sensor(0x0046, 0x0000);		
		
		//HI545DB("framerate(10 base) = %d\n",(HI545.pvPclk*10000)*10 /line_length/frame_length);
		HI545DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);
	}

}   /* write_HI545_shutter */

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI545Reg2Gain(const kal_uint8 iReg)
{
	kal_uint16 iGain = 64;

	iGain = 4 * iReg + 64;

	return iGain;
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI545Gain2Reg(const kal_uint16 Gain)
{
	kal_uint16 iReg;
	kal_uint8 iBaseGain = 64;
	
	iReg = Gain / 4 - 16;
    return iReg;//HI545. sensorGlobalGain

}


static void write_HI545_gain(kal_uint8 gain)
{
#if 1
	HI545_write_cmos_sensor(0x003a, (gain << 8));
#endif
	return;
}

/*************************************************************************
* FUNCTION
*    HI545_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void HI545_SetGain(UINT16 iGain)
{
	unsigned long flags;
    kal_uint16 HI545GlobalGain=0;
	kal_uint16 DigitalGain = 0;

    
	// AG = (regvalue / 16) + 1
	if(iGain > 1024)  
	{
		iGain = 1024;
	}
	if(iGain < 64)
	{
		iGain = 64;
	}
		
	HI545GlobalGain = HI545Gain2Reg(iGain); 
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.realGain = iGain;
	HI545.sensorGlobalGain =HI545GlobalGain;
	spin_unlock(&HI545mipiraw_drv_lock);

	HI545DB("[HI545_SetGain]HI545.sensorGlobalGain=0x%x,HI545.realGain=%d\n",HI545.sensorGlobalGain,HI545.realGain);

	HI545_write_cmos_sensor(0x0046, 0x0100);
	write_HI545_gain(HI545.sensorGlobalGain);	
	HI545_write_cmos_sensor(0x0046, 0x0000);

	
	return;	
}   /*  HI545_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_HI545_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 read_HI545_gain(void)
{
	kal_uint16 read_gain_anolog=0;
	kal_uint16 HI545RealGain_anolog =0;
	kal_uint16 HI545RealGain =0;

	read_gain_anolog=HI545_read_cmos_sensor(0x003a);
	
	HI545RealGain_anolog = HI545Reg2Gain(read_gain_anolog);
	

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorGlobalGain = read_gain_anolog;
	HI545.realGain = HI545RealGain;
	spin_unlock(&HI545mipiraw_drv_lock);
	HI545DB("[read_HI545_gain]HI545RealGain_anolog=0x%x\n",HI545RealGain_anolog);

	return HI545.sensorGlobalGain;
}  /* read_HI545_gain */

// Hynix burst mode Start 
static void hynix_i2c_burst_mode(struct Hynix_Sensor_reg table[])
{
    int err;
	const struct Hynix_Sensor_reg *curr;
	const struct Hynix_Sensor_reg *next;
	u16 buf_count = 0;

	HI545DB("hynix_i2c_burst_mode start \n");

	for (curr = table; curr->addr != HYNIX_TABLE_END; curr++) 
    {
		if (!buf_count) { 

			Hynix_i2c_buf[buf_count] = curr->addr >> 8; buf_count++;
            
			Hynix_i2c_buf[buf_count] = curr->addr & 0xFF; buf_count++;
		}
        
		Hynix_i2c_buf[buf_count] = curr->val >> 8; buf_count++;
        
		Hynix_i2c_buf[buf_count] = curr->val & 0xFF; buf_count++;
     
		next = curr + 1;
		if (next->addr == curr->addr + 2 &&
			buf_count < HYNIX_SIZEOF_I2C_BUF &&
			next->addr != HYNIX_TABLE_END)
        {  
            continue;
        }
        		
        err = iWriteRegI2C(Hynix_i2c_buf, buf_count,HI545MIPI_WRITE_ID);
        
		if (err)
		{	
		    return err;
        }   

		buf_count = 0;
	}

	HI545DB("hynix_i2c_burst_mode end \n");
	return 0;
}
// Hynix burst mode End 
static void HI545_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
        HI545_write_cmos_sensor(HI545SensorReg[i].Addr, HI545SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
        HI545_write_cmos_sensor(HI545SensorReg[i].Addr, HI545SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HI545_write_cmos_sensor(HI545SensorCCT[i].Addr, HI545SensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    HI545_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void HI545_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
         temp_data = HI545_read_cmos_sensor(HI545SensorReg[i].Addr);
		 spin_lock(&HI545mipiraw_drv_lock);
		 HI545SensorReg[i].Para =temp_data;
		 spin_unlock(&HI545mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI545SensorReg[i].Addr; i++)
    {
        temp_data = HI545_read_cmos_sensor(HI545SensorReg[i].Addr);
		spin_lock(&HI545mipiraw_drv_lock);
		HI545SensorReg[i].Para = temp_data;
		spin_unlock(&HI545mipiraw_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    HI545_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_int32  HI545_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

static void HI545_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

static void HI545_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= HI545SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/HI545.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= HI545_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= HI545_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  //MT9P017_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



static kal_bool HI545_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * HI545.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

			 HI545DBSOFIA("HI545????????????????????? :\n ");
		  spin_lock(&HI545mipiraw_drv_lock);
          HI545SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&HI545mipiraw_drv_lock);
          HI545_write_cmos_sensor(HI545SensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&HI545mipiraw_drv_lock);
                    HI545_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&HI545mipiraw_drv_lock);
                    break;
                case 1:
                    HI545_write_cmos_sensor(HI545_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void HI545_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == HI545.sensorMode )	//SXGA size output
	{
		line_length = HI545_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI545_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== HI545.sensorMode )
	{
		line_length = HI545_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI545_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		line_length = HI545_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI545_FULL_PERIOD_LINE_NUMS + iLines;
	}

	//if(HI545.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < HI545_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < HI545_MAX_FRAME_LENGTH);	//0xFFFF

	//Set total frame length
	HI545_write_cmos_sensor(0x0006, frame_length);
	//HI545_write_cmos_sensor(0x0007, frame_length & 0xFF);

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.maxExposureLines = frame_length -4;
	HI545_FeatureControl_PERIOD_PixelNum = line_length;
	HI545_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&HI545mipiraw_drv_lock);

	//Set total line length
	HI545_write_cmos_sensor(0x0008, line_length);
	//HI545_write_cmos_sensor(0x0009, line_length & 0xFF);

}   /*  HI545_SetDummy */

static void HI545PreviewSetting(void)
{	
#if 1
	HI545DB("HI545PreviewSetting_2lane_30fps:\n ");
	
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-545
	//		Mode			 : Preview 
	//		Size			 : 2560 * 1920
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	HI545_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI545_write_cmos_sensor(0x004C, 0x0100); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	
	HI545_write_cmos_sensor(0x000C, 0x0000); 
	
	HI545_write_cmos_sensor(0x0902, 0x4101); 
	HI545_write_cmos_sensor(0x090A, 0x03E4); 
	HI545_write_cmos_sensor(0x090C, 0x0020); 
	HI545_write_cmos_sensor(0x090E, 0x0020); 
	HI545_write_cmos_sensor(0x0910, 0x5D07); 
	HI545_write_cmos_sensor(0x0912, 0x061e); 
	HI545_write_cmos_sensor(0x0914, 0x0407); 
	HI545_write_cmos_sensor(0x0916, 0x0b0a); 
	HI545_write_cmos_sensor(0x0918, 0x0e09); 

	//HI545_write_cmos_sensor(0x020A, 0x0100);
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x0012, 0x00BA); //x_addr_start_hact_h. 
	HI545_write_cmos_sensor(0x0018, 0x0ABD); //x_addr_end_hact_h.   
	
	HI545_write_cmos_sensor(0x0026, 0x0022); //y_addr_start_vact_h. 
	HI545_write_cmos_sensor(0x002C, 0x07A5); //y_addr_end_vact_h.    
	
	HI545_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l  
	HI545_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l  
	HI545_write_cmos_sensor(0x012C, 0x0A00); //2560 digital_crop_image_width 
	HI545_write_cmos_sensor(0x012E, 0x0780); //1920 digital_crop_image_height 
	
	//Image size 2560x1920
	HI545_write_cmos_sensor(0x0110, 0x0A00); //X_output_size_h 
	HI545_write_cmos_sensor(0x0112, 0x0780); //Y_output_size_h 
	
	//HI545_write_cmos_sensor(0x0006, 0x07e1); //07c0); //frame_length_h 2017
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); 
	HI545_write_cmos_sensor(0x0002, 0x04b0); //Fine.int 
	HI545_write_cmos_sensor(0x0700, 0x0590); 

#ifdef HI545_OTP_FUNCTION
	HI545_write_cmos_sensor(0x0A04, 0x011B); // from 0112 to 0113 to 011B DG on for OTP
#else
	HI545_write_cmos_sensor(0x0A04, 0x0113); // from 0112 to 0113
#endif
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off 


	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

    HI545DB("HI545PreviewSetting_2lane exit :\n ");
#endif
}


static void HI545VideoSetting(void)
{

	HI545DB("HI545VideoSetting:\n ");
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-545
	//		Mode			 : Video 
	//		Size			 : 2560 * 1920
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	HI545_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI545_write_cmos_sensor(0x004C, 0x0100); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	
	HI545_write_cmos_sensor(0x000C, 0x0000); 
	
	HI545_write_cmos_sensor(0x0902, 0x4101); 
	HI545_write_cmos_sensor(0x090A, 0x03E4); 
	HI545_write_cmos_sensor(0x090C, 0x0020); 
	HI545_write_cmos_sensor(0x090E, 0x0020); 
	HI545_write_cmos_sensor(0x0910, 0x5D07); 
	HI545_write_cmos_sensor(0x0912, 0x061e); 
	HI545_write_cmos_sensor(0x0914, 0x0407); 
	HI545_write_cmos_sensor(0x0916, 0x0b0a); 
	HI545_write_cmos_sensor(0x0918, 0x0e09); 
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x0012, 0x00BA); //x_addr_start_hact_h. 
	HI545_write_cmos_sensor(0x0018, 0x0ABD); //x_addr_end_hact_h.   
	
	HI545_write_cmos_sensor(0x0026, 0x0022); //y_addr_start_vact_h. 
	HI545_write_cmos_sensor(0x002C, 0x07A5); //y_addr_end_vact_h.   
	
	HI545_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l  
	HI545_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l  
	HI545_write_cmos_sensor(0x012C, 0x0A00); //2560 digital_crop_image_width 
	HI545_write_cmos_sensor(0x012E, 0x0780); //1920 digital_crop_image_height 
	
	//Image size 2560x1920
	HI545_write_cmos_sensor(0x0110, 0x0A00); //X_output_size_h 
	HI545_write_cmos_sensor(0x0112, 0x0780); //Y_output_size_h 
	
	//HI545_write_cmos_sensor(0x0006, 0x07e1); //07c0); //frame_length_h 2017
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); 
	HI545_write_cmos_sensor(0x0002, 0x04b0); //Fine.int 
	HI545_write_cmos_sensor(0x0700, 0x0590); 

#ifdef HI545_OTP_FUNCTION
	HI545_write_cmos_sensor(0x0A04, 0x011B); // from 0112 to 0113 to 011B DG on for OTP
#else
	HI545_write_cmos_sensor(0x0A04, 0x0113); // from 0112 to 0113
#endif
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	

	
	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

	HI545DB("HI545VideoSetting_4:3 exit :\n ");
}


static void HI545CaptureSetting(void)
{

    if(ReEnteyCamera == KAL_TRUE)
    {
		HI545DB("HI545CaptureSetting_2lane_SleepIn :\n ");
    }
	else
	{
		HI545DB("HI545CaptureSetting_2lane_streamOff :\n ");
	}

	HI545DB("HI545CaptureSetting_2lane_OB:\n ");
	
	//////////////////////////////////////////////////////////////////////////
	//	Sensor			 : Hi-545
	//		Mode			 : Capture 
	//		Size			 : 2560 * 1920
	//////////////////////////////////////////////////////////////////////////

	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	HI545_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI545_write_cmos_sensor(0x004C, 0x0100); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	
	HI545_write_cmos_sensor(0x000C, 0x0000); 
	
	HI545_write_cmos_sensor(0x0902, 0x4101); 
	HI545_write_cmos_sensor(0x090A, 0x03E4); 
	HI545_write_cmos_sensor(0x090C, 0x0020); 
	HI545_write_cmos_sensor(0x090E, 0x0020); 
	HI545_write_cmos_sensor(0x0910, 0x5D07); 
	HI545_write_cmos_sensor(0x0912, 0x061e); 
	HI545_write_cmos_sensor(0x0914, 0x0407); 
	HI545_write_cmos_sensor(0x0916, 0x0b0a); 
	HI545_write_cmos_sensor(0x0918, 0x0e09); 
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x0012, 0x00BA); //x_addr_start_hact_h. 
	HI545_write_cmos_sensor(0x0018, 0x0ABD); //x_addr_end_hact_h.   
	
	HI545_write_cmos_sensor(0x0026, 0x0022); //y_addr_start_vact_h. 
	HI545_write_cmos_sensor(0x002C, 0x07A5); //y_addr_end_vact_h.     
	
	HI545_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l  
	HI545_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l  
	HI545_write_cmos_sensor(0x012C, 0x0A00); //2560 digital_crop_image_width 
	HI545_write_cmos_sensor(0x012E, 0x0780); //1920 digital_crop_image_height 
	
	//Image size 2560x1920
	HI545_write_cmos_sensor(0x0110, 0x0A00); //X_output_size_h 
	HI545_write_cmos_sensor(0x0112, 0x0780); //Y_output_size_h 
	
	//HI545_write_cmos_sensor(0x0006, 0x07e1); //07c0); //frame_length_h 2017
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); 
	HI545_write_cmos_sensor(0x0002, 0x04b0); //Fine.int 
	HI545_write_cmos_sensor(0x0700, 0x0590); 

#ifdef HI545_OTP_FUNCTION
	HI545_write_cmos_sensor(0x0A04, 0x011B); // from 0112 to 0113 to 011B DG on for OTP
#else
	HI545_write_cmos_sensor(0x0A04, 0x0113); // from 0112 to 0113
#endif
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off 


	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;
	
	HI545DB("HI545CaptureSetting_2lane exit :\n ");
}

static struct Hynix_Sensor_reg Hynix_firmware_setting[]=
{
        {0x2000, 0x4031},
        {0x2002, 0x83F8},
        {0x2004, 0x4104},
        {0x2006, 0x4307},
        {0x2008, 0x430A},
        {0x200a, 0x4382},
        {0x200c, 0x80CC},
        {0x200e, 0x4382},
        {0x2010, 0x8070},
        {0x2012, 0x43A2},
        {0x2014, 0x0B80},
        {0x2016, 0x0C0A},
        {0x2018, 0x4382},
        {0x201a, 0x0B90},
        {0x201c, 0x0C0A},
        {0x201e, 0x4382},
        {0x2020, 0x0B9C},
        {0x2022, 0x0C0A},
        {0x2024, 0x93D2},
        {0x2026, 0x003D},
        {0x2028, 0x2002},
        {0x202a, 0x4030},
        {0x202c, 0xF634},
        {0x202e, 0x43C2},
        {0x2030, 0x0F82},
        {0x2032, 0x425F},
        {0x2034, 0x0118},
        {0x2036, 0xF37F},
        {0x2038, 0x930F},
        {0x203a, 0x2002},
        {0x203c, 0x0CC8},
        {0x203e, 0x3FF9},
        {0x2040, 0x4F82},
        {0x2042, 0x8098},
        {0x2044, 0x43D2},
        {0x2046, 0x0A80},
        {0x2048, 0x43D2},
        {0x204a, 0x0180},
        {0x204c, 0x43D2},
        {0x204e, 0x019A},
        {0x2050, 0x40F2},
        {0x2052, 0x0009},
        {0x2054, 0x019B},
        {0x2056, 0x12B0},
        {0x2058, 0xFCBA},
        {0x205a, 0x93D2},
        {0x205c, 0x003E},
        {0x205e, 0x2002},
        {0x2060, 0x4030},
        {0x2062, 0xF518},
        {0x2064, 0x4308},
        {0x2066, 0x5038},
        {0x2068, 0x0030},
        {0x206a, 0x480F},
        {0x206c, 0x12B0},
        {0x206e, 0xFCCC},
        {0x2070, 0x403B},
        {0x2072, 0x7606},
        {0x2074, 0x4B29},
        {0x2076, 0x5318},
        {0x2078, 0x480F},
        {0x207a, 0x12B0},
        {0x207c, 0xFCCC},
        {0x207e, 0x4B2A},
        {0x2080, 0x5318},
        {0x2082, 0x480F},
        {0x2084, 0x12B0},
        {0x2086, 0xFCCC},
        {0x2088, 0x4A0D},
        {0x208a, 0xF03D},
        {0x208c, 0x000F},
        {0x208e, 0x108D},
        {0x2090, 0x4B2E},
        {0x2092, 0x5E0E},
        {0x2094, 0x5E0E},
        {0x2096, 0x5E0E},
        {0x2098, 0x5E0E},
        {0x209a, 0x4A0F},
        {0x209c, 0xC312},
        {0x209e, 0x100F},
        {0x20a0, 0x110F},
        {0x20a2, 0x110F},
        {0x20a4, 0x110F},
        {0x20a6, 0x590D},
        {0x20a8, 0x4D87},
        {0x20aa, 0x5000},
        {0x20ac, 0x5F0E},
        {0x20ae, 0x4E87},
        {0x20b0, 0x6000},
        {0x20b2, 0x5327},
        {0x20b4, 0x5038},
        {0x20b6, 0xFFD1},
        {0x20b8, 0x9038},
        {0x20ba, 0x0300},
        {0x20bc, 0x2BD4},
        {0x20be, 0x0261},
        {0x20c0, 0x0000},
        {0x20c2, 0x43A2},
        {0x20c4, 0x0384},
        {0x20c6, 0x42B2},
        {0x20c8, 0x0386},
        {0x20ca, 0x43C2},
        {0x20cc, 0x0180},
        {0x20ce, 0x43D2},
        {0x20d0, 0x003D},
        {0x20d2, 0x40B2},
        {0x20d4, 0x808B},
        {0x20d6, 0x0B88},
        {0x20d8, 0x0C0A},
        {0x20da, 0x40B2},
        {0x20dc, 0x1009},
        {0x20de, 0x0B8A},
        {0x20e0, 0x0C0A},
        {0x20e2, 0x40B2},
        {0x20e4, 0xC40C},
        {0x20e6, 0x0B8C},
        {0x20e8, 0x0C0A},
        {0x20ea, 0x40B2},
        {0x20ec, 0xC9E1},
        {0x20ee, 0x0B8E},
        {0x20f0, 0x0C0A},
        {0x20f2, 0x40B2},
        {0x20f4, 0x0C1E},
        {0x20f6, 0x0B92},
        {0x20f8, 0x0C0A},
        {0x20fa, 0x43D2},
        {0x20fc, 0x0F82},
        {0x20fe, 0x0C3C},
        {0x2100, 0x0C3C},
        {0x2102, 0x0C3C},
        {0x2104, 0x0C3C},
        {0x2106, 0x421F},
        {0x2108, 0x00A6},
        {0x210a, 0x503F},
        {0x210c, 0x07D0},
        {0x210e, 0x3811},
        {0x2110, 0x4F82},
        {0x2112, 0x7100},
        {0x2114, 0x0004},
        {0x2116, 0x0C0D},
        {0x2118, 0x0005},
        {0x211a, 0x0C04},
        {0x211c, 0x000D},
        {0x211e, 0x0C09},
        {0x2120, 0x003D},
        {0x2122, 0x0C1D},
        {0x2124, 0x003C},
        {0x2126, 0x0C13},
        {0x2128, 0x0004},
        {0x212a, 0x0C09},
        {0x212c, 0x0004},
        {0x212e, 0x533F},
        {0x2130, 0x37EF},
        {0x2132, 0x4392},
        {0x2134, 0x8094},
        {0x2136, 0x4382},
        {0x2138, 0x809C},
        {0x213a, 0x4382},
        {0x213c, 0x80B4},
        {0x213e, 0x4382},
        {0x2140, 0x80BA},
        {0x2142, 0x4382},
        {0x2144, 0x80A0},
        {0x2146, 0x40B2},
        {0x2148, 0x0028},
        {0x214a, 0x7000},
        {0x214c, 0x43A2},
        {0x214e, 0x809E},
        {0x2150, 0xB3E2},
        {0x2152, 0x00B4},
        {0x2154, 0x2402},
        {0x2156, 0x4392},
        {0x2158, 0x809E},
        {0x215a, 0x432A},
        {0x215c, 0xB3D2},
        {0x215e, 0x00B4},
        {0x2160, 0x2002},
        {0x2162, 0x4030},
        {0x2164, 0xF508},
        {0x2166, 0x430A},
        {0x2168, 0x4384},
        {0x216a, 0x0002},
        {0x216c, 0x4384},
        {0x216e, 0x0006},
        {0x2170, 0x4382},
        {0x2172, 0x809A},
        {0x2174, 0x4382},
        {0x2176, 0x8096},
        {0x2178, 0x40B2},
        {0x217a, 0x0005},
        {0x217c, 0x7320},
        {0x217e, 0x4392},
        {0x2180, 0x7326},
        {0x2182, 0x12B0},
        {0x2184, 0xF8C6},
        {0x2186, 0x4392},
        {0x2188, 0x731C},
        {0x218a, 0x9382},
        {0x218c, 0x8094},
        {0x218e, 0x200A},
        {0x2190, 0x0B00},
        {0x2192, 0x7302},
        {0x2194, 0x02BC},
        {0x2196, 0x4382},
        {0x2198, 0x7004},
        {0x219a, 0x430F},
        {0x219c, 0x12B0},
        {0x219e, 0xF6C6},
        {0x21a0, 0x12B0},
        {0x21a2, 0xF8C6},
        {0x21a4, 0x4392},
        {0x21a6, 0x80B8},
        {0x21a8, 0x4382},
        {0x21aa, 0x740E},
        {0x21ac, 0xB3E2},
        {0x21ae, 0x0080},
        {0x21b0, 0x2402},
        {0x21b2, 0x4392},
        {0x21b4, 0x740E},
        {0x21b6, 0x431F},
        {0x21b8, 0x12B0},
        {0x21ba, 0xF6C6},
        {0x21bc, 0x4392},
        {0x21be, 0x7004},
        {0x21c0, 0x4A82},
        {0x21c2, 0x7110},
        {0x21c4, 0x9382},
        {0x21c6, 0x8092},
        {0x21c8, 0x2003},
        {0x21ca, 0x9392},
        {0x21cc, 0x7110},
        {0x21ce, 0x211E},
        {0x21d0, 0x9392},
        {0x21d2, 0x7110},
        {0x21d4, 0x2064},
        {0x21d6, 0x0B00},
        {0x21d8, 0x7302},
        {0x21da, 0x0032},
        {0x21dc, 0x4382},
        {0x21de, 0x7004},
        {0x21e0, 0x0B00},
        {0x21e2, 0x7302},
        {0x21e4, 0x03E8},
        {0x21e6, 0x0800},
        {0x21e8, 0x7114},
        {0x21ea, 0x425F},
        {0x21ec, 0x0C9C},
        {0x21ee, 0x4F4E},
        {0x21f0, 0x430F},
        {0x21f2, 0x4E0D},
        {0x21f4, 0x430C},
        {0x21f6, 0x421F},
        {0x21f8, 0x0C9A},
        {0x21fa, 0xDF0C},
        {0x21fc, 0x1204},
        {0x21fe, 0x440F},
        {0x2200, 0x532F},
        {0x2202, 0x120F},
        {0x2204, 0x1212},
        {0x2206, 0x0CA2},
        {0x2208, 0x403E},
        {0x220a, 0x80BC},
        {0x220c, 0x403F},
        {0x220e, 0x8072},
        {0x2210, 0x12B0},
        {0x2212, 0xF72E},
        {0x2214, 0x4F0B},
        {0x2216, 0x425F},
        {0x2218, 0x0CA0},
        {0x221a, 0x4F4E},
        {0x221c, 0x430F},
        {0x221e, 0x4E0D},
        {0x2220, 0x430C},
        {0x2222, 0x421F},
        {0x2224, 0x0C9E},
        {0x2226, 0xDF0C},
        {0x2228, 0x440F},
        {0x222a, 0x522F},
        {0x222c, 0x120F},
        {0x222e, 0x532F},
        {0x2230, 0x120F},
        {0x2232, 0x1212},
        {0x2234, 0x0CA4},
        {0x2236, 0x403E},
        {0x2238, 0x80A2},
        {0x223a, 0x403F},
        {0x223c, 0x8050},
        {0x223e, 0x12B0},
        {0x2240, 0xF72E},
        {0x2242, 0x4F0D},
        {0x2244, 0x430C},
        {0x2246, 0x441E},
        {0x2248, 0x0004},
        {0x224a, 0x442F},
        {0x224c, 0x5031},
        {0x224e, 0x000C},
        {0x2250, 0x9E0F},
        {0x2252, 0x2C01},
        {0x2254, 0x431C},
        {0x2256, 0x8E0F},
        {0x2258, 0x930F},
        {0x225a, 0x3402},
        {0x225c, 0xE33F},
        {0x225e, 0x531F},
        {0x2260, 0x421E},
        {0x2262, 0x0CA2},
        {0x2264, 0xC312},
        {0x2266, 0x100E},
        {0x2268, 0x9E0F},
        {0x226a, 0x2804},
        {0x226c, 0x930C},
        {0x226e, 0x2001},
        {0x2270, 0x531B},
        {0x2272, 0x5C0D},
        {0x2274, 0x90B2},
        {0x2276, 0x0007},
        {0x2278, 0x80BA},
        {0x227a, 0x240C},
        {0x227c, 0x90B2},
        {0x227e, 0x012C},
        {0x2280, 0x80B4},
        {0x2282, 0x2408},
        {0x2284, 0x0900},
        {0x2286, 0x710E},
        {0x2288, 0x0B00},
        {0x228a, 0x7302},
        {0x228c, 0x0320},
        {0x228e, 0x12B0},
        {0x2290, 0xF666},
        {0x2292, 0x3F98},
        {0x2294, 0x4B82},
        {0x2296, 0x0CAC},
        {0x2298, 0x4D82},
        {0x229a, 0x0CAE},
        {0x229c, 0x3FF3},
        {0x229e, 0x0B00},
        {0x22a0, 0x7302},
        {0x22a2, 0x0002},
        {0x22a4, 0x069A},
        {0x22a6, 0x0C1F},
        {0x22a8, 0x0403},
        {0x22aa, 0x0C05},
        {0x22ac, 0x0001},
        {0x22ae, 0x0C01},
        {0x22b0, 0x0003},
        {0x22b2, 0x0C03},
        {0x22b4, 0x000B},
        {0x22b6, 0x0C33},
        {0x22b8, 0x0003},
        {0x22ba, 0x0C03},
        {0x22bc, 0x0653},
        {0x22be, 0x0C03},
        {0x22c0, 0x065B},
        {0x22c2, 0x0C13},
        {0x22c4, 0x065F},
        {0x22c6, 0x0C43},
        {0x22c8, 0x0657},
        {0x22ca, 0x0C03},
        {0x22cc, 0x0653},
        {0x22ce, 0x0C03},
        {0x22d0, 0x0643},
        {0x22d2, 0x0C0F},
        {0x22d4, 0x067D},
        {0x22d6, 0x0C01},
        {0x22d8, 0x077F},
        {0x22da, 0x0C01},
        {0x22dc, 0x0677},
        {0x22de, 0x0C01},
        {0x22e0, 0x0673},
        {0x22e2, 0x0C67},
        {0x22e4, 0x0677},
        {0x22e6, 0x0C03},
        {0x22e8, 0x077D},
        {0x22ea, 0x0C19},
        {0x22ec, 0x0013},
        {0x22ee, 0x0C27},
        {0x22f0, 0x0003},
        {0x22f2, 0x0C45},
        {0x22f4, 0x0675},
        {0x22f6, 0x0C01},
        {0x22f8, 0x0671},
        {0x22fa, 0x4392},
        {0x22fc, 0x7004},
        {0x22fe, 0x430F},
        {0x2300, 0x9382},
        {0x2302, 0x80B8},
        {0x2304, 0x2001},
        {0x2306, 0x431F},
        {0x2308, 0x4F82},
        {0x230a, 0x80B8},
        {0x230c, 0x930F},
        {0x230e, 0x2472},
        {0x2310, 0x0B00},
        {0x2312, 0x7302},
        {0x2314, 0x033A},
        {0x2316, 0x0675},
        {0x2318, 0x0C02},
        {0x231a, 0x0339},
        {0x231c, 0xAE0C},
        {0x231e, 0x0C01},
        {0x2320, 0x003C},
        {0x2322, 0x0C01},
        {0x2324, 0x0004},
        {0x2326, 0x0C01},
        {0x2328, 0x0642},
        {0x232a, 0x0B00},
        {0x232c, 0x7302},
        {0x232e, 0x0386},
        {0x2330, 0x0643},
        {0x2332, 0x0C05},
        {0x2334, 0x0001},
        {0x2336, 0x0C01},
        {0x2338, 0x0003},
        {0x233a, 0x0C03},
        {0x233c, 0x000B},
        {0x233e, 0x0C33},
        {0x2340, 0x0003},
        {0x2342, 0x0C03},
        {0x2344, 0x0653},
        {0x2346, 0x0C03},
        {0x2348, 0x065B},
        {0x234a, 0x0C13},
        {0x234c, 0x065F},
        {0x234e, 0x0C43},
        {0x2350, 0x0657},
        {0x2352, 0x0C03},
        {0x2354, 0x0653},
        {0x2356, 0x0C03},
        {0x2358, 0x0643},
        {0x235a, 0x0C0F},
        {0x235c, 0x067D},
        {0x235e, 0x0C01},
        {0x2360, 0x077F},
        {0x2362, 0x0C01},
        {0x2364, 0x0677},
        {0x2366, 0x0C01},
        {0x2368, 0x0673},
        {0x236a, 0x0C67},
        {0x236c, 0x0677},
        {0x236e, 0x0C03},
        {0x2370, 0x077D},
        {0x2372, 0x0C19},
        {0x2374, 0x0013},
        {0x2376, 0x0C27},
        {0x2378, 0x0003},
        {0x237a, 0x0C45},
        {0x237c, 0x0675},
        {0x237e, 0x0C01},
        {0x2380, 0x0671},
        {0x2382, 0x12B0},
        {0x2384, 0xF666},
        {0x2386, 0x930F},
        {0x2388, 0x2405},
        {0x238a, 0x4292},
        {0x238c, 0x8094},
        {0x238e, 0x809C},
        {0x2390, 0x4382},
        {0x2392, 0x8094},
        {0x2394, 0x9382},
        {0x2396, 0x80B8},
        {0x2398, 0x241D},
        {0x239a, 0x0B00},
        {0x239c, 0x7302},
        {0x239e, 0x069E},
        {0x23a0, 0x0675},
        {0x23a2, 0x0C02},
        {0x23a4, 0x0339},
        {0x23a6, 0xAE0C},
        {0x23a8, 0x0C01},
        {0x23aa, 0x003C},
        {0x23ac, 0x0C01},
        {0x23ae, 0x0004},
        {0x23b0, 0x0C01},
        {0x23b2, 0x0642},
        {0x23b4, 0x0C01},
        {0x23b6, 0x06A1},
        {0x23b8, 0x0C03},
        {0x23ba, 0x06A0},
        {0x23bc, 0x9382},
        {0x23be, 0x80CC},
        {0x23c0, 0x2003},
        {0x23c2, 0x930F},
        {0x23c4, 0x26FF},
        {0x23c6, 0x3EE1},
        {0x23c8, 0x43C2},
        {0x23ca, 0x0A80},
        {0x23cc, 0x0B00},
        {0x23ce, 0x7302},
        {0x23d0, 0xFFF0},
        {0x23d2, 0x3EF8},
        {0x23d4, 0x0B00},
        {0x23d6, 0x7302},
        {0x23d8, 0x069E},
        {0x23da, 0x0675},
        {0x23dc, 0x0C02},
        {0x23de, 0x0301},
        {0x23e0, 0xAE0C},
        {0x23e2, 0x0C01},
        {0x23e4, 0x0004},
        {0x23e6, 0x0C03},
        {0x23e8, 0x0642},
        {0x23ea, 0x0C01},
        {0x23ec, 0x06A1},
        {0x23ee, 0x0C03},
        {0x23f0, 0x06A0},
        {0x23f2, 0x3FE4},
        {0x23f4, 0x0B00},
        {0x23f6, 0x7302},
        {0x23f8, 0x033A},
        {0x23fa, 0x0675},
        {0x23fc, 0x0C02},
        {0x23fe, 0x0301},
        {0x2400, 0xAE0C},
        {0x2402, 0x0C01},
        {0x2404, 0x0004},
        {0x2406, 0x0C03},
        {0x2408, 0x0642},
        {0x240a, 0x3F8F},
        {0x240c, 0x0B00},
        {0x240e, 0x7302},
        {0x2410, 0x0002},
        {0x2412, 0x069A},
        {0x2414, 0x0C1F},
        {0x2416, 0x0402},
        {0x2418, 0x0C05},
        {0x241a, 0x0001},
        {0x241c, 0x0C01},
        {0x241e, 0x0003},
        {0x2420, 0x0C03},
        {0x2422, 0x000B},
        {0x2424, 0x0C33},
        {0x2426, 0x0003},
        {0x2428, 0x0C03},
        {0x242a, 0x0653},
        {0x242c, 0x0C03},
        {0x242e, 0x065B},
        {0x2430, 0x0C13},
        {0x2432, 0x065F},
        {0x2434, 0x0C43},
        {0x2436, 0x0657},
        {0x2438, 0x0C03},
        {0x243a, 0x0653},
        {0x243c, 0x0C03},
        {0x243e, 0x0643},
        {0x2440, 0x0C0F},
        {0x2442, 0x077D},
        {0x2444, 0x0C01},
        {0x2446, 0x067F},
        {0x2448, 0x0C01},
        {0x244a, 0x0677},
        {0x244c, 0x0C01},
        {0x244e, 0x0673},
        {0x2450, 0x0C5F},
        {0x2452, 0x0663},
        {0x2454, 0x0C6F},
        {0x2456, 0x0667},
        {0x2458, 0x0C01},
        {0x245a, 0x0677},
        {0x245c, 0x0C01},
        {0x245e, 0x077D},
        {0x2460, 0x0C33},
        {0x2462, 0x0013},
        {0x2464, 0x0C27},
        {0x2466, 0x0003},
        {0x2468, 0x0C4F},
        {0x246a, 0x0675},
        {0x246c, 0x0C01},
        {0x246e, 0x0671},
        {0x2470, 0x0CFF},
        {0x2472, 0x0C78},
        {0x2474, 0x0661},
        {0x2476, 0x4392},
        {0x2478, 0x7004},
        {0x247a, 0x430F},
        {0x247c, 0x9382},
        {0x247e, 0x80B8},
        {0x2480, 0x2001},
        {0x2482, 0x431F},
        {0x2484, 0x4F82},
        {0x2486, 0x80B8},
        {0x2488, 0x12B0},
        {0x248a, 0xF666},
        {0x248c, 0x930F},
        {0x248e, 0x2405},
        {0x2490, 0x4292},
        {0x2492, 0x8094},
        {0x2494, 0x809C},
        {0x2496, 0x4382},
        {0x2498, 0x8094},
        {0x249a, 0x9382},
        {0x249c, 0x80B8},
        {0x249e, 0x2019},
        {0x24a0, 0x0B00},
        {0x24a2, 0x7302},
        {0x24a4, 0x0562},
        {0x24a6, 0x0665},
        {0x24a8, 0x0C02},
        {0x24aa, 0x0301},
        {0x24ac, 0xA60C},
        {0x24ae, 0x0204},
        {0x24b0, 0xAE0C},
        {0x24b2, 0x0C03},
        {0x24b4, 0x0642},
        {0x24b6, 0x0C13},
        {0x24b8, 0x06A1},
        {0x24ba, 0x0C03},
        {0x24bc, 0x06A0},
        {0x24be, 0x9382},
        {0x24c0, 0x80CC},
        {0x24c2, 0x277F},
        {0x24c4, 0x43C2},
        {0x24c6, 0x0A80},
        {0x24c8, 0x0B00},
        {0x24ca, 0x7302},
        {0x24cc, 0xFFF0},
        {0x24ce, 0x4030},
        {0x24d0, 0xF1C4},
        {0x24d2, 0x0B00},
        {0x24d4, 0x7302},
        {0x24d6, 0x0562},
        {0x24d8, 0x0665},
        {0x24da, 0x0C02},
        {0x24dc, 0x0339},
        {0x24de, 0xA60C},
        {0x24e0, 0x023C},
        {0x24e2, 0xAE0C},
        {0x24e4, 0x0C01},
        {0x24e6, 0x0004},
        {0x24e8, 0x0C01},
        {0x24ea, 0x0642},
        {0x24ec, 0x0C13},
        {0x24ee, 0x06A1},
        {0x24f0, 0x0C03},
        {0x24f2, 0x06A0},
        {0x24f4, 0x9382},
        {0x24f6, 0x80CC},
        {0x24f8, 0x2764},
        {0x24fa, 0x43C2},
        {0x24fc, 0x0A80},
        {0x24fe, 0x0B00},
        {0x2500, 0x7302},
        {0x2502, 0xFFF0},
        {0x2504, 0x4030},
        {0x2506, 0xF1C4},
        {0x2508, 0xB3E2},
        {0x250a, 0x00B4},
        {0x250c, 0x2002},
        {0x250e, 0x4030},
        {0x2510, 0xF168},
        {0x2512, 0x431A},
        {0x2514, 0x4030},
        {0x2516, 0xF168},
        {0x2518, 0x4392},
        {0x251a, 0x760E},
        {0x251c, 0x425F},
        {0x251e, 0x0118},
        {0x2520, 0xF37F},
        {0x2522, 0x930F},
        {0x2524, 0x2005},
        {0x2526, 0x43C2},
        {0x2528, 0x0A80},
        {0x252a, 0x0B00},
        {0x252c, 0x7302},
        {0x252e, 0xFFF0},
        {0x2530, 0x9382},
        {0x2532, 0x760C},
        {0x2534, 0x2002},
        {0x2536, 0x0C64},
        {0x2538, 0x3FF1},
        {0x253a, 0x4F82},
        {0x253c, 0x8098},
        {0x253e, 0x12B0},
        {0x2540, 0xFCBA},
        {0x2542, 0x421F},
        {0x2544, 0x760A},
        {0x2546, 0x903F},
        {0x2548, 0x0200},
        {0x254a, 0x2469},
        {0x254c, 0x930F},
        {0x254e, 0x2467},
        {0x2550, 0x903F},
        {0x2552, 0x0100},
        {0x2554, 0x23E1},
        {0x2556, 0x40B2},
        {0x2558, 0x0005},
        {0x255a, 0x7600},
        {0x255c, 0x4382},
        {0x255e, 0x7602},
        {0x2560, 0x0262},
        {0x2562, 0x0000},
        {0x2564, 0x0222},
        {0x2566, 0x0000},
        {0x2568, 0x0262},
        {0x256a, 0x0000},
        {0x256c, 0x0260},
        {0x256e, 0x0000},
        {0x2570, 0x425F},
        {0x2572, 0x0186},
        {0x2574, 0x4F4C},
        {0x2576, 0x421B},
        {0x2578, 0x018A},
        {0x257a, 0x93D2},
        {0x257c, 0x018F},
        {0x257e, 0x244D},
        {0x2580, 0x425F},
        {0x2582, 0x018F},
        {0x2584, 0x4F4D},
        {0x2586, 0x4308},
        {0x2588, 0x431F},
        {0x258a, 0x480E},
        {0x258c, 0x930E},
        {0x258e, 0x2403},
        {0x2590, 0x5F0F},
        {0x2592, 0x831E},
        {0x2594, 0x23FD},
        {0x2596, 0xFC0F},
        {0x2598, 0x242E},
        {0x259a, 0x430F},
        {0x259c, 0x9D0F},
        {0x259e, 0x2C2B},
        {0x25a0, 0x4B82},
        {0x25a2, 0x7600},
        {0x25a4, 0x4882},
        {0x25a6, 0x7602},
        {0x25a8, 0x4C82},
        {0x25aa, 0x7604},
        {0x25ac, 0x0264},
        {0x25ae, 0x0000},
        {0x25b0, 0x0224},
        {0x25b2, 0x0000},
        {0x25b4, 0x0264},
        {0x25b6, 0x0000},
        {0x25b8, 0x0260},
        {0x25ba, 0x0000},
        {0x25bc, 0x0268},
        {0x25be, 0x0000},
        {0x25c0, 0x0C18},
        {0x25c2, 0x02E8},
        {0x25c4, 0x0000},
        {0x25c6, 0x0C30},
        {0x25c8, 0x02A8},
        {0x25ca, 0x0000},
        {0x25cc, 0x0C30},
        {0x25ce, 0x0C30},
        {0x25d0, 0x0C30},
        {0x25d2, 0x0C30},
        {0x25d4, 0x0C30},
        {0x25d6, 0x0C30},
        {0x25d8, 0x0C30},
        {0x25da, 0x0C30},
        {0x25dc, 0x0C00},
        {0x25de, 0x02E8},
        {0x25e0, 0x0000},
        {0x25e2, 0x0C30},
        {0x25e4, 0x0268},
        {0x25e6, 0x0000},
        {0x25e8, 0x0C18},
        {0x25ea, 0x0260},
        {0x25ec, 0x0000},
        {0x25ee, 0x0C18},
        {0x25f0, 0x531F},
        {0x25f2, 0x9D0F},
        {0x25f4, 0x2BD5},
        {0x25f6, 0x5318},
        {0x25f8, 0x9238},
        {0x25fa, 0x2BC6},
        {0x25fc, 0x0261},
        {0x25fe, 0x0000},
        {0x2600, 0x12B0},
        {0x2602, 0xFCBA},
        {0x2604, 0x4A0F},
        {0x2606, 0x12B0},
        {0x2608, 0xFCCC},
        {0x260a, 0x421F},
        {0x260c, 0x7606},
        {0x260e, 0x4FC2},
        {0x2610, 0x0188},
        {0x2612, 0x4B0A},
        {0x2614, 0x0261},
        {0x2616, 0x0000},
        {0x2618, 0x3F7F},
        {0x261a, 0x432D},
        {0x261c, 0x3FB4},
        {0x261e, 0x421F},
        {0x2620, 0x018A},
        {0x2622, 0x12B0},
        {0x2624, 0xFCCC},
        {0x2626, 0x421F},
        {0x2628, 0x7606},
        {0x262a, 0x4FC2},
        {0x262c, 0x0188},
        {0x262e, 0x0261},
        {0x2630, 0x0000},
        {0x2632, 0x3F72},
        {0x2634, 0x4382},
        {0x2636, 0x0B88},
        {0x2638, 0x0C0A},
        {0x263a, 0x4382},
        {0x263c, 0x0B8A},
        {0x263e, 0x0C0A},
        {0x2640, 0x40B2},
        {0x2642, 0x000C},
        {0x2644, 0x0B8C},
        {0x2646, 0x0C0A},
        {0x2648, 0x40B2},
        {0x264a, 0xB5E1},
        {0x264c, 0x0B8E},
        {0x264e, 0x0C0A},
        {0x2650, 0x40B2},
        {0x2652, 0x641C},
        {0x2654, 0x0B92},
        {0x2656, 0x0C0A},
        {0x2658, 0x43C2},
        {0x265a, 0x003D},
        {0x265c, 0x4030},
        {0x265e, 0xF02E},
        {0x2660, 0x5231},
        {0x2662, 0x4030},
        {0x2664, 0xFCE8},
        {0x2666, 0xE3B2},
        {0x2668, 0x740E},
        {0x266a, 0x425F},
        {0x266c, 0x0118},
        {0x266e, 0xF37F},
        {0x2670, 0x4F82},
        {0x2672, 0x8098},
        {0x2674, 0x930F},
        {0x2676, 0x2005},
        {0x2678, 0x93C2},
        {0x267a, 0x0A82},
        {0x267c, 0x2402},
        {0x267e, 0x4392},
        {0x2680, 0x80CC},
        {0x2682, 0x9382},
        {0x2684, 0x8098},
        {0x2686, 0x2002},
        {0x2688, 0x4392},
        {0x268a, 0x8070},
        {0x268c, 0x421F},
        {0x268e, 0x710E},
        {0x2690, 0x93A2},
        {0x2692, 0x7110},
        {0x2694, 0x2411},
        {0x2696, 0x9382},
        {0x2698, 0x710E},
        {0x269a, 0x240C},
        {0x269c, 0x5292},
        {0x269e, 0x809E},
        {0x26a0, 0x7110},
        {0x26a2, 0x4382},
        {0x26a4, 0x740E},
        {0x26a6, 0x9382},
        {0x26a8, 0x80B6},
        {0x26aa, 0x2402},
        {0x26ac, 0x4392},
        {0x26ae, 0x740E},
        {0x26b0, 0x4392},
        {0x26b2, 0x80B8},
        {0x26b4, 0x430F},
        {0x26b6, 0x4130},
        {0x26b8, 0xF31F},
        {0x26ba, 0x27ED},
        {0x26bc, 0x40B2},
        {0x26be, 0x0003},
        {0x26c0, 0x7110},
        {0x26c2, 0x431F},
        {0x26c4, 0x4130},
        {0x26c6, 0x4F0E},
        {0x26c8, 0x421D},
        {0x26ca, 0x8070},
        {0x26cc, 0x425F},
        {0x26ce, 0x0118},
        {0x26d0, 0xF37F},
        {0x26d2, 0x903E},
        {0x26d4, 0x0003},
        {0x26d6, 0x2405},
        {0x26d8, 0x931E},
        {0x26da, 0x2403},
        {0x26dc, 0x0B00},
        {0x26de, 0x7302},
        {0x26e0, 0x0384},
        {0x26e2, 0x930F},
        {0x26e4, 0x241A},
        {0x26e6, 0x930D},
        {0x26e8, 0x2018},
        {0x26ea, 0x9382},
        {0x26ec, 0x7308},
        {0x26ee, 0x2402},
        {0x26f0, 0x930E},
        {0x26f2, 0x2419},
        {0x26f4, 0x9382},
        {0x26f6, 0x7328},
        {0x26f8, 0x2402},
        {0x26fa, 0x931E},
        {0x26fc, 0x2414},
        {0x26fe, 0x9382},
        {0x2700, 0x710E},
        {0x2702, 0x2402},
        {0x2704, 0x932E},
        {0x2706, 0x240F},
        {0x2708, 0x9382},
        {0x270a, 0x7114},
        {0x270c, 0x2402},
        {0x270e, 0x922E},
        {0x2710, 0x240A},
        {0x2712, 0x903E},
        {0x2714, 0x0003},
        {0x2716, 0x23DA},
        {0x2718, 0x3C06},
        {0x271a, 0x43C2},
        {0x271c, 0x0A80},
        {0x271e, 0x0B00},
        {0x2720, 0x7302},
        {0x2722, 0xFFF0},
        {0x2724, 0x3FD3},
        {0x2726, 0x4F82},
        {0x2728, 0x8098},
        {0x272a, 0x431F},
        {0x272c, 0x4130},
        {0x272e, 0x120B},
        {0x2730, 0x120A},
        {0x2732, 0x1209},
        {0x2734, 0x1208},
        {0x2736, 0x1207},
        {0x2738, 0x1206},
        {0x273a, 0x1205},
        {0x273c, 0x1204},
        {0x273e, 0x8221},
        {0x2740, 0x403B},
        {0x2742, 0x0016},
        {0x2744, 0x510B},
        {0x2746, 0x4F08},
        {0x2748, 0x4E09},
        {0x274a, 0x4BA1},
        {0x274c, 0x0000},
        {0x274e, 0x4B1A},
        {0x2750, 0x0002},
        {0x2752, 0x4B91},
        {0x2754, 0x0004},
        {0x2756, 0x0002},
        {0x2758, 0x4304},
        {0x275a, 0x4305},
        {0x275c, 0x4306},
        {0x275e, 0x4307},
        {0x2760, 0x9382},
        {0x2762, 0x80B2},
        {0x2764, 0x2425},
        {0x2766, 0x438A},
        {0x2768, 0x0000},
        {0x276a, 0x430B},
        {0x276c, 0x4B0F},
        {0x276e, 0x5F0F},
        {0x2770, 0x5F0F},
        {0x2772, 0x580F},
        {0x2774, 0x4C8F},
        {0x2776, 0x0000},
        {0x2778, 0x4D8F},
        {0x277a, 0x0002},
        {0x277c, 0x4B0F},
        {0x277e, 0x5F0F},
        {0x2780, 0x590F},
        {0x2782, 0x41AF},
        {0x2784, 0x0000},
        {0x2786, 0x531B},
        {0x2788, 0x923B},
        {0x278a, 0x2BF0},
        {0x278c, 0x430B},
        {0x278e, 0x4B0F},
        {0x2790, 0x5F0F},
        {0x2792, 0x5F0F},
        {0x2794, 0x580F},
        {0x2796, 0x5F34},
        {0x2798, 0x6F35},
        {0x279a, 0x4B0F},
        {0x279c, 0x5F0F},
        {0x279e, 0x590F},
        {0x27a0, 0x4F2E},
        {0x27a2, 0x430F},
        {0x27a4, 0x5E06},
        {0x27a6, 0x6F07},
        {0x27a8, 0x531B},
        {0x27aa, 0x923B},
        {0x27ac, 0x2BF0},
        {0x27ae, 0x3C18},
        {0x27b0, 0x4A2E},
        {0x27b2, 0x4E0F},
        {0x27b4, 0x5F0F},
        {0x27b6, 0x5F0F},
        {0x27b8, 0x580F},
        {0x27ba, 0x4C8F},
        {0x27bc, 0x0000},
        {0x27be, 0x4D8F},
        {0x27c0, 0x0002},
        {0x27c2, 0x5E0E},
        {0x27c4, 0x590E},
        {0x27c6, 0x41AE},
        {0x27c8, 0x0000},
        {0x27ca, 0x4A2F},
        {0x27cc, 0x903F},
        {0x27ce, 0x0007},
        {0x27d0, 0x2404},
        {0x27d2, 0x531F},
        {0x27d4, 0x4F8A},
        {0x27d6, 0x0000},
        {0x27d8, 0x3FD9},
        {0x27da, 0x438A},
        {0x27dc, 0x0000},
        {0x27de, 0x3FD6},
        {0x27e0, 0x440C},
        {0x27e2, 0x450D},
        {0x27e4, 0x460A},
        {0x27e6, 0x470B},
        {0x27e8, 0x12B0},
        {0x27ea, 0xFD22},
        {0x27ec, 0x4C08},
        {0x27ee, 0x4D09},
        {0x27f0, 0x4C0E},
        {0x27f2, 0x430F},
        {0x27f4, 0x4E0A},
        {0x27f6, 0x4F0B},
        {0x27f8, 0x460C},
        {0x27fa, 0x470D},
        {0x27fc, 0x12B0},
        {0x27fe, 0xFD02},
        {0x2800, 0x8E04},
        {0x2802, 0x7F05},
        {0x2804, 0x440E},
        {0x2806, 0x450F},
        {0x2808, 0xC312},
        {0x280a, 0x100F},
        {0x280c, 0x100E},
        {0x280e, 0x110F},
        {0x2810, 0x100E},
        {0x2812, 0x110F},
        {0x2814, 0x100E},
        {0x2816, 0x411D},
        {0x2818, 0x0002},
        {0x281a, 0x4E8D},
        {0x281c, 0x0000},
        {0x281e, 0x480F},
        {0x2820, 0x5221},
        {0x2822, 0x4134},
        {0x2824, 0x4135},
        {0x2826, 0x4136},
        {0x2828, 0x4137},
        {0x282a, 0x4138},
        {0x282c, 0x4139},
        {0x282e, 0x413A},
        {0x2830, 0x413B},
        {0x2832, 0x4130},
        {0x2834, 0x120A},
        {0x2836, 0x4F0D},
        {0x2838, 0x4E0C},
        {0x283a, 0x425F},
        {0x283c, 0x00BA},
        {0x283e, 0x4F4A},
        {0x2840, 0x503A},
        {0x2842, 0x0010},
        {0x2844, 0x931D},
        {0x2846, 0x242B},
        {0x2848, 0x932D},
        {0x284a, 0x2421},
        {0x284c, 0x903D},
        {0x284e, 0x0003},
        {0x2850, 0x2418},
        {0x2852, 0x922D},
        {0x2854, 0x2413},
        {0x2856, 0x903D},
        {0x2858, 0x0005},
        {0x285a, 0x2407},
        {0x285c, 0x903D},
        {0x285e, 0x0006},
        {0x2860, 0x2028},
        {0x2862, 0xC312},
        {0x2864, 0x100A},
        {0x2866, 0x110A},
        {0x2868, 0x3C24},
        {0x286a, 0x4A0E},
        {0x286c, 0xC312},
        {0x286e, 0x100E},
        {0x2870, 0x110E},
        {0x2872, 0x4E0F},
        {0x2874, 0x110F},
        {0x2876, 0x4E0A},
        {0x2878, 0x5F0A},
        {0x287a, 0x3C1B},
        {0x287c, 0xC312},
        {0x287e, 0x100A},
        {0x2880, 0x3C18},
        {0x2882, 0x4A0E},
        {0x2884, 0xC312},
        {0x2886, 0x100E},
        {0x2888, 0x4E0F},
        {0x288a, 0x110F},
        {0x288c, 0x3FF3},
        {0x288e, 0x4A0F},
        {0x2890, 0xC312},
        {0x2892, 0x100F},
        {0x2894, 0x4F0E},
        {0x2896, 0x110E},
        {0x2898, 0x4F0A},
        {0x289a, 0x5E0A},
        {0x289c, 0x3C0A},
        {0x289e, 0x4A0F},
        {0x28a0, 0xC312},
        {0x28a2, 0x100F},
        {0x28a4, 0x4F0E},
        {0x28a6, 0x110E},
        {0x28a8, 0x4F0A},
        {0x28aa, 0x5E0A},
        {0x28ac, 0x4E0F},
        {0x28ae, 0x110F},
        {0x28b0, 0x3FE3},
        {0x28b2, 0x12B0},
        {0x28b4, 0xFCEC},
        {0x28b6, 0x4E0F},
        {0x28b8, 0xC312},
        {0x28ba, 0x100F},
        {0x28bc, 0x110F},
        {0x28be, 0x110F},
        {0x28c0, 0x110F},
        {0x28c2, 0x413A},
        {0x28c4, 0x4130},
        {0x28c6, 0x120B},
        {0x28c8, 0x120A},
        {0x28ca, 0x1209},
        {0x28cc, 0x1208},
        {0x28ce, 0x425F},
        {0x28d0, 0x0080},
        {0x28d2, 0xF36F},
        {0x28d4, 0x4F0E},
        {0x28d6, 0xF32E},
        {0x28d8, 0x4E82},
        {0x28da, 0x80B6},
        {0x28dc, 0xB3D2},
        {0x28de, 0x0786},
        {0x28e0, 0x2402},
        {0x28e2, 0x4392},
        {0x28e4, 0x7002},
        {0x28e6, 0x4392},
        {0x28e8, 0x80B8},
        {0x28ea, 0x4382},
        {0x28ec, 0x740E},
        {0x28ee, 0x9382},
        {0x28f0, 0x80B6},
        {0x28f2, 0x2402},
        {0x28f4, 0x4392},
        {0x28f6, 0x740E},
        {0x28f8, 0x93C2},
        {0x28fa, 0x00C6},
        {0x28fc, 0x2406},
        {0x28fe, 0xB392},
        {0x2900, 0x732A},
        {0x2902, 0x2403},
        {0x2904, 0xB3D2},
        {0x2906, 0x00C7},
        {0x2908, 0x2412},
        {0x290a, 0x4292},
        {0x290c, 0x01A8},
        {0x290e, 0x0688},
        {0x2910, 0x4292},
        {0x2912, 0x01AA},
        {0x2914, 0x068A},
        {0x2916, 0x4292},
        {0x2918, 0x01AC},
        {0x291a, 0x068C},
        {0x291c, 0x4292},
        {0x291e, 0x01AE},
        {0x2920, 0x068E},
        {0x2922, 0x4292},
        {0x2924, 0x0190},
        {0x2926, 0x0A92},
        {0x2928, 0x4292},
        {0x292a, 0x0192},
        {0x292c, 0x0A94},
        {0x292e, 0x430E},
        {0x2930, 0x425F},
        {0x2932, 0x00C7},
        {0x2934, 0xF35F},
        {0x2936, 0xF37F},
        {0x2938, 0xF21F},
        {0x293a, 0x732A},
        {0x293c, 0x200C},
        {0x293e, 0xB3D2},
        {0x2940, 0x00C7},
        {0x2942, 0x2003},
        {0x2944, 0xB392},
        {0x2946, 0x80A0},
        {0x2948, 0x2006},
        {0x294a, 0xB3A2},
        {0x294c, 0x732A},
        {0x294e, 0x2003},
        {0x2950, 0x9382},
        {0x2952, 0x8094},
        {0x2954, 0x2401},
        {0x2956, 0x431E},
        {0x2958, 0x4E82},
        {0x295a, 0x80B2},
        {0x295c, 0x930E},
        {0x295e, 0x258F},
        {0x2960, 0x4382},
        {0x2962, 0x80BA},
        {0x2964, 0x421F},
        {0x2966, 0x732A},
        {0x2968, 0xF31F},
        {0x296a, 0x4F82},
        {0x296c, 0x80A0},
        {0x296e, 0x425F},
        {0x2970, 0x008C},
        {0x2972, 0x4FC2},
        {0x2974, 0x8092},
        {0x2976, 0x43C2},
        {0x2978, 0x8093},
        {0x297a, 0x425F},
        {0x297c, 0x009E},
        {0x297e, 0x4F4A},
        {0x2980, 0x425F},
        {0x2982, 0x009F},
        {0x2984, 0xF37F},
        {0x2986, 0x5F0A},
        {0x2988, 0x110A},
        {0x298a, 0x110A},
        {0x298c, 0x425F},
        {0x298e, 0x00B2},
        {0x2990, 0x4F4B},
        {0x2992, 0x425F},
        {0x2994, 0x00B3},
        {0x2996, 0xF37F},
        {0x2998, 0x5F0B},
        {0x299a, 0x110B},
        {0x299c, 0x110B},
        {0x299e, 0x403F},
        {0x29a0, 0x00BA},
        {0x29a2, 0x4F6E},
        {0x29a4, 0x403C},
        {0x29a6, 0x0813},
        {0x29a8, 0x403D},
        {0x29aa, 0x007F},
        {0x29ac, 0x90FF},
        {0x29ae, 0x0011},
        {0x29b0, 0x0000},
        {0x29b2, 0x2D13},
        {0x29b4, 0x403C},
        {0x29b6, 0x1009},
        {0x29b8, 0x430D},
        {0x29ba, 0x425E},
        {0x29bc, 0x00BA},
        {0x29be, 0x4E4F},
        {0x29c0, 0x108F},
        {0x29c2, 0xDD0F},
        {0x29c4, 0x4F82},
        {0x29c6, 0x0B90},
        {0x29c8, 0x0C0A},
        {0x29ca, 0x4C82},
        {0x29cc, 0x0B8A},
        {0x29ce, 0x0C0A},
        {0x29d0, 0x425F},
        {0x29d2, 0x0C87},
        {0x29d4, 0x4F4E},
        {0x29d6, 0x425F},
        {0x29d8, 0x0C88},
        {0x29da, 0xF37F},
        {0x29dc, 0x12B0},
        {0x29de, 0xF834},
        {0x29e0, 0x4F82},
        {0x29e2, 0x0C8C},
        {0x29e4, 0x425F},
        {0x29e6, 0x0C85},
        {0x29e8, 0x4F4E},
        {0x29ea, 0x425F},
        {0x29ec, 0x0C89},
        {0x29ee, 0xF37F},
        {0x29f0, 0x12B0},
        {0x29f2, 0xF834},
        {0x29f4, 0x4F82},
        {0x29f6, 0x0C8A},
        {0x29f8, 0x425E},
        {0x29fa, 0x00B7},
        {0x29fc, 0x5E4E},
        {0x29fe, 0x4EC2},
        {0x2a00, 0x0CB0},
        {0x2a02, 0x425F},
        {0x2a04, 0x00B8},
        {0x2a06, 0x5F4F},
        {0x2a08, 0x4FC2},
        {0x2a0a, 0x0CB1},
        {0x2a0c, 0x4A0E},
        {0x2a0e, 0x5E0E},
        {0x2a10, 0x5E0E},
        {0x2a12, 0x5E0E},
        {0x2a14, 0x5E0E},
        {0x2a16, 0x4B0F},
        {0x2a18, 0x5F0F},
        {0x2a1a, 0x5F0F},
        {0x2a1c, 0x5F0F},
        {0x2a1e, 0x5F0F},
        {0x2a20, 0x5F0F},
        {0x2a22, 0x5F0F},
        {0x2a24, 0x5F0F},
        {0x2a26, 0xDF0E},
        {0x2a28, 0x4E82},
        {0x2a2a, 0x0A8E},
        {0x2a2c, 0xB22B},
        {0x2a2e, 0x2401},
        {0x2a30, 0x533B},
        {0x2a32, 0xB3E2},
        {0x2a34, 0x0080},
        {0x2a36, 0x2403},
        {0x2a38, 0x40F2},
        {0x2a3a, 0x0003},
        {0x2a3c, 0x00B5},
        {0x2a3e, 0x40B2},
        {0x2a40, 0x1000},
        {0x2a42, 0x7500},
        {0x2a44, 0x40B2},
        {0x2a46, 0x1001},
        {0x2a48, 0x7502},
        {0x2a4a, 0x40B2},
        {0x2a4c, 0x0803},
        {0x2a4e, 0x7504},
        {0x2a50, 0x40B2},
        {0x2a52, 0x080F},
        {0x2a54, 0x7506},
        {0x2a56, 0x40B2},
        {0x2a58, 0x6003},
        {0x2a5a, 0x7508},
        {0x2a5c, 0x40B2},
        {0x2a5e, 0x0801},
        {0x2a60, 0x750A},
        {0x2a62, 0x40B2},
        {0x2a64, 0x0800},
        {0x2a66, 0x750C},
        {0x2a68, 0x40B2},
        {0x2a6a, 0x1400},
        {0x2a6c, 0x750E},
        {0x2a6e, 0x403F},
        {0x2a70, 0x0003},
        {0x2a72, 0x12B0},
        {0x2a74, 0xF6C6},
        {0x2a76, 0x421F},
        {0x2a78, 0x0098},
        {0x2a7a, 0x821F},
        {0x2a7c, 0x0092},
        {0x2a7e, 0x531F},
        {0x2a80, 0xC312},
        {0x2a82, 0x100F},
        {0x2a84, 0x4F82},
        {0x2a86, 0x0A86},
        {0x2a88, 0x421F},
        {0x2a8a, 0x00AC},
        {0x2a8c, 0x821F},
        {0x2a8e, 0x00A6},
        {0x2a90, 0x531F},
        {0x2a92, 0x4F82},
        {0x2a94, 0x0A88},
        {0x2a96, 0xB0B2},
        {0x2a98, 0x0010},
        {0x2a9a, 0x0A84},
        {0x2a9c, 0x248F},
        {0x2a9e, 0x421E},
        {0x2aa0, 0x068C},
        {0x2aa2, 0xC312},
        {0x2aa4, 0x100E},
        {0x2aa6, 0x4E82},
        {0x2aa8, 0x0782},
        {0x2aaa, 0x4292},
        {0x2aac, 0x068E},
        {0x2aae, 0x0784},
        {0x2ab0, 0xB3D2},
        {0x2ab2, 0x0CB6},
        {0x2ab4, 0x2418},
        {0x2ab6, 0x421A},
        {0x2ab8, 0x0CB8},
        {0x2aba, 0x430B},
        {0x2abc, 0x425F},
        {0x2abe, 0x0CBA},
        {0x2ac0, 0x4F4E},
        {0x2ac2, 0x430F},
        {0x2ac4, 0x4E0F},
        {0x2ac6, 0x430E},
        {0x2ac8, 0xDE0A},
        {0x2aca, 0xDF0B},
        {0x2acc, 0x421F},
        {0x2ace, 0x0CBC},
        {0x2ad0, 0x4F0C},
        {0x2ad2, 0x430D},
        {0x2ad4, 0x421F},
        {0x2ad6, 0x0CBE},
        {0x2ad8, 0x430E},
        {0x2ada, 0xDE0C},
        {0x2adc, 0xDF0D},
        {0x2ade, 0x12B0},
        {0x2ae0, 0xFD22},
        {0x2ae2, 0x4C82},
        {0x2ae4, 0x0194},
        {0x2ae6, 0xB2A2},
        {0x2ae8, 0x0A84},
        {0x2aea, 0x2412},
        {0x2aec, 0x421E},
        {0x2aee, 0x0A96},
        {0x2af0, 0xC312},
        {0x2af2, 0x100E},
        {0x2af4, 0x110E},
        {0x2af6, 0x110E},
        {0x2af8, 0x43C2},
        {0x2afa, 0x0A98},
        {0x2afc, 0x431D},
        {0x2afe, 0x4E0F},
        {0x2b00, 0x9F82},
        {0x2b02, 0x0194},
        {0x2b04, 0x2850},
        {0x2b06, 0x5E0F},
        {0x2b08, 0x531D},
        {0x2b0a, 0x903D},
        {0x2b0c, 0x0009},
        {0x2b0e, 0x2BF8},
        {0x2b10, 0x4292},
        {0x2b12, 0x0084},
        {0x2b14, 0x7524},
        {0x2b16, 0x4292},
        {0x2b18, 0x0088},
        {0x2b1a, 0x7316},
        {0x2b1c, 0x9382},
        {0x2b1e, 0x8092},
        {0x2b20, 0x2403},
        {0x2b22, 0x4292},
        {0x2b24, 0x008A},
        {0x2b26, 0x7316},
        {0x2b28, 0x430E},
        {0x2b2a, 0x421F},
        {0x2b2c, 0x0086},
        {0x2b2e, 0x822F},
        {0x2b30, 0x9F82},
        {0x2b32, 0x0084},
        {0x2b34, 0x2801},
        {0x2b36, 0x431E},
        {0x2b38, 0x4292},
        {0x2b3a, 0x0086},
        {0x2b3c, 0x7314},
        {0x2b3e, 0x93C2},
        {0x2b40, 0x00BC},
        {0x2b42, 0x2007},
        {0x2b44, 0xB31E},
        {0x2b46, 0x2405},
        {0x2b48, 0x421F},
        {0x2b4a, 0x0084},
        {0x2b4c, 0x522F},
        {0x2b4e, 0x4F82},
        {0x2b50, 0x7314},
        {0x2b52, 0x425F},
        {0x2b54, 0x00BC},
        {0x2b56, 0xF37F},
        {0x2b58, 0xFE0F},
        {0x2b5a, 0x2406},
        {0x2b5c, 0x421E},
        {0x2b5e, 0x0086},
        {0x2b60, 0x503E},
        {0x2b62, 0xFFFB},
        {0x2b64, 0x4E82},
        {0x2b66, 0x7524},
        {0x2b68, 0x430E},
        {0x2b6a, 0x421F},
        {0x2b6c, 0x7524},
        {0x2b6e, 0x9F82},
        {0x2b70, 0x809A},
        {0x2b72, 0x2C07},
        {0x2b74, 0x9382},
        {0x2b76, 0x8094},
        {0x2b78, 0x2004},
        {0x2b7a, 0x9382},
        {0x2b7c, 0x8096},
        {0x2b7e, 0x2001},
        {0x2b80, 0x431E},
        {0x2b82, 0x40B2},
        {0x2b84, 0x0032},
        {0x2b86, 0x7522},
        {0x2b88, 0x4292},
        {0x2b8a, 0x7524},
        {0x2b8c, 0x809A},
        {0x2b8e, 0x930E},
        {0x2b90, 0x248B},
        {0x2b92, 0x421F},
        {0x2b94, 0x7316},
        {0x2b96, 0xC312},
        {0x2b98, 0x100F},
        {0x2b9a, 0x832F},
        {0x2b9c, 0x4F82},
        {0x2b9e, 0x7522},
        {0x2ba0, 0x53B2},
        {0x2ba2, 0x7524},
        {0x2ba4, 0x3C81},
        {0x2ba6, 0x431F},
        {0x2ba8, 0x4D0E},
        {0x2baa, 0x533E},
        {0x2bac, 0x930E},
        {0x2bae, 0x2403},
        {0x2bb0, 0x5F0F},
        {0x2bb2, 0x831E},
        {0x2bb4, 0x23FD},
        {0x2bb6, 0x4FC2},
        {0x2bb8, 0x0A98},
        {0x2bba, 0x3FAA},
        {0x2bbc, 0x4292},
        {0x2bbe, 0x0A86},
        {0x2bc0, 0x0782},
        {0x2bc2, 0x421F},
        {0x2bc4, 0x0A88},
        {0x2bc6, 0x4B0E},
        {0x2bc8, 0x930E},
        {0x2bca, 0x2404},
        {0x2bcc, 0xC312},
        {0x2bce, 0x100F},
        {0x2bd0, 0x831E},
        {0x2bd2, 0x23FC},
        {0x2bd4, 0x4F82},
        {0x2bd6, 0x0784},
        {0x2bd8, 0x3F6B},
        {0x2bda, 0x90F2},
        {0x2bdc, 0x0011},
        {0x2bde, 0x00BA},
        {0x2be0, 0x280A},
        {0x2be2, 0x90F2},
        {0x2be4, 0x0051},
        {0x2be6, 0x00BA},
        {0x2be8, 0x2C06},
        {0x2bea, 0x403C},
        {0x2bec, 0x0A13},
        {0x2bee, 0x425E},
        {0x2bf0, 0x00BA},
        {0x2bf2, 0x535E},
        {0x2bf4, 0x3EE4},
        {0x2bf6, 0x90F2},
        {0x2bf8, 0x0051},
        {0x2bfa, 0x00BA},
        {0x2bfc, 0x2804},
        {0x2bfe, 0x90F2},
        {0x2c00, 0xFF81},
        {0x2c02, 0x00BA},
        {0x2c04, 0x2808},
        {0x2c06, 0x90F2},
        {0x2c08, 0xFF81},
        {0x2c0a, 0x00BA},
        {0x2c0c, 0x2807},
        {0x2c0e, 0x90F2},
        {0x2c10, 0xFF91},
        {0x2c12, 0x00BA},
        {0x2c14, 0x2C03},
        {0x2c16, 0x403C},
        {0x2c18, 0x0813},
        {0x2c1a, 0x3FE9},
        {0x2c1c, 0x90F2},
        {0x2c1e, 0xFF91},
        {0x2c20, 0x00BA},
        {0x2c22, 0x280A},
        {0x2c24, 0x90F2},
        {0x2c26, 0xFFB1},
        {0x2c28, 0x00BA},
        {0x2c2a, 0x2C06},
        {0x2c2c, 0x403D},
        {0x2c2e, 0x0060},
        {0x2c30, 0x425E},
        {0x2c32, 0x00BA},
        {0x2c34, 0x536E},
        {0x2c36, 0x3EC3},
        {0x2c38, 0x90F2},
        {0x2c3a, 0xFFB1},
        {0x2c3c, 0x00BA},
        {0x2c3e, 0x2804},
        {0x2c40, 0x90F2},
        {0x2c42, 0xFFC1},
        {0x2c44, 0x00BA},
        {0x2c46, 0x2808},
        {0x2c48, 0x90F2},
        {0x2c4a, 0xFFC1},
        {0x2c4c, 0x00BA},
        {0x2c4e, 0x280B},
        {0x2c50, 0x90F2},
        {0x2c52, 0xFFD1},
        {0x2c54, 0x00BA},
        {0x2c56, 0x2C07},
        {0x2c58, 0x403D},
        {0x2c5a, 0x0050},
        {0x2c5c, 0x425E},
        {0x2c5e, 0x00BA},
        {0x2c60, 0x507E},
        {0x2c62, 0x0003},
        {0x2c64, 0x3EAC},
        {0x2c66, 0x403D},
        {0x2c68, 0x003C},
        {0x2c6a, 0x403F},
        {0x2c6c, 0x00BA},
        {0x2c6e, 0x4F6E},
        {0x2c70, 0x526E},
        {0x2c72, 0x90FF},
        {0x2c74, 0xFFFB},
        {0x2c76, 0x0000},
        {0x2c78, 0x2AA2},
        {0x2c7a, 0x437E},
        {0x2c7c, 0x3EA0},
        {0x2c7e, 0x421F},
        {0x2c80, 0x80BA},
        {0x2c82, 0x903F},
        {0x2c84, 0x0009},
        {0x2c86, 0x2C04},
        {0x2c88, 0x531F},
        {0x2c8a, 0x4F82},
        {0x2c8c, 0x80BA},
        {0x2c8e, 0x3E6A},
        {0x2c90, 0x421F},
        {0x2c92, 0x80B4},
        {0x2c94, 0x903F},
        {0x2c96, 0x012E},
        {0x2c98, 0x2C04},
        {0x2c9a, 0x531F},
        {0x2c9c, 0x4F82},
        {0x2c9e, 0x80B4},
        {0x2ca0, 0x3E61},
        {0x2ca2, 0x4382},
        {0x2ca4, 0x80B4},
        {0x2ca6, 0x3E5E},
        {0x2ca8, 0x4E82},
        {0x2caa, 0x8096},
        {0x2cac, 0xD392},
        {0x2cae, 0x7102},
        {0x2cb0, 0x4138},
        {0x2cb2, 0x4139},
        {0x2cb4, 0x413A},
        {0x2cb6, 0x413B},
        {0x2cb8, 0x4130},
        {0x2cba, 0x0260},
        {0x2cbc, 0x0000},
        {0x2cbe, 0x0C18},
        {0x2cc0, 0x0240},
        {0x2cc2, 0x0000},
        {0x2cc4, 0x0260},
        {0x2cc6, 0x0000},
        {0x2cc8, 0x0C05},
        {0x2cca, 0x4130},
        {0x2ccc, 0x4382},
        {0x2cce, 0x7602},
        {0x2cd0, 0x4F82},
        {0x2cd2, 0x7600},
        {0x2cd4, 0x0270},
        {0x2cd6, 0x0000},
        {0x2cd8, 0x0C07},
        {0x2cda, 0x0270},
        {0x2cdc, 0x0001},
        {0x2cde, 0x421F},
        {0x2ce0, 0x7606},
        {0x2ce2, 0x4FC2},
        {0x2ce4, 0x0188},
        {0x2ce6, 0x4130},
        {0x2ce8, 0xDF02},
        {0x2cea, 0x3FFE},
        {0x2cec, 0x430E},
        {0x2cee, 0x930A},
        {0x2cf0, 0x2407},
        {0x2cf2, 0xC312},
        {0x2cf4, 0x100C},
        {0x2cf6, 0x2801},
        {0x2cf8, 0x5A0E},
        {0x2cfa, 0x5A0A},
        {0x2cfc, 0x930C},
        {0x2cfe, 0x23F7},
        {0x2d00, 0x4130},
        {0x2d02, 0x430E},
        {0x2d04, 0x430F},
        {0x2d06, 0x3C08},
        {0x2d08, 0xC312},
        {0x2d0a, 0x100D},
        {0x2d0c, 0x100C},
        {0x2d0e, 0x2802},
        {0x2d10, 0x5A0E},
        {0x2d12, 0x6B0F},
        {0x2d14, 0x5A0A},
        {0x2d16, 0x6B0B},
        {0x2d18, 0x930C},
        {0x2d1a, 0x23F6},
        {0x2d1c, 0x930D},
        {0x2d1e, 0x23F4},
        {0x2d20, 0x4130},
        {0x2d22, 0xEF0F},
        {0x2d24, 0xEE0E},
        {0x2d26, 0x4039},
        {0x2d28, 0x0021},
        {0x2d2a, 0x3C0A},
        {0x2d2c, 0x1008},
        {0x2d2e, 0x6E0E},
        {0x2d30, 0x6F0F},
        {0x2d32, 0x9B0F},
        {0x2d34, 0x2805},
        {0x2d36, 0x2002},
        {0x2d38, 0x9A0E},
        {0x2d3a, 0x2802},
        {0x2d3c, 0x8A0E},
        {0x2d3e, 0x7B0F},
        {0x2d40, 0x6C0C},
        {0x2d42, 0x6D0D},
        {0x2d44, 0x6808},
        {0x2d46, 0x8319},
        {0x2d48, 0x23F1},
        {0x2d4a, 0x4130},
        {0x2d4c, 0x0000},
        
        {HYNIX_TABLE_END, 0x0000}, // Burst mode End Command. 
};
static void HI545_Sensor_Init(void)
{

	HI545DB("HI545_Sensor_Init 2lane_OB:\n ");	
	
    ReEnteyCamera = KAL_TRUE;
	mDELAY(20);

	//////////////////////////////////////////////////////////////////////////
	//<<<<<<<<<  Sensor Information  >>>>>>>>>>///////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//
	//	Sensor			: Hi-544/545
	//
	//	Initial Ver.	: v0.46S for All
	//	Initial Date		: 2014-07-31
	//
	//	Customs 		 : Internal
	//	AP or B/E		 : 
	//	Demo Board		 : USB3.0 Tool
	//	
	//	Image size		 : 2604x1956
	//	mclk/pclk		 : 24mhz / 88Mhz
	//	MIPI speed(Mbps) : 880Mbps (each lane)
	//	MIPI						 : Non-continuous 
	//	Frame Length	 : 1984
	//	V-Blank 		 : 546us
	//	Line Length 	 : 2880
	//	H-Blank 		 : 386ns / 387ns	
	//	Max Fps 		 : 30fps (= Exp.time : 33ms )	
	//	Pixel order 	 : Blue 1st (=BGGR)
	//	X/Y-flip		 : No-X/Y flip
	//	I2C Address 	 : 0x40(Write), 0x41(Read)
	//	AG				 : x1
	//	DG				 : x1
	//	OTP Type		 : Single Write/read
	//	
	//
	//////////////////////////////////////////////////////////////////////////
	//<<<<<<<<<  Notice >>>>>>>>>/////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//
	//	Notice
	//	1) I2C Address & Data Type is 2Byte. 
	//	2) I2C Data construction that high byte is low addres, and low byte is high address. 
	//		 Initial register address must have used the even number. 
	//		 ex){Address, Data} = {Address(Even, 2byte), Data(2byte)} <==== Used Type
	//												= {Address(Even, 2byte(low address)), Data(1byte)} + {Address(Odd, 2byte(high address)), Data(1byte)} <== Not Used Type
	//												= {0x0000, 0x0F03} => {0x0000, 0x0F} + {0x0001, 0x03}
	//	3) The Continuous Mode of MIPI 2 Lane set is {0x0902, 0x4301}. And, 1lane set is {0x0902, 0x0301}.
	//		 The Non-continuous Mode of MIPI 2 Lane set is {0x0902, 0x4101}. And, 1lane set is {0x0902, 0x0101}.	
	//	4) Analog Gain address is 0x003a. 
	//			ex) 0x0000 = x1, 0x7000 = x8, 0xf000 = x16
	//
	/////////////////////////////////////////////////////////////////////////
	
	
	////////////////////////////////////////////////
	////////////// Hi-544/545 Initial //////////////
	////////////////////////////////////////////////
	
	
	HI545_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	//--- SRAM timing control---//
	HI545_write_cmos_sensor(0x0E00, 0x0101);
	HI545_write_cmos_sensor(0x0E02, 0x0101);
	HI545_write_cmos_sensor(0x0E04, 0x0101);
	HI545_write_cmos_sensor(0x0E06, 0x0101);
	HI545_write_cmos_sensor(0x0E08, 0x0101);
	HI545_write_cmos_sensor(0x0E0A, 0x0101);
	HI545_write_cmos_sensor(0x0E0C, 0x0101);
	HI545_write_cmos_sensor(0x0E0E, 0x0101);
	
	//Firmware 2Lane v0.38, All, AE+545 OTP Burst off FW 20140729
       hynix_i2c_burst_mode(Hynix_firmware_setting);
	HI545_write_cmos_sensor(0x2ffe, 0xf000);
	HI545_write_cmos_sensor(0x3000, 0x00AE);
	HI545_write_cmos_sensor(0x3002, 0x00AE);
	HI545_write_cmos_sensor(0x3004, 0x00AE);
	HI545_write_cmos_sensor(0x3006, 0x00AE);
	HI545_write_cmos_sensor(0x3008, 0x00AE);
	HI545_write_cmos_sensor(0x4000, 0x0400);
	HI545_write_cmos_sensor(0x4002, 0x0400);
	HI545_write_cmos_sensor(0x4004, 0x0C04);
	HI545_write_cmos_sensor(0x4006, 0x0C04);
	HI545_write_cmos_sensor(0x4008, 0x0C04);
	
	//--- FW End ---//
	
	
	//--- Initial Set file ---//
	HI545_write_cmos_sensor(0x0B02, 0x0014);
	HI545_write_cmos_sensor(0x0B04, 0x07CB);
	HI545_write_cmos_sensor(0x0B06, 0x5ED7);
	HI545_write_cmos_sensor(0x0B14, 0x370B); //PLL Main Div[15:8]. 0x1b = Pclk(86.4mhz), 0x37 = Pclk(176mhz)
	HI545_write_cmos_sensor(0x0B16, 0x4A0B);
	HI545_write_cmos_sensor(0x0B18, 0x0000);
	HI545_write_cmos_sensor(0x0B1A, 0x1044);
	
	HI545_write_cmos_sensor(0x004C, 0x0100); //tg_enable.
	HI545_write_cmos_sensor(0x0032, 0x0101); //Normal
	HI545_write_cmos_sensor(0x0036, 0x0048); //ramp_rst_offset
	HI545_write_cmos_sensor(0x0038, 0x4800); //ramp_sig_poffset
	HI545_write_cmos_sensor(0x0138, 0x0004); //pxl_drv_pwr
	HI545_write_cmos_sensor(0x013A, 0x0100); //tx_idle
	HI545_write_cmos_sensor(0x0C00, 0x3BC1); //BLC_ctl1. Line BLC on = 0x3b, off = 0x2A //LBLC_ctl1. [0]en_blc, [1]en_lblc_dpc, [2]en_channel_blc, [3]en_adj_pxl_dpc, [4]en_adp_dead_pxl_th
	HI545_write_cmos_sensor(0x0C0E, 0x0500); //0x07 BLC display On, 0x05 BLC D off //BLC_ctl3. Frame BLC On = 0x05, off=0x04 //FBLC_ctl3. [0]en_fblc, [1]en_blc_bypass, [2]en_fblc_dpc, [5]en_fobp_dig_offset, [7]en_lobp_dpc_bypass 
	HI545_write_cmos_sensor(0x0C10, 0x0510); //dig_blc_offset_h
	HI545_write_cmos_sensor(0x0C16, 0x0000); //fobp_dig_b_offset. red b[7] sign(0:+,1:-), b[6:0] dc offset 128(max)
	HI545_write_cmos_sensor(0x0C18, 0x0000); //fobp_dig_Gb_offset. Gr b[7] sign(0:+,1:-), b[6:0] dc offset 128(max)
	HI545_write_cmos_sensor(0x0C36, 0x0100); //r_g_sum_ctl. [0]:g_sum_en. '1'enable, '0'disable.
	
	//--- MIPI blank time --------------//
	HI545_write_cmos_sensor(0x0902, 0x4101); //mipi_value_clk_trail. MIPI CLK mode [1]'1'cont(0x43),'0'non-cont(0x41) [6]'1'2lane(0x41), '0'1lane(0x01) 
	HI545_write_cmos_sensor(0x090A, 0x03E4); //mipi_vblank_delay_h.
	HI545_write_cmos_sensor(0x090C, 0x0020); //mipi_hblank_short_delay_h.
	HI545_write_cmos_sensor(0x090E, 0x0020); //mipi_hblank_long_delay_h.
	HI545_write_cmos_sensor(0x0910, 0x5D07); //05 mipi_LPX
	HI545_write_cmos_sensor(0x0912, 0x061e); //05 mipi_CLK_prepare
	HI545_write_cmos_sensor(0x0914, 0x0407); //02 mipi_clk_pre
	HI545_write_cmos_sensor(0x0916, 0x0b0a); //09 mipi_data_zero
	HI545_write_cmos_sensor(0x0918, 0x0e09); //0c mipi_clk_post
	//----------------------------------//
	
	//--- Pixel Array Addressing ------//
	HI545_write_cmos_sensor(0x000E, 0x0000); //x_addr_start_lobp_h.
	HI545_write_cmos_sensor(0x0014, 0x003F); //x_addr_end_lobp_h.
	HI545_write_cmos_sensor(0x0010, 0x0050); //x_addr_start_robp_h.
	HI545_write_cmos_sensor(0x0016, 0x008F); //x_addr_end_robp_h.
	HI545_write_cmos_sensor(0x0012, 0x00A4); //x_addr_start_hact_h. 170
	HI545_write_cmos_sensor(0x0018, 0x0AD1); //x_addr_end_hact_h. 2769. 2769-170+1=2606
	HI545_write_cmos_sensor(0x0020, 0x0700); //x_regin_sel
	HI545_write_cmos_sensor(0x0022, 0x0004); //y_addr_start_fobp_h.
	HI545_write_cmos_sensor(0x0028, 0x000B); //y_addr_end_fobp_h.  
	HI545_write_cmos_sensor(0x0024, 0xFFFA); //y_addr_start_dummy_h.
	HI545_write_cmos_sensor(0x002A, 0xFFFF); //y_addr_end_dummy_h.	
	HI545_write_cmos_sensor(0x0026, 0x0012); //y_addr_start_vact_h. 18
	HI545_write_cmos_sensor(0x002C, 0x07B5); //y_addr_end_vact_h.  1973. 1973-18+1=1956
	HI545_write_cmos_sensor(0x0034, 0x0700); //Y_region_sel
	//------------------------------//
	
	//--Crop size 2604x1956 ----///
	HI545_write_cmos_sensor(0x0128, 0x0002); // digital_crop_x_offset_l
	HI545_write_cmos_sensor(0x012A, 0x0000); // digital_crop_y_offset_l
	HI545_write_cmos_sensor(0x012C, 0x0A2C); // digital_crop_image_width
	HI545_write_cmos_sensor(0x012E, 0x07A4); // digital_crop_image_height
	//------------------------------//
	
	//----< Image FMT Size >--------------------//
	//Image size 2604x1956
	HI545_write_cmos_sensor(0x0110, 0x0A2C); //X_output_size_h	   
	HI545_write_cmos_sensor(0x0112, 0x07A4); //Y_output_size_h	   
	//------------------------------------------//
	
	//----< Frame / Line Length >--------------//
	HI545_write_cmos_sensor(0x0006, 0x07c0); //frame_length_h 1984
	HI545_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI545_write_cmos_sensor(0x000A, 0x0DB0); //line_length for binning 3504
	//---------------------------------------//
	
	//--- ETC set ----//
	HI545_write_cmos_sensor(0x003C, 0x0000); //fixed frame off. b[0] '1'enable, '0'disable
	HI545_write_cmos_sensor(0x0000, 0x0000); //orientation. [0]:x-flip, [1]:y-flip.
	HI545_write_cmos_sensor(0x0500, 0x0000); //DGA_ctl.  b[1]'0'OTP_color_ratio_disable, '1' OTP_color_ratio_enable, b[2]'0'data_pedestal_en, '1'data_pedestal_dis.
	HI545_write_cmos_sensor(0x0700, 0x0590); //Scaler Normal 
	HI545_write_cmos_sensor(0x001E, 0x0101); 
	HI545_write_cmos_sensor(0x0032, 0x0101); 
	HI545_write_cmos_sensor(0x0A02, 0x0100); // Fast sleep Enable
	HI545_write_cmos_sensor(0x0116, 0x003B); // FBLC Ratio
	//----------------//
	
	//--- AG / DG control ----------------//
	//AG
	HI545_write_cmos_sensor(0x003A, 0x0000); //Analog Gain.  0x00=x1, 0x70=x8, 0xf0=x16.
	
	//DG
	HI545_write_cmos_sensor(0x0508, 0x0100); //DG_Gr_h.  0x01=x1, 0x07=x8.
	HI545_write_cmos_sensor(0x050a, 0x0100); //DG_Gb_h.  0x01=x1, 0x07=x8.
	HI545_write_cmos_sensor(0x050c, 0x0100); //DG_R_h.	0x01=x1, 0x07=x8.
	HI545_write_cmos_sensor(0x050e, 0x0100); //DG_B_h.	0x01=x1, 0x07=x8.
	//----------------------------------//
	
	
	//-----< Exp.Time >------------------------//
	// Pclk_88Mhz @ Line_length_pclk : 2880 @Exp.Time 33.33ms
	HI545_write_cmos_sensor(0x0002, 0x04b0);	//Fine_int : 33.33ms@Pclk88mhz@Line_length2880 
	HI545_write_cmos_sensor(0x0004, 0x07F4); //coarse_int : 33.33ms@Pclk88mhz@Line_length2880
	
	//--- ISP enable Selection ---------------//
	HI545_write_cmos_sensor(0x0A04, 0x011A); //isp_en. [9]s-gamma,[8]MIPI_en,[6]compresion10to8,[5]Scaler,[4]window,[3]DG,[2]LSC,[1]adpc,[0]tpg
	//----------------------------------------//
	
	HI545_write_cmos_sensor(0x0118, 0x0100); //sleep Off
	
	HI545DB("HI545_Sensor_Init exit :\n ");
}   /*  HI545_Sensor_Init  */

/*
void HI545_Sensor_OTP(void)
{
    HI545OTPSetting();
	HI545_Sensor_OTP_read();
	Hi545_Sensor_OTP_info();
	HI545_Sensor_calc_wbdata();
}
*/
static void HI545OTPSetting(void)
{	
	HI545DB("HI545OTPSetting begin:\n ");
	HI545_OTP_write_cmos_sensor(0x0118, 0x00); //sleep On
	mDELAY(10);
	HI545_OTP_write_cmos_sensor(0x0F02, 0x00); //pll disable
	HI545_OTP_write_cmos_sensor(0x011A, 0x01); //CP TRI_H
	HI545_OTP_write_cmos_sensor(0x011B, 0x09); //IPGM TRIM_H
	HI545_OTP_write_cmos_sensor(0x0D04, 0x01); //Fsync Output enable
	HI545_OTP_write_cmos_sensor(0x0D00, 0x07); //Fsync Output Drivability
	HI545_OTP_write_cmos_sensor(0x004C, 0x01); //TG MCU enable
	HI545_OTP_write_cmos_sensor(0x003E, 0x01); //OTP R/W
	HI545_OTP_write_cmos_sensor(0x0118, 0x01); //sleep off
	mDELAY(10);
    HI545DB("HI545OTPSetting exit :\n ");
}

static kal_uint16 HI545_Sensor_OTP_read(kal_uint16 otp_addr,kal_uint16 otp_addr_one,kal_uint16 otp_addr_two)
{
    kal_uint16 i, data;
	i = otp_addr; 
    HI545_OTP_write_cmos_sensor(0x10a, (i >> 8) & 0xFF); //start address H        
    HI545_OTP_write_cmos_sensor(0x10b, i & 0xFF); //start address L
    HI545_OTP_write_cmos_sensor(0x102, 0x00); //single read
    data = HI545_read_cmos_sensor(0x108); //OTP data read
	return data;
}

static void Hi545_Sensor_OTP_info(struct imgsensor_diff *diff)
{
	uint16_t ModuleHouseID = 0, 
                    AFflag = 0,
			 Year = 0, 
			 Month = 0, 
			 Day = 0, 
			 SensorID = 0, 
			 LensID = 0, 
			 VCMID = 0, 
			 DriverID = 0,
                    FnumberID = 0;
    uint16_t info_flag = 0, 
			 infocheck = 0, 
			 checksum = 0;
	info_flag = HI545_Sensor_OTP_read(0x1801,0,0);
    HI545DB("info_flag = 0x%x \n", info_flag);
    switch (info_flag)
	{	
		case 0x1:
		ModuleHouseID = HI545_Sensor_OTP_read(0x1802,0,0);
             AFflag = HI545_Sensor_OTP_read(0x1803,0,0);
		Year = HI545_Sensor_OTP_read(0x1804,0,0);
		Month = HI545_Sensor_OTP_read(0x1805,0,0);
		Day = HI545_Sensor_OTP_read(0x1806,0,0);
		SensorID = HI545_Sensor_OTP_read(0x1807,0,0);
		LensID = HI545_Sensor_OTP_read(0x1808,0,0);
		VCMID = HI545_Sensor_OTP_read(0x1809,0,0);
		DriverID = HI545_Sensor_OTP_read(0x180a,0,0);
             FnumberID = HI545_Sensor_OTP_read(0x180b,0,0);
		infocheck = HI545_Sensor_OTP_read(0x1812,0,0);
    	break;
		case 0x13:
		ModuleHouseID = HI545_Sensor_OTP_read(0x1813,0,0);
             AFflag = HI545_Sensor_OTP_read(0x1814,0,0);
		Year = HI545_Sensor_OTP_read(0x1815,0,0);
		Month = HI545_Sensor_OTP_read(0x1816,0,0);
		Day = HI545_Sensor_OTP_read(0x1817,0,0);
		SensorID = HI545_Sensor_OTP_read(0x1818,0,0);
		LensID = HI545_Sensor_OTP_read(0x1819,0,0);
		VCMID = HI545_Sensor_OTP_read(0x181a,0,0);
		DriverID = HI545_Sensor_OTP_read(0x181b,0,0);
             FnumberID = HI545_Sensor_OTP_read(0x180c,0,0);
		infocheck = HI545_Sensor_OTP_read(0x1823,0,0);
		break;
		case 0x37:
		ModuleHouseID = HI545_Sensor_OTP_read(0x1824,0,0);
             AFflag = HI545_Sensor_OTP_read(0x1825,0,0);
		Year = HI545_Sensor_OTP_read(0x1826,0,0);
		Month = HI545_Sensor_OTP_read(0x1827,0,0);
		Day = HI545_Sensor_OTP_read(0x1828,0,0);
		SensorID = HI545_Sensor_OTP_read(0x1829,0,0);
		LensID = HI545_Sensor_OTP_read(0x182a,0,0);
		VCMID = HI545_Sensor_OTP_read(0x182b,0,0);
		DriverID = HI545_Sensor_OTP_read(0x182c,0,0);
             FnumberID = HI545_Sensor_OTP_read(0x180d,0,0);
		infocheck = HI545_Sensor_OTP_read(0x1834,0,0);
		break;
		default:		
		HI545DB("HI545_Sensor: info_flag error value: 0x%x \n ",info_flag);

		break;
	}
	checksum = (ModuleHouseID + AFflag + Year + Month + Day + SensorID + LensID + VCMID + DriverID + FnumberID) % 0xFF + 1;
	if (checksum == infocheck)
		{
		HI545DB("HI545_Sensor: Module information checksum PASS\n ");

		}
	else
		{
		HI545DB("HI545_Sensor: Module information checksum Fail\n ");

		}

    diff->ModuleHouseID = ModuleHouseID;
	diff->VCAM          = VCMID;
    HI545DB("ModuleHouseID = 0x%x \n", ModuleHouseID);
    HI545DB("AFflag = 0x%x \n", AFflag);
    HI545DB("Year = 0x%x, Month = 0x%x, Day = 0x%x\n", Year, Month, Day);
    HI545DB("SensorID = 0x%x \n", SensorID);
    HI545DB("LensID = 0x%x \n", LensID);
    HI545DB("VCMID = 0x%x \n", VCMID);
    HI545DB("DriverID = 0x%x \n", DriverID);
    HI545DB("FnumberID = 0x%x \n", FnumberID);


	//HI545_Sensor_calc_wbdata();
}

static kal_uint16 HI545_Sensor_calc_wbdata()
{
	uint16_t wbcheck = 0, 
			 checksum = 0, 
			 wb_flag = 0;
    uint16_t r_gain = 0, 
			 b_gain = 0, 
			 g_gain = 0;
    uint16_t wb_unit_rg_h = 0, 
			 wb_unit_rg_l = 0, 
			 wb_unit_bg_h = 0, 
			 wb_unit_bg_l = 0,
			 wb_unit_gg_h = 0,
			 wb_unit_gg_l = 0,
    		 wb_golden_rg_h = 0, 
			 wb_golden_rg_l = 0, 
			 wb_golden_bg_h = 0, 
			 wb_golden_bg_l = 0,
			 wb_golden_gg_h = 0,
			 wb_golden_gg_l = 0;
    uint16_t wb_unit_r_h = 0, 
			 wb_unit_r_l = 0, 
			 wb_unit_b_h = 0, 
			 wb_unit_b_l = 0,
			 wb_unit_gr_h = 0,
			 wb_unit_gr_l = 0,
			 wb_unit_gb_h = 0,
			 wb_unit_gb_l = 0,
    		 wb_golden_r_h = 0, 
			 wb_golden_r_l = 0, 
			 wb_golden_b_h = 0, 
			 wb_golden_b_l = 0,
			 wb_golden_gr_h = 0,
			 wb_golden_gr_l = 0,
			 wb_golden_gb_h = 0,
			 wb_golden_gb_l = 0;
    uint16_t rg_golden_value = 0, 
			 bg_golden_value = 0, 
			 rg_value = 0, 
			 bg_value = 0;
			 
	wb_flag = HI545_Sensor_OTP_read(0x1835,0,0);
    HI545DB("wb_flag = 0x%x \n", wb_flag);
    switch (wb_flag)
	{
		case 0x1:
		wb_unit_rg_h = HI545_Sensor_OTP_read(0x1836,0,0);
		wb_unit_rg_l = HI545_Sensor_OTP_read(0x1837,0,0);
		wb_unit_bg_h = HI545_Sensor_OTP_read(0x1838,0,0);
		wb_unit_bg_l = HI545_Sensor_OTP_read(0x1839,0,0);
		wb_unit_gg_h = HI545_Sensor_OTP_read(0x183a,0,0);
		wb_unit_gg_l = HI545_Sensor_OTP_read(0x183b,0,0);
		wb_golden_rg_h = HI545_Sensor_OTP_read(0x183c,0,0);
		wb_golden_rg_l = HI545_Sensor_OTP_read(0x183d,0,0);
		wb_golden_bg_h = HI545_Sensor_OTP_read(0x183e,0,0);
		wb_golden_bg_l = HI545_Sensor_OTP_read(0x183f,0,0);
		wb_golden_gg_h = HI545_Sensor_OTP_read(0x1840,0,0);
		wb_golden_gg_l = HI545_Sensor_OTP_read(0x1841,0,0);
		wb_unit_r_h = HI545_Sensor_OTP_read(0x1842,0,0);
		wb_unit_r_l = HI545_Sensor_OTP_read(0x1843,0,0);
		wb_unit_b_h = HI545_Sensor_OTP_read(0x1844,0,0);
		wb_unit_b_l = HI545_Sensor_OTP_read(0x1845,0,0);
		wb_unit_gr_h = HI545_Sensor_OTP_read(0x1846,0,0);
		wb_unit_gr_l = HI545_Sensor_OTP_read(0x1847,0,0);
		wb_unit_gb_h = HI545_Sensor_OTP_read(0x1848,0,0);
		wb_unit_gb_l = HI545_Sensor_OTP_read(0x1849,0,0);
		wb_golden_r_h = HI545_Sensor_OTP_read(0x184a,0,0);
		wb_golden_r_l = HI545_Sensor_OTP_read(0x184b,0,0);
		wb_golden_b_h = HI545_Sensor_OTP_read(0x184c,0,0);
		wb_golden_b_l = HI545_Sensor_OTP_read(0x184d,0,0);
		wb_golden_gr_h = HI545_Sensor_OTP_read(0x184e,0,0);
		wb_golden_gr_l = HI545_Sensor_OTP_read(0x184f,0,0);
		wb_golden_gb_h = HI545_Sensor_OTP_read(0x1850,0,0);
		wb_golden_gb_l = HI545_Sensor_OTP_read(0x1851,0,0);
		wbcheck = HI545_Sensor_OTP_read(0x1853,0,0);
    	break;
		case 0x13:
		wb_unit_rg_h = HI545_Sensor_OTP_read(0x1854,0,0);
		wb_unit_rg_l = HI545_Sensor_OTP_read(0x1855,0,0);
		wb_unit_bg_h = HI545_Sensor_OTP_read(0x1856,0,0);
		wb_unit_bg_l = HI545_Sensor_OTP_read(0x1857,0,0);
		wb_unit_gg_h = HI545_Sensor_OTP_read(0x1858,0,0);
		wb_unit_gg_l = HI545_Sensor_OTP_read(0x1859,0,0);
		wb_golden_rg_h = HI545_Sensor_OTP_read(0x185a,0,0);
		wb_golden_rg_l = HI545_Sensor_OTP_read(0x185b,0,0);
		wb_golden_bg_h = HI545_Sensor_OTP_read(0x185c,0,0);
		wb_golden_bg_l = HI545_Sensor_OTP_read(0x185d,0,0);
		wb_golden_gg_h = HI545_Sensor_OTP_read(0x185e,0,0);
		wb_golden_gg_l = HI545_Sensor_OTP_read(0x186f,0,0);
		wb_unit_r_h = HI545_Sensor_OTP_read(0x1860,0,0);
		wb_unit_r_l = HI545_Sensor_OTP_read(0x1861,0,0);
		wb_unit_b_h = HI545_Sensor_OTP_read(0x1862,0,0);
		wb_unit_b_l = HI545_Sensor_OTP_read(0x1863,0,0);
		wb_unit_gr_h = HI545_Sensor_OTP_read(0x1864,0,0);
		wb_unit_gr_l = HI545_Sensor_OTP_read(0x1865,0,0);
		wb_unit_gb_h = HI545_Sensor_OTP_read(0x1866,0,0);
		wb_unit_gb_l = HI545_Sensor_OTP_read(0x1867,0,0);
		wb_golden_r_h = HI545_Sensor_OTP_read(0x1868,0,0);
		wb_golden_r_l = HI545_Sensor_OTP_read(0x1869,0,0);
		wb_golden_b_h = HI545_Sensor_OTP_read(0x186a,0,0);
		wb_golden_b_l = HI545_Sensor_OTP_read(0x186b,0,0);
		wb_golden_gr_h = HI545_Sensor_OTP_read(0x186c,0,0);
		wb_golden_gr_l = HI545_Sensor_OTP_read(0x186d,0,0);
		wb_golden_gb_h = HI545_Sensor_OTP_read(0x186e,0,0);
		wb_golden_gb_l = HI545_Sensor_OTP_read(0x186f,0,0);
		wbcheck = HI545_Sensor_OTP_read(0x1871,0,0);
		break;
		case 0x37:
		wb_unit_rg_h = HI545_Sensor_OTP_read(0x1872,0,0);
		wb_unit_rg_l = HI545_Sensor_OTP_read(0x1873,0,0);
		wb_unit_bg_h = HI545_Sensor_OTP_read(0x1874,0,0);
		wb_unit_bg_l = HI545_Sensor_OTP_read(0x1875,0,0);
		wb_unit_gg_h = HI545_Sensor_OTP_read(0x1876,0,0);
		wb_unit_gg_l = HI545_Sensor_OTP_read(0x1877,0,0);
		wb_golden_rg_h = HI545_Sensor_OTP_read(0x1878,0,0);
		wb_golden_rg_l = HI545_Sensor_OTP_read(0x1879,0,0);
		wb_golden_bg_h = HI545_Sensor_OTP_read(0x187a,0,0);
		wb_golden_bg_l = HI545_Sensor_OTP_read(0x187b,0,0);
		wb_golden_gg_h = HI545_Sensor_OTP_read(0x187c,0,0);
		wb_golden_gg_l = HI545_Sensor_OTP_read(0x187d,0,0);
		wb_unit_r_h = HI545_Sensor_OTP_read(0x187e,0,0);
		wb_unit_r_l = HI545_Sensor_OTP_read(0x187f,0,0);
		wb_unit_b_h = HI545_Sensor_OTP_read(0x1880,0,0);
		wb_unit_b_l = HI545_Sensor_OTP_read(0x1881,0,0);
		wb_unit_gr_h = HI545_Sensor_OTP_read(0x1882,0,0);
		wb_unit_gr_l = HI545_Sensor_OTP_read(0x1883,0,0);
		wb_unit_gb_h = HI545_Sensor_OTP_read(0x1884,0,0);
		wb_unit_gb_l = HI545_Sensor_OTP_read(0x1885,0,0);
		wb_golden_r_h = HI545_Sensor_OTP_read(0x1886,0,0);
		wb_golden_r_l = HI545_Sensor_OTP_read(0x1887,0,0);
		wb_golden_b_h = HI545_Sensor_OTP_read(0x1888,0,0);
		wb_golden_b_l = HI545_Sensor_OTP_read(0x1889,0,0);
		wb_golden_gr_h = HI545_Sensor_OTP_read(0x188a,0,0);
		wb_golden_gr_l = HI545_Sensor_OTP_read(0x188b,0,0);
		wb_golden_gb_h = HI545_Sensor_OTP_read(0x188c,0,0);
		wb_golden_gb_l = HI545_Sensor_OTP_read(0x188d,0,0);
		wbcheck = HI545_Sensor_OTP_read(0x188f,0,0);
		break;
		default:
		HI545DB("HI545_Sensor: wb_flag error value: 0x%x\n ",wb_flag);

		break;	
	}
	
		checksum = (wb_unit_rg_h + wb_unit_rg_l + wb_unit_bg_h + wb_unit_bg_l + wb_unit_gg_h + wb_unit_gg_l 
			+ wb_golden_rg_h + wb_golden_rg_l + wb_golden_bg_h + wb_golden_bg_l + wb_golden_gg_h + wb_golden_gg_l
			+ wb_unit_r_h + wb_unit_r_l + wb_unit_b_h + wb_unit_b_l + wb_unit_gr_h + wb_unit_gr_l + wb_unit_gb_h + wb_unit_gb_l
			+ wb_golden_r_h + wb_golden_r_l + wb_golden_b_h + wb_golden_b_l + wb_golden_gr_h + wb_golden_gr_l + wb_golden_gb_h + wb_golden_gb_l) % 0xFF + 1;
    HI545DB("wb_unit_rg_h = 0x%x, wb_unit_rg_l = 0x%x, wb_unit_bg_h = 0x%x, wb_unit_bg_l = 0x%x, wb_golden_rg_h = 0x%x, wb_golden_rg_l = 0x%x, wb_golden_bg_h = 0x%x, wb_golden_bg_l = 0x%x, wbcheck = 0x%x\n", 
		wb_unit_rg_h, wb_unit_rg_l, wb_unit_bg_h, wb_unit_bg_l, wb_golden_rg_h, wb_golden_rg_l, wb_golden_bg_h, wb_golden_bg_l, wbcheck);

	if (checksum == wbcheck)
		{
		HI545DB("HI545_Sensor: WB checksum PASS\n ");

		}
	else
		{
		HI545DB("HI545_Sensor: WB checksum Fail\n ");

		}
		
	rg_golden_value = (((wb_golden_rg_h << 4) | ((wb_golden_rg_l>>4) & 0x000f)) & 0x0fff);
	bg_golden_value = (((wb_golden_bg_h << 4) | ((wb_golden_bg_l>>4) & 0x000f)) & 0x0fff);
	rg_value = (((wb_unit_rg_h << 4) | ((wb_unit_rg_l>>4) & 0x000f)) & 0x0fff);
	bg_value = (((wb_unit_bg_h << 4) | ((wb_unit_bg_l>>4) & 0x000f)) & 0x0fff);

	HI545DB("rg_golden_value = 0x%x, bg_golden_value = 0x%x\n", rg_golden_value, bg_golden_value);
    HI545DB("rg_value = 0x%x, bg_value = 0x%x\n", rg_value, bg_value);

    g_gain = 0x100;
    r_gain = (0x100 * rg_golden_value) / rg_value;
    b_gain = (0x100 * bg_golden_value) / bg_value;

    HI545_Sensor_update_wb_gain( r_gain, b_gain, g_gain);

}

static void HI545_Sensor_update_wb_gain(kal_uint32 r_gain, kal_uint32 b_gain, kal_uint32 g_gain)
{
    kal_uint16 r, b;
    HI545_OTP_write_cmos_sensor(0x0A00, 0x00); //sleep On
    HI545_OTP_write_cmos_sensor(0x003e, 0x00); //OTP mode off
    HI545_OTP_write_cmos_sensor(0x0A00, 0x01); //sleep Off

    HI545DB("r_gain = 0x%x, g_gain = 0x%x, b_gain = 0x%x\n", r_gain, g_gain, b_gain);
    HI545_write_cmos_sensor(0x050c,r_gain & 0xFFFF); //r_gain
    HI545_write_cmos_sensor(0x050e,b_gain & 0xFFFF); //b_gain

    r = (HI545_read_cmos_sensor(0x050c) << 8) | HI545_read_cmos_sensor(0x050d);
    b = (HI545_read_cmos_sensor(0x050e) << 8) | HI545_read_cmos_sensor(0x050f);
	HI545DB("Terry Check: r_gain = 0x%x, b_gain = 0x%x\n", r, b);

}

/*************************************************************************
* FUNCTION
*   HI545Open
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static UINT32 HI545Open(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	HI545DB("HI545Open enter :\n ");

	for(i=0;i<3;i++)
	{
		sensor_id = (HI545_read_cmos_sensor(0x0F17)<<8)|HI545_read_cmos_sensor(0x0F16)+1;
		HI545DB("OHI545 READ ID : 0x%x",sensor_id);
		if(sensor_id != HI545MIPI_SENSOR_ID_BILU)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
		{
			break;
		}
	}
	
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorMode = SENSOR_MODE_INIT;
	HI545.HI545AutoFlickerMode = KAL_FALSE;
	HI545.HI545VideoMode = KAL_FALSE;
	spin_unlock(&HI545mipiraw_drv_lock);
	HI545_Sensor_Init();


	spin_lock(&HI545mipiraw_drv_lock);
	HI545.DummyLines= 0;
	HI545.DummyPixels= 0;
	HI545.pvPclk =  ( HI545_PV_CLK / 10000); 
	HI545.videoPclk = ( HI545_VIDEO_CLK / 10000);
	HI545.capPclk = (HI545_CAP_CLK / 10000);

	HI545.shutter = 0x4EA;
	HI545.pvShutter = 0x4EA;
	HI545.maxExposureLines =HI545_PV_PERIOD_LINE_NUMS -4;

	HI545.ispBaseGain = BASEGAIN;//0x40
	HI545.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	HI545.pvGain = 0x1f;
	HI545.realGain = 0x1f;//ispBaseGain as 1x
	spin_unlock(&HI545mipiraw_drv_lock);

#ifdef HI545_OTP_FUNCTION
	HI545OTPSetting();
	HI545_Sensor_calc_wbdata();
#endif

    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HI545GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 HI545GetSensorID(UINT32 *sensorID)
{
    int  retry = 3;

	HI545DB("HI545GetSensorID enter :\n ");

    // check if sensor ID correct
    do {
        *sensorID = ((HI545_read_cmos_sensor(0x0F17)<<8)|HI545_read_cmos_sensor(0x0F16))+1;
        if (*sensorID == HI545MIPI_SENSOR_ID_BILU)
        	{
        		HI545DB("Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        HI545DB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != HI545MIPI_SENSOR_ID_BILU) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

#ifdef HI545_OTP_FUNCTION
    struct imgsensor_diff sensor_diff;
    HI545_Sensor_Init();
    HI545OTPSetting();
    Hi545_Sensor_OTP_info(&sensor_diff);
    if (sensor_diff.ModuleHouseID != 0x7) {
        HI545DB("ModuleHouseID is not 0x7, ModuleHouseID = 0x%04x\n", sensor_diff.ModuleHouseID);
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	if(sensor_diff.VCAM != 0x02){
		HI545DB("quanzhen 2\n");
		*sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
	}
#endif
    HI545DB("quanzhen OK!!\n");
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   HI545_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HI545 to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void HI545_SetShutter(kal_uint32 iShutter)
{

//   if(HI545.shutter == iShutter)
//   		return;

   spin_lock(&HI545mipiraw_drv_lock);
   HI545.shutter= iShutter;
   spin_unlock(&HI545mipiraw_drv_lock);

   HI545_write_shutter(iShutter);
   return;
 
}   /*  HI545_SetShutter   */



/*************************************************************************
* FUNCTION
*   HI545_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 HI545_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2;
	UINT32 shutter =0;
	temp_reg1 = HI545_read_cmos_sensor(0x0004);  
	//temp_reg2 = HI545_read_cmos_sensor(0x0005);    
	//read out register value and divide 16;
	shutter  = temp_reg1;

	return shutter;
}

/*************************************************************************
* FUNCTION
*   HI545_night_mode
*
* DESCRIPTION
*   This function night mode of HI545.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void HI545_NightMode(kal_bool bEnable)
{
}/*	HI545_NightMode */



/*************************************************************************
* FUNCTION
*   HI545Close
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 HI545Close(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(HI545hDrvI2C);
    //e_porting
    ReEnteyCamera = KAL_FALSE;
    return ERROR_NONE;
}	/* HI545Close() */

static void HI545SetFlipMirror(kal_int32 imgMirror)
{
#if 1
	
    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NOMAL:
            HI545_write_cmos_sensor(0x0000, 0x0000);//Set normal
            break;    
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            HI545_write_cmos_sensor(0x0000, 0x0200);	//Set flip
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            HI545_write_cmos_sensor(0x0000, 0x0100);//Set mirror
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            HI545_write_cmos_sensor(0x0000, 0x0300);	//Set mirror & flip
            break;            
    }
#endif	
}


/*************************************************************************
* FUNCTION
*   HI545Preview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 HI545Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	HI545DB("HI545Preview enter:");

	// preview size
	if(HI545.sensorMode == SENSOR_MODE_PREVIEW)
	{
		// do nothing
		// FOR CCT PREVIEW
	}
	else
	{
		//HI545DB("HI545Preview setting!!\n");
		HI545PreviewSetting();
	}
	
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	HI545.DummyPixels = 0;//define dummy pixels and lines
	HI545.DummyLines = 0 ;
	HI545_FeatureControl_PERIOD_PixelNum=HI545_PV_PERIOD_PIXEL_NUMS+ HI545.DummyPixels;
	HI545_FeatureControl_PERIOD_LineNum=HI545_PV_PERIOD_LINE_NUMS+HI545.DummyLines;
	spin_unlock(&HI545mipiraw_drv_lock);

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.imgMirror = sensor_config_data->SensorImageMirror;
	//HI545.imgMirror =IMAGE_NORMAL; //by module layout
	spin_unlock(&HI545mipiraw_drv_lock);
	
	//HI545SetFlipMirror(sensor_config_data->SensorImageMirror);
	HI545SetFlipMirror(IMAGE_H_MIRROR);
	

	HI545DBSOFIA("[HI545Preview]frame_len=%x\n", ((HI545_read_cmos_sensor(0x380e)<<8)+HI545_read_cmos_sensor(0x380f)));

 //   mDELAY(40);
	HI545DB("HI545Preview exit:\n");
    return ERROR_NONE;
}	/* HI545Preview() */



static UINT32 HI545Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	HI545DB("HI545Video enter:");

	if(HI545.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
	{
		HI545VideoSetting();

	}
	spin_lock(&HI545mipiraw_drv_lock);
	HI545.sensorMode = SENSOR_MODE_VIDEO;
	HI545_FeatureControl_PERIOD_PixelNum=HI545_VIDEO_PERIOD_PIXEL_NUMS+ HI545.DummyPixels;
	HI545_FeatureControl_PERIOD_LineNum=HI545_VIDEO_PERIOD_LINE_NUMS+HI545.DummyLines;
	spin_unlock(&HI545mipiraw_drv_lock);


	spin_lock(&HI545mipiraw_drv_lock);
	HI545.imgMirror = sensor_config_data->SensorImageMirror;
	//HI545.imgMirror =IMAGE_NORMAL; //by module layout
	spin_unlock(&HI545mipiraw_drv_lock);
	//HI545SetFlipMirror(sensor_config_data->SensorImageMirror);
	HI545SetFlipMirror(IMAGE_H_MIRROR);

//    mDELAY(40);
	HI545DB("HI545Video exit:\n");
    return ERROR_NONE;
}


static UINT32 HI545Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = HI545.shutter;
	kal_uint32 temp_data;
	//kal_uint32 pv_line_length , cap_line_length,

	if( SENSOR_MODE_CAPTURE== HI545.sensorMode)
	{
		HI545DB("HI545Capture BusrtShot!!!\n");
	}
	else
	{
		HI545DB("HI545Capture enter:\n");
#if 0
		//Record Preview shutter & gain
		shutter=HI545_read_shutter();
		temp_data =  read_HI545_gain();
		spin_lock(&HI545mipiraw_drv_lock);
		HI545.pvShutter =shutter;
		HI545.sensorGlobalGain = temp_data;
		HI545.pvGain =HI545.sensorGlobalGain;
		spin_unlock(&HI545mipiraw_drv_lock);

		HI545DB("[HI545Capture]HI545.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",HI545.shutter, shutter,HI545.sensorGlobalGain);
#endif
		// Full size setting
		HI545CaptureSetting();
	//    mDELAY(20);

		spin_lock(&HI545mipiraw_drv_lock);
		HI545.sensorMode = SENSOR_MODE_CAPTURE;
		HI545.imgMirror = sensor_config_data->SensorImageMirror;
		//HI545.imgMirror =IMAGE_NORMAL; //by module layout
		HI545.DummyPixels = 0;//define dummy pixels and lines
		HI545.DummyLines = 0 ;
		HI545_FeatureControl_PERIOD_PixelNum = HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels;
		HI545_FeatureControl_PERIOD_LineNum = HI545_FULL_PERIOD_LINE_NUMS + HI545.DummyLines;
		spin_unlock(&HI545mipiraw_drv_lock);

		//HI545SetFlipMirror(sensor_config_data->SensorImageMirror);

		HI545SetFlipMirror(IMAGE_H_MIRROR);

		HI545DB("HI545Capture exit:\n");
	}

    return ERROR_NONE;
}	/* HI545Capture() */

static UINT32 HI545GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    HI545DB("HI545GetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorPreviewHeight	= HI545_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorFullWidth		= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= HI545_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth		= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorVideoHeight    = HI545_IMAGE_SENSOR_FULL_HEIGHT;
//    HI545DB("SensorPreviewWidth:  %d.\n", pSensorResolution->SensorPreviewWidth);
//    HI545DB("SensorPreviewHeight: %d.\n", pSensorResolution->SensorPreviewHeight);
//    HI545DB("SensorFullWidth:  %d.\n", pSensorResolution->SensorFullWidth);
//    HI545DB("SensorFullHeight: %d.\n", pSensorResolution->SensorFullHeight);
    return ERROR_NONE;
}   /* HI545GetResolution() */

static UINT32 HI545GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorPreviewResolutionX= HI545_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= HI545_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorFullResolutionX= HI545_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= HI545_IMAGE_SENSOR_FULL_HEIGHT;

	spin_lock(&HI545mipiraw_drv_lock);
	HI545.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&HI545mipiraw_drv_lock);

   	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gb;
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 1;  // from 2 to 3 for test shutter linearity
    pSensorInfo->PreviewDelayFrame = 1;  // from 442 to 666
    pSensorInfo->VideoDelayFrame = 1;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI545_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = HI545_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_FULL_X_START;	//2*HI545_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HI545_FULL_Y_START;	//2*HI545_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI545_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI545_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &HI545SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* HI545GetInfo() */


static UINT32 HI545Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&HI545mipiraw_drv_lock);
		HI545CurrentScenarioId = ScenarioId;
		spin_unlock(&HI545mipiraw_drv_lock);
		//HI545DB("ScenarioId=%d\n",ScenarioId);
		HI545DB("HI545CurrentScenarioId=%d\n",HI545CurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            HI545Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			HI545Video(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HI545Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* HI545Control() */


static UINT32 HI545SetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    HI545DB("[HI545SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&HI545mipiraw_drv_lock);
	VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&HI545mipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		HI545DB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    HI545DB("error frame rate seting\n");

    if(HI545.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	if(HI545.HI545AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 304;
			else if(u2FrameRate==15)
				frameRate= 148;//148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (HI545.videoPclk*10000)/(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/frameRate*10;
    	}
		else
		{
    		if (u2FrameRate==30)
				MIN_Frame_length= HI545_VIDEO_PERIOD_LINE_NUMS;
			else	
				MIN_Frame_length = (HI545.videoPclk*10000) /(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/u2FrameRate;
		}

		if((MIN_Frame_length <=HI545_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = HI545_VIDEO_PERIOD_LINE_NUMS;
			HI545DB("[HI545SetVideoMode]current fps = %d\n", (HI545.videoPclk*10000)  /(HI545_VIDEO_PERIOD_PIXEL_NUMS)/HI545_VIDEO_PERIOD_LINE_NUMS);
		}
		HI545DB("[HI545SetVideoMode]current fps (10 base)= %d\n", (HI545.videoPclk*10000)*10/(HI545_VIDEO_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/MIN_Frame_length);

		if(HI545.shutter + 4 > MIN_Frame_length)
				MIN_Frame_length = HI545.shutter + 4;

		extralines = MIN_Frame_length - HI545_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&HI545mipiraw_drv_lock);
		HI545.DummyPixels = 0;//define dummy pixels and lines
		HI545.DummyLines = extralines ;
		spin_unlock(&HI545mipiraw_drv_lock);
		
		HI545_SetDummy(HI545.DummyPixels,extralines);
    }
	else if(HI545.sensorMode == SENSOR_MODE_CAPTURE)
	{
		HI545DB("-------[HI545SetVideoMode]ZSD???---------\n");
		if(HI545.HI545AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==15)
			    frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (HI545.capPclk*10000) /(HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (HI545.capPclk*10000) /(HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=HI545_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = HI545_FULL_PERIOD_LINE_NUMS;
			HI545DB("[HI545SetVideoMode]current fps = %d\n", (HI545.capPclk*10000) /(HI545_FULL_PERIOD_PIXEL_NUMS)/HI545_FULL_PERIOD_LINE_NUMS);

		}
		HI545DB("[HI545SetVideoMode]current fps (10 base)= %d\n", (HI545.capPclk*10000)*10/(HI545_FULL_PERIOD_PIXEL_NUMS + HI545.DummyPixels)/MIN_Frame_length);

		if(HI545.shutter + 4 > MIN_Frame_length)
				MIN_Frame_length = HI545.shutter + 4;


		extralines = MIN_Frame_length - HI545_FULL_PERIOD_LINE_NUMS;

		spin_lock(&HI545mipiraw_drv_lock);
		HI545.DummyPixels = 0;//define dummy pixels and lines
		HI545.DummyLines = extralines ;
		spin_unlock(&HI545mipiraw_drv_lock);

		HI545_SetDummy(HI545.DummyPixels,extralines);
	}
	HI545DB("[HI545SetVideoMode]MIN_Frame_length=%d,HI545.DummyLines=%d\n",MIN_Frame_length,HI545.DummyLines);

    return KAL_TRUE;
}

static UINT32 HI545SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;

    //HI545DB("[HI545SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
	if(bEnable) {   // enable auto flicker
		spin_lock(&HI545mipiraw_drv_lock);
		HI545.HI545AutoFlickerMode = KAL_TRUE;
		spin_unlock(&HI545mipiraw_drv_lock);
    } else {
    	spin_lock(&HI545mipiraw_drv_lock);
        HI545.HI545AutoFlickerMode = KAL_FALSE;
		spin_unlock(&HI545mipiraw_drv_lock);
        HI545DB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

static UINT32 HI545SetTestPatternMode(kal_bool bEnable)
{
    HI545DB("[HI545SetTestPatternMode] Test pattern enable:%d\n", bEnable);
#if 1
    if(bEnable) 
    {
        HI545_write_cmos_sensor(0x020a,0x0200);        
    }
    else
    {
        HI545_write_cmos_sensor(0x020a,0x0000);  

    }
#endif
    return ERROR_NONE;
}

static UINT32 HI545MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	HI545DB("HI545MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =  HI545_PV_CLK;
			lineLength = HI545_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI545_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI545mipiraw_drv_lock);
			HI545.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&HI545mipiraw_drv_lock);
			HI545_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk =  HI545_VIDEO_CLK;
			lineLength = HI545_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI545_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI545mipiraw_drv_lock);
			HI545.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&HI545mipiraw_drv_lock);
			HI545_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = HI545_CAP_CLK;
			lineLength = HI545_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI545_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&HI545mipiraw_drv_lock);
			HI545.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&HI545mipiraw_drv_lock);
			HI545_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}


static UINT32 HI545MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 240;  // modify by yfx for zsd cc
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}



static UINT32 HI545FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= HI545_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= HI545_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= HI545_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= HI545_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(HI545CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 =  HI545_PV_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 =  HI545_VIDEO_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = HI545_CAP_CLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 =  HI545_CAP_CLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            HI545_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HI545_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            HI545_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //HI545_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            HI545_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HI545_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HI545mipiraw_drv_lock);
                HI545SensorCCT[i].Addr=*pFeatureData32++;
                HI545SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&HI545mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI545SensorCCT[i].Addr;
                *pFeatureData32++=HI545SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&HI545mipiraw_drv_lock);
                HI545SensorReg[i].Addr=*pFeatureData32++;
                HI545SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&HI545mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI545SensorReg[i].Addr;
                *pFeatureData32++=HI545SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HI545MIPI_SENSOR_ID_BILU;
                memcpy(pSensorDefaultData->SensorEngReg, HI545SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HI545SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HI545SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            HI545_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            HI545_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=HI545_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            HI545_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            HI545_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            HI545_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            HI545SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            HI545GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            HI545SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            HI545SetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HI545MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HI545MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
            break;       
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing             
            *pFeatureReturnPara32=HI545_TEST_PATTERN_CHECKSUM;           
            *pFeatureParaLen=4;                             
        break;     
        default:
            break;
    }
    return ERROR_NONE;
}	/* HI545FeatureControl() */


static SENSOR_FUNCTION_STRUCT	SensorFuncHI545=
{
    HI545Open,
    HI545GetInfo,
    HI545GetResolution,
    HI545FeatureControl,
    HI545Control,
    HI545Close
};

UINT32 HI545_MIPI_RAW_BILU_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI545;

    return ERROR_NONE;
}   /* SensorInit() */

