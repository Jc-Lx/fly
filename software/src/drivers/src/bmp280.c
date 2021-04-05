#include "bmp280.h"

static bmp280_t p_bmp280;//校准参数
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_Chack(void)
*	功能说明: 检查BMP280连接正常
*	形    参：
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_Chack(void)
{
	u16 time = 0;
	u8 chip_ID = 0;
	while(time<1000)
	{
		chip_ID = BMP280_Read_Byte(BMP280_CHIPID_REG);
		if(chip_ID==0x57||chip_ID==0x58||chip_ID==0x59)break;//检测到芯片
		else time++;
		delay_ms(1);
	}
	if(time==1000)return 1;//未检测到芯片
	else 
	{
		p_bmp280.chip_id = chip_ID;//记录芯片ID
		return 0;
	}
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_SetSoftReset(void)
*	功能说明: BMP280软件复位，原因不明，函数使用之后芯片无法使用
*	形    参：
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_SetSoftReset(void)
{
	if(BMP280_Write_Byte(BMP280_RESET_REG,BMP280_RESET_VALUE))return 1;
  else return 0;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_Init(void)
*	功能说明: BMP280初始化
*	形    参：
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_Init(void)
{
	//BMP280_GPIO_Init();
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//CSB(PB4)初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_4);

	if(BMP280_Chack())return 1;//BMP280检测芯片
	else
	{
		//if(BMP280_SetSoftReset())return 2;//软件复位使用不了，使用之后用不了
    	if(BMP280_CalibParam())return 3;//
		if(BMP280_SetPowerMode(BMP280_NORMAL_MODE))return 4;
		if(BMP280_SetWorkMode(BMP280_ULTRA_LOW_POWER_MODE))return 5;
		if(BMP280_SetStandbyDurn(BMP280_T_SB_0_5MS))return 6;
	}return 0;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_CalibParam(void)
*	功能说明:BMP280校准参数
*	形    参：
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_CalibParam(void)
{
	  u8 a_data_u8[24],res = 0;
	  memset(a_data_u8,0,24*sizeof(u8));
	  res =BMP280_Read_Len(BMP280_DIG_T1_LSB_REG,24,a_data_u8);
		p_bmp280.calib_param.dig_T1 = (u16)((((u16)((u8)a_data_u8[1]))<<8)|a_data_u8[0]);
		p_bmp280.calib_param.dig_T2 = (s16)((((s16)((s8)a_data_u8[3]))<<8)|a_data_u8[2]);
		p_bmp280.calib_param.dig_T3 = (s16)((((s16)((s8)a_data_u8[5]))<<8)|a_data_u8[4]);
		p_bmp280.calib_param.dig_P1 = (u16)((((u16)((u8)a_data_u8[7]))<<8)|a_data_u8[6]);
		p_bmp280.calib_param.dig_P2 = (s16)((((s16)((s8)a_data_u8[9]))<<8)|a_data_u8[8]);
		p_bmp280.calib_param.dig_P3 = (s16)((((s16)((s8)a_data_u8[11]))<<8)|a_data_u8[10]);
		p_bmp280.calib_param.dig_P4 = (s16)((((s16)((s8)a_data_u8[13]))<<8)|a_data_u8[12]);
		p_bmp280.calib_param.dig_P5 = (s16)((((s16)((s8)a_data_u8[15]))<<8)|a_data_u8[14]);
		p_bmp280.calib_param.dig_P6 = (s16)((((s16)((s8)a_data_u8[17]))<<8)|a_data_u8[16]);
		p_bmp280.calib_param.dig_P7 = (s16)((((s16)((s8)a_data_u8[19]))<<8)|a_data_u8[18]);
		p_bmp280.calib_param.dig_P8 = (s16)((((s16)((s8)a_data_u8[21]))<<8)|a_data_u8[20]);
		p_bmp280.calib_param.dig_P9 = (s16)((((s16)((s8)a_data_u8[23]))<<8)|a_data_u8[22]);
	  return res;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_SetPowerMode(u8 mode)
*	功能说明: 设置BMP280电源工作模式
*	形    参：mode：0,1,2,3 ，
*    		  0：SLEEP_MODE，休眠模式
*    		  1OR2：FORCED_MODE，读取一次后进入SLEEP_MODE.
*    		  3：正常工作模式
*	返 回 值: 0：正常
*         	  1：连接不正常
*         	  2：参数错误 
*********************************************************************************************************
*/
u8 BMP280_SetPowerMode(u8 mode)
{
	u8 v_mode_u8 = 0,res = 0;
			if (mode <= BMP280_NORMAL_MODE) 
			{
			   v_mode_u8 = (p_bmp280.oversamp_temperature<<5)+(p_bmp280.oversamp_pressure<<2)+mode;
				 res = BMP280_Write_Byte(BMP280_CTRLMEAS_REG,v_mode_u8);
		} else res = 2;
		return res;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_SetWorkMode(WORKING_MODE mode)
*	功能说明: 设置BMP280过采样模式设置,可以自己增加模式
*	形    参：BMP280_ULTRA_LOW_POWER_MODE    ,
*  			  BMP280_LOW_POWER_MODE          ,
*			  BMP280_STANDARD_RESOLUTION_MODE,
*			  BMP280_HIGH_RESOLUTION_MODE    ,
*			  BMP280_ULTRA_HIGH_RESOLUTION_MODE
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_SetWorkMode(WORKING_MODE mode)
{
	u8 res = 0,v_data_u8 = 0;
		if (mode <= 0x04) {
			v_data_u8 = BMP280_Read_Byte(BMP280_CTRLMEAS_REG);//读取出控制寄存器的值
			switch (mode)
			{
					case BMP280_ULTRA_LOW_POWER_MODE:
						     p_bmp280.oversamp_temperature =BMP280_P_MODE_x1;
								 p_bmp280.oversamp_pressure    =BMP280_P_MODE_x1;
								 break;
					case BMP280_LOW_POWER_MODE:
						     p_bmp280.oversamp_temperature =BMP280_P_MODE_x1;
								 p_bmp280.oversamp_pressure    =BMP280_P_MODE_x2;
								 break;
					case BMP280_STANDARD_RESOLUTION_MODE:
						     p_bmp280.oversamp_temperature =BMP280_P_MODE_x1;
								 p_bmp280.oversamp_pressure    =BMP280_P_MODE_x4;				
								 break;
					case BMP280_HIGH_RESOLUTION_MODE:
						     p_bmp280.oversamp_temperature =BMP280_P_MODE_x1;
								 p_bmp280.oversamp_pressure    =BMP280_P_MODE_x8;
								 break;
					case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
						     p_bmp280.oversamp_temperature =BMP280_P_MODE_x2;
								 p_bmp280.oversamp_pressure    =BMP280_P_MODE_x16;
								 break;
			}
			v_data_u8 = ((v_data_u8 & ~0xE0) | ((p_bmp280.oversamp_temperature<<5)&0xE0));
			v_data_u8 = ((v_data_u8 & ~0x1C) | ((p_bmp280.oversamp_pressure<<2)&0x1C));
			res = BMP280_Write_Byte(BMP280_CTRLMEAS_REG,v_data_u8);
	} else res = 1;
	return res;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_SetStandbyDurn(BMP280_T_SB standby_durn)
*	功能说明: 闲置时长设置，即两次获取温度和气压的间隔时间长度
*	形    参：standby_durn：
*  BMP280_T_SB_0_5MS              ：0.5ms   
*  BMP280_T_SB_62_5MS             ：62.5ms  
*  BMP280_T_SB_125MS              ：125ms   
*  BMP280_T_SB_250MS              ：250ms   
*  BMP280_T_SB_500MS              ：500ms   
*  BMP280_T_SB_1000MS             ：1000ms  
*  BMP280_T_SB_2000MS             ：2000ms  
*  BMP280_T_SB_4000MS             ：4000ms 
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_SetStandbyDurn(BMP280_T_SB standby_durn)
{
		u8 v_data_u8 = 0;
		v_data_u8 = BMP280_Read_Byte(BMP280_CONFIG_REG);//读取出寄存器的值
		v_data_u8 = ((v_data_u8 & ~0xE0) | ((standby_durn<<5)&0xE0));//高3位
		return BMP280_Write_Byte(BMP280_CONFIG_REG,v_data_u8);
}
/**
  * @brief  获取闲置时长，即两次获取温度和气压的间隔时间长度
  * @param  v_standby_durn_u8：
  *  BMP280_T_SB_0_5MS              ：0.5ms   
  *  BMP280_T_SB_62_5MS             ：62.5ms  
  *  BMP280_T_SB_125MS              ：125ms   
  *  BMP280_T_SB_250MS              ：250ms   
  *  BMP280_T_SB_500MS              ：500ms   
  *  BMP280_T_SB_1000MS             ：1000ms  
  *  BMP280_T_SB_2000MS             ：2000ms  
  *  BMP280_T_SB_4000MS             ：4000ms 
  * @retval 0：正常
  *         1：不正常
  */
u8 BMP280_GetStandbyDurn(u8* v_standby_durn_u8)
{
	u8 res  = 0,v_data_u8 = 0;
	res = v_data_u8 = BMP280_Read_Byte(BMP280_CONFIG_REG);
	*v_standby_durn_u8 = (v_data_u8>>5);
	return res;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_ReadUncompTemperature(int32_t* un_temp)
*	功能说明: 获取未补偿温度
*	形    参：un_temp：数据指针
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_ReadUncompTemperature(int32_t* un_temp)
{
		u8 a_data_u8r[3]= {0,0,0},res=0;
	  res = BMP280_Read_Len(BMP280_TEMPERATURE_MSB_REG,3,a_data_u8r);
	  *un_temp = (int32_t)((((u32)(a_data_u8r[0]))<<12)|(((u32)(a_data_u8r[1]))<<4)|((u32)a_data_u8r[2]>>4));
		return res;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_ReadUncompPressuree(int32_t *un_press)
*	功能说明: 获取未补偿气压
*	形    参：un_temp：数据指针
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_ReadUncompPressuree(int32_t *un_press)
{
		u8 a_data_u8r[3]= {0,0,0},res = 0;
		res = BMP280_Read_Len(BMP280_PRESSURE_MSB_REG,3,a_data_u8r);
		*un_press = (int32_t)((((u32)(a_data_u8r[0]))<<12)|(((u32)(a_data_u8r[1]))<<4)|((u32)a_data_u8r[2]>>4));
		return res;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_ReadUncompPressureTemperature(int32_t *un_press, int32_t *un_temp)
*	功能说明: 获取未补偿气压和温度（一起获取，一次读取6个字节数据，比分开读取速度快一丢丢）
*	形    参：un_press：未补偿气压数据指针，un_temp：未补偿温度数据指针
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_ReadUncompPressureTemperature(int32_t *un_press, int32_t *un_temp)
{
		u8 a_data_u8[6] = {0,0,0,0,0,0},res = 0;
		 res = BMP280_Read_Len(BMP280_PRESSURE_MSB_REG,6,a_data_u8);
		*un_press = (int32_t)((((u32)(a_data_u8[0]))<<12)|(((u32)(a_data_u8[1]))<<4)|((u32)a_data_u8[2]>>4));/*气压*/
		*un_temp = (int32_t)((((u32)(a_data_u8[3]))<<12)| (((u32)(a_data_u8[4]))<<4)|((u32)a_data_u8[5]>>4));/* 温度 */
		return res;
}
/*
*********************************************************************************************************
*	函 数 名: int32_t BMP280_CompensateTemperatureInt32(int32_t un_temp)
*	功能说明: 获取真实的温度
*	形    参：un_temp：未补偿温度数据
*	返 回 值: int32_t：温度值，例如：2255代表22.55 DegC
*********************************************************************************************************
*/
int32_t BMP280_CompensateTemperatureInt32(int32_t un_temp)
{
	int32_t v_x1_u32r = 0;
	int32_t v_x2_u32r = 0;
	int32_t temperature = 0;
	v_x1_u32r = ((((un_temp>>3)-((int32_t)p_bmp280.calib_param.dig_T1<<1)))*((int32_t)p_bmp280.calib_param.dig_T2))>>11;
	v_x2_u32r = (((((un_temp>>4)-((int32_t)p_bmp280.calib_param.dig_T1))*((un_temp>>4)-((int32_t)p_bmp280.calib_param.dig_T1)))>>12)*((int32_t)p_bmp280.calib_param.dig_T3))>>14;
	p_bmp280.calib_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature = (p_bmp280.calib_param.t_fine * 5 + 128)>> 8;
	return temperature;
}
/*
*********************************************************************************************************
*	函 数 名: u32 BMP280_CompensatePressureInt32(int32_t un_press)
*	功能说明: 获取真实的气压
*	形    参：un_press：未补偿气压
*	返 回 值: u32：真实的气压值 
*********************************************************************************************************
*/
u32 BMP280_CompensatePressureInt32(int32_t un_press)
{
	int32_t v_x1_u32r = 0;
	int32_t v_x2_u32r = 0;
	u32 v_pressure_u32 = 0;
	v_x1_u32r = (((int32_t)p_bmp280.calib_param.t_fine)>> 1) - (int32_t)64000;
	v_x2_u32r = (((v_x1_u32r >> 2)* (v_x1_u32r >> 2))>> 11)* ((int32_t)p_bmp280.calib_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *((int32_t)p_bmp280.calib_param.dig_P5))<< 1);
	v_x2_u32r = (v_x2_u32r >> 2)+ (((int32_t)p_bmp280.calib_param.dig_P4)<< 16);
	v_x1_u32r = (((p_bmp280.calib_param.dig_P3*(((v_x1_u32r>>2)*(v_x1_u32r>>2))>>13))>>3)+((((int32_t)p_bmp280.calib_param.dig_P2)* v_x1_u32r)>>1))>>18;
	v_x1_u32r = ((((32768 + v_x1_u32r))* ((int32_t)p_bmp280.calib_param.dig_P1))>> 15);
	v_pressure_u32 = (((u32)(((int32_t)1048576) - un_press)- (v_x2_u32r >> 12)))* 3125;
	if (v_pressure_u32 < 0x80000000)
		if (v_x1_u32r != 0)
			v_pressure_u32 = (v_pressure_u32<< 1)/ ((u32)v_x1_u32r);
		else return 0;
	else if (v_x1_u32r != 0)
		v_pressure_u32 = (v_pressure_u32 / (u32)v_x1_u32r) * 2;
	else	return 0;
	v_x1_u32r = (((int32_t)p_bmp280.calib_param.dig_P9) * ((int32_t)(((v_pressure_u32>> 3)* (v_pressure_u32>> 3))>> 13)))>> 12;
	v_x2_u32r = (((int32_t)(v_pressure_u32 >>	2))	* ((int32_t)p_bmp280.calib_param.dig_P8))>> 13;
	v_pressure_u32 = (u32)((int32_t)v_pressure_u32 + ((v_x1_u32r + v_x2_u32r+ p_bmp280.calib_param.dig_P7)>> 4));
	return v_pressure_u32;
}
/*
*********************************************************************************************************
*	函 数 名: u8 BMP280_ReadPressureTemperature(u32 *press, int32_t *temp)
*	功能说明: 获取真实气压和温度
*	形    参：press：真实的气压指针，temp：真实的温度指针
*	返 回 值: 其他表示失败，0表示正常
*********************************************************************************************************
*/
u8 BMP280_ReadPressureTemperature(u32 *press, int32_t *temp)
{
		int32_t un_press = 0;
		int32_t un_temp = 0;
		u8 res=0;
		res = BMP280_ReadUncompPressureTemperature(&un_press,&un_temp);
		/* 读取真实的温度值和气压值*/
		*temp = BMP280_CompensateTemperatureInt32(un_temp);
		*press = BMP280_CompensatePressureInt32(un_press);
	return res;
}


