#include "mpu9250.h"

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU9250_Init(void)
*	功能说明: MPU9250初始化设置，配置陀螺仪，加速度传感器测量范围与采样速率等
*	形    参：
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU9250_Init(void)
{
    uint8_t res=0;
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
    //SDO(PB3)和NCS(PB5)初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_5);
    GPIO_ResetBits(GPIOB,GPIO_Pin_3);

    //IIC_CfgGpio();     //初始化IIC总线
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    delay_ms(100);  //延时100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±1000dps
	MPU_Set_Accel_Fsr(2);					       	 	//加速度传感器,±8g
    MPU_Set_Rate(1000);						       	 	//设置采样率1000Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID) //器件ID正确
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		//MPU_Set_Rate(50);						       	        //设置采样率为50Hz   
    }else return 3;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		// set AK8963 to 16 bit resolution, 100 Hz update rate
        delay_ms(100);
    }else return 1;

    return 0;
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
*	功能说明: 设置MPU9250陀螺仪传感器满量程范围
*	形    参：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±1000dps
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
*	功能说明: 设置MPU9250加速度传感器满量程范围
*	形    参:fsr:0,±2g;1,±4g;2,±8g;3,±16g
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Set_LPF(u16 lpf)
*	功能说明: 设置MPU9250的数字低通滤波器
*	形    参：lpf:数字低通滤波频率(Hz)
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Set_LPF(u16 lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Set_Rate(u16 rate)
*	功能说明: 设置MPU9250的采样率(假定Fs=1KHz)
*	形    参：rate:4~1000(Hz)
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Set_Rate(u16 rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

/*
*********************************************************************************************************
*	函 数 名: short MPU_Get_Temperature(void)
*	功能说明: 设置MPU9250读取当前温度并返回
*	形    参：
*	返 回 值: 温度值(扩大了100倍)
*********************************************************************************************************
*/
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87; 

    return temp*100;;
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
*	功能说明: 设置MPU9250读取得到陀螺仪值(原始值)
*	形    参：gx,gy,gz:陀螺仪x,y,z轴的原始读数存放指针(带符号)
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Get_Gyroscope(Vector3f_t *gyro)
{
    uint8_t buf[6],res;
    Vector3i_t gyroraw; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		gyroraw.x = ((int16_t)buf[0]<<8)|buf[1];  
		gyroraw.y = ((int16_t)buf[2]<<8)|buf[3];  
		gyroraw.z = ((int16_t)buf[4]<<8)|buf[5];

        //统一传感器坐标到ENU
        // gyroraw.x = gyroraw.x;
        // gyroraw.y = gyroraw.y;
        // gyroraw.z = gyroraw.z;

        gyro->x = gyroraw.x * MPU_G_s1000dps;
        gyro->y = gyroraw.y * MPU_G_s1000dps;
        gyro->z = gyroraw.z * MPU_G_s1000dps;
	} 	
    return res;;
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
*	功能说明: 设置MPU9250读取得到加速度值(原始值)
*	形    参：ax,ay,az:加速度计x,y,z轴的原始读数存放指针(带符号)
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Get_Accelerometer(Vector3f_t *acc)
{
    uint8_t buf[6],res; 
    Vector3i_t accraw; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		accraw.x = ((int16_t)buf[0]<<8)|buf[1];  
		accraw.y = ((int16_t)buf[2]<<8)|buf[3];  
		accraw.z = ((int16_t)buf[4]<<8)|buf[5];

        //统一传感器坐标到ENU
        // accraw.x = accraw.x;
        // accraw.y = accraw.y;
        // accraw.z = accraw.z;

        acc->x = accraw.x * MPU_A_8mg;
        acc->y = accraw.y * MPU_A_8mg;
        acc->z = accraw.z * MPU_A_8mg;
	} 	
    return res;
}

static Vector3i_t magraw;
/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
*	功能说明: 设置MPU9250读取得到磁力计值(原始值)
*	形    参：mx,my,mz:磁力计x,y,z轴的原始读数存放指针(带符号)
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Get_Magnetometer(void)
{
    uint8_t buf[6],res;
    //MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		magraw.y = ((int16_t)buf[1]<<8)|buf[0];  
		magraw.x = ((int16_t)buf[3]<<8)|buf[2];  
		magraw.z = ((int16_t)buf[5]<<8)|buf[4];

        //统一传感器坐标到NEU
        magraw.x = magraw.x;
        magraw.y = magraw.y;
        magraw.z = -magraw.z;

	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    delay_us(5);
    return res;;
}

/**********************************************************************************************************
*函 数 名: QMC5883_Read
*功能说明: 读取地磁传感器数据,并转换为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MagRaw_Read(Vector3f_t* mag)
{
    mag->x = magraw.x * MAG_1G;
    mag->y = magraw.y * MAG_1G;
    mag->z = magraw.z * MAG_1G;
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
*	功能说明: IIC从指定地址写入MPU指定长度数据
*	形    参：addr:器件地址 
*             reg:寄存器地址
*             len:写入数据长度
*             buf:数据区
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_WaitAck())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    // IIC_SendByte(reg);         //写寄存器地址
    // IIC_WaitAck();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC_SendByte(reg);         //写寄存器地址
        IIC_WaitAck();             //等待应答
        IIC_SendByte(buf[i]);  //发送数据
        if(IIC_WaitAck())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
        reg++;
    }
    IIC_Stop();
    return 0;
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
*	功能说明: IIC从指定地址读取MPU指定长度数据
*	形    参：addr:器件地址 
*             reg:寄存器地址
*             len:读出数据长度
*             buf:数据区
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_WaitAck())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    while (len)
    {
        *buf++ = MPU_Read_Byte(addr,reg++);
        len--;

    }
    
    return 0;       
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
*	功能说明: IIC从指定地址写入MPU一字节长度数据
*	形    参：addr:器件地址 
*             reg:寄存器地址
*             data:数据
*	返 回 值: 其他表示失败，0表示成功
*********************************************************************************************************
*/
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_WaitAck())          //等待应答
    {
        IIC_Stop();
        return 2;
    }
    IIC_SendByte(reg);         //写寄存器地址
    IIC_WaitAck();             //等待应答
    IIC_SendByte(data);        //发送数据
    if(IIC_WaitAck())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

/*
*********************************************************************************************************
*	函 数 名: uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
*	功能说明: IIC从指定地址读取MPU一字节长度数据
*	形    参：addr:器件地址 
*             reg:寄存器地址
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res = 0;
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //发送器件地址+写命令
    IIC_WaitAck();             //等待应答
    IIC_SendByte(reg);         //写寄存器地址
    IIC_WaitAck();             //等待应答
	IIC_Start();                
    IIC_SendByte((addr<<1)|1); //发送器件地址+读命令
    IIC_WaitAck();             //等待应答
    res=IIC_ReadByte(0);	   //读数据,发送nACK  
    IIC_Stop();                //产生一个停止条件
    return res;  
}
