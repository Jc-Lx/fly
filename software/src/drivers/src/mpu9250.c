#include "mpu9250.h"

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU9250_Init(void)
*	����˵��: MPU9250��ʼ�����ã����������ǣ����ٶȴ�����������Χ��������ʵ�
*	��    �Σ�
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU9250_Init(void)
{
    uint8_t res=0;
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
    //SDO(PB3)��NCS(PB5)��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_5);
    GPIO_ResetBits(GPIOB,GPIO_Pin_3);

    //IIC_CfgGpio();     //��ʼ��IIC����
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    delay_ms(100);  //��ʱ100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��1000dps
	MPU_Set_Accel_Fsr(2);					       	 	//���ٶȴ�����,��8g
    MPU_Set_Rate(1000);						       	 	//���ò�����1000Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
    if(res==MPU6500_ID) //����ID��ȷ
    {
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		//MPU_Set_Rate(50);						       	        //���ò�����Ϊ50Hz   
    }else return 3;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		// set AK8963 to 16 bit resolution, 100 Hz update rate
        delay_ms(100);
    }else return 1;

    return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
*	����˵��: ����MPU9250�����Ǵ����������̷�Χ
*	��    �Σ�fsr:0,��250dps;1,��500dps;2,��1000dps;3,��1000dps
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
*	����˵��: ����MPU9250���ٶȴ����������̷�Χ
*	��    ��:fsr:0,��2g;1,��4g;2,��8g;3,��16g
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU_Set_LPF(u16 lpf)
*	����˵��: ����MPU9250�����ֵ�ͨ�˲���
*	��    �Σ�lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
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
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU_Set_Rate(u16 rate)
*	����˵��: ����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
*	��    �Σ�rate:4~1000(Hz)
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Set_Rate(u16 rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

/*
*********************************************************************************************************
*	�� �� ��: short MPU_Get_Temperature(void)
*	����˵��: ����MPU9250��ȡ��ǰ�¶Ȳ�����
*	��    �Σ�
*	�� �� ֵ: �¶�ֵ(������100��)
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
*	�� �� ��: uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
*	����˵��: ����MPU9250��ȡ�õ�������ֵ(ԭʼֵ)
*	��    �Σ�gx,gy,gz:������x,y,z���ԭʼ�������ָ��(������)
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
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

        //ͳһ���������굽ENU
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
*	�� �� ��: uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
*	����˵��: ����MPU9250��ȡ�õ����ٶ�ֵ(ԭʼֵ)
*	��    �Σ�ax,ay,az:���ٶȼ�x,y,z���ԭʼ�������ָ��(������)
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
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

        //ͳһ���������굽ENU
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
*	�� �� ��: uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
*	����˵��: ����MPU9250��ȡ�õ�������ֵ(ԭʼֵ)
*	��    �Σ�mx,my,mz:������x,y,z���ԭʼ�������ָ��(������)
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Get_Magnetometer(void)
{
    uint8_t buf[6],res;
    //MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ  
	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		magraw.y = ((int16_t)buf[1]<<8)|buf[0];  
		magraw.x = ((int16_t)buf[3]<<8)|buf[2];  
		magraw.z = ((int16_t)buf[5]<<8)|buf[4];

        //ͳһ���������굽NEU
        magraw.x = magraw.x;
        magraw.y = magraw.y;
        magraw.z = -magraw.z;

	} 	
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    delay_us(5);
    return res;;
}

/**********************************************************************************************************
*�� �� ��: QMC5883_Read
*����˵��: ��ȡ�شŴ���������,��ת��Ϊ��׼��λ
*��    ��: ��������ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
void MagRaw_Read(Vector3f_t* mag)
{
    mag->x = magraw.x * MAG_1G;
    mag->y = magraw.y * MAG_1G;
    mag->z = magraw.z * MAG_1G;
}

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
*	����˵��: IIC��ָ����ַд��MPUָ����������
*	��    �Σ�addr:������ַ 
*             reg:�Ĵ�����ַ
*             len:д�����ݳ���
*             buf:������
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //����������ַ+д����
    if(IIC_WaitAck())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    // IIC_SendByte(reg);         //д�Ĵ�����ַ
    // IIC_WaitAck();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_SendByte(reg);         //д�Ĵ�����ַ
        IIC_WaitAck();             //�ȴ�Ӧ��
        IIC_SendByte(buf[i]);  //��������
        if(IIC_WaitAck())      //�ȴ�ACK
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
*	�� �� ��: uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
*	����˵��: IIC��ָ����ַ��ȡMPUָ����������
*	��    �Σ�addr:������ַ 
*             reg:�Ĵ�����ַ
*             len:�������ݳ���
*             buf:������
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //����������ַ+д����
    if(IIC_WaitAck())          //�ȴ�Ӧ��
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
*	�� �� ��: uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
*	����˵��: IIC��ָ����ַд��MPUһ�ֽڳ�������
*	��    �Σ�addr:������ַ 
*             reg:�Ĵ�����ַ
*             data:����
*	�� �� ֵ: ������ʾʧ�ܣ�0��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //����������ַ+д����
    if(IIC_WaitAck())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 2;
    }
    IIC_SendByte(reg);         //д�Ĵ�����ַ
    IIC_WaitAck();             //�ȴ�Ӧ��
    IIC_SendByte(data);        //��������
    if(IIC_WaitAck())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
*	����˵��: IIC��ָ����ַ��ȡMPUһ�ֽڳ�������
*	��    �Σ�addr:������ַ 
*             reg:�Ĵ�����ַ
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res = 0;
    IIC_Start();
    IIC_SendByte((addr<<1)|0); //����������ַ+д����
    IIC_WaitAck();             //�ȴ�Ӧ��
    IIC_SendByte(reg);         //д�Ĵ�����ַ
    IIC_WaitAck();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_SendByte((addr<<1)|1); //����������ַ+������
    IIC_WaitAck();             //�ȴ�Ӧ��
    res=IIC_ReadByte(0);	   //������,����nACK  
    IIC_Stop();                //����һ��ֹͣ����
    return res;  
}
