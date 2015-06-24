#include "MiniBalance.h"
#include "math.h"
#include "led.h"
#include "mpu6050.h"
#define PI 3.14159265
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
�������ܣ�5MS��ʱ�жϺ��� 5MS��������
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
void TIM1_UP_TIM16_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms��ʱ�ж�
	{   
		  TIM1->SR&=~(1<<0);                                       //===�����ʱ��1�жϱ�־λ		 
			readEncoder();                                           //===��ȡ��������ֵ
  		Led_Flash(400);                                          //===LED��˸;	
  		Get_battery_volt();                                      //===��ȡ��ص�ѹ	          
			key(100);                                                //===ɨ�谴��״̬
		  Get_Angle(Way_Angle);                                    //===������̬	
 			Balance_Pwm =balance(Angle_Balance,Gyro_Balance);        //===ƽ��PID����	
 			Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===�ٶȻ�PID����
 	    Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn); //===ת��PID����     
 		  Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                 //===�������ֵ������PWM
 	  	Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                 //===�������ֵ������PWM
   		Xianfu_Pwm();                                            //===PWM�޷�
      if(Turn_Off(Angle_Balance,Voltage)==0)                   //===����������쳣
 			Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���    		
	}       
} 

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle+0;              //===���ƽ��ĽǶ���ֵ �ͻ�е��� +0��ζ������������0�ȸ��� �������������5�ȸ��� �Ǿ�Ӧ�ü�ȥ5
	 balance=35*Bias+Gyro*0.125;//===����ƽ����Ƶĵ��PWM  PD���� 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
	  static int Velocity,Encoder_Least,Encoder,Movement;
	  static int Encoder_Integral;
	  //=============ң��ǰ�����˲���=======================//
		if(1==Flag_Qian)	Movement=-900;	             //===���ǰ����־λ��1 λ��Ϊ��
		else if(1==Flag_Hou)	  Movement=900;          //===������˱�־λ��1 λ��Ϊ��
	  else  Movement=0;	
   //=============�ٶ�PI������======================//	
		Encoder_Least =Encoder_Left+Encoder_Right;     //===��ȡ�����ٶ�ƫ��
		Encoder *= 0.8;		                             //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                 //===һ�׵�ͨ�˲���    
	  if(Turn_Off(Angle_Balance,Voltage)==0)         //Ϊ�˷�ֹ����Ӱ���û����飬ֻ�е��������ʱ��ſ�ʼ����
		{	
  	Encoder_Integral +=Encoder;                     //===���ֳ�λ�� ����ʱ�䣺5ms
		Encoder_Integral=Encoder_Integral-Movement;     //===����ң�������ݣ�����ǰ������
		}
		if(Encoder_Integral>360000)  	Encoder_Integral=360000;          //===�����޷�
		if(Encoder_Integral<-360000)	Encoder_Integral=-360000;         //===�����޷�	
		Velocity=Encoder*4+Encoder_Integral/140;                        //===�ٶ�PI������	
		if(Turn_Off(Angle_Balance,Voltage)==1)   Encoder_Integral=0;    //===����رպ��������
	  return Velocity;
}

/**************************************************************************
�������ܣ�ת��PD����
��ڲ��������ֱ����������ֱ�������Z��������
����  ֵ��ת�����PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
  	static int Turn_Target,Turn,Encoder_temp,Turn_Convert=3,Turn_Count;
	  int Turn_Bias,Turn_Amplitude=1500/Way_Angle+800;     //===Way_AngleΪ�˲�����������1ʱ������DMP��ȡ��̬��Turn_Amplitudeȡ�󣬿������ͻ����ǣ�ȡС����Ϊ�������˲��㷨Ч���Բ
	  static long Turn_Bias_Integral;
	  //=============ң��������ת����=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
			Turn_Convert=2000/Encoder_temp;
			if(Turn_Convert<3)Turn_Convert=3;
			if(Turn_Convert>10)Turn_Convert=10;
		}	
	  else
		{
			Turn_Convert=3;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(1==Flag_Left)	           Turn_Target+=Turn_Convert; //��ת
		else if(1==Flag_Right)	     Turn_Target-=Turn_Convert; //��ת
		else Turn_Target=0;                                     //ֹͣ
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
  	//=============ת��PD������=======================//
		 Turn_Bias=Encoder_Left-Encoder_Right;         //===����ת���ٶ�ƫ��  
		if(Turn_Off(Angle_Balance,Voltage)==0)         //Ϊ�˷�ֹ����Ӱ���û����飬ֻ�е��������ʱ��ſ�ʼ����
		{	
		Turn_Bias_Integral+=Turn_Bias;                //ת���ٶ�ƫ����ֵõ�ת��ƫ��
		Turn_Bias_Integral-=Turn_Target;              //��ȡң��������
		}
		if(Turn_Bias_Integral>1800)  	Turn_Bias_Integral=1800;          //===�����޷�
		if(Turn_Bias_Integral<-1800)	Turn_Bias_Integral=-1800;         //===�����޷�	
	  Turn=Turn_Bias_Integral*2+gyro/12;                              //===���Z�������ǽ���PD����
	  return Turn;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
			if(moto1<0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto1);
		  if(moto2<0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2);	
}
/**************************************************************************
�������ܣ���ȡ�����������ݲ�������������ת��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void readEncoder(void)
{
	  u16 Encoder_L,Encoder_R;       //===���ұ��������������
		Encoder_R = TIM4 -> CNT;       //===��ȡ��������1����	
		TIM4 -> CNT=0;                 //===����������  
	  Encoder_L= TIM2 -> CNT;        //===��ȡ��������2����	
	  TIM2 -> CNT=0;	               //===����������
		if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  //===��������ת��
  	Encoder_Left=-Encoder_Left;
	  if(Encoder_R>32768)  Encoder_Right=Encoder_R-65000; else  Encoder_Right=Encoder_R;//===��������ת��
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=3500;    //===PWM������3600 ������3500
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
��    �ߣ�ƽ��С��֮��
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<-40||angle>40||1==Flag_Stop||Voltage<1110)//===��ѹ����11.1V �رյ��
			{	                                                 //===��Ǵ���40�ȹرյ��
      temp=1;                                            //===Flag_Stop��1�رյ��
			AIN1=0;                                            //===���������������¶ȹ���ʱ�رյ��
			AIN2=0;
			BIN1=0;
			BIN2=0;
      }
			else
      temp=0;
      return temp;			
}
	
/**************************************************************************
�������ܣ���ȡ�Ƕ�
��ڲ�������ȡ�Ƕȵ��㷨 1����  2�������� 3�������˲�
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	    if(way==1)                                      //DMPû���漰���ϸ��ʱ�����⣬����������ȡ
			{	
			}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		  if(Gyro_Y>32768)  Gyro_Y-=65536;     //��������ת��
			if(Gyro_Z>32768)  Gyro_Z-=65536;     //��������ת��
	  	if(Accel_X>32768) Accel_X-=65536;    //��������ת��
		  if(Accel_Z>32768) Accel_Z-=65536;    //��������ת��
			Gyro_Balance=-Gyro_Y;                                  //����ƽ����ٶ�
	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //���������ļн�	
		  Gyro_Y=Gyro_Y/16.4;                                    //����������ת��	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Y,-Gyro_Y);//�������˲�	
			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);    //�����˲�
	    Angle_Balance=angle;                                   //����ƽ�����
			Gyro_Turn=Gyro_Z;                                      //����ת����ٶ�
	  	}
}
