#include "main.h"
u8 	Way_Angle=2;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� ���е�6050ʹ��DMPʱ����Ҫ������ͣҡ��С��10S���ң��ȴ������ȶ���
u8 	Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //����ң����صı���
u8 	Flag_Stop=1;													//ֹͣ��־λ��
u8	Flag_Show=1;                						// ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //�������
int Moto1,Moto2;                            //���PWM���� 
int Temperature;                            //�¶�
int Voltage;                                //��ѹ����
float Angle_Balance,Gyro_Balance,Gyro_Turn,Gyro_AlarmMove; //ƽ����� ƽ�������� ת�������� �����ܵ��������˶�
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������

int main(void)
{
	SystemInit();                   //=====ϵͳ��ʼ��
	delay_init(72);                 //=====��ʱ����
	usart1_init();                  //=====����1��ʼ�� �����ʣ�115200
	uart3_init(72,9600);            //=====����3��ʼ�� �����ʣ�9600
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
  led_init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
	Adc_Init();	                    //=====��ʼ��ADCģ��
	MiniBalance_PWM_Init(3599,0);   //=====��ʼ��PWM 20KHZ ��Ƶ���Է�ֹ�����Ƶʱ�ļ����
	OLED_Init();	                  //=====��ʼ��OLED ģ��SPI 
	Encoder_Init();                 //=====��ʼ��������1
	Encoder_Init2();	              //=====��ʼ��������2
	delay_ms(200);                  //=====��ʱ�ȴ��ȶ�		
  IIC_Init();                     //=====ģ��IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //=====DMP��ʼ��
	Timer1_Init(49,7199);           //=====5MS��һ���жϷ����� �жϷ�������minibalance.c����
  while(1)
  {
				if(Way_Angle==1)               //DMPû���漰���ϸ��ʱ�����⣬����������ȡ
				{
					Read_DMP();                      //===��ȡ���ٶȺ����
					Angle_Balance=Pitch;             //===����ƽ�����
					Gyro_Balance=gyro[1];            //===����ƽ����ٶ�
					Gyro_Turn=gyro[2];               //===����ת����ٶȣ�����ʣ��һ�����ٶ�������������ֱ���ڣ�����ת�ģ�
					Gyro_AlarmMove = gyro[3];
					Temperature = Read_Temperature();  //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
					if(1==Flag_Show)		oled_show(); //===��ʾ����
				}
				if(Flag_Stop==1||Way_Angle>1)   
				{
					Temperature = Read_Temperature();  //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
					if(1==Flag_Show)		oled_show(); //===��ʾ����
						//else	              DataScope(); //===��ʾ���ر� ����λ��	����ʾ������λ������ͬʱʹ�ã�����Ҳû�б�Ҫ������		
				}
	}	
}

