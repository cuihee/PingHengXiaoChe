#include "key.h"
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��	   	 
	GPIOB->CRL&=0XFF0FFFFF; 
	GPIOB->CRL|=0X00800000;//PB5 �������루��Ϊ������������裩 
} 
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void key(u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single;
		if(0==KEY1&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
			if(flag_key==1) double_key++,count_key=1;	
			if(double_key==2) Flag_Show=!Flag_Show,double_key=0,count_single=0;
		}
		if(1==KEY1)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			if(++count_single>time)	Flag_Stop=!Flag_Stop,double_key=0,count_single=0;	
		}	
}
