#include "timer.h"
#include "mpu6050.h"
#include "led.h"
#include "math.h"
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
�������ܣ���������ȡ�ж�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{
			    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}
/**************************************************************************
�������ܣ���������ȡ�ж�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM4->SR;	
	if(tsr&0X0001)//����ж�
	{
																				
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	 
}
/**************************************************************************
�������ܣ���������ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Encoder_Init2(void)
{
	/* TIM2 clock source enable */ 
	RCC->APB1ENR|=1<<0;       //TIM2ʱ��ʹ��
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��

	/* Configure PA.00,01 as encoder input */
	GPIOA->CRL&=0XFFFFFFF0;//PA0
	GPIOA->CRL|=0X00000004;//��������
	GPIOA->CRL&=0XFFFFFF0F;//PA1
	GPIOA->CRL|=0X00000040;//��������

	/* Enable the TIM2 Update Interrupt */
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM2->DIER|=1<<0;   //��������ж�				
	TIM2->DIER|=1<<6;   //�������ж�
	MY_NVIC_Init(1,3,TIM2_IRQChannel,1);

	/* Timer configuration in Encoder mode */ 
	TIM2->PSC = 0x0;//Ԥ��Ƶ��
	TIM2->ARR = ENCODER_TIM_PERIOD-1;//�趨�������Զ���װֵ 
	TIM2->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM2->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM2->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM2->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM2->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM2->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM2->CNT = COUNTER_RESET;
	TIM2->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}

/**************************************************************************
�������ܣ���������ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Encoder_Init(void)
{
	/* TIM4 clock source enable */ 
	RCC->APB1ENR|=1<<2;       //TIM3ʱ��ʹ��
	/* Enable GPIOB, clock */
	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��

	/* Configure PB.06,07 as encoder input */
	GPIOB->CRL&=0XF0FFFFFF;//PA6
	GPIOB->CRL|=0X08000000;//��������
	GPIOB->CRL&=0X0FFFFFFF;//PA7
	GPIOB->CRL|=0X40000000;//��������

	/* Enable the TIM3 Update Interrupt */
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM4->DIER|=1<<0;   //��������ж�				
	TIM4->DIER|=1<<6;   //�������ж�
	MY_NVIC_Init(1,3,TIM4_IRQChannel,1);

	/* Timer configuration in Encoder mode */ 
	TIM4->PSC = 0x0;//Ԥ��Ƶ��
	TIM4->ARR = ENCODER_TIM_PERIOD-1;//�趨�������Զ���װֵ 
	TIM4->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM4->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM4->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM4->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM4->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM4->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM4->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM4->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM4->CNT = COUNTER_RESET;
	TIM4->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}


/**************************************************************************
�������ܣ���ʱ�жϳ�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void Timer1_Init(u16 arr,u16 psc)  
{  
	RCC->APB2ENR|=1<<11;//TIM2ʱ��ʹ��    
 	TIM1->ARR=arr;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM1->PSC=psc;  //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
	TIM1->DIER|=1<<0;   //��������ж�				
	TIM1->DIER|=1<<6;   //�������ж�	   
	TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��
	MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,1);
}  

/**************************************************************************
�������ܣ�PWM �Լ�������Ƶ�IO��ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB1ENR|=1<<1;       //TIM3ʱ��ʹ��    
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��     
	GPIOB->CRL&=0XFFFFFF00;   //PORTB0 1�������
	GPIOB->CRL|=0X000000BB;   //PORTB0 1�������
	GPIOB->CRH&=0X0000FFFF;   //PORTB12 13 14 15�������
	GPIOB->CRH|=0X22220000;   //PORTB12 13 14 15�������
	TIM3->ARR=arr;//�趨�������Զ���װֵ 
	TIM3->PSC=psc;//Ԥ��Ƶ������Ƶ
	TIM3->CCMR2|=6<<12;//CH4 PWM2ģʽ	
	TIM3->CCMR2|=6<<4; //CH3 PWM2ģʽ	
	TIM3->CCMR2|=1<<11;//CH4Ԥװ��ʹ��	 
	TIM3->CCMR2|=1<<3; //CH3Ԥװ��ʹ��	  
	TIM3->CCER|=1<<12; //CH4���ʹ��	   
	TIM3->CCER|=1<<8;  //CH3���ʹ��	
	TIM3->CR1=0x8000;  //ARPEʹ�� 
	TIM3->CR1|=0x01;   //ʹ�ܶ�ʱ��3 										  
} 

void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	
	u8 IPROFFSET=NVIC_Channel%4;//�����ڵ�ƫ��
	IPROFFSET=IPROFFSET*8+4;    //�õ�ƫ�Ƶ�ȷ��λ��
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//���÷���
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;//ȡ����λ
	if(NVIC_Channel<32)NVIC->ISER[0]|=1<<NVIC_Channel;//ʹ���ж�λ(Ҫ����Ļ�,�෴������OK)
	else NVIC->ISER[1]|=1<<(NVIC_Channel-32);       	    	  				   
}
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//ȡ����λ
	temp1<<=8;
	temp=SCB->AIRCR;  //��ȡ��ǰ������
	temp&=0X0000F8FF; //�����ǰ����
	temp|=0X05FA0000; //д��Կ��
	temp|=temp1;	   
	SCB->AIRCR=temp;  //���÷���	    	  				   
}
