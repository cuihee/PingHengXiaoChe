#include "show.h"
#include "MiniBalance.h"
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
static u32 Count;

void oled_show(void)
{
	  Count=0;
		OLED_Display_On();  //显示屏打开
	//==========================边框显示=====================
													OLED_Fill(0,0, 127,0,1);
													OLED_Fill(0,0, 0,63,1);
													OLED_Fill(127,0, 127,63,1);
													OLED_Fill(0,63, 127,63,1);
		//=============显示滤波器=======================//	
		                      OLED_ShowString(01,0,"ALG-");
		                      OLED_ShowNumber(30,0, Way_Angle,1,12);
	       if(Way_Angle==1)	OLED_ShowString(45,0,"DMP");
		else if(Way_Angle==2)	OLED_ShowString(45,0,"Kalman");
		else if(Way_Angle==3)	OLED_ShowString(45,0,"Hubu");
		//=============显示温度=======================//	
		                      OLED_ShowString(01,10,"TemP");
		                      OLED_ShowNumber(45,10,Temperature/10,2,12);
		                      OLED_ShowNumber(68,10,Temperature%10,1,12);
		                      OLED_ShowString(58,10,".");
		                      OLED_ShowString(80,10,"`C");
		//=============显示编码器1=======================//	
		                      OLED_ShowString(01,20,"Enco");
		if( Encoder_Left<0)		OLED_ShowString(47,20,"-"),
		                      OLED_ShowNumber(56,20,-Encoder_Left,3,12);
		else                 	OLED_ShowString(47,20,"+"),
		                      OLED_ShowNumber(56,20, Encoder_Left,3,12);
  	//=============显示编码器2=======================//				                      
		if(Encoder_Right<0)		OLED_ShowString(85,20,"-"),
		                      OLED_ShowNumber(93,20,-Encoder_Right,3,12);
		else               		OLED_ShowString(85,20,"+"),
		                      OLED_ShowNumber(93,20,Encoder_Right,3,12);	
		//=============显示电压=======================//
		                      OLED_ShowString(01,30,"Volta");
		                      OLED_ShowString(58,30,".");
		                      OLED_ShowString(80,30,"V");
		                      OLED_ShowNumber(45,30,Voltage/100,2,12);
		                      OLED_ShowNumber(68,30,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(62,30,0,2,12);
		//=============显示角度=======================//
		                      OLED_ShowString(01,40,"Angle");
		if(Angle_Balance<0)		OLED_ShowString(46,40,"-"),
													OLED_ShowNumber(54,40,0-Angle_Balance,3,12);
		else					        OLED_ShowString(46,40,"+"),
													OLED_ShowNumber(54,40,Angle_Balance,3,12);
		//=============其他======================//
													
		
		//=============刷新=======================//
		OLED_Refresh_Gram();
	}
	
void DataScope(void)
{   
	  if(++Count==1)
		{	
		OLED_Clear();  
		OLED_Display_Off();		
		}	
		DataScope_Get_Channel_Data( Angle_Balance, 1 );
		DataScope_Get_Channel_Data( Encoder_Right, 2 );
		DataScope_Get_Channel_Data( Encoder_Left, 3 ); 
		DataScope_Get_Channel_Data( Voltage , 4 );   
		DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
		DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
		DataScope_Get_Channel_Data(0, 7 );
		DataScope_Get_Channel_Data( 0, 8 ); 
		DataScope_Get_Channel_Data(0, 9 );  
		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(10);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
		delay_ms(50); //20HZ
}
