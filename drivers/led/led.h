#ifndef __LED_H
#define __LED_H 
/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"
//LED�˿ڶ���
#define LED1 PBout(8)// PD2	
void led_init(void);
void Led_Flash(u16 time);
#endif
