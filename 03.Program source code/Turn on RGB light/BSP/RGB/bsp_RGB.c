#include "bsp_RGB.h"

void init_RGB_GPIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RGB_CLK, ENABLE);	 //ʹ��C�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = RGB_LED_R_PIN|RGB_LED_G_PIN|RGB_LED_B_PIN;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//�ٶ�2MHz
 	GPIO_Init(RGB_PORT, &GPIO_InitStructure);	  //��ʼ��GPIOC
	
 	GPIO_SetBits(RGB_PORT,RGB_LED_R_PIN|RGB_LED_G_PIN|RGB_LED_B_PIN);//�ߵ�ƽ�ص�
}
	
	

	
void init_LED_GPIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��C�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//�ٶ�2MHz
 	GPIO_Init(RGB_PORT, &GPIO_InitStructure);	  //��ʼ��GPIOC
	
 	GPIO_ResetBits(GPIOB,GPIO_Pin_4);

}	

//���� RGB�� ��ɫ
void RGB_control(u8 color)
{
	switch(color)
	{
		case red: 		RGB_SET(0,1,1);break;//��ɫ
		case green:		RGB_SET(1,0,1)break;//��ɫ
		case blue:		RGB_SET(1,1,0)break;//��ɫ
		case yellow:	RGB_SET(0,0,1)break;//��ɫ  
		case purper:	RGB_SET(0,1,0)break;//��ɫ
		case lake:		RGB_SET(1,0,0)break;//��ɫ
		case write:		RGB_SET(0,0,0)break;//��ɫ	
		default   :		RGB_OFF;
	}

}

