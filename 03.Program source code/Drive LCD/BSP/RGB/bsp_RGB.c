#include "bsp_RGB.h"

void init_RGB_GPIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RGB_CLK, ENABLE);	 //使能C端口时钟
	GPIO_InitStructure.GPIO_Pin = RGB_LED_R_PIN|RGB_LED_G_PIN|RGB_LED_B_PIN;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//速度2MHz
 	GPIO_Init(RGB_PORT, &GPIO_InitStructure);	  //初始化GPIOC
	
 	GPIO_SetBits(RGB_PORT,RGB_LED_R_PIN|RGB_LED_G_PIN|RGB_LED_B_PIN);//高电平关灯
}
	
	

	
void init_LED_GPIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能C端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//速度2MHz
 	GPIO_Init(RGB_PORT, &GPIO_InitStructure);	  //初始化GPIOC
	
 	GPIO_ResetBits(GPIOB,GPIO_Pin_4);

}	

//设置 RGB灯 颜色
void RGB_control(u8 color)
{
	switch(color)
	{
		case red: 		RGB_SET(0,1,1);break;//红色
		case green:		RGB_SET(1,0,1)break;//绿色
		case blue:		RGB_SET(1,1,0)break;//蓝色
		case yellow:	RGB_SET(0,0,1)break;//黄色  
		case purper:	RGB_SET(0,1,0)break;//紫色
		case lake:		RGB_SET(1,0,0)break;//青色
		case write:		RGB_SET(0,0,0)break;//白色	
		default   :		RGB_OFF;
	}

}

