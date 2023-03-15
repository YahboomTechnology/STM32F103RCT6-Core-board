#ifndef __BSP_RGB_H
#define __BSP_RGB_H

#include "AllHeader.h"

#define RGB_PORT      GPIOC
#define RGB_CLK				RCC_APB2Periph_GPIOC
#define RGB_LED_R_PIN GPIO_Pin_0
#define RGB_LED_G_PIN GPIO_Pin_1
#define RGB_LED_B_PIN GPIO_Pin_2

#define LED PBout(4)


#define RGB_R PCout(0)
#define RGB_G PCout(1)
#define RGB_B PCout(2)

#define RGB_SET(R,G,B) RGB_R=R; RGB_G=G; RGB_B=B;

#define RGB_ON 	RGB_SET(0,0,0);
#define RGB_OFF RGB_SET(1,1,1);

enum color_RGB
{
	red = 0,
	green,
	blue,
	yellow,
	purper,
	lake,
	write,
	color_max
};

void init_RGB_GPIO(void);
void init_LED_GPIO(void);
void RGB_control(u8 color);

#endif
