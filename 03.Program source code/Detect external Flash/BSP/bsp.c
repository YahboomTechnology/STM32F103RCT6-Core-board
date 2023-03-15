#include "bsp.h"

void BSP_init(void)
{
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	
	delay_init();
//	delay_ms(3000); //等待系统稳定，陀螺仪正完全开机
	
	//LED初始化
	init_LED_GPIO();
	
	//key初始化
	Key_GPIO_Init();
	
	
#if RGB_SWITCH	
	//RGB初始化
	init_RGB_GPIO();
#endif
	
	
#if DEBUG_SWITCH
	//usart1初始化
	USART1_init(115200);
#endif

#if LCD_SWITCH
	//lcd初始化
	LCD_Init();//LCD初始化
#endif

	
#if FLASH_SWITCH
	SpiFlashInit(); //w25q64初始化
	show_flash();
#endif

#if IMU_SWITCH
	//陀螺仪初始化
	icm20607_init();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE); //清屏
#endif
	
//基本定时器初始化

	
	//放到最后才生效，不然还是无法正常使用
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//禁用jlink 只用SWD调试口，PA15、PB3、4做普通IO
	
	
}
