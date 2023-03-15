#include "bsp.h"

void BSP_init(void)
{
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	
	delay_init();
//	delay_ms(3000); //�ȴ�ϵͳ�ȶ�������������ȫ����
	
	//LED��ʼ��
	init_LED_GPIO();
	
	//key��ʼ��
	Key_GPIO_Init();
	
	
#if RGB_SWITCH	
	//RGB��ʼ��
	init_RGB_GPIO();
#endif
	
	
#if DEBUG_SWITCH
	//usart1��ʼ��
	USART1_init(115200);
#endif

#if LCD_SWITCH
	//lcd��ʼ��
	LCD_Init();//LCD��ʼ��
#endif

	
#if FLASH_SWITCH
	SpiFlashInit(); //w25q64��ʼ��
	show_flash();
#endif

#if IMU_SWITCH
	//�����ǳ�ʼ��
	icm20607_init();
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE); //����
#endif
	
//������ʱ����ʼ��

	
	//�ŵ�������Ч����Ȼ�����޷�����ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//����jlink ֻ��SWD���Կڣ�PA15��PB3��4����ͨIO
	
	
}
