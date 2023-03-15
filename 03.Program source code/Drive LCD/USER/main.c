#include "AllHeader.h"
#include "pic.h"//图片


int main(void)
{	
	u8 i;
	//硬件初始化
	BSP_init();
	
	#if LCD_SWITCH
		LCD_Fill(0,0,LCD_W,LCD_H,WHITE);	
		LCD_ShowPicture(20,45,120,29,gImage_pic1);
		LCD_ShowString(10,0,"Hello World!",BLACK,WHITE,16,0);
		LCD_ShowChinese(50,20,"亚博智能",BLUE,WHITE,16,0);	
	#endif
	
	while(1)
	{
		if(Key1_State(0))
			LED =!LED;
#if IMU_SWITCH
			imu_test();		
#endif

		i++;if(i>=color_max) i =red;
		RGB_control(i);
		delay_ms(3000);

		
	}
}


