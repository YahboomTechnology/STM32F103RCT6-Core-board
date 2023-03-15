#include "AllHeader.h"
#include "pic.h"//Í¼Æ¬


int main(void)
{	
	u8 i;
	//Ó²¼þ³õÊ¼»¯
	BSP_init();
	
	
	while(1)
	{
#if IMU_SWITCH
			imu_test();		
#endif
		
		LED =!LED;
//		i++;if(i>=color_max) i =red;
//		RGB_control(i);
		delay_ms(300);

		
	}
}


