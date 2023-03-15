#include "AllHeader.h"
#include "pic.h"//Í¼Æ¬


int main(void)
{	
	u8 i;
	//Ó²¼þ³õÊ¼»¯
	BSP_init();
	
	
	while(1)
	{
		if(Key1_State(0))
			LED =!LED;
#if IMU_SWITCH
			imu_test();		
#endif

		
	}
}


