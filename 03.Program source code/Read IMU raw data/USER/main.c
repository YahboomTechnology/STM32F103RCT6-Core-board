#include "AllHeader.h"
#include "pic.h"//ͼƬ


int main(void)
{	
	u8 i;
	//Ӳ����ʼ��
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


