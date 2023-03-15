#include "AllHeader.h"


/**
* Function       // imu_test
* @author       // yahboom--
* @date         //  2023.02.22   
* @brief         // 检测IMU的数据
* @param[in]     // 无
* @param[out]    // none
* @retval        // NONE
* @par History   // v0.0
*/
void imu_test(void)
{
//	int16_t val_gx = 0;
//	int16_t val_gy = 0;
//	int16_t val_gz = 0;

//	int16_t val_ax = 0;
//	int16_t val_ay = 0;
//	int16_t val_az = 0;
//	
//	//陀螺仪测试
		get_icm_attitude();
//		
//			
//		val_gx = getRawGyroscopeX();
//		val_gy = getRawGyroscopeY();
//		val_gz = getRawGyroscopeZ();
//		
//		
//		//加速度
//		val_ax = getRawAccelerationX();
//		val_ay = getRawAccelerationY();
//		val_az = getRawAccelerationZ();
//		
//		
		#if DEBUG_SWITCH
//			printf("gx=%d, gy=%d, gz=%d \r\n", val_gx, val_gy, val_gz);
			//printf("ax=%d, ay=%d, az=%d \r\n", val_ax, val_ay, val_az);
			//printf("accX = %d,accY=%d,accZ =%d\r\n",g_icm20607.accX,g_icm20607.accY,g_icm20607.accZ);
		#endif


}


