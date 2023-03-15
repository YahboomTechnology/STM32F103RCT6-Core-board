#include "bsp_icm20607.h"
#include "app_math.h"




attitude_t g_attitude;
icm_data_t g_icm20607;

u8 IMU_flag_addr = 0;
unsigned char ICM_ADDRESS = ICM_ADDRESS_0; //IMU——i2C设备地址


// 微分时间，一般取10毫秒，与调用的时间间隔有关
#define DT                  (0.01)
// 在此时间前，陀螺仪数据用来检测。单位为：10毫秒。默认值：500
#define ICM_SKIP            500
// 在上面的基础上，前几个数据不稳定直接抛弃掉。单位为：10毫秒。默认值：20
#define ICM_ABANDON         20


#define OLD_YAW_WEIGHT       0.65f
#define NEW_YAW_WEIGHT       0.35f


static quaternion_t NumQ = {1, 0, 0, 0};
float vecxZ, vecyZ, veczZ;
float wz_acc_tmp[2];


int16_t icm_gyro_x, icm_gyro_y, icm_gyro_z;
int16_t icm_acc_x, icm_acc_y, icm_acc_z;

float g_icm_test_start_yaw = 0;
u16 g_icm_test = 0;


/* I2C写数据 */
static int icm_i2c_write(uint8_t reg, uint8_t data)
{
    return IICwriteByte(ICM_ADDRESS, reg, data);
    //I2C_ICM_ByteWrite(reg, data);
}

/* I2C读数据 */
static int icm_i2c_read(uint8_t reg, uint8_t *data_buf, uint16_t length)
{
    return IICreadBytes(ICM_ADDRESS, reg, length, data_buf);
    //I2C_ICM_BufferRead(reg, data_buf, length);
}

static void icm_write_gyro_offset(int16_t* data_buf)
{
    s16 buf[3] = {0};
    int abc = 0;
    for (int i = 0; i < 3; i++)
    {
        buf[i] = *data_buf + i;
        abc += icm_i2c_write(XG_OFFS_USRH + i*2, buf[i]&0xff);
        abc += icm_i2c_write(XG_OFFS_USRH + i*2 + 1, (buf[i]<<8)&0xff);
    }
    //DEBUG("ABC=%d", abc);
}

/* 读取陀螺仪X轴原始数据 */
int16_t getRawGyroscopeX(void)
{
    uint8_t val[2] = {0};
    icm_i2c_read(GYRO_XOUT_H, val, 2);
    return ((int16_t)val[0] << 8) + val[1];
}

/* 读取陀螺仪Y轴原始数据 */
int16_t getRawGyroscopeY(void)
{
    uint8_t val[2] = {0};
    icm_i2c_read(GYRO_YOUT_H, val, 2);
    return ((int16_t)val[0] << 8) + val[1];
}

/* 读取陀螺仪Z轴原始数据 */
int16_t getRawGyroscopeZ(void)
{
    uint8_t val[2] = {0};
    icm_i2c_read(GYRO_ZOUT_H, val, 2);
    return ((int16_t)val[0] << 8) + val[1];
}

/* 读取加速度计X轴原始数据 */
int16_t getRawAccelerationX(void)
{
    uint8_t val[2] = {0};
    icm_i2c_read(ACCEL_XOUT_H, val, 2);
    return ((int16_t)val[0] << 8) + val[1];
}

/* 读取加速度计Y轴原始数据 */
int16_t getRawAccelerationY(void)
{
    uint8_t val[2] = {0};
    icm_i2c_read(ACCEL_YOUT_H, val, 2);
    return ((int16_t)val[0] << 8) + val[1];
}

/* 读取加速度计Z轴原始数据 */
int16_t getRawAccelerationZ(void)
{
    uint8_t val[2] = {0};
    icm_i2c_read(ACCEL_ZOUT_H, val, 2);//  ACCEL_ZOUT_H
    return ((int16_t)val[0] << 8) + val[1];
}

/* 判断是否是icm20607芯片 */
void icm20607_who_am_i(void)
{
    uint8_t val = 0, state = 1;
    do
    {
        icm_i2c_read(WHO_AM_I, &val, 1); // 读ICM20607的ID


			
        if ((ICM20607_ID != val) & state) // 当ID不对时，只报一次。
        {
					LCD_ShowString(10,0,"IMU is Detecting!",BLACK,WHITE,16,0);
					LCD_ShowString(10,60,"10s reset to try!",BLACK,WHITE,16,0);
            //DEBUG("IMU ID error! WHO_AM_I=0x%02x\n", val);
            //DEBUG("Please press the reset key to reboot\n");
            state = 0;	
        }
				if (ICM20607_ID != val)
				{
						IMU_flag_addr++;
						if(IMU_flag_addr >8) 
					{
						IMU_flag_addr = 0;
					}
					switch(IMU_flag_addr)
					{
						case 0:ICM_ADDRESS = ICM_ADDRESS_0;break;
						case 1:ICM_ADDRESS = ICM_ADDRESS_1;break;
						case 2:ICM_ADDRESS = ICM_ADDRESS_2;break;
						case 3:ICM_ADDRESS = ICM_ADDRESS_3;break;
						case 4:ICM_ADDRESS = ICM_ADDRESS_4;break;
						case 5:ICM_ADDRESS = ICM_ADDRESS_4;break;
						case 6:ICM_ADDRESS = ICM_ADDRESS_6;break;
						case 7:ICM_ADDRESS = ICM_ADDRESS_7;break;
						case 8:ICM_ADDRESS = ICM_ADDRESS_8;break;
						default :ICM_ADDRESS = ICM_ADDRESS_0;
					
					}
				}
				
				
					
    } while (ICM20607_ID != val);
    //DEBUG("WHO_AM_I=0x%02x\n", val);
}

/*
 * ICM20602_GYRO_CONFIG寄存器
 * 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
 * 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
 * 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
 * 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

 * ICM20602_ACCEL_CONFIG寄存器
 * 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
 * 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
 * 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
 * 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
*/
/* 初始化icm20607芯片 */
void icm20607_init(void)
{
    uint8_t val = 0x0;
    IIC_Init(); // 初始化
    delay_ms(10);

    icm_i2c_write(PWR_MGMT_1, 0x80); //复位设备
    delay_ms(100);
    icm20607_who_am_i();
		
    do
    { //等待复位成功
        icm_i2c_read(PWR_MGMT_1, &val, 1);
    } while(0x41 != val);
    icm_i2c_write(PWR_MGMT_1, 0x01);     //时钟设置
    icm_i2c_write(PWR_MGMT_2, 0x00);     //开启陀螺仪和加速度计

    icm_i2c_write(CONFIG, 0x01);         //176HZ 1KHZ  0x01
    icm_i2c_write(SMPLRT_DIV, 0x07);     //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)  0X07
    
     icm_i2c_write(GYRO_CONFIG, 0x18);    //±2000 dps
//    icm_i2c_write(GYRO_CONFIG, 0x08);    //±500 dps

     icm_i2c_write(ACCEL_CONFIG, 0x10);   //±8g

//    icm_i2c_write(ACCEL_CONFIG, 0x00);     // ±2
		   icm_i2c_write(ACCEL_CONFIG_2, 0x23);
		
}

/* 读取陀螺仪的原始数据 */
void icm_get_gyro(void)
{
//    uint8_t dat[6];

//    icm_i2c_read(GYRO_XOUT_H, dat, 6);
//    icm_gyro_x = (int16_t)(((uint16_t)dat[0] << 8 | dat[1]));
//    icm_gyro_y = (int16_t)(((uint16_t)dat[2] << 8 | dat[3]));
//    icm_gyro_z = (int16_t)(((uint16_t)dat[4] << 8 | dat[5]));
			icm_gyro_x =	getRawGyroscopeX();
			icm_gyro_y = getRawGyroscopeY();
			icm_gyro_z = getRawGyroscopeZ();
		
}

/* 读取加速度计的原始数据 */
void icm_get_acc(void)
{
//    uint8_t dat[6];

//    icm_i2c_read(ACCEL_XOUT_H, dat, 6);
//    icm_acc_x = (int16_t)(((uint16_t)dat[0] << 8 | dat[1]));
//    icm_acc_y = (int16_t)(((uint16_t)dat[2] << 8 | dat[3]));
//    icm_acc_z = (int16_t)(((uint16_t)dat[4] << 8 | dat[5]));
	
		icm_acc_x = getRawAccelerationX();
		icm_acc_y = getRawAccelerationY();
		icm_acc_z = getRawAccelerationZ();
		
}

/* 读取ICM姿态角 */
void icm_update_data(void)
{
    icm_get_gyro();
    icm_get_acc();
}

int16_t icm_check_time = 0;
int16_t Deviation_gyro[3] = {0}, Original_gyro[3] = {0};    //陀螺仪静差 和原始数据
int16_t Deviation_acc[3] = {0};    // 加速度计静差


// 获取陀螺仪姿态角
void get_icm_attitude(void)
{
	char bnb[20];
    if (icm_check_time < ICM_SKIP)
    {
        icm_check_time++;
        icm_update_data();
        #if ENABLE_DEBUG_YAW
        DEBUG("icm_check:%d, x:%d, y:%d, z:%d\n", icm_check_time, icm_gyro_x, icm_gyro_y, icm_gyro_z);
        #endif
        if (icm_check_time <= ICM_ABANDON) return;
        Deviation_gyro[0] += icm_gyro_x;
        Deviation_gyro[1] += icm_gyro_y;
        Deviation_gyro[2] += icm_gyro_z;
        Deviation_acc[0] += icm_acc_x;
        Deviation_acc[1] += icm_acc_y;
        Deviation_acc[2] += icm_acc_z;

        if (icm_check_time >= ICM_SKIP)
        {
            // 求平均值
            Deviation_gyro[0] = Deviation_gyro[0] / (ICM_SKIP - ICM_ABANDON);
            Deviation_gyro[1] = Deviation_gyro[1] / (ICM_SKIP - ICM_ABANDON);
            Deviation_gyro[2] = Deviation_gyro[2] / (ICM_SKIP - ICM_ABANDON);
            Deviation_acc[0] = Deviation_acc[0] / (ICM_SKIP - ICM_ABANDON);
            Deviation_acc[1] = Deviation_acc[1] / (ICM_SKIP - ICM_ABANDON);
            Deviation_acc[2] = Deviation_acc[2] / (ICM_SKIP - ICM_ABANDON);
            //DEBUG("deviation gyro:x:%d, y:%d, z:%d\n", Deviation_gyro[0], Deviation_gyro[1], Deviation_gyro[2]);
            //DEBUG("deviation acc:x:%d, y:%d, z:%d\n", Deviation_acc[0], Deviation_acc[1], Deviation_acc[2]);
            //Beep_On_Time(30);
            ////icm_check_calibrate();
        }
    }
    else
    {
        icm_update_data();

        g_icm20607.accX = icm_acc_x - Deviation_acc[0];
        g_icm20607.accY = icm_acc_y - Deviation_acc[1];
        g_icm20607.accZ = icm_acc_z - Deviation_acc[2];
        // g_icm20607.accX = icm_acc_x;
        // g_icm20607.accY = icm_acc_y;
        // g_icm20607.accZ = icm_acc_z;
        g_icm20607.gyroX = icm_gyro_x - Deviation_gyro[0];
        g_icm20607.gyroY = icm_gyro_y - Deviation_gyro[1];
        g_icm20607.gyroZ = icm_gyro_z - Deviation_gyro[2];
			
				g_icm20607.gyroX = g_icm20607.gyroX /16.4 + 8; 
        g_icm20607.gyroY = g_icm20607.gyroY /16.4 + 8;
        g_icm20607.gyroZ = g_icm20607.gyroZ /16.4 + 8; //8:是每个IMU的误差，需要根据实际情况调
			
				g_icm20607.accX = g_icm20607.accX/16.4;
				g_icm20607.accY = g_icm20607.accY/16.4;
				g_icm20607.accZ = g_icm20607.accZ/16.4 -66; 

        //get_attitude_angle(&g_icm20607, &g_attitude, DT); // 四元素算法
//			if (g_icm_test > 0) //不进去
//        {
//            if (g_icm_test == 1)
//            {
//                icm_test_stop();
//            }
//            g_icm_test--;
//        }

//				//显示陀螺仪数据
//				sprintf(bnb,"gyroX =%d  ",g_icm20607.gyroX);
//        LCD_ShowString(10,15,(u8*)bnb,BLACK,WHITE,16,0);
//				
//				sprintf(bnb,"gyroY =%d  ",g_icm20607.gyroY);
//        LCD_ShowString(10,30,(u8*)bnb,BLACK,WHITE,16,0);
//				
//				sprintf(bnb,"gyroZ =%d  ",g_icm20607.gyroZ);
//        LCD_ShowString(10,45,(u8*)bnb,BLACK,WHITE,16,0);
//				
				//显示加速度计数据
				sprintf(bnb,"accX =%d  ",g_icm20607.accX);
        LCD_ShowString(10,15,(u8*)bnb,BLACK,WHITE,16,0);
				
				sprintf(bnb,"accY =%d  ",g_icm20607.accY);
        LCD_ShowString(10,30,(u8*)bnb,BLACK,WHITE,16,0);
				
				sprintf(bnb,"accZ =%d  ",g_icm20607.accZ);
        LCD_ShowString(10,45,(u8*)bnb,BLACK,WHITE,16,0);
//				


				memset(bnb,0,sizeof(bnb));
				delay_ms(100);
    }
}


void icm_test_start(void)
{
    g_icm_test = 500;
}


void icm_test_stop(void)
{
//    int offset = PID_Get_Offset();
//    Motion_Stop();
//    if (offset > -3 && offset < 3)
//    {
//        //Beep_On_Time(50);
//    }
//    //DEBUG("icm_test:%d\n", offset);
}




void icm_check_calibrate(void)
{
    // LED_ON();
    // Deviation_gyro[0];
    u8 dat[6] = {0};
    s16 offset_gyro[3] = {0};
    icm_i2c_read(XG_OFFS_USRH, dat, 6);
    offset_gyro[0] = (int16_t)(((uint16_t)dat[0] << 8 | dat[1]));
    offset_gyro[1] = (int16_t)(((uint16_t)dat[2] << 8 | dat[3]));
    offset_gyro[2] = (int16_t)(((uint16_t)dat[4] << 8 | dat[5]));
    //DEBUG("offset read gyro:%d, %d, %d\n", offset_gyro[0], offset_gyro[1], offset_gyro[2]);

    for (int i = 0; i < 3; i++)
    {
        offset_gyro[i] -= Deviation_gyro[i];
    }
    //DEBUG("offset write gyro:%d, %d, %d\n", offset_gyro[0], offset_gyro[1], offset_gyro[2]);
    icm_write_gyro_offset(offset_gyro);
}


/* 四元素获取  dt：10MS */
void get_attitude_angle(icm_data_t *p_icm, attitude_t *p_angle, float dt)
{
    vector_t Gravity, Acc, Gyro, AccGravity;
    static vector_t GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // 加速度归一化，
    NormQuat = q_rsqrt(squa(p_icm->accX)+ squa(p_icm->accY) +squa(p_icm->accZ)); 

    //归一后可化为单位向量下方向分量
    Acc.x = p_icm->accX * NormQuat;
    Acc.y = p_icm->accY * NormQuat;
    Acc.z = p_icm->accZ * NormQuat;

    //向量叉乘得出的值，叉乘后可以得到旋转矩阵的重力分量在新的加速度分量上的偏差
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    //角速度融合加速度比例补偿值，与上面三句共同形成了PI补偿，得到矫正后的角速度值
    Gyro.x = p_icm->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; //弧度制，此处补偿的是角速度的漂移
    Gyro.y = p_icm->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = p_icm->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
    // 一阶龙格库塔法, 更新四元数
    //矫正后的角速度值积分，得到两次姿态解算中四元数一个实部Q0，三个虚部Q1~3的值的变化
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    //积分后的值累加到上次的四元数中，即新的四元数
    NumQ.q0 += q0_t; 
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 重新四元数归一化，得到单位向量下
    NormQuat = q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //得到四元数的模长
    NumQ.q0 *= NormQuat;                                                               //模长更新四元数值
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /* 计算姿态角 */
    // get_angle(p_angle);
    vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3; /*矩阵(3,1)项*/                                 //地理坐标系下的X轴的重力分量
    vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1; /*矩阵(3,2)项*/                                 //地理坐标系下的Y轴的重力分量
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3; /*矩阵(3,3)项*/ //地理坐标系下的Z轴的重力分量

    // p_angle->yaw = atan2f((2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3), 
    //     (1 - 2 * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3))) * RtA; //偏航角
    /* 弧度转成角度 */
    // p_angle->yaw = atan2(2 * (NumQ.q1 * NumQ.q2 + NumQ.q0 * NumQ.q3), 
    //     NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 - NumQ.q3 * NumQ.q3) * RtA; //yaw
    // p_angle->pitch = asin(vecxZ) * RtA;             //俯仰角
    // p_angle->roll = atan2f(vecyZ, veczZ) * RtA;     //横滚角
    
    #if ENABLE_ROLL_PITCH
    p_angle->pitch = asin(vecxZ);             //俯仰角
    p_angle->roll = atan2f(vecyZ, veczZ);     //横滚角
    #endif

	
	
    p_angle->yaw = atan2(2 * (NumQ.q1 * NumQ.q2 + NumQ.q0 * NumQ.q3), 
        NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 - NumQ.q3 * NumQ.q3); //偏航角


    static float old_yaw = 0;
    p_angle->yaw = OLD_YAW_WEIGHT * old_yaw + NEW_YAW_WEIGHT * p_angle->yaw;
    old_yaw = p_angle->yaw;
}
