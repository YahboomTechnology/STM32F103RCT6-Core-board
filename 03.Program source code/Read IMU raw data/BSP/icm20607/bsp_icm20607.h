#ifndef __BSP_ICM20607_H__
#define __BSP_ICM20607_H__


#include "AllHeader.h"

/* 向量 */
typedef struct _vector_t
{
    float x;
    float y;
    float z;
} vector_t;

/* 姿态角 */
typedef struct _attitude_t
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct _quaternion_t
{ //四元数
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;

typedef struct _icm_data_t
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;

    int16_t Offset[6];
} icm_data_t;

extern attitude_t g_attitude;
extern icm_data_t g_icm20607;

#define     ICM_ADDRESS_0        0x68<<1  //AD0:0即0x68<<1  1即0x69<<1
#define     ICM_ADDRESS_1        0x69  
#define     ICM_ADDRESS_2        0x68  
#define     ICM_ADDRESS_3        0xE8  
#define     ICM_ADDRESS_4        0xE9  
#define     ICM_ADDRESS_5        0x69<<1  
#define     ICM_ADDRESS_6        0xD2  
#define     ICM_ADDRESS_7        0xD1  
#define     ICM_ADDRESS_8        0xD0 


#define     ICM20607_ID        0x05
#define     XG_OFFS_TC_H       0x04
#define     XG_OFFS_TC_L       0x05
#define     YG_OFFS_TC_H       0x07
#define     YG_OFFS_TC_L       0x08
#define     ZG_OFFS_TC_H       0x0A
#define     ZG_OFFS_TC_L       0x0B
#define     SELF_TEST_X_ACCEL  0x0D
#define     SELF_TEST_Y_ACCEL  0x0E
#define     SELF_TEST_Z_ACCEL  0x0F
#define     XG_OFFS_USRH       0x13
#define     XG_OFFS_USRL       0x14
#define     YG_OFFS_USRH       0x15
#define     YG_OFFS_USRL       0x16
#define     ZG_OFFS_USRH       0x17
#define     ZG_OFFS_USRL       0x18
#define     SMPLRT_DIV         0x19
#define     CONFIG             0x1A
#define     GYRO_CONFIG        0x1B
#define     ACCEL_CONFIG       0x1C
#define     ACCEL_CONFIG_2     0x1D
#define     LP_MODE_CFG        0x1E
#define     ACCEL_WOM_X_THR    0x20
#define     ACCEL_WOM_Y_THR    0x21
#define     ACCEL_WOM_Z_THR    0x22
#define     FIFO_EN            0x23
#define     FSYNC_INT          0x36
#define     INT_PIN_CFG        0x37
#define     INT_ENABLE         0x38
#define     FIFO_WM_INT_STATUS 0x39 
#define     INT_STATUS         0x3A
#define     ACCEL_XOUT_H       0x3B
#define     ACCEL_XOUT_L       0x3C
#define     ACCEL_YOUT_H       0x3D
#define     ACCEL_YOUT_L       0x3E
#define     ACCEL_ZOUT_H       0x3F
#define     ACCEL_ZOUT_L       0x40
#define     TEMP_OUT_H         0x41
#define     TEMP_OUT_L         0x42
#define     GYRO_XOUT_H        0x43
#define     GYRO_XOUT_L        0x44
#define     GYRO_YOUT_H        0x45
#define     GYRO_YOUT_L        0x46
#define     GYRO_ZOUT_H        0x47
#define     GYRO_ZOUT_L        0x48
#define     SELF_TEST_X_GYRO   0x50
#define     SELF_TEST_Y_GYRO   0x51
#define     SELF_TEST_Z_GYRO   0x52
#define     FIFO_WM_TH1        0x60
#define     FIFO_WM_TH2        0x61
#define     SIGNAL_PATH_RESET  0x68
#define     ACCEL_INTEL_CTRL   0x69
#define     USER_CTRL          0x6A
#define     PWR_MGMT_1         0x6B
#define     PWR_MGMT_2         0x6C
#define     I2C_IF             0x70
#define     FIFO_COUNTH        0x72
#define     FIFO_COUNTL        0x73
#define     FIFO_R_W           0x74
#define     WHO_AM_I           0x75
#define     XA_OFFSET_H        0x77
#define     XA_OFFSET_L        0x78
#define     YA_OFFSET_H        0x7A
#define     YA_OFFSET_L        0x7B
#define     ZA_OFFSET_H        0x7D
#define     ZA_OFFSET_L        0x7E



extern int16_t icm_gyro_x, icm_gyro_y, icm_gyro_z;
extern int16_t icm_acc_x, icm_acc_y, icm_acc_z;


int16_t getRawGyroscopeX(void);
int16_t getRawGyroscopeY(void);
int16_t getRawGyroscopeZ(void);

int16_t getRawAccelerationX(void);
int16_t getRawAccelerationY(void);
int16_t getRawAccelerationZ(void);


void icm20607_init(void);
void icm20607_who_am_i(void);
void icm_get_gyro(void);
void icm_get_acc(void);

void icm_update_data(void);

void get_icm_attitude(void);

void icm_test_start(void);
void icm_test_stop(void);


void icm_check_calibrate(void);
void get_attitude_angle(icm_data_t *p_icm, attitude_t *p_angle, float dt);

#endif /* __BSP_ICM20607_H__ */
