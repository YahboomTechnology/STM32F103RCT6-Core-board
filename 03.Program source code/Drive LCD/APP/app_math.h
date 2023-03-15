#ifndef __APP_MATH_H__
#define __APP_MATH_H__

//外部文件引用
#include "math.h"

//宏定义区
#define PI 3.1415926f
#define squa(Sq) (((float)Sq) * ((float)Sq))
#define absu16(Math_X) ((Math_X) < 0 ? -(Math_X) : (Math_X))
#define absFloat(Math_X) ((Math_X) < 0 ? -(Math_X) : (Math_X))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

//数据结构声明

//Extern引用
extern const float MY_PI;
extern const float AtR;
extern const float RtA;
extern const float Gyro_G;
extern const float Gyro_Gr;

//函数声明
extern float safe_asin(float v);
extern float arcsin(float x);
extern float arctan(float x);
extern float sine(float x);
extern float cosine(float x);
extern float q_rsqrt(float number);
extern float VariableParameter(float error);

int math_abs(int a);

#endif /* __APP_MATH_H__ */
