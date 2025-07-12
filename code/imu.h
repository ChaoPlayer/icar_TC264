
#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"
#include "APP_Filter.h"
#include "MahonyAHRS.h"
#include "EKF.h"
#include "attitude_cal.h"
#include <math.h>
//====================================================宏定义区====================================================
#define     my_pow(a)       ((a)*(a))           //平方
#define     halfT           0.005f
//====================================================宏定义区====================================================

#define DEGREE_TRANS(degree) degree > 0 ? degree : 360-fabs(degree) //角度转换 转换成0-360

//==================================================结构体定义区===================================================
//imu数据结构体
typedef struct
{
    //加速度计数据
    float   acc_x;
    float   acc_y;
    float   acc_z;

    //陀螺仪数据
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;

    //地磁计数据
    float   mag_x;
    float   mag_y;
    float   mag_z;
    float   Hy;         //考虑车身存在横滚,俯仰等情况下的等效y方向磁场强度
    float   Hx;         //考虑车身存在横滚,俯仰等情况下的等效x方向磁场强度

    //导航数据
    float   yaw;        //偏航角
    float   yaw_rate;   //偏航角速度
    float   pitch;      //俯仰角
    float   pitch_rate; //俯仰角速度
    float   roll;       //横滚角
    float   roll_rate;  //横滚角速度
    float   azimuth;    //航向角
                        //注：航向角和偏航角的区别——在导航领域,可以这么理解：
                        //   偏航和航向都是描述姿态的角度,航向以北为基准heading,偏航以航线或者发射系为基准的偏角,
                        //   大概就是参考系不同而导致他们存在差异,但是可以互相转换.

    //角度数据
    float   CarAngle;   //车辆的偏角
    float   AngleSpeed; //车角速度
    float   TurnAngle_Integral; //角度积分
}IMU_DATA_CLASS;


//零漂结构体
typedef struct
{
     float X_bias;
     float Y_bias;
     float Z_bias;
}BIAS_CLASS;


//imu需要用到的函数,将函数指针作为结构体,方便统一管理
typedef struct
{
    uint8   (*icm20602_init)(void);
    void    (*icm20602_get_acc)(void);
    void    (*icm20602_get_gyro)(void);
    uint8   (*imu963ra_init)(void);
    void    (*imu963ra_get_acc)(void);
    void    (*imu963ra_get_gyro)(void);
    void    (*imu963ra_get_mag)(void);
    void    (*Get_Attitude)(void);
    void    (*BIAS_init)(void);
} IMU_FUNC_CLASS;


typedef struct
{
    float x;
    float y;
    float z;
} _xyz_f_st;


//四元数姿态结算结构体
typedef struct
{
    float w;//q0;
    float x;//q1;
    float y;//q2;
    float z;//q3;

    _xyz_f_st x_vec;
    _xyz_f_st y_vec;
    _xyz_f_st z_vec;

    _xyz_f_st a_acc;
    _xyz_f_st w_acc;

    float rol;
    float pit;
    float yaw;
    float inter_rol;
    float inter_pit;
    float inter_yaw;
} Quater_st;


//
typedef struct
{
    float lpf_1;
    float out;
}_lf_t;

extern  IMU_DATA_CLASS      imudata;
extern  BIAS_CLASS          AccBias, GyroBias,MagBias;
extern  IMU_FUNC_CLASS      imufunc;
extern  Quater_st           quater;
//==================================================结构体定义区===================================================

//===================================================变量声明区===================================================
extern float angle_x;
extern float angle_y;


//===================================================变量声明区===================================================


//===================================================函数声明区===================================================
void    imu_init(void);
void Kalman_Filter(float angle_m, float angle_n);
void Get_Attitude(void);
//===================================================函数声明区===================================================



#endif
