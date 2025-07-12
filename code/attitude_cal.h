#ifndef CODE_ATTITUDE_CAL_H_
#define CODE_ATTITUDE_CAL_H_
#include <stdint.h>
#include <stdlib.h>
#ifndef XYZ_Data
#define XYZ_Data

#define LIMIT( x,min,max )  ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ))

#define WIN_NUM 5




//卡尔曼滤波
#define KALMAN_DT  0.001;

typedef struct
{
  float p;
  float q;//过程噪声协方差
  float r;//测量噪声协方差
  float k;//卡尔曼增益
  float pre_x;//先验预测
  float x;//输出值
} Kalman_node;

extern Kalman_node  kalman_this_time, kalman_last_time;


typedef struct XYZ_Data_f
{
    float x;
    float y;
    float z;
}XYZ_Data_f;

typedef struct XYZ_Data_s32
{
    long x;
    long y;
    long z;
}XYZ_Data_s32;

typedef struct XYZ_Data_s16
{
    short x;
    short y;
    short z;
}XYZ_Data_s16;

typedef struct XYZ_Data_s8
{
    char x;
    char y;
    char z;
}XYZ_Data_s8;
#endif


typedef struct Pose_Flag
{
    uint8_t run;
    uint8_t use_mag;
}Flag;

typedef struct Pose_DInterface
{
    float a_x;
    float a_y;
    float a_z;

    float g_x;
    float g_y;
    float g_z;

    float m_x;
    float m_y;
    float m_z;
}DATA_IN;

typedef struct Pose_Interface
{
    DATA_IN data;
}Pose_Interface;

typedef struct Pose_Data
{
    float yaw;
    float rol;
    float pitch;

    float rotate_matrix[3][3];  //旋转矩阵

    XYZ_Data_f acc_world;               //世界坐标系下的加速度
    XYZ_Data_f mag_world;               //世界坐标系下的磁场强度   --  只与无人机位置有关的量

    XYZ_Data_f acc_correct;         //机体坐标系下的加速度    --  矫正了俯仰翻滚后的加速度
    XYZ_Data_f mag_correct;         //机体坐标系下的磁场强度   --  矫正了俯仰翻滚后的磁场强度
    XYZ_Data_f gyro_correct;        //融合加速度和磁力计数据，矫正后的陀螺仪值
}Pose_Data;

typedef struct Pose_Process
{
    float mag_yaw;                          //磁力计的偏航角
    float mag_yaw_bias;                 //磁力计矫正的偏航角偏差
    float quaternion[4];                //四元数
    XYZ_Data_f error;                       //由加速度计与等效重力的偏差
    XYZ_Data_f error_integral;  //误差积分
}Pose_Process;
////////////////////////////////////////////////////
typedef struct Pose_Parameter
{
    float correct_kp;
    float error_kp;
    float error_ki;
}Pose_Parameter;
////////////////////////////////////////////////////
typedef struct Pose_Module
{
    Flag flag;
    Pose_Interface interface;
    Pose_Process process;
    Pose_Data data;
    Pose_Parameter parameter;
}ATT_Module;


typedef struct
{
    float x;
    float y;
    float z;

} AXIS;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Out_Euler;


extern Out_Euler out_euler;
extern ATT_Module attitude;

//初始化结构体
void init_attitude();
//计算姿态
void calculate_attitude(float cycle);


#define HP_CUT_FRQ 5
#define SAMPLE_RATE  200.0f
#define WIN_NUM 5
void high_pass_filter(float in, float *out);
void LowPassFilter_RC(float Vi, float *Vo);
float window_filter(float data, float *buf, uint8_t len);
void imu_data_process(float ax,float ay,float az,float gx,float gy,float gz,float mx ,float my,float mz);

#endif /* CODE_ATTITUDE_CAL_H_ */
