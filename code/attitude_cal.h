#ifndef CODE_ATTITUDE_CAL_H_
#define CODE_ATTITUDE_CAL_H_
#include <stdint.h>
#include <stdlib.h>
#ifndef XYZ_Data
#define XYZ_Data

#define LIMIT( x,min,max )  ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ))

#define WIN_NUM 5




//�������˲�
#define KALMAN_DT  0.001;

typedef struct
{
  float p;
  float q;//��������Э����
  float r;//��������Э����
  float k;//����������
  float pre_x;//����Ԥ��
  float x;//���ֵ
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

    float rotate_matrix[3][3];  //��ת����

    XYZ_Data_f acc_world;               //��������ϵ�µļ��ٶ�
    XYZ_Data_f mag_world;               //��������ϵ�µĴų�ǿ��   --  ֻ�����˻�λ���йص���

    XYZ_Data_f acc_correct;         //��������ϵ�µļ��ٶ�    --  �����˸���������ļ��ٶ�
    XYZ_Data_f mag_correct;         //��������ϵ�µĴų�ǿ��   --  �����˸���������Ĵų�ǿ��
    XYZ_Data_f gyro_correct;        //�ںϼ��ٶȺʹ��������ݣ��������������ֵ
}Pose_Data;

typedef struct Pose_Process
{
    float mag_yaw;                          //�����Ƶ�ƫ����
    float mag_yaw_bias;                 //�����ƽ�����ƫ����ƫ��
    float quaternion[4];                //��Ԫ��
    XYZ_Data_f error;                       //�ɼ��ٶȼ����Ч������ƫ��
    XYZ_Data_f error_integral;  //������
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

//��ʼ���ṹ��
void init_attitude();
//������̬
void calculate_attitude(float cycle);


#define HP_CUT_FRQ 5
#define SAMPLE_RATE  200.0f
#define WIN_NUM 5
void high_pass_filter(float in, float *out);
void LowPassFilter_RC(float Vi, float *Vo);
float window_filter(float data, float *buf, uint8_t len);
void imu_data_process(float ax,float ay,float az,float gx,float gy,float gz,float mx ,float my,float mz);

#endif /* CODE_ATTITUDE_CAL_H_ */
