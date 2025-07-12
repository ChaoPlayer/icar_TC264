
#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"
#include "APP_Filter.h"
#include "MahonyAHRS.h"
#include "EKF.h"
#include "attitude_cal.h"
#include <math.h>
//====================================================�궨����====================================================
#define     my_pow(a)       ((a)*(a))           //ƽ��
#define     halfT           0.005f
//====================================================�궨����====================================================

#define DEGREE_TRANS(degree) degree > 0 ? degree : 360-fabs(degree) //�Ƕ�ת�� ת����0-360

//==================================================�ṹ�嶨����===================================================
//imu���ݽṹ��
typedef struct
{
    //���ٶȼ�����
    float   acc_x;
    float   acc_y;
    float   acc_z;

    //����������
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;

    //�شż�����
    float   mag_x;
    float   mag_y;
    float   mag_z;
    float   Hy;         //���ǳ�����ں��,����������µĵ�Чy����ų�ǿ��
    float   Hx;         //���ǳ�����ں��,����������µĵ�Чx����ų�ǿ��

    //��������
    float   yaw;        //ƫ����
    float   yaw_rate;   //ƫ�����ٶ�
    float   pitch;      //������
    float   pitch_rate; //�������ٶ�
    float   roll;       //�����
    float   roll_rate;  //������ٶ�
    float   azimuth;    //�����
                        //ע������Ǻ�ƫ���ǵ����𡪡��ڵ�������,������ô��⣺
                        //   ƫ���ͺ�����������̬�ĽǶ�,�����Ա�Ϊ��׼heading,ƫ���Ժ��߻��߷���ϵΪ��׼��ƫ��,
                        //   ��ž��ǲο�ϵ��ͬ���������Ǵ��ڲ���,���ǿ��Ի���ת��.

    //�Ƕ�����
    float   CarAngle;   //������ƫ��
    float   AngleSpeed; //�����ٶ�
    float   TurnAngle_Integral; //�ǶȻ���
}IMU_DATA_CLASS;


//��Ư�ṹ��
typedef struct
{
     float X_bias;
     float Y_bias;
     float Z_bias;
}BIAS_CLASS;


//imu��Ҫ�õ��ĺ���,������ָ����Ϊ�ṹ��,����ͳһ����
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


//��Ԫ����̬����ṹ��
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
//==================================================�ṹ�嶨����===================================================

//===================================================����������===================================================
extern float angle_x;
extern float angle_y;


//===================================================����������===================================================


//===================================================����������===================================================
void    imu_init(void);
void Kalman_Filter(float angle_m, float angle_n);
void Get_Attitude(void);
//===================================================����������===================================================



#endif
