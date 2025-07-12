#include "attitude_cal.h"
#include "imu.h"
#include <math.h>
ATT_Module attitude;
#define ANGLE_MAX       180.0f
#define ANGLE_MIN       -180.0f
//#define PI                  (3.14159265f)
#define ABS(x)                          ((x)>0?(x):-(x))
#define DIV(Number, Prescaler, Threshold) ((Prescaler == 0.0f)? Threshold: (Number/Prescaler))
#define Power2(x)                   (x * x)
#define MAX_2(x,y)                  ((x >= y)? x: y)
#define MAX_3(x,y,z)                ((z >= MAX_2(x,y))? z: MAX_2(x,y))


// 互补滤波参数
#define ALPHA 0.98 // 加速度计和磁力计权重
#define BETA 0.02 // 陀螺仪权重
#define RAD_TO_DEG 57.2957795131f
#define DT  0.001f // 1ms
Out_Euler out_euler;

Out_Euler complementaryFilter(AXIS gyr, AXIS acc)
{
    Out_Euler euler;

    static float roll = 0.0f;
    static float pitch = 0.0f;
    static float yaw = 0.0f;

    // 加速度计测量值转换为欧拉角（俯仰角和横滚角）
    float accelRoll = atan2(acc.y, acc.z) * RAD_TO_DEG;
    float accelPitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
//  float accelYaw = 0.0;  // 加速度计无法直接测量偏航角

    // 陀螺仪积分计算欧拉角变化率
    float gyroRollRate = gyr.x;
    float gyroPitchRate = gyr.y;
    float gyroYawRate = gyr.z;
    // 使用互补滤波器融合传感器数据
    roll = ALPHA * (roll + gyroRollRate * DT) + (1 - ALPHA) * accelRoll;
    pitch = ALPHA * (pitch + gyroPitchRate * DT) + (1 - ALPHA) * accelPitch;

    euler.pitch  = pitch;
    euler.roll = roll;

    return euler;
}


float window_ax[WIN_NUM];
float window_ay[WIN_NUM];
float window_az[WIN_NUM];

float window_gx[WIN_NUM];
float window_gy[WIN_NUM];
float window_gz[WIN_NUM];


float window_mx[WIN_NUM];
float window_my[WIN_NUM];
float window_mz[WIN_NUM];

float translateAngle(float angle)
{
    while (angle > ANGLE_MAX)                   //判断目标角度是否在允许角度范围
        angle -= 360.0f;
    while (angle < ANGLE_MIN)
        angle += 360.0f;
    return angle;
}

float fast_sqrt(float number)
{
    long i;
    float x, y;
    const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * ( long * ) &y;
    i = 0x5f3759df - ( i >> 1 );

    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );
    y = y * ( f - ( x * y * y ) );
    return number * y;
}

float my_sin(float angle)
{
    float sine, rad;
    angle = translateAngle(angle);
    rad = angle * PI / 180.0f;

    sine = (rad < 0) ? rad * (1.27323954f + 0.405284735f * rad) : rad * (1.27323954f - 0.405284735f * rad);
    sine = (sine < 0) ? sine * (-0.225f * (sine + 1) + 1) : sine * (0.225f * (sine - 1) + 1);
    return sine;
}


float my_cos(float angle)
{
    return my_sin(angle + 90.0f);
}

float arctan1(float tan)
{
    float angle = (ABS(tan) > 1.0f) ?                                                                                                                                                               \
                                90.0f - ABS(1.0f / tan) * (45.0f - (ABS(1.0f / tan) - 1.0f) * (14.0f + 3.83f * ABS(1.0f / tan)))    : \
                                ABS(tan) * (45.0f - (ABS(tan) - 1.0f) * (14.0f + 3.83f * ABS(tan)));
    return (tan > 0) ? angle : -angle;
}

float arctan2(float x, float y)
{
    float tan, angle;

    if (x == 0 && y == 0)       //不存在
        return 0;

    if (x == 0)                         //y轴上
    {
        if (y > 0)
            return 90;
        else
            return -90;
    }

    if (y == 0)                         //x轴上
    {
        if (x > 0)
            return 0;
        else
            return -180.0f;
    }

    tan = y / x;
    angle = arctan1(tan);
    if (x < 0 && angle > 0)
        angle -= 180.0f;
    else if (x < 0 && angle < 0)
        angle += 180.0f;
    return angle;
}


float arcsin(float i)
{
    return arctan1(i / fast_sqrt(1 - i * i));
}




void init_attitude()
{
//标志位初始化
    attitude.flag.run = 1;                     //开启计算
    attitude.flag.use_mag = 0;             //使用地磁
//接口初始化
    attitude.interface.data.a_x = 0;
    attitude.interface.data.a_y = 0;
    attitude.interface.data.a_z = 0;
    attitude.interface.data.g_x = 0;
    attitude.interface.data.g_y = 0;
    attitude.interface.data.g_z = 0;
    attitude.interface.data.m_x = 0;
    attitude.interface.data.m_y = 0;
    attitude.interface.data.m_z = 0;
//参数初始化
    attitude.parameter.error_ki = 1.98f; //1.98 //1.25
    attitude.parameter.error_kp = 5.8f; //5.5f
    attitude.parameter.correct_kp = 0.4f;//0.4
//中间变量清空
    attitude.process.error.x = 0;
    attitude.process.error.y = 0;
    attitude.process.error.z = 0;
    attitude.process.error_integral.x = 0;
    attitude.process.error_integral.y = 0;
    attitude.process.error_integral.z = 0;

    attitude.process.quaternion[0] = 1;
    attitude.process.quaternion[1] = 0;
    attitude.process.quaternion[2] = 0;
    attitude.process.quaternion[3] = 0;
//数据初始化
    attitude.data.rotate_matrix[0][0] = 0;
    attitude.data.rotate_matrix[0][1] = 0;
    attitude.data.rotate_matrix[0][2] = 0;
    attitude.data.rotate_matrix[1][0] = 0;
    attitude.data.rotate_matrix[1][1] = 0;
    attitude.data.rotate_matrix[1][2] = 0;
    attitude.data.rotate_matrix[2][0] = 0;
    attitude.data.rotate_matrix[2][1] = 0;
    attitude.data.rotate_matrix[2][2] = 0;

    attitude.data.mag_world.x = 0;
    attitude.data.mag_world.y = 0;
    attitude.data.mag_world.z = 0;

    attitude.data.acc_world.x = 0;
    attitude.data.acc_world.y = 0;
    attitude.data.acc_world.z = 0;

    attitude.data.mag_correct.x = 0;
    attitude.data.mag_correct.y = 0;
    attitude.data.mag_correct.z = 0;

    attitude.data.acc_correct.x = 0;
    attitude.data.acc_correct.y = 0;
    attitude.data.acc_correct.z = 0;

    attitude.data.gyro_correct.x = 0;
    attitude.data.gyro_correct.y = 0;
    attitude.data.gyro_correct.z = 0;

    attitude.data.pitch = 0;
    attitude.data.rol = 0;
    attitude.data.yaw = 0;
}


void simple_3d_trans(XYZ_Data_f *ref, XYZ_Data_f *in, XYZ_Data_f *out) //小范围内正确。
{
    static char pn;
    static float h_tmp_x,h_tmp_y;

    h_tmp_x = fast_sqrt(ref->z * ref->z + ref->y * ref->y);
    h_tmp_y = fast_sqrt(ref->z * ref->z + ref->x * ref->x);

    pn = ref->z < 0 ? -1 : 1;

    out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
    out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
    out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}



void calculate_attitude(float cycle)
{
    float length;
    XYZ_Data_f acc_tmp;
    XYZ_Data_f error;

//    attitude.interface.data.a_x = imudata.acc_x;
//    attitude.interface.data.a_y = imudata.acc_y;
//    attitude.interface.data.a_z = imudata.acc_z;
//    attitude.interface.data.g_x = imudata.gyro_x;
//    attitude.interface.data.g_y = imudata.gyro_y;
//    attitude.interface.data.g_z = imudata.gyro_z;
//    attitude.interface.data.m_x = imudata.mag_x;
//    attitude.interface.data.m_y = imudata.mag_y;
//    attitude.interface.data.m_z = imudata.mag_z;



    if (attitude.flag.run == 0)
        return;

/////////////////////////////////////////////////////////////////////////////////////////////////
    //电子罗盘处理
    if (attitude.flag.use_mag == 1)
    {
        //利用电子罗盘计算yaw
        length = fast_sqrt(attitude.data.mag_correct.x * attitude.data.mag_correct.x + attitude.data.mag_correct.y * attitude.data.mag_correct.y);
        if( attitude.data.mag_correct.x != 0 && attitude.data.mag_correct.y != 0 && attitude.data.mag_correct.z != 0 && length != 0)
        {
            attitude.process.mag_yaw = arctan2(attitude.data.mag_correct.y / length, attitude.data.mag_correct.x / length);
//          attitude.process.mag_yaw = arctan2(attitude.data.mag_correct.y , attitude.data.mag_correct.x);
        }

        //计算yaw偏差
        if(attitude.data.rotate_matrix[2][2] > 0.0f )
        {
            attitude.process.mag_yaw_bias = attitude.parameter.correct_kp * translateAngle(attitude.data.yaw - attitude.process.mag_yaw);
            //矫正值过大 -- 矫正值错误
            if(attitude.process.mag_yaw_bias > 360 || attitude.process.mag_yaw_bias < -360)
            {
                attitude.process.mag_yaw_bias = 0;
            }
        }

        else
        {
            attitude.process.mag_yaw_bias = 0; //角度过大，停止修正，修正的目标值可能不正确
        }
    }

    else
    {
        attitude.process.mag_yaw_bias = 0;
    }
/////////////////////////////////////////////////////////////////////////////////////////////////
    //加速度计处理
    length = fast_sqrt( (attitude.interface.data.a_x) * (attitude.interface.data.a_x) +
                                    (attitude.interface.data.a_y) * (attitude.interface.data.a_y) +
                                    (attitude.interface.data.a_z) * (attitude.interface.data.a_z));

    if( ABS((attitude.interface.data.a_x)) < 1050.0f &&
            ABS((attitude.interface.data.a_y)) < 1050.0f &&
            ABS((attitude.interface.data.a_z)) < 1050.0f )
    {
        //加速度计归一化
        acc_tmp.x = (attitude.interface.data.a_x) / length;
        acc_tmp.y = (attitude.interface.data.a_y) / length;
        acc_tmp.z = (attitude.interface.data.a_z) / length;

        //叉乘计算偏差    --
        if(800.0f < length && length < 1200.0f)
        {
            /* 叉乘得到误差 */
            error.x = (acc_tmp.y * attitude.data.rotate_matrix[2][2] - acc_tmp.z * attitude.data.rotate_matrix[1][2]);
            error.y = (acc_tmp.z * attitude.data.rotate_matrix[0][2] - acc_tmp.x * attitude.data.rotate_matrix[2][2]);
            error.z = (acc_tmp.x * attitude.data.rotate_matrix[1][2] - acc_tmp.y * attitude.data.rotate_matrix[0][2]);

            /* 误差低通 */
            attitude.process.error.x += 1.0f * 3.14f * cycle *(error.x  - attitude.process.error.x );
            attitude.process.error.y += 1.0f * 3.14f * cycle *(error.y  - attitude.process.error.y );
            attitude.process.error.z += 1.0f * 3.14f * cycle *(error.z  - attitude.process.error.z );
        }
    }
    else
    {
        attitude.process.error.x = 0;
        attitude.process.error.y = 0  ;
        attitude.process.error.z = 0 ;
    }

    // 误差积分
    attitude.process.error_integral.x += attitude.process.error.x * attitude.parameter.error_ki * cycle;
    attitude.process.error_integral.y += attitude.process.error.y * attitude.parameter.error_ki * cycle;
    attitude.process.error_integral.z += attitude.process.error.z * attitude.parameter.error_ki * cycle;

    //积分限幅 -- 2°以内
    attitude.process.error_integral.x = LIMIT(attitude.process.error_integral.x, - 0.035f ,0.035f );
    attitude.process.error_integral.y = LIMIT(attitude.process.error_integral.y, - 0.035f ,0.035f );
    attitude.process.error_integral.z = LIMIT(attitude.process.error_integral.z, - 0.035f ,0.035f );

/////////////////////////////////////////////////////////////////////////////////////////////////
    //开始修正陀螺仪值
    attitude.data.gyro_correct.x = ((attitude.interface.data.g_x) - attitude.data.rotate_matrix[0][2] * attitude.process.mag_yaw_bias) * 0.01745329f +
                        (attitude.parameter.error_kp * attitude.process.error.x + attitude.process.error_integral.x) ;
    attitude.data.gyro_correct.y = ((attitude.interface.data.g_y) - attitude.data.rotate_matrix[1][2] * attitude.process.mag_yaw_bias) * 0.01745329f +
                        (attitude.parameter.error_kp * attitude.process.error.y + attitude.process.error_integral.y) ;
    attitude.data.gyro_correct.z = ((attitude.interface.data.g_z) - attitude.data.rotate_matrix[2][2] * attitude.process.mag_yaw_bias) * 0.01745329f +
                        (attitude.parameter.error_kp * attitude.process.error.z + attitude.process.error_integral.z) ;

/////////////////////////////////////////////////////////////////////////////////////////////////
    // 一阶龙格库塔更新四元数值
    attitude.process.quaternion[0] += (-attitude.process.quaternion[1] * attitude.data.gyro_correct.x - attitude.process.quaternion[2] * attitude.data.gyro_correct.y - attitude.process.quaternion[3] * attitude.data.gyro_correct.z) * cycle / 2.0f;
    attitude.process.quaternion[1] +=  (attitude.process.quaternion[0] * attitude.data.gyro_correct.x + attitude.process.quaternion[2] * attitude.data.gyro_correct.z - attitude.process.quaternion[3] * attitude.data.gyro_correct.y) * cycle / 2.0f;
    attitude.process.quaternion[2] +=  (attitude.process.quaternion[0] * attitude.data.gyro_correct.y - attitude.process.quaternion[1] * attitude.data.gyro_correct.z + attitude.process.quaternion[3] * attitude.data.gyro_correct.x) * cycle / 2.0f;
    attitude.process.quaternion[3] +=  (attitude.process.quaternion[0] * attitude.data.gyro_correct.z + attitude.process.quaternion[1] * attitude.data.gyro_correct.y - attitude.process.quaternion[2] * attitude.data.gyro_correct.x) * cycle / 2.0f;

    //四元数归一化
    length = fast_sqrt(attitude.process.quaternion[0] * attitude.process.quaternion[0] +
                                attitude.process.quaternion[1] * attitude.process.quaternion[1] +
                                attitude.process.quaternion[2] * attitude.process.quaternion[2] +
                                attitude.process.quaternion[3] * attitude.process.quaternion[3]);

    if (length != 0)
    {
        attitude.process.quaternion[0] /= length;
        attitude.process.quaternion[1] /= length;
        attitude.process.quaternion[2] /= length;
        attitude.process.quaternion[3] /= length;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////
    //计算旋转矩阵
    attitude.data.rotate_matrix[0][0] =    attitude.process.quaternion[0] * attitude.process.quaternion[0] + attitude.process.quaternion[1] * attitude.process.quaternion[1] -
                                                                        attitude.process.quaternion[2] * attitude.process.quaternion[2] - attitude.process.quaternion[3] * attitude.process.quaternion[3];
    attitude.data.rotate_matrix[0][1] =    2 * (attitude.process.quaternion[1] * attitude.process.quaternion[2] + attitude.process.quaternion[0] * attitude.process.quaternion[3]);
    attitude.data.rotate_matrix[0][2] =  2 * (attitude.process.quaternion[1] * attitude.process.quaternion[3] - attitude.process.quaternion[0] * attitude.process.quaternion[2]);

    attitude.data.rotate_matrix[1][0] =  2 * (attitude.process.quaternion[1] * attitude.process.quaternion[2] - attitude.process.quaternion[0] * attitude.process.quaternion[3]);
    attitude.data.rotate_matrix[1][1] =    attitude.process.quaternion[0] * attitude.process.quaternion[0] - attitude.process.quaternion[1] * attitude.process.quaternion[1] +
                                                                        attitude.process.quaternion[2] * attitude.process.quaternion[2] - attitude.process.quaternion[3] * attitude.process.quaternion[3];
    attitude.data.rotate_matrix[1][2] =  2 * (attitude.process.quaternion[2] * attitude.process.quaternion[3] + attitude.process.quaternion[0] * attitude.process.quaternion[1]);

    attitude.data.rotate_matrix[2][0] =    2 * (attitude.process.quaternion[1] * attitude.process.quaternion[3] + attitude.process.quaternion[0] * attitude.process.quaternion[2]);
    attitude.data.rotate_matrix[2][1] =    2 * (attitude.process.quaternion[2] * attitude.process.quaternion[3] - attitude.process.quaternion[0] * attitude.process.quaternion[1]);
    attitude.data.rotate_matrix[2][2] =    attitude.process.quaternion[0] * attitude.process.quaternion[0] - attitude.process.quaternion[1] * attitude.process.quaternion[1] -
                                                                        attitude.process.quaternion[2] * attitude.process.quaternion[2] + attitude.process.quaternion[3] * attitude.process.quaternion[3];

    //计算世界坐标系下的磁力计值
    if (attitude.flag.use_mag == 1)
    {
        attitude.data.mag_world.x =    attitude.data.rotate_matrix[0][0] * (attitude.interface.data.m_x) +
                                                            attitude.data.rotate_matrix[1][0] * (attitude.interface.data.m_y) +
                                                            attitude.data.rotate_matrix[2][0] * (attitude.interface.data.m_z) ;

        attitude.data.mag_world.y =    attitude.data.rotate_matrix[0][1] * (attitude.interface.data.m_x) +
                                                            attitude.data.rotate_matrix[1][1] * (attitude.interface.data.m_y) +
                                                            attitude.data.rotate_matrix[2][1] * (attitude.interface.data.m_z) ;

        attitude.data.mag_world.z =    attitude.data.rotate_matrix[0][2] * (attitude.interface.data.m_x) +
                                                            attitude.data.rotate_matrix[1][2] * (attitude.interface.data.m_y) +
                                                            attitude.data.rotate_matrix[2][2] * (attitude.interface.data.m_z) ;
    }

    //计算世界坐标系下的加速度值
    attitude.data.acc_world.x =    attitude.data.rotate_matrix[0][0] * (attitude.interface.data.a_x) +
                                                        attitude.data.rotate_matrix[1][0] * (attitude.interface.data.a_y) +
                                                        attitude.data.rotate_matrix[2][0] * (attitude.interface.data.a_z) ;

    attitude.data.acc_world.y =    attitude.data.rotate_matrix[0][1] * (attitude.interface.data.a_x) +
                                                        attitude.data.rotate_matrix[1][1] * (attitude.interface.data.a_y) +
                                                        attitude.data.rotate_matrix[2][1] * (attitude.interface.data.a_z) ;

    attitude.data.acc_world.z =    attitude.data.rotate_matrix[0][2] * (attitude.interface.data.a_x) +
                                                        attitude.data.rotate_matrix[1][2] * (attitude.interface.data.a_y) +
                                                        attitude.data.rotate_matrix[2][2] * (attitude.interface.data.a_z) ;

    //求解欧拉角
    attitude.data.rol = arctan2(attitude.data.rotate_matrix[2][2], attitude.data.rotate_matrix[1][2]);
    attitude.data.pitch = -arcsin(attitude.data.rotate_matrix[0][2]);
    attitude.data.yaw = arctan2(attitude.data.rotate_matrix[0][0], attitude.data.rotate_matrix[0][1]);

/////////////////////////////////////////////////////////////////////////////////////////////////
    //计算机体坐标系矫正后的加速度--不受俯仰和翻滚影响
    attitude.data.acc_correct.x =   attitude.data.acc_world.x * my_cos(attitude.data.yaw) + attitude.data.acc_world.y * my_sin(attitude.data.yaw);
    attitude.data.acc_correct.y =  -attitude.data.acc_world.x * my_sin(attitude.data.yaw) + attitude.data.acc_world.y * my_cos(attitude.data.yaw);
    attitude.data.acc_correct.z =   attitude.data.acc_world.z;

    //计算机体坐标系矫正后的磁场--不受俯仰和翻滚影响
    if (attitude.flag.use_mag == 1)
    {
        XYZ_Data_f ref_v = (XYZ_Data_f){attitude.data.rotate_matrix[0][2], attitude.data.rotate_matrix[1][2], attitude.data.rotate_matrix[2][2]};
        XYZ_Data_f mag_tmp = (XYZ_Data_f){attitude.interface.data.m_x, attitude.interface.data.m_y, attitude.interface.data.m_z};

        length =    fast_sqrt(  (attitude.interface.data.m_x) * (attitude.interface.data.m_x) +
                                        (attitude.interface.data.m_y) * (attitude.interface.data.m_y) +
                                        (attitude.interface.data.m_z) * (attitude.interface.data.m_z));

        if (length != 0)
        {
            simple_3d_trans(&ref_v, &mag_tmp, &attitude.data.mag_correct);//地磁坐标变换
//          attitude.data.mag_correct.z = pose->data.mag_world.z;
//          attitude.data.mag_correct.x = fast_sqrt(1 - (attitude.data.mag_correct.z / length) * (attitude.data.mag_correct.z / length)) * *(attitude.interface.data.m_x);
//          attitude.data.mag_correct.y = fast_sqrt(1 - (attitude.data.mag_correct.z / length) * (pose->data.mag_correct.z / length)) * *(attitude.interface.data.m_y);
        }
    }

}



Kalman_node  kalman_this_time, kalman_last_time;


float a; //地磁偏角α,通过查表得到

void kalman_init(void)
{

    a=0;//地磁偏角α,通过查表得到,查表过程从略

    //初始化卡尔曼时间节点
    //本次时间
    kalman_this_time.k=0;
    kalman_this_time.p=0;
    kalman_this_time.q = 0.0001;   //过程噪声方差,经验值是0.0025-0.003
    kalman_this_time.r = 0.01f;   //测量噪声方差,经验值是0.001

    //上一次时间
    kalman_last_time.k=0;
    kalman_last_time.p=1;
    kalman_last_time.q = kalman_this_time.q;
    kalman_last_time.r = kalman_this_time.r;
    kalman_last_time.x = 0.0f; //给一个初始值,假定是地磁的数值
}

////第三层处理角速度和地磁数据融合,如果把角速度来源改为车轮,也可以得到地磁和车轮的数据融合.
void loop_kalman(float gyro_z,float magYaw,float kalman_dt )//
{
    //卡尔曼先验预测 pre_x
     kalman_this_time.pre_x = kalman_last_time.x + gyro_z * kalman_dt; //计算先验预测=上一时刻的卡尔曼输出值+当前时刻角速度(dω)*测量周期(dt)
    //卡尔曼估计方差p
     kalman_this_time.p = kalman_last_time.p + kalman_last_time.q; //计算先验预测方差=上一时刻的先验预测方差+Q
    //卡尔曼增益 k
     kalman_this_time.k = kalman_this_time.p / (kalman_this_time.p + kalman_this_time.r); //计算卡尔曼增益=本次估计方差/(本次估计方差+R)
    //卡尔曼输出 x
     kalman_this_time.x = kalman_this_time.pre_x + kalman_this_time.k * (magYaw - kalman_this_time.pre_x);
    //卡尔曼刷新 p
     kalman_this_time.p = (1-kalman_this_time.k)*kalman_this_time.p;
    //推动时间节点
    kalman_last_time=kalman_this_time;//把this time 推成last time
    kalman_last_time.x = kalman_this_time.x;
    kalman_last_time.p = kalman_this_time.p;
    kalman_last_time.k = kalman_this_time.k;

}

/**
 * @brief  高通滤波器
 * @author
 * @param  in-输入数据 out-滤波输出数据
 * @return void
 */
void high_pass_filter(float in, float *out)
{
    float rc, coff;
    static float in_p, out_p;

    rc = 1.0 / 2.0 / PI / HP_CUT_FRQ;
    coff = rc / (rc + 1 / SAMPLE_RATE);
    *out = (in - in_p + out_p) * coff;

    out_p = *out;
    in_p = in;
}

/**
  * @brief  implement 1 order RC low pass filter
  *         raw data filtered by a simple RC low pass filter@cufoff=5Hz
  * @param  Vi      :   Vi(k)
  * @param  Vi_p    :   Vi(k-1)
  * @param  Vo      :   Vo(k)
  * @param  Vo_p    :   Vo(k-1)
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void LowPassFilter_RC(float Vi, float *Vo)
{
    float CutFrq, RC, Cof1, Cof2;
    static float    *Vo_p;
    //low pass filter @cutoff frequency = 5 Hz
    CutFrq = 2.0f;
    RC = (float)1.0f/2.0f/PI/CutFrq;
    Cof1 = 1.0f/(1.0f+RC*SAMPLE_RATE);
    Cof2 = RC*SAMPLE_RATE/(1.0f+RC*SAMPLE_RATE);
    *Vo = Cof1 * (Vi) + Cof2 * (*Vo_p);

    //update
    *Vo_p = *Vo;
}

// 冒泡排序
void bubblesort(int32_t *arr, uint8_t len) // 小--》大
{
    int32_t temp;
    uint8_t i, j;
    for (i = 0; i < len - 1; i++) /* 外循环为排序趟数，len个数进行len-1趟 */
    {
        for (j = 0; j < len - 1 - i; j++)
        { /* 内循环为每趟比较的次数，第i趟比较len-i次 */
            if (arr[j] > arr[j + 1])
            { /* 相邻元素比较，若逆序则交换（升序为左大于右，降序反之） */
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
 * @brief
 *
 * @param in
 * @param out
 * @param window_Array
 * @param i
 * @param sliding_Window_Length
 */
float window_filter(float data, float *buf, uint8_t len)
{
    uint8_t i;
    float sum = 0;

    for (i = 1; i < len; i++)
    {
        buf[i - 1] = buf[i];
    }
    buf[len - 1] = data;

    for (i = 0; i < len; i++)
    {
        sum += buf[i];
    }

    sum /= len;

    return sum;
}


void imu_data_process(float ax,float ay,float az,float gx,float gy,float gz,float mx ,float my,float mz)
{

    attitude.interface.data.a_x = window_filter(ax,window_ax,WIN_NUM);
    attitude.interface.data.a_y = window_filter(ay,window_ay,WIN_NUM);
    attitude.interface.data.a_z = window_filter(az,window_az,WIN_NUM);
    attitude.interface.data.g_x = window_filter(gx,window_gx,WIN_NUM);
    attitude.interface.data.g_y = window_filter(gy,window_gy,WIN_NUM);
    attitude.interface.data.g_z = window_filter(gz,window_gz,WIN_NUM);
    attitude.interface.data.m_x = window_filter(mx,window_mx,WIN_NUM);
    attitude.interface.data.m_y = window_filter(my,window_my,WIN_NUM);
    attitude.interface.data.m_z = window_filter(mz,window_mz,WIN_NUM);
}



