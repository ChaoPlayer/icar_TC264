#include "imu.h"

void BIAS_init(void);
void Get_Attitude(void);
//***********************************************����������***********************************************//


//*********************************************�ṹ�嶨����*********************************************//
//����imu���ݽṹ��
IMU_DATA_CLASS imudata = {0};

//����imu�����ṹ��
IMU_FUNC_CLASS  imufunc =
{
        &icm20602_init,                             //ICM20602��ʼ��
        &icm20602_get_acc,                          //��ȡICM20602���ٶȼ�����
        &icm20602_get_gyro,                         //��ȡICM20602����������
        &imu963ra_init,                             //IMU963RA��ʼ��
        &imu963ra_get_acc,                          //��ȡIMU963RA���ٶȼ�����
        &imu963ra_get_gyro,                         //��ȡIMU963RA����������
        &imu963ra_get_mag,                          //��ȡIMU963RA�شż�����
        &Get_Attitude,                              //��ȡ������Ϣ����
        &BIAS_init,                                 //���Ե�Ԫ��Ư��ʼ��
};

//������Ư�ṹ��
BIAS_CLASS  AccBias, GyroBias;

//������Ԫ���ṹ��
Quater_st quater =
{
    1,0,0,0,
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    {0,0,0},
    0,0,0,
    0,0,0,
};
//*********************************************�ṹ�嶨����*********************************************//


//**********************************************����������**********************************************//
float gyro_z_bias;          //������z����Ư,��/sΪ��λ
float yaw_rate_y;
float yaw_rate_x;
float angle_y;
float angle_x;
//**********************************************����������**********************************************//


//***********************************************************��������***********************************************************//
//-------------------------------------------------------------------------------------------------------------------
// @brief           ���Ե�Ԫ��Ư����
// @Author          265deya
// @param           void
// @return          void
// @since           v1.0
// Sample usage:
// @note:
//-------------------------------------------------------------------------------------------------------------------
void BIAS_init(void)
{

    int16 min_mag_x = 0,  min_mag_y = 0,  min_mag_z = 0;
    int16 max_mag_x = 0,  max_mag_y = 0,  max_mag_z = 0;
    GyroBias.X_bias = 0;
    GyroBias.Y_bias = 0;
    GyroBias.Z_bias = 0;
    AccBias.X_bias = 0;
    AccBias.Y_bias = 0;
    AccBias.Z_bias = 0;
    uint32 i = 0;

    for(i = 0; i < 1000; i ++)
    {
       imufunc.imu963ra_get_gyro();
       imufunc.imu963ra_get_acc();
       //imufunc.imu963ra_get_mag();
       AccBias.X_bias += imu963ra_acc_x;
       AccBias.Y_bias += imu963ra_acc_y;
       AccBias.Z_bias += imu963ra_acc_z;
       GyroBias.X_bias += imu963ra_gyro_x;
       GyroBias.Y_bias += imu963ra_gyro_y;
       GyroBias.Z_bias += imu963ra_gyro_z;

//       min_mag_x = min(min_mag_x,imu963ra_mag_x);
//       min_mag_y = min(min_mag_y,imu963ra_mag_y);
//       min_mag_z = min(min_mag_z,imu963ra_mag_z);
//
//       max_mag_x = max(max_mag_x,imu963ra_mag_x);
//       max_mag_y = max(max_mag_y,imu963ra_mag_y);
//       max_mag_z = max(max_mag_z,imu963ra_mag_z);
       system_delay_ms(5);
    }
    AccBias.X_bias *= 0.001;                  //����200��ƽ��ֵ
    AccBias.Y_bias *= 0.001;
    AccBias.Z_bias *= 0.001;
    GyroBias.X_bias *= 0.001;
    GyroBias.Y_bias *= 0.001;
    GyroBias.Z_bias *= 0.001;

//    int16 Xsf = (max_mag_y - min_mag_y) / (max_mag_x - min_mag_x);
//    int16 Ysf = (max_mag_x - min_mag_x) / (max_mag_y - min_mag_y);
//    if (Xsf < 1)
//       {
//           Xsf = 1;
//       }
//
//     if (Ysf < 1)
//     {
//           Ysf = 1;
//     }
//    MagBias.X_bias = ( (max_mag_x - min_mag_x)/2 - max_mag_x) *Xsf;
//    MagBias.Y_bias = ( (max_mag_y - min_mag_y)/2 - max_mag_y) *Ysf;
//    MagBias.Z_bias = 0;

}


//-------------------------------------------------------------------------------------------------------------------
// @brief           һ�׸�ͨ�˲���
// @Author          ?
// @param           hz              ����Ƶ��
// @param           time            ����
// @param           in              ����
// @param           *out            ָ�����
// @return          void
// @since           v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void HPF_1_db(float hz, float time, float in, float *out)
{
    *out += ( 1 / (hz * 6.28f * time + 1)) * (in - *out);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief           imu���ݴ����������
// @Author          265deya
// @param           void
// @return          void
// @since           v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------

static float gyro_x,gyro_y,gyro_z;
static float acc_x,acc_y,acc_z;
static float mag_x,mag_y,mag_z;
void imudata_handle(void)
{


    //����ֵ��ȥ��Ư
    gyro_x = imu963ra_gyro_x - GyroBias.X_bias;
    gyro_y = imu963ra_gyro_y - GyroBias.Y_bias;
    gyro_z = imu963ra_gyro_z - GyroBias.Z_bias;
//#define cheat 1

        #if cheat              //���� ������yaw���ȶ� ȥ���Ƚ�С��ֵ
            if(fabsf(gyro_z)<0.003f){
                gyro_z=0;
            }

        #endif


    acc_x = imu963ra_acc_x - AccBias.X_bias;
    acc_y = imu963ra_acc_y - AccBias.Y_bias;
//    acc_z = imu963ra_acc_z - AccBias.Z_bias;

        acc_z = imu963ra_acc_z;
    mag_x = imu963ra_mag_x;
    mag_y = imu963ra_mag_y;
    mag_z = imu963ra_mag_z;

    //����������ת��Ϊ������
    imudata.acc_x = acc_x / 4098;                   //��ȡ���ٶ�ֵ,��λm/s^2
    imudata.acc_y = acc_y / 4098;
    imudata.acc_z = acc_z / 4098;
    imudata.gyro_x = gyro_x / 14.3;                //��ȡ���ٶ�ֵ,��λ��/s
    imudata.gyro_y = gyro_y / 14.3;
    imudata.gyro_z = gyro_z / 14.3;
//        imudata.acc_x = acc_x / 2049;                   //��ȡ���ٶ�ֵ,��λm/s^2
//        imudata.acc_y = acc_y / 2049;
//        imudata.acc_z = acc_z / 2049;
//        imudata.gyro_x = gyro_x / 7.1;                //��ȡ���ٶ�ֵ,��λ��/s
//        imudata.gyro_y = gyro_y / 7.1;
//        imudata.gyro_z = gyro_z / 7.1;
    imudata.mag_x = mag_x / 3000;                   //��ȡ�شż�����,��λ��˹
    imudata.mag_y = mag_y / 3000;
    imudata.mag_z = mag_z / 3000;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief           �������˲�����
// @Author          ?
// @param           angle_m         ��ǰ�Ƕ�ֵ
// @param           angle_n         Ԥ����ٶ�ֵ
// @return          void
// @since           v1.0
// Sample usage:
// @note��           ��Ȼ���洫����,����Ч����̫����,��������ͬѧ�������Լ���������
//-------------------------------------------------------------------------------------------------------------------
void Kalman_Filter(float angle_m, float angle_n)
{
    static  float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 10, dt = 0.005;
    static  float Pk[2][2] = { {1, 0}, {0, 1 }};
    static  float Pdot[4] = {0, 0, 0, 0};
    static  float angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


    Pdot[0] = Q_angle - Pk[0][1] - Pk[1][0];
    Pdot[1] = - Pk[1][1];
    Pdot[2] = - Pk[1][1];
    Pdot[3] = Q_gyro;
    Pk[0][0] += Pdot[0] * dt;
    Pk[0][1] += Pdot[1] * dt;
    Pk[1][0] += Pdot[2] * dt;
    Pk[1][1] += Pdot[3] * dt;
    angle_err = angle_m - angle_n;                       //�ǶȲ�ֵΪ�˴β����Ƕȼ�ȥԤ��Ƕ�
    PCt_0 =  Pk[0][0];
    PCt_1 =  Pk[1][0];
    E = R_angle + PCt_0;
    K_0 = PCt_0 / E;                                              //����������
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = Pk[0][1];
    Pk[0][0] -= K_0 * t_0;
    Pk[0][1] -= K_0 * t_1;
    Pk[1][0] -= K_1 * t_0;
    Pk[1][1] -= K_1 * t_1;
    angle_n += K_0 * angle_err;                          //���ŽǶ�=Ԥ��ֵ+����������*(����ֵ-Ԥ��ֵ)

}


//-------------------------------------------------------------------------------------------------------------------
// @brief           ��Ԫ����̬����
// @Author          ?
// @param           gx,gy,gz        ������������ٶ�
// @param           ax,ay,az        ���ٶȼ�������ٶ�
// @param           *imu            ������ݽṹ��
// @return          void
// @since           v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, Quater_st *imu)
{
    _lf_t err_lf_x,err_lf_y,err_lf_z;_xyz_f_st vec_err_i;
    float Accel_magnitude;

    float kp = 0.7,ki = 0;

    float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;
    float w_q,x_q,y_q,z_q;
    float acc_length,q_length;
    _xyz_f_st acc_norm;
    _xyz_f_st vec_err;
    _xyz_f_st d_angle;

    kp = 1.0; ki = 0;

    w_q = imu->w;
    x_q = imu->x;
    y_q = imu->y;
    z_q = imu->z;

    //        q0q0 = w_q * w_q;
    q0q1 = w_q * x_q;
    q0q2 = w_q * y_q;//pitch
    q1q1 = x_q * x_q;
    q1q3 = x_q * z_q;//pitch
    q2q2 = y_q * y_q;
    q2q3 = y_q * z_q;
    q3q3 = z_q * z_q;
    q1q2 = x_q * y_q;
    q0q3 = w_q * z_q;

    Accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
    Accel_magnitude = Accel_magnitude / 16384.0f; // Scale to gravity.

        if(Accel_magnitude < 1.0f)
        {
            Accel_magnitude = 1.0f - Accel_magnitude;
        }
        else
        {
        Accel_magnitude = Accel_magnitude - 1.0f;
        }

        if(Accel_magnitude <0.3)
        {
            Accel_magnitude = (0.3f - Accel_magnitude) * 3.3333f;
        }
        else
        {
            Accel_magnitude = (Accel_magnitude - 0.3f) * 3.3333f;
        }

    // ���ٶȼƵĶ�������λ����
    acc_length = sqrt(my_pow(ax) + my_pow(ay) + my_pow(az));
    acc_norm.x = ax / acc_length;
    acc_norm.y = ay / acc_length;
    acc_norm.z = az / acc_length;

    // ���������µ�x������������λ����
    imu->x_vec.x = 1 - (2*q2q2 + 2*q3q3);
    imu->x_vec.y = 2*q1q2 - 2*q0q3;
    imu->x_vec.z = 2*q1q3 + 2*q0q2;

    // ���������µ�y������������λ����
    imu->y_vec.x = 2*q1q2 + 2*q0q3;
    imu->y_vec.y = 1 - (2*q1q1 + 2*q3q3);
    imu->y_vec.z = 2*q2q3 - 2*q0q1;

    // ���������µ�z������������Ч�����������������ٶ�����������λ����
    imu->z_vec.x = 2*q1q3 - 2*q0q2;
    imu->z_vec.y = 2*q2q3 + 2*q0q1;
    imu->z_vec.z = 1 - (2*q1q1 + 2*q2q2);

    // �������������µ��˶����ٶȡ�(����̬�����޹�)
    imu->a_acc.x = ax - 9800 *imu->z_vec.x;
    imu->a_acc.y = ay - 9800 *imu->z_vec.y;
    imu->a_acc.z = az - 9800 *imu->z_vec.z;

    // �������������µ��˶����ٶȡ�(����̬�����޹�)
    imu->w_acc.x = imu->x_vec.x *imu->a_acc.x + imu->x_vec.y *imu->a_acc.y + imu->x_vec.z *imu->a_acc.z;
    imu->w_acc.y = imu->y_vec.x *imu->a_acc.x + imu->y_vec.y *imu->a_acc.y + imu->y_vec.z *imu->a_acc.z;
    imu->w_acc.z = imu->z_vec.x *imu->a_acc.x + imu->z_vec.y *imu->a_acc.y + imu->z_vec.z *imu->a_acc.z;
    // ����ֵ���Ч���������Ĳ����������������
    vec_err.x =  (acc_norm.y * imu->z_vec.z - imu->z_vec.y * acc_norm.z);// * Accel_magnitude;
    vec_err.y = -(acc_norm.x * imu->z_vec.z - imu->z_vec.x * acc_norm.z);// * Accel_magnitude;
    vec_err.z = -(acc_norm.y * imu->z_vec.x - imu->z_vec.y * acc_norm.x);// * Accel_magnitude;

    //    //��ֹƵ��1hz�ĵ�ͨ�޷��˲�
    //    if(systime_half_ms < 8000)
    //    {
    err_lf_x.out = vec_err.x;
    err_lf_y.out = vec_err.y;
    err_lf_z.out = vec_err.z;
    //    }
    //    else
    //    {
    //        limit_filter(halfT * 2,0.001f,&err_lf_x,vec_err.x);
    //        limit_filter(halfT * 2,0.001f,&err_lf_y,vec_err.y);
    //        limit_filter(halfT * 2,0.001f,&err_lf_z,vec_err.z);
    //    }

    //������
    vec_err_i.x += err_lf_x.out * halfT * 2 *ki;
    vec_err_i.y += err_lf_y.out * halfT * 2 *ki;
    vec_err_i.z += err_lf_z.out * halfT * 2 *ki;

    // ����������ת�����ںϾ�������
    d_angle.x = (gx + (err_lf_x.out + vec_err_i.x) * kp) * halfT * 2 / 2 ;
    d_angle.y = (gy + (err_lf_y.out + vec_err_i.y) * kp) * halfT * 2 / 2 ;
    d_angle.z = (gz + (err_lf_z.out + vec_err_i.z) * kp) * halfT * 2 / 2 ;

    // ������̬��
    imu->w = w_q           - x_q*d_angle.x - y_q*d_angle.y - z_q*d_angle.z;
    imu->x = w_q*d_angle.x + x_q           + y_q*d_angle.z - z_q*d_angle.y;
    imu->y = w_q*d_angle.y - x_q*d_angle.z + y_q           + z_q*d_angle.x;
    imu->z = w_q*d_angle.z + x_q*d_angle.y - y_q*d_angle.x + z_q;

    q_length = sqrt(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w /= q_length;
    imu->x /= q_length;
    imu->y /= q_length;
    imu->z /= q_length;

    imu->pit = asin(2*q1q3 - 2*q0q2)*57.30f;
    imu->rol = atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f;
    imu->yaw = -atan2(2*q1q2 + 2*q0q3, -2*q2q2-2*q3q3 + 1)*57.30f;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief           ��ȡС����̬����
// @Author          265deya
// @param           void
// @return          void
// @since           v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------

static uint8_t first_mahony=0;

void Get_Attitude(void)
{


    //��ȡԭʼ����
    imufunc.imu963ra_get_acc();
    imufunc.imu963ra_get_gyro();
    imufunc.imu963ra_get_mag();

    //���ݴ���
    imudata_handle();

    if(first_mahony == 0){
        //MH��ʼ��
        Mahony_Init(1000); //5ms����Ƶ��
        MahonyAHRSinit(imudata.acc_x,imudata.acc_y,imudata.acc_z,0,0,0);
//        MahonyAHRSinit(imudata.acc_x,imudata.acc_y,imudata.acc_z,imudata.mag_x,imudata.mag_y,imudata.mag_z); //�����شż�
        //IMU_QuaternionEKF_Init(1, 0.01, 100000, 1, 0.001f,0); //ekf �˲�

        first_mahony = 1;
        return;
    }

    //��Ԫ����̬����õ�yaw,roll,pitch
    //IMUupdate(imudata.gyro_x, imudata.gyro_y, imudata.gyro_z,imudata.acc_x,  imudata.acc_y,  imudata.acc_z,  &quater);


//
    Mahony_update(imudata.gyro_x, imudata.gyro_y, imudata.gyro_z,imudata.acc_x,  imudata.acc_y,  imudata.acc_z,0,0,0);
    //Mahony_update(imudata.gyro_x, imudata.gyro_y, imudata.gyro_z,imudata.acc_x,  imudata.acc_y,  imudata.acc_z,imudata.mag_x,imudata.mag_y,imudata.mag_z);
    Mahony_computeAngles();
        imudata.yaw   = DEGREE_TRANS(getYaw()); //MH��pitch��roll�ѱ�����
        //imudata.pitch = DEGREE_TRANS(getPitch());
        //imudata.roll  = DEGREE_TRANS(getRoll());
//EKF
   //IMU_QuaternionEKF_Update(imudata.gyro_x*0.0174533f, imudata.gyro_y*0.0174533f, imudata.gyro_z*0.0174533f,imudata.acc_x,  imudata.acc_y,  imudata.acc_z);
//              imudata.yaw=Get_Yaw();//EKFyaw
             // imudata.yaw   = DEGREE_TRANS(Get_Yaw()); //0-360

             // imudata.roll=DEGREE_TRANS(Get_Roll());//EKFroll 0-360
             // imudata.pitch=Get_Pitch(); //EKFpitch

//    imu_data_process(imudata.acc_x,  imudata.acc_y,  imudata.acc_z,imudata.gyro_x, imudata.gyro_y, imudata.gyro_z,imudata.mag_x,imudata.mag_y,imudata.mag_z);
//    calculate_attitude(0.001);
//
//
//    imudata.yaw   = DEGREE_TRANS(attitude.data.yaw);
//    imudata.roll  = DEGREE_TRANS(attitude.data.rol);
//    imudata.pitch = attitude.data.pitch;

}


//-------------------------------------------------------------------------------------------------------------------
// @brief           ���Ե�Ԫ��ʼ��
// @Author          265deya
// @param           void
// @return          void
// @since           v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void imu_init(void)
{
    imufunc.imu963ra_init();
    imufunc.BIAS_init();
    //��ʼ�����ұ�
    Ifx_LutAtan2F32_init();

    //��ʼ����̬����
    init_attitude();
    IMU_QuaternionEKF_Init(10, 0.0001, 1000000, 0.9996, 0.001f,0); //ekf��ʼ��

}
//***********************************************************��������***********************************************************//
