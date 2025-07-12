#include "motor.h"
#include "control.h"

MotorStruct motorStr;
int MotorPwm[2] = {0};

float motor_begin = 2000;
float v_out=0;


void MotorInit()
{
    gpio_init(DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO ��ʼ��Ϊ��� Ĭ����������� ����������
    pwm_init(PWM_CH1,17000,0);                     //�����ʼ��
//    pwm_init(PWM_CH2,17000,0);                    //ATOM 0ģ���ͨ��0 ʹ��P02_4�������PWM  PWMƵ��10kHZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100
    //pwm_init(PWM_SERV,50,servoStr.thresholdMiddle); //�����ʼ��
}
int16 PWM_Limit(float  PWM,int16 max)
{
    if(PWM > -max && PWM < max)
        return (int16)PWM;
    else if( PWM >=  max)
        return (int16) max;
    else
        return (int16) -max;
}
void ALL_Out(void) //������ƺ���
{
    v_out =  SpeedControl;
    v_out =   PWM_Limit((float)v_out,5000); //�޷�
    if(v_out >=0)
    {
        gpio_set_level(DIR, 0);//���
       pwm_set_duty(PWM_CH1,(uint32)v_out);

//        pwm_set_duty(PWM_CH1,(uint32)v_out);
//        pwm_set_duty(PWM_CH2,0);//ѧУ

    }
    else
    {
       gpio_set_level(DIR, 1);//���
      pwm_set_duty(PWM_CH1,(uint32)-v_out);

//        pwm_set_duty(PWM_CH1,0);//ѧУ
//        pwm_set_duty(PWM_CH2,(uint32)-v_out);
    }
//    gpio_set_level(DIR, 1);//���
//    pwm_set_duty(PWM_CH1,2000);

//    pwm_set_duty(PWM_CH2,5000);
}
