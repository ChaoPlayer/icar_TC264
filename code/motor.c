#include "motor.h"
#include "control.h"

MotorStruct motorStr;
int MotorPwm[2] = {0};

float motor_begin = 2000;
float v_out=0;


void MotorInit()
{
    gpio_init(DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高 电机方向控制
    pwm_init(PWM_CH1,17000,0);                     //电机初始化
//    pwm_init(PWM_CH2,17000,0);                    //ATOM 0模块的通道0 使用P02_4引脚输出PWM  PWM频率10kHZ  占空比百分之0/GTM_ATOM0_PWM_DUTY_MAX*100
    //pwm_init(PWM_SERV,50,servoStr.thresholdMiddle); //舵机初始化
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
void ALL_Out(void) //电机控制函数
{
    v_out =  SpeedControl;
    v_out =   PWM_Limit((float)v_out,5000); //限幅
    if(v_out >=0)
    {
        gpio_set_level(DIR, 0);//逐飞
       pwm_set_duty(PWM_CH1,(uint32)v_out);

//        pwm_set_duty(PWM_CH1,(uint32)v_out);
//        pwm_set_duty(PWM_CH2,0);//学校

    }
    else
    {
       gpio_set_level(DIR, 1);//逐飞
      pwm_set_duty(PWM_CH1,(uint32)-v_out);

//        pwm_set_duty(PWM_CH1,0);//学校
//        pwm_set_duty(PWM_CH2,(uint32)-v_out);
    }
//    gpio_set_level(DIR, 1);//逐飞
//    pwm_set_duty(PWM_CH1,2000);

//    pwm_set_duty(PWM_CH2,5000);
}
