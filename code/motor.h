#ifndef __MOTOR_H
#define __MOTOR_H

/****Includes*************************************************/

#include "zf_common_headfile.h"

/****Definitions**********************************************/

#define DIR                  (P21_5)
#define PWM_CH1               (ATOM0_CH2_P21_4)//(ATOM0_CH2_P21_4)//完全模型PWM
//#define PWM_CH2               (ATOM0_CH5_P02_5)//逐飞
//#define PWM_CH2               (ATOM0_CH3_P21_5)//(ATOM0_CH6_P02_6)//学校

//#define PWM_CH1               (ATOM1_CH6_P02_6)//隔壁组
//#define PWM_CH2               (ATOM1_CH7_P02_7)
#define PWM_SERV             (ATOM1_CH1_P33_9)//(ATOM0_CH1_P33_9)// 舵机

typedef struct
{
    float ReductionRatio ;                      //电机减速比
    float EncoderLine ;                         //编码器线数=光栅数16*4
    signed int EncoderValue;                    //编码器实时速度
    float DiameterWheel;                        //轮子直径：mm
    bool CloseLoop;                             //开/闭环模式
    uint16_t Counter;                           //线程计数器
}MotorStruct; //电机参数结构体

extern MotorStruct motorStr;

void ALL_Out(void);
void MotorInit(void);
extern float Right_out ;
extern float Left_out;

//extern float v_out;
//extern float v_out1;
#endif
