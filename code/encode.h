#ifndef __ENCODE_H
#define __ENCODE_H

/****Includes*************************************************/

#include "zf_common_headfile.h"
#include "headfile_code.h"
// **************************** 宏定义 ****************************
#define TASK_ENABLE 0

#define ENCODER1_GPT          TIM2_ENCOEDER
#define count1_pin            TIM2_ENCOEDER_CH2_P33_6
#define dir1_pin              TIM2_ENCOEDER_CH1_P33_7

#define ENCODER2_GPT          TIM6_ENCOEDER
#define count2_pin            TIM6_ENCOEDER_CH1_P20_3
#define dir2_pin              TIM6_ENCOEDER_CH2_P20_0


#define  MOTOR_PWM_MAX              1500        //OCR=95%,禁止满占空比输出，造成MOS损坏
#define  MOTOR_PWM_MIN              -1500       //OCR=95%
#define  MOTOR_SPEED_MAX            10.0f       //电机最大转速(m/s) (0.017,8.04)
#define  MOTOR_CONTROL_CYCLE        0.01f       //电机控制周期T：10ms

/****Definitions**********************************************/

#define SPEED_F 100.0f	//速度获取频率 200

#define v_QD_UNIT 6119.0f//7000.0f//编码器一米的计数

extern float speed_now;
extern int16 v_count;
extern uint32 encoder_count;

void speedcount_init(void);
void GetSpeed();

#endif
