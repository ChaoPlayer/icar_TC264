#ifndef __ENCODE_H
#define __ENCODE_H

/****Includes*************************************************/

#include "zf_common_headfile.h"
#include "headfile_code.h"
// **************************** �궨�� ****************************
#define TASK_ENABLE 0

#define ENCODER1_GPT          TIM2_ENCOEDER
#define count1_pin            TIM2_ENCOEDER_CH2_P33_6
#define dir1_pin              TIM2_ENCOEDER_CH1_P33_7

#define ENCODER2_GPT          TIM6_ENCOEDER
#define count2_pin            TIM6_ENCOEDER_CH1_P20_3
#define dir2_pin              TIM6_ENCOEDER_CH2_P20_0


#define  MOTOR_PWM_MAX              1500        //OCR=95%,��ֹ��ռ�ձ���������MOS��
#define  MOTOR_PWM_MIN              -1500       //OCR=95%
#define  MOTOR_SPEED_MAX            10.0f       //������ת��(m/s) (0.017,8.04)
#define  MOTOR_CONTROL_CYCLE        0.01f       //�����������T��10ms

/****Definitions**********************************************/

#define SPEED_F 100.0f	//�ٶȻ�ȡƵ�� 200

#define v_QD_UNIT 6119.0f//7000.0f//������һ�׵ļ���

extern float speed_now;
extern int16 v_count;
extern uint32 encoder_count;

void speedcount_init(void);
void GetSpeed();

#endif
