#ifndef __MOTOR_H
#define __MOTOR_H

/****Includes*************************************************/

#include "zf_common_headfile.h"

/****Definitions**********************************************/

#define DIR                  (P21_5)
#define PWM_CH1               (ATOM0_CH2_P21_4)//(ATOM0_CH2_P21_4)//��ȫģ��PWM
//#define PWM_CH2               (ATOM0_CH5_P02_5)//���
//#define PWM_CH2               (ATOM0_CH3_P21_5)//(ATOM0_CH6_P02_6)//ѧУ

//#define PWM_CH1               (ATOM1_CH6_P02_6)//������
//#define PWM_CH2               (ATOM1_CH7_P02_7)
#define PWM_SERV             (ATOM1_CH1_P33_9)//(ATOM0_CH1_P33_9)// ���

typedef struct
{
    float ReductionRatio ;                      //������ٱ�
    float EncoderLine ;                         //����������=��դ��16*4
    signed int EncoderValue;                    //������ʵʱ�ٶ�
    float DiameterWheel;                        //����ֱ����mm
    bool CloseLoop;                             //��/�ջ�ģʽ
    uint16_t Counter;                           //�̼߳�����
}MotorStruct; //��������ṹ��

extern MotorStruct motorStr;

void ALL_Out(void);
void MotorInit(void);
extern float Right_out ;
extern float Left_out;

//extern float v_out;
//extern float v_out1;
#endif
