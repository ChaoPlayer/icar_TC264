#ifndef __CONTROL_H
#define __CONTROL_H
#include "zf_common_headfile.h"
#include "headfile_code.h"

typedef enum
{
    STOPM = 1,
    MOTORTEST = 2,
    STAND = 3,
    CLOSELOOP = 4,
}CarMode;

typedef struct
{
    CarMode Mode;
    uint8 Condition;
}CarInfotypedef;

typedef enum { //���PWMƵ��
    FREQ50HZ,
    FREQ300HZ,
} ServoFreq;
extern ServoFreq servo_freq;
extern CarInfotypedef CarInfo;

extern uint8 stop_mode ;
extern float pout;
extern float pwmout;

typedef struct{
  float Error;                          //�ٶ�ƫ��
  float Stan;                           //��׼�ٶ�
  float L_ControlIntegral;
  float R_ControlIntegral;
  float lowest;                    //�ٶ��趨��Сֵ
  float highest;
  float L_Bigeest;
  float R_Bigeest;
}Speed_struct;

typedef struct
{
    float Expect;
    float ErrorMaxLimit;
    float ErrorMinLimit;
    float KP;
    float KI;
    float KD;
    float BaseKP;
    float BaseKI;
    float BaseKD;
    float Error;
    float ErrorFifo[5];
    float ErrorLast;
    float ErrorDtFifo[5];
    float ErrorTemp[4];
    float ErrorDtTemp[4];
    float InOut;
    float Integ;
    float OutPut;
    float OutPutMaxLimit;
    float OutPutMinLimit;
    float RampKP;
    float RampKD;
}StandInfotypedef;

extern Speed_struct speed;
extern float SpeedControl;

typedef struct
{
  void (*SpeedInit)();
  void (*MotorControl)(float* err);
  

} CONTROL_CLASS;
extern CONTROL_CLASS control;

typedef struct {
	float P;
	float I;
	float D;
} PID_CLASS;

extern PID_CLASS MotorPID;

typedef struct
{
    uint16_t thresholdMiddle;                   //�����ֵPWM
    uint16_t thresholdLeft;                     //�������ת�����ֵPWM
    uint16_t thresholdRight;                    //�������ת�����ֵPWM
}ServoStruct;

extern ServoStruct servoStr; //����ṹ��

typedef struct
{
    float vi_Ref;                       //�ٶ�PID���ٶ��趨ֵ
    float vi_FeedBack;                  //�ٶ�PID���ٶȷ���ֵ
    float vi_PreError;                  //�ٶ�PID���ٶ����,vi_Ref - vi_FeedBack
    float vi_PreDerror;                 //�ٶ�PID��ǰһ�Σ��ٶ����֮�d_error-PreDerror;
    float v_Kp;                         //����ϵ����Kp = Kp
    float v_Ki;                         //����ϵ����Ki = Kp * ( T / Ti )
    float v_Kd;                         //΢��ϵ����Kd = KP * Td * T
    float vl_PreU;                      //PID���ֵ
}PIDStruct;



typedef enum
{
    Selfcheck_None = 0,             //��ʼ����
    Selfcheck_MotorA,               //�����ת����
    Selfcheck_MotorB,               //�����ת����
    Selfcheck_MotorC,               //�����ת����
    Selfcheck_MotorD,               //�����ת����
    Selfcheck_MotorE,               //����ջ���������
    Selfcheck_MotorF,               //����ջ���������
    Selfcheck_MotorG,               //����ջ���ת����
    Selfcheck_MotorH,               //����ջ���ת����
    Selfcheck_ServoA,               //�������A
    Selfcheck_Com,                  //ͨ�Ų���
    Selfcheck_Buzzer,               //����������
    Selfcheck_RgbLed,               //��Ч����
    Selfcheck_Key,                  //��������
    Selfcheck_Finish                //�������
}SelfcheckEnum;

typedef struct                              //[���ܳ���������]
{
    float Voltage;                          //��ص�ѹ
    uint8_t Electricity;                    //��ص����ٷֱȣ�0~100
    float SpeedSet;                         //���Ŀ���ٶȣ�m/s
    float SpeedFeedback;                    //���ģ��ʵ���ٶȣ�m/s
    float SpeedMaxRecords;                  //���Լ�¼�����
    uint16_t ServoPwmSet;                   //���PWM����

    uint16_t counterKeyA;                   //����ģʽA������
    bool keyPressed;                        //��������
    bool sprintEnable;                      //�ջ����ʹ��
    uint16_t counterSprint;                 //�ջ����ʱ��
    uint16_t errorCode;                     //�������

    bool selfcheckEnable;                   //���ܳ��Լ�ʹ��
    uint16_t counterSelfcheck;              //�Լ������
    uint8_t timesSendStep;                  //���ͳ�ʱ���ݴ���
    uint16_t counterModuleCheck;            //�Լ������
    SelfcheckEnum selfcheckStep;            //�Լ첽��
    uint8_t speedSampleStep;                //�ٶȲ�������
}IcarStruct;


extern IcarStruct icarStr;
extern PIDStruct pidStr; //�����ٶȵıջ�����

extern float  l_setspeed, r_setspeed;
extern float Ratio_Outter_KP,Ratio_Outter_KD;

extern float kp_init;

extern StandInfotypedef SpeedLoop;

extern float Diraction_Out;
extern float diraction[4];
extern float speed_avr;

void Speed_Loop(void);
void PID_init(void);
void stop(void);
void Serv_Init(void); //�����ؿ��ƺ���
void Serv_set_pwm(uint16_t pwm);
void Icar_init(void);
void Car_control();
void save_servo_freq();
float check_speed();

int read_servo_freq();

extern float speed_init ;
#endif
