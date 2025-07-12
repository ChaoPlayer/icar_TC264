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

typedef enum { //舵机PWM频率
    FREQ50HZ,
    FREQ300HZ,
} ServoFreq;
extern ServoFreq servo_freq;
extern CarInfotypedef CarInfo;

extern uint8 stop_mode ;
extern float pout;
extern float pwmout;

typedef struct{
  float Error;                          //速度偏差
  float Stan;                           //标准速度
  float L_ControlIntegral;
  float R_ControlIntegral;
  float lowest;                    //速度设定最小值
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
    uint16_t thresholdMiddle;                   //舵机中值PWM
    uint16_t thresholdLeft;                     //舵机左向转角最大值PWM
    uint16_t thresholdRight;                    //舵机右向转角最大值PWM
}ServoStruct;

extern ServoStruct servoStr; //舵机结构体

typedef struct
{
    float vi_Ref;                       //速度PID，速度设定值
    float vi_FeedBack;                  //速度PID，速度反馈值
    float vi_PreError;                  //速度PID，速度误差,vi_Ref - vi_FeedBack
    float vi_PreDerror;                 //速度PID，前一次，速度误差之差，d_error-PreDerror;
    float v_Kp;                         //比例系数，Kp = Kp
    float v_Ki;                         //积分系数，Ki = Kp * ( T / Ti )
    float v_Kd;                         //微分系数，Kd = KP * Td * T
    float vl_PreU;                      //PID输出值
}PIDStruct;



typedef enum
{
    Selfcheck_None = 0,             //开始测试
    Selfcheck_MotorA,               //电机正转启动
    Selfcheck_MotorB,               //电机正转采样
    Selfcheck_MotorC,               //电机反转启动
    Selfcheck_MotorD,               //电机反转采样
    Selfcheck_MotorE,               //电机闭环正传启动
    Selfcheck_MotorF,               //电机闭环正传采样
    Selfcheck_MotorG,               //电机闭环反转启动
    Selfcheck_MotorH,               //电机闭环反转采样
    Selfcheck_ServoA,               //舵机测试A
    Selfcheck_Com,                  //通信测试
    Selfcheck_Buzzer,               //蜂鸣器测试
    Selfcheck_RgbLed,               //灯效测试
    Selfcheck_Key,                  //按键测试
    Selfcheck_Finish                //测试完成
}SelfcheckEnum;

typedef struct                              //[智能车驱动主板]
{
    float Voltage;                          //电池电压
    uint8_t Electricity;                    //电池电量百分比：0~100
    float SpeedSet;                         //电机目标速度：m/s
    float SpeedFeedback;                    //电机模型实测速度：m/s
    float SpeedMaxRecords;                  //测试记录最高速
    uint16_t ServoPwmSet;                   //舵机PWM设置

    uint16_t counterKeyA;                   //按键模式A计数器
    bool keyPressed;                        //按键按下
    bool sprintEnable;                      //闭环冲刺使能
    uint16_t counterSprint;                 //闭环冲刺时间
    uint16_t errorCode;                     //错误代码

    bool selfcheckEnable;                   //智能车自检使能
    uint16_t counterSelfcheck;              //自检计数器
    uint8_t timesSendStep;                  //发送超时数据次数
    uint16_t counterModuleCheck;            //自检计数器
    SelfcheckEnum selfcheckStep;            //自检步骤
    uint8_t speedSampleStep;                //速度采样步骤
}IcarStruct;


extern IcarStruct icarStr;
extern PIDStruct pidStr; //用于速度的闭环控制

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
void Serv_Init(void); //舵机相关控制函数
void Serv_set_pwm(uint16_t pwm);
void Icar_init(void);
void Car_control();
void save_servo_freq();
float check_speed();

int read_servo_freq();

extern float speed_init ;
#endif
