#include "control.h"


#define SERVOMAXRIGHT 680//700 //pwm 右最大值700
#define SERVOMAXLEFT  1090//1000 //pwm 左最大值1000
#define SERVOMID 880 //880
#define SERVO_ANGLE_MAX 38.0f //来自与赛曙



/****Definitions**********************************************/
StandInfotypedef GyroLoop;
StandInfotypedef AngleLoop;
StandInfotypedef SpeedLoop;

PIDStruct pidStr;     //速度PID结构体
ServoStruct servoStr; // 舵机参数结构体
IcarStruct icarStr;  //智能车总控结构体
/****Variables************************************************/


float Diraction_Out;
float deriv;
float diraction[4];
float SpeedControl;
ServoFreq servo_freq = 0;

float speed_init = 2.3; //25; //2.35;
float kp_init = 7.8; //17; //15; //4.5;

uint8 stop_mode = 0;

float servo_pwm;


/****Function list********************************************/

void Speed_Loop();
/****Objects**************************************************/
Speed_struct speed;
CarInfotypedef CarInfo;

/****Functions************************************************/

//舵机控制
float FloatRangeProtect(float data, float max_out, float min_out)
{
  (data>=max_out)?max_out:data;
  (data<=min_out)?min_out:data;
  
  return data;
}


#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (8)                                // 存储数据用的页码 倒数第一个页码
void save_servo_freq(){
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX)){ // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                // 擦除这一页
    }
    flash_buffer_clear();
    flash_union_buffer[0].int8_type   = servo_freq;                                   // 向缓冲区第 0 个位置写入     int8   数据
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
}

int read_servo_freq(){
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区
    return flash_union_buffer[0].int8_type;
}

void stop(void)
{
    speed.Stan = 0;
    CarInfo.Mode = STAND;//STOPM;
    Serv_set_pwm(SERVOMID);
    now_state= 10;
}

void PID_init(void)
{
    speed.Stan = 0.0;   //设置速度0.8，0.115
    SpeedLoop.BaseKP = 1.53; //0.75
    SpeedLoop.BaseKI = 0.06; //0.1
    SpeedLoop.BaseKD = 0.01; //0


}
void Serv_Init(void){ //舵机初始化
    servoStr.thresholdMiddle =SERVOMID;//机中值880
    servoStr.thresholdLeft = servoStr.thresholdMiddle;
    servoStr.thresholdRight = servoStr.thresholdMiddle;
    if(servo_freq == FREQ50HZ){
           pwm_init(PWM_SERV,50,servoStr.thresholdMiddle); //舵机初始化
       }else{
           pwm_init(PWM_SERV,300,servoStr.thresholdMiddle*6); //舵机初始化
       }
}

void Serv_set_pwm(uint16_t pwm){

if(pwm >  SERVOMAXLEFT){ //舵机限幅
    pwm = SERVOMAXLEFT;

}else if(pwm < SERVOMAXRIGHT){
        pwm = SERVOMAXRIGHT;
}
if(servo_freq == FREQ50HZ){
    pwm_set_duty(PWM_SERV, pwm);
}else{

    pwm_set_duty(PWM_SERV,pwm*6); // 300Hz
}
}

//电机控制
float SpeedLimitMax, SpeedLimitMaxMin;
void Speed_Loop(void)
{

    static float ki_max = 2.0;  //积分量限幅值

    SpeedLoop.ErrorMaxLimit = 2.0;
    SpeedLoop.ErrorMinLimit = -2.0;

    SpeedLoop.OutPutMaxLimit = 5000;
    SpeedLoop.OutPutMinLimit = -5000;

    SpeedLoop.KP = SpeedLoop.BaseKP;
    SpeedLoop.KI = SpeedLoop.BaseKI;
    SpeedLoop.KD = SpeedLoop.BaseKD;

    SpeedLoop.ErrorFifo[2] = SpeedLoop.ErrorFifo[1];
    SpeedLoop.ErrorFifo[1] = SpeedLoop.ErrorFifo[0];
    SpeedLoop.ErrorFifo[0] = SpeedLoop.Error;

    SpeedLoop.ErrorDtFifo[2] = SpeedLoop.ErrorDtFifo[1];
    SpeedLoop.ErrorDtFifo[1] = SpeedLoop.ErrorDtFifo[0];
    SpeedLoop.ErrorDtFifo[0] = SpeedLoop.ErrorFifo[0] - SpeedLoop.ErrorFifo[1];

    SpeedLoop.Error = (speed.Stan - speed_now) ;

    if(SpeedLoop.Error>=SpeedLoop.ErrorMaxLimit) SpeedLoop.Error = SpeedLoop.ErrorMaxLimit;
    if(SpeedLoop.Error<=SpeedLoop.ErrorMinLimit) SpeedLoop.Error = SpeedLoop.ErrorMinLimit;
        SpeedLoop.Integ += SpeedLoop.Error;
    if(SpeedLoop.Integ> ki_max) SpeedLoop.Integ = ki_max;   //对积分量进行限幅
    if(SpeedLoop.Integ<-ki_max) SpeedLoop.Integ = -ki_max;

    SpeedLoop.OutPut = SpeedLoop.KP * SpeedLoop.Error + SpeedLoop.KI * SpeedLoop.Integ + SpeedLoop.KD * (SpeedLoop.ErrorDtFifo[0] * 0.6 + SpeedLoop.ErrorDtFifo[1] * 0.4) ;

    SpeedLoop.OutPut =  SpeedLoop.OutPut * 5000;
        if(SpeedLoop.OutPut>=SpeedLoop.OutPutMaxLimit) SpeedLoop.OutPut = SpeedLoop.OutPutMaxLimit;
        if(SpeedLoop.OutPut<=SpeedLoop.OutPutMinLimit) SpeedLoop.OutPut = SpeedLoop.OutPutMinLimit;
    SpeedControl = ( 0.2 * SpeedLoop.OutPut + 0.8 * SpeedControl);


}

//车辆控制
void Icar_init(void){
    icarStr.Electricity = 0;                    //电量信息
    icarStr.Voltage = 0;                        //电压
    icarStr.SpeedSet = 0.0f;                    //电机目标速度：m/s
    icarStr.SpeedFeedback = 0.0f;               //电机模型实测速度：m/s
    icarStr.SpeedMaxRecords = 0.0f;
    icarStr.ServoPwmSet = servoStr.thresholdMiddle;
}

void Car_control(){

//闭环冲刺

//按键暂停
}

float check_speed(){
    return icarStr.SpeedSet;
}



