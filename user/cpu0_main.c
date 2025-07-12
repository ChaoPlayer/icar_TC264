
#include "zf_common_headfile.h"
#include "headfile_code.h"
#pragma section all "cpu0_dsram"


// **************************** 代码区域 ****************************
    int datif;
    uint8 dat;
#define TASK_ENABLE 0

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    ips200_init(IPS200_TYPE_PARALLEL8); //初始化显示屏
//    gpio_init(P21_5, GPO,0, GPO_PUSH_PULL);    //初始化引脚用于闪烁，判断程序是否卡死

    speedcount_init(); //初始化编码器
    USB_Edgeboard_Init();
    servo_freq = read_servo_freq();
    Serv_Init(); //舵机参数初始化
    MotorInit();       //初始化电机控制
    adc_init(ADC0_CH8_A8, ADC_8BIT); //初始化电压采集
//
    bee_init(); //初始化蜂鸣器  用法，bee_time = 100; 就是蜂鸣器响100ms
    key_init(10); //按键初始化
    Butterworth_Parameter_Init(); //低通滤波器初始化
    PID_init(); //控制参数初始化

    PIT_init();  //中断初始化
//
      imu_init();
      pit_ms_init(CCU61_CH0,1);
      pit_ms_init(CCU61_CH1,5);
      Icar_init(); //参数初始化
    bee_time = 100;
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    ips200_clear();

    int first_send = 0;
    uint32_t last_system_ms;

    //encoder_clear_count(TIM2_ENCOEDER);

	while (TRUE)
	{

       key_process();
       if(key_press[2]){
       servo_freq = FREQ50HZ;
        save_servo_freq();
       pwm_init(PWM_SERV,50,servoStr.thresholdMiddle); //舵机初始化

       }else if (key_press[3]){
        servo_freq = FREQ300HZ;
        save_servo_freq();
         pwm_init(PWM_SERV,300,servoStr.thresholdMiddle*6); //舵机初始化
         }


       UI_Disp();
       //encoder_check_init();
       //encoder_check();


       int kt = key_task_queue_pop();
       if(kt !=0){
          USB_Edgeboard_TransmitKey((KEYTYPE)kt);
       }
       USB_Edgeboard_Handle();
       //USB_Edgeboard_send_angle(imudata.TurnAngle_Integral);

       //USB_Edgeboard_send_angle(imudata.yaw);




	}

}

#pragma section all restore
