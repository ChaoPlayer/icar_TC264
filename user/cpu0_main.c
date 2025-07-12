
#include "zf_common_headfile.h"
#include "headfile_code.h"
#pragma section all "cpu0_dsram"


// **************************** �������� ****************************
    int datif;
    uint8 dat;
#define TASK_ENABLE 0

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������
    ips200_init(IPS200_TYPE_PARALLEL8); //��ʼ����ʾ��
//    gpio_init(P21_5, GPO,0, GPO_PUSH_PULL);    //��ʼ������������˸���жϳ����Ƿ���

    speedcount_init(); //��ʼ��������
    USB_Edgeboard_Init();
    servo_freq = read_servo_freq();
    Serv_Init(); //���������ʼ��
    MotorInit();       //��ʼ���������
    adc_init(ADC0_CH8_A8, ADC_8BIT); //��ʼ����ѹ�ɼ�
//
    bee_init(); //��ʼ��������  �÷���bee_time = 100; ���Ƿ�������100ms
    key_init(10); //������ʼ��
    Butterworth_Parameter_Init(); //��ͨ�˲�����ʼ��
    PID_init(); //���Ʋ�����ʼ��

    PIT_init();  //�жϳ�ʼ��
//
      imu_init();
      pit_ms_init(CCU61_CH0,1);
      pit_ms_init(CCU61_CH1,5);
      Icar_init(); //������ʼ��
    bee_time = 100;
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
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
       pwm_init(PWM_SERV,50,servoStr.thresholdMiddle); //�����ʼ��

       }else if (key_press[3]){
        servo_freq = FREQ300HZ;
        save_servo_freq();
         pwm_init(PWM_SERV,300,servoStr.thresholdMiddle*6); //�����ʼ��
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
