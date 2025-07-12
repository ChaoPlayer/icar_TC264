#include "encode.h"
#include "math.h"

float speed_now;
uint32 encoder_count;
int16 v_count;
void speedcount_init(void)
{
    //encoder_quad_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);
    encoder_quad_init(TIM2_ENCOEDER,TIM2_ENCOEDER_CH1_P33_7,TIM2_ENCOEDER_CH2_P33_6);
}

void GetSpeed()
{
    static int16 v_filter[4];

    v_count= encoder_get_count(TIM2_ENCOEDER);

    motorStr.EncoderValue = v_count;
    encoder_clear_count(TIM2_ENCOEDER);

//    if(motorStr.EncoderValue > 32767) motorStr.EncoderValue = motorStr.EncoderValue - 65536; //限幅


   //计算实际速度    ---     m/s


    v_filter[0]=v_count;
    for (int i = 3; i > 0; i--)
    {
        v_filter[i] = v_filter[i - 1];
    }

    speed_now = (speed_now*0.15 + 0.85*(0.4*v_filter[0]+0.3*v_filter[1]+0.2*v_filter[2]+0.1*v_filter[3]) * SPEED_F  / v_QD_UNIT);    //求出车速转换为M/S
    icarStr.SpeedFeedback = speed_now;
    encoder_count+= v_count;
    if (speed_now > 8)speed_now = 8;//限速



}

