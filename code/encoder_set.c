#include"encode.h"
#include"math.h"
#include"zf_driver_encoder.h"
#include"zf_device_ips200.h"
#include "headfile_code.h"
#include "control.h"
#include"encode.h"

int16 count_check;

void encoder_check(){
    count_check=encoder_get_count(TIM2_ENCOEDER);
    ips200_show_string(10,200,"Pulse:");
    ips200_show_uint(70,200,count_check,10);
    ips200_show_string(10,100,"encoder:");
    ips200_show_uint(70,100,encoder_count,10);
}

void encoder_check_init(){
    encoder_quad_init(TIM2_ENCOEDER,TIM2_ENCOEDER_CH1_P33_7,TIM2_ENCOEDER_CH2_P33_6);
    //encoder_clear_count(TIM2_ENCOEDER);
}
