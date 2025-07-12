/*
 * bee.c
 *
 *  Created on: 2023年3月5日
 *      Author: 688
 */
#include "bee.h"


uint8 bee_time;
int bee_last = 0;


void bee_init(void)
{
    bee_time = 0;
    gpio_init(P33_10, GPO,0, GPO_PUSH_PULL);
}

void bee(void)
{
    if(bee_time)
    {
        bee_time--;
        gpio_set_level(P33_10, 1);
    }
    else
    {
        gpio_set_level(P33_10, 0);
    }
}

void Buzzer_Enable(BuzzerEnum buzzer) //几个蜂鸣器模式
{

    switch(buzzer)
    {
        case BUZZER_OK:
            bee_time=70;       //70ms
            break;

        case BUZZER_WARNNING:
            bee_time = 100;        //100ms
            break;

        case BUZZER_FINISH:
            bee_time = 60;         //60ms
            break;
        case BUZZER_DING:
            bee_time = 30;         //30ms
            break;
        case BUZZER_START:
            bee_time = 200;        //200ms
            break;
    }

}

