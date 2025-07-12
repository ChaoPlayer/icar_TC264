#include "zf_common_headfile.h"
#include "headfile_code.h"
#include "UI.h"
#include "isr.h"

int now_state = 0;
int last_state = 0;
void key_process(void);
unsigned char ui_page0[10][17] =
{
    "   Showimage    ",
    "   Speed        ",
    "   encoder      ",
    "   OUT_PID      ",
    "   IN_PID       ",
    "   MotorPID     ",
    "   Angle        ",
    "                ",
    "                ",
    "   Votage:      "
};

unsigned char ui_page1[10][17] =
{
    " pageChange     ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    " page2          "
};

unsigned char ui_page2[10][17] =
{
    " pageChange     ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    " page3          "
};

unsigned char ui_page3[10][17] =
{
    " pageChange     ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    " page4          "
};

unsigned char ui_Sensor[10][17] =
{
    "   L :     ",
    "   SL:     ",
    "   M :     ",
    "   SR:     ",
    "   R :     ",
    "   LL:     ",
    "   RR:     ",
    "   HW:     ",
    "   err:    ",
    "   Sensor  "
};

unsigned char ui_Speed[10][17] =
{
    "   SPEED        ",
    "                ",
    "   SPEED_SET    ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};

unsigned char ui_encoder[10][17] =
{
    "   v_Count:     ",
    "                ",
    "       ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};

unsigned char ui_OUTPD[10][17] =
{
    "   OUTP:        ",
    "                ",
    "   OUTD:        ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};
unsigned char ui_INPD[10][17] =
{
    "   INP:        ",
    "                ",
    "   IND:        ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};
unsigned char ui_MotorPID[10][17] =
{
    "   MotorP:      ",
    "                ",
    "   MotorI:      ",
    "                ",
    "   MotorD:      ",
    "                ",
    "   SPEED        ",
    "                ",
    "   SPEED_SET    ",
    "                "
};
unsigned char ui_angle[10][17] =
{
    "   Angle_Y:     ",
    "   Angle_Z:     ",
    "   SPEED_Z      ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};

UI_CLASS ui =
{
    &UI_Disp,
    {0, 0, 0, 0}, 0, -1
};


void num_add(float* num, float step)
{
*num += step;
}

void num_sub(float* num, float step)
{
*num -= step;
if(*num<0)
    *num = 0;
}
void show_sys_state(uint16 x , uint16 y){
    ips200_show_string(x,y, "bee_type:");
    switch(bee_type){
        case 0 :
            ips200_show_string(x+100,y, "OK");
            break;
        case 1:
            ips200_show_string(x+100,y, "Warning");
            break;
        case 2:
            ips200_show_string(x+100,y, "Finish");
            break;
        case 3:
            ips200_show_string(x+100,y, "Ding");
            break;
        case 4:
            ips200_show_string(x+100,y, "Start");
            break;
    }
}

uint8 key_press[4] = {0};

#define FIX_VOLTAGE 1.03

void show_info(void)
{
   ips200_show_string(20,20,"speed_now");
   ips200_show_float(120, 20, speed_now, 4, 4);

   ips200_show_string(20,40,"vcnt");
   ips200_show_int(80, 40, v_count, 4);

   ips200_show_string(20,60,"ServPwmSet");
   ips200_show_int(120, 60,icarStr.ServoPwmSet,4);

   ips200_show_string(20,80,"SpeedSet");
   ips200_show_float(130, 80,icarStr.SpeedSet, 4, 2);


   ips200_show_string(20,100,"Speedstan");
   ips200_show_float(130, 100,speed.Stan, 4, 2);

   ips200_show_string(20,120,"KeyShortPress:");
   ips200_show_int  (140, 120,key_press[0],2);
   ips200_show_int  (160, 120,key_press[1],2);
   ips200_show_string(20,140,"KeyLongPress:");
    ips200_show_int  (140, 140,key_press[2],2);
    ips200_show_int  (160, 140,key_press[3],2);
   ips200_show_string(20,160,"Yaw:");
   ips200_show_string(20,180,"Pitch:");
   ips200_show_string(20,200,"Roll:");

   ips200_show_float(120, 160,imudata.yaw, 4, 4);
   ips200_show_float(120, 180,imudata.pitch, 4, 4);
   ips200_show_float(120, 200,imudata.roll, 4,4);
//      ips200_show_float(120, 160,imudata.CarAngle , 4, 4);
//      ips200_show_float(120, 180,imudata.pitch, 4, 4);
//      ips200_show_float(120, 200,imudata.TurnAngle_Integral, 4,4);



   ips200_show_string(20,220,"encoder:");
   ips200_show_int(120,220,encoder_count,6);

//   ips200_show_int  (140, 120,key_press[2],2);
//   ips200_show_int  (160, 120,key_press[3],2);
   ips200_show_string(20,240,"Voltage:  ");
   ips200_show_float(120, 240, (float)(3.3*4*adc_mean_filter_convert(ADC0_CH8_A8, 5)/256)*FIX_VOLTAGE, 2, 2);

   ips200_show_string(20,260,"ServoFreq:  ");
   if(servo_freq == FREQ50HZ){
       ips200_show_string(120,260,"50HZ");
   }else{
       ips200_show_string(120,260,"300HZ");
   }
}
void UI_Disp(void)
{
    show_info();

}


void key_process(void)
{
        //P22_0,P22_1,P22_2,P22_3
        key_press[0] =(uint8)(key_get_state(KEY_1) == KEY_SHORT_PRESS);
        key_press[1] =(uint8)(key_get_state(KEY_2) == KEY_SHORT_PRESS);
        key_press[2] =(uint8)(key_get_state(KEY_1) == KEY_LONG_PRESS);
        key_press[3] =(uint8)(key_get_state(KEY_2) == KEY_LONG_PRESS);
        if(key_press[0]){
            key_task_queue_push(GO);
        }else if(key_press[1]){
            key_task_queue_push(STOP);
        }
}


