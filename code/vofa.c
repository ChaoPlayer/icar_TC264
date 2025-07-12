#include "vofa.h"

#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;


void vofa_start(void)
{
    vofa_demo();        // demoʾ��
}

void vofa_transmit(uint8_t* buf, uint16_t len)
{
    //CDC_Transmit_FS((uint8_t *)buf, len);
}

void vofa_send_data(uint8_t num, float data)
{
    send_buf[cnt++] = byte0(data);
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
}

void vofa_sendframetail(void)
{
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;

    /* �����ݺ�֡β������� */
    vofa_transmit((uint8_t *)send_buf, cnt);
    cnt = 0;// ÿ�η�����֡β����Ҫ����
}

void vofa_demo(void)
{

    // Call the function to store the data in the buffer
//    vofa_send_data(0, pitch);
//    vofa_send_data(1, roll);
//    vofa_send_data(2, yaw);
//    vofa_send_data(3, pitch_mahony);
//    vofa_send_data(4, roll_mahony);
//    vofa_send_data(5, yaw_mahony);
//    vofa_send_data(6, temp);
    // Call the function to send the frame tail
    vofa_sendframetail();
}

