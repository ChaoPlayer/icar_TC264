/*
 * connectup.h
 *
 *  Created on: 2023��6��15��
 *      Author: admin
 */

#ifndef CODE_CONNECTUP_H_
#define CODE_CONNECTUP_H_
#include "zf_common_headfile.h"
#include "headfile_code.h"


#define USB_FRAME_HEAD               0x42                //USBͨ������֡ͷ
#define USB_FRAME_LENMIN             4                   //USBͨ�������ֽ���̳���
#define USB_FRAME_LENMAX             12                  //USBͨ�������ֽ������

#define USB_ADDR_HEART               0x00                //����������
#define USB_ADDR_CONTROL             0x01                //���ܳ�����
//#define USB_ADDR_SPEEDMODE           0x02                //�ٿ�ģʽ
#define USB_ADDR_Angle               0x02                //�����ǽǶȻش�
#define USB_ADDR_SERVOTHRESHOLD      0x03                //�����ֵ
#define USB_ADDR_BUZZER              0x04                //��������Ч
#define USB_ADDR_LIGHT               0x05                //LED��Ч
#define USB_ADDR_KEYINPUT            0x06                //��������
#define USB_ADDR_BATTERY             0x07                //�����Ϣ


typedef struct
{
    bool receiveStart;                                      //���ݽ��տ�ʼ
    uint8_t receiveIndex;                                   //��������
    bool receiveFinished;                                   //���ݶ��н��ղ�У�����
    uint8_t receiveBuff[USB_FRAME_LENMAX];                  //USB���ն��У���ʱ����
    uint8_t receiveBuffFinished[USB_FRAME_LENMAX];          //USB���ն��У�У��ɹ�
    uint16_t counter;                                       //������
    uint16_t counterDrop;                                   //���߼�����
    bool connected;                                         //��λ��ͨ������״̬
}UsbStruct;

typedef union
{
    uint8_t U8_Buff[2];
    uint16_t U16;
    int16_t S16;
}Bint16_Union; // �����巽��ȡ��16λ�ĵͺ͸�8λ

typedef union
{
    uint8_t U8_Buff[4];
    float Float;
    unsigned long U32;
}Bint32_Union; //ͬ�� ����ȡ32λ�ĵͺ͸�16λ

typedef enum
{   NOKEY = 0,
    GO ,
    STOP,
} KEYTYPE;




typedef struct
{
   uint8  key_task[4];
   int head ;
   int tail ;
   int lenth ;
}keyTaskQueue;
extern keyTaskQueue keytaskqueue;
extern UsbStruct usbStr;
extern int bee_type;
void usb_edgeboard_rx_handler(void);

void USB_Edgeboard_Init();
void USB_Edgeboard_TransmitByte(uint8_t data);
void USB_Edgeboard_Timr();
void USB_Edgeboard_Handle();
void USB_Edgeboard_send_angle(float angle_intergral);
void USB_Edgeboard_TransmitKey(KEYTYPE);
void key_task_queue_push(KEYTYPE);
int key_task_queue_pop();

#endif /* CODE_CONNECTUP_H_ */
