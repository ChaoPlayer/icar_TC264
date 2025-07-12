/*
 * connectup.h
 *
 *  Created on: 2023年6月15日
 *      Author: admin
 */

#ifndef CODE_CONNECTUP_H_
#define CODE_CONNECTUP_H_
#include "zf_common_headfile.h"
#include "headfile_code.h"


#define USB_FRAME_HEAD               0x42                //USB通信序列帧头
#define USB_FRAME_LENMIN             4                   //USB通信序列字节最短长度
#define USB_FRAME_LENMAX             12                  //USB通信序列字节最长长度

#define USB_ADDR_HEART               0x00                //监测软件心跳
#define USB_ADDR_CONTROL             0x01                //智能车控制
//#define USB_ADDR_SPEEDMODE           0x02                //速控模式
#define USB_ADDR_Angle               0x02                //陀螺仪角度回传
#define USB_ADDR_SERVOTHRESHOLD      0x03                //舵机阈值
#define USB_ADDR_BUZZER              0x04                //蜂鸣器音效
#define USB_ADDR_LIGHT               0x05                //LED灯效
#define USB_ADDR_KEYINPUT            0x06                //按键输入
#define USB_ADDR_BATTERY             0x07                //电池信息


typedef struct
{
    bool receiveStart;                                      //数据接收开始
    uint8_t receiveIndex;                                   //接收序列
    bool receiveFinished;                                   //数据队列接收并校验完成
    uint8_t receiveBuff[USB_FRAME_LENMAX];                  //USB接收队列：临时接收
    uint8_t receiveBuffFinished[USB_FRAME_LENMAX];          //USB接收队列：校验成功
    uint16_t counter;                                       //计数器
    uint16_t counterDrop;                                   //掉线计数器
    bool connected;                                         //上位机通信连接状态
}UsbStruct;

typedef union
{
    uint8_t U8_Buff[2];
    uint16_t U16;
    int16_t S16;
}Bint16_Union; // 联合体方便取得16位的低和高8位

typedef union
{
    uint8_t U8_Buff[4];
    float Float;
    unsigned long U32;
}Bint32_Union; //同理 方便取32位的低和高16位

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
