#include "connectup.h"
// 连接到上位机的库

UsbStruct usbStr;
keyTaskQueue keytaskqueue;

// USB/UART 初始化

void USB_Edgeboard_Init(void){

int baund = 115200;

uart_init(UART_2,baund,UART2_TX_P10_5, UART2_RX_P10_6); //初始化发射和接收端口
uart_rx_interrupt(UART_2, 1); //开启接收中断
//uart_tx_interrupt(UART_2, 1); //开启发送中断
//USB数据初始化
usbStr.counter = 0;
usbStr.receiveFinished = false;
usbStr.receiveStart = false;
usbStr.receiveIndex = 0;
usbStr.connected = false;
keytaskqueue.head =0;
keytaskqueue.tail = 0;
keytaskqueue.lenth = 4;

int i = 4;
for(i = 0; i<4 ;i++){
    keytaskqueue.key_task[i] = 0;
}

}
/**
* @brief        USB-Edgeboard发送一个字节数据
* @param
* @ref
* @note
**/
void USB_Edgeboard_TransmitByte(uint8_t data)
{
    uart_write_byte(UART_2, data);
}

void usb_edgeboard_transmit_buff(const uint8 *buff, uint32 len){
    uart_write_buffer(UART_2,buff,len);
}

void usb_edgeboard_rx_handler(void){ //处理接收中断
    uint8 UartRes = 0;
//    UartRes = uart_read_byte(UART_2);
    uart_query_byte(UART_2,&UartRes);
    if(UartRes == USB_FRAME_HEAD && !usbStr.receiveStart)//监测帧头
    {
        usbStr.receiveStart = true;
        usbStr.receiveBuff[0] = UartRes;
        usbStr.receiveBuff[2] = USB_FRAME_LENMIN;
        usbStr.receiveIndex = 1;
    }
    else if(usbStr.receiveIndex == 2)   //接收帧长度
            {
                usbStr.receiveBuff[usbStr.receiveIndex] = UartRes;
                usbStr.receiveIndex++;

                if(UartRes > USB_FRAME_LENMAX || UartRes < USB_FRAME_LENMIN) //帧长错误
                {
                    usbStr.receiveBuff[2] = USB_FRAME_LENMIN;
                    usbStr.receiveIndex = 0;
                    usbStr.receiveStart = false;
                }
    }
    else if(usbStr.receiveStart && usbStr.receiveIndex < USB_FRAME_LENMAX)
    {
        usbStr.receiveBuff[usbStr.receiveIndex] = UartRes;
        usbStr.receiveIndex++;
    }

    //接收帧完毕
            if((usbStr.receiveIndex >= USB_FRAME_LENMAX || usbStr.receiveIndex >= usbStr.receiveBuff[2]) && usbStr.receiveIndex > USB_FRAME_LENMIN)
            {
                uint8_t check = 0;
                uint8_t length = USB_FRAME_LENMIN;

                length = usbStr.receiveBuff[2];
                for(int i=0;i<length-1;i++)
                    check += usbStr.receiveBuff[i];
                if(check == usbStr.receiveBuff[length-1])//校验位
                {
                    memcpy(usbStr.receiveBuffFinished,usbStr.receiveBuff,USB_FRAME_LENMAX);
                    usbStr.receiveFinished = true;

                    //智能车控制指令特殊处理（保障实时性）
                    if(USB_ADDR_CONTROL  == usbStr.receiveBuffFinished[1])
                    {
                        Bint16_Union bint16_Union;
                        Bint32_Union bint32_Union;
                        for(int i=0;i<4;i++)
                            bint32_Union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];

                        bint16_Union.U8_Buff[0] = usbStr.receiveBuffFinished[7];
                        bint16_Union.U8_Buff[1] = usbStr.receiveBuffFinished[8];
                        icarStr.ServoPwmSet = bint16_Union.U16;         //方向
                        Serv_set_pwm(icarStr.ServoPwmSet);
                        icarStr.SpeedSet = bint32_Union.Float;          //速度
                        speed.Stan = icarStr.SpeedSet;
                    }
                    if(!usbStr.connected)//上位机初次连接通信
                    {
                        Buzzer_Enable(BUZZER_START);
                        usbStr.connected = true;
                    }

                    usbStr.counterDrop = 0;
                }

                usbStr.receiveIndex = 0;
                usbStr.receiveStart = false;
            }
}


void USB_Edgeboard_Timr(void)
{
    if(usbStr.connected)//USB通信掉线检测
    {
        usbStr.counterDrop++;
        if(usbStr.counterDrop >3000/2)//3s
        {
            usbStr.connected = false;
            icarStr.selfcheckEnable = false;
            stop();
        }

    }
}
/**
* @brief        USB通信处理函数
* @param
* @ref
* @note
**/
int bee_type = 0;
void USB_Edgeboard_Handle(void)
{

    if(usbStr.receiveFinished)                                                              //接收成功
    {
        usbStr.receiveFinished = false;
        Bint32_Union bint32_Union;
        Bint16_Union bint16_Union;
        switch(usbStr.receiveBuffFinished[1])
          {
          case USB_ADDR_BUZZER :
          bee_type= usbStr.receiveBuffFinished[3];//蜂鸣器音效 //获取目前元素的状态
          if(bee_type == BUZZER_START)          //START
            {
             Buzzer_Enable(BUZZER_START);
            }else if(bee_type == BUZZER_WARNNING)     //WARNNING
             {
                Buzzer_Enable(BUZZER_WARNNING);
             }else if(bee_type == BUZZER_FINISH)     //Finish
             {
                 Buzzer_Enable(BUZZER_FINISH);
              }else {
               Buzzer_Enable(BUZZER_DING);
             }
          break;

    }
}
}

/**
* @brief        USB发送按键信号
* @param        time: 按键时长
* @ref
* @author       ..5
* @note
**/
void USB_Edgeboard_TransmitKey(KEYTYPE key)
{

    uint8_t check = 0;
    uint8_t buff[8];
    Bint16_Union bint16_Union;
    buff[0] = 0x42; //帧头
    buff[1] = USB_ADDR_KEYINPUT; //地址
    buff[2] = 0x06; //帧长
    bint16_Union.U16 = key;//key*200;
    buff[3] = bint16_Union.U8_Buff[0];
    buff[4] = bint16_Union.U8_Buff[1];
    for(int i=0;i<5;i++){
        check += buff[i];
    }

    buff[5] = check;
//    for(int i=0;i<8;i++){
//        USB_Edgeboard_TransmitByte(buff[i]);
//    }
    usb_edgeboard_transmit_buff(buff,8);

}

void USB_Edgeboard_send_angle(float angle_intergral){
    uint8_t check = 0;
    uint8_t buff[8];
    Bint32_Union bint32_Union;
    buff[0] = 0x42; //帧头
    buff[1] = USB_ADDR_Angle; //地址
    buff[2] = 0x08; //帧长
    bint32_Union.Float = angle_intergral;// 发送角度
    buff[3] = bint32_Union.U8_Buff[0];
    buff[4] = bint32_Union.U8_Buff[1];
    buff[5] = bint32_Union.U8_Buff[2];
    buff[6] = bint32_Union.U8_Buff[3];
    for(int i=0;i<7;i++){
            check += buff[i];
        }
    buff[7] = check;
//    for(int i=0;i<8;i++){
//        USB_Edgeboard_TransmitByte(buff[i]);
//    }
    usb_edgeboard_transmit_buff(buff,8);

}


void key_task_queue_push(KEYTYPE kt){
        keytaskqueue.key_task[keytaskqueue.head] = (uint8)kt;
        keytaskqueue.head = (keytaskqueue.head+1)%keytaskqueue.lenth;
}
int key_task_queue_pop(){
    int ret = 0;
    if(keytaskqueue.head != keytaskqueue.tail){
        ret = keytaskqueue.key_task[keytaskqueue.tail];
        keytaskqueue.tail = (keytaskqueue.tail+1)%keytaskqueue.lenth;
    }
    return ret;
}



