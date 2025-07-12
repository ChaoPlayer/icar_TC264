/*
 * bee.h
 *
 *  Created on: 2023��3��5��
 *      Author: 688
 */

#ifndef CODE_BEE_H_
#define CODE_BEE_H_


#include "zf_common_headfile.h"
#include "headfile_code.h"




typedef enum {
    BUZZER_OK = 0,   // ȷ��
    BUZZER_WARNNING, // ����
    BUZZER_FINISH,   // ���
    BUZZER_DING,     // ��ʾ
    BUZZER_START,    // ����
}BuzzerEnum;


extern uint8 bee_time;
void bee_init(void);
void bee(void);
void Buzzer_Enable(BuzzerEnum);
#endif /* CODE_BEE_H_ */
