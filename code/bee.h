/*
 * bee.h
 *
 *  Created on: 2023年3月5日
 *      Author: 688
 */

#ifndef CODE_BEE_H_
#define CODE_BEE_H_


#include "zf_common_headfile.h"
#include "headfile_code.h"




typedef enum {
    BUZZER_OK = 0,   // 确认
    BUZZER_WARNNING, // 报警
    BUZZER_FINISH,   // 完成
    BUZZER_DING,     // 提示
    BUZZER_START,    // 开机
}BuzzerEnum;


extern uint8 bee_time;
void bee_init(void);
void bee(void);
void Buzzer_Enable(BuzzerEnum);
#endif /* CODE_BEE_H_ */
