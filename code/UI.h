#ifndef _UI_h
#define _UI_h
#include "zf_common_headfile.h"
#include "connectup.h"
typedef struct
{
    void (*Disp)(void);
    int  cursor[4], page, enter;
} UI_CLASS;
#define READY "Ready" //0
#define START "SystemStart" //1
#define BRIDGE "Bridge"     //2
#define DEPOT  "Depot" //Î¬ÐÞ³§ 3
#define FARMLAND "Farmland"//4
#define SLOWZONE "SlowZone"//5
#define GRANARY "Granary"//6
#define FREEZONE "Freezone"//7
#define RING "Ring"//8
#define CROSS "Cross"//9
#define ERROR "SystemError"//10
#define FINISH "SystemStop"//11


void UI_Disp(void);
void show_info(void);
void key_process(void);
static void UI_DispUIStrings(uint8 strings[10][17]);
extern int datif;
extern uint8 dat;
extern int now_state;
extern uint8 key_press[4];

#endif

