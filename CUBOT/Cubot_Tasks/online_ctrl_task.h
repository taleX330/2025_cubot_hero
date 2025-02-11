#ifndef _ONLINKCTRLTASK_H_
#define _ONLINKCTRLTASK_H_
#include "stm32h7xx.h"

#define OFF_TIME 30

typedef struct
{
    uint8_t dbus;
    uint8_t referee_sys;
    uint8_t wheel[4];
    uint8_t yaw;
    uint8_t pitch;
    uint8_t fric_top;
	uint8_t fric_left;
    uint8_t fric_right;
    uint8_t load;
	uint8_t external_ecd;
    uint8_t super_cap;
    uint8_t vision;
} OnlineFlag_t;

extern OnlineFlag_t onlineFlag;
extern OnlineFlag_t onlineFlagLast;

void OnlineCheckInit(void);
void OnlineCtrl_Task(void);

#endif

