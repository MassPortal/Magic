#pragma once

enum motor_e {
    M_Z,
    M_Y,
    M_X,
    M_GUARD
};

/* Holding currents */
#define MOTOR_CURRENT_HOLD      29
#define MOTOR_CURRENT_STBY      0
/* 31 is max current - heatsink could be useful */
#define MOTOR_CURRENT_PROBE     9
#define MOTOR_CURRENT_NORMAL    30
/* When starting a move ignore this many ms */
#define MOTOR_STALL_DELAY       80

void motorInit(void);
void tmcSetCurrent(uint8_t mot, uint8_t run, uint8_t hold, uint8_t holdDly);
void startProbeing(void);
void clearProbeing(void);
bool checkProbeing(void);
