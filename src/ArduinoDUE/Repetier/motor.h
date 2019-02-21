#pragma once

enum motor_e {
	M_Z,
	M_Y,
	M_X,
	M_GUARD
};

/* 31 is max current - heatsink could be useful */
#define MOTOR_CURRENT_PROBE     2
#define MOTOR_CURRENT_NORMAL    9


void motorInit(void);
void amisSetCurrent(uint8_t amis, uint8_t current);
void amisEnable(uint8_t amis);
void amisDisable(uint8_t amis);

extern bool usingAmis;