#include <SPI.h>
#include "Repetier.h"
#include "Motor.h"
#include "HAL.h"

#define PIN_MOT_CSX	X_ENABLE_PIN
#define PIN_MOT_CSY	Y_ENABLE_PIN
#define PIN_MOT_CSZ	Z_ENABLE_PIN

#define AMIS_REG_WR		0x00
#define AMIS_REG_CR0	0x01
#define AMIS_REG_CR1	0x02
#define AMIS_REG_CR2	0x03
#define AMIS_REG_CR3	0x09

#define AMIS_REG_SR0	0x04
#define AMIS_REG_SR1	0x05
#define AMIS_REG_SR2	0x06
#define AMIS_REG_SR3	0x07
#define AMIS_REG_SR4	0x0A

static volatile millis_t probingTime = 0;

static uint32_t pinCs[3] = { PIN_MOT_CSX ,PIN_MOT_CSY ,PIN_MOT_CSZ };
bool usingAmis = false;

static void motorPinsInit(void)
{
	SPI.begin();
	SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));
	for (uint32_t amis=0; amis<M_GUARD; amis++) {
		pinMode(pinCs[amis], OUTPUT);
		digitalWrite(pinCs[amis], HIGH);
	}
}

static uint8_t amisPush(uint8_t amis, uint8_t reg, uint8_t val)
{
	uint8_t tmp;
	/* Select chip */
	digitalWrite(pinCs[amis], LOW);
	/* Send reg */
	SPI.transfer(reg);
	/* Send data */
	tmp = SPI.transfer(val);
	/* Unselect chip */
	digitalWrite(pinCs[amis], HIGH);
	return tmp;
}

static void amisRegWrite(uint8_t amis, uint8_t reg, uint8_t val)
{
	/* Set write bit */
	amisPush(amis, reg | (1<<7), val);
}

static uint8_t amisRegRead(uint8_t amis, uint8_t reg)
{
	return amisPush(amis, reg, 0xff);
}

void amisSetCurrent(uint8_t amis, uint8_t current)
{
	if (current < 32) {
		/* 31 == max */
		amisRegWrite(amis, AMIS_REG_CR0, current);
	}
}

void amisEnable(uint8_t amis)
{
	amisRegWrite(amis, AMIS_REG_CR2, (1<<7));
}

void amisDisable(uint8_t amis)
{
	amisRegWrite(amis, AMIS_REG_CR2, 0);
}

void motorInit(void)
{
	motorPinsInit();
	SPI.begin();
	SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));
	for (uint8_t amis=0; amis<M_GUARD; amis++) {
		amisSetCurrent(amis, MOTOR_CURRENT_NORMAL);
	}
}
