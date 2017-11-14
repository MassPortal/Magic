#include <SPI.h>
#include "Repetier.h"
#include "Motor.h"

#define MOT_REG_GCONF       0x00
#define MOT_REG_GSTAT       0x01
#define MOT_REG_IOIN        0x04
#define MOT_REG_CURRENT     0x10
#define MOT_REG_TCOOLTHRS   0x14
#define MOT_REG_CHOPCONF    0x6C
#define MOT_REG_COOLCONF    0x6D
#define MOT_REG_DCCTRL      0x6E
#define MOT_REG_DRVSTATUS   0x6F

static bool endstops[M_GUARD] = {false, false, false};
static volatile millis_t homeingTime = 0;

void tmc_Z_int(void)
{
    //Endstops::lastState |= ENDSTOP_Z_MAX_ID;
    Serial.println(" Motor Z int");
}


void tmc_Y_int(void)
{
    //Endstops::lastState |= ENDSTOP_Y_MAX_ID;
    Serial.println(" Motor Y int");
}

void tmc_X_int(void)
{
    //Endstops::lastState |= ENDSTOP_X_MAX_ID;
    Serial.println(" Motor X int");
}

static const uint8_t TMC_enable[M_GUARD] = {15, 22, 26};
static const uint8_t TMC_step[M_GUARD] = {2, 17, 24};
static const uint8_t TMC_dir[M_GUARD] = {3, 16, 23};
static const uint8_t TMC_cs[M_GUARD] = {25, 27, 29};
static const uint8_t TMC_int[M_GUARD] = {38, 36, 34};
void(*TMC_handlers[M_GUARD]) (void) = {tmc_Z_int, tmc_Y_int, tmc_X_int};

static void motorPinsInit(void)
{
    /* Set up all pins for all motors */
    for (uint8_t mot = 0; mot < M_GUARD; mot++) {
        //pinMode(TMC_enable[mot], OUTPUT);
        //digitalWrite(TMC_enable[mot], LOW); // Active low
        //pinMode(TMC_dir[mot], OUTPUT);
        //digitalWrite(TMC_dir[mot], LOW); //LOW or HIGH
        //pinMode(TMC_step[mot], OUTPUT);
        //digitalWrite(TMC_step[mot], LOW);
        pinMode(TMC_cs[mot], OUTPUT);
        digitalWrite(TMC_cs[mot], HIGH);
        pinMode(TMC_int[mot], INPUT_PULLUP);
        //attachInterrupt(digitalPinToInterrupt(TMC_int[mot]), TMC_handlers[mot], FALLING);
    }
}


static uint8_t motorPush(motor_e mot,uint8_t reg, uint32_t data)
{
    uint8_t status;

    digitalWrite(TMC_cs[mot], LOW);
#if 0
    status = SPI.transfer(reg);
    SPI.transfer((data>>24UL)&0xFF);
    SPI.transfer((data>>16UL)&0xFF);
    SPI.transfer((data>> 8UL)&0xFF);
    SPI.transfer((data>> 0UL)&0xFF);
#else 
    status = SPI.transfer(reg);
    SPI.transfer((data>>24UL)&0xFF);
    SPI.transfer((data>>16UL)&0xFF);
    SPI.transfer((data>> 8UL)&0xFF);
    SPI.transfer((data>> 0UL)&0xFF);
#endif
    digitalWrite(TMC_cs[mot], HIGH);

    return status;
}

static uint8_t motorWrite(motor_e mot,uint8_t reg, uint32_t data)
{
    return motorPush(mot,  reg | (1<<7), data);
}

static uint8_t motorRead(motor_e mot, uint8_t cmd, uint32_t *data)
{
    uint8_t status;

    motorPush(mot, cmd, 0); //set read address

    digitalWrite(TMC_cs[mot], LOW);
#if 1
    status = SPI.transfer(cmd);
    *data  = SPI.transfer(0);
    *data <<=8;
    *data |= SPI.transfer(0);
    *data <<=8;
    *data |= SPI.transfer(0);
    *data <<=8;
    *data |= SPI.transfer(0);
#else

#endif 
    digitalWrite(TMC_cs[mot], HIGH);

    return status;
}

void motorSetCurrent(motor_e mot, uint8_t run, uint8_t hold, uint8_t holdDly)
{
    run = run > 31 ? 31 : run;
    hold = hold > 31 ? 31 : hold;
    holdDly = holdDly > 15 ? 15 : holdDly;
    motorWrite(mot, MOT_REG_CURRENT, hold | run<<8 | holdDly<<16);
}

void motorSetMicroSteps(motor_e mot, uint8_t uSteps)
{
    if (uSteps >= U_STEPS_GUARD) uSteps = U_STEPS_256;
    motorWrite((motor_e)mot, MOT_REG_CHOPCONF,   uSteps << 24 | 0x8008);
}

void motorInit(void)
{
    static bool first = true;
    uint32_t regVal;
    if (first) {
        motorPinsInit();
        SPI.begin();
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    }
    for (uint8_t mot = 0; mot < M_GUARD; mot++) {
        motorWrite((motor_e)mot, MOT_REG_GCONF, 1<<0 | 1<<4 | 1<<7); // Voltage on AIN is current reference & enable interrupts
        motorRead((motor_e)mot, MOT_REG_GCONF, &regVal);
        if (regVal != (1 << 0 | 1 << 4 | 1 << 7)) {
            Serial.print("Motor regwrite fail");
            Serial.println(regVal);
        }
        motorWrite((motor_e)mot, MOT_REG_TCOOLTHRS, 0xfffff); // Always on coolstep & stallguard
        motorWrite((motor_e)mot, MOT_REG_COOLCONF, 0); // Always on coolstep & stallguard
        motorSetCurrent((motor_e)mot, 11, 31, 2);
        motorSetMicroSteps((motor_e)mot, U_STEPS_32);
    }
}

void motorClearInt(void)
{
    uint32_t data;
    uint8_t status;

    for (uint8_t i=0; i<M_GUARD; i++) {
        if (digitalRead(TMC_int[i])) continue;
        status = motorRead((motor_e)i, MOT_REG_GSTAT, &data);
        if (data & (1 << 0) || status & (1 << 0)) motorInit();
#warning this data might be useful
        return; 
        if (data & (1 << 0)) Serial.println("Motor reset");
        if (data & (1 << 1)) Serial.println("Overcurrent or overtemp");
        if (data & (1 << 2)) Serial.println("Undervoltage");

        if(status & (1 << 0)) Serial.println("Stat reset");
        if(status & (1 << 1)) Serial.println("Stat error");
        if(status & (1 << 2)) Serial.println("Stat stallguard");
        if(status & (1 << 3)) Serial.println("Stat standstill");
    }
    /* Clear enstop falgs */
    //Endstops::lastState &= ~(ENDSTOP_Z_MAX_ID|ENDSTOP_Y_MAX_ID|ENDSTOP_X_MAX_ID);
}

void startHomeing(void)
{
    homeingTime = millis();
}

void clearHomeing(void)
{
    homeingTime = 0;
}

bool checkHomeing(void)
{
    return (homeingTime && homeingTime + 80 < millis()) ? true : false;
}