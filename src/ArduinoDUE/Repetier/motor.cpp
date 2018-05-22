#include <SPI.h>
#include "Repetier.h"
#include "Motor.h"
#include "HAL.h"

/* Driver registers */
#define REG_GCONF       0x00
#define REG_GSTAT       0x01
#define REG_IOIN        0x04
#define REG_IHOLD_IRUN  0x10
#define REG_TPOWERDOWN  0x11
#define REG_TSTEP       0x12
#define REG_TPWMTHRS    0x13
#define REG_TCOOLTHRS   0x14
#define REG_THIGH       0x15
#define REG_XDIRECT     0x2d
#define REG_VDCMIN      0x33
#define REG_MSLUT0      0x60
#define REG_MSLUT1      0x61
#define REG_MSLUT2      0x62
#define REG_MSLUT3      0x63
#define REG_MSLUT4      0x64
#define REG_MSLUT5      0x65
#define REG_MSLUT6      0x66
#define REG_MSLUT7      0x67
#define REG_MSLUTSEL    0x68
#define REG_MSLUTSTART  0x69
#define REG_MSCNT       0x6a
#define REG_MSCURACT    0x6b
#define REG_CHOPCONF    0x6c
#define REG_COOLCONF    0x6d
#define REG_DCCTRL      0x6e
#define REG_DRV_STATUS  0x6f
#define REG_PWMCONF     0x70
#define REG_PWM_SCALE   0x71
#define REG_ENCM_CTRL   0x72
#define REG_LOST_STEPS  0x73

/* Packed for 8bits (the more you know) */
typedef enum {
    USTEPS_256,
    USTEPS_128,
    USTEPS_64,
    USTEPS_32,
    USTEPS_16,
    USTEPS_8,
    USTEPS_4,
    USTEPS_2,
    USTEPS_1
} __attribute__((packed)) usteps_e;

static volatile millis_t probeingTime = 0;

static const int8_t TMC_cs[M_GUARD] = {25, 27, 29};
static const int8_t TMC_int[M_GUARD] = {ORIG_Z_MIN_PIN, ORIG_Y_MIN_PIN, ORIG_X_MIN_PIN};

static inline usteps_e ustepsToReg(uint16_t usteps)
{
    switch(usteps) {
    case 256:
        return USTEPS_256;
    case 128:
        return USTEPS_128;
    case 64:
        return USTEPS_64;
    case 32:
        return USTEPS_32;
    case 16:
        return USTEPS_16;
    case 8:
        return USTEPS_8;
    case 4:
        return USTEPS_4;
    case 2:
        return USTEPS_2;
    case 1:
        return USTEPS_1;
    default:
        return USTEPS_32;
    }
}

static void motorPinsInit(void)
{
    /* Set up all pins for all motors */
    for (uint8_t mot=0; mot<M_GUARD; mot++) {
        if (TMC_cs[mot] > -1) {
            pinMode(TMC_cs[mot], OUTPUT);
            digitalWrite(TMC_cs[mot], HIGH);
        }
        if (TMC_int[mot] > -1) {
            pinMode(TMC_int[mot], INPUT_PULLUP);
        }
   }
}

static uint8_t tmcPush(uint8_t mot, uint8_t reg, uint32_t data)
{
    uint8_t status;

    digitalWrite(TMC_cs[mot], LOW);
    status = SPI.transfer(reg);
    SPI.transfer((data>>24UL)&0xFF);
    SPI.transfer((data>>16UL)&0xFF);
    SPI.transfer((data>>8UL)&0xFF);
    SPI.transfer((data>>0UL)&0xFF);
    digitalWrite(TMC_cs[mot], HIGH);

    return status;
}

static uint8_t tmcWrite(uint8_t mot, uint8_t reg, uint32_t data)
{
    return tmcPush(mot,  reg | (1<<7), data);
}

static uint8_t tmcRead(uint8_t mot, uint8_t cmd, uint32_t *data)
{
    uint8_t status;

    tmcPush(mot, cmd, 0); // Set read address

    digitalWrite(TMC_cs[mot], LOW);
    status = SPI.transfer(cmd);
    *data  = SPI.transfer(0);
    *data <<=8;
    *data |= SPI.transfer(0);
    *data <<=8;
    *data |= SPI.transfer(0);
    *data <<=8;
    *data |= SPI.transfer(0);
    digitalWrite(TMC_cs[mot], HIGH);

    return status;
}

void tmcSetCurrent(uint8_t mot, uint8_t run, uint8_t hold, uint8_t holdDly)
{
    run = run > 0x1f ? 0x1f : run;
    hold = hold > 0x1f ? 0x1f : hold;
    holdDly = holdDly > 0xf ? 0xf : holdDly;
    tmcWrite(mot, REG_IHOLD_IRUN, hold | (uint32_t)run<<8 | (uint32_t)holdDly<<16);
}

static void tmcInit(uint8_t mot, bool first)
{
    uint32_t reg, compa;
    reg = 0;
    reg |= ((uint32_t)(3 & 0xful) << 0);                            // Setup TOFF
    reg |= ((uint32_t)(5 & 0x7ul) << 4);                            // Setup VOID hstrt
    reg |= ((uint32_t)(1 & 0xful) << 7);                            // Setup VOID hend
    reg |= ((uint32_t)(2 & 0x3ul) << 15);                           // Setup VOID TBL
    reg |= ((uint32_t)(1 & 0x1ul) << 17);                           // Setup vsense
    reg |= ((uint32_t)(ustepsToReg(MICRO_STEPS) & 0xful) << 24);    // Setup microsteps
    reg |= ((uint32_t)(1 & 0x1ul) << 28);                           // Setup interpol
    reg |= ((uint32_t)(0 & 0x1ul) << 29);                           // NOT Step on toggle
    if (!first) {
        tmcRead(mot, REG_GSTAT, &compa);
#warning kill maybe?
        if (compa & 1<<1) {
            /* This is bad, very bad, very very bad */
            if (mot == M_Z) Printer::disableZStepper();
            else if (mot == M_Y) Printer::disableYStepper();
            else if (mot == M_X) Printer::disableXStepper();
            delay(1);
            if (mot == M_Z) Printer::enableZStepper();
            else if (mot == M_Y) Printer::enableYStepper();
            else if (mot == M_X) Printer::enableXStepper();
            delay(1);
        } else {
            /* Must read to clear, dont care about contents */
            tmcRead(mot, REG_DRV_STATUS, &compa);
            tmcRead(mot, REG_CHOPCONF, &compa);
            /* All is good - motors haven't been reset*/
            if (compa == reg) return;
        }
    }
    tmcWrite(mot, REG_CHOPCONF, reg);                                   // Write the damn thing
    tmcSetCurrent(mot, MOTOR_CURRENT_NORMAL, MOTOR_CURRENT_HOLD, 2);    // Setup currents
    tmcWrite(mot, REG_THIGH, 0);                                        // Never make fullsteps
    tmcWrite(mot, REG_TPOWERDOWN, 0);                                   // power down never
    tmcWrite(mot, REG_COOLCONF, 0 << 16);                               // Stall guard threshold - default
    tmcWrite(mot, REG_TCOOLTHRS, 0);                                    // Setup coolstep threshold (no coolstep/stallguard)
    reg = 0;
    reg |= ((uint32_t)(1 & 0x1ul) << 2);            // Setup stealthChop
    reg |= ((uint32_t)(1 & 0x1ul) << 4);            // Inverse
    reg |= ((uint32_t)(1 & 0x1ul) << 7);            // Enable diag0 interrupt - OD config
    tmcWrite(mot, REG_GCONF, reg);                  // Write main setup
    reg = 0;
    reg |= ((uint32_t)(0xff & 0xfful) << 0);        // Pwm amplitude
    reg |= ((uint32_t)(50 & 0xfful) << 8);          // Pwm gradient
    reg |= ((uint32_t)(2 & 0x3ul) << 16);           // fPWM=2/512 fCLK
    reg |= ((uint32_t)(1 & 1ul) << 18);             // Autoscale enable
    tmcWrite(mot, REG_PWMCONF, reg);
    tmcWrite(mot, REG_TPWMTHRS, 0);
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
    for (uint8_t mot=0; mot<M_GUARD; mot++) {
        tmcInit(mot, first);
    }
    first = false;
}

void startProbeing(void)
{
    for (uint8_t mot=0; mot<M_GUARD; mot++) {
        /* Enable stallGuard */
        tmcWrite(mot, REG_TCOOLTHRS, 0xfffff);
        tmcSetCurrent(mot, MOTOR_CURRENT_PROBE, MOTOR_CURRENT_HOLD, 2);
    }
    probeingTime = millis();
}

void clearProbeing(void)
{
    for (uint8_t mot=0; mot<M_GUARD; mot++) {
        /* Make motors silent */
        tmcWrite(mot, REG_TCOOLTHRS, 0);
        /* Restore currents */
        tmcSetCurrent(mot, MOTOR_CURRENT_NORMAL, MOTOR_CURRENT_HOLD, 2);
    }
    probeingTime = 0;
}

bool checkProbeing(void)
{
    if (probeingTime && probeingTime + MOTOR_STALL_DELAY < millis()) {
        return (!READ(ORIG_Z_MIN_PIN) || !READ(ORIG_Y_MIN_PIN) || !READ(ORIG_X_MIN_PIN)) ? true : false;
    } else {
        return false;
    }
}
