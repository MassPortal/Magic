/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.


    Main author: repetier

    Initial port of HAL to Arduino Due: John Silvia
*/

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

#ifndef HAL_H
#define HAL_H

#include <inttypes.h>
#include "pins.h"
#include "Print.h"
#include "fastio.h"

// Hack to make 84 MHz Due clock work without changes to pre-existing code
// which would otherwise have problems with int overflow.
#undef F_CPU
#define F_CPU       21000000u   // should be factor of F_CPU_TRUE
#define F_CPU_TRUE  84000000u   // actual CPU clock frequency
#define EEPROM_BYTES 4096u      // bytes of eeprom we simulate

// force SdFat to use HAL (whether or not using SW spi)
#define TIMER0_PRESCALE 128u

// Some structures assume no padding, need to add this attribute on ARM
#define PACK    __attribute__ ((packed))
#define INLINE __attribute__((always_inline))

#define FSTRINGVALUE(var,value) const char var[] = value;
#define FSTRINGVAR(var) static const char var[];

#define EXTRUDER_TIMER          TC0
#define EXTRUDER_TIMER_CHANNEL  0
#define EXTRUDER_TIMER_IRQ      ID_TC0
#define EXTRUDER_TIMER_VECTOR   TC0_Handler
#define PWM_TIMER               TC0
#define PWM_TIMER_CHANNEL       1
#define PWM_TIMER_IRQ           ID_TC1
#define PWM_TIMER_VECTOR        TC1_Handler
#define TIMER1_TIMER            TC2
#define TIMER1_TIMER_CHANNEL    2
#define TIMER1_TIMER_IRQ        ID_TC8
#define TIMER1_COMPA_VECTOR     TC8_Handler
#define DELAY_TIMER             TC1
#define DELAY_TIMER_CHANNEL     1
#define DELAY_TIMER_IRQ         ID_TC4  // IRQ not really used, needed for pmc id
#define DELAY_TIMER_CLOCK       TC_CMR_TCCLKS_TIMER_CLOCK2
#define DELAY_TIMER_PRESCALE    8

//#define SERIAL_BUFFER_SIZE      1024
//#define SERIAL_PORT             UART
//#define SERIAL_IRQ              ID_UART
//#define SERIAL_PORT_VECTOR      UART_Handler

// TWI1 if SDA pin = 20  TWI0 for pin = 70
#define TWI_INTERFACE   		TWI1
#define TWI_ID  				ID_TWI1


#define EXTRUDER_CLOCK_FREQ     60000u // extruder stepper interrupt frequency
#define PWM_CLOCK_FREQ          3906u
#define TIMER1_CLOCK_FREQ       244u
#define TIMER1_PRESCALE         2u

#define AD_PRESCALE_FACTOR      84  // 500 kHz ADC clock 
#define AD_TRACKING_CYCLES      4   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES      1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)    (0x1u << channel)
#define ENABLED_ADC_CHANNELS    {TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN}

#define PULLUP(IO,v)            {pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

// INTERVAL / (32Khz/128)  = seconds
#define WATCHDOG_INTERVAL       1024u  // 8sec  (~16 seconds max)



#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif

//#define	READ(pin)  PIO_Get(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin)
#define READ_VAR(pin) (g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin ? 1 : 0) // does return 0 or pin value
#define _READ(pin) (DIO ##  pin ## _PORT->PIO_PDSR & DIO ##  pin ## _PIN ? 1 : 0) // does return 0 or pin value
#define READ(pin) _READ(pin)
//#define	WRITE_VAR(pin, v) PIO_SetOutput(g_APinDescription[pin].pPort, g_APinDescription[pin].ulPin, v, 0, PIO_PULLUP)
#define	WRITE_VAR(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)
#define		_WRITE(port, v)			do { if (v) {DIO ##  port ## _PORT -> PIO_SODR = DIO ## port ## _PIN; } else {DIO ##  port ## _PORT->PIO_CODR = DIO ## port ## _PIN; }; } while (0)
#define WRITE(pin,v) _WRITE(pin,v)

#define	SET_INPUT(pin) pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
  PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
#define	SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
                                      g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)
#define TOGGLE(pin) WRITE(pin,!READ(pin))
#define TOGGLE_VAR(pin) HAL::digitalWrite(pin,!HAL::digitalRead(pin))
#undef LOW
#define LOW         0
#undef HIGH
#define HIGH        1

// Protects a variable scope for interrupts. Uses RAII to force clearance of
// Interrupt block at the end resp. sets them to previous state.
// Uses ABSEPRI to allow higher level interrupts then the one changing firmware data
class InterruptProtectedBlock {
  public:
    INLINE void protect() {
      __disable_irq();
    }

    INLINE void unprotect() {
      __enable_irq();
    }

    INLINE InterruptProtectedBlock(bool later = false) {
      if (!later)
      __disable_irq();
    }

    INLINE ~InterruptProtectedBlock() {
      __enable_irq();
    }
};

#define EEPROM_OFFSET               0
#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
#define ANALOG_INPUT_SAMPLE 6
#define ANALOG_INPUT_MEDIAN 10

// Bits of the ADC converter
#define ANALOG_INPUT_BITS 12
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

// maximum available RAM
#define MAX_RAM 98303u

#define bit_clear(x,y) x&= ~(1<<y) //cbi(x,y)
#define bit_set(x,y)   x|= (1<<y)//sbi(x,y)

/** defines the data direction (reading from I2C device) in i2cStart(),i2cRepStart() */
#define I2C_READ    1
/** defines the data direction (writing to I2C device) in i2cStart(),i2cRepStart() */
#define I2C_WRITE   0

extern int spiDueDividors[];

/** Set max. frequency to 500000 Hz */
#define LIMIT_INTERVAL (F_CPU/500000u)


typedef unsigned int speed_t;
typedef unsigned long ticks_t;
typedef unsigned long millis_t;
typedef unsigned int flag8_t;
typedef int fast8_t;
typedef unsigned int ufast8_t;

#ifndef RFSERIAL
#define RFSERIAL Serial   // Programming port of the due
//#define RFSERIAL SerialUSB  // Native USB Port of the due
#endif

#if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
#if BLUETOOTH_SERIAL == 1
#define BT_SERIAL Serial1
#elif BLUETOOTH_SERIAL == 2
#define BT_SERIAL Serial2
#elif BLUETOOTH_SERIAL == 3
#define BT_SERIAL Serial3
#elif BLUETOOTH_SERIAL == 100
#define BT_SERIAL Serial
#elif BLUETOOTH_SERIAL == 101
#define BT_SERIAL SerialUSB
#endif

class RFDoubleSerial : public Print
{
  public:
    RFDoubleSerial();
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
};
extern RFDoubleSerial BTAdapter;

#endif

union eeval_t {
  uint8_t     b[4];
  float       f;
  uint32_t    i;
  uint16_t    s;
  long        l;
} PACK;

class HAL
{
  public:
    // we use ram instead of eeprom, so reads are faster and safer. Writes store in real eeprom as well
    // as long as hal eeprom functions are used.
    static char virtualEeprom[EEPROM_BYTES];
    static bool wdPinged;
    
    HAL();
    virtual ~HAL();

    // do any hardware-specific initialization here
    static inline void hwSetup(void)
    {
      HAL::i2cInit(TWI_CLOCK_FREQ);
      // make debugging startup easier
      //Serial.begin(115200);
      TimeTick_Configure(F_CPU_TRUE);

      // setup microsecond delay timer
      pmc_enable_periph_clk(DELAY_TIMER_IRQ);
      TC_Configure(DELAY_TIMER, DELAY_TIMER_CHANNEL, TC_CMR_WAVSEL_UP |
                   TC_CMR_WAVE | DELAY_TIMER_CLOCK);
      TC_Start(DELAY_TIMER, DELAY_TIMER_CHANNEL);
#if EEPROM_AVAILABLE && EEPROM_MODE != EEPROM_NONE
      // Copy eeprom to ram for faster access
      uint32_t i;
      for (i = 0; i < EEPROM_BYTES; i += 4) {
        eeval_t v = eprGetValue(i, 4);
        memcopy4(&virtualEeprom[i],&v.i);
      }
#else
      int i,n = 0;
      for (i = 0; i < EEPROM_BYTES; i += 4) {
        memcopy4(&virtualEeprom[i],&n);
      }
#endif
    }

    static uint32_t integer64Sqrt(uint64_t a);
    static uint32_t integer32Sqrt(uint32_t num);
    // return val'val
    static inline unsigned long U16SquaredToU32(unsigned int val)
    {
      return (unsigned long) val * (unsigned long) val;
    }
    static inline unsigned int ComputeV(long timer, long accel)
    {
      return static_cast<unsigned int>((static_cast<int64_t>(timer) * static_cast<int64_t>(accel)) >> 18);
      //return ((timer>>8)*accel)>>10;
    }
    // Multiply two 16 bit values and return 32 bit result
    static inline unsigned long mulu16xu16to32(unsigned int a, unsigned int b)
    {
      return (unsigned long) a * (unsigned long) b;
    }
    // Multiply two 16 bit values and return 32 bit result
    static inline unsigned int mulu6xu16shift16(unsigned int a, unsigned int b)
    {
      return ((unsigned long)a * (unsigned long)b) >> 16;
    }
    static inline unsigned int Div4U2U(unsigned long a, unsigned int b)
    {
      return ((unsigned long)a / (unsigned long)b);
    }
    static inline void digitalWrite(uint8_t pin, uint8_t value)
    {
      WRITE_VAR(pin, value);
    }
    static inline uint8_t digitalRead(uint8_t pin)
    {
      return READ_VAR(pin);
    }
    static inline void pinMode(uint8_t pin, uint8_t mode)
    {
      if (mode == INPUT) {
        SET_INPUT(pin);
      }
      else SET_OUTPUT(pin);
    }
    static long CPUDivU2(speed_t divisor) {
      return F_CPU / divisor;
    }
    static INLINE void delayMicroseconds(uint32_t usec)
    { //usec += 3;
      uint32_t n = usec * (F_CPU_TRUE / 3000000);
      asm volatile(
        "L2_%=_delayMicroseconds:"       "\n\t"
        "subs   %0, #1"                 "\n\t"
        "bge    L2_%=_delayMicroseconds" "\n"
        : "+r" (n) :
      );
    }
    static inline void delayMilliseconds(unsigned int delayMs)
    {
      unsigned int del;
      while (delayMs > 0) {
        del = delayMs > 100 ? 100 : delayMs;
        delay(del);
        delayMs -= del;
        HAL::pingWatchdog();
      }
    }
    static inline void eprSetByte(unsigned int pos, uint8_t value)
    {
      eeval_t v;
      v.b[0] = value;
      eprBurnValue(pos, 1, v);
      *(uint8_t*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetInt16(unsigned int pos, int16_t value)
    {
      eeval_t v;
      v.s = value;
      eprBurnValue(pos, 2, v);
      memcopy2(&virtualEeprom[pos],&value);
    }
    static inline void eprSetInt32(unsigned int pos, int32_t value)
    {
      eeval_t v;
      v.i = value;
      eprBurnValue(pos, 4, v);
      memcopy4(&virtualEeprom[pos],&value);
    }
    static inline void eprSetLong(unsigned int pos, long value)
    {
      eeval_t v;
      v.l = value;
      eprBurnValue(pos, sizeof(long), v);
      memcopy4(&virtualEeprom[pos],&value);
      //*(long*)(void*)&virtualEeprom[pos] = value;
    }
    static inline void eprSetFloat(unsigned int pos, float value)
    {
      eeval_t v;
      v.f = value;
      eprBurnValue(pos, sizeof(float), v);
      memcopy4(&virtualEeprom[pos],&value);
    }
    static inline uint8_t eprGetByte(unsigned int pos)
    {
      return *(uint8_t*)&virtualEeprom[pos];
      //eeval_t v = eprGetValue(pos,1);
      //return v.b[0];
    }
    static inline int16_t eprGetInt16(unsigned int pos)
    {
      int16_t v;
      memcopy2(&v,&virtualEeprom[pos]);
      return v;
      //return *(int16_t*)(void*)&virtualEeprom[pos];
      //eeval_t v;
      //v.i = 0;
      //v = eprGetValue(pos, 2);
      //return v.i;
    }
    static inline int32_t eprGetInt32(unsigned int pos)
    {
      int32_t v;
      memcopy4(&v,&virtualEeprom[pos]);
      return v;
      //eeval_t v = eprGetValue(pos, 4);
      //return v.i;
    }
    static inline long eprGetLong(unsigned int pos)
    {
      int32_t v;
      memcopy4(&v,&virtualEeprom[pos]);
      return v;
      //return *(long*)(void*)&virtualEeprom[pos];
      //eeval_t v = eprGetValue(pos, sizeof(long));
      //return v.l;
    }
    static inline float eprGetFloat(unsigned int pos) {
      float v;
      memcopy4(&v,&virtualEeprom[pos]);
      return v;
      //return *(float*)(void*)&virtualEeprom[pos];
      //eeval_t v = eprGetValue(pos, sizeof(float));
      //return v.f;
    }

    // Write any data type to EEPROM
    static inline void eprBurnValue(unsigned int pos, int size, union eeval_t newvalue)
    {
#if EEPROM_AVAILABLE == EEPROM_I2C
      i2cStartAddr(EEPROM_SERIAL_ADDR << 1 | I2C_WRITE, pos);
      i2cWriting(newvalue.b[0]);        // write first byte
      for (int i = 1; i < size; i++) {
        pos++;
        // writes cannot cross page boundary
        if ((pos % EEPROM_PAGE_SIZE) == 0) {
          // burn current page then address next one
          i2cStop();
          delayMilliseconds(EEPROM_PAGE_WRITE_TIME);
          i2cStartAddr(EEPROM_SERIAL_ADDR << 1, pos);
        } else {
          i2cTxFinished();      // wait for transmission register to empty
        }
        i2cWriting(newvalue.b[i]);
      }
      i2cStop();          // signal end of transaction
      delayMilliseconds(EEPROM_PAGE_WRITE_TIME);   // wait for page write to complete
#endif
    }

    // Read any data type from EEPROM that was previously written by eprBurnValue
    static inline union eeval_t eprGetValue(unsigned int pos, int size)
    {
#if EEPROM_AVAILABLE == EEPROM_I2C
      int i;
      eeval_t v;

      size--;
      // set read location
      i2cStartAddr(EEPROM_SERIAL_ADDR << 1 | I2C_READ, pos);
      // begin transmission from device
      i2cStartBit();
      for (i = 0; i < size; i++) {
        // read an incomming byte
        v.b[i] = i2cReadAck();
      }
      // read last byte
      v.b[i] = i2cReadNak();
      return v;
#endif
    }

    static inline void allowInterrupts()
    {
      //__enable_irq();
    }
    static inline void forbidInterrupts()
    {
      //__disable_irq();
    }
    static void setupTimer();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // hardware SPI
    static void spiBegin();
    // spiClock is 0 to 6, relecting AVR clock dividers 2,4,8,16,32,64,128
    // Due can only go as slow as AVR divider 32 -- slowest Due clock is 329,412 Hz
    static void spiInit(uint8_t spiClock);
    // Write single byte to SPI
    static void spiSend(byte b);
    static void spiSend(const uint8_t* buf , size_t n);
    // Read single byte from SPI
    static uint8_t spiReceive();
    // Read from SPI into buffer
    static void spiReadBlock(uint8_t*buf, uint16_t nbyte);

    // Write from buffer to SPI

    static void spiSendBlock(uint8_t token, const uint8_t* buf);

    // I2C Support
    static void i2cInit(unsigned long clockSpeedHz);
    static void i2cStartWait(unsigned char address);
    static unsigned char i2cStart(unsigned char address);
    static void i2cStartAddr(unsigned char address, unsigned int pos);
    static void i2cStop(void);
    static void i2cStartBit(void);
    static void i2cCompleted (void);
    static void i2cTxFinished(void);
    static void i2cWriting(uint8_t data );
    static unsigned char i2cWrite(uint8_t data );
    static unsigned char i2cReadAck(void);
    static unsigned char i2cReadNak(void);


    // Watchdog support
    inline static void startWatchdog() {
      WDT->WDT_MR = WDT_MR_WDRSTEN | WATCHDOG_INTERVAL | (WATCHDOG_INTERVAL << 16);
      WDT->WDT_CR = 0xA5000001;
    };
    inline static void stopWatchdog() {}
    inline static void pingWatchdog() {
        wdPinged = true;
    };

    inline static float maxExtruderTimerFrequency() {
      return (float)F_CPU_TRUE/32;
    }

#if ANALOG_INPUTS > 0
    static void analogStart(void);
#endif
#if USE_ADVANCE
    static void resetExtruderDirection();
#endif
    static volatile uint8_t insideTimer1;
};

#endif // HAL_H
