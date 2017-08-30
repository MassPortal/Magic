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
*/

#ifndef _REPETIER_H
#define _REPETIER_H

#include <math.h>
#include <stdint.h>

#define REPETIER_VERSION "0.92.8"

// ##########################################################################################
// ##                                  Debug configuration                                 ##
// ##########################################################################################
// These are run time sqitchable debug flags
enum debugFlags {DEB_ECHO = 0x1, DEB_INFO = 0x2, DEB_ERROR = 0x4,DEB_DRYRUN = 0x8,
                 DEB_COMMUNICATION = 0x10, DEB_NOMOVES = 0x20, DEB_DEBUG = 0x40
                };

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
//#define DEBUG_QUEUE_MOVE
/** Allows M111 so set bit 6 (32) which disables moves, at the first tried step. In combination
with a dry run, you can test the speed of path computations, which are still performed. */
#define INCLUDE_DEBUG_NO_MOVE 1
/** Writes the free RAM to output, if it is less then at the last test. Should always return
values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
usage or for seraching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
//#define DEBUG_ADVANCE
/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC
/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT
/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
#define DEBUG_COM_ERRORS
/** Adds a menu point in quick settings to write debg informations to the host in case of hangs where the ui still works. */
#define DEBUG_DELTA_OVERFLOW
//#define DEBUG_DELTA_REALPOS
//#define DEBUG_SPLIT
//#define DEBUG_JAM
// Find the longest segment length during a print
//#define DEBUG_SEGMENT_LENGTH
// Find the maximum real jerk during a print
//#define DEBUG_REAL_JERK
// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG

#define IGNORE_COORDINATE 999999

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define VIRTUAL_AXIS 4
// How big an array to hold X_AXIS..<MAX_AXIS>
#define Z_AXIS_ARRAY 3
#define E_AXIS_ARRAY 4
#define VIRTUAL_AXIS_ARRAY 5


#define A_TOWER 0
#define B_TOWER 1
#define C_TOWER 2
#define TOWER_ARRAY 3
#define E_TOWER_ARRAY 4

//direction flags
#define X_DIRPOS 1
#define Y_DIRPOS 2
#define Z_DIRPOS 4
#define E_DIRPOS 8
#define XYZ_DIRPOS 7
//step flags
#define XSTEP 16
#define YSTEP 32
#define ZSTEP 64
#define ESTEP 128
//combo's
#define XYZ_STEP 112
#define XY_STEP 48
#define XYZE_STEP 240
#define E_STEP_DIRPOS 136
#define Y_STEP_DIRPOS 34
#define X_STEP_DIRPOS 17
#define Z_STEP_DIRPOS 68

// add pid control
#define TEMP_PID 1

#define PRINTER_MODE_FFF 0
#define PRINTER_MODE_LASER 1
#define PRINTER_MODE_CNC 2

#include "Configuration.h"

inline void memcopy2(void *dest,void *source) {
	*((int16_t*)dest) = *((int16_t*)source);
}
inline void memcopy4(void *dest,void *source) {
	*((int32_t*)dest) = *((int32_t*)source);
}

#if FEATURE_Z_PROBE && Z_PROBE_PIN < 0
#error You need to define Z_PROBE_PIN to use z probe!
#endif

#if DISTORTION_CORRECTION
#if !FEATURE_Z_PROBE
#error Distortion correction requires the z probe feature to be enabled and configured!
#endif
#endif

#ifndef MAX_ROOM_TEMPERATURE
#define MAX_ROOM_TEMPERATURE 40
#endif
#ifndef ZHOME_X_POS
#define ZHOME_X_POS IGNORE_COORDINATE
#endif
#ifndef ZHOME_Y_POS
#define ZHOME_Y_POS IGNORE_COORDINATE
#endif

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE 0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS 1
#endif

#ifndef MINMAX_HARDWARE_ENDSTOP_Z2
#define MINMAX_HARDWARE_ENDSTOP_Z2 0
#define Z2_MINMAX_PIN -1
#endif

#define SPEED_MIN_MILLIS 400
#define SPEED_MAX_MILLIS 60
#define SPEED_MAGNIFICATION 100.0f

#ifdef FEATURE_SOFTWARE_LEVELING
#define SOFTWARE_LEVELING FEATURE_SOFTWARE_LEVELING
#endif

/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
*/
#ifndef ROD_RADIUS
#define ROD_RADIUS (PRINTER_RADIUS-END_EFFECTOR_HORIZONTAL_OFFSET-CARRIAGE_HORIZONTAL_OFFSET)
#endif

#ifdef FEATURE_Z_PROBE
#define MANUAL_CONTROL 1
#endif

//Step to split a cirrcle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT 1
#define MM_PER_ARC_SEGMENT_BIG 3
#else
#define MM_PER_ARC_SEGMENT_BIG MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS caluclation is startet to correct the circle interpolation
#define N_ARC_CORRECTION 25

// Test for shared cooler
#if NUM_EXTRUDER == 6 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT3_EXTRUDER_COOLER_PIN && EXT4_EXTRUDER_COOLER_PIN == EXT5_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT2_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT4_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 5 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT3_EXTRUDER_COOLER_PIN && EXT3_EXTRUDER_COOLER_PIN == EXT5_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT2_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 4 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT3_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN == EXT2_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 3 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN && EXT2_EXTRUDER_COOLER_PIN == EXT0_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#elif NUM_EXTRUDER == 2 && EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == EXT1_EXTRUDER_COOLER_PIN
#define SHARED_COOLER 1
#else
#define SHARED_COOLER 0
#endif

#ifndef START_STEP_WITH_HIGH
#define START_STEP_WITH_HIGH 1
#endif

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE == 101
#define SUPPORT_MAX6675
#endif

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE == 102
#define SUPPORT_MAX31855
#endif

// Test for shared coolers between extruders and mainboard
#if EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == FAN_BOARD_PIN
#define SHARED_COOLER_BOARD_EXT 1
#else
#define SHARED_COOLER_BOARD_EXT 0
#endif

#if NUM_EXTRUDER > 0 && EXT0_TEMPSENSOR_TYPE < 101
#define EXT0_ANALOG_INPUTS 1
#define EXT0_SENSOR_INDEX 0
#define EXT0_ANALOG_CHANNEL EXT0_TEMPSENSOR_PIN
#define ACCOMMA0 ,
#else
#define ACCOMMA0
#define EXT0_ANALOG_INPUTS 0
#define EXT0_SENSOR_INDEX EXT0_TEMPSENSOR_PIN
#define EXT0_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 1 && EXT1_TEMPSENSOR_TYPE < 101
#define EXT1_ANALOG_INPUTS 1
#define EXT1_SENSOR_INDEX EXT0_ANALOG_INPUTS
#define EXT1_ANALOG_CHANNEL ACCOMMA0 EXT1_TEMPSENSOR_PIN
#define ACCOMMA1 ,
#else
#define ACCOMMA1 ACCOMMA0
#define EXT1_ANALOG_INPUTS 0
#define EXT1_SENSOR_INDEX EXT1_TEMPSENSOR_PIN
#define EXT1_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 2 && EXT2_TEMPSENSOR_TYPE<101
#define EXT2_ANALOG_INPUTS 1
#define EXT2_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS
#define EXT2_ANALOG_CHANNEL ACCOMMA1 EXT2_TEMPSENSOR_PIN
#define ACCOMMA2 ,
#else
#define ACCOMMA2 ACCOMMA1
#define EXT2_ANALOG_INPUTS 0
#define EXT2_SENSOR_INDEX EXT2_TEMPSENSOR_PIN
#define EXT2_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 3 && EXT3_TEMPSENSOR_TYPE < 101
#define EXT3_ANALOG_INPUTS 1
#define EXT3_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS
#define EXT3_ANALOG_CHANNEL ACCOMMA2 EXT3_TEMPSENSOR_PIN
#define ACCOMMA3 ,
#else
#define ACCOMMA3 ACCOMMA2
#define EXT3_ANALOG_INPUTS 0
#define EXT3_SENSOR_INDEX EXT3_TEMPSENSOR_PIN
#define EXT3_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 4 && EXT4_TEMPSENSOR_TYPE < 101
#define EXT4_ANALOG_INPUTS 1
#define EXT4_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS
#define EXT4_ANALOG_CHANNEL ACCOMMA3 EXT4_TEMPSENSOR_PIN
#define ACCOMMA4 ,
#else
#define ACCOMMA4 ACCOMMA3
#define EXT4_ANALOG_INPUTS 0
#define EXT4_SENSOR_INDEX EXT4_TEMPSENSOR_PIN
#define EXT4_ANALOG_CHANNEL
#endif

#if NUM_EXTRUDER > 5 && EXT5_TEMPSENSOR_TYPE<101
#define EXT5_ANALOG_INPUTS 1
#define EXT5_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS
#define EXT5_ANALOG_CHANNEL ACCOMMA4 EXT5_TEMPSENSOR_PIN
#define ACCOMMA5 ,
#else
#define ACCOMMA5 ACCOMMA4
#define EXT5_ANALOG_INPUTS 0
#define EXT5_SENSOR_INDEX EXT5_TEMPSENSOR_PIN
#define EXT5_ANALOG_CHANNEL
#endif

#if HAVE_HEATED_BED && HEATED_BED_SENSOR_TYPE<101
#define BED_ANALOG_INPUTS 1
#define BED_SENSOR_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS
#define BED_ANALOG_CHANNEL ACCOMMA5 HEATED_BED_SENSOR_PIN
#undef BEKOMMA
#define BED_KOMMA ,
#else
#define BED_ANALOG_INPUTS 0
#define BED_SENSOR_INDEX HEATED_BED_SENSOR_PIN
#define BED_ANALOG_CHANNEL
#define BED_KOMMA ACCOMMA5
#endif

#if FAN_THERMO_THERMISTOR_PIN > -1 && FAN_THERMO_PIN > -1
#define THERMO_ANALOG_INPUTS 1
#define THERMO_ANALOG_INDEX EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS
#define THERMO_ANALOG_CHANNEL BED_KOMMA FAN_THERMO_THERMISTOR_PIN
#else
#define THERMO_ANALOG_INPUTS 0
#define THERMO_ANALOG_CHANNEL
#endif

#if CHAMBER_SENSOR_PIN > -1
#define CHAMBER_ANALOG_INPUTS 1
#define CHAMBER_ANALOG_INDEX THERMO_ANALOG_INPUTS+EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS
#define CHAMBER_ANALOG_CHANNEL BED_KOMMA CHAMBER_SENSOR_PIN
#else
#define CHAMBER_ANALOG_INPUTS 0
#define CHAMBER_ANALOG_CHANNEL
#endif

#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory(false);
#endif

/** \brief number of analog input signals. Normally 1 for each temperature sensor */
#define ANALOG_INPUTS (EXT0_ANALOG_INPUTS+EXT1_ANALOG_INPUTS+EXT2_ANALOG_INPUTS+EXT3_ANALOG_INPUTS+EXT4_ANALOG_INPUTS+EXT5_ANALOG_INPUTS+BED_ANALOG_INPUTS+THERMO_ANALOG_INPUTS+CHAMBER_ANALOG_INPUTS)
#if ANALOG_INPUTS > 0
/** Channels are the MUX-part of ADMUX register */
#define  ANALOG_INPUT_CHANNELS {EXT0_ANALOG_CHANNEL EXT1_ANALOG_CHANNEL EXT2_ANALOG_CHANNEL EXT3_ANALOG_CHANNEL EXT4_ANALOG_CHANNEL EXT5_ANALOG_CHANNEL BED_ANALOG_CHANNEL THERMO_ANALOG_CHANNEL CHAMBER_ANALOG_CHANNEL}
#endif

#ifndef BENDING_CORRECTION_A
#define BENDING_CORRECTION_A 0
#endif

#ifndef BENDING_CORRECTION_B
#define BENDING_CORRECTION_B 0
#endif

#ifndef BENDING_CORRECTION_C
#define BENDING_CORRECTION_C 0
#endif

#ifndef Z_ACCELERATION_TOP
#define Z_ACCELERATION_TOP 0
#endif

#include "HAL.h"
#include "gcode.h"

#if BED_LEDS
#include "Lighting.h"
#endif
#include "Communication.h"

#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t


#undef min
#undef max

class RMath
{
public:
    static inline float min(float a,float b)
    {
        if(a < b) return a;
        return b;
    }
    static inline float max(float a,float b)
    {
        if(a < b) return b;
        return a;
    }
    static inline int32_t min(int32_t a,int32_t b)
    {
        if(a < b) return a;
        return b;
    }
    static inline int32_t min(int32_t a,int32_t b, int32_t c)
    {
        if(a < b) return a < c ? a : c;
        return b<c ? b : c;
    }
    static inline float min(float a,float b, float c)
    {
        if(a < b) return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline int32_t max(int32_t a,int32_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline int min(int a,int b)
    {
        if(a < b) return a;
        return b;
    }
    static inline uint16_t min(uint16_t a,uint16_t b)
    {
        if(a < b) return a;
        return b;
    }
    static inline int16_t max(int16_t a,int16_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline uint16_t max(uint16_t a,uint16_t b)
    {
        if(a < b) return b;
        return a;
    }
    static inline unsigned long absLong(long a)
    {
        return a >= 0 ? a : -a;
    }
    static inline int32_t sqr(int32_t a)
    {
        return a*a;
    }
    static inline uint32_t sqr(uint32_t a)
    {
        return a*a;
    }
    static inline int64_t sqr(int64_t a)
    {
        return a*a;
    }
    static inline uint64_t sqr(uint64_t a)
    {
        return a*a;
    }
    static inline float sqr(float a)
    {
        return a*a;
    }
};

class RVector3
{
public:
    float x, y, z;
    RVector3(float _x = 0,float _y = 0,float _z = 0):x(_x),y(_y),z(_z) {};
    RVector3(const RVector3 &a):x(a.x),y(a.y),z(a.z) {};

    inline bool operator==(const RVector3 &rhs)
    {
        return x==rhs.x && y==rhs.y && z==rhs.z;
    }
    inline bool operator!=(const RVector3 &rhs)
    {
        return !(*this==rhs);
    }
    inline RVector3& operator=(const RVector3 &rhs)
    {
        if(this!=&rhs)
        {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        return *this;
    }

    inline RVector3& operator+=(const RVector3 &rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline RVector3& operator-=(const RVector3 &rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    inline RVector3 operator-() const
    {
        return RVector3(-x,-y,-z);
    }

    inline float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    inline RVector3 cross(const RVector3 &b) const
    {
        return RVector3(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x);
    }
    inline float scalar(const RVector3 &b) const
    {
        return (x*b.x+y*b.y+z*b.z);
    }
    inline RVector3 scale(float factor) const
    {
        return RVector3(x*factor,y*factor,z*factor);
    }
    inline float angle(RVector3 &direction)
    {
        return static_cast<float>(acos(scalar(direction)/(length()*direction.length())));
    }
};
	inline RVector3 operator+(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
	{
		lhs.x += rhs.x;
		lhs.y += rhs.y;
		lhs.z += rhs.z;
		return lhs;
	}

	inline RVector3 operator-(RVector3 lhs, const RVector3& rhs) // first arg by value, second by const ref
	{
		lhs.x -= rhs.x;
		lhs.y -= rhs.y;
		lhs.z -= rhs.z;
		return lhs;
	}

    inline RVector3 operator*(const RVector3 &lhs,float rhs) {
        return lhs.scale(rhs);
    }

    inline RVector3 operator*(float lhs,const RVector3 &rhs) {
        return rhs.scale(lhs);
    }
extern const uint8 osAnalogInputChannels[];
//extern uint8 osAnalogInputCounter[ANALOG_INPUTS];
//extern uint osAnalogInputBuildup[ANALOG_INPUTS];
//extern uint8 osAnalogInputPos; // Current sampling position
#if ANALOG_INPUTS > 0
extern volatile uint osAnalogInputValues[ANALOG_INPUTS];
#endif
#define PWM_HEATED_BED NUM_EXTRUDER
#define PWM_BOARD_FAN PWM_HEATED_BED+1
#define PWM_FAN1 PWM_BOARD_FAN+1
#define PWM_FAN2 PWM_FAN1+1
#define PWM_FAN3 PWM_FAN2+1
#define PWM_FAN_THERMO PWM_FAN3+1
#define PWM_CHAMB PWM_FAN_THERMO+1
#define NUM_PWM PWM_CHAMB+1
extern uint8_t pwm_pos[NUM_PWM]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
extern int maxadv;
#endif
extern int maxadv2;
extern float maxadvspeed;
#endif


#include "Extruder.h"

extern uint8_t transformCartesianStepsToDeltaSteps(int32_t cartesianPosSteps[], int32_t deltaPosSteps[]);
#ifndef FEATURE_DITTO_PRINTING
#define FEATURE_DITTO_PRINTING false
#endif
#if FEATURE_DITTO_PRINTING && (NUM_EXTRUDER > 4 || NUM_EXTRUDER < 2)
#error Ditto printing requires 2 - 4 extruder.
#endif

extern millis_t maxInactiveTime;
extern millis_t stepperInactiveTime;

extern void motorCurrentControlInit();

#include "Printer.h"
#include "motion.h"
extern long baudrate;

#include "HAL.h"


extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
#if FEATURE_FAN_CONTROL
extern uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
extern uint8_t fan2Kickstart;
#endif
#if FEATURE_VENTILATION
extern uint8_t fan3Kickstart;
#endif

extern volatile int waitRelax; // Delay filament relax at the end of print, could be a simple timeout

#define STR(s) #s
#define XSTR(s) STR(s)
#include "Commands.h"
#include "Eeprom.h"

#define SQRT(x) HAL::integer32Sqrt(x)

#include "Drivers.h"

#endif /* _REPETIER_H */
