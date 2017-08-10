#ifndef PINS_H
#define PINS_H


/*
The board assignment defines the capabilities of the motherboard and the used pins.
Each board definition follows the following scheme:

STEPPER_CURRENT_CONTROL
  CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
  CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
*/

#define CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does

#define EEPROM_NONE 0
#define EEPROM_I2C  1
#define EEPROM_SPI_ALLIGATOR 2

// RADDS Board
// http://www.dr-henschke.de/RADDS_due.html

/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN     24
#define ORIG_X_DIR_PIN      23
#define ORIG_X_MIN_PIN      28
#define ORIG_X_MAX_PIN      34
#define ORIG_X_ENABLE_PIN   26

#define ORIG_Y_STEP_PIN     17 
#define ORIG_Y_DIR_PIN      16
#define ORIG_Y_MIN_PIN      30
#define ORIG_Y_MAX_PIN      36
#define ORIG_Y_ENABLE_PIN   22

#define ORIG_Z_STEP_PIN     2
#define ORIG_Z_DIR_PIN      3
#define ORIG_Z_MIN_PIN      32
#define ORIG_Z_MAX_PIN      38
#define ORIG_Z_ENABLE_PIN   15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     13
// Due analog pin #54
#define TEMP_0_PIN       7 
#define HEATER_1_PIN     7 
#define TEMP_1_PIN       3 
// Due analog pin #58
#define HEATER_2_PIN     12
// Due analog pin #55
#define TEMP_2_PIN       6 
#define HEATER_3_PIN     11
// Due analog pin #56
#define TEMP_3_PIN       5 
// Due analog pin #57
#define TEMP_4_PIN       4 

// Dua analog pin #59 = A5 -> AD 2
#define THERMOCOUPLE_0_PIN  2   

#define ORIG_E0_STEP_PIN    61
#define ORIG_E0_DIR_PIN     60
#define ORIG_E0_ENABLE_PIN  62

#define ORIG_E1_STEP_PIN    64
#define ORIG_E1_DIR_PIN     63
#define ORIG_E1_ENABLE_PIN  65

#define ORIG_E2_STEP_PIN    51
#define ORIG_E2_DIR_PIN     53
#define ORIG_E2_ENABLE_PIN  49

// Extra driver on extension board
#define ORIG_E3_STEP_PIN    35
#define ORIG_E3_DIR_PIN     33
#define ORIG_E3_ENABLE_PIN  37

// Extra driver on extension port
#define ORIG_E4_STEP_PIN    29
#define ORIG_E4_DIR_PIN     27
#define ORIG_E4_ENABLE_PIN  31

#define EXTENSION_BOARD_MS1 67
#define EXTENSION_BOARD_MS2 68
#define EXTENSION_BOARD_MS3 69
// 66 -> not connected
// 25 -> not connected
// To set microstepping on startup set START_GCODE to e.g.
// "M42 P67 S255\nM42 P68 S255\nM42 P69 S255"

#define LED_PIN 	   -1
#define PAUSE_LED_PIN	-1
#define BED_LED_PIN		5
//9 
#define ORIG_FAN_PIN	HEATER_3_PIN
//8 
#define ORIG_FAN2_PIN   8
#define ORIG_FAN3_PIN   9
#define ORIG_PS_ON_PIN          -1
#define KILL_PIN	   50
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 				20  	
// 21 or 71
#define SCL_PIN 				21  	

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,
#define E2_PINS ORIG_E2_STEP_PIN,ORIG_E2_DIR_PIN,ORIG_E2_ENABLE_PIN,
#define E3_PINS ORIG_E3_STEP_PIN,ORIG_E3_DIR_PIN,ORIG_E3_ENABLE_PIN,
#define E4_PINS ORIG_E4_STEP_PIN,ORIG_E4_DIR_PIN,ORIG_E4_ENABLE_PIN,

#define TWI_CLOCK_FREQ          400e3
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1

#ifndef SDSSORIG
#define SDSSORIG -1
#endif

#ifndef STEPPER_CURRENT_CONTROL // Set default stepper current control if not set yet.
#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_MANUAL
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#endif

#if NUM_EXTRUDER < 2
#undef E1_PINS
#define E1_PINS
#endif

#if NUM_EXTRUDER<3
#undef E2_PINS
#define E2_PINS
#endif

#ifndef HEATER_PINS_INVERTED
#define HEATER_PINS_INVERTED 0
#endif

#define SPI_PIN         87
#define SPI_CHAN        1
#define MOSI_PIN        75
#define MISO_PIN        74
#define SCK_PIN         76

// Original pin assignmats to be used in configuration tool
#define X_STEP_PIN ORIG_X_STEP_PIN
#define X_DIR_PIN ORIG_X_DIR_PIN
#define X_ENABLE_PIN ORIG_X_ENABLE_PIN
#define X_MIN_PIN ORIG_X_MIN_PIN
#define X_MAX_PIN ORIG_X_MAX_PIN

#define Y_STEP_PIN ORIG_Y_STEP_PIN
#define Y_DIR_PIN ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN ORIG_Y_ENABLE_PIN
#define Y_MIN_PIN ORIG_Y_MIN_PIN
#define Y_MAX_PIN ORIG_Y_MAX_PIN

#define Z_STEP_PIN ORIG_Z_STEP_PIN
#define Z_DIR_PIN ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN ORIG_Z_ENABLE_PIN
#define Z_MIN_PIN ORIG_Z_MIN_PIN
#define Z_MAX_PIN ORIG_Z_MAX_PIN

#define E0_STEP_PIN ORIG_E0_STEP_PIN
#define E0_DIR_PIN ORIG_E0_DIR_PIN
#define E0_ENABLE_PIN ORIG_E0_ENABLE_PIN

#define E1_STEP_PIN ORIG_E1_STEP_PIN
#define E1_DIR_PIN ORIG_E1_DIR_PIN
#define E1_ENABLE_PIN ORIG_E1_ENABLE_PIN

#define E2_STEP_PIN ORIG_E2_STEP_PIN
#define E2_DIR_PIN ORIG_E2_DIR_PIN
#define E2_ENABLE_PIN ORIG_E2_ENABLE_PIN

#define E3_STEP_PIN ORIG_E3_STEP_PIN
#define E3_DIR_PIN ORIG_E3_DIR_PIN
#define E3_ENABLE_PIN ORIG_E3_ENABLE_PIN

#define E4_STEP_PIN ORIG_E4_STEP_PIN
#define E4_DIR_PIN ORIG_E4_DIR_PIN
#define E4_ENABLE_PIN ORIG_E4_ENABLE_PIN

#define FAN_PIN ORIG_FAN_PIN
#ifdef ORIG_FAN2_PIN
#define FAN2_PIN ORIG_FAN2_PIN
#endif
#ifdef ORIG_FAN3_PIN
#define FAN3_PIN ORIG_FAN3_PIN
#endif

#define PS_ON_PIN ORIG_PS_ON_PIN

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, ORIG_PS_ON_PIN, \
						HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN }
#endif /* PINS_H */

