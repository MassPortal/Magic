//Module for managing lighting

#ifndef _Lighting_h
#define _Lighting_h

#include "Repetier.h"
#include "Adafruit_NeoPixel.h"

#define LED_COUNT 19
#define LED_LOOP_DEVIDER 60
#define LED_EXTRUDER 5
#ifndef LED_MAX_RELATIVE_BRIGHTNESS
#define LED_MAX_RELATIVE_BRIGHTNESS 0.25
#endif									
#define LED_BASE_TEMP 30

class Lighting
{
 public:
	 Lighting();

	 Adafruit_NeoPixel LED = Adafruit_NeoPixel(LED_COUNT, 5, NEO_GRB + NEO_KHZ800);
	 enum ShowType {
		 Off,
		 SolidRed,
		 SolidBlue,
		 SolidGreen,
		 FixedRGB,
		 ShowTemperatures
	 };
	 uint8_t LED_CNT, EXT_LED;
	 int  ThisStep;
	 int  CurrentShow;
	 int  CurrentShowStep;
	 bool UpdateNeeded;
	 float BedTarget;
	 float BedCurrent;
	 float ExtruderTarget;
	 float ExtruderCurrent;
	 uint32_t LastPositionHash;
	 float LedBrightness;
	 void init();
	 void SetAllLeds(uint8_t r, uint8_t g, uint8_t b);
	 void SetAllBedLeds(uint8_t r, uint8_t g, uint8_t b);
	 void SetLed(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
	 void SetLedInstantly(uint8_t i, uint8_t r, uint8_t g, uint8_t b);
	 void CommitLeds();
	 void factoryTest();
	 void loop();
	 void ShowTemps();
	 void SetShowType(ShowType SType);
	 int ary[LED_COUNT][3];
	
};

extern Lighting Light;

#endif
