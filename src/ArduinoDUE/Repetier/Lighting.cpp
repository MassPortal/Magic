#include "Lighting.h"
#include "Extruder.h"
#include "Printer.h"

Lighting::Lighting()
{
	CurrentShow		= 0;
	CurrentShowStep	= 0;
	UpdateNeeded	= false;
	BedTarget		= 70; //we initialize this to some temerature to be considered hot, to detect hot-to-touch bed right after boot
	BedCurrent		= 0;
	ExtruderTarget	= 160;
	ExtruderCurrent	= 0;
	ThisStep = LED_LOOP_DEVIDER+1;
	LED_CNT = 0;
	EXT_LED = 0;
}

bool Lighting::dimTo(uint8_t target[LED_COUNT][3])
{
    bool done = true;
    bool change;
    uint16_t inc;
    uint8_t i, k;
    static uint8_t now[LED_COUNT][3];
    for (change = false, i = 0; i < LED_CNT; i++) {
        for (k = 0; k<3; k++) {
            if (target[i][k] != now[i][k]) {
                inc = now[i][k]/0x1F + 1;
                if (now[i][k] < target[i][k]) {
                    if (inc + now[i][k]> target[i][k]) {
                        now[i][k] = target[i][k];
                    } else {
                        now[i][k] += inc;
                    }
                } else {
                    if (inc > now[i][k] || now[i][k] - inc < target[i][k]) {
                        now[i][k] = target[i][k];
                    } else {
                        now[i][k] -= (inc/2 + 1);
                    }
                }
                change = true;
                done = false;
            }
            if (change) LED.setPixelColor(i, now[i][0], now[i][1], now[i][2]);
        }
    }
    if (!done) LED.show();
    return done;
}

void Lighting::init()
{
	//factoryTest();
	/*LED.setOutput(BED_LED_PIN); 
	LED.setColorOrderGRB(); 
								 */
	LED.begin();
    uint32_t maxIter = 0;
	//smooth fade in to blue to avoid instant turn-on. total time of this blocking code is 250ms. worth it.
	//slower/longer fade would cause problems to boot and/or connect host software
	LED_CNT = Printer::ledCount(false);
	EXT_LED = Printer::ledCount(true);
	Com::printInfoFLN("LEDs init");
	//SetAllLeds(0, 0, 0);
    LedBrightness = (EEPROM_MODE > 0) ? EEPROM::bedLedBrightness() : LED_MAX_RELATIVE_BRIGHTNESS;
    memset(ary,0xff/4, sizeof(ary));
    if (!(Printer::ledVal > 1) || !(LedBrightness > 0.01)) return;

    //while ("false lololo")
    for (uint32_t i = 0,j = 1*LED_CNT/3,k = 2*LED_CNT/3; i < LED_CNT; i++, j++,k++) {
        if (j == LED_CNT) j = 0;
        if (k == LED_CNT) k = 0;
        ary[i][0] = 0xff;
        ary[i][1] = 0xff;
        ary[i][2] = 0xff;
        ary[j][0] = 0xff;
        ary[j][1] = 0xff;
        ary[j][2] = 0xff;
        ary[k][0] = 0xff;
        ary[k][1] = 0xff;
        ary[k][2] = 0xff;
        while (!dimTo(ary) || maxIter++ >= 20) delay(3);
        maxIter = 0;
        ary[i][0] = 0xff/4;
        ary[i][1] = 0xff/4;
        ary[i][2] = 0xff/4;
        ary[j][0] = 0xff/4;
        ary[j][1] = 0xff/4;
        ary[j][2] = 0xff/4;
        ary[k][0] = 0xff/4;
        ary[k][1] = 0xff/4;
        ary[k][2] = 0xff/4;
    }
    while (!dimTo(ary));
}

void Lighting::factoryTest(){
	for (int i = 0; i < LED_CNT; i++)
	{
		SetLedInstantly(i, 0, 0, 0);
		delay(20); // Wait (ms)
	}
	for (int i = 0; i < LED_CNT; i++)
	{
		SetLedInstantly(i, 255, 0, 0);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_CNT; i++)
	{
		SetLedInstantly(i, 0, 255, 0);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_CNT; i++)
	{
		SetLedInstantly(i, 0, 0, 255);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	for (int i = 0; i < LED_CNT; i++)
	{
		SetLedInstantly(i, 255, 255, 255);
		delay(20); // Wait (ms)
	}
	delay(400); // Wait (ms)
	SetAllLeds(0, 0, 0);
}
void Lighting::loop()
{
    /* If printer has commands in buffer */
    if (PrintLine::hasLines() || Printer::isZProbingActive()) return;

	ThisStep++;
	if (ThisStep <LED_LOOP_DEVIDER) return; //only update leds every x loops
	ThisStep = 0;

    SetShowType(ShowTemperatures); //temorary - to test bed heating with leds
	
	switch (CurrentShow) {
	case Off:
		SetAllLeds(0, 0, 0);
		break;
	case SolidRed:
		SetAllLeds(255, 0, 0);
		break;
	case SolidGreen:
		SetAllLeds(0, 255, 0);
		break;
	case SolidBlue:
		SetAllLeds(0, 0, 255);
		break;
	case ShowTemperatures:
		ShowTemps();
		break;
	}
	//Printer::setZProbingActive(false);
}
void Lighting::ShowTemps()
{
	BedCurrent			= Extruder::getHeatedBedTemperature();
	if (BedCurrent < 35 && Extruder::getHeatedBedTargetTemperature() < 35) BedCurrent = 0;
	ExtruderCurrent		= Extruder::current->tempControl.currentTemperatureC;
	if (ExtruderCurrent < 40 && Extruder::current->tempControl.targetTemperatureC < 40) ExtruderCurrent = 0;

	//these checks for 0 enable non-interupted correct-colored lighting throughout cooldown process
	if (Extruder::getHeatedBedTargetTemperature()>0) 
		BedTarget		= Extruder::getHeatedBedTargetTemperature();
	else
		BedTarget		= 80;
	if (Extruder::current->tempControl.targetTemperatureC>0)
		ExtruderTarget	= Extruder::current->tempControl.targetTemperatureC;
	else
		ExtruderTarget	= 170;

	//bed leds (all except middle one (5th)
	int b = (BedCurrent) * 255 / (BedTarget);
	const uint8_t reductor = 3;
	if (b>255) b = 255;
	if (b<0) b = 0;
	int bg = 0;
	int bb = 0;
	//make blue more visible
	if (b < 115) {
		bg = (255 - b) / reductor;
		bb = 255 - b;
		b = b / reductor * 0.5;
	}
	else  {
		bg = 255 - b;
		bb = (255 - b) / reductor;
	}

	//extruder led (5th)
	int e = (ExtruderCurrent)* 255 / (ExtruderTarget);
	if (e>255) e = 255;
	if (e<0) e = 0;
	int eg = 0;
	int eb = 0;
	//make blue more visible
	if (e < 100) {
		eg = (255 - e) / reductor;
		eb = 255 - e;
		e = e / reductor * 0.5;
	}
	else  {
		eg = 255 - e;
		eb = (255 - e) / reductor;
	}

	ary[EXT_LED][0] = 255;
	ary[EXT_LED][1] = 255;
	ary[EXT_LED][2] = 255;
	if (b < 1)
		SetAllBedLeds(255, 255, 255);
	else
		SetAllBedLeds(b, bg, bb);
		ary[EXT_LED][0] = b;
		ary[EXT_LED][1] = bg;
		ary[EXT_LED][2] = bb;
	
	if (e <  1)	 {
		SetLed(EXT_LED, 255, 255, 255);
	}
	else 	   {
		SetLed(EXT_LED, e, eg, eb);
		ary[EXT_LED][0] = e;
		ary[EXT_LED][1] = eg;
		ary[EXT_LED][2] = eb;
	}
	if (Printer::ledVal > 1 && LedBrightness > 0.01) {
#if DEBUGGING
		Com::printInfoFLN("LEDsShowTemp");
#endif
		CommitLeds();
	}

}
void Lighting::SetShowType(ShowType SType)
{
	CurrentShow = SType;
	CurrentShowStep = 0;
	UpdateNeeded = true;
}

void Lighting::SetAllLeds(uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < LED_CNT; i++)
	{
		SetLed(i, r, g, b);
		ary[i][0] = r;
		ary[i][1] = g;
		ary[i][2] = b;
	}
	CommitLeds();
}
void Lighting::SetAllBedLeds(uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < LED_CNT; i++)
	{
		if (!(i==EXT_LED)) {
			SetLed(i, r, g, b);
			ary[i][0] = r;
			ary[i][1] = g;
			ary[i][2] = b;
		}
	}
}

//Low level wrappers
void Lighting::SetLed(uint8_t i, uint8_t r, uint8_t g, uint8_t b)
{
	LED.setPixelColor(i,
		r*LedBrightness,
		g*LedBrightness,
		b*LedBrightness);
}
void Lighting::SetLedInstantly(uint8_t i, uint8_t r, uint8_t g, uint8_t b)
{
	SetLed(i, r, g, b);
	CommitLeds();
}
void Lighting::CommitLeds()
{
	if (Printer::ledVal > 1 && LedBrightness > 0.01)
		LED.show(); // Sends the data to the LEDs
}

Lighting Light = Lighting();




