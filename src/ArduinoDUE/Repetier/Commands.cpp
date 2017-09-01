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

#include "Repetier.h"

const int8_t sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

static bool monitorTemps = false;

void Commands::commandLoop(void)
{
    GCode::readFromSerial();
    /* Gets NULL if there is nothing new */
    /* Checks code, executes it, pops it*/
    Commands::executeGCode(GCode::peekCurrentCommand());
}

void Commands::checkForPeriodicalActions(void)
{
    static millis_t timer100ms;
    static millis_t timer500ms;

    if (timer100ms + 100 < millis()) {
        Extruder::manageTemperatures();
#if BED_LEDS
        Light.loop();
#endif
        timer100ms = millis();
    }
    if (monitorTemps) {
        if (timer500ms + 500 < millis()) {
            Commands::printTemperatures(false);
            timer500ms = millis();
        }
    }
}

/** \brief Waits until movement cache is empty.

  Some commands expect no movement, before they can execute. This function
  waits, until the steppers are stopped. In the meanwhile it buffers incoming
  commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves()
{
    while(PrintLine::hasLines())
    {
        GCode::readFromSerial();
        Printer::defaultLoopActions();
    }
}

void Commands::printCurrentPosition(const char* s)
{
    float x, y, z;
    Printer::realPosition(x, y, z);
    if (isnan(x) || isinf(x) || isnan(y) || isinf(y) || isnan(z) || isinf(z))
    {
        Com::printErrorFLN(s); // flag where the error condition came from
    }
    x += Printer::coordinateOffset[X_AXIS];
    y += Printer::coordinateOffset[Y_AXIS];
    z += Printer::coordinateOffset[Z_AXIS];
    Com::printF(Com::tXColon, x * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceYColon, y * (Printer::unitIsInches ? 0.03937 : 1), 2);
    Com::printF(Com::tSpaceZColon, z * (Printer::unitIsInches ? 0.03937 : 1), 3);
    Com::printFLN(Com::tSpaceEColon, Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 4);
    //Com::printF("OffX:",Printer::offsetX); // to debug offset handling
    //Com::printFLN(" OffY:",Printer::offsetY);
}

void Commands::printTemperatures(bool showRaw)
{
#if NUM_EXTRUDER > 0
    float temp = Extruder::current->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE == 0
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
#else
    Com::printF(Com::tTColon,temp);
    Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
#if HAVE_HEATED_BED
    Com::printF(Com::tSpaceBColon,Extruder::getHeatedBedTemperature());
    Com::printF(Com::tSpaceSlash,heatedBedController.targetTemperatureC,0);
    if(showRaw)
    {
        Com::printF(Com::tSpaceRaw,(int)NUM_EXTRUDER);
        Com::printF(Com::tColon,(1023 << 2) - heatedBedController.currentTemperature);
    }
    //Com::printF(Com::tSpaceBAtColon,(pwm_pos[heatedBedController.pwmIndex])); // Show output of autotune when tuning!
#endif
#endif
#if TEMP_PID
    //Com::printF(Com::tSpaceAtColon,(autotuneIndex == 255 ? pwm_pos[Extruder::current->id] : pwm_pos[autotuneIndex])); // Show output of autotune when tuning!
#endif
#if NUM_EXTRUDER > 1 && MIXING_EXTRUDER == 0 && !MIXING_SEMI // do not run
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
    {
        Com::printF(Com::tSpaceT,(int)i);
        Com::printF(Com::tColon,extruder[i].tempControl.currentTemperatureC);
        Com::printF(Com::tSpaceSlash,extruder[i].tempControl.targetTemperatureC,0);
#if TEMP_PID
        Com::printF(Com::tSpaceAt,(int)i);
        Com::printF(Com::tColon,(pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of autotune when tuning!
#endif
        if(showRaw)
        {
            Com::printF(Com::tSpaceRaw,(int)i);
            Com::printF(Com::tColon,(1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[i].tempControl.currentTemperature);
        }
    }
#elif NUM_EXTRUDER == 1
    if(showRaw)
    {
            Com::printF(Com::tSpaceRaw,(int)0);
            Com::printF(Com::tColon,(1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[0].tempControl.currentTemperature);
    }
#endif
    Com::println();
#endif
}

void Commands::printTemperature() {
	Com::printF("FW:4=", Extruder::getChamberTemperature());
	Com::printFLN("#Chamber temp");
}

void Commands::changeFeedrateMultiply(int factor)
{
    if(factor < 25) factor = 25;
    if(factor > 500) factor = 500;
    Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;
    Com::printFLN(Com::tSpeedMultiply, factor);
}

void Commands::changeFlowrateMultiply(int factor)
{
    if(factor < 25) factor = 25;
    if(factor > 200) factor = 200;
    Printer::extrudeMultiply = factor;
    if(Extruder::current->diameter <= 0)
        Printer::extrusionFactor = 0.01f * static_cast<float>(factor);
    else
        Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);
    Com::printFLN(Com::tFlowMultiply, factor);
}

uint32_t fanKickstart;
uint32_t fan2Kickstart;
uint32_t fan3Kickstart;

void Commands::setFanSpeed(int speed, bool immediately)
{
#if FAN_PIN>-1
    if(Printer::fanSpeed == speed)
        return;
    speed = constrain(speed,0,255);
    Printer::fanSpeed = speed;
    if(PrintLine::linesCount == 0 || immediately) {
        if(Printer::mode == PRINTER_MODE_FFF)
    {
	        for(fast8_t i = 0; i < PRINTLINE_CACHE_SIZE; i++)
			    PrintLine::lines[i].secondSpeed = speed;         // fill all printline buffers with new fan speed value
        }
        Printer::setFanSpeedDirectly(speed);
	}
        Com::printFLN(Com::tFanspeed,speed); // send only new values to break update loops!
#endif
}

void Commands::setFan2Speed(int speed)
{
	#if FAN2_PIN >- 1
	speed = constrain(speed,0,255);
	Printer::setFan2SpeedDirectly(speed);
	Com::printFLN(Com::tFan2speed,speed); // send only new values to break update loops!
	#endif
}
void Commands::setFan3Speed(int speed)
{
	#if FAN3_PIN >- 1
	speed = constrain(speed,0,255);
	Printer::setFan3SpeedDirectly(speed);
	Com::printFLN(Com::tFan3speed,speed); // send only new values to break update loops!
	#endif
}
void Commands::reportPrinterUsage()
{
#if EEPROM_MODE != 0
    float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
    Com::printF(Com::tPrintedFilament, dist, 2);
    Com::printF(Com::tSpacem);
    bool alloff = true;
#if NUM_EXTRUDER > 0
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 15) alloff = false;
#endif
    int32_t seconds = (alloff ? 0 : (millis() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
    int32_t tmp = seconds / 86400;
    seconds -= tmp * 86400;
    Com::printF(Com::tPrintingTime,tmp);
    tmp = seconds / 3600;
    Com::printF(Com::tSpaceDaysSpace,tmp);
    seconds -= tmp * 3600;
    tmp = seconds / 60;
    Com::printF(Com::tSpaceHoursSpace,tmp);
    Com::printFLN(Com::tSpaceMin);
#endif
}
#if STEPPER_CURRENT_CONTROL == CURRENT_CONTROL_DIGIPOT
// Digipot methods for controling current and microstepping

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
int digitalPotWrite(int address, uint16_t value) // From Arduino DigitalPotControl example
{
    if(value > 255)
        value = 255;
    WRITE(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
    HAL::spiSend(address); //  send in the address and value via SPI:
    HAL::spiSend(value);
    WRITE(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
}

void setMotorCurrent(uint8_t driver, uint16_t current)
{
    if(driver > 4) return;
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
}

void setMotorCurrentPercent( uint8_t channel, float level)
{
    uint16_t raw_level = ( level * 255 / 100 );
    setMotorCurrent(channel,raw_level);
}
#endif

void motorCurrentControlInit() //Initialize Digipot Motor Current
{
#if DIGIPOTSS_PIN && DIGIPOTSS_PIN > -1
    HAL::spiInit(0); //SPI.begin();
    SET_OUTPUT(DIGIPOTSS_PIN);
#ifdef MOTOR_CURRENT_PERCENT
    const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
    for(int i = 0; i <= 4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrentPercent(i,digipot_motor_current[i]);
#else
    const uint8_t digipot_motor_current[] = MOTOR_CURRENT;
    for(int i = 0; i <= 4; i++)
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        setMotorCurrent(i,digipot_motor_current[i]);
#endif
#endif
}
#endif

/**
  \brief Execute the Arc command stored in com.
*/
#if ARC_SUPPORT
void Commands::processArc(GCode *com)
{
    float position[Z_AXIS_ARRAY];
    Printer::realPosition(position[X_AXIS],position[Y_AXIS],position[Z_AXIS]);
    if(!Printer::setDestinationStepsFromGCode(com)) return; // For X Y Z E F
    float offset[2] = {Printer::convertToMM(com->hasI() ? com->I : 0),Printer::convertToMM(com->hasJ() ? com->J : 0)};
    float target[E_AXIS_ARRAY] = {Printer::realXPosition(),Printer::realYPosition(),Printer::realZPosition(),Printer::destinationSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS]};
    float r;
    if (com->hasR())
    {
        /*
          We need to calculate the center of the circle that has the designated radius and passes
          through both the current position and the target position. This method calculates the following
          set of equations where [x,y] is the vector from current to target position, d == magnitude of
          that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
          the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
          length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
          [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

          d^2 == x^2 + y^2
          h^2 == r^2 - (d/2)^2
          i == x/2 - y/d*h
          j == y/2 + x/d*h

                                                               O <- [i,j]
                                                            -  |
                                                  r      -     |
                                                      -        |
                                                   -           | h
                                                -              |
                                  [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                            | <------ d/2 ---->|

          C - Current position
          T - Target position
          O - center of circle that pass through both C and T
          d - distance from C to T
          r - designated radius
          h - distance from center of CT to O

          Expanding the equations:

          d -> sqrt(x^2 + y^2)
          h -> sqrt(4 * r^2 - x^2 - y^2)/2
          i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
          j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

          Which can be written:

          i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
          j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

          Which we for size and speed reasons optimize to:

          h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
          i = (x - (y * h_x2_div_d))/2
          j = (y + (x * h_x2_div_d))/2

        */
        r = Printer::convertToMM(com->R);
        // Calculate the change in position along each selected axis
        double x = target[X_AXIS]-position[X_AXIS];
        double y = target[Y_AXIS]-position[Y_AXIS];

        double h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)
        // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
        // real CNC, and thus - for practical reasons - we will terminate promptly:
        if(isnan(h_x2_div_d))
        {
            Com::printErrorFLN(Com::tInvalidArc);
            return;
        }
        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
        if (com->G==3)
        {
            h_x2_div_d = -h_x2_div_d;
        }

        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
           the left hand circle will be generated - when it is negative the right hand circle is generated.


                                                         T  <-- Target position

                                                         ^
              Clockwise circles with this center         |          Clockwise circles with this center will have
              will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                               \         |          /
        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                         |
                                                         |

                                                         C  <-- Current position                                 */


        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
        // even though it is advised against ever generating such circles in a single line of g-code. By
        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
        // travel and thus we get the unadvisably long arcs as prescribed.
        if (r < 0)
        {
            h_x2_div_d = -h_x2_div_d;
            r = -r; // Finished with r. Set to positive for mc_arc
        }
        // Complete the operation by calculating the actual center of the arc
        offset[0] = 0.5*(x-(y*h_x2_div_d));
        offset[1] = 0.5*(y+(x*h_x2_div_d));

    }
    else     // Offset mode specific computations
    {
        r = hypot(offset[0], offset[1]); // Compute arc radius for arc
    }
    // Set clockwise/counter-clockwise sign for arc computations
    uint8_t isclockwise = com->G == 2;
    // Trace the arc
    PrintLine::arc(position, target, offset, r, isclockwise);
}
#endif
extern void runBedLeveling(GCode *com);
/**
  \brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode *com)
{
    uint32_t codenum; //throw away variable
	switch (com->G)
	{
	case 0: // G0 -> G1
	case 1: // G1
#if defined(SUPPORT_LASER) && SUPPORT_LASER
	{ // disable laser for G0 moves
		bool laserOn = LaserDriver::laserOn;
		if (com->G == 0 && Printer::mode == PRINTER_MODE_LASER) {
			LaserDriver::laserOn = false;
		}
#endif // defined
		if (com->hasS()) Printer::setNoDestinationCheck(com->S != 0);
		if (Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
			if (!PrintLine::queueDeltaMove(ALWAYS_CHECK_ENDSTOPS, true, true))
			{
				Com::printWarningFLN("executeGCode / queueDeltaMove returns error");
			}
#ifdef DEBUG_QUEUE_MOVE
		{

			InterruptProtectedBlock noInts;
			int lc = (int)PrintLine::linesCount;
			int lp = (int)PrintLine::linesPos;
			int wp = (int)PrintLine::linesWritePos;
			int n = (wp - lp);
			if (n < 0) n += PRINTLINE_CACHE_SIZE;
			noInts.unprotect();
			if (n != lc)
				Com::printFLN("Buffer corrupted");
		}
#endif
#if defined(SUPPORT_LASER) && SUPPORT_LASER
		LaserDriver::laserOn = laserOn;
	}
#endif // defined
	break;
#if ARC_SUPPORT
	case 2: // CW Arc
	case 3: // CCW Arc MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
#if defined(SUPPORT_LASER) && SUPPORT_LASER
	{ // disable laser for G0 moves
		bool laserOn = LaserDriver::laserOn;
		if (com->G == 0 && Printer::mode == PRINTER_MODE_LASER) {
			LaserDriver::laserOn = false;
		}
#endif // defined
		processArc(com);
#if defined(SUPPORT_LASER) && SUPPORT_LASER
		LaserDriver::laserOn = laserOn;
	}
#endif // defined
	break;
#endif
	case 4: // G4 dwell
		Commands::waitUntilEndOfAllMoves();
		codenum = 0;
		if (com->hasP()) codenum = com->P; // milliseconds to wait
		if (com->hasS()) codenum = com->S * 1000; // seconds to wait
		codenum += millis();  // keep track of when we started waiting
		while ((uint32_t)(codenum - millis()) < 2000000000)
		{
			GCode::readFromSerial();
			Commands::checkForPeriodicalActions();
		}
		break;
#if FEATURE_RETRACTION && NUM_EXTRUDER > 0
	case 10: // G10 S<1 = long retract, 0 = short retract = default> retracts filament accoridng to stored setting
#if NUM_EXTRUDER > 1
		Extruder::current->retract(true, com->hasS() && com->S > 0);
#else
		Extruder::current->retract(true, false);
#endif
		break;
	case 11: // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
#if NUM_EXTRUDER > 1
		Extruder::current->retract(false, com->hasS() && com->S > 0);
#else
		Extruder::current->retract(false, false);
#endif
		break;
#endif // FEATURE_RETRACTION
	case 20: // G20 Units to inches
		Printer::unitIsInches = 1;
		break;
	case 21: // G21 Units to mm
		Printer::unitIsInches = 0;
		break;
	case 28:  //G28 Home all Axis one at a time
	{
		uint8_t homeAllAxis = (com->hasNoXYZ() && !com->hasE());
		if (com->hasE())
			Printer::currentPositionSteps[E_AXIS] = 0;
		if (homeAllAxis || !com->hasNoXYZ())
			Printer::homeAxis(homeAllAxis || com->hasX(), homeAllAxis || com->hasY(), homeAllAxis || com->hasZ());
		Printer::updateCurrentPosition();
	}
	break;
#if FEATURE_Z_PROBE
	case 29: // G29 3 points, build average or distortion compensation
	{
#if DISTORTION_CORRECTION
		float oldFeedrate = Printer::feedrate;
		Printer::measureDistortion();
		Printer::feedrate = oldFeedrate;
#else
		// It is not possible to go to the edges at the top, also users try
		// it often and wonder why the coordinate system is then wrong.
		// For that reason we ensure a correct behaviour by code.
		Printer::homeAxis(true, true, true);
		Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance() + EEPROM::zProbeHeight(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
		bool oldAutolevel = Printer::isAutolevelActive();
		Printer::setAutolevelActive(false);
		float sum = 0, last, oldFeedrate = Printer::feedrate;
		Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		sum = Printer::runZProbe(true, false, Z_PROBE_REPETITIONS);
		if (sum < -1) break;
		Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		last = Printer::runZProbe(false, false);
		if (last < -2) break;
		sum += last;
		Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		last = Printer::runZProbe(false, true);
		if (last < -3) break;
		sum += last;
		sum *= 0.33333333333333;
		Com::printFLN(Com::tZProbeAverage, sum);
		if (com->hasS() && com->S)
		{
#if MAX_HARDWARE_ENDSTOP_Z
			Printer::updateCurrentPosition();
			Printer::zLength += sum - Printer::currentPosition[Z_AXIS];
			Printer::updateDerivedParameter();
			Printer::homeAxis(true, true, true);
			Com::printInfoFLN(Com::tZProbeZReset);
			Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength);
#else
			Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
			Com::printFLN("Adjusted z origin");
#endif
		}
		Printer::feedrate = oldFeedrate;
		Printer::setAutolevelActive(oldAutolevel);
		if (com->hasS() && com->S == 2)
			EEPROM::storeDataIntoEEPROM();
		Printer::updateCurrentPosition(true);
		printCurrentPosition("G29 ");
		Printer::feedrate = oldFeedrate;
#endif // DISTORTION_CORRECTION
	}
	break;
	case 30: // G30 single probe set Z0
	{
		/*
		uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
		//bool oldAutolevel = Printer::isAutolevelActive();
		//Printer::setAutolevelActive(false);
		Printer::runZProbe(p & 1,p & 2);
		//Printer::setAutolevelActive(oldAutolevel);
		Printer::updateCurrentPosition(p & 1);
		//printCurrentPosition("G30 ");
		*/
		/*
		Have a P parameter which is used after init probing with G30 T.
		It calculates the the Z-probe height and saves it into EEPROM
		*/
		float deviation;
		Printer::setAutolevelActive(false);

		if (com->hasP()) {
			Com::printFLN("Adj. probe height");
			float probeHeight;
			/*
			Assume that the current zProbeHeight contains deviation offset from total length + actuation
			height of the Z-probe.
			The actual zProbeHeight = the previous value - deviation offset.
			*/
			probeHeight = EEPROM::zProbeHeight() - (Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			Com::print("\nThe new Z probe height: ");
			Com::printFloat(probeHeight, 4);
			HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, probeHeight);
			EEPROM::storeDataIntoEEPROM(false);
			Com::print(" has been stored into EEPROM\n");
			Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			//Move to the bed center
			Printer::moveTo(0.0, 0.0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		}

		/*
		Initialization parameter T moves the nozzle safe distance above the bed and Y-10 to prepare
		for manual height measurement.
		*/
		if (com->hasT()) {
			Printer::homeAxis(true, true, true);
			Printer::updateCurrentPosition();
			//Move to safe distance above the bed
			Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			//Move to P1
			Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());

			Com::printFLN("Probing...");
			//Run probe and get the deviation
			deviation = Printer::runZProbe(true, true);
			//Move to the bed center
			Printer::moveTo(0.0, 0.0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			Com::printFLN("Measuring...");
			Printer::homeAxis(true, true, true);
			Printer::moveTo(0, 0, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			Printer::moveTo(EEPROM::zProbeX1() - EEPROM::zProbeXOffset(), EEPROM::zProbeY1() - EEPROM::zProbeYOffset(), EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
			Printer::updateCurrentPosition(true);
			printCurrentPosition("M114 ");
			//Store the deviation offset temporarily as Z-probe height
			HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, EEPROM::zProbeBedDistance() - deviation + EEPROM::zProbeXY1offset());
			EEPROM::storeDataIntoEEPROM(false);
			Com::printFLN("Please adjust 0:");
		}

		/*
		If there are no additional parameters for pre-setup	then we are ready to change the zLength.
		(Assuming we have made the zProbeHeight and "0"-ing measurement with "G30 T", adjusting to 0,
		and THEN "G30 P" to store the new zProbeHeight value.)
		*/
		if (!com->hasT()) {
			Com::printFLN("Adjusting...");
			float newLength;
			Printer::homeAxis(true, true, true);
			Printer::updateCurrentPosition();
			//Move to safe distance above the bed
			Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			//Move to P1
			Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());

			Com::printFLN("Probing...");
			//Run probe and get the deviation
			deviation = Printer::runZProbe(true, true);
			//Move to the bed center
			Printer::moveTo(0.0, 0.0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());

			newLength = Printer::zLength - (EEPROM::zProbeBedDistance() - deviation);
			Com::print("\nThe new zLength: ");
			Com::printFloat(newLength, 4);

			/*
			If this is the second phase of init-calibration (parameter P) or simply G30 R (single probe),
			then store the adjusted height in EEPROM.
			*/
			if (com->hasR() || com->hasP()) {
				Printer::zLength = newLength;
				HAL::eprSetFloat(EPR_Z_LENGTH, newLength);
				EEPROM::storeDataIntoEEPROM(false);
				Com::print(" has been stored into EEPROM.\n");
				EEPROM::update(com);
                Printer::moveTo(0.0, 0.0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
				Printer::homeAxis(true, true, true);
			}

			Printer::updateCurrentPosition(true);
			Com::printFLN("Probe height calibration complete");
		}
		Commands::waitUntilEndOfAllMoves();
	}
	break;
	case 31:  // G31 display hall sensor output
		Endstops::update();
		Endstops::update();
		Com::printF(Com::tZProbeState);
		Com::printF(Endstops::zProbe() ? Com::tHSpace : Com::tLSpace);
		Com::println();
		break;
#if FEATURE_AUTOLEVEL
	case 32: // G32 Auto-Bed leveling
	{

#if DISTORTION_CORRECTION
		Printer::distortion.disable(true); // if level has changed, distortion is also invalid
#endif 
		EEPROM::readDataFromEEPROM(false);
		Printer::setAutolevelActive(false);
		// Check to see if the printer has been factory-calibrated
		if (cmpf(EEPROM::zProbeHeight(), Z_PROBE_HEIGHT)) {
			Com::printErrorFLN("The Z-probe height has not been measured!");
			break;
		}
		//Suspend heating of bed during probing to avoid interference with inductive sensors
#if HAVE_HEATED_BED
		float lastBedTemp = 0;
		if (!Printer::debugDryrun()) {
			Commands::waitUntilEndOfAllMoves();
			lastBedTemp = heatedBedController.targetTemperatureC;
			Extruder::setHeatedBedTemperature(0);
		}
#endif
		// Suspend fans
		static int lastFanSpeed = Printer::getFanSpeed();
		Commands::setFanSpeed(0);

		//remember and reset horizontal rod radius
		//float oldRadius = Printer::radius0;
		//Printer::radius0 = ROD_RADIUS;

		// It is not possible to go to the edges at the top, also users try
		// it often and wonder why the coordinate system is then wrong.
		// For that reason we ensure a correct behavior by code.
		Printer::homeAxis(true, true, true);
		Printer::moveTo(0, 0, EEPROM::zProbeBedDistance() + EEPROM::zProbeHeight(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
		//bool iterate = com->hasP() && com->P>0;
		Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;
		float h1, h2, h3, oldFeedrate = Printer::feedrate;
		Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		h1 = Printer::runZProbe(true, false, Z_PROBE_REPETITIONS);
		if (h1 < 0) {
			Printer::resetTransformationMatrix(false);
			Printer::homeAxis(true, true, true);
			break;
		}
		Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		h2 = Printer::runZProbe(false, false);
		if (h2 < 0) {
			Printer::resetTransformationMatrix(false);
			Printer::homeAxis(true, true, true);
			break;
		}
		Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		h3 = Printer::runZProbe(false, true);
		if (h3 < 0) {
			Printer::resetTransformationMatrix(false);
			Printer::homeAxis(true, true, true);
			break;
		}
		Printer::moveTo(0, 0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
#if DEBUGGING
		Com::printFLN("h1: ", Z_MAX_LENGTH - Printer::zLength + EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() * 2) - h1);
		Com::printFLN("h2: ", Z_MAX_LENGTH - Printer::zLength + EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() * 2) - h2);
		Com::printFLN("h3: ", Z_MAX_LENGTH - Printer::zLength + EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() * 2) - h3);
#endif

		//Allows additional offset for each probe point to compensate head slanting at maximum dimensions
		if (com->hasX())
			h1 += com->X;
		if (com->hasY())
			h2 += com->Y;
		if (com->hasZ())
			h3 += com->Z;
		//Head slanting compensation for each measurement point
		float foff = EEPROM::zProbeXY1offset();
		if (EEPROM::zProbeXY1offset() != 0.0) {
#if DEBUGGING
			Com::printFLN("XY1 offset: ", EEPROM::zProbeXY1offset());
#endif	
			if (abs(EEPROM::zProbeXY1offset()) > 3.0) {
				foff = EEPROM::zProbeXY1offset() - h1;
				HAL::eprSetFloat(EPR_Z_PROBE_XY1_OFFSET, foff);
				Com::printFLN("XY1 offset after: ", EEPROM::zProbeXY1offset());
			}
			h1 += foff;
		}

		if (EEPROM::zProbeXY2offset() != 0.0) {
#if DEBUGGING
			Com::printFLN("XY2 offset: ", EEPROM::zProbeXY2offset());
#endif	
			foff = EEPROM::zProbeXY2offset();
			if (abs(EEPROM::zProbeXY2offset()) > 3.0) {
				foff = EEPROM::zProbeXY2offset() - h2;
				HAL::eprSetFloat(EPR_Z_PROBE_XY2_OFFSET, foff);
				Com::printFLN("XY2 offset after: ", EEPROM::zProbeXY2offset());
			}
			h2 += foff;
		}
		if (EEPROM::zProbeXY3offset() != 0.0) {
#if DEBUGGING
			Com::printFLN("XY3 offset: ", EEPROM::zProbeXY3offset());
#endif
			foff = EEPROM::zProbeXY3offset();
			if (abs(EEPROM::zProbeXY3offset()) > 3.0) {
				foff = EEPROM::zProbeXY3offset() - h3;
				HAL::eprSetFloat(EPR_Z_PROBE_XY3_OFFSET, foff);
				Com::printFLN("XY3 offset after: ", EEPROM::zProbeXY3offset());
			}
			h3 += foff;
		}

#if DEBUGGING
		Com::printFLN("h1d: ", Z_MAX_LENGTH - Printer::zLength + EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() * 2) - h1);
		Com::printFLN("h2d: ", Z_MAX_LENGTH - Printer::zLength + EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() * 2) - h2);
		Com::printFLN("h3d: ", Z_MAX_LENGTH - Printer::zLength + EEPROM::zProbeBedDistance() + (EEPROM::zProbeHeight() * 2) - h3);
#endif 
		if (h3 < -1) break;
#if defined(MOTORIZED_BED_LEVELING) && defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS >= 2
		// h1 is reference heights, h2 => motor 0, h3 => motor 1
		h2 -= h1;
		h3 -= h1;
		MotorDriverInterface *motor2 = getMotorDriver(0);
		MotorDriverInterface *motor3 = getMotorDriver(1);
		motor2->setCurrentAs(0);
		motor3->setCurrentAs(0);
		motor2->gotoPosition(h2);
		motor3->gotoPosition(h3);
		motor2->disable();
		motor3->disable(); // now bed is even
		Printer::currentPositionSteps[Z_AXIS] = h1 * Printer::axisStepsPerMM[Z_AXIS];
#else // defined(MOTORIZED_BED_LEVELING)
		Printer::buildTransformationMatrix(h1, h2, h3);
		//-(Rxx*Ryz*y-Rxz*Ryx*y+(Rxz*Ryy-Rxy*Ryz)*x)/(Rxy*Ryx-Rxx*Ryy)
		// z = z-deviation from origin due to bed transformation
#if DEBUGGING
        float z = -((Printer::autolevelTransformation[0] * Printer::autolevelTransformation[5] -
			Printer::autolevelTransformation[2] * Printer::autolevelTransformation[3]) *
			(float)Printer::currentPositionSteps[Y_AXIS] * Printer::invAxisStepsPerMM[Y_AXIS] +
			(Printer::autolevelTransformation[2] * Printer::autolevelTransformation[4] -
				Printer::autolevelTransformation[1] * Printer::autolevelTransformation[5]) *
			(float)Printer::currentPositionSteps[X_AXIS] * Printer::invAxisStepsPerMM[X_AXIS]) /
			(Printer::autolevelTransformation[1] * Printer::autolevelTransformation[3] - Printer::autolevelTransformation[0] * Printer::autolevelTransformation[4]);
		Printer::zMin = 0;

		Com::printFLN("Z: ", z);
#endif
		//Parameter for compensating total height. E.g. in case of blue tape.		
		if (com->hasI())
			Printer::zLength += com->I;
		if (com->hasS() && com->S < 4 && com->S > 0)
		{
#if MAX_HARDWARE_ENDSTOP_Z

#if DEBUGGING
			Com::printFLN(" Current pos. Z: ", Printer::currentPosition[Z_AXIS]);
#endif
			Com::printFLN("Old printer height: ", Printer::zLength);
			//Printer::zLength += (h3 + z) - tempfl;
			float avgH = (h1 + h2 + h3) / 3;
			Com::printFLN("Height compensation: ", avgH - EEPROM::zProbeBedDistance());
			Printer::zLength += (avgH - EEPROM::zProbeBedDistance());

			Com::printFLN("New printer height: ", Printer::zLength);
#endif
			Printer::setAutolevelActive(true);
			//Z-length compensation. NB! Inverted contrary to I!
			if (EEPROM::zProbeZOffset() != 0.0) {
				Com::printFLN("Z probe z offset: ", EEPROM::zProbeZOffset());
				Printer::zLength -= EEPROM::zProbeZOffset();
				Com::printFLN("Adjusted height with coating: ", Printer::zLength);
			}

			//restore horizontal rod radius
			//Printer::radius0 = oldRadius;
			if (com->S == 2)
				EEPROM::storeDataIntoEEPROM(false);
		}
		else
		{
			if (com->hasS() && com->S == 3)
				EEPROM::storeDataIntoEEPROM(false);
		}
#if DEBUGGING
		printCurrentPosition("G32 ");
#endif 
#endif // defined(MOTORIZED_BED_LEVELING)
		Printer::updateDerivedParameter();
		Printer::updateCurrentPosition(true);
#if DEBUGGING
		printCurrentPosition("G32 ");
#endif
		if (!com->hasP()) { //If we have the P param. don't do homing
			Printer::homeAxis(true, true, true);
		}
			Printer::feedrate = oldFeedrate;
			//Resume bed heating
#if HAVE_HEATED_BED
			if (!Printer::debugDryrun()) {
				Commands::waitUntilEndOfAllMoves();
				Extruder::setHeatedBedTemperature(lastBedTemp);
			}
#endif
			//Restore fan speed
			Commands::setFanSpeed(lastFanSpeed, false);
			Commands::setFanSpeed(lastFanSpeed);
		}
		break;
#endif
	case 36: // G36
		if (com->hasS()) {
			Printer::setAutolevelActive(false);
			//Printer::zLength = Z_MAX_LENGTH;
			//HAL::eprSetFloat(EPR_Z_LENGTH, Z_MAX_LENGTH);
			Printer::zLength = retDefHeight();
			HAL::eprSetFloat(EPR_Z_LENGTH, Printer::zLength);

			Printer::updateDerivedParameter();
			Printer::homeAxis(true, true, true);
			Printer::updateCurrentPosition(true);
			Printer::moveTo(0, 0, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);

			//Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;
			Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		}
		Printer::updateCurrentPosition(true);
		Printer::updateDerivedParameter();
		printCurrentPosition("M114 ");
		if (com->hasX()) {
			float xf = EEPROM::zProbeBedDistance() - (Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			Com::printFLN(" xf: ", xf);
			Com::printFLN(" xz: ", Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			HAL::eprSetFloat(EPR_Z_PROBE_XY1_OFFSET, xf);
			Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		}
		if (com->hasY()) {
			float yf = EEPROM::zProbeBedDistance() - (Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			Com::printFLN(" yf: ", yf);
			Com::printFLN(" yz: ", Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			HAL::eprSetFloat(EPR_Z_PROBE_XY2_OFFSET, yf);
			Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		}
		if (com->hasZ()) {
			float zf = EEPROM::zProbeBedDistance() - (Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			Com::printFLN(" zf: ", zf);
			Com::printFLN(" zz: ", Printer::currentPositionSteps[Z_AXIS] / Printer::axisStepsPerMM[Z_AXIS]);
			HAL::eprSetFloat(EPR_Z_PROBE_XY3_OFFSET, zf);
			Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeBedDistance() * 2, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			Printer::moveTo(0, 0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			Printer::homeAxis(true, true, true);
		}
		Printer::updateCurrentPosition(true);
		Printer::updateDerivedParameter();
		printCurrentPosition("M114 ");

		break;
#endif
	case 37: {
		Com::printFLN("DIY measure...");
		float px0 = 0.0,
			py0 = 0.0,
			px1 = EEPROM::zProbeX1(),
			px2 = EEPROM::zProbeX2(),
			px3 = EEPROM::zProbeX3(),
			py1 = EEPROM::zProbeY1(),
			py2 = EEPROM::zProbeY2(),
			py3 = EEPROM::zProbeY3(),

			p2x4 = 0.0,
			p2y4 = -79.0,
			p2x5 = 68.42,
			p2y5 = 39.5,
			p2x6 = -68.42,
			p2y6 = 39.5,

			p3x4 = 0.0,
			p3y4 = -118.5,
			p3x5 = 102.634,
			p3y5 = 59.254,
			p3x6 = -102.634,
			p3y6 = 59.254,

			p4x4 = 0.0,
			p4y4 = -158,
			p4x5 = 136.846,
			p4y5 = 79.006,
			p4x6 = -136.846,
			p4y6 = 79.006,

			pAbove = 5.0;

		//If MP40
		if (Commands::retDefHeight() > 400) {
			//If above probing height
			if (Printer::currentPosition[Z_AXIS] > EEPROM::zProbeBedDistance() + 1.0) {
				//Go to first position at 0.0
				Printer::homeAxis(true, true, true);
				//Move to safe distance above the bed
				Printer::moveToReal(px0, py0, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			//If in the center 
			else if ((abs(Printer::currentPosition[X_AXIS] - px0) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py0) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 2nd position
				Printer::moveToReal(px1, py1, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			//If in the 2nd position
			else if ((abs(Printer::currentPosition[X_AXIS] - px1) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py1) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 3rd position
				Printer::moveToReal(px2, py2, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - px2) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py2) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 4th position
				Printer::moveToReal(px3, py3, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - px3) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py3) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 5th position
				Printer::moveToReal(p4x4, p4y4, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p4x4) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p4y4) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 6th position
				Printer::moveToReal(p4x5, p4y5, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p4x5) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p4y5) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 7th position
				Printer::moveToReal(p4x6, p4y6, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p4x6) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p4y6) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to first position
				Printer::moveToReal(px0, py0, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else
				Com::printWarningFLN("Not in a valid position!");
		}
		//If MP30
		else if (Commands::retDefHeight() > 300) {
			//If above probing height
			if (Printer::currentPosition[Z_AXIS] > EEPROM::zProbeBedDistance() + 1.0) {
				//Go to first position at 0.0
				Printer::homeAxis(true, true, true);
				//Move to safe distance above the bed
				Printer::moveToReal(px0, py0, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			//If in the center 
			else if ((abs(Printer::currentPosition[X_AXIS] - px0) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py0) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 2nd position
				Printer::moveToReal(px1, py1, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			//If in the 2nd position
			else if ((abs(Printer::currentPosition[X_AXIS] - px1) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py1) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 3rd position
				Printer::moveToReal(px2, py2, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - px2) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py2) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 4th position
				Printer::moveToReal(px3, py3, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - px3) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py3) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 5th position
				Printer::moveToReal(p3x4, p3y4, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p3x4) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p3y4) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 6th position
				Printer::moveToReal(p3x5, p3y5, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p3x5) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p3y5) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 7th position
				Printer::moveToReal(p3x6, p3y6, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p3x6) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p3y6) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to first position
				Printer::moveToReal(px0, py0, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else
				Com::printWarningFLN("Not in a valid position!");
		}
		//If MP20
		else {
			//If above probing height
			if (Printer::currentPosition[Z_AXIS] > EEPROM::zProbeBedDistance() + 1.0) {
				//Go to first position at 0.0
				Printer::homeAxis(true, true, true);
				//Move to safe distance above the bed
				Printer::moveToReal(px0, py0, EEPROM::zProbeBedDistance(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			//If in the center 
			else if ((abs(Printer::currentPosition[X_AXIS] - px0) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py0) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 2nd position
				Printer::moveToReal(px1, py1, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			//If in the 2nd position
			else if ((abs(Printer::currentPosition[X_AXIS] - px1) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py1) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 3rd position
				Printer::moveToReal(px2, py2, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - px2) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py2) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 4th position
				Printer::moveToReal(px3, py3, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - px3) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - py3) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 5th position
				Printer::moveToReal(p2x4, p2y4, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p2x4) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p2y4) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 6th position
				Printer::moveToReal(p2x5, p2y5, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p2x5) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p2y5) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to 7th position
				Printer::moveToReal(p2x6, p2y6, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else if ((abs(Printer::currentPosition[X_AXIS] - p2x6) < 1.0) && (abs(Printer::currentPosition[Y_AXIS] - p2y6) < 1.0)) {
				//Move to safe distance above the bed
				Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, pAbove, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				//Go to first position
				Printer::moveToReal(px0, py0, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
				Printer::updateCurrentPosition(true);
				printCurrentPosition("M114 ");
			}
			else
				Com::printWarningFLN("Not in a valid position!");
		}

	}
			 break;
#if DISTORTION_CORRECTION
	case 33: {
		if (com->hasL()) { // G33 L0 - List distortion matrix
			Printer::distortion.showMatrix();
		}
		else if (com->hasR()) { // G33 R0 - Reset distortion matrix
			Printer::distortion.resetCorrection();
		}
		else if (com->hasX() || com->hasY() || com->hasZ()) { // G33 X<xpos> Y<ypos> Z<zCorrection> - Set correction for nearest point
			if (com->hasX() && com->hasY() && com->hasZ()) {
				Printer::distortion.set(com->X, com->Y, com->Z);
			}
			else {
				Com::printErrorFLN("You need to define X, Y and Z to set a point!");
			}
		}
		else { // G33
			float oldFeedrate = Printer::feedrate;
			Printer::measureDistortion();
			Printer::feedrate = oldFeedrate;
		}
	}
			 break;
#endif
	case 35: 
		Com::printErrorFLN("ERR2: Not a latching switch probe!");
		break;
			 /*
			 Custom(-izable) probing function for measuring at 
				or around a given point.
			 Possible parameters:
			 R - go home and center before/after probing procedure
				0 - [default] do not go home at all
				1 - go home after finishing
				2 - go home before probing
				3 - go home both before and after
			 P[0-4] - probing coordinate point. Default - 0 = at center,
				1-3 at corresponding predefined zProbe XY points,
				4 - use with X and Y parameters to define custom point
			 X/Y[-/+radius corrdinates] - use with P4
			 I[0.0-999.0] - distance between each probing point repetition. Default = 1.0
			 S[1-999] - ^2 how far to probe arond given coordinate (square center)
			 Z[] - don't disable autolevel before probing
			 J[0/1] - allow probing below zMaxLength. NB! Probe won't
				trigger if given bed probing point is actually
				lower than max Z length and will output defined 
				Z-probe-bed	distance. Default = allow
			 E.g.:
			 G38 P4 X-69.42 Y-39.5 R3 J0 S1
			 Does homing, moves to X:-69.42 Y:-39.5, probes bed once
				(if reachable) and returns home.			
			 */
	case 38:
	{
		if (!com->hasZ()) 
			Printer::setAutolevelActive(false);
		if (com->hasR() && com->R > 1.1) {
			Printer::homeAxis(true, true, true);
			Printer::moveTo(0, 0, EEPROM::zProbeBedDistance() + EEPROM::zProbeHeight(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
		}
		Printer::allowBelow = true;
		if (com->hasJ() && com->J < 1)
			Printer::allowBelow = false;
		//bool iterate = com->hasP() && com->P>0;
		/*Printer::coordinateOffset[X_AXIS] = Printer::coordinateOffset[Y_AXIS] = Printer::coordinateOffset[Z_AXIS] = 0;*/
		int ST = 0;
		int Max = 1;
		float ptx = 0.0;
		float pty = 0.0;
		float incr = 1.0;
		if (com->hasS() && com->S > 0)
			Max = com->S;
		if (com->hasI() && com->I != 0.0)
			incr = com->I;
        float zx1 = 0;
        float zy1 = 0;
		if (com->hasP())
			switch (com->P) {
			case 0: {
				zx1 = 0.0;
				zy1 = 0.0;
			}
					break;
			case 1: {
				zx1 = EEPROM::zProbeX1();
				zy1 = EEPROM::zProbeY1();
			}
					break;
			case 2: {
				zx1 = EEPROM::zProbeX2();
				zy1 = EEPROM::zProbeY2();
			}
					break;
			case 3: {
				zx1 = EEPROM::zProbeX3();
				zy1 = EEPROM::zProbeY3();
			}
					break;
			case 4: { //custom point at coordinates given
				zx1 = 0.0;
				zy1 = 0.0;
				if (com->hasX())
					zx1 = com->X;
				if (com->hasY())
					zy1 = com->Y;
			}
					break;
			default: {
				zx1 = 0.0;
				zy1 = 0.0;
			}
					break;
			}

		ptx = zx1 - (Max / 2);
		pty = zy1 - (Max / 2);

		Printer::moveTo(zx1, zy1, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());

		for (int mx = ST; (ptx + mx) < Max; mx += incr) {
			Printer::moveTo(ptx + mx, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			for (int my = ST; (pty + my) < Max; my += incr) {
				Printer::moveTo(IGNORE_COORDINATE, pty + my, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
				Printer::runZProbe(true, false, Z_PROBE_REPETITIONS);
			}
		}
		//Com::printFLN("Finished");
		//Printer::setAutolevelActive(false);
		if (com->hasR() && ((com->R > 0.1 && com->R < 2) || com->R > 2.1)) {
			Printer::moveTo(0.0 + EEPROM::zProbeXOffset(), 0.0 + EEPROM::zProbeYOffset(), EEPROM::zProbeBedDistance() + EEPROM::zProbeHeight(), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
			Printer::homeAxis(true, true, true);
		}
		Printer::allowBelow = true;
	}
		break;
    case 39:
        if (com->hasI()) {
            Printer::babyStep(com->I);
        }
        break;
    case 90: // G90
        Printer::relativeCoordinateMode = false;
        if(com->internalCommand)
            Com::printInfoFLN("Absolute positioning");
        break;
    case 91: // G91
        Printer::relativeCoordinateMode = true;
        if(com->internalCommand)
            Com::printInfoFLN("Relative positioning");
        break;
    case 92: // G92
    {
        float xOff = Printer::coordinateOffset[X_AXIS];
        float yOff = Printer::coordinateOffset[Y_AXIS];
        float zOff = Printer::coordinateOffset[Z_AXIS];
        if(com->hasX()) xOff = Printer::convertToMM(com->X) - Printer::currentPosition[X_AXIS];
        if(com->hasY()) yOff = Printer::convertToMM(com->Y) - Printer::currentPosition[Y_AXIS];
        if(com->hasZ()) zOff = Printer::convertToMM(com->Z) - Printer::currentPosition[Z_AXIS];
        Printer::setOrigin(xOff, yOff, zOff);
        if(com->hasE())
        {
            Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
        }
    }
    break;
    case 100: // G100 Calibrate floor or rod radius
    {
        // Using manual control, adjust hot end to contact floor.
        // G100 <no arguments> No action. Avoid accidental floor reset.
        // G100 [X] [Y] [Z] set floor for argument passed in. Number ignored and may be absent.
        // G100 R with X Y or Z flag error, sets only floor or radius, not both.
        // G100 R[n] Add n to radius. Adjust to be above floor if necessary
        // G100 R[0] set radius based on current z measurement. Moves to (0,0,0)
        float currentZmm = Printer::currentPosition[Z_AXIS];
        if (currentZmm/Printer::zLength > 0.1)
        {
            Com::printErrorFLN("Calibration code is limited to bottom 10% of Z height");
            break;
        }
        if (com->hasR())
        {
            if (com->hasX() || com->hasY() || com->hasZ())
                Com::printErrorFLN("Cannot set radius and floor at same time.");
            else if (com->R != 0)
            {
                //add r to radius
                if (abs(com->R) <= 10) EEPROM::incrementRodRadius(com->R);
                else Com::printErrorFLN("Calibration movement is limited to 10mm.");
            }
            else
            {
                // auto set radius. Head must be at 0,0 and touching
                // Z offset will be corrected for.
                if (Printer::currentPosition[X_AXIS] == 0
                        && Printer::currentPosition[Y_AXIS] == 0)
                {
                    if(Printer::isLargeMachine())
                    {
                        // calculate radius assuming we are at surface
                        // If Z is greater than 0 it will get calculated out for correct radius
                        // Use either A or B tower as they acnhor x cartesian axis and always have
                        // Radius distance to center in simplest set up.
                        float h = Printer::deltaDiagonalStepsSquaredB.f;
                        unsigned long bSteps = Printer::currentDeltaPositionSteps[B_TOWER];
                        // The correct Rod Radius would put us here at z==0 and B height is
                        // square root (rod length squared minus rod radius squared)
                        // Reverse that to get calculated Rod Radius given B height
                        h -= RMath::sqr((float)bSteps);
                        h = sqrt(h);
                        EEPROM::setRodRadius(h*Printer::invAxisStepsPerMM[Z_AXIS]);
                    }
                    else
                    {
                        // calculate radius assuming we are at surface
                        // If Z is greater than 0 it will get calculated out for correct radius
                        // Use either A or B tower as they acnhor x cartesian axis and always have
                        // Radius distance to center in simplest set up.
                        unsigned long h = Printer::deltaDiagonalStepsSquaredB.l;
                        unsigned long bSteps = Printer::currentDeltaPositionSteps[B_TOWER];
                        // The correct Rod Radius would put us here at z==0 and B height is
                        // square root (rod length squared minus rod radius squared)
                        // Reverse that to get calculated Rod Radius given B height
                        h -= RMath::sqr(bSteps);
                        h = SQRT(h);
                        EEPROM::setRodRadius(h*Printer::invAxisStepsPerMM[Z_AXIS]);
                    }
                }
                else
                    Com::printErrorFLN("First move to touch at x,y=0,0 to auto-set radius.");
            }
        }
        else
        {
            bool tooBig = false;
            if (com->hasX())
            {
                if (abs(com->X) <= 10)
                    EEPROM::setTowerXFloor(com->X + currentZmm + Printer::xMin);
                else tooBig = true;
            }
            if (com->hasY())
            {
                if (abs(com->Y) <= 10)
                    EEPROM::setTowerYFloor(com->Y + currentZmm + Printer::yMin);
                else tooBig = true;
            }
            if (com->hasZ())
            {
                if (abs(com->Z) <= 10)
                    EEPROM::setTowerZFloor(com->Z + currentZmm + Printer::zMin);
                else tooBig = true;
            }
            if (tooBig)
                Com::printErrorFLN("Calibration movement is limited to 10mm.");
        }
        // after adjusting zero, physical position is out of sync with memory position
        // this could cause jerky movement or push head into print surface.
        // moving gets back into safe zero'ed position with respect to newle set floor or Radius.
        Printer::moveTo(IGNORE_COORDINATE,IGNORE_COORDINATE,12.0,IGNORE_COORDINATE,IGNORE_COORDINATE);
        break;
    }
    case 131: // G131 Remove offset
    {
        float cx,cy,cz;
        Printer::realPosition(cx,cy,cz);
        float oldfeedrate = Printer::feedrate;
        Printer::offsetX = 0;
        Printer::offsetY = 0;
        Printer::moveToReal(cx,cy,cz,IGNORE_COORDINATE,Printer::homingFeedrate[X_AXIS]);
        Printer::feedrate = oldfeedrate;
        Printer::updateCurrentPosition();
    }
    break;
    case 132: // G132 Calibrate endstop offsets
    {
// This has the probably unintended side effect of turning off leveling.
        Printer::setAutolevelActive(false); // don't let transformations change result!
        Printer::coordinateOffset[X_AXIS] = 0;
        Printer::coordinateOffset[Y_AXIS] = 0;
        Printer::coordinateOffset[Z_AXIS] = 0;
// I think this is coded incorrectly, as it depends on the biginning position of the
// of the hot end, and so should first move to x,y,z= 0,0,0, but as that may not
// be possible if the printer is not in the homes/zeroed state, the printer
// cannot safely move to 0 z coordinate without crashong into the print surface.
// so other than commenting, I'm not meddling.
// but you will always get different counts from different positions.
        Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
        int32_t m = RMath::max(Printer::stepsRemainingAtXHit,RMath::max(Printer::stepsRemainingAtYHit,Printer::stepsRemainingAtZHit));
        int32_t offx = m-Printer::stepsRemainingAtXHit;
        int32_t offy = m-Printer::stepsRemainingAtYHit;
        int32_t offz = m-Printer::stepsRemainingAtZHit;

		Com::printFLN(Com::tTower1,((Printer::zMaxSteps*2)-Printer::stepsRemainingAtXHit)/Printer::axisStepsPerMM[X_AXIS]);
		Com::printFLN(Com::tTower2,((Printer::zMaxSteps*2)-Printer::stepsRemainingAtYHit)/Printer::axisStepsPerMM[Y_AXIS]);
		Com::printFLN(Com::tTower3,((Printer::zMaxSteps*2)-Printer::stepsRemainingAtZHit)/Printer::axisStepsPerMM[Z_AXIS]);
        Com::printFLN(Com::tTower1,offx/Printer::axisStepsPerMM[X_AXIS]);
        Com::printFLN(Com::tTower2,offy/Printer::axisStepsPerMM[Y_AXIS]);
        Com::printFLN(Com::tTower3,offz/Printer::axisStepsPerMM[Z_AXIS]);
#if EEPROM_MODE != 0
        if(com->hasS() && com->S > 0)
        {
            EEPROM::setDeltaTowerXOffsetSteps(offx);
            EEPROM::setDeltaTowerYOffsetSteps(offy);
            EEPROM::setDeltaTowerZOffsetSteps(offz);
        }
#endif
        PrintLine::moveRelativeDistanceInSteps(0, 0, -5*Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);
        Printer::homeAxis(true,true,true);
    }
    break;
    case 133: // G133 Measure steps to top
    {
        bool oldAuto = Printer::isAutolevelActive();
        Printer::setAutolevelActive(false); // don't let transformations change result!
        Printer::currentPositionSteps[X_AXIS] = 0;
        Printer::currentPositionSteps[Y_AXIS] = 0;
        Printer::currentPositionSteps[Z_AXIS] = 0;
        Printer::coordinateOffset[X_AXIS] = 0;
        Printer::coordinateOffset[Y_AXIS] = 0;
        Printer::coordinateOffset[Z_AXIS] = 0;
        Printer::currentDeltaPositionSteps[A_TOWER] = 0;
        Printer::currentDeltaPositionSteps[B_TOWER] = 0;
        Printer::currentDeltaPositionSteps[C_TOWER] = 0;
// similar to comment above, this will get a different answer from any different starting point
// so it is unclear how this is helpful. It must start at a well defined point.
        Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
        int32_t offx = HOME_DISTANCE_STEPS-Printer::stepsRemainingAtXHit;
        int32_t offy = HOME_DISTANCE_STEPS-Printer::stepsRemainingAtYHit;
        int32_t offz = HOME_DISTANCE_STEPS-Printer::stepsRemainingAtZHit;
        Com::printFLN(Com::tTower1,offx);
        Com::printFLN(Com::tTower2,offy);
        Com::printFLN(Com::tTower3,offz);
        Printer::setAutolevelActive(oldAuto);
        PrintLine::moveRelativeDistanceInSteps(0, 0, Printer::axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
        Printer::homeAxis(true,true,true);
    }
    break;
    case 135: // G135
        Com::printF("CompDelta:",Printer::currentDeltaPositionSteps[A_TOWER]);
        Com::printF(Com::tComma,Printer::currentDeltaPositionSteps[B_TOWER]);
        Com::printFLN(Com::tComma,Printer::currentDeltaPositionSteps[C_TOWER]);
#ifdef DEBUG_REAL_POSITION
        Com::printF("RealDelta:",Printer::realDeltaPositionSteps[A_TOWER]);
        Com::printF(Com::tComma,Printer::realDeltaPositionSteps[B_TOWER]);
        Com::printFLN(Com::tComma,Printer::realDeltaPositionSteps[C_TOWER]);
#endif
        Printer::updateCurrentPosition();
        Com::printF("PosFromSteps:");
        printCurrentPosition("G134 ");
        break;

#if FEATURE_Z_PROBE
    case 134: // - G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in nozzle!) Px = reference extruder, Sx = only measure extrude x against reference, Zx = add to measured z distance for Sx for correction.
        {
            float z = com->hasZ() ? com->Z : 0;
            int p = com->hasP() ? com->P : 0;
            int s = com->hasS() ? com->S : -1;
            extruder[p].zOffset = 0;
            Extruder::selectExtruderById(p);
            float refHeight = Printer::runZProbe(true,false,true);
            for(int i = 0;i < NUM_EXTRUDER;i++) {
                if(i == p) continue;
                if(s >= 0 && i != s) continue;
                extruder[i].zOffset = 0;
                Extruder::selectExtruderById(i);
                float height = Printer::runZProbe(false,false);
                extruder[i].zOffset = (height - refHeight + z)*Printer::axisStepsPerMM[Z_AXIS];
            }
            Extruder::selectExtruderById(p);
            Printer::runZProbe(false,true);
#if EEPROM_MODE != 0
            EEPROM::storeDataIntoEEPROM(0);
#endif
        }
        break;
#endif
#if defined(NUM_MOTOR_DRIVERS) && NUM_MOTOR_DRIVERS > 0
    case 201:
        commandG201(*com);
        break;
    case 202:
        commandG202(*com);
        break;
    case 203:
        commandG203(*com);
        break;
    case 204:
        commandG204(*com);
        break;
#endif // defined
    default:
        if(Printer::debugErrors())
        {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
}
/**
  \brief Execute the G command stored in com.
*/
void Commands::processMCode(GCode *com)
{
    switch( com->M )
    {
    case 3: // Spindle/laser on
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        if(Printer::mode == PRINTER_MODE_LASER) {
            if(com->hasS())
                LaserDriver::intensity = constrain(com->S,0,255);
            LaserDriver::laserOn = true;
            Com::printFLN("LaserOn:",(int)LaserDriver::intensity);
        }
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        if(Printer::mode == PRINTER_MODE_CNC) {
            waitUntilEndOfAllMoves();
            CNCDriver::spindleOnCW(com->hasS() ? com->S : 0);
        }
#endif // defined
        break;
    case 4: // Spindle CCW
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        if(Printer::mode == PRINTER_MODE_CNC) {
            waitUntilEndOfAllMoves();
            CNCDriver::spindleOnCCW(com->hasS() ? com->S : 0);
        }
#endif // defined
        break;
    case 5: // Spindle/laser off
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        if(Printer::mode == PRINTER_MODE_LASER) {
            LaserDriver::laserOn = false;
        }
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        if(Printer::mode == PRINTER_MODE_CNC) {
            waitUntilEndOfAllMoves();
            CNCDriver::spindleOff();
        }
#endif // defined
        break;
    case 42: //M42 -Change pin status via gcode
        if (com->hasP())
        {
            int pin_number = com->P;
            for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++)
            {
                if (sensitive_pins[i] == pin_number)
                {
                    pin_number = -1;
                    break;
                }
            }
            if (pin_number > -1)
            {
                if(com->hasS())
                {
                    if(com->S >= 0 && com->S <= 255)
                    {
                        pinMode(pin_number, OUTPUT);
                        digitalWrite(pin_number, com->S > 0xff/2 ? 1 : 0);
                        Com::printF(Com::tSetOutputSpace, pin_number);
                        Com::printFLN(Com::tSpaceToSpace,(int)com->S);
                    }
                    else
                        Com::printErrorFLN("Illegal S value for M42");
                }
                else
                {
                    pinMode(pin_number, INPUT_PULLUP);
                    Com::printF(Com::tSpaceToSpace, pin_number);
                    Com::printFLN(Com::tSpaceIsSpace, digitalRead(pin_number));
                }
            }
            else
            {
                Com::printErrorFLN("Pin can not be set by M42, is in sensitive pins! ");
            }
        }
        break;
    case 80: // M80 - ATX Power On
        break;
    case 81: // M81 - ATX Power Off
        break;
    case 82: // M82
        Printer::relativeExtruderCoordinateMode = false;
        break;
    case 83: // M83
        Printer::relativeExtruderCoordinateMode = true;
        break;
    case 84: // M84
        if(com->hasS())
        {
            stepperInactiveTime = com->S * 1000;
        }
        else
        {
            Commands::waitUntilEndOfAllMoves();
            Printer::kill(true);
        }
        break;
    case 85: // M85
        if(com->hasS())
            maxInactiveTime = (int32_t)com->S * 1000;
        else
            maxInactiveTime = 0;
        break;
    case 92: // M92
        if(com->hasX()) Printer::axisStepsPerMM[X_AXIS] = com->X;
        if(com->hasY()) Printer::axisStepsPerMM[Y_AXIS] = com->Y;
        if(com->hasZ()) Printer::axisStepsPerMM[Z_AXIS] = com->Z;
        Printer::updateDerivedParameter();
        if(com->hasE())
        {
            Extruder::current->stepsPerMM = com->E;
            Extruder::selectExtruderById(Extruder::current->id);
        }
        break;
    case 99: // M99 S<time>
    {
        millis_t wait = 10000;
        if(com->hasS())
            wait = 1000*com->S;
        if(com->hasX())
            Printer::disableXStepper();
        if(com->hasY())
            Printer::disableYStepper();
        if(com->hasZ())
            Printer::disableZStepper();
        wait += millis();
        while(wait-millis() < 100000)
        {
            Printer::defaultLoopActions();
        }
        if(com->hasX())
            Printer::enableXStepper();
        if(com->hasY())
            Printer::enableYStepper();
        if(com->hasZ())
            Printer::enableZStepper();
    }
    break;

    case 104: // M104 temperature
#if NUM_EXTRUDER > 0
		if (reportTempsensorError()) break;
        if(Printer::debugDryrun()) break;
#ifdef EXACT_TEMPERATURE_TIMING
        Commands::waitUntilEndOfAllMoves();
#else
        if(com->hasP() || (com->hasS() && com->S == 0))
            Commands::waitUntilEndOfAllMoves();
#endif
        if (com->hasS()) 
        {	
			if (com->hasT() && com->T < NUM_EXTRUDER)
				Extruder::setTemperatureForExtruder(com->S, com->T, com->hasF() && com->F > 0);
            else
                Extruder::setTemperatureForExtruder(com->S, Extruder::current->id, com->hasF() && com->F > 0);
        }
#if BED_LEDS
		//Light.ShowTemps();
#endif
#endif
        break;
    case 140: // M140 set bed temp
        if(reportTempsensorError()) break;
        if(Printer::debugDryrun()) break;
        if (com->hasS()) Extruder::setHeatedBedTemperature(com->S);
#if BED_LEDS
		//Light.ShowTemps();
#endif
        break;
    case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
		if (com->hasS())
			printTemperature();
		else
			printTemperatures(com->hasX());
        break;
    case 109: // M109 - Wait for extruder heater to reach target.
#if NUM_EXTRUDER > 0
    {
        if(reportTempsensorError()) break;
        if(Printer::debugDryrun()) break;
        Commands::waitUntilEndOfAllMoves();
        Extruder *actExtruder = Extruder::current;
        if(com->hasT() && com->T < NUM_EXTRUDER) actExtruder = &extruder[com->T];
        if (com->hasS()) Extruder::setTemperatureForExtruder(com->S, actExtruder->id, /*com->hasF() && com->F > 0,*/ true);
    }
#endif
    break;
    case 190: // M190 - Wait bed for heater to reach target.
		{
#if HAVE_HEATED_BED
        if(Printer::debugDryrun()) break;
        Commands::waitUntilEndOfAllMoves();
#if HAVE_HEATED_BED
        if (com->hasS()) Extruder::setHeatedBedTemperature(com->S);
#if defined(SKIP_M190_IF_WITHIN) && SKIP_M190_IF_WITHIN>0
        if(abs(heatedBedController.currentTemperatureC-heatedBedController.targetTemperatureC) < SKIP_M190_IF_WITHIN) break;
#endif
	    uint32_t codenum; //throw away variable
        codenum = millis();
        while(heatedBedController.currentTemperatureC + 0.5 < heatedBedController.targetTemperatureC && heatedBedController.targetTemperatureC > 25.0)
        {
            if( (millis()-codenum) > 1000 )   //Print Temp Reading every 1 second while heating up.
            {
                printTemperatures();
                codenum = millis();
            }
            Commands::checkForPeriodicalActions();
        }
#endif
#endif
		}
        break;
#if NUM_TEMPERATURE_LOOPS > 0
    case 116: // Wait for temperatures to reach target temperature
        for(fast8_t h = 0; h < NUM_TEMPERATURE_LOOPS; h++)
        {
            tempController[h]->waitForTargetTemperature();
        }
        break;
#endif
#if FAN_PIN>-1
    case 106: // M106 Fan On
        if(com->hasP())
			if(com->P == 1) setFan2Speed(com->hasS() ? com->S : 255);
		    else setFan3Speed(com->hasS() ? com->S : 255);
		else
            setFanSpeed(com->hasS() ? com->S : 255, true);
        break;
    case 107: // M107 Fan Off
        if(com->hasP())
			if(com->P == 1) setFan2Speed(0);
		    else setFan3Speed(0);
		else
            setFanSpeed(0, true);
        break;
#endif
    case 111: // M111 enable/disable run time debug flags
        if(com->hasS()) Printer::setDebugLevel(static_cast<uint8_t>(com->S));
        if(com->hasP())
        {
            if (com->P > 0) Printer::debugSet(static_cast<uint8_t>(com->P));
            else Printer::debugReset(static_cast<uint8_t>(-com->P));
        }
        if(Printer::debugDryrun())   // simulate movements without printing
        {
#if NUM_EXTRUDER>1
            for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
                Extruder::setTemperatureForExtruder(0, i);
#else
            Extruder::setTemperatureForExtruder(0, 0);
#endif
#if HAVE_HEATED_BED != 0
            Extruder::setHeatedBedTemperature(0);
#endif
        }
        break;
    case 115: // M115
        Com::printFLN(Com::tFirmware);
        reportPrinterUsage();
        Printer::reportPrinterMode();
        break;
    case 114: // M114
        printCurrentPosition("M114 ");
        break;
    case 117: // M117 message to lcd
        break;
    case 119: // M119
        Commands::waitUntilEndOfAllMoves();
        Endstops::update();
        Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
        Endstops::report();
        break;
#if MIXING_EXTRUDER > 0
    case 163: // M163 S<extruderNum> P<weight>  - Set weight for this mixing extruder drive
        if(com->hasS() && com->hasP() && com->S < NUM_EXTRUDER && com->S >= 0)
            Extruder::setMixingWeight(com->S, com->P);
		Extruder::recomputeMixingExtruderSteps();
        break;
    case 164: /// M164 S<virtNum> P<0 = dont store eeprom,1 = store to eeprom> - Store weights as virtual extruder S
        if(!com->hasS() || com->S < 0 || com->S >= VIRTUAL_EXTRUDER) break; // ignore illigal values
        for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        {
            extruder[i].virtualWeights[com->S] = extruder[i].mixingW;
        }
#if EEPROM_MODE != 0
        if(com->hasP() && com->P != 0)  // store permanently to eeprom
            EEPROM::storeMixingRatios();
#endif
        break;
#endif // MIXING_EXTRUDER
    case 200: // M200 T<extruder> D<diameter>
    {
        uint8_t extruderId = Extruder::current->id;
        if(com->hasT() && com->T < NUM_EXTRUDER)
            extruderId = com->T;
        float d = 0;
        if(com->hasR())
            d = com->R;
        if(com->hasD())
            d = com->D;
        extruder[extruderId].diameter = d;
        if(extruderId == Extruder::current->id)
            changeFlowrateMultiply(Printer::extrudeMultiply);
        if(d == 0)
        {
            Com::printFLN("Disabled volumetric extrusion for extruder ",static_cast<int>(extruderId));
        }
        else
        {
            Com::printF("Set volumetric extrusion for extruder ",static_cast<int>(extruderId));
            Com::printFLN(" to ",d);
        }
    }
    break;
#if RAMP_ACCELERATION
    case 201: // M201
        if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
        if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
        if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
        if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
        Printer::updateDerivedParameter();
        break;
    case 202: // M202
        if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = com->X;
        if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
        if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
        if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
        Printer::updateDerivedParameter();
        break;
#endif
    case 203: // M203 Temperature monitor
        monitorTemps = (com->hasS() && com->S) ? true : false;
        break;
    case 204: // M204
    {
        TemperatureController *temp = &Extruder::current->tempControl;
        if(com->hasS())
        {
            if(com->S<0) break;
            if(com->S<NUM_EXTRUDER) temp = &extruder[com->S].tempControl;
#if HAVE_HEATED_BED
            else temp = &heatedBedController;
#else
            else break;
#endif
        }
        if(com->hasX()) temp->pidPGain = com->X;
        if(com->hasY()) temp->pidIGain = com->Y;
        if(com->hasZ()) temp->pidDGain = com->Z;
        temp->updateTempControlVars();
    }
    break;
    case 205: // M205 Show EEPROM settings
        EEPROM::writeSettings();
        break;
    case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
        EEPROM::update(com);
        break;
    case 207: // M207 X<XY jerk> Z<Z Jerk>
        if(com->hasX())
            Printer::maxJerk = com->X;
        if(com->hasE())
        {
            Extruder::current->maxStartFeedrate = com->E;
            Extruder::selectExtruderById(Extruder::current->id);
        }
        Com::printFLN(Com::tJerkColon,Printer::maxJerk);
        break;
    case 209: // M209 S<0/1> Enable/disable autoretraction
        if(com->hasS())
            Printer::setAutoretract(com->S != 0);
        break;
    case 220: // M220 S<Feedrate multiplier in percent>
        changeFeedrateMultiply(com->getS(100));
        break;
    case 221: // M221 S<Extrusion flow multiplier in percent>
        changeFlowrateMultiply(com->getS(100));
        break;
    case 228: // M228 P<pin> S<state 0/1> - Wait for pin getting state S
        if(!com->hasS() || !com->hasP())
            break;
        {
            bool comp = com->S;
            if(com->hasX()) {
                if(com->X == 0)
                    HAL::pinMode(com->S,INPUT);
                else
                    HAL::pinMode(com->S,INPUT_PULLUP);
            }
            do {
                Commands::checkForPeriodicalActions();
            } while(HAL::digitalRead(com->P) != comp);
        }
        break;
#if USE_ADVANCE
    case 223: // M223 Extruder interrupt test
        if(com->hasS())
        {
            InterruptProtectedBlock noInts;
            Printer::extruderStepsNeeded += com->S;
		}
        break;
    case 232: // M232
        Com::printF(Com::tLinearStepsColon,maxadv2);
#if ENABLE_QUADRATIC_ADVANCE
        Com::printF(Com::tQuadraticStepsColon,maxadv);
#endif
        Com::printFLN(Com::tCommaSpeedEqual,maxadvspeed);
#if ENABLE_QUADRATIC_ADVANCE
        maxadv=0;
#endif
        maxadv2=0;
        maxadvspeed=0;
        break;
#endif
#if USE_ADVANCE
    case 233: // M233
        if(com->hasY())
            Extruder::current->advanceL = com->Y;
        Com::printF(Com::tLinearLColon,Extruder::current->advanceL);
#if ENABLE_QUADRATIC_ADVANCE
        if(com->hasX())
            Extruder::current->advanceK = com->X;
        Com::printF(Com::tQuadraticKColon,Extruder::current->advanceK);
#endif
        Com::println();
        Printer::updateAdvanceFlags();
        break;
#endif
#if Z_HOME_DIR>0 && MAX_HARDWARE_ENDSTOP_Z
    case 251: // M251
        Printer::zLength -= Printer::currentPosition[Z_AXIS];
        Printer::currentPositionSteps[Z_AXIS] = 0;
        Printer::updateDerivedParameter();
        transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentDeltaPositionSteps);
        Printer::updateCurrentPosition();
        Com::printFLN(Com::tZProbePrinterHeight,Printer::zLength);
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(false);
        Com::printFLN(Com::tEEPROMUpdated);
#endif
        Commands::printCurrentPosition("M251 ");
        break;
#endif
    case 281: // Trigger watchdog
    {
        Com::printInfoFLN("Triggering watchdog. If activated, the printer will reset.");
        Printer::kill(false);
        HAL::delayMilliseconds(200); // write output, make sure heaters are off for safety
        InterruptProtectedBlock noInts;
        while (1);
    }
    break;
    case 302: // M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will disallow.
        Printer::setColdExtrusionAllowed(!com->hasS() || (com->hasS() && com->S != 0));
        break;
    case 303: // M303
    {
#if defined(TEMP_PID) && NUM_TEMPERATURE_LOOPS>0
        int temp = 150;
        int cont = 0;
        int cycles = 5;
        if(com->hasS()) temp = com->S;
        if(com->hasP()) cont = com->P;
        if(com->hasR()) cycles = static_cast<int>(com->R);
        if(cont>=NUM_TEMPERATURE_LOOPS) cont = NUM_TEMPERATURE_LOOPS;
        tempController[cont]->autotunePID(temp,cont,cycles,com->hasX());
#endif
    }
    break;

#if FEATURE_AUTOLEVEL
    case 320: // M320 Activate autolevel
        Printer::setAutolevelActive(true);
        if(com->hasS() && com->S)
        {
            EEPROM::storeDataIntoEEPROM(false);
        }
        break;
    case 321: // M321 Deactivate autoleveling
        Printer::setAutolevelActive(false);
        if(com->hasS() && com->S)
        {
            if(com->S == 3)
                Printer::resetTransformationMatrix(false);
            EEPROM::storeDataIntoEEPROM(false);
        }
        break;
    case 322: // M322 Reset autoeveling matrix
        Printer::resetTransformationMatrix(false);
        if(com->hasS() && com->S)
        {
            EEPROM::storeDataIntoEEPROM(false);
        }
        break;
#endif // FEATURE_AUTOLEVEL
#if DISTORTION_CORRECTION
    case 323: // M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 = permanent = default
        if(com->hasS())
        {
            if(com->S > 0)
                Printer::distortion.enable(com->hasP() && com->P == 1);
            else
                Printer::distortion.disable(com->hasP() && com->P == 1);
        }
        else
        {
            Printer::distortion.reportStatus();
        }
        break;
#endif // DISTORTION_CORRECTION
    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(com->hasS()) for(int i = 0; i <= 4; i++) microstepMode(i, com->S);
        if(com->hasX()) microstepMode(0, (uint8_t)com->X);
        if(com->hasY()) microstepMode(1, (uint8_t)com->Y);
        if(com->hasZ()) microstepMode(2, (uint8_t)com->Z);
        if(com->hasE()) microstepMode(3, (uint8_t)com->E);
        if(com->hasP()) microstepMode(4, com->P); // Original B but is not supported here
        microstepReadings();
#endif
    }
    break;
    case 355: // M355 - non-existant case light configuration
        Com::printInfoFLN("No case lights");
        break;
    case 360: // M360 - show configuration
        Printer::showConfiguration();
        break;
    case 400: // M400 Finish all moves
        Commands::waitUntilEndOfAllMoves();
        break;
    case 401: // M401 Memory position
        Printer::MemoryPosition();
        break;
    case 402: // M402 Go to stored position
        Printer::GoToMemoryPosition(com->hasX(),com->hasY(),com->hasZ(),com->hasE(),(com->hasF() ? com->F : Printer::feedrate));
        break;
    case 450:
        Printer::reportPrinterMode();
        break;
    case 451:
        Printer::mode = PRINTER_MODE_FFF;
        Printer::reportPrinterMode();
        break;
    case 452:
#if defined(SUPPORT_LASER) && SUPPORT_LASER
        Printer::mode = PRINTER_MODE_LASER;
#endif
        Printer::reportPrinterMode();
        break;
    case 453:
#if defined(SUPPORT_CNC) && SUPPORT_CNC
        Printer::mode = PRINTER_MODE_CNC;
#endif
        Printer::reportPrinterMode();
        break;
    case 500: // M500
    {
#if EEPROM_MODE != 0
        EEPROM::storeDataIntoEEPROM(false);
        Com::printInfoFLN(Com::tConfigStoredEEPROM);
#else
        Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
    }
    break;
    case 501: // M501
    {
#if EEPROM_MODE != 0
        EEPROM::readDataFromEEPROM(true);
        Extruder::selectExtruderById(Extruder::current->id);
        Com::printInfoFLN(Com::tConfigLoadedEEPROM);
#else
        Com::printErrorFLN(Com::tNoEEPROMSupport);
#endif
    }
    break;
    case 502: // M502
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        break;
#ifdef DEBUG_QUEUE_MOVE
    case 533: // M533 Write move data
    {
        InterruptProtectedBlock noInts;
        int lc = (int)PrintLine::linesCount;
        int lp = (int)PrintLine::linesPos;
        int wp = (int)PrintLine::linesWritePos;
        int n = (wp-lp);
        if(n < 0) n += PRINTLINE_CACHE_SIZE;
        noInts.unprotect();
        if(n != lc)
            Com::printFLN("Buffer corrupted");
        Com::printF("Buf:",lc);
        Com::printF(",LP:",lp);
        Com::printFLN(",WP:",wp);
        if(PrintLine::cur == NULL)
        {
            Com::printFLN("No move");
            if(PrintLine::linesCount > 0)
            {
                PrintLine &cur = PrintLine::lines[PrintLine::linesPos];
                Com::printF("JFlags:", (int)cur.joinFlags);
                Com::printFLN("Flags:", (int)cur.flags);
                if(cur.isWarmUp())
                {
                    Com::printFLN("warmup:", (int)cur.getWaitForXLinesFilled());
                }
            }
        }
        else
        {
            Com::printF("Rem:", PrintLine::cur->stepsRemaining);
            Com::printFLN("Int:", Printer::interval);
        }
    }
        break;
#endif // DEBUG_QUEUE_MOVE
#ifdef DEBUG_SEGMENT_LENGTH
    case 534: // M534
        Com::printFLN("Max. segment size:", Printer::maxRealSegmentLength);
        if(com->hasS())
            Printer::maxRealSegmentLength = 0;
        break;
#endif
        /*      case 535:  // M535
                    Com::printF("Last commanded position:",Printer::lastCmdPos[X_AXIS]);
                    Com::printF(Com::tComma,Printer::lastCmdPos[Y_AXIS]);
                    Com::printFLN(Com::tComma,Printer::lastCmdPos[Z_AXIS]);
                    Com::printF("Current position:",Printer::currentPosition[X_AXIS]);
                    Com::printF(Com::tComma,Printer::currentPosition[Y_AXIS]);
                    Com::printFLN(Com::tComma,Printer::currentPosition[Z_AXIS]);
                    Com::printF("Position steps:",Printer::currentPositionSteps[X_AXIS]);
                    Com::printF(Com::tComma,Printer::currentPositionSteps[Y_AXIS]);
                    Com::printFLN(Com::tComma,Printer::currentPositionSteps[Z_AXIS]);
        #if NONLINEAR_SYSTEM
              Com::printF("Nonlin. position steps:",Printer::currentDeltaPositionSteps[A_TOWER]);
              Com::printF(Com::tComma,Printer::currentDeltaPositionSteps[B_TOWER]);
              Com::printFLN(Com::tComma,Printer::currentDeltaPositionSteps[C_TOWER]);
        #endif // NONLINEAR_SYSTEM
                    break;*/
        /* case 700: // M700 test new square root function
              if(com->hasS())
                  Com::printFLN(Com::tInfo,(int32_t)HAL::integerSqrt(com->S));
              break;*/
	case 880: //M880 print all settings for auto-updater
		Com::print("UI_PRINTER_COMPANY: ");	Com::println(UI_PRINTER_COMPANY);
		Com::print("UI_PRINTER_NAME: ");	Com::println(UI_PRINTER_NAME);
		Com::print("HARDWARE_VERSION: ");	Com::println(HARDWARE_VERSION);
		Com::print("FIRMWARE_VERSION: ");	Com::println(FIRMWARE_VERSION);
		Com::print("PRINTER_ID: ");			Com::print((int)Printer::PrinterId);	Com::println();
		break;
	case 881://M881 print UI_PRINTER_COMPANY
		Com::print("UI_PRINTER_COMPANY: ");	Com::println(UI_PRINTER_COMPANY);
		break;
	case 882://M882 print UI_PRINTER_NAME
		Com::print("UI_PRINTER_NAME: ");	Com::println(UI_PRINTER_NAME);
		break;
	case 883://M883 print HARDWARE_VERSION
		Com::print("HARDWARE_VERSION: ");	Com::println(HARDWARE_VERSION);
		break;
	case 884://M884 print FIRMWARE_VERSION
		Com::print("FIRMWARE_VERSION: ");	Com::println(FIRMWARE_VERSION);
		break;
	case 885://M885 print PRINTER_ID
		Com::print("PRINTER_ID: ");			Com::print((int)Printer::PrinterId);	Com::println();
		break;
	case 890://M890 factory led test
#if BED_LEDS
		Com::printFLN("BED LEDS: ", Printer::ledCount(false));
		Light.factoryTest();
#endif
		break;
	case 896: //Run custom action by its ID
	    break;
	
	case 897: //Custom bed coating command
	if(com->hasI())
	{
		if(com->I > -20.0 && com->I < 200.0)	 {
			#if EEPROM_MODE != 0
			//If there is something to change
			if (EEPROM::zProbeZOffset() != com->I)
			{
				//Calculate compensation value
				float comp = com->I - EEPROM::zProbeZOffset();
				//Adjust max z length
				Printer::zLength -= comp + Printer::offsetZ;
				//Store the new values in EEPROM
				HAL::eprSetFloat(EPR_Z_LENGTH, Printer::zLength);
				HAL::eprSetFloat(EPR_Z_PROBE_Z_OFFSET, com->I);
				Printer::updateCurrentPosition(true);
				Printer::updateDerivedParameter();
				printCurrentPosition("M114 ");
				EEPROM::storeDataIntoEEPROM(false);
			}
			#endif
			Printer::zBedOffset = com->I;
			//Display message
			Printer::homeAxis(true, true, true);
			Printer::updateCurrentPosition(true);
			Printer::updateDerivedParameter();
			Commands::printCurrentPosition("UI_ACTION_HOMEALL "); 
			}
		else
			Com::printWarningFLN("Not a valid bed coating adjustment!");
	}
	if(com->hasS())
	{
		EEPROM::readDataFromEEPROM(false);
		Com::print("\nCurrent coating[mm]:");
		Com::printFloat(EEPROM::zProbeZOffset(), 2);
		Com::println();
	}
	break;
	case 898: //Status command
		if (com->hasS() && com->S > 0 && com->S < 9999)
		{
			Com::printF("FW:", com->S);
			Com::printF("=");
			switch (com->S) {
			case 1: //Front door
                Com::printFLN("No front door");
				break;
			case 2: //Side doors
                Com::printFLN("No side door");
				break;
			case 3: //Z probe switch
				Endstops::update();
				Endstops::update();
				if ((Printer::probeType == 2) ? !Endstops::zProbe(): Endstops::zProbe())
					Com::printFLN("1#Probe switch activated");
				else
					Com::printFLN("0#Probe switch deactivated");
				break;
			case 4: //Chamber temperature	
				Com::printFloat(Extruder::getChamberTemperature(),2);
				Com::printFLN("#Chamber temp");
				break;
			case 5: //Resends for current session
				Com::printF("",Printer::resends);
				Com::printFLN("#Re-sends ");
				break;
			default:
				Com::printFLN("-1#Error: Not a valid request!");
				break;
			}
		}
		break;
    case 601:
        if(com->hasS() && com->S > 0)
            Extruder::pauseExtruders();
        else
            Extruder::unpauseExtruders();
        break;
    case 602:
        Com::printFLN("No jam control");
        break;
    case 907: // M907 Set digital trimpot/DAC motor current using axis codes.
    {
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
        // If "S" is specified, use that as initial default value, then update each axis w/ specific values as found later.
        if(com->hasS())
        {
            for(int i = 0; i < 10; i++)
            {
                setMotorCurrentPercent(i, com->S);
            }
        }

        if(com->hasX()) setMotorCurrentPercent(0, (float)com->X);
        if(com->hasY()) setMotorCurrentPercent(1, (float)com->Y);
        if(com->hasZ()) setMotorCurrentPercent(2, (float)com->Z);
        if(com->hasE()) setMotorCurrentPercent(3, (float)com->E);
#endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
#if STEPPER_CURRENT_CONTROL != CURRENT_CONTROL_MANUAL
        uint8_t channel,current;
        if(com->hasP() && com->hasS())
            setMotorCurrent((uint8_t)com->P, (unsigned int)com->S);
#endif
    }
    break;
    case 909: // M909 Read digital trimpot settings.
        break;
    case 910: // M910 - Commit digipot/DAC value to external EEPROM
        break;
    case 887: // M887 echo received message
        if(com->hasString())
        {
            Com::printFLN(Com::tEcho, com->text); // for debug purposes only
        }
        break;
    default:
        if(Printer::debugErrors())
        {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
}

/**
  \brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com)
{
    if (!com) return;
    if(com->hasG()) processGCode(com);
    else if(com->hasM()) processMCode(com);
    else if(com->hasT())      // Process T code
    {
        Commands::waitUntilEndOfAllMoves();
        Extruder::selectExtruderById(com->T);
    }
    else
    {
        if(Printer::debugErrors())
        {
            Com::printF(Com::tUnknownCommand);
            com->printCommand();
        }
    }
#ifdef DEBUG_DRYRUN_ERROR
    if(Printer::debugDryrun()) {
        Com::printFLN("Dryrun was enabled");
        com->printCommand();
        Printer::debugReset(8);
}
#endif
    com->popCurrentCommand();
}

void Commands::emergencyStop()
{
    HAL::resetHardware();
}

void Commands::checkFreeMemory(bool print)
{
    static size_t lowest = MAX_RAM;
    size_t current;

    current = HAL::getFreeRam();
    lowest = (lowest < current) ? lowest : current;

    if (print) {
        Com::printF("RAM: ", (uint32_t)lowest);
        Com::printF("/ ", (uint32_t)current);
        Com::printFLN("/ ", (uint32_t)MAX_RAM);
    }
}
// Compare floating point variables
bool cmpf(float a, float b)
{
	return (fabs(a - b) < 0.0001f);
}

//Get height according to the HW version stored in EEPROM
float Commands::retDefHeight()
{
	switch (EEPROM::getHWVer()) {
	case 2020:
		return 230.0;
		break;
	case 2030:
	case 2031:
	case 2032:
		return 250.0;
		break;
	case 3010:
		return 360.0;
		break;
	case 3510:
		return 410.0;
		break;
	case 4010:
		return 465.0;
		break;
	default:
		return 210.0;
	}
}

//Get tower rotation direction depending on HW version
void Commands::fillDefAxisDir()
{
	switch (EEPROM::getHWVer()) {
	case 2020:
	case 3010:
	case 3510:
	case 4010:
		Printer::retDefAxisDir[X_AXIS] = true;
		Printer::retDefAxisDir[Y_AXIS] = false;
		Printer::retDefAxisDir[Z_AXIS] = true;
		break;
	case 2030:
	case 2031:
	case 2032:
		Printer::retDefAxisDir[X_AXIS] = false;
		Printer::retDefAxisDir[Y_AXIS] = false;
		Printer::retDefAxisDir[Z_AXIS] = false;
		break;
	default:
		Printer::retDefAxisDir[X_AXIS] = true;
		Printer::retDefAxisDir[Y_AXIS] = false;
		Printer::retDefAxisDir[Z_AXIS] = true;
        break;
	}
}
