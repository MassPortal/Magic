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

/**

Coordinate system transformations:

Level 1: G-code => Coordinates like send via g-codes.

Level 2: Real coordinates => Coordinates corrected by coordinate shift via G92
         currentPosition and lastCmdPos are from this level.
Level 3: Transformed and shifter => Include extruder offset and bed rotation.
         These variables are only stored temporary.

Level 4: Step position => Level 3 converted into steps for motor position
        currentPositionSteps and destinationPositionSteps are from this level.

Level 5: Nonlinear motor step position, only for nonlinear drive systems
         destinationDeltaSteps


*/

#ifndef PRINTER_H_INCLUDED
#define PRINTER_H_INCLUDED

union floatLong
{
    float f;
    uint32_t l;
    uint64_t L;
};

#define PRINTER_FLAG0_STEPPER_DISABLED      (1<<0)
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT (1<<1)
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT     (1<<2)
#define PRINTER_FLAG0_FORCE_CHECKSUM        (1<<3)
#define PRINTER_FLAG0_MANUAL_MOVE_MODE      (1<<4)
#define PRINTER_FLAG0_AUTOLEVEL_ACTIVE      (1<<5)
#define PRINTER_FLAG0_ZPROBEING             (1<<6)
#define PRINTER_FLAG0_LARGE_MACHINE         (1<<7)
#define PRINTER_FLAG1_HOMED                 (1<<0)
#define PRINTER_FLAG1_AUTOMOUNT             (1<<1)
#define PRINTER_FLAG1_ANIMATION             (1<<2)
#define PRINTER_FLAG1_ALLKILLED             (1<<3)
#define PRINTER_FLAG1_UI_ERROR_MESSAGE      (1<<4)
#define PRINTER_FLAG1_NO_DESTINATION_CHECK  (1<<5)
#define PRINTER_FLAG1_POWER_ON              (1<<6)
#define PRINTER_FLAG1_ALLOW_COLD_EXTRUSION  (1<<7)
#define PRINTER_FLAG2_BLOCK_RECEIVING       (1<<0)
#define PRINTER_FLAG2_AUTORETRACT           (1<<1)
#define PRINTER_FLAG2_RESET_FILAMENT_USAGE  (1<<2)
#define PRINTER_FLAG2_IGNORE_M106_COMMAND   (1<<3) //unused
#define PRINTER_FLAG2_DEBUG_JAM             (1<<4)
#define PRINTER_FLAG2_JAMCONTROL_DISABLED   (1<<5)
#define PRINTER_FLAG2_HOMING                (1<<6)
#define PRINTER_FLAG2_ALL_E_MOTORS          (1<<7) // Set all e motors flag
#define PRINTER_FLAG3_X_DIR                 (1<<0)
#define PRINTER_FLAG3_Y_DIR                 (1<<1)
#define PRINTER_FLAG3_Z_DIR                 (1<<2)

// define an integer number of steps more than large enough to get to endstop from anywhere
#define HOME_DISTANCE_STEPS (Printer::zMaxSteps-Printer::zMinSteps+1000)
#define HOME_DISTANCE_MM (HOME_DISTANCE_STEPS * invAxisStepsPerMM[Z_AXIS])

#if DISTORTION_CORRECTION
class Distortion
{
public:
    Distortion();
    void init();
    void enable(bool permanent = true);
    void disable(bool permanent = true);
    void measure(void);
    int32_t correct(int32_t x, int32_t y, int32_t z) const;
    void updateDerived();
    void reportStatus();
	bool isEnabled() {return enabled;}
	int32_t zMaxSteps() {return zEnd;}	
	void set(float x,float y,float z);
	void showMatrix();		
    void resetCorrection();
private:
    int matrixIndex(fast8_t x, fast8_t y) const;
    int32_t getMatrix(int index) const;
    void setMatrix(int32_t val, int index);
    bool isCorner(fast8_t i, fast8_t j) const;
    INLINE int32_t extrapolatePoint(fast8_t x1, fast8_t y1, fast8_t x2, fast8_t y2) const;
    void extrapolateCorner(fast8_t x, fast8_t y, fast8_t dx, fast8_t dy);
    void extrapolateCorners();
// attributes
    int32_t step;
    int32_t radiusCorrectionSteps;
    int32_t zStart,zEnd;
#if !DISTORTION_PERMANENT
    int32_t matrix[DISTORTION_CORRECTION_POINTS * DISTORTION_CORRECTION_POINTS];
#endif
    bool enabled;
};
#endif //DISTORTION_CORRECTION

class Endstops {
public:
    static void init(void);
    static void report(void);
    static INLINE bool anyXYZMax() {
        return xMaxHit || yMaxHit || zMaxHit;
    }
    static INLINE bool xMax() {
        return xMaxHit;
    }
    static INLINE bool yMax() {
        return yMaxHit;
    }
    static INLINE bool zMax() {
        return zMaxHit;
    }
    static INLINE bool zProbe() {
        return probeHit;
    }
private:
    static volatile bool xMaxHit;
    static volatile bool yMaxHit;
    static volatile bool zMaxHit;
    static volatile bool probeHit;
    static void xMaxRead(void);
    static void yMaxRead(void);
    static void zMaxRead(void);
    static void probeRead(void);
};

#ifndef DEFAULT_PRINTER_MODE
#if NUM_EXTRUDER > 0
#define DEFAULT_PRINTER_MODE PRINTER_MODE_FFF
#elif defined(SUPPORT_LASER) && SUPPORT_LASER
#define DEFAULT_PRINTER_MODE PRINTER_MODE_LASER
#elif defined(SUPPORT_CNC) && SUPPORT_CNC
#define DEFAULT_PRINTER_MODE PRINTER_MODE_CNC
#else
#error No supported printer mode compiled
#endif
#endif

class Printer
{
    static uint8_t debugLevel;
public:
	static long PrinterId;
	static uint8_t probeType; //1-inductive, 2-toggle switch
	static uint16_t bedType; //0 - not set, 1 - no heatbed, >=2 has heatbed
#if USE_ADVANCE
    static volatile int extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
    static ufast8_t maxExtruderSpeed;            ///< Timer delay for end extruder speed
    //static uint8_t extruderAccelerateDelay;     ///< delay between 2 speec increases
    static int advanceStepsSet;
#if ENABLE_QUADRATIC_ADVANCE
    static long advanceExecuted;             ///< Executed advance steps
#endif
#endif
    static float axisStepsPerMM[];
    static float invAxisStepsPerMM[];
    static float maxFeedrate[];
    static float homingFeedrate[];
    static float maxAccelerationMMPerSquareSecond[];
    static float maxTravelAccelerationMMPerSquareSecond[];
    static unsigned long maxPrintAccelerationStepsPerSquareSecond[];
    static unsigned long maxTravelAccelerationStepsPerSquareSecond[];
    static uint8_t relativeCoordinateMode;    ///< Determines absolute (false) or relative Coordinates (true).
    static uint8_t relativeExtruderCoordinateMode;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

    static uint8_t unitIsInches;
    static uint8_t mode;
    static uint8_t fanSpeed; // Last fan speed set with M106/M107

    static float zBedOffset;
    static uint8_t flag0,flag1; // 1 = stepper disabled, 2 = use external extruder interrupt, 4 = temp Sensor defect, 8 = homed
    static uint8_t flag2, flag3;
    static fast8_t stepsPerTimerCall;
    static uint32_t interval;    ///< Last step duration in ticks.
    static uint32_t timer;              ///< used for acceleration/deceleration timing
    static uint32_t stepNumber;         ///< Step number in current move.
    static float coordinateOffset[Z_AXIS_ARRAY];
	static bool retDefAxisDir[Z_AXIS_ARRAY];
    static int32_t currentPositionSteps[E_AXIS_ARRAY];     ///< Position in steps from origin.
    static float currentPosition[Z_AXIS_ARRAY];
	static float lastProbeActHeight;
    static float lastCmdPos[Z_AXIS_ARRAY]; ///< Last coordinates send by gcodes
    static int32_t destinationSteps[E_AXIS_ARRAY];         ///< Target position in steps.
    static float extrudeMultiplyError; ///< Accumulated error during extrusion
    static float extrusionFactor; ///< Extrusion multiply factor
    static int32_t maxDeltaPositionSteps;
    static int32_t currentDeltaPositionSteps[E_TOWER_ARRAY];
    static floatLong deltaDiagonalStepsSquaredA;
    static floatLong deltaDiagonalStepsSquaredB;
    static floatLong deltaDiagonalStepsSquaredC;
    static float deltaMaxRadiusSquared;
    static int32_t deltaFloorSafetyMarginSteps;
    static int32_t deltaAPosXSteps;
    static int32_t deltaAPosYSteps;
    static int32_t deltaBPosXSteps;
    static int32_t deltaBPosYSteps;
    static int32_t deltaCPosXSteps;
    static int32_t deltaCPosYSteps;
    static int32_t realDeltaPositionSteps[TOWER_ARRAY];
    static int16_t travelMovesPerSecond;
    static int16_t printMovesPerSecond;
    static float radius0;
    static int32_t stepsRemainingAtZHit;
    static int32_t stepsRemainingAtXHit;
    static int32_t stepsRemainingAtYHit;
#if SOFTWARE_LEVELING
    static int32_t levelingP1[3];
    static int32_t levelingP2[3];
    static int32_t levelingP3[3];
#endif
#if FEATURE_AUTOLEVEL
    static float autolevelTransformation[9]; ///< Transformation matrix
#endif
	static int16_t resends;
    static float minimumSpeed;               ///< lowest allowed speed to keep integration error small
    static float minimumZSpeed;              ///< lowest allowed speed to keep integration error small
    static int32_t xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    static int32_t yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    static int32_t zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
    static int32_t xMinSteps;                   ///< For software endstops, limit of move in negative direction.
    static int32_t yMinSteps;                   ///< For software endstops, limit of move in negative direction.
    static int32_t zMinSteps;                   ///< For software endstops, limit of move in negative direction.
    static float xLength;
    static float xMin;
    static float yLength;
    static float yMin;
    static float zLength;
    static float zMin;
    static float feedrate;                   ///< Last requested feedrate.
    static int feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
    static unsigned int extrudeMultiply;     ///< Flow multiplier in percdent (factor 1 = 100)
    static float maxJerk;                    ///< Maximum allowed jerk in mm/s
    static float offsetX;                     ///< X-offset for different extruder positions.
    static float offsetY;                     ///< Y-offset for different extruder positions.
    static float offsetZ;                     ///< Y-offset for different extruder positions.
    static speed_t vMaxReached;         ///< Maximumu reached speed
    static uint32_t msecondsPrinting;            ///< Milliseconds of printing time (means time with heated extruder)
    static float filamentPrinted;            ///< mm of filament printed since counting started
    static float memoryX;
    static float memoryY;
    static float memoryZ;
    static float memoryE;
    static float memoryF;
	static bool allowBelow; /// allow probing below Z max length
#ifdef DEBUG_SEGMENT_LENGTH
    static float maxRealSegmentLength;
#endif
#ifdef DEBUG_REAL_JERK
    static float maxRealJerk;
#endif
    static void reportPrinterMode();
	static void setDebugLevel(uint8_t newLevel);
    static INLINE bool debugEcho()
    {
        return (debugLevel & DEB_ECHO);
    }

    static INLINE bool debugInfo()
    {
        return (debugLevel & DEB_INFO);
    }

    static INLINE bool debugErrors()
    {
        return (debugLevel & DEB_ERROR);
    }

    static INLINE bool debugDryrun()
    {
        return (debugLevel & DEB_DRYRUN);
    }

    static INLINE bool debugCommunication()
    {
        return (debugLevel & DEB_COMMUNICATION);
    }

    static INLINE bool debugNoMoves()
    {
        return (debugLevel & DEB_NOMOVES);
    }

    static INLINE bool debugFlag(uint8_t flags)
    {
        return (debugLevel & flags);
    }

    static INLINE void debugSet(uint8_t flags)
    {
        setDebugLevel(debugLevel | flags);
    }

    static INLINE void debugReset(uint8_t flags)
    {
        setDebugLevel(debugLevel & ~flags);
    }
    /** Sets the pwm for the fan speed. Gets called by motion control ot Commands::setFanSpeed. */
    static void setFanSpeedDirectly(uint8_t speed);
    static void setFan2SpeedDirectly(uint8_t speed);
	static void setFan3SpeedDirectly(uint8_t speed);

	static INLINE bool xDirection()
	{
		return !bool(flag3 & PRINTER_FLAG3_X_DIR);
	}

	static INLINE void setXdir(bool b)
	{
		flag3 = (b ? flag3 & ~PRINTER_FLAG3_X_DIR : flag3 | PRINTER_FLAG3_X_DIR);
	}

	static INLINE bool yDirection()
	{
		return !bool(flag3 & PRINTER_FLAG3_Y_DIR);
	}

	static INLINE void setYdir(bool b)
	{
		flag3 = (b ? flag3 & ~PRINTER_FLAG3_Y_DIR : flag3 | PRINTER_FLAG3_Y_DIR);
	}

	static INLINE bool zDirection()
	{
		return !bool(flag3 & PRINTER_FLAG3_Z_DIR);
	}

	static INLINE void setZdir(bool b)
	{
		flag3 = (b ? flag3 & ~PRINTER_FLAG3_Z_DIR : flag3 | PRINTER_FLAG3_Z_DIR);
	}

    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, !X_ENABLE_ON);
#endif
    }

    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, !Y_ENABLE_ON);
#endif
    }
    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON);
#endif
#if FEATURE_THREE_ZSTEPPER && (Z3_ENABLE_PIN > -1)
        WRITE(Z3_ENABLE_PIN, !Z_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for x direction. */
    static INLINE void  enableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN, X_ENABLE_ON);
#endif
    }

    /** \brief Enable stepper motor for y direction. */
    static INLINE void  enableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN, Y_ENABLE_ON);
#endif
    }
    /** \brief Enable stepper motor for z direction. */
    static INLINE void  enableZStepper()
    {
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON);
#endif
#if FEATURE_THREE_ZSTEPPER && (Z3_ENABLE_PIN > -1)
        WRITE(Z3_ENABLE_PIN, Z_ENABLE_ON);
#endif
    }

    static INLINE void setXDirection(bool positive)
    {
        if(positive)
        {
            WRITE(X_DIR_PIN,!xDirection());
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN,!xDirection());
#endif
        }
        else
        {
            WRITE(X_DIR_PIN, xDirection());
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN, xDirection());
#endif
        }
    }

    static INLINE void setYDirection(bool positive)
    {
        if(positive)
        {
            WRITE(Y_DIR_PIN, !yDirection());
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, !yDirection());
#endif
        }
        else
        {
            WRITE(Y_DIR_PIN, yDirection());
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN, yDirection());
#endif
        }
    }
    static INLINE void setZDirection(bool positive)
    {
        if(positive)
        {
            WRITE(Z_DIR_PIN, !zDirection());
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, !zDirection());
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_DIR_PIN, !zDirection());
#endif
        }
        else
        {
            WRITE(Z_DIR_PIN, zDirection());
#if FEATURE_TWO_ZSTEPPER
            WRITE(Z2_DIR_PIN, zDirection());
#endif
#if FEATURE_THREE_ZSTEPPER
            WRITE(Z3_DIR_PIN, zDirection());
#endif
        }
    }

    static INLINE bool getZDirection()
    {
        return ((READ(Z_DIR_PIN) != 0) ^ zDirection());
    }

    static INLINE bool getYDirection()
    {
        return((READ(Y_DIR_PIN) != 0) ^ yDirection());
    }

    static INLINE bool getXDirection()
    {
        return((READ(X_DIR_PIN) != 0) ^ xDirection());
    }

    static INLINE uint8_t isLargeMachine()
    {
        return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
    }

    static INLINE void setLargeMachine(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
    }

    static INLINE uint8_t isAdvanceActivated()
    {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    }

    static INLINE void setAdvanceActivated(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    }

    static INLINE uint8_t isHomed()
    {
        return flag1 & PRINTER_FLAG1_HOMED;
    }

    static INLINE void setHomed(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_HOMED : flag1 & ~PRINTER_FLAG1_HOMED);
    }

    static INLINE uint8_t isAllKilled()
    {
        return flag1 & PRINTER_FLAG1_ALLKILLED;
    }

    static INLINE void setAllKilled(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLKILLED : flag1 & ~PRINTER_FLAG1_ALLKILLED);
    }

    static INLINE uint8_t isNoDestinationCheck()
    {
        return flag1 & PRINTER_FLAG1_NO_DESTINATION_CHECK;
    }

    static INLINE void setNoDestinationCheck(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_NO_DESTINATION_CHECK : flag1 & ~PRINTER_FLAG1_NO_DESTINATION_CHECK);
    }

    static INLINE uint8_t isPowerOn()
    {
        return flag1 & PRINTER_FLAG1_POWER_ON;
    }

    static INLINE void setPowerOn(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_POWER_ON : flag1 & ~PRINTER_FLAG1_POWER_ON);
    }

    static INLINE uint8_t isColdExtrusionAllowed()
    {
        return flag1 & PRINTER_FLAG1_ALLOW_COLD_EXTRUSION;
    }

    static INLINE void setColdExtrusionAllowed(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLOW_COLD_EXTRUSION : flag1 & ~PRINTER_FLAG1_ALLOW_COLD_EXTRUSION);
        if(b)
            Com::printFLN("Cold extrusion allowed");
        else
            Com::printFLN("Cold extrusion disallowed");
    }
    static INLINE uint8_t isAutoretract()
    {
        return flag2 & PRINTER_FLAG2_AUTORETRACT;
    }

    static INLINE void setAutoretract(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_AUTORETRACT : flag2 & ~PRINTER_FLAG2_AUTORETRACT);
        Com::printFLN("Autoretract:",b);
    }
    static INLINE uint8_t isHoming()
    {
        return flag2 & PRINTER_FLAG2_HOMING;
    }

    static INLINE void setHoming(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_HOMING : flag2 & ~PRINTER_FLAG2_HOMING);
    }
    static INLINE uint8_t isAllEMotors()
    {
        return flag2 & PRINTER_FLAG2_ALL_E_MOTORS;
    }

    static INLINE void setAllEMotors(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_ALL_E_MOTORS : flag2 & ~PRINTER_FLAG2_ALL_E_MOTORS);
    }
    static INLINE float convertToMM(float x)
    {
        return (unitIsInches ? x*25.4 : x);
    }
    static INLINE bool areAllSteppersDisabled()
    {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static INLINE void setAllSteppersDiabled()
    {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;
    }
    static INLINE void unsetAllSteppersDisabled()
    {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;
#if FAN_BOARD_PIN>-1
        pwm_pos[pwm_board_fan] = 255;
#endif // FAN_BOARD_PIN
    }
    static INLINE bool isAnyTempsensorDefect()
    {
        return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    }
    static INLINE void setAnyTempsensorDefect()
    {
        flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
    }
    static INLINE void unsetAnyTempsensorDefect()
    {
        flag0 &= ~PRINTER_FLAG0_TEMPSENSOR_DEFECT;
    }
    static INLINE bool isManualMoveMode()
    {
        return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE void setManualMoveMode(bool on)
    {
        flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
    }
    static INLINE bool isAutolevelActive()
    {
        return (flag0 & PRINTER_FLAG0_AUTOLEVEL_ACTIVE)!=0;
    }
    static void setAutolevelActive(bool on);
    static INLINE void setZProbingActive(bool on)
    {
        flag0 = (on ? flag0 | PRINTER_FLAG0_ZPROBEING : flag0 & ~PRINTER_FLAG0_ZPROBEING);
    }
    static INLINE bool isZProbingActive()
    {
        return (flag0 & PRINTER_FLAG0_ZPROBEING);
    }
    static INLINE void startXStep()
    {
        WRITE(X_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
    }
    static INLINE void startYStep()
    {
        WRITE(Y_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
    }
    static INLINE void startZStep()
    {
        WRITE(Z_STEP_PIN,START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN,START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
        WRITE(Z3_STEP_PIN,START_STEP_WITH_HIGH);
#endif
    }
    static INLINE void endXYZSteps()
    {
        WRITE(X_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_TWO_XSTEPPER
        WRITE(X2_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
        WRITE(Y_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_TWO_YSTEPPER
        WRITE(Y2_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
        WRITE(Z_STEP_PIN,!START_STEP_WITH_HIGH);
#if FEATURE_TWO_ZSTEPPER
        WRITE(Z2_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
#if FEATURE_THREE_ZSTEPPER
        WRITE(Z3_STEP_PIN,!START_STEP_WITH_HIGH);
#endif
    }
    static INLINE speed_t updateStepsPerTimerCall(speed_t vbase)
    {
        if(vbase > STEP_DOUBLER_FREQUENCY)
        {
#if ALLOW_QUADSTEPPING
            if(vbase > STEP_DOUBLER_FREQUENCY * 2)
            {
                Printer::stepsPerTimerCall = 4;
                return vbase >> 2;
            }
            else
            {
                Printer::stepsPerTimerCall = 2;
                return vbase >> 1;
            }
#else
            Printer::stepsPerTimerCall = 2;
            return vbase >> 1;
#endif
        }
        else
        {
            Printer::stepsPerTimerCall = 1;
        }
        return vbase;
    }
    static INLINE void disableAllowedStepper()
    {
        if(DISABLE_X) disableXStepper();
        if(DISABLE_Y) disableYStepper();
        if(DISABLE_Z) disableZStepper();
    }
    static INLINE float realXPosition()
    {
        return currentPosition[X_AXIS];
    }

    static INLINE float realYPosition()
    {
        return currentPosition[Y_AXIS];
    }

    static INLINE float realZPosition()
    {
        return currentPosition[Z_AXIS];
    }
    static INLINE void realPosition(float &xp, float &yp, float &zp)
    {
        xp = currentPosition[X_AXIS];
        yp = currentPosition[Y_AXIS];
        zp = currentPosition[Z_AXIS];
    }
    static INLINE void insertStepperHighDelay()
    {
#if STEPPER_HIGH_DELAY>0
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
    }
    static void updateDerivedParameter();
    static void updateCurrentPosition(bool copyLastCmd = false);
    static void kill(uint8_t only_steppers);
    static void updateAdvanceFlags();
    static void setup();
    static void defaultLoopActions();
    static uint8_t setDestinationStepsFromGCode(GCode *com);
    static uint8_t moveTo(float x,float y,float z,float e,float f);
    static uint8_t moveToReal(float x,float y,float z,float e,float f,bool pathOptimize = true);
    static void homeAxis(bool xaxis,bool yaxis,bool zaxis); /// Home axis
    static void setOrigin(float xOff,float yOff,float zOff);
    static bool isPositionAllowed(float x,float y,float z);
    static INLINE int getFanSpeed()
    {
        return (int)pwm_pos[pwm_fan1];
    }
    static INLINE int getFan2Speed()
    {
	    return (int)pwm_pos[pwm_fan2];
    }
	static INLINE int getFan3Speed()
	{
		return (int)pwm_pos[pwm_fan3];
	}
    static INLINE void setDeltaPositions(long xaxis, long yaxis, long zaxis)
    {
        currentDeltaPositionSteps[A_TOWER] = xaxis;
        currentDeltaPositionSteps[B_TOWER] = yaxis;
        currentDeltaPositionSteps[C_TOWER] = zaxis;
    }
    static void deltaMoveToTopEndstops(float feedrate);
    static float runZMaxProbe();
	static void startProbing(void);
	static void finishProbing();
    static float runZProbe(bool first,bool last,uint8_t repeat = Z_PROBE_REPETITIONS);
    static void waitForZProbeStart();
    static float bendingCorrectionAt(float x,float y);
    // Moved outside FEATURE_Z_PROBE to allow auto-level functional test on
    // system without Z-probe
#if FEATURE_AUTOLEVEL
    static void transformToPrinter(float x,float y,float z,float &transX,float &transY,float &transZ);
    static void transformFromPrinter(float x,float y,float z,float &transX,float &transY,float &transZ);
    static void resetTransformationMatrix(bool silent);
    static void buildTransformationMatrix(float h1,float h2,float h3);
#endif
#if DISTORTION_CORRECTION
    static void measureDistortion(void);
    static Distortion distortion;
#endif
    static void MemoryPosition();
    static void GoToMemoryPosition(bool x,bool y,bool z,bool e,float feed);
    static void babyStep(float Zmm);
    static void showConfiguration();
	static uint8_t ledVal;
	static INLINE uint8_t ledCount(bool extLed) {
		switch (Printer::ledVal) {
		case 0:
		case 1: return 0;
			break;
		case 2: //MP20 old
			if (extLed)
				return 5;
			else
				return 12;
			break;
		case 3:
			if (extLed)
				return 0;
			else
				return 1;
			break;
		case 102: //MP20 new
			if (extLed)
				return 0;
			else
				return 12;
			break;
		case 103: //MP30
			if (extLed)
				return 0;
			else
				return 15;
			break;
		case 104: //MP40
			if (extLed)
				return 0;
			else
				return 19;
			break;
		case 105: //MP40
			if (extLed)
				return 0;
			else
				return 16;
			break;
		default:
			return 0;
		}
	}
private:
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
};

#endif // PRINTER_H_INCLUDED
