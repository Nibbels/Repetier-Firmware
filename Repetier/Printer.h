/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

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
*/


#ifndef PRINTER_H
#define PRINTER_H


// ##########################################################################################
// ##   status flags
// ##########################################################################################

#define PRINTER_FLAG0_STEPPER_DISABLED          1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT     2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT         4
#define PRINTER_FLAG0_FORCE_CHECKSUM            8
#define PRINTER_FLAG0_MANUAL_MOVE_MODE          16
#define PRINTER_FLAG0_LARGE_MACHINE             128

//#define PRINTER_FLAG1_HOMED                     1 -> egal geworden.
#define PRINTER_FLAG1_AUTOMOUNT                 2
#define PRINTER_FLAG1_ANIMATION                 4
#define PRINTER_FLAG1_ALLKILLED                 8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE          16
#define PRINTER_FLAG1_NO_DESTINATION_CHECK      32
#define PRINTER_FLAG1_Z_ORIGIN_SET              64

#define PRINTER_FLAG2_RESET_FILAMENT_USAGE      4
#define PRINTER_FLAG2_HOMING                    64
 
#define PRINTER_FLAG3_X_HOMED                   1 // flag3 alike original repetier
#define PRINTER_FLAG3_Y_HOMED                   2 // flag3 alike original repetier
#define PRINTER_FLAG3_Z_HOMED                   4 // flag3 alike original repetier
#define PRINTER_FLAG3_PRINTING                  8 // flag3 alike original repetier


class Printer
{
public:
#if USE_ADVANCE
    static volatile int     extruderStepsNeeded;                // This many extruder steps are still needed, <0 = reverse steps needed.
    static uint8_t          maxExtruderSpeed;                   // Timer delay for end extruder speed
    static volatile int     advanceStepsSet;

#ifdef ENABLE_QUADRATIC_ADVANCE
    static long             advanceExecuted;                    // Executed advance steps
#endif // ENABLE_QUADRATIC_ADVANCE
#endif // USE_ADVANCE

    static uint8_t          menuMode;
    static float            axisStepsPerMM[];
    static float            invAxisStepsPerMM[];
    static float            maxFeedrate[];
    static float            homingFeedrate[];
    static float            maxAccelerationMMPerSquareSecond[];
    static float            maxTravelAccelerationMMPerSquareSecond[];
#if FEATURE_MILLING_MODE
    static short            max_milling_all_axis_acceleration;
#endif // FEATURE_MILLING_MODE
    static unsigned long    maxPrintAccelerationStepsPerSquareSecond[];
    static unsigned long    maxTravelAccelerationStepsPerSquareSecond[];
    static uint8_t          relativeCoordinateMode;             // Determines absolute (false) or relative Coordinates (true).
    static uint8_t          relativeExtruderCoordinateMode;     // Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
    static uint8_t          unitIsInches;
    static uint8_t          debugLevel;
    static uint8_t          flag0;
    static uint8_t          flag1;
    static uint8_t          flag2;
    static uint8_t          flag3;
    static uint8_t          stepsPerTimerCall;
    static uint16_t         stepsDoublerFrequency;
    static volatile unsigned long interval;                     // Last step duration in ticks.
    static volatile float   v;                                  // Last planned printer speed.
    static unsigned long    timer;                              // used for acceleration/deceleration timing
    static unsigned long    stepNumber;                         // Step number in current move.
#if FEATURE_DIGIT_FLOW_COMPENSATION
    static unsigned short   interval_mod;                       // additional step duration in ticks to slow the printer down live
#endif // FEATURE_DIGIT_FLOW_COMPENSATION

    static float            originOffsetMM[3];
    static volatile long    queuePositionTargetSteps[4];        // Target position in steps.
    static volatile long    queuePositionLastSteps[4];          // Position in steps from origin.
    static volatile float   queuePositionLastMM[3];             // Position in mm from origin.
    static volatile float   queuePositionCommandMM[3];          // Last coordinates send by gcodes

    static long             maxSteps[3];                        // For software endstops, limit of move in positive direction.
    static long             minSteps[3];                        // For software endstops, limit of move in negative direction.
    static float            lengthMM[3];
    static float            minMM[2];
    static float            feedrate;                           // Last requested feedrate.
    static int              feedrateMultiply;                   // Multiplier for feedrate in percent (factor 1 = 100)
    static int              extrudeMultiply;                    // Flow multiplier in percdent (factor 1 = 100)
    static float            extrudeMultiplyError;               //< Accumulated error during extrusion
    static float            extrusionFactor;                    //< Extrusion multiply factor
    static float            maxJerk;                            // Maximum allowed jerk in mm/s
    static float            maxZJerk;                           // Maximum allowed jerk in z direction in mm/s
    static float            extruderOffset[3];                  // offset for different extruder positions.
    static speed_t          vMaxReached;                        // Maximumu reached speed
    static unsigned long    msecondsPrinting;                   // Milliseconds of printing time (means time with heated extruder)
    static unsigned long    msecondsMilling;                    // Milliseconds of milling time
    static float            filamentPrinted;                    // mm of filament printed since counting started
    static long             ZOffset;                            // Z Offset in um
    static char             ZMode;                              // Z Scale
    static char             moveMode[3];                        // move mode which is applied within the Position X/Y/Z menus

#if ENABLE_BACKLASH_COMPENSATION
    static float            backlash[3];
    static uint8_t          backlashDir;
#endif // ENABLE_BACKLASH_COMPENSATION

#if FEATURE_MEMORY_POSITION
    static float            memoryX;
    static float            memoryY;
    static float            memoryZ;
    static float            memoryE;
#endif // FEATURE_MEMORY_POSITION

#if FEATURE_HEAT_BED_Z_COMPENSATION
    static volatile char    doHeatBedZCompensation;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
    static volatile char    doWorkPartZCompensation;
    static volatile long    staticCompensationZ;                // this is the z-delta which can occur in case the x/y position of the z-origin from the work part scan is different to the x/y position of the z-origin from the moment of the start of the milling
#endif // FEATURE_WORK_PART_Z_COMPENSATION

    static volatile long    queuePositionCurrentSteps[3];
    static volatile char    stepperDirection[3];              // this is the current x/y/z-direction from the processing of G-Codes
    static volatile char    blockAll;

    static volatile long    currentZSteps;

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    static volatile long    compensatedPositionTargetStepsZ;
    static volatile long    compensatedPositionCurrentStepsZ;
    static volatile float   compensatedPositionOverPercE;
    static volatile float   compensatedPositionCollectTinyE;
    
    static volatile long    queuePositionZLayerCurrent_cand;
    static volatile long    queuePositionZLayerCurrent;
    static volatile long    queuePositionZLayerLast;

    static volatile char    endZCompensationStep;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    static volatile long    directPositionTargetSteps[4];
    static volatile long    directPositionCurrentSteps[4];
    static long             directPositionLastSteps[4];
    static char             waitMove;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_MILLING_MODE
    static char             operatingMode;
    static float            drillFeedrate;
    static float            drillZDepth;
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
    static char             ZEndstopType;
    static char             ZEndstopUnknown;
    static char             lastZDirection;
    static char             endstopZMinHit;
    static char             endstopZMaxHit;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#if FEATURE_CONFIGURABLE_MILLER_TYPE
    static char             MillerType;
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#if STEPPER_ON_DELAY
    static char             enabledStepper[3];
#endif // STEPPER_ON_DELAY

#if FEATURE_BEEPER
    static char             enableBeeper;
#endif // FEATURE_BEEPER

#if FEATURE_CASE_LIGHT
    static char             enableCaseLight;
#endif // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
    static char             RGBLightMode;
    static char             RGBLightStatus;  
    static  unsigned long   RGBLightIdleStart;
    static char             RGBButtonBackPressed;
    static char             RGBLightModeForceWhite;
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_230V_OUTPUT
    static char             enable230VOutput;
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
    static char             enableFET1;
    static char             enableFET2;
    static char             enableFET3;
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_CASE_FAN
    static bool ignoreFanOn;
    static unsigned long    prepareFanOff;
    static unsigned long    fanOffDelay;
#endif // FEATURE_CASE_FAN

#if FEATURE_TYPE_EEPROM
    static unsigned char    wrongType;
#endif // FEATURE_TYPE_EEPROM

#if FEATURE_UNLOCK_MOVEMENT
    static unsigned char    g_unlock_movement;
#endif //FEATURE_UNLOCK_MOVEMENT

    static uint8_t          motorCurrent[5];

#if FEATURE_ZERO_DIGITS
    static bool             g_pressure_offset_active;
    static short            g_pressure_offset;
#endif // FEATURE_ZERO_DIGITS

    static INLINE void setMenuMode(uint8_t mode,bool on)
    {
        if(on)
            menuMode |= mode;
        else
            menuMode &= ~mode;
    } // setMenuMode

    static INLINE bool isMenuMode(uint8_t mode)
    {
        return (menuMode & mode)==mode;
    }// isMenuMode

    static INLINE bool debugEcho()
    {
        return ((debugLevel & 1)!=0);
    } // debugEcho

    static INLINE bool debugInfo()
    {
        return ((debugLevel & 2)!=0);
    } // debugInfo

    static INLINE bool debugErrors()
    {
        return ((debugLevel & 4)!=0);
    } // debugErrors

    static INLINE bool debugDryrun()
    {
        return ((debugLevel & 8)!=0);
    } // debugDryrun

    static INLINE bool debugCommunication()
    {
        return ((debugLevel & 16)!=0);
    } // debugCommunication
    
#ifdef INCLUDE_DEBUG_NO_MOVE
    static INLINE bool debugNoMoves()
    {
        return ((debugLevel & 32)!=0);
    }// debugNoMoves
#endif // INCLUDE_DEBUG_NO_MOVE

    /** \brief Disable stepper motor for x direction. */
    static INLINE void disableXStepper()
    {
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN,!X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN,!X_ENABLE_ON);
#endif // FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        Printer::enabledStepper[X_AXIS] = 0;
#endif // STEPPER_ON_DELAY

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed( /*false ,*/ false , -1 , -1 );
        cleanupXPositions();

    } // disableXStepper

    /** \brief Disable stepper motor for y direction. */
    static INLINE void disableYStepper()
    {
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN,!Y_ENABLE_ON);
#endif // FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        Printer::enabledStepper[Y_AXIS] = 0;
#endif // STEPPER_ON_DELAY

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed( /*false ,*/ -1, false , -1 );
        cleanupYPositions();

    } // disableYStepper

    /** \brief Disable stepper motor for z direction. */
    static INLINE void disableZStepper()
    {
        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed( /*false ,*/ -1 , -1 , false ); // disable CMP mit wait ist bei unhome Z mit drin. //Printer::disableCMPnow(true); //fahre vom heizbett auf 0 bevor stepper aus.

#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON);
#endif // FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        Printer::enabledStepper[Z_AXIS] = 0;
#endif // STEPPER_ON_DELAY

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
        Printer::lastZDirection = 0;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

        setZOriginSet(false);
        cleanupZPositions();

    } // disableZStepper

    /** \brief Enable stepper motor for x direction. */
    static INLINE void enableXStepper()
    {
        unmarkAllSteppersDisabled();
#if (X_ENABLE_PIN > -1)
        WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif // (X_ENABLE_PIN > -1)

#if FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)
        WRITE(X2_ENABLE_PIN,X_ENABLE_ON);
#endif // FEATURE_TWO_XSTEPPER && (X2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        if( !Printer::enabledStepper[X_AXIS] )
        {
            Printer::enabledStepper[X_AXIS] = 1;
            HAL::delayMilliseconds( STEPPER_ON_DELAY );
        }
#endif // STEPPER_ON_DELAY
    } // enableXStepper

    /** \brief Enable stepper motor for y direction. */
    static INLINE void enableYStepper()
    {
        unmarkAllSteppersDisabled();
#if (Y_ENABLE_PIN > -1)
        WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif // (Y_ENABLE_PIN > -1)

#if FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)
        WRITE(Y2_ENABLE_PIN,Y_ENABLE_ON);
#endif // FEATURE_TWO_YSTEPPER && (Y2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        if( !Printer::enabledStepper[Y_AXIS] )
        {
            Printer::enabledStepper[Y_AXIS] = 1;
            HAL::delayMilliseconds( STEPPER_ON_DELAY );
        }
#endif // STEPPER_ON_DELAY
    } // enableYStepper

    /** \brief Enable stepper motor for z direction. */
    static INLINE void enableZStepper()
    {
        unmarkAllSteppersDisabled();
#if (Z_ENABLE_PIN > -1)
        WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif // (Z_ENABLE_PIN > -1)

#if FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)
        WRITE(Z2_ENABLE_PIN,Z_ENABLE_ON);
#endif // FEATURE_TWO_ZSTEPPER && (Z2_ENABLE_PIN > -1)

#if STEPPER_ON_DELAY
        if( !Printer::enabledStepper[Z_AXIS] )
        {
            Printer::enabledStepper[Z_AXIS] = 1;
            HAL::delayMilliseconds( STEPPER_ON_DELAY );
        }
#endif // STEPPER_ON_DELAY
    } // enableZStepper

    static INLINE void setXDirection(bool positive)
    {
        if(positive)
        {
            // extruder moves to the right
            WRITE(X_DIR_PIN,!INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN,!INVERT_X_DIR);
#endif // FEATURE_TWO_XSTEPPER
            stepperDirection[X_AXIS] = 1;
        }
        else
        {
            // extruder moves to the left
            WRITE(X_DIR_PIN,INVERT_X_DIR);
#if FEATURE_TWO_XSTEPPER
            WRITE(X2_DIR_PIN,INVERT_X_DIR);
#endif // FEATURE_TWO_XSTEPPER
            stepperDirection[X_AXIS] = -1;
        }
    } // setXDirection

    static INLINE void setYDirection(bool positive)
    {
        if(positive)
        {
            // heat bed moves to the front
            WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN,!INVERT_Y_DIR);
#endif // FEATURE_TWO_YSTEPPER
            stepperDirection[Y_AXIS] = 1;
        }
        else
        {
            // heat bed moves to the back
            WRITE(Y_DIR_PIN,INVERT_Y_DIR);
#if FEATURE_TWO_YSTEPPER
            WRITE(Y2_DIR_PIN,INVERT_Y_DIR);
#endif // FEATURE_TWO_YSTEPPER
            stepperDirection[Y_AXIS] = -1;
        }
    } // setYDirection

    static INLINE void setZDirection(bool positive)
    {
        if(positive)
        {
            // heat bed moves to the bottom
            WRITE( Z_DIR_PIN, !INVERT_Z_DIR );
#if FEATURE_TWO_ZSTEPPER
            WRITE( Z2_DIR_PIN, !INVERT_Z_DIR );
#endif // FEATURE_TWO_YSTEPPER
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
            lastZDirection = 1;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
            stepperDirection[Z_AXIS] = 1;
        }
        else
        {
            // heat bed moves to the top
            WRITE( Z_DIR_PIN, INVERT_Z_DIR );
#if FEATURE_TWO_ZSTEPPER
            WRITE( Z2_DIR_PIN, INVERT_Z_DIR );
#endif // FEATURE_TWO_YSTEPPER
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
            lastZDirection = -1;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
            stepperDirection[Z_AXIS] = -1;
        }
    } // setZDirection

    static INLINE void startXStep()
    {
        WRITE( X_STEP_PIN, HIGH );
    #if FEATURE_TWO_XSTEPPER
        WRITE( X2_STEP_PIN, HIGH );
    #endif // FEATURE_TWO_XSTEPPER
    } // startXStep

    static INLINE void startYStep()
    {
        WRITE( Y_STEP_PIN, HIGH );
    #if FEATURE_TWO_YSTEPPER
        WRITE( Y2_STEP_PIN, HIGH );
    #endif // FEATURE_TWO_YSTEPPER
    } // startYStep
        
    static INLINE void startZStep()
    {
        WRITE( Z_STEP_PIN, HIGH );
    #if FEATURE_TWO_ZSTEPPER
        WRITE( Z2_STEP_PIN, HIGH );
    #endif // FEATURE_TWO_ZSTEPPER
        Printer::currentZSteps += (Printer::getZDirectionIsPos() ? 1 : -1);
    } // startZStep

    static INLINE void endZStep( void )
    {
        WRITE( Z_STEP_PIN, LOW );
    #if FEATURE_TWO_ZSTEPPER
        WRITE( Z2_STEP_PIN, LOW );
    #endif // FEATURE_TWO_ZSTEPPER
    } // endZStep

    static INLINE void endYStep( void )
    {
        WRITE( Y_STEP_PIN, LOW );
    } // endZStep

    static INLINE void endXStep( void )
    {
        WRITE( X_STEP_PIN, LOW );
    } // endZStep

    static INLINE void endXYZSteps()
    {
        endXStep();
        endYStep();
        endZStep();
    } // endXYZSteps
    
    static INLINE bool getZDirectionIsPos()
    {
        return ((READ(Z_DIR_PIN)!=0) ^ INVERT_Z_DIR);
    } // getZDirectionIsPos

    static INLINE bool getYDirectionIsPos()
    {
        return((READ(Y_DIR_PIN)!=0) ^ INVERT_Y_DIR);
    } // getYDirection

    static INLINE bool getXDirectionIsPos()
    {
        return((READ(X_DIR_PIN)!=0) ^ INVERT_X_DIR);
    } // getXDirection

    static INLINE uint8_t isLargeMachine()
    {
        return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
    } // isLargeMachine

    static INLINE void setLargeMachine(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
    } // setLargeMachine

    static INLINE uint8_t isAdvanceActivated()
    {
        return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
    } // isAdvanceActivated

    static INLINE void setAdvanceActivated(uint8_t b)
    {
        flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
    } // setAdvanceActivated

    static INLINE uint8_t isAxisHomed(int8_t b) //X_AXIS 0, Y_AXIS 1, Z_AXIS 2
    {
        switch(b){
            case X_AXIS: {
                return (flag3 & PRINTER_FLAG3_X_HOMED);
                }
            case Y_AXIS: {
                return (flag3 & PRINTER_FLAG3_Y_HOMED);
                }
            case Z_AXIS: {
                return (flag3 & PRINTER_FLAG3_Z_HOMED);
                }
        } 
        return 0;
    } // isAxisHomed

    static inline uint8_t isZHomeSafe() //experimentelle funktion, die nicht viel abdeckt, das ist ein test. ... TODO: merge with function isHomingAllowed
    {
        bool problematisch = false;
        if( Extruder::current->zOffset ) problematisch = true; //wenn rechtes gefedertes Hotend tiefer, dann evtl. kollision
        if( g_offsetZCompensationSteps > 0 ) problematisch = true; //wenn matrix positiv, dann evtl. problem
        if( isAxisHomed(Y_AXIS) && Printer::queuePositionCurrentSteps[Y_AXIS] + Printer::directPositionCurrentSteps[Y_AXIS] <= 5*YAXIS_STEPS_PER_MM ) problematisch = false; //vorherige Probleme egal, wenn bett nach hinten gefahren
#if FEATURE_ALIGN_EXTRUDERS
        if( g_nAlignExtrudersStatus ) problematisch = false; //das homing passiert in Z einzeln, liegt aber neben dem Bett.
#endif // FEATURE_ALIGN_EXTRUDERS
#if FEATURE_HEAT_BED_Z_COMPENSATION
        if( g_nHeatBedScanStatus ) problematisch = false; //das homing passiert in Z einzeln, liegt aber neben dem Bett.
#endif //FEATURE_HEAT_BED_Z_COMPENSATION
        if(problematisch) return 0; //während Z-Scan gibts einen homeZ, der ist aber nicht relevant, den case gibts nicht!
        else return 1;
    } // isZHomeSafe

    static int8_t anyHomeDir(uint8_t axis);
#if FEATURE_CHECK_HOME
    static bool anyEndstop( uint8_t axis );
    static void changeAxisDirection( uint8_t axis, int8_t direction );
    static void startAxisStep( uint8_t axis );
    static void endAxisStep(uint8_t axis);
    static void stepAxisStep(uint8_t axis, uint8_t slower = 1);
    static int8_t checkHome(int8_t axis);
#endif //FEATURE_CHECK_HOME

    static INLINE bool areAxisHomed() //X_AXIS && Y_AXIS && Z_AXIS
    {
        return (bool)(Printer::isAxisHomed(Z_AXIS) && Printer::isAxisHomed(Y_AXIS) && Printer::isAxisHomed(X_AXIS));
    } // areAxisHomed

    static inline void setHomed(/*uint8_t b,*/ int8_t x = -1, int8_t y = -1, int8_t z = -1)
    {
        //flag1 = (b ? flag1 | PRINTER_FLAG1_HOMED : flag1 & ~PRINTER_FLAG1_HOMED);
        if(x != -1) flag3 = (x ? flag3 | PRINTER_FLAG3_X_HOMED : flag3 & ~PRINTER_FLAG3_X_HOMED);
        if(y != -1) flag3 = (y ? flag3 | PRINTER_FLAG3_Y_HOMED : flag3 & ~PRINTER_FLAG3_Y_HOMED);
        if(z != -1) flag3 = (z ? flag3 | PRINTER_FLAG3_Z_HOMED : flag3 & ~PRINTER_FLAG3_Z_HOMED);  
        //if((flag3 & PRINTER_FLAG3_X_HOMED) && (flag3 & PRINTER_FLAG3_Y_HOMED) && (flag3 & PRINTER_FLAG3_Z_HOMED)) flag1 |= PRINTER_FLAG1_HOMED;
        //if(!(flag3 & PRINTER_FLAG3_X_HOMED) && !(flag3 & PRINTER_FLAG3_Y_HOMED) && !(flag3 & PRINTER_FLAG3_Z_HOMED)) flag1 &= ~PRINTER_FLAG1_HOMED;
        
#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        if( !isAxisHomed(Z_AXIS) ){
            Printer::disableCMPnow(true); //true == wait for move while HOMING
        }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    } // setHomed

    static INLINE uint8_t isZOriginSet()
    {
        return flag1 & PRINTER_FLAG1_Z_ORIGIN_SET;
    } // isZOriginSet

    static INLINE void setZOriginSet(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_Z_ORIGIN_SET : flag1 & ~PRINTER_FLAG1_Z_ORIGIN_SET);
    } // setZOriginSet

    static INLINE uint8_t isAllKilled()
    {
        return flag1 & PRINTER_FLAG1_ALLKILLED;
    } // isAllKilled

    static INLINE void setAllKilled(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ALLKILLED : flag1 & ~PRINTER_FLAG1_ALLKILLED);
    } // setAllKilled

    static INLINE uint8_t isAutomount()
    {
        return flag1 & PRINTER_FLAG1_AUTOMOUNT;
    } // isAutomount

    static INLINE void setAutomount(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_AUTOMOUNT : flag1 & ~PRINTER_FLAG1_AUTOMOUNT);
    } // setAutomount

    static INLINE uint8_t isAnimation()
    {
        return flag1 & PRINTER_FLAG1_ANIMATION;
    } // isAnimation

    static INLINE void setAnimation(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_ANIMATION : flag1 & ~PRINTER_FLAG1_ANIMATION);
    } // setAnimation

    static INLINE uint8_t isUIErrorMessage()
    {
        return flag1 & PRINTER_FLAG1_UI_ERROR_MESSAGE;
    } // isUIErrorMessage

    static INLINE void setUIErrorMessage(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_UI_ERROR_MESSAGE : flag1 & ~PRINTER_FLAG1_UI_ERROR_MESSAGE);
    } // setUIErrorMessage

    static INLINE uint8_t isNoDestinationCheck()
    {
        return flag1 & PRINTER_FLAG1_NO_DESTINATION_CHECK;
    } // isNoDestinationCheck

    static INLINE void setNoDestinationCheck(uint8_t b)
    {
        flag1 = (b ? flag1 | PRINTER_FLAG1_NO_DESTINATION_CHECK : flag1 & ~PRINTER_FLAG1_NO_DESTINATION_CHECK);
    } // setNoDestinationCheck

    static INLINE uint8_t isPrinting()
    {
        return flag3 & PRINTER_FLAG3_PRINTING;
    }

    static INLINE void setPrinting(bool b)
    {
        flag3 = (b ? flag3 | PRINTER_FLAG3_PRINTING : flag3 & ~PRINTER_FLAG3_PRINTING);
        if(!b){
            Printer::setMenuMode(MENU_MODE_SD_PRINTING, b);
        }
        Printer::setMenuMode(MENU_MODE_PRINTING, b);
    }

    static INLINE void toggleAnimation()
    {
        setAnimation(!isAnimation());
    } // toggleAnimation

    static INLINE float convertToMM(float x)
    {
        return (unitIsInches ? x*25.4 : x);
    } // convertToMM

    static INLINE bool isXMinEndstopHit()
    {
#if X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
        return READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING;
#else
        return false;
#endif // X_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_X
    } // isXMinEndstopHit

    static INLINE bool isYMinEndstopHit()
    {
#if Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
        return READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING;
#else
        return false;
#endif // Y_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Y
    } // isYMinEndstopHit

    static INLINE bool isZMinEndstopHit()
    {
#if Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS

        if( ZEndstopType == ENDSTOP_TYPE_SINGLE )
        {
#if FEATURE_MILLING_MODE
            if( operatingMode == OPERATING_MODE_PRINT )
            {
                // in case there is only one z-endstop and we are in operating mode "print", the z-min endstop must be connected
                return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
            }

            // in case there is only one z-endstop and we are in operating mode "mill", the z-min endstop is not connected and can not be detected
            return false;
#else // FEATURE_MILLING_MODE
            // in case there is only one z-endstop and we are in operating mode "print", the z-min endstop must be connected
            return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
#endif // FEATURE_MILLING_MODE
        }

        // we end up here in case the z-min and z-max endstops are connected in a circuit
        if( READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING )
        {
            // either the min or the max endstop is hit
            if( ZEndstopUnknown )
            {
                // this is save as long as we do not allow any movement except the z-homing while ZEndstopUnknown is != 0
                return false;
            }

            if( endstopZMaxHit == ENDSTOP_IS_HIT )
            {
                // when the z-max endstop is hit already we know that the z-min endstop is not hit
                return false;
            }

            if( endstopZMinHit == ENDSTOP_IS_HIT )
            {
                // we remember that the z-min endstop is hit at the moment
                return true;
            }

            if( lastZDirection > 0 )
            {
                // z-min was not hit and we are moving downwards, so z-min can not become hit right now
                return false;
            }

            // the last z-direction is unknown or the heat bed has been moved upwards, thus we have to assume that the z-min endstop is hit
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS && DEBUG_CONFIGURABLE_Z_ENDSTOPS
            Com::printF( PSTR( "Z-Min hit") );
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS && DEBUG_CONFIGURABLE_Z_ENDSTOPS

            endstopZMinHit = ENDSTOP_IS_HIT;
            endstopZMaxHit = ENDSTOP_NOT_HIT;
            return true;
        }

        // no z endstop is hit
        if( endstopZMinHit == ENDSTOP_IS_HIT )
        {
            endstopZMinHit = ENDSTOP_WAS_HIT;
        }
        if( endstopZMaxHit == ENDSTOP_IS_HIT )
        {
            endstopZMaxHit = ENDSTOP_WAS_HIT;
        }
        ZEndstopUnknown = 0;
        return false;

#else // FEATURE_CONFIGURABLE_Z_ENDSTOPS
        return READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING;
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else //Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
        return false;
#endif // Z_MIN_PIN>-1 && MIN_HARDWARE_ENDSTOP_Z
    } // isZMinEndstopHit

    static INLINE bool isXMaxEndstopHit()
    {
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
        return READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING;
#else
        return false;
#endif // X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
    } // isXMaxEndstopHit

    static INLINE bool isYMaxEndstopHit()
    {
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
        return READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING;
#else
        return false;
#endif // Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
    } // isYMaxEndstopHit

    static inline bool isZMaxEndstopHit()
    {
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z

 #if FEATURE_CONFIGURABLE_Z_ENDSTOPS

        if( ZEndstopType == ENDSTOP_TYPE_SINGLE )
        {
  #if FEATURE_MILLING_MODE
            if( operatingMode == OPERATING_MODE_MILL )
            {
                // in case there is only one z-endstop and we are in operating mode "mill", the z-max endstop must be connected
                return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
            }
  #endif // FEATURE_MILLING_MODE
            // in case there is only one z-endstop and we are in operating mode "print", the z-max endstop is not connected and can not be detected
            return false;
        }

        // we end up here in case the z-min and z-max endstops are connected in a circuit
        if( READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING )
        {
            // either the min or the max endstop is hit
            if( ZEndstopUnknown )
            {
                // this is save as long as we do not allow any movement except the z-homing while ZEndstopUnknown is != 0
                return false;
            }

            if( endstopZMinHit == ENDSTOP_IS_HIT )
            {
                // when the z-min endstop is hit already we know that the z-max endstop is not hit
                return false;
            }

            if( endstopZMaxHit == ENDSTOP_IS_HIT )
            {
                // we remember that the z-max endstop is hit at the moment
                return true;
            }
                
            if( lastZDirection < 0 )
            {
                // z-max was not hit and we are moving upwards, so z-max can not become hit right now
                return false;
            }

            if( Printer::isAxisHomed(Z_AXIS) )
            {
                if( currentZSteps < Z_MIN_DISTANCE )
                {
                    // we are close to z-min, so z-max can not become hit right now
  #if DEBUG_CONFIGURABLE_Z_ENDSTOPS
                    Com::printF( PSTR( "Z-Max not hit") );
  #endif // DEBUG_CONFIGURABLE_Z_ENDSTOPS
                    return false;
                }
            }

            // the last z-direction is unknown or the heat bed has been moved downwards, thus we have to assume that the z-max endstop is hit
  #if FEATURE_CONFIGURABLE_Z_ENDSTOPS && DEBUG_CONFIGURABLE_Z_ENDSTOPS
            Com::printF( PSTR( "Z-Max hit") );
  #endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS && DEBUG_CONFIGURABLE_Z_ENDSTOPS

            endstopZMinHit = ENDSTOP_NOT_HIT;
            endstopZMaxHit = ENDSTOP_IS_HIT;
            return true;
        }

        // no z endstop is hit
        if( endstopZMinHit == ENDSTOP_IS_HIT )
        {
            endstopZMinHit = ENDSTOP_WAS_HIT;
        }
        if( endstopZMaxHit == ENDSTOP_IS_HIT )
        {
            endstopZMaxHit = ENDSTOP_WAS_HIT;
        }
        ZEndstopUnknown = 0;
        return false;

 #else
        return READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING;
 #endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else
        return false;
#endif // Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
    } // isZMaxEndstopHit

    static INLINE bool areAllSteppersDisabled()
    {
        return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
    } // areAllSteppersDisabled

    static INLINE void markAllSteppersDisabled()
    {
        flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;

        // when the stepper is disabled we loose our home position because somebody else can move our mechanical parts
        setHomed( /*false ,*/ false, false, false ); //mag sein, dass wir das nicht brauchen, weil sowieso die einzelnen stepper deaktiviert werden müssen.
        setZOriginSet(false);
    } // markAllSteppersDisabled

    static INLINE void unmarkAllSteppersDisabled()
    {
        flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;

#if FAN_BOARD_PIN>-1
        pwm_pos[NUM_EXTRUDER+1] = 255;
#endif // FAN_BOARD_PIN
    } // unmarkAllSteppersDisabled
    
    static void disableAllSteppersNow()
    {
        markAllSteppersDisabled();
        disableXStepper();
        disableYStepper();
        disableZStepper();
        Extruder::disableAllExtruders();
    } // disableAllSteppersNow
    
    static INLINE uint8_t isHoming()
    {
        return flag2 & PRINTER_FLAG2_HOMING;
    }

    static INLINE void setHoming(uint8_t b)
    {
        flag2 = (b ? flag2 | PRINTER_FLAG2_HOMING : flag2 & ~PRINTER_FLAG2_HOMING);
    }

    static INLINE bool isAnyTempsensorDefect()
    {
#if FEATURE_MILLING_MODE
        if( Printer::operatingMode != OPERATING_MODE_PRINT )
        {
            // we do not support temperature sensors in case we are not in operating mode print
            return 0;
        }
#endif // FEATURE_MILLING_MODE

        return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
    } // isAnyTempsensorDefect

    static INLINE bool isManualMoveMode()
    {
        return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
    } // isManualMoveMode

    static INLINE void setManualMoveMode(bool on)
    {
        flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
    } // setManualMoveMode

    static INLINE float lastCalculatedXPosition()
    {
        // return all values in [mm]
        return queuePositionLastMM[X_AXIS];
    } // lastCalculatedXPosition

    static INLINE float lastCalculatedYPosition()
    {
        // return all values in [mm]
        return queuePositionLastMM[Y_AXIS];
    } // lastCalculatedYPosition

    static INLINE float lastCalculatedZPosition()
    {
        // return all values in [mm]
        return queuePositionLastMM[Z_AXIS];
    } // lastCalculatedZPosition

    static INLINE void lastCalculatedPosition(float &xp,float &yp,float &zp)
    {
        // return all values in [mm]
        xp = lastCalculatedXPosition();
        yp = lastCalculatedYPosition();
        zp = lastCalculatedZPosition();
    } // lastCalculatedPosition

    static INLINE float targetXPosition()
    {
        // return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionTargetSteps[X_AXIS] + (float)directPositionTargetSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
#else
        return (float)queuePositionTargetSteps[X_AXIS] * invAxisStepsPerMM[X_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // targetXPosition

    static INLINE float targetYPosition()
    {
        // return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionTargetSteps[Y_AXIS] + (float)directPositionTargetSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
#else
        return (float)queuePositionTargetSteps[Y_AXIS] * invAxisStepsPerMM[Y_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // targetYPosition

    static inline float targetZPosition()
    {
        // return all values in [mm]
        float   fvalue = (float)queuePositionTargetSteps[Z_AXIS];


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        // add the current z-compensation
        fvalue += (float)Printer::compensatedPositionCurrentStepsZ;
        fvalue += (float)g_nZScanZPosition;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
        fvalue += (float)g_nZOriginPosition[Z_AXIS];
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        // add the current manual z-steps
        fvalue += (float)Printer::directPositionTargetSteps[Z_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

        fvalue *= Printer::invAxisStepsPerMM[Z_AXIS];
        return fvalue;

    } // targetZPosition

    static inline void targetPosition(float &xp,float &yp,float &zp)
    {
        // return all values in [mm]
        xp = targetXPosition();
        yp = targetYPosition();
        zp = targetZPosition();

    } // targetPosition
    
    static inline float currentXPosition()
    {
        // return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionCurrentSteps[X_AXIS] + (float)directPositionCurrentSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
#else
        return (float)queuePositionCurrentSteps[X_AXIS] * invAxisStepsPerMM[X_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // currentXPosition

    static INLINE float currentYPosition()
    {
        // return all values in [mm]
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        return ((float)queuePositionCurrentSteps[Y_AXIS] + (float)directPositionCurrentSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
#else
        return (float)queuePositionCurrentSteps[Y_AXIS] * invAxisStepsPerMM[Y_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

    } // currentYPosition

    static inline long currentZPositionSteps()
    {
        // return all values in [steps]
        long    value = queuePositionCurrentSteps[Z_AXIS] + Extruder::current->zOffset; //offset negativ, das ist normalerweise zu kompensieren/für den Betrachter uninteressant. also rausrechnen.


#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
        // add the current z-compensation
        value += Printer::compensatedPositionCurrentStepsZ; //da drin: zoffset + senseoffset + digitcompensation
        value += g_nZScanZPosition;
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
        value += g_nZOriginPosition[Z_AXIS];
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
        // add the current manual z-steps
        value += Printer::directPositionCurrentSteps[Z_AXIS];
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

        return value;

    } // currentZPositionSteps

    static inline float currentZPosition()
    {
        if (Printer::ZMode == Z_VALUE_MODE_LAYER)
        {
            // show the G-Code Commanded Z //offset negativ, das ist hier uninteressant.
            return (queuePositionCurrentSteps[Z_AXIS] + Extruder::current->zOffset) * Printer::invAxisStepsPerMM[Z_AXIS];
        }

        // return all values in [mm]
        float   fvalue = (float)currentZPositionSteps();

        if (Printer::ZMode <= Z_VALUE_MODE_Z_MIN)
        {
            // show the z-distance to z-min (print) or to the z-origin (mill)

            // When we see Z-Min the Extruder::current->zOffset (negative number) is not what we want to see. We want to see the diff with sensor-zeroing.
            fvalue -= (float)Extruder::current->zOffset; //adds z-Offset for T1 again to really show the axis-scale towards z-min hardware switch.
            
        }

        else if (Printer::ZMode == Z_VALUE_MODE_SURFACE)
        {
            // show the z-distance to the surface of the heat bed (print) or work part (mill)
#if FEATURE_HEAT_BED_Z_COMPENSATION
            fvalue -= (float)getHeatBedOffset();
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
            fvalue -= (float)getWorkPartOffset();
#endif // FEATURE_WORK_PART_Z_COMPENSATION
        }

        fvalue *= Printer::invAxisStepsPerMM[Z_AXIS];
        return fvalue;

    } // currentZPosition

    static inline void currentPosition(float &xp,float &yp,float &zp)
    {
        // return all values in [mm]
        xp = currentXPosition();
        yp = currentYPosition();
        zp = currentZPosition();

    } // currentPosition

    static INLINE void insertStepperHighDelay()
    {
#if STEPPER_HIGH_DELAY>0
        HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif // #if STEPPER_HIGH_DELAY>0
    } // insertStepperHighDelay

    static void constrainQueueDestinationCoords();
    static void constrainDirectDestinationCoords();
    static void updateDerivedParameter();
    static void updateCurrentPosition(bool copyLastCmd = false);
    static void kill(uint8_t only_steppers);
    static void updateAdvanceFlags();
    static void setup();
    static void defaultLoopActions();
    static uint8_t setDestinationStepsFromGCode(GCode *com);
    static uint8_t setDestinationStepsFromMenu( float relativeX, float relativeY, float relativeZ );
    static void moveTo(float x,float y,float z,float e,float feedrate);
    static void moveToReal(float x,float y,float z,float e,float feedrate);
    static void homeAxis(bool xaxis,bool yaxis,bool zaxis); /// Home axis
    static void setOrigin(float xOff,float yOff,float zOff);

    static INLINE int getFanSpeed(bool percent = false)
    {
        if(!percent) return (int)pwm_pos[NUM_EXTRUDER+2]; //int
        if(!pwm_pos[NUM_EXTRUDER+2]) return 0; //%
        if(pwm_pos[NUM_EXTRUDER+2] <= 3) return 1; //%
        return (int)(pwm_pos[NUM_EXTRUDER+2]*100/255); //%
    } // getFanSpeed

#if FEATURE_MEMORY_POSITION
    static void MemoryPosition();
    static void GoToMemoryPosition(bool x,bool y,bool z,bool e,float feed);
#endif // FEATURE_MEMORY_POSITION

    static bool allowQueueMove( void );

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
    static bool allowDirectMove( void );
    static bool allowDirectSteps( void );
    static bool processAsDirectSteps( void );
    static void resetDirectPosition( void );
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

#if FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION
    static void performZCompensation( void );
#if FEATURE_SENSIBLE_PRESSURE
    static void enableSenseOffsetnow( void );
#endif // FEATURE_SENSIBLE_PRESSURE
    static void enableCMPnow( void );
    static void disableCMPnow( bool wait = false );
#endif // FEATURE_HEAT_BED_Z_COMPENSATION || FEATURE_WORK_PART_Z_COMPENSATION

    static void stopPrint();
    
private:
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();

};

#endif // PRINTER_H
