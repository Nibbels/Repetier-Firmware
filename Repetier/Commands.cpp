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


#include "Repetier.h"

int         Commands::lowestRAMValue     = MAX_RAM;
int         Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop()
{
    GCode::readFromSerial();
    GCode *code = GCode::peekCurrentCommand();
    if(code)
    {
#if SDSUPPORT
        if(sd.savetosd)
        {
            if(!(code->hasM() && code->M == 29))   // still writing to file
                sd.writeCommand(code);
            else
                sd.finishWrite();
#ifdef ECHO_ON_EXECUTE
            code->echoCommand();
#endif
        }
        else
#endif //SDSUPPORT
            Commands::executeGCode(code);
        code->popCurrentCommand();
    }
    Printer::defaultLoopActions();
} // commandLoop


void Commands::checkForPeriodicalActions(enum FirmwareState state)
{
    bool buttonactive = ((HAL::timeInMilliseconds() - uid.lastButtonStart < 15000) ? true : false);
    
    if( state != NotBusy ){
        GCode::keepAlive( state );
    }
    
    if(execute10msPeriodical){ //set by PWM-Timer
      execute10msPeriodical=0;

      HAL::tellWatchdogOk();  //dieses freigabesignal sollte aus dem PWM-Timer kommen, denn dann ist klar, dass auch der noch läuft. Dann laufen für den Watchdogreset der Timer und checkForPeriodicalActions().
      doZCompensation();  //100x pro sekunde -> 100mm/s : jede 1mm Weg

    }

    if(execute16msPeriodical){ //set by internal Watchdog-Timer
      execute16msPeriodical = 0;
      if(buttonactive) UI_SLOW;

    }

    if(execute100msPeriodical){ //set by PWM-Timer
      execute100msPeriodical=0;

      loopRF();

      if(!buttonactive) UI_SLOW;

      Extruder::manageTemperatures();
      Commands::printTemperatures(); //selfcontrolling timediff

    }
} // checkForPeriodicalActions


/** \brief Waits until movement cache is empty.
    Some commands expect no movement, before they can execute. This function
    waits, until the steppers are stopped. In the meanwhile it buffers incoming
    commands and manages temperatures. */
void Commands::waitUntilEndOfAllMoves()
{
    bool    bWait = false;

    if( PrintLine::hasLines() )     bWait = true;
#if FEATURE_FIND_Z_ORIGIN
    if( g_nFindZOriginStatus )      bWait = true;
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_HEAT_BED_Z_COMPENSATION
    bWait = (Printer::needsCMPwait() ? true : bWait);
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

    while( bWait )
    {
        Commands::checkForPeriodicalActions( Processing );

        bWait = false;
        if( PrintLine::hasLines() )     bWait = true;
        else                            GCode::readFromSerial(); //normalerweise braucht repetiert hier bei PrintLine::haslines kein readserial! aber wenn wir für die anderen wait=1 readserial wollen, evtl. schon. 
        
#if FEATURE_FIND_Z_ORIGIN
        if( g_nFindZOriginStatus )      bWait = true;
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_HEAT_BED_Z_COMPENSATION
        bWait = (Printer::needsCMPwait() ? true : bWait);
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

    }

} // waitUntilEndOfAllMoves


/** \brief Waits until M3900 is finished. */
void Commands::waitUntilEndOfZOS()
{
    char    bWait = 0;

    if( g_nZOSScanStatus )       bWait = 1;

    while( bWait )
    {
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions( Processing );

        bWait = 0;
        if( g_nZOSScanStatus )       bWait = 1;
    }

} // waitUntilEndOfZOS


void Commands::printCurrentPosition()
{
    float x,y,z;
    Printer::currentPosition(x,y,z);
    x += Printer::originOffsetMM[X_AXIS];
    y += Printer::originOffsetMM[Y_AXIS];
    z += Printer::originOffsetMM[Z_AXIS];
    Com::printF(Com::tXColon,x*(Printer::unitIsInches?0.03937:1),2);
    Com::printF(Com::tSpaceYColon,y*(Printer::unitIsInches?0.03937:1),2);
    Com::printF(Com::tSpaceZColon,z*(Printer::unitIsInches?0.03937:1),2);
    Com::printFLN(Com::tSpaceEColon,Printer::queuePositionLastSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS]*(Printer::unitIsInches?0.03937:1),2);
} // printCurrentPosition


void Commands::printTemperatures(bool showRaw)
{
    static millis_t lastTemperatureSignal = 0;
    millis_t now = HAL::timeInMilliseconds();
    if( (now - lastTemperatureSignal) > 1000 ){
        lastTemperatureSignal = now;


        Com::printF(Com::tTColon,Extruder::current->tempControl.currentTemperatureC,1);
        Com::printF(Com::tSpaceSlash,Extruder::current->tempControl.targetTemperatureC,0);
        // Show output of autotune when tuning!
        Com::printF(Com::tSpaceAtColon,(pwm_pos[Extruder::current->id]));

#if HAVE_HEATED_BED
        Com::printF(Com::tSpaceBColon,Extruder::getHeatedBedTemperature(),1);
        Com::printF(Com::tSpaceSlash,heatedBedController.targetTemperatureC,0);

        if(showRaw)
        {
            Com::printF(Com::tSpaceRaw,(int)NUM_EXTRUDER);
            Com::printF(Com::tColon,(1023<<(2-ANALOG_REDUCE_BITS))-heatedBedController.currentTemperature);
        }
        Com::printF(Com::tSpaceBAtColon,(pwm_pos[heatedBedController.pwmIndex])); // Show output of autotune when tuning!
#endif // HAVE_HEATED_BED

#if NUM_EXTRUDER>1
        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
        {
            Com::printF(Com::tSpaceT,(int)i);
            Com::printF(Com::tColon,extruder[i].tempControl.currentTemperatureC,1);
            Com::printF(Com::tSpaceSlash,extruder[i].tempControl.targetTemperatureC,0);

            Com::printF(Com::tSpaceAt,(int)i);
            Com::printF(Com::tColon,(pwm_pos[extruder[i].tempControl.pwmIndex])); // Show output of autotune when tuning!

            if(showRaw)
            {
                Com::printF(Com::tSpaceRaw,(int)i);
                Com::printF(Com::tColon,(1023<<(2-ANALOG_REDUCE_BITS))-extruder[i].tempControl.currentTemperature);
            }
        }
#endif // NUM_EXTRUDER

#if RESERVE_ANALOG_INPUTS
        //Act as heated chamber ambient temperature for Repetier-Server 0.86.2+ ---> Letter C
        TemperatureController* act = &optTempController;
        act->updateCurrentTemperature();
        Com::printF(Com::tSpaceChamber );
        Com::printF(Com::tColon,act->currentTemperatureC,1); //temp
        Com::printF(Com::tSpaceSlash,0,0); // ziel-temp
        Com::printF(Com::tSpaceCAtColon,0); // leistung
#endif // RESERVE_ANALOG_INPUTS

#if FEATURE_PRINT_PRESSURE
        Com::printF(Com::tSpace);
        Com::printF(Com::tF);
        Com::printF(Com::tColon,(int)g_nLastDigits); //Kraft
#if FEATURE_ZERO_DIGITS
        Com::printF(Com::tSpaceSlash,(int)(Printer::g_pressure_offset_active ? Printer::g_pressure_offset : 0) ); //Offset
#else
        Com::printF(Com::tSpaceSlash,0); //Offset 0
#endif // FEATURE_ZERO_DIGITS
        Com::printF(Com::tSpaceAtColon,0); //Ziel ^^, nein ich halte mich nur an die PWM-Syntax
#endif //FEATURE_PRINT_PRESSURE

    Com::println();
    }
} // printTemperatures


void Commands::changeFeedrateMultiply(int factor)
{
    if(factor<25) factor=25;
    if(factor>500) factor=500;
    Printer::feedrate *= (float)factor/(float)Printer::feedrateMultiply;
    Printer::feedrateMultiply = factor;

    if( Printer::debugInfo() )
    {
        Com::printFLN(Com::tSpeedMultiply,factor);
    }

} // changeFeedrateMultiply


void Commands::changeFlowrateMultiply(float factorpercent)
{
    if(factorpercent < 25.0f)   factorpercent = 25.0f;
    if(factorpercent > 200.0f)  factorpercent = 200.0f;
    Printer::extrudeMultiply = static_cast<int>(factorpercent);
    
    //if(Extruder::current->diameter <= 0)
        Printer::extrusionFactor = 0.01f * static_cast<float>(factorpercent);
    //else
    //    Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);

    Com::printFLN(Com::tFlowMultiply,factorpercent);
} // changeFlowrateMultiply


#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
uint8_t fanKickstart = 0;
void Commands::setFanSpeed(int speed)
{
    uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
    if(pwm_pos[NUM_EXTRUDER+2] == trimmedSpeed)
        return;
    
    Printer::setMenuMode(MENU_MODE_FAN_RUNNING,trimmedSpeed!=0);
    Com::printFLN(Com::tFanspeed,(trimmedSpeed == 1) ? 2 : trimmedSpeed ); //bei 1 zeigt repetierserver / repetierhost 0% an, was nicht stimmt. Das ist etwas Pfusch, aber nun funktionierts.
    if(fanKickstart == 0 && trimmedSpeed > pwm_pos[NUM_EXTRUDER+2] && trimmedSpeed < 85) {
        if(pwm_pos[NUM_EXTRUDER+2]) fanKickstart = FAN_KICKSTART_TIME / 100;
        else                        fanKickstart = FAN_KICKSTART_TIME / 25;
    }
    pwm_pos[NUM_EXTRUDER+2] = trimmedSpeed;
} // setFanSpeed
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
void Commands::adjustFanFrequency(uint8_t speed_mode = COOLER_PWM_SPEED){ //0 = ~15hz, ~1=30hz, ... 4=240hz.
    InterruptProtectedBlock noInts;
    cooler_pwm_speed = (speed_mode <= COOLER_MODE_MAX ? speed_mode : COOLER_PWM_SPEED); // > 4 should not happen, only if you change code and want that. 
    cooler_pwm_step = (1 << cooler_pwm_speed);
    cooler_pwm_mask = (255 << cooler_pwm_speed) & 0xff;
}
void Commands::adjustFanMode(uint8_t output_mode){ //0 = pwm, 1 = pdm
    cooler_mode = (output_mode ? COOLER_MODE_PDM : COOLER_MODE_PWM);
    Printer::setMenuMode(MENU_MODE_FAN_MODE_PDM, (bool)cooler_mode);
}
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

void Commands::reportPrinterUsage()
{
#if EEPROM_MODE!=0
#if FEATURE_MILLING_MODE
    if( Printer::operatingMode == OPERATING_MODE_PRINT )
    {
#endif // FEATURE_MILLING_MODE
        if( Printer::debugInfo() )
        {
            float dist = Printer::filamentPrinted*0.001+HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
            Com::printF(Com::tPrintedFilament,dist,2);
            Com::printF(Com::tSpacem);
        }
        bool alloff = true;

        for(uint8_t i=0; i<NUM_EXTRUDER; i++)
            if(tempController[i]->targetTemperatureC > 0) alloff = false;

        if( Printer::debugInfo() )
        {
            int32_t seconds =  (alloff ? 0 : (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000)+HAL::eprGetInt32(EPR_PRINTING_TIME);
            int32_t tmp     =  seconds/86400;
            seconds         -= tmp*86400;

            Com::printF(Com::tPrintingTime,tmp);
            tmp = seconds/3600;
            Com::printF(Com::tSpaceDaysSpace,tmp);
            seconds -= tmp*3600;
            tmp = seconds/60;

            Com::printF(Com::tSpaceHoursSpace,tmp);
            Com::printFLN(Com::tSpaceMin);
        }
#if FEATURE_SERVICE_INTERVAL
        if( Printer::debugInfo() )
        {
            float dist_service = Printer::filamentPrinted*0.001+HAL::eprGetFloat(EPR_PRINTING_DISTANCE_SERVICE);
            Com::printF(Com::tPrintedFilamentService,dist_service,2);
            Com::printF(Com::tSpacem);

            int32_t uSecondsServicePrint =  (alloff ? 0 : (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000)+HAL::eprGetInt32(EPR_PRINTING_TIME_SERVICE);
            int32_t tmp_service     =  uSecondsServicePrint/86400;
            uSecondsServicePrint            -= tmp_service*86400;

            Com::printF(Com::tPrintingTimeService,tmp_service);
            tmp_service = uSecondsServicePrint/3600;
            Com::printF(Com::tSpaceDaysSpace,tmp_service);
            uSecondsServicePrint -= tmp_service*3600;
            tmp_service = uSecondsServicePrint/60;

            Com::printF(Com::tSpaceHoursSpace,tmp_service);
            Com::printFLN(Com::tSpaceMin);
        }
#endif // FEATURE_SERVICE_INTERVAL
#if FEATURE_MILLING_MODE
    }
    else
    {
        if( Printer::debugInfo() )
        {
            int32_t seconds =  (HAL::timeInMilliseconds()-Printer::msecondsMilling)/1000 + HAL::eprGetInt32(EPR_MILLING_TIME);
            int32_t tmp     =  seconds/86400;
            seconds         -= tmp*86400;

            Com::printF(Com::tMillingTime,tmp);
            tmp = seconds/3600;
            Com::printF(Com::tSpaceDaysSpace,tmp);
            seconds -= tmp*3600;
            tmp = seconds/60;

            Com::printF(Com::tSpaceHoursSpace,tmp);
            Com::printFLN(Com::tSpaceMin);
        }
#if FEATURE_SERVICE_INTERVAL
        if( Printer::debugInfo() )
        {
            int32_t uSecondsServicePrint =  (HAL::timeInMilliseconds()-Printer::msecondsMilling)/1000 + HAL::eprGetInt32(EPR_MILLING_TIME_SERVICE);
            int32_t tmp_service     =  uSecondsServicePrint/86400;
            uSecondsServicePrint            -= tmp_service*86400;

            Com::printF(Com::tMillingTimeService,tmp_service);
            tmp_service = uSecondsServicePrint/3600;
            Com::printF(Com::tSpaceDaysSpace,tmp_service);
            uSecondsServicePrint -= tmp_service*3600;
            tmp_service = uSecondsServicePrint/60;

            Com::printF(Com::tSpaceHoursSpace,tmp_service);
            Com::printFLN(Com::tSpaceMin);
        }
#endif // FEATURE_SERVICE_INTERVAL
    }
#endif // FEATURE_MILLING_MODE
#endif // EEPROM_MODE
} // reportPrinterUsage


/** \brief Execute the command stored in com. */
void Commands::executeGCode(GCode *com)
{
    uint32_t codenum; //throw away variable
    if(com->hasG())
    {
      switch(com->G)
      {
        case 0: // G0 -> G1
        {
            if(!isMovingAllowed(PSTR("G0")))
            {
                break;
            }
            
            // fall through
        }
        case 1: // G1
        {
            if(!isMovingAllowed(PSTR("G1")))
            {
                break;
            }
            if(com->hasS())
            {
                Printer::setNoDestinationCheck(com->S!=0);
            }
            if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
            {
                PrintLine::prepareQueueMove(ALWAYS_CHECK_ENDSTOPS,true, Printer::feedrate);
            }
            break;
        }

#if FEATURE_ARC_SUPPORT
        case 2: // G2 - Clockwise Arc
        {
            if(!isMovingAllowed(PSTR("G2")))
            {
                break;
            }
            
            // fall through
        }
        case 3: // G3 - Counter-clockwise Arc
        {
            if(!isMovingAllowed(PSTR("G3")))
            {
                break;
            }

            float position[3];
            Printer::lastCalculatedPosition(position[X_AXIS],position[Y_AXIS],position[Z_AXIS]); //fill with queuePositionLastMM[]
            if(!Printer::setDestinationStepsFromGCode(com)) break; // For X Y Z E F

            float offset[2] = {Printer::convertToMM(com->hasI()?com->I:0),Printer::convertToMM(com->hasJ()?com->J:0)};
            float target[4] = {Printer::queuePositionLastMM[X_AXIS],Printer::queuePositionLastMM[Y_AXIS],Printer::queuePositionLastMM[Z_AXIS],Printer::queuePositionTargetSteps[E_AXIS]*Printer::invAxisStepsPerMM[E_AXIS]};
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
                    if( Printer::debugErrors() )
                    {
                        Com::printErrorFLN(Com::tInvalidArc);
                    }
                    break;
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
            PrintLine::arc(position, target, offset,r, isclockwise);
            break;
        }
#endif // FEATURE_ARC_SUPPORT

        case 4: // G4 dwell
        {
            Commands::waitUntilEndOfAllMoves(); //G4
            codenum = 0;
            if(com->hasP()) codenum = com->P; // milliseconds to wait
            if(com->hasS()) codenum = com->S * 1000; // seconds to wait
            codenum += HAL::timeInMilliseconds();  // keep track of when we started waiting

            while((uint32_t)(codenum-HAL::timeInMilliseconds())  < 2000000000 )
            {
                Commands::checkForPeriodicalActions( Processing );
            }
            break;
        }
        case 20: // G20 - Units to inches
        {
            Printer::unitIsInches = 1;
            break;
        }
        case 21: // G21 - Units to mm
        {
            Printer::unitIsInches = 0;
            break;
        }
        case 28:  // G28 - Home all axes one at a time
        {
            if(!isHomingAllowed(com))
            {
                break;
            }
            uint8_t home_all_axis = (com->hasNoXYZ() && !com->hasE());
            if(com->hasE())
            {
                Printer::queuePositionLastSteps[E_AXIS] = 0;
            }
            if(home_all_axis || !com->hasNoXYZ())
                Printer::homeAxis(home_all_axis || com->hasX(),home_all_axis || com->hasY(),home_all_axis || com->hasZ());
            Printer::updateCurrentPosition();
        }
        break;

#if FEATURE_MILLING_MODE
        case 80: // G80
        {
            if( isSupportedGCommand( com->G, OPERATING_MODE_MILL ) )
            {
                // there is not a lot to do at the moment because drilling cycles are not supported
                Printer::drillFeedrate = 0.0;
                Printer::drillZDepth   = 0.0;
            }
            break;
        }
        case 81: // G81
        {
            if( isSupportedGCommand( com->G, OPERATING_MODE_MILL ) )
            {
                char    szTemp[40];
                float   exitZ;


/*              Com::printFLN( PSTR( "G81 detected" ) );
                if( com->hasX() )   Com::printFLN( PSTR( "X = " ), com->X );
                if( com->hasY() )   Com::printFLN( PSTR( "Y = " ), com->Y );
                if( com->hasZ() )   Com::printFLN( PSTR( "Z = " ), com->Z );
                if( com->hasR() )   Com::printFLN( PSTR( "R = " ), com->R );
                if( com->hasF() )   Com::printFLN( PSTR( "F = " ), com->F );
*/
                if(!isMovingAllowed(PSTR("G81")))
                {
                    break;
                }

                // safety checks
                if( !com->hasZ() && !Printer::drillZDepth )
                {
                    if( Printer::debugErrors() )
                    {
                        Com::printFLN( PSTR( "G81: aborted (the Z position is not defined)" ) );
                    }
                    break;
                }
                if( !com->hasF() && !Printer::drillFeedrate )
                {
                    if( Printer::debugErrors() )
                    {
                        Com::printFLN( PSTR( "G81: aborted (the drilling feedrate is not defined)" ) );
                    }
                    break;
                }

                if( Printer::relativeCoordinateMode )
                {
                    // move to the position of the (to be drilled) hole
                    strcpy( szTemp, "G1 X" );
                    addFloat( szTemp, com->hasX() ? com->X : 0, 3 );
                    strcat( szTemp, " Y" );
                    addFloat( szTemp, com->hasY() ? com->Y : 0, 3 );
                    strcat( szTemp, " Z" );
                    addFloat( szTemp, com->hasR() ? com->R : 0, 3 );
                    strcat( szTemp, " F" );
                    addFloat( szTemp, Printer::maxFeedrate[X_AXIS], 3 );
                    GCode::executeString( szTemp );

                    // in order to leave the hole, we must travel the drilling path in reverse direction
                    exitZ = -(com->hasZ() ? com->Z : Printer::drillZDepth);
                }
                else
                {
                    // move to the position of the (to be drilled) hole
                    strcpy( szTemp, "G1" );
                    if( com->hasX() )
                    {
                        strcat( szTemp, " X" );
                        addFloat( szTemp, com->X, 3 );
                    }
                    if( com->hasY() )
                    {
                        strcat( szTemp, " Y" );
                        addFloat( szTemp, com->Y, 3 );
                    }
                    if( com->hasR() )
                    {
                        strcat( szTemp, " Z" );
                        addFloat( szTemp, com->R, 3 );
                    }
                    strcat( szTemp, " F" );
                    addFloat( szTemp, Printer::maxFeedrate[X_AXIS], 3 );
                    GCode::executeString( szTemp );

                    // in order to leave the hole, we must return to our start position
                    exitZ = Printer::queuePositionLastMM[Z_AXIS];
                }

                // drill the hole
                strcpy( szTemp, "G1 Z" );
                addFloat( szTemp, com->hasZ() ? com->Z : Printer::drillZDepth, 3 );
                strcat( szTemp, " F" );
                addFloat( szTemp, com->hasF() ? com->F : Printer::drillFeedrate, 3 );

                GCode::executeString( szTemp );

                // get out of the hole
                strcpy( szTemp, "G1 Z" );
                addFloat( szTemp, exitZ, 3 );
                strcat( szTemp, " F" );
                addFloat( szTemp, Printer::maxFeedrate[Z_AXIS], 3 );

                GCode::executeString( szTemp );

                if( com->hasZ() )   Printer::drillZDepth   = com->Z;
                if( com->hasF() )   Printer::drillFeedrate = com->F;
            }
            break;
        }
#endif // FEATURE_MILLING_MODE

        case 90: // G90
        {
            Printer::relativeCoordinateMode = false;
            if(com->internalCommand)
                Com::printInfoFLN(PSTR("Absolute positioning"));
            break;
        }
        case 91: // G91
        {
            Printer::relativeCoordinateMode = true;
            if(com->internalCommand)
                Com::printInfoFLN(PSTR("Relative positioning"));
            break;
        }
        case 92: // G92
        {
            if(!com->hasNoXYZ())
            {
                // set the origin only in case we got any x, y and/or z offset
                float xOff = Printer::originOffsetMM[X_AXIS];
                float yOff = Printer::originOffsetMM[Y_AXIS];
                float zOff = Printer::originOffsetMM[Z_AXIS];

                if(com->hasX()) xOff = Printer::convertToMM(com->X)-Printer::queuePositionLastMM[X_AXIS];
                if(com->hasY()) yOff = Printer::convertToMM(com->Y)-Printer::queuePositionLastMM[Y_AXIS];
                if(com->hasZ()) zOff = Printer::convertToMM(com->Z)-Printer::queuePositionLastMM[Z_AXIS];
                Printer::setOrigin(xOff,yOff,zOff);
            }
            if(com->hasE())
            {
                Printer::queuePositionTargetSteps[E_AXIS] = Printer::queuePositionLastSteps[E_AXIS] = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
                /* Repetier: https://github.com/repetier/Repetier-Firmware/commit/a63c660289b760faf033fdfd86bbc69c1050cfc9 ?
                  Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
                wissen:
                Printer::queuePositionTargetSteps[axis] <---> Printer::destinationSteps[axis]
                Printer::queuePositionCurrentSteps[axis] <---> Printer::currentPositionSteps[axis]
                */
            }
            break;
        }
      }
      previousMillisCmd = HAL::timeInMilliseconds();
    }
    else if(com->hasM())    // Process M Code
    {
        switch( com->M )
        {
#if SDSUPPORT
            case 20: // M20 - list SD card
            {
                sd.ls();
                break;
            }
            case 21: // M21 - init SD card
            {
                sd.mount();
                break;
            }
            case 22: // M22 - release SD card
            {
                sd.unmount();
                break;
            }
            case 23: // M23 - Select file
            {
                if(com->hasString())
                {
                    sd.fat.chdir();
                    sd.selectFileByName(com->text);
                }
                break;
            }
            case 24: // M24 - Start SD print
            {
#if FEATURE_PAUSE_PRINTING
                if( g_pauseStatus == PAUSE_STATUS_PAUSED ) 
                {
                    continuePrint();
                }
                else
#endif // FEATURE_PAUSE_PRINTING
                {
                    sd.startPrint();
                }
                break;
            }
            case 25: // M25 - Pause SD print
            {
#if FEATURE_PAUSE_PRINTING
                pausePrint();
#endif // FEATURE_PAUSE_PRINTING
                break;
            }
            case 26: // M26 - Set SD index
            {
                if( Printer::debugErrors() )
                {
                    Com::printFLN( PSTR( "M26: not supported" ) );
                }

/*              if(com->hasS())
                    sd.setIndex(com->S);
*/            break;
            }
            case 27: // M27 - Get SD status
                sd.printStatus();
                break;
            case 28: // M28 - Start SD write
                if(com->hasString())
                    sd.startWrite(com->text);
                break;
            case 29: // M29 - Stop SD write
                //processed in write to file routine above
                //savetosd = false;
                break;
            case 30: // M30 - filename - Delete file
                if(com->hasString())
                {
                    sd.fat.chdir();
                    sd.deleteFile(com->text);
                }
                break;
            case 32: // M32 - directoryname
                if(com->hasString())
                {
                    sd.fat.chdir();
                    sd.makeDirectory(com->text);
                }
                break;
#endif // SDSUPPORT

            case 104: // M104 - set extruder temp
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
#if NUM_EXTRUDER>0
                    if(reportTempsensorError()) break;
                    previousMillisCmd = HAL::timeInMilliseconds();
                    if(Printer::debugDryrun()) break;

                    //TODO man müsste das in den Movecache legen!
                    if(com->hasP() || (com->hasS() && com->S == 0))
                    Commands::waitUntilEndOfAllMoves(); //M104

                    if (com->hasS())
                    {
                        if(com->hasT())
                            Extruder::setTemperatureForExtruder(com->S,com->T,com->hasF() && com->F>0);
                        else
                            Extruder::setTemperatureForExtruder(com->S,Extruder::current->id,com->hasF() && com->F>0);
                    }
#endif // NUM_EXTRUDER>0
                }
                break;
            }
            case 105: // M105 - get temperature. Always returns the current temperature, doesn't wait until move stopped
            {
                printTemperatures(com->hasX());
                break;
            }
            case 140: // M140 - set bed temp
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
                    if(reportTempsensorError()) break;
                    previousMillisCmd = HAL::timeInMilliseconds();
                    if(Printer::debugDryrun()) break;
                    if (com->hasS()) Extruder::setHeatedBedTemperature(com->S,com->hasF() && com->F>0);
                }
                break;
            }
            case 109: // M109 - Wait for extruder heater to reach target.
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
#if NUM_EXTRUDER>0
                    if(reportTempsensorError()) break;
                    previousMillisCmd = HAL::timeInMilliseconds();
                    if(Printer::debugDryrun()) break;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    Printer::waitMove = 1; //brauche ich das, wenn ich sowieso warte bis der movecache leer ist?
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

                    g_uStartOfIdle = 0; //M109
                    Commands::waitUntilEndOfAllMoves(); //M109
                    Extruder *actExtruder = Extruder::current;
                    if (com->hasT() && com->T<NUM_EXTRUDER) actExtruder = &extruder[com->T];
                    if (com->hasS()) Extruder::setTemperatureForExtruder(com->S,actExtruder->id,com->hasF() && com->F>0);

                    if (fabs(actExtruder->tempControl.targetTemperatureC - actExtruder->tempControl.currentTemperatureC) < TEMP_TOLERANCE)
                    {
                        // we are already in range
#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                        Printer::waitMove = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                        break;
                    }

#if RETRACT_DURING_HEATUP
                    uint8_t     retracted = 0;
#endif // RETRACT_DURING_HEATUP
                    millis_t    waituntil   = 0;
                    millis_t    currentTime;
                    bool        isTempReached;
                    bool        longTempTime = false; // random init
                    bool        dirRising = true;     // random init
                    float       settarget = -1;       // random init in °C

                    do
                    {
                        Commands::printTemperatures();
                        Commands::checkForPeriodicalActions( WaitHeater );

                        //Anpassung an die neue Situation falls der Bediener am Display-Menü des Druckers während Aufheizzeit was umstellt.
                        if(settarget != actExtruder->tempControl.targetTemperatureC){
                            settarget = actExtruder->tempControl.targetTemperatureC; //in °C
                            dirRising = (actExtruder->tempControl.targetTemperatureC > actExtruder->tempControl.currentTemperatureC);
                            longTempTime = (fabs(actExtruder->tempControl.targetTemperatureC - actExtruder->tempControl.currentTemperatureC) > 40.0f ? true : false);
                            if( dirRising ){
                                UI_STATUS_UPD(UI_TEXT_HEATING_EXTRUDER);
                            }else{
                                UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
                            }
                        }

                        currentTime = HAL::timeInMilliseconds();
                        isTempReached = (dirRising ? actExtruder->tempControl.currentTemperatureC >= actExtruder->tempControl.targetTemperatureC - TEMP_TOLERANCE
                                                   : actExtruder->tempControl.currentTemperatureC <= actExtruder->tempControl.targetTemperatureC + TEMP_TOLERANCE);

#if RETRACT_DURING_HEATUP
                        if( dirRising ){
                            if (!retracted 
                                && longTempTime
                                && actExtruder == Extruder::current
                                && actExtruder->waitRetractUnits > 0
                                && actExtruder->tempControl.currentTemperatureC >= actExtruder->waitRetractTemperature)
                            {
                                PrintLine::moveRelativeDistanceInSteps(0,0,0,-actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS],actExtruder->maxFeedrate,false,false);
                                retracted = 1;
                            }
                        }
#endif // RETRACT_DURING_HEATUP
                        if( !dirRising ){
                            if( actExtruder->tempControl.currentTemperatureC <= MAX_ROOM_TEMPERATURE ){
                                isTempReached = true;
                                //never wait longer than reaching lowest allowed temperature. 
                                //This might still be a long-run-bug if you have heated chamber/hot summer and wrong settings in MAX_ROOM_TEMPERATURE!
                            }
                        }

                        if(waituntil == 0 && isTempReached) //waituntil bleibt 0 bis temperatur einmal erreicht.
                            {
                                waituntil = currentTime+1000UL*(millis_t)actExtruder->watchPeriod; // now wait for temp. to stabalize
                            }
                    }
                    while(waituntil==0 || (waituntil!=0 && (millis_t)(waituntil-currentTime)<2000000000UL));

#if RETRACT_DURING_HEATUP
                    if (retracted && actExtruder==Extruder::current)
                    {
                        PrintLine::moveRelativeDistanceInSteps(0,0,0,actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS],actExtruder->maxFeedrate,false,false);
                    }
#endif // RETRACT_DURING_HEATUP
#endif // NUM_EXTRUDER>0

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    Printer::waitMove = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    g_uStartOfIdle    = HAL::timeInMilliseconds(); //end of M109
                }
                previousMillisCmd = HAL::timeInMilliseconds();
                break;
            }
            case 190: // M190 - Wait bed for heater to reach target.
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
#if HAVE_HEATED_BED
                    if(Printer::debugDryrun()) break;

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    Printer::waitMove = 1; //brauche ich das, wenn ich sowieso warte bis der movecache leer ist?
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING

                    g_uStartOfIdle = 0; //M190
                    Commands::waitUntilEndOfAllMoves(); //M190
                    if (com->hasS()) Extruder::setHeatedBedTemperature(com->S,com->hasF() && com->F>0);

                    bool    dirRising = true; // random init
                    float   settarget = -1;   // random init in °C

                    while( fabs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) > TEMP_TOLERANCE )
                    {
                        //Init und Anpassung an die neue Situation falls der Bediener am Display-Menü des Druckers während Aufheizzeit was umstellt.
                        if(settarget != heatedBedController.targetTemperatureC){
                            settarget = heatedBedController.targetTemperatureC; //in °C
                            dirRising = (heatedBedController.targetTemperatureC > heatedBedController.currentTemperatureC);
                            if( dirRising ){
                                UI_STATUS_UPD(UI_TEXT_HEATING_BED);
                            }else{
                                UI_STATUS_UPD(UI_TEXT_COOLING_DOWN);
                            }
                        }

                        Commands::printTemperatures();
                        Commands::checkForPeriodicalActions( WaitHeater );
                        if( !dirRising && heatedBedController.currentTemperatureC <= MAX_ROOM_TEMPERATURE ) break;
                    }

#if FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    Printer::waitMove = 0;
#endif // FEATURE_EXTENDED_BUTTONS || FEATURE_PAUSE_PRINTING
                    g_uStartOfIdle    = HAL::timeInMilliseconds()+5000; //end of M190
#endif // HAVE_HEATED_BED
                }

                previousMillisCmd = HAL::timeInMilliseconds();
                break;
            }
            case 116: // M116 - Wait for temperatures to reach target temperature
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
                    if(Printer::debugDryrun()) break;
                    {
                        bool allReached = false;
                        while(!allReached)
                        {
                            allReached = true;
                            Commands::printTemperatures();
                            Commands::checkForPeriodicalActions( WaitHeater );

                            for( uint8_t h=0; h<NUM_TEMPERATURE_LOOPS; h++ )
                            {
                                TemperatureController *act = tempController[h];
                                if( act->targetTemperatureC > MAX_ROOM_TEMPERATURE && fabs( act->targetTemperatureC-act->currentTemperatureC ) > TEMP_TOLERANCE )
                                {
                                    allReached = false;
                                }
                            }
                        }
                    }
                }
                break;
            }

#if FEATURE_DITTO_PRINTING
            case 280:   // M280
            {
                if(com->hasS())   // Set ditto mode S: 0 = off, 1 = on
                {
                    Extruder::dittoMode = com->S;
                }
                break;
            }
#endif // FEATURE_DITTO_PRINTING

#if BEEPER_PIN >= 0
            case 300:   // M300
            {
                int beepS = 1;
                int beepP = 1000;
                if(com->hasS()) beepS = com->S;
                if(com->hasP()) beepP = com->P;
                HAL::tone(BEEPER_PIN, beepS);
                HAL::delayMilliseconds(beepP);
                HAL::noTone(BEEPER_PIN);
            }
            break;
#endif // BEEPER_PIN >= 0

            case 303:   // M303
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
#if NUM_TEMPERATURE_LOOPS > 0
                int temp = 150;
                int cont = 0;
                int cycles = 10;
                int method = 0; //0 = Classic PID
                if(com->hasS()) temp = com->S; //Verwechsle ich immer, weil T wie Temperatur, aber T ist 0..255
                if(com->hasP()) cont = com->P;
                if(com->hasR()) cycles = static_cast<int>(com->R);
                if(com->hasJ()) method = static_cast<int>(com->J); //original Repetier used hasC, we dont have that in this version of repetier.
                if(cont >= NUM_TEMPERATURE_LOOPS) cont = NUM_TEMPERATURE_LOOPS -1;
                if(cont < 0) cont = 0;
                tempController[cont]->autotunePID(temp,cont,cycles,com->hasX(), method);
#endif // NUM_TEMPERATURE_LOOPS > 0
                }
                break;
            }

#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
            case 106:   // M106 - Fan On
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
                    setFanSpeed(com->hasS()?com->S:255);
                }
                break;
            }
            case 107:   // M107 - Fan Off
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
                    setFanSpeed(0);
                }
                break;
            }
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

            case 80:    // M80 - ATX Power On
            {
#if PS_ON_PIN > -1
                Commands::waitUntilEndOfAllMoves();  //M80 command
                previousMillisCmd = HAL::timeInMilliseconds();
                SET_OUTPUT(PS_ON_PIN); //GND
                WRITE(PS_ON_PIN, (POWER_INVERTING ? HIGH : LOW));
#endif // PS_ON_PIN > -1
                break;
            }
            case 81:    // M81 - ATX Power Off
            {
#if PS_ON_PIN > -1
                Commands::waitUntilEndOfAllMoves(); //M81
                SET_OUTPUT(PS_ON_PIN); //GND
                WRITE(PS_ON_PIN,(POWER_INVERTING ? LOW : HIGH));
#endif // PS_ON_PIN > -1
                break;
            }
            case 82:    // M82
            {
                Printer::relativeExtruderCoordinateMode = false;
                break;
            }
            case 83:    // M83
            {
                Printer::relativeExtruderCoordinateMode = true;
                break;
            }
            case 84:    // M84
            {
                if(com->hasS())
                {
                    stepperInactiveTime = com->S * 1000;
                }
                else
                {
                    Commands::waitUntilEndOfAllMoves(); //M84
                    Printer::kill(true);
                }
                break;
            }
            case 85:    // M85
            {
                if(com->hasS())
                    maxInactiveTime = (long)com->S * 1000;
                else
                    maxInactiveTime = 0;
                break;
            }
            case 99:    // M99 S<time> 
            //Nibbels: 050118 Ich halte den Befehl für tendentiell gefährlich. Man sollte nicht abschalten, sondern Strom senken, oder hat das einen sinn? Vermutlich muss danach Homing und CMP deaktiviert werden!
            {
                millis_t wait = 10000L;
                if(com->hasS())
                    wait = 1000*com->S;
                if(com->hasX())
                    Printer::disableXStepper();
                if(com->hasY())
                    Printer::disableYStepper();
                if(com->hasZ())
                    Printer::disableZStepper();
                wait += HAL::timeInMilliseconds();

                while(wait-HAL::timeInMilliseconds() < 100000L)
                {
                    Printer::defaultLoopActions();
                }
                if(com->hasX())
                    Printer::enableXStepper();
                if(com->hasY())
                    Printer::enableYStepper();
                if(com->hasZ())
                    Printer::enableZStepper();
                break;
            }
            case 111:   // M111
            {
                if(com->hasS())
                {
                    Printer::debugLevel = com->S;
                }
                if(Printer::debugDryrun())   // simulate movements without printing
                {
                    Extruder::setTemperatureForAllExtruders(0, false);
                }
                break;
            }
            case 114:   // M114
            {
                printCurrentPosition();
                break;
            }
            case 115:   // M115
            {
                if( Printer::debugInfo() )
                {
                    Com::printFLN(Com::tFirmware);
                }
                reportPrinterUsage();
                break;
            }
            case 117:   // M117 - message to lcd
            {
                if(com->hasString())
                {
                    UI_STATUS_UPD_RAM(com->text);
                }
                break;
            }
            case 119:   // M119
            {
                Commands::waitUntilEndOfAllMoves(); //M119

                if( !Printer::debugInfo() )
                {
                    break;
                }

#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
                Com::printF(Com::tXMinColon);
                Com::printF(Printer::isXMinEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif // (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X

#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
                Com::printF(Com::tXMaxColon);
                Com::printF(Printer::isXMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif // (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X

#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
                Com::printF(Com::tYMinColon);
                Com::printF(Printer::isYMinEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif // (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y

#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
                Com::printF(Com::tYMaxColon);
                Com::printF(Printer::isYMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif // (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y

#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
#if FEATURE_MILLING_MODE
                if( Printer::operatingMode == OPERATING_MODE_PRINT )
                {
#endif // FEATURE_MILLING_MODE
                    // in operating mode "print", the min endstop is used
                    Com::printF(Com::tZMinColon);
                    Com::printF(Printer::isZMinEndstopHit()?Com::tHSpace:Com::tLSpace);
#if FEATURE_MILLING_MODE
                }
                /* else
                {
                    // in operating mode "mill", the min endstop is not used
                } */
#endif // FEATURE_MILLING_MODE
#endif // (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z

#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
#if MOTHERBOARD == DEVICE_TYPE_RF1000
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
#if FEATURE_MILLING_MODE
                if( Printer::operatingMode == OPERATING_MODE_MILL )
                {
                    // in operating mode "mill", the max endstop is used
                    Com::printF(Com::tZMaxColon);
                    Com::printF(Printer::isZMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
                }
                else
                {
#endif // FEATURE_MILLING_MODE
                    if( Printer::ZEndstopType == ENDSTOP_TYPE_CIRCUIT )
                    {
                        // in operating mode "print", the max endstop is used only in case both z-endstops are in a circle
                        Com::printF(Com::tZMaxColon);
                        Com::printF(Printer::isZMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
                    }
#if FEATURE_MILLING_MODE
                }
#endif // FEATURE_MILLING_MODE
#else
#if FEATURE_MILLING_MODE
                if( Printer::operatingMode == OPERATING_MODE_MILL )
                {
                    // in operating mode "mill", the max endstop is used
                    Com::printF(Com::tZMaxColon);
                    Com::printF(Printer::isZMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
                }
                else
                {
#endif // FEATURE_MILLING_MODE
                    // in operating mode "print", the max endstop is used only in case both z-endstops are in a circle
#if FEATURE_MILLING_MODE
                }
#endif // FEATURE_MILLING_MODE
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000_V2
                // the RF2000 uses the max endstop in all operating modes
                Com::printF(Com::tZMaxColon);
                Com::printF(Printer::isZMaxEndstopHit()?Com::tHSpace:Com::tLSpace);
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000_V2
#endif // (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z

                Com::println();
                break;
            }

#if BEEPER_TYPE>0
            case 120:   // M120 - Test beeper function
            {
                if(com->hasS() && com->hasP())
                    beep(com->S,com->P); // Beep test
                break;
            }
#endif // BEEPER_TYPE>0

            case 201:   // M201
            {
#if FEATURE_MILLING_MODE
                if( Printer::operatingMode == OPERATING_MODE_PRINT )
                {
#endif // FEATURE_MILLING_MODE
                    if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
                    if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
                    if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
                    if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
                    Printer::updateDerivedParameter();
#if FEATURE_MILLING_MODE
                }
#endif  // FEATURE_MILLING_MODE
                break;
            }
            case 202:   // M202
            {
#if FEATURE_MILLING_MODE
                if( Printer::operatingMode == OPERATING_MODE_PRINT )
                {
#endif // FEATURE_MILLING_MODE
                    if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = com->X;
                    if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
                    if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
                    if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
                    Printer::updateDerivedParameter();
#if FEATURE_MILLING_MODE
                }
#endif  // FEATURE_MILLING_MODE
                break;
            }

            case 203:   // M203 - Temperature monitor
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
                {
                    if(com->hasS())
                    {
                        if(com->S<NUM_EXTRUDER) manageMonitor = com->S;
#if HAVE_HEATED_BED
                        else manageMonitor=NUM_EXTRUDER; // Set 100 to heated bed
#endif // HAVE_HEATED_BED
                    }
                }
                break;
            }
            case 204:   // M204
            {
                if( isSupportedMCommand( com->M, OPERATING_MODE_PRINT ) )
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
#endif // HAVE_HEATED_BED

                    }
                    if(com->hasX()) temp->pidPGain = com->X;
                    if(com->hasY()) temp->pidIGain = com->Y;
                    if(com->hasZ()) temp->pidDGain = com->Z;
                    temp->updateTempControlVars();
                }
                break;
            }
            case 503:   // M503 (fall through)
            case 205:   // M205 - Show EEPROM settings
            {
                EEPROM::writeSettings();
                break;
            }
            case 206:   // M206 - T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
            {
                EEPROM::update(com);
                break;
            }
            case 207:   // M207 - X<XY jerk> Z<Z Jerk>
            {
                if(com->hasX())
                    Printer::maxJerk = com->X;
                if(com->hasE())
                {
                    Extruder::current->maxStartFeedrate = com->E;
                    Extruder::selectExtruderById(Extruder::current->id);
                }
                if(com->hasZ())
                    Printer::maxZJerk = com->Z;

                if( Printer::debugInfo() )
                {
                    Com::printF(Com::tJerkColon,Printer::maxJerk);
                    Com::printFLN(Com::tZJerkColon,Printer::maxZJerk);
                }
                break;
            }
            case 220:   // M220 - S<Feedrate multiplier in percent>
            {
                changeFeedrateMultiply(com->getS(100));
                break;
            }
            case 221:   // M221 - S<Extrusion flow multiplier in percent>
            {
                changeFlowrateMultiply(static_cast<float>(com->getS(100)));
                break;
            }

#if USE_ADVANCE
            case 223:   // M223 - Extruder interrupt test
            {
                if(com->hasS())
                {
                    InterruptProtectedBlock noInts; //BEGIN_INTERRUPT_PROTECTED
                    Printer::extruderStepsNeeded += com->S;
                }
                break;
            }
            case 232:   // M232
            {
                if( Printer::debugInfo() )
                {
                    Com::printF(Com::tLinearStepsColon,maxadv2);

#ifdef ENABLE_QUADRATIC_ADVANCE
                    Com::printF(Com::tQuadraticStepsColon,maxadv);
#endif // ENABLE_QUADRATIC_ADVANCE

                    Com::printFLN(Com::tCommaSpeedEqual,maxadvspeed);
                }

#ifdef ENABLE_QUADRATIC_ADVANCE
                maxadv=0;
#endif // ENABLE_QUADRATIC_ADVANCE

                maxadv2=0;
                maxadvspeed=0;
                break;
            }
            case 233:   // M233
            {
                if(com->hasY())
                    Extruder::current->advanceL = com->Y;
                Com::printF(Com::tLinearLColon,Extruder::current->advanceL);
#ifdef ENABLE_QUADRATIC_ADVANCE
                if(com->hasX())
                    Extruder::current->advanceK = com->X;
                Com::printF(Com::tQuadraticKColon,Extruder::current->advanceK);
#endif // ENABLE_QUADRATIC_ADVANCE
                Com::println();
                Printer::updateAdvanceFlags();
                break;
            }
#endif // USE_ADVANCE
#if FEATURE_CASE_LIGHT
// Idee und Teilcode und Vorarbeit von WESSIX 
            //Code schaltet X19, nicht zwingend das licht! 
                case 355: // M355  - Turn case light on/off / Turn X19 on and off.
                if(com->hasS()){
                    if(com->S == 1 || com->S == 0){
                        Printer::enableCaseLight = com->S;
                    }else{
                        Com::printFLN(PSTR("M355 Error S=0||1"));
                    }
                }else{
                        if( Printer::enableCaseLight )  Printer::enableCaseLight = 0;
                        else Printer::enableCaseLight = 1;
                }
                WRITE(CASE_LIGHT_PIN, Printer::enableCaseLight);
                Com::printFLN(PSTR("M355: X19 set to "),Printer::enableCaseLight);
                break;
// Ende Idee und Teilcode von WESSIX 
#endif // FEATURE_CASE_LIGHT
            case 400:   // M400 - Finish all moves
            {
                Commands::waitUntilEndOfAllMoves(); //M400 (normal gcode wait)
                break;
            }

#if FEATURE_MEMORY_POSITION
            case 401:   // M401 - Memory position
            {
                Printer::MemoryPosition();
                break;
            }
            case 402:   // M402 - Go to stored position
            {
                Printer::GoToMemoryPosition(com->hasX(),com->hasY(),com->hasZ(),com->hasE(),(com->hasF() ? com->F : Printer::feedrate));
                break;
            }
#endif // FEATURE_MEMORY_POSITION

            case 908:   // M908 - Control digital trimpot directly.
            {
                if( com->hasP() && com->hasS() ){
                    uint8_t current = com->S;
                    if(com->hasP() > 3 + NUM_EXTRUDER) break;
                    if(current > 150){
                       //break;
                    }else if(current < MOTOR_CURRENT_MIN){
                       //break;
                    }else{
                       setMotorCurrent((uint8_t)com->P, current); //ohne einschränkung?
                    }
                }
                break;
            }
            case 500:   // M500
            {
#if EEPROM_MODE!=0
                EEPROM::storeDataIntoEEPROM(false);
                Com::printInfoF(Com::tConfigStoredEEPROM);
#else
                if( Printer::debugErrors() )
                {
                    Com::printErrorF(Com::tNoEEPROMSupport);
                }
#endif // EEPROM_MODE!=0
                break;
            }
            case 501:   // M501
            {
#if EEPROM_MODE!=0
                EEPROM::readDataFromEEPROM();
                Extruder::selectExtruderById(Extruder::current->id);

                if( Printer::debugInfo() )
                {
                    Com::printInfoF(Com::tConfigLoadedEEPROM);
                }
#else
                if( Printer::debugErrors() )
                {
                    Com::printErrorF(Com::tNoEEPROMSupport);
                }
#endif // EEPROM_MODE!=0
                break;
            }
            case 502:   // M502
            {
                EEPROM::restoreEEPROMSettingsFromConfiguration();

#if FEATURE_AUTOMATIC_EEPROM_UPDATE
                EEPROM::storeDataIntoEEPROM(false);
#endif // FEATURE_AUTOMATIC_EEPROM_UPDATE
                EEPROM::initializeAllOperatingModes();

                UI_STATUS( UI_TEXT_RESTORE_DEFAULTS );
                showInformation( PSTR(UI_TEXT_CONFIGURATION), PSTR(UI_TEXT_RESTORE_DEFAULTS), PSTR(UI_TEXT_OK) );
                break;
            }

#if FEATURE_SERVO && MOTHERBOARD == DEVICE_TYPE_RF1000
            case 340:   // M340
            {
                if(com->hasP() && com->P<4 && com->P>=0)
                {
                    int s = 0;
                    if(com->hasS())
                        s = com->S;
                    HAL::servoMicroseconds(com->P,s);
                }
                break;
            }
#endif // FEATURE_SERVO && MOTHERBOARD == DEVICE_TYPE_RF1000

#if FEATURE_SERVO && (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000_V2)
            case 340:   // M340
            {
                if( com->hasP() )
                {
                    switch( com->P )
                    {
                        case 1:
                        {
                            if( com->hasS() )
                            {
                                int S = com->S;
                                if ( S >= 800 && S <= 2200 )
                                {
                                    Com::printFLN( PSTR( " 1. servo value [uS] =  "), S );
                                    OCR5A = 2*S;
                                }
                                else
                                {
                                    Com::printFLN( PSTR( " 1. servo value out of range ") );
                                }
                            }
                            break;
                        }
                        case 2:
                        {
                            if( com->hasS() )
                            {
                                int S = com->S;
                                if ( S >= 800 && S <= 2200 )
                                {
                                    Com::printFLN( PSTR( " 2. servo value [uS] =  "), S );
                                    OCR5B = 2*S;
                                }
                                else
                                {
                                    Com::printFLN( PSTR( " 2. servo value out of range ") );
                                }
                            }
                            break;
                        }
                        case 3:
                        {
                            if( com->hasS() )
                            {
                                int S = com->S;
                                if ( S >= 800 && S <= 2200 )
                                {
                                    Com::printFLN( PSTR( " 3. servo value [uS] =  "), S );
                                    OCR5C = 2*S;
                                }
                                else
                                {
                                    Com::printFLN( PSTR( " 3. servo value out of range ") );
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    showInvalidSyntax( com->M );
                }
                break;
            }
#endif // FEATURE_SERVO && (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000_V2)

            default:
            {
                // we may have to process RF specific commands
                processCommand( com );
                break;
            }
        }
    }
    else if(com->hasT())      // Process T code
    {
        Commands::waitUntilEndOfAllMoves(); //Tn-Code (change Extruder)
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

} // executeGCode


void Commands::emergencyStop()
{
    HAL::resetHardware(); //RF2000 / RF1000!
} // emergencyStop


void Commands::checkFreeMemory()
{
    int newfree = HAL::getFreeRam();
    if(newfree<lowestRAMValue)
    {
        lowestRAMValue = newfree;
    }
} // checkFreeMemory


void Commands::writeLowestFreeRAM()
{
    if(lowestRAMValueSend>lowestRAMValue)
    {
        lowestRAMValueSend = lowestRAMValue;

        if( true ) //Printer::debugInfo()
        {
            Com::printFLN(Com::tFreeRAM,lowestRAMValue);
        }
    }
} // writeLowestFreeRAM
