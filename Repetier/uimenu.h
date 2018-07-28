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


#if !defined(UI_MENU_H) && defined(UI_MAIN)
#define UI_MENU_H

/*
The menu configuration uses dynamic strings. These dynamic strings can contain
a placeholder for special values. During print these placeholder are exchanged
by their current value. Everything else is printed exactly as written.

A placeholder always has 3 chars. It starts with a % followed by 2 characters
defining the value. You can use any placeholder in any position, also it doesn't
always make sense.

List of placeholder:
%11 : Kill Line if Millingmode

%ew : tell me which sensor is defect.
%ec : Current extruder temperature
%eb : Current heated bed temperature
%e0..9 : Temp. of extruder 0..9
%er : Extruder relative mode
%Ec : Target temperature of current extruder
%Eb : Target temperature of heated bed
%E0-9 : Target temperature of extruder 0..9
%os : Status message
%oB : Buffer length
%om : Speed multiplier
%ov : active Speed
%op : Is double or quadstepping?
%of : flow multiplier
%oc : Connection baudrate
%o0..9 : Output level extruder 0..9 is % including %sign
%oC : Output level current extruder
%ob : Output level heated bed
%%% : The % char
%x0 : X position
%x1 : Y position
%x2 : Z position
%hx : X homed
%hy : Y homed
%hz : Z homed
%ha : all homed
%x3 : Current extruder position
%sx : State of x min endstop
%sX : State of x max endstop
%sy : State of y min endstop
%sY : State of y max endstop
%sz : State of z min endstop
%sZ : State of z max endstop
%sC : State of the z compensation
%sS : State of the z compensation -> SensiblePressure
%sM : State of the z compensation -> SensiblePressureDigits
%sm : State of the sensible pressure in eeprom
%so : State of the sensible pressure maxoffset in eeprom
%sa : State of the sensible pressure autostarter
%do : Debug echo state
%di : Debug info state
%de : Debug error state
%dd : Debug dry run state
%db : beeper state
%O0 : OPS mode = 0
%O1 : OPS mode = 1
%O2 : OPS mode = 2
%Or : OPS retract distance
%Ob : OPS backslash distance
%Od : OPS min distance
%Oa : OPS move after
%ax : X acceleration during print moves
%ay : Y acceleration during print moves
%az : Z acceleration during print moves
%aX : X acceleration during travel moves
%aY : Y acceleration during travel moves
%aZ : Z acceleration during travel moves
%aj : Max. jerk
%aJ : Max. Z-jerk
%fx : Max. feedrate x direction
%fy : Max. feedrate y direction
%fz : Max. feedrate z direction
%fe : Max. feedrate current extruder
%fX : Homing feedrate x direction
%fY : Homing feedrate y direction
%fZ : Homing feedrate z direction
%Fs : Fan speed
%Fh : Fan frequency in Hz
%Fm : Fan modulation type PWM or PDM
%FU : Fan modulation minimum = 1% FanSpeed
%FO : Fan modulation maximum = 100% FanSpeed
%PN : Printer name
%Se : Steps per mm current extruder
%S0 : Steps per mm extruder0
%S1 : Steps per mm extruder1
%Sz : Mikrometer per Z-Single_Step (Z_Axis)
%SM : Matrix has changed in Ram and is ready to Save. -> *)
%Ss : active current value of HEAT_BED_SCAN_Z_START_MM
%is : Stepper inactive time in seconds
%ip : Max. inactive time in seconds
%X0..9 : Extruder selected marker
%Xi : PID I gain
%Xp : PID P gain
%Xd : PID D gain
%Xm : PID drive min
%XM : PID drive max
%XD : PID max
%XS : Temperature Sensor
%Xw : Extruder watch period in seconds
%Xt : Description for PID autotune type in menu
%XT : Extruder wait retract temperature
%XU : Extruder wait retract unit
%Xa : Advance K value
%Xl : Advance L value
%Xb : E0 Advance L value
%Xc : E1 Advance L value
%Xg : Printer::stepsDoublerFrequency
%Xf : Extruder max. start feedrate
%XF : Extruder max. feedrate
%XA : Extruder max. acceleration
%XZ : Milling special max. acceleration
%XE : Extruder Stepper Microsteps
%Xx : XY Stepper Microsteps
%Xz : Z Stepper Microsteps

// RF specific:
%s1 : current value of the strain gauge
%Dx : scan step size x
%Dy : scan step size y
%lo : light state
%li : Light: White/Color
%ou : 230V output on/off
%ol : FET1 output on/off
%on : FET2 output on/off
%OM : operating mode
%OZ : z endstop type
%z0 : Z Offset
%zm : Z Scale
%zs : Z-Schraube korrektur mm
%zS : Z-Schraube korrektur Umdrehungen
%zF : Z-Schraube falsch
%zD : Z-Schraube richtung
%mt : miller type
%mY : menu yes
%mN : menu no
%m1 : message line 1
%m2 : message line 2
%m3 : message line 3
%m4 : message line 4
%Oa : Active Extruder
%OE : Extruder offset X [mm]
%OF : Extruder offset Y [mm]
%OS : Extruder spring displacement Z [mm]
%px : mode of the Position X menu
%py : mode of the Position Y menu
%pz : mode of the Position Z menu
%pl : g_nEmergencyPauseDigitsMin [1700/kg]
%ph : g_nEmergencyPauseDigitsMax [1700/kg]
%pL : g_nZEmergencyStopAllMin [1700/kg]
%pH : g_nZEmergencyStopAllMax [1700/kg]
%HB : active heat bed z matrix
%HO : active heat bed min z offset in um
%WP : active work part z matrix
%Z1-Z4: Page5 service intervall
%Z5-Z8: Page4 printing/milling time

%wx : current wobblefix offset in x [um] (Bauchtanz)
%wy : current wobblefix offset in y [um] (Bauchtanz)
%wz : current wobblefix offset in z [um] (Hub)
%wa : current wobblefix amplitude for X
%wb : current wobblefix amplitude for Y(x_0)
%wc : current wobblefix amplitude for Y(x_245)
%wd : current wobblefix amplitude for Z-lift
%wp : current wobblefix phase for Z-lift (Hub)
%wP : current wobblefix phase for YX-wobble (Bauchtanz)

%MX : Motorcurrent X
%MY : Motorcurrent Y
%MZ : Motorcurrent Z
%M0 : Motorcurrent T0
%M1 : Motorcurrent T1

%Ca : Caliper Active Reading
%Cm : Caliper Average
%Cs : Caliper Filament Standard
%Cc : Caliper Correction
%Cn : needed flow multi for measurement

%CU : digit flow lower limit
%CO : digit flow higher limit
%CF : digit flow flowrate
%CE : digit flow feedrate

%FH : Digit Homing to Zero ON/OFF
%FC : Digits force-bend-hotend-down Compensation Z ON/OFF

%LL : Last Layer (Direct + Queue + Extr. Zoffset)
%LC : Current Layer (Direct + Queue + Extr. Zoffset)
%LH : Layer Height
%LP : ECMP %
%Lm : g_minZCompensationSteps
%LM : g_maxZCompensationSteps
*/


/* ============= PAGES DEFINITION =============
If you are not iside a menu, the firmware displays pages with information.
Especially if you have only a small display it is convenient to have
more then one information page.
*/

/* Define your pages using dynamic strings. To define a page use
UI_PAGE4(name,row1,row2,row3,row4);
for 4 row displays and
UI_PAGE2(name,row1,row2);
for 2 row displays. You can add additional pages or change the default pages like you want.
*/

#if UI_ROWS>=4
    #if HAVE_HEATED_BED==true
        #if UI_COLS<=16
            UI_PAGE4(ui_page1,"%U1%ec/%EcB%eb/%Eb%U2","Z:%x2 mm %sC%hz",UI_TEXT_STRAIN_GAUGE,"%os")
        #else
            UI_PAGE4(ui_page1,"%U1%ec/%Ec\002 B%eb/%Eb\002%U2","Z:%x2 mm %sC%hz",UI_TEXT_STRAIN_GAUGE,"%os")
        #endif // UI_COLS<=16
    #else
        UI_PAGE4(ui_page1,UI_TEXT_PAGE_EXTRUDER,"Z:%x2 mm %sC%hz",UI_TEXT_STRAIN_GAUGE,"%os")
    #endif // HAVE_HEATED_BED==true

    UI_PAGE4(ui_page2,"X:%x0 mm %hx","Y:%x1 mm %hy","Z:%x2 mm %sC%hz","%os")

    #if NUM_EXTRUDER>1
        UI_PAGE4(ui_page3,UI_TEXT_PAGE_EXTRUDER1,UI_TEXT_PAGE_EXTRUDER2,UI_TEXT_PAGE_BED,"%os")
    #else
        UI_PAGE4(ui_page3,UI_TEXT_PAGE_EXTRUDER1,"",UI_TEXT_PAGE_BED,"%os")
    #endif // NUM_EXTRUDER>1

    #if EEPROM_MODE!=0
        UI_PAGE4(ui_page4,"%Z5","%Z6","%Z7","%Z8")

        #define UI_PRINTTIME_PAGES ,&ui_page4
        #define UI_PRINTTIME_COUNT 1
    #else
        #define UI_PRINTTIME_PAGES
        #define UI_PRINTTIME_COUNT 0
    #endif // EEPROM_MODE!=0

    #if EEPROM_MODE!=0 && FEATURE_SERVICE_INTERVAL
        UI_PAGE4(ui_page5,"%Z1","%Z2","%Z3","%Z4")
        #define UI_SERVICE_PAGES ,&ui_page5
        #define UI_SERVICE_COUNT 1
    #else
        #define UI_SERVICE_PAGES
        #define UI_SERVICE_COUNT 0
    #endif // EEPROM_MODE && FEATURE_SERVICE_INTERVAL

    #if UI_COLS<=16
        UI_PAGE4(ui_page_mod,UI_TEXT_STRAIN_GAUGE_SPEED,
                        "zO: %z0um zM: %HB",
                        "sO: %sSum%sM",
                        "Z: %x2mm %sC%hz")
    #else
        UI_PAGE4(ui_page_mod,UI_TEXT_STRAIN_GAUGE_SPEED,
                        "zO: %z0 um zMat: %HB",
                        "sO: %sS um %sM",
                        "Z: %x2 mm %sC%hz")
    #endif
    #define UI_MOD_PAGES , &ui_page_mod
    #define UI_MOD_COUNT 1

    #if UI_COLS<=16
        UI_PAGE4(ui_page_mod2,  "LayerHeight:%LH", /*12+LH(4)*/
                                "Speed:%ovmm/s",   /*10+%ov(6)*/
                                "zCMP:%Lm..%LM",   /*7+5+4*/
                                "eCMP%LP%%%@%LC"   /*4+LP%5+@1+6*/
                                )
    #else
        UI_PAGE4(ui_page_mod2,  "Layer Height: %LHmm",
                                "Speed:    %ovmm/s",
                                "zCMP: %Lm..%LM mm",   /*7+5+4*/
                                "eCMP: %LP%%%@Z:%LC" /*6+LP%5+"@Z:"3+6:*/
                                )
    #endif

    #define UI_MOD2_PAGES , &ui_page_mod2
    #define UI_MOD2_COUNT 1

    #if FEATURE_Kurt67_WOBBLE_FIX
        #if UI_COLS<=16
            UI_PAGE4(ui_page_mod_wobble,  "Off:X%wxY%wy", /*%wz*/
                                          "X:A%waum ",
                                          "Y:A%wb..%wcum",
                                          "Phase:%wP%%%Pi"/* "Z:A%wdumP%wp%%%"*/
                                    )
        #else
            UI_PAGE4(ui_page_mod_wobble,  "OffsetX:%wxY:%wyum", /* %wz*/
                                          "X:A%waum",
                                          "Y:A%wb..%wcum",
                                          "PhaseXY:%wP%%%Pi"/* "Z:A%wdum Pz:%wp%%%" */
                                    )
        #endif
        #define UI_MODWobble_PAGE , &ui_page_mod_wobble
        #define UI_MODWobble_COUNT 1
    #else
        #define UI_MODWobble_PAGE
        #define UI_MODWobble_COUNT 0
    #endif

    /* Merge pages together. Use the following pattern:
    #define UI_PAGES {&name1,&name2,&name3} */
    #define UI_PAGES {&ui_page1 UI_MOD_PAGES, &ui_page2, &ui_page3 UI_PRINTTIME_PAGES UI_SERVICE_PAGES UI_MOD2_PAGES UI_MODWobble_PAGE}

    // How many pages do you want to have. Minimum is 1.
    #define UI_NUM_PAGES 3+UI_PRINTTIME_COUNT+UI_SERVICE_COUNT+UI_MOD_COUNT+UI_MOD2_COUNT+UI_MODWobble_COUNT
#endif // UI_ROWS>=4

/* ============ MENU definition ================

The menu works the same as pages. In addion you need to define what the lines do
or where to jump to. For that reason, the menu structure needs to be entered in
reverse order. Starting from the leaves, the menu structure is defined.
*/

/*
At first define all actions available in the menu. The actions define, what the
next/previous button will do. All menu actions work the same:
next/previous changes the value
ok sets the value if not already done and goes back to previous menu.
*/


// error/warning/information message
UI_MENU_ACTION4(ui_menu_message,UI_ACTION_DUMMY,"%m1","%m2","%m3","%m4")

/** \brief Positions submenus */
#if UI_ROWS>=4
UI_MENU_ACTION4C(ui_menu_xpos,UI_ACTION_XPOSITION,UI_TEXT_ACTION_XPOSITION4)
UI_MENU_ACTION4C(ui_menu_ypos,UI_ACTION_YPOSITION,UI_TEXT_ACTION_YPOSITION4)
UI_MENU_ACTION4C(ui_menu_zpos,UI_ACTION_ZPOSITION,UI_TEXT_ACTION_ZPOSITION4)
UI_MENU_ACTION4C(ui_menu_xpos_fast,UI_ACTION_XPOSITION_FAST,UI_TEXT_ACTION_XPOSITION_FAST4)
UI_MENU_ACTION4C(ui_menu_ypos_fast,UI_ACTION_YPOSITION_FAST,UI_TEXT_ACTION_YPOSITION_FAST4)
UI_MENU_ACTION4C(ui_menu_zpos_fast,UI_ACTION_ZPOSITION_FAST,UI_TEXT_ACTION_ZPOSITION_FAST4)
#endif // UI_ROWS>=4

UI_MENU_ACTION2C(ui_menu_epos,UI_ACTION_EPOSITION,UI_TEXT_ACTION_EPOSITION_FAST2)

/* Next step is to define submenus leading to the action. */

/** \brief Positionening menu */
#if FEATURE_SET_TO_XY_ORIGIN
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_xy_origin,UI_TEXT_SET_XY_ORIGIN,UI_ACTION_SET_XY_ORIGIN,0,MENU_MODE_PRINTER)
#define SET_TO_XY_ORIGIN_ENTRY ,&ui_menu_quick_xy_origin
#define SET_TO_XY_ORIGIN_COUNT 1
#else
#define SET_TO_XY_ORIGIN_ENTRY
#define SET_TO_XY_ORIGIN_COUNT 0
#endif // FEATURE_SET_TO_XY_ORIGIN

UI_MENU_ACTIONCOMMAND(ui_menu_back,UI_TEXT_BACK,UI_ACTION_BACK)
#if UI_HAS_BACK_KEY==0
#define UI_MENU_ADDCONDBACK &ui_menu_back,
#define UI_MENU_BACKCNT 1
#else
#define UI_MENU_ADDCONDBACK
#define UI_MENU_BACKCNT 0
#endif // UI_HAS_BACK_KEY==0

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_home_all,UI_TEXT_HOME_ALL,UI_ACTION_HOME_ALL,0, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND(ui_menu_home_x,UI_TEXT_HOME_X,UI_ACTION_HOME_X)
UI_MENU_ACTIONCOMMAND(ui_menu_home_y,UI_TEXT_HOME_Y,UI_ACTION_HOME_Y)
UI_MENU_ACTIONCOMMAND(ui_menu_home_z,UI_TEXT_HOME_Z,UI_ACTION_HOME_Z)
UI_MENU_ACTIONSELECTOR(ui_menu_go_xpos,UI_TEXT_X_POSITION,ui_menu_xpos)
UI_MENU_ACTIONSELECTOR(ui_menu_go_ypos,UI_TEXT_Y_POSITION,ui_menu_ypos)
UI_MENU_ACTIONSELECTOR(ui_menu_go_zpos,UI_TEXT_Z_POSITION,ui_menu_zpos)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_go_epos,UI_TEXT_E_POSITION,ui_menu_epos,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)

//Nibbels: Das Einstellmenü für die Z-Step-Höhe:
UI_MENU_ACTIONCOMMAND(ui_menu_config_single_steps,UI_TEXT_CONFIG_SINGLE_STEPS,UI_ACTION_CONFIG_SINGLE_STEPS)
#define UI_CONFIG_SINGLE_STEPS ,&ui_menu_config_single_steps
#define UI_CONFIG_SINGLE_STEPS_CNT 1

#if !UI_SPEEDDEPENDENT_POSITIONING
UI_MENU_ACTIONSELECTOR(ui_menu_go_xfast,UI_TEXT_X_POS_FAST,ui_menu_xpos_fast)
UI_MENU_ACTIONSELECTOR(ui_menu_go_yfast,UI_TEXT_Y_POS_FAST,ui_menu_ypos_fast)
UI_MENU_ACTIONSELECTOR(ui_menu_go_zfast,UI_TEXT_Z_POS_FAST,ui_menu_zpos_fast)
#define UI_SPEED 2
#define UI_SPEED_X ,&ui_menu_go_xfast,&ui_menu_go_xpos
#define UI_SPEED_Y ,&ui_menu_go_yfast,&ui_menu_go_ypos
#define UI_SPEED_Z ,&ui_menu_go_zfast,&ui_menu_go_zpos
#else
#define UI_SPEED 1
#define UI_SPEED_X ,&ui_menu_go_xpos
#define UI_SPEED_Y ,&ui_menu_go_ypos
#define UI_SPEED_Z ,&ui_menu_go_zpos
#endif // UI_SPEEDDEPENDENT_POSITIONING

#define UI_MENU_POSITIONS {UI_MENU_ADDCONDBACK &ui_menu_home_all,&ui_menu_home_x,&ui_menu_home_y,&ui_menu_home_z UI_SPEED_X UI_SPEED_Y UI_SPEED_Z UI_CONFIG_SINGLE_STEPS SET_TO_XY_ORIGIN_ENTRY  ,&ui_menu_go_epos}
UI_MENU(ui_menu_positions,UI_MENU_POSITIONS,5 + 3 * UI_SPEED + UI_MENU_BACKCNT + UI_CONFIG_SINGLE_STEPS_CNT+SET_TO_XY_ORIGIN_COUNT)


UI_MENU_ACTIONCOMMAND(ui_menu_z_mode,UI_TEXT_Z_MODE,UI_ACTION_ZMODE)
#define UI_MENU_Z_MODE_COUNT 1
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_heat_bed_scan,UI_TEXT_DO_HEAT_BED_SCAN,UI_ACTION_RF_SCAN_HEAT_BED,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_HEAT_BED_SCAN_COUNT 1

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_heat_bed_scan_pla,UI_TEXT_HEAT_BED_SCAN_PLA,UI_ACTION_RF_SCAN_HEAT_BED_PLA,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_heat_bed_scan_abs,UI_TEXT_HEAT_BED_SCAN_ABS,UI_ACTION_RF_SCAN_HEAT_BED_ABS,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_HEAT_BED_MODE_COND   , &ui_menu_heat_bed_scan_pla, &ui_menu_heat_bed_scan_abs
#define UI_MENU_HEAT_BED_MODE_COUNT 2

#if FEATURE_HEAT_BED_Z_COMPENSATION
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_mhier_z_scan,UI_TEXT_DO_MHIER_BED_SCAN,UI_ACTION_RF_DO_MHIER_BED_SCAN,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_mhier_auto_matrix_leveling,UI_TEXT_DO_MHIER_AUTO_MATRIX_LEVELING,UI_ACTION_RF_DO_MHIER_AUTO_MATRIX_LEVELING,MENU_MODE_PRINTER,MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_HEAT_MHIER_COND  , &ui_menu_mhier_z_scan , &ui_menu_mhier_auto_matrix_leveling
#define UI_MENU_HEAT_MHIER_COUNT 2
#else
#define UI_MENU_HEAT_MHIER_COND
#define UI_MENU_HEAT_MHIER_COUNT 0
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if NUM_EXTRUDER>1 && UI_SHOW_TIPDOWN_IN_ZCONFIGURATION
UI_MENU_ACTION2C(ui_menu_extruder_offsetz2,UI_ACTION_EXTRUDER_OFFSET_Z,UI_TEXT_EXTRUDER_OFFSET_Z2)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_extruder_offset_z,UI_TEXT_EXTRUDER_OFFSET_Z,ui_menu_extruder_offsetz2, MENU_MODE_PRINTER, MENU_MODE_MILLER)
#define EXTRUDER_OFFSET_TYPE_ENTRY_Z ,&ui_menu_extruder_offset_z
#define EXTRUDER_OFFSET_TYPE_COUNT_Z 1
#else
#define EXTRUDER_OFFSET_TYPE_ENTRY_Z
#define EXTRUDER_OFFSET_TYPE_COUNT_Z 0
#endif // NUM_EXTRUDER>1

#if FEATURE_ALIGN_EXTRUDERS
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_zcomp_alignextruder, UI_TEXT_ALIGN_EXTRUDERS, UI_ACTION_ALIGN_EXTRUDERS, MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define Z_ALIGN_EXTRUDER_ENTRY ,&ui_menu_zcomp_alignextruder
#define Z_ALIGN_EXTRUDER_COUNT 1
#else
#define Z_ALIGN_EXTRUDER_ENTRY
#define Z_ALIGN_EXTRUDER_COUNT 0
#endif // FEATURE_ALIGN_EXTRUDERS

#if FEATURE_HEAT_BED_Z_COMPENSATION
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_save_active_zMatrix,UI_TEXT_DO_SAVE_ACTIVE_ZMATRIX,UI_ACTION_RF_DO_SAVE_ACTIVE_ZMATRIX,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_SAVE_ACTIVE_ZMATRIX  , &ui_menu_save_active_zMatrix
#define UI_MENU_SAVE_ACTIVE_ZMATRIX_COUNT 1
#else
#define UI_MENU_SAVE_ACTIVE_ZMATRIX
#define UI_MENU_SAVE_ACTIVE_ZMATRIX_COUNT 0
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_work_part_scan,UI_TEXT_DO_WORK_PART_SCAN,UI_ACTION_RF_SCAN_WORK_PART,0,MENU_MODE_PRINTER | MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_WORKPART_SCAN_COUNT 1

UI_MENU_ACTION4C(ui_menu_zoffset,UI_ACTION_ZOFFSET,UI_TEXT_ACTION_ZOFFSET)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_zoffset_z,UI_TEXT_Z_OFFSET,ui_menu_zoffset,MENU_MODE_PRINTER,0)
#define UI_MENU_Z_OFFSET_COUNT 1

#if FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION
UI_MENU_CHANGEACTION_FILTER(ui_menu_set_z_matrix_heat_bed,UI_TEXT_SET_Z_MATRIX_HEAT_BED,UI_ACTION_RF_SET_Z_MATRIX_HEAT_BED,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
 #define UI_MENU_SET_Z_MATRIX_CND   , &ui_menu_set_z_matrix_heat_bed
 #define UI_MENU_SET_Z_MATRIX_COUNT 1
#else
 #define UI_MENU_SET_Z_MATRIX_CND
 #define UI_MENU_SET_Z_MATRIX_COUNT 0
#endif // FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION
UI_MENU_CHANGEACTION_FILTER(ui_menu_set_z_scan_start_lift,UI_TEXT_SCAN_START_HEIGHT,UI_ACTION_RF_SCAN_START_HEIGHT, 0, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
 #define UI_MENU_SET_SCAN_START_HEIGHTCND    , &ui_menu_set_z_scan_start_lift
 #define UI_MENU_SET_SCAN_START_HEIGHT_COUNT 1
#else
 #define UI_MENU_SET_SCAN_START_HEIGHT_CND
 #define UI_MENU_SET_SCAN_START_HEIGHT_COUNT 0
#endif // FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION

/** \brief Z calibration menu */
#if FEATURE_MILLING_MODE

#if FEATURE_FIND_Z_ORIGIN
 // in case the milling mode is enabled, we need the following menus
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_set_z_origin,UI_TEXT_SET_Z_ORIGIN,UI_ACTION_SET_Z_ORIGIN,0,MENU_MODE_PRINTER)
 #define UI_MENU_SET_Z_ORIGIN_ENTRY , &ui_menu_set_z_origin
 #define UI_MENU_SET_Z_ORIGIN_COUNT 1
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_find_z_origin,UI_TEXT_FIND_Z_ORIGIN,UI_ACTION_RF_FIND_Z_ORIGIN,0,MENU_MODE_PRINTER)
 #define UI_MENU_FIND_Z_ORIGIN_COND  ,&ui_menu_find_z_origin
 #define UI_MENU_FIND_Z_ORIGIN_COUNT 1
#else
 #define UI_MENU_SET_Z_ORIGIN_ENTRY
 #define UI_MENU_SET_Z_ORIGIN_COUNT 0
 #define UI_MENU_FIND_Z_ORIGIN_COND
 #define UI_MENU_FIND_Z_ORIGIN_COUNT 0
#endif // FEATURE_FIND_Z_ORIGIN

UI_MENU_CHANGEACTION_FILTER(ui_menu_set_z_matrix_work_part,UI_TEXT_SET_Z_MATRIX_WORK_PART,UI_ACTION_RF_SET_Z_MATRIX_WORK_PART,0,MENU_MODE_PRINTER)
#define UI_MENU_SET_Z_MATRIX_MILLING_COUNT 1
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_set_xy_start,UI_TEXT_SET_SCAN_XY_START,UI_ACTION_RF_SET_SCAN_XY_START,0,MENU_MODE_PRINTER)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_set_xy_end,UI_TEXT_SET_SCAN_XY_END,UI_ACTION_RF_SET_SCAN_XY_END,0,MENU_MODE_PRINTER)
UI_MENU_CHANGEACTION_FILTER(ui_menu_set_delta_x,UI_TEXT_SET_SCAN_DELTA_X,UI_ACTION_RF_SET_SCAN_DELTA_X,0,MENU_MODE_PRINTER)
UI_MENU_CHANGEACTION_FILTER(ui_menu_set_delta_y,UI_TEXT_SET_SCAN_DELTA_Y,UI_ACTION_RF_SET_SCAN_DELTA_Y,0,MENU_MODE_PRINTER)
#define UI_MENU_SET_YX_MILLING_COND  ,&ui_menu_set_xy_start, &ui_menu_set_xy_end, &ui_menu_set_delta_x, &ui_menu_set_delta_y
#define UI_MENU_SET_YX_MILLING_COUNT 4

#define UI_MENU_Z {UI_MENU_ADDCONDBACK &ui_menu_heat_bed_scan UI_MENU_HEAT_BED_MODE_COND UI_MENU_HEAT_MHIER_COND, &ui_menu_zoffset_z EXTRUDER_OFFSET_TYPE_ENTRY_Z Z_ALIGN_EXTRUDER_ENTRY, &ui_menu_z_mode, &ui_menu_work_part_scan UI_MENU_SET_Z_ORIGIN_ENTRY UI_MENU_FIND_Z_ORIGIN_COND UI_MENU_SET_Z_MATRIX_CND UI_MENU_SET_SCAN_START_HEIGHTCND UI_MENU_SAVE_ACTIVE_ZMATRIX, &ui_menu_set_z_matrix_work_part UI_MENU_SET_YX_MILLING_COND}
UI_MENU(ui_menu_z,UI_MENU_Z,UI_MENU_BACKCNT + UI_MENU_HEAT_BED_SCAN_COUNT + UI_MENU_HEAT_BED_MODE_COUNT + UI_MENU_HEAT_MHIER_COUNT + UI_MENU_Z_OFFSET_COUNT + Z_ALIGN_EXTRUDER_COUNT + EXTRUDER_OFFSET_TYPE_COUNT_Z + UI_MENU_Z_MODE_COUNT + UI_MENU_WORKPART_SCAN_COUNT + UI_MENU_SET_Z_ORIGIN_COUNT + UI_MENU_FIND_Z_ORIGIN_COUNT + UI_MENU_SET_Z_MATRIX_COUNT + UI_MENU_SET_SCAN_START_HEIGHT_COUNT + UI_MENU_SAVE_ACTIVE_ZMATRIX_COUNT + UI_MENU_SET_Z_MATRIX_MILLING_COUNT + UI_MENU_SET_YX_MILLING_COUNT )

#else // FEATURE_MILLING_MODE

#define UI_MENU_Z {UI_MENU_ADDCONDBACK &ui_menu_heat_bed_scan UI_MENU_HEAT_BED_MODE_COND UI_MENU_HEAT_MHIER_COND ,&ui_menu_zoffset_z EXTRUDER_OFFSET_TYPE_ENTRY_Z, &ui_menu_z_mode UI_MENU_SET_Z_MATRIX_CND UI_MENU_SET_SCAN_START_HEIGHTCND UI_MENU_SAVE_ACTIVE_ZMATRIX}
UI_MENU(ui_menu_z,UI_MENU_Z,UI_MENU_BACKCNT + UI_MENU_HEAT_BED_SCAN_COUNT + UI_MENU_HEAT_BED_MODE_COUNT + UI_MENU_HEAT_MHIER_COUNT + UI_MENU_Z_OFFSET_COUNT + EXTRUDER_OFFSET_TYPE_COUNT_Z + UI_MENU_Z_MODE_COUNT +   UI_MENU_SET_Z_MATRIX_COUNT + UI_MENU_SET_SCAN_START_HEIGHT_COUNT + UI_MENU_SAVE_ACTIVE_ZMATRIX_COUNT )

#endif // FEATURE_MILLING_MODE

/** \brief Extruder menu */
UI_MENU_CHANGEACTION_FILTER(ui_menu_quick_flowmultiply,UI_TEXT_FLOW_MULTIPLY,UI_ACTION_FLOWRATE_MULTIPLY,MENU_MODE_PRINTER,0)

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_preheat_pla,UI_TEXT_PREHEAT_PLA,UI_ACTION_PREHEAT_PLA,MENU_MODE_PRINTER,0)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_preheat_abs,UI_TEXT_PREHEAT_ABS,UI_ACTION_PREHEAT_ABS,MENU_MODE_PRINTER,0)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_cooldown,UI_TEXT_COOLDOWN,UI_ACTION_COOLDOWN,MENU_MODE_PRINTER,0)

UI_MENU_CHANGEACTION(ui_menu_ext_temp0,UI_TEXT_EXTR0_TEMP,UI_ACTION_EXTRUDER0_TEMP)
UI_MENU_CHANGEACTION(ui_menu_ext_temp1,UI_TEXT_EXTR1_TEMP,UI_ACTION_EXTRUDER1_TEMP)
UI_MENU_CHANGEACTION(ui_menu_bed_temp, UI_TEXT_BED_TEMP,UI_ACTION_HEATED_BED_TEMP)

UI_MENU_ACTIONCOMMAND(ui_menu_active_extruder,UI_TEXT_ACTIVE_EXTRUDER,UI_ACTION_ACTIVE_EXTRUDER)

//UI_MENU_ACTIONCOMMAND(ui_menu_ext_sel1,UI_TEXT_EXTR1_SELECT,UI_ACTION_SELECT_EXTRUDER1);
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off0,UI_TEXT_EXTR0_OFF,UI_ACTION_EXTRUDER0_OFF)
UI_MENU_ACTIONCOMMAND(ui_menu_ext_off1,UI_TEXT_EXTR1_OFF,UI_ACTION_EXTRUDER1_OFF)
UI_MENU_ACTIONCOMMAND(ui_menu_bed_off, UI_TEXT_BED_OFF  ,UI_ACTION_HEATED_BED_OFF)

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_ext_origin,UI_TEXT_SET_E_ORIGIN,UI_ACTION_SET_E_ORIGIN,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED )

/** \brief Extruder->Load Filament-> */
/** \brief Extruder->Unload Filament-> */

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_extruder_mount_filament1,UI_TEXT_MOUNT_FILAMENT_SOFT,UI_ACTION_MOUNT_FILAMENT_SOFT,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_extruder_mount_filament2,UI_TEXT_MOUNT_FILAMENT_HARD,UI_ACTION_MOUNT_FILAMENT_HARD,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_LOAD {UI_MENU_ADDCONDBACK &ui_menu_extruder_mount_filament1, &ui_menu_extruder_mount_filament2 }
UI_MENU(ui_menu_extruder_mount,UI_MENU_LOAD,UI_MENU_BACKCNT+2)
UI_MENU_SUBMENU(ui_submenu_extruder_mount, UI_TEXT_MOUNT_FILAMENT, ui_menu_extruder_mount)

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_extruder_unmount_filament1,UI_TEXT_UNMOUNT_FILAMENT_SOFT,UI_ACTION_UNMOUNT_FILAMENT_SOFT,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_extruder_unmount_filament2,UI_TEXT_UNMOUNT_FILAMENT_HARD,UI_ACTION_UNMOUNT_FILAMENT_HARD,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_UNLOAD {UI_MENU_ADDCONDBACK &ui_menu_extruder_unmount_filament1, &ui_menu_extruder_unmount_filament2 }
UI_MENU(ui_menu_extruder_unmount,UI_MENU_UNLOAD,UI_MENU_BACKCNT+2)
UI_MENU_SUBMENU(ui_submenu_extruder_unmount, UI_TEXT_UNMOUNT_FILAMENT, ui_menu_extruder_unmount)

#define UI_MENU_EXTRUDER_MOUNT_COND &ui_submenu_extruder_mount, &ui_submenu_extruder_unmount,
#define UI_MENU_EXTRUDER_MOUNT_COUNT 2

/** End load unload */

#if NUM_EXTRUDER>1
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_temp1,&ui_menu_ext_off0,&ui_menu_ext_off1,&ui_menu_bed_off,&ui_menu_active_extruder,
#define UI_MENU_EXTCNT 6
#else
#define UI_MENU_EXTCOND &ui_menu_ext_temp0,&ui_menu_ext_off0,&ui_menu_bed_off,
#define UI_MENU_EXTCNT 3
#endif // NUM_EXTRUDER>1

#if HAVE_HEATED_BED==true
#define UI_MENU_BEDCOND &ui_menu_bed_temp,
#define UI_MENU_BEDCNT 1
#else
#define UI_MENU_BEDCOND
#define UI_MENU_BEDCNT 0
#endif // HAVE_HEATED_BED==true

#define UI_MENU_EXTRUDER {UI_MENU_ADDCONDBACK UI_MENU_BEDCOND UI_MENU_EXTCOND &ui_menu_go_epos, &ui_menu_quick_preheat_pla, &ui_menu_quick_preheat_abs, &ui_menu_quick_cooldown, UI_MENU_EXTRUDER_MOUNT_COND &ui_menu_quick_flowmultiply, &ui_menu_ext_origin}
UI_MENU(ui_menu_extruder,UI_MENU_EXTRUDER,UI_MENU_BACKCNT+UI_MENU_BEDCNT+UI_MENU_EXTCNT+4+1+UI_MENU_EXTRUDER_MOUNT_COUNT+1)

/** \brief Caliper Filament Reader menu */
#if FEATURE_READ_CALIPER
 UI_MENU_ACTIONCOMMAND( ui_menu_cal_directread,UI_TEXT_CAL_DIRECT_READ,UI_ACTION_CAL_RESET)
 UI_MENU_CHANGEACTION ( ui_menu_cal_correct,   UI_TEXT_CAL_CORRECT,    UI_ACTION_CAL_CORRECT)
 UI_MENU_ACTIONCOMMAND( ui_menu_cal_showreset, UI_TEXT_CAL_SHOW,       UI_ACTION_CAL_SET)
 UI_MENU_CHANGEACTION ( ui_menu_cal_standard,  UI_TEXT_CAL_STANDARD,   UI_ACTION_CAL_STANDARD)

 #define UI_MENU_CONF_CALIPER {UI_MENU_ADDCONDBACK &ui_menu_cal_directread, &ui_menu_cal_correct, &ui_menu_cal_showreset, &ui_menu_cal_standard}
 UI_MENU(ui_menu_settings_cal,UI_MENU_CONF_CALIPER,UI_MENU_BACKCNT+4)

 UI_MENU_SUBMENU_FILTER(ui_menu_conf_cal, UI_TEXT_CAL_CONF_MENU, ui_menu_settings_cal, MENU_MODE_PRINTER,0)
 #define UI_MENU_CONFIGURATION_CAL_COND &ui_menu_conf_cal,
 #define UI_MENU_CONFIGURATION_CAL_COUNT 1

#else //FEATURE_READ_CALIPER
 #define UI_MENU_CONFIGURATION_CAL_COND
 #define UI_MENU_CONFIGURATION_CAL_COUNT 0
#endif //FEATURE_READ_CALIPER

/** \brief Quick menu */
#if PS_ON_PIN>=0
UI_MENU_ACTIONCOMMAND(ui_menu_quick_power,UI_TEXT_POWER,UI_ACTION_POWER)
#define MENU_PSON_COUNT 1
#define MENU_PSON_ENTRY ,&ui_menu_quick_power
#else
#define MENU_PSON_COUNT 0
#define MENU_PSON_ENTRY
#endif // PS_ON_PIN>=0

UI_MENU_ACTION4C(ui_menu_quick_stop_print_ack, UI_ACTION_STOP_ACK, UI_TEXT_STOP_PRINT_ACK)
//das stoppen kann auch über den repetierserver laufen, wenn man dem "RequestStop:" schickt, also parallel machen!! und für normale drucke das menü auch zulassen:
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_quick_stop_print, UI_TEXT_STOP_PRINT, ui_menu_quick_stop_print_ack, MENU_MODE_SD_PRINTING | MENU_MODE_PRINTING, MENU_MODE_MILLER)
UI_MENU_ACTION4C(ui_menu_quick_stop_mill_ack, UI_ACTION_STOP_ACK, UI_TEXT_STOP_MILL_ACK)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_quick_stop_mill, UI_TEXT_STOP_MILL, ui_menu_quick_stop_mill_ack, MENU_MODE_SD_PRINTING | MENU_MODE_PRINTING, MENU_MODE_PRINTER)


UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_stopstepper,UI_TEXT_DISABLE_STEPPER,UI_ACTION_DISABLE_STEPPER,0, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)

UI_MENU_CHANGEACTION(ui_menu_quick_speedmultiply,UI_TEXT_SPEED_MULTIPLY,UI_ACTION_FEEDRATE_MULTIPLY)

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_quick_output_object,UI_TEXT_OUTPUT_OBJECT,UI_ACTION_RF_OUTPUT_OBJECT,0, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define OUTPUT_OBJECT_COUNT 1
#define OUTPUT_OBJECT_ENTRY ,&ui_menu_quick_output_object

#if FEATURE_PARK
UI_MENU_ACTIONCOMMAND(ui_menu_quick_park,UI_TEXT_PARK_HEAT_BED,UI_ACTION_RF_PARK)
#define PARK_COUNT 1
#define PARK_ENTRY ,&ui_menu_quick_park
#else
#define PARK_COUNT 0
#define PARK_ENTRY
#endif // FEATURE_PARK

#if FEATURE_230V_OUTPUT
UI_MENU_ACTIONCOMMAND(ui_menu_toggle_230V_output,UI_TEXT_230V_OUTPUT,UI_ACTION_230V_OUTPUT)
#define OUTPUT_230V_ENTRY ,&ui_menu_toggle_230V_output
#define OUTPUT_230V_COUNT 1
#else
#define OUTPUT_230V_ENTRY
#define OUTPUT_230V_COUNT 0
#endif // FEATURE_230V_OUTPUT

#if FEATURE_CASE_LIGHT //Output X19 @RF2000/RF1000
UI_MENU_ACTIONCOMMAND(ui_menu_light_mode,UI_TEXT_LIGHTS_ONOFF,UI_ACTION_LIGHTS_ONOFF)
#define LIGHT_MODE_ENTRY ,&ui_menu_light_mode
#define LIGHT_MODE_COUNT 1
#else
#define LIGHT_MODE_ENTRY
#define LIGHT_MODE_COUNT 0
#endif // FEATURE_CASE_LIGHT

#if FEATURE_24V_FET_OUTPUTS
UI_MENU_ACTIONCOMMAND(ui_menu_toggle_FET1_output,UI_TEXT_FET1_OUTPUT,UI_ACTION_FET1_OUTPUT)
#define OUTPUT_FET1_ENTRY ,&ui_menu_toggle_FET1_output
#define OUTPUT_FET1_COUNT 1
#else
#define OUTPUT_FET1_ENTRY
#define OUTPUT_FET1_COUNT 0
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_24V_FET_OUTPUTS
UI_MENU_ACTIONCOMMAND(ui_menu_toggle_FET2_output,UI_TEXT_FET2_OUTPUT,UI_ACTION_FET2_OUTPUT)
#define OUTPUT_FET2_ENTRY ,&ui_menu_toggle_FET2_output
#define OUTPUT_FET2_COUNT 1
#else
#define OUTPUT_FET2_ENTRY
#define OUTPUT_FET2_COUNT 0
#endif // FEATURE_24V_FET_OUTPUTS

UI_MENU_ACTION4C(ui_menu_quick_reset_ack,UI_ACTION_RF_RESET_ACK,UI_TEXT_RESET_ACK)
UI_MENU_ACTIONSELECTOR(ui_menu_quick_reset,UI_TEXT_RESET,ui_menu_quick_reset_ack)
#define RESET_VIA_MENU_COUNT 1
#define RESET_VIA_MENU_ENTRY ,&ui_menu_quick_reset

//das LIGHT_MODE_ENTRY ist der X19! Könnte verwirrend sein... RGB-Light gibts in Configuration->General
#define UI_MENU_QUICK {UI_MENU_ADDCONDBACK &ui_menu_quick_stop_print, &ui_menu_home_all, &ui_menu_quick_stopstepper, &ui_menu_quick_stop_mill OUTPUT_OBJECT_ENTRY ,&ui_menu_quick_speedmultiply, UI_MENU_CONFIGURATION_CAL_COND &ui_menu_quick_flowmultiply  PARK_ENTRY OUTPUT_230V_ENTRY LIGHT_MODE_ENTRY OUTPUT_FET1_ENTRY OUTPUT_FET2_ENTRY RESET_VIA_MENU_ENTRY MENU_PSON_ENTRY }
UI_MENU(ui_menu_quick,UI_MENU_QUICK,6+UI_MENU_BACKCNT+MENU_PSON_COUNT+OUTPUT_OBJECT_COUNT+UI_MENU_CONFIGURATION_CAL_COUNT+PARK_COUNT+OUTPUT_230V_COUNT+LIGHT_MODE_COUNT+OUTPUT_FET1_COUNT+OUTPUT_FET2_COUNT+RESET_VIA_MENU_COUNT)

/** \brief Fan menu */

#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
UI_MENU_CHANGEACTION(ui_menu_fan_fanspeed, UI_TEXT_ACTION_FANSPEED,UI_ACTION_FANSPEED)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_fan_off,UI_TEXT_FAN_OFF,UI_ACTION_FAN_OFF,MENU_MODE_FAN_RUNNING,0)
UI_MENU_ACTIONCOMMAND(ui_menu_fan_25,UI_TEXT_FAN_25,UI_ACTION_FAN_25)
UI_MENU_ACTIONCOMMAND(ui_menu_fan_50,UI_TEXT_FAN_50,UI_ACTION_FAN_50)
UI_MENU_ACTIONCOMMAND(ui_menu_fan_75,UI_TEXT_FAN_75,UI_ACTION_FAN_75)
UI_MENU_ACTIONCOMMAND(ui_menu_fan_full,UI_TEXT_FAN_FULL,UI_ACTION_FAN_FULL)

//fan settings for part fan and frequency setting for secondary coolers, but they are factor 4 faster in PWM.
UI_MENU_CHANGEACTION_FILTER(ui_menu_fan_hz,      UI_TEXT_FAN_HZ,             UI_ACTION_FAN_HZ,    0, MENU_MODE_FAN_MODE_PDM)
UI_MENU_ACTIONCOMMAND(      ui_menu_fan_mode,    UI_TEXT_FAN_MODE,           UI_ACTION_FAN_MODE)
UI_MENU_CHANGEACTION(       ui_menu_fan_pwm_min, UI_TEXT_FAN_PART_FAN_PWM_MIN, UI_ACTION_FAN_PART_FAN_PWM_MIN)
UI_MENU_CHANGEACTION(       ui_menu_fan_pwm_max, UI_TEXT_FAN_PART_FAN_PWM_MAX, UI_ACTION_FAN_PART_FAN_PWM_MAX)
#define UI_MENU_FANHZMODE_CNT 2

#define UI_MENU_FAN {UI_MENU_ADDCONDBACK &ui_menu_fan_fanspeed,&ui_menu_fan_off,&ui_menu_fan_25,&ui_menu_fan_50,&ui_menu_fan_75,&ui_menu_fan_full}
UI_MENU(ui_menu_fan,UI_MENU_FAN,UI_MENU_BACKCNT+6)
UI_MENU_SUBMENU_FILTER(ui_menu_fan_sub,UI_TEXT_FANSPEED,ui_menu_fan,MENU_MODE_PRINTER,0)
#define UI_MENU_FAN_COND &ui_menu_fan_sub,
#define UI_MENU_FAN_CNT 1
#else
#define UI_MENU_FAN_COND
#define UI_MENU_FAN_CNT 0
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

#define UI_MENU_CONF_FAN {UI_MENU_ADDCONDBACK &ui_menu_fan_mode, &ui_menu_fan_hz, &ui_menu_fan_pwm_min, &ui_menu_fan_pwm_max}
UI_MENU(ui_menu_settings_fan,UI_MENU_CONF_FAN,UI_MENU_BACKCNT+1+1+1+1)

/** \brief Configuration->Fan menu */
UI_MENU_SUBMENU_FILTER(ui_menu_conf_fan, UI_TEXT_FAN_CONF_MENU, ui_menu_settings_fan, MENU_MODE_PRINTER,0)
#define UI_MENU_CONFIGURATION_FAN_COND &ui_menu_conf_fan,
#define UI_MENU_CONFIGURATION_FAN_COUNT 1

/** \brief SD card menu */
#if SDSUPPORT
 /* Things that are needed for RF1000 mount/unmount and RF2000 automount */
 #define UI_MENU_SD_FILESELECTOR {&ui_menu_back}
 UI_MENU_FILESELECT(ui_menu_sd_fileselector,UI_MENU_SD_FILESELECTOR,1)
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_print_file,     UI_TEXT_PRINT_FILE,     UI_ACTION_SD_PRINT, MENU_MODE_SD_MOUNTED, MENU_MODE_MILLER | MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_mill_file,     UI_TEXT_MILL_FILE,     UI_ACTION_SD_PRINT, MENU_MODE_SD_MOUNTED, MENU_MODE_PRINTER | MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#endif // SDSUPPORT

#if SDSUPPORT && (MOTHERBOARD == DEVICE_TYPE_RF1000)
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_pause_print,    UI_TEXT_PAUSE_PRINT,    UI_ACTION_SD_PAUSE, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING, MENU_MODE_PAUSED | MENU_MODE_MILLER)
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_continue_print, UI_TEXT_CONTINUE_PRINT, UI_ACTION_SD_CONTINUE, MENU_MODE_PAUSED, MENU_MODE_MILLER)
 UI_MENU_ACTION4C(ui_menu_sd_stop_print_ack, UI_ACTION_STOP_ACK, UI_TEXT_STOP_PRINT_ACK)
 UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_sd_stop_print, UI_TEXT_STOP_PRINT, ui_menu_sd_stop_print_ack, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING, MENU_MODE_MILLER)

 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_pause_mill,    UI_TEXT_PAUSE_MILL,    UI_ACTION_SD_PAUSE, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING, MENU_MODE_PAUSED | MENU_MODE_PRINTER)
 UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_continue_mill, UI_TEXT_CONTINUE_MILL, UI_ACTION_SD_CONTINUE, MENU_MODE_PAUSED, MENU_MODE_PRINTER)
 UI_MENU_ACTION4C(ui_menu_sd_stop_mill_ack, UI_ACTION_STOP_ACK, UI_TEXT_STOP_MILL_ACK)
 UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_sd_stop_mill, UI_TEXT_STOP_MILL, ui_menu_sd_stop_mill_ack, MENU_MODE_SD_PRINTING, MENU_MODE_PRINTER)

 #if defined(SDCARDDETECT) && SDCARDDETECT>-1
  #define UI_MOUNT_CNT 0
  #define UI_MOUNT_CMD
 #else
  UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_unmount,UI_TEXT_UNMOUNT_CARD,UI_ACTION_SD_UNMOUNT,MENU_MODE_SD_MOUNTED, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
  UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_mount,UI_TEXT_MOUNT_CARD,UI_ACTION_SD_MOUNT,0,MENU_MODE_SD_MOUNTED)
  #define UI_MOUNT_CNT 2
  #define UI_MOUNT_CMD ,&ui_menu_sd_mount,&ui_menu_sd_unmount
 #endif // (SDCARDDETECT) && SDCARDDETECT>-1
//UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_delete,UI_TEXT_DELETE_FILE,UI_ACTION_SD_DELETE,MENU_MODE_SD_MOUNTED, MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
 #define UI_MENU_SD {UI_MENU_ADDCONDBACK &ui_menu_sd_print_file,&ui_menu_sd_mill_file,&ui_menu_sd_pause_print,&ui_menu_sd_pause_mill,&ui_menu_sd_stop_print,&ui_menu_sd_stop_mill,&ui_menu_sd_continue_print,&ui_menu_sd_continue_mill /*,&ui_menu_sd_delete*/ UI_MOUNT_CMD}
 UI_MENU(ui_menu_sd,UI_MENU_SD,UI_MENU_BACKCNT+8/*+1*/+UI_MOUNT_CNT)
 UI_MENU_SUBMENU_FILTER(ui_menu_sd_sub,UI_TEXT_SD_CARD,ui_menu_sd,0, MENU_MODE_PRINTING)

 #define UI_MENU_SD_COND &ui_menu_sd_sub,
 #define UI_MENU_SD_CNT 1
#else
 #define UI_MENU_SD_COND
 #define UI_MENU_SD_CNT 0
#endif // SDSUPPORT

#if SHOW_DEBUGGING_MENU
/** \brief Debugging menu */
UI_MENU_ACTIONCOMMAND(ui_menu_debug_echo,   UI_TEXT_DBG_ECHO,   UI_ACTION_DEBUG_ECHO)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_info,   UI_TEXT_DBG_INFO,   UI_ACTION_DEBUG_INFO)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_error,  UI_TEXT_DBG_ERROR,  UI_ACTION_DEBUG_ERROR)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_dryrun, UI_TEXT_DBG_DRYRUN, UI_ACTION_DEBUG_DRYRUN)

#define UI_MENU_DEBUGGING {UI_MENU_ADDCONDBACK &ui_menu_debug_echo,&ui_menu_debug_info,&ui_menu_debug_error,&ui_menu_debug_dryrun}
UI_MENU(ui_menu_debugging,UI_MENU_DEBUGGING,4+UI_MENU_BACKCNT)
#endif // SHOW_DEBUGGING_MENU

/** \brief Acceleration settings */
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_printx,  UI_TEXT_PRINT_X, UI_ACTION_PRINT_ACCEL_X, 0, MENU_MODE_MILLER)
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_printy,  UI_TEXT_PRINT_Y, UI_ACTION_PRINT_ACCEL_Y, 0, MENU_MODE_MILLER)
UI_MENU_CHANGEACTION_FILTER(ui_menu_accel_printz,  UI_TEXT_PRINT_Z, UI_ACTION_PRINT_ACCEL_Z, 0, MENU_MODE_MILLER)
UI_MENU_CHANGEACTION(ui_menu_accel_travelx, UI_TEXT_MOVE_X,  UI_ACTION_MOVE_ACCEL_X)
UI_MENU_CHANGEACTION(ui_menu_accel_travely, UI_TEXT_MOVE_Y,  UI_ACTION_MOVE_ACCEL_Y)
UI_MENU_CHANGEACTION(ui_menu_accel_travelz, UI_TEXT_MOVE_Z,  UI_ACTION_MOVE_ACCEL_Z)
UI_MENU_CHANGEACTION(ui_menu_accel_jerk,    UI_TEXT_JERK,    UI_ACTION_MAX_JERK)
UI_MENU_CHANGEACTION(ui_menu_accel_zjerk,   UI_TEXT_ZJERK,   UI_ACTION_MAX_ZJERK)
#define UI_MENU_ACCEL {UI_MENU_ADDCONDBACK &ui_menu_accel_printx, &ui_menu_accel_printy, &ui_menu_accel_printz, &ui_menu_accel_travelx, &ui_menu_accel_travely, &ui_menu_accel_travelz, &ui_menu_accel_jerk, &ui_menu_accel_zjerk}
UI_MENU(ui_menu_accel,UI_MENU_ACCEL,8+UI_MENU_BACKCNT)

/** \brief Feedrates */
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxx,  UI_TEXT_FEED_MAX_X,  UI_ACTION_MAX_FEEDRATE_X)
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxy,  UI_TEXT_FEED_MAX_Y,  UI_ACTION_MAX_FEEDRATE_Y)
UI_MENU_CHANGEACTION(ui_menu_feedrate_maxz,  UI_TEXT_FEED_MAX_Z,  UI_ACTION_MAX_FEEDRATE_Z)
UI_MENU_CHANGEACTION(ui_menu_feedrate_homex, UI_TEXT_FEED_HOME_X, UI_ACTION_HOMING_FEEDRATE_X)
UI_MENU_CHANGEACTION(ui_menu_feedrate_homey, UI_TEXT_FEED_HOME_Y, UI_ACTION_HOMING_FEEDRATE_Y)
UI_MENU_CHANGEACTION(ui_menu_feedrate_homez, UI_TEXT_FEED_HOME_Z, UI_ACTION_HOMING_FEEDRATE_Z)
#define UI_MENU_FEEDRATE {UI_MENU_ADDCONDBACK &ui_menu_quick_speedmultiply, &ui_menu_feedrate_maxx, &ui_menu_feedrate_maxy, &ui_menu_feedrate_maxz, &ui_menu_feedrate_homex, &ui_menu_feedrate_homey, &ui_menu_feedrate_homez}
UI_MENU(ui_menu_feedrate,UI_MENU_FEEDRATE,6+1+UI_MENU_BACKCNT)

/** \brief General configuration settings */
/** \brief Configuration->General menu */

UI_MENU_ACTION2C(ui_menu_stepper2,UI_ACTION_STEPPER_INACTIVE,UI_TEXT_STEPPER_OFF2)
UI_MENU_ACTION2C(ui_menu_maxinactive2,UI_ACTION_MAX_INACTIVE,UI_TEXT_ALL_OFF2)
UI_MENU_CHANGEACTION(ui_menu_general_baud,UI_TEXT_BAUDRATE,UI_ACTION_BAUDRATE)
UI_MENU_ACTIONSELECTOR(ui_menu_general_stepper_inactive,UI_TEXT_STEPPER_OFF,ui_menu_stepper2)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_general_max_inactive,UI_TEXT_ALL_OFF,ui_menu_maxinactive2,MENU_MODE_PRINTER,0)

#if FEATURE_BEEPER
UI_MENU_ACTIONCOMMAND(ui_menu_general_beeper,UI_TEXT_BEEPER,UI_ACTION_BEEPER)
#define BEEPER_MODE_COUNT   1
#define BEEPER_MODE_ENTRY   ,&ui_menu_general_beeper
#else
#define BEEPER_MODE_COUNT   0
#define BEEPER_MODE_ENTRY
#endif // FEATURE_BEEPER

#if FEATURE_RGB_LIGHT_EFFECTS
UI_MENU_CHANGEACTION(ui_menu_toggle_rgb_light,UI_TEXT_RGB_LIGHT_MODE,UI_ACTION_RGB_LIGHT_MODE)
#define RGB_LIGHT_ENTRY ,&ui_menu_toggle_rgb_light
#define RGB_LIGHT_COUNT 1
#else
#define RGB_LIGHT_ENTRY
#define RGB_LIGHT_COUNT 0
#endif // FEATURE_RGB_LIGHT_EFFECTS

#if FEATURE_MILLING_MODE
UI_MENU_ACTIONCOMMAND(ui_menu_operating_mode,UI_TEXT_OPERATING_MODE,UI_ACTION_OPERATING_MODE)
#define OPERATING_MODE_ENTRY ,&ui_menu_operating_mode
#define OPERATING_MODE_COUNT 1

#if MOTHERBOARD == DEVICE_TYPE_RF1000
UI_MENU_ACTIONCOMMAND(ui_menu_z_endstop_type,UI_TEXT_Z_ENDSTOP_TYPE,UI_ACTION_Z_ENDSTOP_TYPE)
#define Z_ENDSTOP_TYPE_ENTRY ,&ui_menu_z_endstop_type
#define Z_ENDSTOP_TYPE_COUNT 1
#else
#define Z_ENDSTOP_TYPE_ENTRY
#define Z_ENDSTOP_TYPE_COUNT 0
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000

#if FEATURE_CONFIGURABLE_MILLER_TYPE
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_miller_type,UI_TEXT_MILLER_TYPE,UI_ACTION_MILLER_TYPE,MENU_MODE_MILLER,0)
#define MILLER_TYPE_ENTRY ,&ui_menu_miller_type
#define MILLER_TYPE_COUNT 1
#else
#define MILLER_TYPE_ENTRY
#define MILLER_TYPE_COUNT 0
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

#else

#define OPERATING_MODE_ENTRY
#define OPERATING_MODE_COUNT 0
#define Z_ENDSTOP_TYPE_ENTRY
#define Z_ENDSTOP_TYPE_COUNT 0
#define MILLER_TYPE_ENTRY
#define MILLER_TYPE_COUNT 0

#endif // FEATURE_MILLING_MODE

#if NUM_EXTRUDER>1
UI_MENU_ACTION2C(ui_menu_extruder_offsetx2,UI_ACTION_EXTRUDER_OFFSET_X,UI_TEXT_EXTRUDER_OFFSET_X2)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_extruder_offset_x,UI_TEXT_EXTRUDER_OFFSET_X,ui_menu_extruder_offsetx2, MENU_MODE_PRINTER, MENU_MODE_MILLER)
UI_MENU_ACTION2C(ui_menu_extruder_offsety2,UI_ACTION_EXTRUDER_OFFSET_Y,UI_TEXT_EXTRUDER_OFFSET_Y2)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_extruder_offset_y,UI_TEXT_EXTRUDER_OFFSET_Y,ui_menu_extruder_offsety2, MENU_MODE_PRINTER, MENU_MODE_MILLER)
#define EXTRUDER_OFFSET_TYPE_ENTRY_XY ,&ui_menu_extruder_offset_x ,&ui_menu_extruder_offset_y
#define EXTRUDER_OFFSET_TYPE_COUNT_XY 2
#else
#define EXTRUDER_OFFSET_TYPE_ENTRY_XY
#define EXTRUDER_OFFSET_TYPE_COUNT_XY 0
#endif // NUM_EXTRUDER>1

/** \brief Configuration->WobbleFix-> */
#if FEATURE_Kurt67_WOBBLE_FIX

 UI_MENU_CHANGEACTION( ui_menu_wobble_fix_pxy , UI_TEXT_WOBBLE_FIX_PHASEXY, UI_ACTION_WOBBLE_FIX_PHASEXY )
 UI_MENU_CHANGEACTION( ui_menu_wobble_fix_ax  , UI_TEXT_WOBBLE_FIX_AMPX,    UI_ACTION_WOBBLE_FIX_AMPX )
 UI_MENU_CHANGEACTION( ui_menu_wobble_fix_ay1 , UI_TEXT_WOBBLE_FIX_AMPY1,   UI_ACTION_WOBBLE_FIX_AMPY1 )
 UI_MENU_CHANGEACTION( ui_menu_wobble_fix_ay2 , UI_TEXT_WOBBLE_FIX_AMPY2,   UI_ACTION_WOBBLE_FIX_AMPY2 )
 /*
 UI_MENU_CHANGEACTION( ui_menu_wobble_fix_pz  , UI_TEXT_WOBBLE_FIX_PHASEZ,  UI_ACTION_WOBBLE_FIX_PHASEZ )
 UI_MENU_CHANGEACTION( ui_menu_wobble_fix_az  , UI_TEXT_WOBBLE_FIX_AMPZ,    UI_ACTION_WOBBLE_FIX_AMPZ )
*/
 #define UI_MENU_WOBBLE {UI_MENU_ADDCONDBACK &ui_menu_wobble_fix_pxy, &ui_menu_wobble_fix_ax, &ui_menu_wobble_fix_ay1, &ui_menu_wobble_fix_ay2 /*, &ui_menu_wobble_fix_pz, &ui_menu_wobble_fix_az*/  }
 UI_MENU(ui_menu_wobble_fix,UI_MENU_WOBBLE,UI_MENU_BACKCNT+4 /*+2*/)

 /** \brief Configuration->WobbleFix menu */
 UI_MENU_SUBMENU(ui_menu_conf_wobble, UI_TEXT_WOBBLE, ui_menu_wobble_fix)
 #define UI_MENU_CONFIGURATION_WOBBLE_COND &ui_menu_conf_wobble,
 #define UI_MENU_CONFIGURATION_WOBBLE_COUNT 1
#else //not FEATURE_Kurt67_WOBBLE_FIX
 #define UI_MENU_CONFIGURATION_WOBBLE_COND
 #define UI_MENU_CONFIGURATION_WOBBLE_COUNT 0
#endif //FEATURE_Kurt67_WOBBLE_FIX

/** \brief Configuration->DMS-Features->SenseOffset */
#if FEATURE_SENSIBLE_PRESSURE
 UI_MENU_CHANGEACTION( ui_menu_senseoffset_digits, UI_TEXT_SENSEOFFSET_DIGITS,    UI_ACTION_SENSEOFFSET_DIGITS )
 UI_MENU_CHANGEACTION( ui_menu_senseoffset_max,    UI_TEXT_SENSEOFFSET_MAX,       UI_ACTION_SENSEOFFSET_MAX )
 UI_MENU_ACTIONCOMMAND(ui_menu_senseoffset_auto,   UI_TEXT_SENSEOFFSET_AUTOSTART, UI_ACTION_SENSEOFFSET_AUTOSTART )

 #define UI_MENU_CONF_SENSIBLE_PRESSURE {UI_MENU_ADDCONDBACK &ui_menu_senseoffset_digits, &ui_menu_senseoffset_max, &ui_menu_senseoffset_auto}
 UI_MENU(ui_submenu_settings_sense_offset,UI_MENU_CONF_SENSIBLE_PRESSURE,UI_MENU_BACKCNT+3)

 UI_MENU_SUBMENU_FILTER(ui_menu_conf_senseoffset, UI_TEXT_SENSE_OFFSET_MENU, ui_submenu_settings_sense_offset, MENU_MODE_PRINTER,0)
 #define UI_MENU_CONFIGURATION_SENSIBLE_PRESSURE_COND &ui_menu_conf_senseoffset,
 #define UI_MENU_CONFIGURATION_SENSIBLE_PRESSURE_COUNT 1

#else //FEATURE_SENSIBLE_PRESSURE
 #define UI_MENU_CONFIGURATION_SENSIBLE_PRESSURE_COND
 #define UI_MENU_CONFIGURATION_SENSIBLE_PRESSURE_COUNT 0
#endif //FEATURE_SENSIBLE_PRESSURE

/** \brief Configuration->DMS-Features->Emergency Pause */
#if FEATURE_EMERGENCY_PAUSE
 UI_MENU_CHANGEACTION( ui_menu_emergency_pause_min, UI_TEXT_EMERGENCY_PAUSE_MIN, UI_ACTION_EMERGENCY_PAUSE_MIN )
 UI_MENU_CHANGEACTION( ui_menu_emergency_pause_max, UI_TEXT_EMERGENCY_PAUSE_MAX, UI_ACTION_EMERGENCY_PAUSE_MAX )

 #define UI_MENU_CONF_EMERGENCY_PAUSE {UI_MENU_ADDCONDBACK &ui_menu_emergency_pause_min, &ui_menu_emergency_pause_max}
 UI_MENU(ui_menu_settings_emerg_pause,UI_MENU_CONF_EMERGENCY_PAUSE,UI_MENU_BACKCNT+2)

 UI_MENU_SUBMENU_FILTER(ui_menu_conf_emerg_pause, UI_TEXT_EMERGENCY_PAUSE_MENU, ui_menu_settings_emerg_pause, MENU_MODE_PRINTER,0)
 #define UI_MENU_CONFIGURATION_EMERGENCY_PAUSE_COND &ui_menu_conf_emerg_pause,
 #define UI_MENU_CONFIGURATION_EMERGENCY_PAUSE_COUNT 1

#else //FEATURE_EMERGENCY_PAUSE
 #define UI_MENU_CONFIGURATION_EMERGENCY_PAUSE_COND
 #define UI_MENU_CONFIGURATION_EMERGENCY_PAUSE_COUNT 0
#endif //FEATURE_EMERGENCY_PAUSE

/** \brief Configuration->DMS-Features->Emergency Z-Stop */
#if FEATURE_EMERGENCY_STOP_ALL
 UI_MENU_CHANGEACTION( ui_menu_emergency_zstop_min, UI_TEXT_EMERGENCY_ZSTOP_MIN, UI_ACTION_EMERGENCY_ZSTOP_MIN )
 UI_MENU_CHANGEACTION( ui_menu_emergency_zstop_max, UI_TEXT_EMERGENCY_ZSTOP_MAX, UI_ACTION_EMERGENCY_ZSTOP_MAX )

 #define UI_MENU_CONF_EMERGENCY_ZSTOP {UI_MENU_ADDCONDBACK &ui_menu_emergency_zstop_min, &ui_menu_emergency_zstop_max}
 UI_MENU(ui_menu_settings_emerg_zstop,UI_MENU_CONF_EMERGENCY_ZSTOP,UI_MENU_BACKCNT+2)

 UI_MENU_SUBMENU_FILTER(ui_menu_conf_emerg_zstop, UI_TEXT_EMERGENCY_ZSTOP_MENU, ui_menu_settings_emerg_zstop, MENU_MODE_PRINTER,0)
 #define UI_MENU_CONFIGURATION_EMERGENCY_ZSTOP_COND &ui_menu_conf_emerg_zstop,
 #define UI_MENU_CONFIGURATION_EMERGENCY_ZSTOP_COUNT 1

#else //FEATURE_EMERGENCY_STOP_ALL
 #define UI_MENU_CONFIGURATION_EMERGENCY_ZSTOP_COND
 #define UI_MENU_CONFIGURATION_EMERGENCY_ZSTOP_COUNT 0
#endif //FEATURE_EMERGENCY_STOP_ALL

/** \brief Configuration->DMS-Features->Zero Digits ON OFF */
#if FEATURE_ZERO_DIGITS
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_zero_digits_homing,UI_TEXT_DO_FEATURE_ZERO_DIGITS,UI_ACTION_FEATURE_ZERO_DIGITS,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_FEATURE_ZERO_DIGITS  &ui_menu_zero_digits_homing,
#define UI_MENU_FEATURE_ZERO_DIGITS_COUNT 1
#else
#define UI_MENU_FEATURE_ZERO_DIGITS
#define UI_MENU_FEATURE_ZERO_DIGITS_COUNT 0
#endif // FEATURE_ZERO_DIGITS

/** \brief Configuration->DMS-Features->DIGIT COMPENSATION ON OFF */
#if FEATURE_DIGIT_Z_COMPENSATION
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_digits_cmp,UI_TEXT_DO_DIGIT_COMPENSATION,UI_ACTION_DIGIT_COMPENSATION,MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_FEATURE_DIGITS_CMP  &ui_menu_digits_cmp
#define UI_MENU_FEATURE_DIGITS_CMP_COUNT 1
#else
#define UI_MENU_FEATURE_DIGITS_CMP
#define UI_MENU_FEATURE_DIGITS_CMP_COUNT 0
#endif // FEATURE_DIGIT_Z_COMPENSATION

/** \brief Configuration->DMS-Features->Flow Compensation menu */
#if FEATURE_DIGIT_FLOW_COMPENSATION
 UI_MENU_CHANGEACTION( ui_menu_flow_min, UI_TEXT_FLOW_MIN, UI_ACTION_FLOW_MIN)
 UI_MENU_CHANGEACTION( ui_menu_flow_max, UI_TEXT_FLOW_MAX, UI_ACTION_FLOW_MAX)
 UI_MENU_CHANGEACTION( ui_menu_flow_df,  UI_TEXT_FLOW_DF,  UI_ACTION_FLOW_DF)
 UI_MENU_CHANGEACTION( ui_menu_flow_dv,  UI_TEXT_FLOW_DV,  UI_ACTION_FLOW_DV)

 #define UI_MENU_CONF_FLOW_COMPENSATION {UI_MENU_ADDCONDBACK &ui_menu_flow_min, &ui_menu_flow_max, &ui_menu_flow_df, &ui_menu_flow_dv}
 UI_MENU(ui_menu_settings_flow,UI_MENU_CONF_FLOW_COMPENSATION,UI_MENU_BACKCNT+4)

 UI_MENU_SUBMENU_FILTER(ui_menu_conf_flow, UI_TEXT_FLOW_MENU, ui_menu_settings_flow, MENU_MODE_PRINTER,0)
 #define UI_MENU_CONFIGURATION_FLOW_COMPENSATION_COND &ui_menu_conf_flow,
 #define UI_MENU_CONFIGURATION_FLOW_COMPENSATION_COUNT 1

#else //FEATURE_DIGIT_FLOW_COMPENSATION
 #define UI_MENU_CONFIGURATION_FLOW_COMPENSATION_COND
 #define UI_MENU_CONFIGURATION_FLOW_COMPENSATION_COUNT 0
#endif //FEATURE_DIGIT_FLOW_COMPENSATION

#define UI_MENU_DMS {UI_MENU_ADDCONDBACK UI_MENU_CONFIGURATION_SENSIBLE_PRESSURE_COND UI_MENU_CONFIGURATION_EMERGENCY_PAUSE_COND UI_MENU_CONFIGURATION_EMERGENCY_ZSTOP_COND UI_MENU_CONFIGURATION_FLOW_COMPENSATION_COND UI_MENU_FEATURE_ZERO_DIGITS UI_MENU_FEATURE_DIGITS_CMP }
UI_MENU(ui_menu_dms,UI_MENU_DMS,UI_MENU_BACKCNT+UI_MENU_CONFIGURATION_SENSIBLE_PRESSURE_COUNT+UI_MENU_CONFIGURATION_EMERGENCY_PAUSE_COUNT+UI_MENU_CONFIGURATION_EMERGENCY_ZSTOP_COUNT+UI_MENU_CONFIGURATION_FLOW_COMPENSATION_COUNT+UI_MENU_FEATURE_ZERO_DIGITS_COUNT+UI_MENU_FEATURE_DIGITS_CMP_COUNT)

/** \brief Configuration->DMS Features menu */
UI_MENU_SUBMENU_FILTER(ui_menu_conf_dms, UI_TEXT_DMS, ui_menu_dms, MENU_MODE_PRINTER, 0)
#define UI_MENU_CONFIGURATION_DMS_COND &ui_menu_conf_dms,
#define UI_MENU_CONFIGURATION_DMS_COUNT 1

//############################################################### PID MENU

UI_MENU_ACTION4C(ui_menu_pid_choose_classicpid_ack,UI_ACTION_CHOOSE_CLASSICPID,UI_TEXT_PID_ACK)
UI_MENU_ACTIONSELECTOR(ui_menu_pid_choose_classicpid,UI_ACTION_TEXT_CLASSICPID,ui_menu_pid_choose_classicpid_ack)
UI_MENU_ACTION4C(ui_menu_pid_choose_lesserintegral_ack,UI_ACTION_CHOOSE_LESSERINTEGRAL,UI_TEXT_PID_ACK)
UI_MENU_ACTIONSELECTOR(ui_menu_pid_choose_lesserintegral,UI_ACTION_TEXT_PESSEN,ui_menu_pid_choose_lesserintegral_ack)
UI_MENU_ACTION4C(ui_menu_pid_choose_no_ack,UI_ACTION_CHOOSE_NO,UI_TEXT_PID_ACK)
UI_MENU_ACTIONSELECTOR(ui_menu_pid_choose_no,UI_ACTION_TEXT_NO,ui_menu_pid_choose_no_ack)
UI_MENU_ACTION4C(ui_menu_pid_choose_tyreus_lyben_ack,UI_ACTION_CHOOSE_TYREUS_LYBEN,UI_TEXT_PID_ACK)
UI_MENU_ACTIONSELECTOR(ui_menu_pid_choose_tyreus_lyben,UI_ACTION_TEXT_TYREUS_LYBEN,ui_menu_pid_choose_tyreus_lyben_ack)
UI_MENU_CHANGEACTION(ui_menu_pid_choose_drivemin,UI_TEXT_EXTR_DMIN,UI_ACTION_CHOOSE_DMIN)
UI_MENU_CHANGEACTION(ui_menu_pid_choose_drivemax,UI_TEXT_EXTR_DMAX,UI_ACTION_CHOOSE_DMAX)
UI_MENU_CHANGEACTION(ui_menu_pid_choose_PIDmax,UI_TEXT_EXTR_PMAX,UI_ACTION_CHOOSE_PIDMAX)
UI_MENU_CHANGEACTION(ui_menu_pid_choose_sensor,UI_TEXT_EXTR_SENSOR_TYPE,UI_ACTION_CHOOSE_SENSOR)

#define UI_MENU_PID_CHOOSE {UI_MENU_ADDCONDBACK &ui_menu_pid_choose_lesserintegral, &ui_menu_pid_choose_classicpid, &ui_menu_pid_choose_no, &ui_menu_pid_choose_tyreus_lyben, &ui_menu_pid_choose_drivemin, &ui_menu_pid_choose_drivemax, &ui_menu_pid_choose_PIDmax, &ui_menu_pid_choose_sensor}
UI_MENU(ui_menu_pid_choose, UI_MENU_PID_CHOOSE, 8)

UI_MENU_SUBMENU(ui_menu_pid_ext0_cond,  UI_TEXT_EXTRUDER " 0", ui_menu_pid_choose)
#define UI_MENU_PID_EXT0_COND   &ui_menu_pid_ext0_cond
#define UI_MENU_PID_EXT0_COUNT 1

#if NUM_EXTRUDER>1
 UI_MENU_SUBMENU(ui_menu_pid_ext1_cond,  UI_TEXT_EXTRUDER " 1", ui_menu_pid_choose)
 #define UI_MENU_PID_EXT1_COND   ,&ui_menu_pid_ext1_cond
 #define UI_MENU_PID_EXT1_COUNT 1
#else
 #define UI_MENU_PID_EXT1_COND
 #define UI_MENU_PID_EXT1_COUNT 0
#endif //NUM_EXTRUDER>1

#if HAVE_HEATED_BED==true
 UI_MENU_SUBMENU(ui_menu_pid_bed_cond,  UI_TEXT_BED,            ui_menu_pid_choose)
 #define UI_MENU_PID_BED_COND   ,&ui_menu_pid_bed_cond
 #define UI_MENU_PID_BED_COUNT 1
#else
 #define UI_MENU_PID_BED_COND
 #define UI_MENU_PID_BED_COUNT 0
#endif // HAVE_HEATED_BED==true

#define UI_MENU_PID_COND   {UI_MENU_ADDCONDBACK UI_MENU_PID_EXT0_COND UI_MENU_PID_EXT1_COND UI_MENU_PID_BED_COND}
#define UI_MENU_PID_COUNT   UI_MENU_BACKCNT + UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT
UI_MENU(ui_menu_pid,UI_MENU_PID_COND,UI_MENU_PID_COUNT)

//############################################################### PID MENU

//############################################################### MOTOR MENU
//hier wird davon ausgegangen dass 1 extruder immer da ist und der zweite optional. Auch im Millingmode.
UI_MENU_CHANGEACTION(ui_menu_motor_x,UI_TEXT_MOTOR_X,UI_ACTION_CHOOSE_MOTOR_X)
#define UI_MENU_MOTOR_X_COND   &ui_menu_motor_x
#define UI_MENU_MOTOR_X_COUNT 1
UI_MENU_CHANGEACTION(ui_menu_motor_y,UI_TEXT_MOTOR_Y,UI_ACTION_CHOOSE_MOTOR_Y)
#define UI_MENU_MOTOR_Y_COND   ,&ui_menu_motor_y
#define UI_MENU_MOTOR_Y_COUNT 1
UI_MENU_CHANGEACTION(ui_menu_motor_z,UI_TEXT_MOTOR_Z,UI_ACTION_CHOOSE_MOTOR_Z)
#define UI_MENU_MOTOR_Z_COND   ,&ui_menu_motor_z
#define UI_MENU_MOTOR_Z_COUNT 1
UI_MENU_CHANGEACTION_FILTER(ui_menu_motor_e0,     UI_TEXT_MOTOR_E0,          UI_ACTION_CHOOSE_MOTOR_E0, MENU_MODE_PRINTER,0) //extruder nur bei printermode
UI_MENU_CHANGEACTION_FILTER(ui_menu_motorsteps_e0,UI_TEXT_EXTR_STEPS0,       UI_ACTION_EXTR_STEPS_E0,   MENU_MODE_PRINTER,0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_advanceL_e0,  UI_TEXT_EXTR_ADVANCE_L_E0, UI_ACTION_ADVANCE_L_E0,    MENU_MODE_PRINTER,0)
#define UI_MENU_MOTOR_E0_COND   ,&ui_menu_motor_e0, &ui_menu_motorsteps_e0, &ui_menu_advanceL_e0
#define UI_MENU_MOTOR_E0_COUNT 3
//UI_ACTION_EXTR_STEPS
#if NUM_EXTRUDER>1
 UI_MENU_CHANGEACTION_FILTER(ui_menu_motor_e1,     UI_TEXT_MOTOR_E1,          UI_ACTION_CHOOSE_MOTOR_E1, MENU_MODE_PRINTER,0) //extruder nur bei printermode
 UI_MENU_CHANGEACTION_FILTER(ui_menu_motorsteps_e1,UI_TEXT_EXTR_STEPS1,       UI_ACTION_EXTR_STEPS_E1,   MENU_MODE_PRINTER,0)
 UI_MENU_CHANGEACTION_FILTER(ui_menu_advanceL_e1,  UI_TEXT_EXTR_ADVANCE_L_E1, UI_ACTION_ADVANCE_L_E1,    MENU_MODE_PRINTER,0)
 #define UI_MENU_MOTOR_E1_COND   ,&ui_menu_motor_e1, &ui_menu_motorsteps_e1, &ui_menu_advanceL_e1
 #define UI_MENU_MOTOR_E1_COUNT 3
#else
 #define UI_MENU_MOTOR_E1_COND
 #define UI_MENU_MOTOR_E1_COUNT 0
#endif //NUM_EXTRUDER>1

UI_MENU_CHANGEACTION(ui_menu_freq_dbl,UI_TEXT_FREQ_DBL,UI_ACTION_FREQ_DBL)
#define UI_MENU_FREQ_DBL_COND   ,&ui_menu_freq_dbl
#define UI_MENU_FREQ_DBL_COUNT 1

#if FEATURE_ADJUSTABLE_MICROSTEPS
UI_MENU_CHANGEACTION_FILTER(ui_menu_microsteps_xy, UI_TEXT_MICRO_STEPS_XY, UI_ACTION_MICROSTEPS_XY, MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_CHANGEACTION_FILTER(ui_menu_microsteps_z,  UI_TEXT_MICRO_STEPS_Z , UI_ACTION_MICROSTEPS_Z, MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
UI_MENU_CHANGEACTION_FILTER(ui_menu_microsteps_e,  UI_TEXT_MICRO_STEPS_E , UI_ACTION_MICROSTEPS_E, MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
    #define UI_MENU_ADJUSTABLE_MICROSTEPS_COND   ,&ui_menu_microsteps_xy ,&ui_menu_microsteps_z ,&ui_menu_microsteps_e
    #define UI_MENU_ADJUSTABLE_MICROSTEPS_COUNT  3
#else
    #define UI_MENU_ADJUSTABLE_MICROSTEPS_COND
    #define UI_MENU_ADJUSTABLE_MICROSTEPS_COUNT  0
#endif //FEATURE_ADJUSTABLE_MICROSTEPS

#define UI_MENU_MOTOR_COND   {UI_MENU_ADDCONDBACK UI_MENU_MOTOR_X_COND UI_MENU_MOTOR_Y_COND UI_MENU_MOTOR_Z_COND UI_MENU_MOTOR_E0_COND UI_MENU_MOTOR_E1_COND UI_MENU_FREQ_DBL_COND UI_MENU_ADJUSTABLE_MICROSTEPS_COND}
#define UI_MENU_MOTOR_COUNT   UI_MENU_BACKCNT + UI_MENU_MOTOR_X_COUNT + UI_MENU_MOTOR_Y_COUNT + UI_MENU_MOTOR_Z_COUNT + UI_MENU_MOTOR_E0_COUNT + UI_MENU_MOTOR_E1_COUNT + UI_MENU_FREQ_DBL_COUNT + UI_MENU_ADJUSTABLE_MICROSTEPS_COUNT
UI_MENU(ui_menu_motor,UI_MENU_MOTOR_COND,UI_MENU_MOTOR_COUNT)
//############################################################### MOTOR MENU

#define UI_MENU_GENERAL {UI_MENU_ADDCONDBACK &ui_menu_general_baud,&ui_menu_general_stepper_inactive,&ui_menu_general_max_inactive BEEPER_MODE_ENTRY RGB_LIGHT_ENTRY OPERATING_MODE_ENTRY Z_ENDSTOP_TYPE_ENTRY MILLER_TYPE_ENTRY EXTRUDER_OFFSET_TYPE_ENTRY_XY}
UI_MENU(ui_menu_general,UI_MENU_GENERAL,UI_MENU_BACKCNT+1+1+1+BEEPER_MODE_COUNT+RGB_LIGHT_COUNT+OPERATING_MODE_COUNT+Z_ENDSTOP_TYPE_COUNT+MILLER_TYPE_COUNT+EXTRUDER_OFFSET_TYPE_COUNT_XY)

/** \brief Configuration menu */
UI_MENU_SUBMENU(ui_menu_conf_general,  UI_TEXT_GENERAL,        ui_menu_general)
UI_MENU_SUBMENU_FILTER(ui_menu_conf_accel,    UI_TEXT_ACCELERATION,   ui_menu_accel, MENU_MODE_PRINTER, MENU_MODE_MILLER)
#if FEATURE_MILLING_MODE
 UI_MENU_CHANGEACTION_FILTER(ui_menu_help_accel_mill,UI_TEXT_ACCEL_MILL,UI_ACTION_MILL_ACCELERATION,MENU_MODE_MILLER,MENU_MODE_PRINTER)
 #define UI_MENU_ACCEL_MILL_COND &ui_menu_help_accel_mill,
 #define UI_MENU_ACCEL_MILL_COUNT 1
#else
 #define UI_MENU_ACCEL_MILL_COND
 #define UI_MENU_ACCEL_MILL_COUNT 0
#endif //FEATURE_MILLING_MODE
UI_MENU_SUBMENU(ui_menu_conf_feed,     UI_TEXT_FEEDRATE,       ui_menu_feedrate)
UI_MENU_SUBMENU_FILTER(ui_menu_conf_pid, UI_TEXT_TEMPERATURES, ui_menu_pid, MENU_MODE_PRINTER, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
//#if MENU_MODE_PRINTER
//#define MENU_MODE_PRINTER_COUNT 1
//#else
//#define MENU_MODE_PRINTER_COUNT 0
//#endif //MENU_MODE_PRINTER
UI_MENU_SUBMENU(ui_menu_conf_motor, UI_TEXT_STEPPER,      ui_menu_motor)
UI_MENU_SUBMENU(ui_menu_z_calibration, UI_TEXT_ZCALIB,         ui_menu_z)

UI_MENU_ACTION4C(ui_menu_restore_defaults_ack,UI_ACTION_RESTORE_DEFAULTS,UI_TEXT_RESTORE_DEFAULTS4)
UI_MENU_ACTIONSELECTOR_FILTER(ui_menu_restore_defaults,UI_TEXT_RESTORE_DEFAULTS,ui_menu_restore_defaults_ack, 0, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)

#define UI_MENU_CONFIGURATION {UI_MENU_ADDCONDBACK &ui_menu_conf_general, &ui_menu_z_calibration, UI_MENU_CONFIGURATION_DMS_COND UI_MENU_CONFIGURATION_WOBBLE_COND UI_MENU_CONFIGURATION_FAN_COND &ui_menu_conf_pid, &ui_menu_conf_motor, &ui_menu_conf_accel, UI_MENU_ACCEL_MILL_COND &ui_menu_conf_feed,  &ui_menu_restore_defaults }
UI_MENU(ui_menu_configuration,UI_MENU_CONFIGURATION,UI_MENU_BACKCNT+1+1+UI_MENU_CONFIGURATION_DMS_COUNT+UI_MENU_CONFIGURATION_FAN_COUNT+5+UI_MENU_ACCEL_MILL_COUNT+UI_MENU_CONFIGURATION_WOBBLE_COUNT)

/** \brief Main menu */
UI_MENU_SUBMENU(ui_menu_main1, UI_TEXT_QUICK_SETTINGS,  ui_menu_quick)
#define UI_MENU_QUICK_SETTINGS_ENTRY    &ui_menu_main1,
#define UI_MENU_QUICK_SETTINGS_COUNT    1
UI_MENU_SUBMENU_FILTER(ui_menu_main2, UI_TEXT_POSITION, ui_menu_positions, 0, MENU_MODE_PRINTING | MENU_MODE_SD_PRINTING | MENU_MODE_PAUSED)
#define UI_MENU_POSITION_ENTRY    &ui_menu_main2,
#define UI_MENU_POSITION_COUNT    1
UI_MENU_SUBMENU_FILTER(ui_menu_main3, UI_TEXT_EXTRUDER, ui_menu_extruder, MENU_MODE_PRINTER, 0)
#define UI_MENU_EXTRUDER_ENTRY    &ui_menu_main3,
#define UI_MENU_EXTRUDER_COUNT    1

#if SHOW_DEBUGGING_MENU
UI_MENU_SUBMENU(ui_menu_main4, UI_TEXT_DEBUGGING,       ui_menu_debugging)
#define DEBUGGING_MENU_ENTRY    &ui_menu_main4,
#define DEBUGGING_MENU_COUNT    1
#else
#define DEBUGGING_MENU_ENTRY
#define DEBUGGING_MENU_COUNT    0
#endif // SHOW_DEBUGGING_MENU

UI_MENU_SUBMENU(ui_menu_main5, UI_TEXT_CONFIGURATION,  ui_menu_configuration)
#define UI_MENU_CONFIGURATION_ENTRY    &ui_menu_main5
#define UI_MENU_CONFIGURATION_COUNT    1

#if SDSUPPORT && (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2)
#define UI_MENU_RF2000_FILE_COND    &ui_menu_sd_print_file, &ui_menu_sd_mill_file,
#define UI_MENU_RF2000_FILE_COUNT   2
#else
#define UI_MENU_RF2000_FILE_COND
#define UI_MENU_RF2000_FILE_COUNT   0
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

#define UI_MENU_MAIN {UI_MENU_ADDCONDBACK UI_MENU_QUICK_SETTINGS_ENTRY UI_MENU_RF2000_FILE_COND UI_MENU_POSITION_ENTRY UI_MENU_EXTRUDER_ENTRY UI_MENU_FAN_COND UI_MENU_SD_COND DEBUGGING_MENU_ENTRY UI_MENU_CONFIGURATION_ENTRY}
UI_MENU(ui_menu_main,UI_MENU_MAIN,UI_MENU_BACKCNT+UI_MENU_QUICK_SETTINGS_COUNT+UI_MENU_RF2000_FILE_COUNT+UI_MENU_POSITION_COUNT+UI_MENU_EXTRUDER_COUNT+UI_MENU_FAN_CNT+UI_MENU_SD_CNT+DEBUGGING_MENU_COUNT+UI_MENU_CONFIGURATION_COUNT)

/* Define menus accessible by action commands

You can create up to 10 user menus which are accessible by the action commands UI_ACTION_SHOW_USERMENU1 until UI_ACTION_SHOW_USERMENU10
Do this the same way as with the menus above or you use one of the above menus. Then add a define like

#define UI_USERMENU1 ui_menu_conf_feed

which assigns the menu stored in ui_menu_conf_feed to the action UI_ACTION_SHOW_USERMENU1. Make sure only to change the numbers and not the name of the define.

When do you need this? You might want a fast button to change the temperature. In the default menu you have no menu
to change the temperature and view it the same time. So you need to make an action menu for this like:
UI_MENU_ACTION4C(ui_menu_extrtemp,UI_ACTION_EXTRUDER0_TEMP,"Temp. 0  :%E0\002C","","","");
Then you assign this menu to a usermenu:
#define UI_USERMENU2 ui_menu_extrtemp

Now you can assign the action  UI_ACTION_SHOW_USERMENU2+UI_ACTION_TOPMENU to a key and that will now show the temperture screen and allows
the change of temperature with the next/previous buttons. */

#endif // UI_MENU_H
