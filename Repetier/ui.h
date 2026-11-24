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

#ifndef UI_H
#define UI_H

#include "gcode.h"

// ----------------------------------------------------------------------------
//                          Action codes
// 1-999     : Autorepeat
// 1000-1999 : Execute
// 2000-2999 : Write code
// 4000-4999 : Show menu
// Add UI_ACTION_TOPMENU to show a menu as top menu
// ----------------------------------------------------------------------------

//Vorsicht: Hier beim Topmenü wird extra das Bit abgefragt: Kein anderer Befehl sollte 2^13 drin haben, sonst siehe ui.cpp :: case UI_ACTION_TOPMENU -> Rückfall ins Hauptmenü.
#define UI_ACTION_TOPMENU 8192

#define UI_ACTION_NEXT 1
#define UI_ACTION_PREVIOUS 2

#define UI_ACTION_DUMMY 10000
#define UI_ACTION_BACK 1000
#define UI_ACTION_OK 1001
#define UI_ACTION_EMERGENCY_STOP 1004
#define UI_ACTION_XPOSITION 1005
#define UI_ACTION_YPOSITION 1006
#define UI_ACTION_ZPOSITION 1007
#define UI_ACTION_EPOSITION 1008
#define UI_ACTION_HOME_ALL 1021
#define UI_ACTION_HOME_X 1022
#define UI_ACTION_HOME_Y 1023
#define UI_ACTION_HOME_Z 1024
#define UI_ACTION_SELECT_EXTRUDER1 1025
#define UI_ACTION_PRINT_ACCEL_X 1032
#define UI_ACTION_PRINT_ACCEL_Y 1033
#define UI_ACTION_PRINT_ACCEL_Z 1034
#define UI_ACTION_MOVE_ACCEL_X 1035
#define UI_ACTION_MOVE_ACCEL_Y 1036
#define UI_ACTION_MOVE_ACCEL_Z 1037
#define UI_ACTION_MAX_JERK 1038
#define UI_ACTION_MAX_ZJERK 1039
#define UI_ACTION_BAUDRATE 1040
#define UI_ACTION_HOMING_FEEDRATE_X 1041
#define UI_ACTION_HOMING_FEEDRATE_Y 1042
#define UI_ACTION_HOMING_FEEDRATE_Z 1043
#define UI_ACTION_MAX_FEEDRATE_X 1044
#define UI_ACTION_MAX_FEEDRATE_Y 1045
#define UI_ACTION_MAX_FEEDRATE_Z 1046
#define UI_ACTION_FAN_OFF 1050
#define UI_ACTION_FAN_25 1051
#define UI_ACTION_FAN_50 1052
#define UI_ACTION_FAN_75 1053
#define UI_ACTION_FAN_FULL 1054
#define UI_ACTION_FEEDRATE_MULTIPLY 1055
#define UI_ACTION_STEPPER_INACTIVE 1056
#define UI_ACTION_DEBUG_ECHO 1074
#define UI_ACTION_DEBUG_INFO 1075
#define UI_ACTION_DEBUG_ERROR 1076
#define UI_ACTION_DEBUG_DRYRUN 1077
#define UI_ACTION_PREHEAT_PLA 1079
#define UI_ACTION_COOLDOWN 1080
#define UI_ACTION_HEATED_BED_OFF 1081
#define UI_ACTION_EXTRUDER0_OFF 1082
#define UI_ACTION_EXTRUDER1_OFF 1083
#define UI_ACTION_HEATED_BED_TEMP 1084
#define UI_ACTION_EXTRUDER0_TEMP 1085
#define UI_ACTION_EXTRUDER1_TEMP 1086
#define UI_ACTION_DISABLE_STEPPER 1090
#define UI_ACTION_SET_E_ORIGIN 1091
#define UI_ACTION_EXTRUDER_RELATIVE 1092
#define UI_ACTION_SELECT_EXTRUDER0 1093

#define UI_ACTION_PREHEAT_ABS 1095
#define UI_ACTION_FLOWRATE_MULTIPLY 1096

#define UI_ACTION_FANSPEED 1106
#define UI_ACTION_LIGHTS_ONOFF 1107

#define UI_ACTION_STOP_ACK 1109

#define UI_ACTION_UNMOUNT_FILAMENT_SOFT 1110
#define UI_ACTION_UNMOUNT_FILAMENT_HARD 1111
#define UI_ACTION_MOUNT_FILAMENT_SOFT 1112
#define UI_ACTION_MOUNT_FILAMENT_HARD 1113

#define UI_ACTION_MAX_INACTIVE 1114

#define UI_ACTION_230V_OUTPUT 1122
#define UI_ACTION_RGB_LIGHT_MODE 1123
#define UI_ACTION_EXTRUDER_OFFSET_X 1124
#define UI_ACTION_EXTRUDER_OFFSET_Y 1125
#define UI_ACTION_RESTORE_DEFAULTS 1126
#define UI_ACTION_ACTIVE_EXTRUDER 1127
#define UI_ACTION_ZOFFSET 1128
#define UI_ACTION_RIGHT 1129
#define UI_ACTION_ZMODE 1130

#define UI_ACTION_EXTR_ACCELERATION_E0 1137
#define UI_ACTION_EXTR_MAX_FEEDRATE_E0 1138
#define UI_ACTION_EXTR_START_FEEDRATE_E0 1139

#define UI_ACTION_EXTR_ACCELERATION_E1 1140
#define UI_ACTION_EXTR_MAX_FEEDRATE_E1 1141
#define UI_ACTION_EXTR_START_FEEDRATE_E1 1142

//Nibbels : nicht in processbutton sondern in executeaction!
#define UI_ACTION_RF_DO_MHIER_BED_SCAN 1666
#define UI_ACTION_CONFIG_SINGLE_STEPS 1667
#define UI_ACTION_RF_DO_SAVE_ACTIVE_ZMATRIX 1668
#define UI_ACTION_RF_DO_MHIER_AUTO_MATRIX_LEVELING 1669

#define UI_ACTION_EMERGENCY_PAUSE_MIN 1670
#define UI_ACTION_EMERGENCY_PAUSE_MAX 1671
#define UI_ACTION_EMERGENCY_ZSTOP_MIN 1672
#define UI_ACTION_EMERGENCY_ZSTOP_MAX 1673

#define UI_ACTION_CHOOSE_CLASSICPID 1674     //pid kram
#define UI_ACTION_CHOOSE_LESSERINTEGRAL 1675 //pid kram
#define UI_ACTION_CHOOSE_NO 1677             //pid kram
#define UI_ACTION_CHOOSE_TYREUS_LYBEN 1701   //pid kram

#define UI_ACTION_CHOOSE_DMIN 1678
#define UI_ACTION_CHOOSE_DMAX 1679
#define UI_ACTION_CHOOSE_PIDMAX 1680
#define UI_ACTION_CHOOSE_SENSOR 1681
#define UI_ACTION_CHOOSE_MOTOR_X 1682
#define UI_ACTION_CHOOSE_MOTOR_Y 1683
#define UI_ACTION_CHOOSE_MOTOR_Z 1684
#define UI_ACTION_CHOOSE_MOTOR_E0 1685
#define UI_ACTION_CHOOSE_MOTOR_E1 1686
#define UI_ACTION_EXTR_STEPS_E0 1687
#define UI_ACTION_EXTR_STEPS_E1 1688
#define UI_ACTION_ADVANCE_L_E0 1689
#define UI_ACTION_ADVANCE_L_E1 1690
#define UI_ACTION_SHIFT_INTERVAL 1691

#define UI_ACTION_FAN_HZ 1692
#define UI_ACTION_FAN_MODE 1693

#define UI_ACTION_DIGIT_HOMING 1699
#define UI_ACTION_DIGIT_COMPENSATION 1700
//1701 oben
#define UI_ACTION_FLOW_MIN 1702
#define UI_ACTION_FLOW_MAX 1703
#define UI_ACTION_FLOW_DF 1704
#define UI_ACTION_FLOW_DV 1705

#define UI_ACTION_MICROSTEPS_XY 1707
#define UI_ACTION_MICROSTEPS_Z 1708
#define UI_ACTION_MICROSTEPS_E 1709

#define UI_ACTION_SENSEOFFSET_DIGITS 1710
#define UI_ACTION_SENSEOFFSET_MAX 1711
#define UI_ACTION_SENSEOFFSET_AUTOSTART 1712

#define UI_ACTION_WOBBLE_FIX_PHASEXY 1713
#define UI_ACTION_WOBBLE_FIX_AMPX 1715
#define UI_ACTION_WOBBLE_FIX_AMPY1 1716
#define UI_ACTION_WOBBLE_FIX_AMPY2 1717

#define UI_ACTION_FAN_PART_FAN_PWM_MIN 1719
#define UI_ACTION_FAN_PART_FAN_PWM_MAX 1720

#define UI_ACTION_FET1_OUTPUT 2001
#define UI_ACTION_FET2_OUTPUT 2002

#define UI_ACTION_CONFIG_SINGLE_STEPS_KOSYS 2003
#define UI_ACTION_CONFIG_POSITION_FEEDRATE 2004

#define UI_ACTION_MENU_XPOS 4000
#define UI_ACTION_MENU_YPOS 4001
#define UI_ACTION_MENU_ZPOS 4002
#define UI_ACTION_MENU_QUICKSETTINGS 4007
#define UI_ACTION_MENU_EXTRUDER 4008
#define UI_ACTION_MENU_POSITIONS 4009


// Load basic language definition to make sure all values are defined
#include "uilang.h"

// Stock LCD geometry, with separate widths for the two panels.
#define UI_ROWS 4
#if MOTHERBOARD == DEVICE_TYPE_RF1000
#define UI_COLS 16        //check MAX_COLS when changed
#define UI_SPACER ""
#endif // MOTHERBOARD == DEVICE_TYPE_RF1000

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
#define UI_COLS 20        //check MAX_COLS when changed
#define UI_SPACER "    "
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

//mtype usw.
#define UI_MENU_TYPE_SUBMENU 2
#define UI_MENU_TYPE_MODIFICATION_MENU 3

typedef struct
{
    const char* text;    // Menu text
    uint8_t menuType;    // 0 = info, 2 = submenu, 3 = direct action, 4 = modify action
    unsigned int action; // must be int so it gets 32 bit on arm!
    uint8_t filter;      // allows dynamic menu filtering based on active menu mode bits set.
    uint8_t nofilter;    // Hide if one of these bits are set

    bool showEntry() const;

} const UIMenuEntry;

typedef struct
{
    // 0 = info page
    // 2 = submenu
    // 3 = modification menu
    unsigned char menuType;
    int id; // Type of modification
    int numEntries;

    const UIMenuEntry* const* entries;

} const UIMenu;


// Key codes
#define UI_KEYS_INIT_BUTTON_LOW(pin) \
    SET_INPUT(pin); \
    PULLUP(pin, HIGH);

#define UI_KEYS_BUTTON_LOW(pin, action_) \
    if (READ(pin) == 0) \
        action = action_;


// I2C keymask tests

#define UI_STRING(name, text) const char PROGMEM name[] = text;


#define UI_PAGE4(name, row1, row2, row3, row4) \
    UI_STRING(name##_1txt, row1) \
    UI_STRING(name##_2txt, row2) UI_STRING(name##_3txt, row3) UI_STRING(name##_4txt, row4) \
        UIMenuEntry name##_1 PROGMEM \
        = { name##_1txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_2 PROGMEM = { name##_2txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_3 PROGMEM = { name##_3txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_4 PROGMEM = { name##_4txt, 0, 0, 0, 0 }; \
    const UIMenuEntry* const name##_entries[] PROGMEM = { &name##_1, &name##_2, &name##_3, &name##_4 }; \
    const UIMenu name PROGMEM = { 0, 0, 4, name##_entries };


#define UI_MENU_ACTION4(name, action, row1, row2, row3, row4) \
    UI_STRING(name##_1txt, row1) \
    UI_STRING(name##_2txt, row2) UI_STRING(name##_3txt, row3) UI_STRING(name##_4txt, row4) \
        UIMenuEntry name##_1 PROGMEM \
        = { name##_1txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_2 PROGMEM = { name##_2txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_3 PROGMEM = { name##_3txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_4 PROGMEM = { name##_4txt, 0, 0, 0, 0 }; \
    const UIMenuEntry* const name##_entries[] PROGMEM = { &name##_1, &name##_2, &name##_3, &name##_4 }; \
    const UIMenu name PROGMEM = { 3, action, 4, name##_entries };

#define UI_MENU_ACTION2(name, action, row1, row2) \
    UI_STRING(name##_1txt, row1) \
    UI_STRING(name##_2txt, row2) \
        UIMenuEntry name##_1 PROGMEM \
        = { name##_1txt, 0, 0, 0, 0 }; \
    UIMenuEntry name##_2 PROGMEM = { name##_2txt, 0, 0, 0, 0 }; \
    const UIMenuEntry* const name##_entries[] PROGMEM = { &name##_1, &name##_2 }; \
    const UIMenu name PROGMEM = { 3, action, 2, name##_entries };

#define UI_MENU_CHANGEACTION(name, row, action) UI_STRING(name##_txt, row) \
                                                UIMenuEntry name PROGMEM = { name##_txt, 4, action, 0, 0 };
#define UI_MENU_ACTIONCOMMAND(name, row, action) UI_STRING(name##_txt, row) \
                                                 UIMenuEntry name PROGMEM = { name##_txt, 3, action, 0, 0 };
#define UI_MENU_ACTIONSELECTOR(name, row, entries) UI_STRING(name##_txt, row) \
                                                   UIMenuEntry name PROGMEM = { name##_txt, 2, (unsigned int)&entries, 0, 0 };
#define UI_MENU_SUBMENU(name, row, entries) UI_STRING(name##_txt, row) \
                                            UIMenuEntry name PROGMEM = { name##_txt, 2, (unsigned int)&entries, 0, 0 };
#define UI_MENU_CHANGEACTION_FILTER(name, row, action, filter, nofilter) UI_STRING(name##_txt, row) \
                                                                         UIMenuEntry name PROGMEM = { name##_txt, 4, action, filter, nofilter };
#define UI_MENU_ACTIONCOMMAND_FILTER(name, row, action, filter, nofilter) UI_STRING(name##_txt, row) \
                                                                          UIMenuEntry name PROGMEM = { name##_txt, 3, action, filter, nofilter };
#define UI_MENU_ACTIONSELECTOR_FILTER(name, row, entries, filter, nofilter) UI_STRING(name##_txt, row) \
                                                                            UIMenuEntry name PROGMEM = { name##_txt, 2, (unsigned int)&entries, filter, nofilter };
#define UI_MENU_SUBMENU_FILTER(name, row, entries, filter, nofilter) UI_STRING(name##_txt, row) \
                                                                     UIMenuEntry name PROGMEM = { name##_txt, 2, (unsigned int)&entries, filter, nofilter };
#define UI_MENU(name, items, itemsCnt) \
    const UIMenuEntry* const name##_entries[] PROGMEM = items; \
    const UIMenu name PROGMEM = { 2, 0, itemsCnt, name##_entries };
// Maximum size of a row - if row is larger, text gets scrolled
#define MAX_COLS 28 //Anzahl Speicherplätze für Bytes in Display-Zeile, wenn mehr als Display, dann Scrolling. Kann gleich oder größer Displayzeilenlänge sein.

#define UI_FLAG_SLOW_ACTION_RUNNING 4
#define UI_FLAG_KEY_TEST_RUNNING 8

#define MAX_MENU_LEVELS 6

class UIDisplay {
public:
    volatile uint8_t flags;
    uint8_t col;                      // current col for buffer prefill
    uint8_t menuLevel;                // current menu level, 0 = info, 1 = group, 2 = groupdata select, 3 = value change
    uint8_t menuPos[MAX_MENU_LEVELS]; // Positions in menu
    void* menu[MAX_MENU_LEVELS];      // Menus active
    uint8_t menuTop[MAX_MENU_LEVELS]; // Top row in menu
    int8_t shift;                     // Display shift for scrolling text
    int activeAction; // action for ok/next/previous
    int lastAction;
    volatile int jogButtonAction; // Key that started the current stoppable move.
    millis_t lastSwitch; // Last time display switched pages
    millis_t lastRefresh;
    volatile int lastButtonAction;
    volatile millis_t lastButtonStart;
    millis_t nextRepeat;   // Time of next autorepeat
    millis_t lastNextPrev; // for increasing speed settings
    float lastNextAccumul; // Accumulated value
    int repeatDuration;    // Time beween to actions if autorepeat is enabled
    char statusMsg[MAX_COLS + 1];
    char printCols[MAX_COLS + 1];
    char locked;

    UIDisplay();

    void addInt(int value, uint8_t digits, char fillChar = ' '); // Print int into printCols
    void addLong(long value, char digits);
    void addFloat(float number, char fixdigits, uint8_t digits);
    void addStringP(FSTRINGPARAM(text));
    void ui_init_keys();
    void ui_check_keys(int& action);
    void menuEsc();
    void okAction();
    void rightAction();
    void backAction();
    void nextPreviousAction(int8_t next);
    void senseOffsetUpAction();
    void senseOffsetDownAction();
    void zOffsetPlusAction();
    void zOffsetMinusAction();
    void createChar(uint8_t location, const uint8_t charmap[]);
    void initialize(); // Initialize display and keys
    void initializeLCD(bool normal = true);
    void printRow(uint8_t r, char* txt, char* txt2, uint8_t changeAtCol); // Print row on display
    void printRowP(uint8_t r, PGM_P txt);
    void parse(char* txt, bool ram); /// Parse output and write to printCols;
    void refreshPage();
    const char* cachedRow(uint8_t row) const;
    void executeAction(int action);
    void slowAction();

    void finishAction(int action);
    void pressPlay();
    void changeSwitchCase(int action, int8_t increment);
    void mainSwitchCase(int action);

    void fastAction();
    void showMessage(const void* line1, const void* line2, const void* line3, const void* line4);
    void scheduleMessageReturn(millis_t deadline);
    void pushMenu(void* men, bool refresh);
    void adjustMenuPos();
    void setStatusP(PGM_P txt, bool error = false);
    void setStatus(char* txt, bool error = false, bool force = false);


    void lock();
    void unlock();
    void exitmenu();

private:
    const void* messageLine1;
    const void* messageLine2;
    const void* messageLine3;
    const void* messageLine4;
    millis_t autoReturnTime_ = 0;
    bool autoReturnMessage_ = false;
    char confirmation_ = 0;
};

extern UIDisplay uid;

#if UI_COLS == 16
#define UI_LINE_OFFSETS \
    { 0, 0x40, 0x10, 0x50 } // 4x16
#elif UI_COLS == 20
//#define UI_LINE_OFFSETS {0,0x20,0x40,0x60} // 4x20 with KS0073
#define UI_LINE_OFFSETS \
    { 0, 0x40, 0x14, 0x54 } // 4x20 with HD44780
#else
#error Unknown combination of rows/columns - define UI_LINE_OFFSETS manually.
#endif // UI_COLS==16

#include "uilang.h"
#include "uimenu.h"

#define UI_STATUS(status) uid.setStatusP(PSTR(status));
#define UI_STATUS_UPD(status) \
    { \
        uid.setStatusP(PSTR(status)); \
        uid.refreshPage(); \
    }
#define UI_STATUS_UPD_RAM(status) \
    { \
        uid.setStatus(status); \
        uid.refreshPage(); \
    }

#endif // UI_H
