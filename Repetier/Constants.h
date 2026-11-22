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

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define REPETIER_VERSION "1.45.03"
#define UI_PRINTER_COMPANY "Conrad Community"
#define UI_VERSION_STRING "V " REPETIER_VERSION

#define BIGC0 "\001\002\003    " UI_SPACER "Renkforce"
#define BIGC1 "\004         " UI_SPACER UI_PRINTER_NAME
#define BIGC2 "\005\006\007ommunity Mod"
#define BIGC3 UI_SPACER REPETIER_VERSION

// ##########################################################################################
// ##    basic definitions
// ##########################################################################################

#define DEVICE_TYPE_RF1000 13
#define DEVICE_TYPE_RF2000 14
#define DEVICE_TYPE_RF2000v2 15

// Give an enabled stepper driver time to settle before the first step.
constexpr unsigned int stepperEnableDelayMs = 25;

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define VIRTUAL_AXIS 4


#define MENU_MODE_PAUSED 4
#define MENU_MODE_FAN_RUNNING 8
#define MENU_MODE_PRINTING 16
#define MENU_MODE_FAN_MODE_PDM 128

#define IGNORE_COORDINATE 99999.0f

#define TASK_NO_TASK 0
#define TASK_ENABLE_Z_COMPENSATION 1
#define TASK_DISABLE_Z_COMPENSATION 2
#define TASK_PAUSE_PRINT 4
#define TASK_PAUSE_PRINT_AND_MOVE 5
#define TASK_MOVE_FROM_BUTTON 10
#define TASK_MOVE_POSITION_MANUAL 11
#define TASK_ENABLE_SENSE_OFFSET 100
#define TASK_UNLOCK_DISPLAY_MSG 101

#define RGB_MODE_OFF 0
#define RGB_MODE_WHITE 1
#define RGB_MODE_AUTOMATIC 2
#define RGB_MODE_MANUAL 3

#define RGB_STATUS_NOT_AUTOMATIC 0
#define RGB_STATUS_AUTOMATIC 1
#define RGB_STATUS_PRINTING 11
#define RGB_STATUS_HEATING 12
#define RGB_STATUS_COOLING 13
#define RGB_STATUS_IDLE 14
#define RGB_STATUS_COLOR_CHANGE 15

#define MOVE_MODE_SINGLE_STEPS 1
#define MOVE_MODE_1_MM 2
#define MOVE_MODE_10_MM 3
#define MOVE_MODE_50_MM 4
#define MOVE_MODE_SINGLE_MOVE 5

#define HEAT_BED_SCAN_MODE_PLA 1
#define HEAT_BED_SCAN_MODE_ABS 2

#define Z_VALUE_MODE_Z_MIN 1 // show the distance to Z-min
#define Z_VALUE_MODE_LAYER 3 // show the nominal layer height

#define STOP_BECAUSE_OF_Z_MIN 1
#define STOP_BECAUSE_OF_Z_BLOCK 2

// ##########################################################################################
// ##    data types
// ##########################################################################################

#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t

#endif // CONSTANTS_H
