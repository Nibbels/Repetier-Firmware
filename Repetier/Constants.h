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
#define UI_VERSION_STRING "V " REPETIER_VERSION

// ##########################################################################################
// ##    basic definitions
// ##########################################################################################

#define DEVICE_TYPE_RF1000 13
#define DEVICE_TYPE_RF2000 14
#define DEVICE_TYPE_RF2000v2 15

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#define TASK_NO_TASK 0
#define TASK_ENABLE_Z_COMPENSATION 1
#define TASK_DISABLE_Z_COMPENSATION 2
#define TASK_PAUSE_PRINT 4
#define TASK_PAUSE_PRINT_AND_MOVE 5
#define TASK_MOVE_FROM_BUTTON 10
#define TASK_MOVE_POSITION_MANUAL 11
#define TASK_ENABLE_SENSE_OFFSET 100
#define TASK_UNLOCK_DISPLAY_MSG 101

#define STOP_BECAUSE_OF_Z_MIN 1
#define STOP_BECAUSE_OF_Z_BLOCK 2

#endif // CONSTANTS_H
