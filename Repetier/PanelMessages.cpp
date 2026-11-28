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

FSTRINGVALUE(ui_text_error, UI_TEXT_ERROR)
FSTRINGVALUE(ui_text_warning, UI_TEXT_WARNING)
FSTRINGVALUE(ui_text_information, UI_TEXT_INFORMATION)
FSTRINGVALUE(ui_text_heat_bed_scan, UI_TEXT_HEAT_BED_SCAN)
FSTRINGVALUE(ui_text_output_object, UI_TEXT_OUTPUT_OBJECT)
FSTRINGVALUE(ui_text_set_origin, UI_TEXT_SET_ORIGIN)
FSTRINGVALUE(ui_text_pause, UI_TEXT_PAUSE)
FSTRINGVALUE(ui_text_home, UI_TEXT_HOME)
FSTRINGVALUE(ui_text_z_compensation, UI_TEXT_Z_COMPENSATION)
FSTRINGVALUE(ui_text_x_axis, UI_TEXT_X_AXIS)
FSTRINGVALUE(ui_text_y_axis, UI_TEXT_Y_AXIS)
FSTRINGVALUE(ui_text_z_axis, UI_TEXT_Z_AXIS)
FSTRINGVALUE(ui_text_extruder, UI_TEXT_EXTRUDER)
FSTRINGVALUE(ui_text_autodetect_pid, UI_TEXT_AUTODETECT_PID)
FSTRINGVALUE(ui_text_temperature_manager, UI_TEXT_TEMPERATURE_MANAGER)
FSTRINGVALUE(ui_text_home_unknown, UI_TEXT_HOME_UNKNOWN)
FSTRINGVALUE(ui_text_saving_needless, UI_TEXT_SAVING_NEEDLESS)
FSTRINGVALUE(ui_text_operation_denied, UI_TEXT_OPERATION_DENIED)
FSTRINGVALUE(ui_text_emergency_pause, UI_TEXT_EMERGENCY_PAUSE)
FSTRINGVALUE(ui_text_emergency_stop, UI_TEXT_EMERGENCY_STOP)
FSTRINGVALUE(ui_text_invalid_matrix, UI_TEXT_INVALID_MATRIX)
FSTRINGVALUE(ui_text_min_reached, UI_TEXT_MIN_REACHED)
FSTRINGVALUE(ui_text_max_reached, UI_TEXT_MAX_REACHED)
FSTRINGVALUE(ui_text_temperature_wrong, UI_TEXT_TEMPERATURE_WRONG)
FSTRINGVALUE(ui_text_timeout, UI_TEXT_TIMEOUT)
FSTRINGVALUE(ui_text_sensor_error, UI_TEXT_SENSOR_ERROR)
FSTRINGVALUE(ui_text_heater_error, UI_TEXT_HEATER_ERROR)

FSTRINGVALUE(ui_text_manual, UI_TEXT_MANUAL)
FSTRINGVALUE(ui_text_saving_success, UI_TEXT_SAVING_SUCCESS)

void showIdle(void) {
    UI_STATUS(UI_TEXT_PRINTER_READY);
} // showIdle

void showError(const void* line2, const void* line3, const void* line4) {
    uid.showMessage((const void*)ui_text_error, line2, line3, line4);

} // showError

void showWarning(const void* line2, const void* line3, const void* line4) {
    uid.showMessage((const void*)ui_text_warning, line2, line3, line4);

} // showWarning

void showInformation(const void* line2, const void* line3, const void* line4) {
    uid.showMessage((const void*)ui_text_information, line2, line3, line4);

} // showInformation

void showMyPage(const void* line1, const void* line2, const void* line3, const void* line4) {
    uid.showMessage(line1, line2, line3, line4);

} // showMyPage
