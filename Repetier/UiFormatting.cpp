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

// LCD template parsing and field rendering; no menu navigation or setting edits.

#include "Repetier.h"
#include "StepExecutor.h"
#include "ZScrewAdjustment.h"

void UIDisplay::addInt(int16_t value, uint8_t digits, char fillChar) {
    if (col >= MAX_COLS)
        return;
    const bool negative = value < 0;
    uint16_t magnitude = static_cast<uint16_t>(value);
    if (negative)
        magnitude = 0U - magnitude;
    uint8_t dig = negative;
    char buf[7]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[6];
    buf[6] = 0;
    do {
        const uint16_t quotient = magnitude / 10;
        *--str = '0' + (magnitude - 10 * quotient);
        magnitude = quotient;
        dig++;
    } while (magnitude);
    if (negative)
        printCols[col++] = '-';
    if (digits < 6)
        while (dig < digits) {
            *--str = fillChar; //' ';
            dig++;
        }
    while (*str && col < MAX_COLS) {
        printCols[col++] = *str;
        str++;
    }

} // addInt

void UIDisplay::addLong(int32_t value, char digits) {
    if (col >= MAX_COLS)
        return;
    const bool negative = value < 0;
    uint32_t magnitude = static_cast<uint32_t>(value);
    if (negative)
        magnitude = 0UL - magnitude;
    uint8_t dig = negative;
    char buf[13]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[12];
    buf[12] = 0;
    do {
        const uint32_t quotient = magnitude / 10;
        *--str = '0' + (magnitude - 10 * quotient);
        magnitude = quotient;
        dig++;
    } while (magnitude);
    if (negative)
        printCols[col++] = '-';
    if (digits <= 11)
        while (dig < digits) {
            *--str = ' ';
            dig++;
        }
    while (*str && col < MAX_COLS) {
        printCols[col++] = *str;
        str++;
    }

} // addLong

const float roundingTable[] PROGMEM = { 0.5, 0.05, 0.005, 0.0005, 0.00005 };
void UIDisplay::addFloat(float number, char fixdigits, uint8_t digits) {
    if (col >= MAX_COLS)
        return;
    // Handle negative numbers
    if (number < 0.0) {
        printCols[col++] = '-';
        if (col >= MAX_COLS)
            return;
        number = -number;
        fixdigits--;
    }

    digits = (digits <= 4 ? digits : 4);
    number += pgm_read_float(&roundingTable[digits]); // for correct rounding

    // The signed 32-bit formatter needs a finite, representable integer part.
    if (!(number < 2147483648.0f)) {
        printCols[col++] = '?';
        return;
    }
    const int32_t int_part = static_cast<int32_t>(number);
    float remainder = number - float(int_part);
    addLong(int_part, fixdigits);
    if (col >= UI_COLS)
        return;

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        printCols[col++] = '.';
    }

    // Extract digits from the remainder one at a time
    while (col < MAX_COLS && digits-- > 0) {
        remainder *= 10.0;
        uint8_t toPrint = uint8_t(remainder);
        printCols[col++] = '0' + toPrint;
        remainder -= toPrint;
    }

} // addFloat

void UIDisplay::addStringP(FSTRINGPARAM(text)) {
    while (col < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0)
            return;
        printCols[col++] = c;
    }

} // addStringP

FSTRINGVALUE(ui_text_on, UI_TEXT_ON)
FSTRINGVALUE(ui_text_off, UI_TEXT_OFF)
FSTRINGVALUE(ui_text_0, UI_TEXT_0)
FSTRINGVALUE(ui_text_1, UI_TEXT_1)
FSTRINGVALUE(ui_text_na, UI_TEXT_NA)
FSTRINGVALUE(ui_yes, UI_TEXT_YES)
FSTRINGVALUE(ui_no, UI_TEXT_NO)
FSTRINGVALUE(ui_ok, UI_TEXT_OK)
FSTRINGVALUE(ui_fail, UI_TEXT_FAIL)
FSTRINGVALUE(ui_neetfix, UI_TEXT_O_SCAN_NEEDFIX)
FSTRINGVALUE(ui_up, UI_TEXT_UP)
FSTRINGVALUE(ui_down, UI_TEXT_DOWN)
FSTRINGVALUE(ui_selected, UI_TEXT_SEL)
FSTRINGVALUE(ui_unselected, UI_TEXT_NOSEL)
FSTRINGVALUE(ui_text_z_mode_min, UI_TEXT_Z_MODE_MIN)
FSTRINGVALUE(ui_text_z_mode_surface, UI_TEXT_Z_MODE_SURFACE)
FSTRINGVALUE(ui_text_z_mode_gcode, UI_TEXT_Z_MODE_GCODE)
FSTRINGVALUE(ui_text_hotend_v1, UI_TEXT_HOTEND_V1)
FSTRINGVALUE(ui_text_hotend_v2, UI_TEXT_HOTEND_V2)
FSTRINGVALUE(ui_text_z_compensation_active, UI_TEXT_Z_COMPENSATION_ACTIVE)


void UIDisplay::parse(char* txt, bool ram, bool allowMessageExpansion) {
    int ivalue = 0;
    float fvalue = 0;

    while (col < MAX_COLS) {
        char c = (ram ? *(txt++) : pgm_read_byte(txt++));
        if (c == 0)
            break; // finished
        if (c != '%') {
            printCols[col++] = c;
            continue;
        }

        // A status can be clipped at the display width in the middle of a field.
        const char c1 = ram ? *txt++ : pgm_read_byte(txt++);
        if (!c1) {
            printCols[col++] = '%';
            break;
        }
        const char c2 = ram ? *txt++ : pgm_read_byte(txt++);
        if (!c2) {
            printCols[col++] = '%';
            if (col < MAX_COLS)
                printCols[col++] = c1;
            break;
        }
        renderField(c1, c2, ram, allowMessageExpansion, ivalue, fvalue);
    }
    printCols[col] = 0;
} // parse

// Format a complete placeholder. Keep the legacy scratch values for the whole template.
void UIDisplay::renderField(char c1, char c2, bool ram, bool allowMessageExpansion, int& ivalue, float& fvalue) {
    switch (c1) {
        case '%': {
            if (c2 == '%' && col < MAX_COLS)
                printCols[col++] = '%'; // %%% : The % char
            break;
        }
        case 'V': {
            if(c2=='c') {addInt(bedCutoffCandidate_,0);break;}
            using Profile = PreheatProfiles::Profile;
            const bool profileA = c2 == 'b' || c2 == 'e';
            if (profileA || c2 == 'B' || c2 == 'E') {
                const auto temperatures = Printer::preheatProfiles.get(profileA ? Profile::A : Profile::B);
                addInt(c2 == 'b' || c2 == 'B' ? temperatures.bed : temperatures.hotend, 0);
            }
            break;
        }
        case 'a': // Acceleration settings
        {
            if (c2 == 'x')
                addFloat(Printer::printAcceleration(X_AXIS), 5, 0); // %ax : X acceleration during print moves
            else if (c2 == 'y')
                addFloat(Printer::printAcceleration(Y_AXIS), 5, 0); // %ay : Y acceleration during print moves
            else if (c2 == 'z')
                addFloat(Printer::printAcceleration(Z_AXIS), 5, 0); // %az : Z acceleration during print moves
            else if (c2 == 'X')
                addFloat(Printer::travelAcceleration(X_AXIS), 5, 0); // %aX : X acceleration during travel moves
            else if (c2 == 'Y')
                addFloat(Printer::travelAcceleration(Y_AXIS), 5, 0); // %aY : Y acceleration during travel moves
            else if (c2 == 'Z')
                addFloat(Printer::travelAcceleration(Z_AXIS), 5, 0); // %aZ : Z acceleration during travel moves
            else if (c2 == 'j')
                addFloat(Printer::xyJerkLimit(), 3, 1); // %aj : Max. jerk
            else if (c2 == 'J')
                addFloat(Printer::zJerkLimit(), 3, 2); // %aJ : Max. Z-jerk
            break;
        }
        case 'D': {
            if (c2 == 'x')
                addLong(scanStrideMm(X_AXIS), 3); // %Dx : scan step size x
            else if (c2 == 'y')
                addLong(scanStrideMm(Y_AXIS), 3); // %Dy : scan step size y
            break;
        }

        case 'H': {
            if (c2 == 'B')
                addLong(activeBedMatrixSlot(), 1); // %HB : active heat bed z matrix
            else if (c2 == 'O')
                addFloat((float)bedMatrixHighestZSteps() * Printer::mmPerStep(Z_AXIS) * 1000.0f, 3, 0); // %HO : active heat bed min z offset in um
            break;
        }

        case 'e':
        case 'E':
            renderTemperatureField(c1, c2, fvalue);
            break;

        case 'F': // FAN speed
        {
            if (c2 == 's') { // %Fs : Fan speed in Percent
                addFloat(Printer::getFanSpeed(false) / 2.55f, 3, 1);
            } else if (c2 == 'h') { // %Fh : Fan frequency in Hz/Teiler
                addFloat(15.3f / HAL::partFanDivisor(), 2, 1);
            } else if (c2 == 'm') { // %Fm : Fan modulation type PWM or PDM
                if (HAL::partFanMode() == PART_FAN_MODE_PDM)
                    addStringP(PSTR("PDM"));
                else
                    addStringP(PSTR("PWM"));
            } else if (c2 == 'U') { // %FU : Fan modulation minimum = 1% FanSpeed
                addInt(HAL::partFanMinDuty(), 3);
            } else if (c2 == 'O') { // %FO : Fan modulation maximum = 100% FanSpeed
                addInt(HAL::partFanMaxDuty(), 3);
            }
            if (c2 == 'H') // %FH : Digit Homing to Zero ON/OFF
            {
                addStringP(Printer::strainGauge.zeroingEnabled() ? ui_text_on : ui_text_off);
            }
            if (c2 == 'C') // %FC : Digits force-bend-hotend-down Compensation Z ON/OFF
            {
                addStringP(Printer::digitCompensation.digitZEnabled() ? ui_text_on : ui_text_off);
            }
            break;
        }
        case 'f': {
            if (c2 == 'x')
                addFloat(Printer::axisMaxFeedrate(X_AXIS), 5, 0); // %fx : Max. feedrate x direction
            else if (c2 == 'y')
                addFloat(Printer::axisMaxFeedrate(Y_AXIS), 5, 0); // %fy : Max. feedrate y direction
            else if (c2 == 'z')
                addFloat(Printer::axisMaxFeedrate(Z_AXIS), 5, 0); // %fz : Max. feedrate z direction
            else if (c2 == 'e')
                addFloat(extruder[0].maxFeedrate, 5, 0); // %fe : Max. feedrate extruder 0
#if NUM_EXTRUDER > 1
            else if (c2 == 'E')
                addFloat(extruder[1].maxFeedrate, 5, 0); // %fE : Max. feedrate extruder 1
#endif                                                   // NUM_EXTRUDER>1
            else if (c2 == 'X')
                addFloat(Printer::axisHomingFeedrate(X_AXIS), 5, 0); // %fX : Homing feedrate x direction
            else if (c2 == 'Y')
                addFloat(Printer::axisHomingFeedrate(Y_AXIS), 5, 0); // %fY : Homing feedrate y direction
            else if (c2 == 'Z')
                addFloat(Printer::axisHomingFeedrate(Z_AXIS), 5, 0); // %fZ : Homing feedrate z direction

            break;
        }
        case 'i': {
            if (c2 == 's')
                addLong(Printer::inactivity.stepperTimeout() / 1000, 4); // %is : Stepper inactive time in seconds
            else if (c2 == 'p')
                addLong(Printer::inactivity.shutdownTimeout() / 1000, 4); // %ip : Max. inactive time in seconds
            break;
        }
        case 'O': // ops related stuff
        {
#if NUM_EXTRUDER > 1
            if (c2 == 'a') // %Oa : Active Extruder
            {
                if (Extruder::current->id == 0) {
                    addStringP(ui_text_0);
                } else {
                    addStringP(ui_text_1);
                }
            }
#endif // NUM_EXTRUDER>1

#if NUM_EXTRUDER > 1
            if (c2 == 'E') // %OE : Extruder offset X [mm]
            {
                addFloat(extruder[1].offsetMM[X_AXIS], 4, 3);
            }
            if (c2 == 'F') // %OF : Extruder offset Y [mm]
            {
                addFloat(extruder[1].offsetMM[Y_AXIS], 4, 3);
            }
#endif // NUM_EXTRUDER>1

            break;
        }
        case 'h': {
            if (c2 == 'x' && col < MAX_COLS) // %hx : x homed
            {
                if (Printer::isAxisHomed(X_AXIS))
                    printCols[col++] = '*';
            } else if (c2 == 'y' && col < MAX_COLS) // %hy : y homed
            {
                if (Printer::isAxisHomed(Y_AXIS))
                    printCols[col++] = '*';
            } else if (c2 == 'z' && col < MAX_COLS) // %hz : z homed
            {
                if (Printer::zMotion.isHomed())
                    printCols[col++] = '*';
            } else if (c2 == 'X' && col < MAX_COLS) // %hX : x homed
            {
                if (Printer::isAxisHomed(X_AXIS))
                    printCols[col++] = ':';
                else
                    printCols[col++] = '?';
            } else if (c2 == 'Y' && col < MAX_COLS) // %hY : y homed
            {
                if (Printer::isAxisHomed(Y_AXIS))
                    printCols[col++] = ':';
                else
                    printCols[col++] = '?';
            } else if (c2 == 'Z' && col < MAX_COLS) // %hZ : z homed
            {
                if (Printer::zMotion.isHomed())
                    printCols[col++] = ':';
                else
                    printCols[col++] = '?';
            }
            break;
        }
        case 'l': {
            if (c2 == 'a')
                addInt(lastAction, 4);

            else if (c2 == 'o')
                addStringP(AccessoryOutputs::state(AccessoryOutputs::Output::CaseLight) ? ui_text_on : ui_text_off); // %lo : Lights on/off


            break;
        }

        case 'm': {
            if (c2 == 'Y' && col < MAX_COLS) // %mY : menu yes
            {
                if (confirmation_)
                    printCols[col++] = CHAR_SELECTED;
                else
                    printCols[col++] = ' ';
            } else if (c2 == 'N' && col < MAX_COLS) // %mN : menu no
            {
                if (confirmation_)
                    printCols[col++] = ' ';
                else
                    printCols[col++] = CHAR_SELECTED;
            } else if (!ram && allowMessageExpansion && c2 == '1') // %m1 : message line 1
            {
                if (messageLine1 != 0) {
                    col = 0;                                   // Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine1, false, false); //addStringP((char PROGMEM *)messageLine1);
                }
                break;
            } else if (!ram && allowMessageExpansion && c2 == '2') // %m2 : message line 2
            {
                if (messageLine2 != 0) {
                    col = 0;                                   // Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine2, false, false); //addStringP((char PROGMEM *)messageLine2);
                }
                break;
            } else if (!ram && allowMessageExpansion && c2 == '3') // %m3 : message line 3
            {
                if (messageLine3 != 0) {
                    col = 0;                                   // Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine3, false, false); //addStringP((char PROGMEM *)messageLine3);
                }
                break;
            } else if (!ram && allowMessageExpansion && c2 == '4') // %m4 : message line 4
            {
                if (messageLine4 != 0) {
                    col = 0;                                   // Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine4, false, false); //addStringP((char PROGMEM *)messageLine4);
                }
                break;
            }
            break;
        }

        case 'M': {
            if (c2 == 'X' && col < MAX_COLS) // %MX : Motorcurrent X
            {
                addFloat(Drv8711::configuredCurrent(X_AXIS) / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                addInt((int)Drv8711::configuredCurrent(X_AXIS), 3);
            } else if (c2 == 'Y' && col < MAX_COLS) // %MY : Motorcurrent Y
            {
                addFloat(Drv8711::configuredCurrent(Y_AXIS) / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                addInt((int)Drv8711::configuredCurrent(Y_AXIS), 3);
            } else if (c2 == 'Z') // %MZ : Motorcurrent Z
            {
                addFloat(Drv8711::configuredCurrent(Z_AXIS) / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                addInt((int)Drv8711::configuredCurrent(Z_AXIS), 3);
            } else if (c2 == '0') // %M0 : Motorcurrent T0
            {
                addFloat(Drv8711::configuredCurrent(E_AXIS) / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                if (Printer::pause.isWarmPause())
                    addStringP(PSTR(UI_TEXT_PAUSED));
                else
                    addInt((int)Drv8711::configuredCurrent(E_AXIS), 3);
            } else if (c2 == '1') // %M1 : Motorcurrent T1
            {
                addFloat(Drv8711::configuredCurrent(E_AXIS + 1) / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                if (Printer::pause.isWarmPause())
                    addStringP(PSTR(UI_TEXT_PAUSED));
                else
                    addInt((int)Drv8711::configuredCurrent(E_AXIS + 1), 3);
            }
            break;
        }

        case 'o': {
            if (c2 == 's') {
                if (!ram) {
                    if (statusIsTemplate_) {
                        parse(statusMsg, true, false);
                    } else {
                        for (const char* text = statusMsg; col < MAX_COLS && *text; ++text)
                            printCols[col++] = *text;
                    }
                }
                break;
            }
            if (c2 == 'f') // %of : flow multiplier
            {
                addInt(100 * Printer::flowMultiplier()
                           * Printer::digitCompensation.extrusionFactor()
                       ,
                       3);
                break;
            }
            if (c2 == 'm') // %om : Speed multiplier
            {
                addInt(Printer::feedrateMultiplier() * Printer::digitCompensation.feedrateFactor(), 3);
                break;
            }
            if (c2 == 'M') // %oM : Printer::requestedFeedrate()
            {
                addInt(Printer::requestedFeedrate(), 3);
                break;
            }
            if (c2 == 'v') // %ov : Active Speed
            {
                addFloat(StepExecutor::nominalFeedrate() * Printer::digitCompensation.feedrateFactor(), 3, 1);
                break;
            }
            if (c2 == 'p') // %op : Is single double or quadstepping?
            {
                addInt(StepExecutor::stepsPerInterrupt(), 1);
                addStringP(PSTR("ST"));

                break;
            }

#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
            if (c2 == 'u') {
                addStringP(AccessoryOutputs::state(AccessoryOutputs::Output::Mains) ? ui_text_on : ui_text_off); // %ou : 230V output on/off
                break;
            }

            if (c2 == 'l') {
                addStringP(AccessoryOutputs::state(AccessoryOutputs::Output::Fet1) ? ui_text_on : ui_text_off); // %ol : FET 1 X42 output on/off
                break;
            }

            if (c2 == 'n') {
                addStringP(AccessoryOutputs::state(AccessoryOutputs::Output::Fet2) ? ui_text_on : ui_text_off); // %on : FET 2 X44 output on/off
                break;
            }
#endif // MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2

            // Extruder output level
            if (c2 >= '0' && c2 <= '9') // %o0..9 : Output level extruder 0..9 is % including %sign
            {
                ivalue = HAL::outputDuty(c2 - '0');
            }

            else if (c2 == 'b') // %ob : Output level heated bed
            {
                ivalue = HAL::outputDuty(heatedBedController.pwmIndex);
            }

            else if (c2 == 'C') // %oC : Output level current extruder
            {
                ivalue = HAL::outputDuty(Extruder::current->id);
            }

            ivalue = (ivalue * 100) / 255;
            addInt(ivalue, 3);
            if (col < MAX_COLS)
                printCols[col++] = '%';
            break;
        }
        case 'x':
        case 'y':
        case 'z':
            renderPositionField(c1, c2, fvalue);
            break;

        case 'X':
            renderExtruderField(c1, c2);
            break;

        case 's':
        case 'S':
        case 'p':
            renderCalibrationField(c1, c2);
            break;

        case 'P': {
            if (c2 == 'N')
                addStringP(PSTR(UI_PRINTER_NAME)); // %PN : Printer name
            break;
        }
        case 'U': // %U1: Page1
        {
            if (c2 == '1') // temperature icon
            {
                if (!normalCharacters_)
                    loadNormalCharacters();
                addStringP(PSTR("\005"));
            } else if (c2 == '2') // replace line with new line
            {
            }
            break;
        }
        case 'C':
        case 'L':
        case 'w':
            renderCompensationField(c1, c2);
            break;

    }
}

void UIDisplay::renderTemperatureField(char c1, char c2, float& fvalue) {
    switch (c1) {
        case 'e': // Extruder temperature
        {
            if (c2 == 'r') // %er : Extruder relative mode
            {
                addStringP(Printer::gcodeModes.relativeEOverride() ? ui_yes : ui_no);
                break;
            }


            if (c2 == 'w') { //%ew : tell me which sensor(s) are defect.
                bool addone = false;
                for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
                    TemperatureController* act = tempController[controller];
                    if (act->isSensorDefect() || act->isSensorDecoupled()) {
                        if (addone)
                            addStringP(Com::tSlash);
                        addone = true;
                        if (controller < NUM_EXTRUDER) {
                            if (col < MAX_COLS)
                                printCols[col++] = 'E';
                            addInt(controller, 1);
                        } else if (controller <= NUM_EXTRUDER) {
                            addStringP(PSTR("Bed"));
                        } else {
                            addStringP(PSTR("Opt"));
                        }
                    }
                }
                break;
            }

            if (Printer::isAnyTempsensorDefect()) {
                uint8_t countDefect = 0;
                uint8_t countDecoupled = 0;
                for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
                    TemperatureController* act = tempController[controller];
                    if (act->isSensorDefect()) {
                        countDefect++;
                    }
                    if (act->isSensorDecoupled()) {
                        countDecoupled++;
                    }
                }
                if (countDecoupled > countDefect) {
                    addStringP(PSTR("dec"));
                } else {
                    addStringP(PSTR("def"));
                }
                break;
            }

            if (!(Printer::temperatureReadingsAvailable())) {
                // avoid to show the current temperatures before we have measured them
                addStringP(PSTR(" ? "));
                break;
            }

            if (c2 == 'c') {
                fvalue = Extruder::current->tempControl.currentTemperatureC; // %ec : Current extruder temperature
            } else if (c2 >= '0' && c2 <= '9') {
                uint8_t nr = c2 - '0';
                fvalue = extruder[nr].tempControl.currentTemperatureC; // %e0..9 : Temp. of extruder 0..9
            }
            else if (c2 == 'b') {
                fvalue = Extruder::getHeatedBedTemperature(); // %eb : Current heated bed temperature
            }
            addFloat(fvalue, 3, 0);
            break;
        }
        case 'E': // Target extruder temperature
        {

            if (c2 == 'c') {
                fvalue = Extruder::current->tempControl.targetTemperatureC; // %Ec : Target temperature of current extruder
            } else if (c2 >= '0' && c2 <= '9') {
                fvalue = extruder[c2 - '0'].tempControl.targetTemperatureC; // %E0-9 : Target temperature of extruder 0..9
            }
            else if (c2 == 'b') {
                fvalue = heatedBedController.targetTemperatureC; // %Eb : Target temperature of heated bed
            }
            addFloat(fvalue, 3, 0);
            break;
        }

    }
}

void UIDisplay::renderPositionField(char c1, char c2, float& fvalue) {
    switch (c1) {
        case 'x': {
            char bDefect = false;

            if (c2 >= '0' && c2 <= '6') {
                if (c2 == '0') // %x0 : X position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::xyMotion.nominalSteps(X_AXIS) * Printer::mmPerStep(X_AXIS);
                    }
                } else if (c2 == '1') // %x1 : Y position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::xyMotion.nominalSteps(Y_AXIS) * Printer::mmPerStep(Y_AXIS);
                    }
                } else if (c2 == '2') // %x2 : Z position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::currentZPositionMM();
                    }
                } else if (c2 == '3') // %x3 : X offset position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::directSteps(X_AXIS) * Printer::mmPerStep(X_AXIS);
                    }
                } else if (c2 == '4') // %x4 : Y offset position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::directSteps(Y_AXIS) * Printer::mmPerStep(Y_AXIS);
                    }
                } else if (c2 == '5') // %x5 : Z offset position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        // Wenn nur CurrentSteps[Z_AXIS] ausgegeben wird, ist auch interssant, viel die differenz aller anderer versätze ist:
                        if (Printer::zDisplayMode() == Z_VALUE_MODE_Z_MIN) { //->Z_VALUE_MODE_LAYER
                            if (col < MAX_COLS)
                                printCols[col++] = 'm';
                            if (col < MAX_COLS)
                                printCols[col++] = 'm';
                        } else {
                            fvalue = Printer::zMotion.motorOffsetSteps() * Printer::mmPerStep(Z_AXIS);
                            if (col < MAX_COLS)
                                printCols[col++] = ' ';
                            addFloat(fvalue, 3, 2);
                        }
                        break;
                    }
                } else // %x6 : Current extruder position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::moveTarget.machineMM(E_AXIS);
                    }
                }

                if (bDefect) {
                    addStringP(PSTR(" def"));
                } else {
                    addFloat(fvalue, 3, 2);
                }
            }
            break;
        }
        case 'y': {
            break;
        }
        case 'z': // %z0 : Z Offset
        {
            if (c2 == '0') {
                addInt(Printer::zOffset.manualMicrometers(), 4);
            } else if (c2 == 'm') // %zm : Z Scale
            {
                addStringP(Printer::zDisplayMode() == Z_VALUE_MODE_Z_MIN ? ui_text_z_mode_min : ui_text_z_mode_gcode);
            } else if (c2 == 's') // %zs : Z-Schraube korrektur mm
            {
                addFloat(ZScrewAdjustment::current().millimetres, 1, 3);
            } else if (c2 == 'S') // %zS : Z-Schraube korrektur Umdrehungen
            {
                addFloat(ZScrewAdjustment::current().turns, 1, 1);
            } else if (c2 == 'F') // %zF : Z-Schraube falsch
            {
                if (ZScrewAdjustment::current().status == ZScrewAdjustment::Status::Ok)
                    addStringP(ui_ok); //ok
                else if (ZScrewAdjustment::current().status == ZScrewAdjustment::Status::BedTooHigh)
                    addStringP(ui_neetfix); // Zu hohes bett
                else
                    addStringP(ui_fail); //fail
            } else if (c2 == 'D')        // %zD : Z-Schraube richtung
            {
                if (ZScrewAdjustment::current().millimetres > 0)
                    addStringP(ui_up);
                else
                    addStringP(ui_down);
            }
            break;
        }
    }
}

void UIDisplay::renderExtruderField(char c1, char c2) {
    switch (c1) {
        case 'X': // Extruder related
        {
            if (c2 >= '0' && c2 <= '9') // %X0..9 : Extruder selected marker
            {
                addStringP(Extruder::current->id == c2 - '0' ? ui_selected : ui_unselected);
            } else if (c2 == 'm') // %Xm : PID drive min
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(-1 * tempController[menuPos[menuLevel - 1]]->pidDriveMin, 3);
                } else {
                    addInt(Extruder::current->tempControl.pidDriveMin, 3);
                }
            } else if (c2 == 'M') // %XM : PID drive max
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(tempController[menuPos[menuLevel - 1]]->pidDriveMax, 3);
                } else {
                    addInt(Extruder::current->tempControl.pidDriveMax, 3);
                }
            } else if (c2 == 'D') // %XD : PID max
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(tempController[menuPos[menuLevel - 1]]->pidMax * 100 / 255, 3);
                } else {
                    addInt(Extruder::current->tempControl.pidMax * 100 / 255, 3);
                }
            } else if (c2 == 'S') // %XS : Temperature Sensor
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(tempController[menuPos[menuLevel - 1]]->sensorType, 2); // Mit type 100 wärens 3 zeichen, aber das kommt in praxis nicht vor.
                    switch (tempController[menuPos[menuLevel - 1]]->sensorType) {
                    case 1: {
                        addStringP(PSTR(UI_TEXT_SENSOR_1));
                        break;
                    }
                    case 3: {
                        addStringP(PSTR(UI_TEXT_SENSOR_3));
                        break;
                    }
                    case 4: {
                        addStringP(PSTR(UI_TEXT_SENSOR_4));
                        break;
                    }
                    case 8: {
                        addStringP(PSTR(UI_TEXT_SENSOR_8));
                        break;
                    }
                    case 13: {
                        addStringP(PSTR(UI_TEXT_SENSOR_13));
                        break;
                    }
                    }
                } else {
                    addInt(Extruder::current->tempControl.sensorType, 3);
                }
            }

            else if (c2 == 't') // %Xt : autotune profile confirmation
            {
                col = 0;
                if (menuPos[menuLevel - 1] == 0)
                    addStringP(PSTR("Less overshoot"));
                else
                    addStringP(PSTR("Faster response"));
            }

            else if (c2 == 'l') // %Xl : Advance L value
            {
                addFloat(Extruder::current->advanceL, 3, 0);
            } else if (c2 == 'b') // %Xb : E0 Advance L value
            {
                addFloat(extruder[0].advanceL, 3, 0);
            } else if (c2 == 'c') // %Xc : E1 Advance L value
            {
                addFloat(extruder[1].advanceL, 3, 0);
            }

            else if (c2 == 'f') // %Xf : Extruder 0 Acceleration
            {
                addFloat(extruder[0].maxAcceleration, 5, 0);
            }
#if NUM_EXTRUDER > 1
            else if (c2 == 'F') // %XF : Extruder 1 Acceleration
            {
                addFloat(extruder[1].maxAcceleration, 5, 0);
            }
#endif                          //NUM_EXTRUDER>1
            else if (c2 == 'h') // %Xh : Extruder 0 maxEJerk
            {
                addFloat(extruder[0].maxEJerk, 5, 0);
            }
#if NUM_EXTRUDER > 1
            else if (c2 == 'H') // %XH : Extruder 1 maxEJerk
            {
                addFloat(extruder[1].maxEJerk, 5, 0);
            }
#endif //NUM_EXTRUDER>1

            else if (c2 == 'E') // %XE : Extruder Stepper Microsteps
            {
                addInt(drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(E_AXIS)), 3);
            }
            else if (c2 == 'g') // %Xg : StepExecutor::packingInterval()
            {
                addInt(StepExecutor::packingInterval(), 4);
            }
            else if (c2 == 'x') // %Xx : XY Stepper Microsteps
            {
                addInt(drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(X_AXIS)), 3);
            } else if (c2 == 'z') // %Xz : Z Stepper Microsteps
            {
                addInt(drv8711ModeValue_2_MicroSteps(Drv8711::microstepMode(Z_AXIS)), 3);
            }
            break;
        }
    }
}

void UIDisplay::renderCalibrationField(char c1, char c2) {
    switch (c1) {
        case 's': // Endstop positions
        {
            if (c2 == 'x') // %sx : State of x min endstop
            {
                addStringP(Printer::isXMinEndstopHit() ? ui_text_on : ui_text_off);
            }
            if (c2 == 'X') // %sX : State of x max endstop
                addStringP(ui_text_na);

            if (c2 == 'y') // %sy : State of y min endstop
                addStringP(Printer::isYMinEndstopHit() ? ui_text_on : ui_text_off);

            if (c2 == 'Y') // %sY : State of y max endstop
                addStringP(ui_text_na);

            if (c2 == 'z') // %sz : State of z min endstop
            {
                addStringP(Printer::isZMinEndstopHit() ? ui_text_on : ui_text_off);
            }
            if (c2 == 'Z') // %sZ : State of z max endstop
            {
#if MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
                addStringP(Printer::isZMaxEndstopHit() ? ui_text_on : ui_text_off);
#else
                addStringP(ui_text_na);
#endif // RF2000 / RF2000v2
            }
            if (c2 == 'C') // %sC : State of the z compensation
            {
                if (Printer::doHeatBedZCompensation) {
                    addStringP(ui_text_z_compensation_active);
                }

            }

            if (c2 == 'S') // %sS : State of the sensible offset
            {
                if (Printer::doHeatBedZCompensation) {
                    addInt(Printer::zOffset.senseMicrometers(), 4);
                } else {
                    addInt(0, 4);
                }
            }

            if (c2 == 'M') // %sM : State of the sensible offset
            {
                const int16_t threshold = Printer::senseOffsetControl.limits().thresholdDigits;
                if (Printer::doHeatBedZCompensation && threshold > 0) {
                    if (Printer::senseOffsetControl.inLearningRange() && Printer::pause.allowsQueueAdvance())
                        addStringP(PSTR("^"));
                    else
                        addStringP(PSTR("@"));
                    addInt(threshold, 5);
                } else {
                    addStringP(ui_text_off);
                }
            }
            if (c2 == 'm') // %sm : State of the sensible pressure in eeprom
            {
                addInt(Printer::senseOffsetControl.storedLimits().thresholdDigits, 5);
            }
            if (c2 == 'o') // %so : State of the sensible pressure maxoffset in eeprom
            {
                addInt(Printer::senseOffsetControl.storedLimits().maxMicrometers, 5);
            }
            if (c2 == 'a') // %sa : State of the sensible pressure autostarter
            {
                addStringP(Printer::senseOffsetControl.autoStart() ? ui_text_on : ui_text_off);
            }

            if (c2 == '1') // %s1 : current value of the strain gauge
            {
                addInt(Printer::strainGauge.lastKnownDigits(), 5);
            }

            if (c2 == 'k') // %sk : Positioning coordinate system for XY (move by gode or directoffset)
            {
                if (Printer::manualMotion.coordinateSystem() == KOSYS_GCODE) {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                } else /* if (Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET) */ {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                }
            }

            if (c2 == 'K') // %sK : Feedrate source config for direct positioning moves
            {
                if (Printer::manualMotion.feedrateSource() == FEEDRATE_DIRECTCONFIG) {
                    addStringP(PSTR(UI_TEXT_MOVE_FEEDRATE_STANDARD));
                } else /* if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) */ {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                }
            }

            if (c2 == 'f') // %sf : Feedrate for menu postition xy
            {
                if (Printer::manualMotion.feedrateSource() == FEEDRATE_DIRECTCONFIG) {
                    addInt(STANDARD_POSITION_FEEDRATE_XY, 3);
                } else /* if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE) */ {
                    addInt(Printer::requestedFeedrate(), 3);
                }
                addStringP(PSTR("mm/s"));
            }
            if (c2 == 'F') // %sF : Feedrate for menu postition z
            {
                if (Printer::manualMotion.feedrateSource() == FEEDRATE_GCODE && Printer::requestedFeedrate() <= STANDARD_POSITION_FEEDRATE_Z) {
                    addInt(Printer::requestedFeedrate(), 3);
                } else {
                    addInt(STANDARD_POSITION_FEEDRATE_Z, 3);
                }
                addStringP(PSTR("mm/s"));
            }

            break;
        }
        case 'S': {
            if (c2 == '0')
                addFloat(extruder[0].stepsPerMM, 3, 0); // %S0 : Steps per mm extruder0
            else if (c2 == '1')
                addFloat(extruder[1].stepsPerMM, 3, 0); // %S1 : Steps per mm extruder1
            else if (c2 == 'e')
                addFloat(Extruder::current->stepsPerMM, 3, 0); // %Se : Steps per mm current extruder
            else if (c2 == 'z')
                addFloat(Printer::manualMotion.steps(Z_AXIS) * Printer::mmPerStep(Z_AXIS) * 1000.0f, 4, 1); // %Sz : Mikrometer per Z-Single_Step (Z_Axis)
            else if (c2 == 'M' && col < MAX_COLS) {
                if (bedMatrixHasUnsavedChanges())
                    printCols[col++] = '*';
            } // %SM : Matrix has changed in Ram and is ready to Save. -> *)
            else if (c2 == 's')
                addFloat(scanStartLiftMm(), 1, 1); // %Ss : active current value of defaultHeatBedScanLiftMm
            break;
        }
        case 'p': {
            if (c2 == 'x') // %px: mode of the Position X menu
            {
                switch (Printer::manualMotion.mode(X_AXIS)) {
                case MOVE_MODE_SINGLE_STEPS: {
                    addFloat(Printer::manualMotion.steps(X_AXIS) * Printer::mmPerStep(X_AXIS) * 1000.0f, 1, 0);
                    addStringP(PSTR(" um"));
                    break;
                }
                case MOVE_MODE_1_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_1_MM));
                    break;
                }
                case MOVE_MODE_10_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_10_MM));
                    break;
                }
                case MOVE_MODE_50_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_50_MM));
                    break;
                }
                case MOVE_MODE_SINGLE_MOVE: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                    break;
                }
                }
                if (Printer::manualMotion.mode(X_AXIS) == MOVE_MODE_SINGLE_MOVE || Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET || Printer::pause.isPaused()) {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                    addStringP(PSTR(")"));
                } else {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                    addStringP(PSTR(")"));
                }
                break;
            }
            if (c2 == 'y') // %py: mode of the Position Y menu
            {
                switch (Printer::manualMotion.mode(Y_AXIS)) {
                case MOVE_MODE_SINGLE_STEPS: {
                    addFloat(Printer::manualMotion.steps(Y_AXIS) * Printer::mmPerStep(Y_AXIS) * 1000.0f, 1, 0);
                    addStringP(PSTR(" um"));
                    break;
                }
                case MOVE_MODE_1_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_1_MM));
                    break;
                }
                case MOVE_MODE_10_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_10_MM));
                    break;
                }
                case MOVE_MODE_50_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_50_MM));
                    break;
                }
                case MOVE_MODE_SINGLE_MOVE: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                    break;
                }
                }
                if (Printer::manualMotion.mode(Y_AXIS) == MOVE_MODE_SINGLE_MOVE || Printer::manualMotion.coordinateSystem() == KOSYS_DIRECTOFFSET || Printer::pause.isPaused()) {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                    addStringP(PSTR(")"));
                } else {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                    addStringP(PSTR(")"));
                }
                break;
            }
            if (c2 == 'z') // %pz: mode of the Position Z menu
            {
                if (Printer::pause.isPaused()) {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                } else {
                    switch (Printer::manualMotion.mode(Z_AXIS)) {
                    case MOVE_MODE_SINGLE_STEPS: {
                        addFloat(Printer::manualMotion.steps(Z_AXIS) * Printer::mmPerStep(Z_AXIS) * 1000.0f, 1, 1);
                        addStringP(PSTR(" um"));
                        break;
                    }
                    case MOVE_MODE_1_MM: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_1_MM));
                        break;
                    }
                    case MOVE_MODE_10_MM: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_10_MM));
                        break;
                    }
                    case MOVE_MODE_50_MM: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_50_MM));
                        break;
                    }
                    case MOVE_MODE_SINGLE_MOVE: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                        break;
                    }
                    }
                }

                // Z axis is always direct offset move because gcode does not make sense here.

                //{
                addStringP(PSTR(" ("));
                addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                addStringP(PSTR(")"));
                //}
                //else {
                //	addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                //}
                break;
            }
            if (c2 == 'l') // Emergency pause minimum in DMS digits.
            {
                addLong(Printer::pressureProtection.pauseLimits().minimum, 6);
                break;
            }
            if (c2 == 'h') // Emergency pause maximum in DMS digits.
            {
                addLong(Printer::pressureProtection.pauseLimits().maximum, 6);
                break;
            }
            if (c2 == 'L') // Z stop and extrusion inhibit minimum in DMS digits.
            {
                addLong(Printer::pressureProtection.stopLimits().minimum, 6);
                break;
            }
            if (c2 == 'H') // Z stop and extrusion inhibit maximum in DMS digits.
            {
                addLong(Printer::pressureProtection.stopLimits().maximum, 6);
                break;
            }
            break;
        }
    }
}

void UIDisplay::renderCompensationField(char c1, char c2) {
    switch (c1) {
        case 'C': {
            if (c2 == 'U') // %CU : digit flow lower limit
            {
                addInt(Printer::digitCompensation.flowProfile().minDigits, 5);
            } else if (c2 == 'O') // %CO : digit flow higher limit
            {
                addInt(Printer::digitCompensation.flowProfile().maxDigits, 5);
            } else if (c2 == 'F') // %CF : digit flow flowrate
            {
                addInt(Printer::digitCompensation.flowProfile().extrusionPercent, 3);
            } else if (c2 == 'E')                                 // %CE : digit flow feedrate
            {
                addInt(Printer::digitCompensation.flowProfile().speedPercent, 3);
            }
            break;
        }
        case 'L': {
            const auto& layers = Printer::layers;
            if (c2 == 'L') // %LL : Last Layer (Queue)
            {
                addFloat(layers.previousLayerSteps() * Printer::mmPerStep(Z_AXIS), 3, 2);
                break;
            } else if (c2 == 'C') // %LC : Current Layer (Queue)
            {
                addFloat(layers.currentLayerSteps() * Printer::mmPerStep(Z_AXIS), 3, 2);
                break;
            } else if (c2 == 'H') // %LH : Layer Height
            {
                addFloat((layers.currentLayerSteps() - layers.previousLayerSteps()) * Printer::mmPerStep(Z_AXIS), 1, 2);
                break;
            } else if (c2 == 'P') // %LP : ECMP %
            {
                addFloat(Printer::extrusionCompensation.ratio() * 100, 1, 2);
                break;
            } else if (c2 == 'm') // %Lm : Fade start
            {
                addFloat(layers.fadeStartSteps() * Printer::mmPerStep(Z_AXIS), 1, 2);
                break;
            } else if (c2 == 'M') // %LM : Fade end
            {
                addFloat(layers.fadeEndSteps() * Printer::mmPerStep(Z_AXIS), 2, 2);
                break;
            }
            break;
        }
        case 'w': //wobblefix
        {
            if (c2 == 'x') // %wx : current wobblefix offset in x [um] (Bauchtanz)
            {
                addInt(Printer::wobbleCorrection.offsetMM(X_AXIS) * 1000, 4);
                break;
            } else if (c2 == 'y') // %wy : current wobblefix offset in y [um] (Bauchtanz)
            {
                addInt(Printer::wobbleCorrection.offsetMM(Y_AXIS) * 1000, 4);
                break;
            }

            else if (c2 == 'a') // %wa : current wobblefix amplitude for X
            {
                addInt(Printer::wobbleCorrection.profile().xMicrometers, 4);
                break;
            } else if (c2 == 'b') // %wb : current wobblefix amplitude for Y(x_0)
            {
                addInt(Printer::wobbleCorrection.profile().yAtXMinMicrometers, 4);
                break;
            } else if (c2 == 'c') // %wc : current wobblefix amplitude for Y(x_245)
            {
                addInt(Printer::wobbleCorrection.profile().yAtXMaxMicrometers, 4);
                break;
            } else if (c2 == 'P') // %wP : current wobblefix phase for YX-wobble (Bauchtanz)
            {
                addInt(long(float(Printer::wobbleCorrection.profile().phase) * 1.8f), 4);
                break;
            }

            break;
        }
    }
}

void UIDisplay::setStatusP(PGM_P txt, bool error) {
    if (!error && Printer::isUIErrorMessage()) {
        return;
    }

    uint8_t i = 0;
    while (i < UI_COLS) {
        uint8_t c = pgm_read_byte(txt++);
        if (!c)
            break;
        statusMsg[i++] = c;
    }
    statusMsg[i] = 0;
    statusIsTemplate_ = true;
    if (error)
        Printer::setUIErrorMessage(true);

} // setStatusP

void UIDisplay::setStatus(char* txt, bool error) {
    if (!error && Printer::isUIErrorMessage())
        return;

    uint8_t i = 0;
    while (*txt && i < UI_COLS)
        statusMsg[i++] = *txt++;
    statusMsg[i] = 0;
    statusIsTemplate_ = false;

    if (error)
        Printer::setUIErrorMessage(true);

} // setStatus
