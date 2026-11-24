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
#include "CaseFan.h"

#if (MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2) && CASE_FAN_PIN > -1
namespace CaseFan {

namespace {
millis_t offDelayMs = CASE_FAN_OFF_DELAY;
millis_t offStartedAtMs = 0;
bool offPending = false;
bool automaticOnInhibited = false;

void requestOn() {
    offPending = false;
    WRITE(CASE_FAN_PIN, HIGH);
}
}

void initialize() {
    SET_OUTPUT(CASE_FAN_PIN);
    WRITE(CASE_FAN_PIN, LOW);
}

void turnOn() {
    automaticOnInhibited = false;
    requestOn();
}

bool requestOff(bool inhibitAutomatic) {
    if (inhibitAutomatic)
        automaticOnInhibited = true;
    offPending = offDelayMs != 0;
    if (offPending) {
        offStartedAtMs = HAL::timeInMilliseconds();
    } else {
        WRITE(CASE_FAN_PIN, LOW);
    }
    return offPending;
}

void setOffDelay(uint32_t delayMs) { offDelayMs = delayMs; }
uint32_t offDelay() { return offDelayMs; }

void updateFromHotendTargets() {
    if (automaticOnInhibited) {
        offPending = false;
        offDelayMs = 0;
        return;
    }
    for (uint8_t i = 0; i < NUM_EXTRUDER; ++i) {
        if (tempController[i]->targetTemperatureC > CASE_FAN_ON_TEMPERATURE) {
            requestOn();
            return;
        }
    }
    requestOff(false);
}

void service() {
    if (!offPending || HAL::timeInMilliseconds() - offStartedAtMs <= offDelayMs)
        return;
    offPending = false;
    if (!automaticOnInhibited)
        WRITE(CASE_FAN_PIN, LOW);
}

} // namespace CaseFan
#endif
