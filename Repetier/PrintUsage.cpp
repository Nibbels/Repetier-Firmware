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

#include "HAL.h"
#include "PrintUsage.h"
#include "Extruder.h"
#include "Commands.h"
#include "Eeprom.h"

namespace {
int32_t totalHeatedSeconds(uint32_t extraSeconds) {
    const int32_t saved = HAL::eprGetInt32(EPR_PRINTING_TIME);
    const uint32_t total = (saved > 0 ? static_cast<uint32_t>(saved) : 0) + extraSeconds;
    return total > INT32_MAX ? INT32_MAX : static_cast<int32_t>(total);
}
}

void PrintUsage::finishHeating(uint32_t now) {
    if (pendingFilamentMm_ == 0.0f)
        return;

    const uint32_t elapsedSeconds = (now - heatingStartedAt_) / 1000;
    HAL::eprSetInt32(EPR_PRINTING_TIME, totalHeatedSeconds(elapsedSeconds));
    HAL::eprSetFloat(EPR_PRINTING_DISTANCE, filamentMetres());
    pendingFilamentMm_ = 0.0f;
    EEPROM::updateChecksum();
    Commands::reportPrinterUsage();
}

int32_t PrintUsage::heatedSeconds() const {
    uint32_t elapsedSeconds = 0;
    if (Extruder::hasActiveHotendTarget())
        elapsedSeconds = (HAL::timeInMilliseconds() - heatingStartedAt_) / 1000;
    return totalHeatedSeconds(elapsedSeconds);
}

float PrintUsage::filamentMetres() const {
    return HAL::eprGetFloat(EPR_PRINTING_DISTANCE) + pendingFilamentMm_ * 0.001f;
}
