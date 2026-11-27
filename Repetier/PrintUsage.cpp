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
int32_t totalHeatedSeconds(int32_t saved,uint32_t extraSeconds) {
    const uint32_t total = (saved > 0 ? static_cast<uint32_t>(saved) : 0) + extraSeconds;
    return total > INT32_MAX ? INT32_MAX : static_cast<int32_t>(total);
}
}

void PrintUsage::finishHeating(uint32_t now) {
    if (pendingFilamentMm_ == 0.0f)
        return;

    const uint32_t elapsedSeconds = (now - heatingStartedAt_) / 1000;
    auto totals=EEPROM::usageTotals();
    totals.seconds=totalHeatedSeconds(totals.seconds,elapsedSeconds);
    totals.metres+=pendingFilamentMm_*0.001f;
    pendingFilamentMm_ = 0.0f;
    EEPROM::storeUsageTotals(totals);
    Commands::reportPrinterUsage();
}

int32_t PrintUsage::heatedSeconds() const {
    uint32_t elapsedSeconds = 0;
    if (Extruder::hasActiveHotendTarget())
        elapsedSeconds = (HAL::timeInMilliseconds() - heatingStartedAt_) / 1000;
    return totalHeatedSeconds(EEPROM::usageTotals().seconds,elapsedSeconds);
}

float PrintUsage::filamentMetres() const {
    return EEPROM::usageTotals().metres + pendingFilamentMm_ * 0.001f;
}
