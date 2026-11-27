#include "PrintUsage.h"
#include <cassert>
#include <cmath>

enum { EPR_PRINTING_TIME, EPR_PRINTING_DISTANCE };

namespace HAL {
int32_t seconds = 60;
float metres = 1.25f;
uint32_t now = 0;
uint32_t timeInMilliseconds() { return now; }
int32_t eprGetInt32(int offset) { assert(offset == EPR_PRINTING_TIME); return seconds; }
float eprGetFloat(int offset) { assert(offset == EPR_PRINTING_DISTANCE); return metres; }
void eprSetInt32(int offset, int32_t time) { assert(offset == EPR_PRINTING_TIME); seconds = time; }
void eprSetFloat(int offset, float distance) { assert(offset == EPR_PRINTING_DISTANCE); metres = distance; }
}
namespace Extruder {
bool heating = false;
bool hasActiveHotendTarget() { return heating; }
}
namespace EEPROM {
struct UsageTotals {int32_t seconds;float metres;};
UsageTotals usageTotals();void storeUsageTotals(UsageTotals);
unsigned saves = 0;
void updateChecksum() { ++saves; }
}
#include "usage-storage-firmware.inc"
namespace Printer {
PrintUsage printUsage;
}
namespace Commands {
void reportPrinterUsage() {
    assert(Printer::printUsage.heatedSeconds() == HAL::seconds);
    assert(Printer::printUsage.filamentMetres() == HAL::metres);
}
}

#include "print-usage-firmware.inc"

int main() {
    auto& usage = Printer::printUsage;
    assert(usage.heatedSeconds() == 60 && usage.filamentMetres() == 1.25f);
    Extruder::heating = true;
    usage.beginHeating(UINT32_MAX - 999);
    usage.recordPlannedFilament(350.0f);
    usage.recordPlannedFilament(-100.0f);
    HAL::now = 2500;
    assert(usage.heatedSeconds() == 63);
    assert(std::fabs(usage.filamentMetres() - 1.5f) < 0.0001f);

    Extruder::heating = false;
    usage.finishHeating(HAL::now);
    usage.finishHeating(HAL::now);
    assert(EEPROM::saves == 1 && usage.heatedSeconds() == 63);
    assert(std::fabs(usage.filamentMetres() - 1.5f) < 0.0001f);

    usage.beginHeating(HAL::now);
    usage.finishHeating(HAL::now + 5000);
    assert(EEPROM::saves == 1 && usage.heatedSeconds() == 63);

    HAL::seconds = INT32_MAX - 1;
    usage.beginHeating(HAL::now);
    usage.recordPlannedFilament(750.0f);
    HAL::now += 3000;
    usage.finishHeating(HAL::now);
    assert(usage.heatedSeconds() == INT32_MAX && EEPROM::saves == 2);
    assert(std::fabs(usage.filamentMetres() - 2.25f) < 0.0001f);
    EEPROM::storeUsageTotals(EEPROM::usageTotals());
    assert(EEPROM::saves==2);
}
