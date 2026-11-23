#include "Repetier.h"

PrintStop Printer::printStop;

void PrintStop::service(uint32_t now) {
    switch (phase_) {
    case Phase::Draining:
        // Drain incoming bytes while the command parser rejects new work.
        GCode::readFromSerial();
        if (static_cast<int32_t>(now - phaseSince_) <= 2000)
            return;
        if (PrintLine::hasLines() || PrintLine::directMoveActive()) {
            phaseSince_ = now;
            return;
        }

        Extruder::setHeatedBedTemperature(0);
        Extruder::setTemperatureForAllExtruders(0);
        g_nZOSScanStatus = 0;
        g_nHeatBedScanStatus = 0;
        Com::printFLN(PSTR("Stop complete"));
        Printer::setPrinting(false);
        phaseSince_ = now;
        phase_ = Phase::WaitingToOutput;
        return;

    case Phase::WaitingToOutput:
        if (static_cast<int32_t>(now - phaseSince_) > 1000) {
            phase_ = Phase::Idle;
            outputObject(false);
        }
        return;

    case Phase::Idle:
        return;
    }
}
