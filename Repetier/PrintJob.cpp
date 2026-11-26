#include "Repetier.h"
#include "GCodeInput.h"

PrintJob Printer::printJob;

void PrintJob::service(uint32_t now) {
    switch (phase_) {
    case Phase::Draining:
        // Drain incoming bytes while the command parser rejects new work.
        GCodeInput::readFromSerial();
        if (static_cast<int32_t>(now - phaseSince_) <= 2000)
            return;
        if (PrintLine::hasLines() || PrintLine::directMoveActive()) {
            phaseSince_ = now;
            return;
        }

        Extruder::setHeatedBedTemperature(0);
        Extruder::setTemperatureForAllExtruders(0);
        discardScans();
        Com::printFLN(PSTR("Stop complete"));
        phaseSince_ = now;
        phase_ = Phase::WaitingToOutput;
        return;

    case Phase::WaitingToOutput:
        if (static_cast<int32_t>(now - phaseSince_) > 1000) {
            {
                InterruptProtectedBlock lock;
                Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::PrintStop);
                phase_ = Phase::Idle;
            }
            outputObject(false);
        }
        return;

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    case Phase::ThermalEmergency:
        return; // Never drain into outputObject or release the safety lock.
#endif
    case Phase::Idle:
    case Phase::Printing:
    case Phase::FilamentService:
        return;
    }
}
