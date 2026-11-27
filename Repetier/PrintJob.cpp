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
        // Only the outer command loop may release the stop inhibit and
        // start a new output script; older wait/plan frames must unwind.
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

void PrintJob::servicePendingOutput(uint32_t now) {
    {
        InterruptProtectedBlock lock;
        if (phase_ != Phase::WaitingToOutput ||
            static_cast<int32_t>(now - phaseSince_) <= 1000)
            return;
        Printer::extrusionOutput.allow(ExtrusionOutput::Inhibit::PrintStop);
        phase_ = Phase::Idle;
    }
    outputObject(false);
}
