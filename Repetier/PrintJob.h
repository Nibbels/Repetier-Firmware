#ifndef PRINT_JOB_H
#define PRINT_JOB_H

#include <stdint.h>

class PrintJob {
public:
    bool isPrinting() const { return phase_ == Phase::Printing; }

    bool isBusy() const {
        const Phase phase = phase_;
        return phase == Phase::Printing || phase == Phase::FilamentService || phase == Phase::Draining;
    }

    bool commandsBlocked() const {
        const Phase phase = phase_;
        return phase == Phase::Draining || phase == Phase::WaitingToOutput
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
            || phase == Phase::ThermalEmergency
#endif
            ;
    }

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
    bool emergencyLocked() const { return phase_ == Phase::ThermalEmergency; }
    // Caller holds the interrupt lock. Terminal for this boot, even if the
    // old job was idle or already waiting for its automatic output move.
    bool beginThermalEmergency() {
        if(emergencyLocked()) return false;
        phase_=Phase::ThermalEmergency;
        return true;
    }
#endif
    void admitPrintMotion() {
        if (phase_ == Phase::Idle)
            phase_ = Phase::Printing;
    }
    void finishPrintMotion() {
        if (phase_ == Phase::Printing)
            phase_ = Phase::Idle;
    }

    void beginFilamentService() {
        if (phase_ == Phase::Idle)
            phase_ = Phase::FilamentService;
    }
    void finishFilamentService() {
        if (phase_ == Phase::FilamentService)
            phase_ = Phase::Idle;
    }

    // Called with interrupts masked before the planner and pause are reset.
    bool beginStop(uint32_t now) {
        if (phase_ != Phase::Printing && phase_ != Phase::FilamentService)
            return false;
        phaseSince_ = now;
        phase_ = Phase::Draining;
        return true;
    }

    void service(uint32_t now);

private:
    enum class Phase : uint8_t { Idle, Printing, FilamentService, Draining, WaitingToOutput
#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
        , ThermalEmergency
#endif
    };

    volatile Phase phase_ = Phase::Idle;
    uint32_t phaseSince_ = 0;
};

#endif // PRINT_JOB_H
