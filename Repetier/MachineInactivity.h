#ifndef MACHINE_INACTIVITY_H
#define MACHINE_INACTIVITY_H

#include <stdint.h>

class MachineInactivity {
public:
    constexpr MachineInactivity(uint32_t stepperTimeout, uint32_t shutdownTimeout)
        : stepperTimeout_(stepperTimeout), shutdownTimeout_(shutdownTimeout) {}

    void recordActivity(uint32_t now) { lastActivity_ = now; }

    uint32_t stepperTimeout() const { return stepperTimeout_; }
    uint32_t shutdownTimeout() const { return shutdownTimeout_; }
    void setStepperTimeout(uint32_t duration) { stepperTimeout_ = duration; }
    void setShutdownTimeout(uint32_t duration) { shutdownTimeout_ = duration; }

    bool stepperTimeoutExpired(uint32_t now) const {
        return stepperTimeout_ != 0 && now - lastActivity_ > stepperTimeout_;
    }
    bool shutdownTimeoutExpired(uint32_t now) const {
        return shutdownTimeout_ != 0 && now - lastActivity_ > shutdownTimeout_;
    }

private:
    uint32_t stepperTimeout_;
    uint32_t shutdownTimeout_;
    uint32_t lastActivity_ = 0;
};

#endif
