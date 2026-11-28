#include <cassert>
#include <cstdint>
#include <cstdio>

#include "../Repetier/MachineInactivity.h"

int main() {
    MachineInactivity inactivity(6000, 15000);
    assert(inactivity.stepperTimeout() == 6000 && inactivity.shutdownTimeout() == 15000);

    inactivity.recordActivity(1000);
    assert(!inactivity.stepperTimeoutExpired(7000));
    assert(inactivity.stepperTimeoutExpired(7001));
    assert(!inactivity.shutdownTimeoutExpired(16000));
    assert(inactivity.shutdownTimeoutExpired(16001));

    inactivity.recordActivity(16001);
    assert(!inactivity.stepperTimeoutExpired(16001) && !inactivity.shutdownTimeoutExpired(16001));

    inactivity.setStepperTimeout(0);
    inactivity.setShutdownTimeout(0);
    assert(!inactivity.stepperTimeoutExpired(600000) && !inactivity.shutdownTimeoutExpired(600000));

    inactivity.setStepperTimeout(20);
    inactivity.setShutdownTimeout(50);
    inactivity.recordActivity(UINT32_MAX - 20);
    assert(!inactivity.stepperTimeoutExpired(UINT32_MAX));
    assert(inactivity.stepperTimeoutExpired(0));
    assert(!inactivity.shutdownTimeoutExpired(29));
    assert(inactivity.shutdownTimeoutExpired(30));
    std::puts("Machine inactivity replay passed");
}
