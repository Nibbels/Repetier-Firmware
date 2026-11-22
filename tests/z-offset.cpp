#include "../Repetier/ZOffset.h"

#include <assert.h>

namespace {

constexpr float stepsPerMm = 1280.0f;
constexpr int16_t senseLimit = 300;
constexpr int32_t manualLimit = 10000;
constexpr int16_t buttonStep = 5;

void reloadPreservesSenseOffset() {
    ZOffset offset;
    const int32_t savedManual = 50;
    offset.setManualMicrometers(savedManual, stepsPerMm);
    assert(offset.effectiveSteps() == 64);

    offset.addSenseMicrometers(30, senseLimit, stepsPerMm);
    assert(offset.effectiveSteps() == 102);

    offset.setManualMicrometers(savedManual, stepsPerMm);
    assert(offset.manualMicrometers() == 50);
    assert(offset.senseMicrometers() == 30);
    assert(offset.effectiveSteps() == 102);

    offset.setManualMicrometers(70, 640.0f);
    assert(offset.manualMicrometers() == 70);
    assert(offset.senseMicrometers() == 30);
    assert(offset.effectiveSteps() == 64);
}

void bedButtonsTransferLiveOffset() {
    ZOffset offset;
    offset.addSenseMicrometers(200, senseLimit, stepsPerMm);
    const int32_t initialSteps = offset.effectiveSteps();

    for (int16_t transferred = buttonStep; transferred <= 200; transferred += buttonStep) {
        offset.adjustWithBedButton(buttonStep, manualLimit, stepsPerMm);
        assert(offset.effectiveSteps() > initialSteps);
        offset.adjustWithBedButton(-buttonStep, manualLimit, stepsPerMm);
        assert(offset.manualMicrometers() == transferred);
        assert(offset.senseMicrometers() == 200 - transferred);
        assert(offset.effectiveSteps() == initialSteps);
    }

    offset.setManualMicrometers(10, stepsPerMm);
    offset.addSenseMicrometers(2, senseLimit, stepsPerMm);
    offset.adjustWithBedButton(-buttonStep, manualLimit, stepsPerMm);
    assert(offset.manualMicrometers() == 7);
    assert(offset.senseMicrometers() == 0);

    offset.setManualMicrometers(-2, stepsPerMm);
    offset.adjustWithBedButton(buttonStep, manualLimit, stepsPerMm);
    assert(offset.manualMicrometers() == 0);
    offset.setManualMicrometers(2, stepsPerMm);
    offset.addSenseMicrometers(2, senseLimit, stepsPerMm);
    offset.adjustWithBedButton(-buttonStep, manualLimit, stepsPerMm);
    assert(offset.manualMicrometers() == 2);
    assert(offset.senseMicrometers() == 0);

    offset.setManualMicrometers(manualLimit, stepsPerMm);
    offset.adjustWithBedButton(buttonStep, manualLimit, stepsPerMm);
    assert(offset.manualMicrometers() == manualLimit);
    offset.setManualMicrometers(-manualLimit, stepsPerMm);
    offset.adjustWithBedButton(-buttonStep, manualLimit, stepsPerMm);
    assert(offset.manualMicrometers() == -manualLimit);
}

void clearAndTransferUseTheExpectedPortion() {
    ZOffset offset;
    offset.setManualMicrometers(50, stepsPerMm);
    offset.addSenseMicrometers(100, 30, stepsPerMm);
    assert(offset.senseMicrometers() == 30);
    assert(offset.manualSteps(stepsPerMm) == 64);

    // M3902 Z0 shifts the matrix by the manual portion before zeroing it.
    offset.setManualMicrometers(0, stepsPerMm);
    assert(offset.senseMicrometers() == 30);
    assert(offset.effectiveSteps() == 38);

    offset.setManualMicrometers(50, stepsPerMm);
    offset.clearSenseOffset(stepsPerMm);
    assert(offset.manualMicrometers() == 50);
    assert(offset.senseMicrometers() == 0);
    assert(offset.effectiveSteps() == 64);
}

void scaleChangesConvertFromMicrometers() {
    ZOffset offset;
    offset.setManualMicrometers(-7, stepsPerMm);
    assert(offset.effectiveSteps() == -8);
    offset.refreshSteps(2560.0f);
    assert(offset.effectiveSteps() == -17);
    offset.refreshSteps(stepsPerMm);
    assert(offset.effectiveSteps() == -8);
}

} // namespace

int main() {
    reloadPreservesSenseOffset();
    bedButtonsTransferLiveOffset();
    clearAndTransferUseTheExpectedPortion();
    scaleChangesConvertFromMicrometers();
}
