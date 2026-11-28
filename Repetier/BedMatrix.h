#ifndef BED_MATRIX_H
#define BED_MATRIX_H

#include "CalibrationScan.h"

namespace BedMatrix {

void restoreAtStartup(void (*progress)() = nullptr);
void clearWorking();
// Caller owns idle motion and heater admission and must not acknowledge
// the associated settings recovery until this durable invalidation succeeds.
bool invalidateAllStored(void (*progress)() = nullptr);
bool reloadActive();
void beginScanGrid();
void recordScanColumn(uint8_t x, short positionMm);
void recordScanPoint(uint8_t x, uint8_t y, short positionMm, short zSteps);
uint8_t maxIndex(uint8_t axis);
uint8_t storedMaxIndex(uint8_t axis);
short entry(uint8_t x, uint8_t y);
char prepareWorking();
void updateExtents();
void report(char format = 0);
void applyThermalOffset(short zSteps);
bool saveScannedGrid();

struct ZOffsetResult {
    bool overflow;
    bool tooHigh;
    bool aboveZero;
};

ZOffsetResult applyZOffset(long offset, ZOffsetScanPoint point, float gradient, long maximumHeight);
void finishZOffset();

} // namespace BedMatrix

#endif
