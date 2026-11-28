#ifndef PRINT_USAGE_H
#define PRINT_USAGE_H

#include <stdint.h>

class PrintUsage {
public:
    void beginHeating(uint32_t now) {
        heatingStartedAt_ = now;
        pendingFilamentMm_ = 0.0f;
    }

    void recordPlannedFilament(float distanceMm) { pendingFilamentMm_ += distanceMm; }
    void finishHeating(uint32_t now);

    int32_t heatedSeconds() const;
    float filamentMetres() const;

private:
    uint32_t heatingStartedAt_ = 0;
    float pendingFilamentMm_ = 0.0f;
};

#endif
