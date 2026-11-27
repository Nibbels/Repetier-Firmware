#pragma once
#include <stdint.h>

// Diagnostic history has no authority over the active fault latch, heater
// admission or settings recovery. Only the top-level command loop services it.
#if defined(RF_THERMAL_EMERGENCY_EXPERIMENT) && defined(RF_EEPROM_ZONE_EXPERIMENT)
enum class ScanAbort : uint8_t;
namespace DiagnosticHistory {
constexpr uint16_t ReadCommand=3950;
enum class ScanKind : uint8_t { HeatBed, ZOffset };
void request();
void recordScan(ScanAbort reason,ScanKind kind);
void service();
}
#endif
