#include "Repetier.h"
#include "EepromZoneAvr.h"

namespace EepromZones {
namespace {
bool settingsOwned=false;
bool runtimeServicesReady=false;
}
bool& AvrDevice::settingsBusy(){return settingsOwned;}
bool AvrDevice::logMayAccess(){return !settingsOwned && (SREG & _BV(SREG_I))!=0;}
void AvrDevice::enableRuntimeServices(){runtimeServicesReady=true;}
bool AvrDevice::foreground() const {return (SREG & _BV(SREG_I))!=0;}
bool AvrDevice::ready() const {return eeprom_is_ready() && !(SPMCSR & _BV(SPMEN));}
uint32_t AvrDevice::now() const {return HAL::timeInMilliseconds();}
uint8_t AvrDevice::readReady(uint16_t pos) {return HAL::eprGetByte(pos);}
void AvrDevice::updateReady(uint16_t pos,uint8_t value) {HAL::eprSetByte(pos,value);}

void AvrDevice::service() {
    // Startup and watchdog enable are different states.
    if(!runtimeServicesReady) return;
    if(HAL::takeScanTick()) HAL::tellWatchdogOk();
    if(HAL::takeTemperatureTick()) Extruder::manageTemperatures();
    if(Printer::heatingInhibited() || Printer::blockAll) return;
    // Maintain existing Z correction and nonblocking DMS acquisition. No
    // slowAction, print-job transition, scan dispatch, G-code dispatch,
    // pause motion, feature tick consumption or recursive persistence.
    // Full settings loads/resets require idle motion and heater inhibition.
    // Live small edits still need a measured maximum feature-service gap.
    if(HAL::takeCompensationTick()) recalculateZCompensation();
    Printer::strainGauge.service(HAL::timeInMilliseconds());
}
}

#ifdef RF_THERMAL_EMERGENCY_EXPERIMENT
#include "DiagnosticHistory.h"
#include "AvrEventStore.h"
#include "EventLogService.h"
#include "ThermalEmergencyReport.h"
#include "UsbSerial.h"

namespace DiagnosticHistory {
namespace {
constexpr uint16_t logBase=0x0E00;
constexpr uint8_t logSlots=16;
static_assert(logBase>=EepromZones::End, "diagnostic history overlaps settings");
static_assert(uint32_t(logBase)+EventLog::Ring<logSlots>::Bytes==uint32_t(E2END)+1,
              "diagnostic history must end at the physical EEPROM boundary");
EepromZones::AvrDevice logOwner;
EventLog::AvrStore<logBase,logSlots,EepromZones::AvrDevice> store(logOwner);
UsbSerial sink;
EventLog::Service<logSlots,NUM_TEMPERATURE_LOOPS,ThermalEmergency::ReportLine,
                  ThermalEmergency::Cause,UsbSerial> history(sink);
}

void request() { history.requestHistory(); }

void recordScan(ScanAbort reason,ScanKind kind) {
    // RAM-only admission at the confirmed abort. If occupied or failed, the
    // existing local diagnosis remains available; never delay scan retreat.
    history.submitScan(reason,kind==ScanKind::HeatBed ? EventLog::ScanContext::HeatBed :
                       EventLog::ScanContext::ZOffset,HAL::timeInMilliseconds());
}

void service() {
    // Called only at the top-level command-loop boundary, AFTER the thermal
    // and protocol services. Never from settings progress, ISR or a wait.
    const auto& ledger=ThermalEmergency::faultLedger();
    if(!ledger.active()) {
        if(!history.historyPending()&&!history.scanPending())return;
        if(Printer::printJob.isBusy() || Printer::pause.isPaused() ||
           PrintLine::hasLines() || PrintLine::directMoveActive() ||
           Printer::needsCMPwait() || isAnyScanRunning() ||
           TemperatureController::autotuneActive())return;
    }
    if(!EepromZones::AvrDevice::logMayAccess())return;
    // At most one byte launch or one bounded read/output step; EEPROM busy
    // returns immediately. No WDT feed, no motion/heating permission change.
    history.service(store,ledger);
}
}
#endif
