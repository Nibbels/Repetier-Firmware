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
