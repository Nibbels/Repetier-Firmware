#include "Repetier.h"
#include "EepromZoneAvr.h"

namespace EepromZones {
bool AvrDevice::foreground() const {return (SREG & _BV(SREG_I))!=0;}
bool AvrDevice::ready() const {return eeprom_is_ready();}
uint32_t AvrDevice::now() const {return HAL::timeInMilliseconds();}
uint8_t AvrDevice::readReady(uint16_t pos) {return HAL::eprGetByte(pos);}
void AvrDevice::updateReady(uint16_t pos,uint8_t value) {HAL::eprSetByte(pos,value);}

void AvrDevice::service() {
    // At initialization PWM/ADC periodic flags and the watchdog have not
    // started. Do not execute live control or stage hardware through them.
    if(!HAL::watchdogEnabled()) return;
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
