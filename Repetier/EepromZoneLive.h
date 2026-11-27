// Binding between the zone codec and actual firmware-owned settings.
// No legacy EEPROM addresses; no physical writes. A session must validate
// the entire image and inhibit dependent outputs before applyBlock.
#ifndef RF_EEPROM_ZONE_LIVE_H
#define RF_EEPROM_ZONE_LIVE_H
#include "EepromZonePatch.h"
namespace EepromZones {
class LiveSettings {
public:
    // Capture only fields owned by mutable live state. Usage and BedSafety
    // are intentionally not part of M500; they have separate transactions.
    static constexpr uint32_t BulkMask=Active & ~(blockBit(Usage)|blockBit(BedSafety));
    bool operator()(uint8_t id,uint8_t* data,uint8_t n) const;
    // BedSafety is staged separately and may only be published after a
    // successful complete session. Usage belongs to PrintUsage.
    bool applyBlock(uint8_t id,const uint8_t* data,uint8_t n,uint32_t alerts);
    // Narrow post-commit publication for DMIN/DMAX only. No target,
    // calibration identity, derivative history or sensor changes.
    bool applyDriveLimits(uint8_t id,const uint8_t* data,uint8_t n,
                          const Patch* edits,uint8_t count);
    void finishMotion();
    uint8_t bedCutoff() const {return bedCutoff_;}
private:
    uint8_t bedCutoff_=0;
};
}
#endif
