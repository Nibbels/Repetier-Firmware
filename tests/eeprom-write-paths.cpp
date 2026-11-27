// Extract actual HAL EEPROM accessors, UI microstep switch and persistence owner.
// Host adapters count byte programming and reject any EEPROM access with IRQs masked.
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
using PGM_P = const char*;
struct GCode;
#include "Eeprom.h"
#define EEPROM_OFFSET 0
#define DRV8711_NUM_CHANNELS 5
enum { X_AXIS, Y_AXIS, Z_AXIS, E_AXIS };
enum { UI_ACTION_MICROSTEPS_XY, UI_ACTION_MICROSTEPS_Z, UI_ACTION_MICROSTEPS_E };
static bool masked = false;
static std::array<uint8_t, 4096> image{};
static unsigned writes = 0, reads = 0, checksums = 0;
static size_t address(const void* ptr) { return reinterpret_cast<uintptr_t>(ptr); }
static void get(void* dest, const void* src, size_t n) {
    assert(!masked && address(src)+n <= image.size());
    reads += n;
    memcpy(dest, image.data()+address(src), n);
}
static void update(const void* src, void* dest, size_t n) {
    assert(!masked && address(dest)+n <= image.size());
    for (size_t i=0; i<n; ++i) {
        ++reads;
        uint8_t byte = static_cast<const uint8_t*>(src)[i];
        uint8_t& old = image[address(dest)+i];
        if (old != byte) { old = byte; ++writes; }
    }
}
static void eeprom_update_byte(uint8_t* p, uint8_t v) { update(&v,p,1); }
static void eeprom_update_word(uint16_t* p, uint16_t v) { update(&v,p,2); }
static void eeprom_update_dword(uint32_t* p, uint32_t v) { update(&v,p,4); }
static void eeprom_update_block(const void* v, void* p, size_t n) { update(v,p,n); }
static uint8_t eeprom_read_byte(uint8_t* p) { uint8_t v; get(&v,p,1); return v; }
static uint16_t eeprom_read_word(uint16_t* p) { uint16_t v; get(&v,p,2); return v; }
static uint32_t eeprom_read_dword(uint32_t* p) { uint32_t v; get(&v,p,4); return v; }
static void eeprom_read_block(void* v,const void* p,size_t n) { get(v,p,n); }
struct HAL {
#include "hal-eeprom.inc"
};
struct InterruptProtectedBlock {
    bool old=masked;
    InterruptProtectedBlock() { masked=true; }
    ~InterruptProtectedBlock() { masked=old; }
};
namespace Drv8711 {
    uint8_t modes[5];
    uint8_t microstepMode(uint8_t i) { assert(i<5); return modes[i]; }
    void setMicrostepMode(uint8_t i,uint8_t v) { assert(masked && i<5); modes[i]=v; }
}
static unsigned short drv8711ModeValue_2_MicroSteps(uint8_t m) { return 1U << std::min<int>(m,8); }
struct Extruder { uint8_t id; float stepsPerMM; static Extruder* current; static void applyCurrentSettings() { assert(masked); } };
std::array<Extruder, NUM_EXTRUDER> extruder{};
Extruder* Extruder::current=&extruder.at(0);
namespace Printer {
    struct Job { bool busy=false; bool isBusy(){return busy;} } printJob;
    struct Pause { bool allowed=true; bool allowsQueueAdvance(){return allowed;} void scaleParkSteps(int,float){assert(masked);} } pause;
    struct Z { void rescaleMotorReference(float){assert(masked);} } zMotion;
    struct Layers { void rescaleFadeBounds(float){assert(masked);} } layers;
    struct Manual { uint16_t z=100; void rescaleSteps(int axis,float f){assert(masked && axis==Z_AXIS); z=uint16_t(z*f);} uint16_t steps(int axis){assert(axis==Z_AXIS);return z;} } manualMotion;
    struct Offset { void refreshSteps(float){assert(!masked);} } zOffset;
    float steps[3];
    float stepsPerMm(int i){assert(i<3);return steps[i];}
    void setStepsPerMm(int i,float v){assert(masked && i<3);steps[i]=v;}
    void disableAllSteppersNow(){}
    void updateDerivedParameter(){assert(masked);}
}
struct PrintLine { static bool hasLines(){return false;} };
static void invalidateBedMatrixForReload(){assert(masked);}
#define INCREMENT_MIN_MAX(a, steps, lo, hi) a = std::min<int>(hi,std::max<int>(lo,(a + increment * steps)))
int EEPROM::getExtruderOffset(uint8_t i) { assert(i<NUM_EXTRUDER); return EEPROM_EXTRUDER_OFFSET + i*EEPROM_EXTRUDER_LENGTH; }
void EEPROM::updateChecksum() { assert(!masked); ++checksums; }
#include "microstep-persist.inc"
static void edit(int action, int8_t increment) {
    switch (action) {
#include "microstep-ui.inc"
    }
}
static void setup() {
    image.fill(0xff); writes=reads=checksums=0;
    for (auto& m:Drv8711::modes) m=5;
    for (auto& s:Printer::steps) s=100;
    for (uint8_t i=0;i<NUM_EXTRUDER;++i) extruder.at(i)={i,100.0f};
    Printer::manualMotion.z=100;
    Printer::printJob.busy=false; Printer::pause.allowed=true;
}
int main() {
    setup();
    HAL::eprSetInt32(500,100); writes=0;
    HAL::eprSetInt32(500,101); assert(writes==1);
    HAL::eprSetInt32(500,101); assert(writes==1);
    HAL::eprSetInt16(510,0x100); writes=0;
    HAL::eprSetInt16(510,0x101); assert(writes==1);
    HAL::eprSetFloat(520,1.0f); writes=0;
    HAL::eprSetFloat(520,1.0f); assert(writes==0);
    HAL::eprSetFloat(520,2.0f); assert(writes==2);
    setup();
    edit(UI_ACTION_MICROSTEPS_XY,1);
    assert(!masked && checksums==1 && Drv8711::modes[0]==6 && Drv8711::modes[1]==6);
    assert(Printer::steps[0]==200 && Printer::steps[1]==200);
    assert(HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM)==200);
    assert(HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM)==200);
    assert(HAL::eprGetByte(EPR_RF_MICRO_STEPS_USED)==0xAB);
    writes=0; edit(UI_ACTION_MICROSTEPS_XY,1); assert(writes==0 && checksums==1);
    // Mismatched pair: secondary already at target, primary still changes.
    Drv8711::modes[0]=4; Drv8711::modes[1]=5; Printer::steps[0]=100; Printer::steps[1]=200;
    edit(UI_ACTION_MICROSTEPS_XY,1);
    assert(Drv8711::modes[0]==5 && Drv8711::modes[1]==5);
    assert(Printer::steps[0]==200 && Printer::steps[1]==200);
    // Z coupled steps, jog increment, matrix invalidation and IRQ-enabled save.
    edit(UI_ACTION_MICROSTEPS_Z,-1);
    assert(Printer::steps[2]==50 && Printer::manualMotion.z==50);
    assert(HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM)==50 && HAL::eprGetInt16(EPR_RF_MOD_Z_STEP_SIZE)==50);
    // E1 is a physical driver on one-tool configurations, not an Extruder object.
    edit(UI_ACTION_MICROSTEPS_E,2);
    assert(Drv8711::modes[3]==7 && Drv8711::modes[4]==7);
    for(uint8_t i=0;i<NUM_EXTRUDER;++i) {
        assert(extruder.at(i).stepsPerMM==400);
        assert(HAL::eprGetFloat(EEPROM::getExtruderOffset(i)+EPR_EXTRUDER_STEPS_PER_MM)==400);
    }
    if(NUM_EXTRUDER==1) assert(HAL::eprGetInt32(EEPROM_EXTRUDER_OFFSET+EEPROM_EXTRUDER_LENGTH)==-1);
    unsigned before=checksums; writes=0;
    Printer::printJob.busy=true; edit(UI_ACTION_MICROSTEPS_E,-1);
    assert(writes==0 && checksums==before);
    Printer::printJob.busy=false; Printer::pause.allowed=false; edit(UI_ACTION_MICROSTEPS_E,-1);
    assert(writes==0 && checksums==before);
    printf("EEPROM byte updates / microstep IRQ boundary, %d tool(s): PASS\n",NUM_EXTRUDER);
}
