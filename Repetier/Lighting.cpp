#include "Repetier.h"
#include "Lighting.h"

namespace Lighting {
namespace {
constexpr bool hasWhite = !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN);
// One board-specific backend selection, after boot's board-family check.
#ifdef RGB_LIGHT_R_PIN
constexpr bool hasRgb = true;
constexpr uint8_t rgbMask = (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
constexpr uint8_t pinMask = (1<<PH3)|(1<<PH4)|(1<<PH5);
static_assert(RGB_LIGHT_R_PIN==6 && RGB_LIGHT_G_PIN==7 && RGB_LIGHT_B_PIN==8,
              "X43 must map to PH3/PH4/PH5 (Timer4)");
static_assert(!HOTEND_COOLER_USES_PIN(RGB_LIGHT_R_PIN) &&
              !HOTEND_COOLER_USES_PIN(RGB_LIGHT_G_PIN) &&
              !HOTEND_COOLER_USES_PIN(RGB_LIGHT_B_PIN), "RGB pins cannot also drive a hotend cooler");

void initializeRgb() {
    // Preserve established PWM period/polarity; hardware only, no ISR.
    TCCR4A=0;
    TCCR4B=0;
    {
        // PORTH also contains the E1 heater (PH6). Never overwrite a
        // concurrent heater-PWM update with a stale read/modify/write.
        InterruptProtectedBlock lock;
        PORTH &= uint8_t(~pinMask);
        DDRH |= pinMask;
    }
    TCNT4=0;
    ICR4=20480;
    OCR4A=0; OCR4B=0; OCR4C=0;
    TCCR4A=(1<<WGM41);
    TCCR4B=(1<<WGM42)|(1<<WGM43)|(1<<CS41);
}

void setRgb(Color color) {
    // Endpoints are digital off/on; intermediate levels use PWM.
    uint8_t pins=0,enabled=0;
    if(color.r==255)pins|=1<<PH3;
    else if(color.r)enabled|=1<<COM4A1;
    if(color.g==255)pins|=1<<PH4;
    else if(color.g)enabled|=1<<COM4B1;
    if(color.b==255)pins|=1<<PH5;
    else if(color.b)enabled|=1<<COM4C1;
    {
        InterruptProtectedBlock lock;
        PORTH=(PORTH & uint8_t(~pinMask))|pins;
    }
    OCR4A=uint16_t(color.r)*80; OCR4B=uint16_t(color.g)*80; OCR4C=uint16_t(color.b)*80;
    TCCR4A=(TCCR4A & ~rgbMask)|enabled;
}
#else
constexpr bool hasRgb = false;
void initializeRgb() {}
void setRgb(Color) {}
#endif
}

bool available() { return hasWhite || hasRgb; }

void initialize() {
    if(hasWhite) { WRITE(CASE_LIGHT_PIN,LOW); SET_OUTPUT(CASE_LIGHT_PIN); }
    initializeRgb();
}

void set(Color color) {
    setRgb(color);
    // RF2000 has a separate binary case output; neutral whites use both.
    // RF1000 falls back to white for any nonblack color.
    if(hasWhite)WRITE(CASE_LIGHT_PIN,hasRgb ?
        (color.r && color.r==color.g && color.g==color.b) :
        (color.r || color.g || color.b));
}

void serviceAlarm(uint32_t now) {
    // 512 ms each phase, wrap-safe. No delays or new scheduler state.
    // Pass scalar channel values instead of selecting an addressable RGB constant.
    set({uint8_t((now & 512UL) ? 255 : 0), 0, 0});
}
}
