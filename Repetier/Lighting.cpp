#include "Repetier.h"
#include "Lighting.h"
namespace Lighting {
namespace {
constexpr bool hasWhite = !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN);
// Boot validates the board family before lighting initialization.
#ifdef RGB_LIGHT_R_PIN
constexpr uint8_t rgbMask = (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
static_assert(!HOTEND_COOLER_USES_PIN(RGB_LIGHT_R_PIN) &&
              !HOTEND_COOLER_USES_PIN(RGB_LIGHT_G_PIN) &&
              !HOTEND_COOLER_USES_PIN(RGB_LIGHT_B_PIN), "RGB pins cannot also drive a hotend cooler");
#endif
}
bool available() {
#ifdef RGB_LIGHT_R_PIN
    return true;
#else
    return hasWhite;
#endif
}
void initialize() {
    if(hasWhite) { WRITE(CASE_LIGHT_PIN,LOW); SET_OUTPUT(CASE_LIGHT_PIN); }
#ifdef RGB_LIGHT_R_PIN
    // Preserve established PWM period/polarity; hardware only, no ISR.
    TCCR4A=0;
    TCCR4B=0;
    WRITE(RGB_LIGHT_R_PIN,LOW); WRITE(RGB_LIGHT_G_PIN,LOW); WRITE(RGB_LIGHT_B_PIN,LOW);
    SET_OUTPUT(RGB_LIGHT_R_PIN); SET_OUTPUT(RGB_LIGHT_G_PIN); SET_OUTPUT(RGB_LIGHT_B_PIN);
    TCNT4=0;
    ICR4=20480;
    OCR4A=0; OCR4B=0; OCR4C=0;
    TCCR4A=(1<<WGM41);
    TCCR4B=(1<<WGM42)|(1<<WGM43)|(1<<CS41);
#endif
}
void set(Color color) {
#ifdef RGB_LIGHT_R_PIN
    // Endpoints are digital off/on; intermediate levels use PWM.
    WRITE(RGB_LIGHT_R_PIN,color.r==255);
    WRITE(RGB_LIGHT_G_PIN,color.g==255);
    WRITE(RGB_LIGHT_B_PIN,color.b==255);
    OCR4A=uint16_t(color.r)*80; OCR4B=uint16_t(color.g)*80; OCR4C=uint16_t(color.b)*80;
    uint8_t enabled=0;
    if(color.r && color.r!=255)enabled|=1<<COM4A1;
    if(color.g && color.g!=255)enabled|=1<<COM4B1;
    if(color.b && color.b!=255)enabled|=1<<COM4C1;
    TCCR4A=(TCCR4A & ~rgbMask)|enabled;
    // The binary case light follows neutral white requests.
    if(hasWhite)WRITE(CASE_LIGHT_PIN,color.r && color.r==color.g && color.g==color.b);
#else
    if(hasWhite)WRITE(CASE_LIGHT_PIN,color.r || color.g || color.b);
#endif
}
void serviceAlarm(uint32_t now) {
    // 512 ms each phase, wrap-safe. No delays or new scheduler state.
    set((now & 512UL) ? Red : Black);
}
}
