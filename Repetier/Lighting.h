#ifndef LIGHTING_H
#define LIGHTING_H
#include <stdint.h>
namespace Lighting {
// RGB888, independent linear 8-bit intensities, no alpha.
// Black is off. Monochrome maps nonblack to on.
struct Color { uint8_t r,g,b; };
constexpr Color Black{0,0,0}, White{255,255,255}, Red{255,0,0};
bool available();
void initialize();
void set(Color color);
// Terminal heater fault, foreground only, after thermal service.
void serviceAlarm(uint32_t now);
}
#endif
