#ifndef RGB_LIGHTING_H
#define RGB_LIGHTING_H

#include <stdint.h>

class GCode;

namespace RgbLighting {
char mode();
bool forcedWhite();
void requestWhiteToggle();
void restoreMode(char mode);
void selectMode(char mode);
void reset();
void initialize();
void updateStatus();
void advanceFade();

void setHeatingLight(GCode* command);
void setPrintingLight(GCode* command);
void setCoolingLight(GCode* command);
void setIdleLight(GCode* command);
void setManualLight(GCode* command);
void setRgbLightMode(GCode* command);
}

#endif
