#ifndef RGB_LIGHTING_H
#define RGB_LIGHTING_H

#include <stdint.h>

#define RGB_MODE_OFF 0
#define RGB_MODE_WHITE 1
#define RGB_MODE_AUTOMATIC 2
#define RGB_MODE_MANUAL 3

#define RGB_STATUS_NOT_AUTOMATIC 0
#define RGB_STATUS_AUTOMATIC 1
#define RGB_STATUS_PRINTING 11
#define RGB_STATUS_HEATING 12
#define RGB_STATUS_COOLING 13
#define RGB_STATUS_IDLE 14
#define RGB_STATUS_COLOR_CHANGE 15

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
