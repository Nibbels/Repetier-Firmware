#ifndef CASE_FAN_H
#define CASE_FAN_H

#include <stdint.h>

namespace CaseFan {

void initialize();
void turnOn();
bool requestOff(bool inhibitAutomatic);
void setOffDelay(uint32_t delayMs);
uint32_t offDelay();
void updateFromHotendTargets();
void service();

}

#endif
