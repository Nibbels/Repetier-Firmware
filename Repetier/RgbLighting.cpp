#include "Repetier.h"

#ifdef RGB_LIGHT_R_PIN
namespace RgbLighting {
namespace {
char selectedMode;
char lightStatus;
uint32_t idleStartedAt;
bool whiteToggleRequested;
bool forceWhite;
unsigned char g_uRGBHeatingR = RGB_HEATING_R;
unsigned char g_uRGBHeatingG = RGB_HEATING_G;
unsigned char g_uRGBHeatingB = RGB_HEATING_B;
unsigned char g_uRGBPrintingR = RGB_PRINTING_R;
unsigned char g_uRGBPrintingG = RGB_PRINTING_G;
unsigned char g_uRGBPrintingB = RGB_PRINTING_B;
unsigned char g_uRGBCoolingR = RGB_COOLING_R;
unsigned char g_uRGBCoolingG = RGB_COOLING_G;
unsigned char g_uRGBCoolingB = RGB_COOLING_B;
unsigned char g_uRGBIdleR = RGB_IDLE_R;
unsigned char g_uRGBIdleG = RGB_IDLE_G;
unsigned char g_uRGBIdleB = RGB_IDLE_B;
unsigned char g_uRGBManualR = RGB_MANUAL_R;
unsigned char g_uRGBManualG = RGB_MANUAL_G;
unsigned char g_uRGBManualB = RGB_MANUAL_B;
volatile unsigned char g_uRGBCurrentR = 0;
volatile unsigned char g_uRGBCurrentG = 0;
volatile unsigned char g_uRGBCurrentB = 0;
volatile unsigned char g_uRGBTargetR = 0;
volatile unsigned char g_uRGBTargetG = 0;
volatile unsigned char g_uRGBTargetB = 0;
void setTargetColors(uint8_t R, uint8_t G, uint8_t B) {
    InterruptProtectedBlock lock;
    g_uRGBTargetR = R;
    g_uRGBTargetG = G;
    g_uRGBTargetB = B;

} // setTargetColors

void writeLeds(uint8_t R, uint8_t G, uint8_t B) {
    if (R > 0) {
        TCCR4A |= (1 << COM4A1); // R > 0 PWM start
        OCR4A = R * 80;
    } else {
        TCCR4A &= ~(1 << COM4A1); // R = 0 PWM stop
    }

    if (G > 0) {
        TCCR4A |= (1 << COM4B1); // G > 0 PWM start
        OCR4B = G * 80;
    } else {
        TCCR4A &= ~(1 << COM4B1); // G = 0 PWM stop
    }

    if (B > 0) {
        TCCR4A |= (1 << COM4C1); // B > 0 PWM start
        OCR4C = B * 80;
    } else {
        TCCR4A &= ~(1 << COM4C1); // B = 0 PWM stop
    }

    g_uRGBCurrentR = R;
    g_uRGBCurrentG = G;
    g_uRGBCurrentB = B;

} // writeLeds

} // namespace

char mode() { return selectedMode; }
bool forcedWhite() { return forceWhite; }
void requestWhiteToggle() { whiteToggleRequested = true; }

// Persisted profile order is printing, heating, cooling, idle, manual.
void captureProfiles(uint8_t* data) {
    data[0]=g_uRGBPrintingR;
    data[1]=g_uRGBPrintingG;
    data[2]=g_uRGBPrintingB;
    data[3]=g_uRGBHeatingR;
    data[4]=g_uRGBHeatingG;
    data[5]=g_uRGBHeatingB;
    data[6]=g_uRGBCoolingR;
    data[7]=g_uRGBCoolingG;
    data[8]=g_uRGBCoolingB;
    data[9]=g_uRGBIdleR;
    data[10]=g_uRGBIdleG;
    data[11]=g_uRGBIdleB;
    data[12]=g_uRGBManualR;
    data[13]=g_uRGBManualG;
    data[14]=g_uRGBManualB;
}
void restoreProfiles(const uint8_t* data) {
    g_uRGBPrintingR=data[0];
    g_uRGBPrintingG=data[1];
    g_uRGBPrintingB=data[2];
    g_uRGBHeatingR=data[3];
    g_uRGBHeatingG=data[4];
    g_uRGBHeatingB=data[5];
    g_uRGBCoolingR=data[6];
    g_uRGBCoolingG=data[7];
    g_uRGBCoolingB=data[8];
    g_uRGBIdleR=data[9];
    g_uRGBIdleG=data[10];
    g_uRGBIdleB=data[11];
    g_uRGBManualR=data[12];
    g_uRGBManualG=data[13];
    g_uRGBManualB=data[14];
}
void restoreMode(char mode) {
    selectedMode = mode;
    if (mode == RGB_MODE_AUTOMATIC) {
        lightStatus = RGB_STATUS_AUTOMATIC;
        idleStartedAt = 0;
    } else {
        lightStatus = RGB_STATUS_NOT_AUTOMATIC;
    }
}

void reset() {
    restoreMode(RGB_LIGHT_DEFAULT_MODE);
    idleStartedAt = 0;
    whiteToggleRequested = false;
    forceWhite = false;
}

void selectMode(char mode) {
    selectedMode = mode;
    lightStatus = mode == RGB_MODE_AUTOMATIC ? RGB_STATUS_AUTOMATIC : RGB_STATUS_NOT_AUTOMATIC;
    forceWhite = false;
    switch (mode) {
    case RGB_MODE_OFF: setTargetColors(0, 0, 0); break;
    case RGB_MODE_WHITE: setTargetColors(255, 255, 255); break;
    case RGB_MODE_MANUAL: setTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB); break;
    }
    EEPROM::storeRgbMode(selectedMode);
}

void initialize() {
    writeLeds(0, 0, 0);

    switch (selectedMode) {
    case RGB_MODE_OFF: {
        setTargetColors(0, 0, 0);
        break;
    }
    case RGB_MODE_WHITE: {
        setTargetColors(255, 255, 255);
        break;
    }
    case RGB_MODE_AUTOMATIC: {
        setTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
        break;
    }
    case RGB_MODE_MANUAL: {
        setTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
        break;
    }
    }
}
void advanceFade() {
    char change = 0;
    if (g_uRGBTargetR > g_uRGBCurrentR) {
        g_uRGBCurrentR++;
        change = 1;
    } else if (g_uRGBTargetR < g_uRGBCurrentR) {
        g_uRGBCurrentR--;
        change = 1;
    }
    if (g_uRGBTargetG > g_uRGBCurrentG) {
        g_uRGBCurrentG++;
        change = 1;
    } else if (g_uRGBTargetG < g_uRGBCurrentG) {
        g_uRGBCurrentG--;
        change = 1;
    }
    if (g_uRGBTargetB > g_uRGBCurrentB) {
        g_uRGBCurrentB++;
        change = 1;
    } else if (g_uRGBTargetB < g_uRGBCurrentB) {
        g_uRGBCurrentB--;
        change = 1;
    }

    if (change) {
        writeLeds(g_uRGBCurrentR, g_uRGBCurrentG, g_uRGBCurrentB);
    }
}

struct RgbProfile {
    uint8_t* components[3];
    uint8_t index;
    PGM_P messagePrefix;
    bool active;
};

void setRgbComponent(GCode& command, const RgbProfile& profile) {
    if (!command.hasP() || !command.hasS() || command.P < 1 || command.P > 3)
        return;

    const uint8_t channel = command.P - 1;
    const char channelName = channel == 0 ? 'R' : channel == 1 ? 'G' : 'B';
    if (command.S < 0 || command.S > 255) {
        Com::printF(profile.messagePrefix);
        Com::print(channelName);
        Com::printFLN(PSTR(" out of range "));
        return;
    }

    auto& component = *profile.components[channel];
    component = command.S;
    EEPROM::storeRgbColor(profile.index,channel,component);
    if (profile.active && !forceWhite)
        setTargetColors(*profile.components[0],*profile.components[1],*profile.components[2]);
    Com::printF(profile.messagePrefix);
    Com::print(channelName);
    Com::printFLN(PSTR(" = "), component);
}

// RGB lighting.
void setHeatingLight(GCode* pCommand) {
    const bool active = selectedMode == RGB_MODE_AUTOMATIC && lightStatus == RGB_STATUS_HEATING;
    setRgbComponent(*pCommand, {{&g_uRGBHeatingR,&g_uRGBHeatingG,&g_uRGBHeatingB},1,
                                PSTR("RGB_HEATING_"), active});
}

void setPrintingLight(GCode* pCommand) {
    const bool active = selectedMode == RGB_MODE_AUTOMATIC && lightStatus == RGB_STATUS_PRINTING;
    setRgbComponent(*pCommand, {{&g_uRGBPrintingR,&g_uRGBPrintingG,&g_uRGBPrintingB},0,
                                PSTR("RGB_PRINTING_"), active});
}

void setCoolingLight(GCode* pCommand) {
    const bool active = selectedMode == RGB_MODE_AUTOMATIC && lightStatus == RGB_STATUS_COOLING;
    setRgbComponent(*pCommand, {{&g_uRGBCoolingR,&g_uRGBCoolingG,&g_uRGBCoolingB},2,
                                PSTR("RGB_COOLING_"), active});
}

void setIdleLight(GCode* pCommand) {
    const bool active = selectedMode == RGB_MODE_AUTOMATIC && lightStatus == RGB_STATUS_IDLE;
    setRgbComponent(*pCommand, {{&g_uRGBIdleR,&g_uRGBIdleG,&g_uRGBIdleB},3,
                                PSTR("RGB_IDLE_"), active});
}

void setManualLight(GCode* pCommand) {
    const bool active = selectedMode == RGB_MODE_MANUAL;
    setRgbComponent(*pCommand, {{&g_uRGBManualR,&g_uRGBManualG,&g_uRGBManualB},4,
                                PSTR("RGB_MANUAL_"), active});
}

void setRgbLightMode(GCode* command) {
    if (command->hasP() && command->P >= RGB_MODE_OFF && command->P <= RGB_MODE_MANUAL)
        selectMode(command->P);
}
void updateStatus() {
    char newStatus = RGB_STATUS_IDLE;

    if (whiteToggleRequested) {
        // toggle the white light
        forceWhite = !forceWhite;

        if (forceWhite) {
            setTargetColors(255, 255, 255);
        } else {
            if (selectedMode == RGB_MODE_OFF) {
                setTargetColors(0, 0, 0);
            } else if (selectedMode == RGB_MODE_AUTOMATIC) {
                lightStatus = RGB_STATUS_AUTOMATIC;
            } else if (selectedMode == RGB_MODE_MANUAL) {
                setTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
            }
        }

        whiteToggleRequested = 0;
    }

    if (forceWhite) {
        // there is nothing to do in case we shall display the white light
        return;
    }

    if (lightStatus == RGB_STATUS_NOT_AUTOMATIC) {
        // there is nothing to do in case we shall not change the RGB colors automatically
        return;
    }


    // operating mode print
    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if (extruder[i].tempControl.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
            if (fabs(extruder[i].tempControl.targetTemperatureC - extruder[i].tempControl.currentTemperatureC) < RGB_LIGHT_TEMP_TOLERANCE) {
                // we have reached the target temperature
                newStatus = RGB_STATUS_PRINTING;
            } else if (extruder[i].tempControl.targetTemperatureC > extruder[i].tempControl.currentTemperatureC) {
                // we are still heating
                newStatus = RGB_STATUS_HEATING;
            } else {
                // we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
            }
        }
    }

    if (heatedBedController.targetTemperatureC > MAX_ROOM_TEMPERATURE) {
        if (fabs(heatedBedController.targetTemperatureC - heatedBedController.currentTemperatureC) < RGB_LIGHT_TEMP_TOLERANCE) {
            // we have reached the target temperature
            if (newStatus == RGB_STATUS_IDLE) {
                newStatus = RGB_STATUS_PRINTING;
            }
        } else if (heatedBedController.targetTemperatureC > heatedBedController.currentTemperatureC) {
            // we are still heating
            newStatus = RGB_STATUS_HEATING;
        } else {
            // we end up here in case the target temperature is below the current temperature (this happens typically when the target temperature is reduced after the first layer)
        }
    }

    for (uint8_t i = 0; i < NUM_EXTRUDER; i++) {
        if ((extruder[i].tempControl.currentTemperatureC - extruder[i].tempControl.targetTemperatureC) > COOLDOWN_THRESHOLD) {
            // we shall cool down
            if (newStatus == RGB_STATUS_IDLE) {
                newStatus = RGB_STATUS_COOLING;
            }
        }
    }

    if ((heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) > COOLDOWN_THRESHOLD) {
        // we shall cool down
        if (newStatus == RGB_STATUS_IDLE) {
            newStatus = RGB_STATUS_COOLING;
        }
    }

    if (newStatus != lightStatus) {
        if (newStatus == RGB_STATUS_IDLE && lightStatus == RGB_STATUS_COLOR_CHANGE) {
            // when we are in color change mode already we shall not switch back to idle
        } else {
            lightStatus = newStatus;
            //          Com::printFLN( PSTR( "new RGB light status: " ), lightStatus );

            switch (lightStatus) {
            case RGB_STATUS_PRINTING: {
                setTargetColors(g_uRGBPrintingR, g_uRGBPrintingG, g_uRGBPrintingB);
                break;
            }
            case RGB_STATUS_HEATING: {
                setTargetColors(g_uRGBHeatingR, g_uRGBHeatingG, g_uRGBHeatingB);
                break;
            }
            case RGB_STATUS_COOLING: {
                setTargetColors(g_uRGBCoolingR, g_uRGBCoolingG, g_uRGBCoolingB);
                break;
            }
            case RGB_STATUS_IDLE: // fall through
            default: {
                idleStartedAt = HAL::timeInMilliseconds();
                lightStatus = RGB_STATUS_IDLE;
                setTargetColors(g_uRGBIdleR, g_uRGBIdleG, g_uRGBIdleB);
                break;
            }
            }
        }
    }

    // Entering IDLE always records the start, including clock value zero.
    if (lightStatus == RGB_STATUS_IDLE) {
        if ((static_cast<uint32_t>(HAL::timeInMilliseconds()) - idleStartedAt) / 1000 > RGB_LIGHT_COLOR_CHANGE_DELAY) {
            lightStatus = RGB_STATUS_COLOR_CHANGE;
            idleStartedAt = 0;

            //          Com::printFLN( PSTR( "new RGB light status: " ), lightStatus );
            setTargetColors(255, 0, 0);
        }
    }

    if (lightStatus == RGB_STATUS_COLOR_CHANGE) {
        if (g_uRGBTargetR == g_uRGBCurrentR && g_uRGBTargetG == g_uRGBCurrentG && g_uRGBTargetB == g_uRGBCurrentB) {
            if (g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 0) {
                g_uRGBTargetG = 255;
            } else if (g_uRGBTargetR == 255 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0) {
                g_uRGBTargetR = 0;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 0) {
                g_uRGBTargetB = 255;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 255 && g_uRGBTargetB == 255) {
                g_uRGBTargetG = 0;
            } else if (g_uRGBTargetR == 0 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255) {
                g_uRGBTargetR = 255;
            } else if (g_uRGBTargetR == 255 && g_uRGBTargetG == 0 && g_uRGBTargetB == 255) {
                g_uRGBTargetB = 0;
            }
        }
    }

    return;

} // updateRGBLightStatus

} // namespace RgbLighting
#endif
