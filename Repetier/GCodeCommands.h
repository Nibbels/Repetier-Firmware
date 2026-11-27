#ifndef GCODE_COMMANDS_H
#define GCODE_COMMANDS_H

#include <stdint.h>

class GCode;

namespace GCodeCommands {

// The receiver handles these M codes before queuing and decodes panel text.
enum : uint16_t { ResetLineNumber = 110, EmergencyReset = 112, PanelMessage = 117 };
constexpr bool takesPanelText(uint16_t code) { return code == PanelMessage; }

void showInvalidSyntax(unsigned int currentMCode);

// Motion, extrusion and pressure.
void queueLinearMove(GCode* com);
void dwell(GCode* com);
void homeAxes(GCode* com);
void selectAbsolutePositioning(GCode* com);
void selectRelativePositioning(GCode* com);
void setLogicalPosition(GCode* com);
void selectAbsoluteExtrusion();
void selectRelativeExtrusion();
void configureStepperShutdown(GCode* com);
void setPrintAcceleration(GCode* com);
void setTravelAcceleration(GCode* com);
void setFeedrateMultiplier(GCode* com);
void setFlowMultiplier(GCode* com);
void configureAdvance(GCode* com);
void finishMotion();
void setManualZOffset(GCode* pCommand);
void testHomingRepeatability(GCode* pCommand);
void requestPrintPause(GCode* pCommand);
void setPauseParkingOffsets(GCode* pCommand);
void setEmergencyPauseDigits(GCode* pCommand);
void setSenseOffsetParameters(GCode* pCommand);
void setDigitFlowParameters(GCode* pCommand);

// Thermal control and board outputs.
void setHotendTemperature(GCode* com);
void reportTemperature(GCode* com);
void startPartFan(GCode* com);
void stopPartFan();
void waitForHotend(GCode* com);
void waitForTemperatures();
void setBedTemperature(GCode* com);
void waitForBed(GCode* com);
void setCaseLight(GCode* com);
void turnOnCaseFan();
void scheduleCaseFanOff(GCode* pCommand);
void setAccessoryFet(GCode* pCommand);
void setMainsOutput(GCode* pCommand);
void loadFilament(GCode* pCommand);
void unloadFilament(GCode* pCommand);
void readOptionalTemperature();

// Configuration, diagnostics and panel access.
bool rejectLegacySimulation(GCode* com);
void setDebugLevel(GCode* com);
void reportPosition();
void reportFirmware();
void setPanelMessage(GCode* com);
void reportEndstops();
void reportEeprom();
void setEepromValue(GCode* com);
void reportConfiguration();
void restoreConfigurationDefaults();
void accessPanel(GCode* pCommand);

// Operations backed by calibration state in RF.cpp.
namespace Calibration {
void disableBedCompensation();
void enableBedCompensation();
void selectBedMatrix(GCode* pCommand);
void reportBedMatrix(GCode* pCommand);
void configureZOffsetScan(GCode* pCommand);
void adjustBedMatrix(GCode* pCommand);
} // namespace Calibration

} // namespace GCodeCommands

#endif
