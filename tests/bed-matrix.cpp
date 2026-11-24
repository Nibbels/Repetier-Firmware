#include <cassert>
#include <cstdint>
#include <cstring>
#include "TwiTransfer.h"

using byte = uint8_t;
enum { X_AXIS, Y_AXIS, Z_AXIS };
enum { COMPENSATION_MATRIX_MAX_X = 6, COMPENSATION_MATRIX_MAX_Y = 5 };
enum { EEPROM_FORMAT = 7, EEPROM_DELAY = 10, I2C_ADDRESS_EXTERNAL_EEPROM = 0x50 };
enum { EEPROM_OFFSET_HEADER_FORMAT = 0, EEPROM_OFFSET_SECTOR_FORMAT = 0,
       EEPROM_OFFSET_DIMENSION_X = 2, EEPROM_OFFSET_DIMENSION_Y = 4,
       EEPROM_OFFSET_MICRO_STEPS = 6, EEPROM_OFFSET_MATRIX_START = 28,
       EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX = 4, EEPROM_MAX_HEAT_BED_SECTORS = 9 };
#define PSTR(text) text

short g_ZCompensationMatrix[COMPENSATION_MATRIX_MAX_X][COMPENSATION_MATRIX_MAX_Y];
uint8_t g_uZMatrixMax[2];
bool g_ZMatrixChangedInRam;
long g_offsetZCompensationSteps;
short g_ZCompensationMax;
char g_nActiveHeatBed = 1;

constexpr unsigned int EEPROM_SECTOR_SIZE = 128;
constexpr unsigned int sector = EEPROM_SECTOR_SIZE;
constexpr unsigned int payload = sector + EEPROM_OFFSET_MATRIX_START;
uint8_t eeprom[512]{};
unsigned address = 0;
int readFailure = -1;
int writeFailure = -1;
unsigned payloadReads = 0;
unsigned writes = 0;
bool writeProtected = false;
bool scanning = false;

namespace HAL {
uint32_t timeInMilliseconds() { return 0; }
void delayMilliseconds(unsigned int) {}
}
namespace Printer {
float axisLengthMM[3] = {50, 40, 200};
float axisStepsPerMM[3] = {100, 100, 100};
float stepsPerMm(uint8_t axis) { return axisStepsPerMM[axis]; }
float axisLengthMm(uint8_t axis) { return axisLengthMM[axis]; }
void setAxisLengthMm(uint8_t axis, float value) { axisLengthMM[axis] = value; }

bool debugErrors() { return true; }
bool debugInfo() { return false; }
void disableCMPnow(bool wait) { assert(wait); }
}
namespace Com {
constexpr const char* tError = "error";
template<typename... T> void printF(T...) {}
template<typename... T> void printFLN(T...) {}
}
enum { Processing };
namespace GCodeInput {
void keepAlive(int state) { assert(state == Processing); }
}
namespace Drv8711 { uint8_t microstepMode(uint8_t) { return 5; } }
unsigned short drv8711ModeValue_2_MicroSteps(uint8_t mode) { return 1 << mode; }
static void determineCompensationOffsetZ();
bool isAnyScanRunning() { return scanning; }

void TwiTransfer::finishActive() {}

bool TwiTransfer::write(uint8_t device, const uint8_t* bytes, uint8_t length, uint32_t) {
    assert(device == I2C_ADDRESS_EXTERNAL_EEPROM && (length == 2 || length == 3));
    address = (unsigned(bytes[0]) << 8) | bytes[1];
    assert(address < sizeof(eeprom));
    state_ = State::Done;
    if (length == 3) {
        ++writes;
        if (int(address) == writeFailure)
            state_ = State::Error;
        else if (!writeProtected)
            eeprom[address] = bytes[2];
    }
    return true;
}

bool TwiTransfer::read(uint8_t device, uint8_t length, uint32_t) {
    assert(device == I2C_ADDRESS_EXTERNAL_EEPROM && length == 1);
    if (address >= payload)
        ++payloadReads;
    state_ = int(address) == readFailure ? State::Error : State::Done;
    bytes_[0] = eeprom[address];
    return true;
}

bool TwiTransfer::wait() { return succeeded(); }

long getZMatrixDepth_CurrentXY();
void prepareZCompensation() { assert(false && "unexpected reload in diagnostic replay"); }
static char loadCompensationMatrix(unsigned int, void (*progress)() = nullptr);
#include "bed-matrix-firmware.inc"
long getZMatrixDepth_CurrentXY() { return getZMatrixDepth(0, 0); }

void storeWord(unsigned offset, uint16_t value) {
    eeprom[offset] = value >> 8;
    eeprom[offset + 1] = value;
}

void storeMatrix(unsigned lastX, unsigned lastY, unsigned microsteps = 32) {
    storeWord(EEPROM_OFFSET_HEADER_FORMAT, EEPROM_FORMAT);
    storeWord(sector + EEPROM_OFFSET_SECTOR_FORMAT, EEPROM_FORMAT);
    storeWord(sector + EEPROM_OFFSET_DIMENSION_X, lastX);
    storeWord(sector + EEPROM_OFFSET_DIMENSION_Y, lastY);
    storeWord(sector + EEPROM_OFFSET_MICRO_STEPS, microsteps);
    unsigned offset = payload;
    for (unsigned x = 0; x <= lastX; ++x) {
        for (unsigned y = 0; y <= lastY; ++y) {
            int16_t value;
            if (!x && !y)
                value = EEPROM_FORMAT;
            else if (!y)
                value = 10 * x;
            else if (!x)
                value = 10 * y;
            else
                value = 9 - 6 * int(x) - 4 * int(y);
            storeWord(offset, value);
            offset += 2;
        }
    }
    payloadReads = 0;
}

int main() {
    storeMatrix(COMPENSATION_MATRIX_MAX_X - 1, COMPENSATION_MATRIX_MAX_Y - 1);
    g_ZMatrixChangedInRam = true;
    assert(loadCompensationMatrix(sector) == 0);
    assert(g_ZCompensationMatrix[0][0] == EEPROM_FORMAT && g_ZCompensationMatrix[1][1] == -1);
    assert(g_offsetZCompensationSteps == -1 && g_ZCompensationMax == -37 && !g_ZMatrixChangedInRam);
    assert(g_uZMatrixMax[X_AXIS] == COMPENSATION_MATRIX_MAX_X - 1);
    assert(g_uZMatrixMax[Y_AXIS] == COMPENSATION_MATRIX_MAX_Y - 1);
    assert(getZMatrixDepth(0, 0) == -1);
    assert(getZMatrixDepth(1000, 1000) == -1);
    assert(getZMatrixDepth(1500, 1500) == -6);
    assert(getZMatrixDepth(2000, 0) == -7);
    assert(getZMatrixDepth(9999, 1500) == -27);
    assert(getZMatrixDepth(9999, 9999) == -37);

    adjustCompensationMatrix(-20);
    assert(g_offsetZCompensationSteps == -20 && g_ZCompensationMax == -56);
    assert(getZMatrixDepth(1500, 1500) == -25 && !g_ZMatrixChangedInRam);
    setMatrixNull();
    assert(g_ZMatrixChangedInRam && g_offsetZCompensationSteps == 0 && g_ZCompensationMax == 0);
    g_ZCompensationMatrix[3][2] = -250;
    g_ZMatrixChangedInRam = false;
    fixKeramikLochInMatrix();
    assert(g_ZCompensationMatrix[3][2] == 0 && g_ZMatrixChangedInRam);
    assert(g_offsetZCompensationSteps == 0 && g_ZCompensationMax == 0);
    assert(g_ZCompensationMatrix[1][0] == 10 && g_ZCompensationMatrix[0][1] == 10);
    g_ZMatrixChangedInRam = false;
    fixKeramikLochInMatrix();
    assert(!g_ZMatrixChangedInRam);
    g_ZCompensationMatrix[1][1] = -250;
    fixKeramikLochInMatrix();
    assert(g_ZCompensationMatrix[1][1] == -250 && !g_ZMatrixChangedInRam);

    storeMatrix(COMPENSATION_MATRIX_MAX_X, 2);
    assert(loadCompensationMatrix(sector) != 0 && payloadReads == 0 && !g_ZCompensationMatrix[0][0]);
    storeMatrix(2, COMPENSATION_MATRIX_MAX_Y);
    assert(loadCompensationMatrix(sector) != 0 && payloadReads == 0 && !g_ZCompensationMatrix[0][0]);
    storeMatrix(2, 1);
    assert(loadCompensationMatrix(sector) != 0 && payloadReads == 0);
    storeMatrix(2, 2, 0);
    assert(loadCompensationMatrix(sector) != 0 && payloadReads == 0);
    storeMatrix(2, 2, 3);
    assert(loadCompensationMatrix(sector) != 0 && payloadReads == 0);
    storeMatrix(2, 2, 16);
    assert(loadCompensationMatrix(sector) == 0 && g_ZCompensationMatrix[2][2] == -22);
    storeWord(payload + 2 * (1 * 3 + 1), 0x7fff);
    assert(loadCompensationMatrix(sector) != 0 && !g_ZCompensationMatrix[0][0]);
    storeMatrix(2, 2);
    assert(loadCompensationMatrix(sector) == 0);
    readFailure = payload + 7;
    assert(loadCompensationMatrix(sector) != 0 && !g_ZCompensationMatrix[0][0]);
    readFailure = -1;
    assert(loadCompensationMatrix(sector) == 0);
    storeWord(payload, EEPROM_FORMAT - 1);
    assert(loadCompensationMatrix(sector) != 0 && !g_ZCompensationMatrix[0][0]);

    storeMatrix(COMPENSATION_MATRIX_MAX_X - 1, COMPENSATION_MATRIX_MAX_Y - 1);
    assert(loadCompensationMatrix(sector) == 0);
    Printer::setAxisLengthMm(X_AXIS, 51);
    assert(prepareCompensationMatrix() != 0 && g_ZCompensationMatrix[1][0] == 10);
    Printer::setAxisLengthMm(X_AXIS, 50);
    Printer::setAxisLengthMm(Y_AXIS, 41);
    assert(prepareCompensationMatrix() != 0 && g_ZCompensationMatrix[0][1] == 10);

    storeMatrix(COMPENSATION_MATRIX_MAX_X - 2, COMPENSATION_MATRIX_MAX_Y - 2);
    assert(loadCompensationMatrix(sector) == 0 && prepareCompensationMatrix() == 0);
    assert(g_uZMatrixMax[X_AXIS] == COMPENSATION_MATRIX_MAX_X - 1);
    assert(g_uZMatrixMax[Y_AXIS] == COMPENSATION_MATRIX_MAX_Y - 1);
    assert(g_ZCompensationMatrix[1][0] == 0 && g_ZCompensationMatrix[0][1] == 0);
    assert(g_ZCompensationMatrix[5][0] == 50 && g_ZCompensationMatrix[0][4] == 41);
    assert(g_ZCompensationMatrix[5][4] == g_ZCompensationMatrix[4][3]);

    g_ZMatrixChangedInRam = true;
    writeProtected = true;
    assert(!saveCompensationMatrix(sector));
    assert(g_ZMatrixChangedInRam && ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, sector) == EEPROM_FORMAT);
    writeProtected = false;
    writeFailure = payload + 7;
    assert(!saveCompensationMatrix(sector));
    assert(g_ZMatrixChangedInRam && ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, sector) == 0);
    writeFailure = -1;
    assert(saveCompensationMatrix(sector * 2) && g_ZMatrixChangedInRam);
    assert(saveActiveBedMatrix() && !g_ZMatrixChangedInRam);
    assert(ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, sector) == EEPROM_FORMAT);
    assert(ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, sector + EEPROM_OFFSET_DIMENSION_X) == g_uZMatrixMax[X_AXIS]);
    assert(ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, sector + EEPROM_OFFSET_DIMENSION_Y) == g_uZMatrixMax[Y_AXIS]);
    assert(ExternalEeprom::readWord(I2C_ADDRESS_EXTERNAL_EEPROM, sector + EEPROM_OFFSET_MICRO_STEPS) == 32);
    unsigned offset = payload;
    for (unsigned x = 0; x <= g_uZMatrixMax[X_AXIS]; ++x)
        for (unsigned y = 0; y <= g_uZMatrixMax[Y_AXIS]; ++y) {
            const uint16_t word = static_cast<uint16_t>(g_ZCompensationMatrix[x][y]);
            assert(eeprom[offset++] == (word >> 8));
            assert(eeprom[offset++] == (word & 0xff));
        }
    assert(loadCompensationMatrix(sector) == 0 && g_ZCompensationMatrix[0][4] == 41);

    const unsigned writesBefore = writes;
    scanning = true;
    assert(!saveCompensationMatrix(sector) && writes == writesBefore);
    scanning = false;
    initCompensationMatrix();
    assert(!saveCompensationMatrix(sector) && writes == writesBefore);
    assert(loadCompensationMatrix(sector) == 0);
    assert(clearCompensationMatrix(sector) && loadCompensationMatrix(sector) != 0);

    storeWord(EEPROM_OFFSET_ACTIVE_HEAT_BED_Z_MATRIX, 1);
    writeProtected = true;
    assert(!saveActiveHeatBedToExtEEPROM(2));
    assert(loadActiveHeatBedFromExtEEPROM() && g_nActiveHeatBed == 1);
    writeProtected = false;
    assert(saveActiveHeatBedToExtEEPROM(2) && loadActiveHeatBedFromExtEEPROM() && g_nActiveHeatBed == 2);
}
