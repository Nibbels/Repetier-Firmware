/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Repetier.h"

#if DRV8711_NUM_CHANNELS != 5
#error DRV8711 channel mapping and EEPROM layout require five channels.
#endif

namespace {

// Configuration shadow, not a readback of the write-only driver registers.
// Only foreground setup, persistence and panel settings access these bytes.
uint8_t configuredCurrents[DRV8711_NUM_CHANNELS] = { 0 };
uint8_t microstepModes[DRV8711_NUM_CHANNELS] = { 0 };

enum class Channel : uint8_t { X = 1, Y, Z, E0, E1 };
enum class Register : uint8_t {
    Control = 0,
    Torque = 1,
    OffTime = 2,
    Decay = 4,
    Stall = 5,
    Drive = 6,
    Status = 7
};

constexpr uint16_t writeWord(Register address, uint16_t data) {
    return (static_cast<uint16_t>(address) << 12) | data;
}

constexpr uint8_t clampedMode(uint8_t mode) {
    return mode > 8 ? 8 : mode;
}

Channel channelForAxis(uint8_t axis) {
    return static_cast<Channel>(axis + 1);
}

void configureSerialOutputs() {
    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);
}

// fastio requires the pin as a macro argument to generate direct GPIO writes.
#define DRV8711_SET_SELECT(pin, select) \
    do {                               \
        WRITE(pin, LOW);               \
        if (select) {                  \
            SET_OUTPUT(pin);           \
            WRITE(pin, HIGH);          \
        }                              \
    } while (0)

template <bool Select, bool Broadcast = false>
void setChipSelect(Channel channel) {
    // Broadcast starts at E1 and visits the pins in their original order.
    switch (channel) {
    case Channel::E1:
        DRV8711_SET_SELECT(O1_SCS_PIN, Select);
        if (!Broadcast)
            return;
        // fall through
    case Channel::E0:
        DRV8711_SET_SELECT(O0_SCS_PIN, Select);
        if (!Broadcast)
            return;
        // fall through
    case Channel::Z:
        DRV8711_SET_SELECT(Z_SCS_PIN, Select);
        if (!Broadcast)
            return;
        // fall through
    case Channel::Y:
        DRV8711_SET_SELECT(Y_SCS_PIN, Select);
        if (!Broadcast)
            return;
        // fall through
    case Channel::X:
        DRV8711_SET_SELECT(X_SCS_PIN, Select);
        break;
    }
}
#undef DRV8711_SET_SELECT

void drv8711Transmit(unsigned short command) {
    InterruptProtectedBlock noInts;
    for (char i = 15; i >= 0; i--) {
        WRITE(DRV_SDATI, command & (0x01 << i));
        HAL::delayMicroseconds(1);
        WRITE(DRV_SCLK, 1);
        HAL::delayMicroseconds(5);
        WRITE(DRV_SCLK, 0);
    }
    noInts.unprotect();
}

template <bool Broadcast = false>
void writeRegister(Channel channel, Register address, uint16_t data) {
    setChipSelect<true, Broadcast>(channel);
    drv8711Transmit(writeWord(address, data));
    setChipSelect<false, Broadcast>(channel);
}

void broadcastRegister(Register address, uint16_t data) {
    writeRegister<true>(Channel::E1, address, data);
}

void adjustMicroSteps(Channel channel) {
    const uint8_t axis = static_cast<uint8_t>(channel) - 1;
    const uint8_t mode = clampedMode(microstepModes[axis]);
    // MODE occupies bits 3–6. Keep the configured enable, gain and dead time.
    writeRegister(channel, Register::Control, 0x0E01 | (mode << 3));
    HAL::delayMicroseconds(1);
}

void writeMotorCurrent(Channel channel, uint8_t level) {
    // Above the motor's saturation current, additional current adds heat.
    configureSerialOutputs();
    writeRegister(channel, Register::Torque, 0x0100 | level);
}

} // namespace

uint8_t drv8711MicroSteps_2_ModeValue(unsigned short microsteps) {
    // Non-power-of-two values use the position of the highest set bit.
    uint8_t mode = 0;
    while (microsteps >>= 1)
        ++mode;
    return mode;
}

unsigned short drv8711Axis_2_InitMicrosteps(uint8_t axis) {
    if (axis == Z_AXIS)
        return RF_MICRO_STEPS_Z;
    if (axis == E_AXIS || axis == E_AXIS + 1)
        return RF_MICRO_STEPS_E;
    return RF_MICRO_STEPS_XY;
}

unsigned short drv8711ModeValue_2_MicroSteps(uint8_t modeValue) {
    return 1U << clampedMode(modeValue);
}

namespace Drv8711 {

uint8_t configuredCurrent(uint8_t axis) {
    return configuredCurrents[axis];
}

void stageCurrent(uint8_t axis, uint8_t level) {
    configuredCurrents[axis] = level;
}

void applyConfiguredCurrent(uint8_t axis) {
    writeMotorCurrent(channelForAxis(axis), configuredCurrents[axis]);
}

uint8_t microstepMode(uint8_t axis) {
    return microstepModes[axis];
}

void setMicrostepMode(uint8_t axis, uint8_t mode) {
    microstepModes[axis] = mode;
    adjustMicroSteps(channelForAxis(axis));
}

bool setPairedMicrostepMode(uint8_t firstAxis, uint8_t mode) {
    // Preserve the panel's existing paired-axis policy: only reprogram both
    // drivers when the second axis differs from the requested primary mode.
    microstepModes[firstAxis] = mode;
    if (microstepModes[firstAxis + 1] == mode)
        return false;
    microstepModes[firstAxis + 1] = mode;
    adjustMicroSteps(channelForAxis(firstAxis));
    adjustMicroSteps(channelForAxis(firstAxis + 1));
    return true;
}

} // namespace Drv8711

void drv8711Init() {
    WRITE(DRV_RESET1, LOW);
    SET_OUTPUT(DRV_RESET1);
    configureSerialOutputs();

    // Configure the inputs as pullups.
    WRITE(DRV_SDATO, HIGH);
    WRITE(DRV_FAULT, HIGH);
    WRITE(X_STALL_PIN, HIGH);
    WRITE(Y_STALL_PIN, HIGH);
    WRITE(Z_STALL_PIN, HIGH);
    WRITE(O0_STALL_PIN, HIGH);
    WRITE(O1_STALL_PIN, HIGH);

    // Reset all DRV8711s (active high).
    WRITE(DRV_RESET1, HIGH);
    HAL::delayMicroseconds(5000);
    WRITE(DRV_RESET1, LOW);
    HAL::delayMicroseconds(5000);

    // Start with the board's microstep defaults until EEPROM settings load.
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        microstepModes[axis] = drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(axis));
    }
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        adjustMicroSteps(channelForAxis(axis));
    }

    // TOFF = 0x20 (16 us) suppresses audible chirping. PWMMODE = 0 uses the indexer.
    broadcastRegister(Register::OffTime, 0x020);
    HAL::delayMicroseconds(1);

    // Leave BLANK (0x03) at its hardware reset defaults, TBLANK = 0x80 and ABT = 0.
    // The legacy command 0xB2D7 was a read. Writing BLANK can change motor behavior.

    // DECMOD = 5 (automatic mixed decay), TDECAY = 0x04 (2 us).
    broadcastRegister(Register::Decay, 0x504);
    HAL::delayMicroseconds(1);

    broadcastRegister(Register::Stall, 0x83C);
    HAL::delayMicroseconds(1);

    broadcastRegister(Register::Drive, 0x0F0);
    HAL::delayMicroseconds(1);

    broadcastRegister(Register::Status, 0x000);

    // Initialize the saved defaults for all channels, then program the fitted motors.
    const unsigned short currents[] = MOTOR_CURRENT_NORMAL;
    configuredCurrents[X_AXIS] = currents[X_AXIS];
    configuredCurrents[Y_AXIS] = currents[Y_AXIS];
    configuredCurrents[Z_AXIS] = currents[Z_AXIS];
    configuredCurrents[E_AXIS] = currents[E_AXIS];
    configuredCurrents[E_AXIS + 1] = currents[E_AXIS + 1];

    for (uint8_t axis = 0; axis < 3 + NUM_EXTRUDER; axis++) {
        writeMotorCurrent(channelForAxis(axis), configuredCurrents[axis]);
    }
}

void setMotorCurrent(unsigned char driver, uint8_t level) {
    writeMotorCurrent(static_cast<Channel>(driver), level);
}

void setExtruderCurrent(uint8_t nr, uint8_t current) {
    if (nr > NUM_EXTRUDER - 1)
        nr = NUM_EXTRUDER - 1;

    writeMotorCurrent(channelForAxis(E_AXIS + nr), current);

    if (Printer::debugInfo()) {
        Com::printF(PSTR("Extruder"), (short)nr);
        Com::printFLN(PSTR(" = "), (uint32_t)current);
    }
}
