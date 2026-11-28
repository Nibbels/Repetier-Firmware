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

// Chip selects on extended I/O require a read-modify-write. PORTL is
// shared with motion pins; protect ONLY the single GPIO update, not the frame.
// The guard restores the caller's SREG (it never unconditionally enables IRQs).
#define DRV8711_SET_SELECT(pin, select) \
    do {                               \
        InterruptProtectedBlock guard; \
        WRITE(pin, select);            \
    } while (0)

// Configure inactive selects once, before the first transaction. Keep each
// latch/direction update indivisible with respect to other PORTL users.
#define DRV8711_INIT_SELECT(pin) \
    do {                        \
        InterruptProtectedBlock guard; \
        WRITE(pin, LOW);         \
        SET_OUTPUT(pin);         \
    } while (0)

void configureChipSelects() {
    DRV8711_INIT_SELECT(O1_SCS_PIN);
    DRV8711_INIT_SELECT(O0_SCS_PIN);
    DRV8711_INIT_SELECT(Z_SCS_PIN);
    DRV8711_INIT_SELECT(Y_SCS_PIN);
    DRV8711_INIT_SELECT(X_SCS_PIN);
}
#undef DRV8711_INIT_SELECT

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

// TI SLVSC40H sections 6.6 and 7.5: SCLK <=4 MHz, high/low >=25 ns,
// SDATI setup >=5 ns, hold >=1 ns; SCS inactive >=100 ns. The supported
// 16 MHz AVR boards use atomic SBI/CBI on PD5/PD6. No ISR owns this bus.
// Foreground alone selects/transmits; IRQs may extend an interval, not
// change a bit or introduce an edge. Verify linked instructions in CI.
static inline void serialTimingMargin() {
    __asm__ __volatile__("nop\n\tnop" ::: "memory");
}

void drv8711Transmit(uint16_t command) {
    // Shift by a fixed one bit instead of recalculating (1 << i) on AVR.
    for (uint8_t remaining = 16; remaining; --remaining) {
        WRITE(DRV_SDATI, command & 0x8000U);
        WRITE(DRV_SCLK, HIGH);
        serialTimingMargin();
        WRITE(DRV_SCLK, LOW);
        command <<= 1;
    }
}

template <bool Broadcast = false>
void writeRegister(Channel channel, Register address, uint16_t data) {
    setChipSelect<true, Broadcast>(channel);
    drv8711Transmit(writeWord(address, data));
    setChipSelect<false, Broadcast>(channel);
    // Two cycles at 16 MHz alone satisfy the 100 ns inactive minimum.
    serialTimingMargin();
}

void broadcastRegister(Register address, uint16_t data) {
    writeRegister<true>(Channel::E1, address, data);
}

void adjustMicroSteps(Channel channel) {
    const uint8_t axis = static_cast<uint8_t>(channel) - 1;
    const uint8_t mode = clampedMode(microstepModes[axis]);
    // MODE occupies bits 3–6. Keep the configured enable, gain and dead time.
    writeRegister(channel, Register::Control, 0x0E01 | (mode << 3));
}

void writeMotorCurrent(Channel channel, uint8_t level) {
    // Above the motor's saturation current, additional current adds heat.
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
    configureChipSelects();

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

    // Leave BLANK (0x03) at its hardware reset defaults, TBLANK = 0x80 and ABT = 0.
    // The legacy command 0xB2D7 was a read. Writing BLANK can change motor behavior.

    // DECMOD = 5 (automatic mixed decay), TDECAY = 0x04 (2 us).
    broadcastRegister(Register::Decay, 0x504);

    broadcastRegister(Register::Stall, 0x83C);

    broadcastRegister(Register::Drive, 0x0F0);

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
    // Invalid raw channels must not clock an unselected bus.
    if (driver < 1 || driver > DRV8711_NUM_CHANNELS)
        return;
    writeMotorCurrent(static_cast<Channel>(driver), level);
}

void setExtruderCurrent(uint8_t nr, uint8_t current) {
    if (nr > NUM_EXTRUDER - 1)
        nr = NUM_EXTRUDER - 1;

    writeMotorCurrent(channelForAxis(E_AXIS + nr), current);

}
