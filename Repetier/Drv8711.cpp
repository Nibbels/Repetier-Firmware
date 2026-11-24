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
#error Wer DRV8711_NUM_CHANNELS ändert, muss sich wirklich reindenken und von den Achszuordnungen bis zu den EEPROM-Speicherplätzen alles überdenken. Normalerweise darf dieser Wert nicht anders als 5 sein. Ausser Conrad baut ein Board mit 3 Extrudersteppern oder ähnlich.
#endif // DRV8711_NUM_CHANNELS != 5

void drv8711Transmit(unsigned short command) {
    // transfer the command (= direction, address and data)
    InterruptProtectedBlock noInts; //HAL::forbidInterrupts();
    for (char i = 15; i >= 0; i--) {
        WRITE(DRV_SDATI, command & (0x01 << i));
        HAL::delayMicroseconds(1);
        WRITE(DRV_SCLK, 1);
        HAL::delayMicroseconds(5);
        WRITE(DRV_SCLK, 0);
    }
    noInts.unprotect(); //HAL::allowInterrupts();

} // drv8711Transmit

void drv8711EnableAll(void) {
    // enable the chip select of all present DRV8711
    switch (DRV8711_NUM_CHANNELS) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        SET_OUTPUT(O1_SCS_PIN);
        WRITE(O1_SCS_PIN, HIGH);
    } // fall through
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        SET_OUTPUT(O0_SCS_PIN);
        WRITE(O0_SCS_PIN, HIGH);
    } // fall through
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        SET_OUTPUT(Z_SCS_PIN);
        WRITE(Z_SCS_PIN, HIGH);
    } // fall through
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        SET_OUTPUT(Y_SCS_PIN);
        WRITE(Y_SCS_PIN, HIGH);
    } // fall through
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        SET_OUTPUT(X_SCS_PIN);
        WRITE(X_SCS_PIN, HIGH);
    }
    }

} // drv8711EnableAll

void drv8711DisableAll(void) {
    // disable the chip select of all present DRV8711
    switch (DRV8711_NUM_CHANNELS) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
    } // fall through
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
    } // fall through
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
    } // fall through
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
    } // fall through
    case 1: {
        WRITE(X_SCS_PIN, LOW);
    }
    }

} // drv8711DisableAll

void drv8711Enable(unsigned char driver) {
    // enable the chip select of the DRV8711
    switch (driver) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        SET_OUTPUT(O1_SCS_PIN);
        WRITE(O1_SCS_PIN, HIGH);
        break;
    }
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        SET_OUTPUT(O0_SCS_PIN);
        WRITE(O0_SCS_PIN, HIGH);
        break;
    }
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        SET_OUTPUT(Z_SCS_PIN);
        WRITE(Z_SCS_PIN, HIGH);
        break;
    }
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        SET_OUTPUT(Y_SCS_PIN);
        WRITE(Y_SCS_PIN, HIGH);
        break;
    }
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        SET_OUTPUT(X_SCS_PIN);
        WRITE(X_SCS_PIN, HIGH);
        break;
    }
    }

} // drv8711Enable

void drv8711Disable(unsigned char driver) {
    // disable the chip select of the DRV8711
    switch (driver) {
    case 5: {
        WRITE(O1_SCS_PIN, LOW);
        break;
    }
    case 4: {
        WRITE(O0_SCS_PIN, LOW);
        break;
    }
    case 3: {
        WRITE(Z_SCS_PIN, LOW);
        break;
    }
    case 2: {
        WRITE(Y_SCS_PIN, LOW);
        break;
    }
    case 1: {
        WRITE(X_SCS_PIN, LOW);
        break;
    }
    }

} // drv8711Disable

uint8_t getmsb(unsigned int x) {
    int r = 0;
    if (x < 1)
        return 0;
    while (x >>= 1)
        r++;
    return r;
}

uint8_t drv8711MicroSteps_2_ModeValue(unsigned short microsteps) { //unknown steps fall back to highest bit!
    return getmsb(microsteps);                                     //highest bit << 1 -> 256->8, 128->7, ... integer log2
} //drv8711MicroSteps_2_ModeValue

unsigned short drv8711Axis_2_InitMicrosteps(uint8_t axis) {
    return (axis == X_AXIS || axis == Y_AXIS ? RF_MICRO_STEPS_XY : (axis == Z_AXIS ? RF_MICRO_STEPS_Z : (axis == E_AXIS || axis == E_AXIS + 1 ? RF_MICRO_STEPS_E : RF_MICRO_STEPS_XY /*error fallback*/
                                                                                                         )));
}

unsigned short drv8711ModeValue_2_MicroSteps(uint8_t modeValue) {
    if (modeValue > 8)
        return 256; //constrain, but ERROR -> avoid this!
    return (1 << modeValue);
} // drv8711ModeValue_2_MicroSteps

unsigned short drv8711MicroSteps_2_Register00Hex(unsigned short microsteps) {

    /* http://www.ti.com/product/DRV8711/datasheet/detailed-description#SLVSC405764
    DTIME
    00: 400 ns dead time
    01: 450 ns dead time
    10: 650 ns dead time
    11: 850 ns dead time

    ISENSE amplifier gain set
    00: Gain of 5
    01: Gain of 10
    10: Gain of 20
    11: Gain of 40

    EXSTALL
    0: Internal stall detect
    1: External stall detect

    MODE
    0000: Full-step, 71% current
    0001: Half step
    0010: 1/4 step
    0011: 1/8 step
    0100: 1/16 step
    0101: 1/32 step
    0110: 1/64 step
    0111: 1/128 step
    1000: 1/256 step
    1001 – 1111: Reserved
    */
    // ADRESS 11..8 7..4 3..0
    unsigned short Register00Hex = 0x0E01; // 0000 1110 0XXX X001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = XXXX, EXSTALL = 0, ISGAIN = 10, DTIME = 11

    Register00Hex += (drv8711MicroSteps_2_ModeValue(microsteps) << 3); //schiebe zu bit 3..6

    return Register00Hex;
} // drv8711MicroSteps_2_Register00Hex

void drv8711adjustMicroSteps(unsigned char driver) {
    drv8711Enable(driver);
    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5
    unsigned short reg00bytes = drv8711MicroSteps_2_Register00Hex(drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[driver - 1 /*=axis*/]));
    if (reg00bytes)
        drv8711Transmit(reg00bytes);
    drv8711Disable(driver);
    HAL::delayMicroseconds(1);
}

void drv8711Init(void) {
    // configure the pins
    WRITE(DRV_RESET1, LOW);
    SET_OUTPUT(DRV_RESET1);


    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    // configure the following inputs as pullup
    WRITE(DRV_SDATO, HIGH);
    WRITE(DRV_FAULT, HIGH);
    WRITE(X_STALL_PIN, HIGH);
    WRITE(Y_STALL_PIN, HIGH);
    WRITE(Z_STALL_PIN, HIGH);
    WRITE(O0_STALL_PIN, HIGH);
    WRITE(O1_STALL_PIN, HIGH);

    // reset all DRV8711 (active high)
    WRITE(DRV_RESET1, HIGH);


    HAL::delayMicroseconds(5000);
    WRITE(DRV_RESET1, LOW);


    HAL::delayMicroseconds(5000);

    // configure all registers except the motor current (= no register 01)

    //DRV8711_REGISTER_00:
    //init to stock settings: update in eeprom init.
    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        Printer::motorMicroStepsModeValue[axis] = drv8711MicroSteps_2_ModeValue(drv8711Axis_2_InitMicrosteps(axis)); //init
    }
    for (uint8_t driver = 1; driver <= DRV8711_NUM_CHANNELS; driver++) {
        drv8711adjustMicroSteps(driver);
    }
    // ADRESS 11..8 7..4 3..0

// Register 0x02: OFF (Off-Time Register)
// TOFF wurde auf 0x20 (16µs) reduziert, um die Chopping-Frequenz zu erhöhen
// und hörbares Zwitschern zu beseitigen. – Beibehalten.
// PWMMODE = 0 (interner Indexer, Standard für Mikroschritt) – Beibehalten.
#define DRV8711_REGISTER_02 0x2020 // 0010 0000 0010 0000: TOFF = 00100000 (0x20), PWMMODE = 0

// Register 0x03: BLANK (Blanking Time Register) -- intentionally NOT written.
// drv8711Init() hardware-resets the drivers above, leaving the DRV8711 default:
// TBLANK = 0x80 (2.56 us), ABT = 0 (adaptive blanking disabled).
// The previously copied 0xB2D7 has SPI bit 15 set (READ), so the old
// drv8711Transmit(0xB2D7) never applied the intended blanking settings.
// Do not silently replace it with a write such as 0x3102 (1.0 us, ABT on):
// blanking/decay timing can affect motor current, torque and missed steps.
// Keeping BLANK at its reset default preserves the existing motor behavior.

// Register 0x04: DECAY (Decay Mode Register)
// TDECAY wurde von 0x14 (10µs) auf 0x04 (2µs) geändert.
// Eine kürzere Decay-Zeit kann in manchen Fällen zu einer noch glatteren
// Stromwellenform und somit zu einer weiteren Geräuschreduzierung führen.
// DECMOD = 101 (Auto Mixed Decay) – Beibehalten, da dies für leisen Betrieb vorteilhaft ist.
#define DRV8711_REGISTER_04 0x4504 // 0100 0101 0000 0100: TDECAY = 00000100 (0x04), DECMOD = 101

#define DRV8711_REGISTER_05 0x583C // 0101   1000  0011 1100: SDTHR = 00111100, SDCNT = 00, VDIV = 10
#define DRV8711_REGISTER_06 0x60F0 // 0110   0000  1111 0000: OCPTH = 00, OCPDEG = 00, TDRIVEN = 11, TDRIVEP = 11, IDRIVEN = 00, IDRIVEP = 00
#define DRV8711_REGISTER_07 0x7000 // 0111   0000  0000 0000: OTS = 0, AOCP = 0, BOCP = 0, UVLO = 0, APDF = 0, BPDF = 0, STD = 0, STDLAT = 0

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_02);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_04);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_05);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_06);
    drv8711DisableAll();
    HAL::delayMicroseconds(1);

    drv8711EnableAll();
    drv8711Transmit(DRV8711_REGISTER_07);
    drv8711DisableAll();

    // set all motor currents
    const unsigned short uMotorCurrentUse[] = MOTOR_CURRENT_NORMAL; //--> {x,y,z,e1,e2} siehe RFx000.h
    Printer::motorCurrent[X_AXIS] = uMotorCurrentUse[X_AXIS];
    Printer::motorCurrent[Y_AXIS] = uMotorCurrentUse[Y_AXIS];
    Printer::motorCurrent[Z_AXIS] = uMotorCurrentUse[Z_AXIS];
    Printer::motorCurrent[E_AXIS + 0] = uMotorCurrentUse[E_AXIS + 0];
    Printer::motorCurrent[E_AXIS + 1] = uMotorCurrentUse[E_AXIS + 1]; //egal ob NUM_EXTRUDER == 1 oder 2

    for (uint8_t axis = 0; axis < DRV8711_NUM_CHANNELS; axis++) {
        if (axis < 3 + NUM_EXTRUDER)
            setMotorCurrent(axis + 1, Printer::motorCurrent[axis]);
    }

} // drv8711Init

void setMotorCurrent(unsigned char driver, uint8_t level) {
    // NOTE: Do not increase the current endlessly. In case the engine reaches its current saturation, the engine and the driver can heat up and loss power.
    // When the saturation is reached, more current causes more heating and more power loss.
    // In case of engines with lower quality, the saturation current may be reached before the nominal current.

    //X = 1
    //Y = 2
    //Z = 3
    //E0 = 4
    //E1 = 5

    // configure the pins
    WRITE(DRV_SCLK, LOW);
    SET_OUTPUT(DRV_SCLK);
    WRITE(DRV_SDATI, LOW);
    SET_OUTPUT(DRV_SDATI);

    drv8711Enable(driver);

    // we have to write to register 01
    unsigned short command = 0x1100 + (short)level;
    drv8711Transmit(command);

    drv8711Disable(driver);

} // setMotorCurrent

void setExtruderCurrent(uint8_t nr, uint8_t current) {
    if (nr > NUM_EXTRUDER - 1)
        nr = NUM_EXTRUDER - 1; //0 ist T0, 1 ist T1


    // set the current for the extruder motor

    setMotorCurrent(1 + E_AXIS + nr, current); //das ist Extruder-ID+1, also nicht [0,1,2] -> Extruder 4 = [3], sondern [4]. Extruder5 wäre [5]

    if (Printer::debugInfo()) {
        Com::printF(PSTR("Extruder"), (short)nr);
        Com::printFLN(PSTR(" = "), (uint32_t)current);
    }
} // setExtruderCurrent
