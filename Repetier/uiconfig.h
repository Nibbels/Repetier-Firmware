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

#ifndef UI_CONFIG_H
#define UI_CONFIG_H

/* What type of chip is used for I2C communication
0 : PCF8574 or PCF8574A or compatible chips.
1 : MCP23017
*/
#define UI_DISPLAY_I2C_CHIPTYPE 0

// 0x40 till 0x4e for PCF8574, 0x40 for the adafruid RGB shield, 0x40 - 0x4e for MCP23017
// Official addresses have a value half as high!
#define UI_DISPLAY_I2C_ADDRESS 0x4e

// For MCP 23017 define which pins should be output
#define UI_DISPLAY_I2C_OUTPUT_PINS 65504

// Set the output mask that is or'd over the output data. This is needed to activate
// a backlight switched over the I2C.
// The adafruit RGB shields enables a light if the bit is not set. Bits 6-8 are used for backlight.
#define UI_DISPLAY_I2C_OUTPUT_START_MASK 0

// For MCP which inputs are with pullup. 31 = pins 0-4 for adafruid rgb shield buttons
#define UI_DISPLAY_I2C_PULLUP 31

/* How fast should the I2C clock go. The PCF8574 work only with the lowest setting 100000.
A MCP23017 can run also with 400000 Hz */
#define UI_I2C_CLOCKSPEED 100000L

// Under which address can the key status requested. This is the address of your PCF8574 where the keys are connected.
// If you use a MCP23017 the address from display is used also for keys.
#define UI_I2C_KEY_ADDRESS 0x40

#endif // UI_CONFIG_H
