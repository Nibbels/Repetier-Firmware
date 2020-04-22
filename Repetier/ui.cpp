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

#define UI_MAIN
#include "Repetier.h"
extern const int8_t encoder_table[16] PROGMEM;

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#if UI_PRINT_AUTORETURN_TO_MENU_AFTER || UI_MILL_AUTORETURN_TO_MENU_AFTER
millis_t g_nAutoReturnTime = 0;
bool g_nAutoReturnMessage = false;
#endif // UI_PRINT_AUTORETURN_TO_MENU_AFTER || UI_MILL_AUTORETURN_TO_MENU_AFTER

char g_nYesNo = 0; // 0 = no, 1 = yes
volatile char g_nContinueButtonPressed = 0;
char g_nServiceRequest = 0;

void beep(uint8_t duration, uint8_t count) {
#if FEATURE_BEEPER && defined(BEEPER_PIN) && BEEPER_PIN >= 0
    if (!Printer::enableBeeper) {
        // we shall not beep
        return;
    }

    SET_OUTPUT(BEEPER_PIN);
#ifndef BEEPER_TYPE_INVERTING
#define BEEPER_TYPE_INVERTING false
#endif // BEEPER_TYPE_INVERTING

    for (uint8_t i = 0; i < count; i++) {
#if BEEPER_TYPE_INVERTING
        WRITE(BEEPER_PIN, LOW);
        HAL::delayMilliseconds(duration);
        WRITE(BEEPER_PIN, HIGH);
#else
        WRITE(BEEPER_PIN, HIGH);
        HAL::delayMilliseconds(duration);
        WRITE(BEEPER_PIN, LOW);
#endif // BEEPER_TYPE_INVERTING
        HAL::delayMilliseconds(duration);
    }
#endif // FEATURE_BEEPER && defined(BEEPER_PIN) && BEEPER_PIN>=0
} // beep

bool UIMenuEntry::showEntry() const {
    bool ret = true;
    uint8_t f, f2;

    f = HAL::readFlashByte((const prog_char*)&filter);
    if (f != 0)
        ret = (f & Printer::menuMode) != 0;
    f2 = HAL::readFlashByte((const prog_char*)&nofilter);
    if (ret && f2 != 0) {
        ret = (f2 & Printer::menuMode) == 0;
    }
    return ret;
} // showEntry

#if UI_DISPLAY_TYPE != 0
UIDisplay uid;
char displayCache[UI_ROWS][MAX_COLS + 1];

// Menu up sign - code 1
// ..*.. 4
// .***. 14
// *.*.* 21
// ..*.. 4
// ***.. 28
// ..... 0
// ..... 0
// ..... 0
const uint8_t character_back[8] PROGMEM = { 4, 14, 21, 4, 28, 0, 0, 0 };

// Degrees sign - code 2
// ..*.. 4
// .*.*. 10
// ..*.. 4
// ..... 0
// ..... 0
// ..... 0
// ..... 0
// ..... 0
const uint8_t character_degree[8] PROGMEM = { 4, 10, 4, 0, 0, 0, 0, 0 };

// selected - code 3
// ..... 0
// ***** 31
// ***** 31
// ***** 31
// ***** 31
// ***** 31
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_selected[8] PROGMEM = { 0, 31, 31, 31, 31, 31, 0, 0 };

// unselected - code 4
// ..... 0
// ***** 31
// *...* 17
// *...* 17
// *...* 17
// *...* 17
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_unselected[8] PROGMEM = { 0, 31, 17, 17, 17, 31, 0, 0 };

// unselected - code 5
// ..*.. 4
// .*.*. 10
// .*.*. 10
// .*.*. 10
// .*.*. 10
// .***. 14
// ***** 31
// ***** 31
// .***. 14
const uint8_t character_temperature[8] PROGMEM = { 4, 10, 10, 10, 14, 31, 31, 14 };

// unselected - code 6
// ..... 0
// ***.. 28
// ***** 31
// *...* 17
// *...* 17
// ***** 31
// ..... 0
// ..... 0
const uint8_t character_folder[8] PROGMEM = { 0, 28, 31, 17, 17, 31, 0, 0 };

// printer ready - code 7
// *...* 17
// .*.*. 10
// ..*.. 4
// *...* 17
// ..*.. 4
// .*.*. 10
// *...* 17
// *...* 17
const byte character_ready[8] PROGMEM = { 17, 10, 4, 17, 4, 10, 17, 17 };

const long baudrates[] PROGMEM = { 9600, 14400, 19200, 28800, 38400, 56000, 57600, 76800, 111112, 115200, 128000, 230400, 250000, 256000,
                                   460800, 500000, 921600, 1000000, 1500000, 0 };

const byte c1[8] PROGMEM = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00001,
    B00010,
    B00011,
    B00011
};

const byte c2[8] PROGMEM = {
    B00000,
    B00000,
    B00101,
    B01011,
    B01111,
    B10111,
    B11111,
    B11110
};

const byte c3[8] PROGMEM = {
    B00000,
    B01001,
    B10110,
    B01111,
    B11111,
    B11111,
    B00111,
    B00011
};

const byte c4[8] PROGMEM = {
    B01111,
    B01011,
    B01111,
    B01111,
    B10111,
    B10111,
    B01111,
    B10111
};

const byte c7[8] PROGMEM = {
    B01111,
    B10111,
    B01011,
    B00111,
    B00001,
    B00011,
    B00000,
    B00000
};

const byte c8[8] PROGMEM = {
    B00000,
    B11000,
    B11000,
    B11100,
    B11110,
    B11111,
    B11111,
    B00111
};

const byte c9[8] PROGMEM = {
    B00000,
    B00000,
    B00001,
    B00011,
    B10111,
    B11111,
    B11111,
    B11110
};

bool normalchars = false;

#define LCD_ENTRYMODE 0x04 /**< Set entrymode */

/** @name GENERAL COMMANDS */
/*@{*/
#define LCD_CLEAR 0x01 /**< Clear screen */
#define LCD_HOME 0x02  /**< Cursor move to first digit */
/*@}*/

/** @name ENTRYMODES */
/*@{*/
#define LCD_ENTRYMODE 0x04                       /**< Set entrymode */
#define LCD_INCREASE LCD_ENTRYMODE | 0x02        /**<    Set cursor move direction -- Increase */
#define LCD_DECREASE LCD_ENTRYMODE | 0x00        /**<    Set cursor move direction -- Decrease */
#define LCD_DISPLAYSHIFTON LCD_ENTRYMODE | 0x01  /**<    Display is shifted */
#define LCD_DISPLAYSHIFTOFF LCD_ENTRYMODE | 0x00 /**<    Display is not shifted */
/*@}*/

/** @name DISPLAYMODES */
/*@{*/
#define LCD_DISPLAYMODE 0x08                   /**< Set displaymode */
#define LCD_DISPLAYON LCD_DISPLAYMODE | 0x04   /**<    Display on */
#define LCD_DISPLAYOFF LCD_DISPLAYMODE | 0x00  /**<    Display off */
#define LCD_CURSORON LCD_DISPLAYMODE | 0x02    /**<    Cursor on */
#define LCD_CURSOROFF LCD_DISPLAYMODE | 0x00   /**<    Cursor off */
#define LCD_BLINKINGON LCD_DISPLAYMODE | 0x01  /**<    Blinking on */
#define LCD_BLINKINGOFF LCD_DISPLAYMODE | 0x00 /**<    Blinking off */
/*@}*/

/** @name SHIFTMODES */
/*@{*/
#define LCD_SHIFTMODE 0x10                    /**< Set shiftmode */
#define LCD_DISPLAYSHIFT LCD_SHIFTMODE | 0x08 /**<    Display shift */
#define LCD_CURSORMOVE LCD_SHIFTMODE | 0x00   /**<    Cursor move */
#define LCD_RIGHT LCD_SHIFTMODE | 0x04        /**<    Right shift */
#define LCD_LEFT LCD_SHIFTMODE | 0x00         /**<    Left shift */
/*@}*/

/** @name DISPLAY_CONFIGURATION */
/*@{*/
#define LCD_CONFIGURATION 0x20             /**< Set function */
#define LCD_8BIT LCD_CONFIGURATION | 0x10  /**<    8 bits interface */
#define LCD_4BIT LCD_CONFIGURATION | 0x00  /**<    4 bits interface */
#define LCD_2LINE LCD_CONFIGURATION | 0x08 /**<    2 line display */
#define LCD_1LINE LCD_CONFIGURATION | 0x00 /**<    1 line display */
#define LCD_5X10 LCD_CONFIGURATION | 0x04  /**<    5 X 10 dots */
#define LCD_5X7 LCD_CONFIGURATION | 0x00   /**<    5 X 7 dots */

#define LCD_SETCGRAMADDR 0x40

#define lcdPutChar(value) lcdWriteByte(value, 1)
#define lcdCommand(value) lcdWriteByte(value, 0)

static const uint8_t LCDLineOffsets[] PROGMEM = UI_LINE_OFFSETS;
static const char versionString[] PROGMEM = UI_VERSION_STRING;

#if UI_DISPLAY_TYPE == 1 || UI_DISPLAY_TYPE == 2

UIDisplay::UIDisplay() {
    locked = 0;
} // UIDisplay

void lcdWriteNibble(uint8_t value) {
    WRITE(UI_DISPLAY_D4_PIN, value & 1);
    WRITE(UI_DISPLAY_D5_PIN, value & 2);
    WRITE(UI_DISPLAY_D6_PIN, value & 4);
    WRITE(UI_DISPLAY_D7_PIN, value & 8);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH); // enable pulse must be >450ns
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t");

} // lcdWriteNibble

void lcdWriteByte(uint8_t c, uint8_t rs) {
#if UI_DISPLAY_RW_PIN < 0
    HAL::delayMicroseconds(UI_DELAYPERCHAR);
#else
    SET_INPUT(UI_DISPLAY_D4_PIN);
    SET_INPUT(UI_DISPLAY_D5_PIN);
    SET_INPUT(UI_DISPLAY_D6_PIN);
    SET_INPUT(UI_DISPLAY_D7_PIN);
    WRITE(UI_DISPLAY_RW_PIN, HIGH);
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    uint8_t busy;
    do {
        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        __asm__("nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t");
        busy = READ(UI_DISPLAY_D7_PIN);
        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        __asm__("nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t");
        WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
        __asm__("nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t");
        WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
        __asm__("nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t");
    } while (busy);
    SET_OUTPUT(UI_DISPLAY_D4_PIN);
    SET_OUTPUT(UI_DISPLAY_D5_PIN);
    SET_OUTPUT(UI_DISPLAY_D6_PIN);
    SET_OUTPUT(UI_DISPLAY_D7_PIN);
    WRITE(UI_DISPLAY_RW_PIN, LOW);
#endif // UI_DISPLAY_RW_PIN<0

    WRITE(UI_DISPLAY_RS_PIN, rs);
    WRITE(UI_DISPLAY_D4_PIN, c & 0x10);
    WRITE(UI_DISPLAY_D5_PIN, c & 0x20);
    WRITE(UI_DISPLAY_D6_PIN, c & 0x40);
    WRITE(UI_DISPLAY_D7_PIN, c & 0x80);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH); // enable pulse must be >450ns
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t");

    WRITE(UI_DISPLAY_D4_PIN, c & 0x01);
    WRITE(UI_DISPLAY_D5_PIN, c & 0x02);
    WRITE(UI_DISPLAY_D6_PIN, c & 0x04);
    WRITE(UI_DISPLAY_D7_PIN, c & 0x08);
    WRITE(UI_DISPLAY_ENABLE_PIN, HIGH); // enable pulse must be >450ns
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t");
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t");

} // lcdWriteByte

void initCspecchars() {
    uid.createChar(1, c1);
    uid.createChar(2, c2);
    uid.createChar(3, c3);
    uid.createChar(4, c4);
    uid.createChar(5, c7);
    uid.createChar(6, c8);
    uid.createChar(7, c9);
    normalchars = false;
}

void initNSpecchars() {
    uid.createChar(1, character_back);
    uid.createChar(2, character_degree);
    uid.createChar(3, character_selected);
    uid.createChar(4, character_unselected);
    uid.createChar(5, character_temperature);
    uid.createChar(6, character_folder);
    uid.createChar(7, character_ready);
    normalchars = true;
}

void UIDisplay::initializeLCD(bool normal) {
    // bring all display pins into a defined state
    SET_INPUT(UI_DISPLAY_D4_PIN);
    SET_INPUT(UI_DISPLAY_D5_PIN);
    SET_INPUT(UI_DISPLAY_D6_PIN);
    SET_INPUT(UI_DISPLAY_D7_PIN);
    SET_INPUT(UI_DISPLAY_RS_PIN);

#if UI_DISPLAY_RW_PIN > -1
    SET_INPUT(UI_DISPLAY_RW_PIN);
#endif // UI_DISPLAY_RW_PIN>-1

    SET_INPUT(UI_DISPLAY_ENABLE_PIN);

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way before 4.5V.
    // is this delay long enough for all cases??
    HAL::delayMilliseconds(500);
    SET_OUTPUT(UI_DISPLAY_D4_PIN);
    SET_OUTPUT(UI_DISPLAY_D5_PIN);
    SET_OUTPUT(UI_DISPLAY_D6_PIN);
    SET_OUTPUT(UI_DISPLAY_D7_PIN);
    SET_OUTPUT(UI_DISPLAY_RS_PIN);

#if UI_DISPLAY_RW_PIN > -1
    SET_OUTPUT(UI_DISPLAY_RW_PIN);
#endif // UI_DISPLAY_RW_PIN>-1

    SET_OUTPUT(UI_DISPLAY_ENABLE_PIN);

    // Now we pull both RS and R/W low to begin commands
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

    //put the LCD into 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    // at this point we are in 8 bit mode but of course in this
    // interface 4 pins are dangling unconnected and the values
    // on them don't matter for these instructions.
    WRITE(UI_DISPLAY_RS_PIN, LOW);
    HAL::delayMicroseconds(10);
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(5000); // I have one LCD for which 4500 here was not long enough.
    // second try
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(150); // wait
    // third go!
    lcdWriteNibble(0x03);
    HAL::delayMicroseconds(150);
    // finally, set to 4-bit interface
    lcdWriteNibble(0x02);
    HAL::delayMicroseconds(150);
    // finally, set # lines, font size, etc.
    lcdCommand(LCD_4BIT | LCD_2LINE | LCD_5X7);

    lcdCommand(LCD_CLEAR);                                       //- Clear Screen
    HAL::delayMilliseconds(2);                                   // clear is slow operation
    lcdCommand(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);              //- Entrymode (Display Shift: off, Increment Address Counter)
    lcdCommand(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF); //- Display on
    uid.lastSwitch = uid.lastRefresh = HAL::timeInMilliseconds();
    if (normal) {
        initNSpecchars();
    }
} // initializeLCD

// ----------- end direct LCD driver

void UIDisplay::printRow(uint8_t r, char* txt, char* txt2, uint8_t changeAtCol) {
#if MAX_COLS < UI_COLS && FEATURE_SEE_DISPLAY
#error if you set MAX_COLS to a tiny value you risk overflows. Probably not only within FEATURE_SEE_DISPLAY
#endif
    changeAtCol = RMath::min((uint8_t)UI_COLS, changeAtCol);
    uint8_t col = 0;

    // Set row
    if (r >= UI_ROWS)
        return;

    lcdWriteByte(128 + HAL::readFlashByte((const char*)&LCDLineOffsets[r]), 0); // Position cursor
    char c;
    while ((c = *txt) != 0x00 && col < changeAtCol) {
        txt++;
        lcdPutChar(c);
#if FEATURE_SEE_DISPLAY
        //cache whatever you write to the display!
        displayCache[r][col] = c;
#endif //FEATURE_SEE_DISPLAY
        col++;
    }
    while (col < changeAtCol) {
        lcdPutChar(' ');
#if FEATURE_SEE_DISPLAY
        //cache whatever you write to the display!
        displayCache[r][col] = ' ';
#endif //FEATURE_SEE_DISPLAY
        col++;
    }

    if (txt2 != NULL) {
        while ((c = *txt2) != 0x00 && col < UI_COLS) {
            txt2++;
            lcdPutChar(c);
#if FEATURE_SEE_DISPLAY
            //cache whatever you write to the display!
            displayCache[r][col] = c;
#endif //FEATURE_SEE_DISPLAY
            col++;
        }
        while (col < UI_COLS) {
            lcdPutChar(' ');
#if FEATURE_SEE_DISPLAY
            //cache whatever you write to the display!
            displayCache[r][col] = ' ';
#endif //FEATURE_SEE_DISPLAY
            col++;
        }
    }
#if FEATURE_SEE_DISPLAY
    //if we had sdcard files last we would see more than 20 bytes. Keep 0 at end.
    displayCache[r][col] = 0;
#endif //FEATURE_SEE_DISPLAY
} // printRow
#endif // UI_DISPLAY_TYPE==1 || UI_DISPLAY_TYPE==2

void UIDisplay::ui_init_keys() {
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_1); // push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_2); // push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_3); // push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_4); // push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_5); // push button, connects gnd to pin

    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E1); // PINJ.2, 80, X12.1 - push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E2); // PINJ.4, 81, X12.2 - push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E3); // PINJ.5, 82, X12.3 - push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E4); // PINJ.6, 83, X12.4 - push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E5); // PINH.7, 85, X12.6 - push button, connects gnd to pin
    UI_KEYS_INIT_BUTTON_LOW(ENABLE_KEY_E6); // PINH.2, 86, X12.7 - push button, connects gnd to pin
} // ui_init_keys

void UIDisplay::ui_check_keys(int& action) {
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_1, UI_ACTION_OK);       // push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_2, UI_ACTION_NEXT);     // push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_5, UI_ACTION_PREVIOUS); // push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_4, UI_ACTION_BACK);     // push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_3, UI_ACTION_RIGHT);    // push button, connects gnd to pin

    UI_KEYS_BUTTON_LOW(ENABLE_KEY_E1, UI_ACTION_RF_HEAT_BED_UP);      // PINJ.2, 80, X12.1 - push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_E2, UI_ACTION_RF_HEAT_BED_DOWN);    // PINJ.4, 81, X12.2 - push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_E3, UI_ACTION_RF_EXTRUDER_RETRACT); // PINJ.5, 82, X12.3 - push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_E4, UI_ACTION_RF_EXTRUDER_OUTPUT);  // PINJ.6, 83, X12.4 - push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_E5, UI_ACTION_RF_CONTINUE);         // PINH.7, 85, X12.6 - push button, connects gnd to pin
    UI_KEYS_BUTTON_LOW(ENABLE_KEY_E6, UI_ACTION_RF_PAUSE);            // PINH.2, 86, X12.7 - push button, connects gnd to pin
} // ui_check_keys

void UIDisplay::initialize() {
    flags = 0;
    exitmenu();
    shift = -2;
    lastAction = 0;
    lastButtonAction = 0;
    activeAction = 0;
    statusMsg[0] = 0;

    messageLine1 = NULL;
    messageLine2 = NULL;
    messageLine3 = NULL;
    messageLine4 = NULL;

    ui_init_keys();

#if SDSUPPORT
    cwd[0] = '/';
    cwd[1] = 0;
    folderLevel = 0;
#endif // SDSUPPORT

    initializeLCD(false);
    initCspecchars();

    for (uint8_t y = 0; y < UI_ROWS; y++)
        displayCache[y][0] = 0;
    printRowP(0, PSTR(BIGC0));
    printRowP(1, PSTR(BIGC1));
#if UI_ROWS > 3
    printRowP(UI_ROWS - 2, PSTR(BIGC2));
#endif // UI_ROWS>3
#if UI_ROWS > 2
    printRowP(UI_ROWS - 1, PSTR(BIGC3));
#endif // UI_ROWS>2

    if (READ(5) == 0 && READ(11) == 0 && READ(42) == 0) {
        g_nServiceRequest = 1;
    }
} // initialize

#if UI_DISPLAY_TYPE == 1 || UI_DISPLAY_TYPE == 2
void UIDisplay::createChar(uint8_t location, const uint8_t charmap[]) {
    location &= 0x7; // we only have 8 locations 0-7
    lcdCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
        lcdPutChar(pgm_read_byte(&(charmap[i])));
    }

} // createChar
#endif // UI_DISPLAY_TYPE==1 || UI_DISPLAY_TYPE==2

void UIDisplay::printRowP(uint8_t r, PGM_P txt) {
    if (r >= UI_ROWS)
        return;
    col = 0;
    addStringP(txt);
    printCols[col] = 0;
    printRow(r, printCols, NULL, UI_COLS);

} // printRowP

void UIDisplay::addInt(int value, uint8_t digits, char fillChar) {
    if (col >= MAX_COLS)
        return;
    uint8_t dig = 0, neg = 0;
    if (value < 0) {
        value = -value;
        neg = 1;
        dig++;
    }
    char buf[7]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[6];
    buf[6] = 0;
    do {
        unsigned int m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    } while (value);
    if (neg)
        printCols[col++] = '-';
    if (digits < 6)
        while (dig < digits) {
            *--str = fillChar; //' ';
            dig++;
        }
    while (*str && col < MAX_COLS) {
        printCols[col++] = *str;
        str++;
    }

} // addInt

void UIDisplay::addLong(long value, char digits) {
    if (col >= MAX_COLS)
        return;
    uint8_t dig = 0, neg = 0;
    if (value < 0) {
        neg = 1;
        value = -value;
        dig++;
    }
    char buf[13]; // Assumes 8-bit chars plus zero byte.
    char* str = &buf[12];
    buf[12] = 0;
    do {
        unsigned long m = value;
        value /= 10;
        char c = m - 10 * value;
        *--str = c + '0';
        dig++;
    } while (value);
    if (neg)
        printCols[col++] = '-';
    if (digits <= 11)
        while (dig < digits) {
            *--str = ' ';
            dig++;
        }
    while (*str && col < MAX_COLS) {
        printCols[col++] = *str;
        str++;
    }

} // addLong

const float roundingTable[] PROGMEM = { 0.5, 0.05, 0.005, 0.0005, 0.00005 };
void UIDisplay::addFloat(float number, char fixdigits, uint8_t digits) {
    if (col >= MAX_COLS)
        return;
    // Handle negative numbers
    if (number < 0.0) {
        printCols[col++] = '-';
        if (col >= MAX_COLS)
            return;
        number = -number;
        fixdigits--;
    }

    digits = (digits <= 4 ? digits : 4);
    number += pgm_read_float(&roundingTable[digits]); // for correct rounding

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    float remainder = number - (float)int_part;
    addLong(int_part, fixdigits);
    if (col >= UI_COLS)
        return;

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        printCols[col++] = '.';
    }

    // Extract digits from the remainder one at a time
    while (col < MAX_COLS && digits-- > 0) {
        remainder *= 10.0;
        uint8_t toPrint = uint8_t(remainder);
        printCols[col++] = '0' + toPrint;
        remainder -= toPrint;
    }

} // addFloat

void UIDisplay::addStringP(FSTRINGPARAM(text)) {
    while (col < MAX_COLS) {
        uint8_t c = HAL::readFlashByte(text++);
        if (c == 0)
            return;
        printCols[col++] = c;
    }

} // addStringP

UI_STRING(ui_text_on, UI_TEXT_ON)
UI_STRING(ui_text_off, UI_TEXT_OFF)
UI_STRING(ui_text_0, UI_TEXT_0)
UI_STRING(ui_text_1, UI_TEXT_1)
UI_STRING(ui_text_white, UI_TEXT_WHITE)
UI_STRING(ui_text_color, UI_TEXT_COLOR)
UI_STRING(ui_text_manual, UI_TEXT_MANUAL)
UI_STRING(ui_text_unknown, UI_TEXT_UNKNOWN)
UI_STRING(ui_text_na, UI_TEXT_NA)
UI_STRING(ui_yes, UI_TEXT_YES)
UI_STRING(ui_no, UI_TEXT_NO)
UI_STRING(ui_ok, UI_TEXT_OK)
UI_STRING(ui_fail, UI_TEXT_FAIL)
UI_STRING(ui_neetfix, UI_TEXT_O_SCAN_NEEDFIX)
UI_STRING(ui_up, UI_TEXT_UP)
UI_STRING(ui_down, UI_TEXT_DOWN)
UI_STRING(ui_selected, UI_TEXT_SEL)
UI_STRING(ui_unselected, UI_TEXT_NOSEL)
UI_STRING(ui_text_print_mode, UI_TEXT_PRINT_MODE)
UI_STRING(ui_text_mill_mode, UI_TEXT_MILL_MODE)
UI_STRING(ui_text_z_single, UI_TEXT_Z_SINGLE)
UI_STRING(ui_text_z_circuit, UI_TEXT_Z_CIRCUIT)
UI_STRING(ui_text_z_mode_min, UI_TEXT_Z_MODE_MIN)
UI_STRING(ui_text_z_mode_surface, UI_TEXT_Z_MODE_SURFACE)
UI_STRING(ui_text_z_mode_gcode, UI_TEXT_Z_MODE_GCODE)
UI_STRING(ui_text_z_mode_z_origin, UI_TEXT_Z_MODE_Z_ORIGIN)
UI_STRING(ui_text_hotend_v1, UI_TEXT_HOTEND_V1)
UI_STRING(ui_text_hotend_v2, UI_TEXT_HOTEND_V2)
UI_STRING(ui_text_miller_one_track, UI_TEXT_MILLER_ONE_TRACK)
UI_STRING(ui_text_miller_two_tracks, UI_TEXT_MILLER_TWO_TRACKS)
UI_STRING(ui_text_z_compensation_active, UI_TEXT_Z_COMPENSATION_ACTIVE)

    ; // needed because the development tool does not recognize the ; within UI_STRING definition right.

void UIDisplay::parse(char* txt, bool ram) {
    int ivalue = 0;
    float fvalue = 0;

    while (col < MAX_COLS) {
        char c = (ram ? *(txt++) : pgm_read_byte(txt++));
        if (c == 0)
            break; // finished
        if (c != '%') {
            printCols[col++] = c;
            continue;
        }

        // dynamic parameter, parse meaning and replace
        char c1 = (ram ? *(txt++) : pgm_read_byte(txt++));
        char c2 = (ram ? *(txt++) : pgm_read_byte(txt++));
        switch (c1) {
        case '%': {
            if (c2 == '%' && col < MAX_COLS)
                printCols[col++] = '%'; // %%% : The % char
            break;
        }
        case '1': {
            if (c2 == '1') { // %11 : Kill Line if Millingmode and use following string for display, else stop here
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_MILL) {
                    for (uint8_t n = 0; n < MAX_COLS + 1; n++)
                        printCols[n] = 0; //clear all text
                    col = 0;              //reset linemarker
                } else
#endif            // FEATURE_MILLING_MODE
                { //this is not milling mode: stop here
                    if (col < MAX_COLS)
                        printCols[col++] = 0; //write 0 to end of string
                    col = MAX_COLS;           //end while
                }
            }
            break;
        }
        case 'a': // Acceleration settings
        {
            if (c2 == 'x')
                addFloat(Printer::maxAccelerationMMPerSquareSecond[X_AXIS], 5, 0); // %ax : X acceleration during print moves
            else if (c2 == 'y')
                addFloat(Printer::maxAccelerationMMPerSquareSecond[Y_AXIS], 5, 0); // %ay : Y acceleration during print moves
            else if (c2 == 'z')
                addFloat(Printer::maxAccelerationMMPerSquareSecond[Z_AXIS], 5, 0); // %az : Z acceleration during print moves
            else if (c2 == 'X')
                addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS], 5, 0); // %aX : X acceleration during travel moves
            else if (c2 == 'Y')
                addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS], 5, 0); // %aY : Y acceleration during travel moves
            else if (c2 == 'Z')
                addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS], 5, 0); // %aZ : Z acceleration during travel moves
            else if (c2 == 'j')
                addFloat(Printer::maxXYJerk, 3, 1); // %aj : Max. jerk
            else if (c2 == 'J')
                addFloat(Printer::maxZJerk, 3, 2); // %aJ : Max. Z-jerk
            break;
        }
        case 'd': {
            if (c2 == 'o')
                addStringP(Printer::debugEcho() ? ui_text_on : ui_text_off); // %do : Debug echo state
            else if (c2 == 'i')
                addStringP(Printer::debugInfo() ? ui_text_on : ui_text_off); // %di : Debug info state
            else if (c2 == 'e')
                addStringP(Printer::debugErrors() ? ui_text_on : ui_text_off); // %de : Debug error state
            else if (c2 == 'd')
                addStringP(Printer::debugDryrun() ? ui_text_on : ui_text_off); // %dd : Debug dry run state
            else if (c2 == 'b')
                addStringP(Printer::enableBeeper ? ui_text_on : ui_text_off); // %db : beeper state
            break;
        }
        case 'D': {
            if (c2 == 'x')
                addLong(g_nScanXStepSizeMM, 3); // %Dx : scan step size x
            else if (c2 == 'y')
                addLong(g_nScanYStepSizeMM, 3); // %Dy : scan step size y
            break;
        }

#if FEATURE_HEAT_BED_Z_COMPENSATION
        case 'H': {
            if (c2 == 'B')
                addLong(g_nActiveHeatBed, 1); // %HB : active heat bed z matrix
            else if (c2 == 'O')
                addFloat((float)g_offsetZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS] * 1000.0f, 3, 0); // %HO : active heat bed min z offset in um
            break;
        }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
        case 'W': {
            if (c2 == 'P')
                addLong(g_nActiveWorkPart, 1); // %WP : active work part z matrix
            break;
        }
#endif // FEATURE_WORK_PART_Z_COMPENSATION

        case 'e': // Extruder temperature
        {
            if (c2 == 'r') // %er : Extruder relative mode
            {
                addStringP(Printer::relativeExtruderCoordinateMode ? ui_yes : ui_no);
                break;
            }

#if FEATURE_MILLING_MODE
            if (Printer::operatingMode == OPERATING_MODE_MILL) {
                // we do not maintain temperatures in milling mode
                addStringP(PSTR("   "));
                break;
            }
#endif // FEATURE_MILLING_MODE

            if (c2 == 'w') { //%ew : tell me which sensor(s) are defect.
                bool addone = false;
                for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
                    TemperatureController* act = tempController[controller];
                    if (act->isSensorDefect() || act->isSensorDecoupled()) {
                        if (addone)
                            addStringP(Com::tSlash);
                        addone = true;
                        if (controller < NUM_EXTRUDER) {
                            if (col < MAX_COLS)
                                printCols[col++] = 'E';
                            addInt(controller, 1);
                        } else if (controller <= NUM_EXTRUDER) {
                            addStringP(PSTR("Bed"));
                        } else {
                            addStringP(PSTR("Opt"));
                        }
                    }
                }
                break;
            }

            if (Printer::isAnyTempsensorDefect()) {
                uint8_t countDefect = 0;
                uint8_t countDecoupled = 0;
                for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++) {
                    TemperatureController* act = tempController[controller];
                    if (act->isSensorDefect()) {
                        countDefect++;
                    }
                    if (act->isSensorDecoupled()) {
                        countDecoupled++;
                    }
                }
                if (countDecoupled > countDefect) {
                    addStringP(PSTR("dec"));
                } else {
                    addStringP(PSTR("def"));
                }
                break;
            }

            if (!(Printer::flag2 & PRINTER_FLAG2_GOT_TEMPS)) {
                // avoid to show the current temperatures before we have measured them
                addStringP(PSTR(" ? "));
                break;
            }

            if (c2 == 'c') {
                fvalue = Extruder::current->tempControl.currentTemperatureC; // %ec : Current extruder temperature
            } else if (c2 >= '0' && c2 <= '9') {
                uint8_t nr = c2 - '0';
                fvalue = extruder[nr].tempControl.currentTemperatureC; // %e0..9 : Temp. of extruder 0..9
            }
#if HAVE_HEATED_BED
            else if (c2 == 'b') {
                fvalue = Extruder::getHeatedBedTemperature(); // %eb : Current heated bed temperature
            }
#endif // HAVE_HEATED_BED
            addFloat(fvalue, 3, 0);
            break;
        }
        case 'E': // Target extruder temperature
        {
#if FEATURE_MILLING_MODE
            if (Printer::operatingMode == OPERATING_MODE_MILL) {
                // we do not maintain temperatures in milling mode
                addStringP(PSTR("   "));
                break;
            }
#endif // FEATURE_MILLING_MODE

            if (c2 == 'c') {
                fvalue = Extruder::current->tempControl.targetTemperatureC; // %Ec : Target temperature of current extruder
            } else if (c2 >= '0' && c2 <= '9') {
                fvalue = extruder[c2 - '0'].tempControl.targetTemperatureC; // %E0-9 : Target temperature of extruder 0..9
            }
#if HAVE_HEATED_BED
            else if (c2 == 'b') {
                fvalue = heatedBedController.targetTemperatureC; // %Eb : Target temperature of heated bed
            }
#endif // HAVE_HEATED_BED
            addFloat(fvalue, 3, 0);
            break;
        }

        case 'F': // FAN speed
        {
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
            if (c2 == 's') { // %Fs : Fan speed in Percent
                addFloat(Printer::getFanSpeed(false) / 2.55f, 3, 1);
            } else if (c2 == 'h') { // %Fh : Fan frequency in Hz/Teiler
                addFloat(15.3f / part_fan_pwm_speed, 2, 1);
            } else if (c2 == 'm') { // %Fm : Fan modulation type PWM or PDM
                if (part_fan_frequency_modulation == PART_FAN_MODE_PDM)
                    addStringP(PSTR("PDM"));
                else
                    addStringP(PSTR("PWM"));
            } else if (c2 == 'U') { // %FU : Fan modulation minimum = 1% FanSpeed
                addInt(part_fan_pwm_min, 3);
            } else if (c2 == 'O') { // %FO : Fan modulation maximum = 100% FanSpeed
                addInt(part_fan_pwm_max, 3);
            }
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL
#if FEATURE_ZERO_DIGITS
            if (c2 == 'H') // %FH : Digit Homing to Zero ON/OFF
            {
                addStringP(Printer::g_pressure_offset_active ? ui_text_on : ui_text_off);
            }
#endif // FEATURE_ZERO_DIGITS
#if FEATURE_DIGIT_Z_COMPENSATION
            if (c2 == 'C') // %FC : Digits force-bend-hotend-down Compensation Z ON/OFF
            {
                addStringP(g_nDigitZCompensationDigits_active ? ui_text_on : ui_text_off);
            }
#endif // FEATURE_DIGIT_Z_COMPENSATION
            break;
        }
        case 'f': {
            if (c2 == 'x')
                addFloat(Printer::maxFeedrate[X_AXIS], 5, 0); // %fx : Max. feedrate x direction
            else if (c2 == 'y')
                addFloat(Printer::maxFeedrate[Y_AXIS], 5, 0); // %fy : Max. feedrate y direction
            else if (c2 == 'z')
                addFloat(Printer::maxFeedrate[Z_AXIS], 5, 0); // %fz : Max. feedrate z direction
            else if (c2 == 'e')
                addFloat(extruder[0].maxFeedrate, 5, 0); // %fe : Max. feedrate extruder 0
#if NUM_EXTRUDER > 1
            else if (c2 == 'E')
                addFloat(extruder[1].maxFeedrate, 5, 0); // %fE : Max. feedrate extruder 1
#endif                                                   // NUM_EXTRUDER>1
            else if (c2 == 'X')
                addFloat(Printer::homingFeedrate[X_AXIS], 5, 0); // %fX : Homing feedrate x direction
            else if (c2 == 'Y')
                addFloat(Printer::homingFeedrate[Y_AXIS], 5, 0); // %fY : Homing feedrate y direction
            else if (c2 == 'Z')
                addFloat(Printer::homingFeedrate[Z_AXIS], 5, 0); // %fZ : Homing feedrate z direction

            break;
        }
        case 'i': {
            if (c2 == 's')
                addLong(stepperInactiveTime / 1000, 4); // %is : Stepper inactive time in seconds
            else if (c2 == 'p')
                addLong(maxInactiveTime / 1000, 4); // %ip : Max. inactive time in seconds
            break;
        }
        case 'O': // ops related stuff
        {
#if NUM_EXTRUDER > 1
            if (c2 == 'a') // %Oa : Active Extruder
            {
                if (Extruder::current->id == 0) {
                    addStringP(ui_text_0);
                } else {
                    addStringP(ui_text_1);
                }
            }
#endif // NUM_EXTRUDER>1

#if NUM_EXTRUDER > 1
            if (c2 == 'E') // %OE : Extruder offset X [mm]
            {
                addFloat(extruder[1].offsetMM[X_AXIS], 4, 3);
            }
            if (c2 == 'F') // %OF : Extruder offset Y [mm]
            {
                addFloat(extruder[1].offsetMM[Y_AXIS], 4, 3);
            }
            if (c2 == 'S') // %OS : Extruder spring displacement Z [mm]
            {
                addFloat(extruder[1].offsetMM[Z_AXIS], 4, 3);
            }
#endif // NUM_EXTRUDER>1

            if (c2 == 'M') // %OM : operating mode
            {
#if FEATURE_MILLING_MODE
                addStringP(Printer::operatingMode == OPERATING_MODE_PRINT ? ui_text_print_mode : ui_text_mill_mode);
#else
                addStringP(ui_text_print_mode);
#endif // FEATURE_MILLING_MODE
            }
            if (c2 == 'Z') // %OZ : z endstop type
            {
#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
                addStringP(Printer::ZEndstopType == ENDSTOP_TYPE_SINGLE ? ui_text_z_single : ui_text_z_circuit);
#else
                addStringP(ui_text_z_single);
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS
            }
            break;
        }
        case 'h': {
            if (c2 == 'x' && col < MAX_COLS) // %hx : x homed
            {
                if (Printer::flag3 & PRINTER_FLAG3_X_HOMED)
                    printCols[col++] = '*';
            } else if (c2 == 'y' && col < MAX_COLS) // %hy : y homed
            {
                if (Printer::flag3 & PRINTER_FLAG3_Y_HOMED)
                    printCols[col++] = '*';
            } else if (c2 == 'z' && col < MAX_COLS) // %hz : z homed
            {
                if (Printer::flag3 & PRINTER_FLAG3_Z_HOMED)
                    printCols[col++] = '*';
            } else if (c2 == 'X' && col < MAX_COLS) // %hX : x homed
            {
                if (Printer::flag3 & PRINTER_FLAG3_X_HOMED)
                    printCols[col++] = ':';
                else
                    printCols[col++] = '?';
            } else if (c2 == 'Y' && col < MAX_COLS) // %hY : y homed
            {
                if (Printer::flag3 & PRINTER_FLAG3_Y_HOMED)
                    printCols[col++] = ':';
                else
                    printCols[col++] = '?';
            } else if (c2 == 'Z' && col < MAX_COLS) // %hZ : z homed
            {
                if (Printer::flag3 & PRINTER_FLAG3_Z_HOMED)
                    printCols[col++] = ':';
                else
                    printCols[col++] = '?';
            }
            break;
        }
        case 'l': {
            if (c2 == 'a')
                addInt(lastAction, 4);

#if FEATURE_CASE_LIGHT
            else if (c2 == 'o')
                addStringP(Printer::enableCaseLight ? ui_text_on : ui_text_off); // %lo : Lights on/off
#endif                                                                           // FEATURE_CASE_LIGHT

#if FEATURE_RGB_LIGHT_EFFECTS
            else if (c2 == 'i') // %li : Light: White/Color
            {
                if (Printer::RGBLightModeForceWhite) {
                    addStringP(ui_text_white);
                } else {
                    switch (Printer::RGBLightMode) {
                    case RGB_MODE_OFF:
                        addStringP(ui_text_off);
                        break;
                    case RGB_MODE_WHITE:
                        addStringP(ui_text_white);
                        break;
                    case RGB_MODE_AUTOMATIC:
                        addStringP(ui_text_color);
                        break;
                    case RGB_MODE_MANUAL:
                        addStringP(ui_text_manual);
                        break;
                    }
                }
            }
#endif // FEATURE_RGB_LIGHT_EFFECTS

            break;
        }

        case 'm': {
            if (c2 == 'Y' && col < MAX_COLS) // %mY : menu yes
            {
                if (g_nYesNo)
                    printCols[col++] = CHAR_SELECTED;
                else
                    printCols[col++] = ' ';
            } else if (c2 == 'N' && col < MAX_COLS) // %mN : menu no
            {
                if (g_nYesNo)
                    printCols[col++] = ' ';
                else
                    printCols[col++] = CHAR_SELECTED;
            } else if (c2 == 't') // %mt : miller type
            {
#if FEATURE_CONFIGURABLE_MILLER_TYPE
                addStringP(Printer::MillerType == MILLER_TYPE_ONE_TRACK ? ui_text_miller_one_track : ui_text_miller_two_tracks);
#endif                            // FEATURE_CONFIGURABLE_MILLER_TYPE
            } else if (c2 == '1') // %m1 : message line 1
            {
                if (messageLine1 != 0) {
                    col = 0;                                   //Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine1, false); //addStringP((char PROGMEM *)messageLine1);
                }
                break;
            } else if (c2 == '2') // %m2 : message line 2
            {
                if (messageLine2 != 0) {
                    col = 0;                                   //Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine2, false); //addStringP((char PROGMEM *)messageLine2);
                }
                break;
            } else if (c2 == '3') // %m3 : message line 3
            {
                if (messageLine3 != 0) {
                    col = 0;                                   //Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine3, false); //addStringP((char PROGMEM *)messageLine3);
                }
                break;
            } else if (c2 == '4') // %m4 : message line 4
            {
                if (messageLine4 != 0) {
                    col = 0;                                   //Nibbels: kill first space because this is menutab
                    parse((char PROGMEM*)messageLine4, false); //addStringP((char PROGMEM *)messageLine4);
                }
                break;
            }
            break;
        }

        case 'M': {
            if (c2 == 'X' && col < MAX_COLS) // %MX : Motorcurrent X
            {
                addFloat(Printer::motorCurrent[X_AXIS] / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                addInt((int)Printer::motorCurrent[X_AXIS], 3);
            } else if (c2 == 'Y' && col < MAX_COLS) // %MY : Motorcurrent Y
            {
                addFloat(Printer::motorCurrent[Y_AXIS] / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                addInt((int)Printer::motorCurrent[Y_AXIS], 3);
            } else if (c2 == 'Z') // %MZ : Motorcurrent Z
            {
                addFloat(Printer::motorCurrent[Z_AXIS] / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                addInt((int)Printer::motorCurrent[Z_AXIS], 3);
            } else if (c2 == '0') // %M0 : Motorcurrent T0
            {
                addFloat(Printer::motorCurrent[E_AXIS] / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                if (g_pauseStatus == PAUSE_STATUS_PAUSED)
                    addStringP(PSTR(UI_TEXT_PAUSED));
                else
                    addInt((int)Printer::motorCurrent[E_AXIS], 3);
            } else if (c2 == '1') // %M1 : Motorcurrent T1
            {
                addFloat(Printer::motorCurrent[E_AXIS + 1] / 63.0f, 1, 2); //(126 = ~2A) *2.0f/126.0f
                if (col < MAX_COLS)
                    printCols[col++] = 'A';
                if (col < MAX_COLS)
                    printCols[col++] = ' ';
                if (g_pauseStatus == PAUSE_STATUS_PAUSED)
                    addStringP(PSTR(UI_TEXT_PAUSED));
                else
                    addInt((int)Printer::motorCurrent[E_AXIS + 1], 3);
            }
            break;
        }

        case 'o': {
            if (c2 == 's') // %os : Status message
            {
                if (locked) {
                    //Com::printFLN( PSTR( "locked" ) );
                    // do not change the status message in case it is locked
                    parse(statusMsg, true);
                } else {
#if SDSUPPORT
                    if (sd.sdactive && sd.sdmode) {
                        if (g_pauseMode) {
                            // do not show the printing/milling progress while we are paused
                            parse(statusMsg, true);
                        } else {
#if FEATURE_MILLING_MODE
                            if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
                                addStringP(PSTR(UI_TEXT_PRINT_POS));
                                unsigned long percent;
                                if (sd.filesize < 20000000)
                                    percent = sd.sdpos * 100 / sd.filesize;
                                else
                                    percent = (sd.sdpos >> 8) * 100 / (sd.filesize >> 8);
                                addInt((int)percent, 3);
                                if (col < MAX_COLS)
                                    printCols[col++] = '%';
#if FEATURE_MILLING_MODE
                            } else {
                                if (!Printer::isZOriginSet()) {
                                    parse(statusMsg, true);
                                } else {
                                    addStringP(PSTR(UI_TEXT_MILL_POS));

                                    unsigned long percent;
                                    if (sd.filesize < 20000000)
                                        percent = sd.sdpos * 100 / sd.filesize;
                                    else
                                        percent = (sd.sdpos >> 8) * 100 / (sd.filesize >> 8);
                                    addInt((int)percent, 3);
                                    if (col < MAX_COLS)
                                        printCols[col++] = '%';
                                }
                            }
#endif // FEATURE_MILLING_MODE
                        }
                    } else
#endif // SDSUPPORT

                        parse(statusMsg, true);
                }
                break;
            }
            if (c2 == 'c') // %oc : Connection baudrate
            {
                addLong(baudrate, 6);
                break;
            }
            if (c2 == 'f') // %of : flow multiplier
            {
                addInt(100 * Printer::menuExtrusionFactor
#if FEATURE_DIGIT_FLOW_COMPENSATION
                           * Printer::dynamicExtrusionFactor
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
                       ,
                       3);
                break;
            }
            if (c2 == 'm') // %om : Speed multiplier
            {
#if FEATURE_DIGIT_FLOW_COMPENSATION
                addInt(Printer::feedrateMultiply * Printer::dynamicFeedrateFactor, 3);
#else
                addInt(Printer::feedrateMultiply, 3);
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
                break;
            }
            if (c2 == 'M') // %oM : Printer::feedrate
            {
                addInt(Printer::feedrate, 3);
                break;
            }
            if (c2 == 'v') // %ov : Active Speed
            {
#if FEATURE_DIGIT_FLOW_COMPENSATION
                addFloat(Printer::v * Printer::dynamicFeedrateFactor, 3, 1);
#else
                addFloat(Printer::v, 3, 2);
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
                break;
            }
            if (c2 == 'p') // %op : Is single double or quadstepping?
            {
                addInt(Printer::stepsPerTimerCall, 1);
                addStringP(PSTR("ST"));

                break;
            }

#if FEATURE_230V_OUTPUT
            if (c2 == 'u') {
                addStringP(Printer::enable230VOutput ? ui_text_on : ui_text_off); // %ou : 230V output on/off
                break;
            }
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
            if (c2 == 'l') {
                addStringP(Printer::enableFET1 ? ui_text_on : ui_text_off); // %ol : FET 1 X42 output on/off
                break;
            }
#endif // FEATURE_24V_FET_OUTPUTS

#if FEATURE_24V_FET_OUTPUTS
            if (c2 == 'n') {
                addStringP(Printer::enableFET2 ? ui_text_on : ui_text_off); // %on : FET 2 X44 output on/off
                break;
            }
#endif // FEATURE_24V_FET_OUTPUTS

            // Extruder output level
            if (c2 >= '0' && c2 <= '9') // %o0..9 : Output level extruder 0..9 is % including %sign
            {
                ivalue = pwm_pos[c2 - '0'];
            }

#if HAVE_HEATED_BED
            else if (c2 == 'b') // %ob : Output level heated bed
            {
                ivalue = pwm_pos[heatedBedController.pwmIndex];
            }
#endif // HAVE_HEATED_BED

            else if (c2 == 'C') // %oC : Output level current extruder
            {
                ivalue = pwm_pos[Extruder::current->id];
            }

            ivalue = (ivalue * 100) / 255;
            addInt(ivalue, 3);
            if (col < MAX_COLS)
                printCols[col++] = '%';
            break;
        }
        case 'x': {
            char bDefect = false;

            if (c2 >= '0' && c2 <= '6') {
                if (c2 == '0') // %x0 : X position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::currentSteps[X_AXIS] * Printer::axisMMPerSteps[X_AXIS];
                    }
                } else if (c2 == '1') // %x1 : Y position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::currentSteps[Y_AXIS] * Printer::axisMMPerSteps[Y_AXIS];
                    }
                } else if (c2 == '2') // %x2 : Z position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::currentZPositionMM();
                    }
                } else if (c2 == '3') // %x3 : X offset position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::directCurrentSteps[X_AXIS] * Printer::axisMMPerSteps[X_AXIS];
                    }
                } else if (c2 == '4') // %x4 : Y offset position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::directCurrentSteps[Y_AXIS] * Printer::axisMMPerSteps[Y_AXIS];
                    }
                } else if (c2 == '5') // %x5 : Z offset position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        // Wenn nur CurrentSteps[Z_AXIS] ausgegeben wird, ist auch interssant, viel die differenz aller anderer versätze ist:
                        if (Printer::ZMode == Z_VALUE_MODE_Z_MIN) { //->Z_VALUE_MODE_LAYER
                            if (col < MAX_COLS)
                                printCols[col++] = 'm';
                            if (col < MAX_COLS)
                                printCols[col++] = 'm';
                        } else {
                            fvalue = (Printer::currentZSteps - Printer::currentSteps[Z_AXIS]) * Printer::axisMMPerSteps[Z_AXIS];
                            if (col < MAX_COLS)
                                printCols[col++] = ' ';
                            addFloat(fvalue, 3, 2);
                        }
                        break;
                    }
                } else // %x6 : Current extruder position
                {
                    if (Printer::blockAll) {
                        // we can not move any more
                        bDefect = true;
                    } else {
                        fvalue = Printer::destinationMM[E_AXIS];
                    }
                }

                if (bDefect) {
                    addStringP(PSTR(" def"));
                } else {
                    addFloat(fvalue, 3, 2);
                }
            }
            break;
        }
        case 'y': {
            break;
        }
        case 'z': // %z0 : Z Offset
        {
            if (c2 == '0') {
                addInt(Printer::ZOffset, 4);
            } else if (c2 == 'm') // %zm : Z Scale
            {
                addStringP(Printer::ZMode == Z_VALUE_MODE_Z_MIN ? ui_text_z_mode_min : ui_text_z_mode_gcode);
            } else if (c2 == 's') // %zs : Z-Schraube korrektur mm
            {
                addFloat(g_ZSchraubenSollKorrekturWarm_mm, 1, 3);
            } else if (c2 == 'S') // %zS : Z-Schraube korrektur Umdrehungen
            {
                addFloat(g_ZSchraubenSollDrehungenWarm_U, 1, 1);
            } else if (c2 == 'F') // %zF : Z-Schraube falsch
            {
                if (g_ZSchraubeOk < 0)
                    addStringP(ui_ok); //ok
                else if (g_ZSchraubeOk > 0)
                    addStringP(ui_neetfix); //zu hohes bett
                else
                    addStringP(ui_fail); //fail
            } else if (c2 == 'D')        // %zD : Z-Schraube richtung
            {
                if (g_ZSchraubenSollKorrekturWarm_mm > 0)
                    addStringP(ui_up);
                else
                    addStringP(ui_down);
            }
            break;
        }
        case 'X': // Extruder related
        {
#if NUM_EXTRUDER > 0
            if (c2 >= '0' && c2 <= '9') // %X0..9 : Extruder selected marker
            {
                addStringP(Extruder::current->id == c2 - '0' ? ui_selected : ui_unselected);
            } else if (c2 == 'm') // %Xm : PID drive min
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(-1 * tempController[menuPos[menuLevel - 1]]->pidDriveMin, 3);
                } else {
                    addInt(Extruder::current->tempControl.pidDriveMin, 3);
                }
            } else if (c2 == 'M') // %XM : PID drive max
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(tempController[menuPos[menuLevel - 1]]->pidDriveMax, 3);
                } else {
                    addInt(Extruder::current->tempControl.pidDriveMax, 3);
                }
            } else if (c2 == 'D') // %XD : PID max
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(tempController[menuPos[menuLevel - 1]]->pidMax * 100 / 255, 3);
                } else {
                    addInt(Extruder::current->tempControl.pidMax * 100 / 255, 3);
                }
            } else if (c2 == 'S') // %XS : Temperature Sensor
            {
                if (menuLevel == 4 && menuPos[menuLevel - 1] < NUM_TEMPERATURE_LOOPS) {
                    addInt(tempController[menuPos[menuLevel - 1]]->sensorType, 2); //mit type 100 wärens 3 zeichen, aber das kommt in praxis nicht vor.
                    switch (tempController[menuPos[menuLevel - 1]]->sensorType) {
                    case 1: {
                        addStringP(PSTR(UI_TEXT_SENSOR_1));
                        break;
                    }
                    case 3: {
                        addStringP(PSTR(UI_TEXT_SENSOR_3));
                        break;
                    }
                    case 4: {
                        addStringP(PSTR(UI_TEXT_SENSOR_4));
                        break;
                    }
                    case 8: {
                        addStringP(PSTR(UI_TEXT_SENSOR_8));
                        break;
                    }
                    case 13: {
                        addStringP(PSTR(UI_TEXT_SENSOR_13));
                        break;
                    }
                    }
                } else {
                    addInt(Extruder::current->tempControl.sensorType, 3);
                }
            }

            else if (c2 == 't') // %Xt : Description for PID autotune type in menu
            {
                col = 0;                          //reset linemarker -> start at first display char. overwrite things before this %tag -> erstes Zeichen brauchen wir in diesem menü nicht.
                switch (menuPos[menuLevel - 1]) { //das sagt mir, in welchem untermenü ich bin. die zahl drin entspricht fast dem Jx aus dem M303, aber J0 und J1 sind der intuitiven Ordnung halber vertauscht. J2 ist rausgelassen worden, weil wir das eigentlich nicht brauchen. J3->2 J4->3
                case 0: {
                    addStringP(PSTR(UI_ACTION_TEXT_PESSEN_TIPP)); //tipp für pessen integral rule
                    break;
                }
                case 1: {
                    addStringP(PSTR(UI_ACTION_TEXT_CLASSICPID_TIPP)); //tipp für classic pid
                    break;
                }
                case 2: {
                    addStringP(PSTR(UI_ACTION_TEXT_NO_TIPP)); //tipp für no overshoot
                    break;
                }
                case 3: {
                    addStringP(PSTR(UI_ACTION_TEXT_TYREUS_LYBEN_TIPP)); //tipp für tyreus lyben
                    break;
                }
                }
            }
#if RETRACT_DURING_HEATUP
            else if (c2 == 'T') // %XT : Extruder wait retract temperature
            {
                addInt(Extruder::current->waitRetractTemperature, 4);
            } else if (c2 == 'U') // %XU : Extruder wait retract unit
            {
                addInt(Extruder::current->waitRetractUnits, 2);
            }
#endif // RETRACT_DURING_HEATUP

#if USE_ADVANCE
            else if (c2 == 'l') // %Xl : Advance L value
            {
                addFloat(Extruder::current->advanceL, 3, 0);
            } else if (c2 == 'b') // %Xb : E0 Advance L value
            {
                addFloat(extruder[0].advanceL, 3, 0);
            } else if (c2 == 'c') // %Xc : E1 Advance L value
            {
                addFloat(extruder[1].advanceL, 3, 0);
            }
#endif // USE_ADVANCE

            else if (c2 == 'f') // %Xf : Extruder 0 Acceleration
            {
                addFloat(extruder[0].maxAcceleration, 5, 0);
            }
#if NUM_EXTRUDER > 1
            else if (c2 == 'F') // %XF : Extruder 1 Acceleration
            {
                addFloat(extruder[1].maxAcceleration, 5, 0);
            }
#endif                          //NUM_EXTRUDER>1
            else if (c2 == 'h') // %Xh : Extruder 0 maxEJerk
            {
                addFloat(extruder[0].maxEJerk, 5, 0);
            }
#if NUM_EXTRUDER > 1
            else if (c2 == 'H') // %XH : Extruder 1 maxEJerk
            {
                addFloat(extruder[1].maxEJerk, 5, 0);
            }
#endif //NUM_EXTRUDER>1

#if FEATURE_ADJUSTABLE_MICROSTEPS
            else if (c2 == 'E') // %XE : Extruder Stepper Microsteps
            {
                addInt(drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[E_AXIS]), 3);
            }
#endif                          //FEATURE_ADJUSTABLE_MICROSTEPS
#endif                          // NUM_EXTRUDER>0
            else if (c2 == 'g') // %Xg : Printer::stepsPackingMinInterval
            {
                addInt(Printer::stepsPackingMinInterval, 4);
            }
#if FEATURE_ADJUSTABLE_MICROSTEPS
            else if (c2 == 'x') // %Xx : XY Stepper Microsteps
            {
                addInt(drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[X_AXIS]), 3);
            } else if (c2 == 'z') // %Xz : Z Stepper Microsteps
            {
                addInt(drv8711ModeValue_2_MicroSteps(Printer::motorMicroStepsModeValue[Z_AXIS]), 3);
            }
#endif //FEATURE_ADJUSTABLE_MICROSTEPS
#if FEATURE_MILLING_MODE
            else if (c2 == 'Z') // %XZ : Milling special max. acceleration
            {
                addInt(Printer::max_milling_all_axis_acceleration, 3);
            }
#endif // FEATURE_MILLING_MODE
            break;
        }
        case 's': // Endstop positions
        {
            if (c2 == 'x') // %sx : State of x min endstop
            {
#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
                addStringP(Printer::isXMinEndstopHit() ? ui_text_on : ui_text_off);
#else
                addStringP(ui_text_na);
#endif // (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
            }
            if (c2 == 'X') // %sX : State of x max endstop
#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
                addStringP(Printer::isXMaxEndstopHit() ? ui_text_on : ui_text_off);
#else
                addStringP(ui_text_na);
#endif // (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X

            if (c2 == 'y') // %sy : State of y min endstop
#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
                addStringP(Printer::isYMinEndstopHit() ? ui_text_on : ui_text_off);
#else
                addStringP(ui_text_na);
#endif // (Y_MIN_PIN > -1)&& MIN_HARDWARE_ENDSTOP_Y

            if (c2 == 'Y') // %sY : State of y max endstop
#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
                addStringP(Printer::isYMaxEndstopHit() ? ui_text_on : ui_text_off);
#else
                addStringP(ui_text_na);
#endif // (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y

            if (c2 == 'z') // %sz : State of z min endstop
            {
#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
                if (Printer::ZEndstopUnknown) {
                    addStringP(ui_text_unknown);
                } else {
                    addStringP(Printer::isZMinEndstopHit() ? ui_text_on : ui_text_off);
                }
#else
                addStringP(Printer::isZMinEndstopHit() ? ui_text_on : ui_text_off);
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else
                addStringP(ui_text_na);
#endif // (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
            }
            if (c2 == 'Z') // %sZ : State of z max endstop
            {
#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
                if (Printer::ZEndstopUnknown) {
                    addStringP(ui_text_unknown);
                } else {
                    addStringP(Printer::isZMaxEndstopHit() ? ui_text_on : ui_text_off);
                }
#else
                addStringP(Printer::isZMaxEndstopHit() ? ui_text_on : ui_text_off);
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

#else
                addStringP(ui_text_na);
#endif // (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
            }
            if (c2 == 'C') // %sC : State of the z compensation
            {
#if FEATURE_HEAT_BED_Z_COMPENSATION
                if (Printer::doHeatBedZCompensation) {
                    addStringP(ui_text_z_compensation_active);
                }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION
                if (Printer::doWorkPartZCompensation) {
                    addStringP(ui_text_z_compensation_active);
                }
#endif // FEATURE_WORK_PART_Z_COMPENSATION
            }

            if (c2 == 'S') // %sS : State of the sensible offset
            {
#if FEATURE_SENSIBLE_PRESSURE
                if (Printer::doHeatBedZCompensation) {
                    addInt((int)g_nSensiblePressureOffset, 4);
                } else {
                    addInt(0, 4);
                }
#endif // FEATURE_SENSIBLE_PRESSURE
            }

            if (c2 == 'M') // %sM : State of the sensible offset
            {
#if FEATURE_SENSIBLE_PRESSURE
                if (Printer::doHeatBedZCompensation && g_nSensiblePressureDigits > 0) {
                    if (g_nSensiblePressure1stMarke)
                        addStringP(PSTR("^"));
                    else
                        addStringP(PSTR("@"));
                    addInt((int)g_nSensiblePressureDigits, 5);
                } else {
                    addStringP(ui_text_off);
                }
#endif // FEATURE_SENSIBLE_PRESSURE
            }
            if (c2 == 'm') // %sm : State of the sensible pressure in eeprom
            {
#if FEATURE_SENSIBLE_PRESSURE
                addInt(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS), 5);
#endif // FEATURE_SENSIBLE_PRESSURE
            }
            if (c2 == 'o') // %so : State of the sensible pressure maxoffset in eeprom
            {
#if FEATURE_SENSIBLE_PRESSURE
                addInt(HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX), 5);
#endif // FEATURE_SENSIBLE_PRESSURE
            }
            if (c2 == 'a') // %sa : State of the sensible pressure autostarter
            {
#if FEATURE_SENSIBLE_PRESSURE
                addStringP(Printer::g_senseoffset_autostart ? ui_text_on : ui_text_off);
#endif // FEATURE_SENSIBLE_PRESSURE
            }

            if (c2 == '1') // %s1 : current value of the strain gauge
            {
                addInt(g_nLastDigits, 5);
            }

            if (c2 == 'k') // %sk : Positioning coordinate system for XY (move by gode or directoffset)
            {
                if (Printer::moveKosys == KOSYS_GCODE) {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                } else /* if (Printer::moveKosys == KOSYS_DIRECTOFFSET) */ {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                }
            }

            if (c2 == 'K') // %sK : Feedrate source config for direct positioning moves
            {
                if (Printer::movePositionFeedrateChoice == FEEDRATE_DIRECTCONFIG) {
                    addStringP(PSTR(UI_TEXT_MOVE_FEEDRATE_STANDARD));
                } else /* if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE) */ {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                }
            }

            if (c2 == 'f') // %sf : Feedrate for menu postition xy
            {
                if (Printer::movePositionFeedrateChoice == FEEDRATE_DIRECTCONFIG) {
                    addInt(STANDARD_POSITION_FEEDRATE_XY, 3);
                } else /* if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE) */ {
                    addInt(Printer::feedrate, 3);
                }
                addStringP(PSTR("mm/s"));
            }
            if (c2 == 'F') // %sF : Feedrate for menu postition z
            {
                if (Printer::movePositionFeedrateChoice == FEEDRATE_GCODE && Printer::feedrate <= STANDARD_POSITION_FEEDRATE_Z) {
                    addInt(Printer::feedrate, 3);
                } else {
                    addInt(STANDARD_POSITION_FEEDRATE_Z, 3);
                }
                addStringP(PSTR("mm/s"));
            }

            break;
        }
        case 'S': {
            if (c2 == '0')
                addFloat(extruder[0].stepsPerMM, 3, 0); // %S0 : Steps per mm extruder0
            else if (c2 == '1')
                addFloat(extruder[1].stepsPerMM, 3, 0); // %S1 : Steps per mm extruder1
            else if (c2 == 'e')
                addFloat(Extruder::current->stepsPerMM, 3, 0); // %Se : Steps per mm current extruder
            else if (c2 == 'z')
                addFloat(g_nManualSteps[Z_AXIS] * Printer::axisMMPerSteps[Z_AXIS] * 1000.0f, 4, 1); // %Sz : Mikrometer per Z-Single_Step (Z_Axis)
            else if (c2 == 'M' && col < MAX_COLS) {
                if (g_ZMatrixChangedInRam)
                    printCols[col++] = '*';
            } // %SM : Matrix has changed in Ram and is ready to Save. -> *)
#if FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION
            else if (c2 == 's')
                addFloat(g_scanStartZLiftMM, 1, 1); // %Ss : active current value of HEAT_BED_SCAN_Z_START_MM
#endif                                              // FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION
            break;
        }
        case 'p': {
            if (c2 == 'x') // %px: mode of the Position X menu
            {
                switch (Printer::moveMode[X_AXIS]) {
                case MOVE_MODE_SINGLE_STEPS: {
                    addFloat(g_nManualSteps[X_AXIS] * Printer::axisMMPerSteps[X_AXIS] * 1000.0f, 1, 0);
                    addStringP(PSTR(" um"));
                    break;
                }
                case MOVE_MODE_1_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_1_MM));
                    break;
                }
                case MOVE_MODE_10_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_10_MM));
                    break;
                }
                case MOVE_MODE_50_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_50_MM));
                    break;
                }
                case MOVE_MODE_SINGLE_MOVE: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                    break;
                }
                }
                if (Printer::moveMode[X_AXIS] == MOVE_MODE_SINGLE_MOVE || Printer::moveKosys == KOSYS_DIRECTOFFSET || g_pauseMode /*we are in pause position menu*/) {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                    addStringP(PSTR(")"));
                } else {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                    addStringP(PSTR(")"));
                }
                break;
            }
            if (c2 == 'y') // %py: mode of the Position Y menu
            {
                switch (Printer::moveMode[Y_AXIS]) {
                case MOVE_MODE_SINGLE_STEPS: {
                    addFloat(g_nManualSteps[Y_AXIS] * Printer::axisMMPerSteps[Y_AXIS] * 1000.0f, 1, 0);
                    addStringP(PSTR(" um"));
                    break;
                }
                case MOVE_MODE_1_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_1_MM));
                    break;
                }
                case MOVE_MODE_10_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_10_MM));
                    break;
                }
                case MOVE_MODE_50_MM: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_50_MM));
                    break;
                }
                case MOVE_MODE_SINGLE_MOVE: {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                    break;
                }
                }
                if (Printer::moveMode[Y_AXIS] == MOVE_MODE_SINGLE_MOVE || Printer::moveKosys == KOSYS_DIRECTOFFSET || g_pauseMode /*we are in pause position menu Y*/) {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                    addStringP(PSTR(")"));
                } else {
                    addStringP(PSTR(" ("));
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                    addStringP(PSTR(")"));
                }
                break;
            }
            if (c2 == 'z') // %pz: mode of the Position Z menu
            {
                if (g_pauseMode /*we are in pause position menu Z*/) {
                    addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                } else {
                    switch (Printer::moveMode[Z_AXIS]) {
                    case MOVE_MODE_SINGLE_STEPS: {
                        addFloat(g_nManualSteps[Z_AXIS] * Printer::axisMMPerSteps[Z_AXIS] * 1000.0f, 1, 1);
                        addStringP(PSTR(" um"));
                        break;
                    }
                    case MOVE_MODE_1_MM: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_1_MM));
                        break;
                    }
                    case MOVE_MODE_10_MM: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_10_MM));
                        break;
                    }
                    case MOVE_MODE_50_MM: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_50_MM));
                        break;
                    }
                    case MOVE_MODE_SINGLE_MOVE: {
                        addStringP(PSTR(UI_TEXT_MOVE_MODE_SINGLE_MOVE));
                        break;
                    }
                    }
                }

                // Z axis is always direct offset move because gcode does not make sense here.

                //if (Printer::moveMode[Z_AXIS] == MOVE_MODE_SINGLE_MOVE || Printer::moveKosys == KOSYS_DIRECTOFFSET || g_pauseMode /*we are in pause position menu Z*/)
                //{
                addStringP(PSTR(" ("));
                addStringP(PSTR(UI_TEXT_MOVE_MODE_OFFSET));
                addStringP(PSTR(")"));
                //}
                //else {
                //	addStringP(PSTR(UI_TEXT_MOVE_MODE_GCODE));
                //}
                break;
            }
#if FEATURE_EMERGENCY_PAUSE
            if (c2 == 'l') // %pl : g_nEmergencyPauseDigitsMin [1700/kg]
            {
                addLong(g_nEmergencyPauseDigitsMin, 6);
                break;
            }
            if (c2 == 'h') // %ph : g_nEmergencyPauseDigitsMax [1700/kg]
            {
                addLong(g_nEmergencyPauseDigitsMax, 6);
                break;
            }
#endif //FEATURE_EMERGENCY_PAUSE
#if FEATURE_EMERGENCY_STOP_Z_AND_E
            if (c2 == 'L') // %pL : g_nEmergencyStopZAndEMin [1700/kg]
            {
                addLong(g_nEmergencyStopZAndEMin, 6);
                break;
            }
            if (c2 == 'H') // %pH : g_nEmergencyStopZAndEMax [1700/kg]
            {
                addLong(g_nEmergencyStopZAndEMax, 6);
                break;
            }
#endif // FEATURE_EMERGENCY_STOP_Z_AND_E
            break;
        }
        case 'P': {
            if (c2 == 'N')
                addStringP(PSTR(UI_PRINTER_NAME)); // %PN : Printer name
            break;
        }
        case 'U': // %U1: Page1
        {
            if (c2 == '1') // temperature icon
            {
                if (!normalchars)
                    initNSpecchars();
                addStringP(PSTR("\005"));
            } else if (c2 == '2') // replace line with new line
            {
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_MILL) {
                    for (uint8_t n = 0; n < MAX_COLS + 1; n++)
                        printCols[n] = 0; //clear all text
                    col = 0;              //reset linemarker
#if FEATURE_230V_OUTPUT
#if FEATURE_CASE_LIGHT
                    addStringP(PSTR("MIL X19:")); // -> Die restliche Zeile wird komplett überschrieben und der rest verworfen. Weil Millingmode keine Temperaturen hat.
                    addStringP(Printer::enableCaseLight ? ui_text_on : ui_text_off);
                    addStringP(PSTR(" 230V:"));
                    addStringP(Printer::enable230VOutput ? ui_text_on : ui_text_off);
#else
                    addStringP(PSTR("MILLER  230V:"));
                    addStringP(Printer::enable230VOutput ? ui_text_on : ui_text_off);
#endif
#else //FEATURE_230V_OUTPUT
#if FEATURE_CASE_LIGHT
                    addStringP(PSTR("MILLER X19:")); // -> Die restliche Zeile wird komplett überschrieben und der rest verworfen. Weil Millingmode keine Temperaturen hat.
                    addStringP(Printer::enableCaseLight ? ui_text_on : ui_text_off);
#else
                    addStringP(PSTR("MILLER"));
#endif
#endif //FEATURE_230V_OUTPUT
                }
#endif // FEATURE_MILLING_MODE
            }
            break;
        }
        case 'C': {
#if FEATURE_READ_CALIPER
            if (c2 == 'a') // %Ca : Caliper Active Reading
            {
                addLong(abs(caliper_um) + caliper_collect_adjust, 5);
            } else if (c2 == 'm') // %Cm : Caliper Average in mm
            {
                addFloat(0.001f * caliper_collect_um / caliper_collect_count, 2, 3);
            } else if (c2 == 's') // %Cs : Caliper Filament Standard
            {
                addFloat(0.001f * caliper_filament_standard, 2, 3);
            } else if (c2 == 'c') // %Cc : Caliper Correction
            {
                addInt(caliper_collect_adjust, 4);
            } else if (c2 == 'n') // %Cn : needed flow multi for measurement
            {
                if (caliper_collect_um && caliper_collect_count) {
                    float dim = (float)caliper_filament_standard / (caliper_collect_um / caliper_collect_count);
                    float multi = 100.0f * dim * dim;
                    addInt((int)multi, 3);
                } else {
                    addInt(100, 3);
                }
            }
#endif //FEATURE_READ_CALIPER
#if FEATURE_DIGIT_FLOW_COMPENSATION
            if (c2 == 'U') // %CU : digit flow lower limit
            {
                addInt((int)g_nDigitFlowCompensation_Fmin, 5);
            } else if (c2 == 'O') // %CO : digit flow higher limit
            {
                addInt((int)g_nDigitFlowCompensation_Fmax, 5);
            } else if (c2 == 'F') // %CF : digit flow flowrate
            {
                addInt((int)g_nDigitFlowCompensation_intense, 3); //eigentlcih sollten wir F und E hier tauschen... egal.
            } else if (c2 == 'E')                                 // %CE : digit flow feedrate
            {
                addInt((int)g_nDigitFlowCompensation_speed_intense, 3);
            }
#endif // FEATURE_DIGIT_FLOW_COMPENSATION
            break;
        }
        case 'L': {
            if (c2 == 'L') // %LL : Last Layer (Queue)
            {
                addFloat(float(Printer::queuePositionZLayerLast * Printer::axisMMPerSteps[Z_AXIS]), 3, 2);
                break;
            } else if (c2 == 'C') // %LC : Current Layer (Queue)
            {
                addFloat(float(Printer::queuePositionZLayerCurrent * Printer::axisMMPerSteps[Z_AXIS]), 3, 2);
                break;
            } else if (c2 == 'H') // %LH : Layer Height
            {
                addFloat(float(Printer::queuePositionZLayerCurrent - Printer::queuePositionZLayerLast) * Printer::axisMMPerSteps[Z_AXIS], 1, 2);
                break;
            } else if (c2 == 'P') // %LP : ECMP %
            {
                addFloat(Printer::compensatedPositionOverPercE * 100, 1, 2);
                break;
            } else if (c2 == 'm') // %Lm : g_minZCompensationSteps
            {
                addFloat(float(g_minZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS]), 1, 2);
                break;
            } else if (c2 == 'M') // %LM : g_maxZCompensationSteps
            {
                addFloat(float(g_maxZCompensationSteps * Printer::axisMMPerSteps[Z_AXIS]), 2, 2);
                break;
            }
            break;
        }
#if FEATURE_Kurt67_WOBBLE_FIX
        case 'w': //wobblefix
        {
            if (c2 == 'x') // %wx : current wobblefix offset in x [um] (Bauchtanz)
            {
                addInt(Printer::lastWobbleFixOffset[X_AXIS] * 1000, 4);
                break;
            } else if (c2 == 'y') // %wy : current wobblefix offset in y [um] (Bauchtanz)
            {
                addInt(Printer::lastWobbleFixOffset[Y_AXIS] * 1000, 4);
                break;
            }

            else if (c2 == 'a') // %wa : current wobblefix amplitude for X
            {
                addInt(Printer::wobbleAmplitudes[0], 4);
                break;
            } else if (c2 == 'b') // %wb : current wobblefix amplitude for Y(x_0)
            {
                addInt(Printer::wobbleAmplitudes[1], 4);
                break;
            } else if (c2 == 'c') // %wc : current wobblefix amplitude for Y(x_245)
            {
                addInt(Printer::wobbleAmplitudes[2], 4);
                break;
            } else if (c2 == 'P') // %wP : current wobblefix phase for YX-wobble (Bauchtanz)
            {
                addInt(long(float(Printer::wobblePhaseXY) * 1.8f), 4);
                break;
            }

            break;
        }
#endif            //FEATURE_Kurt67_WOBBLE_FIX
        case 'Z': // %Z1-Z4: Page5 service intervall, %Z5-Z8: Page4 printing/milling time
        {
            if (c2 == '1') // Shows text printing/milling time since last service
            {
#if FEATURE_SERVICE_INTERVAL
                addStringP(PSTR(UI_TEXT_SERVICE_TIME));
#endif                            // FEATURE_SERVICE_INTERVAL
            } else if (c2 == '2') // Shows printing/milling time since last service
            {
#if FEATURE_SERVICE_INTERVAL
#if EEPROM_MODE != 0
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
                    bool alloff = true;
                    for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
                        if (tempController[i]->targetTemperatureC > 0)
                            alloff = false;

                    long uSecondsServicePrint = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME_SERVICE);
                    long tmp_service = uSecondsServicePrint / 86400;
                    uSecondsServicePrint -= tmp_service * 86400;
                    addInt(tmp_service, 5);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_DAYS));
                    tmp_service = uSecondsServicePrint / 3600;
                    addInt(tmp_service, 2);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_HOURS));
                    uSecondsServicePrint -= tmp_service * 3600;
                    tmp_service = uSecondsServicePrint / 60;
                    addInt(tmp_service, 2, '0');
                    addStringP(PSTR(UI_TEXT_PRINTTIME_MINUTES));
#if FEATURE_MILLING_MODE
                } else {
                    long uSecondsServicePrint = (HAL::timeInMilliseconds() - Printer::msecondsMilling) / 1000 + HAL::eprGetInt32(EPR_MILLING_TIME_SERVICE);
                    long tmp_service = uSecondsServicePrint / 86400;
                    uSecondsServicePrint -= tmp_service * 86400;
                    addInt(tmp_service, 5);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_DAYS));
                    tmp_service = uSecondsServicePrint / 3600;
                    addInt(tmp_service, 2);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_HOURS));
                    uSecondsServicePrint -= tmp_service * 3600;
                    tmp_service = uSecondsServicePrint / 60;
                    addInt(tmp_service, 2, '0');
                    addStringP(PSTR(UI_TEXT_PRINTTIME_MINUTES));
                }
#endif                            // FEATURE_MILLING_MODE
#endif                            // EEPROM_MODE
#endif                            // FEATURE_SERVICE_INTERVAL
            } else if (c2 == '3') // Shows text printed filament since last service
            {
#if FEATURE_SERVICE_INTERVAL
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_PRINT)
#endif // FEATURE_MILLING_MODE
                {
                    addStringP(PSTR(UI_TEXT_PRINT_FILAMENT));
                }
#endif                            // FEATURE_SERVICE_INTERVAL
            } else if (c2 == '4') // Shows printed filament since last service
            {
#if FEATURE_SERVICE_INTERVAL
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_PRINT)
#endif // FEATURE_MILLING_MODE
                {
#if EEPROM_MODE != 0
                    float dist_service = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE_SERVICE);
                    addFloat(dist_service, 6, 1);
                    addStringP(PSTR(" m"));
#endif // EEPROM_MODE
                }
#endif                            // FEATURE_SERVICE_INTERVAL
            } else if (c2 == '5') // Shows text printing/milling time
            {
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
                    addStringP(PSTR(UI_TEXT_PRINT_TIME));
#if FEATURE_MILLING_MODE
                } else {
                    addStringP(PSTR(UI_TEXT_MILL_TIME));
                }
#endif                            // FEATURE_MILLING_MODE
            } else if (c2 == '6') // Shows printing/milling time
            {
#if EEPROM_MODE != 0
#if FEATURE_MILLING_MODE
                if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
                    bool alloff = true;
                    for (uint8_t i = 0; i < NUM_EXTRUDER; i++)
                        if (tempController[i]->targetTemperatureC > 0)
                            alloff = false;

                    long seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
                    long tmp = seconds / 86400;
                    seconds -= tmp * 86400;
                    addInt(tmp, 5);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_DAYS));
                    tmp = seconds / 3600;
                    addInt(tmp, 2);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_HOURS));
                    seconds -= tmp * 3600;
                    tmp = seconds / 60;
                    addInt(tmp, 2, '0');
                    addStringP(PSTR(UI_TEXT_PRINTTIME_MINUTES));
#if FEATURE_MILLING_MODE
                } else {
                    long seconds = (HAL::timeInMilliseconds() - Printer::msecondsMilling) / 1000 + HAL::eprGetInt32(EPR_MILLING_TIME);
                    long tmp = seconds / 86400;
                    seconds -= tmp * 86400;
                    addInt(tmp, 5);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_DAYS));
                    tmp = seconds / 3600;
                    addInt(tmp, 2);
                    addStringP(PSTR(UI_TEXT_PRINTTIME_HOURS));
                    seconds -= tmp * 3600;
                    tmp = seconds / 60;
                    addInt(tmp, 2, '0');
                    addStringP(PSTR(UI_TEXT_PRINTTIME_MINUTES));
                }
#endif                            // FEATURE_MILLING_MODE
#endif                            // EEPROM_MODE
            } else if (c2 == '7') // Shows text printed filament
            {
                addStringP(PSTR(UI_TEXT_PRINT_FILAMENT));
            } else if (c2 == '8') // Shows printed filament
            {
#if EEPROM_MODE != 0
                float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
                addFloat(dist, 6, 1);
                addStringP(PSTR(" m"));
#endif // EEPROM_MODE
            }
            break;
        }
        }
    }
    printCols[col] = 0;

} // parse

void UIDisplay::setStatusP(PGM_P txt, bool error) {
    if (locked) {
        // we shall not update the display
        return;
    }
    if (!error && Printer::isUIErrorMessage()) {
        return;
    }

    uint8_t i = 0;
    while (i < UI_COLS) {
        uint8_t c = pgm_read_byte(txt++);
        if (!c)
            break;
        statusMsg[i++] = c;
    }
    statusMsg[i] = 0;
    if (error)
        Printer::setUIErrorMessage(true);

} // setStatusP

void UIDisplay::setStatus(char* txt, bool error, bool force) {
    if (locked && !force) {
        // we shall not update the display
        return;
    }
    if (!error && Printer::isUIErrorMessage())
        return;

    uint8_t i = 0;
    while (*txt && i < UI_COLS)
        statusMsg[i++] = *txt++;
    statusMsg[i] = 0;

    if (error)
        Printer::setUIErrorMessage(true);

} // setStatus

const UIMenu* const ui_pages[UI_NUM_PAGES] PROGMEM = UI_PAGES;

#if SDSUPPORT
uint8_t nFilesOnCard;
void UIDisplay::updateSDFileCount() {
    dir_t* p = NULL;
    SdBaseFile* root = sd.fat.vwd();

    root->rewind();
    nFilesOnCard = 0;
    while ((p = root->getLongFilename(p, NULL, 0, NULL))) {
        if (!(DIR_IS_FILE(p) || DIR_IS_SUBDIR(p)))
            continue;
        if (folderLevel >= SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0] == '.' && p->name[1] == '.'))
            continue;
        nFilesOnCard++;
        if (nFilesOnCard == 254)
            return;
    }
} // updateSDFileCount

void getSDFilenameAt(byte filePos, char* filename) {
    dir_t* p = NULL;
    SdBaseFile* root = sd.fat.vwd();

    root->rewind();
    while ((p = root->getLongFilename(p, tempLongFilename, 0, NULL))) {
        if (!DIR_IS_FILE(p) && !DIR_IS_SUBDIR(p))
            continue;
        if (uid.folderLevel >= SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0] == '.' && p->name[1] == '.'))
            continue;
        if (filePos--)
            continue;
        strcpy(filename, tempLongFilename);
        if (DIR_IS_SUBDIR(p))
            strcat(filename, "/"); // Set marker for directory
        break;
    }
} // getSDFilenameAt

bool UIDisplay::isDirname(char* name) {
    while (*name)
        name++;
    name--;
    return *name == '/';

} // isDirname

void UIDisplay::goDir(char* name) {
    char* p = cwd;
    while (*p)
        p++;
    if (name[0] == '.' && name[1] == '.') {
        if (folderLevel == 0)
            return;
        p--;
        p--;
        while (*p != '/')
            p--;
        p++;
        *p = 0;
        folderLevel--;
    } else {
        if (folderLevel >= SD_MAX_FOLDER_DEPTH)
            return;
        while (*name)
            *p++ = *name++;
        *p = 0;
        folderLevel++;
    }
    sd.fat.chdir(cwd);
    updateSDFileCount();
} // goDir

void sdrefresh(uint16_t& r, char cache[UI_ROWS][MAX_COLS + 1]) {
    dir_t* p = NULL;
    byte offset = uid.menuTop[uid.menuLevel];
    SdBaseFile* root;
    byte length, skip;

    sd.fat.chdir(uid.cwd);
    root = sd.fat.vwd();
    root->rewind();

    skip = (offset > 0 ? offset - 1 : 0);

    while (int16_t(r) + offset < int16_t(nFilesOnCard) + 1 && r < UI_ROWS && (p = root->getLongFilename(p, tempLongFilename, 0, NULL))) {
        // done if past last used entry
        // skip deleted entry and entries for . and  ..
        // only list subdirectories and files
        if ((DIR_IS_FILE(p) || DIR_IS_SUBDIR(p))) {
            if (uid.folderLevel >= SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0] == '.' && p->name[1] == '.'))
                continue;
            if (skip > 0) {
                skip--;
                continue;
            }

            uid.printCols[0] = ' ';
            uid.col = 1;

            if (DIR_IS_SUBDIR(p))
                uid.printCols[uid.col++] = 6; // Prepend folder symbol
            length = RMath::min((int)strlen(tempLongFilename), (int)(MAX_COLS - uid.col));
            memcpy(uid.printCols + uid.col, tempLongFilename, length);
            uid.col += length;
            uid.printCols[uid.col] = 0;

            uint8_t curShift = (uid.shift <= 0 ? 0 : uid.shift);
            uint8_t curLen = strlen(uid.printCols);

            if (curLen > UI_COLS) {
                // this file name is longer than the available width of the display
                curShift = RMath::min(curLen - UI_COLS, curShift);
            } else {
                curShift = 0;
            }

            if (r + offset == uid.menuPos[uid.menuLevel]) {
                // the menu cursor is placed at this file name at the moment
                uid.printCols[curShift] = (char)CHAR_SELECTOR;
            } else {
                // this file name is above/below the current menu cursor item
                uid.printCols[curShift] = ' ';
            }

            if (DIR_IS_SUBDIR(p))
                uid.printCols[curShift + 1] = 6; // Prepend folder symbol

            strcpy(cache[r++], uid.printCols);
        }
    }
} // sdrefresh
#endif // SDSUPPORT

// Refresh current menu page
void UIDisplay::refreshPage() {
    uint16_t r;
#if SDSUPPORT
    uint8_t mtype = UI_MENU_TYPE_INFO;
#endif //SDSUPPORT
    char cache[UI_ROWS][MAX_COLS + 1];
    adjustMenuPos();

#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
#if UI_PRINT_AUTORETURN_TO_MENU_AFTER
        // Reset timeout on menu back when user active on menu
        if (encoderLast != encoderStartScreen)
            g_nAutoReturnTime = HAL::timeInMilliseconds() + UI_PRINT_AUTORETURN_TO_MENU_AFTER;
#endif // UI_PRINT_AUTORETURN_TO_MENU_AFTER
#if FEATURE_MILLING_MODE
    } else {
#if UI_MILL_AUTORETURN_TO_MENU_AFTER
        // Reset timeout on menu back when user active on menu
        if (encoderLast != encoderStartScreen)
            g_nAutoReturnTime = HAL::timeInMilliseconds() + UI_MILL_AUTORETURN_TO_MENU_AFTER;
#endif // UI_MILL_AUTORETURN_TO_MENU_AFTER
    }
#endif // FEATURE_MILLING_MODE

    encoderStartScreen = encoderLast;

    // Copy result into cache
    if (menuLevel == 0) {
        UIMenu* men = (UIMenu*)pgm_read_word(&(ui_pages[menuPos[0]]));
        uint16_t nr = pgm_read_word(&(men->numEntries));
        UIMenuEntry** entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
        for (r = 0; r < nr && r < UI_ROWS; r++) {
            UIMenuEntry* ent = (UIMenuEntry*)pgm_read_word(&(entries[r]));
            col = 0;
            parse((char*)pgm_read_word(&(ent->text)), false);
            strcpy(cache[r], printCols);
        }
    } else {
        UIMenu* men = (UIMenu*)menu[menuLevel];
        uint16_t nr = pgm_read_word(&(men->numEntries));
#if SDSUPPORT
        mtype = pgm_read_byte((void*)&(men->menuType));
#endif //SDSUPPORT
        uint8_t offset = menuTop[menuLevel];
        UIMenuEntry** entries = (UIMenuEntry**)pgm_read_word(&(men->entries));

        for (r = 0; r + offset < nr && r < UI_ROWS;) {
            UIMenuEntry* ent = (UIMenuEntry*)pgm_read_word(&(entries[r + offset]));
            if (!ent->showEntry()) {
                offset++;
                continue;
            }

            unsigned char entType = pgm_read_byte(&(ent->menuType));
            int entAction = (int)pgm_read_word(&(ent->action));

            printCols[0] = ' ';
            col = 1;
            parse((char*)pgm_read_word(&(ent->text)), false);
            while (col < UI_COLS - 1)
                printCols[col++] = ' ';
            printCols[col] = 0;

            if (entType >= 2 && entType <= 4) {
                // this is a menu item
                uint8_t curShift = (shift <= 0 ? 0 : shift);
                uint8_t curLen = strlen(printCols);

                if (entType == 2) {
                    // this menu item contains submenus
                    curLen++; // one additional character is needed for the submenu marker
                }

                if (curLen > UI_COLS) {
                    // this menu item is longer than the available width of the display
                    curShift = RMath::min(curLen - UI_COLS, curShift);
                } else {
                    curShift = 0;
                }

                if (r + offset == menuPos[menuLevel] && activeAction != entAction) {
                    // the menu cursor is placed at this item at the moment
                    printCols[curShift] = (char)CHAR_SELECTOR;
                } else if (activeAction == entAction) {
                    // this menu item is selected (and can be changed) at the moment
                    printCols[curShift] = CHAR_SELECTED;
                } else {
                    // this menu item is above/below the current menu cursor item
                    printCols[curShift] = ' ';
                }

                if (entType == 2) {
                    printCols[UI_COLS - 1 + curShift] = (char)CHAR_RIGHT; // arrow right
                    printCols[UI_COLS + curShift] = 0;                    // arrow right
                }
            }

            strcpy(cache[r], printCols);
            r++;
        }
    }

#if SDSUPPORT
    if (mtype == UI_MENU_TYPE_FILE_SELECTOR) {
        sdrefresh(r, cache);
    }
#endif // SDSUPPORT

    printCols[0] = 0;
    while (r < UI_ROWS)
        strcpy(cache[r++], printCols);

    // Compute transition
    uint8_t transition = 0; // 0 = display, 1 = up, 2 = down, 3 = left, 4 = right

    uint8_t loops = 1;
    uint8_t dt = 1, y;
    if (transition == 1 || transition == 2)
        loops = UI_ROWS;
    else if (transition > 2) {
        dt = (UI_COLS + UI_COLS - 1) / 16;
        loops = UI_COLS + 1 / dt;
    }
    uint8_t off0 = (shift <= 0 ? 0 : shift);
    uint8_t scroll = dt;
    uint8_t off[UI_ROWS];
    if (transition == 0) {
        for (y = 0; y < UI_ROWS; y++)
            strcpy(displayCache[y], cache[y]);
    }
    for (y = 0; y < UI_ROWS; y++) {
        uint8_t len = strlen(displayCache[y]);
        off[y] = len > UI_COLS ? RMath::min(len - UI_COLS, off0) : 0;
    }
    for (uint8_t l = 0; l < loops; l++) {
        if (encoderLast != encoderStartScreen) {
            scroll = 200;
        }
        scroll += dt;

        if (transition == 0) {
            for (y = 0; y < UI_ROWS; y++)
                printRow(y, &cache[y][off[y]], NULL, UI_COLS);
        }
    }
} // refreshPage

void UIDisplay::showMessage(bool refresh) {
    pushMenu((void*)&ui_menu_message, refresh);

    // show this message until the user acknowledges it
    g_nAutoReturnTime = 0;

} // showMessage

void UIDisplay::pushMenu(void* men, bool refresh) {
    if (men == menu[menuLevel]) {
        refreshPage();
        return;
    }
    if (menuLevel == (MAX_MENU_LEVELS - 1)) {
        return;
    }
    menuLevel++;
    menu[menuLevel] = men;
    menuTop[menuLevel] = menuPos[menuLevel] = 0;

#if SDSUPPORT
    UIMenu* men2 = (UIMenu*)menu[menuLevel];
    if (pgm_read_byte(&(men2->menuType)) == 1) // Open files list
        updateSDFileCount();
#endif // SDSUPPORT

    if (refresh)
        refreshPage();

} // pushMenu

void UIDisplay::menuEsc() {
    if (menuLevel == 1 && menuPos[0] == 1) {
        //der würde in das modmenü zurückgehen, da sind aber die rechtst links tasten anders belegt, daher nicht da rein! sonst evtl. verstellen von DigitLimit.
        menuPos[0] = 0;
    }
    if (menuLevel > 0)
        menuLevel--;
}

void UIDisplay::okAction() {
    if (Printer::isUIErrorMessage()) {
        Printer::setUIErrorMessage(false);
        return;
    }

    if (menuLevel == 0) // Enter menu
    {
        menuLevel = 1;
        menuTop[1] = menuPos[1] = 0;
        menu[1] = (void*)&ui_menu_main;
        BEEP_SHORT
        return;
    }

    UIMenu* men = (UIMenu*)menu[menuLevel];
    uint8_t mtype = pgm_read_byte(&(men->menuType));
    UIMenuEntry** entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    UIMenuEntry* ent = (UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]]));
    unsigned char entType = pgm_read_byte(&(ent->menuType)); // 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command, 4 = modify action
    int action = pgm_read_word(&(ent->action));

    if (mtype == UI_MENU_TYPE_MODIFICATION_MENU) // action menu
    {
        action = pgm_read_word(&(men->id));
        finishAction(action);
        menuEsc();
        return;
    }
    if (mtype == UI_MENU_TYPE_SUBMENU && entType == 4) // Modify action
    {
        if (activeAction) // finish action
        {
            finishAction(action);
            activeAction = 0;
        } else
            activeAction = action;
        return;
    }

#if SDSUPPORT
    if (mtype == UI_MENU_TYPE_FILE_SELECTOR) {
        if ((menuPos[menuLevel] == 0 && folderLevel == 0) /* Selected back instead of file */
            || !sd.sdactive)                              /* No SD -> drop menuposition */
        {
            menuEsc();
            return;
        }
        if (menuPos[menuLevel] == 0 && folderLevel > 0) {
            goDir((char*)"..");
            menuTop[menuLevel] = 0;
            menuPos[menuLevel] = 1;
            refreshPage();
            return;
        }
        uint8_t filePos = menuPos[menuLevel] - 1;
        char filename[LONG_FILENAME_LENGTH + 1];

        getSDFilenameAt(filePos, filename);
        if (isDirname(filename)) // Directory change selected
        {
            goDir(filename);
            menuTop[menuLevel] = 0;
            menuPos[menuLevel] = 1;
            refreshPage();
            return;
        }

        sd.file.close();
        sd.fat.chdir(cwd);

        if (sd.selectFile(filename, false)) {
            sd.startPrint();
            BEEP_START_PRINTING
            exitmenu();
        }

        return;
    }
#endif // SDSUPPORT

    if (entType == 2) { // Enter submenu
        pushMenu((void*)action, false);
        BEEP_SHORT
        return;
    }
    if (entType == 3) {
        executeAction(action);
        return;
    }
    menuEsc();
} // okAction

void UIDisplay::senseOffsetUpAction() {
    //we are in the Mod menu
    if (g_nSensiblePressureDigits == EMERGENCY_PAUSE_DIGITS_MAX * 0.8 || g_nSensiblePressureDigits == 32767) {
        //ist max, dann auf 0.
        g_nSensiblePressureDigits = 0;
    } else if (g_nSensiblePressureDigits > EMERGENCY_PAUSE_DIGITS_MAX * 0.8 - 250 || g_nSensiblePressureDigits > 32767 - 250) {
        //stößt oben an, hier noch check auf overflow:
        if (EMERGENCY_PAUSE_DIGITS_MAX * 0.8 < 32767) {
            g_nSensiblePressureDigits = EMERGENCY_PAUSE_DIGITS_MAX * 0.8; //maximalstellung, das ist aber schon irre hoch. abhängig von messdose und maximaltragkraft vs. genauigkeit der zelle.
        } else {
            g_nSensiblePressureDigits = 32767; //die variable ist short, nie mehr rein als nötig ^^.
        }
    } else {
        //TPE braucht mini werte, wenn es sinnvoll sein soll. Darum der Ternary, sodass man per Knopf auch kleinste Zahlen justieren kann.
        g_nSensiblePressureDigits += (g_nSensiblePressureDigits >= 2000) ? 250 : (g_nSensiblePressureDigits >= 500) ? 100 : 50; //decrement pro Knopfklick. Man kann ja auf der Taste bleiben.
                                                                                                                                //g_nSensiblePressureDigits += 250; //decrement pro Knopfklick. Man kann ja auf der Taste bleiben.

        //Wir speichern nur Werte automatisch per Knopf, die im Alltag sinn machen können. Ab 500:
        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
        if (g_nSensiblePressureDigits >= 500 && oldval != g_nSensiblePressureDigits) {
            HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, g_nSensiblePressureDigits);
            EEPROM::updateChecksum(); //deshalb die prüfung
        }
    }
}

void UIDisplay::senseOffsetDownAction() {
    if (g_nSensiblePressureDigits == 0) {
        if (EMERGENCY_PAUSE_DIGITS_MAX * 0.8 < 32767) {
            g_nSensiblePressureDigits = EMERGENCY_PAUSE_DIGITS_MAX * 0.8; //maximalstellung, das ist aber schon irre hoch. abhängig von messdose und maximaltragkraft vs. genauigkeit der zelle.
        } else {
            g_nSensiblePressureDigits = 32767; //die variable ist short, nie mehr rein als nötig ^^.
        }
    } else if (g_nSensiblePressureDigits < 50) { // ex 250
        g_nSensiblePressureDigits = 0;
    } else {
        //TPE braucht mini werte, wenn es sinnvoll sein soll. Darum der Ternary, sodass man per Knopf auch kleinste Zahlen justieren kann.
        //g_nSensiblePressureDigits -= 250; //decrement pro Knopfklick. Man kann ja auf der Taste bleiben.
        g_nSensiblePressureDigits -= (g_nSensiblePressureDigits >= 2000) ? 250 : (g_nSensiblePressureDigits >= 500) ? 100 : 50; //decrement pro Knopfklick. Man kann ja auf der Taste bleiben.

        //Wir speichern nur Werte automatisch per Knopf, die im Alltag sinn machen können. Ab 500:
        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
        if (g_nSensiblePressureDigits >= 500 && oldval != g_nSensiblePressureDigits) {
            HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, g_nSensiblePressureDigits);
            EEPROM::updateChecksum(); //deshalb die prüfung
        }
    }
}

void UIDisplay::zOffsetPlusAction() {
    long nTemp = Printer::ZOffset; //um --> mm*1000
    nTemp += Z_OFFSET_BUTTON_STEPS;
    //beim Überschreiten von 0, soll 0 erreicht werden, sodass man nicht mit krummen Zahlen rumhantieren muss.
    if (nTemp < Z_OFFSET_BUTTON_STEPS && nTemp > 0)
        nTemp = 0;
    if (nTemp < -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000))
        nTemp = -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);
    if (nTemp > (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000))
        nTemp = (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);
    Printer::ZOffset = nTemp;
#if FEATURE_SENSIBLE_PRESSURE
    g_staticZSteps = long(((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000);
#else
    g_staticZSteps = long((Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000);
#endif //FEATURE_SENSIBLE_PRESSURE
    if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != Printer::ZOffset) {
        HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::ZOffset);
        EEPROM::updateChecksum();
    }
}

void UIDisplay::zOffsetMinusAction() {
    long nTemp = Printer::ZOffset; //um --> mm*1000
    nTemp -= Z_OFFSET_BUTTON_STEPS;
    //beim Unterschreiten von 0, soll 0 erreicht werden, sodass man nicht mit krummen Zahlen rumhantieren muss.
    if (nTemp < 0 && nTemp > -Z_OFFSET_BUTTON_STEPS)
        nTemp = 0;
#if FEATURE_SENSIBLE_PRESSURE
    /* IDEE: Wenn automatisches Offset und wir korrigieren dagegen, soll erst dieses abgebaut werden */
    if (g_nSensiblePressureOffset > 0) { //aus: dann 0, an: dann > 0
                                         //automatik hat das bett runtergefahren, wir fahren es mit negativem offset hoch.
                                         //blöd: damit ist die automatik evtl. weiter am limit und kann nachfolgend nichts mehr tun.
                                         //also erst ausgleichen! dann verändern.
        if (g_nSensiblePressureOffset > Z_OFFSET_BUTTON_STEPS) {
            nTemp += Z_OFFSET_BUTTON_STEPS;
            g_nSensiblePressureOffset -= Z_OFFSET_BUTTON_STEPS;
        } else {
            nTemp += g_nSensiblePressureOffset;
            g_nSensiblePressureOffset = 0;
        }
    }
#endif //FEATURE_SENSIBLE_PRESSURE
    if (nTemp < -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000))
        nTemp = -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);
    if (nTemp > (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000))
        nTemp = (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000);
    Printer::ZOffset = nTemp;
#if FEATURE_SENSIBLE_PRESSURE
    g_staticZSteps = long(((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000);
#else
    g_staticZSteps = long((Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000);
#endif //FEATURE_SENSIBLE_PRESSURE
    if (HAL::eprGetInt32(EPR_RF_Z_OFFSET) != Printer::ZOffset) {
        HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::ZOffset);
        EEPROM::updateChecksum();
    }
}

void UIDisplay::rightAction() {
#if FEATURE_SENSIBLE_PRESSURE
    if (menuLevel == 0 && menuPos[0] == 1) { //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
        senseOffsetUpAction();
        beep(1, 4);
    } else
#endif
    {
        if (menu[menuLevel] == &ui_menu_xpos) {
            Printer::moveMode[X_AXIS]++;
            if (Printer::moveMode[X_AXIS] > MOVE_MODE_SINGLE_MOVE) {
                Printer::moveMode[X_AXIS] = MOVE_MODE_SINGLE_STEPS;
            }
            refreshPage();

            HAL::eprSetByte(EPR_RF_MOVE_MODE_X, Printer::moveMode[X_AXIS]);
            EEPROM::updateChecksum();
        } else if (menu[menuLevel] == &ui_menu_ypos) {
            Printer::moveMode[Y_AXIS]++;
            if (Printer::moveMode[Y_AXIS] > MOVE_MODE_SINGLE_MOVE) {
                Printer::moveMode[Y_AXIS] = MOVE_MODE_SINGLE_STEPS;
            }
            refreshPage();

            HAL::eprSetByte(EPR_RF_MOVE_MODE_Y, Printer::moveMode[Y_AXIS]);
            EEPROM::updateChecksum();
        } else if (menu[menuLevel] == &ui_menu_zpos) {
            Printer::moveMode[Z_AXIS]++;
            if (Printer::moveMode[Z_AXIS] > MOVE_MODE_SINGLE_MOVE) {
                Printer::moveMode[Z_AXIS] = MOVE_MODE_SINGLE_STEPS;
            }
            refreshPage();

            HAL::eprSetByte(EPR_RF_MOVE_MODE_Z, Printer::moveMode[Z_AXIS]);
            EEPROM::updateChecksum();
        }
    }
} // rightAction

void UIDisplay::backAction() {
#if FEATURE_SENSIBLE_PRESSURE
    if (menuLevel == 0 && menuPos[0] == 1) {
        //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
        //we are in the Mod menu
        //verkleinern des Digit-Limits
        senseOffsetDownAction();
        beep(1, 4);
    } else
#endif
    {
#if FEATURE_RGB_LIGHT_EFFECTS
        if (menuLevel == 0) {
            Printer::RGBButtonBackPressed = 1;
        }
#endif // FEATURE_RGB_LIGHT_EFFECTS
        menuEsc();
        Printer::setAutomount(false);
        activeAction = 0;
        g_nYesNo = 0;
    }
} // backAction

//increment ist ne variable, global -> Knopfrichtung.
//#define INCREMENT_MIN_MAX(a,steps,_min,_max) if ( (increment<0) && (_min>=0) && (a<_min-increment*steps) ) {a=_min;} else { a+=increment*steps; if(a<_min) a=_min; else if(a>_max) a=_max;};
// this version not have single byte variable rollover bug
#define INCREMENT_MIN_MAX(a, steps, _min, _max) a = constrain((a + increment * steps), _min, _max);

#define INCREMENT_MAX(a, steps, _max) \
    if (increment < 0 && a <= -1 * increment * steps) \
        a = 0; \
    else { \
        a += increment * steps; \
        if (a > _max) \
            a = _max; \
    };
//#define INCREMENT_MAX(a,steps,_max) a = constrain((a + increment * steps), 0, _max); //Nibbels: ist für unsigned variablen gegen warning. 2x vorkommen

void UIDisplay::adjustMenuPos() {
    if (menuLevel == 0)
        return;
    UIMenu* men = (UIMenu*)menu[menuLevel];
    UIMenuEntry** entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    uint8_t mtype = HAL::readFlashByte((const prog_char*)&(men->menuType));
    if (mtype != UI_MENU_TYPE_SUBMENU)
        return;

    while (menuPos[menuLevel] > 0) {
        if (((UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]])))->showEntry())
            break;

        menuPos[menuLevel]--;
    }
    while (1) {
        if (((UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]])))->showEntry())
            break;

        menuPos[menuLevel]++;
    }
    uint8_t skipped = 0;
    bool modified;

    if (menuTop[menuLevel] > menuPos[menuLevel])
        menuTop[menuLevel] = menuPos[menuLevel];
    do {
        skipped = 0;
        modified = false;
        for (uint8_t r = menuTop[menuLevel]; r < menuPos[menuLevel]; r++) {
            UIMenuEntry* ent = (UIMenuEntry*)pgm_read_word(&(entries[r]));
            if (!ent->showEntry())
                skipped++;
        }
        if (menuTop[menuLevel] + skipped + UI_ROWS - 1 < menuPos[menuLevel]) {
            menuTop[menuLevel] = menuPos[menuLevel] + 1 - UI_ROWS;
            modified = true;
        }

    } while (modified);
} // adjustMenuPos

void UIDisplay::finishAction(int action) {
    switch (action) {
    case UI_ACTION_RF_RESET_ACK: {
        if (g_nYesNo != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        Com::printFLN(PSTR("Reset via Menu"));
        HAL::delayMilliseconds(100);
        Commands::emergencyStop();
        break;
    }

    case UI_ACTION_STOP_ACK: {
        if (g_nYesNo != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        exitmenu();
        Printer::stopPrint();
        break;
    }

    case UI_ACTION_RESTORE_DEFAULTS: {
        if (g_nYesNo != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        EEPROM::restoreEEPROMSettingsFromConfiguration();
        EEPROM::storeDataIntoEEPROM(false);
        EEPROM::initializeAllOperatingModes();

        exitmenu();
        UI_STATUS(UI_TEXT_RESTORE_DEFAULTS);
        break;
    }

    case UI_ACTION_CHOOSE_LESSERINTEGRAL:
    case UI_ACTION_CHOOSE_CLASSICPID:
    case UI_ACTION_CHOOSE_NO:
    case UI_ACTION_CHOOSE_TYREUS_LYBEN: {
        if (g_nYesNo != 1) {
            // continue only in case the user has chosen "Yes"
            break;
        }
        unsigned char heater = menuPos[menuLevel - 2]; //0..1..2 mit zwei extrudern und bett. passt zum autotunesystem, weil UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT
        int method = menuPos[menuLevel - 1];           //0..1..2..3 passt nicht mehr zum J-Listing des M303

        //Esthetics: Switch method for better order in menu. I want the menu to start with pessen integral, because it is the most responsive one. We dont need some overshoot within menu because it has no known use for us.
        /*
                                             responsiveness of tunings: J1pessen++ J0classic+ J2someovershoot J3noovershoot- J4tyreuslyben--
                                             -- is for slow beds
                                             ++ is for fast hotends
                                             */
        if (method == 0) {
            method = 1; //J1
        } else if (method == 1) {
            method = 0; //J0
        } else if (method == 2) {
            method = 3; //J3
        } else if (method == 3) {
            method = 4; //J4
        }

        /*
        Line 1059: #define PRECISE_HEAT_BED_SCAN_BED_TEMP_PLA          60                                                                  // [°C]
        Line 1062: #define PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_PLA     230                                                                 // [°C]
        */
#if NUM_TEMPERATURE_LOOPS > 0
        int temperature = PRECISE_HEAT_BED_SCAN_EXTRUDER_TEMP_PLA;
        int cycles = 10;
        if (UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT - 1 == heater && UI_MENU_PID_BED_COUNT > 0) {
            //Das ist das Heizbett und kein Extruder
            temperature = PRECISE_HEAT_BED_SCAN_BED_TEMP_PLA; //Bett nicht so hoch testen, wie Extruder.
            cycles = 16;                                      //Bett ist erfahrungsgemäß viel träger. Mehr Zyklen erhöhen die Genauigkeit des Ergebnis.
        }
        bool writeeeprom = true;
        if (heater >= NUM_TEMPERATURE_LOOPS)
            heater = NUM_TEMPERATURE_LOOPS - 1;
        //show menu and message to user: He cant do anything until autotune is over.
        exitmenu();
        menuPos[0] = 3; //show temps
        tempController[heater]->autotunePID(temperature, heater, cycles, writeeeprom, method);
#else
        Com::printFLN(PSTR("PID Autotune Error: Noo Temperature-Loops defined!??"));
#endif // NUM_TEMPERATURE_LOOPS > 0
        break;
    }
    }
} // finishAction

void UIDisplay::changeSwitchCase(int action, int8_t increment) {
    switch (action) {
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    case UI_ACTION_FANSPEED: {
        int speed = (int)Printer::getFanSpeed() + (int)increment;
        if (speed > 255)
            speed = 0;
        if (speed < 0)
            speed = 255;
        Commands::setFanSpeed((uint8_t)speed);
        break;
    }
    case UI_ACTION_FAN_HZ: {
        if (increment > 0) {
            Commands::adjustFanFrequency((part_fan_pwm_speed == PART_FAN_MODE_MAX ? 1 : part_fan_pwm_speed + 1));
        } else {
            Commands::adjustFanFrequency((part_fan_pwm_speed == 1 ? PART_FAN_MODE_MAX : part_fan_pwm_speed - 1));
        }
        HAL::eprSetByte(EPR_RF_PART_FAN_SPEED, part_fan_pwm_speed);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_FAN_PART_FAN_PWM_MIN: {
        int temp = part_fan_pwm_min;
        INCREMENT_MIN_MAX(temp, 1, 1, 239);
        if (temp <= (int)part_fan_pwm_max - 16) {
            part_fan_pwm_min = temp;
        }
        //recalculate active pwm value out of fanSpeed for easy tune-in.
        //(Tune-In: set fan to 1% and rise minimum until it starts.)
        Commands::setFanSpeed(fanSpeed, true);
        HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MIN, part_fan_pwm_min);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_FAN_PART_FAN_PWM_MAX: {
        int temp = part_fan_pwm_max;
        INCREMENT_MIN_MAX(temp, 1, 16, 255);
        if (temp >= part_fan_pwm_min + 16) {
            part_fan_pwm_max = temp;
        }
        //recalculate active pwm value out of fanSpeed for easy tune-in.
        //(Tune-In: set fan to 100% and decrease maximum until fan slows down slightly.)
        Commands::setFanSpeed(fanSpeed, true);
        HAL::eprSetByte(EPR_RF_PART_FAN_PWM_MAX, part_fan_pwm_max);
        EEPROM::updateChecksum();
        break;
    }
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL
    case UI_ACTION_XPOSITION: {
        /*
        XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons.
        XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you stick to coordinates direction.
        see configuration.h
        */
        moveXAction(XYZ_POSITION_BUTTON_DIRECTION * increment);
        break;
    }
    case UI_ACTION_YPOSITION: {
        /*
        XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons.
        XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you stick to coordinates direction.
        see configuration.h
        */
        moveYAction(XYZ_POSITION_BUTTON_DIRECTION * increment);
        break;
    }
    case UI_ACTION_ZPOSITION: {
        /*
        XYZ_POSITION_BUTTON_DIRECTION = -1 : This fits to you if you want more intuitivity when choosing the Up-Down-Buttons.
        XYZ_POSITION_BUTTON_DIRECTION = 1 : This fits more if you stick to coordinates direction.
        see configuration.h
        */
        moveZAction(XYZ_POSITION_BUTTON_DIRECTION * increment);
        break;
    }
    case UI_ACTION_ZOFFSET: {
        INCREMENT_MIN_MAX(Printer::ZOffset, Z_OFFSET_MENU_STEPS, -(HEAT_BED_Z_COMPENSATION_MAX_MM * 1000), (HEAT_BED_Z_COMPENSATION_MAX_MM * 1000));
#if FEATURE_SENSIBLE_PRESSURE
        g_staticZSteps = ((Printer::ZOffset + g_nSensiblePressureOffset) * Printer::axisStepsPerMM[Z_AXIS]) / 1000;
#else
        g_staticZSteps = (Printer::ZOffset * Printer::axisStepsPerMM[Z_AXIS]) / 1000;
#endif
        HAL::eprSetInt32(EPR_RF_Z_OFFSET, Printer::ZOffset);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EPOSITION: {
        Printer::queueRelativeMMCoordinates(0, 0, 0, (float)increment / Printer::menuExtrusionFactor, UI_SET_EXTRUDER_FEEDRATE, true); //klappt nicht in Pause!!
        Commands::printCurrentPosition();
        break;
    }
    case UI_ACTION_HEATED_BED_TEMP: {
#if HAVE_HEATED_BED == true
        int tmp = (int)heatedBedController.targetTemperatureC;
        if (tmp < UI_SET_MIN_HEATED_BED_TEMP)
            tmp = 0;
        tmp += increment;
        if (tmp == 1)
            tmp = UI_SET_MIN_HEATED_BED_TEMP;
        if (tmp < UI_SET_MIN_HEATED_BED_TEMP)
            tmp = 0;
        else if (tmp > UI_SET_MAX_HEATED_BED_TEMP)
            tmp = UI_SET_MAX_HEATED_BED_TEMP;
        Extruder::setHeatedBedTemperature(tmp);
#endif // HAVE_HEATED_BED
        break;
    }
    case UI_ACTION_EXTRUDER0_TEMP: {
        int tmp = (int)extruder[0].tempControl.targetTemperatureC;
        if (tmp < UI_SET_MIN_EXTRUDER_TEMP)
            tmp = 0;
        tmp += increment;
        if (tmp == 1)
            tmp = UI_SET_MIN_EXTRUDER_TEMP;
        if (tmp < UI_SET_MIN_EXTRUDER_TEMP)
            tmp = 0;
        else if (tmp > UI_SET_MAX_EXTRUDER_TEMP)
            tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp, 0);
        break;
    }
    case UI_ACTION_EXTRUDER1_TEMP: {
#if NUM_EXTRUDER > 1
        int tmp = (int)extruder[1].tempControl.targetTemperatureC;
        tmp += increment;
        if (tmp == 1)
            tmp = UI_SET_MIN_EXTRUDER_TEMP;
        if (tmp < UI_SET_MIN_EXTRUDER_TEMP)
            tmp = 0;
        else if (tmp > UI_SET_MAX_EXTRUDER_TEMP)
            tmp = UI_SET_MAX_EXTRUDER_TEMP;
        Extruder::setTemperatureForExtruder(tmp, 1);
#endif // NUM_EXTRUDER>1
        break;
    }

#if NUM_EXTRUDER > 1
    case UI_ACTION_EXTRUDER_OFFSET_X: {
        float oldXOffset = extruder[1].offsetMM[X_AXIS];
        INCREMENT_MIN_MAX(extruder[1].offsetMM[X_AXIS], 0.25, 32, 36);

        if (Printer::isAxisHomed(X_AXIS) && Extruder::current->id == extruder[1].id) {
            // Shift the extruder-offset negatively to stay at the same point after switch
            int32_t dx = (extruder[1].offsetMM[X_AXIS] - oldXOffset) * Printer::axisStepsPerMM[X_AXIS];
            Printer::offsetRelativeStepsCoordinates(-dx, 0, 0, 0);
        }

        HAL::eprSetFloat(EEPROM::getExtruderOffset(1) + EPR_EXTRUDER_X_OFFSET, extruder[1].offsetMM[X_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EXTRUDER_OFFSET_Y: {
        float oldYOffset = extruder[1].offsetMM[Y_AXIS];
        INCREMENT_MIN_MAX(extruder[1].offsetMM[Y_AXIS], 0.25, -2, 2);

        if (Printer::isAxisHomed(Y_AXIS) && Extruder::current->id == extruder[1].id) {
            int32_t dy = (extruder[1].offsetMM[Y_AXIS] - oldYOffset) * Printer::axisStepsPerMM[Y_AXIS];
            // Shift the extruder-offset negatively to stay at the same point after switch
            Printer::offsetRelativeStepsCoordinates(0, -dy, 0, 0);
        }

        HAL::eprSetFloat(EEPROM::getExtruderOffset(1) + EPR_EXTRUDER_Y_OFFSET, extruder[1].offsetMM[Y_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EXTRUDER_OFFSET_Z: {
        float oldZOffset = extruder[1].offsetMM[Z_AXIS];
        //Das hier ist nur dazu gedacht, um eine Tip-Down-Nozzle auf per ToolChange auf die Korrekte Höhe zu justieren.
        INCREMENT_MIN_MAX(extruder[1].offsetMM[Z_AXIS], 0.025, -2, 0);

        if (Printer::isAxisHomed(Z_AXIS) && Extruder::current->id == extruder[1].id) {
            int32_t dz = (extruder[1].offsetMM[Z_AXIS] - oldZOffset) * Printer::axisStepsPerMM[Z_AXIS];
            // Shift the extruder-offset negatively to stay at the same point after switch
            Printer::offsetRelativeStepsCoordinates(0, 0, -dz, 0);
        }

        HAL::eprSetFloat(EEPROM::getExtruderOffset(1) + EPR_EXTRUDER_Z_OFFSET, extruder[1].offsetMM[Z_AXIS]); //mm negativ
        EEPROM::updateChecksum();
        break;
    }
#endif // NUM_EXTRUDER>1

    case UI_ACTION_FEEDRATE_MULTIPLY: {
        int fr = Printer::feedrateMultiply;
        INCREMENT_MIN_MAX(fr, 1, 25, 500);
        Commands::changeFeedrateMultiply(fr);
        break;
    }
    case UI_ACTION_FLOWRATE_MULTIPLY: {
        float eF = Printer::menuExtrusionFactor;
        INCREMENT_MIN_MAX(eF, 0.01f, 0.25f, 2.0f);
        Commands::changeFlowrateMultiply(eF);
        break;
    }
    case UI_ACTION_STEPPER_INACTIVE: {
        stepperInactiveTime -= stepperInactiveTime % 1000;
        INCREMENT_MAX(stepperInactiveTime, 60000UL, 10080000UL);

        HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME, stepperInactiveTime);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_INACTIVE: {
        maxInactiveTime -= maxInactiveTime % 1000;
        INCREMENT_MAX(maxInactiveTime, 60000UL, 10080000UL);

        HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME, maxInactiveTime);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_PRINT_ACCEL_X: {
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[X_AXIS], ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_X_MAX_ACCEL, Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_PRINT_ACCEL_Y: {
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[Y_AXIS], ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Y_MAX_ACCEL, Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_PRINT_ACCEL_Z: {
        INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[Z_AXIS], ACCELERATION_MENU_CHANGE_Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Z_MAX_ACCEL, Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MOVE_ACCEL_X: {
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS], ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL, Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MOVE_ACCEL_Y: {
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS], ACCELERATION_MENU_CHANGE_XY, ACCELERATION_MIN_XY, ACCELERATION_MAX_XY);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL, Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MOVE_ACCEL_Z: {
        INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS], ACCELERATION_MENU_CHANGE_Z, ACCELERATION_MIN_Z, ACCELERATION_MAX_Z);
        Printer::updateDerivedParameter();

        HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL, Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_JERK: {
        INCREMENT_MIN_MAX(Printer::maxXYJerk, 0.1f, 1.0f, 33.3f); //RFx000: Limit 33.3 sind willkürlich grob faktor 3 von normalwert.

        HAL::eprSetFloat(EPR_MAX_XYJERK, Printer::maxXYJerk);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_ZJERK: {
        INCREMENT_MIN_MAX(Printer::maxZJerk, 0.05f, 0.05f, 2.0f); //RFx000: Limit 2 sind Max XY-Jerk / Stepratenverhältnis XY->Z von ~16.8 gibt etwas unter 2.

        HAL::eprSetFloat(EPR_MAX_ZJERK, Printer::maxZJerk);
        EEPROM::updateChecksum();
        break;
    }
#if FEATURE_READ_CALIPER
    case UI_ACTION_CAL_STANDARD: {
        if (caliper_filament_standard >= 2600) {
            INCREMENT_MIN_MAX(caliper_filament_standard, 10, 2590, 3000);
            if (caliper_filament_standard == 2590) {
                caliper_filament_standard = 1750;
            }
        } else {
            INCREMENT_MIN_MAX(caliper_filament_standard, 10, 1600, 1910);
            if (caliper_filament_standard == 1910) {
                caliper_filament_standard = 2850;
            }
        }

        HAL::eprSetInt16(EPR_RF_CAL_STANDARD, caliper_filament_standard);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_CAL_CORRECT: {
        INCREMENT_MIN_MAX(caliper_collect_adjust, 1, -127, 127);
        HAL::eprSetByte(EPR_RF_CAL_ADJUST, caliper_collect_adjust);
        EEPROM::updateChecksum();
        break;
    }
#endif //FEATURE_READ_CALIPER
    case UI_ACTION_MILL_ACCELERATION: {
#if FEATURE_MILLING_MODE
        if (!Printer::isPrinting()) {
            INCREMENT_MIN_MAX(Printer::max_milling_all_axis_acceleration, 2, 1, 200);
            Printer::updateDerivedParameter();
            HAL::eprSetInt16(EPR_RF_MILL_ACCELERATION, Printer::max_milling_all_axis_acceleration);
            EEPROM::updateChecksum();
        }
#endif // FEATURE_MILLING_MODE
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_X: {
        INCREMENT_MIN_MAX(Printer::homingFeedrate[X_AXIS], 5, 5, MAX_FEEDRATE_X);

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
            HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_PRINT, Printer::homingFeedrate[X_AXIS]);
#if FEATURE_MILLING_MODE
        } else {
            HAL::eprSetFloat(EPR_X_HOMING_FEEDRATE_MILL, Printer::homingFeedrate[X_AXIS]);
        }
#endif // FEATURE_MILLING_MODE
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_Y: {
        INCREMENT_MIN_MAX(Printer::homingFeedrate[Y_AXIS], 5, 5, MAX_FEEDRATE_Y);

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
            HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_PRINT, Printer::homingFeedrate[Y_AXIS]);
#if FEATURE_MILLING_MODE
        } else {
            HAL::eprSetFloat(EPR_Y_HOMING_FEEDRATE_MILL, Printer::homingFeedrate[Y_AXIS]);
        }
#endif // FEATURE_MILLING_MODE
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_HOMING_FEEDRATE_Z: {
        INCREMENT_MIN_MAX(Printer::homingFeedrate[Z_AXIS], 1, 1, MAX_FEEDRATE_Z);

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
            HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_PRINT, Printer::homingFeedrate[Z_AXIS]);
#if FEATURE_MILLING_MODE
        } else {
            HAL::eprSetFloat(EPR_Z_HOMING_FEEDRATE_MILL, Printer::homingFeedrate[Z_AXIS]);
        }
#endif // FEATURE_MILLING_MODE
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_X: {
        INCREMENT_MIN_MAX(Printer::maxFeedrate[X_AXIS], 5, 1, MAX_FEEDRATE_X);

        HAL::eprSetFloat(EPR_X_MAX_FEEDRATE, Printer::maxFeedrate[X_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_Y: {
        INCREMENT_MIN_MAX(Printer::maxFeedrate[Y_AXIS], 5, 1, MAX_FEEDRATE_Y);

        HAL::eprSetFloat(EPR_Y_MAX_FEEDRATE, Printer::maxFeedrate[Y_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_MAX_FEEDRATE_Z: {
        INCREMENT_MIN_MAX(Printer::maxFeedrate[Z_AXIS], 1, 1, MAX_FEEDRATE_Z);

        HAL::eprSetFloat(EPR_Z_MAX_FEEDRATE, Printer::maxFeedrate[Z_AXIS]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_BAUDRATE: {
#if EEPROM_MODE != 0
        unsigned char p = 0;
        int32_t rate;
        do {
            rate = pgm_read_dword(&(baudrates[p]));
            if (rate == baudrate)
                break;
            p++;

        } while (rate != 0);

        if (rate == 0 && p >= 2)
            p -= 2;
        else if (rate == 0 && p == 1)
            p = 0;

        if (p + increment >= 0)
            p += increment;
        else if (p + increment <= 0)
            p = 0;

        //if(p<0) p = 0;
        rate = pgm_read_dword(&(baudrates[p]));
        if (rate == 0 && p >= 1)
            p--;
        baudrate = pgm_read_dword(&(baudrates[p]));

        HAL::eprSetInt32(EPR_BAUDRATE, baudrate);
        EEPROM::updateChecksum();
#endif // EEPROM_MODE!=0
        break;
    }

#if FEATURE_RGB_LIGHT_EFFECTS
    case UI_ACTION_RGB_LIGHT_MODE: {
        INCREMENT_MIN_MAX(Printer::RGBLightMode, 1, 0, 3);

        HAL::eprSetByte(EPR_RF_RGB_LIGHT_MODE, Printer::RGBLightMode);
        EEPROM::updateChecksum();

        switch (Printer::RGBLightMode) {
        case RGB_MODE_OFF: {
            Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
            Printer::RGBLightModeForceWhite = 0;
            setRGBTargetColors(0, 0, 0);
            break;
        }
        case RGB_MODE_WHITE: {
            Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
            Printer::RGBLightModeForceWhite = 0;
            setRGBTargetColors(255, 255, 255);
            break;
        }
        case RGB_MODE_AUTOMATIC: {
            // the firmware will determine the current RGB colors automatically
            Printer::RGBLightStatus = RGB_STATUS_AUTOMATIC;
            Printer::RGBLightModeForceWhite = 0;
            break;
        }
        case RGB_MODE_MANUAL: {
            Printer::RGBLightStatus = RGB_STATUS_NOT_AUTOMATIC;
            Printer::RGBLightModeForceWhite = 0;
            setRGBTargetColors(g_uRGBManualR, g_uRGBManualG, g_uRGBManualB);
            break;
        }
        }
        break;
    }
#endif // FEATURE_RGB_LIGHT_EFFECTS

    case UI_ACTION_EXTR_STEPS_E0:
    case UI_ACTION_EXTR_STEPS_E1: {
        if (!Printer::isMenuMode(MENU_MODE_PAUSED) && !Printer::isPrinting()) {
#if NUM_EXTRUDER > 1
            uint8_t eNr = (UI_ACTION_EXTR_STEPS_E1 == action) ? 1 : 0;
#else
            uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
            INCREMENT_MIN_MAX(extruder[eNr].stepsPerMM, 1, 1, 9999);
            if (eNr == Extruder::current->id)
                Extruder::selectExtruderById(eNr); //übernehmen der werte

            HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_STEPS_PER_MM, extruder[eNr].stepsPerMM);
            EEPROM::updateChecksum();
        }
        break;
    }

#if USE_ADVANCE
    case UI_ACTION_ADVANCE_L_E0:
    case UI_ACTION_ADVANCE_L_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_ADVANCE_L_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        float step = (extruder[eNr].advanceL < 30.0f) ? 1.0f : ((extruder[eNr].advanceL < 50.0f) ? 5.0f : 10.0f);
        INCREMENT_MIN_MAX(extruder[eNr].advanceL, step, 0.0f, 250.0f);
        // advance < 20 might be problematic
        // https://github.com/repetier/Repetier-Firmware/issues/837#issuecomment-455852008
        if (extruder[eNr].advanceL < 20.0f) {
            if (increment > 0) {
                extruder[eNr].advanceL = 20;
            } else {
                extruder[eNr].advanceL = 0;
            }
        }
        Printer::updateAdvanceActivated();

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_ADVANCE_L, extruder[eNr].advanceL);
        EEPROM::updateChecksum();
        break;
    }
#endif //USE_ADVANCE

    case UI_ACTION_EXTR_ACCELERATION_E0:
    case UI_ACTION_EXTR_ACCELERATION_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_EXTR_ACCELERATION_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        INCREMENT_MIN_MAX(extruder[eNr].maxAcceleration, 200, 200, 10000);
        if (eNr == Extruder::current->id)
            Extruder::selectExtruderById(eNr);

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_MAX_ACCELERATION, extruder[eNr].maxAcceleration);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_EXTR_MAX_FEEDRATE_E0:
    case UI_ACTION_EXTR_MAX_FEEDRATE_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_EXTR_MAX_FEEDRATE_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        INCREMENT_MIN_MAX(extruder[eNr].maxFeedrate, 1, extruder[eNr].maxEJerk, 60);
        if (eNr == Extruder::current->id)
            Extruder::selectExtruderById(eNr);

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_MAX_FEEDRATE, extruder[eNr].maxFeedrate);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_EXTR_START_FEEDRATE_E0:
    case UI_ACTION_EXTR_START_FEEDRATE_E1: {
#if NUM_EXTRUDER > 1
        uint8_t eNr = (UI_ACTION_EXTR_START_FEEDRATE_E1 == action) ? 1 : 0;
#else
        uint8_t eNr = 0;
#endif //NUM_EXTRUDER > 1
        INCREMENT_MIN_MAX(extruder[eNr].maxEJerk, 1, 1, extruder[eNr].maxFeedrate);
        if (eNr == Extruder::current->id)
            Extruder::selectExtruderById(eNr);

        HAL::eprSetFloat(EEPROM::getExtruderOffset(eNr) + EPR_EXTRUDER_MAX_START_FEEDRATE, extruder[eNr].maxEJerk);
        EEPROM::updateChecksum();
        break;
    }

#if FEATURE_WORK_PART_Z_COMPENSATION
    case UI_ACTION_RF_SET_Z_MATRIX_WORK_PART: {
        if (Printer::doWorkPartZCompensation) {
            // do not allow to change the current work part z-compensation matrix while the z-compensation is active
            showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
            break;
        }

        INCREMENT_MIN_MAX(g_nActiveWorkPart, 1, 1, EEPROM_MAX_WORK_PART_SECTORS);
        switchActiveWorkPart(g_nActiveWorkPart);
        break;
    }
    case UI_ACTION_RF_SET_SCAN_DELTA_X: {
        INCREMENT_MIN_MAX(g_nScanXStepSizeMM, 1, WORK_PART_SCAN_X_STEP_SIZE_MIN_MM, 100);
        g_nScanXStepSizeSteps = (long)((float)g_nScanXStepSizeMM * Printer::axisStepsPerMM[X_AXIS]);
        break;
    }
    case UI_ACTION_RF_SET_SCAN_DELTA_Y: {
        INCREMENT_MIN_MAX(g_nScanYStepSizeMM, 1, WORK_PART_SCAN_Y_STEP_SIZE_MIN_MM, 100);
        g_nScanYStepSizeSteps = (long)((float)g_nScanYStepSizeMM * Printer::axisStepsPerMM[Y_AXIS]);
        break;
    }
#endif // FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_HEAT_BED_Z_COMPENSATION
    case UI_ACTION_RF_SET_Z_MATRIX_HEAT_BED: {
        if (Printer::doHeatBedZCompensation) {
            // do not allow to change the current heat bed z-compensation matrix while the z-compensation is active
            showError((void*)ui_text_z_compensation, (void*)ui_text_operation_denied);
            break;
        }

        INCREMENT_MIN_MAX(g_nActiveHeatBed, 1, 1, EEPROM_MAX_HEAT_BED_SECTORS);
        switchActiveHeatBed(g_nActiveHeatBed);
        break;
    }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION
    case UI_ACTION_RF_SCAN_START_HEIGHT: {
        INCREMENT_MIN_MAX(g_scanStartZLiftMM, 0.1f, 0.3f, 6.0f);
        HAL::eprSetFloat(EPR_ZSCAN_START_MM, g_scanStartZLiftMM); //mm zlift vor den scans.
        EEPROM::updateChecksum();
        break;
    }
#endif // FEATURE_WORK_PART_Z_COMPENSATION || FEATURE_HEAT_BED_Z_COMPENSATION

    case UI_ACTION_RF_RESET_ACK:
    case UI_ACTION_STOP_ACK:
    case UI_ACTION_RESTORE_DEFAULTS:
    case UI_ACTION_CHOOSE_LESSERINTEGRAL:
    case UI_ACTION_CHOOSE_CLASSICPID:
    case UI_ACTION_CHOOSE_NO:
    case UI_ACTION_CHOOSE_TYREUS_LYBEN: {
        INCREMENT_MIN_MAX(g_nYesNo, 1, 0, 1);
        break;
    }

#if FEATURE_EMERGENCY_PAUSE
    case UI_ACTION_EMERGENCY_PAUSE_MIN: {
        INCREMENT_MIN_MAX(g_nEmergencyPauseDigitsMin, 200, EMERGENCY_PAUSE_DIGITS_MIN, g_nEmergencyPauseDigitsMax);
        HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMIN, g_nEmergencyPauseDigitsMin);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EMERGENCY_PAUSE_MAX: {
        INCREMENT_MIN_MAX(g_nEmergencyPauseDigitsMax, 200, g_nEmergencyPauseDigitsMin, EMERGENCY_PAUSE_DIGITS_MAX);
        HAL::eprSetInt32(EPR_RF_EMERGENCYPAUSEDIGITSMAX, g_nEmergencyPauseDigitsMax);
        EEPROM::updateChecksum();
        break;
    }
#endif //FEATURE_EMERGENCY_PAUSE

#if FEATURE_EMERGENCY_STOP_Z_AND_E
    case UI_ACTION_EMERGENCY_ZSTOP_MIN: {
        INCREMENT_MIN_MAX(g_nEmergencyStopZAndEMin, 200, EMERGENCY_STOP_DIGITS_MIN, g_nEmergencyStopZAndEMax);
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMIN, g_nEmergencyStopZAndEMin);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_EMERGENCY_ZSTOP_MAX: {
        INCREMENT_MIN_MAX(g_nEmergencyStopZAndEMax, 200, g_nEmergencyStopZAndEMin, EMERGENCY_STOP_DIGITS_MAX);
        HAL::eprSetInt16(EPR_RF_EMERGENCYZSTOPDIGITSMAX, g_nEmergencyStopZAndEMax);
        EEPROM::updateChecksum();
        break;
    }
#endif //FEATURE_EMERGENCY_STOP_Z_AND_E

#if FEATURE_SENSIBLE_PRESSURE
    case UI_ACTION_SENSEOFFSET_DIGITS: {
        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS);
        INCREMENT_MIN_MAX(oldval, 100, 500, EMERGENCY_PAUSE_DIGITS_MAX);
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_DIGITS, oldval);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_SENSEOFFSET_MAX: {
        short oldval = HAL::eprGetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX);
        INCREMENT_MIN_MAX(oldval, 10, 10, 300);
        HAL::eprSetInt16(EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX, oldval);
        EEPROM::updateChecksum();
        break;
    }
#endif //FEATURE_SENSIBLE_PRESSURE

#if FEATURE_Kurt67_WOBBLE_FIX
    //Antibauchtanz:
    case UI_ACTION_WOBBLE_FIX_PHASEXY: {
        INCREMENT_MIN_MAX(Printer::wobblePhaseXY, 1, -100, 101);
        if (Printer::wobblePhaseXY == 101) {
            Printer::wobblePhaseXY = -99;
        }
        if (Printer::wobblePhaseXY == -100) {
            Printer::wobblePhaseXY = 100;
        }
        HAL::eprSetByte(EPR_RF_MOD_WOBBLE_FIX_PHASEXY, Printer::wobblePhaseXY);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPX: {
        INCREMENT_MIN_MAX(Printer::wobbleAmplitudes[0], 5, -995, 995);
        HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPX, Printer::wobbleAmplitudes[0]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPY1: {
        INCREMENT_MIN_MAX(Printer::wobbleAmplitudes[1], 5, -995, 995);
        HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY1, Printer::wobbleAmplitudes[1]);
        EEPROM::updateChecksum();
        break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPY2: {
        INCREMENT_MIN_MAX(Printer::wobbleAmplitudes[2], 5, -995, 995);
        HAL::eprSetInt16(EPR_RF_MOD_WOBBLE_FIX_AMPY2, Printer::wobbleAmplitudes[2]);
        EEPROM::updateChecksum();
        break;
    }
    //Antikippeln:
    /*
    case UI_ACTION_WOBBLE_FIX_PHASEZ:
    {
    INCREMENT_MIN_MAX(Printer::wobblePhaseZ,1,-100,100);
    break;
    }
    case UI_ACTION_WOBBLE_FIX_AMPZ:
    {
    INCREMENT_MIN_MAX(Printer::wobbleAmplitudes[3],5,-995,995);
    break;
    }*/
#endif //FEATURE_Kurt67_WOBBLE_FIX

    case UI_ACTION_CHOOSE_DMIN: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; //0..1..2 mit zwei extrudern und bett. passt zum autotunesystem, weil UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidDriveMin;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidDriveMin = drive;
                if (UI_MENU_PID_BED_COUNT > 0 && UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT - 1 == heater) {
                    //Das ist das Heizbett
                    HAL::eprSetByte(EPR_BED_DRIVE_MIN, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_DRIVE_MIN, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_DMAX: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; //0..1..2 mit zwei extrudern und bett. passt zum autotunesystem, weil UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidDriveMax;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidDriveMax = drive;
                if (UI_MENU_PID_BED_COUNT > 0 && UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT - 1 == heater) {
                    //Das ist das Heizbett
                    HAL::eprSetByte(EPR_BED_DRIVE_MAX, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_DRIVE_MAX, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_PIDMAX: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; //0..1..2 mit zwei extrudern und bett. passt zum autotunesystem, weil UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->pidMax;
                INCREMENT_MIN_MAX(drive, 1, 1, 255);
                tempController[heater]->pidMax = drive;
                if (UI_MENU_PID_BED_COUNT > 0 && UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT - 1 == heater) {
                    //Das ist das Heizbett
                    HAL::eprSetByte(EPR_BED_PID_MAX, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_PID_MAX, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_SENSOR: {
        if (menuLevel == 4) {                        //identifikation des temperaturzyklus anhand der position im menü. Das ist nicht 100% sauber, aber funktioniert.
            uint8_t heater = menuPos[menuLevel - 1]; //0..1..2 mit zwei extrudern und bett. passt zum autotunesystem, weil UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT
            if (heater < NUM_TEMPERATURE_LOOPS) {
                int drive = tempController[heater]->sensorType;
                if (UI_MENU_PID_BED_COUNT > 0 && UI_MENU_PID_EXT0_COUNT + UI_MENU_PID_EXT1_COUNT + UI_MENU_PID_BED_COUNT - 1 == heater) {
                    //Das ist das Heizbett
                    switch (drive) {
                    case 3: {
                        drive = 4;
                        break;
                    } //4 ist der 10K Thermistor vom hersteller der matten (? rf1k_mhj11)
                    default: {
                        drive = 3;
                        break;
                    }
                    }
                    tempController[heater]->sensorType = drive;
                    HAL::eprSetByte(EPR_RF_HEATED_BED_SENSOR_TYPE, (uint8_t)drive);
                    EEPROM::updateChecksum();
                } else {
                    //Extruder
                    if (increment > 0) {
                        switch (drive) {
                        case 3: {
                            drive = 8;
                            break;
                        }
                        case 8: {
                            drive = 13;
                            break;
                        } //add more sensors for menu-tweaking here, those are the most common for RFx000
                        case 13: {
                            drive = 1;
                            break;
                        }
                        default: {
                            drive = 3;
                            break;
                        }
                        }
                    } else { //== 0 gibts nicht, soweit ich weiß
                        switch (drive) {
                        case 3: {
                            drive = 1;
                            break;
                        }
                        case 1: {
                            drive = 13;
                            break;
                        }
                        case 13: {
                            drive = 8;
                            break;
                        } //add more sensors for menu-tweaking here, those are the most common for RFx000
                        default: {
                            drive = 3;
                            break;
                        }
                        }
                    }
                    tempController[heater]->sensorType = drive;
                    if (heater <= NUM_EXTRUDER - 1) { //paranoid doublecheck
                        HAL::eprSetByte(EEPROM::getExtruderOffset(heater) + EPR_EXTRUDER_SENSOR_TYPE, (uint8_t)drive);
                        EEPROM::updateChecksum();
                    }
                }
            }
        }
        break;
    }
    case UI_ACTION_CHOOSE_MOTOR_X:
    case UI_ACTION_CHOOSE_MOTOR_Y:
    case UI_ACTION_CHOOSE_MOTOR_Z:
    case UI_ACTION_CHOOSE_MOTOR_E0:
    case UI_ACTION_CHOOSE_MOTOR_E1: {
        uint8_t steppernr = menuPos[menuLevel];
        if (steppernr == 6)
            steppernr = 4;   //das ist etwas stümperhaft, aber ich brauche die Zeilennummer um auszuwählen und die Einstellungen für die Extruderstepper gehören drüber, also muss Extruder 2 zwei Zeilen runter...
        if (steppernr < 5) { // aktuell gibts nur 5
            int drive = Printer::motorCurrent[steppernr];
            const short uMotorCurrentMax[] = MOTOR_CURRENT_MAX;
            INCREMENT_MIN_MAX(drive, 1, MOTOR_CURRENT_MIN + 1, uMotorCurrentMax[steppernr]); //von 40 bis maximal das was in der config steht.
            Printer::motorCurrent[steppernr] = drive;
            HAL::eprSetByte(EPR_RF_MOTOR_CURRENT + steppernr, (uint8_t)drive);
            EEPROM::updateChecksum();
            setMotorCurrent(steppernr + 1, drive);
            Com::printF(PSTR("Stepper"), steppernr + 1);
            Com::printFLN(PSTR(" = "), drive);
        }
        break;
    }
#if FEATURE_DIGIT_FLOW_COMPENSATION
    case UI_ACTION_FLOW_MIN: {
        INCREMENT_MIN_MAX(g_nDigitFlowCompensation_Fmin, 200, 0, g_nDigitFlowCompensation_Fmax);
        break;
    }
    case UI_ACTION_FLOW_MAX: {
        short max = (g_nEmergencyPauseDigitsMax ? abs(g_nEmergencyPauseDigitsMax) : abs(EMERGENCY_PAUSE_DIGITS_MAX));
        INCREMENT_MIN_MAX(g_nDigitFlowCompensation_Fmax, 200, g_nDigitFlowCompensation_Fmin, max);
        break;
    }
    case UI_ACTION_FLOW_DF: {
        INCREMENT_MIN_MAX(g_nDigitFlowCompensation_intense, 1, -99, 99);
        //flowmulti wird zur laufzeit geändert, anhand von steigung intense -> aber je nach cache erst sehr spät.
        break;
    }
    case UI_ACTION_FLOW_DV: {
        INCREMENT_MIN_MAX(g_nDigitFlowCompensation_speed_intense, 1, -90, 99);
        //feedmulti wird zur laufzeit geändert, anhand von steigung intense
        break;
    }
#endif //FEATURE_DIGIT_FLOW_COMPENSATION
    case UI_ACTION_SHIFT_INTERVAL: {
        INCREMENT_MIN_MAX(Printer::stepsPackingMinInterval, -100, MIN_STEP_PACKING_MIN_INTERVAL, MAX_STEP_PACKING_MIN_INTERVAL);
        HAL::eprSetInt16(EPR_RF_STEP_PACKING_MIN_INTERVAL, Printer::stepsPackingMinInterval);
        EEPROM::updateChecksum();
        break;
    }
#if FEATURE_ADJUSTABLE_MICROSTEPS
    case UI_ACTION_MICROSTEPS_XY:
    case UI_ACTION_MICROSTEPS_Z:
    case UI_ACTION_MICROSTEPS_E: {
        if (!Printer::isPrinting() && !PrintLine::linesCount && g_pauseStatus == PAUSE_STATUS_NONE) {
            Printer::disableAllSteppersNow(); //Stepper und Homing ausmachen.
                                              //We cannot use the old coordinates anymore.

            InterruptProtectedBlock noInts;
            bool changed[5] = { false, false, false, false, false };
            switch (action) {
            case UI_ACTION_MICROSTEPS_XY: {
                INCREMENT_MIN_MAX(Printer::motorMicroStepsModeValue[0], 1, 4, 6);
                if (Printer::motorMicroStepsModeValue[1] != Printer::motorMicroStepsModeValue[0]) { //only adjust, when changed.
                    Printer::motorMicroStepsModeValue[1] = Printer::motorMicroStepsModeValue[0];    //sync x and y
                    drv8711adjustMicroSteps(1);                                                     //adjust driver chip X=1
                    drv8711adjustMicroSteps(2);                                                     //adjust driver chip Y=2
                    changed[0] = changed[1] = true;
                }
                break;
            }
            case UI_ACTION_MICROSTEPS_Z: {
                uint8_t temp = Printer::motorMicroStepsModeValue[2];
                INCREMENT_MIN_MAX(Printer::motorMicroStepsModeValue[2], 1, 4, 5);
                if (temp != Printer::motorMicroStepsModeValue[2]) { //only adjust, when changed.
                    drv8711adjustMicroSteps(3);                     //adjust driver chip Z=3
                    changed[2] = true;
                }
                break;
            }
            case UI_ACTION_MICROSTEPS_E: {
                INCREMENT_MIN_MAX(Printer::motorMicroStepsModeValue[3], 1, 4, 7);
                if (Printer::motorMicroStepsModeValue[4] != Printer::motorMicroStepsModeValue[3]) { //only adjust, when changed.
                    Printer::motorMicroStepsModeValue[4] = Printer::motorMicroStepsModeValue[3];    //sync E0 and E1
                    drv8711adjustMicroSteps(4);                                                     //adjust driver chip E0=4
                    drv8711adjustMicroSteps(5);                                                     //adjust driver chip E1=5
                    changed[3] = changed[4] = true;
                }
                break;
            }
            }

            bool updateall = false;
            if (HAL::eprGetByte(EPR_RF_MICRO_STEPS_USED) != 0xAB)
                updateall = true;
            float stepsmm_korrekturfactor = (increment > 0 ? 2.0f : 0.5f);
            bool updatederived = false;
            bool updateextruder = false;

            //anpassen der eeprom-werte und anpassen der steps/mm sodass die geschwindigkeit weiterhin passt.
            for (int i = 0; i < DRV8711_NUM_CHANNELS; i++) {
                if ((!changed[i] && updateall) || changed[i]) { //erstes oder veränderndes schreiben
                    HAL::eprSetByte(EPR_RF_MICRO_STEPS_X + i, Printer::motorMicroStepsModeValue[i]);
                }
                if (changed[i]) { //nur dann die axis-steps anpassen, wenn wirklich was geändert wurde.
                    switch (i) {
                    case X_AXIS:
                    case Y_AXIS:
                    case Z_AXIS: {
                        Printer::axisStepsPerMM[i] *= stepsmm_korrekturfactor;
                        g_nPauseSteps[i] *= stepsmm_korrekturfactor;
                        if (i == Z_AXIS) {
                            g_staticZSteps *= stepsmm_korrekturfactor;          //adjust static offset from Z-Offset to fit new Microstepping
                            Printer::currentZSteps *= stepsmm_korrekturfactor;  //adjust critical z-counter for drive over switch limits
                            g_maxZCompensationSteps *= stepsmm_korrekturfactor; //preadjust max compensation steps for z-CMP (gets autoadjusted but the user might override autoadjustement)
                            g_minZCompensationSteps *= stepsmm_korrekturfactor; //preadjust max compensation steps for z-CMP (gets autoadjusted but the user might override autoadjustement)
                            g_nManualSteps[Z_AXIS] *= stepsmm_korrekturfactor;
                            HAL::eprSetInt16(EPR_RF_MOD_Z_STEP_SIZE, g_nManualSteps[Z_AXIS]);
                            g_ZCompensationMatrix[0][0] = EEPROM_FORMAT - 1; //force the zmatrix in ram to be invalid and to reload it later.
                                                                             //korrektur der aktiven kompensation/zoffset ist durch fehlendes homing unterbunden.
                        }
                        updatederived = true; //übernehmen der werte in offsets und infaxissteps, accel usw..
                        HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM + 4 * i, Printer::axisStepsPerMM[i]);
                        break;
                    }
                    case E_AXIS:
                    case E_AXIS + 1: {
                        //i-3 ist hier 0 oder 1
                        uint8_t etr = i - 3; //3-3 =0 oder 4-1 =1
                        extruder[etr].stepsPerMM *= stepsmm_korrekturfactor;
                        HAL::eprSetFloat(EEPROM::getExtruderOffset(etr) + EPR_EXTRUDER_STEPS_PER_MM, extruder[etr].stepsPerMM);
                        if (etr == Extruder::current->id)
                            updateextruder = true; //übernehmen der werte in offsets und infaxissteps, accel usw..
                        break;
                    }
                    }
                }
            }
            //übernehmen der werte in offsets und infaxissteps, accel usw..
            if (updatederived)
                Printer::updateDerivedParameter();
            if (updateextruder)
                Extruder::selectExtruderById(Extruder::current->id);
            if (updateall)
                HAL::eprSetByte(EPR_RF_MICRO_STEPS_USED, 0xAB); //erstes schreiben markiert eepromwerte als gültig
            EEPROM::updateChecksum();
            noInts.unprotect();
        }
        break;
    }
#endif //FEATURE_ADJUSTABLE_MICROSTEPS
    }
} //changeSwitchCase

void UIDisplay::mainSwitchCase(int action) {
    bool skipBeep = false;
    switch (action) {
        /*
        * HARDWARE BUTTONS
        */
    case UI_ACTION_OK: {
        okAction();
        skipBeep = true; // Prevent double beep
        g_nYesNo = 0;
        break;
    }
    case UI_ACTION_BACK: {
        backAction();
        break;
    }
    case UI_ACTION_NEXT: {
        nextPreviousAction(1);
        break;
    }
    case UI_ACTION_RIGHT: {
        rightAction();
        break;
    }
    case UI_ACTION_PREVIOUS: {
        nextPreviousAction(-1);
        break;
    }
    case UI_ACTION_RF_HEAT_BED_UP: {
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps

        //DO NOT MOVE Z: ALTER Z-OFFSET
        if (uid.menuLevel == 0 && uid.menuPos[0] == 1) {
            //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
            zOffsetMinusAction();
            beep(1, 4);
        } //ELSE DO MOVE Z:
        else {
            moveZAction(-1);
        }
        break;
    }
    case UI_ACTION_RF_HEAT_BED_DOWN: {
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds();   //prevent inactive shutdown of steppers/temps
                                                         //DO NOT MOVE Z: ALTER Z-OFFSET
        if (uid.menuLevel == 0 && uid.menuPos[0] == 1) { //wenn im Mod-Menü für Z-Offset/Matrix Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
            zOffsetPlusAction();
            beep(1, 4);
        } //ELSE DO MOVE Z:
        else {
            moveZAction(1);
        }
        break;
    }
    case UI_ACTION_RF_EXTRUDER_OUTPUT: {
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_MILL) {
            if (Printer::feedrate < RMath::min(Printer::maxFeedrate[X_AXIS], Printer::maxFeedrate[Y_AXIS]) - 1)
                Printer::feedrate++;
        } else
#endif // FEATURE_MILLING_MODE
        {
            if (uid.menuLevel == 0 && uid.menuPos[0] == 1) { //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
                                                             //we are in the Mod menu
                                                             //so dont retract, change the speed of the print to a lower speed instead of retracting:
                                                             //limits handled by change-function!
                Commands::changeFeedrateMultiply(Printer::feedrateMultiply + 1);
                beep(1, 4);
            } else {
                Printer::offsetRelativeStepsCoordinates(0, 0, 0, int32_t(g_nManualSteps[E_AXIS]));

                //In case of double pause and in case we tempered with the retract, we dont want to drive the E-Axis back to some old location - that much likely causes emergency block.
                g_nContinueSteps[E_AXIS] = 0;
            }
        }
        break;
    }
    case UI_ACTION_RF_EXTRUDER_RETRACT: {
        // show that we are active
        previousMillisCmd = HAL::timeInMilliseconds(); //prevent inactive shutdown of steppers/temps

#if FEATURE_MILLING_MODE
        if (Printer::operatingMode == OPERATING_MODE_MILL) {
            if (Printer::feedrate > 1)
                Printer::feedrate--;
        } else
#endif // FEATURE_MILLING_MODE
        {
            if (uid.menuLevel == 0 && uid.menuPos[0] == 1) { //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
                                                             //we are in the Mod menu
                                                             //so dont retract, change the speed of the print to a lower speed instead of retracting:
                                                             //limits handled by change-function!
                Commands::changeFeedrateMultiply(Printer::feedrateMultiply - 1);
                beep(1, 4);
            } else {
                Printer::offsetRelativeStepsCoordinates(0, 0, 0, -1 * int32_t(g_nManualSteps[E_AXIS]));

                //In case of double pause and in case we tempered with the retract, we dont want to drive the E-Axis back to some old location - that much likely causes emergency block.
                g_nContinueSteps[E_AXIS] = 0;
            }
        }
        break;
    }
    case UI_ACTION_RF_PAUSE: {
        pausePrint();
        break;
    }
    case UI_ACTION_RF_CONTINUE: {
        continuePrint();
        break;
    }

    /*
    * FUNCTIONS
    */
    case UI_ACTION_EMERGENCY_STOP: {
        Commands::emergencyStop();
        break;
    }
    case UI_ACTION_HOME_ALL: {
        if (PrintLine::linesCount) {
            // do not allow homing via the menu while we are printing
            Com::printFLN(Com::tPrintingIsInProcessError);
            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(true, true, true);
        break;
    }
    case UI_ACTION_HOME_X: {
        if (PrintLine::linesCount) {
            // do not allow homing via the menu while we are printing
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(true, false, false);
        break;
    }
    case UI_ACTION_HOME_Y: {
        if (PrintLine::linesCount) {
            // do not allow homing via the menu while we are printing
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(false, true, false);
        break;
    }
    case UI_ACTION_HOME_Z: {
        if (PrintLine::linesCount) {
            // do not allow homing via the menu while we are printing
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }
            showError((void*)ui_text_home, (void*)ui_text_operation_denied);
            break;
        }
        if (!isHomingAllowed(NULL, 1)) {
            break;
        }
        exitmenu();
        Printer::homeAxis(false, false, true);
        break;
    }
    case UI_ACTION_SET_XY_ORIGIN: {
        Printer::setOrigin(-Printer::destinationMM[X_AXIS], -Printer::destinationMM[Y_AXIS], Printer::originOffsetMM[Z_AXIS]);
        BEEP_ACCEPT_SET_POSITION
        skipBeep = true; // Prevent double beep
        break;
    }
    case UI_ACTION_DEBUG_ECHO: {
        if (Printer::debugEcho())
            Printer::debugLevel -= 1;
        else
            Printer::debugLevel += 1;
        break;
    }
    case UI_ACTION_DEBUG_INFO: {
        if (Printer::debugInfo())
            Printer::debugLevel -= 2;
        else
            Printer::debugLevel += 2;
        break;
    }
    case UI_ACTION_DEBUG_ERROR: {
        if (Printer::debugErrors())
            Printer::debugLevel -= 4;
        else
            Printer::debugLevel += 4;
        break;
    }
    case UI_ACTION_DEBUG_DRYRUN: {
        if (Printer::debugDryrun())
            Printer::debugLevel -= 8;
        else
            Printer::debugLevel += 8;
        if (Printer::debugDryrun()) // simulate movements without printing
        {
            Extruder::setTemperatureForAllExtruders(0, false);
#if HAVE_HEATED_BED == true
            Extruder::setHeatedBedTemperature(0);
#endif // HAVE_HEATED_BED==true
        }
        break;
    }

#if FEATURE_CASE_LIGHT
    case UI_ACTION_LIGHTS_ONOFF: {
        if (Printer::enableCaseLight)
            Printer::enableCaseLight = 0;
        else
            Printer::enableCaseLight = 1;
        WRITE(CASE_LIGHT_PIN, Printer::enableCaseLight);

        HAL::eprSetByte(EPR_RF_CASE_LIGHT_MODE, Printer::enableCaseLight);
        EEPROM::updateChecksum();

        break;
    }
#endif // FEATURE_CASE_LIGHT

#if FEATURE_230V_OUTPUT
    case UI_ACTION_230V_OUTPUT: {
        if (Printer::enable230VOutput)
            Printer::enable230VOutput = 0;
        else
            Printer::enable230VOutput = 1;
        WRITE(OUTPUT_230V_PIN, Printer::enable230VOutput);

        // after a power-on, the 230 V plug always shall be turned off - thus, we do not store this setting to the EEPROM
        // HAL::eprSetByte( EPR_RF_230V_OUTPUT_MODE, Printer::enable230VOutput );
        // EEPROM::updateChecksum();

        break;
    }
#endif // FEATURE_230V_OUTPUT

#if FEATURE_24V_FET_OUTPUTS
#if FET1 > -1
    case UI_ACTION_FET1_OUTPUT: {
        if (Printer::enableFET1)
            Printer::enableFET1 = 0;
        else
            Printer::enableFET1 = 1;
        WRITE(FET1, Printer::enableFET1);

        HAL::eprSetByte(EPR_RF_FET1_MODE, Printer::enableFET1);
        EEPROM::updateChecksum();
        break;
    }
#endif // FET1
#if FET2 > -1
    case UI_ACTION_FET2_OUTPUT: {
        if (Printer::enableFET2)
            Printer::enableFET2 = 0;
        else
            Printer::enableFET2 = 1;
        WRITE(FET2, Printer::enableFET2);

        HAL::eprSetByte(EPR_RF_FET2_MODE, Printer::enableFET2);
        EEPROM::updateChecksum();
        break;
    }
#endif // FET2
#endif // FEATURE_24V_FET_OUTPUTS

    case UI_ACTION_CONFIG_SINGLE_STEPS: {
        configureMANUAL_STEPS_Z(1);
        break;
    }

    case UI_ACTION_CONFIG_SINGLE_STEPS_KOSYS: {
        if (Printer::moveKosys)
            Printer::moveKosys = false;
        else
            Printer::moveKosys = true;

        HAL::eprSetByte(EPR_RF_MOVE_MODE_XY_KOSYS, Printer::moveKosys);
        EEPROM::updateChecksum();
        break;
    }

    case UI_ACTION_CONFIG_POSITION_FEEDRATE: {
        if (Printer::movePositionFeedrateChoice)
            Printer::movePositionFeedrateChoice = false;
        else
            Printer::movePositionFeedrateChoice = true;

        HAL::eprSetByte(EPR_RF_MOVE_POSITION_FEEDRATE, Printer::movePositionFeedrateChoice);
        EEPROM::updateChecksum();
        break;
    }

#if FEATURE_MILLING_MODE
    case UI_ACTION_OPERATING_MODE: {
        char deny = 0;

        if (PrintLine::linesCount)
            deny = 1; // the operating mode can not be switched while the printing is in progress

#if FEATURE_HEAT_BED_Z_COMPENSATION
        if (g_nHeatBedScanStatus || g_nZOSScanStatus)
            deny = 1; // the operating mode can not be switched while a heat bed scan / ZOS is in progress
#endif                // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_ALIGN_EXTRUDERS
        if (g_nAlignExtrudersStatus)
            deny = 1;
#endif //FEATURE_ALIGN_EXTRUDERS

#if FEATURE_WORK_PART_Z_COMPENSATION
        if (g_nWorkPartScanStatus)
            deny = 1; // the operating mode can not be switched while a work part scan is in progress
#endif                // FEATURE_WORK_PART_Z_COMPENSATION

#if FEATURE_FIND_Z_ORIGIN
        if (g_nFindZOriginStatus)
            deny = 1; // the operating mode can not be switched while the z-origin is searched
#endif                // FEATURE_FIND_Z_ORIGIN

        if (deny) {
            showError((void*)ui_text_change_mode, (void*)ui_text_operation_denied);
            break;
        }

        // disable and turn off everything before we switch the operating mode
        Printer::switchEverythingOff();

        if (Printer::operatingMode == OPERATING_MODE_PRINT) {
            switchOperatingMode(OPERATING_MODE_MILL);
        } else {
            switchOperatingMode(OPERATING_MODE_PRINT);
        }

        HAL::eprSetByte(EPR_RF_OPERATING_MODE, Printer::operatingMode);
        EEPROM::updateChecksum();

        break;
    }
#endif // FEATURE_MILLING_MODE

#if FEATURE_CONFIGURABLE_Z_ENDSTOPS
    case UI_ACTION_Z_ENDSTOP_TYPE: {
        if (PrintLine::linesCount) {
            // the z-endstop type can not be switched while the printing is in progress
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_change_z_type, (void*)ui_text_operation_denied);
            break;
        }

        Printer::lastZDirection = 0;
        Printer::endstopZMinHit = ENDSTOP_NOT_HIT;
        Printer::endstopZMaxHit = ENDSTOP_NOT_HIT;

        if (Printer::ZEndstopType == ENDSTOP_TYPE_SINGLE) {
            Printer::ZEndstopType = ENDSTOP_TYPE_CIRCUIT;

            if (Printer::isZMinEndstopHit() || Printer::isZMaxEndstopHit()) {
                // a z-endstop is active at the moment, but both z-endstops are within one circuit so we do not know which one is the pressed one
                // in this situation we do not allow any moving into z-direction before a z-homing has been performed
                Printer::ZEndstopUnknown = 1;
            }
        } else {
            Printer::ZEndstopType = ENDSTOP_TYPE_SINGLE;
            Printer::ZEndstopUnknown = 0;
        }

        // update our information about the currently active Z-endstop
        Printer::isZMinEndstopHit();
        Printer::isZMaxEndstopHit();

        HAL::eprSetByte(EPR_RF_Z_ENDSTOP_TYPE, Printer::ZEndstopType);
        EEPROM::updateChecksum();
        break;
    }
#endif // FEATURE_CONFIGURABLE_Z_ENDSTOPS

    case UI_ACTION_ZMODE: {
        if (Printer::ZMode == 1)
            Printer::ZMode = 2;
        else
            Printer::ZMode = 1;

        HAL::eprSetByte(EPR_RF_Z_MODE, Printer::ZMode);
        EEPROM::updateChecksum();
        break;
    }

#if FEATURE_CONFIGURABLE_MILLER_TYPE
    case UI_ACTION_MILLER_TYPE: {
        if (PrintLine::linesCount) {
            // the hotend type can not be switched while the printing is in progress
            if (Printer::debugErrors()) {
                Com::printFLN(Com::tPrintingIsInProcessError);
            }

            showError((void*)ui_text_change_miller_type, (void*)ui_text_operation_denied);
            break;
        }

        if (Printer::MillerType == MILLER_TYPE_ONE_TRACK) {
            Printer::MillerType = MILLER_TYPE_TWO_TRACKS;
            g_nScanContactPressureDelta = MT2_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA;
            g_nScanRetryPressureDelta = MT2_WORK_PART_SCAN_RETRY_PRESSURE_DELTA;
        } else {
            Printer::MillerType = MILLER_TYPE_ONE_TRACK;
            g_nScanContactPressureDelta = MT1_WORK_PART_SCAN_CONTACT_PRESSURE_DELTA;
            g_nScanRetryPressureDelta = MT1_WORK_PART_SCAN_RETRY_PRESSURE_DELTA;
        }

        HAL::eprSetByte(EPR_RF_MILLER_TYPE, Printer::MillerType);
        EEPROM::updateChecksum();
        break;
    }
#endif // FEATURE_CONFIGURABLE_MILLER_TYPE

    case UI_ACTION_PREHEAT_PLA: {
        g_uStartOfIdle = HAL::timeInMilliseconds() + 10000; //preheat PLA just got selected
        UI_STATUS_UPD(UI_TEXT_PREHEAT_PLA);
        Extruder::setTemperatureForAllExtruders(UI_SET_PRESET_EXTRUDER_TEMP_PLA, false);
#if HAVE_HEATED_BED == true
        Extruder::setHeatedBedTemperature(UI_SET_PRESET_HEATED_BED_TEMP_PLA);
#endif // HAVE_HEATED_BED==true
        break;
    }
    case UI_ACTION_PREHEAT_ABS: {
        g_uStartOfIdle = HAL::timeInMilliseconds() + 10000; //preheat ABS just got selected
        UI_STATUS_UPD(UI_TEXT_PREHEAT_ABS);
        Extruder::setTemperatureForAllExtruders(UI_SET_PRESET_EXTRUDER_TEMP_ABS, false);
#if HAVE_HEATED_BED == true
        Extruder::setHeatedBedTemperature(UI_SET_PRESET_HEATED_BED_TEMP_ABS);
#endif // HAVE_HEATED_BED==true
        break;
    }
    case UI_ACTION_COOLDOWN: {
        UI_STATUS_UPD(UI_TEXT_COOLDOWN);
        Extruder::setTemperatureForAllExtruders(0, false);
#if HAVE_HEATED_BED == true
        Extruder::setHeatedBedTemperature(0);
#endif // HAVE_HEATED_BED==true
        break;
    }
    case UI_ACTION_HEATED_BED_OFF: {
#if HAVE_HEATED_BED == true
        Extruder::setHeatedBedTemperature(0);
#endif // HAVE_HEATED_BED==true
        break;
    }
    case UI_ACTION_EXTRUDER0_OFF: {
        Extruder::setTemperatureForExtruder(0, 0);
        break;
    }
    case UI_ACTION_EXTRUDER1_OFF: {
#if NUM_EXTRUDER > 1
        Extruder::setTemperatureForExtruder(0, 1);
#endif // NUM_EXTRUDER>1
        break;
    }
    case UI_ACTION_DISABLE_STEPPER: {
        Printer::disableAllSteppersNow();
        break;
    }
    case UI_ACTION_MOUNT_FILAMENT_SOFT:
    case UI_ACTION_MOUNT_FILAMENT_HARD:
    case UI_ACTION_UNMOUNT_FILAMENT_SOFT:
    case UI_ACTION_UNMOUNT_FILAMENT_HARD: {
        g_uStartOfIdle = 0;
        while (Printer::checkAbortKeys())
            Commands::checkForPeriodicalActions(); //dont quit script by holding the ok longer than 1ms if no temp is involved. -> min einmal OK loslassen.
        bool unmount = (action == UI_ACTION_UNMOUNT_FILAMENT_SOFT || action == UI_ACTION_UNMOUNT_FILAMENT_HARD);
        exitmenu();

        if (unmount) {
            UI_STATUS_UPD(UI_TEXT_UNMOUNT_FILAMENT);
            if (action == UI_ACTION_UNMOUNT_FILAMENT_SOFT) {
                GCode::executeFString(Com::tUnmountFilamentSoft);
            } else {
                GCode::executeFString(Com::tUnmountFilamentHard);
            }
        } else {
            UI_STATUS_UPD(UI_TEXT_MOUNT_FILAMENT);
            if (action == UI_ACTION_MOUNT_FILAMENT_SOFT) {
                /* Diese PLA Temp ist bei ~180 °C, die meisten Filamente werden gerade so weich. Wenn man keine mindesetens "weiche" Temperatur eingestellt hat, soll geheizt werden.*/
                if (Extruder::current->tempControl.targetTemperatureC < UI_SET_PRESET_EXTRUDER_TEMP_PLA) {
                    Extruder::setTemperatureForExtruder(UI_SET_PRESET_EXTRUDER_TEMP_PLA, Extruder::current->id, true);
                    Extruder::current->tempControl.waitForTargetTemperature();
                    GCode::executeFString(Com::tMountFilamentSoft);
                    Extruder::setTemperatureForExtruder(0, Extruder::current->id, false);
                } else {
                    GCode::executeFString(Com::tMountFilamentSoft);
                }
            } else {
                GCode::executeFString(Com::tMountFilamentHard);
            }
        }
        g_uStartOfIdle = HAL::timeInMilliseconds(); // UI_ACTION_UNMOUNT_FILAMENT UI_ACTION_MOUNT_FILAMENT
        break;
    }
    case UI_ACTION_SET_E_ORIGIN: {
        Printer::setEAxisSteps(0); //G92 E0
        break;
    }
    case UI_ACTION_EXTRUDER_RELATIVE: {
        Printer::relativeExtruderCoordinateMode = !Printer::relativeExtruderCoordinateMode;
        break;
    }
    case UI_ACTION_SELECT_EXTRUDER0: {
        Extruder::selectExtruderById(0);
        break;
    }
    case UI_ACTION_SELECT_EXTRUDER1: {
#if NUM_EXTRUDER > 1
        Extruder::selectExtruderById(1);
#endif // NUM_EXTRUDER>1
        break;
    }
#if NUM_EXTRUDER >= 1
    case UI_ACTION_ACTIVE_EXTRUDER: {
        if (Extruder::current->id == 0)
            Extruder::selectExtruderById(1);
        else
            Extruder::selectExtruderById(0);
        break;
    }
#endif // NUM_EXTRUDER == 2

#if FEATURE_BEEPER
    case UI_ACTION_BEEPER: {
        if (Printer::enableBeeper)
            Printer::enableBeeper = 0;
        else
            Printer::enableBeeper = 1;

        HAL::eprSetByte(EPR_RF_BEEPER_MODE, Printer::enableBeeper);
        EEPROM::updateChecksum();
        break;
    }
#endif // FEATURE_BEEPER

#if SDSUPPORT
    case UI_ACTION_SD_PRINT: {
        if (sd.sdactive) {
            pushMenu((void*)&ui_menu_sd_fileselector, false);
        }
        break;
    }
    case UI_ACTION_SD_PAUSE: {
        exitmenu();
        pausePrint();
        break;
    }
    case UI_ACTION_SD_CONTINUE: {
        exitmenu();
        continuePrint();
        break;
    }
    case UI_ACTION_SD_UNMOUNT: {
        sd.unmount();
        break;
    }
    case UI_ACTION_SD_MOUNT: {
        sd.mount(/*not silent mount*/);
        break;
    }
#endif // SDSUPPORT

#if FEATURE_READ_CALIPER
    case UI_ACTION_CAL_RESET: {
        InterruptProtectedBlock noInts;
        caliper_collect_um = 0;
        caliper_collect_count = 0;
        noInts.unprotect();
        BEEP_ACCEPT_SET_POSITION
        skipBeep = true; // Prevent double beep
        break;
    }
    case UI_ACTION_CAL_SET: {
        if (caliper_collect_um && caliper_collect_count) {
            float dim = (float)caliper_filament_standard / (caliper_collect_um / caliper_collect_count);
            float newExtrusionFactor = dim * dim;
            Commands::changeFlowrateMultiply(newExtrusionFactor);
            Com::printFLN(PSTR("Set Flowrate Multiplier: "), uint8_t(newExtrusionFactor * 100));
            BEEP_ACCEPT_SET_POSITION
            skipBeep = true; // Prevent double beep
        }
        break;
    }
#endif //FEATURE_READ_CALIPER

#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    case UI_ACTION_FAN_OFF: {
        Commands::setFanSpeed((uint8_t)0);
        break;
    }
    case UI_ACTION_FAN_25: {
        Commands::setFanSpeed((uint8_t)64);
        break;
    }
    case UI_ACTION_FAN_50: {
        Commands::setFanSpeed((uint8_t)128);
        break;
    }
    case UI_ACTION_FAN_75: {
        Commands::setFanSpeed((uint8_t)192);
        break;
    }
    case UI_ACTION_FAN_FULL: {
        Commands::setFanSpeed((uint8_t)255);
        break;
    }
    case UI_ACTION_FAN_MODE: {
        Commands::adjustFanMode((part_fan_frequency_modulation ? PART_FAN_MODE_PWM : PART_FAN_MODE_PDM)); //0 = pwm, 1 = pdm
        HAL::eprSetByte(EPR_RF_FAN_MODE, part_fan_frequency_modulation);
        EEPROM::updateChecksum();
        break;
    }
#endif // FAN_PIN>-1 && FEATURE_FAN_CONTROL

    case UI_ACTION_MENU_XPOS: {
        pushMenu((void*)&ui_menu_xpos, false);
        break;
    }
    case UI_ACTION_MENU_YPOS: {
        pushMenu((void*)&ui_menu_ypos, false);
        break;
    }
    case UI_ACTION_MENU_ZPOS: {
        pushMenu((void*)&ui_menu_zpos, false);
        break;
    }
    case UI_ACTION_MENU_QUICKSETTINGS: {
        pushMenu((void*)&ui_menu_quick, false);
        break;
    }
    case UI_ACTION_MENU_EXTRUDER: {
        pushMenu((void*)&ui_menu_extruder, false);
        break;
    }
    case UI_ACTION_MENU_POSITIONS: {
        pushMenu((void*)&ui_menu_positions, false);
        break;
    }

#if MAX_HARDWARE_ENDSTOP_Z
    case UI_ACTION_SET_Z_ORIGIN: {
        setZOrigin();
        break;
    }
#endif // MAX_HARDWARE_ENDSTOP_Z

#if FEATURE_ZERO_DIGITS
    case UI_ACTION_FEATURE_ZERO_DIGITS: {
        Printer::g_pressure_offset_active = (Printer::g_pressure_offset_active ? false : true);
        HAL::eprSetByte(EPR_RF_ZERO_DIGIT_STATE, (Printer::g_pressure_offset_active ? 1 : 2)); //2 ist false, < 1 ist true
        EEPROM::updateChecksum();
        break;
    }
#endif // FEATURE_ZERO_DIGITS
#if FEATURE_DIGIT_Z_COMPENSATION
    case UI_ACTION_DIGIT_COMPENSATION: {
        g_nDigitZCompensationDigits_active = (g_nDigitZCompensationDigits_active ? false : true);
        HAL::eprSetByte(EPR_RF_DIGIT_CMP_STATE, (g_nDigitZCompensationDigits_active ? 1 : 2)); //2 ist false, < 1 ist true
        EEPROM::updateChecksum();
        break;
    }
#endif // FEATURE_DIGIT_Z_COMPENSATION
#if FEATURE_SENSIBLE_PRESSURE
    case UI_ACTION_SENSEOFFSET_AUTOSTART: {
        Printer::g_senseoffset_autostart = (Printer::g_senseoffset_autostart ? false : true);
        HAL::eprSetByte(EPR_RF_MOD_SENSEOFFSET_AUTOSTART, (int8_t)Printer::g_senseoffset_autostart);
        EEPROM::updateChecksum();
        break;
    }
#endif //FEATURE_SENSIBLE_PRESSURE
#if FEATURE_HEAT_BED_Z_COMPENSATION
    case UI_ACTION_RF_DO_MHIER_BED_SCAN: {
        //macht an, wenn an, macht aus:
        startZOScan();
        //gehe zurück und zeige dem User was passiert.
        exitmenu();
        break;
    }
    case UI_ACTION_RF_DO_MHIER_AUTO_MATRIX_LEVELING: {
        //macht an, wenn an, macht aus:
        startZOScan(true); //Scan aber an vielen Punkten und Gewichtet.
                           //gehe zurück und zeige dem User was passiert.
        exitmenu();
        break;
    }
    case UI_ACTION_RF_DO_SAVE_ACTIVE_ZMATRIX: {
        // save the determined values to the EEPROM
        if (g_ZMatrixChangedInRam) {
            exitmenu();
            saveCompensationMatrix((unsigned int)(EEPROM_SECTOR_SIZE * g_nActiveHeatBed));
            if (Printer::debugInfo()) {
                Com::printFLN(PSTR("Manual Input: the heat bed compensation matrix has been saved"));
            }
            showInformation((void*)ui_text_manual, (void*)ui_text_saving_success);
        } else {
            showInformation((void*)ui_text_manual, (void*)ui_text_saving_needless);
        }
        break;
    }
#endif // FEATURE_HEAT_BED_Z_COMPENSATION
#if FEATURE_ALIGN_EXTRUDERS
    case UI_ACTION_ALIGN_EXTRUDERS: {
        exitmenu();
        startAlignExtruders();
        break;
    }
#endif // FEATURE_ALIGN_EXTRUDERS

#if FEATURE_HEAT_BED_Z_COMPENSATION
    case UI_ACTION_RF_SCAN_HEAT_BED: {
        g_nHeatBedScanMode = 0;
        startHeatBedScan();
        //gehe zurück und zeige dem User was passiert.
        uid.exitmenu();
        break;
    }
    case UI_ACTION_RF_SCAN_HEAT_BED_PLA: {
        g_nHeatBedScanMode = HEAT_BED_SCAN_MODE_PLA;
        startHeatBedScan();
        //gehe zurück und zeige dem User was passiert.
        uid.exitmenu();
        break;
    }
    case UI_ACTION_RF_SCAN_HEAT_BED_ABS: {
        g_nHeatBedScanMode = HEAT_BED_SCAN_MODE_ABS;
        startHeatBedScan();
        //gehe zurück und zeige dem User was passiert.
        uid.exitmenu();
        break;
    }

#if FEATURE_WORK_PART_Z_COMPENSATION
    case UI_ACTION_RF_SCAN_WORK_PART: {
        startWorkPartScan(0);
        break;
    }
    case UI_ACTION_RF_SET_SCAN_XY_START: {
        setScanXYStart();
        break;
    }
    case UI_ACTION_RF_SET_SCAN_XY_END: {
        setScanXYEnd();
        break;
    }
#endif // FEATURE_WORK_PART_Z_COMPENSATION
#endif // FEATURE_HEAT_BED_Z_COMPENSATION

#if FEATURE_ALIGN_EXTRUDERS
    case UI_ACTION_RF_ALIGN_EXTRUDERS: {
        startAlignExtruders();
        break;
    }
#endif // FEATURE_ALIGN_EXTRUDERS

    case UI_ACTION_RF_OUTPUT_OBJECT: {
        outputObject(); //als UI_ACTION_RF_OUTPUT_OBJECT
        break;
    }

#if FEATURE_FIND_Z_ORIGIN
    case UI_ACTION_RF_FIND_Z_ORIGIN: {
        startFindZOrigin();
        break;
    }
#endif // FEATURE_FIND_Z_ORIGIN

#if FEATURE_PARK
    case UI_ACTION_RF_PARK: {
        parkPrinter();
        break;
    }
#endif // FEATURE_PARK
    }

    if (!skipBeep)
        BEEP_SHORT
} //mainSwitchCase

void UIDisplay::nextPreviousAction(int8_t next) {
    if (Printer::isUIErrorMessage()) {
        Printer::setUIErrorMessage(false);
        return;
    }
    millis_t actTime = HAL::timeInMilliseconds();
    millis_t dtReal;
    millis_t dt = dtReal = actTime - lastNextPrev;
    lastNextPrev = actTime;

    if (dt < SPEED_MAX_MILLIS)
        dt = SPEED_MAX_MILLIS;
    if (dt > SPEED_MIN_MILLIS) {
        dt = SPEED_MIN_MILLIS;
        lastNextAccumul = 1;
    }
    float f = (float)(SPEED_MIN_MILLIS - dt) / (float)(SPEED_MIN_MILLIS - SPEED_MAX_MILLIS);
    lastNextAccumul = 1.0f + (float)SPEED_MAGNIFICATION * f * f * f;

    if (menuLevel == 0) {
        lastSwitch = HAL::timeInMilliseconds();
        if ((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0)) {
#if FEATURE_MILLING_MODE
            if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
                menuPos[0]++;
                if (menuPos[0] >= UI_NUM_PAGES)
                    menuPos[0] = 0;
#if FEATURE_MILLING_MODE
            } else {
                menuPos[0]++;
                if (menuPos[0] == 1 || menuPos[0] == 3) //kein modmenü und kein temperaturmenü im Millingmode
                {
                    menuPos[0]++;
                }
                if (menuPos[0] >= UI_NUM_PAGES) {
                    menuPos[0] = 0;
                }
            }
#endif // FEATURE_MILLING_MODE
        } else {
#if FEATURE_MILLING_MODE
            if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
                menuPos[0] = (menuPos[0] == 0 ? UI_NUM_PAGES - 1 : menuPos[0] - 1);
#if FEATURE_MILLING_MODE
            } else {
                menuPos[0] = (menuPos[0] == 0 ? UI_NUM_PAGES - 1 : menuPos[0] - 1);
                if (menuPos[0] == 1 || menuPos[0] == 3) //kein modmenü und kein temperaturmenü im Millingmode
                {
                    menuPos[0]--; //kann in diesem if nicht -1 werden, könnte es aber bei veränderung!
                }
            }
#endif // FEATURE_MILLING_MODE
        }
        return;
    }

    UIMenu* men = (UIMenu*)menu[menuLevel];
    uint8_t nr = (uint8_t)pgm_read_word(&(men->numEntries));
    uint8_t mtype = HAL::readFlashByte((const prog_char*)&(men->menuType));
    UIMenuEntry** entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
    UIMenuEntry* ent = (UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]]));
    UIMenuEntry* testEnt;
    uint8_t entType = HAL::readFlashByte((const prog_char*)&(ent->menuType)); // 0 = Info, 1 = Headline, 2 = submenu ref, 3 = direct action command
    (void)entType;                                                            //ignore unused error Nibbels
    int action = pgm_read_word(&(ent->action));

    if (mtype == UI_MENU_TYPE_SUBMENU && activeAction == 0) // browse through menu items
    {
        if ((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0)) {
            //up-to-bottom-Patch
            uint8_t vorher = menuPos[menuLevel];
            if (menuPos[menuLevel] < nr - 1)
                menuPos[menuLevel]++;
            else
                menuPos[menuLevel] = 0;
            //gehe maximal einmal im kreis, auch wenn keins der menüpunkte sauber konfiguriert ist ^^.
            while (menuPos[menuLevel] != vorher) {
                testEnt = (UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]]));
                if (testEnt->showEntry())
                    break;

                if (menuPos[menuLevel] < nr - 1)
                    menuPos[menuLevel]++;
                else
                    menuPos[menuLevel] = 0; //0..nr-1; nr ist anzahl submenüpunkte
            }
            //alle untermenüpunkte sind entweder nicht vorhanden oder falsch oder unzulässig: also geh einfach zurück auf das was anfangs dastand.
            testEnt = (UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]]));
            if (!testEnt->showEntry()) {
                // this new chosen menu item shall not be displayed - revert the so-far used menu item
                menuPos[menuLevel] = vorher;
            }
            //up-to-bottom-Patch
        } else {
            //down-to-top-Patch
            uint8_t vorher = menuPos[menuLevel];
            if (menuPos[menuLevel] > 0)
                menuPos[menuLevel]--;
            else
                menuPos[menuLevel] = nr - 1; //0..nr-1; nr ist anzahl submenüpunkte
            //gehe maximal einmal im kreis, auch wenn keins der menüpunkte sauber konfiguriert ist ^^.
            while (menuPos[menuLevel] != vorher) {
                testEnt = (UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]]));
                if (testEnt->showEntry())
                    break;

                if (menuPos[menuLevel] > 0)
                    menuPos[menuLevel]--;
                else
                    menuPos[menuLevel] = nr - 1; //0..nr-1; nr ist anzahl submenüpunkte
            }
            //alle untermenüpunkte sind entweder nicht vorhanden oder falsch oder unzulässig: also geh einfach zurück auf das was anfangs dastand.
            testEnt = (UIMenuEntry*)pgm_read_word(&(entries[menuPos[menuLevel]]));
            if (!testEnt->showEntry()) {
                // this new chosen menu item shall not be displayed - revert the so-far used menu item
                menuPos[menuLevel] = vorher;
            }
            //down-to-top-Patch ende
        }
        shift = -2; // reset shift position
        adjustMenuPos();
        return;
    }

#if SDSUPPORT
    if (mtype == UI_MENU_TYPE_FILE_SELECTOR) // SD listing
    {
        //Com::printF( PSTR( "SD listing: " ), menuPos[menuLevel] ); Com::printFLN( PSTR( " / " ), menuLevel );
        if ((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0)) {
            menuPos[menuLevel] += 1;
            if (menuPos[menuLevel] > nFilesOnCard)
                menuPos[menuLevel] = 0;
        } else {
            if (menuPos[menuLevel] > 0)
                menuPos[menuLevel] -= 1;
            else
                menuPos[menuLevel] = nFilesOnCard;
        }
        if (menuTop[menuLevel] > menuPos[menuLevel]) {
            menuTop[menuLevel] = menuPos[menuLevel];
        } else if (menuTop[menuLevel] + UI_ROWS <= menuPos[menuLevel]) {
            menuTop[menuLevel] = (menuPos[menuLevel] + 1);
            menuTop[menuLevel] -= static_cast<uint16_t>(UI_ROWS); // DO NOT COMBINE IN ONE LINE - WILL NOT COMPILE CORRECTLY THEN!
        }
        shift = -2; // reset shift position
        return;
    }
#endif // SDSUPPORT

    if (mtype == UI_MENU_TYPE_MODIFICATION_MENU)
        action = pgm_read_word(&(men->id));
    else
        action = activeAction;

#if UI_INVERT_INCREMENT_DIRECTION
    changeSwitchCase(action, -next);
#else
    changeSwitchCase(action, next);
#endif // UI_INVERT_INCREMENT_DIRECTION

#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
#if UI_PRINT_AUTORETURN_TO_MENU_AFTER != 0
        g_nAutoReturnTime = HAL::timeInMilliseconds() + UI_PRINT_AUTORETURN_TO_MENU_AFTER;
#endif // UI_PRINT_AUTORETURN_TO_MENU_AFTER!=0
#if FEATURE_MILLING_MODE
    } else {
#if UI_MILL_AUTORETURN_TO_MENU_AFTER != 0
        g_nAutoReturnTime = HAL::timeInMilliseconds() + UI_MILL_AUTORETURN_TO_MENU_AFTER;
#endif // UI_MILL_AUTORETURN_TO_MENU_AFTER!=0
    }
#endif // FEATURE_MILLING_MODE
} // nextPreviousAction

// Actions are events from user input. Depending on the current state, each
// action can behave differently. Other actions do always the same like home, disable extruder etc.
void UIDisplay::executeAction(int action) {
    if (Printer::blockAll) {
        if (action == UI_ACTION_OK || action == UI_ACTION_RF_CONTINUE) {
            Com::printFLN(PSTR("Restart after Emergency-Stop"));
            HAL::delayMilliseconds(100);
            Commands::emergencyStop();
        }
        // do not allow any user inputs when we have been blocked
        return;
    }

    if (action & UI_ACTION_TOPMENU) // Go to start menu
    {
        action -= UI_ACTION_TOPMENU;
        exitmenu();
    } else {
        mainSwitchCase(action);
    }

    refreshPage();

#if FEATURE_MILLING_MODE
    if (Printer::operatingMode == OPERATING_MODE_PRINT) {
#endif // FEATURE_MILLING_MODE
#if UI_PRINT_AUTORETURN_TO_MENU_AFTER != 0
        g_nAutoReturnTime = HAL::timeInMilliseconds() + UI_PRINT_AUTORETURN_TO_MENU_AFTER;
#endif // UI_PRINT_AUTORETURN_TO_MENU_AFTER!=0
#if FEATURE_MILLING_MODE
    } else {
#if UI_MILL_AUTORETURN_TO_MENU_AFTER != 0
        g_nAutoReturnTime = HAL::timeInMilliseconds() + UI_MILL_AUTORETURN_TO_MENU_AFTER;
#endif // UI_MILL_AUTORETURN_TO_MENU_AFTER!=0
    }
#endif // FEATURE_MILLING_MODE
} // executeAction

void UIDisplay::slowAction() {
    millis_t time = HAL::timeInMilliseconds();
    uint8_t refresh = 0;

    // Update key buffer
    InterruptProtectedBlock noInts; //HAL::forbidInterrupts();

    if ((flags & UI_FLAG_SLOW_ACTION_RUNNING) == 0) {
        flags |= UI_FLAG_SLOW_ACTION_RUNNING;

        // Reset click encoder
        //HAL::forbidInterrupts(); //_-> Ist schon protected!! in jedem aller fälle. entweder ist das mist oder vorher n bug.
        int8_t epos = encoderPos;
        encoderPos = 0;
        noInts.unprotect(); //HAL::allowInterrupts();
        if (epos) {
            Com::writeToAll = true;
            nextPreviousAction(epos); //Nibbels: Funktion, die rechts links auswertet oder Errors wegklicken lässt. abhängig von this->encoderPos .. sonst verwendet mit -1 oder 1
            BEEP_SHORT
            refresh = 1;
        }
        if (lastAction != lastButtonAction) {
#if FEATURE_UNLOCK_MOVEMENT
            //Bei beliebiger user interaktion oder Homing soll G1 etc. erlaubt werden. Dann ist der Drucker nicht abgestürzt, sondern bedient worden.
            Printer::g_unlock_movement = 1;
#endif //FEATURE_UNLOCK_MOVEMENT
            if (lastButtonAction == 0) {
                lastAction = 0;
                noInts.protect(); //HAL::forbidInterrupts();
                flags &= ~3;
            } else if (time - lastButtonStart > UI_KEY_BOUNCETIME) // New key pressed
            {
                lastAction = lastButtonAction;
                Com::writeToAll = true;
                executeAction(lastAction);
                nextRepeat = time + UI_KEY_FIRST_REPEAT;
                repeatDuration = UI_KEY_FIRST_REPEAT;
                //TODO hier ist ein anderer Code bei REpetier. "DelayedAction" -> Prüfen, warum.
            }
        } else if (lastAction < 1000 && lastAction) // Repeatable key
        {
            if (time - nextRepeat < 10000) {
                //TODO hier ist ein anderer Code bei REpetier. "DelayedAction" -> Prüfen, warum.
                executeAction(lastAction);
                repeatDuration -= UI_KEY_REDUCE_REPEAT;
                if (repeatDuration < UI_KEY_MIN_REPEAT)
                    repeatDuration = UI_KEY_MIN_REPEAT;
                nextRepeat = time + repeatDuration;
            }
        }
        noInts.protect(); //HAL::forbidInterrupts();
        flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;
    }
    noInts.unprotect(); //HAL::allowInterrupts();

#if UI_PRINT_AUTORETURN_TO_MENU_AFTER || UI_MILL_AUTORETURN_TO_MENU_AFTER
    if (menuLevel > 0 && g_nAutoReturnTime && g_nAutoReturnTime < time) {
        if (menu[menuLevel] != &ui_menu_message || g_nAutoReturnMessage) {
            lastSwitch = time;
            exitmenu();
            activeAction = 0;
            g_nAutoReturnMessage = false;
        }
        g_nAutoReturnTime = 0;
    }
#endif // UI_PRINT_AUTORETURN_TO_MENU_AFTER || UI_MILL_AUTORETURN_TO_MENU_AFTER

    if (menuLevel == 0 && time > 4000) {
        if (time - lastSwitch > UI_PAGES_DURATION) {
            lastSwitch = time;
#if !defined(UI_DISABLE_AUTO_PAGESWITCH) || !UI_DISABLE_AUTO_PAGESWITCH
            menuPos[0]++;
            if (menuPos[0] >= UI_NUM_PAGES)
                menuPos[0] = 0;
#endif // !defined(UI_DISABLE_AUTO_PAGESWITCH) || !UI_DISABLE_AUTO_PAGESWITCH
            refresh = 1;
        } else if (time - lastRefresh >= 1000)
            refresh = 1;
    } else if (time - lastRefresh >= 800) {
        //UIMenu *men = (UIMenu*)menu[menuLevel];
        //uint8_t mtype = pgm_read_byte((void*)&(men->menuType));
        //(void)mtype; //ignore unused error Nibbels -> ignore all by repetier  https://github.com/repetier/Repetier-Firmware/commit/57c21814d8f8946852fa27af2a7c31831b493ec1
        refresh = 1;
    }

    if (refresh) {
        if (menuLevel > 1 || Printer::isAutomount()) {
            shift++;
            if (shift + UI_COLS > MAX_COLS + 1)
                shift = -2;
        } else
            shift = -2;

        refreshPage();
        lastRefresh = time;
    }
} // slowAction

void UIDisplay::fastAction() {
    // Check keys
    InterruptProtectedBlock noInts;

    if ((flags & (UI_FLAG_KEY_TEST_RUNNING + UI_FLAG_SLOW_KEY_ACTION)) == 0) {
        flags |= UI_FLAG_KEY_TEST_RUNNING;

        int16_t nextAction = 0;
        uid.ui_check_keys(nextAction);

        if (lastButtonAction != nextAction) {
            lastButtonStart = HAL::timeInMilliseconds();
            lastButtonAction = nextAction;
            flags |= UI_FLAG_FAST_KEY_ACTION;
            if (nextAction == UI_ACTION_RF_CONTINUE) {
                g_nContinueButtonPressed = 1;
            }
        }

        if (!nextAction) {
            // no key is pressed at the moment
            if (PrintLine::direct.task == DIRECT_RUNNING_STOPPABLE) {
                // the current direct movement has been started via a hardware or menu button - these movements shall be stopped as soon as the button is released
                PrintLine::stopDirectMove();
            }
        }

        flags &= ~UI_FLAG_KEY_TEST_RUNNING;
    }
} // fastAction

void UIDisplay::lock() {
    locked = 1;
    return;

} // lock

void UIDisplay::unlock() {
    locked = 0;
    return;
} // unlock

void UIDisplay::exitmenu() {
    menuLevel = 0;
    if (uid.menuPos[menuLevel] == 1 || uid.menuPos[menuLevel] > 3)
        uid.menuPos[menuLevel] = 0;
    return;
} // unlock

#if UI_ENCODER_SPEED == 0
const int8_t encoder_table[16] PROGMEM = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 }; // Full speed
#elif UI_ENCODER_SPEED == 1
const int8_t encoder_table[16] PROGMEM = { 0, 0, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 0, 0 }; // Half speed
#else
const int8_t encoder_table[16] PROGMEM = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0 }; // Quart speed
#endif // UI_ENCODER_SPEED==0

#endif //  UI_DISPLAY_TYPE!=0
