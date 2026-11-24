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
#include "UiMenus.h"
#include "uimenu.h"

constexpr millis_t menuAutoReturnDelayMs = 120000;



bool UIMenuEntry::showEntry() const {
    bool ret = true;
    uint8_t f, f2;

    const uint8_t menuMode = Printer::activeMenuModes();
    f = HAL::readFlashByte((const prog_char*)&filter);
    if (f != 0)
        ret = (f & menuMode) != 0;
    f2 = HAL::readFlashByte((const prog_char*)&nofilter);
    if (ret && f2 != 0) {
        ret = (f2 & menuMode) == 0;
    }
    return ret;
} // showEntry

UIDisplay uid;

const char* UIDisplay::cachedRow(uint8_t row) const {
    return displayCache_[row];
}

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
    HAL::delayMicroseconds(320); // The LCD R/W line is tied low.

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

void UIDisplay::loadStartupCharacters() {
    createChar(1, c1);
    createChar(2, c2);
    createChar(3, c3);
    createChar(4, c4);
    createChar(5, c7);
    createChar(6, c8);
    createChar(7, c9);
    normalCharacters_ = false;
}

void UIDisplay::loadNormalCharacters() {
    createChar(1, character_back);
    createChar(2, character_degree);
    createChar(3, character_selected);
    createChar(4, character_unselected);
    createChar(5, character_temperature);
    createChar(7, character_ready);
    normalCharacters_ = true;
}

void UIDisplay::initializeLCD(bool normal) {
    // bring all display pins into a defined state
    SET_INPUT(UI_DISPLAY_D4_PIN);
    SET_INPUT(UI_DISPLAY_D5_PIN);
    SET_INPUT(UI_DISPLAY_D6_PIN);
    SET_INPUT(UI_DISPLAY_D7_PIN);
    SET_INPUT(UI_DISPLAY_RS_PIN);


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


    SET_OUTPUT(UI_DISPLAY_ENABLE_PIN);

    // Select the LCD command register.
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
        loadNormalCharacters();
    }
} // initializeLCD

// ----------- end direct LCD driver

void UIDisplay::printRow(uint8_t r, char* txt, char* txt2, uint8_t changeAtCol) {
#if MAX_COLS < UI_COLS
#error MAX_COLS must accommodate the LCD columns and the display report
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
        //cache whatever you write to the display!
        displayCache_[r][col] = c;
        col++;
    }
    while (col < changeAtCol) {
        lcdPutChar(' ');
        //cache whatever you write to the display!
        displayCache_[r][col] = ' ';
        col++;
    }

    if (txt2 != NULL) {
        while ((c = *txt2) != 0x00 && col < UI_COLS) {
            txt2++;
            lcdPutChar(c);
            //cache whatever you write to the display!
            displayCache_[r][col] = c;
            col++;
        }
        while (col < UI_COLS) {
            lcdPutChar(' ');
            //cache whatever you write to the display!
            displayCache_[r][col] = ' ';
            col++;
        }
    }
    displayCache_[r][col] = 0;
} // printRow

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


    initializeLCD(false);
    loadStartupCharacters();

    for (uint8_t y = 0; y < UI_ROWS; y++)
        displayCache_[y][0] = 0;
    printRowP(0, PSTR(BIGC0));
    printRowP(1, PSTR(BIGC1));
    printRowP(UI_ROWS - 2, PSTR(BIGC2));
    printRowP(UI_ROWS - 1, PSTR(BIGC3));

} // initialize

void UIDisplay::createChar(uint8_t location, const uint8_t charmap[]) {
    location &= 0x7; // we only have 8 locations 0-7
    lcdCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
        lcdPutChar(pgm_read_byte(&(charmap[i])));
    }

} // createChar

void UIDisplay::printRowP(uint8_t r, PGM_P txt) {
    if (r >= UI_ROWS)
        return;
    col = 0;
    addStringP(txt);
    printCols[col] = 0;
    printRow(r, printCols, NULL, UI_COLS);

} // printRowP

const UIMenu* const ui_pages[] PROGMEM = UI_PAGES;
constexpr uint8_t panelPageCount = sizeof(ui_pages) / sizeof(ui_pages[0]);


// Refresh current menu page
void UIDisplay::refreshPage() {
    uint16_t r;
    char cache[UI_ROWS][MAX_COLS + 1];
    adjustMenuPos();

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


    printCols[0] = 0;
    while (r < UI_ROWS)
        strcpy(cache[r++], printCols);

    for (uint8_t row = 0; row < UI_ROWS; ++row)
        strcpy(displayCache_[row], cache[row]);

    const uint8_t scroll = shift > 0 ? shift : 0;
    for (uint8_t row = 0; row < UI_ROWS; ++row) {
        const uint8_t length = strlen(cache[row]);
        const uint8_t offset = length > UI_COLS ? RMath::min(length - UI_COLS, scroll) : 0;
        printRow(row, &cache[row][offset], NULL, UI_COLS);
    }
} // refreshPage

void UIDisplay::showMessage(const void* line1, const void* line2, const void* line3, const void* line4) {
    messageLine1 = line1;
    messageLine2 = line2;
    messageLine3 = line3;
    messageLine4 = line4;
    pushMenu((void*)&ui_menu_message, true);
    autoReturnTime_ = 0;
}

void UIDisplay::scheduleMessageReturn(millis_t deadline) {
    autoReturnMessage_ = true;
    autoReturnTime_ = deadline;
}

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


    if (entType == 2) { // Enter submenu
        pushMenu((void*)action, false);
        return;
    }
    if (entType == 3) {
        executeAction(action);
        return;
    }
    menuEsc();
} // okAction

void UIDisplay::rightAction() {
    if (menuLevel == 0 && menuPos[0] == 1) {
        senseOffsetUpAction();
        return;
    }

    uint8_t axis;
    int address;
    const auto* currentMenu = menu[menuLevel];
    if (currentMenu == &ui_menu_xpos) {
        axis = X_AXIS;
        address = EPR_RF_MOVE_MODE_X;
    } else if (currentMenu == &ui_menu_ypos) {
        axis = Y_AXIS;
        address = EPR_RF_MOVE_MODE_Y;
    } else if (currentMenu == &ui_menu_zpos) {
        axis = Z_AXIS;
        address = EPR_RF_MOVE_MODE_Z;
    } else {
        return;
    }

    Printer::manualMotion.cycleMode(axis);
    refreshPage();
    HAL::eprSetByte(address, Printer::manualMotion.mode(axis));
    EEPROM::updateChecksum();
}

void UIDisplay::backAction() {
    if (menuLevel == 0 && menuPos[0] == 1) {
        //wenn im Mod-Menü für Z-Offset/Matrix Sense-Offset/Limiter, dann anders!
        //we are in the Mod menu
        //verkleinern des Digit-Limits
        senseOffsetDownAction();
    } else
    {
#ifdef RGB_LIGHT_R_PIN
        if (menuLevel == 0) {
            RgbLighting::requestWhiteToggle();
        }
#endif // RGB_LIGHT_R_PIN
        menuEsc();
        activeAction = 0;
        confirmation_ = 0;
    }
} // backAction

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
        if (next > 0) {
            menuPos[0]++;
            if (menuPos[0] >= panelPageCount)
                menuPos[0] = 0;
        } else {
            menuPos[0] = (menuPos[0] == 0 ? panelPageCount - 1 : menuPos[0] - 1);
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
        if (next > 0) {
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


    if (mtype == UI_MENU_TYPE_MODIFICATION_MENU)
        action = pgm_read_word(&(men->id));
    else
        action = activeAction;

    changeSwitchCase(action, -next);

    autoReturnTime_ = HAL::timeInMilliseconds() + menuAutoReturnDelayMs;
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

    autoReturnTime_ = HAL::timeInMilliseconds() + menuAutoReturnDelayMs;
} // executeAction

void UIDisplay::slowAction() {
    millis_t time = HAL::timeInMilliseconds();
    uint8_t refresh = 0;

    // Update key buffer
    InterruptProtectedBlock noInts; //HAL::forbidInterrupts();

    if ((flags & UI_FLAG_SLOW_ACTION_RUNNING) == 0) {
        flags |= UI_FLAG_SLOW_ACTION_RUNNING;

        const int buttonAction = lastButtonAction;
        const millis_t buttonAge = HAL::timeInMilliseconds() - lastButtonStart;
        noInts.unprotect();
        if (lastAction != buttonAction) {
            //Bei beliebiger user interaktion oder Homing soll G1 etc. erlaubt werden. Dann ist der Drucker nicht abgestürzt, sondern bedient worden.
            Printer::allowMovement();
            if (buttonAction == 0) {
                lastAction = 0;
            } else if (buttonAge > UI_KEY_BOUNCETIME) // New key pressed
            {
                lastAction = buttonAction;
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

    if (menuLevel > 0 && autoReturnTime_ && autoReturnTime_ < time) {
        if (menu[menuLevel] != &ui_menu_message || autoReturnMessage_) {
            lastSwitch = time;
            exitmenu();
            activeAction = 0;
            autoReturnMessage_ = false;
        }
        autoReturnTime_ = 0;
    }

    if (menuLevel == 0 && time > 4000) {
        if (time - lastSwitch > UI_PAGES_DURATION) {
            lastSwitch = time;
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
        if (menuLevel > 1) {
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

    if ((flags & UI_FLAG_KEY_TEST_RUNNING) == 0) {
        flags |= UI_FLAG_KEY_TEST_RUNNING;

        int16_t nextAction = 0;
        uid.ui_check_keys(nextAction);

        if (lastButtonAction != nextAction) {
            lastButtonStart = HAL::timeInMilliseconds();
            lastButtonAction = nextAction;
        }

        const bool jogKeyStillPressed = nextAction && nextAction == jogButtonAction;
        if (PrintLine::hasStoppableDirectMove() && !jogKeyStillPressed) {
            // Releasing the jog key or selecting another key ends the jog.
            PrintLine::stopDirectMove();
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
