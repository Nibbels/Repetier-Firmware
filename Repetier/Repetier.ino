/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.
    Verified build: Arduino CLI 1.2.2, Arduino AVR Boards 1.8.6, ATmega2560.

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

/**
\mainpage RFx000 community-derived Repetier-Firmware
<CENTER>Copyright &copy; 2011-2017 by repetier and Conrad Electronic SE</CENTER>

\section Intro Introduction

 This is not the official Firmware from Conrad Electronic SE.
 This review branch is derived from the community firmware. Current architecture and build evidence
 are in docs/ and coordination/ in Nibbels/Repetier-Firmware. Historical community documentation:
  - http://www.rf1000.de/wiki/index.php/Kategorie:CommunityMod_Firmware 
  - http://rf1000.de 
  - https://github.com/RF1000community/Repetier-Firmware

\section GCodes Implemented GCodes

Implemented Codes

 Standard RF1000/RF2000/RF2000v2 gcodes: http://www.rf1000.de/wiki/index.php/GCodes.
 Special mod version RF1000/RF2000/RF2000v2 gcodes: http://www.rf1000.de/wiki/index.php/Gcodes
*/

#include "Repetier.h"

void setup() {
    Printer::setup();
    initRF();
} // setup

void loop() {
    Commands::commandLoop();
} // loop
