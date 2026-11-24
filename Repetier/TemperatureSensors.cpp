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

#define NUMTEMPS_1 28
// Epcos B57560G0107F000
const short temptable_1[NUMTEMPS_1][2] PROGMEM = {
    { 0, 4000 }, { 92, 2400 }, { 105, 2320 }, { 121, 2240 }, { 140, 2160 }, { 162, 2080 }, { 189, 2000 }, { 222, 1920 }, { 261, 1840 }, { 308, 1760 }, { 365, 1680 }, { 434, 1600 }, { 519, 1520 }, { 621, 1440 }, { 744, 1360 }, { 891, 1280 }, { 1067, 1200 }, { 1272, 1120 }, { 1771, 960 }, { 2357, 800 }, { 2943, 640 }, { 3429, 480 }, { 3760, 320 }, { 3869, 240 }, { 3912, 200 }, { 3948, 160 }, { 4077, -160 }, { 4094, -440 }
};

// is 200k thermistor
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM = {
    { 1 * 4, 848 * 8 }, { 54 * 4, 275 * 8 }, { 107 * 4, 228 * 8 }, { 160 * 4, 202 * 8 }, { 213 * 4, 185 * 8 }, { 266 * 4, 171 * 8 }, { 319 * 4, 160 * 8 }, { 372 * 4, 150 * 8 }, { 425 * 4, 141 * 8 }, { 478 * 4, 133 * 8 }, { 531 * 4, 125 * 8 }, { 584 * 4, 118 * 8 }, { 637 * 4, 110 * 8 }, { 690 * 4, 103 * 8 }, { 743 * 4, 95 * 8 }, { 796 * 4, 86 * 8 }, { 849 * 4, 77 * 8 }, { 902 * 4, 65 * 8 }, { 955 * 4, 49 * 8 }, { 1008 * 4, 17 * 8 }, { 1020 * 4, 0 * 8 } //safety
};

// mendel-parts thermistor (EPCOS G550) = NTC mit 100kOhm
#define NUMTEMPS_3 28

const short temptable_3[NUMTEMPS_3][2] PROGMEM = {
    { 1 * 4, 864 * 8 }, { 21 * 4, 300 * 8 }, { 25 * 4, 290 * 8 }, { 29 * 4, 280 * 8 }, { 33 * 4, 270 * 8 }, { 39 * 4, 260 * 8 }, { 46 * 4, 250 * 8 }, { 54 * 4, 240 * 8 }, { 64 * 4, 230 * 8 }, { 75 * 4, 220 * 8 }, { 90 * 4, 210 * 8 }, { 107 * 4, 200 * 8 }, { 128 * 4, 190 * 8 }, { 154 * 4, 180 * 8 }, { 184 * 4, 170 * 8 }, { 221 * 4, 160 * 8 }, { 265 * 4, 150 * 8 }, { 316 * 4, 140 * 8 }, { 375 * 4, 130 * 8 }, { 441 * 4, 120 * 8 }, { 513 * 4, 110 * 8 }, { 588 * 4, 100 * 8 }, { 734 * 4, 80 * 8 }, { 856 * 4, 60 * 8 }, { 938 * 4, 40 * 8 }, { 986 * 4, 20 * 8 }, { 1008 * 4, 0 * 8 }, { 1018 * 4, -20 * 8 }
};

// is 10k thermistor
#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM = {
    { 1 * 4, 430 * 8 }, { 54 * 4, 137 * 8 }, { 107 * 4, 107 * 8 }, { 160 * 4, 91 * 8 }, { 213 * 4, 80 * 8 }, { 266 * 4, 71 * 8 }, { 319 * 4, 64 * 8 }, { 372 * 4, 57 * 8 }, { 425 * 4, 51 * 8 }, { 478 * 4, 46 * 8 }, { 531 * 4, 41 * 8 }, { 584 * 4, 35 * 8 }, { 637 * 4, 30 * 8 }, { 690 * 4, 25 * 8 }, { 743 * 4, 20 * 8 }, { 796 * 4, 14 * 8 }, { 849 * 4, 7 * 8 }, { 902 * 4, 0 * 8 }, { 955 * 4, -11 * 8 }, { 1008 * 4, -35 * 8 }
};

// ATC Semitec 104GT-2 / E3D Hotend Thermistor
#define NUMTEMPS_8 34
const short temptable_8[NUMTEMPS_8][2] PROGMEM = {
    { 0, 8000 }, { 69, 2400 }, { 79, 2320 }, { 92, 2240 }, { 107, 2160 }, { 125, 2080 }, { 146, 2000 }, { 172, 1920 }, { 204, 1840 }, { 244, 1760 }, { 291, 1680 }, { 350, 1600 }, { 422, 1520 }, { 511, 1440 }, { 621, 1360 }, { 755, 1280 }, { 918, 1200 }, { 1114, 1120 }, { 1344, 1040 }, { 1608, 960 }, { 1902, 880 }, { 2216, 800 }, { 2539, 720 }, { 2851, 640 }, { 3137, 560 }, { 3385, 480 }, { 3588, 400 }, { 3746, 320 }, { 3863, 240 }, { 3945, 160 }, { 4002, 80 }, { 4038, 0 }, { 4061, -80 }, { 4075, -160 }
};

// 100k Honeywell 135-104LAG-J01
#define NUMTEMPS_9 58 // 58 explicit points; no zero-filled tail after the 0 C endpoint
const short temptable_9[NUMTEMPS_9][2] PROGMEM = {
    { 1 * 4, 941 * 8 }, { 19 * 4, 362 * 8 }, { 37 * 4, 299 * 8 }, //top rating 300C
    { 55 * 4, 266 * 8 },
    { 73 * 4, 245 * 8 },
    { 91 * 4, 229 * 8 },
    { 109 * 4, 216 * 8 },
    { 127 * 4, 206 * 8 },
    { 145 * 4, 197 * 8 },
    { 163 * 4, 190 * 8 },
    { 181 * 4, 183 * 8 },
    { 199 * 4, 177 * 8 },
    { 217 * 4, 171 * 8 },
    { 235 * 4, 166 * 8 },
    { 253 * 4, 162 * 8 },
    { 271 * 4, 157 * 8 },
    { 289 * 4, 153 * 8 },
    { 307 * 4, 149 * 8 },
    { 325 * 4, 146 * 8 },
    { 343 * 4, 142 * 8 },
    { 361 * 4, 139 * 8 },
    { 379 * 4, 135 * 8 },
    { 397 * 4, 132 * 8 },
    { 415 * 4, 129 * 8 },
    { 433 * 4, 126 * 8 },
    { 451 * 4, 123 * 8 },
    { 469 * 4, 121 * 8 },
    { 487 * 4, 118 * 8 },
    { 505 * 4, 115 * 8 },
    { 523 * 4, 112 * 8 },
    { 541 * 4, 110 * 8 },
    { 559 * 4, 107 * 8 },
    { 577 * 4, 105 * 8 },
    { 595 * 4, 102 * 8 },
    { 613 * 4, 99 * 8 },
    { 631 * 4, 97 * 8 },
    { 649 * 4, 94 * 8 },
    { 667 * 4, 92 * 8 },
    { 685 * 4, 89 * 8 },
    { 703 * 4, 86 * 8 },
    { 721 * 4, 84 * 8 },
    { 739 * 4, 81 * 8 },
    { 757 * 4, 78 * 8 },
    { 775 * 4, 75 * 8 },
    { 793 * 4, 72 * 8 },
    { 811 * 4, 69 * 8 },
    { 829 * 4, 66 * 8 },
    { 847 * 4, 62 * 8 },
    { 865 * 4, 59 * 8 },
    { 883 * 4, 55 * 8 },
    { 901 * 4, 51 * 8 },
    { 919 * 4, 46 * 8 },
    { 937 * 4, 41 * 8 },
    { 955 * 4, 35 * 8 },
    { 973 * 4, 27 * 8 },
    { 991 * 4, 17 * 8 },
    { 1009 * 4, 1 * 8 },
    { 1023 * 4, 0 } //to allow internal 0 degrees C
};

// 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
#define NUMTEMPS_10 20
const short temptable_10[NUMTEMPS_10][2] PROGMEM = {
    { 1 * 4, 704 * 8 }, { 54 * 4, 216 * 8 }, { 107 * 4, 175 * 8 }, { 160 * 4, 152 * 8 }, { 213 * 4, 137 * 8 }, { 266 * 4, 125 * 8 }, { 319 * 4, 115 * 8 }, { 372 * 4, 106 * 8 }, { 425 * 4, 99 * 8 }, { 478 * 4, 91 * 8 }, { 531 * 4, 85 * 8 }, { 584 * 4, 78 * 8 }, { 637 * 4, 71 * 8 }, { 690 * 4, 65 * 8 }, { 743 * 4, 58 * 8 }, { 796 * 4, 50 * 8 }, { 849 * 4, 42 * 8 }, { 902 * 4, 31 * 8 }, { 955 * 4, 17 * 8 }, { 1008 * 4, 0 }
};

// 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
#define NUMTEMPS_11 31
const short temptable_11[NUMTEMPS_11][2] PROGMEM = {
    { 1 * 4, 936 * 8 }, { 36 * 4, 300 * 8 }, { 71 * 4, 246 * 8 }, { 106 * 4, 218 * 8 }, { 141 * 4, 199 * 8 }, { 176 * 4, 185 * 8 }, { 211 * 4, 173 * 8 }, { 246 * 4, 163 * 8 }, { 281 * 4, 155 * 8 }, { 316 * 4, 147 * 8 }, { 351 * 4, 140 * 8 }, { 386 * 4, 134 * 8 }, { 421 * 4, 128 * 8 }, { 456 * 4, 122 * 8 }, { 491 * 4, 117 * 8 }, { 526 * 4, 112 * 8 }, { 561 * 4, 107 * 8 }, { 596 * 4, 102 * 8 }, { 631 * 4, 97 * 8 }, { 666 * 4, 92 * 8 }, { 701 * 4, 87 * 8 }, { 736 * 4, 81 * 8 }, { 771 * 4, 76 * 8 }, { 806 * 4, 70 * 8 }, { 841 * 4, 63 * 8 }, { 876 * 4, 56 * 8 }, { 911 * 4, 48 * 8 }, { 946 * 4, 38 * 8 }, { 981 * 4, 23 * 8 }, { 1005 * 4, 5 * 8 }, { 1016 * 4, 0 }
};

// 100k RS thermistor 198-961 (4.7k pullup)
#define NUMTEMPS_12 31
const short temptable_12[NUMTEMPS_12][2] PROGMEM = {
    { 1 * 4, 929 * 8 }, { 36 * 4, 299 * 8 }, { 71 * 4, 246 * 8 }, { 106 * 4, 217 * 8 }, { 141 * 4, 198 * 8 }, { 176 * 4, 184 * 8 }, { 211 * 4, 173 * 8 }, { 246 * 4, 163 * 8 }, { 281 * 4, 154 * 8 }, { 316 * 4, 147 * 8 }, { 351 * 4, 140 * 8 }, { 386 * 4, 134 * 8 }, { 421 * 4, 128 * 8 }, { 456 * 4, 122 * 8 }, { 491 * 4, 117 * 8 }, { 526 * 4, 112 * 8 }, { 561 * 4, 107 * 8 }, { 596 * 4, 102 * 8 }, { 631 * 4, 97 * 8 }, { 666 * 4, 91 * 8 }, { 701 * 4, 86 * 8 }, { 736 * 4, 81 * 8 }, { 771 * 4, 76 * 8 }, { 806 * 4, 70 * 8 }, { 841 * 4, 63 * 8 }, { 876 * 4, 56 * 8 }, { 911 * 4, 48 * 8 }, { 946 * 4, 38 * 8 }, { 981 * 4, 23 * 8 }, { 1005 * 4, 5 * 8 }, { 1016 * 4, 0 * 8 }
};

/*
// PT100 E3D
#define NUMTEMPS_13 19
const short temptable_13[NUMTEMPS_13][2] PROGMEM =
{
    {0,0},{908,8},{942,10*8},{982,20*8},{1015,8*30},{1048,8*40},{1080,8*50},{1113,8*60},{1146,8*70},{1178,8*80},{1211,8*90},{1276,8*110},{1318,8*120}
    ,{1670,8*230},{2455,8*500},{3445,8*900},{3666,8*1000},{3871,8*1100},{4095,8*2000}
};
*/

//TODO: Tabelle startet erst bei 92°C ... Kleine Rundungszacken am Anfang, weil alles *4.
#define NUMTEMPS_13 61 // NTC 3950 100k thermistor - Conrad V3
const short temptable_13[NUMTEMPS_13][2] PROGMEM = {
    { 23 * 4, 300 * 8 }, { 25 * 4, 295 * 8 }, { 27 * 4, 290 * 8 }, { 28 * 4, 285 * 8 }, { 31 * 4, 280 * 8 }, { 33 * 4, 275 * 8 }, { 35 * 4, 270 * 8 }, { 38 * 4, 265 * 8 }, { 41 * 4, 260 * 8 }, { 44 * 4, 255 * 8 }, { 48 * 4, 250 * 8 }, { 52 * 4, 245 * 8 }, { 56 * 4, 240 * 8 }, { 61 * 4, 235 * 8 }, { 66 * 4, 230 * 8 }, { 71 * 4, 225 * 8 }, { 78 * 4, 220 * 8 }, { 84 * 4, 215 * 8 }, { 92 * 4, 210 * 8 }, { 100 * 4, 205 * 8 }, { 109 * 4, 200 * 8 }, { 120 * 4, 195 * 8 }, { 131 * 4, 190 * 8 }, { 143 * 4, 185 * 8 }, { 156 * 4, 180 * 8 }, { 171 * 4, 175 * 8 }, { 187 * 4, 170 * 8 }, { 205 * 4, 165 * 8 }, { 224 * 4, 160 * 8 }, { 245 * 4, 155 * 8 }, { 268 * 4, 150 * 8 }, { 293 * 4, 145 * 8 }, { 320 * 4, 140 * 8 }, { 348 * 4, 135 * 8 }, { 379 * 4, 130 * 8 }, { 411 * 4, 125 * 8 }, { 445 * 4, 120 * 8 }, { 480 * 4, 115 * 8 }, { 516 * 4, 110 * 8 }, { 553 * 4, 105 * 8 }, { 591 * 4, 100 * 8 }, { 628 * 4, 95 * 8 }, { 665 * 4, 90 * 8 }, { 702 * 4, 85 * 8 }, { 737 * 4, 80 * 8 }, { 770 * 4, 75 * 8 }, { 801 * 4, 70 * 8 }, { 830 * 4, 65 * 8 }, { 857 * 4, 60 * 8 }, { 881 * 4, 55 * 8 }, { 903 * 4, 50 * 8 }, { 922 * 4, 45 * 8 }, { 939 * 4, 40 * 8 }, { 954 * 4, 35 * 8 }, { 966 * 4, 30 * 8 }, { 977 * 4, 25 * 8 }, { 985 * 4, 20 * 8 }, { 993 * 4, 15 * 8 }, { 999 * 4, 10 * 8 }, { 1004 * 4, 5 * 8 }, { 1008 * 4, 0 * 8 }
};

// Thermistor NTC 3950 100k Ohm (other source)
#define NUMTEMPS_14 103
const short temptable_14[NUMTEMPS_14][2] PROGMEM = {
    { 1 * 4, 938 * 8 }, { 11 * 4, 423 * 8 }, { 21 * 4, 351 * 8 }, { 31 * 4, 314 * 8 }, { 41 * 4, 290 * 8 }, { 51 * 4, 272 * 8 }, { 61 * 4, 258 * 8 }, { 71 * 4, 247 * 8 }, { 81 * 4, 237 * 8 }, { 91 * 4, 229 * 8 }, { 101 * 4, 221 * 8 }, { 111 * 4, 215 * 8 }, { 121 * 4, 209 * 8 }, { 131 * 4, 204 * 8 }, { 141 * 4, 199 * 8 }, { 151 * 4, 195 * 8 }, { 161 * 4, 190 * 8 }, { 171 * 4, 187 * 8 }, { 181 * 4, 183 * 8 }, { 191 * 4, 179 * 8 }, { 201 * 4, 176 * 8 }, { 211 * 4, 173 * 8 }, { 221 * 4, 170 * 8 }, { 231 * 4, 167 * 8 }, { 241 * 4, 165 * 8 }, { 251 * 4, 162 * 8 }, { 261 * 4, 160 * 8 }, { 271 * 4, 157 * 8 }, { 281 * 4, 155 * 8 }, { 291 * 4, 153 * 8 }, { 301 * 4, 150 * 8 }, { 311 * 4, 148 * 8 }, { 321 * 4, 146 * 8 }, { 331 * 4, 144 * 8 }, { 341 * 4, 142 * 8 }, { 351 * 4, 140 * 8 }, { 361 * 4, 139 * 8 }, { 371 * 4, 137 * 8 }, { 381 * 4, 135 * 8 }, { 391 * 4, 133 * 8 }, { 401 * 4, 131 * 8 }, { 411 * 4, 130 * 8 }, { 421 * 4, 128 * 8 }, { 431 * 4, 126 * 8 }, { 441 * 4, 125 * 8 }, { 451 * 4, 123 * 8 }, { 461 * 4, 122 * 8 }, { 471 * 4, 120 * 8 }, { 481 * 4, 119 * 8 }, { 491 * 4, 117 * 8 }, { 501 * 4, 116 * 8 }, { 511 * 4, 114 * 8 }, { 521 * 4, 113 * 8 }, { 531 * 4, 111 * 8 }, { 541 * 4, 110 * 8 }, { 551 * 4, 108 * 8 }, { 561 * 4, 107 * 8 }, { 571 * 4, 105 * 8 }, { 581 * 4, 104 * 8 }, { 591 * 4, 102 * 8 }, { 601 * 4, 101 * 8 }, { 611 * 4, 100 * 8 }, { 621 * 4, 98 * 8 }, { 631 * 4, 97 * 8 }, { 641 * 4, 95 * 8 }, { 651 * 4, 94 * 8 }, { 661 * 4, 92 * 8 }, { 671 * 4, 91 * 8 }, { 681 * 4, 90 * 8 }, { 691 * 4, 88 * 8 }, { 701 * 4, 87 * 8 }, { 711 * 4, 85 * 8 }, { 721 * 4, 84 * 8 }, { 731 * 4, 82 * 8 }, { 741 * 4, 81 * 8 }, { 751 * 4, 79 * 8 }, { 761 * 4, 77 * 8 }, { 771 * 4, 76 * 8 }, { 781 * 4, 74 * 8 }, { 791 * 4, 72 * 8 }, { 801 * 4, 71 * 8 }, { 811 * 4, 69 * 8 }, { 821 * 4, 67 * 8 }, { 831 * 4, 65 * 8 }, { 841 * 4, 63 * 8 }, { 851 * 4, 62 * 8 }, { 861 * 4, 60 * 8 }, { 871 * 4, 57 * 8 }, { 881 * 4, 55 * 8 }, { 891 * 4, 53 * 8 }, { 901 * 4, 51 * 8 }, { 911 * 4, 48 * 8 }, { 921 * 4, 45 * 8 }, { 931 * 4, 42 * 8 }, { 941 * 4, 39 * 8 }, { 951 * 4, 36 * 8 }, { 961 * 4, 32 * 8 }, { 971 * 4, 28 * 8 }, { 981 * 4, 23 * 8 }, { 991 * 4, 17 * 8 }, { 1001 * 4, 9 * 8 }, { 1011 * 4, -1 * 8 }, { 1021 * 4, -26 * 8 }
};

// E3D PT100 Board (direct AD voltage in / PTC type)
#define NUMTEMPS_15 19
const short temptable_15[NUMTEMPS_15][2] PROGMEM = {
    { 0, 0 }, { 908, 8 }, { 942, 10 * 8 }, { 982, 20 * 8 }, { 1015, 8 * 30 }, { 1048, 8 * 40 }, { 1080, 8 * 50 }, { 1113, 8 * 60 }, { 1146, 8 * 70 }, { 1178, 8 * 80 }, { 1211, 8 * 90 }, { 1276, 8 * 110 }, { 1318, 8 * 120 }, { 1670, 8 * 230 }, { 2455, 8 * 500 }, { 3445, 8 * 900 }, { 3666, 8 * 1000 }, { 3871, 8 * 1100 }, { 4095, 8 * 2000 }
};

// Leave the retired user-table slots (sensor IDs 5–7) empty to preserve existing sensor numbers.
const short* const temptables[15] PROGMEM = {
    &temptable_1[0][0], &temptable_2[0][0], &temptable_3[0][0], &temptable_4[0][0],
    nullptr, nullptr, nullptr,
    &temptable_8[0][0], &temptable_9[0][0], &temptable_10[0][0], &temptable_11[0][0],
    &temptable_12[0][0], &temptable_13[0][0], &temptable_14[0][0], &temptable_15[0][0],
};
const uint8_t temptables_num[15] PROGMEM = {
    NUMTEMPS_1,
    NUMTEMPS_2,
    NUMTEMPS_3,
    NUMTEMPS_4,
    0,
    0,
    0,
    NUMTEMPS_8,
    NUMTEMPS_9,
    NUMTEMPS_10,
    NUMTEMPS_11,
    NUMTEMPS_12,
    NUMTEMPS_13,
    NUMTEMPS_14,
    NUMTEMPS_15
};

namespace {
constexpr uint8_t temperatureCalibrationPoints[] = { 60, 80, 100, 120, 140, 160 };
constexpr int8_t temperatureCalibrationOffsets[] = SENSOR_TEMPERATURE_OFFSETS_C;
constexpr uint8_t temperatureCalibrationCount = sizeof(temperatureCalibrationPoints);
static_assert(temperatureCalibrationCount == sizeof(temperatureCalibrationOffsets), "Each temperature point needs one offset");

constexpr bool hasTemperatureCalibration(uint8_t point = 0) {
    return point < temperatureCalibrationCount
        && (temperatureCalibrationOffsets[point] != 0 || hasTemperatureCalibration(point + 1));
}

float calibratedSensorTemperature(float sensorCelsius) {
    constexpr bool needsCorrection = hasTemperatureCalibration();
    if (!needsCorrection)
        return sensorCelsius;

    // The curve starts at (0 °C, 0 °C offset) and keeps the last offset above the final point.
    uint8_t low = 0;
    int8_t lowOffset = 0;
    for (uint8_t point = 0; point < temperatureCalibrationCount; ++point) {
        const uint8_t high = temperatureCalibrationPoints[point];
        const int8_t highOffset = temperatureCalibrationOffsets[point];
        if (sensorCelsius < high) {
            const float slope = (highOffset - lowOffset) / float(high - low);
            const float offset = slope * (sensorCelsius - low) + lowOffset;
            return sensorCelsius + offset;
        }
        low = high;
        lowOffset = highOffset;
    }
    return sensorCelsius + lowOffset;
}
}

bool TemperatureController::updateCurrentTemperature() {
    const uint8_t type = sensorType;
    // get raw temperature
    switch (type) {
    case 0:
        currentTemperature = 25;
        break;
    case 1:  // Epcos B57560G0107F000
    case 2:  // is 200k thermistor
    case 3:  // V2 Sensor Conrad Renkforce / mendel-parts thermistor (EPCOS G550) = NTC mit 100kOhm
    case 4:  // is 10k thermistor
    case 8:  // E3D Thermistor
    case 9:  // 100k Honeywell 135-104LAG-J01
    case 10: // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
    case 11: // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
    case 12: // 100k RS thermistor 198-961 (4.7k pullup)
    case 13: // NTC 3950 100k thermistor - Conrad V3
    case 14: // Thermistor NTC 3950 100k Ohm
    {
        currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - (HAL::temperatureAdcSample(sensorPin) >> (ANALOG_REDUCE_BITS)); // Convert to 10 bit result
        break;
    }
    case 53:  // E3D PT100 Board
    case 60:  // HEATER_USES_AD8495 (Delivers 5mV/degC)
    case 100: // AD595
    {
        currentTemperature = (HAL::temperatureAdcSample(sensorPin) >> (ANALOG_REDUCE_BITS));
        break;
    }

    default:
        // An unsupported ID may still be stored in EEPROM. Report a sensor fault.
        currentTemperature = 4095;
        currentTemperatureC = MAX_DEFECT_TEMPERATURE + 1.0f;
        return false;
    }

    switch (type) {
    case 0:
        currentTemperatureC = 25;
        break;
        //NTC
    case 1:
    case 2:
    case 3:
    case 4:
    case 8: //E3D Thermistor
    case 9:
    case 10:
    case 11:
    case 12:
    case 13: // Conrad V3
    case 14: // Thermistor NTC 3950 100k Ohm
    //PTC
    case 53: // E3D PT100 Board
    {
        const bool isNtc = type != 53;
        const uint8_t tableIndex = isNtc ? type - 1 : 14; // Sensor 53 uses table 15.

        uint8_t num = pgm_read_byte(&temptables_num[tableIndex]) << 1;
        uint8_t i = 2;
        const int16_t* temptable = (const int16_t*)pgm_read_word(&temptables[tableIndex]);
        int16_t oldraw = pgm_read_word(&temptable[0]);
        int16_t oldtemp = pgm_read_word(&temptable[1]);
        int16_t newtemp = 0;

        //NTC: Suche Temperatur andersrum, weil Steigung der Umrechnungskurve invertiert ist.
        if (isNtc)
            currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;

        while (i < num) {
            int16_t newraw = pgm_read_word(&temptable[i++]);
            newtemp = pgm_read_word(&temptable[i++]);
            if (newraw > currentTemperature) {
                const float sensorCelsius = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
                currentTemperatureC = calibratedSensorTemperature(sensorCelsius);
                return true;
            }
            oldtemp = newtemp;
            oldraw = newraw;
        }
        // Overflow: Set to last value in the table
        currentTemperatureC = calibratedSensorTemperature(TEMP_INT_TO_FLOAT(newtemp));

        break;
    }
    case 60: // AD8495 (Delivers 5mV/degC vs the AD595's 10mV)
    {
        currentTemperatureC = calibratedSensorTemperature((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));

        break;
    }
    case 100: // AD595
    {
        currentTemperatureC = calibratedSensorTemperature((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));

        break;
    }
    }
    return true;
} // updateCurrentTemperature
