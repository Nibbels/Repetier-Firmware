#ifndef DISPLAY_MODES_H
#define DISPLAY_MODES_H

// Shared menu-state bits used by the panel, print lifecycle and fan controls.
#define MENU_MODE_PAUSED 4
#define MENU_MODE_FAN_RUNNING 8
#define MENU_MODE_PRINTING 16
#define MENU_MODE_FAN_MODE_PDM 128

// Stored Z display values. The existing 1/2 toggle and default 3 are independent.
#define Z_VALUE_MODE_Z_MIN 1 // show the distance to Z-min
#define Z_VALUE_MODE_LAYER 3 // show the nominal layer height

#endif
