#ifndef BOARD_CONFIGURATION_H
#define BOARD_CONFIGURATION_H

// Resolve user configuration, model defaults and physical connector assignments.
#include "Constants.h"

//https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html#Stringizing for using number-#defines in string-#defines
#define xstr(a) str(a)
#define str(a) #a

#include "Configuration.h"

#if MOTHERBOARD == DEVICE_TYPE_RF1000
#include "RF1000.h"
#elif MOTHERBOARD == DEVICE_TYPE_RF2000 || MOTHERBOARD == DEVICE_TYPE_RF2000v2
#include "RF2000.h"
#else
#error Unsupported RFx000 model
#endif

#ifndef EXT0_HOTEND_PROFILE
#define EXT0_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#endif
#ifndef EXT1_HOTEND_PROFILE
#define EXT1_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#endif
#ifndef EXT0_TEMPSENSOR_TYPE
#define EXT0_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#endif
#ifndef EXT1_TEMPSENSOR_TYPE
#define EXT1_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#endif

#include "pins.h"

#endif // BOARD_CONFIGURATION_H
