// RFx000 internal EEPROM schema. No legacy migration. Header/feature slots
// occupy exactly the first KiB of the upper EEPROM bank. The remaining KiB
// is not owned by this service. Do not assign new IDs by reordering existing ones.
#ifndef RF_EEPROM_ZONE_LAYOUT_H
#define RF_EEPROM_ZONE_LAYOUT_H
#include <stdint.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

namespace EepromZones {
constexpr uint16_t Base = 0x0800;
constexpr uint16_t End = 0x0C00; // exclusive
constexpr uint8_t SlotSize = 32;
constexpr uint8_t PayloadOffset = 3;
constexpr uint8_t MaxPayload = 29;
constexpr uint16_t Schema = 1;
enum Block : uint8_t {
    Admin = 0,
    Geometry = 1,
    Feedrates = 2,
    Jerk = 3,
    Acceleration = 4,
    MotorCurrent = 5,
    Extruder0Motion = 6,
    Extruder1Motion = 7,
    Hotend0 = 8,
    Hotend1 = 9,
    BedPid = 10,
    BedSafety = 11,
    Preheat = 12,
    ZOffset = 13,
    SenseOffset = 14,
    Pressure = 15,
    Scan = 16,
    Wobble = 17,
    Panel = 18,
    Lighting = 19,
    PartFan = 20,
    System = 21,
    Usage = 22,
    Last = Usage
};
constexpr uint32_t bit(uint8_t id) { return id < 32 ? (uint32_t(1) << id) : 0; }
constexpr uint32_t Active = (bit(Last+1) - 1) & ~uint32_t(1);
constexpr uint32_t AllAlerts = Active | bit(Admin);
constexpr uint16_t address(uint8_t id) { return Base + (uint16_t(id) << 5); }
constexpr uint8_t blockOf(uint16_t pos) { return pos >= Base && pos < End ? uint8_t((pos-Base)>>5) : 255; }

inline uint8_t length(uint8_t id) {
#ifdef __AVR__
    static const uint8_t lengths[] PROGMEM = {0,29,24,8,26,5,29,29,20,20,19,1,8,4,7,12,6,7,6,20,4,12,8};
    return id <= Last ? pgm_read_byte(lengths + id) : 0;
#else
    static const uint8_t lengths[] = {0,29,24,8,26,5,29,29,20,20,19,1,8,4,7,12,6,7,6,20,4,12,8};
    return id <= Last ? lengths[id] : 0;
#endif
}

// Physical addresses, not public M206 IDs until host compatibility is tested.
// Grouped arrays (motor currents, PID identity) start at their first byte.
enum class Field : uint16_t {
    XAXIS_STEPS_PER_MM = 0x0823,
    YAXIS_STEPS_PER_MM = 0x0827,
    ZAXIS_STEPS_PER_MM = 0x082B,
    X_LENGTH = 0x082F,
    Y_LENGTH = 0x0833,
    Z_LENGTH = 0x0837,
    RF_MICRO_STEPS_X = 0x083B,
    RF_MICRO_STEPS_Y = 0x083C,
    RF_MICRO_STEPS_Z = 0x083D,
    RF_MOD_Z_STEP_SIZE = 0x083E,
    X_MAX_FEEDRATE = 0x0843,
    Y_MAX_FEEDRATE = 0x0847,
    Z_MAX_FEEDRATE = 0x084B,
    X_HOMING_FEEDRATE_PRINT = 0x084F,
    Y_HOMING_FEEDRATE_PRINT = 0x0853,
    Z_HOMING_FEEDRATE_PRINT = 0x0857,
    MAX_XYJERK = 0x0863,
    MAX_ZJERK = 0x0867,
    X_MAX_ACCEL = 0x0883,
    Y_MAX_ACCEL = 0x0887,
    Z_MAX_ACCEL = 0x088B,
    X_MAX_TRAVEL_ACCEL = 0x088F,
    Y_MAX_TRAVEL_ACCEL = 0x0893,
    Z_MAX_TRAVEL_ACCEL = 0x0897,
    RF_STEP_PACKING_MIN_INTERVAL = 0x089B,
    RF_MOTOR_CURRENT = 0x08A3,
    E0_STEPS_PER_MM = 0x08C3,
    E0_MAX_FEEDRATE = 0x08C7,
    E0_MAX_START_FEEDRATE = 0x08CB,
    E0_MAX_ACCELERATION = 0x08CF,
    E0_ADVANCE_L = 0x08D3,
    E0_X_OFFSET = 0x08D7,
    E0_Y_OFFSET = 0x08DB,
    RF_MICRO_STEPS_E0 = 0x08DF,
    E1_STEPS_PER_MM = 0x08E3,
    E1_MAX_FEEDRATE = 0x08E7,
    E1_MAX_START_FEEDRATE = 0x08EB,
    E1_MAX_ACCELERATION = 0x08EF,
    E1_ADVANCE_L = 0x08F3,
    E1_X_OFFSET = 0x08F7,
    E1_Y_OFFSET = 0x08FB,
    RF_MICRO_STEPS_E1 = 0x08FF,
    E0_PID_PGAIN = 0x0903,
    E0_PID_IGAIN = 0x0907,
    E0_PID_DGAIN = 0x090B,
    E0_DRIVE_MIN = 0x090F,
    E0_DRIVE_MAX = 0x0910,
    E0_PID_MAX = 0x0911,
    E0_SENSOR_TYPE = 0x0912,
    PID_CALIBRATION_SLOT_0 = 0x0913,
    E0_COOLER_SPEED = 0x0916,
    E1_PID_PGAIN = 0x0923,
    E1_PID_IGAIN = 0x0927,
    E1_PID_DGAIN = 0x092B,
    E1_DRIVE_MIN = 0x092F,
    E1_DRIVE_MAX = 0x0930,
    E1_PID_MAX = 0x0931,
    E1_SENSOR_TYPE = 0x0932,
    PID_CALIBRATION_SLOT_1 = 0x0933,
    E1_COOLER_SPEED = 0x0936,
    BED_PID_PGAIN = 0x0943,
    BED_PID_IGAIN = 0x0947,
    BED_PID_DGAIN = 0x094B,
    BED_DRIVE_MIN = 0x094F,
    BED_DRIVE_MAX = 0x0950,
    BED_PID_MAX = 0x0951,
    RF_HEATED_BED_SENSOR_TYPE = 0x0952,
    PID_CALIBRATION_SLOT_2 = 0x0953,
    NEW_BED_EMERGENCY_C = 0x0963,
    RF_PREHEAT_A_BED = 0x0983,
    RF_PREHEAT_A_HOTEND = 0x0985,
    RF_PREHEAT_B_BED = 0x0987,
    RF_PREHEAT_B_HOTEND = 0x0989,
    RF_Z_OFFSET = 0x09A3,
    RF_MOD_SENSEOFFSET_OFFSET_MAX = 0x09C3,
    RF_MOD_SENSEOFFSET_DIGITS = 0x09C5,
    RF_MOD_SENSEOFFSET_AUTOSTART = 0x09C7,
    RF_ZERO_DIGIT_STATE = 0x09C8,
    RF_DIGIT_CMP_STATE = 0x09C9,
    RF_EMERGENCYPAUSEDIGITSMIN = 0x09E3,
    RF_EMERGENCYPAUSEDIGITSMAX = 0x09E7,
    RF_EMERGENCYZSTOPDIGITSMIN = 0x09EB,
    RF_EMERGENCYZSTOPDIGITSMAX = 0x09ED,
    RF_MOD_ZOS_SCAN_POINT_X = 0x0A03,
    RF_MOD_ZOS_SCAN_POINT_Y = 0x0A04,
    ZSCAN_START_MM = 0x0A05,
    RF_MOD_WOBBLE_FIX_PHASEXY = 0x0A23,
    RF_MOD_WOBBLE_FIX_AMPX = 0x0A24,
    RF_MOD_WOBBLE_FIX_AMPY1 = 0x0A26,
    RF_MOD_WOBBLE_FIX_AMPY2 = 0x0A28,
    RF_MOVE_MODE_X = 0x0A43,
    RF_MOVE_MODE_Y = 0x0A44,
    RF_MOVE_MODE_Z = 0x0A45,
    RF_Z_MODE = 0x0A46,
    RF_MOVE_MODE_XY_KOSYS = 0x0A47,
    RF_MOVE_POSITION_FEEDRATE = 0x0A48,
    RF_CASE_LIGHT_MODE = 0x0A63,
    RF_FET1_MODE = 0x0A64,
    RF_FET2_MODE = 0x0A65,
    RF_FET3_MODE = 0x0A66,
    RF_RGB_LIGHT_MODE = 0x0A67,
    RF_RGB_PRINTING_R = 0x0A68,
    RF_RGB_PRINTING_G = 0x0A69,
    RF_RGB_PRINTING_B = 0x0A6A,
    RF_RGB_HEATING_R = 0x0A6B,
    RF_RGB_HEATING_G = 0x0A6C,
    RF_RGB_HEATING_B = 0x0A6D,
    RF_RGB_COOLING_R = 0x0A6E,
    RF_RGB_COOLING_G = 0x0A6F,
    RF_RGB_COOLING_B = 0x0A70,
    RF_RGB_IDLE_R = 0x0A71,
    RF_RGB_IDLE_G = 0x0A72,
    RF_RGB_IDLE_B = 0x0A73,
    RF_RGB_MANUAL_R = 0x0A74,
    RF_RGB_MANUAL_G = 0x0A75,
    RF_RGB_MANUAL_B = 0x0A76,
    RF_FAN_MODE = 0x0A83,
    RF_PART_FAN_SPEED = 0x0A84,
    RF_PART_FAN_PWM_MIN = 0x0A85,
    RF_PART_FAN_PWM_MAX = 0x0A86,
    BAUDRATE = 0x0AA3,
    MAX_INACTIVE_TIME = 0x0AA7,
    STEPPER_INACTIVE_TIME = 0x0AAB,
    PRINTING_TIME = 0x0AC3,
    PRINTING_DISTANCE = 0x0AC7,
};
constexpr uint16_t address(Field f) { return static_cast<uint16_t>(f); }

namespace Header {
constexpr uint16_t Magic = Base;
constexpr uint16_t Version = Base+4;
constexpr uint16_t Profile = Base+6;
constexpr uint16_t Crc = Base+8;
constexpr uint16_t Master = Base+10;
constexpr uint16_t Intent = Base+14;
constexpr uint16_t IntentInverse = Base+18;
constexpr uint16_t State = Base+22;
constexpr uint16_t Alerts = Base+23;
constexpr uint16_t AlertsInverse = Base+27;
constexpr uint16_t StateInverse = Base+31;
constexpr uint32_t Signature = 0xB7584652UL; // LE bytes 'R','F','X',0xB7
constexpr uint8_t Clean = 0xA5;
constexpr uint8_t Preparing = 0x3C;
constexpr uint8_t Writing = 0x5A;
}
static_assert(address(32) == End, "settings bank boundary");
static_assert(address(Last) + SlotSize <= End, "feature slots fit");
static_assert(Header::StateInverse+1 == Base+SlotSize, "metadata confined to slot 00");
}
#endif
