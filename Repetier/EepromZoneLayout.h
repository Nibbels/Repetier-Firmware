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
constexpr uint16_t Schema = 3; // pre-release: fixed application UART
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
constexpr uint32_t blockBit(uint8_t id) { return id < 32 ? (uint32_t(1) << id) : 0; }
constexpr uint32_t Active = (blockBit(Last+1) - 1) & ~uint32_t(1);
constexpr uint32_t AllAlerts = Active | blockBit(Admin);
constexpr uint16_t address(uint8_t id) { return Base + (uint16_t(id) << 5); }
constexpr uint8_t blockOf(uint16_t pos) { return pos >= Base && pos < End ? uint8_t((pos-Base)>>5) : 255; }

inline uint8_t length(uint8_t id) {
#ifdef __AVR__
    static const uint8_t lengths[] PROGMEM = {0,29,24,8,26,5,29,29,20,20,19,1,8,4,7,12,6,7,6,4,4,8,8};
    return id <= Last ? pgm_read_byte(lengths + id) : 0;
#else
    static const uint8_t lengths[] = {0,29,24,8,26,5,29,29,20,20,19,1,8,4,7,12,6,7,6,4,4,8,8};
    return id <= Last ? lengths[id] : 0;
#endif
}

// Physical addresses, not public M206 IDs until host compatibility is tested.
// Grouped arrays (motor currents, PID identity) start at their first byte.
enum class Field : uint16_t {
    F_XAXIS_STEPS_PER_MM = 0x0823,
    F_YAXIS_STEPS_PER_MM = 0x0827,
    F_ZAXIS_STEPS_PER_MM = 0x082B,
    F_X_LENGTH = 0x082F,
    F_Y_LENGTH = 0x0833,
    F_Z_LENGTH = 0x0837,
    F_RF_MICRO_STEPS_X = 0x083B,
    F_RF_MICRO_STEPS_Y = 0x083C,
    F_RF_MICRO_STEPS_Z = 0x083D,
    F_RF_MOD_Z_STEP_SIZE = 0x083E,
    F_X_MAX_FEEDRATE = 0x0843,
    F_Y_MAX_FEEDRATE = 0x0847,
    F_Z_MAX_FEEDRATE = 0x084B,
    F_X_HOMING_FEEDRATE_PRINT = 0x084F,
    F_Y_HOMING_FEEDRATE_PRINT = 0x0853,
    F_Z_HOMING_FEEDRATE_PRINT = 0x0857,
    F_MAX_XYJERK = 0x0863,
    F_MAX_ZJERK = 0x0867,
    F_X_MAX_ACCEL = 0x0883,
    F_Y_MAX_ACCEL = 0x0887,
    F_Z_MAX_ACCEL = 0x088B,
    F_X_MAX_TRAVEL_ACCEL = 0x088F,
    F_Y_MAX_TRAVEL_ACCEL = 0x0893,
    F_Z_MAX_TRAVEL_ACCEL = 0x0897,
    F_RF_STEP_PACKING_MIN_INTERVAL = 0x089B,
    F_RF_MOTOR_CURRENT = 0x08A3,
    F_E0_STEPS_PER_MM = 0x08C3,
    F_E0_MAX_FEEDRATE = 0x08C7,
    F_E0_MAX_START_FEEDRATE = 0x08CB,
    F_E0_MAX_ACCELERATION = 0x08CF,
    F_E0_ADVANCE_L = 0x08D3,
    F_E0_X_OFFSET = 0x08D7,
    F_E0_Y_OFFSET = 0x08DB,
    F_RF_MICRO_STEPS_E0 = 0x08DF,
    F_E1_STEPS_PER_MM = 0x08E3,
    F_E1_MAX_FEEDRATE = 0x08E7,
    F_E1_MAX_START_FEEDRATE = 0x08EB,
    F_E1_MAX_ACCELERATION = 0x08EF,
    F_E1_ADVANCE_L = 0x08F3,
    F_E1_X_OFFSET = 0x08F7,
    F_E1_Y_OFFSET = 0x08FB,
    F_RF_MICRO_STEPS_E1 = 0x08FF,
    F_E0_PID_PGAIN = 0x0903,
    F_E0_PID_IGAIN = 0x0907,
    F_E0_PID_DGAIN = 0x090B,
    F_E0_DRIVE_MIN = 0x090F,
    F_E0_DRIVE_MAX = 0x0910,
    F_E0_PID_MAX = 0x0911,
    F_E0_SENSOR_TYPE = 0x0912,
    F_PID_CALIBRATION_SLOT_0 = 0x0913,
    F_E0_COOLER_SPEED = 0x0916,
    F_E1_PID_PGAIN = 0x0923,
    F_E1_PID_IGAIN = 0x0927,
    F_E1_PID_DGAIN = 0x092B,
    F_E1_DRIVE_MIN = 0x092F,
    F_E1_DRIVE_MAX = 0x0930,
    F_E1_PID_MAX = 0x0931,
    F_E1_SENSOR_TYPE = 0x0932,
    F_PID_CALIBRATION_SLOT_1 = 0x0933,
    F_E1_COOLER_SPEED = 0x0936,
    F_BED_PID_PGAIN = 0x0943,
    F_BED_PID_IGAIN = 0x0947,
    F_BED_PID_DGAIN = 0x094B,
    F_BED_DRIVE_MIN = 0x094F,
    F_BED_DRIVE_MAX = 0x0950,
    F_BED_PID_MAX = 0x0951,
    F_RF_HEATED_BED_SENSOR_TYPE = 0x0952,
    F_PID_CALIBRATION_SLOT_2 = 0x0953,
    F_NEW_BED_EMERGENCY_C = 0x0963,
    F_RF_PREHEAT_A_BED = 0x0983,
    F_RF_PREHEAT_A_HOTEND = 0x0985,
    F_RF_PREHEAT_B_BED = 0x0987,
    F_RF_PREHEAT_B_HOTEND = 0x0989,
    F_RF_Z_OFFSET = 0x09A3,
    F_RF_MOD_SENSEOFFSET_OFFSET_MAX = 0x09C3,
    F_RF_MOD_SENSEOFFSET_DIGITS = 0x09C5,
    F_RF_MOD_SENSEOFFSET_AUTOSTART = 0x09C7,
    F_RF_ZERO_DIGIT_STATE = 0x09C8,
    F_RF_DIGIT_CMP_STATE = 0x09C9,
    F_RF_EMERGENCYPAUSEDIGITSMIN = 0x09E3,
    F_RF_EMERGENCYPAUSEDIGITSMAX = 0x09E7,
    F_RF_EMERGENCYZSTOPDIGITSMIN = 0x09EB,
    F_RF_EMERGENCYZSTOPDIGITSMAX = 0x09ED,
    F_RF_MOD_ZOS_SCAN_POINT_X = 0x0A03,
    F_RF_MOD_ZOS_SCAN_POINT_Y = 0x0A04,
    F_ZSCAN_START_MM = 0x0A05,
    F_RF_MOD_WOBBLE_FIX_PHASEXY = 0x0A23,
    F_RF_MOD_WOBBLE_FIX_AMPX = 0x0A24,
    F_RF_MOD_WOBBLE_FIX_AMPY1 = 0x0A26,
    F_RF_MOD_WOBBLE_FIX_AMPY2 = 0x0A28,
    F_RF_MOVE_MODE_X = 0x0A43,
    F_RF_MOVE_MODE_Y = 0x0A44,
    F_RF_MOVE_MODE_Z = 0x0A45,
    F_RF_Z_MODE = 0x0A46,
    F_RF_MOVE_MODE_XY_KOSYS = 0x0A47,
    F_RF_MOVE_POSITION_FEEDRATE = 0x0A48,
    F_RF_CASE_LIGHT_MODE = 0x0A63,
    F_RF_FET1_MODE = 0x0A64,
    F_RF_FET2_MODE = 0x0A65,
    F_RF_FET3_MODE = 0x0A66,
    // 0x0A67..0x0A76: free former RGB payload. Remaining slots unchanged.
    F_RF_FAN_MODE = 0x0A83,
    F_RF_PART_FAN_SPEED = 0x0A84,
    F_RF_PART_FAN_PWM_MIN = 0x0A85,
    F_RF_PART_FAN_PWM_MAX = 0x0A86,
    F_MAX_INACTIVE_TIME = 0x0AA3,
    F_STEPPER_INACTIVE_TIME = 0x0AA7,
    // 0x0AAB..0x0AAE: free former baud payload.
    F_PRINTING_TIME = 0x0AC3,
    F_PRINTING_DISTANCE = 0x0AC7,
};
constexpr uint16_t address(Field f) { return static_cast<uint16_t>(f); }


// Exact internal field boundaries. Array-valued groups are not arbitrary
// byte ranges: motor current allows a single channel or the full group;
// PID identity is written as a single three-byte value by its owner.
// Store exact immutable field starts and widths in Flash, rather than a
// compiler-generated dense switch table in SRAM. Each entry packs a
// ten-bit settings-bank offset and the width (1..4). This cold-path lookup
// performs at most seven table reads for the current inventory.
constexpr uint16_t widthEntry(Field field,uint8_t width) {
    return uint16_t((address(field)-Base) | (uint16_t(width)<<10));
}
constexpr uint16_t widthEntryOffset(uint16_t entry) {return entry & 0x03FF;}
constexpr bool widthEntriesSorted(const uint16_t* entries,uint8_t n) {
    return n<2 || (widthEntryOffset(entries[0])<widthEntryOffset(entries[1])
        && widthEntriesSorted(entries+1,uint8_t(n-1)));
}
inline uint8_t fieldWidth(uint16_t pos) {
    if(pos>=address(Field::F_RF_MOTOR_CURRENT) && pos<address(Field::F_RF_MOTOR_CURRENT)+5)
        return 1;
    if(pos<Base || pos>=End)return 0;
    static constexpr uint16_t entries[]
#ifdef __AVR__
        PROGMEM
#endif
        = {
        widthEntry(Field::F_XAXIS_STEPS_PER_MM,4),
        widthEntry(Field::F_YAXIS_STEPS_PER_MM,4),
        widthEntry(Field::F_ZAXIS_STEPS_PER_MM,4),
        widthEntry(Field::F_X_LENGTH,4),
        widthEntry(Field::F_Y_LENGTH,4),
        widthEntry(Field::F_Z_LENGTH,4),
        widthEntry(Field::F_RF_MICRO_STEPS_X,1),
        widthEntry(Field::F_RF_MICRO_STEPS_Y,1),
        widthEntry(Field::F_RF_MICRO_STEPS_Z,1),
        widthEntry(Field::F_RF_MOD_Z_STEP_SIZE,2),
        widthEntry(Field::F_X_MAX_FEEDRATE,4),
        widthEntry(Field::F_Y_MAX_FEEDRATE,4),
        widthEntry(Field::F_Z_MAX_FEEDRATE,4),
        widthEntry(Field::F_X_HOMING_FEEDRATE_PRINT,4),
        widthEntry(Field::F_Y_HOMING_FEEDRATE_PRINT,4),
        widthEntry(Field::F_Z_HOMING_FEEDRATE_PRINT,4),
        widthEntry(Field::F_MAX_XYJERK,4),
        widthEntry(Field::F_MAX_ZJERK,4),
        widthEntry(Field::F_X_MAX_ACCEL,4),
        widthEntry(Field::F_Y_MAX_ACCEL,4),
        widthEntry(Field::F_Z_MAX_ACCEL,4),
        widthEntry(Field::F_X_MAX_TRAVEL_ACCEL,4),
        widthEntry(Field::F_Y_MAX_TRAVEL_ACCEL,4),
        widthEntry(Field::F_Z_MAX_TRAVEL_ACCEL,4),
        widthEntry(Field::F_RF_STEP_PACKING_MIN_INTERVAL,2),
        widthEntry(Field::F_E0_STEPS_PER_MM,4),
        widthEntry(Field::F_E0_MAX_FEEDRATE,4),
        widthEntry(Field::F_E0_MAX_START_FEEDRATE,4),
        widthEntry(Field::F_E0_MAX_ACCELERATION,4),
        widthEntry(Field::F_E0_ADVANCE_L,4),
        widthEntry(Field::F_E0_X_OFFSET,4),
        widthEntry(Field::F_E0_Y_OFFSET,4),
        widthEntry(Field::F_RF_MICRO_STEPS_E0,1),
        widthEntry(Field::F_E1_STEPS_PER_MM,4),
        widthEntry(Field::F_E1_MAX_FEEDRATE,4),
        widthEntry(Field::F_E1_MAX_START_FEEDRATE,4),
        widthEntry(Field::F_E1_MAX_ACCELERATION,4),
        widthEntry(Field::F_E1_ADVANCE_L,4),
        widthEntry(Field::F_E1_X_OFFSET,4),
        widthEntry(Field::F_E1_Y_OFFSET,4),
        widthEntry(Field::F_RF_MICRO_STEPS_E1,1),
        widthEntry(Field::F_E0_PID_PGAIN,4),
        widthEntry(Field::F_E0_PID_IGAIN,4),
        widthEntry(Field::F_E0_PID_DGAIN,4),
        widthEntry(Field::F_E0_DRIVE_MIN,1),
        widthEntry(Field::F_E0_DRIVE_MAX,1),
        widthEntry(Field::F_E0_PID_MAX,1),
        widthEntry(Field::F_E0_SENSOR_TYPE,1),
        widthEntry(Field::F_PID_CALIBRATION_SLOT_0,3),
        widthEntry(Field::F_E0_COOLER_SPEED,1),
        widthEntry(Field::F_E1_PID_PGAIN,4),
        widthEntry(Field::F_E1_PID_IGAIN,4),
        widthEntry(Field::F_E1_PID_DGAIN,4),
        widthEntry(Field::F_E1_DRIVE_MIN,1),
        widthEntry(Field::F_E1_DRIVE_MAX,1),
        widthEntry(Field::F_E1_PID_MAX,1),
        widthEntry(Field::F_E1_SENSOR_TYPE,1),
        widthEntry(Field::F_PID_CALIBRATION_SLOT_1,3),
        widthEntry(Field::F_E1_COOLER_SPEED,1),
        widthEntry(Field::F_BED_PID_PGAIN,4),
        widthEntry(Field::F_BED_PID_IGAIN,4),
        widthEntry(Field::F_BED_PID_DGAIN,4),
        widthEntry(Field::F_BED_DRIVE_MIN,1),
        widthEntry(Field::F_BED_DRIVE_MAX,1),
        widthEntry(Field::F_BED_PID_MAX,1),
        widthEntry(Field::F_RF_HEATED_BED_SENSOR_TYPE,1),
        widthEntry(Field::F_PID_CALIBRATION_SLOT_2,3),
        widthEntry(Field::F_NEW_BED_EMERGENCY_C,1),
        widthEntry(Field::F_RF_PREHEAT_A_BED,2),
        widthEntry(Field::F_RF_PREHEAT_A_HOTEND,2),
        widthEntry(Field::F_RF_PREHEAT_B_BED,2),
        widthEntry(Field::F_RF_PREHEAT_B_HOTEND,2),
        widthEntry(Field::F_RF_Z_OFFSET,4),
        widthEntry(Field::F_RF_MOD_SENSEOFFSET_OFFSET_MAX,2),
        widthEntry(Field::F_RF_MOD_SENSEOFFSET_DIGITS,2),
        widthEntry(Field::F_RF_MOD_SENSEOFFSET_AUTOSTART,1),
        widthEntry(Field::F_RF_ZERO_DIGIT_STATE,1),
        widthEntry(Field::F_RF_DIGIT_CMP_STATE,1),
        widthEntry(Field::F_RF_EMERGENCYPAUSEDIGITSMIN,4),
        widthEntry(Field::F_RF_EMERGENCYPAUSEDIGITSMAX,4),
        widthEntry(Field::F_RF_EMERGENCYZSTOPDIGITSMIN,2),
        widthEntry(Field::F_RF_EMERGENCYZSTOPDIGITSMAX,2),
        widthEntry(Field::F_RF_MOD_ZOS_SCAN_POINT_X,1),
        widthEntry(Field::F_RF_MOD_ZOS_SCAN_POINT_Y,1),
        widthEntry(Field::F_ZSCAN_START_MM,4),
        widthEntry(Field::F_RF_MOD_WOBBLE_FIX_PHASEXY,1),
        widthEntry(Field::F_RF_MOD_WOBBLE_FIX_AMPX,2),
        widthEntry(Field::F_RF_MOD_WOBBLE_FIX_AMPY1,2),
        widthEntry(Field::F_RF_MOD_WOBBLE_FIX_AMPY2,2),
        widthEntry(Field::F_RF_MOVE_MODE_X,1),
        widthEntry(Field::F_RF_MOVE_MODE_Y,1),
        widthEntry(Field::F_RF_MOVE_MODE_Z,1),
        widthEntry(Field::F_RF_Z_MODE,1),
        widthEntry(Field::F_RF_MOVE_MODE_XY_KOSYS,1),
        widthEntry(Field::F_RF_MOVE_POSITION_FEEDRATE,1),
        widthEntry(Field::F_RF_CASE_LIGHT_MODE,1),
        widthEntry(Field::F_RF_FET1_MODE,1),
        widthEntry(Field::F_RF_FET2_MODE,1),
        widthEntry(Field::F_RF_FET3_MODE,1),
        widthEntry(Field::F_RF_FAN_MODE,1),
        widthEntry(Field::F_RF_PART_FAN_SPEED,1),
        widthEntry(Field::F_RF_PART_FAN_PWM_MIN,1),
        widthEntry(Field::F_RF_PART_FAN_PWM_MAX,1),
        widthEntry(Field::F_MAX_INACTIVE_TIME,4),
        widthEntry(Field::F_STEPPER_INACTIVE_TIME,4),
        widthEntry(Field::F_PRINTING_TIME,4),
        widthEntry(Field::F_PRINTING_DISTANCE,4),
    };
    constexpr unsigned count=sizeof(entries)/sizeof(entries[0]);
    static_assert(count<=127,"field width search budget");
    static_assert(widthEntriesSorted(entries,uint8_t(count)),"field starts must be sorted and unique");
    const uint16_t offset=pos-Base;
    uint8_t first=0,last=uint8_t(count);
    while(first<last) {
        const uint8_t mid=first+(last-first)/2;
#ifdef __AVR__
        const uint16_t entry=pgm_read_word(entries+mid);
#else
        const uint16_t entry=entries[mid];
#endif
        const uint16_t candidate=widthEntryOffset(entry);
        if(offset==candidate)return uint8_t(entry>>10);
        if(offset<candidate)last=mid;
        else first=mid+1;
    }
    return 0;
}

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
