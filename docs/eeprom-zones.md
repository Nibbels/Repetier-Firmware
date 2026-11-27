# New internal EEPROM architecture: owner decision and design candidate

**Owner decision 2026-11-27:** break the legacy EEPROM settings ABI. First installation of the new layout initializes ALL user settings from the new board/retrofit-specific firmware defaults, including old PID/DMAX, microsteps, mechanical calibration and usage counters. No legacy data migration. Later updates of the same new schema preserve valid new settings; no unconditional reset on every boot. This supersedes Task0034 A63/A68 v9->v10 preservation and the historic-address constraint in AGENTS/TASKS for this work only. Still preserve printing protocol, hardware support, simulation evidence and safety requirements. Do not edit the integration branch or other workers' production scope. No flash/merge/hardware tests authorized.

**Status:** architecture and field-level candidate only. No EEPROM writes, firmware change, actual address assignment, or measured runtime improvement yet. New production integration should be staged under Task0034 exclusive ownership (A70/A71 revised; separate EEPROM integration work package if needed), before A72/A73; do not start a parallel production worker. No further PID optimization.

## Evidence and capacity

Pinned Task0034 head 06c1956d41e00695d90a8be63ede749274ed4b10. CI artifact 15167769162 (tested merge 920309ae09b49d95c97eeaa559ba74312af0b725) contains the unchanged firmware sources used for the inventory; A69 changed only tooling/replay/docs. Source scan of 123 sketch files (read/write callers in Eeprom, GCodeMotion/Thermal, UiActions/Settings/Formatting, ui, PanelMotion, PreheatProfiles, PrintUsage, Printer, RF, RgbLighting, ZOffsetScan, HAL). Checked Eeprom.h, all direct HAL::eprGet/Set references, dynamic motor/microstep arrays, extruder offsets, RGB table and PID calibration slots. Current HAL EEPROM_OFFSET=0; Eeprom.cpp checksum and clear iterate 0..2047. Historical community_development 49a60a97eefb5dbd5b5eeea4344f090979c2d070 Eeprom.h/Eeprom.cpp/HAL.h also show offset 0, same 2048-byte checksum/clear range and known named fields below 2048. This does not prove arbitrary historic/third-party firmware, unrestricted legacy raw M206 or upload tool cannot touch upper bytes. Preserve legacy bank; verify actual historical access and EEPROM retention before promising downgrade compatibility. New firmware never reads or writes legacy settings; downgrade itself does not have the new safety guarantees. Re-upgrade of an existing valid new schema preserves that schema unless explicitly reset; physical printer changes made while using old firmware may require recommissioning.

ATmega2560 internal EEPROM 4096 bytes; prefer 0x0800..0x0FFF for the new schema, leave 0x0000..0x07FF untouched. 303 bytes of existing active persisted payload + 1 new bed limit = 304 bytes (largest two-extruder/RGB board, fixed physical heater/driver slots, inactive fields initialized canonically). Historical format markers and generic magic/checksum/version/count are not payload; replace with new header/metadata. Explicitly initialize all fields, including ones previously only saved via panel or G-code. No assumption that 0xFF is a valid default. Keep 230V mains off on boot and never persist its on state.

External I2C 0x50 bed matrix is a separate address space (1536-byte sectors, format 7, active slot). External 0x51 board identity on RF2000 family is not user settings and must not be erased. Proposed full configuration reset invalidates old compensation matrix eligibility and requires a fresh scan or an explicitly qualified compatible matrix policy; do not silently reuse old geometry/microstep calibration, do not erase physical board identity, do not destroy external matrix data merely to reserve internal addresses. RF1000 lacks board identity chip. Investigate separate new matrix format/generation policy under A72 without claiming migration.

## Candidate physical layout (absolute byte addresses)

| Range | Reserved bytes | Purpose |
|---|---:|---|
| 0x0800–0x083F | 64 | New magic, schema, board/profile incl. X44, initialized state, independent header integrity |
| 0x0840–0x087F | 64 | Ordered CRC16 directory: 21 entries = 42 B, rest reserve |
| 0x0880–0x08FF | 128 | Rotating master CRC32 records (e.g. 16 x 8 B, sequence, CRC, commit marker; exact encoding pending) |
| 0x0900–0x0B9F | 672 | 21 fixed 32-byte slots; only defined payload bytes covered |
| 0x0BA0–0x0FFF | 1120 | Reserve for durable transaction journal, redundant safety records, future blocks |

928 bytes reserved before journal, 1120 spare. Explicit fixed-width LE serialization, no raw C++ object layout/ABI dependence. Float binary32 (finite/range checks); CRC includes schema, block ID and exact length (not uninitialized padding); master covers ordered directory and header identity, domain separated. Slot boundaries permit 32-byte payload without per-slot metadata; block CRCs remain at fixed directory addresses. Table is candidate, not release ABI. Store immutable schema metadata in Flash; do not allocate large EEPROM mirror in scarce SRAM. Use bounded small buffers or streaming, no EEPROM in ISR.

| ID | Owner/function | Payload slot | CRC16 address | Used bytes |
|---:|---|---|---|---:|
| 01 | XYZ geometry and resolution | 0x0900–0x091F | 0x0840 | 29 |
| 02 | Axis speed and jerk | 0x0920–0x093F | 0x0842 | 32 |
| 03 | Acceleration and packing | 0x0940–0x095F | 0x0844 | 26 |
| 04 | Stepper current | 0x0960–0x097F | 0x0846 | 5 |
| 05 | Extruder 0 motion | 0x0980–0x099F | 0x0848 | 29 |
| 06 | Extruder 1 motion | 0x09A0–0x09BF | 0x084A | 29 |
| 07 | Hotend 0 PID and sensor | 0x09C0–0x09DF | 0x084C | 20 |
| 08 | Hotend 1 PID and sensor | 0x09E0–0x09FF | 0x084E | 20 |
| 09 | Bed PID and sensor | 0x0A00–0x0A1F | 0x0850 | 19 |
| 10 | Bed safety limit | 0x0A20–0x0A3F | 0x0852 | 1 |
| 11 | Preheat profiles | 0x0A40–0x0A5F | 0x0854 | 8 |
| 12 | Manual Z offset | 0x0A60–0x0A7F | 0x0856 | 4 |
| 13 | DMS and SenseOffset | 0x0A80–0x0A9F | 0x0858 | 7 |
| 14 | Pressure safety | 0x0AA0–0x0ABF | 0x085A | 12 |
| 15 | Scan settings | 0x0AC0–0x0ADF | 0x085C | 6 |
| 16 | Wobble compensation | 0x0AE0–0x0AFF | 0x085E | 7 |
| 17 | Panel and jog modes | 0x0B00–0x0B1F | 0x0860 | 6 |
| 18 | Lighting and accessories | 0x0B20–0x0B3F | 0x0862 | 20 |
| 19 | Part fan | 0x0B40–0x0B5F | 0x0864 | 4 |
| 20 | Serial and inactivity | 0x0B60–0x0B7F | 0x0866 | 12 |
| 21 | Usage counters | 0x0B80–0x0B9F | 0x0868 | 8 |

## Field-level address inventory

Widths refer to the current AVR storage types; arrays represented by grouped contiguous fields (motor current five bytes, PID calibration each 3-byte tag/sensor/power identity). New values use same widths initially to minimize semantic changes. Candidate offsets are deterministic; all mapped old payload bytes and new payload bytes were checked for overlap, slot size <=32, all EPR defines classified; 303 old payload bytes, 304 new. Firmware reachability and semantic tests on four builds still required.

#### 01 XYZ geometry and resolution

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_XAXIS_STEPS_PER_MM` | 3 | 0x0900 | 4 |
| `EPR_YAXIS_STEPS_PER_MM` | 7 | 0x0904 | 4 |
| `EPR_ZAXIS_STEPS_PER_MM` | 11 | 0x0908 | 4 |
| `EPR_X_LENGTH` | 145 | 0x090C | 4 |
| `EPR_Y_LENGTH` | 149 | 0x0910 | 4 |
| `EPR_Z_LENGTH` | 153 | 0x0914 | 4 |
| `EPR_RF_MICRO_STEPS_X` | 1943 | 0x0918 | 1 |
| `EPR_RF_MICRO_STEPS_Y` | 1944 | 0x0919 | 1 |
| `EPR_RF_MICRO_STEPS_Z` | 1945 | 0x091A | 1 |
| `EPR_RF_MOD_Z_STEP_SIZE` | 1900 | 0x091B | 2 |

#### 02 Axis speed and jerk

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_X_MAX_FEEDRATE` | 15 | 0x0920 | 4 |
| `EPR_Y_MAX_FEEDRATE` | 19 | 0x0924 | 4 |
| `EPR_Z_MAX_FEEDRATE` | 23 | 0x0928 | 4 |
| `EPR_X_HOMING_FEEDRATE_PRINT` | 27 | 0x092C | 4 |
| `EPR_Y_HOMING_FEEDRATE_PRINT` | 31 | 0x0930 | 4 |
| `EPR_Z_HOMING_FEEDRATE_PRINT` | 35 | 0x0934 | 4 |
| `EPR_MAX_XYJERK` | 39 | 0x0938 | 4 |
| `EPR_MAX_ZJERK` | 47 | 0x093C | 4 |

#### 03 Acceleration and packing

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_X_MAX_ACCEL` | 51 | 0x0940 | 4 |
| `EPR_Y_MAX_ACCEL` | 55 | 0x0944 | 4 |
| `EPR_Z_MAX_ACCEL` | 59 | 0x0948 | 4 |
| `EPR_X_MAX_TRAVEL_ACCEL` | 63 | 0x094C | 4 |
| `EPR_Y_MAX_TRAVEL_ACCEL` | 67 | 0x0950 | 4 |
| `EPR_Z_MAX_TRAVEL_ACCEL` | 71 | 0x0954 | 4 |
| `EPR_RF_STEP_PACKING_MIN_INTERVAL` | 1964 | 0x0958 | 2 |

#### 04 Stepper current

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOTOR_CURRENT` | 1923 | 0x0960 | 5 |

#### 05 Extruder 0 motion

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `E0:EPR_EXTRUDER_STEPS_PER_MM` | 200 | 0x0980 | 4 |
| `E0:EPR_EXTRUDER_MAX_FEEDRATE` | 204 | 0x0984 | 4 |
| `E0:EPR_EXTRUDER_MAX_START_FEEDRATE` | 208 | 0x0988 | 4 |
| `E0:EPR_EXTRUDER_MAX_ACCELERATION` | 212 | 0x098C | 4 |
| `E0:EPR_EXTRUDER_ADVANCE_L` | 246 | 0x0990 | 4 |
| `E0:EPR_EXTRUDER_X_OFFSET` | 231 | 0x0994 | 4 |
| `E0:EPR_EXTRUDER_Y_OFFSET` | 235 | 0x0998 | 4 |
| `EPR_RF_MICRO_STEPS_E0` | 1946 | 0x099C | 1 |

#### 06 Extruder 1 motion

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `E1:EPR_EXTRUDER_STEPS_PER_MM` | 300 | 0x09A0 | 4 |
| `E1:EPR_EXTRUDER_MAX_FEEDRATE` | 304 | 0x09A4 | 4 |
| `E1:EPR_EXTRUDER_MAX_START_FEEDRATE` | 308 | 0x09A8 | 4 |
| `E1:EPR_EXTRUDER_MAX_ACCELERATION` | 312 | 0x09AC | 4 |
| `E1:EPR_EXTRUDER_ADVANCE_L` | 346 | 0x09B0 | 4 |
| `E1:EPR_EXTRUDER_X_OFFSET` | 331 | 0x09B4 | 4 |
| `E1:EPR_EXTRUDER_Y_OFFSET` | 335 | 0x09B8 | 4 |
| `EPR_RF_MICRO_STEPS_E1` | 1947 | 0x09BC | 1 |

#### 07 Hotend 0 PID and sensor

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `E0:EPR_EXTRUDER_PID_PGAIN` | 218 | 0x09C0 | 4 |
| `E0:EPR_EXTRUDER_PID_IGAIN` | 222 | 0x09C4 | 4 |
| `E0:EPR_EXTRUDER_PID_DGAIN` | 226 | 0x09C8 | 4 |
| `E0:EPR_EXTRUDER_DRIVE_MIN` | 245 | 0x09CC | 1 |
| `E0:EPR_EXTRUDER_DRIVE_MAX` | 217 | 0x09CD | 1 |
| `E0:EPR_EXTRUDER_PID_MAX` | 230 | 0x09CE | 1 |
| `E0:EPR_EXTRUDER_SENSOR_TYPE` | 294 | 0x09CF | 1 |
| `PID_CALIBRATION_SLOT_0` | 1978 | 0x09D0 | 3 |
| `E0:EPR_EXTRUDER_COOLER_SPEED` | 254 | 0x09D3 | 1 |

#### 08 Hotend 1 PID and sensor

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `E1:EPR_EXTRUDER_PID_PGAIN` | 318 | 0x09E0 | 4 |
| `E1:EPR_EXTRUDER_PID_IGAIN` | 322 | 0x09E4 | 4 |
| `E1:EPR_EXTRUDER_PID_DGAIN` | 326 | 0x09E8 | 4 |
| `E1:EPR_EXTRUDER_DRIVE_MIN` | 345 | 0x09EC | 1 |
| `E1:EPR_EXTRUDER_DRIVE_MAX` | 317 | 0x09ED | 1 |
| `E1:EPR_EXTRUDER_PID_MAX` | 330 | 0x09EE | 1 |
| `E1:EPR_EXTRUDER_SENSOR_TYPE` | 394 | 0x09EF | 1 |
| `PID_CALIBRATION_SLOT_1` | 1981 | 0x09F0 | 3 |
| `E1:EPR_EXTRUDER_COOLER_SPEED` | 354 | 0x09F3 | 1 |

#### 09 Bed PID and sensor

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_BED_PID_PGAIN` | 108 | 0x0A00 | 4 |
| `EPR_BED_PID_IGAIN` | 112 | 0x0A04 | 4 |
| `EPR_BED_PID_DGAIN` | 116 | 0x0A08 | 4 |
| `EPR_BED_DRIVE_MIN` | 124 | 0x0A0C | 1 |
| `EPR_BED_DRIVE_MAX` | 107 | 0x0A0D | 1 |
| `EPR_BED_PID_MAX` | 120 | 0x0A0E | 1 |
| `EPR_RF_HEATED_BED_SENSOR_TYPE` | 1922 | 0x0A0F | 1 |
| `PID_CALIBRATION_SLOT_2` | 1984 | 0x0A10 | 3 |

#### 10 Bed safety limit

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `NEW_BED_EMERGENCY_C` | new | 0x0A20 | 1 |

#### 11 Preheat profiles

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_PREHEAT_A_BED` | 1970 | 0x0A40 | 2 |
| `EPR_RF_PREHEAT_A_HOTEND` | 1972 | 0x0A42 | 2 |
| `EPR_RF_PREHEAT_B_BED` | 1974 | 0x0A44 | 2 |
| `EPR_RF_PREHEAT_B_HOTEND` | 1976 | 0x0A46 | 2 |

#### 12 Manual Z offset

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_Z_OFFSET` | 1038 | 0x0A60 | 4 |

#### 13 DMS and SenseOffset

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX` | 1906 | 0x0A80 | 2 |
| `EPR_RF_MOD_SENSEOFFSET_DIGITS` | 1908 | 0x0A82 | 2 |
| `EPR_RF_MOD_SENSEOFFSET_AUTOSTART` | 1953 | 0x0A84 | 1 |
| `EPR_RF_ZERO_DIGIT_STATE` | 1937 | 0x0A85 | 1 |
| `EPR_RF_DIGIT_CMP_STATE` | 1938 | 0x0A86 | 1 |

#### 14 Pressure safety

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_EMERGENCYPAUSEDIGITSMIN` | 1910 | 0x0AA0 | 4 |
| `EPR_RF_EMERGENCYPAUSEDIGITSMAX` | 1914 | 0x0AA4 | 4 |
| `EPR_RF_EMERGENCYZSTOPDIGITSMIN` | 1918 | 0x0AA8 | 2 |
| `EPR_RF_EMERGENCYZSTOPDIGITSMAX` | 1920 | 0x0AAA | 2 |

#### 15 Scan settings

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_ZOS_SCAN_POINT_X` | 1904 | 0x0AC0 | 1 |
| `EPR_RF_MOD_ZOS_SCAN_POINT_Y` | 1905 | 0x0AC1 | 1 |
| `EPR_ZSCAN_START_MM` | 1949 | 0x0AC2 | 4 |

#### 16 Wobble compensation

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_WOBBLE_FIX_PHASEXY` | 1954 | 0x0AE0 | 1 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPX` | 1955 | 0x0AE1 | 2 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPY1` | 1957 | 0x0AE3 | 2 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPY2` | 1959 | 0x0AE5 | 2 |

#### 17 Panel and jog modes

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOVE_MODE_X` | 1042 | 0x0B00 | 1 |
| `EPR_RF_MOVE_MODE_Y` | 1043 | 0x0B01 | 1 |
| `EPR_RF_MOVE_MODE_Z` | 1044 | 0x0B02 | 1 |
| `EPR_RF_Z_MODE` | 1045 | 0x0B03 | 1 |
| `EPR_RF_MOVE_MODE_XY_KOSYS` | 1046 | 0x0B04 | 1 |
| `EPR_RF_MOVE_POSITION_FEEDRATE` | 1047 | 0x0B05 | 1 |

#### 18 Lighting and accessories

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_CASE_LIGHT_MODE` | 1001 | 0x0B20 | 1 |
| `EPR_RF_FET1_MODE` | 1020 | 0x0B21 | 1 |
| `EPR_RF_FET2_MODE` | 1021 | 0x0B22 | 1 |
| `EPR_RF_FET3_MODE` | 1022 | 0x0B23 | 1 |
| `EPR_RF_RGB_LIGHT_MODE` | 1019 | 0x0B24 | 1 |
| `EPR_RF_RGB_PRINTING_R` | 1023 | 0x0B25 | 1 |
| `EPR_RF_RGB_PRINTING_G` | 1024 | 0x0B26 | 1 |
| `EPR_RF_RGB_PRINTING_B` | 1025 | 0x0B27 | 1 |
| `EPR_RF_RGB_HEATING_R` | 1026 | 0x0B28 | 1 |
| `EPR_RF_RGB_HEATING_G` | 1027 | 0x0B29 | 1 |
| `EPR_RF_RGB_HEATING_B` | 1028 | 0x0B2A | 1 |
| `EPR_RF_RGB_COOLING_R` | 1029 | 0x0B2B | 1 |
| `EPR_RF_RGB_COOLING_G` | 1030 | 0x0B2C | 1 |
| `EPR_RF_RGB_COOLING_B` | 1031 | 0x0B2D | 1 |
| `EPR_RF_RGB_IDLE_R` | 1032 | 0x0B2E | 1 |
| `EPR_RF_RGB_IDLE_G` | 1033 | 0x0B2F | 1 |
| `EPR_RF_RGB_IDLE_B` | 1034 | 0x0B30 | 1 |
| `EPR_RF_RGB_MANUAL_R` | 1035 | 0x0B31 | 1 |
| `EPR_RF_RGB_MANUAL_G` | 1036 | 0x0B32 | 1 |
| `EPR_RF_RGB_MANUAL_B` | 1037 | 0x0B33 | 1 |

#### 19 Part fan

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_FAN_MODE` | 1930 | 0x0B40 | 1 |
| `EPR_RF_PART_FAN_SPEED` | 1963 | 0x0B41 | 1 |
| `EPR_RF_PART_FAN_PWM_MIN` | 1961 | 0x0B42 | 1 |
| `EPR_RF_PART_FAN_PWM_MAX` | 1962 | 0x0B43 | 1 |

#### 20 Serial and inactivity

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_BAUDRATE` | 75 | 0x0B60 | 4 |
| `EPR_MAX_INACTIVE_TIME` | 79 | 0x0B64 | 4 |
| `EPR_STEPPER_INACTIVE_TIME` | 83 | 0x0B68 | 4 |

#### 21 Usage counters

| Field (existing symbol unless NEW) | Old physical byte | New physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_PRINTING_TIME` | 125 | 0x0B80 | 4 |
| `EPR_PRINTING_DISTANCE` | 129 | 0x0B84 | 4 |

### Historical metadata and retired addresses

Do not migrate `EPR_MAGIC_BYTE`, `EPR_INTEGRITY_BYTE`, `EPR_VERSION`, `EPR_CHECK_NUM_EXTRUDERS`, `EPR_RF_MICRO_STEPS_USED`, `EPR_RF_PREHEAT_PROFILE_TAG`, `EPR_RF_PID_CALIBRATION`. New header/directory replaces legacy format/checksum/count; preheat and microstep validity provided by new block transaction/explicit initialized values. Retain the semantics of per-heater calibration validity within each PID block (three bytes; not a historical protocol version comparison). Do not carry retired/unused `EPR_ACCELERATION_TYPE`, `EPR_EXTRUDER_SPEED`, `EPR_BACKLASH_X`, `EPR_BACKLASH_Y`, `EPR_BACKLASH_Z`, `EPR_RF_230V_OUTPUT_MODE`, `EPR_RF_HOTEND_TYPE`, `EPR_EXTRUDER_ADVANCE_K` or historic reserved gaps. Mains output always off at boot. E1 and unsupported RGB/FET fields remain fixed-schema placeholders, canonical safe values, inaccessible where absent.

## Integrity: the master is NOT a substitute for payload verification

- Master CRC32 protects the ordered stored block CRC16 values and schema identity, not the payload itself. A flipped payload bit with unchanged directory leaves master valid; must recompute every block from actual bytes on boot/M501 before applying settings. Read defined payload once while validating/loading, not a 2048-byte scan; no unverified live settings. Valid master + invalid block means a local inconsistency; invalid master alone cannot locate a block or distinguish damaged master, CRC directory, interrupted transaction, or header. Compare every block plus header and journal; a matching CRC is not semantic validity or mathematical proof of no corruption.
- Normal transaction: validate typed proposed values and cross-field constraints; quiesce dependent outputs/motion, record durable in-progress metadata before first changed byte, reject reentrant mutation while yielding to essential services. Update only differing bytes, recompute affected block(s) on readback, verify stored CRC(s), read ordered directory (42 bytes), publish rotating master record commit marker last and verify before clearing transaction intent/allowing application. Do not recalculate unrelated block CRCs or blindly bless an unknown directory during a normal save. No-op writes neither data nor metadata. An M500 groups all changed blocks into one transaction/master commit; coalesce menu edits as appropriate without changing accepted save semantics. M501 checks first, M502 explicitly resets new settings, never updates old bank. A pending transaction and invalid master are not two independent damaged blocks.
- Durable transaction record needs target block bitmask, sequence/format/integrity and verified marker protocol; a power cut at ANY data, CRC, master, journal, or reset write may leave previous or new complete configuration, or explicit invalid/disabled state. A/B per critical record or bounded journal using spare space are candidates, not implemented. Do not choose an older higher cutoff on corruption; if a lower current cutoff cannot be proven, heat admission remains off until deliberate recommissioning. Master sequence wrap, torn markers and directory rollback must be modeled. Do not claim CRC detects arbitrary multi-bit corruption or hardware power-switch failure.
- Per-block CRC16 and rotating master CRC32 cost additional metadata writes versus a single 8-bit checksum. Read/CPU savings are large; physical write savings require per-byte update, transaction batching and no-op suppression. Master would otherwise be an EEPROM wear hotspot even though only one small payload changes. Evaluate ring rotation, journal/CRC cell wear, usage counters and menu key repeats, power-cut frequency, true EEPROM busy time and foreground yields. Current eprSetInt16/32/Float writes the entire changed multibyte value; use byte-level update semantics where safe, with verified asynchronous/nonblocking scheduling or bounded service gaps. No extra long interrupts-off region. Keep hot loop data in RAM, not EEPROM; no full 2048-byte iteration on ordinary change. A one-block update reads <=32 payload bytes + 42 directory bytes = <=74 core integrity bytes, plus header/journal/verification traffic, vs existing 2047 payload scan. Boot defined payload 304 B + directory 42 B + header/master/journal metadata. CRC CPU overhead and actual read/write traffic/latency still need measured evidence.
- A single verified ordinary-block fault: show persistent block-specific alert + serial log; stage board defaults for that block and require OK acknowledgment; retain all other valid values. On repeated boot before acknowledgement, alert persists; do not silently erase fault evidence. If >=2 independently confirmed invalid blocks after journal/master/header diagnosis, owner requests full new-bank factory reset, with clear multi-block alert and explicit commissioning. Do not trigger factory reset solely from master mismatch or one interrupted multi-block transaction. Before resetting, log affected zones/values only if independently trusted.
- Safety/commissioning exception: invalid heater sensor, PID/power identity, pressure, geometry/microstep or temperature-limit block must inhibit corresponding heat/motion until validated settings and required calibration/homing. Immutable measured bed ceiling 150 C, emergency setting default120 on explicit initial commissioning, max target = cutoff -10 K (0 for invalid/disabled); no automatic increase of a previously lower limit after corruption, M502 or multi-block reset. Fresh erased upper bank and erased previously commissioned bank are indistinguishable without external trusted provenance; explicit initial commissioning/limit confirmation rather than automatic heating. A general OK that acknowledges a reset is not approval to silently raise thermal cutoff. Even full factory reset requires deliberate thermal commissioning if previous setting is unknown. Keep independent measured absolute monitoring, no stale rearm. Old firmware after downgrade ignores these protections.
- Coupled fields: Microstep mode, steps/mm, Z jog conversion and geometry have atomic dependency closure; extruder motion steps and microstep must agree, pressure pair bounds and feedrate/homing pairs validated together; changed sensor/power invalidates PID calibration, changed geometry invalidates matrix/homing, preheat/scan admission rechecks current thermal cap. Multiple blocks may legitimately change in one transaction. Revalidate entire configuration before activation; no running job sees half-applied dependent fields.

## Host / UI API and tests

M205/M206 use typed explicit field whitelist, NOT raw arbitrary byte writes. New physical addresses are deliberately distinct from old P values, so reject old saved Host profiles instead of aliasing to a new setting. Current strict ASCII parser and handler restrict P<2048; update to new address range only after exact whitelist/width/validation. Existing GCode P is int32_t; verify Repetier-Host/Server M205 parsing and M206 ASCII/binary high address round-trip before choosing final public IDs. If host cannot support high addresses, use explicit versioned logical field IDs with nonoverlapping legacy acceptance or reject old configuration imports; never silently alias old raw writes. CRC directory, journal, header and calibration identity not directly writable. Keep regular command/ACK semantics, no Task0033 UART production work. No direct writes from menu/G-code/modules outside one persistence owner. Field validation used by host, panel and defaults/loading.

Tests before A70/A71 close: generated address/size/coverage assertions all boards; initial legacy bank preserved/new defaults including both heater slots and all panel-only values; no old import, same-schema update, wrong model/X44, arbitrary erased/corrupt upper bank, downgrade/re-upgrade; real firmware-backed power cut at every write and readback failure, CRC single-bit and selected multibit mutations, valid master corrupted payload counterexample, corrupted directory and master-only recovery, journal replay/sequence wrap, single/multi-block alerts and persistent acknowledgement; DMAX/PID survival of unrelated block changes and manual edits AFTER initial reset; M500/501/502, M205/206 old and new addresses, all actual direct writers, no-op/changed byte counts; critical block inhibits and no raised bed limit on corruption; measure worst-case service intervals and AVR flash/SRAM/stack on all four builds, including combined safety image. Existing PID simulations/history retained. No hardware runs.

## Handover

A69 CI 45930076134 on 06c1956d41e00695d90a8be63ede749274ed4b10 completed success per GitHub Actions; artifact inspection and final resource evidence remain separate. The new schema is a scope change requested by owner after that run. Existing A63/A68 small-record research retained as evidence but old-layout migration no longer target. Implementation not started in this documentation commit. Next: exact legacy upper-bound/Host high-address validation, transaction model with power-cut/wear analysis, canonical schema/accessor and all direct caller inventory, then integrated A70/A71 (no parallel production). A72/A73 require latest full four-board evidence. Do not merge/flash without separate approval.
