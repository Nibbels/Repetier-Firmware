# New internal EEPROM architecture: owner decision and design candidate

**Owner decision 2026-11-27:** break the legacy EEPROM settings ABI. First installation of the new layout initializes ALL user settings from the new board/retrofit-specific firmware defaults, including old PID/DMAX, microsteps, mechanical calibration and usage counters. No legacy data migration. Later updates of the same new schema preserve valid new settings; no unconditional reset on every boot. This supersedes Task0034 A63/A68 v9->v10 preservation and the historic-address constraint in AGENTS/TASKS for this work only. Still preserve printing protocol, hardware support, simulation evidence and safety requirements. Do not edit the integration branch or other workers' production scope. No flash/merge/hardware tests authorized.

**Status:** architecture and field-level candidate only. No EEPROM writes, firmware change, actual address assignment, or measured runtime improvement yet. New production integration should be staged under Task0034 exclusive ownership (A70/A71 revised; separate EEPROM integration work package if needed), before A72/A73; do not start a parallel production worker. No further PID optimization.

## Evidence and capacity

Pinned Task0034 head 06c1956d41e00695d90a8be63ede749274ed4b10. CI artifact 15167769162 (tested merge 920309ae09b49d95c97eeaa559ba74312af0b725) contains the unchanged firmware sources used for the inventory; A69 changed only tooling/replay/docs. Source scan of 123 sketch files (read/write callers in Eeprom, GCodeMotion/Thermal, UiActions/Settings/Formatting, ui, PanelMotion, PreheatProfiles, PrintUsage, Printer, RF, RgbLighting, ZOffsetScan, HAL). Checked Eeprom.h, all direct HAL::eprGet/Set references, dynamic motor/microstep arrays, extruder offsets, RGB table and PID calibration slots. Current HAL EEPROM_OFFSET=0; Eeprom.cpp checksum and clear iterate 0..2047. Historical community_development 49a60a97eefb5dbd5b5eeea4344f090979c2d070 Eeprom.h/Eeprom.cpp/HAL.h also show offset 0, same 2048-byte checksum/clear range and known named fields below 2048. Verified additional downgrade caveat: at historical commit 49a60a97eefb5dbd5b5eeea4344f090979c2d070, EEPROM::update in Repetier/Eeprom.cpp forwards M206 P directly to HAL::eprSetByte/Int16/Int32/Float without an address range check; HAL.h uses EEPROM_OFFSET=0. Thus legacy raw writes can touch the new bank even though ordinary settings and reset do not. Isolation is an address convention, not a hardware protection. Arbitrary historic/third-party firmware and upload-tool EEPROM retention still unqualified. Preserve legacy bank; verify actual historical access and EEPROM retention before promising downgrade compatibility. New firmware never reads or writes legacy settings; downgrade itself does not have the new safety guarantees. Re-upgrade of an existing valid new schema preserves that schema unless explicitly reset; physical printer changes made while using old firmware may require recommissioning.

ATmega2560 internal EEPROM 4096 bytes; prefer 0x0800..0x0FFF for the new schema, leave 0x0000..0x07FF untouched. 303 bytes of existing active persisted payload + 1 new bed limit = 304 bytes (largest two-extruder/RGB board, fixed physical heater/driver slots, inactive fields initialized canonically). Historical format markers and generic magic/checksum/version/count are not payload; replace with new header/metadata. Explicitly initialize all fields, including ones previously only saved via panel or G-code. No assumption that 0xFF is a valid default. Keep 230V mains off on boot and never persist its on state.

External I2C 0x50 bed matrix is a separate address space (1536-byte sectors, format 7, active slot). External 0x51 board identity on RF2000 family is not user settings and must not be erased. Proposed full configuration reset invalidates old compensation matrix eligibility and requires a fresh scan or an explicitly qualified compatible matrix policy; do not silently reuse old geometry/microstep calibration, do not erase physical board identity, do not destroy external matrix data merely to reserve internal addresses. RF1000 lacks board identity chip. Investigate separate new matrix format/generation policy under A72 without claiming migration.

## Compact 32-byte slot layout (owner refinement, 2026-11-27)

Supersedes earlier directory and second-half metadata proposals. Entire internal configuration service is confined to 0x0800..0x0BFF, first 1024 bytes of the new upper bank. Slot 00 owns all global metadata; 01..31 are feature slots. 0x0C00..0x0FFF is wholly unallocated, never reserved, scanned, erased or written by this service. Lower 0x0000..0x07FF remains legacy. No allocator, no directory, no EEPROM mirror. Block address = 0x0800 + (uint16_t(id) << 5); require id <32.

Feature slot: CRC16 offsets 0..1, length/state offset 2, payload offsets 3..31 (max29 bytes). Free payload = 29-length. Use schema-known exact length before reads; CRC covers schema/block ID/length/payload, not padding. 0xFF uninitialized; 0xFE incomplete candidate; other invalid lengths rejected. No redundant free-space count. Use static schema metadata in Flash. Explicit fixed-width LE encoding, finite/range checks, no raw object ABI. The 32-byte speed+jerk payload was split into speed/homing 24 B and jerk 8 B. Larger features may use independent blocks or one coupled multi-block transaction.

| Range | Bytes | Purpose |
|---|---:|---|
| 0x0000–0x07FF | 2048 | Legacy bank, never loaded by new firmware |
| 0x0800–0x081F | 32 | Block 00, global metadata |
| 0x0820–0x0ADF | 704 | 22 defined feature slots |
| 0x0AE0–0x0BFF | 288 | Nine unallocated feature slots |
| 0x0C00–0x0FFF | 1024 | Unallocated independent future space |

304 payload + 66 feature CRC/length bytes; 23 occupied slots (736 B), 9 spare. Master reads 44 CRC bytes at fixed stride. No routine wear leveling or deferred write machinery; owner reports edits rare, preserve immediate saves for manual Z-offset, DMAX and motor current. Optimize no-ops, per-byte difference and grouped writes. Evaluate any coalescing only for demonstrated repeated writes, never change accepted durability semantics silently.

### Block 00 candidate (32 bytes, not yet implemented)

| Offset | Bytes | Purpose |
|---|---:|---|
| 0..3 | 4 | New magic, initialized last |
| 4..5 | 2 | Schema version |
| 6..7 | 2 | Board/retrofit profile identity |
| 8..9 | 2 | Static header CRC16 (offsets 0..7) |
| 10..13 | 4 | Master CRC32 over header identity and ordered feature CRC16 values |
| 14..17 | 4 | Multi-block transaction bitmask (bits 1..31) |
| 18..21 | 4 | Complement of transaction mask |
| 22 | 1 | Transaction state/commit marker |
| 23..26 | 4 | Pending block-alert mask |
| 27..30 | 4 | Complement of alert mask |
| 31 | 1 | Complement of transaction state |

This is a compact candidate, not qualified encoding. No master rotation, no external journal or A/B records reserved. Durable multi-block intent must precede first changed payload; complete master before clearing intent. Single-block marker may suffice if power-cut replay demonstrates safe recovery; do not write redundant metadata unnecessarily. Interrupted or malformed intent must be treated conservatively (inhibit dependent output and report ambiguity), never silently certify mixed settings. A damaged alert mask must produce a persistent conservative alert, not lose required acknowledgement. Verify all real power-cut positions before accepting this encoding. If it cannot meet safety requirements within block00, use an additional normal feature slot in the first KiB only after explicit justification, not the free upper half. No automatic elevation of unknown individual bed cutoff. Full reset/initialization must keep magic invalid until complete, never erase unused slots or unrelated banks. Master protects directory-equivalent CRC values only; boot/M501 still recompute payload CRCs.

| ID | Owner/function | Physical slot | Payload bytes | Free payload |
|---:|---|---|---:|---:|
| 01 | XYZ geometry and resolution | 0x0820–0x083F | 29 | 0 |
| 02 | Axis max and homing feedrates | 0x0840–0x085F | 24 | 5 |
| 03 | Axis jerk | 0x0860–0x087F | 8 | 21 |
| 04 | Acceleration and packing | 0x0880–0x089F | 26 | 3 |
| 05 | Stepper current | 0x08A0–0x08BF | 5 | 24 |
| 06 | Extruder 0 motion | 0x08C0–0x08DF | 29 | 0 |
| 07 | Extruder 1 motion | 0x08E0–0x08FF | 29 | 0 |
| 08 | Hotend 0 PID and sensor | 0x0900–0x091F | 20 | 9 |
| 09 | Hotend 1 PID and sensor | 0x0920–0x093F | 20 | 9 |
| 10 | Bed PID and sensor | 0x0940–0x095F | 19 | 10 |
| 11 | Bed safety limit | 0x0960–0x097F | 1 | 28 |
| 12 | Preheat profiles | 0x0980–0x099F | 8 | 21 |
| 13 | Manual Z offset | 0x09A0–0x09BF | 4 | 25 |
| 14 | DMS and SenseOffset | 0x09C0–0x09DF | 7 | 22 |
| 15 | Pressure safety | 0x09E0–0x09FF | 12 | 17 |
| 16 | Scan settings | 0x0A00–0x0A1F | 6 | 23 |
| 17 | Wobble compensation | 0x0A20–0x0A3F | 7 | 22 |
| 18 | Panel and jog modes | 0x0A40–0x0A5F | 6 | 23 |
| 19 | Lighting and accessories | 0x0A60–0x0A7F | 20 | 9 |
| 20 | Part fan | 0x0A80–0x0A9F | 4 | 25 |
| 21 | Serial and inactivity | 0x0AA0–0x0ABF | 12 | 17 |
| 22 | Usage counters | 0x0AC0–0x0ADF | 8 | 21 |

## Field-level address inventory (block00 candidate)

125 field rows; 303 old payload bytes + 1 new bed limit. Exact overlap and bounds validation executed in JS: 22 feature slots, 304 unique addresses, no overlap, max length29; nine spare slots. Fixed inactive physical heater/driver fields canonicalized. Widths initially match existing AVR storage types; slot metadata inaccessible to host writes. IDs and addresses below supersede previous proposals.

#### 01 XYZ geometry and resolution

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_XAXIS_STEPS_PER_MM` | 3 | 0x0823 | 4 |
| `EPR_YAXIS_STEPS_PER_MM` | 7 | 0x0827 | 4 |
| `EPR_ZAXIS_STEPS_PER_MM` | 11 | 0x082B | 4 |
| `EPR_X_LENGTH` | 145 | 0x082F | 4 |
| `EPR_Y_LENGTH` | 149 | 0x0833 | 4 |
| `EPR_Z_LENGTH` | 153 | 0x0837 | 4 |
| `EPR_RF_MICRO_STEPS_X` | 1943 | 0x083B | 1 |
| `EPR_RF_MICRO_STEPS_Y` | 1944 | 0x083C | 1 |
| `EPR_RF_MICRO_STEPS_Z` | 1945 | 0x083D | 1 |
| `EPR_RF_MOD_Z_STEP_SIZE` | 1900 | 0x083E | 2 |

#### 02 Axis max and homing feedrates

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_X_MAX_FEEDRATE` | 15 | 0x0843 | 4 |
| `EPR_Y_MAX_FEEDRATE` | 19 | 0x0847 | 4 |
| `EPR_Z_MAX_FEEDRATE` | 23 | 0x084B | 4 |
| `EPR_X_HOMING_FEEDRATE_PRINT` | 27 | 0x084F | 4 |
| `EPR_Y_HOMING_FEEDRATE_PRINT` | 31 | 0x0853 | 4 |
| `EPR_Z_HOMING_FEEDRATE_PRINT` | 35 | 0x0857 | 4 |

#### 03 Axis jerk

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_MAX_XYJERK` | 39 | 0x0863 | 4 |
| `EPR_MAX_ZJERK` | 47 | 0x0867 | 4 |

#### 04 Acceleration and packing

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_X_MAX_ACCEL` | 51 | 0x0883 | 4 |
| `EPR_Y_MAX_ACCEL` | 55 | 0x0887 | 4 |
| `EPR_Z_MAX_ACCEL` | 59 | 0x088B | 4 |
| `EPR_X_MAX_TRAVEL_ACCEL` | 63 | 0x088F | 4 |
| `EPR_Y_MAX_TRAVEL_ACCEL` | 67 | 0x0893 | 4 |
| `EPR_Z_MAX_TRAVEL_ACCEL` | 71 | 0x0897 | 4 |
| `EPR_RF_STEP_PACKING_MIN_INTERVAL` | 1964 | 0x089B | 2 |

#### 05 Stepper current

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOTOR_CURRENT` | 1923 | 0x08A3 | 5 |

#### 06 Extruder 0 motion

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E0:EPR_EXTRUDER_STEPS_PER_MM` | 200 | 0x08C3 | 4 |
| `E0:EPR_EXTRUDER_MAX_FEEDRATE` | 204 | 0x08C7 | 4 |
| `E0:EPR_EXTRUDER_MAX_START_FEEDRATE` | 208 | 0x08CB | 4 |
| `E0:EPR_EXTRUDER_MAX_ACCELERATION` | 212 | 0x08CF | 4 |
| `E0:EPR_EXTRUDER_ADVANCE_L` | 246 | 0x08D3 | 4 |
| `E0:EPR_EXTRUDER_X_OFFSET` | 231 | 0x08D7 | 4 |
| `E0:EPR_EXTRUDER_Y_OFFSET` | 235 | 0x08DB | 4 |
| `EPR_RF_MICRO_STEPS_E0` | 1946 | 0x08DF | 1 |

#### 07 Extruder 1 motion

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E1:EPR_EXTRUDER_STEPS_PER_MM` | 300 | 0x08E3 | 4 |
| `E1:EPR_EXTRUDER_MAX_FEEDRATE` | 304 | 0x08E7 | 4 |
| `E1:EPR_EXTRUDER_MAX_START_FEEDRATE` | 308 | 0x08EB | 4 |
| `E1:EPR_EXTRUDER_MAX_ACCELERATION` | 312 | 0x08EF | 4 |
| `E1:EPR_EXTRUDER_ADVANCE_L` | 346 | 0x08F3 | 4 |
| `E1:EPR_EXTRUDER_X_OFFSET` | 331 | 0x08F7 | 4 |
| `E1:EPR_EXTRUDER_Y_OFFSET` | 335 | 0x08FB | 4 |
| `EPR_RF_MICRO_STEPS_E1` | 1947 | 0x08FF | 1 |

#### 08 Hotend 0 PID and sensor

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E0:EPR_EXTRUDER_PID_PGAIN` | 218 | 0x0903 | 4 |
| `E0:EPR_EXTRUDER_PID_IGAIN` | 222 | 0x0907 | 4 |
| `E0:EPR_EXTRUDER_PID_DGAIN` | 226 | 0x090B | 4 |
| `E0:EPR_EXTRUDER_DRIVE_MIN` | 245 | 0x090F | 1 |
| `E0:EPR_EXTRUDER_DRIVE_MAX` | 217 | 0x0910 | 1 |
| `E0:EPR_EXTRUDER_PID_MAX` | 230 | 0x0911 | 1 |
| `E0:EPR_EXTRUDER_SENSOR_TYPE` | 294 | 0x0912 | 1 |
| `PID_CALIBRATION_SLOT_0` | 1978 | 0x0913 | 3 |
| `E0:EPR_EXTRUDER_COOLER_SPEED` | 254 | 0x0916 | 1 |

#### 09 Hotend 1 PID and sensor

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E1:EPR_EXTRUDER_PID_PGAIN` | 318 | 0x0923 | 4 |
| `E1:EPR_EXTRUDER_PID_IGAIN` | 322 | 0x0927 | 4 |
| `E1:EPR_EXTRUDER_PID_DGAIN` | 326 | 0x092B | 4 |
| `E1:EPR_EXTRUDER_DRIVE_MIN` | 345 | 0x092F | 1 |
| `E1:EPR_EXTRUDER_DRIVE_MAX` | 317 | 0x0930 | 1 |
| `E1:EPR_EXTRUDER_PID_MAX` | 330 | 0x0931 | 1 |
| `E1:EPR_EXTRUDER_SENSOR_TYPE` | 394 | 0x0932 | 1 |
| `PID_CALIBRATION_SLOT_1` | 1981 | 0x0933 | 3 |
| `E1:EPR_EXTRUDER_COOLER_SPEED` | 354 | 0x0936 | 1 |

#### 10 Bed PID and sensor

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_BED_PID_PGAIN` | 108 | 0x0943 | 4 |
| `EPR_BED_PID_IGAIN` | 112 | 0x0947 | 4 |
| `EPR_BED_PID_DGAIN` | 116 | 0x094B | 4 |
| `EPR_BED_DRIVE_MIN` | 124 | 0x094F | 1 |
| `EPR_BED_DRIVE_MAX` | 107 | 0x0950 | 1 |
| `EPR_BED_PID_MAX` | 120 | 0x0951 | 1 |
| `EPR_RF_HEATED_BED_SENSOR_TYPE` | 1922 | 0x0952 | 1 |
| `PID_CALIBRATION_SLOT_2` | 1984 | 0x0953 | 3 |

#### 11 Bed safety limit

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `NEW_BED_EMERGENCY_C` | new | 0x0963 | 1 |

#### 12 Preheat profiles

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_PREHEAT_A_BED` | 1970 | 0x0983 | 2 |
| `EPR_RF_PREHEAT_A_HOTEND` | 1972 | 0x0985 | 2 |
| `EPR_RF_PREHEAT_B_BED` | 1974 | 0x0987 | 2 |
| `EPR_RF_PREHEAT_B_HOTEND` | 1976 | 0x0989 | 2 |

#### 13 Manual Z offset

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_Z_OFFSET` | 1038 | 0x09A3 | 4 |

#### 14 DMS and SenseOffset

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX` | 1906 | 0x09C3 | 2 |
| `EPR_RF_MOD_SENSEOFFSET_DIGITS` | 1908 | 0x09C5 | 2 |
| `EPR_RF_MOD_SENSEOFFSET_AUTOSTART` | 1953 | 0x09C7 | 1 |
| `EPR_RF_ZERO_DIGIT_STATE` | 1937 | 0x09C8 | 1 |
| `EPR_RF_DIGIT_CMP_STATE` | 1938 | 0x09C9 | 1 |

#### 15 Pressure safety

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_EMERGENCYPAUSEDIGITSMIN` | 1910 | 0x09E3 | 4 |
| `EPR_RF_EMERGENCYPAUSEDIGITSMAX` | 1914 | 0x09E7 | 4 |
| `EPR_RF_EMERGENCYZSTOPDIGITSMIN` | 1918 | 0x09EB | 2 |
| `EPR_RF_EMERGENCYZSTOPDIGITSMAX` | 1920 | 0x09ED | 2 |

#### 16 Scan settings

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_ZOS_SCAN_POINT_X` | 1904 | 0x0A03 | 1 |
| `EPR_RF_MOD_ZOS_SCAN_POINT_Y` | 1905 | 0x0A04 | 1 |
| `EPR_ZSCAN_START_MM` | 1949 | 0x0A05 | 4 |

#### 17 Wobble compensation

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_WOBBLE_FIX_PHASEXY` | 1954 | 0x0A23 | 1 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPX` | 1955 | 0x0A24 | 2 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPY1` | 1957 | 0x0A26 | 2 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPY2` | 1959 | 0x0A28 | 2 |

#### 18 Panel and jog modes

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOVE_MODE_X` | 1042 | 0x0A43 | 1 |
| `EPR_RF_MOVE_MODE_Y` | 1043 | 0x0A44 | 1 |
| `EPR_RF_MOVE_MODE_Z` | 1044 | 0x0A45 | 1 |
| `EPR_RF_Z_MODE` | 1045 | 0x0A46 | 1 |
| `EPR_RF_MOVE_MODE_XY_KOSYS` | 1046 | 0x0A47 | 1 |
| `EPR_RF_MOVE_POSITION_FEEDRATE` | 1047 | 0x0A48 | 1 |

#### 19 Lighting and accessories

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_CASE_LIGHT_MODE` | 1001 | 0x0A63 | 1 |
| `EPR_RF_FET1_MODE` | 1020 | 0x0A64 | 1 |
| `EPR_RF_FET2_MODE` | 1021 | 0x0A65 | 1 |
| `EPR_RF_FET3_MODE` | 1022 | 0x0A66 | 1 |
| `EPR_RF_RGB_LIGHT_MODE` | 1019 | 0x0A67 | 1 |
| `EPR_RF_RGB_PRINTING_R` | 1023 | 0x0A68 | 1 |
| `EPR_RF_RGB_PRINTING_G` | 1024 | 0x0A69 | 1 |
| `EPR_RF_RGB_PRINTING_B` | 1025 | 0x0A6A | 1 |
| `EPR_RF_RGB_HEATING_R` | 1026 | 0x0A6B | 1 |
| `EPR_RF_RGB_HEATING_G` | 1027 | 0x0A6C | 1 |
| `EPR_RF_RGB_HEATING_B` | 1028 | 0x0A6D | 1 |
| `EPR_RF_RGB_COOLING_R` | 1029 | 0x0A6E | 1 |
| `EPR_RF_RGB_COOLING_G` | 1030 | 0x0A6F | 1 |
| `EPR_RF_RGB_COOLING_B` | 1031 | 0x0A70 | 1 |
| `EPR_RF_RGB_IDLE_R` | 1032 | 0x0A71 | 1 |
| `EPR_RF_RGB_IDLE_G` | 1033 | 0x0A72 | 1 |
| `EPR_RF_RGB_IDLE_B` | 1034 | 0x0A73 | 1 |
| `EPR_RF_RGB_MANUAL_R` | 1035 | 0x0A74 | 1 |
| `EPR_RF_RGB_MANUAL_G` | 1036 | 0x0A75 | 1 |
| `EPR_RF_RGB_MANUAL_B` | 1037 | 0x0A76 | 1 |

#### 20 Part fan

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_FAN_MODE` | 1930 | 0x0A83 | 1 |
| `EPR_RF_PART_FAN_SPEED` | 1963 | 0x0A84 | 1 |
| `EPR_RF_PART_FAN_PWM_MIN` | 1961 | 0x0A85 | 1 |
| `EPR_RF_PART_FAN_PWM_MAX` | 1962 | 0x0A86 | 1 |

#### 21 Serial and inactivity

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_BAUDRATE` | 75 | 0x0AA3 | 4 |
| `EPR_MAX_INACTIVE_TIME` | 79 | 0x0AA7 | 4 |
| `EPR_STEPPER_INACTIVE_TIME` | 83 | 0x0AAB | 4 |

#### 22 Usage counters

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_PRINTING_TIME` | 125 | 0x0AC3 | 4 |
| `EPR_PRINTING_DISTANCE` | 129 | 0x0AC7 | 4 |

### Historical metadata and retired addresses

Do not migrate `EPR_MAGIC_BYTE`, `EPR_INTEGRITY_BYTE`, `EPR_VERSION`, `EPR_CHECK_NUM_EXTRUDERS`, `EPR_RF_MICRO_STEPS_USED`, `EPR_RF_PREHEAT_PROFILE_TAG`, `EPR_RF_PID_CALIBRATION`. New header/directory replaces legacy format/checksum/count; preheat and microstep validity provided by new block transaction/explicit initialized values. Retain the semantics of per-heater calibration validity within each PID block (three bytes; not a historical protocol version comparison). Do not carry retired/unused `EPR_ACCELERATION_TYPE`, `EPR_EXTRUDER_SPEED`, `EPR_BACKLASH_X`, `EPR_BACKLASH_Y`, `EPR_BACKLASH_Z`, `EPR_RF_230V_OUTPUT_MODE`, `EPR_RF_HOTEND_TYPE`, `EPR_EXTRUDER_ADVANCE_K` or historic reserved gaps. Mains output always off at boot. E1 and unsupported RGB/FET fields remain fixed-schema placeholders, canonical safe values, inaccessible where absent.

## Integrity: the master is NOT a substitute for payload verification

- Master CRC32 protects the ordered stored block CRC16 values and schema identity, not the payload itself. A flipped payload bit with unchanged directory leaves master valid; must recompute every block from actual bytes on boot/M501 before applying settings. Read defined payload once while validating/loading, not a 2048-byte scan; no unverified live settings. Valid master + invalid block means a local inconsistency; invalid master alone cannot locate a block or distinguish damaged master, CRC directory, interrupted transaction, or header. Compare every block plus header and journal; a matching CRC is not semantic validity or mathematical proof of no corruption.
- Normal transaction: validate typed proposed values and cross-field constraints; quiesce dependent outputs/motion, record durable in-progress metadata before first changed byte, reject reentrant mutation while yielding to essential services. Update only differing bytes, recompute affected block(s) on readback, verify stored CRC(s), read the 22 fixed-stride CRC16 values (44 bytes), publish the fixed master CRC and commit state last and verify before clearing transaction intent/allowing application. Do not recalculate unrelated block CRCs or blindly bless an unknown directory during a normal save. No-op writes neither data nor metadata. An M500 groups all changed blocks into one transaction/master commit; coalesce menu edits as appropriate without changing accepted save semantics. M501 checks first, M502 explicitly resets new settings, never updates old bank. A pending transaction and invalid master are not two independent damaged blocks.
- Durable transaction record needs target block bitmask, sequence/format/integrity and verified marker protocol; a power cut at ANY data, CRC, master, journal, or reset write may leave previous or new complete configuration, or explicit invalid/disabled state. The compact block00 transaction intent is the candidate; no external journal reservation. Do not choose an older higher cutoff on corruption; if a lower current cutoff cannot be proven, heat admission remains off until deliberate recommissioning. Torn markers, master interruption and multi-block recovery must be modeled. Do not claim CRC detects arbitrary multi-bit corruption or hardware power-switch failure.
- Per-block CRC16 and fixed master CRC32 cost additional metadata writes versus a single 8-bit checksum. Read/CPU savings are large; physical write savings require per-byte update, transaction batching and no-op suppression. Master would otherwise be an EEPROM wear hotspot even though only one small payload changes. Evaluate actual write counts and busy/service gaps; no rotation unless measured need justifies it within this KiB. Current eprSetInt16/32/Float writes the entire changed multibyte value; use byte-level update semantics where safe, with verified asynchronous/nonblocking scheduling or bounded service gaps. No extra long interrupts-off region. Keep hot loop data in RAM, not EEPROM; no full 2048-byte iteration on ordinary change. A one-block update reads <=29 payload bytes + 44 stored CRC bytes = <=73 core integrity bytes, plus length/header/journal/verification traffic, vs existing 2047 payload scan. Boot defined payload 304 B + 66 slot metadata bytes + global metadata. CRC CPU overhead and actual read/write traffic/latency still need measured evidence.
- A single verified ordinary-block fault: show persistent block-specific alert + serial log; stage board defaults for that block and require OK acknowledgment; retain all other valid values. On repeated boot before acknowledgement, alert persists; do not silently erase fault evidence. If >=2 independently confirmed invalid blocks after journal/master/header diagnosis, owner requests full new-bank factory reset, with clear multi-block alert and explicit commissioning. Do not trigger factory reset solely from master mismatch or one interrupted multi-block transaction. Before resetting, log affected zones/values only if independently trusted.
- Safety/commissioning exception: invalid heater sensor, PID/power identity, pressure, geometry/microstep or temperature-limit block must inhibit corresponding heat/motion until validated settings and required calibration/homing. Immutable measured bed ceiling 150 C, emergency setting default120 on explicit initial commissioning, max target = cutoff -10 K (0 for invalid/disabled); no automatic increase of a previously lower limit after corruption, M502 or multi-block reset. Fresh erased upper bank and erased previously commissioned bank are indistinguishable without external trusted provenance; explicit initial commissioning/limit confirmation rather than automatic heating. A general OK that acknowledges a reset is not approval to silently raise thermal cutoff. Even full factory reset requires deliberate thermal commissioning if previous setting is unknown. Keep independent measured absolute monitoring, no stale rearm. Old firmware after downgrade ignores these protections.
- Coupled fields: Microstep mode, steps/mm, Z jog conversion and geometry have atomic dependency closure; extruder motion steps and microstep must agree, pressure pair bounds and feedrate/homing pairs validated together; changed sensor/power invalidates PID calibration, changed geometry invalidates matrix/homing, preheat/scan admission rechecks current thermal cap. Multiple blocks may legitimately change in one transaction. Revalidate entire configuration before activation; no running job sees half-applied dependent fields.

## Host / UI API and tests

M205/M206 use typed explicit field whitelist, NOT raw arbitrary byte writes. New physical addresses are deliberately distinct from old P values, so reject old saved Host profiles instead of aliasing to a new setting. Current strict ASCII parser and handler restrict P<2048; update to new address range only after exact whitelist/width/validation. Existing GCode P is int32_t; verify Repetier-Host/Server M205 parsing and M206 ASCII/binary high address round-trip before choosing final public IDs. If host cannot support high addresses, use explicit versioned logical field IDs with nonoverlapping legacy acceptance or reject old configuration imports; never silently alias old raw writes. Slot CRC/length bytes, master, journal, header and calibration identity not directly writable. Keep regular command/ACK semantics, no Task0033 UART production work. No direct writes from menu/G-code/modules outside one persistence owner. Field validation used by host, panel and defaults/loading.

Tests before A70/A71 close: generated address/size/coverage assertions all boards; initial legacy bank preserved/new defaults including both heater slots and all panel-only values; no old import, same-schema update, wrong model/X44, arbitrary erased/corrupt upper bank, downgrade/re-upgrade; real firmware-backed power cut at every write and readback failure, CRC single-bit and selected multibit mutations, valid master corrupted payload counterexample, corrupted directory and master-only recovery, journal replay/sequence wrap, single/multi-block alerts and persistent acknowledgement; DMAX/PID survival of unrelated block changes and manual edits AFTER initial reset; M500/501/502, M205/206 old and new addresses, all actual direct writers, no-op/changed byte counts; critical block inhibits and no raised bed limit on corruption; measure worst-case service intervals and AVR flash/SRAM/stack on all four builds, including combined safety image. Existing PID simulations/history retained. No hardware runs.

## Handover

A69 CI 45930076134 on 06c1956d41e00695d90a8be63ede749274ed4b10 completed success per GitHub Actions; artifact inspection and final resource evidence remain separate. The new schema is a scope change requested by owner after that run. Existing A63/A68 small-record research retained as evidence but old-layout migration no longer target. Implementation not started in this documentation commit. Next: exact legacy upper-bound/Host high-address validation, transaction model with power-cut/wear analysis, canonical schema/accessor and all direct caller inventory, then integrated A70/A71 (no parallel production). A72/A73 require latest full four-board evidence. Do not merge/flash without separate approval.

## Write-frequency and latency audit (2026-11-27, pinned 1bbbaf2af44d1f51506261f59ad6df723afc5a0a)

Static source audit of PrintUsage.cpp, Extruder.cpp, UiActions.cpp, UiSettings.cpp, ui.cpp, Configuration.h, PanelMotion.cpp, GCodeMotion.cpp, GCodeThermal.cpp, GCodeSystem.cpp, TemperatureControl.cpp, RgbLighting.cpp, RF.cpp, ZOffsetScan.cpp, PreheatProfiles.cpp, HAL.h, Eeprom.cpp. Not field telemetry; no claims about actual failure incidence or real average write frequency. Upper bank likely unused by ordinary historical firmware, not proof of unworn cells or immunity to age/retention/raw writes.

| Path | Actual trigger | Optimization |
|---|---|---|
| PrintUsage.cpp:37–40; Extruder.cpp:174–179 | Last hotend target transitions active->inactive; return if pending filament zero. Saves time and distance, then full checksum. Not every second/step. | Keep job-boundary commit, byte-difference writes, optionally cache saved totals (8 B RAM tradeoff) if read-frequency profiling supports it. No periodic saves unless requested. |
| UiActions.cpp:66–71; UiSettings.cpp:217 | Manual Z buttons and menu increments save immediately, storeManualZOffset checks equality | Live compensation stays RAM only; preserve DOWN->UP 5um takeover; Preserve immediate save (owner reports rare edits); only reconsider coalescing with measured need and explicit durability policy. |
| ui.cpp:1095–1111, Configuration.h:353–359 | Repeatable keys first delay500ms, decrement50ms, minimum50ms => up to20 actions/s under service budget | Immediate saves retained for ordinary settings, no-op detection; only coalesce where justified by measured workload. For safety-critical edits immediate safe inhibition/application; do not delay required durable admission or treat old cutoff as committed. |
| UiSettings.cpp:530–620 | Microstep edit only idle, but eprSetByte/Int16/Float executed inside InterruptProtectedBlock before unprotect at618 | Move all EEPROM I/O outside masked region; keep only atomic derived state publishing in guard. Multiple changed cells can otherwise wait milliseconds per cell with IRQs disabled; actual upper bound needs linked timing. Group affected geometry/extruder blocks and invalidate homing/matrix. |
| GCodeMotion.cpp:511–544 | M3909 P/S save only if changed; P and S can trigger two full checksum scans in one command | Preserve normal unchanged startcode no-op; validate both and commit one block once. Repeated varying profiles legitimately persist under current contract, consider explicit volatile mode only with owner approval. |
| GCodeMotion.cpp:447–449,496–498 | Pressure min/max save pair and call checksum even if unchanged | Reject invalid pair, no-op entire transaction if unchanged, validate together. |
| GCodeThermal.cpp:308–341, RgbLighting.cpp:100–101,181–182; UiActions.cpp | FET/lighting state and RGB component setting persisted on commands/toggles; checksum even if byte unchanged on several paths | Automatic RGB status/fade/white override does not persist; only explicit mode/profile edits and manual/M3300 FET state changes persist. Owner reports rare use. Preserve semantics, no-op suppression. Automated accessory control can be a real repeated-write source, not observed frequency. |
| UiSettings.cpp:932–1052; TemperatureControl.cpp:940–958 | Manual DMIN/DMAX, PID/sensor edits; autotune stores only on successful requested save | One PID block transaction, preserve calibration coupling and manual DMAX. No background PID EEPROM writes in reviewed paths. |
| GCodeSystem.cpp:122,146; PreheatProfiles.cpp:62–89 | M500/M502 bulk save; menu profile edits | Compare actual serialized bytes, group changed blocks; no-op master/intent/CRC on identical values. M502 deliberate reset. |
| ZOffsetScan.cpp:505–538 | Changed scan points only | No-op and grouped scan configuration where appropriate. |
| HAL.h:405–431 | Byte setter compares old byte. Int16/Int32/Float setters compare whole value, then call eeprom_write_word/dword/block (not update variants) | Use per-byte difference updates. Example 32-bit counter 0x00000064 -> 0x00000065 should program one byte, not four; also avoid float equality pitfalls (NaN, signed zero). Explicit canonical wire bytes. AVR EEPROM hardware busy waits must yield or be split into bounded nonblocking steps with essential thermal services; do not call EEPROM from ISR. |

Microchip datasheet ATmega2560 EEPROM endurance at least 100000 erase/write cycles per cell (not hard failure threshold); datasheet https://www.microchip.com/bin/mchp/product-ds.ATmega2560.pdf. Worst-case illustrative hottest-cell lifetime at one physical write per commit: 1/day 274 y, 5/day 54.8 y, 20/day 13.7 y, 100/day 2.74 y; 20/s continuous 1.39 h. If a marker is programmed twice per commit, halve those bounds. Real writes depend on byte changes, cell temperature, programming mode, old wear and usage. No field failure reports established. Fresh address range reduces presumed prior wear only. Main priority is removing repeated writes and foreground/IRQ busy time, not universal wear-leveling. No master/intent rotation in the baseline; no per-feature log filesystem, garbage collection, dynamic allocation or routine counter wear leveling. First quantify master/intent/header cells (including commit marker twice), no-op, single-byte edit, repeat burst, M3909 two parameters, M500 changed/unchanged, usage counter rollover. Owner selects fixed block00 master, no routine wear leveling. Revisit only if measured evidence requires it, within the defined KiB. Never relax power-failure or critical cutoff invariants to save writes. Do not add a per-slot invalidation marker write on every save if verified durable global intent already covers same interruption. No routine checksum refresh on read-only operations; preserve fault evidence and validate pre-existing block before blessing modified bytes. Keep master only over committed CRCs; validate prior master/journal before modifying and don't legitimize unrelated corruption. No full upper-bank wipe/scan on ordinary writes.

Open implementation gates: EEPROM transaction power-cut model including journal/marker and master, paired feature commits, UI coalescing durability/explicit ACK semantics, upper host address ABI, full production source integration and four-board CI. The current task is design/audit only; no firmware writes or measured runtime changes. A69 full CI previously green.

## Owner refinement and first implementation checkpoint (2026-11-27)

Block00-only global metadata, nine free slots, second upper KiB entirely unallocated. No routine wear leveling, no delayed saves for rarely changed manual Z/DMAX/motor current; retain job-boundary usage counter. RGB automatic fades/status are RAM/GPIO only (RgbLighting::advanceFade and status logic); explicit RGB mode/component and FET M3300 saves only. Preserve existing behavior; later block-level no-op handles repeat commands. Modular ownership: EEPROM schema/integrity/transaction and typed persistent settings belong to dedicated EEPROM modules, HAL only raw byte transport, UI and heater/motion services own live state. Avoid monolithic Eeprom.cpp growth; future code split with existing build manifests/replays coordinated.

First prerequisite implementation (legacy layout still active until new schema integration): HAL eprSetByte/Int16/Int32/Float use avr-libc eeprom_update_* per-byte operations, not full changed scalar writes. No change in addresses or ordinary save/ACK semantics; raw float bytes now compared (signed zero/NaN representation). UI microstep edit retains disabled/homing-invalidated steppers and bounded RAM/driver critical section, performs no EEPROM access within it. EEPROM::storeMicrostepConfiguration handles mode/geometry/jog writes afterward, early return when unchanged and initialized. Fixed existing absent E1 array access on one-tool builds (physical E1 driver still paired), mismatched pair update and actual per-axis ratio for clamped/multiple increments; no driver protocol change. Remaining driver/derived critical section and EEPROM foreground busy waits still need linked worst-case timing audit; this patch only removes EEPROM from masked region. Added extracted-source host tests for byte changes/no-op and microstep IRQ boundaries, both tool counts, mismatch, multi-increment, Z coupling and busy rejection; retain all legacy snapshot tests. CI pending. No firmware schema activation, hardware tests, flash or merge.


## A70a executable storage candidate

New modules `Repetier/EepromZoneLayout.h`, `EepromZoneIntegrity.h`, `EepromZoneStorage.h` implement fixed physical addresses and explicit storage transaction/inspection operations. They are not yet wired to live settings. Static header CRC16, feature CRC16 (CCITT-FALSE, initFFFF, poly1021, no xorout), master CRC32 (reflected IEEE, init/xoroutFFFFFFFF), fixed little-endian fields. CRC domains include exact identity, ID and length; master reads 44 stored feature CRC bytes, not padding. Block00 Preparing 0x3C written before intent changes; Writing 0x5A only after verified intent+complement; Clean 0xA5 only after verified feature CRC/master. Preparing alone cannot have changed feature bytes; interrupted Writing marks all intended blocks uncertain even if their CRCs match. Recovery persists pending alerts before replacing old intent. Invalid alert pair conservatively raises all pending alerts. Header first signature byte invalidated before full initialization, published last. No automatic heat permission: initialize/repair force bed cutoff 0, explicit commissioning required. Core eligibility requires valid header/master, clean state, valid alert pair, target CRC and no target alert; caller must also validate semantic/coupled settings and safety admission. Normal commits verify affected prior payload only; no-op writes nothing. No A/B or spare slots used. Byte backend must guarantee stable readback; provider immutable, non-reentrant, bounded live scheduler service still required. `tests/eeprom-zones.cpp` uses actual engine and exhaustive per-write torn-byte/power-cut simulation; invoked from existing hosted EEPROM test. Existing golden legacy replay remains. No release ABI, physical test or production activation yet.


### A70a follow-up: nested interruption and status pair

Byte31 is used for the complement of the state byte at offset22; both must match before Preparing/Writing/Clean is recognized. No extra storage. A repair of an unresolved transaction does not pass through Preparing; preserve the dirty/invalid state while replacing its intent, after recording alerts. Otherwise a power cut during repair could hide a CRC-valid mixed transaction. Clean is only published after master and payloads are complete. Host test adds interrupted explicit cutoff increase followed by exhaustive torn-byte repair and a further recovery; automatic repair never authorizes the higher limit. Normal no-op comparison validates prior payload and compares proposed bytes in one read pass; identical bytes skip readback and program. This is a core refinement, live integration pending.


## A70b factory block provider and codec

`EepromZoneCodec.h` provides bounded LE encoding/decoding in a caller-owned <=29-byte buffer and complete byte coverage checking (missing/overlapping/wrong-block fields fail). `EepromZoneDefaults.h` builds all 22 feature records from actual resolved board defaults, not from historical cells or mutable RAM, with stable model/tool/X44 identity; any other retrofit profile change must be reviewed. Factory bed permission is disabled, tuning identities cleared, inactive tool payload canonical, all five driver modes/current values explicit. Explicitly initialize fields historically omitted by full store (usage, remembered SenseOffset threshold 0, RGB profiles, DMS, fan, manual Z); no old fallback logic. RGB default mode follows RGB_LIGHT_DEFAULT_MODE (previous restore hardcoded automatic despite board default; owner choice of new firmware defaults), RGB color values from configuration; actual legacy RGB code saves profile bytes but never loads them (source audit RgbLighting.cpp/Eeprom.cpp); new live loader must restore them via RgbLighting owner. X44 FET2 reserved to cooler, default accessory state 0, mains never persisted. Preheat stored profile values independent of current bed admission cap. Host test uses all four real board configurations and exercises actual factory provider/core, byte coverage and default samples. Integration with live printer and semantic field validation still pending; this is not activated firmware.


### Step-queue isolation

SenseOffset saved limits have a four-byte foreground-published RAM snapshot. The queue activates that pair without reading EEPROM, including while a byte write is busy. M3909 P/S and UI changes share a foreground save helper. This is a prerequisite independent of switching the live layout; canonical host/AVR validation remains required.
