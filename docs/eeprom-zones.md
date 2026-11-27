# New internal EEPROM architecture: owner decision and design candidate

**Owner decision 2026-11-27:** break the legacy EEPROM settings ABI. First installation of the new layout initializes ALL user settings from the new board/retrofit-specific firmware defaults, including old PID/DMAX, microsteps, mechanical calibration and usage counters. No legacy data migration. Later updates of the same new schema preserve valid new settings; no unconditional reset on every boot. This supersedes Task0034 A63/A68 v9->v10 preservation and the historic-address constraint in AGENTS/TASKS for this work only. Still preserve printing protocol, hardware support, simulation evidence and safety requirements. Do not edit the integration branch or other workers' production scope. No flash/merge/hardware tests authorized.

**Status:** architecture and field-level candidate only. No EEPROM writes, firmware change, actual address assignment, or measured runtime improvement yet. New production integration should be staged under Task0034 exclusive ownership (A70/A71 revised; separate EEPROM integration work package if needed), before A72/A73; do not start a parallel production worker. No further PID optimization.

## Evidence and capacity

Pinned Task0034 head 06c1956d41e00695d90a8be63ede749274ed4b10. CI artifact 15167769162 (tested merge 920309ae09b49d95c97eeaa559ba74312af0b725) contains the unchanged firmware sources used for the inventory; A69 changed only tooling/replay/docs. Source scan of 123 sketch files (read/write callers in Eeprom, GCodeMotion/Thermal, UiActions/Settings/Formatting, ui, PanelMotion, PreheatProfiles, PrintUsage, Printer, RF, RgbLighting, ZOffsetScan, HAL). Checked Eeprom.h, all direct HAL::eprGet/Set references, dynamic motor/microstep arrays, extruder offsets, RGB table and PID calibration slots. Current HAL EEPROM_OFFSET=0; Eeprom.cpp checksum and clear iterate 0..2047. Historical community_development 49a60a97eefb5dbd5b5eeea4344f090979c2d070 Eeprom.h/Eeprom.cpp/HAL.h also show offset 0, same 2048-byte checksum/clear range and known named fields below 2048. Verified additional downgrade caveat: at historical commit 49a60a97eefb5dbd5b5eeea4344f090979c2d070, EEPROM::update in Repetier/Eeprom.cpp forwards M206 P directly to HAL::eprSetByte/Int16/Int32/Float without an address range check; HAL.h uses EEPROM_OFFSET=0. Thus legacy raw writes can touch the new bank even though ordinary settings and reset do not. Isolation is an address convention, not a hardware protection. Arbitrary historic/third-party firmware and upload-tool EEPROM retention still unqualified. Preserve legacy bank; verify actual historical access and EEPROM retention before promising downgrade compatibility. New firmware never reads or writes legacy settings; downgrade itself does not have the new safety guarantees. Re-upgrade of an existing valid new schema preserves that schema unless explicitly reset; physical printer changes made while using old firmware may require recommissioning.

ATmega2560 internal EEPROM 4096 bytes; prefer 0x0800..0x0FFF for the new schema, leave 0x0000..0x07FF untouched. 303 bytes of existing active persisted payload + 1 new bed limit = 304 bytes (largest two-extruder/RGB board, fixed physical heater/driver slots, inactive fields initialized canonically). Historical format markers and generic magic/checksum/version/count are not payload; replace with new header/metadata. Explicitly initialize all fields, including ones previously only saved via panel or G-code. No assumption that 0xFF is a valid default. Keep 230V mains off on boot and never persist its on state.

External I2C 0x50 bed matrix is a separate address space (1536-byte sectors, format 7, active slot). External 0x51 board identity on RF2000 family is not user settings and must not be erased. Proposed full configuration reset invalidates old compensation matrix eligibility and requires a fresh scan or an explicitly qualified compatible matrix policy; do not silently reuse old geometry/microstep calibration, do not erase physical board identity, do not destroy external matrix data merely to reserve internal addresses. RF1000 lacks board identity chip. Investigate separate new matrix format/generation policy under A72 without claiming migration.

## Compact 32-byte slot layout (owner refinement, 2026-11-27)

Supersedes the earlier separate directory / 21-slot layout (retained in Git commit 410e459). The first 1024 bytes **of the new upper bank**, not the historic physical lower EEPROM, contain 32 fixed slots. The remaining 1024 bytes contain only global metadata, optional bounded transaction/recovery records and reserve. No allocator, no address directory or full EEPROM SRAM mirror. Block address = 0x0800 + (uint16_t(id) << 5); require id <32. Each slot contains CRC16 at offsets 0..1, length/state at offset 2, up to 29 payload bytes at offsets 3..31. Free payload = 29-length; do not store separately or read/write padding. Length 0..29 reserved for defined schema lengths, 0xFF uninitialized, 0xFE optionally incomplete (not mandatory two writes per transaction when durable global intent suffices), all other values invalid. Validate exact schema length before any read/address calculation; CRC includes fixed schema/block identity and length. CRC16 variant and byte order specified before implementation. No dynamic parsing of untrusted lengths. Fixed-size schema/length tables in Flash, no expensive general serialization, no runtime heap. Do not write full 32-byte slots merely because reserved.

The previous 32-byte speed+jerk payload is the only one exceeding the new 29-byte payload. Split speed/homing (24 B) and jerk (8 B), maintaining cross-field max/homing validation in one block. Larger future features may use multiple independently valid slots, or a coupled atomic transaction if semantically inseparable.

| Range | Bytes | Purpose |
|---|---:|---|
| 0x0000–0x07FF | 2048 | Legacy settings preserved, never loaded by new firmware |
| 0x0800–0x0BFF | 1024 | 32 x 32-byte feature slots |
| 0x0C00–0x0C1F | 32 | Candidate global schema/hardware-profile header |
| 0x0C20–0x0C9F | 128 | Candidate small rotating master records, exact encoding/need to be measured |
| 0x0CA0–0x0FFF | 864 | Durable multi-block transaction intent, critical redundant records if justified, future reserve |

22 occupied slots = 704 B (304 payload + 66 header/CRC + 334 unused padding), 10 unused slots = 320 B. 1024 bytes upper second half available for metadata/recovery. 22 block CRCs = 44 B at fixed stride; compute master from them, not from slot padding, using only schema-declared active IDs; header identity domain separated. No separate CRC directory. Candidate addresses, not final ABI. The master cannot validate payload, so boot/M501 still recompute each actual block CRC. One-block update core integrity reads at most 29+44=73 bytes plus length/header/intent/readback traffic; boot payload+slot metadata=370 bytes plus global metadata. Compare actual CPU/EEPROM-busy/WCET before claiming speedup.

| ID | Owner/function | Physical slot | Payload bytes | Free payload |
|---:|---|---|---:|---:|
| 00 | XYZ geometry and resolution | 0x0800–0x081F | 29 | 0 |
| 01 | Axis max and homing feedrates | 0x0820–0x083F | 24 | 5 |
| 02 | Axis jerk | 0x0840–0x085F | 8 | 21 |
| 03 | Acceleration and packing | 0x0860–0x087F | 26 | 3 |
| 04 | Stepper current | 0x0880–0x089F | 5 | 24 |
| 05 | Extruder 0 motion | 0x08A0–0x08BF | 29 | 0 |
| 06 | Extruder 1 motion | 0x08C0–0x08DF | 29 | 0 |
| 07 | Hotend 0 PID and sensor | 0x08E0–0x08FF | 20 | 9 |
| 08 | Hotend 1 PID and sensor | 0x0900–0x091F | 20 | 9 |
| 09 | Bed PID and sensor | 0x0920–0x093F | 19 | 10 |
| 10 | Bed safety limit | 0x0940–0x095F | 1 | 28 |
| 11 | Preheat profiles | 0x0960–0x097F | 8 | 21 |
| 12 | Manual Z offset | 0x0980–0x099F | 4 | 25 |
| 13 | DMS and SenseOffset | 0x09A0–0x09BF | 7 | 22 |
| 14 | Pressure safety | 0x09C0–0x09DF | 12 | 17 |
| 15 | Scan settings | 0x09E0–0x09FF | 6 | 23 |
| 16 | Wobble compensation | 0x0A00–0x0A1F | 7 | 22 |
| 17 | Panel and jog modes | 0x0A20–0x0A3F | 6 | 23 |
| 18 | Lighting and accessories | 0x0A40–0x0A5F | 20 | 9 |
| 19 | Part fan | 0x0A60–0x0A7F | 4 | 25 |
| 20 | Serial and inactivity | 0x0A80–0x0A9F | 12 | 17 |
| 21 | Usage counters | 0x0AA0–0x0ABF | 8 | 21 |

## Field-level address inventory (compact candidate)

125 field rows; 303 old payload bytes + 1 new bed limit. Exact overlap and bounds validation executed on this inventory in JS: 22 slots, 304 unique addresses, no overlap, max length29; 10 spare slots. Fixed inactive physical heater/driver fields canonicalized. Widths are existing AVR storage widths, not a commitment to retain unnecessary metadata. CRC and lengths are excluded from host write whitelist. Field offsets below supersede original draft.

#### 00 XYZ geometry and resolution

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_XAXIS_STEPS_PER_MM` | 3 | 0x0803 | 4 |
| `EPR_YAXIS_STEPS_PER_MM` | 7 | 0x0807 | 4 |
| `EPR_ZAXIS_STEPS_PER_MM` | 11 | 0x080B | 4 |
| `EPR_X_LENGTH` | 145 | 0x080F | 4 |
| `EPR_Y_LENGTH` | 149 | 0x0813 | 4 |
| `EPR_Z_LENGTH` | 153 | 0x0817 | 4 |
| `EPR_RF_MICRO_STEPS_X` | 1943 | 0x081B | 1 |
| `EPR_RF_MICRO_STEPS_Y` | 1944 | 0x081C | 1 |
| `EPR_RF_MICRO_STEPS_Z` | 1945 | 0x081D | 1 |
| `EPR_RF_MOD_Z_STEP_SIZE` | 1900 | 0x081E | 2 |

#### 01 Axis max and homing feedrates

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_X_MAX_FEEDRATE` | 15 | 0x0823 | 4 |
| `EPR_Y_MAX_FEEDRATE` | 19 | 0x0827 | 4 |
| `EPR_Z_MAX_FEEDRATE` | 23 | 0x082B | 4 |
| `EPR_X_HOMING_FEEDRATE_PRINT` | 27 | 0x082F | 4 |
| `EPR_Y_HOMING_FEEDRATE_PRINT` | 31 | 0x0833 | 4 |
| `EPR_Z_HOMING_FEEDRATE_PRINT` | 35 | 0x0837 | 4 |

#### 02 Axis jerk

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_MAX_XYJERK` | 39 | 0x0843 | 4 |
| `EPR_MAX_ZJERK` | 47 | 0x0847 | 4 |

#### 03 Acceleration and packing

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_X_MAX_ACCEL` | 51 | 0x0863 | 4 |
| `EPR_Y_MAX_ACCEL` | 55 | 0x0867 | 4 |
| `EPR_Z_MAX_ACCEL` | 59 | 0x086B | 4 |
| `EPR_X_MAX_TRAVEL_ACCEL` | 63 | 0x086F | 4 |
| `EPR_Y_MAX_TRAVEL_ACCEL` | 67 | 0x0873 | 4 |
| `EPR_Z_MAX_TRAVEL_ACCEL` | 71 | 0x0877 | 4 |
| `EPR_RF_STEP_PACKING_MIN_INTERVAL` | 1964 | 0x087B | 2 |

#### 04 Stepper current

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOTOR_CURRENT` | 1923 | 0x0883 | 5 |

#### 05 Extruder 0 motion

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E0:EPR_EXTRUDER_STEPS_PER_MM` | 200 | 0x08A3 | 4 |
| `E0:EPR_EXTRUDER_MAX_FEEDRATE` | 204 | 0x08A7 | 4 |
| `E0:EPR_EXTRUDER_MAX_START_FEEDRATE` | 208 | 0x08AB | 4 |
| `E0:EPR_EXTRUDER_MAX_ACCELERATION` | 212 | 0x08AF | 4 |
| `E0:EPR_EXTRUDER_ADVANCE_L` | 246 | 0x08B3 | 4 |
| `E0:EPR_EXTRUDER_X_OFFSET` | 231 | 0x08B7 | 4 |
| `E0:EPR_EXTRUDER_Y_OFFSET` | 235 | 0x08BB | 4 |
| `EPR_RF_MICRO_STEPS_E0` | 1946 | 0x08BF | 1 |

#### 06 Extruder 1 motion

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E1:EPR_EXTRUDER_STEPS_PER_MM` | 300 | 0x08C3 | 4 |
| `E1:EPR_EXTRUDER_MAX_FEEDRATE` | 304 | 0x08C7 | 4 |
| `E1:EPR_EXTRUDER_MAX_START_FEEDRATE` | 308 | 0x08CB | 4 |
| `E1:EPR_EXTRUDER_MAX_ACCELERATION` | 312 | 0x08CF | 4 |
| `E1:EPR_EXTRUDER_ADVANCE_L` | 346 | 0x08D3 | 4 |
| `E1:EPR_EXTRUDER_X_OFFSET` | 331 | 0x08D7 | 4 |
| `E1:EPR_EXTRUDER_Y_OFFSET` | 335 | 0x08DB | 4 |
| `EPR_RF_MICRO_STEPS_E1` | 1947 | 0x08DF | 1 |

#### 07 Hotend 0 PID and sensor

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E0:EPR_EXTRUDER_PID_PGAIN` | 218 | 0x08E3 | 4 |
| `E0:EPR_EXTRUDER_PID_IGAIN` | 222 | 0x08E7 | 4 |
| `E0:EPR_EXTRUDER_PID_DGAIN` | 226 | 0x08EB | 4 |
| `E0:EPR_EXTRUDER_DRIVE_MIN` | 245 | 0x08EF | 1 |
| `E0:EPR_EXTRUDER_DRIVE_MAX` | 217 | 0x08F0 | 1 |
| `E0:EPR_EXTRUDER_PID_MAX` | 230 | 0x08F1 | 1 |
| `E0:EPR_EXTRUDER_SENSOR_TYPE` | 294 | 0x08F2 | 1 |
| `PID_CALIBRATION_SLOT_0` | 1978 | 0x08F3 | 3 |
| `E0:EPR_EXTRUDER_COOLER_SPEED` | 254 | 0x08F6 | 1 |

#### 08 Hotend 1 PID and sensor

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `E1:EPR_EXTRUDER_PID_PGAIN` | 318 | 0x0903 | 4 |
| `E1:EPR_EXTRUDER_PID_IGAIN` | 322 | 0x0907 | 4 |
| `E1:EPR_EXTRUDER_PID_DGAIN` | 326 | 0x090B | 4 |
| `E1:EPR_EXTRUDER_DRIVE_MIN` | 345 | 0x090F | 1 |
| `E1:EPR_EXTRUDER_DRIVE_MAX` | 317 | 0x0910 | 1 |
| `E1:EPR_EXTRUDER_PID_MAX` | 330 | 0x0911 | 1 |
| `E1:EPR_EXTRUDER_SENSOR_TYPE` | 394 | 0x0912 | 1 |
| `PID_CALIBRATION_SLOT_1` | 1981 | 0x0913 | 3 |
| `E1:EPR_EXTRUDER_COOLER_SPEED` | 354 | 0x0916 | 1 |

#### 09 Bed PID and sensor

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_BED_PID_PGAIN` | 108 | 0x0923 | 4 |
| `EPR_BED_PID_IGAIN` | 112 | 0x0927 | 4 |
| `EPR_BED_PID_DGAIN` | 116 | 0x092B | 4 |
| `EPR_BED_DRIVE_MIN` | 124 | 0x092F | 1 |
| `EPR_BED_DRIVE_MAX` | 107 | 0x0930 | 1 |
| `EPR_BED_PID_MAX` | 120 | 0x0931 | 1 |
| `EPR_RF_HEATED_BED_SENSOR_TYPE` | 1922 | 0x0932 | 1 |
| `PID_CALIBRATION_SLOT_2` | 1984 | 0x0933 | 3 |

#### 10 Bed safety limit

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `NEW_BED_EMERGENCY_C` | new | 0x0943 | 1 |

#### 11 Preheat profiles

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_PREHEAT_A_BED` | 1970 | 0x0963 | 2 |
| `EPR_RF_PREHEAT_A_HOTEND` | 1972 | 0x0965 | 2 |
| `EPR_RF_PREHEAT_B_BED` | 1974 | 0x0967 | 2 |
| `EPR_RF_PREHEAT_B_HOTEND` | 1976 | 0x0969 | 2 |

#### 12 Manual Z offset

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_Z_OFFSET` | 1038 | 0x0983 | 4 |

#### 13 DMS and SenseOffset

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_SENSEOFFSET_OFFSET_MAX` | 1906 | 0x09A3 | 2 |
| `EPR_RF_MOD_SENSEOFFSET_DIGITS` | 1908 | 0x09A5 | 2 |
| `EPR_RF_MOD_SENSEOFFSET_AUTOSTART` | 1953 | 0x09A7 | 1 |
| `EPR_RF_ZERO_DIGIT_STATE` | 1937 | 0x09A8 | 1 |
| `EPR_RF_DIGIT_CMP_STATE` | 1938 | 0x09A9 | 1 |

#### 14 Pressure safety

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_EMERGENCYPAUSEDIGITSMIN` | 1910 | 0x09C3 | 4 |
| `EPR_RF_EMERGENCYPAUSEDIGITSMAX` | 1914 | 0x09C7 | 4 |
| `EPR_RF_EMERGENCYZSTOPDIGITSMIN` | 1918 | 0x09CB | 2 |
| `EPR_RF_EMERGENCYZSTOPDIGITSMAX` | 1920 | 0x09CD | 2 |

#### 15 Scan settings

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_ZOS_SCAN_POINT_X` | 1904 | 0x09E3 | 1 |
| `EPR_RF_MOD_ZOS_SCAN_POINT_Y` | 1905 | 0x09E4 | 1 |
| `EPR_ZSCAN_START_MM` | 1949 | 0x09E5 | 4 |

#### 16 Wobble compensation

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOD_WOBBLE_FIX_PHASEXY` | 1954 | 0x0A03 | 1 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPX` | 1955 | 0x0A04 | 2 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPY1` | 1957 | 0x0A06 | 2 |
| `EPR_RF_MOD_WOBBLE_FIX_AMPY2` | 1959 | 0x0A08 | 2 |

#### 17 Panel and jog modes

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_MOVE_MODE_X` | 1042 | 0x0A23 | 1 |
| `EPR_RF_MOVE_MODE_Y` | 1043 | 0x0A24 | 1 |
| `EPR_RF_MOVE_MODE_Z` | 1044 | 0x0A25 | 1 |
| `EPR_RF_Z_MODE` | 1045 | 0x0A26 | 1 |
| `EPR_RF_MOVE_MODE_XY_KOSYS` | 1046 | 0x0A27 | 1 |
| `EPR_RF_MOVE_POSITION_FEEDRATE` | 1047 | 0x0A28 | 1 |

#### 18 Lighting and accessories

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_CASE_LIGHT_MODE` | 1001 | 0x0A43 | 1 |
| `EPR_RF_FET1_MODE` | 1020 | 0x0A44 | 1 |
| `EPR_RF_FET2_MODE` | 1021 | 0x0A45 | 1 |
| `EPR_RF_FET3_MODE` | 1022 | 0x0A46 | 1 |
| `EPR_RF_RGB_LIGHT_MODE` | 1019 | 0x0A47 | 1 |
| `EPR_RF_RGB_PRINTING_R` | 1023 | 0x0A48 | 1 |
| `EPR_RF_RGB_PRINTING_G` | 1024 | 0x0A49 | 1 |
| `EPR_RF_RGB_PRINTING_B` | 1025 | 0x0A4A | 1 |
| `EPR_RF_RGB_HEATING_R` | 1026 | 0x0A4B | 1 |
| `EPR_RF_RGB_HEATING_G` | 1027 | 0x0A4C | 1 |
| `EPR_RF_RGB_HEATING_B` | 1028 | 0x0A4D | 1 |
| `EPR_RF_RGB_COOLING_R` | 1029 | 0x0A4E | 1 |
| `EPR_RF_RGB_COOLING_G` | 1030 | 0x0A4F | 1 |
| `EPR_RF_RGB_COOLING_B` | 1031 | 0x0A50 | 1 |
| `EPR_RF_RGB_IDLE_R` | 1032 | 0x0A51 | 1 |
| `EPR_RF_RGB_IDLE_G` | 1033 | 0x0A52 | 1 |
| `EPR_RF_RGB_IDLE_B` | 1034 | 0x0A53 | 1 |
| `EPR_RF_RGB_MANUAL_R` | 1035 | 0x0A54 | 1 |
| `EPR_RF_RGB_MANUAL_G` | 1036 | 0x0A55 | 1 |
| `EPR_RF_RGB_MANUAL_B` | 1037 | 0x0A56 | 1 |

#### 19 Part fan

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_RF_FAN_MODE` | 1930 | 0x0A63 | 1 |
| `EPR_RF_PART_FAN_SPEED` | 1963 | 0x0A64 | 1 |
| `EPR_RF_PART_FAN_PWM_MIN` | 1961 | 0x0A65 | 1 |
| `EPR_RF_PART_FAN_PWM_MAX` | 1962 | 0x0A66 | 1 |

#### 20 Serial and inactivity

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_BAUDRATE` | 75 | 0x0A83 | 4 |
| `EPR_MAX_INACTIVE_TIME` | 79 | 0x0A87 | 4 |
| `EPR_STEPPER_INACTIVE_TIME` | 83 | 0x0A8B | 4 |

#### 21 Usage counters

| Field | Old physical byte | Candidate new physical byte | Bytes |
|---|---:|---:|---:|
| `EPR_PRINTING_TIME` | 125 | 0x0AA3 | 4 |
| `EPR_PRINTING_DISTANCE` | 129 | 0x0AA7 | 4 |


### Historical metadata and retired addresses

Do not migrate `EPR_MAGIC_BYTE`, `EPR_INTEGRITY_BYTE`, `EPR_VERSION`, `EPR_CHECK_NUM_EXTRUDERS`, `EPR_RF_MICRO_STEPS_USED`, `EPR_RF_PREHEAT_PROFILE_TAG`, `EPR_RF_PID_CALIBRATION`. New header/directory replaces legacy format/checksum/count; preheat and microstep validity provided by new block transaction/explicit initialized values. Retain the semantics of per-heater calibration validity within each PID block (three bytes; not a historical protocol version comparison). Do not carry retired/unused `EPR_ACCELERATION_TYPE`, `EPR_EXTRUDER_SPEED`, `EPR_BACKLASH_X`, `EPR_BACKLASH_Y`, `EPR_BACKLASH_Z`, `EPR_RF_230V_OUTPUT_MODE`, `EPR_RF_HOTEND_TYPE`, `EPR_EXTRUDER_ADVANCE_K` or historic reserved gaps. Mains output always off at boot. E1 and unsupported RGB/FET fields remain fixed-schema placeholders, canonical safe values, inaccessible where absent.

## Integrity: the master is NOT a substitute for payload verification

- Master CRC32 protects the ordered stored block CRC16 values and schema identity, not the payload itself. A flipped payload bit with unchanged directory leaves master valid; must recompute every block from actual bytes on boot/M501 before applying settings. Read defined payload once while validating/loading, not a 2048-byte scan; no unverified live settings. Valid master + invalid block means a local inconsistency; invalid master alone cannot locate a block or distinguish damaged master, CRC directory, interrupted transaction, or header. Compare every block plus header and journal; a matching CRC is not semantic validity or mathematical proof of no corruption.
- Normal transaction: validate typed proposed values and cross-field constraints; quiesce dependent outputs/motion, record durable in-progress metadata before first changed byte, reject reentrant mutation while yielding to essential services. Update only differing bytes, recompute affected block(s) on readback, verify stored CRC(s), read the 22 fixed-stride CRC16 values (44 bytes), publish rotating master record commit marker last and verify before clearing transaction intent/allowing application. Do not recalculate unrelated block CRCs or blindly bless an unknown directory during a normal save. No-op writes neither data nor metadata. An M500 groups all changed blocks into one transaction/master commit; coalesce menu edits as appropriate without changing accepted save semantics. M501 checks first, M502 explicitly resets new settings, never updates old bank. A pending transaction and invalid master are not two independent damaged blocks.
- Durable transaction record needs target block bitmask, sequence/format/integrity and verified marker protocol; a power cut at ANY data, CRC, master, journal, or reset write may leave previous or new complete configuration, or explicit invalid/disabled state. A/B per critical record or bounded journal using spare space are candidates, not implemented. Do not choose an older higher cutoff on corruption; if a lower current cutoff cannot be proven, heat admission remains off until deliberate recommissioning. Master sequence wrap, torn markers and directory rollback must be modeled. Do not claim CRC detects arbitrary multi-bit corruption or hardware power-switch failure.
- Per-block CRC16 and rotating master CRC32 cost additional metadata writes versus a single 8-bit checksum. Read/CPU savings are large; physical write savings require per-byte update, transaction batching and no-op suppression. Master would otherwise be an EEPROM wear hotspot even though only one small payload changes. Evaluate ring rotation, journal/CRC cell wear, usage counters and menu key repeats, power-cut frequency, true EEPROM busy time and foreground yields. Current eprSetInt16/32/Float writes the entire changed multibyte value; use byte-level update semantics where safe, with verified asynchronous/nonblocking scheduling or bounded service gaps. No extra long interrupts-off region. Keep hot loop data in RAM, not EEPROM; no full 2048-byte iteration on ordinary change. A one-block update reads <=29 payload bytes + 44 stored CRC bytes = <=73 core integrity bytes, plus length/header/journal/verification traffic, vs existing 2047 payload scan. Boot defined payload 304 B + 66 slot metadata bytes + global metadata. CRC CPU overhead and actual read/write traffic/latency still need measured evidence.
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
| UiActions.cpp:66–71; UiSettings.cpp:217 | Manual Z buttons and menu increments save immediately, storeManualZOffset checks equality | Live compensation stays RAM only; preserve DOWN->UP 5um takeover; coalesce repeat bursts, save final value after key release/short idle, explicit save flush. Document abrupt power-loss window and test no lost accepted durable acknowledgement. |
| ui.cpp:1095–1111, Configuration.h:353–359 | Repeatable keys first delay500ms, decrement50ms, minimum50ms => up to20 actions/s under service budget | Do not commit each repeat; typed RAM edit + dirty bit, one durable commit per edit burst. For safety-critical edits immediate safe inhibition/application; do not delay required durable admission or treat old cutoff as committed. |
| UiSettings.cpp:530–620 | Microstep edit only idle, but eprSetByte/Int16/Float executed inside InterruptProtectedBlock before unprotect at618 | Move all EEPROM I/O outside masked region; keep only atomic derived state publishing in guard. Multiple changed cells can otherwise wait milliseconds per cell with IRQs disabled; actual upper bound needs linked timing. Group affected geometry/extruder blocks and invalidate homing/matrix. |
| GCodeMotion.cpp:511–544 | M3909 P/S save only if changed; P and S can trigger two full checksum scans in one command | Preserve normal unchanged startcode no-op; validate both and commit one block once. Repeated varying profiles legitimately persist under current contract, consider explicit volatile mode only with owner approval. |
| GCodeMotion.cpp:447–449,496–498 | Pressure min/max save pair and call checksum even if unchanged | Reject invalid pair, no-op entire transaction if unchanged, validate together. |
| GCodeThermal.cpp:308–341, RgbLighting.cpp:100–101,181–182; UiActions.cpp | FET/lighting state and RGB component setting persisted on commands/toggles; checksum even if byte unchanged on several paths | Separate live output changes from persistent defaults where operator compatibility allows; no new semantics silently. At least no-op suppression and coalesce interactive RGB adjustments. Automated accessory control can be a real repeated-write source, not observed frequency. |
| UiSettings.cpp:932–1052; TemperatureControl.cpp:940–958 | Manual DMIN/DMAX, PID/sensor edits; autotune stores only on successful requested save | One PID block transaction, preserve calibration coupling and manual DMAX. No background PID EEPROM writes in reviewed paths. |
| GCodeSystem.cpp:122,146; PreheatProfiles.cpp:62–89 | M500/M502 bulk save; menu profile edits | Compare actual serialized bytes, group changed blocks; no-op master/intent/CRC on identical values. M502 deliberate reset. |
| ZOffsetScan.cpp:505–538 | Changed scan points only | No-op and grouped scan configuration where appropriate. |
| HAL.h:405–431 | Byte setter compares old byte. Int16/Int32/Float setters compare whole value, then call eeprom_write_word/dword/block (not update variants) | Use per-byte difference updates. Example 32-bit counter 0x00000064 -> 0x00000065 should program one byte, not four; also avoid float equality pitfalls (NaN, signed zero). Explicit canonical wire bytes. AVR EEPROM hardware busy waits must yield or be split into bounded nonblocking steps with essential thermal services; do not call EEPROM from ISR. |

Microchip datasheet ATmega2560 EEPROM endurance at least 100000 erase/write cycles per cell (not hard failure threshold); datasheet https://www.microchip.com/bin/mchp/product-ds.ATmega2560.pdf. Worst-case illustrative hottest-cell lifetime at one physical write per commit: 1/day 274 y, 5/day 54.8 y, 20/day 13.7 y, 100/day 2.74 y; 20/s continuous 1.39 h. If a marker is programmed twice per commit, halve those bounds. Real writes depend on byte changes, cell temperature, programming mode, old wear and usage. No field failure reports established. Fresh address range reduces presumed prior wear only. Main priority is removing repeated writes and foreground/IRQ busy time, not universal wear-leveling. A small master/intent rotation is a candidate only if shared metadata is the actual aggregate hotspot; no per-feature log filesystem, garbage collection, dynamic allocation or routine counter wear leveling. First quantify master/intent/header cells (including commit marker twice), no-op, single-byte edit, repeat burst, M3909 two parameters, M500 changed/unchanged, usage counter rollover. A fixed master record with bounded recovery may suffice for realistic workloads; compare to simple ring (e.g. 8/16) and choose smallest safe measured solution. Never relax power-failure or critical cutoff invariants to save writes. Do not add a per-slot invalidation marker write on every save if verified durable global intent already covers same interruption. No routine checksum refresh on read-only operations; preserve fault evidence and validate pre-existing block before blessing modified bytes. Keep master only over committed CRCs; validate prior master/journal before modifying and don't legitimize unrelated corruption. No full upper-bank wipe/scan on ordinary writes.

Open implementation gates: EEPROM transaction power-cut model including journal/marker and master, paired feature commits, UI coalescing durability/explicit ACK semantics, upper host address ABI, full production source integration and four-board CI. The current task is design/audit only; no firmware writes or measured runtime changes. A69 full CI previously green.
