# Task0004 — legacy source audit, 2026-11-24

## Scope and reproducibility

- Legacy tree: `49a60a97eefb5dbd5b5eeea4344f090979c2d070` (community_development, read only).
- Review tree: `1fe264b804cc67aa5bf197d0106d84f2d7cf8796` (ai-firmware-review).
- Manager's subsequent target `70c0ad65682f37866c96d4b1f9a97380cde078ae` modifies only coordination files; production snapshot is unchanged.
- The exact GitHub compare has 190 changed paths, 116 C++/header/INO paths including 7 deletions. Every one of the 109 surviving/added paths was fetched at the immutable review SHA. The six Task0004-source-*.json files contain blob SHAs, line-based lexical function spans, static/volatile/extern candidates, guards, includes, waits and stale-comment candidates. All six JSON files parsed successfully, with 109 unique paths and valid blob SHA syntax. The scope is the compare's source set; a full Git tree scan is still needed to independently rule out unchanged paths outside that set.
- No production file edited. The inventory is lexical, not a C++ AST, preprocessor trace, call graph, linked image or runtime profile.

Run from a local checkout containing both commits (Python 3.8+):

```sh
python3 coordination/audits/Task0004-scan.py --self-test
python3 coordination/audits/Task0004-scan.py \
  --base 49a60a97eefb5dbd5b5eeea4344f090979c2d070 \
  --head 1fe264b804cc67aa5bf197d0106d84f2d7cf8796 > /tmp/Task0004-full.json
git diff --name-status 49a60a97eefb5dbd5b5eeea4344f090979c2d070 1fe264b804cc67aa5bf197d0106d84f2d7cf8796
git ls-tree -r --name-only 1fe264b804cc67aa5bf197d0106d84f2d7cf8796
```

The scanner uses Git object contents rather than the working directory; includes source files outside Repetier if present, except tests and coordination. It counts normalized line multisets and five-line windows across moved files, duplicate windows, macro token counts, low-reference function candidates and identifier spelling references. Neither comments nor strings count as symbol references. It deliberately does not infer unused includes or inactive variant guards from spelling. Resolve candidates using compiler dependency/preprocessing output, map/nm and explicit ownership review. No dead-code removal on lexical evidence.

## Source inventory

29,579 physical lines; 21,830 normalized lines (comments stripped, nonempty lines counted); 1,184 approximate single-line-signature function spans; 512 static/volatile/extern candidate lines (includes declarations, constants and local statics; NOT 512 mutable globals); 136 wait/loop candidates; 616 preprocessor guard directives; 208 include directives; 11 stale-comment candidates. No literal #if 0 guard matched. No removal-related comment matched in the scanned source. The JSON inventory preserves line evidence; function lengths include braces/comments and may be inaccurate for macro-heavy code or conditionally compiled bodies.

| Largest function | Approx. physical lines | Owner |
|---|---:|---|
| UIDisplay::parse | 1144 | 0009 |
| UIDisplay::changeSwitchCase | 885 | 0009 |
| UIDisplay::mainSwitchCase | 499 | 0009 |
| EEPROM::readDataFromEEPROM | 355 | 0005 |
| Printer::setup | 219 | 0014 |
| GCodeInput::parseAscii | 205 | 0008 |
| TemperatureController::autotunePID | 196 | 0010 |
| Extruder::manageTemperatures | 176 | 0010 |
| GCodeInput::parseBinary | 159 | 0008 |
| StepExecutor::performMove | 159 | 0014 (timing risk) |
| configureZOffsetScan | 159 | 0012 / 0014 coordination |
| RGB updateStatus | 140 | 0011 |
| loadCompensationMatrix / adjustBedMatrix | 137 each | 0006 |
| HAL::CPUDivU2 | 112 | 0013 (AVR timing risk) |

### Same-path normalized overlap independently measured

Whitespace collapsed, comments removed, nonempty lines retained (including directives); multiset intersection, contiguous five-line window intersection. Denominator is review-tree count, not original count. This is source overlap, not semantics, and can count coincidental short/common lines.

| File | Legacy lines | Review lines | Matched lines | Matched 5-line windows / review windows |
|---|---:|---:|---:|---:|
| Communication.cpp | 497 | 418 | 414 | 317 / 414 |
| Eeprom.cpp | 1195 | 783 | 540 | 251 / 779 |
| HAL.cpp | 1152 | 653 | 461 | 300 / 649 |
| Printer.cpp | 1459 | 577 | 338 | 71 / 573 |
| ui.cpp | 4521 | 784 | 684 | 467 / 780 |
| Extruder.cpp | 1162 | 279 | 196 | 69 / 275 |
| RF.cpp | 8960 | 213 | 142 | 18 / 209 |
| motion.cpp | 1150 | 206 | 106 | 22 / 202 |
| gcode.cpp | 1016 | 81 | 81 | 76 / 77 |

A small remaining file can be almost entirely old code because the other responsibilities moved to new modules. Full-tree cross-path comparison/duplicate and macro reports are implemented in the scanner but not yet executed against the real Git trees. Baseline source-byte figures in TASKS.md are historical, not remeasured linked sizes.

## Verified findings and prioritization

| Location / evidence | Classification / owner / action |
|---|---|
| Compare removes FatStructs.h, SDCard.cpp, SdFat.cpp, SdFat.h, u8glib_ex.h | Verified paths removed; not proof no linked external library. Task0013/0016 verify ELF and dependency files. Also removed RF2000v2.h (consolidated board header) and uiconfig.h; do not infer model/UI functionality removed. |
| Repetier.vcxproj lines 165–197 and .filters: 33 ClCompile entries in each, identical sets; compare shows 14 surviving .cpp absent | Verified manifest discrepancy, high integration risk, Task0013 (or authorized serialized integration). Missing GCodeCommands.cpp, GCodeInput.cpp, GCodeMotion.cpp, GCodeSystem.cpp, GCodeThermal.cpp, IdleStatus.cpp, MotionPlanner.cpp, MoveQueue.cpp, PrintJob.cpp, PrintPause.cpp, RgbLighting.cpp, StepExecutor.cpp, UsbSerial.cpp, ZMotion.cpp. CMake uses aux_source_directory; actual Visual Micro build failure remains untested. |
| CMakeLists.txt uses minimum 2.8 and ArduinoToolchain; vcxproj retains absolute Windows paths and old Arduino/AVR metadata | Stale-tooling candidate, Task0013. Verify current supported build paths before cleanup. |
| Includes across inventory | Arduino.h, AVR io/EEPROM/interrupt/pgmspace/watchdog, util/twi.h, inttypes.h, math.h, stdint.h. No direct Wire/SPI/SdFat/u8glib includes observed in scanned set; linked library footprint remains Task0003/0013 evidence. |
| Eeprom.cpp 1006/1016/1026/1036 | Four 4 ms delays in writeFloat/writeLong/writeInt/writeByte after serial EPR output. Duplicate pacing policy; Task0005 coordinate with 0002/0008. Preserve host EPR framing and pacing until serial throughput/replay verifies removal. Check checksum loop's periodic service, do not remove. |
| ExternalEeprom.cpp 24–60; TwiTransfer.cpp 119–128 | EEPROM operations finish active transfer, delay, then synchronous wait; wait services TWI and timeout but no explicit periodic heater service in that loop. Word operations split into bytes. Task0005/0013 measure worst-case service gaps, EEPROM write-cycle and bus fault behavior before changing. Distinguish from async StrainGauge. High safety risk if altered without replay. |
| BedMatrix.cpp 26–49 | Six file-local state declarations (offset, max, active slot, dirty byte, mesh array, extents). Query readers at 36–40; invalidate writer at 41; matrixSpan reads grid at 60–77; storage mutation in load/save/prepare. Already private to TU, not unowned globals; Task0006 consolidate cohesive state only with persistence checks. |
| TemperatureControl.cpp 20, 25–200 | autotuneIndex and function-static extruderTempErrors; manageTemperatures reads index, updates error count; autotune owns index lifecycle. Task0010 encapsulate with explicit reset/fault semantics. waitForTargetTemperature 230 services periodic actions; not a blind spin. Preserve heater shutdown and cancellation. |
| RgbLighting.cpp 15–69 | Named profiles/current/target component state in private namespace; setTargetColors writes target atomically; writeLeds writes timer registers/current; updateStatus drives mode/status. Task0011 consider color/profile structure; preserve volatile/interrupt boundary and PWM timing. |
| StepExecutor.cpp 105–128 | Function-static mass/waitSteps preserve compensation speed ramp; same function reads/writes both on timer path. Task0014 assess explicit owner/reset only; do not change ramp, cycles, ISR instructions accidentally. |
| HAL.cpp 32–44, 333–350, 476, 563–569 | ADC buffers, heartbeat, periodic flags, stepperWait and PWM requests already HAL-owned. temperatureAdcSample reads under interrupt lock; analogStart initializes buffers and waits first conversion intentionally. stepperWait is marked used; never classify by spelling. Task0013 evaluate only with assembly/trace. |
| Drv8711.cpp 257/261 | Two 5 ms reset delays surround DRV_RESET1 pulse; hardware sequencing, not proven removable. Task0001 preserve timing. |
| Printer.cpp 356/379, 435 | Direct motion waits service periodic actions; wrong-board boot halt is deliberately nonreturning before output init. Conditional/protected behavior, not dead code. Task0014 preserve. |
| UiActions.cpp 398; ui.cpp 979/984 | Filament script release loop calls periodic actions, prevents immediate abort on held OK. Two old DelayedAction TODO comments in key dispatch. Task0009 investigate before changing behavior. |
| Repetier.h 21–47 | Broad umbrella dependencies and model selection; RF2000/v2 share header. Task0015 split includes only after API ownership stabilizes. Check board conditional/retrofit paths with all four builds. |

## Open verification / gates

- Full tree scanner, cross-file references and actual reader/writer classification, macro/constant/unused include assessment, per-variant inactive guard proof and linked symbol inventory remain open. A function with one spelling occurrence can be used from a macro, callback, linker entry, ISR or external protocol; no new verified-dead functions claimed.
- Local toolchain/environment: Git 2.47.3 present, `git ls-remote https://github.com/Nibbels/Repetier-Firmware.git refs/heads/ai-firmware-review` failed DNS resolution; no local checkout or avr-g++ found. GitHub connector provided exact immutable file content but no build execution. No workflow on target per manager handover.
- Local `python3 /tmp/Task0004-scan.py --self-test` and `python3 -m py_compile /tmp/Task0004-scan.py` passed. Synthetic two-commit Git rename test passed (same five-line sequence, removed old path). Local git hash-object matched repository scanner blob `012e82511c524d541df37f7a8ee61398b194b6d4` exactly. Six JSON inventory files independently parsed and validated for count and unique paths.
- Four AVR variants and actual checkout replays not run; no Flash/SRAM/cycle deltas measured. Production diff empty, but that does not substitute for build gates. Do not merge before required gates pass. Task0003 is the shared build solution owner; no edits to its files.
