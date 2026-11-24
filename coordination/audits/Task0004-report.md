# Task0004 — legacy firmware audit

The source scan covers the complete legacy and review Git trees. The compiler inventory covers all 48 project translation units in each of the four Arduino Mega 2560 builds. The work in PR #68 consists of audit tools, source inventories and coordination records.

## Reproduce the source inventory

- Legacy `community_development` snapshot `49a60a97eefb5dbd5b5eeea4344f090979c2d070`.
- Review source snapshot `98ddc4f334576b3a9740af34cf678a2fd428d33c`. Its firmware and tests match `1fe264b8`, the snapshot used by the six `Task0004-source-*.json` files. Checked all 109 path/blob pairs against the full Git tree.
- `Repetier` tree `203ae92251bfe4c144b50ea9ebac20d333857cbf` and `tests` tree `a66e5d3f7107e1f89163ac87aaba91ed0cfbf995` also match the AVR-tested worker commit `e5e40135d628e7ea227ff38c2fcbddc232e5e589`.

Run from the repository with both snapshot commits available.

```sh
python3 coordination/audits/Task0004-scan.py --self-test
python3 coordination/audits/Task0004-scan.py \
  --base 49a60a97eefb5dbd5b5eeea4344f090979c2d070 \
  --head 98ddc4f334576b3a9740af34cf678a2fd428d33c > /tmp/Task0004-lexical.json
git diff --name-status 49a60a97eefb5dbd5b5eeea4344f090979c2d070 98ddc4f334576b3a9740af34cf678a2fd428d33c
```

| Source measure | Legacy | Review |
|---|---:|---:|
| C, C++, headers and sketches outside tests and coordination | 37 files | 109 files |
| Tracked source bytes | 2,186,047 | 1,109,597 |
| Original source paths surviving with identical blob | | 0 |

Source size fell by 1,076,450 bytes (49.2%). The old tree supplied matches for 14,160 of the review tree's 21,830 normalized lines and 6,355 of its 21,394 five-line windows, pairing each legacy occurrence at most once across all paths. Same-path counts were 6,712 lines and 3,396 windows. Normalization removes comments and collapses whitespace. These counts measure source reuse after moves and splits. Section sizes come from the linked builds below.

The output lists 1,184 approximate function spans, 154 low-reference function candidates, 511 lines with static, volatile or extern tokens, 582 constant declaration candidates, 1,899 macro definitions, 208 include directives, 616 guard directives and 136 possible wait/loop lines. Candidate counts include declarations, name collisions and ordinary loops. The scanner found 253 repeated five-line windows, of which 50 recur in different files. The repeated EPR send-and-delay blocks and the `showInvalidSyntax` branches in scan command handlers are useful duplication examples. Brace-only windows and common switch patterns need manual review.

| Location | Legacy normalized lines | Review normalized lines | Matched lines | Matched five-line windows |
|---|---:|---:|---:|---:|
| Communication.cpp | 497 | 418 | 414 | 317 |
| Eeprom.cpp | 1,195 | 783 | 540 | 251 |
| HAL.cpp | 1,152 | 653 | 461 | 300 |
| Printer.cpp | 1,459 | 577 | 338 | 71 |
| ui.cpp | 4,521 | 784 | 684 | 467 |
| Extruder.cpp | 1,162 | 279 | 196 | 69 |
| RF.cpp | 8,960 | 213 | 142 | 18 |
| motion.cpp | 1,150 | 206 | 106 | 22 |
| gcode.cpp | 1,016 | 81 | 81 | 76 |

| Longest function candidates | Approximate physical lines | Owner |
|---|---:|---|
| `UIDisplay::parse`, `UIDisplay::changeSwitchCase`, `UIDisplay::mainSwitchCase` | 1,144 / 885 / 499 | 0009 |
| `EEPROM::readDataFromEEPROM` | 355 | 0005 |
| `Printer::setup` | 219 | 0014 |
| `GCodeInput::parseAscii`, `GCodeInput::parseBinary` | 205 / 159 | 0008 |
| `TemperatureController::autotunePID`, `Extruder::manageTemperatures` | 196 / 176 | 0010 |
| `StepExecutor::performMove`, `configureZOffsetScan` | 159 / 159 | 0014 / 0012 |
| RGB `updateStatus` | 140 | 0011 |
| `loadCompensationMatrix`, `adjustBedMatrix` | 137 each | 0006 |
| `HAL::CPUDivU2` | 112 | 0013 |

## Linked AVR and host verification

Tested integration base `9dab8d061998b4064751df66e069d756b8ae62b9` and worker head `e5e40135d628e7ea227ff38c2fcbddc232e5e589`. The subsequent target `98ddc4f3` changed coordination only. Built from clean checkouts using Arduino CLI 1.3.1, prebuilt Arduino AVR core 1.8.6 and AVR GCC 7.3.0-atmel3.6.1-arduino7. All 11 Python replay entry points plus the standalone `machine-inactivity.cpp` and `z-offset.cpp` passed with host GCC 14.2.0. The replay tooling compiles the two standalone tests with `-std=c++11 -O2 -Wall -Wextra -Werror`.

| Target | Flash bytes | Global SRAM bytes | SRAM left before stack and heap |
|---|---:|---:|---:|
| RF1000 | 179,106 | 6,484 | 1,708 |
| RF2000 | 184,340 | 6,638 | 1,554 |
| RF2000v2 | 184,342 | 6,758 | 1,434 |
| RF2000 X44 | 179,232 | 6,620 | 1,572 |

Each row has zero Flash and SRAM delta versus the integration base. The HEX, EEP and bootloader-inclusive HEX matched byte for byte in every pair. The executed AVR instructions, including the timer hot paths, are therefore identical. Physical timing measurements remain open on their respective hardware tasks.

The runner from [Task0003 PR #67](https://github.com/Nibbels/Repetier-Firmware/pull/67) at `3bdf9ab6` provided the capture steps. Arduino CLI 1.3.1 passed the quotes embedded in its map option literally to the linker. The Work run used `-Wl,-Map=<absolute-path>,--cref` in a scratch copy of that script. The following commands reproduce the build flags and produce the input files for the compiler audit. Use the path to a clean checkout as `source` and a path without spaces for the build output.

```sh
source="$PWD"
evidence=$(mktemp -d)
while read -r variant flags; do
    build="$evidence/$variant/build"
    mkdir -p "$build"
    arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 \
      --build-path "$build" \
      --build-property "compiler.cpp.extra_flags=$flags" \
      --build-property "compiler.c.elf.extra_flags=-Wl,-Map=$build/firmware.map,--cref" \
      --verbose "$source/Repetier" > "$evidence/$variant/compile.txt" 2>&1 || exit 1
done <<'TARGETS'
RF1000 -DMOTHERBOARD=DEVICE_TYPE_RF1000
RF2000 -DMOTHERBOARD=DEVICE_TYPE_RF2000
RF2000v2 -DMOTHERBOARD=DEVICE_TYPE_RF2000v2
RF2000-X44 -DMOTHERBOARD=DEVICE_TYPE_RF2000 -DNUM_EXTRUDER=1 -DEXT0_EXTRUDER_COOLER_PIN=X44_PIN
TARGETS
python3 coordination/audits/Task0004-avr-audit.py "$evidence" "$source/Repetier" > /tmp/Task0004-avr.json
```

Use the compiler from `compile_commands.json` for the corresponding `avr-size -A`, `avr-nm -a -S -C` and `avr-objdump -h -d -C` on each `Repetier.ino.elf`. Flash is `.text + .data`. Global SRAM is `.data + .bss + .noinit` when the last section exists.

| HEX SHA-256 | Build |
|---|---|
| `f4959f2cc35adf0e52aa70273d980ab2ea6bb87797875b033d8534d432618a93` | RF1000 |
| `f7bb4d1ef845916d458ad16fbdf60d036791c18429524645cc01d118e03cb9e2` | RF2000 |
| `9da743f79af4d44f06a387b58cbddc7f77432186f7b4b0909e291023c44fe5f0` | RF2000v2 |
| `b4ed4cd89d635728664c8ee09252e7a15259da058ebfa76dd23b80170ee5bb40` | RF2000 X44 |

### Integration refresh after the Communication refactor

[PR #66](https://github.com/Nibbels/Repetier-Firmware/pull/66) merged while this audit was in progress. The full lexical scan, all four AVR builds and compiler inventories, and all 14 host replays passed again on `5388b4fe3b5719c9352fd45de71e953f0780b7ee`. Only `Communication.cpp` changed in the firmware tree. There are still 109 source files with 1,109,522 bytes. The updated tree has 14,139 matches in 21,821 normalized lines and 6,304 matches in 21,385 five-line windows against the legacy tree. The compiler state and guard counts below are unchanged.

| Target at `5388b4fe` | Flash bytes | Global SRAM bytes |
|---|---:|---:|
| RF1000 | 179,006 | 6,484 |
| RF2000 | 184,240 | 6,638 |
| RF2000v2 | 184,242 | 6,758 |
| RF2000 X44 | 179,132 | 6,620 |

The 100-byte Flash decrease per target comes from PR #66. The Task0004 firmware and replay tree IDs match the refreshed integration target.

## Compiler inventory

The audit tool replays the compilation database with GCC directive traces and macro use logging, collects included files from dependency files and reads direct references to mutable static storage from the initial GCC call graph. The generated JSON records all observed reads, writes and address-taking sites by function and translation unit. Indirect access through an object or pointer and symbols in inline assembly still need source review. The guard report lists builds that emitted lines in each branch body. An outer condition can match while its entire body is suppressed by a nested guard.

| Evidence across four builds | RF1000 | RF2000 | RF2000v2 | RF2000 X44 |
|---|---:|---:|---:|---:|
| Project translation units | 48 | 48 | 48 | 48 |
| Compiler-observed mutable static storage definitions before LTO | 126 | 161 | 161 | 162 |
| Project macro names expanded or tested as defined | 1,185 | 1,326 | 1,326 | 1,281 |

Of 333 nonempty guard bodies, 53 emitted no lines in any build and 182 appeared in only some builds. Examples are the RF1000 dual-extruder pin assignments, the RF2000v2 hardware defaults and the single-extruder branch used by RF2000 X44. Invalid-configuration stops for missing board or watchdog settings and incompatible pins are also absent from valid builds. Preserve their diagnostics. There were 227 project macro names unobserved across all four builds. This set includes retrofit options, unused old definitions and external configuration contracts.

Arduino dependencies were nine core headers plus the Mega variant pin header. The map resolves six Arduino core members in each build. `wiring.c` provides timekeeping and delay, `wiring_digital.c` provides GPIO, `WMath.cpp` provides `random()` used by Z-offset scanning, and the remaining members are `main.cpp`, `new.cpp` and `hooks.c`. The map also shows AVR libc numeric parsing in GCodeInput plus EEPROM, math and compiler support code. No Arduino library archives are selected. All four symbol lists lack SdFat, u8glib, TwoWire, SPIClass and Arduino HardwareSerial. The firmware provides USB serial through `UsbSerial` on USART0.

The full tree diff confirms the removal of `FatStructs.h`, `SDCard.cpp`, `SdFat.cpp`, `SdFat.h` and `u8glib_ex.h`, totaling 491,976 bytes in the legacy tree. `RF2000v2.h` and `uiconfig.h` were also removed and their responsibilities moved into the shared RF2000 header and panel code. SD removal with its protocol changes is recorded in [0ae25f2](https://github.com/Nibbels/Repetier-Firmware/commit/0ae25f2a57b8e68f6dc734ff6f849962581a9ee4).

## Manual checks and downstream owners

| Evidence | Classification and owner |
|---|---|
| `Communication.h:21` declares `axisNames` with neither a definition nor a reader in the tree. `ui.cpp:257` declares private `versionString` with no caller and no ELF symbol in four builds. The active version report references `UI_VERSION_STRING` at `GCodeSystem.cpp:238`. | Verified unused declaration and private constant in the audited tree. Task0002 or Task0009 can remove within scope. Low risk. |
| `Constants.h:22/45` defines `UI_PRINTER_COMPANY` and `VIRTUAL_AXIS`. `ui.cpp:209` defines `LCD_HOME`. Each name occurs only at its definition and is unobserved by GCC in all four builds. | Unused tracked macros, Tasks0015 and 0009. `HotendProfiles::v1` is an optional configuration preset even though there is no in-tree caller. Preserve retrofit settings. |
| `gcode.h:192` inline `GCode::setFormatError()` and `HAL.h:455` inline `HAL::timeInMicroseconds()` have no call site or linked symbol in the audited tree. | Unused within current firmware, Tasks0008 and 0013. Check compatibility before any public-header cleanup. |
| `Eeprom.h:66–68` retains legacy backlash addresses. `EPR_RF_230V_OUTPUT_MODE` appears in commented-out EEPROM accesses. | EEPROM layout and retired-field evidence, Task0005. Preserve addresses and verify existing persistence semantics. |
| `HAL.h:54` includes `Print.h` directly after `Stream.h`. Blanking just the direct `Print.h` include in scratch left `avr-g++ -E -P` output byte-identical for every sketch translation unit in all four configurations. Blanking only `Stream.h` changes the preprocessed declarations, and all 192 `-fsyntax-only` checks still pass. | Confirmed redundant `Print.h` include and `Stream.h` cleanup candidate, Task0013. Reproduce by replaying `compile_commands.json` on a sketch copy with `-c`, `-MMD` and `-o <object>` removed. |
| `Printer::axisStepsPerMM_` is defined in Printer.cpp. The RF2000 graph has direct readers in 20 translation-unit/function pairs and writers in EEPROM, the tool settings and the panel. `Printer::feedrateMultiply` has one writer in `Commands::changeFeedrateMultiply` and readers in command, planner and panel paths. | Cross-module calibration and motion state, Task0014. Preserve publishing and EEPROM behavior. |
| BedMatrix.cpp privately owns six working mesh variables. Query accessors read them, invalidate writes the format cell, and the storage path loads, adjusts and saves the mesh. RGB owns profiles and volatile target/current state in a private namespace. | Existing owners, Tasks0006 and 0011. Audit reset/persistence in BedMatrix and atomic publication in RGB. |
| `autotuneIndex` is assigned by `autotunePID` and read by `manageTemperatures`. The latter both reads and writes function-static `extruderTempErrors`. `StepExecutor::stepSlowedZCompensation` owns reads and writes of its function-static `mass` and `waitSteps`. | State has local owners. Tasks0010 and 0014 can make reset paths explicit. Heater safety and interrupt timing need separate validation. |
| GCC shows a write to `stepperWait` in the timer vector. AVR inline assembly also performs `lds` and `sts` on the symbol in HAL.cpp. | Live ISR state, Task0013. Review assembly before judging storage use. |
| Eeprom.cpp has four 4 ms waits after EPR serial reports. ExternalEeprom.cpp waits for the active TWI transfer and uses synchronous operations with a timeout. The motion, hotend and filament waits service periodic work. Two DRV8711 reset delays bracket the reset pulse. | Wait inventory, Tasks0005, 0010, 0013 and 0001. Check UART pacing, watchdog/heater servicing and the hardware reset sequence before changing behavior. |
| `ui.cpp:979/984` still refers to old DelayedAction dispatch. `ZScrewAdjustment.cpp:109` records the open RF2000v2 tuning check. | Comment review for Task0009 and hardware verification for Task0014. |
| Both Visual Micro manifests list 33 explicit C++ compile entries. The Arduino compile database needs 47 C++ sources plus the sketch. The missing source entries are GCodeCommands, GCodeInput, GCodeMotion, GCodeSystem, GCodeThermal, IdleStatus, MotionPlanner, MoveQueue, PrintJob, PrintPause, RgbLighting, StepExecutor, UsbSerial and ZMotion. | Verified manifest discrepancy, Task0013. Visual Micro compilation remains untested. CMake auto-discovers the C++ files and retains a 2.8-era ArduinoToolchain and legacy configure scripts. Check supported build paths together. |

The full inventories feed the listed owners. Treat candidates involving function pointers, menu tables, serial entry points, EEPROM layouts, ISR assembly and inactive retrofit branches as open until the relevant owner's checks resolve them.
