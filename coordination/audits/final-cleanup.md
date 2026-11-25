# Final cleanup after Tasks 0019 and 0020

Baseline: [b189d9b0519b90a0a7ea9dfb47f9c3ed0a4bed38](https://github.com/Nibbels/Repetier-Firmware/commit/b189d9b0519b90a0a7ea9dfb47f9c3ed0a4bed38), Task0020 merged in #87, no open integration PR at claim. Scope: final cleanup per manager handover. No printer operation, no code or configuration changes in the sketch. The legacy build systems remain separate from the verified release build.

## Root inventory and decisions

| File | Actual consumer / purpose | Disposition |
|---|---|---|
| `.clang-format` | clang-format and editor integrations discover this root configuration; historical changelog records formatter use. Not a CI build input. | Keep; do not format the repository or assume a particular formatter version. |
| `.gitignore` | Git ignores HEX, DB and VS metadata; not a compile input. | Keep, add root /api-doc/ for generated Doxygen output. |
| `AGENTS.md` | Maintainer/worker entry point, explicitly linked by README.md and coordination. | Keep. |
| `Doxyfile` | Doxygen HTML API generation; no workflow invocation. Repetier.ino contains the Doxygen mainpage. Previous INPUT, OUTPUT_DIRECTORY and PROJECT_LOGO were absolute paths to a developer checkout and an external Repetier-Host image. | Update paths to repository-relative Repetier and api-doc; remove external logo dependency. Run from repository root. No Doxygen executable available here; actual documentation generation is not verified, old settings may warn with current Doxygen. |
| `README.de_DE.md` | German entry documentation, linked from both README flag bars and solution items. | Keep; fix current branch instructions and link current CI evidence, mark Jenkins historical. |
| `README.md` | GitHub landing page; both READMEs, solution items and docs index use it. | Keep; same corrections. |
| `de.png` | German language flag in both READMEs. | Keep unchanged. |
| `us.png` | English language flag in both READMEs. | Keep unchanged. |
| `Repetier.sln` | Visual Studio 14 solution opens Repetier/Repetier.vcxproj; solution items include both READMEs and changelog. | Keep as a unit with both Visual Micro manifests; legacy, not a verified release build. |
| `flash_arduino.sh` | Linux operator wrapper for avrdude, wiring, ATmega2560, /dev/ttyUSB0, 115200. Equivalent command documented in both READMEs; Task0013 tested quoting with a stub. | Keep; not called by CI, no physical upload claimed. |
| `changelog.txt` | History and provenance, linked by docs index and historical line references in first-layer and Z-offset docs; solution item. | Keep original line numbering and contents. |
| `gpl.txt` | GPL v3 license text; firmware headers grant GPL v3 or later and require the license to accompany distribution. | Keep verbatim, link from docs index. |
| `repetier communication protocol.txt` | Historical binary protocol design note, explicitly incomplete; not an executable input. Current serial behavior belongs to GCodeInput/UsbSerial and serial-protocol replay. | Keep original reference (avoid breaking historic URLs), link from docs index and label historical. |

Both README language-bar image names and relative documentation paths resolve in the tracked tree; current review branch and workflow links point to the actual repository and workflow. The older external forum, wiki, images, upstream and Jenkins links are historical; remote availability was not verified. Do not silently substitute another firmware release for this candidate.

## Build paths as units

- **Verified Arduino path:** `Repetier/Repetier.ino` remains the sketch entry point. The official check uses Arduino CLI 1.2.2, AVR Boards 1.8.6, FQBN arduino:avr:mega:cpu=atmega2560 and tools/task0003-baseline.py for RF1000, RF2000, RF2000v2 and RF2000-X44. IDE installation and manual Linux avrdude upload remain available; real upload/reset still requires the consolidated device test.
- **CMake path:** Repetier/CMakeLists.txt, configure, configure.bat, cmake/ArduinoToolchain.cmake, cmake/Platform/Arduino.cmake and the bundled upstream/fork README files form one legacy build system. It uses CMake 2.8 minimum, legacy Arduino IDE SDK discovery (hardware/tools/avr, hardware/arduino/avr), --std=c++11, setup_arduino_sketch, aux_source_directory and generate_arduino_firmware; the three board options set MOTHERBOARD. The Bash/Windows wrappers create an out-of-source build and expose CLI or GUI configuration. CMake automatically discovers source files. It does not run in current CI and has no explicit X44 variant; current installed CLI package layout is not the same as the legacy SDK. Its library supports upload targets when a port is supplied, but this project does not set one. A CLion/Make/legacy SDK user or external build job could lose configuration, model selection, IDE integration and HEX generation by removal. No actual external users or successful contemporary build established. Retain the unit, including MPL 2.0 notices and original documentation, pending a deliberate support decision. Do not merely bump cmake_minimum_required or delete wrappers alone.
- **Visual Studio / Visual Micro path:** Repetier.sln + Repetier/Repetier.vcxproj + Repetier/Repetier.vcxproj.filters. The project declares VisualMicro capability, Mega ATmega2560 board configuration and wiring upload properties, maps the sketch and explicit C++/header entries, and references a generated __vm IntelliSense header. It also contains old machine-dependent Arduino/AVR paths, toolset and board recipes. Task0013 reconciled both manifests; tests/task0013-visual-micro.py asserts completeness and filters in standard CI. This is not an actual Visual Micro build test. A Windows Visual Micro user could lose opening the configured project, IntelliSense/source grouping, AVR build/upload/debug integration (if locally installed/configured) by removal. Native MSVC is not the AVR build. No actual IDE compilation or external user verified. Preserve the whole unit until retirement is explicitly chosen; do not delete only the solution or weaken the manifest replay.

## Legacy module names

| Pair | Actual boundary and direct consumers | Decision |
|---|---|---|
| motion.cpp/h | PrintLine stores prepared move geometry, queue/direct admission and warm-up/stop-tail operations; MotionPlanner owns lookahead, MoveQueue owns storage and StepExecutor owns pulses. Shared by firmware umbrella and planner/executor and source-extraction pause-sense-offset replay. | PrintLine.cpp/h would describe the primary class more precisely, but the header/source comments already explain the boundary and existing names are widely linked. Keep to avoid compile order, include, manifest and test churn without a behavioral/ownership benefit. |
| gcode.cpp/h | Decoded GCode command data, FirmwareState enum, printCommand reporting. GCodeInput owns parsing and serial admission; separate GCode* dispatch units execute commands. serial-protocol replay directly extracts gcode.cpp. | Keep; case-only GCode rename has portability issues on Windows, and name is already meaningful. |
| ui.cpp/h | UIDisplay, LCD transport and boot glyphs, input sampling, navigation and menu tables (instantiated once); UiFormatting/Settings/Actions already own rendering/edit/command paths. boot-eye, ui-workflows and pause-sense-offset replays directly read these paths. | Keep; UiDisplay naming would be reasonable but no concrete ownership gap warrants changing this broad, established interface in final cleanup. |

No changes to compile order, class layout, include guards, version, board flags, ISR, EEPROM, LCD, protocol or operator workflow. Existing Task0016/0019 inventories remain applicable (118 firmware C/C++/INO files after Task0020). The old AVR diagnosis note is moved using the identical Git blob, preserving both disassembly commands, the avr-nm command and links. Capture tooling now logs image SHA-256 in each variant's summary, in addition to the existing artifact, allowing comparison without trusting a size-only check.

## Evidence and remaining gates

Task0020 integrated baseline: [CI 45521424012](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45521424012), [four-variant artifact 14979823911](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45521424012/artifacts/14979823911); the Task0020 record reports all 16 output files and allocated ELF sections identical to Task0019. Baseline flash/global SRAM: RF1000 181372/6495, RF2000 186570/6653, RF2000v2 186574/6773, X44 181542/6633 bytes. Exact application/bootloader HEX and EEP hashes are in [Task0019](../tasks/Task0019.md).

Final PR CI and candidate comparison pending. Physical printer checks remain in [Task0016 device plan](Task0016-device-plan.md); Astra review and owner acceptance still required. Local checkout is unavailable due DNS/network restriction; no local AVR, Visual Micro or Doxygen toolchain. Inspection via pinned GitHub blobs, not a claim of a complete local git grep or a legacy build. 
