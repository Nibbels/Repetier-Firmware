# RFx000 manager history — retained technical decisions and evidence

Historical records, not current assignments. Owner-approved cleanup (2026-11-27): obsolete start prompts, superseded toolchain investigations, historical GPT-model assignments and redundant task-status messages removed. Current instructions: [AGENTS.md](../AGENTS.md), [TASKS.md](TASKS.md), [MANAGER_HANDOVER.md](MANAGER_HANDOVER.md). Per-task records and GitHub PR/CI history are authoritative for completed work.

## Native optimization baseline (2026-11-25)

The previous cleanup baseline is [96f2c8b](https://github.com/Nibbels/Repetier-Firmware/commit/96f2c8b95f021ec3291c15207fbf61063e2f986e) with [CI evidence](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45522064800/artifacts/14981100689). Preserve it as a comparison checkpoint. The native optimization dispatch established one production task at a time, verification and self-merge before its successor, and read-only preparation subject to ownership. Reject changes without measurable benefit or with safety/behavior regressions. No physical flash or test authorization. See [Task0021](tasks/Task0021.md) and [0022–0030](TASKS.md#atmega2560-native-optimization); PR #90 records the PID/autotune work.

## Final cleanup after Tasks 0019 and 0020

The source-directory cleanup moves the AVR diagnosis note to [docs/overflow_decompile.txt](../docs/overflow_decompile.txt), preserving its commands and links. See [final cleanup audit](audits/final-cleanup.md) for dispositions, build compatibility and evidence.

Review the following repository-root files during final cleanup. Record current consumers and a keep, move, update or retire recommendation for each. Check working links in both READMEs, licensing, and compatibility with supported build and flash workflows before proposing changes.

- Formatting, ignore rules and worker instructions: `.clang-format`, `.gitignore`, `AGENTS.md`.
- Documentation and language links: `Doxyfile`, `README.de_DE.md`, `README.md`, `de.png`, `us.png`.
- Build and flashing: `Repetier.sln` (review together with the Visual Micro manifests), `flash_arduino.sh`.
- History, license and protocol reference: `changelog.txt`, `gpl.txt`, `repetier communication protocol.txt`.

Date: 2026-11-24 (Europe/Berlin). Repository [Nibbels/Repetier-Firmware](https://github.com/Nibbels/Repetier-Firmware), integration branch **ai-firmware-review**, never community_development. Read [AGENTS.md](../AGENTS.md), [TASKS.md](TASKS.md), individual task files and live branches/PRs first. Repository is the authoritative durable state, not chat memory.

## Historical verification checkpoints (2026-11-24)

- Wave 1: PRs [#65](https://github.com/Nibbels/Repetier-Firmware/pull/65), [#66](https://github.com/Nibbels/Repetier-Firmware/pull/66), [#67](https://github.com/Nibbels/Repetier-Firmware/pull/67) and [#68](https://github.com/Nibbels/Repetier-Firmware/pull/68). Task0003 measured 51.7 seconds for 15 host replays and four AVR builds with packages present. The combined firmware saves 148–154 Flash bytes with unchanged global SRAM. Physical timing and stack measurements remained open at that checkpoint. Task0013 owned the 14 missing Visual Micro source entries identified by Task0004.
- Wave 2: [214baf4](https://github.com/Nibbels/Repetier-Firmware/commit/214baf4b504be71c0fce8661207930bd01d459f6), [four firmware images and evidence](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45449055408/artifacts/14946438316). EEPROM PR [#73](https://github.com/Nibbels/Repetier-Firmware/pull/73); sensor PR [#74](https://github.com/Nibbels/Repetier-Firmware/pull/74) with [green four-variant check](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45440129585); BedMatrix PR [#72](https://github.com/Nibbels/Repetier-Firmware/pull/72) and [candidate check](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45430403246).
- Task0008 receiver verification: [PR #71](https://github.com/Nibbels/Repetier-Firmware/pull/71), [green check](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45425850940). The receiver retains its architecture. Invalid binary text frames use the resend path. All four variants add 6 bytes of Flash and zero bytes of global SRAM. Physical throughput and a USB print remained open at that checkpoint.
- Wave 3 preparation: PRs [#75](https://github.com/Nibbels/Repetier-Firmware/pull/75) and [#76](https://github.com/Nibbels/Repetier-Firmware/pull/76) merged with passing checks; neither claims production changes. Reuse the merged workflow replays.
- Earlier final audit: Tasks 0001–0015 and 0017 checkpoint [PR #84](https://github.com/Nibbels/Repetier-Firmware/pull/84), merge 08caec4d, [green host and four-variant AVR verification](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45509948156). Task0016 audit PR #86 and Task0019/0020 sequencing are recorded in the corresponding task records.
- Additional host verification: checkout 9dab8d0, tests/serial-protocol.py passed (seven scenarios plus USART0).

## Retained technical findings and review rules

- Task0001 prepared a host regression trace; Task0002 identified INT32_MIN signed overflow and prepared a fixture golden replay (not actual checkout). See their task records for final results.
- M3902 Z0 interrupted-save finding: distinguish unsaved RAM edits from durable active-slot saves and decide intended recovery behavior before designing a cross-EEPROM change.
- UI tests must cover both actual LCD widths (see current AGENTS.md for geometry), preheat presets and manual Z handoff with the merged EEPROM contract.
- Thermal review: explicit zero heater demand on invalid readings, fault and autotune exit paths; test the bed-controller path and keep HAL freshness/physical latency separate.
- Hardware testing remains deferred until owner approval. After structural integration review the complete candidate, address material findings and recheck the resulting SHA. Stop after verifying behavior if no worthwhile change is identified. Preserve the checkpoint for comparison and record physical unknowns.
- Verify repaired scope and active PRs; do not duplicate branches. Do not lower build gates or invent test results. Verify exact-checkout host replays and four variant builds, compare Flash/SRAM and relevant timing. Draft PRs remain unmerged until gates pass. Use actual merge evidence (not a static status field alone).
- Release the next wave only after predecessor tasks are merged and verified, or explicitly revise the wave barrier with rationale and dependency checks. Use audit findings to refine future tasks. Keep manager status and handover durable in coordination; preserve worker-owned files.

## Baseline and plan

Audit baseline [3a27bd8](https://github.com/Nibbels/Repetier-Firmware/commit/3a27bd83dfc4711fa2694d18b311b67426f91936). Original 37 source/header/INO files, 2,186,047 bytes; review 109 files, 1,109,597 bytes (-49.2% source, not Flash). All surviving original C++/headers changed, but normalized line overlap is high in DRV8711 (100%), Communication (99%), TemperatureControl (92%), UI and EEPROM. SdFat, FatStructs, SDCard and u8glib removed (~492 kB source). Obsolete Arduino-CMake and IDE metadata remain; actual linked Arduino dependencies must be measured. Audit and plan commits [82eaa31](https://github.com/Nibbels/Repetier-Firmware/commit/82eaa311b11c7111143bb98fe5a07369205925c7), [1fe264b](https://github.com/Nibbels/Repetier-Firmware/commit/1fe264b804cc67aa5bf197d0106d84f2d7cf8796).

- Wave 1: Task0001 driver internals, 0002 Communication, 0003 reproducible build/resource baseline, 0004 read-only legacy inventory.
- Wave 2: 0005 EEPROM, 0006 BedMatrix, 0007 sensors, 0008 G-Code input.
- Wave 3: 0009 UI, 0010 thermal control, 0011 RGB, 0012 ScanSettings.
- Exclusive waves: 0013 HAL/Arduino/fastio/build, 0014 global ownership & motor params, 0015 shared headers/config, 0016 final audit.
- Freeze shared APIs and high-conflict files until assigned wave. Shared replay/build manifest edits need serialized final integration (see TASKS.md). No cross-scope opportunistic edits; no behavior changes mixed into refactors. Do not treat source similarity as proof of dead code. Preserve EEPROM layout, host protocol, safety, retrofit connectors, timing, Z/DMS/ECMP, operator Z DOWN→UP transfer. Four AVR variants (RF1000, RF2000, RF2000v2, RF2000 X44) and relevant replays; hardware-only tests explicitly open.

## Historical coordination repair

**Coordination repair:** initial task-generation template swapped prerequisites and write scope in all 16 files. This handover commit repairs all 16 on the integration branch (no firmware changes). For already active worker branches, synchronize latest target, resolve only task-file field conflicts in favor of corrected scope/prerequisites while retaining worker status and evidence; do not replace the worker's report. Workers may need an explicit resume prompt.

## Optional external compute — retained owner constraints

The owner already operates T14 with a Gamemation MCP/plugin and execution harness and has used self-hosted GitHub Actions on the laptop. For additional compute, prefer evaluating reuse of existing infrastructure over designing a new VM. T14 and ALUBUTYL remain optional, not prerequisites for RFx000; use existing hosted GitHub Actions first. Verify runner scope/registration, labels, exact checkout, toolchain availability and isolation from Gamemation jobs before any use. Do not assume the Gamemation MCP grants RFx000 workspace access. A separately registered repository-scoped runner with its own account, work directory and rfx000-avr label may be considered if genuinely needed; avoid disrupting existing runners or game workloads and keep untrusted fork code off the game VM. No installation or runner changes are authorized by this historical record. The owner prefers considering infrastructure changes when current credits run out. Consider task-appropriate acceptance gates for non-firmware audit/documentation changes, rather than demanding unrelated firmware builds; keep build gates for production changes and baseline measurement.
