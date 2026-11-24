# RFx000 Manager 01 → Manager 02 handover

Date: 2026-11-24 (Europe/Berlin). User wants a long-lived manager chat in ChatGPT Work (Astra selected by user); worker chats currently use GPT-6.2 High outside Work. Repository [Nibbels/Repetier-Firmware](https://github.com/Nibbels/Repetier-Firmware), integration branch **ai-firmware-review**, never community_development. Read [AGENTS.md](../AGENTS.md), [TASKS.md](TASKS.md), individual task files and live branches/PRs first. Repository is the authoritative durable state, not chat memory.

## Mandate

Coordinate firmware modernization, maintain task boundaries and wave scheduling, audit correctness and test evidence, unblock workers pragmatically, and refine tasks as evidence arrives. Workers implement, test, synchronize and merge their own PRs; manager does not routinely take over or approve them. User wants minimal merge conflicts and autonomous work. Maintain concise durable handovers (3–5 lines plus links in chat; detailed evidence in task files), update this manager handover when important decisions or blockers change. Manager may implement a task directly only when intentionally assigned or when a small coordination repair is needed; do not duplicate active work. Do not claim background work or tests not run.

## Baseline and plan

Audit baseline [3a27bd8](https://github.com/Nibbels/Repetier-Firmware/commit/3a27bd83dfc4711fa2694d18b311b67426f91936). Original 37 source/header/INO files, 2,186,047 bytes; review 109 files, 1,109,597 bytes (-49.2% source, not Flash). All surviving original C++/headers changed, but normalized line overlap is high in DRV8711 (100%), Communication (99%), TemperatureControl (92%), UI and EEPROM. SdFat, FatStructs, SDCard and u8glib removed (~492 kB source). Obsolete Arduino-CMake and IDE metadata remain; actual linked Arduino dependencies must be measured. Audit and plan commits [82eaa31](https://github.com/Nibbels/Repetier-Firmware/commit/82eaa311b11c7111143bb98fe5a07369205925c7), [1fe264b](https://github.com/Nibbels/Repetier-Firmware/commit/1fe264b804cc67aa5bf197d0106d84f2d7cf8796).

- Wave 1: Task0001 driver internals, 0002 Communication, 0003 reproducible build/resource baseline, 0004 read-only legacy inventory.
- Wave 2: 0005 EEPROM, 0006 BedMatrix, 0007 sensors, 0008 G-Code input.
- Wave 3: 0009 UI, 0010 thermal control, 0011 RGB, 0012 ScanSettings.
- Exclusive waves: 0013 HAL/Arduino/fastio/build, 0014 global ownership & motor params, 0015 shared headers/config, 0016 final audit.
- Freeze shared APIs and high-conflict files until assigned wave. Shared replay/build manifest edits need serialized final integration (see TASKS.md). No cross-scope opportunistic edits; no behavior changes mixed into refactors. Do not treat source similarity as proof of dead code. Preserve EEPROM layout, host protocol, safety, retrofit connectors, timing, Z/DMS/ECMP, operator Z DOWN→UP transfer. Four AVR variants (RF1000, RF2000, RF2000v2, RF2000 X44) and relevant replays; hardware-only tests explicitly open.

## Live snapshot before handover

Target was 1fe264b when inspected. Four worker chats were started by user. **This snapshot is not a live status; refresh on entry.**
- Task0001: [draft PR #65](https://github.com/Nibbels/Repetier-Firmware/pull/65), branch task/Task0001-drv8711. Host regression trace prepared; no production edit; blocked by incorrect scope and missing AVR toolchain.
- Task0002: [draft PR #66](https://github.com/Nibbels/Repetier-Firmware/pull/66), branch task/Task0002-communication. Host golden replay prepared (fixture, not actual checkout); INT32_MIN signed overflow identified. No production edit. Same two blockers.
- Task0003: branch task/Task0003-build-baseline observed at 1fe264b; build baseline not yet verified.
- Task0004: branch task/Task0004-legacy-audit observed at bcf2e22; task marked IN_PROGRESS, no open PR observed at snapshot.

**Coordination repair:** initial task-generation template swapped prerequisites and write scope in all 16 files. This handover commit repairs all 16 on the integration branch (no firmware changes). For already active worker branches, synchronize latest target, resolve only task-file field conflicts in favor of corrected scope/prerequisites while retaining worker status and evidence; do not replace the worker's report. Workers may need an explicit resume prompt. No production worker PR has been merged as of inspection.

## Immediate manager priorities

1. Verify repaired scope and active PRs, communicate correction/resume to existing workers (via user prompts or PR comments as appropriate); do not create duplicate branches. Review Task0003/0004 results.
2. Solve the real toolchain blocker once for all workers: their normal-chat containers reported host g++ but no avr-gcc/avr-g++, arduino-cli or PlatformIO and no network clone; target had no GitHub Actions workflow. Work may offer a different environment but verify access, exact checkout, toolchain and runner before assuming it. Investigate reproducible build and GitHub Actions or user-provided runner, respecting task ownership. Let Task0003 own the solution; if CI manifest required, explicitly reserve its ownership for this task. Do not lower build gates or invent test results. If environment remains blocked, report precisely and request only necessary user action.
3. Verify exact-checkout host replays and four variant builds, compare Flash/SRAM and relevant timing. Draft PRs remain unmerged until gates pass. Use actual merge evidence (not a static status field alone).
4. Release the next wave only after predecessor tasks are merged and verified, or explicitly revise the wave barrier with rationale and dependency checks. Use audit findings to refine future tasks.
5. Keep manager status and handover durable in coordination; preserve worker-owned files. No firmware modification in this handover.

## Start prompt

Take over as RFx000 Manager 02. Read AGENTS.md, coordination/TASKS.md and coordination/MANAGER_HANDOVER.md on ai-firmware-review in Nibbels/Repetier-Firmware. Refresh actual GitHub state and active worker branches/PRs. Resolve coordination and test-infrastructure blockers, maintain the task waves, ensure workers self-merge, and keep durable manager notes. Do not duplicate active worker work. Start with the scope correction and AVR build blocker.

## Infrastructure clarification from user (2026-11-24)

The user already operates a T14 VM with MCP/plugin and execution harness for Gamemation, and has previously used self-hosted GitHub Actions on the local laptop. Prefer evaluating reuse of that existing infrastructure over designing a new VM. Verify runner scope/registration and labels for Nibbels/Repetier-Firmware, AVR toolchain availability, actual checkout and isolation from Gamemation jobs. Do not assume the existing Gamemation-specific MCP grants RFx000 workspace access. A separately registered RFx000 runner on the same VM is an option if needed; avoid disrupting existing runners or game workloads. Task0003 tooling is available in draft PR #67; Task0004 source inventory is in draft PR #68. All four draft PRs were unmerged at latest inspection. User prefers considering the infrastructure change when current credits run out; no VM changes or runner installation have been performed by this manager. Consider task-appropriate acceptance gates for non-firmware audit/documentation changes, rather than demanding unrelated firmware builds; keep build gates for production changes and baseline measurement.

## Manager 02 decision, 2026-11-24

The user moved Tasks 0001–0004 to GPT-6.2 Sol Max in Work. Continue their existing branches and PRs. Tasks 0005–0016 now specify host preparation and target evidence individually. See [staging rules](TASKS.md#staging-work-around-the-avr-toolchain). The manager cloned target 9dab8d0 and ran tests/serial-protocol.py successfully (seven scenarios plus USART0).

T14 feasibility: the Gamemation status endpoint reported GM-CI-T14, Windows x64 and one runner process. Firmware registration and an installed toolchain are unverified. First try a clean checkout at the candidate SHA and Task0003's proposed pinned Arduino AVR Boards 1.8.6 with Arduino CLI and Python. Its draft script accepts --avr-only and resolves Windows .exe tools. Run the host suite in Work on the same SHA. Time a full four-variant run and retain the artifacts before deciding on permanent CI. A later repository-scoped runner with its own account, work directory and rfx000-avr label could serialize trusted firmware builds independently of game jobs. Keep untrusted fork code off the game VM. No installation or runner changes were requested in this review.
