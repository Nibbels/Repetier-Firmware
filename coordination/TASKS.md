# RFx000 parallel refactoring tasks

Baseline audit: 2026-11-24, `3a27bd83dfc4711fa2694d18b311b67426f91936` on `ai-firmware-review`; original comparison: `community_development` (read-only).
This index and the per-task files are the durable coordination state, following the Gamemation Manager/Worker pattern with **worker self-merge**. All tasks initially OPEN; only wave 1 READY. A later wave is ready when every prior wave task is merged and verified; derive status from its task file and GitHub history, not edits to this index.

## Delivery and stopping point

Modernize the whole maintained firmware. Use the Task0004 inventory plus all files added since to account for every production module at final review. Clear state ownership, understandable control flow and removal of proven duplication are concrete benefits. Record a reason when an existing implementation already meets the standard. File moves, line churn and source-size reduction alone do not demonstrate completion.

For each task, identify the concrete benefit and the smallest change that delivers it. Close the task once its agreed scope and applicable checks are satisfied. An audit can recommend keeping the implementation when further restructuring has no demonstrated benefit. Record optional improvements as follow-ups. Use integrated wave commits as reproducible software checkpoints. Continue the planned refactoring through Task0016 with host replays, four AVR builds and task-specific evidence. Record each hardware-only question once for the consolidated final printer session; intermediate device runs are not wave prerequisites. Schedule further refactoring according to its contribution to a usable firmware release. Record any deferred scope and review downstream dependencies before rescheduling it.

## Waves

| Wave | Parallel tasks | Shared-file policy |
|---|---|---|
| 1 | 0001, 0002, 0003, 0004 | Isolated driver/communication internals; tooling and audit only |
| 2 | 0005, 0006, 0007, 0008 | Frozen external APIs; persistence vs matrix vs sensors vs parser |
| 3 | 0009, 0010, 0011, 0012 | One UI owner; thermal/RGB/scan contracts frozen |
| 4 | 0013 | Exclusive hardware/build changes |
| 5 | 0014 | Exclusive global ownership/API migration |
| 6 | 0015 | Exclusive common headers/configuration |
| 6b | 0017 | Exclusive operator dry-run retirement and legacy host handling |
| 7 | 0016 | Exclusive integration and final audit |

## Firmware PR checks

Open or update your PR against `ai-firmware-review`. GitHub Actions runs **RFx000 firmware / Host replays and four AVR builds** for changes outside documentation and coordination. Each PR has an independent job. Use the check on your latest PR revision for the standard host suite and AVR evidence. The checkout is GitHub's test merge commit; record the PR head, tested SHA from summary.json and run link in the task record. Download the firmware-evidence artifact for the four ELF/HEX files, maps, resource totals and logs (available for seven days). Keep important deltas in the task record. Compare against the appropriate target baseline and run any task-specific tests or timing checks. If the target advances, follow the synchronization rule below; a documentation-only update may reuse evidence for identical build and test inputs.

For local diagnosis, run the affected replay or use `python3 tools/task0003-baseline.py --avr-only ../new-evidence-dir` from a clean checkout with the pinned tools. A green hosted run covers the same standard build; workers need no local AVR installation to use CI. On failure inspect the first failing step and logs, then fix the task and update the same PR.

## Worker contract (read before work)

1. Read AGENTS.md, this index, own task and relevant mechanism docs. Verify actual current target HEAD and open PRs/branches. Target is `ai-firmware-review`, NEVER community_development. Check predecessor merge evidence. One worker per Task ID; claim with unique branch `task/TaskNNNN-short-name` from latest target. Check for an existing branch/PR for same task, resume rather than duplicate. Record status and ownership in own task file on branch; other task files and index are read-only.
2. Exclusive write scope is a hard boundary. Other files may be read and tested. New files must belong to your module and have distinct names. During waves 1–3, keep cross-module APIs, EEPROM layout, UI/protocol semantics and hardware timing stable. Do not change RF.h, Printer.h/.cpp, Configuration.h, Repetier.h, pins.h, HAL.h/.cpp, AGENTS.md, docs/maintenance-tasks.md or shared roadmap unless explicitly in scope. Do not perform repo-wide formatting. If an unavoidable cross-scope change is needed, document it in own task and defer to designated later task; don't silently expand ownership.
3. **Shared integration exception:** Visual Micro `Repetier.vcxproj` / `.filters`, existing build manifests, and source-extraction replay `tests/pause-sense-offset.py` / `.cpp` are shared write surfaces, not parallel development files. Workers may prepare their required deltas locally and create task-private tests. Integrate those shared-file deltas only in a short serialized merge window, based on latest target; never overwrite another worker's additions. If an existing shared replay fails because a function moved, adapt source extraction, don't weaken assertions. CMake auto-discovers .cpp files, Visual Micro does not. Do not merge an incomplete/broken supported build. If no reliable merge-window coordination is available, split into internal header-only changes or defer shared edits to the exclusive later wave; do not bypass tests.
4. All workers self-merge via their own PR: synchronize/rebase or merge latest target, inspect full diff and ownership, run the relevant tests. For changes affecting firmware or its build, build all four AVR variants (RF1000, RF2000, RF2000v2, RF2000 X44 retrofit), compare Flash/global SRAM and timing for hot paths. If target moves after checks, refresh and rerun affected checks before merge; verify PR expected HEAD SHA and latest target, use normal GitHub merge (no force push to integration branch). A shared-file merge window means only one such integration PR is in final synchronization/testing/merge at a time; claim via task PR comment with explicit release, check other open PRs before claiming. If contention/ambiguous claims, wait for other worker or defer shared edit; no manual manager approval needed for green in-scope merges. Do not make other workers resolve your conflicts. If toolchain/hardware unavailable, record BLOCKED or hardware verification open; never invent results.
5. Own task file is updated in PR with scope/diff, tests, exact tested HEAD, baseline and resource deltas, unresolved items and intended merge status. After merge verify target contains task change and PR is merged; add merge SHA/evidence to own task file only in a small follow-up if needed. Keep handover 3–5 lines with PR/commit link. No deleting other worker branches or changing unrelated files. Manager schedules waves and resolves genuine scope collisions; does not have to merge every PR.
6. Existing docs and behavior constraints apply: preserve EEPROM/protocol, retrofit connector definitions, heater safety, Z/DMS/ECMP/Advance and special panel offset transfer. Separate pure refactors from behavior changes; hardware-only verification stays open. No new feature removal without explicit proof/authorization. Do not claim zero conflicts; disjoint ownership prevents most textual conflicts, latest-target validation catches semantic conflicts.

## Staging work around the AVR toolchain

Read-only preparation can start in a later wave once an owner takes the task. Work on a task-private report or host replay can merge independently after reproduction on the exact checkout and diff review. This applies only when the diff has no effect on firmware, its configuration, or supported build inputs. Keep the original task open if binary evidence or implementation is outstanding. Record the evidence SHA, remaining checks and PR links in the task file. A partial PR does not release a successor wave.

Source audits (including Task0004) may close on a reproducible full source scan, manual checks and clearly marked unknowns. Leave per-variant reachability, linked usage and byte savings to Task0003 or the named downstream owner where evidence is missing. A fixture-only inventory is insufficient. Task0003 may land host-tested tooling first. Its baseline is complete only with real output for all four variants.

Production edits still start in their assigned wave. A worker can implement and run host tests locally, then keep the PR in draft for a shared build host. Supply the tested candidate SHA and compare against its target using identical core, compiler and flags. Attach or link four build logs, ELF/map/size results and the task-specific timing evidence where applicable. Record open physical checks separately. Workers verify the current target and self-merge as above. Host and AVR jobs can run on different machines against the same source. Existing wave ordering governs production integration.

## Task index

| Task | Wave | Title | Initial status |
|---|---|---|---|
| [Task0001](tasks/Task0001.md) | 1 | DRV8711 intern | READY |
| [Task0002](tasks/Task0002.md) | 1 | Communication und Zahlenformatierung | READY |
| [Task0003](tasks/Task0003.md) | 1 | Build- und Ressourcenbaseline | READY |
| [Task0004](tasks/Task0004.md) | 1 | Vollständiger Altcode-Audit | READY |
| [Task0005](tasks/Task0005.md) | 2 | EEPROM-Architektur | WAITING |
| [Task0006](tasks/Task0006.md) | 2 | BedMatrix | WAITING |
| [Task0007](tasks/Task0007.md) | 2 | Temperatursensoren | WAITING |
| [Task0008](tasks/Task0008.md) | 2 | G-Code-Eingang | WAITING |
| [Task0009](tasks/Task0009.md) | 3 | UI und Menüsystem | WAITING |
| [Task0010](tasks/Task0010.md) | 3 | Temperaturregelung | WAITING |
| [Task0011](tasks/Task0011.md) | 3 | RGB-Beleuchtung | WAITING |
| [Task0012](tasks/Task0012.md) | 3 | ScanSettings | WAITING |
| [Task0013](tasks/Task0013.md) | 4 | HAL, Arduino Core, fastio und Buildsystem | WAITING |
| [Task0014](tasks/Task0014.md) | 5 | Modulübergreifende Zustandsverwaltung | WAITING |
| [Task0015](tasks/Task0015.md) | 6 | Header, Konfiguration und Abhängigkeiten | WAITING |
| [Task0017](tasks/Task0017.md) | 6b | Operator-Dry-Run entfernen und thermische Fehlersperre erhalten | WAITING |
| [Task0016](tasks/Task0016.md) | 7 | Abschlussaudit und Integration | WAITING |

## Audit coverage

Prior audit points 1–2: byte/size history and already removed SdFat/FatStructs/SDCard/u8glib (Task0004/0016 verify, not reimplement). Point 3: normalized source overlap (0004). Point 4: driver, EEPROM, UI, Communication, HAL, thermal, matrix (0001/2/5/6/7/9/10/13/14). Point 5: library audit (0003/13). Point 6: obsolete build tooling (0003/13). Point 7: remaining modules, global state and includes (0008/11/12/14/15). Point 8: this wave schedule. Point 9 and additional audit steps: 0003/0004/0016. Existing hardware measurements and behavioral optimization roadmap are not silently marked complete.

Baseline C++/header/INO: 37 files / 2,186,047 bytes original; 109 files / 1,109,597 bytes review. All original surviving source files changed at blob level. Selected normalized code-line overlap: DRV8711 100%, Communication 99%, TemperatureControl 92%, ui.cpp 81%, TemperatureSensors and UiFormatting 75%, UiSettings 67%, EEPROM 66%, HAL 65%, BedMatrix 62%, ScanSettings 51%, Printer 48%, RGB 34%, ZOffsetScan 25%, HeatBedScan 20%. Heuristic, not proof of identical behavior.
