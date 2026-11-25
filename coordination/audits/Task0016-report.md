# Task0016 — final integrated audit through Task0018

## Current candidate — final bootlogo correction c484e193

**Supersedes every earlier candidate and image hash below.** Target `c484e193c1a6bf4da3477704ba9315f134a3775e`, includes Task0018 #85 and the final two-row CGRAM correction. Synchronized PR #86 at `fc81e4e9efe56202d5f9bd6eabbd6ede7cb8a75f`, verified synthetic checkout `c03c820063aa475c2747b493105ba2c0a481e0e8`. [Full audit CI 45516842966](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45516842966) succeeded; [firmware-evidence artifact 14978117811](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45516842966/artifacts/14978117811) downloaded, summary SHA matches and failures=[]; complete 28 host replay entry points (26 Python, two standalone), all four builds, original pre-wave baseline rebuilt, full Task0004 lexical and 192-unit compiler audit. The boot-eye replay independently asserts actual lower-left and lower-right CGRAM bytes and their DDRAM positions, not just equality to the source table; both 16- and 20-column logs PASS. Local targeted negative/positive replay of the previous artifact's real glyph renderer with just c484e193 applied rejects the old pattern and accepts the correction for both widths. No added firmware behavior in #86 relative to current target.

| Variant | Flash B | Pre-wave delta B | Global SRAM B | Pre-wave delta B | Remaining B | SHA-256 application HEX |
|---|---:|---:|---:|---:|---:|---|
| RF1000 | 181372 | +2266 | 6495 | +11 | 1697 | `94b56a665798b7fc3bbc5c6082533918a85a85e55c9fbc8310dfefbb5ab02931` |
| RF2000 | 186570 | +2230 | 6653 | +15 | 1539 | `d2faca3a9b2320fb4aa89395498155092e313ee62f223d953b0f41c2489cb23b` |
| RF2000v2 | 186574 | +2232 | 6773 | +15 | 1419 | `552edc2cddec77d8dec7d67d8384d8f94587fefe528cd2f38adfc2da765d047f` |
| RF2000-X44 | 181542 | +2310 | 6633 | +13 | 1559 | `650d75ad368903765beb11c0be4c2ba7b96416529c943abab03a7898b4e45616` |

Compared to previous combined audit artifact 14976774249, **exactly four changed loaded bytes in each of the eight application and bootloader-inclusive images** (Intel HEX checksums validated and addresses decoded). All changes lie in the symbol `bootEyeGlyphs`, offsets 19/20/27/28 (4→7, 7→0, 4→28, 28→0). Addresses: RF1000 0x505a/0x505b/0x5062/0x5063; RF2000 0x5388/0x5389/0x5390/0x5391; v2 0x538a/0x538b/0x5392/0x5393; X44 0x52dc/0x52dd/0x52e4/0x52e5. Every other loaded byte, all four EEP files, .text/.data/.bss sizes and ISR symbol addresses/sizes unchanged. Thus this patch does not change executable AVR timing or EEPROM ABI. Full source inventory unchanged (116 source files, 1,124,676 B; lexical counts and all per-file metrics identical to prior combined audit; all four AVR target inventories and guard maps exactly equal). The physical appearance, LCD ROM, boot duration, stack and watchdog behavior remain unmeasured.

Temporary full-audit workflow restored byte-for-byte to current target after verification. Final standard CI and integration evidence recorded below/in Task0016. No flash or physical operation authorized; separate Astra-Max review and owner acceptance required.


## Exact verification

Starting integration target `a15dbbdfa9212802bab5c5253e520b2e2d37f81a` (Task0001–0015 and 0017 integrated; checked merged PRs #65–68, #70–84, #69 closed unmerged infrastructure experiment). [PR #86](https://github.com/Nibbels/Repetier-Firmware/pull/86). Before header edits: branch 0601935c06e053501767117241763cfdbea3bf26, tested merge 3624703f3218be8a01d8f8383efc6c9fda64a9a1, [green run 45512426057](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45512426057), artifact 14976851011. After header edits: branch `4a80df0b52a5bb3a268067764661a85494e6fbb0`, exact tested merge `e82ca9059c379000381a78e4a5bdd21fa434fa8f`, [green run 45513102069](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45513102069), [artifact 14977071921](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45513102069/artifacts/14977071921). All 27 host replay entry points (25 Python and two standalone), four AVR builds, full lexical and compiler audit and rebuilt original wave baseline passed. The artifact contains both original and current build evidence, source/AVR inventory JSONs, ELF/map/disassembly and images. All 12 application/bootloader HEX and EEP images before/after header edits byte-identical; source comparison across all 116 RF1000 sketch source/header files after removing comments and generated line directives and normalizing whitespace gave zero non-comment differences. 28 modified files. Preserved licenses and attribution. No executable firmware changes by this worker. The temporary audit CI steps were restored to target workflow after capturing evidence.

**Historical integration blocker (resolved):** Task0018 [PR #85](https://github.com/Nibbels/Repetier-Firmware/pull/85) was opened concurrently after exclusive audit claim. Its head 10cc9cb024db37c69af0f0b8d042b901eae0a7a4 had green run 45512454790 but was still open and unmerged at verification. Task0018's own task record requests inclusion in final audit; it changes RF.cpp, BedMatrix and UI boot paths. Do not claim final audit or merge while this integration barrier remains. The task owner was asked in the PR to self-merge first, after which Task0016 must refresh the target and repeat verification. Do not merge another worker's PR. This is a verified Task0017-integrated checkpoint, not the final Task0018-inclusive release candidate.

## Reproduction

Use a clean full checkout (both historical SHAs present), Arduino CLI 1.2.2, AVR core 1.8.6, Mega ATmega2560, pinned compiler as in the capture. On the exact tested checkout:
```sh
python3 tools/task0003-baseline.py /tmp/rfx-final
python3 coordination/audits/Task0004-scan.py --self-test
python3 coordination/audits/Task0004-scan.py --base 49a60a97eefb5dbd5b5eeea4344f090979c2d070 --head HEAD > /tmp/legacy.json
python3 coordination/audits/Task0004-scan.py --base 3a27bd83dfc4711fa2694d18b311b67426f91936 --head HEAD > /tmp/wave-delta.json
python3 coordination/audits/Task0004-avr-audit.py /tmp/rfx-final "$PWD/Repetier" > /tmp/avr.json
```
Original baseline rebuilt at 3a27bd83dfc4711fa2694d18b311b67426f91936 using the same capture tool from the tested head (override ROOT to original detached worktree; no source modifications), --avr-only. Full command logs and baseline summary under task0016-original-build/ in artifact. CI workflow temporarily added these steps, then restored.

## Historical checkpoint resource and timing comparison (before Task0018)

| Variant | Original flash B | Current flash B | Delta B | Original SRAM B | Current SRAM B | Delta B | SRAM remaining before stack |
|---|---:|---:|---:|---:|---:|---:|---:|
| RF1000 | 179106 | 180196 | +1090 | 6484 | 6495 | +11 | 1697 |
| RF2000 | 184340 | 185394 | +1054 | 6638 | 6653 | +15 | 1539 |
| RF2000v2 | 184342 | 185396 | +1054 | 6758 | 6773 | +15 | 1419 |
| RF2000-X44 | 179232 | 180366 | +1134 | 6620 | 6633 | +13 | 1559 |

Original is pre-wave audit, not the already-reduced Task0003 final synchronized baseline. Flash .text+.data, SRAM .data+.bss+.noinit. Queue 2861 B, command queue 175 B, matrix 1300/1100/1218/1250 B, uid 253 B. Static totals are not stack high-water evidence. Original/current EEPROM images identical all variants. Header changes alone: zero Flash/SRAM and no executable instruction changes.

| Variant | Timer1 COMPA B | Timer0 COMPA B | Timer0 COMPB original → current B | Timer0 OVF / USART RX / UDRE / WDT B |
|---|---:|---:|---:|---|
| RF1000 | 1452 → 1452 | 264 → 264 | 842 → 830 | 148 / 84 / 96 / 96 unchanged |
| RF2000 | 1452 → 1452 | 314 → 314 | 1052 → 1046 | same |
| RF2000v2 | 1452 → 1452 | 314 → 314 | 1052 → 1046 | same |
| RF2000-X44 | 1452 → 1452 | 264 → 264 | 1040 → 1034 | same |

Verified isolated Timer0 COMPA STEP pulse from linked disassembly for all six enabled E outputs on both baseline and pre-header candidate; instruction sequence from rising edge to falling edge: two 2-cycle stores, LDI 1, 8 DEC 8 cycles, 7 taken and 1 untaken BRNE 15 cycles, two NOP 2 cycles, CBI 2 cycles = 32 cycles (2 µs at 16 MHz), unchanged. Byte-identical header images carry this result forward. Timer0 COMPB changed for ADC freshness (Task0013); smaller size does not establish worst-case cycles. Task0013 compared corresponding isolated hot paths and documented ADC service budget; Task0014 documented 6-cycle atomic threshold write; Task0017 compared 32 control-flow/opcode structures (not all operands). No claim of whole-firmware cycle equivalence or concurrent interrupt/physical timing. See predecessor task records and device plan.

### Historical application image SHA-256 (superseded checkpoint)

| Variant | Repetier.ino.hex SHA-256 |
|---|---|
| RF1000 | c657db341e935f34ddea505c3d50a2c23c6cc50315377bbf1599048d6d8db0eb |
| RF2000 | 7cbfe010767e0d8e972d1d28676413bdcc1eb706bceb7ddd2d6351e234f3727c |
| RF2000v2 | 4059bb3fbff9348304bcdd7470647bae9fde92cc94188c090ef069818f114318 |
| RF2000-X44 | d4b3352a983cbf80953e7917cad8be3de151882d2701f20b229e9c33512434a8 |

Artifact folders each contain application, bootloader-inclusive HEX and EEP; ordinary update uses application only as appropriate to verified board/upload procedure. Not approved to flash until manager/owner acceptance.

## Historical checkpoint source audit (before Task0018)

Legacy 37 source files / 2186047 B; pre-wave 109 / 1109597 B; current 116 / 1119989 B (before header corrections 1117255 B). 22019 normalized source lines; 13723 matched legacy line multiset and 5558 of 21555 five-line windows (counted at most once per old occurrence). Against pre-wave 21287 matched lines / 19679 windows. All 109 original paths accounted for, seven new headers. [Complete per-module ownership ledger](Task0016-inventory.md).

Lexical candidate counts: 1237 approximate functions, 158 low-reference functions, 524 static/volatile/extern lines, 597 constants, 1900 macro definitions, 239 includes, 632 guard directives, 133 wait/loop lines, 239 repeated five-line windows. These are not compiler call graph or dead-code verdicts. Actual GCC replay: 48 project units each, mutable static definitions 129/160/160/161, observed project macros 1189/1330/1330/1285; 341 nonempty guard bodies, 52 emit no lines in all variants, 185 conditional across variants, 225 macros unobserved in all. Compared to original 126/161/161/162 mutable definitions and 333 guard bodies. Compiler inventory lists observed direct reads/writes/address-taking, not resolved indirect ownership or inline assembly. No further deletion justified by counts alone.

Largest files: UiFormatting.cpp 54833 B, Eeprom.cpp 49885 B, UiSettings.cpp 44582 B, uimenu.h 42034 B, HAL.cpp 37860 B. Longest lexical functions: changeSetting 470 lines, renderField 468, mainSwitchCase 459, readDataFromEEPROM 354, renderCalibrationField 272, autotunePID 226, parseAscii 205, setup 195. Previously parse 1144, changeSwitchCase 885, mainSwitchCase 499. UI decomposition genuine but residual long dispatch domains; do not equate physical lines with linked costs. Duplicated five-line windows fell from 253 to 239; some are switch/guard patterns or brace-only. Macro names and unused public declarations may be external/retrofit contracts. See ownership inventory for retained old code, remaining architecture issues and optional scope.

Linked core maps retain WMath.cpp.o, hooks.c.o, main.cpp.o, new.cpp.o, wiring.c.o, wiring_digital.c.o; no SdFat, u8glib, TwoWire, SPIClass, Arduino HardwareSerial, Print or Stream symbols in four linked symbol inventories. Header transitive dependencies still include Arduino.h and its declarations. Original SD/graphics five paths remain absent. No physical verification of alternative IDE/CMake or stack/interrupt timing.

## Release questions

**Not ready for unconditional hardware release.** Raw electrical rail validity is an existing safety gap: supported conversion may return plausible 0 C from a broken sensor, and the shared fault debounce is not an electrical diagnosis. Task0013 freshness rejects expired batches but timestamp is publication time, not oldest conversion; a stalled timebase is not independent cutoff. Task0010/0017 software inhibits are proven for recognized faults, not all physical faults. Manager must review/resolve or explicitly constrain testing to isolated output loads before any live heater operation. M3902 durability, RGB profile reload, accessory X45 override, scan stop and measured timing remain distinct. Full [device plan](Task0016-device-plan.md) preserves Task0017 G-code sequence, with explicit isolation and stop rules. No printer flashed or energized. Separate manager maximum-effort review and owner acceptance remain mandatory.

## Task0018 standalone comparison (historical)

Owner requires PR #85 in the final candidate. Its standalone green [CI 45512454790](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45512454790) at test merge `fe236e9144349bd0be945af52825de3adc91882d` has artifact [14975592524](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45512454790/artifacts/14975592524), downloaded and checked summary failures=[]; boot-eye replay present. Relative to the Task0017 checkpoint: Flash +1176/+1176/+1178/+1176 B, global SRAM unchanged (RF1000/RF2000/RF2000v2/X44). See inventory for the six existing production paths and boot callback design, and stage 1a of the device plan for both LCD widths, error handling and boot latency. These are standalone PR results; no final combined audit or image claim until #85 merges and #86 is synchronized and tested.

## Historical combined audit before c484e193 (superseded image set)

Task0018 #85 merged as `9abbdd9444a63cbfaaa0b6edaa0988e9651f58f7`; current target before #86 `1ffaeb49fb7968df7b69ee8087be858b17af1cbd`. Only #86 remains. Both parents merged on worker branch, preserving the three overlapping header comments in RF.cpp/ui.cpp/ui.h without changing Task0018 code. [Run 45515228916](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45515228916) **success**, tested head `00285d07db5f76ddc950161cc8cb146cc12f54af`, exact tested checkout `f23628147be0ef34b2e99250d22b027b4b957eb4`. [Artifact 14976774249](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45515228916/artifacts/14976774249) downloaded and verified, summary failures=[], baseline rebuild and both source comparisons and full compiler audit green. 28 host entry points (26 Python, two standalone), including boot-eye 16/20 columns PASS, four AVR builds. Original toolchain and baseline unchanged. Temporary workflow restored for normal PR check. No live test.

| Variant | Pre-wave flash | Final flash | Delta | Pre-wave SRAM | Final SRAM | Delta | SRAM before stack |
|---|---:|---:|---:|---:|---:|---:|---:|
| RF1000 | 179106 | 181372 | +2266 | 6484 | 6495 | +11 | 1697 |
| RF2000 | 184340 | 186570 | +2230 | 6638 | 6653 | +15 | 1539 |
| RF2000v2 | 184342 | 186574 | +2232 | 6758 | 6773 | +15 | 1419 |
| RF2000-X44 | 179232 | 181542 | +2310 | 6620 | 6633 | +13 | 1559 |

Relative to standalone Task0018 artifact, all four Flash and SRAM totals identical, EEP files byte-identical. Decoded Intel HEX (checksums validated, extended segment and linear addresses resolved) compared byte-by-byte: all eight application and bootloader-inclusive HEX images differ by exactly **one** byte each, the M115 COMPILED date changing Nov 24 to Nov 25 2026 (addresses 0x2423/0x23aa/0x23aa/0x22ff). All other loaded bytes identical. Thus executable content unchanged by synchronization/header edits. ISR symbol lengths remain those in preceding timing table, including Timer1 COMPA 1452 bytes and Timer0 COMPA 264/314/314/264; Task0018 adds only foreground boot work. Same executable bytes as standalone #85, whose task record documents structural opcode/branch comparisons. No new global SRAM; stack and actual boot/service latency remain unmeasured.

| Variant | Final application HEX SHA-256 (audit artifact) |
|---|---|
| RF1000 | c13e4cc0b7f1b1d96b268cfdbc47591913436a3532cc8555d3b7704d11cd1d4d |
| RF2000 | e22ae54b91f4956faca4c2ffc332645515376fe113f9634f36e514a3958255ed |
| RF2000v2 | 96b7a548abebb41d0223b5d7e06da14682a4b930d13087fc8bdebcfad8568130 |
| RF2000-X44 | 56cc1cc7df5897c709c5d03585b8bc7df1fbadaada75bdfd56689f02c0c4ffec |

Source audit: 116 files / 1,124,676 B; no new firmware paths from Task0018. 22,129 normalized lines, 13,738 matched legacy lines and 5,550 of 21,665 five-line windows; against pre-wave 21,283 matched lines / 19,628 windows. 1,243 approximate functions, 158 low-reference functions, 526 static/volatile/extern candidates, 605 constants, 1900 macros, 239 includes, 632 guard directives, 136 waits, 239 duplicate windows. Largest five files and longest functions unchanged from checkpoint. GCC 48 units per variant (192 total), mutable state counts 129/160/160/161, observed macros 1189/1330/1330/1285; 341 guard bodies, 52 never emitted, 185 conditional, 225 macros unobserved in all four. Task0018 now explicitly accounted for in inventory and device stage 1a. No further production edits justified in worker scope. See release questions above (especially electrical sensor rail validity, independent heater protection and physical timing). Candidate is for separate Astra-Max review, **not** authorization to flash or heat.

## Standard workflow verification and latest target synchronization

- [Normal CI 45517259745](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45517259745) green, exact tested branch `ae3f4a7627a0a706f7869fbee5f334d7a31ab72e`, test merge checkout `30ff3f60031c743765b48c729dd4556bf54bddf0`; [artifact 14977693716](https://github.com/Nibbels/Repetier-Firmware/actions/runs/45517259745/artifacts/14977693716). Downloaded: summary failures=[], both boot-eye widths PASS, all target sections and resource totals equal full-audit artifact 14978117811; all twelve application HEX / bootloader HEX / EEP files byte-identical. This normal workflow is byte-identical to target. All prior hashes remain superseded by the c484e193-inclusive set above/in the report.
- Target advanced from `c484e193c1a6bf4da3477704ba9315f134a3775e` to `884c60db0c825edbdf72c50fced260b213cbcdaa` during the normal check, **coordination files only** (MANAGER_HANDOVER.md, TASKS.md, new Task0019.md); no changes to source, tests, toolchain or workflow. Merge latest target into #86, retain manager's new dispatch unchanged, and verify latest inputs again. Manager schedules Task0019 only after #86, so Task0016 delivers a verified baseline for 0019 and the Astra-Max review; its subsequent firmware changes will require new candidate evidence. No physical tests.
