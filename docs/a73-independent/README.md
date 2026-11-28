# Independent A73 qualification decision for PR #102

**Result for run 1072: qualification blocked.** A foreground critical section can wait for USART transmit progress while the only consumer is masked. This invalidates a finite global software response guarantee for every maskable interrupt, including heater lease expiry. The independent reset path has different premises. This report concerns the four linked application images. Production ownership remains with Task0034.

## Provenance

| Run | PR head | Tested merge | GitHub conclusion | Artifact |
| --- | --- | --- | --- | --- |
| [1058](https://github.com/Nibbels/Repetier-Firmware/actions/runs/46149413306) | 95415cff552517a40a844cc1935f588baf9a5069 | 11a26d209e801a02d626e13b82b5cd71173348e8 | Cancelled. Uploaded summary has zero failures and four verified images. | 15265705520 |
| [1072](https://github.com/Nibbels/Repetier-Firmware/actions/runs/46150218850) | bb4456ffec2c7c1caef5fae68721797352ab9695 | 94867e8726863a9ca1f8319a27fe6d3ac361d436 | Failure. RF1000 scan replay line 305 expected Limit and the specified pulse count. Four images verified. | 15265542310 |

The initial integration head was b2b09b732f0af2ccd23137f9a9386a90b0a77158. The accompanying JSON files record archive SHA-256, each ELF, map, HEX and listing digest, and the respective instruction addresses. Run 1057 was used only to inspect the artifact layout. The canonical combined images are at each board root. Separate reset-only and lease-only experiment directories would give different stack and SRAM numbers.

## Software defect and production handoff

In Repetier/UiSettings.cpp, the protected microstep-change block around line 516 calls Printer::updateDerivedParameter() before restoring SREG. In Repetier/Printer.cpp the XY and Z minimum jerk adjustments print ordinary protocol lines. Com::printF reaches HAL::serialWriteByte and UsbSerial::write in Repetier/UsbSerial.cpp. The linked tail poll is LDS, CP, BREQ to the same LDS. At insufficient remaining ring capacity the foreground cannot complete the message. Only the UDRE ISR advances tx_.tail. There is no timeout, space reservation or polling-mode transmit in that loop. The idle-motion menu admission permits a warm printer, and imposes no lower bound on available TX slots. The separate jerk menu sets the limit directly. Changes in physical resolution also change the computed minimum. The joined conditions matter: an ordinary microstep change with no warning or adequate free slots does not establish a stall.

| 1072 image | Masked call to derived settings | TX tail poll |
| --- | ---: | ---: |
| RF1000 | 0x22596 | 0x12934 |
| RF2000 | 0x12a74 | 0x22b2e |
| RF2000v2 | 0x12a7a | 0x22b2e |
| RF2000-X44 | 0x1aff4 | 0x107fa |

Task0034 should move reporting and derived calculations outside the long critical section, preserving the atomic publication of mutually consistent state and exclusive access during the resolution change. Cover TX backpressure and both jerk messages in a simulation. Check the alternate applyCurrentSettings() call for transitive reporting. Review all callers restoring an already-masked SREG. Changing the serial writer globally would change protocol semantics and would leave lengthy masked floating-point work on this path. Task0032 must carry the corrected admission/publication contract forward. A production repair and renewed linked qualification are required.

## Linked results for run 1072

| Board | SRAM before stack | Reviewed foreground model | IRQ envelope | Conditional remainder | Timer3 handler to final heater latch low |
| --- | ---: | ---: | ---: | ---: | ---: |
| RF1000 | 2036 B | 1669 B | 104 B | 263 B | 54 cycles |
| RF2000 | 1978 B | 1666 B | 104 B | 208 B | 56 cycles |
| RF2000v2 | 1860 B | 1666 B | 104 B | 90 B | 56 cycles |
| RF2000-X44 | 1967 B | 1672 B | 104 B | 191 B | 49 cycles |

Each root archive's recovery verification and recorded image hashes were checked. Reset-to-main in the application uses 3 stack bytes before main, with interrupts disabled. The existing checker verified the one linked constructor and all 16 candidate jump tables per variant. Main's return PC is already counted. No named malloc/calloc/realloc/operator new allocator was present in the 1058 symbol inventory. An unaccounted allocator, invalid pointer alias, unsupported nonlocal transfer, stack corruption or a different application invalidates the stack calculation.

The foreground walker follows instruction-level pushes, frame adjustments, caller depth at each call site, three-byte PC and tail transfers. Two narrower lifetime exclusions suffice without enabling all of the old phase hypotheses. StartLine rechecks admission after the initial wait, admits print motion and keeps active_ set through its periodic pumps. The ordinary idle transition tests that flag. Stop progresses to Draining/WaitingToOutput or terminal fault, with release confined to the outer command loop. A newly admitted scan, filament job, second startline or autotune cannot enter that suspended frame. Separately, an autotune entered with panel exclusion already owned has checked that no scan is running. Panel dispatch remains excluded during its pump, the receive service queues ordinary commands, and scan starts belong to panel/foreground command processing. Thus that context cannot launch a scan dispatch. This second exclusion is deliberately conditional on prior ownership. The reviewed source fingerprints gate these reductions. Candidate witnesses are conservative sums and can still contain impossible paths.

The SRAM remainder assumes the reviewed ABI, switch/callback target ownership, guard lifetimes, valid SRAM stores and no unbudgeted dynamic storage. It includes one Timer1 plus the largest nested handler at any foreground peak. It does not require the two peaks to coincide. No additional engineering allowance has been subtracted and no positive allocatable margin is certified. In particular 90 B cannot be assigned to a queue increase. The additional broad phase model on 1072 gives 1452/1436/1436/1447 foreground bytes in table order and is excluded from the budget.

## Interrupts and heater outputs

All distinct strong vector roots in the linked symbols were walked. Timer3 compare aliases share code. The Timer1 I/OCIE1A product graph reached 9904/10190/10190/9859 state pairs. At every modeled instruction boundary with I set, OCIE1A was clear. This includes the shared tail store reached with either bit value. The other walked ISRs retain I clear in their bodies. The result assumes valid indirect stores obey their device/SRAM ownership. Every explicit IN/OUT/LDS/STS to SREG or TIMSK0–5, decoded CLI/SEI/RETI and relevant BSET/BCLR is inventoried on reached foreground/ISR instructions. Calculated-address stores need ownership reasoning. This register inventory alone is insufficient to certify arbitrary pointer aliases.

| Handler | Stack incl. entry PC | 1072 modeled masked body cycles |
| --- | ---: | --- |
| Timer1 | 59 B | 17 unresolved masked cycles in the abstract graph per board. These are graph loops, not 17 CPU clocks. |
| Advance (Timer0 A) | 14/16/16/14 B | 138/160/160/138 |
| PWM (Timer0 B) | 45 B | 1679/1956/1956/1774 with byte-constant tracking only |
| Timer0 overflow | 12 B | 97 |
| USART RX / UDRE | 12 / 11 B | 66 / 68 |
| Timer3 compare | 22 B | 106/110/110/101 |

The timing table starts at the handler and excludes vector admission. The PWM values are conservative instruction-cost results. At 250000 baud and 8N1 one character takes 640 nominal CPU clocks. The PWM upper bounds alone neither establish a receive overrun nor guarantee loss-free reception. Production Timer1 uses bounded one-pulse guarded moves, but unguarded packing, direction-dependent paths, floating-point helpers and divider loops still appear in the masked graph. Safe response in a repaired image also requires competing-source minimum arrival intervals, delayed/coalesced flags, post-enable/return instruction ordering and the longest remaining foreground mask interval.

The Timer3 leaf clears PB4 at cycle 47 from its first handler instruction. RF1000 clears PH5 at 54. RF2000/v2 clear PH6 at 54 and PE2 at 56. X44 clears PE2 at 49. For these straight-line paths the decoder checks the compare vector slots, call destination, exact pin and AND mask. At nominal 16 MHz the final latch clears after 3.375/3.5/3.5/3.0625 microseconds. The lease clock is 12500 ticks at 16 microseconds per tick. An active output at maximum duty still requires a serviced compare/PWM or reset to go low. The pinned local renewal guard is 1024 cycles; the CI renewal check reports Timer3 280, Timer5 80, leaf 185 cycles. None of those local results bounds pending-compare-to-handler latency when the foreground can stall under CLI.

## Guarded motion and hardware limits

M3028 uses the Timer1 direct slot with one pulse per guarded tick. The direct executor samples the requested contact before output and retains directional limits. Scan/retreat publication uses an already-enabled Z driver, frozen correction and central Z pulse accounting. The positive emergency retreat has a distance admission limit; RF1000 uses its software maximum, the RF2000 family uses the upper switch. A thermal terminal fence precedes ordinary Timer1 dispatch. Run 1072 adds a reference requirement and checks the final driver/reference state. The failing scan replay prevents a clean disposition of these new paths. The separate 32-case mechanical emergency replay passed. Neither result measures physical contact, torque, actual pin waveform or travel.

Independent reset timing, fitted supervisor, gate bias during reset, physical power-stage opening, stuck switch/MOSFET and motor settling remain hardware dependent. A firmware GPIO write documents latch intent. No hardware tests or flashing were performed.

## Reproduction and requalification

The audit branch adds a composed entry point in tools/task0034/global-a73-evidence.py. Use Python 3 with assertions enabled and NetworkX. Its help describes the archive, expected SHA-256, tested merge and optional linked checks. Default identity checks reject failed suites. The explicit forensic option retains every archive failure and can inspect verified images after a host replay fails. The linked report is diagnostic and never grants release. When foreground ownership changes the reviewed-owners option rejects the unreviewed source fingerprint. The baseline foreground graph can still be generated without it. Changed source landmarks return null and require inspection. Five small mutation tests cover masking order, forced bit value, unknown writer and a shared writer reached with either bit value. Here, NetworkX was unavailable in the executor; the calculations used SciPy strongly connected components and Python graphlib topological sorting with an adjacency adapter implementing the graph operations used. The committed tools retain the repository's NetworkX dependency. Re-run on the final cutover ELF and canonical CI before adopting a new result.
