# UART recovery and host interoperability

2026-11-28 owner decision, Task0032. This contract supersedes the earlier
terminal-on-first-detected-RX-loss prototype. P0 exclusive production
ownership is unchanged. This is a source review and implementation contract,
not a new shipping parser or a certification of all host versions.

## Inspected evidence

| Host | Evidence and concrete behavior | Limits |
|---|---|---|
| Repetier-Host | [Historical open-source PrinterConnection.cs](https://github.com/InlineTwin/Repetier-Host/blob/master/src/RepetierHost/model/PrinterConnection.cs), inspected blob c2a7d7ee454781d565ad98ef4aeb1cd2ee8b5866. ResendLine (281ff): looks up low-16-bit line in history; closes on more than five consecutive errors or empty history; clears outstanding credit for streaming; in binary mode discards output buffer, waits, sends 32 zero bytes and waits, then retransmits history from requested line. Response parser (758ff) detects Resend:, ignores its following ok; ordinary ok advances credit. | Historical source, not current proprietary Host 2.3.2. Test current supported Host binary/ASCII and ping-pong/streaming in software before claiming compatibility. |
| Repetier-Server | [Communication errors](https://www.repetier-server.com/knowledgebase/4012/), [manual 1.4](https://www.repetier-server.com/manuals/1.4/index.html), [Host configuration](https://www.repetier.com/documentation/repetier-host/rh-installation-and-configuration/). Numbered/checksummed resend, occasional corrected errors tolerated, multiple outstanding bytes governed by receive credit, numbered ok for missing acknowledgments. | Public documentation, not current proprietary transport source; verify actual binary sync and timeout behavior. |
| Repetier host stop | [Upstream Pause handling.txt](https://github.com/repetier/Repetier-Firmware/blob/master/doc/Pause%20handling.txt): RequestStop: intended for Host >=2.0 and Server >=0.9; host can ignore hints. [Observed upstream fatal response](https://forum.repetier.com/d/9773-regression-in-current-devel-firmware-stops-immediately-claiming-homing-failed/3). RFx000 Printer::stopPrint already sends RequestStop: and // action:cancel; Communication defines fatal:. | Never rely on host stop action to make physical hardware safe. No M999 resume after fatal transport loss. |
| OctoPrint | [Current serial_comm.py](https://github.com/OctoPrint/OctoPrint/blob/main/src/octoprint/plugins/serial_connector/serial_comm.py), inspected blob 4ab87eed45bb943f821128ab9fe9b03308b8b17e: _handle_resend_request (4430ff), history/line selection, consecutive-resend limit, stale-request filtering. _handle_errors (4130ff) treats fatal: and !! as error paths; fatal: can disconnect; settings/hooks may override. Recoverable errors and resend-request errors matched by substring before fatal handling. [Configuration](https://docs.octoprint.org/en/main/modules/schema.html) documents 10 consecutive resends by default, supportResendsWithoutOk detection, swallowOkAfterResend. [Action commands](https://docs.octoprint.org/en/main/features/action_commands.html). | Do not put checksum, format error, expected line or other recoverable-error keywords into the fatal line. Do not send an extra fake command ACK. Host can send M112 on error; already terminal firmware must not auto-reboot. |
| Cura direct USB | [USBPrinterOutputDevice.py](https://github.com/Ultimaker/Cura/blob/main/plugins/USBPrinting/USBPrinterOutputDevice.py), blob 85b98e532b6d74e534a88af6105fbeec69e9b89a: numbered XOR print lines, Resend/rs sets file position; ok drives next line, !! cancels print (335ff). Cancel sends heater-off, fan-off, homing and motor-disable commands. | Unnumbered manual commands also exist. Local fatal lock must reject cleanup motion/motor disable/restart requests. |
| Printrun/Pronterface | [printcore.py](https://github.com/kliment/Printrun/blob/master/printrun/printcore.py), blob 4819738d11d3fa1e6d9fc37b4579deabbb7d09e0: _listen (327ff) recognizes Resend/rs and immediately sets clear; _sendnext (587ff) repeats stored sentlines; _send numbers/checksums ordinary print lines; manual priority traffic can be unnumbered. Error prefix logs/callback; no universal automatic cancellation demonstrated. | Replay the separate Resend and following ok timing, including races. Cannot guarantee host UI/job cancellation. Local printer lock mandatory. |
| PrusaSlicer, OrcaSlicer and other upload clients | [PrusaSlicer host upload documentation](https://help.prusa3d.com/article/sending-files-to-octoprint-duet_1663): upload G-code to print host; the connected host owns printer transport. | Do not claim a new UART protocol for each slicer. |

Evidence inspected 2026-11-28. Blob SHAs identify source contents; branch URLs
can move. These are source/documentation observations, not live integration tests.

## Existing RFx000 wire contract

Pinned reviewed production source 188eeeb0c9a272d93a7230247b794748318ece86,
Repetier/GCodeInput.cpp, UsbSerial.cpp, Printer.cpp, Communication.cpp.
Recheck actual final P0 source before adapting.

- Keep established ASCII responses in ASCII and Repetier binary V1/V2 alike:
  request exactly the next *not admitted* line using the existing
  \r\nResend:<unsigned-decimal-line>\r\nok\r\n (CRLF shown as escapes).
  This final plain ok is the resend-control handshake, NOT confirmation that
  the bad command was accepted. Preserve normal ok <line> only for trusted
  committed command admission; duplicate already accepted lines never execute
  twice. Preserve legacy session startup/reset semantics outside recovery.
- Sender history may retransmit the requested command plus all following
  outstanding commands, not just one packet. ACK means admission, not movement
  completion. Maintain last committed line across retries; no rewind of
  already accepted work. Respect 16-bit binary line sequence wrap and existing
  32-bit response convention; test wrap, initial line/reset and duplicates.
- requestResend currently sets waitingForResend to 30/14, binary resync consumes
  31 consecutive zero bytes, and the historical Host supplies 32. The 200 ms
  idle retry is not a bounded end-to-end attempt counter. Normal serialFlush
  drains TX, not RX, and may block. Do not confuse these mechanisms or reuse
  waitingForResend as a fatal-retry budget.
- Current parser may mutate currentLineNumber, borrowedText and formatErrors
  before validation. Its format-error path stops rejecting after three format
  errors. A malformed recovery packet must NEVER become trusted merely because
  it has failed repeatedly. Review normal format-error bypass as well; do not
  add a new compatibility escape for malformed recovery input. M110/M112
  side effects are currently before normal line-number checking; the trust
  fence must precede all special-command effects.
- Checksum collisions are possible even in numbered streams (XOR and Fletcher
  modulo 255); independently flag FE/DOR/UPE and SW RX overflow. No claim that
  every unflagged corruption is detectable.

## Desired bounded recovery

Use a single foreground owner; ISR reads status before UDR, latches loss and
quarantines subsequent bytes without formatting, stopping, busy waiting or
EEPROM writes. Keep the existing short admission fence, with an explicit
race-safe ring discard/reopen handshake. Preserve previous admitted command
and motion queues, including borrowed text lifetime and committed ACK intent.
Do not reset positions, pause state, temperature targets or motor enable on
a correctable error. Let already validated motion complete under existing
planning/thermal rules. New untrusted commands cannot be admitted.

Recovery modes:

1. Healthy: ordinary checksummed ASCII and binary V1/V2, plus existing
   supported unnumbered traffic.
2. Quarantine: discard uncommitted frame and unread tainted bytes at a safe
   ownership boundary; do not discard admitted work. Record cause and the
   next required sequence. Determine whether the lost work is replayable.
3. Resend: publish one complete ordered short TX response via the existing
   nonblocking span API. Count actual transmitted/request-published attempts,
   not every corrupt byte, ISR flag, duplicate packet or service visit. A
   temporarily full TX ring cannot consume all attempts instantly. Prevent
   repeated requests from stale in-flight commands. Keep thermal/periodic
   services progressing; no nested generic service loop in the UART driver.
4. Resync: numbered ASCII needs a clean line boundary and valid checksum,
   expected line, no loss since the candidate boundary; discard stale tail
   and out-of-order frames. Binary V1/V2 uses the existing zero sync sequence
   with the same subsequent checks for expected line, supported frame shape,
   length, Fletcher checksum and absence of new loss. A corrupted first byte
   cannot choose recovery mode; use last committed/verified session mode.
   31 zeros alone do not prove an external boundary (valid binary payload
   can contain zeros); validate actual host handshake, in-flight bounds and
   repeated faults. Do not invent a sync protocol that existing hosts do not
   implement. Never accept an unnumbered frame or M110 reset as recovery
   evidence; do not execute falsely parsed emergency or machine commands.
   Valid local emergency/thermal handling remains independent.
5. On expected verified frame commit, return Healthy; reset consecutive
   request budget immediately. Optional saturating diagnostic counters for
   corrected episodes/causes/fatal episodes, independent of policy. Do not
   write EEPROM per event, do not stop after a cumulative count of corrected
   errors. No extra permanent buffer; reuse existing frame/session where safe.
6. If the retry budget expires without trusted progress, or the stream cannot
   be replayed safely, enter Fatal once. Unnumbered commands without a known
   retransmission history have no universal Resend identity. Don't silently
   treat mixed unnumbered traffic as successfully replayed by a later numbered
   frame. Retain normal unnumbered compatibility; fail closed for an
   unrecoverable ambiguous loss, rather than inventing a RETRY protocol.

Initial **proposed** engineering constants: three Resend requests for the
same missing line, then fatal if the third fails; 2 s receiver-ready response
window per issued request, not the existing 200 ms interbyte timeout or an
arbitrary timeout for an executing move. Confirm/adjust by host emulation and
measured software-only replays. Start response window after the resend has
been issued and the receiver can accept the replacement; don't charge timeout
for known legitimate firmware backpressure (long wait, full admitted queue,
borrowed text). Distinguish indefinite TX lack of progress, which needs a
bounded transport-failure path independent of successful transmission. Use
wrap-safe elapsed arithmetic; unrelated bytes cannot extend the attempt
forever. A new loss invalidates old sync evidence, not the episode budget.
No adaptive baud or queue size change. Correctable error diagnostics are
informational, not fatal.

## Fatal response and machine state

First latch the local terminal transport state and perform the common
mechanical/heater-safe stop independent of UART TX or host cooperation.
No queued/direct/scan/Advance motion, no automatic output or homing, retain
required motor holding torque, revoke heater permission and keep cooling,
watchdog and actual thermal fault reporting active. Do not fabricate a thermal
sensor fault or overwrite its first-fault evidence. Reject M110, M112, M999,
host cancel/end scripts, UI actions or reconnect as ways to reopen terminal
state. Use a distinct transport cause.

Preferred response sequence (one bounded nonblocking line at a time):
- RequestStop:
- fatal: RX recovery failed
- !! RX recovery failed
- Error: RX recovery failed (legacy error-display fallback for Printrun)

All lines terminated CRLF. Keep fatal wording clear of OctoPrint's known
recoverable-error substrings. Existing // action:cancel is an optional
compatibility fallback; don't send multiple cancel triggers unnecessarily.
Never rely on receipt of these lines, or on sending a G-code M112 from the
printer, for physical stop. No misleading ok for refused new machine work.
Preserve real thermal diagnostics priority if they arise simultaneously.
Host reactions can vary by version/settings; in particular older Printrun
may only log an error and wait. Local fail-safe remains mandatory.

## Required acceptance before shipping

Reuse Task0033 recovery-contract.js (abstract admission fence and external
boundary witness) and actual-source loss-gate-replay.py/tx-ack-replay.py
evidence. Extend rather than replace them with a second unconnected model.
Run actual receiver/ISR injection and software host replay against final
P0 for ASCII and binary V1/V2, ping-pong and streaming with in-flight tails:
- FE, DOR, UPE and ring-full at every relevant frame position, including
  first-byte protocol mode ambiguity, valid-looking XOR/Fletcher collision
  with a flagged byte, and wrapped ring cursors.
- One/two failed retransmissions then successful expected packet; many
  separated recovered errors never abort. Third failed request and no
  response cause one fatal transition with no additional work. TX full,
  stuck TX, delayed ACK, lost ACK, replay duplicates, out-of-order future
  commands, stale sync and losses during resync. True line wrap/reset and
  mixed unnumbered/manual commands. Prove no double admission or bogus ACK.
- Real history replay behavior of historical Host (32-zero binary handshake,
  credit and resend-control ok), current Repetier Host/Server binary/ASCII
  via virtual serial or equivalent software-only transport, OctoPrint,
  Printrun, Cura serial. Test both recovery and fatal response processing,
  including host ignored fatal/cancel and cleanup scripts. If current
  proprietary binaries cannot be run, explicitly report not qualified,
  never substitute a model as proof of their actual behavior.
- Keep periodic thermal/cooling/watchdog and local safety independent;
  preserve admitted text, no mutation before the final admission fence.
  Validate concurrent loss/ACK publication and terminal stop with final
  P0 API, all four canonical builds/recovery verifications, SRAM/stack and
  actual linked IRQ/service timing. No hardware experiments or flash.
