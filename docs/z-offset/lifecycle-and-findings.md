# Z state lifecycle and source-backed risk register

**Status:** code-traced observations, not reproduced physical-print failures. These findings are candidate explanations for reports that the effective total Z-offset sometimes changes unexpectedly. Do not casually fix any of them in the motor-control path; see safe validation steps below.

## Lifecycle: where values begin and when they change

| Event | Persistent manual `Printer::ZOffset` | Dynamic `g_nSensiblePressureOffset` | Learned `g_min` / `g_max` | Bed matrix |
| --- | --- | --- | --- | --- |
| **Power-up / firmware reset** | Restored from MCU EEPROM; `g_staticZSteps` reconstructed | RAM initializes **0** | Restored to header defaults (0.33/10 mm with live steps/mm) by EEPROM load | Selected sector loaded from external I²C EEPROM; extrema recalculated |
| **Ordinary Z homing (`homeZAxis`)** | **Preserved** | **Reset to 0 after CMP is disabled**; threshold deactivated | **FIXED:** all 3 markers and learned min/max reset to defaults in print mode | **Preserved** |
| **Disable Z stepper (`disableZStepper`)** | Preserved | No dedicated reset | Current/last markers reset, but **GuessNew and learned min/max are not** | Preserved |
| **M3001 enable compensation** | Preserved | Auto-enable from EEPROM threshold possible | Preserved | Valid matrix required |
| **M3912 adaptive priming** | Preserved | Applied correction retained and new SenseOffset increments suspended | Phase flag suspends learning and all three markers are reset. Learned limits stay in effect during priming | Preserved |
| **First real layer recognized** | Preserved | Potentially increasing, if active and eligible | Newly learned from model first-layer height | Preserved |
| **Normal printing finishes / stopPrint()** | Preserved | **No unconditional cleanup** | **No unconditional cleanup** | Preserved |
| **M3000 (compensation disabled)** | Preserved | **FIXED:** remaining SenseOffset cleared regardless of active digits, with pressure history reset | **No boundary reset** | Matrix remains |
| **M3909 P0** | Preserved | Applied positive offset **intentionally remains** while CMP is enabled; pressure-history cleared; CMP disable then removes offset | Preserved | Preserved |
| **M3006 change** | Updates and immediately saves | Variable unchanged | Preserved | Preserved |
| **M3902 Z0 then matrix save** | Sets and saves menu offset to 0 | Variable unchanged | Preserved | Transfers former manual offset to RAM matrix; external save is separate |

`handleStartPrint()` is not a reliable new-print initialization event: it marks 'printing' when queue depth exceeds two. `Printer::stopPrint()` and the later `handleStopPrint()` clean up the move queue/heaters/print markers but do not explicitly zero SenseOffset or reinitialize the learned Z boundaries.

### Finding F1 (FIXED ON REVIEW BRANCH): M3006 must preserve live SenseOffset

**Fixed in [b4ae1dc](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc53a184c7a866db987f4419413912a58d5)** on `ai-firmware-review`. Before this patch, both `M3006 Z<mm>` and `M3006 S<um>` set `g_staticZSteps` from only the requested manual offset even when `g_nSensiblePressureOffset` was nonzero. The dynamic variable remained unchanged, causing the effective Z correction to temporarily omit the accumulated SenseOffset until another update recomputed the full sum.

The patch sets `Printer::ZOffset` first, then calculates `g_staticZSteps` using the same combined `(Printer::ZOffset + g_nSensiblePressureOffset)` expression already used in the Mod Menu and pressure controller. The `#if FEATURE_SENSIBLE_PRESSURE` fallback preserves the original calculation when this feature is compiled out. EEPROM persistence of *only the manual offset* is unchanged; the dynamic portion remains in RAM.

**Regression scenarios (static source checks done; firmware/hardware tests pending):**
- `Printer::ZOffset=40 µm`, `g_nSensiblePressureOffset=100 µm`, `M3006 S50` → effective combined correction `150 µm`, EEPROM manual `50 µm`.
- Same initial state, `M3006 Z0.05` → same result, subject to normal micrometre and motor-step truncation.
- `g_nSensiblePressureOffset=0` → ordinary `M3006` semantics unchanged.
- With `FEATURE_SENSIBLE_PRESSURE=0` → code retains original M3006 calculation without accessing unavailable SenseOffset globals.
- The Z-up/down **live transfer** of dynamic offset into EEPROM manual offset remains unchanged.

**Still open:** all other writers of `g_staticZSteps` and the print lifecycle, particularly `setupForPrinting()`, `M3909 P0`, sequential prints, and stale `g_min` / `g_max` state. Do not mark the broader Z-offset symptom fixed without hardware evidence.

[Source: patched M3006](../../Repetier/RF.cpp#L6587-L6642) · [menu calculation](../../Repetier/ui.cpp#L2821-L2878) · [SenseOffset controller](../../Repetier/RF.cpp#L5748-L5760).

### Finding F2 (FIXED): previously disabled SenseOffset can now be cleaned up

Before [733833a](https://github.com/Nibbels/Repetier-Firmware/commit/733833a7c062b461c9fbb9de8a72ea202a65c812), `M3909 P0` disabled the threshold without clearing the live dynamic offset. Later `M3000` could not clear that offset because the cleanup required a **nonzero** threshold. The periodic handler now clears any remaining `g_nSensiblePressureOffset` whenever bed CMP is disabled, regardless of whether `M3909 P0` was already issued.

**Intentional safety:** `M3909 P0` alone still retains the already-applied clearance **while CMP is active**; it only stops new corrections. Once CMP is off, cleanup reconstructs `g_staticZSteps` from the saved manual offset. The pressure average, sample count and previous pressure are also cleared whenever regulation is inactive, so stale readings do not influence the next activation. No additional EEPROM writes.

Tests still required on a physical printer: `M3909 P0` during a first layer, subsequent `M3000`, re-enable with `M3001`/autostart, and cancellation.

[Source](../../Repetier/RF.cpp#L5623-L5710).

### Finding F3 (PARTIALLY FIXED): consecutive-print lifecycle

**Fixed on [ba93708](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708d41c84fcce16872720b6b954da70ea157) for sequences that home Z.** `homeZAxis()` disables CMP through `setHomed(-1,-1,false)` and waits for compensation to return to zero. Then it resets `queuePositionZLayerGuessNew`, `Current`, `Last`, `g_minZCompensationSteps`, `g_maxZCompensationSteps` and `g_diffZCompensationSteps`, using live Z steps/mm for the default bounds. It clears the transient SenseOffset and active pressure threshold, then reconstructs `g_staticZSteps` from the **preserved EEPROM-backed** `Printer::ZOffset`. The matrix remains untouched.

Prints begun without a fresh Z home may retain learned fade boundaries during priming. The `M3912` phase flag controls the transition to the DMS brake even with inherited limits. `M3000` clears the dynamic offset. `G28 Z` resets the learned bounds as well.

**Conclusion:** Z home is the implemented safe primary lifecycle boundary. Separate analysis is required for no-homing workflows; do not reset fade limits while a real layer is being printed.

[Patched Z home](../../Repetier/Printer.cpp#L1556-L1657) · [print-state heuristics](../../Repetier/RF.cpp#L5969-L6098) · [M3912](../../Repetier/RF.cpp#L9094-L9263).

### Finding F4 (HIGH): matrix-offset transfer is not an atomic persistent transaction

`M3902 Z0` transfers the current *manual menu* offset to the **RAM** bed matrix and immediately sets/saves `Printer::ZOffset=0` in **internal MCU EEPROM**. The matrix remains only marked `g_ZMatrixChangedInRam=1` until `M3902 S<sector>` or the Save Matrix UI explicitly writes the **external I²C EEPROM**.

If the printer resets or power is lost before that external write, the persistent pair is inconsistent: old matrix + zero manual offset. If the matrix is deliberately saved to a sector that is **not** the active boot sector, restart can also pick up a different effective baseline. Even a normal ZOS scan can change the RAM matrix without committing it automatically.

No implicit transaction or rollback makes both stores atomic. **Treat an offset transfer as an unsafe two-step operation unless both parts are verified saved.**

[Source: M3902 Z0](../../Repetier/RF.cpp#L8850-L8953) · [save operation](../../Repetier/RF.cpp#L8953-L8965) · [external write](../../Repetier/RF.cpp#L5184-L5284).

### Finding F5 (INTENTIONAL FEATURE, NOT A BUG): repeated Z-up/down transfers transient SenseOffset into saved manual Z-offset

This behavior is explicitly documented in **RF.01.37.mod (2017-01-25)**: the Mod Menu lets the operator “push SenseOffset to your Z-Offset with the Z-Up and Z-Down Button.”

Within Mod Menu page 2, bed DOWN adds +5 µm to **saved** `Printer::ZOffset`; bed UP preferentially consumes 5 µm from **dynamic** `g_nSensiblePressureOffset` rather than reducing saved offset. Therefore a completed pair transfers 5 µm into EEPROM and restores approximately the original total clearance. For example: static 0 / dynamic 200 / total 200 → DOWN static +5 / dynamic 200 / total 205 → UP static +5 / dynamic 195 / total 200. This is an intentional live calibration technique for sheet-thickness differences or small physical shifts, and must be retained.

The invariant is approximate around clamps, zero snapping and concurrently rising SenseOffset. Each persistent increment writes internal EEPROM. The real risk is **changing or resetting either component without preserving this operator-controlled state transfer**, not the asymmetric button mapping itself.

[Detailed workflow, pitfalls and test cases](history-and-workflows.md) · [Button implementation](../../Repetier/ui.cpp#L2821-L2879) · [Button mapping](../../Repetier/ui.cpp#L4100-L4126) · [Original 2017 changelog](../../changelog.txt#L1760-L1768).

### Finding F6 (MEDIUM): Z-offset scan uses volatile RAM matrix state

The point Z-offset scan (`M3900`) updates the current RAM matrix, not directly `Printer::ZOffset`. For a full learning rate, it first reloads the unmodified matrix from external EEPROM; unsaved RAM edits can be discarded on the next scan. After a successful scan, the matrix is marked dirty and must be **explicitly saved**. `g_offsetZCompensationSteps` and `g_ZCompensationMax` are recomputed, affecting the current Z baseline even if menu Z-offset stays constant.

[Source: ZOS load](../../Repetier/RF.cpp#L2255-L2277) · [edit and dirty flag](../../Repetier/RF.cpp#L2483-L2561) · [successful finish](../../Repetier/RF.cpp#L2585-L2606).

### Finding F7 (FIXED FOR Z HOME): homing resets all layer markers

The pre-patch `homeZAxis()` did not clear layer markers or learned min/max. **It now does** after disabling CMP, whether or not the stepper was disabled first ([ba93708](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708d41c84fcce16872720b6b954da70ea157)). `disableZStepper()` still clears only `Current` and `Last`; a subsequent Z home resolves this discrepancy. X-only and Y-only homing deliberately do not reset Z first-layer state.

[Source: homeZAxis](../../Repetier/Printer.cpp#L1556-L1657) · [disableZStepper](../../Repetier/Printer.h#L333-L370).

### Finding F8 (DESIGN CONSTRAINT): future lifecycle reset must distinguish durable takeover from disposable correction

Because the operator may transfer dynamic SenseOffset into EEPROM-backed manual Z-offset while printing, Z homing **preserves `Printer::ZOffset`** and discards only the untransferred RAM remainder after disabling CMP. No automatic EEPROM/matrix writes occur. Any future no-homing reset must honor the same physical safety and manual-transfer invariants.

[Documented live transfer](history-and-workflows.md).

## What has *not* been established

- No evidence yet identifies which of F1–F6 caused the user's historical intermittent Z changes.
- No direct runtime instrumentation of the combined Z target, sensor digits, matrix sector, EEPROM offset and active state.
- No hardware verification of the motor's physical clearance after specific transitions.

## Safe diagnostic plan before modifying behavior

Instrument **events**, not high-rate ISR cycles: after boot, after EEPROM/matrix load, after homing, before and after M3001/M3000, before/after M3912, upon M3909/M3006, on ordinary Z homing (with vs. without prior stepper disable), on print stop/finish, after ZOS and matrix save. Report:

`Printer::ZOffset` [µm]; `g_nSensiblePressureOffset` [µm]; `g_staticZSteps` [steps]; `g_offsetZCompensationSteps` and `g_ZCompensationMax` [steps]; `g_min`/`g_max` [steps]; active matrix sector and dirty flag; current nominal Z; target and current compensation steps; enabled CMP/SenseOffset; selected tool; DMS readings; bed/nozzle temperature.

A test must not abruptly remove a stored positive clearance correction near the bed. First define the safe new-print boundary and whether controlled Z retraction/homing/compensation-off is needed. Develop isolated commits only after the state transitions are understood.
