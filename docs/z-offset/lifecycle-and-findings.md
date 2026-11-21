# Z state lifecycle and source-backed risk register

**Status:** code-traced observations, not reproduced physical-print failures. These findings are candidate explanations for reports that the effective total Z-offset sometimes changes unexpectedly. Do not casually fix any of them in the motor-control path; see safe validation steps below.

## Lifecycle: where values begin and when they change

| Event | Persistent manual `Printer::ZOffset` | Dynamic `g_nSensiblePressureOffset` | Learned `g_min` / `g_max` | Bed matrix |
| --- | --- | --- | --- | --- |
| **Power-up / firmware reset** | Restored from MCU EEPROM; `g_staticZSteps` reconstructed | RAM initializes **0** | Restored to header defaults (0.33/10 mm with live steps/mm) by EEPROM load | Selected sector loaded from external I²C EEPROM; extrema recalculated |
| **Ordinary Z homing (`homeZAxis`)** | Preserved | No dedicated reset | **Neither layer markers nor learned min/max boundaries are explicitly reset**; homing disables CMP and re-establishes coordinates | Preserved |
| **Disable Z stepper (`disableZStepper`)** | Preserved | No dedicated reset | Current/last markers reset, but **GuessNew and learned min/max are not** | Preserved |
| **M3001 enable compensation** | Preserved | Auto-enable from EEPROM threshold possible | Preserved | Valid matrix required |
| **M3912 adaptive priming** | Preserved | No dedicated reset | Prime deliberately excluded from learning; current/last markers reset afterward, not learned limits | Preserved |
| **First real layer recognized** | Preserved | Potentially increasing, if active and eligible | Newly learned from model first-layer height | Preserved |
| **Normal printing finishes / stopPrint()** | Preserved | **No unconditional cleanup** | **No unconditional cleanup** | Preserved |
| **M3000 (compensation disabled)** | Preserved | Controller can reset offset only if its stored live digit threshold is **nonzero** when the background check sees CMP off | No boundary reset | Matrix remains |
| **M3909 P0** | Preserved | **Current positive offset remains**; P0 clears active threshold only | Preserved | Preserved |
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

### Finding F2 (HIGH): M3909 P0 disables control but does not clear its accumulated correction

`M3909 P0` sets `g_nSensiblePressureDigits=0` and logs disabled. It does **not** zero `g_nSensiblePressureOffset` and does not recompute `g_staticZSteps`. The periodic cleanup in the SenseOffset controller is gated on `if (g_nSensiblePressureDigits && !doHeatBedZCompensation)`; after P0 the first condition is false, so even disabling heat-bed compensation does not trigger that cleanup. The positive correction can therefore remain in live derived Z state and its underlying RAM variable. **Some retention might have been intentional to avoid an unsafe downward Z jump**, but the state semantics are not explicit and are important between prints.

`M3000` on its own may clear the dynamic offset **only if** the threshold is still nonzero and the periodic SenseOffset code runs after bed compensation is off. Never replace this with an abrupt reset during printing.

[Source: M3909 P0](../../Repetier/RF.cpp#L8975-L9037) · [conditional clear](../../Repetier/RF.cpp#L5660-L5677).

### Finding F3 (HIGH): stopping/finishing a print is not a full Z-state reset

On boot, `g_nSensiblePressureOffset=0` and EEPROM load sets default `g_min`/`g_max`. No equivalent universal reset was found in `stopPrint()`, `handleStopPrint()`, `handleStartPrint()` or `M3912`. This gives two classes of consecutive-print state leakage:

1. Accumulated positive clearance correction may carry into the next print, depending on end-G-code/compensation deactivation.
2. Learned `g_minZCompensationSteps` may remain from the prior print. With prior first layer 0.40 mm, a new 0.352 mm M3912 line can satisfy SenseOffset's `Z<=g_min` gate but fail Digit Flow's opposite `Z>g_min` gate — the wrong feedback mechanism for priming.

The first is a **clear missing unconditional reset** (not necessarily a bug without defining desired retention); the second is a **concrete code-path hazard**, not yet a demonstrated field failure. Do not reset bounds in the middle of an active print or invalidate a genuine new layer in a sequential object.

[Source: print stop](../../Repetier/Printer.cpp#L1788-L1840) · [stop completion](../../Repetier/RF.cpp#L5970-L6060) · [M3912 end](../../Repetier/RF.cpp#L9230-L9254) · [boot bound reset](../../Repetier/Eeprom.cpp#L615-L624).

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

### Finding F7 (MEDIUM): ordinary homing is not equivalent to disabling Z steppers

`homeZAxis()` uses `setHomed(-1,-1,false)`, `resetDirectAxis(Z)`, a homing sequence and `setZAxisSteps(...)`, but does **not** explicitly clear `queuePositionZLayerCurrent`, `Last`, `GuessNew`, or learned `g_min`/`g_max`. By contrast, `disableZStepper()` resets `Current`/`Last`, omitting `GuessNew` and learned limits. Whether the stepper was disabled *before* homing can therefore change logical layer state while leaving the same apparent Z home position. This may affect a second print's first-layer detection.

[Source: homeZAxis](../../Repetier/Printer.cpp#L1556-L1653) · [setHomed](../../Repetier/Printer.h#L621-L650) · [disableZStepper](../../Repetier/Printer.h#L333-L370) · [setZAxisSteps](../../Repetier/Printer.h#L1028-L1035).

### Finding F8 (DESIGN CONSTRAINT): future lifecycle reset must distinguish durable takeover from disposable correction

Because an operator can intentionally transfer 5 µm increments from the dynamic SenseOffset to EEPROM-backed manual offset **while the machine is printing**, it would be incorrect for a new print lifecycle fix to assume all observed positive clearance is temporary. At the next verified safe print boundary, preserving `Printer::ZOffset` is mandatory; clearing any remaining `g_nSensiblePressureOffset` must be coordinated with live Z motion and bed distance. Do not couple transfer to scan-matrix saves or automatically shift EEPROM offset when force rises.

[Documented live transfer](history-and-workflows.md).

## What has *not* been established

- No evidence yet identifies which of F1–F6 caused the user's historical intermittent Z changes.
- No direct runtime instrumentation of the combined Z target, sensor digits, matrix sector, EEPROM offset and active state.
- No hardware verification of the motor's physical clearance after specific transitions.

## Safe diagnostic plan before modifying behavior

Instrument **events**, not high-rate ISR cycles: after boot, after EEPROM/matrix load, after homing, before and after M3001/M3000, before/after M3912, upon M3909/M3006, on ordinary Z homing (with vs. without prior stepper disable), on print stop/finish, after ZOS and matrix save. Report:

`Printer::ZOffset` [µm]; `g_nSensiblePressureOffset` [µm]; `g_staticZSteps` [steps]; `g_offsetZCompensationSteps` and `g_ZCompensationMax` [steps]; `g_min`/`g_max` [steps]; active matrix sector and dirty flag; current nominal Z; target and current compensation steps; enabled CMP/SenseOffset; selected tool; DMS readings; bed/nozzle temperature.

A test must not abruptly remove a stored positive clearance correction near the bed. First define the safe new-print boundary and whether controlled Z retraction/homing/compensation-off is needed. Develop isolated commits only after the state transitions are understood.
