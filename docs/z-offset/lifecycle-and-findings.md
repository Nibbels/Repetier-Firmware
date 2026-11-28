# Z state lifecycle and source-backed risk register

**Status:** code-traced observations, not reproduced physical-print failures. These findings are candidate explanations for reports that the effective total Z-offset sometimes changes unexpectedly. Do not casually fix any of them in the motor-control path; see safe validation steps below.

## Lifecycle: where values begin and when they change

| Event | Saved manual Z offset | Live SenseOffset | Learned `g_min` / `g_max` | Bed matrix |
| --- | --- | --- | --- | --- |
| **Power-up / firmware reset** | Restored from MCU EEPROM and converted to steps | RAM initializes **0** | Restored to header defaults (0.33/10 mm with live steps/mm) by EEPROM load | Selected sector loaded from external I²C EEPROM; extrema recalculated |
| **EEPROM reload (`M501` / `M206`)** | Reloaded from EEPROM and combined with live SenseOffset using the loaded Z steps/mm | Preserved | Restored to the scaled defaults | Preserved |
| **Ordinary Z homing (`homeZAxis`)** | Preserved | Cleared with sample history and the active threshold after bed compensation is disabled | Defaults restored and all three layer markers and ECMP cleared | Preserved |
| **Disable Z stepper (`disableZStepper`)** | Preserved | No dedicated reset | Confirmed layers cleared. The pending candidate and fade bounds persist across motor disable | Preserved |
| **M3001 enable compensation** | Preserved | Auto-enable from EEPROM threshold possible | Preserved | Valid matrix required |
| **M3912 adaptive priming** | Preserved | Applied correction retained and new SenseOffset increments suspended | Phase flag suspends learning and all three markers are reset. Learned limits stay in effect during priming | Preserved |
| **First real layer recognized** | Preserved | Potentially increasing, if active and eligible | Newly learned from model first-layer height | Preserved |
| **Normal printing finishes / stopPrint()** | Preserved | **No unconditional cleanup** | **No unconditional cleanup** | Preserved |
| **M3000 (compensation disabled)** | Preserved | **FIXED:** remaining SenseOffset cleared regardless of active digits, with pressure history reset | **No boundary reset** | Matrix remains |
| **M3909 P0** | Preserved | Applied positive offset **intentionally remains** while CMP is enabled; pressure-history cleared; CMP disable then removes offset | Preserved | Preserved |
| **M3006 change** | Updates and immediately saves | Variable unchanged | Preserved | Preserved |
| **M3902 Z0 then matrix save** | Sets and saves menu offset to 0 | Variable unchanged | Preserved | Transfers former manual offset to RAM matrix; external save is separate |

`handleStartPrint()` marks printing status when queue depth exceeds two. `Printer::stopPrint()` and the stop sequence clear job motion and turn off the heaters. Z homing initializes layer and live SenseOffset state for the next print.

### Finding F1 (FIXED ON REVIEW BRANCH): M3006 must preserve live SenseOffset

**Fixed in [b4ae1dc](https://github.com/Nibbels/Repetier-Firmware/commit/b4ae1dc53a184c7a866db987f4419413912a58d5)** on `ai-firmware-review`. Before this patch, both `M3006 Z<mm>` and `M3006 S<um>` set `g_staticZSteps` from only the requested manual offset even when `g_nSensiblePressureOffset` was nonzero. The dynamic variable remained unchanged, causing the effective Z correction to temporarily omit the accumulated SenseOffset until another update recomputed the full sum.

The original patch added the live value to the manual offset when handling `M3006`. The current firmware performs that calculation in `Printer::zOffset` for both `M3006` and EEPROM reloads. EEPROM stores the manual portion. [Offset ledger and regression](offset-ledger.md)

**Regression scenarios**

- With 40 µm manual and 100 µm live, `M3006 S50` produces 150 µm of combined correction and saves 50 µm.
- Same initial state, `M3006 Z0.05` → same result, subject to normal micrometre and motor-step truncation.
- With zero live offset, `M3006` applies the requested manual correction.
- The Z-up/down **live transfer** of dynamic offset into EEPROM manual offset remains unchanged.

Sequential prints without Z homing and physical verification remain open.

[M3006](../../Repetier/GCodeMotion.cpp) · [Mod Menu buttons](../../Repetier/UiActions.cpp) · [SenseOffset controller](../../Repetier/PressureFeatures.cpp).

### Finding F2 (FIXED): previously disabled SenseOffset can now be cleaned up

Before [733833a](https://github.com/Nibbels/Repetier-Firmware/commit/733833a7c062b461c9fbb9de8a72ea202a65c812), `M3909 P0` disabled the threshold without clearing the live dynamic offset. Later `M3000` could not clear that offset because the cleanup required a **nonzero** threshold. The periodic handler now clears the live SenseOffset whenever bed CMP is disabled, regardless of whether `M3909 P0` was already issued.

**Intentional safety.** `M3909 P0` stops new corrections and retains the already-applied clearance while CMP is active. Once CMP is off, the handler clears the live portion and calculates the effective steps from the saved manual offset. It clears the pressure average, sample count and previous pressure whenever regulation is inactive.

Tests still required on a physical printer: `M3909 P0` during a first layer, subsequent `M3000`, re-enable with `M3001`/autostart, and cancellation.

[Source](../../Repetier/PressureFeatures.cpp).

### Finding F3 (PARTIALLY FIXED): consecutive-print lifecycle

The standard serial workflow uses `G28` as its start boundary and `M3079` as its end sequence. Output finishes its move and disables the motors and bed compensation. Z homing restores the fade defaults and clears the learned state before the next prime or model move. The host replay checks homing between different first-layer heights and model extrusion without automatic priming. A workflow that skips Z homing can still carry learned fade bounds into a subsequent print. [USB print start](../first-layer-control/README.md#usb-print-start)

### Finding F4 (HIGH): matrix-offset transfer is not an atomic persistent transaction

`M3902 Z0` transfers the current manual offset to the **RAM** bed matrix and immediately saves a manual offset of zero in **internal MCU EEPROM**. The matrix remains marked `g_ZMatrixChangedInRam=1` until `M3902 S<sector>` or the Save Matrix UI explicitly writes the **external I²C EEPROM**.

If the printer resets or power is lost before that external write, the persistent pair is inconsistent: old matrix + zero manual offset. If the matrix is deliberately saved to a sector that is **not** the active boot sector, restart can also pick up a different effective baseline. Even a normal ZOS scan can change the RAM matrix without committing it automatically.

No implicit transaction or rollback makes both stores atomic. **Treat an offset transfer as an unsafe two-step operation unless both parts are verified saved.**

[M3902 Z0 and matrix save](../../Repetier/BedMatrix.cpp) · [External EEPROM transport](../../Repetier/ExternalEeprom.cpp).

### Finding F5 (INTENTIONAL FEATURE, NOT A BUG): repeated Z-up/down transfers transient SenseOffset into saved manual Z-offset

This behavior is explicitly documented in **RF.01.37.mod (2017-01-25)**: the Mod Menu lets the operator “push SenseOffset to your Z-Offset with the Z-Up and Z-Down Button.”

Within Mod Menu page 2, bed DOWN adds +5 µm to the saved manual offset. Bed UP consumes up to 5 µm from live SenseOffset first. Therefore a completed pair transfers 5 µm into EEPROM and restores approximately the original total clearance. For example: static 0 / dynamic 200 / total 200 → DOWN static +5 / dynamic 200 / total 205 → UP static +5 / dynamic 195 / total 200. This is an intentional live calibration technique for sheet-thickness differences or small physical shifts, and must be retained.

The invariant is approximate around clamps, zero snapping and concurrently rising SenseOffset. Each persistent increment writes internal EEPROM. The real risk is **changing or resetting either component without preserving this operator-controlled state transfer**, not the asymmetric button mapping itself.

[Detailed workflow, pitfalls and test cases](history-and-workflows.md) · [Button implementation](../../Repetier/UiActions.cpp) · [Button mapping](../../Repetier/UiActions.cpp) · [Original 2017 changelog](../../changelog.txt#L1760-L1768).

### Finding F6 (MEDIUM): Z-offset scan uses volatile RAM matrix state

The point Z-offset scan (`M3900`) updates the current RAM matrix. For a full learning rate, it first reloads the unmodified matrix from external EEPROM; unsaved RAM edits can be discarded on the next scan. After a successful scan, the matrix is marked dirty and must be **explicitly saved**. `g_offsetZCompensationSteps` and `g_ZCompensationMax` are recomputed, affecting the current Z baseline even if menu Z-offset stays constant.

[Z-offset scan](../../Repetier/ZOffsetScan.cpp) · [Matrix edit and pending save](../../Repetier/BedMatrix.cpp).

### Finding F7 (FIXED FOR Z HOME): homing resets all layer markers

The pre-patch `homeZAxis()` did not clear layer markers or learned min/max. **It now does** after disabling CMP, whether or not the stepper was disabled first ([ba93708](https://github.com/Nibbels/Repetier-Firmware/commit/ba93708d41c84fcce16872720b6b954da70ea157)). `disableZStepper()` still clears only `Current` and `Last`; a subsequent Z home resolves this discrepancy. X-only and Y-only homing deliberately do not reset Z first-layer state.

[Source: homeZAxis](../../Repetier/Homing.cpp) · [disableZStepper](../../Repetier/Printer.h#L333-L370).

### Finding F8 (DESIGN CONSTRAINT): future lifecycle reset must distinguish durable takeover from disposable correction

Because the operator may transfer dynamic SenseOffset into EEPROM-backed manual Z-offset while printing, Z homing **preserves the manual offset** and discards only the untransferred RAM remainder after disabling CMP. No automatic EEPROM/matrix writes occur. Any future no-homing reset must honor the same physical safety and manual-transfer invariants.

[Documented live transfer](history-and-workflows.md).

## What has *not* been established

- No evidence yet identifies which of F1–F6 caused the user's historical intermittent Z changes.
- No direct runtime instrumentation of the combined Z target, sensor digits, matrix sector, EEPROM offset and active state.
- No hardware verification of the motor's physical clearance after specific transitions.

## Safe diagnostic plan before modifying behavior

Instrument **events**, not high-rate ISR cycles: after boot, after EEPROM/matrix load, after homing, before and after M3001/M3000, before/after M3912, upon M3909/M3006, on ordinary Z homing (with vs. without prior stepper disable), on print stop/finish, after ZOS and matrix save. Report:

Include the offset owner's manual and live values in µm and its effective step count. Record `g_offsetZCompensationSteps`, `g_ZCompensationMax`, `g_min` and `g_max` in steps. Include the active matrix sector and dirty flag, nominal Z, target and applied compensation, enabled CMP and SenseOffset, tool, DMS force, and bed and nozzle temperatures.

A test must not abruptly remove a stored positive clearance correction near the bed. First define the safe new-print boundary and whether controlled Z retraction/homing/compensation-off is needed. Develop isolated commits only after the state transitions are understood.
