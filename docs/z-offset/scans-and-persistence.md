# Full bed scan, one-point Z-offset scan and matrix persistence

## Two different measurement jobs

1. **Full heat-bed scan (HBS, M3010)** measures the **2D surface shape**. Its saved matrix must reflect a reproducible thermal/mechanical state. Scanning a bed while it is still warping from heat-up can create a matrix that represents a transient shape, not the stable print surface.
2. **One-point Z-offset scan (ZOS, M3900; M3901 sets parameters)** measures contact at one X/Y location and shifts/adjusts an **existing** matrix. It is not a substitute for scanning bed shape; its default result is a vertical translation of a previously measured shape. Optional learning/weighted modes can locally alter matrix values, so 'one point = uniform shift' is true only in the ordinary unweighted mode.

**Mechanical reality:** ideal contact is a clean, stiff nozzle tip touching a clean, stiff bed. Residual filament, an elastic bed covering, a hot nozzle oozing material, bowing springs, cable drag and filament tension can shift the detected DMS trigger. Hot nozzle calibration can be useful for thermal accuracy, but only with a clean tip and stable mechanics. Sensor 'digits' are force-related; they do not directly establish an absolute gap.

## Z-min screw adjustment and scan reach

The Z-min switch establishes physical motor Z zero during homing. Adjust the trigger position to keep clearance at the closest point of the bed under the expected warm conditions and allow the nozzle to touch the deepest point anywhere in the scanned XY area. When all contact points lie past the trigger, the deepest point needs travel equal to the gap at the closest point plus the bed's peak-to-valley variation. Both HBS and ZOS need room to detect DMS contact before the approach limit. HBS measures the grid, and ordinary ZOS updates its reference from one location. [Homing reference](../../Repetier/Printer.cpp#L1641-L1655) · [Scan approach limits](../../Repetier/RF.cpp#L4684-L4758)

The screw adjustment minimizes unnecessary approach distance. An early trigger puts the bed farther from the nozzle at home. Extra travel on approaches from that reference costs additional moves, settling delays and DMS reads. Both scans use a coarse approach followed by a fine approach with smaller steps. The scan uses `Z_ENDSTOP_DRIVE_OVER` to set the maximum probing depth. Its runtime cost comes from the distance actually traversed. [HBS probe sequence](../../Repetier/RF.cpp#L958-L1005) · [ZOS probe sequence](../../Repetier/RF.cpp#L2379-L2454) · [Scan limit setup](../../Repetier/RF.cpp#L4832-L4861)

After ZOS updates the matrix, `calculateZScrewCorrection()` uses its largest signed contact value, `g_offsetZCompensationSteps`, which represents the closest point to the nozzle at Z zero. It estimates further thermal expansion and aims for about 50 µm of clearance there under the projected warm conditions. It converts the suggested shift to screw turns assuming 0.5 mm per turn. Full-bed reach still depends on the most negative contact value and the switch assembly's configured overtravel. Both extrema matter when evaluating the adjustment. [Matrix extrema](../../Repetier/RF.cpp#L5157-L5175) · [Screw-turn advice](../../Repetier/RF.cpp#L2650-L2729) · [Thermal assumptions](thermal-and-force.md#historical-values-found-in-source-advisory-only)

## The HBS heating sequence

In heated scan modes, the firmware sets scan bed targets around **60 °C PLA** or **100 °C ABS** and initially brings the nozzle to a lower **100 °C** scanning temperature; a later step heats the nozzle to **210 °C PLA** or **240 °C ABS** and corrects the matrix for the hot nozzle's changed length (`adjustCompensationMatrix`). It waits for temperature tolerance, not an independently validated **10+ minute heat-soak** of the entire mechanism. These setpoints and temperature behavior are code defaults, not a general prescription for other hotends/filaments.

For repeatability, record bed and nozzle temperature, heating duration, material in the nozzle, sensor idle force and the matrix slot. A cold-state scan can be repeatable, as can a properly heat-soaked scan, but don't blend drifting mechanical states unknowingly. Heating conditions depend on machine build and the intended printing process.

[Source: scan heating stages](../../Repetier/RF.cpp#L590-L680) · [hot nozzle stage](../../Repetier/RF.cpp#L1498-L1557) · [temperature checks](../../Repetier/RF.cpp#L3030-L3120) · [hot-nozzle matrix shift](../../Repetier/RF.cpp#L1715-L1735).

## ZOS (M3900) details

The firmware:
1. Disables bed Z compensation for calibration and loads the active matrix from external EEPROM if the current matrix is not valid **or the learning rate is 1.0** (default).
2. Picks an existing matrix scan point (configurable) with an optional **random X/Y displacement up to ~3 mm**. Thus repeated scans need not contact the **exact** same micro-location.
3. Samples DMS idle force, checks its stability, sets contact/retry/idle thresholds and performs a fast approach/back-off followed by slow fine approach.
4. After backing off, checks whether force recovered; if not, retries/aborts, explicitly treating filament residue or drifting DMS as plausible causes.
5. Estimates an offset `dZ=measuredContactZ - interpolatedMatrixZAtXY`. In ordinary mode, applies `learningRate*dZ` uniformly to matrix elements. Distance-weighted learning and repeated auto-level scans are optional distinct modes.
6. Recalculates matrix extrema, marks `g_ZMatrixChangedInRam=1`, finishes, and prints Z-screw adjustment advice. **The normal ZOS completion does not save the matrix to external EEPROM**.

Current defaults include a **33-digit** point-scan contact threshold and a **single fine-scan iteration**; retries are possible. These are especially relevant to false triggering and mechanical compliance. `readAveragePressure()` rejects unstable batches (min-to-max range too high) but **cannot guarantee absence of slow monotonic drift** over minutes: a smoothly moving force baseline might still pass each local stability window.

[Source: ZOS state machine](../../Repetier/RF.cpp#L2092-L2610) · [contact/recovery](../../Repetier/RF.cpp#L2320-L2465) · [average pressure](../../Repetier/RF.cpp#L4544-L4601) · [matrix update](../../Repetier/RF.cpp#L2482-L2561).

## Two separate nonvolatile stores: DO NOT CONFUSE THEM

| Item | Nonvolatile storage | How it is written |
| --- | --- | --- |
| Manual Z offset, `Printer::zOffset.manualMicrometers()` | **Internal MCU EEPROM** `EPR_RF_Z_OFFSET` | Changed by M3006 / UI, immediately written with checksum; `M3902 Z0` also writes zero here |
| Bed matrix (multiple slots and chosen active slot) | **External I²C EEPROM** | HBS saves on completion; `M3902 S<n>` or UI 'Save Matrix' for RAM modifications; matrix selection in M3009 / menu |

### Critical transfer operation: M3902 Z0

`M3902 Z0` shifts **the RAM matrix by the existing manual offset in steps**, changes the menu offset to zero and **immediately persists zero in internal EEPROM**. However the matrix is *only in RAM* until a separate save is issued.

For a consistent restart, save the corrected matrix to the intended **active boot slot**, verify the write/slot, and *then* regard the transfer as durable. There is no atomic transaction across the two chips. If the printer resets between steps the effective physical Z baseline changes. The dynamic SenseOffset portion is **not** included in this transfer.

**M3902 Z<nonzero>** shifts the matrix without changing the stored manual menu offset. `M3902 S<n>` writes the working RAM matrix into slot `n`. `M3009 S<n>` selects the active slot but refuses changes while Z compensation is active. A different selected slot can make the whole correction appear shifted.

### Safe operator practices (not firmware guarantees)

- Prefer clean and stable thermal conditions, a clean nozzle, tautness-free cables and filament path, and a known matrix slot for comparison.
- Inspect the matrix's effective offset / warpage and ZOS dZ before committing; large unexpected dZ is a reason to investigate contamination or sensor drift, not to blindly accept.
- Save after modifying RAM matrix and verify the active slot. Reboot/reload verification can reveal RAM-vs-EEPROM divergence, but never do so while the nozzle is very near the bed.
- Avoid transferring the manual offset into the matrix while printing or when uncertain about which store will be committed.

[Source: M3902 matrix manipulation](../../Repetier/RF.cpp#L8820-L8966) · [matrix save/load](../../Repetier/RF.cpp#L5184-L5504) · [matrix selection](../../Repetier/RF.cpp#L6640-L6680) · [save UI](../../Repetier/ui.cpp#L4785-L4800).

## Reliability issues to investigate later

The pressure threshold can be crossed early by sticky extrusion residue, a flexible bed interface, DMS preload from cable/filament, or a baseline changing while thermal expansion settles. A fine scan repeated at several fixed XY sites and stable temperature can help distinguish a global offset from local surface contamination, tilt and sensor drift. Do not silently change contact thresholds to 'fix' this without experiments.
