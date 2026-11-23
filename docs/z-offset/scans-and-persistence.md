# Full bed scan, one-point Z-offset scan and matrix persistence

## One point probe for both scan jobs

`ScanProbe` returns a measured machine Z position. The shared sequence establishes a stable idle reference, approaches in coarse increments until it sees contact, backs off into the idle-force band, then measures and releases contact with the configured fine increments. The result is the position after fine release. Before each approach or release decision, the probe waits for one conversion that started after the settling delay. The scan keeps the travel limit, retry budget and cancel state active.

| Caller | Use of the measured point |
| --- | --- |
| Heat-bed scan, `M3010` | Fill a mesh cell. The hot-nozzle calibration uses the same probe to measure the change in nozzle length. |
| Z-offset scan, `M3900` | Adjust an existing matrix using the measured Z and the interpolated Z at the chosen XY. Automatic leveling repeats the same probe at seven points. |

The current contact deltas are 10 digits by default for the mesh and 33 for Z-offset, relative to the measured idle value. The probe uses the same configured fine increment for both. A lost DMS read or a sample at or beyond ±10,000 corrected digits aborts the scan, permits retreat and prevents automatic homing in the abort path. The 10,000-digit backstop uses the conservative end of the maintainer's 10,000–11,000 digit estimate for protecting the strain strips. Bed contact uses the much smaller deltas above. Establish the bed's permissible contact force for each mounted surface.

Repeatability depends on nozzle cleanliness, surface compliance, filament, cable load and thermal state. Compare repeated measurements at the same XY across both callers on the intended bed and hotend. [Probe implementation](../../Repetier/RF.cpp) · [Mesh replay](../../tests/heat-bed-scan.py) · [Point scan replay](../../tests/z-offset-scan.py)

## Shared DMS acquisition

`Printer::strainGauge` starts individual 16-bit ADS1100 conversions on its foreground schedule. It waits for the busy indication to clear and fetches the result in a separate I²C frame. This covers a conversion completing between the result bytes and the status byte. The maximum start rate defaults to 8 Hz. Firmware code can lower it to 1–7 Hz through `setSampleRateHz()`. TI specifies 6.5 to 11.5 conversions/s at this resolution, with 8 typical. Conversion time and foreground scheduling determine the delivered rate. [ADC settings and timing](https://www.ti.com/lit/ds/symlink/ads1100.pdf) · [Acquisition code](../../Repetier/StrainGauge.cpp)

The AVR TWI peripheral clocks the I²C bytes at 100 kHz. The foreground handles each byte completion as it services the printer and resumes the transfer. It returns while events are pending and leaves the TWI interrupt disabled. The external matrix EEPROM shares the bus and still waits for its own reads and writes. [Bus driver](../../Repetier/TwiTransfer.cpp) · [Foreground schedule](../../Repetier/Commands.cpp) · [Bus and ADC replay](../../tests/twi-transfer.py)

Each new conversion adds its raw digits, sequence number, earliest possible conversion start and read time to an eight-entry ring. A new entry overwrites the oldest when full, and `history(0, sample)` copies the newest without removing it. The bytes of a new entry are written outside the brief publication lock. Foreground modules or an interrupt can copy a published snapshot. The current motion timers use the existing prepared compensation values.

The consumer applies the current Digit Homing offset to the raw sample. Printing controls reuse the latest valid sample between conversions to preserve their service cadence. Reading the cache leaves the print controls untouched. The periodic pressure checks, accepted pressure measurements and filament programs explicitly hand their readings to Digit Compensation. The panel and serial `F:` field display the last known value. On bus or converter failure the live result becomes unavailable and the service retries. The last valid value stays in the history. A live result also expires when its age from conversion start exceeds the configured interval plus 200 ms.

Idle references, stability checks and Digit Homing average three fresh conversions by default. `M3053` sets the count, `M3054` sets minimum start spacing and `M3055` sets the allowable spread. The old default of fifteen reads 15 ms apart could reread the same conversion several times. One fresh conversion per settled probe increment keeps the measurement count separate from the stability windows. The new per-step decision and the effect of force noise on contact and release need repeatability measurements on the printer. [Pressure windows](../../Repetier/PressureMeasurement.h)

## Guarded moves on the common motion path

Use a bounded machine-space Z move with a cancellation condition and a result containing the executed position and stop reason. The planner prepares its move and speed cap. The step executor checks hard endstops and a published stop condition before emitting another forward step. The DMS service owns I²C transactions, sample validity and measurement timing in the foreground. A stale sample, sensor error or travel limit ends the approach. Probe moves preserve the job's logical coordinates and leave the retry and retreat decisions to the scan.

The DMS service now supplies the shared measurement. A scan wants a low contact threshold, a short verified reaction and a controlled retreat. The start line wants a continuously adjusted speed and extrusion factor. A common motion interface can serve both policies. The scan's hard stop uses the raw sample with the Digit Homing correction so a smoothing filter does not consume its time budget.

For a continuous probe, bound the maximum travel after the trigger by

```text
v_z × T_detection,max + d_stop(v_z) + d_quantization + d_uncertainty ≤ d_budget
```

`T_detection,max` includes ADC integration and phase, filtering, sample age, the longest foreground service gap, I²C and stop publication. `d_stop` includes any planner or step-executor tail after publication. Establish `d_budget` on the actual mount and bed using the smaller of their respective margins after the trigger. The historical calibration gives about 0.05 mm of deflection at 5,500 digits. The firmware's Digit-Z estimate is about 0.076 mm at 10,000 digits and 0.084 mm at 11,000 digits before quantization. Use these numbers to size a displacement measurement. The bed force limit and worst measured stopping distance set the usable speed. The [deflection history](thermal-and-force.md#elastic-hotend-deflection-and-digit-z) records the calibration.

At the ADS1100 datasheet minimum of 6.5 conversions/s, one 16-bit conversion takes about 154 ms before bus and software delays. Measure acquisition latency and noise, nozzle deflection, point repeatability and peak bed load at the existing step sizes. Then compare a bounded approach on the shared step executor with that baseline, including abort, stale data and endstop traces. [ADS1100 datasheet](https://www.ti.com/lit/ds/symlink/ads1100.pdf) · [Watchdog and heater service budgets](../heater-safety.md)

## Z-min screw adjustment and scan reach

The Z-min switch establishes physical motor Z zero during homing. Adjust the trigger position to keep clearance at the closest point of the bed under the expected warm conditions and allow the nozzle to touch the deepest point anywhere in the scanned XY area. When all contact points lie past the trigger, the deepest point needs travel equal to the gap at the closest point plus the bed's peak-to-valley variation. Both HBS and ZOS need room to detect DMS contact before the approach limit. HBS measures the grid, and ordinary ZOS updates its reference from one location. [Homing reference](../../Repetier/Printer.cpp#L1641-L1655) · [Scan approach limits](../../Repetier/RF.cpp#L4684-L4758)

The screw adjustment minimizes unnecessary approach distance. An early trigger puts the bed farther from the nozzle at home. Extra travel on approaches from that reference costs additional moves, settling delays and DMS reads. Both scans use the same coarse and fine approach. The scan uses `Z_ENDSTOP_DRIVE_OVER` to derive the mechanical approach limit. The time spent probing depends on the distance and the sampling delays at each increment. [Scan probe and limits](../../Repetier/RF.cpp)

After ZOS updates the matrix, `calculateZScrewCorrection()` uses its largest signed contact value, `g_offsetZCompensationSteps`, which represents the closest point to the nozzle at Z zero. It estimates further thermal expansion and aims for about 50 µm of clearance there under the projected warm conditions. It converts the suggested shift to screw turns assuming 0.5 mm per turn. Full-bed reach still depends on the most negative contact value and the switch assembly's configured overtravel. Both extrema matter when evaluating the adjustment. [Matrix extrema](../../Repetier/RF.cpp#L5157-L5175) · [Screw-turn advice](../../Repetier/RF.cpp#L2650-L2729) · [Thermal assumptions](thermal-and-force.md#historical-values-found-in-source-advisory-only)

## The HBS heating sequence

In heated scan modes, the firmware sets scan bed targets around **60 °C PLA** or **100 °C ABS** and initially brings the nozzle to a lower **100 °C** scanning temperature; a later step heats the nozzle to **210 °C PLA** or **240 °C ABS** and corrects the matrix for the hot nozzle's changed length (`adjustCompensationMatrix`). It waits for temperature tolerance, not an independently validated **10+ minute heat-soak** of the entire mechanism. These setpoints and temperature behavior are code defaults, not a general prescription for other hotends/filaments.

For repeatability, record bed and nozzle temperature, heating duration, material in the nozzle, sensor idle force and the matrix slot. A cold-state scan can be repeatable, as can a properly heat-soaked scan, but don't blend drifting mechanical states unknowingly. Heating conditions depend on machine build and the intended printing process.

[Source: scan heating stages](../../Repetier/RF.cpp#L590-L680) · [hot nozzle stage](../../Repetier/RF.cpp#L1498-L1557) · [temperature checks](../../Repetier/RF.cpp#L3030-L3120) · [hot-nozzle matrix shift](../../Repetier/RF.cpp#L1715-L1735).

## ZOS (M3900) details

The firmware:
1. Disables bed Z compensation for calibration and loads the active matrix from external EEPROM if the current matrix is not valid **or the learning rate is 1.0** (default).
2. Picks an existing matrix scan point (configurable) with a **random X/Y displacement up to ~3 mm**. Thus repeated scans need not contact the **exact** same micro-location.
3. Acquires a stable DMS idle reference and uses the shared point probe. Force must return to the idle band during release.
4. Estimates an offset `dZ=measuredZ - interpolatedMatrixZAtXY`. In ordinary mode, applies `learningRate*dZ` uniformly to matrix elements. Distance-weighted learning and repeated auto-level scans are optional distinct modes.
5. Recalculates matrix extrema, marks `g_ZMatrixChangedInRam=1`, finishes, and prints Z-screw adjustment advice. **The normal ZOS completion does not save the matrix to external EEPROM**.

The measurement requires a bounded spread within each sample window. Slow baseline drift remains possible even when each window is stable.

[Z-offset scan implementation](../../Repetier/RF.cpp) · [Offset scan replay](../../tests/z-offset-scan.py)

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
