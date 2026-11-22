# SenseOffset / SensiblePressure (M3909)

## Why it exists

Wessix proposed, and Nibbels coded, a safeguard for printing very close to the heated bed. If a nozzle-to-bed gap is too small, material must flow radially through a thin opening. This can create substantially more resistance than the nozzle outlet alone. For a 0.4 mm nozzle, the nominal circular outlet is `pi*r^2 ~= 0.126 mm^2` (`r=0.2 mm`). A crude annular side-area comparison `2*pi*r*dz` gives `~0.025 mm^2` at `dz=0.02 mm` and `~0.126 mm^2` at `dz=0.10 mm`. **This is geometrical intuition, not a hydrodynamic law**; real viscous/non-Newtonian resistance depends on gap geometry, flow and temperature. The strain gauges measure force, not direct fluid pressure.

SenseOffset responds to **excessive DMS digits** by increasing nozzle clearance incrementally and never reducing it again just because measured force fell. High readings may also be caused by a clog or temperature/material/feed problems; thus changes must be limited.

Digit-Z uses a reversible force-to-Z estimate for elastic hotend deflection. SenseOffset retains learned first-layer clearance, for example as the printer warms and the gap drifts. [Deflection estimate and timing](../z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## Input, gating and response

- `Configuration.h` sets `SENSIBLE_PRESSURE_INTERVAL=50 ms` and `SENSIBLE_PRESSURE_MAX_OFFSET=180 um` as defaults.
- `M3909 P<digits> S<max_um>` sets threshold and maximum additional offset; source bounds `S` to **1..300 um**. `M3909 P0` disables control. EEPROM may retain thresholds. `M3001` may queue an auto-enable of SenseOffset if configured.
- SenseOffset needs active bed Z compensation and an enabled positive force threshold.
- It can react only when **`Printer::currentSteps[Z_AXIS] <= g_minZCompensationSteps`**. That is a **nominal step-coordinate condition**, not a measurement of actual gap.
- The first decision takes **10 accepted samples**, about half a second at 50 ms spacing. The interval check can skip a poll that arrives exactly 50 ms after the last sample. Later decisions carry half the averaging window forward.
- It takes the **absolute** average reading. Above the configured limit it increases `g_nSensiblePressureOffset` by at least **1 um** when correction is warranted; larger pressure deviations can increase the increment, with tapering as offset approaches the maximum.
- The result changes `g_staticZSteps = ((Printer::ZOffset + g_nSensiblePressureOffset) * axisStepsPerMM[Z]) / 1000`.
- Below the pressure threshold, the additional offset **does not decrease**. It persists when leaving the low-Z region; loss/disable of bed compensation is a separate deactivation/reset path.

This is deliberately **not** a bidirectional PID loop. It is a bounded, positive-only first-layer clearance correction; the cap limits inappropriate reactions to a clogged nozzle or sensor error.

## Coupling to learned layer height

`g_minZCompensationSteps` controls both where full scanned-bed following ends and where SenseOffset may regulate. The fallback limit is 0.33 mm. Generated `M3912` priming suspends SenseOffset for the entire line, including with a higher limit inherited from an earlier print. For a manually scripted prime, enable SenseOffset after priming and account for `M3001` AutoStart. [Manual start-line ordering](automatic-start-line.md#manual-start-lines)

Extruder-specific Z offsets and different coordinate origins require care; current source has a TODO about which Z quantity should be used for SenseOffset eligibility.

## Intentional manual persistence of learned clearance

The operator can use alternating Z/bed DOWN and UP buttons in Mod Menu page 2 to transfer the positive SenseOffset adjustment into the saved manual `Printer::ZOffset` in 5 µm increments while approximately maintaining the combined correction. This is explicitly documented in the original 2017 changelog and must survive future fixes. See [live transfer procedure and conditions](../z-offset/history-and-workflows.md).

## Relevant source
- [Globals, names and defaults](../../Repetier/RF.cpp#L193-L199)
- [Strain-gauge reads and controller](../../Repetier/RF.cpp#L5622-L5775)
- [M3909 and EEPROM handling](../../Repetier/RF.cpp#L9012-L9076)
- [Enable via queue task](../../Repetier/Printer.cpp#L1098-L1109)
- [Config](../../Repetier/Configuration.h#L87-L100)
- [Original intention in changelog](../../changelog.txt#L1775-L1784)
