# SenseOffset / SensiblePressure (M3909)

## Why it exists

Wessix proposed, and Nibbels coded, a safeguard for printing very close to the heated bed. If a nozzle-to-bed gap is too small, material must flow radially through a thin opening. This can create substantially more resistance than the nozzle outlet alone. For a 0.4 mm nozzle, the nominal circular outlet is `pi*r^2 ~= 0.126 mm^2` (`r=0.2 mm`). A crude annular side-area comparison `2*pi*r*dz` gives `~0.025 mm^2` at `dz=0.02 mm` and `~0.126 mm^2` at `dz=0.10 mm`. **This is geometrical intuition, not a hydrodynamic law**; real viscous/non-Newtonian resistance depends on gap geometry, flow and temperature. The strain gauges measure force, not direct fluid pressure.

SenseOffset responds to **excessive DMS digits** by increasing nozzle clearance incrementally and never reducing it again just because measured force fell. High readings may also be caused by a clog or temperature/material/feed problems; thus changes must be limited.

Digit-Z uses a reversible force-to-Z estimate for elastic hotend deflection. SenseOffset retains learned first-layer clearance, for example as the printer warms and the gap drifts. [Deflection estimate and timing](../z-offset/thermal-and-force.md#elastic-hotend-deflection-and-digit-z)

## Input, gating and response

`Printer::senseOffsetControl` manages the active force threshold, maximum clearance, AutoStart and the sampling history. An accepted sample can propose an increment in micrometres. The foreground checks eligibility and applies the increment to `ZOffset`, which owns the applied clearance. [Controller](../../Repetier/SenseOffsetControl.h) · [Application](../../Repetier/RF.cpp#L2958)

- `Configuration.h` sets `SENSIBLE_PRESSURE_INTERVAL=50 ms` and `SENSIBLE_PRESSURE_MAX_OFFSET=180 um` as defaults.
- `M3909 P<digits> S<max_um>` sets threshold and maximum additional offset; source bounds `S` to **1..300 um**. `M3909 P0` disables control. EEPROM may retain thresholds. `M3001` may queue an auto-enable of SenseOffset if configured.
- SenseOffset needs active bed Z compensation and an enabled positive force threshold.
- It can react only when **`Printer::zMotion.nominalSteps() <= Printer::layers.fadeStartSteps()`**. The condition uses the executing nominal Z.
- The first decision takes **10 accepted samples**, about half a second at 50 ms spacing. The interval check can skip a poll that arrives exactly 50 ms after the last sample. Later decisions carry half the averaging window forward.
- It takes the **absolute** average reading. Above the configured limit it increases the live SenseOffset by at least **1 µm** when correction is warranted; larger pressure deviations can increase the increment, with tapering as offset approaches the maximum.
- `Printer::zOffset` converts the sum of the manual and live values to `effectiveSteps()`. [Offset ledger](../z-offset/offset-ledger.md)
- Below the pressure threshold, the additional offset **does not decrease**. It persists when leaving the low-Z region; loss/disable of bed compensation is a separate deactivation/reset path.

`M3909 P` stores a positive threshold for activation when the queue reaches the enable task. `P0` disables learning immediately and retains the stored threshold for later activation. The queue can also enable learning through `M3001` AutoStart. A poll snapshots the active limit pair and checks it again when committing a calculated increment. This rejects a pending increase if the queue changes either limit during the calculation. [Command](../../Repetier/RF.cpp#L5054) · [Queued settings load](../../Repetier/Printer.cpp#L778)

This is deliberately **not** a bidirectional PID loop. It is a bounded, positive-only first-layer clearance correction; the cap limits inappropriate reactions to a clogged nozzle or sensor error.

## Coupling to learned layer height

`Printer::layers.fadeStartSteps()` controls both where full scanned-bed following ends and where SenseOffset may regulate. The fallback limit is 0.33 mm. Generated `M3912` priming suspends SenseOffset for the entire line, including with a higher limit inherited from an earlier print. For a manually scripted prime, enable SenseOffset after priming and account for `M3001` AutoStart. [Manual start-line ordering](automatic-start-line.md#manual-start-lines)

SenseOffset checks the executing nominal Z against the learned limit. The G-code origin is resolved before this check. On the first sample outside the range the legacy window clears a positive sum and its sample count. The previous average and a nonpositive sum can carry across that boundary. Priming, pause and Z homing clear the whole window.

## Pause and resume

An accepted pause suspends automatic SenseOffset learning through the retract, park, jog, reheat and return. `Printer::pause` records the boundary for the next DMS poll, which resets the pressure average, sample count and previous average even if the pause ended between polls. The live offset and threshold are retained while bed compensation is active. The Mod Menu can still adjust the offset manually. [Pause transitions](../development-roadmap.md#pause-and-stop-transitions)

The bed profile and fade use the frozen print position. Digit-Z continues to follow live force, including force from parked manual extrusion. After return, SenseOffset needs ten fresh eligible samples before it can add clearance. [Pressure handler](../../Repetier/RF.cpp)

Run the host replay from the repository root with Python 3 and a C++11 compiler.

```sh
python3 tests/pause-sense-offset.py
```

## Intentional manual persistence of learned clearance

The operator can use alternating Z/bed DOWN and UP buttons in Mod Menu page 2 to transfer the positive SenseOffset adjustment into the saved manual Z offset in 5 µm increments while approximately maintaining the combined correction. This is explicitly documented in the original 2017 changelog and must survive future fixes. See [live transfer procedure and conditions](../z-offset/history-and-workflows.md).

## Relevant source
- [Controller and runtime state](../../Repetier/SenseOffsetControl.h)
- [Shared strain-gauge sampling](../../Repetier/RF.cpp#L2999)
- [Configuration defaults](../../Repetier/Configuration.h#L73)
- [Original intention in changelog](../../changelog.txt#L1775-L1784)
