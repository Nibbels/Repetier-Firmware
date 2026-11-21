# Thermal expansion, real nozzle gap and DMS interpretation

## Physical model (engineering intuition)

The nozzle-to-bed gap depends on the entire mechanical path: heated bed, heater/support plate, Z frame/screws, hotend, carriage and strain-gauge mount. Bed heating and hotend heating may change effective nozzle/bed separation in different directions; slow heat transfer means the gap may continue drifting after sensors have reached their targets.

The original project's intent was to allow a carefully calibrated, slightly close nominal first-layer setting, and use bounded SenseOffset to *increase clearance when high extrusion force suggests the gap became too tight*. This assumes DMS force limits are calibrated for actual nozzle, material, temperature, feedrate and extrusion flow. It does not prove that measured force uniquely maps to Z gap.

## Historical values found in source: advisory only

`RF.cpp::calculateZScrewCorrection()` uses:
- **0.95 µm / °C** for hotend length change;
- **1.5 µm / °C** for bed expansion;
- **150 µm** as a maximum additional 'Nachdehnung' / warming allowance;
- **50 µm** desired remaining margin at the projected upper temperatures.

It prints **Z-screw correction advice** after ZOS and does **not** apply live corrections to `Printer::ZOffset` or `g_staticZSteps` as bed/hotend temperature changes. The coefficients are approximate assumptions for the historic hardware, **not calibrated for E3D** or interchangeable hotends. The precise thermal drift in the first ~10 minutes was not established from source; a rough 0.01 mm observed transient and the 150 µm source allowance are **different quantities** and should not be equated.

This distinction is essential for diagnosing a 'random Z-offset': an observed physical gap change can occur while every software offset variable is unchanged.

[Source: helper and values](../../Repetier/RF.cpp#L2638-L2750) · [helper called on ZOS finish](../../Repetier/RF.cpp#L2585-L2605).

## Illustrative SenseOffset thresholds (not universal defaults)

User experience, as examples of meaningful DMS digits limits with correctly configured flow/material:
- PLA: roughly **1500–2000 digits**;
- PETG: roughly **2500–3500 digits**;
- flexible filament: **below ~1000 digits**.

These are **empirical machine-specific examples**, not recommended universal settings. Extrusion force depends on nozzle diameter and geometry, polymer rheology, temperature, print speed, flow, sensor baseline and mounting. A small nozzle or a cold/blocked hotend can raise digits without a bed-distance error. The firmware's baseline/zero-digits option can also affect displayed digit readings.

Only enable aggressive first-layer clearance correction after establishing the normal force for the chosen nozzle/material/temperature/speed combination, and maintain an appropriate maximum offset (current compiled default 180 µm; menu/G-code can configure up to 300 µm).

## Contact scan error sources

The nozzle/hotend is held through DMS strain gauges, but the filament strand, heater wires, thermistor and fan wiring also exert external loads. Thermal expansion or changing filament tension can appear as force even without true nozzle/bed contact. A residue of melted filament at the nozzle tip can act like a deformable spacer, or ooze while measuring. A compliant build surface or contaminated spot also changes the force-versus-displacement contact curve.

A good calibration requires stable heat distribution, minimal side loading and a clean, firm contact interface. `readAveragePressure()` checks short-window repeatability, not a fundamental guarantee of exact zero distance. Repeat readings, temperature logs and carefully managed cable/filament routing are better evidence than assuming every stable digit value means firm contact.

## Operator capitalization of a learned correction

If SenseOffset accumulates significant positive correction and the operator concludes the **persistent manual** Z baseline is miscalibrated (for example after swapping magnetic build plates or slight mechanical movement when removing a print), the firmware has a deliberate Mod Menu Z-up/down procedure to **transfer** this correction into EEPROM-backed `Printer::ZOffset` while approximately maintaining total clearance. This is a manual decision, **not automatic thermal compensation**. Material/temperature-dependent force changes should not automatically become a permanent offset. See [Mod Menu live transfer](history-and-workflows.md).

## Observational troubleshooting

Record **temperature and elapsed time** alongside the Z ledger: bed/nozzle actual and target temps, time since heating started, last HBS/ZOS temperature profile, tool, scanned matrix slot, menu ZOffset, dynamic SenseOffset, active compensation and measured DMS digits. Distinguish (1) software state jump, (2) continuous thermal drift, and (3) a bad scan that installed a biased matrix.

Relevant technical details: [scan reliability](scans-and-persistence.md), [offset lifecycle](lifecycle-and-findings.md), [SenseOffset](../first-layer-control/sense-offset.md).
