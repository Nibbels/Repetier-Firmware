# Thermal expansion, real nozzle gap and DMS interpretation

## Physical model (engineering intuition)

The nozzle-to-bed gap depends on the entire mechanical path: heated bed, heater/support plate, Z frame/screws, hotend, carriage and strain-gauge mount. Bed heating and hotend heating may change effective nozzle/bed separation in different directions; slow heat transfer means the gap may continue drifting after sensors have reached their targets.

The original project's intent was to allow a carefully calibrated, slightly close nominal first-layer setting, and use bounded SenseOffset to *increase clearance when high extrusion force suggests the gap became too tight*. This assumes DMS force limits are calibrated for actual nozzle, material, temperature, feedrate and extrusion flow. It does not prove that measured force uniquely maps to Z gap.

## Elastic hotend deflection and Digit-Z

Extrusion resistance loads the hotend's DMS mount. The mount flexes and the nozzle tip moves toward the bed. Digit-Z lowers the bed by an estimate of that elastic displacement. The original 2017 implementation called this `FEATURE_SENSIBLE_COMPENSATION` and used **0.01 mm per 1,000 digits**. The later source note gives roughly **0.05 mm of deflection at 5,500 digits**. These are historical estimates for the measured assembly. [Original implementation](https://github.com/Nibbels/Repetier-Firmware/commit/e4fba6b263942764cac9912d31430a4848298477) · [Later source note](../../Repetier/RF.cpp#L3273-L3287)

The current implementation uses `abs(filteredDigits) / 131072` mm, truncated to whole microsteps and capped at 600 steps. This gives **7.63 µm per 1,000 digits** before quantization. At 5,500 digits and the compiled 1,280 Z steps/mm it requests **53 steps, or 41.4 µm**, about 9 µm below the 50 µm estimate. The divisor and the single historical measurement establish the approximate scale. Actual accuracy across force, direction and different hotend mounts requires displacement measurements. [Digit-Z calculation](../../Repetier/BedCompensation.h#L64) · [Microstep setting](../../Repetier/Configuration.h#L238) · [Steps per mm](../../Repetier/RF2000.h#L134)

**Reversibility requirement.** At the same steady DMS reading, the elastic correction should settle to the same Z step count after a load excursion. The formula already gives the same steps for the same filtered input. A host replay at a steady raw reading of 5,530 digits produced 53 steps after approaching from below and 54 after approaching from above. The average drops the division remainder on each update, which can leave it up to three digits from a constant input. A trial that retained this remainder converged to 54 steps from both directions. [DMS filter](../../Repetier/DigitCompensation.h#L56) · [Open verification task](../maintenance-tasks.md#digit-z)

The ADS1100 shown in the RF1000 and RF2000 schematics uses continuous conversion at nominal **8 samples/s** with the firmware's `0x8C` setting. Normal firmware service reads it about every 50 ms, so multiple reads can contain the same conversion. After startup, each read adds one quarter of the new reading to three quarters of the previous filtered result. A host replay of a step in the reported reading reached about **90% after eight reads**, roughly **0.4 s** at that polling rate. The observation starts at the first changed reading. ADC conversion, foreground target refresh and actual stepper motion all contribute to the load-to-bed delay. [RF1000 schematic](https://asset.conrad.com/media10/add/160267/c1/-/en/001007508CD01/circuit-diagramm-1007508-renkforce-rf1000-3d-printer.pdf) · [RF2000 schematic](https://asset.conrad.com/media10/add/160267/c1/-/en/001395717CD01/schema-electrique-1395717-renkforce-rf2000-imprimante-3d-appareil-pret-a-lemploi-banc-dimpression-chauffant.pdf) · [ADS1100 Tables V–VII](https://www.ti.com/lit/ds/symlink/ads1100.pdf#page=10) · [Foreground scheduling](../../Repetier/Commands.cpp#L70-L92) · [Stepper following](../../Repetier/motion.cpp#L969-L1056)

To establish whether the response is fast enough, measure nozzle deflection through rising and falling force at fixed XYZ, then record DMS, target Z correction and applied Z correction together during a load change. This gives both the force-to-deflection slope and the residual gap error during a transient. SenseOffset remains the slower first-layer clearance adjustment and can retain learned clearance as the load falls. [SenseOffset](../first-layer-control/sense-offset.md)

## Historical values found in source: advisory only

`RF.cpp::calculateZScrewCorrection()` uses:
- **0.95 µm / °C** for hotend length change;
- **1.5 µm / °C** for bed expansion;
- **150 µm** as a maximum additional 'Nachdehnung' / warming allowance;
- **50 µm** desired remaining margin at the projected upper temperatures.

It prints **Z-screw correction advice** after ZOS and does **not** apply live corrections to the manual or effective offset held by `Printer::zOffset` as bed/hotend temperature changes. The coefficients are approximate assumptions for the historic hardware, **not calibrated for E3D** or interchangeable hotends. The precise thermal drift in the first ~10 minutes was not established from source; a rough 0.01 mm observed transient and the 150 µm source allowance are **different quantities** and should not be equated.

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

If SenseOffset accumulates significant positive correction and the operator concludes the **persistent manual** Z baseline is miscalibrated (for example after swapping magnetic build plates or slight mechanical movement when removing a print), the firmware has a deliberate Mod Menu Z-up/down procedure to **transfer** this correction into the EEPROM-backed manual Z offset while approximately maintaining total clearance. This is a manual decision, **not automatic thermal compensation**. Material/temperature-dependent force changes should not automatically become a permanent offset. See [Mod Menu live transfer](history-and-workflows.md).

## Observational troubleshooting

Record **temperature and elapsed time** alongside the Z ledger: bed/nozzle actual and target temps, time since heating started, last HBS/ZOS temperature profile, tool, scanned matrix slot, menu ZOffset, dynamic SenseOffset, active compensation and measured DMS digits. Distinguish (1) software state jump, (2) continuous thermal drift, and (3) a bad scan that installed a biased matrix.

Relevant technical details: [scan reliability](scans-and-persistence.md), [offset lifecycle](lifecycle-and-findings.md), [SenseOffset](../first-layer-control/sense-offset.md).
