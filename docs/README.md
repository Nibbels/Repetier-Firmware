# Firmware engineering notes

These are source-grounded design documents, complementary to [README.md](../README.md) and [changelog.txt](../changelog.txt).

See [firmware fixes and candidate pulls](maintenance-tasks.md) for current changes and open tasks.

## Z-offset, calibration and persistence
Start at [Z-offset subsystem: engineering map](z-offset/README.md) for the offset ledger, boot/print lifecycle audit, HBS/ZOS, two EEPROM stores, physical thermal drift and source-backed candidate causes of sudden Z shifts. See [Mod Menu live transfer](z-offset/history-and-workflows.md): alternating Z-up/down deliberately converts SenseOffset into the saved manual Z-offset.

## Coordinate geometry and extrusion
[Nominal G-code vs machine-step Z, fade-out geometry and the logical E timeline](first-layer-control/coordinate-frames-and-extrusion.md), including the now-fixed nonzero absolute-E M3912 case. [Pressure-braked M3912 state management](first-layer-control/m3912-state-boundary.md).

## First-layer control
Start with [**System overview and future-chat handover**](first-layer-control/README.md).

- [SenseOffset / SensiblePressure](first-layer-control/sense-offset.md)
- [Bed-scan following, fade-out, automatic layer detection, material/ECMP](first-layer-control/z-compensation-and-ecmp.md)
- [Automatic pressure-responsive start line M3912](first-layer-control/automatic-start-line.md)
- [Lifecycle concerns, scenarios and validation](first-layer-control/test-scenarios.md)
- [Historical rationale and source map](first-layer-control/history.md)

**Status:** documentation of existing design, not a change to firmware or evidence of hardware tests.
