# Firmware engineering notes

These are source-grounded design documents, complementary to [README.md](../README.md) and [changelog.txt](../changelog.txt).

## Z-offset, calibration and persistence
Start at [Z-offset subsystem: engineering map](z-offset/README.md) for the offset ledger, boot/print lifecycle audit, HBS/ZOS, two EEPROM stores, physical thermal drift and source-backed candidate causes of sudden Z shifts.

## First-layer control
Start with [**System overview and future-chat handover**](first-layer-control/README.md).

- [SenseOffset / SensiblePressure](first-layer-control/sense-offset.md)
- [Bed-scan following, fade-out, automatic layer detection, material/ECMP](first-layer-control/z-compensation-and-ecmp.md)
- [Automatic pressure-responsive start line M3912](first-layer-control/automatic-start-line.md)
- [Lifecycle concerns, scenarios and validation](first-layer-control/test-scenarios.md)
- [Historical rationale and source map](first-layer-control/history.md)

**Status:** documentation of existing design, not a change to firmware or evidence of hardware tests.
