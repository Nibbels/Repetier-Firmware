# Firmware engineering notes

These are source-grounded design documents, complementary to [README.md](../README.md) and [changelog.txt](../changelog.txt).

Start with the [development roadmap](development-roadmap.md) for the active coordinate and motion-compensation block and the intended firmware architecture. The [maintenance list](maintenance-tasks.md) tracks fixes and open tasks by development block.

## Build, licensing and diagnostics

Doxygen API generation is retired; use the maintained Markdown notes and source. Completed private research CI definitions have been retired without removing their source-pinned evidence. The canonical firmware and active Task0032 simulation workflows remain.

- [Final build and cleanup audit](../coordination/audits/final-cleanup.md)
- [AVR disassembly and RAM inspection commands](overflow_decompile.txt)
- [Historical Repetier communication protocol](../repetier%20communication%20protocol.txt) (not a complete current specification)
- [GNU GPL v3](../gpl.txt)

## Motion throughput and real-time timing

The [performance goal and timing plan](motion-timing.md) traces step generation, serial streaming and peripheral service on the ATmega2560. It defines the preparation and measurements for changing Pressure Advance scheduling and step packing.

## Board connectors and retrofits

The [board connector reference](board-connectors.md) maps RF1000 and RF2000 family outputs, sensor ports and expansion headers. It includes the single-extruder RF2000 setup with an E3D heatsink fan on X44.

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
