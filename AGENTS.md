# Firmware maintenance handover

**Development roadmap:** [Architecture and development blocks](docs/development-roadmap.md). The active block is coordinate systems and motion compensation. The [maintenance list](docs/maintenance-tasks.md) tracks its fixes and later work. Evaluate step packing in the motion smoothness and performance block. Review optional features before migrating their code. [Milling mode has been retired](docs/development-roadmap.md#feature-retirement).

**Verified coordinate semantics.** `Printer::layers` owns the pending and confirmed model layers and both fade boundaries in nominal planned Z steps. The foreground compares them with the nominal position executing in `ZMotion`. The M3912 patch restores the incoming logical E coordinate. [Coordinate frames and extrusion timeline](docs/first-layer-control/coordinate-frames-and-extrusion.md) · [M3912 patch](https://github.com/Nibbels/Repetier-Firmware/commit/5f848c9a5cd0347c6d45fd9d3f13afd7344cb775)

**M3912 uses an isolated pressure-braked priming phase.** `Printer::startLine` owns the phase, generates its moves and restores the temporary state after draining the queue. High DMS force progressively constrains XY speed and reduces E flow, like the audible change when a chainsaw enters wood. Manual start code enables SenseOffset with `M3909` after priming. [State ledger and test plan](docs/first-layer-control/m3912-state-boundary.md)

**Start with [docs/first-layer-control/README.md](docs/first-layer-control/README.md) before editing first-layer, Z compensation, DMS or priming behavior. For the full Z-offset sum, boot/print lifecycle, matrix EEPROM and thermal/scanning risks, read [docs/z-offset/README.md](docs/z-offset/README.md).** The linked documentation explains original intent, actual execution paths, risks and test scenarios.

This is shared AVR/ATmega2560 Repetier firmware for **RF1000, RF2000 and RF2000v2**. Motion planning, SenseOffset and bed correction are common code. Do not apply model-only changes without a demonstrated model-specific cause. Stepper timing and SRAM/flash limits matter; avoid expensive arithmetic in ISRs.

**Board connectors and retrofits.** Preserve named physical connector definitions and the existing controls for available switched outputs. A port can be unused in the factory build and used for equipment such as an E3D V6 or Volcano heatsink fan or a water-cooling pump. The RF2000 family shares its pin map and configuration with model-specific defaults. Check the [board notes and X44 example](docs/board-connectors.md) before changing pin assignments or retiring an output path.

**Design essentials.** DMS measures strain-gauge force. Excess force may indicate inadequate nozzle clearance, a clog or material, temperature or speed differences. SenseOffset (`M3909`) adds bounded positive clearance on the first layer. Bed-profile compensation follows the scanned bed, then fades its XY variation across later layers with geometric E-material compensation (ECMP). `M3912` primes at nominal Z=0.35 mm and suspends layer learning and SenseOffset for the duration of the line. The learned `fadeStartSteps()` boundary controls both the start of profile fade and eligibility for SenseOffset.

**Z motion.** `Printer::zMotion` owns the execution counters and travel rules. Use its operations for queued, direct, scan and compensation steps. A clipped nominal step or a step balanced by compensation can advance the path without an emitted pulse. Run `python3 tests/z-motion.py` when editing these paths. [Execution model](docs/development-roadmap.md#z-step-execution)

**Bed compensation.** `BedCompensation` calculates the bed-profile correction and ECMP ratio from executing nominal Z, planned layer bounds and a matrix reader. `recalculateZCompensation()` adds the effective offset and active Digit-Z, then publishes the Z target and ECMP ratio together under interrupt protection. [Calculation and test cases](docs/first-layer-control/z-compensation-and-ecmp.md)

**Digit compensation.** `Printer::digitCompensation` owns the DMS filter, the Digit-Z switch, the Digit Flow profile and its live factors. Keep sampling and regulation in the foreground. Timer1 reads the atomically published `intervalScale()` for queued XY moves. M220 and M221 still control the user speed and flow overrides. Run `python3 tests/pause-sense-offset.py` for the DMS sample and M3912 replays. [DMS controls](docs/development-roadmap.md#job-boundaries-and-dms)

**Strain gauge.** `Printer::strainGauge` owns ADC access and the Digit Homing reference. Keep the I²C driver independent of motion. `sampleStrainGauge()` supplies the corrected sample to the filter and to its caller. [Driver](Repetier/StrainGauge.cpp)

**SenseOffset.** `Printer::senseOffsetControl` owns the active limits, AutoStart and the pressure learning window. `ZOffset` owns the applied clearance. The foreground proposes an increment from one snapshot of the limits and checks that snapshot and the print phase when applying it. The step queue can activate stored limits during the calculation. [SenseOffset settings and replay](docs/first-layer-control/sense-offset.md)

**DMS pressure protection.** `Printer::pressureProtection` owns the limits, sampling windows and extrusion inhibit. The foreground requests pause and stop actions from the machine. The step executor reads one byte for the inhibit. [Pressure checks](docs/development-roadmap.md#job-boundaries-and-dms)

**Pause and stop.** `Printer::pause` owns the pause phase and return position. The queue, the panel and the DMS handler use its request and query methods. `Printer::printStop` owns timed abort cleanup and command admission during a stop. Run `python3 tests/pause-sense-offset.py` when changing these transitions. [Pause and stop transitions](docs/development-roadmap.md#pause-and-stop-transitions)

**Print input.** USB over UART0 is the sole external G-code input. All three models buffer 24 planned moves. The SD removal record links to the commit containing the full deletion and to the last revision with the card implementation. [SD support removal](docs/maintenance-tasks.md#sd-support-removal)

**Non-negotiable operator workflow:** In Mod Menu page 2 the Z/bed DOWN→UP button pair intentionally transfers 5 µm of live SenseOffset into the EEPROM-backed manual part of `Printer::zOffset` with approximately unchanged final clearance (see [live takeover procedure](docs/z-offset/history-and-workflows.md) and the original January 2017 changelog). Do not “fix” the asymmetric button behavior or reset dynamic offsets while a manual transfer is underway.

**Print start.** The standard USB start sequence runs `G28` before enabling bed compensation. Z homing restores the fade bounds and clears layer tracking, ECMP and live SenseOffset with its sample history. `M3079` runs the output move and disables the motors and bed compensation at the end of a print. [USB print start](docs/first-layer-control/README.md#usb-print-start) · [Test scenarios](docs/first-layer-control/test-scenarios.md)

**Git workflow:** `ai-firmware-review` is the isolated working branch; keep `community_development` unchanged until specifically instructed. The DRV8711 BLANK invalid read command was removed without changing actual default electrical timings in [commit 8f00ed8](https://github.com/Nibbels/Repetier-Firmware/commit/8f00ed8df6a0e825dee368c1233ffc4934349faa).

For future chats: read this file and the mechanism docs, verify against code and changelog, distinguish verified behavior from hypotheses, then propose isolated changes and tests rather than guessing.
