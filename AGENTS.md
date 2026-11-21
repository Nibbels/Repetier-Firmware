# Firmware maintenance handover

**Start with [docs/first-layer-control/README.md](docs/first-layer-control/README.md) before editing first-layer, Z compensation, DMS or priming behavior. For the full Z-offset sum, boot/print lifecycle, matrix EEPROM and thermal/scanning risks, read [docs/z-offset/README.md](docs/z-offset/README.md).** The linked documentation explains original intent, actual execution paths, risks and test scenarios.

This is shared AVR/ATmega2560 Repetier firmware for **RF1000, RF2000 and RF2000v2**. Motion planning, SenseOffset and bed correction are common code. Do not apply model-only changes without a demonstrated model-specific cause. Stepper timing and SRAM/flash limits matter; avoid expensive arithmetic in ISRs.

**Design essentials:** DMS measures strain-gauge force, not direct melt pressure; excess force may indicate inadequate nozzle clearance, a clog or material/temperature/speed differences. SenseOffset (M3909) adds only positive, limited clearance on the first layer. Bed-profile compensation follows the scanned bed then fades its XY variation across later layers, with geometric E-material compensation (ECMP). M3912 primes at special Z=0.352 mm and responds to force by reducing speed and extrusion; it must not be mistaken for the first model layer or trigger SenseOffset during intentional heavy priming. The dynamically learned `g_minZCompensationSteps` controls both start of profile fade and eligibility for SenseOffset.

**Investigation in progress:** some learned compensation bounds may persist between prints. Inspect [test scenarios / hypotheses](docs/first-layer-control/test-scenarios.md) before modifying reset behavior. A fork's RF2000v2-only post-priming workaround is not justified for this shared planner.

**Git workflow:** `ai-firmware-review` is the isolated working branch; keep `community_development` unchanged until specifically instructed. The DRV8711 BLANK invalid read command was removed without changing actual default electrical timings in [commit 8f00ed8](https://github.com/Nibbels/Repetier-Firmware/commit/8f00ed8df6a0e825dee368c1233ffc4934349faa).

For future chats: read this file and the mechanism docs, verify against code and changelog, distinguish verified behavior from hypotheses, then propose isolated changes and tests rather than guessing.
