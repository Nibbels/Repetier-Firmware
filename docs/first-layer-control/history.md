# Historical intent and source map

The relevant original decisions were documented over years. Read these before replacing the shared heuristics with a model-specific workaround.

| Date / version | Recorded intent |
| --- | --- |
| **2017-01-16**, RF.01.35d | SenseOffset introduced from Wessix's idea: bounded, positive-only near-bed force correction. [Changelog](../../changelog.txt#L1775-L1784) |
| **2017-03** | Early guidance to put the start line above 0.35 mm or enable SenseOffset afterward, avoiding false corrections during priming. [Changelog](../../changelog.txt#L1667-L1672) |
| **2018-01-01/02**, RF.01.38.05 | Automatic first-layer detection and dynamic `g_min`/`g_max`: full bed correction through first layer, fade out roughly <=5% per layer, with ECMP development. [Changelog](../../changelog.txt#L1088-L1103), [commit a02580d](https://github.com/Nibbels/Repetier-Firmware/commit/a02580d4b8c2) |
| **2018-01-28**, RF.01.38.21 | M3912 automated priming and pressure-responsive Digit Flow to avoid excessive priming force. [Changelog](../../changelog.txt#L1005-L1013) |
| **2018-05-12** | SenseOffset stored in EEPROM, optional autostart with M3001. [Changelog](../../changelog.txt#L730-L735) |
| **2019-01-08**, 01.43.36 | Fix automatic layer detection for SenseOffset and E-percent compensation. [Changelog](../../changelog.txt#L366-L368) |
| **2019-01-27**, 01.43.52 | Set special prime Z=0.352 mm to exclude it from first-layer learning; auto-adjust compensation boundaries always on. [Changelog](../../changelog.txt#L279-L282) |
| **2019-02-17**, 01.43.81 | M3912 clears `Current`/`Last` after prime for first model layers higher than priming; fallback upper bound enlarged from 3 to 10 mm. [Commit ffdcc65](https://github.com/Nibbels/Repetier-Firmware/commit/ffdcc65844b4) |
| **2026**, third-party experiment | RF2000v2-only post-prime state/boundary reset proposed but not yet validated on this branch. [Patch](https://github.com/r3dacto/Repetier-FirmwareCodex/commit/d59ca5e53c1351c47a9ab67606806968c82f976e) |

## Code entry points

| Topic | Source |
| --- | --- |
| Compile-time DMS features, time/offset constants | [Configuration.h](../../Repetier/Configuration.h#L86-L100) |
| Common nominal boundaries / special prime height | [RF1000.h](../../Repetier/RF1000.h#L762-L773), [RF2000 family](../../Repetier/RF2000.h) |
| Planner first-layer detector, auto bounds | [motion.cpp](../../Repetier/motion.cpp#L173-L219) |
| Matrix Z interpolation, fade-out, ECMP | [BedMatrix.cpp](../../Repetier/BedMatrix.cpp) · [BedCompensation.h](../../Repetier/BedCompensation.h) |
| Realized E-stepping / material correction | [motion.cpp](../../Repetier/motion.cpp#L1385-L1500) |
| DMS feed/material adjustment | [PressureFeatures.cpp](../../Repetier/PressureFeatures.cpp) |
| SenseOffset control | [PressureFeatures.cpp](../../Repetier/PressureFeatures.cpp) |
| M3909 / M3912 | [GCodeMotion.cpp](../../Repetier/GCodeMotion.cpp) · [StartLine.cpp](../../Repetier/StartLine.cpp) |
| M3001 optional SenseOffset auto-enable | [BedMatrix.cpp](../../Repetier/BedMatrix.cpp) |
| Layer marker reset on Z disabling | [Printer.h](../../Repetier/Printer.h#L355-L371) |

**Terminology:** SenseOffset adjusts clearance from excessive force. Digit Flow Compensation adjusts extrusion and feed from DMS. ECMP adjusts extrusion for geometric layer-thickness changes through the Z fade. These are three distinct mechanisms.
