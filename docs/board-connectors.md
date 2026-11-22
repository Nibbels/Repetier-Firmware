# RFx000 board connectors and retrofits

Keep the connector designators and signal definitions in [pins.h](../Repetier/pins.h) even when a factory profile leaves the connector free. They describe the board and let a retrofit name the connector in its configuration. RF2000 and RF2000v2 use the same firmware pin map and share [RF2000.h](../Repetier/RF2000.h). `MOTHERBOARD` still selects the model defaults and identifies the machine.

## Switched outputs

The numbers below are firmware digital pin numbers. Connector contact numbers and wiring are in the drawings.

| Connector | RF1000 | RF2000 and RF2000v2 | Firmware role |
| --- | --- | --- | --- |
| X19 / OUT1 | `X19_PIN = 25` | `X19_PIN = 25` | Switched accessory output via `M355` and the light menu. Can be assigned to a hotend heatsink fan. |
| X24 / OUT2 | `X24_PIN = 27` | `X24_PIN = 27` | Part cooling via `M106` / `M107`. On the RF1000 drawing the optional X20 connector shares OUT2. |
| X42 / OUT3 | — | `X42_PIN = 33` (`FET1`) | `M3300 P1 S0` / `S1` and the menu, or an assigned automatic hotend cooler. |
| X44 / OUT4 | — | `X44_PIN = 32` (`FET2`) | `M3300 P2 S0` / `S1` and the menu, or an assigned automatic hotend cooler. |
| X45 / OUT5 | — | `X45_PIN = 58` (`FET3`) | Automatic case fan. `M3120` / `M3121` and the existing `M3300 P3` address this same output. |
| X43 | — | `X43_R_PIN = 6`, `X43_G_PIN = 7`, `X43_B_PIN = 8` | RGB lighting. |
| X4 / HZ1 and X8 / HZ2 | `X4_PIN = 10`, `X8_PIN = 9` | Same | Heater circuits for T0 and T1. The active second heater depends on `NUM_EXTRUDER`. |
| X10 / HZ3 | `X10_PIN = 8` | `X10_PIN = 91` | Shared with the bed heater control signal. X10 is optional in the RF1000 drawing. |

X42 and X44 are independently switched outputs. For instance, when X44 runs the hotend cooler, X42 can still switch a water-cooling pump via `M3300 P1`. X19 can separately switch an accessory via `M355` when available for manual control. The mains socket X39 on the RF2000 drawing has its own `OUTPUT_230V_PIN` / `M3301` path.

## Sensor and expansion connections

- X5, X6 and X9 use ADC channels 13, 14 and 15. X35 on the RF2000 family uses ADC channel 12 (`X35_ADC_CHANNEL`) for the optional temperature sensor read by `M105` and `M3941`.
- The RF1000 X25 optional I/O header and the RF2000 family X33 and X34 expansion headers have `RESERVE_*` definitions in `pins.h`. The comments identify the connector contacts. On the RF1000 board, X25 also exposes signals shared with the watchdog and bed heater. On the RF2000 family, X34 carries the existing I²C bus for the DMS and board EEPROMs.
- The RF1000 uses X13, X14 and X15 for X-min, Y-min and Z-min. The RF2000 family adds X36 for Z-max. The installed switches use active-low inputs with pull-ups.
- X21 is the stock four-line character LCD connector. The stock LCD wiring ties R/W low, so firmware writes use the existing 320 μs delay.

### Temperature sensor IDs

The sensor ID selects the ADC-to-temperature conversion. Set the hotend sensor defaults with `EXT0_TEMPSENSOR_TYPE` and `EXT1_TEMPSENSOR_TYPE` in [Configuration.h](../Repetier/Configuration.h). The hotend profile independently sets the temperature limit and factory PID values.

| ID | Conversion | Sensor |
| --- | --- | --- |
| `1` | NTC | EPCOS B57560G0107F000, shown as Pico/RepRap on the LCD |
| `2` | NTC | 200 kΩ thermistor |
| `3` | NTC | EPCOS G550, 100 kΩ, used in the Conrad Renkforce V2 hotend and listed as the Mendel Parts thermistor |
| `4` | NTC | 10 kΩ thermistor |
| `8` | NTC | ATC Semitec 104-GT2 for E3D hotends, with a 300 °C entry in the calibration table |
| `9` | NTC | Honeywell 135-104LAG-J01, 100 kΩ |
| `10` | NTC | Vishay NTCS0603E3104FXT, 100 kΩ, 0603 SMD, 4.7 kΩ pull-up |
| `11` | NTC | GE Sensing AL03006-58.2K-97-G1, 100 kΩ, 4.7 kΩ pull-up |
| `12` | NTC | RS 198-961, 100 kΩ, 4.7 kΩ pull-up |
| `13` | NTC | 100 kΩ, 3950 K, Conrad V3 calibration table |
| `14` | NTC | 100 kΩ, 3950 K, alternate calibration table |
| `53` | Pt100 interface | E3D PT100 board with analog voltage output to the ADC |
| `60` | Thermocouple amplifier | AD8495, scaled at 5 mV/°C |
| `100` | Thermocouple amplifier | AD595, scaled at 10 mV/°C |

The RF1000 and RF2000 hotends default to `3`, and the RF2000v2 hotend defaults to `13`. The heated bed defaults to `3` on all three models. The RF2000 family's X35 input defaults to `14`.

The LCD offers `1`, `3`, `8` and `13` for hotends and `3` and `4` for the bed. For hotends and the bed, the other conversions can be selected through the build defaults or EEPROM. A configured EEPROM sensor value takes precedence at startup. [Conversion code](../Repetier/Extruder.cpp)

## RF2000 single-extruder example with an E3D V6 or Volcano

This reproduces the documented retrofit with the T0 heatsink fan on X44. Add the settings near the model selection in [Configuration.h](../Repetier/Configuration.h), or pass the same definitions to the compiler.

```cpp
#define MOTHERBOARD DEVICE_TYPE_RF2000
#define NUM_EXTRUDER 1
#define EXT0_EXTRUDER_COOLER_PIN X44_PIN
#define SDSUPPORT 0
#define MAX_ROOM_TEMPERATURE 25
```

`X44_PIN` resolves to the same `32` as the previous configuration. The two-extruder RF2000 factory profile still defaults to both cooler pins unassigned. An assigned heatsink fan uses the existing per-tool cooler setting and temperature logic. The cooler owns X44 in this example, so `M3300 P2` and the X44 accessory menu cannot switch it off. The same ownership rule applies to X42 and X19.

Choose the [sensor ID](#temperature-sensor-ids) for the installed sensor. The shipped hotend profiles are defined in [Configuration.h](../Repetier/Configuration.h). Stored PID settings take precedence at boot.

`SDSUPPORT=0` and `MAX_ROOM_TEMPERATURE=25` describe this specific machine. The common defaults are SD enabled and a 33 °C room-temperature threshold.

## Board references

- Conrad I851 CTC V2.1 main-board drawing for RF1000, sheets 1–2, supplied as `001197630-sp-01-en-HAUPTPLATINE_I851_CTC_V2_1_KOMPLETT.pdf`. [RF1000 schematic](https://asset.conrad.com/media10/add/160267/c1/-/en/001007508CD01/circuit-diagramm-1007508-renkforce-rf1000-3d-printer.pdf)
- Conrad I851 CTC V3.0 drawing for RF2000, sheets 1–2 and 6, supplied as `001395717-sp-01-en-RENKFORCE_3D_DRUCKER_RF2000.pdf`. The firmware numbers in the table follow the board mapping used by `fastio` and the RF profiles. [RF2000 schematic](https://asset.conrad.com/media10/add/160267/c1/-/en/001395717CD01/schema-electrique-1395717-renkforce-rf2000-imprimante-3d-appareil-pret-a-lemploi-banc-dimpression-chauffant.pdf)
- Conrad RF2000v2 installation and conversion guide, main PCB wiring diagram, page 66. [RF2000v2 wiring diagram](https://www.manualslib.de/manual/488186/Renkforce-Rf2000-V2-1563101.html?page=66)
