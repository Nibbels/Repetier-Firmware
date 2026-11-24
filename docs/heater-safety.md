# Heater control, watchdog and PWM shutdown

## Requirements

Heating requires fresh, plausible sensor data and a foreground loop that completes temperature regulation and fault checks within a bounded time. A paused print or a stationary hotend can intentionally heat while those services run. A stall in firmware progress must terminate heater power within a measured, bounded time. Reset and boot must enter with the heater outputs off.

| Fault | Required response |
| --- | --- |
| Open, shorted, disconnected or stale temperature input | Inhibit power when the input becomes invalid. Detect open and short circuits from the expected electrical input range for each fitted sensor and interface. A low result from a broken wire must never increase requested heater power. |
| Plausible temperature with a detached sensor or failed heater | Check the measured temperature rise against elapsed time and applied power during heating. Check deviation during temperature hold and heating after the request goes to zero. Set and validate the limits for each hotend and bed profile. |
| Confirmed thermal fault | Clear the heater request and the active output, latch the fault and stop the print. Keep the fault latched through pause, configuration changes and autotune until the recovery condition is met. |
| Delayed interrupt, stalled foreground or a stuck power switch | Bound the duration of a high pulse. Strip power on loss of valid control progress. Use an independent over-temperature limit that can break heater power even if the normal switch fails closed. |

## Present firmware

- The Timer0 COMPB handler advances the heater PWM and ADC sampling. It latches duty at the beginning of the heater frame and drives both edges in software. At 16 MHz the nominal compare interval is 256 µs and the heater frame is 32.768 ms. The full-duty code holds the output high. A blocked handler can extend a high pulse. [PWM and ADC](../Repetier/HAL.cpp)
- The foreground updates temperatures on the roughly 100 ms flag. It converts supported sensors, checks the converted range and counts repeated faults. The stock decoupling defaults use a 12 s observation period, a minimum 1 °C rise on full power and a ±20 °C hold corridor. Persistent faults turn the targets off and request print stop. Audit the interval before the first invalid sample can affect power, ADC freshness, sensor-dependent range limits, shared fault counting and the autotune path. [Temperature controller](../Repetier/Extruder.cpp) · [Defaults](../Repetier/Configuration.h)
- The AVR watchdog runs in interrupt mode. Its nominal 16 ms interrupt schedules foreground work and toggles the external watchdog pin while the foreground heartbeat is recent. The PWM handler supplies a roughly 10 ms flag that the foreground consumes to refresh that heartbeat. The current main-loop timeout is 20,000 ms. This heartbeat establishes a foreground visit, and the next design must tie heater permission to successful thermal service as well. [Watchdog](../Repetier/HAL.cpp) · [Heartbeat](../Repetier/HAL.h) · [Periodic services](../Repetier/Commands.cpp) · [Microchip watchdog modes](https://developerhelp.microchip.com/xwiki/bin/view/products/mcu-mpu/8-bit-avr/structure/wdt/)
- The RF1000 and RF2000 board drawings show a TPS3820-50 on the reset circuit. TI lists a nominal 0.2 s watchdog interval for that part while WDI is driven. The published RF2000 drawing labels WDI PC0, and the RF2000-family firmware selects PA7. Verify the fitted board revision, WDI routing, reset path and heater gate levels through reset and boot before using that supervisor in a timing guarantee. [Board drawings](board-connectors.md#board-references) · [Pin definitions](../Repetier/pins.h) · [TI TPS3820 datasheet](https://www.ti.com/lit/ds/symlink/tps3820.pdf)

The Z-offset retry waits on a scan deadline. DMS sample spacing, idle-force settling and pressure retries are also represented by deadlines. The DMS service advances the hardware TWI transfer by byte events and continues the periodic work while an event is pending. Longer Z lifts and retreats run the periodic services after every 16 pulses, about 3.2 ms of configured edge delays per group. EEPROM, homing and the remaining foreground services still need an end-to-end latency measurement. [Scan move implementation](../Repetier/ScanProbe.cpp) · [DMS acquisition](z-offset/scans-and-persistence.md#shared-dms-acquisition)

## Bound the falling edge in hardware

An AVR timer can clear its OC output on compare while interrupts are masked, provided the timer clock runs and the heater's physical output is wired to that OC pin. A free-running hardware PWM can also keep applying the last duty during a foreground crash. Give each pulse a hardware end and require a fresh authorization to start another pulse. Evaluate compare-output clear with software rearm and an independent, expiring heater enable. Confirm reset bias and power-off behavior on the fitted output stage. [ATmega2560 datasheet, sections 2.3, 16.6, 17 and 20.6](https://www.microchip.com/bin/mchp/product-ds.ATmega2560.pdf)

| Heater control | MCU pin in firmware | Compare-output candidate |
| --- | --- | --- |
| Hotend 0 | PB4 | OC2A |
| Hotend 1, where fitted | PH6 | OC2B on the same timer |
| RF1000 bed | PH5 | OC4C |
| RF2000 and RF2000v2 bed | PE2 | No OC output on PE2. An interrupt-independent falling edge here would require different wiring or an external gate. |

Verify Timer2 mode and channel sharing, Timer4 assignments and board routing before allocating a compare output. Keep the timing budget for step generation, ADC, UART and fans in the comparison. For a closed switch fault, evaluate a separate sensing and power-interruption path sized for the heater. [RF pin assignments](../Repetier/pins.h) · [Example of independent heater limit protection](https://www.watlow.com/-/media/documents/user-manuals/watlow-flange-and-circulation-heater-installation-and-maintenance-manual.ashx)

## Verification and scan refactoring

Keep scan steps bounded and preserve their yields to periodic services. Record the longest gap between heater updates, the longest interval with interrupts masked and the longest high pulse under scan, storage, serial and stepper load. State dispatch, the time spent inside each action and foreground progress all matter for the watchdog.

Use disconnected heaters or an instrumented test load for fault injection. Replay broken-wire, short, stale sample, no-rise, overshoot, hold drift and autotune faults. Trace PWM and the safety enable for masked interrupts during a high pulse, a stalled foreground with interrupts still running, watchdog reset and boot. Verify the shutdown time at the power stage for each board and heater configuration.
