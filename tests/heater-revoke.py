#!/usr/bin/env python3
"""Exercise the actual HAL zero-demand and PWM frame-start source on host."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
hal = (root / "Repetier/HAL.cpp").read_text()
start = hal.index("void HAL::setHeaterDuty(")
end = hal.index("\nvoid HAL::setBoardFan", start)
setter = hal[start:end]
start = hal.index("    if (pwm_count_heater == 0) {", hal.index("ISR(PWM_TIMER_VECTOR)"))
end = hal.index("    if (pwm_count_cooler == 0)", start)
frame = hal[start:end]
# Establish the assumption that heater HIGH writes only occur in the frame block.
isr = hal[hal.index("ISR(PWM_TIMER_VECTOR)"):hal.index("} // ISR(PWM_TIMER_VECTOR)")]
for pin in ("EXT0_HEATER_PIN", "EXT1_HEATER_PIN", "HEATED_BED_HEATER_PIN"):
    assert isr.count(f"WRITE({pin}, HIGH)") == frame.count(f"WRITE({pin}, HIGH)") == 1

prelude = r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#define EXT0_HEATER_PIN 0
#define EXT1_HEATER_PIN 1
#define HEATED_BED_HEATER_PIN 2
#define LOW false
#define HIGH true
static bool pins[3];
static bool irq = true;
static unsigned writes;
struct InterruptProtectedBlock {
    bool was = irq;
    InterruptProtectedBlock() { irq = false; }
    ~InterruptProtectedBlock() { irq = was; }
};
#define WRITE(pin, level) do { assert(!irq); pins[pin] = level; ++writes; } while (0)
static volatile uint8_t requestedDuty[NUM_EXTRUDER + 3];
static constexpr uint8_t heaterPwmMask = 0xfe;
static uint8_t pwm_pos_set[NUM_EXTRUDER + 3];
struct HAL { static void setHeaterDuty(uint8_t, uint8_t); };
static void startFrame(uint8_t pwm_count_heater) {
    InterruptProtectedBlock lock;
"""
main = r"""
int main() {
    unsigned cases = 0;
    for (unsigned channel = 0; channel <= NUM_EXTRUDER; ++channel) {
        unsigned pin = channel == NUM_EXTRUDER ? HEATED_BED_HEATER_PIN : channel;
        for (unsigned previous = 0; previous < 256; ++previous) {
            for (unsigned phase = 0; phase < 256; phase += 2) {
                for (bool enabled : {false, true}) {
                    irq = enabled;
                    for (auto &p : pins) p = true;
                    for (auto &d : requestedDuty) d = 254;
                    HAL::setHeaterDuty(channel, previous);
                    // Simulate the most adverse latched state: heater still high.
                    pins[pin] = true;
                    pwm_pos_set[channel] = previous & 254;
                    HAL::setHeaterDuty(channel, 0);
                    assert(irq == enabled);
                    assert(requestedDuty[channel] == 0);
                    assert(!pins[pin]);
                    for (unsigned other = 0; other < 3; ++other)
                        if (other != pin) assert(pins[other]);
                    startFrame(phase);
                    assert(!pins[pin]);
                    startFrame(0);
                    assert(!pins[pin]);
                    // A fresh nonzero demand does not itself write the output.
                    unsigned before = writes;
                    HAL::setHeaterDuty(channel, 128);
                    assert(writes == before && !pins[pin] && irq == enabled);
                    startFrame(0);
                    assert(pins[pin]);
                    ++cases;
                }
            }
        }
    }
    for (unsigned channel = NUM_EXTRUDER + 1; channel < 256; ++channel) {
        unsigned before = writes;
        HAL::setHeaterDuty(channel, 0);
        HAL::setHeaterDuty(channel, 255);
        assert(writes == before);
    }
    std::printf("heaters=%u revoke cases=%u\n", NUM_EXTRUDER + 1, cases);
}
"""
with tempfile.TemporaryDirectory() as temp:
    source = Path(temp) / "revoke.cpp"
    source.write_text("#include <initializer_list>\n" + prelude + frame + "\n}\n" + setter + "\n" + main)
    for count in (1, 2):
        exe = Path(temp) / "revoke"
        subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-O2",
                        f"-DNUM_EXTRUDER={count}", str(source), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
