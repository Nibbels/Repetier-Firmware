#!/usr/bin/env python3
"""Task0010: actual sensor, controller, PWM and scheduler safety replays.

GAP marks unresolved sensor validity behavior, not a safety requirement.
Software duty requests and simulated pin transitions are distinct; this is
not physical sensor/relay or AVR timing certification.
"""
import os
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

FW = Path(__file__).resolve().parents[1] / "Repetier"


def section(source, start, end):
    a = source.index(start)
    return source[a:source.index(end, a)]


def macro(source, name):
    return re.search(r"(?m)^\s*#define\s+" + name + r"\s+([^/\n]+)", source)[1].strip()


HARNESS = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
using millis_t = uint32_t;
#define PROGMEM
#define TEMP_INT_TO_FLOAT(t) ((float)(t) / (float)(1 << CELSIUS_EXTRA_BITS))
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define LOW 0
#define HIGH 1
#define LED_PIN -1
#define EXT0_HEATER_PIN 0
#define EXT1_HEATER_PIN 1
#define HEATED_BED_HEATER_PIN NUM_EXTRUDER
#define EXT0_EXTRUDER_COOLER_PIN -1
#define EXT1_EXTRUDER_COOLER_PIN -1
#define FAN_BOARD_PIN -1
#define WATCHDOG_PIN 5
#define NUM_TEMPERATURE_LOOPS (NUM_EXTRUDER + 1)
#define PRINTER_FLAG2_GOT_TEMPS 1
#define PSTR(s) s
#define UI_STATUS_UPD(x) do {} while (0)
static uint8_t pins[8];
#define WRITE(pin,value) (pins[pin] = (value))
#define READ(pin) (pins[pin])
static uint16_t PWM_OCR;
static millis_t now;
static volatile uint16_t temperatureAdcValues[NUM_TEMPERATURE_LOOPS];
struct InterruptProtectedBlock {};
static uint8_t pgm_read_byte(const uint8_t* p) { return *p; }
static int16_t pgm_read_word(const int16_t* p) { return *p; }
static const int16_t* pgm_read_word(const int16_t* const* p) { return *p; }
struct HAL {
    static millis_t timeInMilliseconds() { return now; }
    static uint16_t temperatureAdcSample(uint8_t);
    static void setHeaterDuty(uint8_t, uint8_t);
    static uint8_t outputDuty(uint8_t);
    static void tellWatchdogOk();
    static bool takeScanTick();
    static bool takeCompensationTick();
    static bool takeFeatureTick();
    static bool takeTemperatureTick();
    static bool temperatureTickPending();
    static bool watchdogEnabled();
};
#include "controller.inc"
struct Extruder {
    TemperatureController tempControl;
    uint8_t coolerPWM, coolerSpeed;
    static Extruder* current;
    static void manageTemperatures();
};
static Extruder extruder[] = {
    {{0, 13, 0, 280}, 0, 255},
#if NUM_EXTRUDER > 1
    {{1, 13, 1, 280}, 0, 255},
#endif
};
Extruder* Extruder::current = &extruder[0];
static TemperatureController bed{NUM_EXTRUDER, 13, NUM_EXTRUDER, 150};
static TemperatureController* tempController[NUM_TEMPERATURE_LOOPS] = {
    &extruder[0].tempControl,
#if NUM_EXTRUDER > 1
    &extruder[1].tempControl,
#endif
    &bed
};
namespace Printer {
    static bool fault;
    static unsigned stops;
    static unsigned flag2, debugLevel;
    static bool isAnyTempsensorDefect() { return fault; }
    static void setSomeTempsensorDefect(bool f) { fault = f; }
    static void stopPrint() {
        for (auto* t : tempController)
            assert(t->targetTemperatureC == 0 && HAL::outputDuty(t->pwmIndex) == 0);
        ++stops;
    }
    static bool debugDryrun() { return (debugLevel & 8) != 0; }
    struct { void cancel() {} void schedule(millis_t) {} } idleStatus;
    struct { bool isReheating() { return true; } } pause;
    struct { void service(millis_t) {} } strainGauge;
}
namespace Com {
    #include "com.inc"
    template<class... T> void printErrorFLN(T...) {}
    template<class... T> void printInfoFLN(T...) {}
    template<class... T> void printF(T...) {}
    template<class... T> void printFLN(T...) {}
    void println() {}
}
static unsigned stores;
static TemperatureController* tuned;
namespace EEPROM {
    void storeDataIntoEEPROM() {
        assert(tuned && tuned->targetTemperatureC == 0 && HAL::outputDuty(tuned->pwmIndex) == 0);
        ++stores;
    }
}
namespace RMath { using std::min; using std::max; }
static int ui_text_temperature_manager, ui_text_sensor_error, ui_text_heater_error;
static int ui_text_autodetect_pid, ui_text_temperature_wrong, ui_text_timeout;
static void showError(void*, void*) {
    if (tuned) assert(HAL::outputDuty(tuned->pwmIndex) == 0);
}
enum FirmwareState { NotBusy, WaitHeater };
struct Commands {
    static void checkForPeriodicalActions(FirmwareState);
    static void printTemperatures() {}
};
static void reportTempsensorAndHeaterErrors();
#include "conversion.inc"
#include "hal.inc"
#include "control.inc"
static unsigned visits;
static millis_t stride = 10;
enum class Injection { None, Reach, LateUnsupported, LateLow, Transient, High, Cutoff, Cycles, OtherFault };
static Injection injection;
static bool postTemperature = true;
namespace GCodeInput {
    void keepAlive(FirmwareState) {
        ++visits;
        assert(visits < 1000);
        now += stride;
        scanTick = true;
        if (postTemperature) temperatureTick = true;
        if (injection == Injection::Reach && visits == 3) {
            extruder[0].tempControl.sensorType = 60;
            temperatureAdcValues[0] = 102;
        }
        if (injection == Injection::LateUnsupported || injection == Injection::Transient)
            tuned->sensorType = 255;
        if (injection == Injection::LateLow) {
            tuned->sensorType = 14;
            temperatureAdcValues[tuned->sensorPin] = 4092;
        }
        if (injection == Injection::High)
            temperatureAdcValues[tuned->sensorPin] = 820; // ~190-200 C
        if (injection == Injection::Cutoff)
            temperatureAdcValues[tuned->sensorPin] = 670; // ~153-164 C, above bed cutoff but below target+40
        if (injection == Injection::Cycles)
            temperatureAdcValues[tuned->sensorPin] = visits % 2 ? 470 : 330;
        if (injection == Injection::OtherFault)
            assert(tuned != &bed); // bed already has 10 consecutive invalid samples

    }
}
static struct {
    millis_t lastButtonTime() { return 0; }
    void slowAction() {
        if (injection == Injection::Transient) {
            assert(HAL::outputDuty(tuned->pwmIndex) == 0);
            tuned->sensorType = 60; // valid again before autotune's next read
        }
    }
} uid;
static void recalculateZCompensation() {}
static void loopFeatures() {}
static void handleScanWorkTasks(bool) {}
#include "scheduler.inc"

static void service() {
    now += 100;
    Extruder::manageTemperatures();
}
static void allOff() {
    assert(Printer::fault && Printer::stops == 1 && Printer::debugDryrun());
    for (auto* c : tempController)
        assert(c->targetTemperatureC == 0 && HAL::outputDuty(c->pwmIndex) == 0);
}
int main(int argc, char** argv) {
    assert(argc == 2);
    const char* name = argv[1];
    auto& c = extruder[0].tempControl;
    for (auto* t : tempController) {
        t->sensorType = 60;
        temperatureAdcValues[t->sensorPin] = 102; // plausible room-temperature sample
        t->pidMax = 255;
        t->targetTemperatureC = 100;
        t->pidPGain = 2;
        t->pidIGain = 0.5;
        t->pidDGain = 3;
        t->decoupleTestPeriod = EXT0_DECOUPLE_TEST_PERIOD;
    }
    if (!std::strcmp(name, "rail")) {
        c.sensorType = 13;
        temperatureAdcValues[0] = 4092;
        service();
        // GAP: supported rail maps to 0 C, so max duty is requested.
        assert(c.currentTemperatureC == 0 && !Printer::fault);
        assert(HAL::outputDuty(0) == 255);
        c.setTargetTemperature(0);
        assert(HAL::outputDuty(0) == 0);
    } else if (!std::strcmp(name, "invalid-low")) {
        c.sensorType = 14;
        temperatureAdcValues[0] = 4092;
        for (int i = 0; i < 10; ++i) {
            HAL::setHeaterDuty(0, 255);
            service();
            assert(c.currentTemperatureC < MIN_DEFECT_TEMPERATURE);
            assert(!Printer::fault && HAL::outputDuty(0) == 0);
            assert(HAL::outputDuty(bed.pwmIndex) == 255);
            assert(c.tempPointer == 0); // invalid readings never enter the PID history
        }
        service();
        assert(c.isSensorDefect());
        allOff();
    } else if (!std::strcmp(name, "recovery")) {
        service();
        assert(HAL::outputDuty(0) == 255);
        c.sensorType = 14;
        temperatureAdcValues[0] = 4092;
        service();
        assert(HAL::outputDuty(0) == 0 && !Printer::fault);
        c.sensorType = 60;
        temperatureAdcValues[0] = 102;
        service();
        assert(HAL::outputDuty(0) == 255 && !Printer::fault);
    } else if (!std::strcmp(name, "disabled")) {
        c.sensorType = 0;
        bed.sensorType = 0;
        HAL::setHeaterDuty(0, 255);
        HAL::setHeaterDuty(bed.pwmIndex, 255);
        service();
        assert(!Printer::fault && HAL::outputDuty(0) == 0 && HAL::outputDuty(bed.pwmIndex) == 0);
    } else if (!std::strcmp(name, "invalid-high")) {
        c.sensorType = 60;
        temperatureAdcValues[0] = 4092;
        HAL::setHeaterDuty(0, 255);
        service();
        assert(c.currentTemperatureC > MAX_DEFECT_TEMPERATURE);
        assert(!Printer::fault && HAL::outputDuty(0) == 0 && c.tempPointer == 0);
    } else if (!std::strcmp(name, "bed-invalid")) {
        bed.sensorType = 14;
        temperatureAdcValues[bed.sensorPin] = 4092;
        HAL::setHeaterDuty(bed.pwmIndex, 255);
        service();
        assert(HAL::outputDuty(bed.pwmIndex) == 0 && HAL::outputDuty(0) == 255 && !Printer::fault);
    } else if (!std::strcmp(name, "unsupported")) {
        c.sensorType = 255;
        HAL::setHeaterDuty(0, 255);
        for (int i = 0; i < 10; ++i) {
            service();
            assert(HAL::outputDuty(0) == 0 && !Printer::fault);
        }
        service();
        assert(c.isSensorDefect());
        allOff();
    } else if (!std::strcmp(name, "no-rise")) {
        for (auto* t : tempController) if (t != &c) t->setTargetTemperature(0);
        service();
        assert(c.isDecoupleFull() && HAL::outputDuty(0) == 255);
        now += EXT0_DECOUPLE_TEST_PERIOD;
        for (int i = 0; i < 10; ++i) {
            service();
            assert(!Printer::fault && HAL::outputDuty(0) == 255);
        }
        service();
        assert(c.isSensorDecoupled());
        allOff();
    } else if (!std::strcmp(name, "pwm")) {
        for (auto* t : tempController) HAL::setHeaterDuty(t->pwmIndex, 255);
        pwmSlice();
        for (auto* t : tempController) {
            assert(pins[t->pwmIndex] == HIGH);
            t->setTargetTemperature(0);
            assert(HAL::outputDuty(t->pwmIndex) == 0);
        }
        // Request zero is not a synchronous pin clear; frame must advance.
        for (int i = 0; i < 127; ++i) {
            pwmSlice();
            for (auto* t : tempController) assert(pins[t->pwmIndex] == HIGH);
        }
        pwmSlice();
        for (auto* t : tempController) assert(pins[t->pwmIndex] == LOW);
    } else if (!std::strcmp(name, "watchdog")) {
        watchdogActive = true;
        now = 100;
        Commands::checkForPeriodicalActions(NotBusy);
        assert(watchdogHeartbeat == 0);
        scanTick = true;
        Commands::checkForPeriodicalActions(NotBusy);
        assert(watchdogHeartbeat == now && !temperatureTick);
        pingWatchdog();
        assert(pins[WATCHDOG_PIN] == 1);
        now += WATCHDOG_MAIN_LOOP_TIMEOUT;
        pingWatchdog();
        assert(pins[WATCHDOG_PIN] == 1);
    } else if (!std::strcmp(name, "wait")) {
        c.sensorType = 13;
        temperatureAdcValues[0] = 4092;
        c.currentTemperatureC = 0;
        c.setTargetTemperature(25);
        injection = Injection::Reach;
        c.waitForTargetTemperature();
        assert(visits == 3 && watchdogHeartbeat == now);
        assert(std::fabs(c.currentTemperatureC - 25) <= TEMP_TOLERANCE && HAL::outputDuty(0) == 0);
    } else if (!std::strncmp(name, "autotune-", 9)) {
        const bool useBed = std::strstr(name, "-bed-") != nullptr;
        tuned = useBed ? &bed : &c;
        auto& t = *tuned;
        for (auto* other : tempController) other->setTargetTemperature(0);
        t.setTargetTemperature(100); // must not restart normal heating after autotune
        HAL::setHeaterDuty(t.pwmIndex, 255);
        t.sensorType = 60;
        temperatureAdcValues[t.sensorPin] = 330;
        float target = 100;
        bool store = false;
        const float oldP = t.pidPGain, oldI = t.pidIGain, oldD = t.pidDGain;
        const bool entry = std::strstr(name, "entry") != nullptr;
        if (!std::strcmp(name, "autotune-entry-unsupported")) t.sensorType = 255;
        else if (!std::strcmp(name, "autotune-entry-low")) {
            t.sensorType = 14;
            temperatureAdcValues[t.sensorPin] = 4092;
        } else if (!std::strcmp(name, "autotune-entry-disabled")) t.sensorType = 0;
        else if (!std::strcmp(name, "autotune-entry-latched")) Printer::fault = true;
        else if (!std::strcmp(name, "autotune-entry-dryrun")) Printer::debugLevel = 8;
        else if (!std::strcmp(name, "autotune-entry-high")) temperatureAdcValues[t.sensorPin] = 820;
        else if (!std::strcmp(name, "autotune-entry-target")) target = 500;
        else if (!std::strcmp(name, "autotune-entry-controller")) {}
        else if (std::strstr(name, "-late")) {
            injection = Injection::LateUnsupported;
            postTemperature = false; // fault between periodic temperature services
        } else if (!std::strcmp(name, "autotune-low")) {
            injection = Injection::LateLow;
            postTemperature = false;
        } else if (!std::strcmp(name, "autotune-transient")) injection = Injection::Transient;
        else if (!std::strcmp(name, "autotune-high")) {
            injection = Injection::High;
            postTemperature = false;
        } else if (!std::strcmp(name, "autotune-bed-cutoff")) {
            target = 130;
            injection = Injection::Cutoff;
            postTemperature = false;
        } else if (std::strstr(name, "-timeout")) stride = 100000;
        else if (std::strstr(name, "-success")) {
            injection = Injection::Cycles;
            stride = 6000;
            store = std::strstr(name, "-store") != nullptr;
        } else if (!std::strcmp(name, "autotune-other-fault")) {
            bed.sensorType = 255;
            for (int i = 0; i < 10; ++i) service();
            assert(!Printer::fault);
            injection = Injection::OtherFault;
        } else assert(false);
        const uint8_t id = !std::strcmp(name, "autotune-entry-controller") ? NUM_TEMPERATURE_LOOPS : t.pwmIndex;
        t.autotunePID(target, id, 5, store, 0);
        assert(autotuneIndex == 255 && HAL::outputDuty(t.pwmIndex) == 0 && t.targetTemperatureC == 0);
        if (entry) assert(visits == 0);
        else assert(visits > 0 && watchdogHeartbeat == now);
        if (std::strstr(name, "-success")) {
            assert(visits == 12 && !Printer::fault);
            assert(stores == unsigned(store));
            if (store) assert(t.pidPGain != oldP && std::isfinite(t.pidPGain) && std::isfinite(t.pidIGain) && std::isfinite(t.pidDGain));
        } else assert(stores == 0);
        if (!store) assert(t.pidPGain == oldP && t.pidIGain == oldI && t.pidDGain == oldD);
        if (injection == Injection::OtherFault) allOff();
        // One subsequent normal service must not re-enable the old target.
        service();
        assert(HAL::outputDuty(t.pwmIndex) == 0);
    } else {
        assert(false);
    }
    std::printf("%s: passed\n", name);
}
'''


def main():
    header = (FW / "Extruder.h").read_text()
    control = (FW / "TemperatureControl.cpp").read_text()
    conversion = (FW / "TemperatureSensors.cpp").read_text()
    hal = (FW / "HAL.cpp").read_text()
    config = (FW / "Configuration.h").read_text()
    commands = (FW / "Commands.cpp").read_text()
    include = '#include "Repetier.h"'
    fragments = {
        "controller.inc": section(header, "#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL", "}; // TemperatureController") + "};",
        "conversion.inc": conversion.split(include, 1)[1],
        "control.inc": control.split(include, 1)[1],
        "scheduler.inc": section(commands, "void Commands::checkForPeriodicalActions(", "bool Commands::waitForMotion("),
        "com.inc": "\n".join("constexpr int " + n + " = 0;" for n in sorted(set(re.findall(r"Com::(t[A-Z]\w*)", control)))),
        "hal.inc": (
            "namespace {\n" + section(hal, "volatile unsigned long watchdogHeartbeat", "HAL::HAL()")
            + section(hal, "uint16_t HAL::temperatureAdcSample(", "void HAL::analogStart(")
            + section(hal, "void HAL::tellWatchdogOk()", "\n}") + "\n}\n"
            + section(hal, "constexpr uint8_t heaterPwmStep", "constexpr uint8_t partFanPwmStep")
            + "static volatile uint8_t requestedDuty[NUM_EXTRUDER + 3];\n"
            + section(hal, "uint8_t HAL::outputDuty(", "void HAL::setBoardFan(")
            + "void pwmSlice() {\n"
            + section(hal, "    static uint8_t pwm_count_heater", "    pwm_count_cooler += coolerPwmStep;")
            + "    pwm_count_cooler += coolerPwmStep;\n}\n"
        ),
    }
    scenarios = (
        "rail", "invalid-low", "invalid-high", "bed-invalid", "recovery", "disabled",
        "unsupported", "no-rise", "pwm", "watchdog", "wait",
        "autotune-entry-unsupported", "autotune-entry-low", "autotune-entry-disabled",
        "autotune-entry-latched", "autotune-entry-dryrun", "autotune-entry-high",
        "autotune-entry-target", "autotune-entry-controller", "autotune-late",
        "autotune-low", "autotune-transient", "autotune-high", "autotune-timeout",
        "autotune-other-fault", "autotune-success", "autotune-success-store",
        "autotune-bed-late", "autotune-bed-timeout", "autotune-bed-cutoff", "autotune-bed-success-store",
    )
    with TemporaryDirectory(prefix="rfx-temperature-control-") as tmp:
        work = Path(tmp)
        for name, data in fragments.items():
            (work / name).write_text(data)
        (work / "replay.cpp").write_text(HARNESS)
        for board, tools in (("RF1000", 1), ("RF2000", 2), ("RF2000", 1)):
            profile = (FW / (board + ".h")).read_text()
            defines = {n: macro(profile, n) for n in (
                "SENSOR_TEMPERATURE_OFFSETS_C", "MIN_DEFECT_TEMPERATURE", "MAX_DEFECT_TEMPERATURE")}
            defines.update({n: macro(config, n) for n in (
                "EXTRUDER_FAN_COOL_TEMP", "DECOUPLING_TEST_MAX_HOLD_VARIANCE",
                "DECOUPLING_TEST_MIN_TEMP_RISE", "EXT0_DECOUPLE_TEST_PERIOD",
                "MAX_ROOM_TEMPERATURE", "WATCHDOG_MAIN_LOOP_TIMEOUT",
                "PID_CONTROL_RANGE", "TEMP_TOLERANCE",
                "PID_CONTROL_DRIVE_MAX_LIMIT_FACTOR", "PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR")})
            defines["ANALOG_REDUCE_BITS"] = macro((FW / "HAL.h").read_text(), "ANALOG_REDUCE_BITS")
            defines["CELSIUS_EXTRA_BITS"] = macro(header, "CELSIUS_EXTRA_BITS")
            defines["NUM_EXTRUDER"] = str(tools)
            exe = work / "replay"
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2",
                            *[f"-D{k}={v}" for k, v in defines.items()],
                            str(work / "replay.cpp"), "-o", str(exe)], check=True)
            for scenario in scenarios:
                subprocess.run([str(exe), scenario], check=True, timeout=10)
            print(f"{board} / {tools} hotend(s): {len(scenarios)} cases passed", flush=True)


if __name__ == "__main__":
    main()
