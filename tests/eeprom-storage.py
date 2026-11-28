#!/usr/bin/env python3
"""Exercise the standard zoned storage, real owners and microstep admission."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
FW = ROOT / "Repetier"


def main():
    preheat = (FW / "PreheatProfiles.cpp").read_text(encoding="utf-8")
    hal = (FW / "HAL.h").read_text(encoding="utf-8")
    ui = (FW / "UiSettings.cpp").read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="eeprom-storage-") as temp:
        work = pathlib.Path(temp)
        owner = preheat[preheat.index("namespace {"):preheat.index("#ifndef RF_EEPROM_ZONE_EXPERIMENT")]
        (work / "preheat-owner.inc").write_text(owner, encoding="utf-8")
        live = (FW / "EepromZoneLive.cpp").read_text(encoding="utf-8")
        (work / "live-settings.inc").write_text(live.replace('#include "Repetier.h"\n', "", 1).replace('#include "StepExecutor.h"\n', "", 1), encoding="utf-8")
        runtime = (FW / "EepromZoneRuntime.cpp").read_text(encoding="utf-8")
        def section(first, last):
            start = runtime.index(first)
            return runtime[start:runtime.index(last, start)]
        (work / "runtime-menu.inc").write_text(
            section("bool idle() {", "bool persist(const Patch*")
            + section("bool EEPROM::settingsReady()", "void EEPROM::init()")
            + section("bool EEPROM::storePressureLimits(", "void EEPROM::storeMicrostepConfiguration("),
            encoding="utf-8")
        zone_exe = work / "zones"
        subprocess.run(["c++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I", str(FW), str(ROOT / "tests" / "eeprom-zones.cpp"),
                        "-o", str(zone_exe)], check=True)
        subprocess.run([str(zone_exe)], check=True)
        setters = hal[hal.index("    static inline void eprSetByte("):hal.index("    // Faster version of InterruptProtectedBlock.")]
        (work / "hal-eeprom.inc").write_text(setters, encoding="utf-8")
        micro = ui[ui.index("    case UI_ACTION_MICROSTEPS_XY:"):ui.index("\n    }\n}\n\nvoid UIDisplay::changeMotionSetting")]
        (work / "microstep-ui.inc").write_text(micro, encoding="utf-8")
        persist = section("void EEPROM::storeMicrostepConfiguration(", "void EEPROM::storeCaseLight(")
        (work / "microstep-persist.inc").write_text(persist, encoding="utf-8")
        for tools in (1, 2):
            exe = work / "write-paths"
            subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra",
                            "-Wno-int-to-pointer-cast", "-I", str(FW), "-I", str(work),
                            f"-DNUM_EXTRUDER={tools}", str(ROOT / "tests" / "eeprom-write-paths.cpp"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
        for model in ('RF1000','RF2000','RF2000v2','RF2000-X44'):
            flags = ["-DMOTHERBOARD=DEVICE_TYPE_" + model.split("-")[0]]
            if model.endswith("-X44"):
                flags += ["-DNUM_EXTRUDER=1", "-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"]
            default_exe = work / "zone-defaults"
            subprocess.run(["c++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-I", str(FW), "-I", str(work), *flags, str(ROOT / "tests" / "eeprom-zone-defaults.cpp"),
                            "-o", str(default_exe)], check=True)
            subprocess.run([str(default_exe)], check=True)
            exe = work / "replay"
            subprocess.run(["c++", "-std=c++17", "-I", str(FW), "-I", str(work),
                            *flags, str(ROOT / "tests" / "eeprom-storage.cpp"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(f"{model}: zoned factory, session and actual owner roundtrip PASS",flush=True)


if __name__ == "__main__":
    main()
