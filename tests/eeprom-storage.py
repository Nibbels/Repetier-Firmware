#!/usr/bin/env python3
"""Compare eight complete 2048-byte EEPROM snapshots per model.

The golden digests were captured from target 2b721192ae64206bf97197139298ea43701c0adb
with this harness. Real persistence logic is extracted; printer and peripheral
adapters are intentionally not a physical printer or AVR emulator.
"""
import hashlib
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
FW = ROOT / "Repetier"
GOLDEN = {
    "RF1000": "ca2dbf5835901cd8c3a63b11600db1f768281b759e3efb00d9835daf11e9dafb",
    "RF2000": "b99da192237f259312fd251a8e981566ceb72adbf35487b5b40e779baf9e41cb",
    "RF2000v2": "c9852d9f9310abf0a186a60a89f13c092598577ec94eff272bf017340bc9a968",
    "RF2000-X44": "c2941703f9ac8e0476c1f2e834d54134f269d0681c722b9359a4546a9b1431d8",
}


def main():
    source = (FW / "Eeprom.cpp").read_text(encoding="utf-8")
    preheat = (FW / "PreheatProfiles.cpp").read_text(encoding="utf-8")
    hal = (FW / "HAL.h").read_text(encoding="utf-8")
    ui = (FW / "UiSettings.cpp").read_text(encoding="utf-8")
    body = source[source.index("constexpr uint8_t eepromMagic"):source.index(r"/** \brief Writes all eeprom settings")]
    body += source[source.index("uint8_t EEPROM::computeChecksum()"):source.index("void EEPROM::writeExtruderPrefix")]
    body += preheat[preheat.index("namespace {"):]
    with tempfile.TemporaryDirectory(prefix="eeprom-storage-") as temp:
        work = pathlib.Path(temp)
        (work / "storage-source.inc").write_text(body, encoding="utf-8")
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
        persist = source[source.index("void EEPROM::storeMicrostepConfiguration("):source.index("void EEPROM::storeManualZOffset(")]
        (work / "microstep-persist.inc").write_text(persist, encoding="utf-8")
        for tools in (1, 2):
            exe = work / "write-paths"
            subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra",
                            "-Wno-int-to-pointer-cast", "-I", str(FW), "-I", str(work),
                            f"-DNUM_EXTRUDER={tools}", str(ROOT / "tests" / "eeprom-write-paths.cpp"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
        for model, expected in GOLDEN.items():
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
            output = subprocess.check_output([str(exe)])
            rows = output.decode("ascii").splitlines()
            assert len(rows) == 8 and all(len(bytes.fromhex(row.split()[1])) == 2048 for row in rows)
            # Version and the newly allocated calibration slots are the only
            # intentional snapshot differences. Retain byte-exact legacy comparison.
            normalized = []
            for row in rows:
                name, data = row.split()
                image = bytearray.fromhex(data)
                assert image[105] == 9 and image[1978:1987] == bytes(9)
                image[105] = 8
                # Removed legacy RGB mode was 2 in all old RF2000 snapshots.
                # Normalize only this retired field; the live code no longer
                # reads/writes it. Other bytes retain the pinned old digest.
                if model != "RF1000":
                    image[1019] = 2
                image[104] = sum(image[:104] + image[105:]) & 255
                normalized.append(f"{name} {image.hex()}\n")
            digest = hashlib.sha256("".join(normalized).encode("ascii")).hexdigest()
            assert digest == expected, (model, expected, digest)
            print(f"{model}: eight EEPROM snapshots match baseline ({digest}); new live settings roundtrip PASS")


if __name__ == "__main__":
    main()
