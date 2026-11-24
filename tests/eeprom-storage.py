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
    body = source[source.index("constexpr uint8_t eepromMagic"):source.index(r"/** \brief Writes all eeprom settings")]
    body += source[source.index("uint8_t EEPROM::computeChecksum()"):source.index("void EEPROM::writeExtruderPrefix")]
    body += preheat[preheat.index("namespace {"):]
    with tempfile.TemporaryDirectory(prefix="eeprom-storage-") as temp:
        work = pathlib.Path(temp)
        (work / "storage-source.inc").write_text(body, encoding="utf-8")
        for model, expected in GOLDEN.items():
            flags = ["-DMOTHERBOARD=DEVICE_TYPE_" + model.split("-")[0]]
            if model.endswith("-X44"):
                flags += ["-DNUM_EXTRUDER=1", "-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"]
            exe = work / "replay"
            subprocess.run(["c++", "-std=c++17", "-I", str(FW), "-I", str(work),
                            *flags, str(ROOT / "tests" / "eeprom-storage.cpp"),
                            "-o", str(exe)], check=True)
            output = subprocess.check_output([str(exe)])
            rows = output.decode("ascii").splitlines()
            assert len(rows) == 8 and all(len(bytes.fromhex(row.split()[1])) == 2048 for row in rows)
            digest = hashlib.sha256(output).hexdigest()
            assert digest == expected, (model, expected, digest)
            print(f"{model}: eight EEPROM snapshots match baseline ({digest})")


if __name__ == "__main__":
    main()
