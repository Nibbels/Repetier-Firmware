#!/usr/bin/env python3
"""Exercise real accessory output ownership on each configured model."""
import pathlib
import subprocess
import tempfile

SRC = pathlib.Path(__file__).resolve().parents[1] / "Repetier"
def owner(path):
    s = path.read_text()
    end = "} // namespace AccessoryOutputs"
    return s[s.index("namespace AccessoryOutputs {"):s.index(end)+len(end)]

prefix = r"""
#include <cassert>
#include <cstdint>
#include <vector>
#include <utility>
#include "Constants.h"
#include "Configuration.h"
#if MOTHERBOARD == DEVICE_TYPE_RF1000
#include "RF1000.h"
#else
#include "RF2000.h"
#endif
#define EXT0_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#define EXT1_HOTEND_PROFILE DEFAULT_HOTEND_PROFILE
#define EXT0_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#define EXT1_TEMPSENSOR_TYPE DEFAULT_HOTEND_SENSOR_TYPE
#define __AVR_ATmega2560__ 1
#include "pins.h"
using Event = std::pair<int,int>;
std::vector<Event> actual;
#define WRITE(pin,val) actual.emplace_back(pin, !!(val))
#define SET_OUTPUT(pin) actual.emplace_back(pin, 2)
"""
body = r"""
using namespace AccessoryOutputs;
int main() {
    std::vector<Event> expected;
    assert(state(Output::CaseLight) == CASE_LIGHTS_DEFAULT_ON);
    stage(Output::CaseLight, 1);
#if MOTHERBOARD != DEVICE_TYPE_RF1000
    stage(Output::Mains, 1);
    stage(Output::Fet1, 1);
    stage(Output::Fet2, 0);
    stage(Output::Fet3, 1);
    expected.emplace_back(OUTPUT_230V_PIN,2);
    expected.emplace_back(OUTPUT_230V_PIN,0);
#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
    expected.emplace_back(FET1,2);
    expected.emplace_back(FET1,1);
#endif
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
    expected.emplace_back(FET2,2);
    expected.emplace_back(FET2,0);
#endif
#if FET3 > -1 && CASE_FAN_PIN < 0
    expected.emplace_back(FET3,2);
    expected.emplace_back(FET3,1);
#endif
#endif
    assert(actual.empty());
    initializeBoardOutputs();
#if MOTHERBOARD != DEVICE_TYPE_RF1000
    assert(state(Output::Mains)==0);
#endif
    initializeCaseLight();
#if !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    expected.emplace_back(CASE_LIGHT_PIN,2);
    expected.emplace_back(CASE_LIGHT_PIN,1);
#endif
    assert(actual==expected);
    actual.clear(); expected.clear();
    stage(Output::CaseLight, static_cast<char>(255));
    assert(static_cast<uint8_t>(state(Output::CaseLight))==255);
    assert(actual.empty());
    toggle(Output::CaseLight);
    assert(state(Output::CaseLight)==0);
#if !HOTEND_COOLER_USES_PIN(CASE_LIGHT_PIN)
    expected.emplace_back(CASE_LIGHT_PIN,0);
#endif
#if MOTHERBOARD != DEVICE_TYPE_RF1000
    set(Output::Mains,1);
    expected.emplace_back(OUTPUT_230V_PIN,1);
    set(Output::Fet1,0);
    set(Output::Fet2,1);
    set(Output::Fet3,0);
#if FET1 > -1 && !HOTEND_COOLER_USES_PIN(FET1)
    expected.emplace_back(FET1,0);
#endif
#if FET2 > -1 && !HOTEND_COOLER_USES_PIN(FET2)
    expected.emplace_back(FET2,1);
#endif
#if FET3 > -1
    expected.emplace_back(FET3,0);
#endif
#endif
    assert(actual==expected);
}
"""
with tempfile.TemporaryDirectory() as tmp:
    source = pathlib.Path(tmp)/"replay.cpp"
    source.write_text(prefix+owner(SRC/"RF.h")+"\n"+owner(SRC/"RF.cpp")+body)
    for model, flags in [
        ("RF1000", ["-DMOTHERBOARD=DEVICE_TYPE_RF1000"]),
        ("RF2000", ["-DMOTHERBOARD=DEVICE_TYPE_RF2000"]),
        ("RF2000v2", ["-DMOTHERBOARD=DEVICE_TYPE_RF2000v2"]),
        ("RF2000-X44", ["-DMOTHERBOARD=DEVICE_TYPE_RF2000", "-DNUM_EXTRUDER=1", "-DEXT0_EXTRUDER_COOLER_PIN=X44_PIN"]),
    ]:
        executable = pathlib.Path(tmp)/model
        subprocess.run(["g++","-std=c++11","-I",str(SRC),*flags,str(source),"-o",str(executable)],check=True)
        subprocess.run([str(executable)],check=True)
        print(model+": accessory GPIO replay passed")
