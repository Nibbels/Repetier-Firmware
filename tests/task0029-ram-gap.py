"""Exercise the actual allocation-free SRAM-gap routine with and without an allocator.

Host weak symbol resolution models the optional AVR linker symbol. This is
not a hardware stack-watermark test. The linked AVR images must independently
prove that malloc/free and the allocator globals are no longer retained.
"""
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

root = Path(__file__).resolve().parents[1]
hal = (root / "Repetier" / "HAL.cpp").read_text()
start = hal.index('extern "C" {\nextern char __heap_start;')
end = hal.index("} // getFreeRam", start) + 1
routine = hal[start:end]
harness = r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
extern "C" { char __heap_start; }
static uintptr_t SP;
static bool interrupts = true;
struct InterruptProtectedBlock {
    bool saved;
    InterruptProtectedBlock() : saved(interrupts) { interrupts = false; }
    ~InterruptProtectedBlock() { interrupts = saved; }
};
struct HAL { static int getFreeRam(); };
""" + routine + r"""
#ifdef WITH_ALLOCATOR
extern "C" { char* __brkval = nullptr; }
#endif
int main() {
    const auto base = reinterpret_cast<uintptr_t>(&__heap_start);
    for (bool initial : {false, true}) {
        interrupts = initial;
        for (unsigned gap : {0u, 1u, 32u, 2079u, 8192u}) {
            SP = base + gap;
            assert(HAL::getFreeRam() == int(gap));
            assert(interrupts == initial);
        }
#ifdef WITH_ALLOCATOR
        __brkval = reinterpret_cast<char*>(base + 128);
        SP = base + 1024;
        assert(HAL::getFreeRam() == 896 && interrupts == initial);
        SP = base + 100;
        assert(HAL::getFreeRam() == -28 && interrupts == initial);
        __brkval = nullptr;
#endif
    }
    std::puts("Allocation-free SRAM gap: passed");
}
"""
# AVR pointers and int are 16-bit; host addresses may be wide. The routine's
# signed integer subtraction is valid for the ATmega2560's SRAM address range.
# Use a non-PIE executable to keep test addresses in the host int range.
with TemporaryDirectory() as tmp:
    src = Path(tmp) / "gap.cpp"
    exe = Path(tmp) / "gap"
    src.write_text("#include <initializer_list>\n" + harness)
    for flags in ([], ["-DWITH_ALLOCATOR"]):
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++11", "-O2", "-Wall",
                        "-Wextra", "-Werror", "-no-pie", *flags, str(src), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
