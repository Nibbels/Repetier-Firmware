#!/usr/bin/env python3
"""Replay the actual opt-in HAL lease path with abstract AVR register semantics."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
hal=(root/"Repetier/HAL.cpp").read_text()
start=hal.index("#ifdef RF_HEATER_LEASE_EXPERIMENT\n// Non-release candidate")
end=hal.index("#endif\n} // namespace",start)+len("#endif")
state=hal[start:end]
start=hal.index('void HAL::completeThermalWatchdogPass()')
feed=hal[start:hal.index('\n#endif',start)]
start=hal.index('void HAL::tellWatchdogOk()')
heartbeat=hal[start:hal.index('\n// ================== Interrupt handling',start)]
start=hal.index("#ifdef RF_HEATER_LEASE_EXPERIMENT\n// Timer3 is independent")
adc=hal[start:hal.index("\n#endif",start)+len("\n#endif")]
start=hal.index("#ifdef RF_HEATER_LEASE_EXPERIMENT\nHAL::HeaterSample::~HeaterSample")
ticket=hal[start:hal.index("\n#endif",start)+len("\n#endif")]
header=(root/"Repetier/HAL.h").read_text()
start=header.index("    class HeaterSample {")
ticket_decl=header[start:header.index("    static bool heaterLeaseFault();",start)]
start=hal.index("void HAL::setHeaterDuty(")
setter=hal[start:hal.index("\nvoid HAL::setBoardFan",start)]
start=hal.index("    if (pwm_count_heater == 0) {",hal.index("ISR(PWM_TIMER_VECTOR)"))
frame=hal[start:hal.index("    if (pwm_count_cooler == 0)",start)]
prelude=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#define RF_HEATER_LEASE_EXPERIMENT 1
#define F_CPU 16000000UL
#define EXT0_HEATER_PIN 0
#define EXT1_HEATER_PIN 1
#define HEATED_BED_HEATER_PIN 2
#define LOW false
#define HIGH true
#define _BV(x) (1u<<(x))
#define OCIE3A 1
#define OCIE3B 2
#define OCIE3C 3
#define OCF3A 1
#define OCF3B 2
#define OCF3C 3
#define TOV3 0
#define CS32 2
static bool irq=true,pins[3];
static uint8_t TIMSK3,TCCR3A,TCCR3B,TCCR3C;
static uint16_t TCNT3,OCR3A,OCR3B,OCR3C;
struct W1C {
    uint8_t bits=0;
    operator uint8_t() const {return bits;}
    void operator=(uint8_t mask){bits &= uint8_t(~mask);}
} TIFR3;
struct InterruptProtectedBlock {
    bool was=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=was;}
};
#define WRITE(pin,level) do {assert(!irq);pins[pin]=level;} while(0)
static volatile uint8_t requestedDuty[NUM_EXTRUDER+3];
static constexpr uint8_t heaterPwmMask=0xfe;
static uint8_t pwm_pos_set[NUM_EXTRUDER+3];
constexpr uint8_t TemperatureAdcCount=NUM_EXTRUDER+1;
static uint16_t temperatureAdcValues[TemperatureAdcCount]={};
struct HAL {
    // Diagnostic validity is modeled independently of grant freshness.
    static bool temperatureAdcSample(uint8_t c,uint16_t& raw) {
        raw=temperatureAdcValues[c];return true;
    }
    TICKET_DECL
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    static void completeThermalWatchdogPass();
    static void tellWatchdogOk();
    static uint32_t timeInMilliseconds(){return simulatedMs;}
    static uint32_t simulatedMs;
#endif
};
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
uint32_t HAL::simulatedMs=0;
static bool watchdogActive=true;
static uint32_t watchdogHeartbeat;
static unsigned watchdogFeeds;
#define wdt_reset() do {assert(!irq);++watchdogFeeds;} while(0)
#endif
namespace Printer {static bool blocked=false;bool heatingInhibited(){return blocked;}}
"""
main=r"""
static unsigned pin(unsigned c){return c==NUM_EXTRUDER?2:c;}
static void resetFixture(uint16_t start){
    for(auto& d:requestedDuty)d=0;
    for(auto& p:pins)p=false;
    heaterLeaseReady=false;heaterLeaseFailed=false;
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    heaterGrantProgress=0;watchdogActive=true;watchdogHeartbeat=0;watchdogFeeds=0;
    HAL::simulatedMs=0;
#endif
    heaterBatchValid=false;heaterBatchPrevious=false;
    for(auto& s:heaterSampleState)s=0;
    Printer::blocked=false;irq=true;
    beginHeaterLease();TCNT3=start;
}
static void advance(uint32_t ticks){
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
        if(ticks>uint16_t(leaseDeadline(c)-TCNT3))TIFR3.bits|=leaseMask(c);
    if(ticks>=65536u-TCNT3)TIFR3.bits|=_BV(TOV3);
    TCNT3=uint16_t(TCNT3+ticks);
}
static void publish(uint8_t c,uint16_t raw=1234){
    InterruptProtectedBlock lock;
    synchronizeHeaterSamples();
    heaterBatchStart=TCNT3;heaterBatchValid=true;heaterBatchPrevious=false;
    publishHeaterSample(c,raw);temperatureAdcValues[c]=raw;
}
static void grant(uint8_t c,uint8_t duty){
    publish(c);
    HAL::HeaterSample sample;uint16_t raw;
    assert(sample.read(c,raw)&&raw==1234);
    HAL::setHeaterDuty(c,duty,&sample);
}
static void expireVector(){InterruptProtectedBlock lock;failHeaterLease();}
int main(){
    unsigned long cases=0;
    for(uint32_t start=0;start<65536;++start)
        for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
            for(uint32_t elapsed:{0u,1u,12495u,12496u,12500u,12501u,65536u}){
                resetFixture(start);
                TIFR3.bits=leaseMask(c); // inactive stale flag
                grant(c,255);
                assert(!pins[pin(c)]&&requestedDuty[c]==255&&(TIMSK3&leaseMask(c)));
                assert(!(TIFR3&leaseMask(c))&&irq);
                startFrame(0);assert(pins[pin(c)]);
                advance(elapsed);
                grant(c,128);
                const bool live=elapsed<12500u-4u;
                assert(heaterLeaseFailed!=live);
                assert(requestedDuty[c]==(live?128:0));
                if(live){
                    HAL::setHeaterDuty(c,0);
                    assert(!pins[pin(c)]&&!(TIMSK3&leaseMask(c)));
                    grant(c,255);startFrame(0);
                    assert(pins[pin(c)]);
                }else{
                    assert(!pins[pin(c)]&&TIMSK3==0);
                    grant(c,255);startFrame(0);
                    assert(!pins[pin(c)]&&requestedDuty[c]==0);
                }
                assert(irq);++cases;
            }
    resetFixture(0);
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)grant(c,255);
    startFrame(0);advance(12501);
    startFrame(0);assert(heaterLeaseFailed); // PWM wins dispatch priority
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)assert(!pins[pin(c)]&&!requestedDuty[c]);
    resetFixture(0);
    grant(0,255);startFrame(0);expireVector();
    assert(!pins[0]&&heaterLeaseFailed&&TIMSK3==0);
    resetFixture(0);Printer::blocked=true;grant(0,255);
    assert(!requestedDuty[0]&&!pins[0]);
    resetFixture(0);heaterLeaseReady=false;HAL::setHeaterDuty(0,255);
    assert(!requestedDuty[0]&&!pins[0]);
    resetFixture(0);grant(0,255);startFrame(0);
    advance(12501);HAL::setHeaterDuty(0,0);
    assert(heaterLeaseFailed&&!pins[0]&&TIMSK3==0);
    // An unbound positive call and a repeated reading cannot renew.
    // A reserved young batch may survive a newer publication exactly once.
    resetFixture(0);HAL::setHeaterDuty(0,255);assert(!requestedDuty[0]);
    publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));HAL::setHeaterDuty(0,255,&a);
     assert(requestedDuty[0]==255);advance(100);
     HAL::setHeaterDuty(0,128,&a);assert(requestedDuty[0]==255&&OCR3A==12500);}
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));HAL::setHeaterDuty(0,128,&a);
     assert(requestedDuty[0]==255&&OCR3A==12500);}
    publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw)&&raw==1234);
     publish(0,2345);advance(10);
     HAL::setHeaterDuty(0,128,&a);assert(requestedDuty[0]==128&&OCR3A==12610);
     HAL::setHeaterDuty(0,255,&a);assert(requestedDuty[0]==128&&OCR3A==12610);}
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw)&&raw==2345);
     HAL::setHeaterDuty(1,128,&a);assert(!requestedDuty[1]);
     advance(10);HAL::setHeaterDuty(0,128,&a);assert(requestedDuty[0]==128&&OCR3A==12620);}
    resetFixture(0);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(6251);
     HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
    resetFixture(65530);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(10);
     HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
    // Every start phase near wrap: a live independent compare is a witness,
    // not a renewal of any other channel's reservation.
    for(unsigned start=65536-6250;start<65536;++start){
        resetFixture(uint16_t(start));grant(1,255);publish(0);
        HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
        const unsigned elapsed=65536-start;
        advance(elapsed);
        HAL::setHeaterDuty(0,255,&a);
        assert(requestedDuty[0]==255&&requestedDuty[1]==255&&!heaterLeaseFailed);
        assert(!(TIFR3.bits&_BV(TOV3)));
    }
    // The same retained sample cannot be rejuvenated by the modular age
    // alias after a second witnessed wrap, even with continuous other heat.
    resetFixture(65530);grant(1,255);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(10);
     synchronizeHeaterSamples();assert(heaterSampleState[0]&ReservationPrevious);
     for(unsigned i=0;i<11;++i){advance(6000);grant(1,255);}
     assert(!(heaterSampleState[0]&SampleReserved));
     HAL::setHeaterDuty(0,255,&a);
     assert(!requestedDuty[0]&&requestedDuty[1]==255&&!heaterLeaseFailed);}
    // A witnessed wrap may preserve a partial ADC batch and its old start.
    resetFixture(65530);grant(1,255);
    heaterBatchStart=TCNT3;heaterBatchValid=true;heaterBatchPrevious=false;
    advance(10);publishHeaterSample(0,1234);temperatureAdcValues[0]=1234;
    assert(heaterSampleState[0]&SamplePrevious);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw)&&raw==1234);
     HAL::setHeaterDuty(0,255,&a);assert(requestedDuty[0]==255);}
    // An expired compare is not a witness. Its sticky flag must survive
    // sample synchronization and cause fail-off, even after many wraps.
    resetFixture(65530);grant(1,255);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(131072);
     synchronizeHeaterSamples();
     assert(!(heaterSampleState[0]&SampleReserved)&&(TIFR3.bits&leaseMask(1)));
     HAL::setHeaterDuty(0,255,&a);assert(heaterLeaseFailed&&TIMSK3==0&&!requestedDuty[0]);}
    // No ABA, even with identical values and arbitrary publication count.
    resetFixture(0);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
     for(unsigned i=0;i<131073;++i)publish(0);
     HAL::HeaterSample nested;assert(nested.read(0,raw));
     HAL::setHeaterDuty(0,255,&nested);assert(!requestedDuty[0]);
     HAL::setHeaterDuty(0,255,&a);assert(requestedDuty[0]==255);
     HAL::setHeaterDuty(0,128,&a);assert(requestedDuty[0]==255);}
    // New publication cannot make old ticket young.
    resetFixture(0);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(6251);
     publish(0);HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
     HAL::setHeaterDuty(0,255,&a);assert(requestedDuty[0]==255);}
    // A newer rail invalidates the outstanding decision, even if a later
    // plausible sample is published before it attempts to grant.
    for(uint16_t rail:{0,4092}){
     resetFixture(0);publish(0);
     {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
      publish(0,rail);publish(0);
      HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
     publish(0,rail);
     {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw)&&raw==rail);
      HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
    }
    resetFixture(0);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(131072);
     publish(0);HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
    resetFixture(0);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
     HAL::setHeaterDuty(0,0);HAL::setHeaterDuty(0,255,&a);
     assert(!requestedDuty[0]);}
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
     HAL::setHeaterDuty(0,255,&a);assert(!requestedDuty[0]);}
    // Abandoning an unused reservation does not burn the batch (autotune).
    resetFixture(0);publish(0);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));}
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));
     HAL::setHeaterDuty(0,255,&a);assert(requestedDuty[0]==255);}
    resetFixture(0);heaterBatchStart=0;heaterBatchValid=true;heaterBatchPrevious=false;advance(6251);
    publishHeaterSample(0,1234);
    {HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));HAL::setHeaterDuty(0,255,&a);
     assert(!requestedDuty[0]);}
#ifdef RF_HEATER_RESET_WDT_EXPERIMENT
    resetFixture(0);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==1);
    HAL::simulatedMs=100;
    HAL::tellWatchdogOk();assert(watchdogFeeds==1&&watchdogHeartbeat==0);
    grant(0,255);startFrame(0);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==2&&watchdogHeartbeat==100);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==2);
    grant(1,255);HAL::completeThermalWatchdogPass();assert(watchdogFeeds==2);
    grant(0,255);HAL::completeThermalWatchdogPass();assert(watchdogFeeds==3);
    grant(0,128);HAL::completeThermalWatchdogPass();assert(watchdogFeeds==3);
    HAL::setHeaterDuty(1,0);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==4);
    advance(12501);
    HAL::completeThermalWatchdogPass(); // fault is all-off, retain diagnostics
    assert(heaterLeaseFailed&&watchdogFeeds==5&&TIMSK3==0);
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)assert(!pins[pin(c)]&&!requestedDuty[c]);
    grant(0,255);assert(!requestedDuty[0]);
    watchdogActive=false;HAL::completeThermalWatchdogPass();assert(watchdogFeeds==5);
    resetFixture(0);grant(0,255);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==1);
    publish(0);HAL::HeaterSample a;uint16_t raw;assert(a.read(0,raw));advance(6251);
    HAL::setHeaterDuty(0,255,&a);HAL::completeThermalWatchdogPass();
    assert(watchdogFeeds==1); // stale reservation cannot count as progress
    std::puts("thermal watchdog progress checks passed");
#endif
    std::printf("heaters=%u actual HAL grant/expiry cases=%lu; exact sample checks passed\n",NUM_EXTRUDER+1,cases);
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"lease.cpp";exe=Path(temp)/"lease"
    cpp.write_text(prelude.replace("TICKET_DECL",ticket_decl)+adc+"\n"+state+"\n"+ticket+"\n"+setter+"\n#ifdef RF_HEATER_RESET_WDT_EXPERIMENT\n"+feed+"\n"+heartbeat+"\n#endif\n"+
                   "\nstatic void startFrame(uint8_t pwm_count_heater){InterruptProtectedBlock lock;\n"+frame+"\n}\n"+main)
    for n in (1,2):
        for flags in ([], ["-DRF_HEATER_RESET_WDT_EXPERIMENT=1"]):
            subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror","-O2",
                            f"-DNUM_EXTRUDER={n}",*flags,str(cpp),"-o",str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
