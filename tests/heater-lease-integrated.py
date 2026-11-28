#!/usr/bin/env python3
"""Replay the actual standard HAL lease path with abstract AVR register semantics."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
hal=(root/"Repetier/HAL.cpp").read_text()
start=hal.index("// Non-release candidate: /256")
end=hal.index("\n} // namespace",start)
state=hal[start:end]
# Prove the fallback is the first executable action in both actual ISR bodies.
for vector in ("TIMER1_COMPA_vect", "PWM_TIMER_VECTOR"):
    body=hal.split("ISR("+vector+") {",1)[1]
    lines=[line.strip() for line in body.splitlines() if line.strip() and not line.strip().startswith("//")]
    assert lines[0]=="expirePendingHeaterLease();", vector
advance=hal.split("ISR(EXTRUDER_TIMER_VECTOR, __attribute__((flatten))) {",1)[1]
assert next(line.strip() for line in advance.splitlines() if line.strip() and not line.strip().startswith("//"))=="if (pendingHeaterExpiry()) cutExpiredHeaterLease();"
start=hal.index('constexpr uint8_t thermalPassComplete=0x80;')
feed=hal[start:hal.index('\nISR(TIMER3_COMPA_vect)',start)]
start=hal.index("// Distinguish pre-start boot")
progress=hal[start:hal.index("// Separate bytes allow each ISR",start)]
start=hal.index('void HAL::stopWatchdog()')
heartbeat=hal[start:hal.index('\n// ================== Interrupt handling',start)]
start=hal.index("// Timer3 is independent")
adc=hal[start:hal.index("inline __attribute__((always_inline)) void serviceTemperatureAdc()",start)]
start=hal.index("HAL::HeaterSample::~HeaterSample")
ticket=hal[start:hal.index("\nvoid HAL::analogStart(",start)]
header=(root/"Repetier/HAL.h").read_text()
start=header.index("    class HeaterSample {")
ticket_decl=header[start:header.index("    static bool heaterLeaseFault();",start)]
start=hal.index("void HAL::setHeaterDuty(")
setter=hal[start:hal.index("\nvoid HAL::setBoardFan",start)]
reset=hal[hal.index("void HAL::resetHardware()"):hal.index("} // resetHardware")+1]
start=hal.index("    if (pwm_count_heater == 0) {",hal.index("ISR(PWM_TIMER_VECTOR)"))
frame=hal[start:hal.index("    if (pwm_count_cooler == 0)",start)]
prelude=r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
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
static bool irq=true,pins[4];
static uint8_t PWM_TIMSK;
#define PWM_OCIE 2
#define WATCHDOG_PIN 3
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
#define READ(pin) (pins[pin])
static volatile uint8_t requestedDuty[NUM_EXTRUDER+3];
static constexpr uint8_t heaterPwmMask=0xfe;
static uint8_t pwm_pos_set[NUM_EXTRUDER+3];
constexpr uint8_t TemperatureAdcCount=NUM_EXTRUDER+1;
static uint16_t temperatureAdcValues[TemperatureAdcCount]={};
struct HAL {
    // Diagnostic validity is modeled independently of grant freshness.
    static bool temperatureAdcSample(uint8_t c,uint16_t& raw
                                     ,uint32_t* publication=nullptr
    ) {
        if(publication)*publication=0; // fixture models grant identity separately
        raw=temperatureAdcValues[c];return true;
    }
    TICKET_DECL
    static void forbidInterrupts(){irq=false;}
    static void resetHardware();
    static void completeThermalWatchdogPass(bool heatDemand=false);
    static void completeAutotuneWatchdogPass();
    static void stopWatchdog();
    static uint32_t timeInMilliseconds(){return simulatedMs;}
    static uint32_t simulatedMs;
};
uint32_t HAL::simulatedMs=0;
static bool watchdogActive=true;
static bool watchdogPinOutput=true;
#define SET_INPUT(pin) do {assert(pin==WATCHDOG_PIN);watchdogPinOutput=false;} while(0)
static unsigned watchdogFeeds;
static uint8_t TIMSK5,TCCR5A,TCCR5B,TCCR5C;
static uint16_t TCNT5,OCR5A;
static W1C TIFR5;
#define OCF5A 1
#define CS52 2
static uint32_t heaterResetCookie;
constexpr uint32_t heaterLiveCookie=0xd39a6c51UL;
#define wdt_reset() do {assert(!irq);++watchdogFeeds;} while(0)
namespace Printer {
static bool blocked=false;
bool heatingInhibited(){return blocked;}
static bool defect=false;
bool isAnyTempsensorDefect(){return defect;}
}
"""
main=r"""
static unsigned pin(unsigned c){return c==NUM_EXTRUDER?2:c;}
static void resetFixture(uint16_t start){
    for(auto& d:requestedDuty)d=0;
    for(auto& p:pins)p=false;
    heaterLeaseReady=false;heaterLeaseFailed=false;
    heaterGrantProgress=0;watchdogActive=true;watchdogStarted=true;
    watchdogPinOutput=true;watchdogFeeds=0;heaterResetCookie=0;
    beginWatchdogProgress();
    HAL::simulatedMs=0;
    heaterBatchValid=false;heaterBatchPrevious=false;
    for(auto& s:heaterSampleState)s=0;
    Printer::blocked=false;irq=true;
    Printer::defect=false;
    beginHeaterLease();TCNT3=start;
}
static void advance(uint32_t ticks){
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
        if(ticks>uint16_t(leaseDeadline(c)-TCNT3))TIFR3.bits|=leaseMask(c);
    if(ticks>=65536u-TCNT3)TIFR3.bits|=_BV(TOV3);
    TCNT3=uint16_t(TCNT3+ticks);
}
// Keep the Timer5 clock frozen during isolated Timer3/ADC replay cases.
// Advance both hardware clocks explicitly in the staged-watchdog cases.
static void advanceProgress(uint32_t ticks){
    if(ticks>=uint16_t(OCR5A-TCNT5))TIFR5.bits|=_BV(OCF5A);
    TCNT5=uint16_t(TCNT5+ticks);
    advance(ticks);
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
static void checkPriorityIndependentExpiry() {
    // The actual leaf called at the head of Timer1 and PWM. Exhaust all
    // register combinations, including inactive stale flags, overflow
    // and unused compare C on single-tool hardware.
    constexpr unsigned supported = (_BV(OCIE3A)|_BV(OCIE3B)
#if NUM_EXTRUDER > 1
                                   |_BV(OCIE3C)
#endif
                                   );
    for (bool inlinePath:{false,true})
    for (unsigned flags=0;flags<256;++flags) for(unsigned mask=0;mask<256;++mask) {
        resetFixture(0);
        InterruptProtectedBlock lock;
        TIMSK3=uint8_t(mask); TIFR3.bits=uint8_t(flags);
        heaterGrantProgress=0xff;
        for (uint8_t c=0;c<=NUM_EXTRUDER;++c) {
            requestedDuty[c]=255; pins[pin(c)]=true;
        }
        if (inlinePath) { if (pendingHeaterExpiry()) cutExpiredHeaterLease(); }
        else expirePendingHeaterLease();
        const bool expired=(flags&mask&supported)!=0;
        assert(heaterLeaseFailed==expired);
        assert(TIFR3.bits==flags); // no W1C, even on failure
        assert(TIMSK3==(expired?0:mask));
        assert(heaterGrantProgress==(expired?0:0xff));
        for (uint8_t c=0;c<=NUM_EXTRUDER;++c) {
            assert(requestedDuty[c]==(expired?0:255));
            assert(pins[pin(c)]!=expired);
        }
        // A second visit with the original sticky flag cannot rearm.
        expirePendingHeaterLease();
        assert(heaterLeaseFailed==expired);
    }
    std::puts("priority-independent expiry: 131072 inline/shared-leaf flag/mask cases PASS");
}
// Exhaust the modular window and compare-flag states for each fixed channel.
static bool fixedLive(unsigned c,uint16_t now) {
    switch(c) {
    case 0:return fixedLeaseLive<0>(now);
    case 1:return fixedLeaseLive<1>(now);
#if NUM_EXTRUDER > 1
    case 2:return fixedLeaseLive<2>(now);
#endif
    default:assert(false);return false;
    }
}
static void checkFixedChannelEquivalence() {
    resetFixture(65530);
    InterruptProtectedBlock lock;
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c) {
        for(uint32_t left=0;left<65536;++left) {
            writeLeaseDeadline(c,uint16_t(TCNT3+left));
            for(uint8_t flag=0;flag<2;++flag) {
                TIFR3.bits=flag?leaseMask(c):0;
                const bool expected=left>heaterLeaseGuard && left<=heaterLeaseDuration && !flag;
                assert(fixedLive(c,TCNT3)==expected);
                assert(leaseLive(c,TCNT3)==expected);
            }
        }
    }
    const unsigned combinations=1u<<(NUM_EXTRUDER+1);
    // Each subset of active, armed and expired channels, including no demand.
    for(unsigned active=0;active<combinations;++active)
        for(unsigned armed=0;armed<combinations;++armed)
            for(unsigned expired=0;expired<combinations;++expired) {
                heaterLeaseFailed=false;
                TIMSK3=uint8_t(armed<<OCIE3A);
                TIFR3.bits=uint8_t(expired<<OCF3A);
                for(uint8_t c=0;c<=NUM_EXTRUDER;++c) {
                    requestedDuty[c]=(active&(1u<<c))?255:0;
                    writeLeaseDeadline(c,uint16_t(TCNT3+heaterLeaseDuration));
                }
                assert(heaterClockContinuity()==bool(active&armed&~expired));
                const bool good=!(active&expired);
                assert(checkHeaterLeases()==good);
                assert(heaterLeaseFailed!=good);
                if(!good) {
                    assert(TIMSK3==0);
                    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
                        assert(requestedDuty[c]==0 && !pins[pin(c)]);
                }
            }
}
static uint8_t oldWrapState(uint8_t s,bool continuous) {
    if(!continuous) s &= SampleBusy;
    else {
        if(s&SamplePrevious)
            s &= uint8_t(~(SampleValid|SampleFresh|SampleMatch|SamplePrevious));
        else if(s&SampleValid) s |= SamplePrevious;
        if(s&ReservationPrevious)
            s &= uint8_t(~(SampleReserved|SampleMatch|ReservationPrevious));
        else if(s&SampleReserved) s |= ReservationPrevious;
    }
    return s;
}
static void checkSampleWrapEquivalence() {
    for(bool continuous:{false,true})for(unsigned state=0;state<256;++state) {
        uint8_t before[TemperatureAdcCount];
        for(uint8_t c=0;c<TemperatureAdcCount;++c) {
            before[c]=uint8_t(state+61*c);
            heaterSampleState[c]=before[c];
        }
        if(continuous) HeaterSampleWrap<true,TemperatureAdcCount>::apply();
        else HeaterSampleWrap<false,TemperatureAdcCount>::apply();
        for(uint8_t c=0;c<TemperatureAdcCount;++c)
            assert(heaterSampleState[c]==oldWrapState(before[c],continuous));
    }
}
static void checkZeroRevocation(){
    unsigned long cases=0;
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
        for(unsigned duty=0;duty<256;++duty)
            for(unsigned phase=0;phase<256;phase+=2)
                for(bool enabled:{false,true}){
                    resetFixture(0);
                    for(uint8_t other=0;other<=NUM_EXTRUDER;++other)
                        grant(other,other==c?uint8_t(duty):254);
                    // Force the stale latched pin and duty independently of
                    // the requested PWM value. Zero must clear it immediately.
                    pins[pin(c)]=true;pwm_pos_set[c]=uint8_t(duty&254);
                    for(uint8_t other=0;other<=NUM_EXTRUDER;++other)
                        if(other!=c)pins[pin(other)]=true;
                    irq=enabled;
                    HAL::setHeaterDuty(c,0);
                    assert(irq==enabled&&!requestedDuty[c]&&!pins[pin(c)]&&!(TIMSK3&leaseMask(c)));
                    for(uint8_t other=0;other<=NUM_EXTRUDER;++other)
                        if(other!=c)assert(pins[pin(other)]&&requestedDuty[other]==254);
                    startFrame(uint8_t(phase));startFrame(0);
                    assert(!pins[pin(c)]);
                    HAL::setHeaterDuty(c,128);
                    assert(!requestedDuty[c]&&!pins[pin(c)]); // no unbound grant
                    grant(c,128);
                    assert(requestedDuty[c]==128&&!pins[pin(c)]&&irq==enabled);
                    startFrame(0);assert(pins[pin(c)]);
                    ++cases;
                }
    resetFixture(0);
    for(unsigned channel=NUM_EXTRUDER+1;channel<256;++channel){
        HAL::setHeaterDuty(uint8_t(channel),0);
        HAL::setHeaterDuty(uint8_t(channel),255);
        assert(TIMSK3==0&&!heaterLeaseFailed);
    }
    std::printf("zero revoke: %lu duty/phase/IRQ cases; invalid channels rejected\\n",cases);
}
struct Restart {};
static void restart(){
    assert(!irq&&!(PWM_TIMSK&_BV(PWM_OCIE))&&TIMSK3==0&&TCCR3B==0);
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
        assert(!requestedDuty[c]&&!pins[pin(c)]);
    throw Restart{};
}
static void (*resetFunc)()=restart;
RESET_SOURCE
static void checkPreWatchdogReset(){
    for(bool enabled:{false,true}){
        resetFixture(0);
        // Model the pre-watchdog fallback only. An armed runtime never
        // jumps to zero; its intentionally non-returning wait is audited
        // at the linked instruction and recovery-boot level.
        for(uint8_t c=0;c<=NUM_EXTRUDER;++c){grant(c,255);pins[pin(c)]=true;}
        watchdogStarted=false;watchdogActive=false;irq=enabled;PWM_TIMSK=255;
        try{HAL::resetHardware();assert(false);}catch(const Restart&){}
        assert(!irq&&PWM_TIMSK==uint8_t(255&~_BV(PWM_OCIE)));
        startFrame(0);
        for(uint8_t c=0;c<=NUM_EXTRUDER;++c)assert(!pins[pin(c)]);
    }
    std::puts("pre-watchdog reset: two saved-I cases, PWM and Timer3 disabled");
}
int main(){
    checkZeroRevocation();
    checkPreWatchdogReset();
    checkSampleWrapEquivalence();
    checkPriorityIndependentExpiry();
    checkFixedChannelEquivalence();
    // Independently of duty-byte consistency, a frame without positive
    // lease authorization must never drive a rising heater edge.
    for(unsigned mode=0;mode<2;++mode) {
        resetFixture(0);
        for(uint8_t c=0;c<=NUM_EXTRUDER;++c) {
            requestedDuty[c]=255;
            pwm_pos_set[c]=255;
            writeLeaseDeadline(c,heaterLeaseDuration);
        }
        heaterLeaseReady=mode!=0;
        heaterLeaseFailed=mode!=0;
        startFrame(0);
        for(uint8_t c=0;c<=NUM_EXTRUDER;++c)
            assert(!pins[pin(c)]&&pwm_pos_set[c]==0);
        assert(irq);
    }
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
    resetFixture(0);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==1);
    HAL::simulatedMs=100;
    assert(watchdogFeeds==1&&OCR5A==watchdogProgressDuration); // elapsed logical time alone cannot feed
    grant(0,255);startFrame(0);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==2&&heaterResetCookie==heaterLiveCookie);
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
    // Phase/guard and sticky compare replay on the actual Timer5 functions.
    // A successful new thermal pass can renew only a still-live permission.
    for(uint32_t phase=0;phase<65536;++phase){
        for(uint32_t elapsed:{0u,1u,12495u,12496u,12500u,12501u,65536u,131072u}){
            resetFixture(0);
            TCNT5=uint16_t(phase);
            OCR5A=uint16_t(phase+watchdogProgressDuration);
            advanceProgress(elapsed);
            const bool live=elapsed<watchdogProgressDuration-watchdogProgressGuard;
            assert(watchdogProgressLive()==live);
            const uint16_t previous=OCR5A;
            HAL::completeThermalWatchdogPass();
            assert(watchdogFeeds==unsigned(live));
            if(live) assert(OCR5A==uint16_t(TCNT5+watchdogProgressDuration));
            else {
                assert(OCR5A==previous);
                grant(0,255);assert(!requestedDuty[0]&&heaterLeaseFailed);
                if(elapsed>=watchdogProgressDuration)
                    assert(TIFR5.bits&_BV(OCF5A));
            }
        }
    }
    resetFixture(0);
    watchdogActive=false;
    startFrame(0);assert(!heaterLeaseFailed); // pre-start PWM boot
    watchdogActive=true;
    grant(0,255);startFrame(0);assert(pins[0]);
    HAL::completeThermalWatchdogPass(true);assert(watchdogFeeds==1);
    HAL::setHeaterDuty(0,0);
    advanceProgress(6250);
    HAL::completeThermalWatchdogPass(true);
    assert(watchdogFeeds==2&&heaterResetCookie==heaterLiveCookie);
    advanceProgress(6250);
    HAL::completeThermalWatchdogPass(false);
    assert(watchdogFeeds==3&&heaterResetCookie==0);
    advanceProgress(12501);startFrame(0);
    assert(heaterLeaseFailed&&!pins[0]);
    HAL::completeThermalWatchdogPass(false);assert(watchdogFeeds==3);
    // A regular pass without a new autotune grant waits. The sampled
    // autotune decision may complete that pass, but never supply a new
    // regular pass by itself.
    resetFixture(0);grant(0,255);
    HAL::completeThermalWatchdogPass(true);assert(watchdogFeeds==1);
    advanceProgress(6250);
    HAL::completeThermalWatchdogPass(true);assert(watchdogFeeds==1);
    grant(0,255);HAL::completeAutotuneWatchdogPass();assert(watchdogFeeds==2);
    advanceProgress(6250);
    grant(0,255);HAL::completeAutotuneWatchdogPass();assert(watchdogFeeds==2);
    HAL::completeThermalWatchdogPass(true);assert(watchdogFeeds==3);
    HAL::completeAutotuneWatchdogPass();assert(watchdogFeeds==3);
    // Repeated runtime stop must not float the external supervisor or
    // leave any heat power enabled. Only a new boot may float WDI.
    resetFixture(0);grant(0,255);startFrame(0);
    pins[WATCHDOG_PIN]=true;
    HAL::stopWatchdog();HAL::stopWatchdog();
    assert(watchdogStarted&&!watchdogActive&&watchdogPinOutput&&pins[WATCHDOG_PIN]);
    assert(!pins[0]&&!requestedDuty[0]);
    HAL::completeThermalWatchdogPass(true);assert(watchdogFeeds==0);
    resetFixture(0);watchdogStarted=false;watchdogActive=false;irq=false;
    HAL::stopWatchdog();assert(!watchdogPinOutput&&!pins[WATCHDOG_PIN]);
    irq=true;
    // A terminal fault can retain diagnostics only on a regular pass
    // with no requested heat, no live lease and every output LOW.
    resetFixture(0);grant(0,255);startFrame(0);
    Printer::defect=true;
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==0);
    HAL::setHeaterDuty(0,0);
    for(uint8_t c=0;c<=NUM_EXTRUDER;++c){
        pins[pin(c)]=true;
        HAL::completeThermalWatchdogPass();assert(watchdogFeeds==0);
        pins[pin(c)]=false;
        TIMSK3=leaseMask(c);
        OCR3A=OCR3B=OCR3C=uint16_t(TCNT3+12500);
        HAL::completeThermalWatchdogPass();assert(watchdogFeeds==0);
        TIMSK3=0;
    }
    HAL::completeAutotuneWatchdogPass();assert(watchdogFeeds==0);
    HAL::completeThermalWatchdogPass();assert(watchdogFeeds==1);
    std::puts("emergency watchdog all-off proof passed");
    std::puts("thermal watchdog progress checks passed; 524288 sticky Timer5 deadline cases");
    std::printf("heaters=%u actual HAL grant/expiry cases=%lu; exact sample checks passed\n",NUM_EXTRUDER+1,cases);
}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"lease.cpp";exe=Path(temp)/"lease"
    cpp.write_text(prelude.replace("TICKET_DECL",ticket_decl)+adc+"\n"+progress+"\n"+state+"\n"+ticket+"\n"+setter+"\n"+feed+"\n"+heartbeat+"\n"+
                   "\nstatic void startFrame(uint8_t pwm_count_heater){InterruptProtectedBlock lock;\n"+frame+"\n}\n"+main.replace("RESET_SOURCE",reset))
    for n in (1,2):
        subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror","-O2",
                        f"-DNUM_EXTRUDER={n}",str(cpp),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
