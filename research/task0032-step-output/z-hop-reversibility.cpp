#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "Constants.h"
enum {Z_MIN_PIN,Z_MAX_PIN,Z_DIR_PIN,Z_STEP_PIN};
constexpr bool INVERT_Z_DIR=false,LOW=false,HIGH=true;
constexpr float Z_ENDSTOP_DRIVE_OVER=0.25f;
constexpr uint16_t XYZ_STEPPER_HIGH_DELAY=100,XYZ_STEPPER_LOW_DELAY=100;
#define INLINE inline

static bool irq=true,dir=true,high=false;
static int32_t physical=0,lower=0,upper=100000;
static uint64_t pulses=0,dirChanges=0;
struct InterruptProtectedBlock {
    bool old=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=old;}
    void unprotect(){irq=old;}
};
bool READ(int pin) {
    if(pin==Z_DIR_PIN)return dir;
    if(pin==Z_MIN_PIN)return physical>lower;
    assert(pin==Z_MAX_PIN);return physical<upper;
}
void WRITE(int pin,bool value) {
    if(pin==Z_DIR_PIN) {
        assert(!high);
        dirChanges+=dir!=value;dir=value;return;
    }
    assert(pin==Z_STEP_PIN);
    if(value){assert(!high);physical+=dir?1:-1;++pulses;}
    high=value;
}
namespace HAL {
void allowInterrupts(){irq=true;}
void forbidInterrupts(){irq=false;}
void delayMicroseconds(int){}
}
#include "ZMotion.h"
#include "actual-z-motion.inc"

using Path=ZMotion::Path;
static ZMotion z(100.0f);
static int32_t N,D,C,T;
static uint64_t cancelled=0,nominal=0,correction=0;
static uint32_t rng=0x9e3779b9;
static uint32_t random32(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static void reset(int32_t at) {
    z.configureTravel(100.0f,1000.0f);
    z.setHomed(false);z.setMotorReference(at);z.setNominalSteps(at);
    z.resetDirectPosition();z.clearScanAndCorrection();z.setHomed(true);
    N=physical=at;D=C=T=0;high=false;dir=true;irq=true;
}
static void target(int32_t t){T=t;z.requestCorrection(t);}
static void ledger(){
    assert(!high&&z.motorSteps()==physical);
    assert(z.nominalSteps()==N&&z.directPosition().steps()==D);
    assert(physical==N+D+C);
    assert(z.correctionCanMove()==(T!=C));
}
static bool nominalRaw(Path p,int8_t sign,bool legacyRestore=true){
    InterruptProtectedBlock lock;
#if EXPLICIT_DIR
    (void)legacyRestore;
    const bool ok=z.stepPath(p,sign);
#else
    if(legacyRestore)ZMotion::setDirection(sign>0);
    const bool ok=z.stepPath(p);
#endif
    assert(!irq);ZMotion::finishPulse();return ok;
}
static void nominalStep(int8_t sign,bool direct=false){
    const auto prior=pulses;
    const auto oldDir=dirChanges;
    const bool balance=T!=C&&((T>C)!=(sign>0));
    assert(nominalRaw(direct?Path::Direct:Path::Queued,sign));
    ++nominal;
    if(direct)D+=sign;else N+=sign;
    if(balance){
        C-=sign;++cancelled;assert(pulses==prior);
#if EXPLICIT_DIR
        assert(dirChanges==oldDir);
#else
        (void)oldDir;
#endif
    }else assert(pulses==prior+1);
    ledger();
}
static void correctionStep(){
    const auto prior=pulses;
    const int32_t pending=T-C;
    InterruptProtectedBlock lock;
    assert(z.stepCorrection()==pending);
    ZMotion::finishPulse();
    if(pending){C+=pending>0?1:-1;++correction;}
    assert(pulses==prior+bool(pending));ledger();
}
static void settle(){while(C!=T)correctionStep();ledger();}
static void counterexample(){
    reset(50000);
    // Nominal negative move already activated; correction changes DIR
    // before its next nominal step. Current whole-move reservation excludes
    // this sequence. It is an intentionally constructed handoff example.
    ZMotion::setDirection(false);
    target(1);correctionStep();
    const auto old=physical;
    assert(nominalRaw(Path::Queued,-1,false));
#if EXPLICIT_DIR
    --N;ledger();assert(physical==old-1);
#else
    assert(physical==old+1&&z.nominalSteps()==N+1);
    std::puts("legacy counterfactual DIR alias reproduced");
#endif
}
static void reversibility(){
    reset(50000);target(17);settle();
    const int32_t origin=physical,nominalOrigin=N;
    for(unsigned i=0;i<10000;i++){
        const int steps=1+int(random32()%1280);
        const int8_t sign=random32()&1?1:-1;
        const bool direct=(i%7)==0;
        for(int leg=0;leg<2;leg++){
            const int8_t s=leg?-sign:sign;
            if(direct)z.requestDirectMove(s*steps);
            for(int j=0;j<steps;j++){
                if(!(random32()%17))target(int32_t(random32()%129)-64);
                if(!(random32()%3))correctionStep();
                nominalStep(s,direct);
            }
            if(direct){assert(z.directPosition().remaining()==0);z.stopDirectMove();}
        }
        // Real sensor/bed changes may legitimately leave a different offset.
        // Return to the *same* target and settle to test exact closure.
        target(17);settle();
        assert(N==nominalOrigin&&D==0&&C==17&&physical==origin);
    }
}
static void limits(){
    reset(100000);target(4);
    // Blocked positive correction must not consume negative retreat.
    const auto old=pulses;
    assert(nominalRaw(Path::Queued,-1));--N;
    assert(physical==99999&&pulses==old+1);
    nominalStep(-1);assert(physical==99999&&C==1);
    reset(-25);target(-4);z.requestDirectMove(2);
    assert(nominalRaw(Path::Direct,1));++D;
    assert(physical==-24);
    nominalStep(1,true);assert(physical==-24&&C==-1);
    // Suppressed upper-limit steps require a separate overtravel ledger.
    reset(100000);
    for(int i=0;i<3;i++)assert(nominalRaw(Path::Queued,1));
    assert(z.nominalSteps()==100003&&physical==100000&&z.hasQueuedOvertravel());
    for(int i=0;i<3;i++)assert(nominalRaw(Path::Queued,-1));
    assert(z.nominalSteps()==100000&&physical==100000&&!z.hasQueuedOvertravel());
}
int main(){
    counterexample();reversibility();limits();
    std::printf("Z hop explicit=%d cycles=10000 nominal=%llu corrections=%llu cancelled=%llu physical_rises=%llu\n",EXPLICIT_DIR,(unsigned long long)nominal,(unsigned long long)correction,(unsigned long long)cancelled,(unsigned long long)pulses);
}
