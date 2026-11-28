#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include "Constants.h"
enum {Z_MIN_PIN,Z_MAX_PIN,Z_DIR_PIN,Z_STEP_PIN};
constexpr bool INVERT_Z_DIR=false,LOW=false,HIGH=true;
constexpr float Z_ENDSTOP_DRIVE_OVER=0.25f;
#define INLINE inline
static bool irq=true,dir=true,high=false;
static int32_t physical=0,upper=100000;
static uint64_t pulses=0,dirChanges=0;
struct InterruptProtectedBlock {
    bool old=irq;
    InterruptProtectedBlock(){irq=false;}
    ~InterruptProtectedBlock(){irq=old;}
    void unprotect(){irq=old;}
};
bool READ(int pin) {
    if(pin==Z_DIR_PIN)return dir;
    if(pin==Z_MIN_PIN)return physical>0;
    assert(pin==Z_MAX_PIN);return physical<upper;
}
void WRITE(int pin,bool value) {
    assert(!irq); // physical DIR/STEP writes must be inside the edge transaction
    if(pin==Z_DIR_PIN){assert(!high);dirChanges+=dir!=value;dir=value;return;}
    assert(pin==Z_STEP_PIN);
    if(value){assert(!high);physical+=dir?1:-1;++pulses;}
    high=value;
}
namespace HAL {
void allowInterrupts(){irq=true;}
void forbidInterrupts(){irq=false;}
}
#include "ZMotion.h"
#include "actual-z-motion.inc"
using Path=ZMotion::Path;
static ZMotion z(100.0f);
static int32_t N,D,C,T,S;
static uint64_t cancelled=0,nominal=0,correction=0,scan=0;
static uint32_t rng=0x9e3779b9;
static uint32_t random32(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static void reset(int32_t at){
    z.configureTravel(100,1000);z.setHomed(false);z.setMotorReference(at);z.setNominalSteps(at);
    z.resetDirectPosition();z.clearScanAndCorrection();z.setHomed(true);
    N=physical=at;D=C=T=S=0;high=false;dir=true;irq=true;
}
static void target(int32_t t){T=t;z.requestCorrection(t);}
static void ledger(){
    assert(!high&&z.motorSteps()==physical);
    assert(z.nominalSteps()==N&&z.directPosition().steps()==D&&z.scanSteps()==S);
    assert(physical==N+D+C+S);
}
static bool path(Path p,int8_t sign){
    InterruptProtectedBlock lock;bool ok=z.stepPath(p,sign);
    assert(!irq);ZMotion::finishPulse();return ok;
}
static void correctionStep(){
    int32_t pending=T-C;const auto before=pulses;
    InterruptProtectedBlock lock;assert(z.stepCorrection()==pending);ZMotion::finishPulse();
    if(pending){C+=pending>0?1:-1;++correction;}
    assert(pulses==before+bool(pending));ledger();
}
static void settle(){while(C!=T)correctionStep();}
static void pathStep(int8_t sign,bool direct){
    const auto before=pulses,oldDir=dirChanges;
    const bool balance=T!=C&&((T>C)!=(sign>0));
    assert(path(direct?Path::Direct:Path::Queued,sign));
    ++nominal;if(direct)D+=sign;else N+=sign;
    if(balance){C-=sign;++cancelled;assert(pulses==before&&dirChanges==oldDir);}
    else assert(pulses==before+1);
    ledger();
}
int main(){
    // Deliberately leave DIR opposite to the logical scan direction, with pending CMP.
    for(int8_t s:{int8_t(-1),int8_t(1)}){
        reset(50000);target(-s);correctionStep();target(20*s);
        assert(dir==(s<0));assert(path(Path::Scan,s));S+=s;++scan;ledger();
        assert(z.correctionTarget()==T&&z.correctionCanMove());
        assert(path(Path::Scan,-s));S-=s;++scan;ledger();
    }
    reset(50000);target(17);settle();const auto origin=physical,nominalOrigin=N;
    for(unsigned i=0;i<10000;++i){
        int steps=1+int(random32()%1280);int8_t sign=random32()&1?1:-1;
        bool direct=(i%7)==0;
        for(int leg=0;leg<2;++leg){
            int8_t s=leg?-sign:sign;
            if(direct)z.requestDirectMove(s*steps);
            for(int j=0;j<steps;++j){
                if(!(random32()%17))target(int32_t(random32()%129)-64);
                if(!(random32()%3))correctionStep();
                pathStep(s,direct);
            }
            if(direct){assert(z.directPosition().remaining()==0);z.stopDirectMove();}
        }
        target(17);settle();assert(N==nominalOrigin&&D==0&&C==17&&physical==origin);ledger();
    }
    reset(100000);target(4);auto before=pulses;
    assert(path(Path::Queued,-1));--N;assert(physical==99999&&pulses==before+1);ledger();
    pathStep(-1,false);assert(physical==99999&&C==1);
    reset(-25);target(-4);z.requestDirectMove(2);assert(path(Path::Direct,1));++D;ledger();
    pathStep(1,true);assert(physical==-24&&C==-1);
    reset(100000);for(int i=0;i<3;i++)assert(path(Path::Queued,1));
    assert(z.nominalSteps()==100003&&physical==100000&&z.hasQueuedOvertravel());
    for(int i=0;i<3;i++)assert(path(Path::Queued,-1));
    assert(z.nominalSteps()==100000&&physical==100000&&!z.hasQueuedOvertravel());
    std::printf("10000 exact hop closures, %llu nominal, %llu correction, %llu cancellations, %llu scan edges, %llu physical pulses\n",
        (unsigned long long)nominal,(unsigned long long)correction,(unsigned long long)cancelled,(unsigned long long)scan,(unsigned long long)pulses);
}
