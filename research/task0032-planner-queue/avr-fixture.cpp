// Compiled only in a temporary AVR fixture with the actual motion/queue headers.
// No physical timers/interrupt sources; runtime settings are explicit fixture inputs.
#include "Repetier.h"
#include "MotionPlanner.h"
#include <limits.h>
#include "estimator-candidate.h"

extern "C" {
volatile uint8_t phase=0, fixture_id=0, queue_depth=0;
volatile float sink=0;
volatile uint8_t runtime_seed=0;
}
static void mark(uint8_t p) {
    asm volatile("" ::: "memory");
    phase=p;
    asm volatile("" ::: "memory");
}

float Printer::maxFeedrate_[4]={200,200,9,25};
float Printer::maxXYJerk_=8, Printer::maxZJerk_=.5;
uint32_t Printer::maxPrintAccelerationStepsPerSquareSecond_[4]={100000,100000,100000,100000};
uint32_t Printer::maxTravelAccelerationStepsPerSquareSecond_[4]={100000,100000,100000,100000};
IdleStatus Printer::idleStatus;
ExtrusionOutput Printer::extrusionOutput;
static Extruder extruder;
Extruder* Extruder::current=&extruder;

static float fullSeconds(const PrintLine& m,uint32_t total) {
    const auto& p=m.stepProfile;
    const uint32_t a=p.accelSteps<total?p.accelSteps:total;
    const uint32_t d=p.decelSteps<total-a?p.decelSteps:total-a;
    float peak=RMath::min(float(p.vMax),31250.0f);
    if(a) peak=RMath::min(peak,sqrtf(float(p.vStart)*p.vStart+2.0f*m.lookahead.accelerationPrim*a));
    peak=RMath::max(1.0f,peak);
    float s=(total-a-d)/peak;
    if(a) s+=2.0f*a/(RMath::min(float(p.vStart),peak)+peak);
    if(d) s+=2.0f*d/(RMath::min(float(p.vEnd),peak)+peak);
    return s;
}
enum class Estimate:uint8_t { ExactTimeOnly, Lower, PacedTicks, FastRoot };
template<Estimate method>
float estimate(const MoveQueue& q) {
    uint8_t index,entries;
    int32_t front;
    {
        InterruptProtectedBlock lock;
        index=q.readIndex_;entries=q.count_;
        front=q.active_?q.active_->stepsRemaining:-1;
    }
    float result=0;
    uint32_t ticks=0;
    for(uint8_t n=0;n<entries;++n,MoveQueue::nextIndex(index)) {
        const auto& m=q.slots_[index];
        if(m.flags&(FLAG_WARMUP|FLAG_QUEUE_TASK))continue;
        const int32_t total=m.delta[m.primaryAxis];
        const int32_t left=n==0&&front>=0?front:total;
        if(left<=0||total<=0)continue;
        if(method==Estimate::ExactTimeOnly)
            result+=left==total?fullSeconds(m,total):MoveQueue::estimateSeconds(m,total,left);
        else if(method==Estimate::FastRoot) {
            const auto& p=m.stepProfile;
            result+=PlannerEstimateResearch::time<true>(total,left,p.accelSteps,p.decelSteps,m.lookahead.accelerationPrim,p.vStart,p.vEnd,p.vMax);
        } else if(method==Estimate::Lower)
            result+=float(left)/RMath::max(1.0f,RMath::min(float(m.stepProfile.vMax),31250.0f));
        else {
            if(m.timeInTicks<=0)continue; // invalid tick fields are excluded; candidate must expose invalid in production
            const uint32_t t=m.timeInTicks;
            if(left!=total)result+=float(t)*left/total;
            else ticks=UINT32_MAX-ticks<t?UINT32_MAX:ticks+t;
        }
    }
    return method==Estimate::PacedTicks?(float(ticks)+result)/F_CPU:result;
}
#ifndef ESTIMATOR_ONLY
#define ESTIMATOR_ONLY 0
#endif
static void measure(const MoveQueue& q) {
    if(ESTIMATOR_ONLY==0||ESTIMATOR_ONLY==7){
        mark(7);sink=q.remainingMotion().seconds;mark(135);
    }
    if(ESTIMATOR_ONLY==0||ESTIMATOR_ONLY==8){
        mark(8);sink=estimate<Estimate::ExactTimeOnly>(q);mark(136);
    }
    if(ESTIMATOR_ONLY==0||ESTIMATOR_ONLY==9){
        mark(9);sink=estimate<Estimate::Lower>(q);mark(137);
    }
    if(ESTIMATOR_ONLY==0||ESTIMATOR_ONLY==11){
        mark(11);sink=estimate<Estimate::FastRoot>(q);mark(139);
    }
    if(ESTIMATOR_ONLY==0||ESTIMATOR_ONLY==10){
        mark(10);sink=estimate<Estimate::PacedTicks>(q);mark(138);
    }
}

int main() {
    sei();
    auto& q=PrintLine::queue;
    // Volatile loads prevent isolated whole-program optimization from treating
    // all configured settings as immutable constants.
    Printer::maxFeedrate_[0]=200+runtime_seed;
    Printer::maxFeedrate_[1]=200+runtime_seed;
    Printer::maxXYJerk_=8+runtime_seed;
    extruder.maxEJerk=6+runtime_seed;
    for(uint8_t f=0;f<8;++f) {
        fixture_id=f;
        q.clear();
        Printer::extrusionOutput.advanceEnabled_=f==6;
        extruder.advanceL=f==6?100.0f:float(runtime_seed);
        for(uint8_t a=0;a<4;++a) {
            Printer::maxPrintAccelerationStepsPerSquareSecond_[a]=100000+runtime_seed;
            Printer::maxTravelAccelerationStepsPerSquareSecond_[a]=100000+runtime_seed;
        }
        if(f==7) {
            Printer::maxTravelAccelerationStepsPerSquareSecond_[0]=240000;
            Printer::maxTravelAccelerationStepsPerSquareSecond_[1]=165000;
        }
        for(uint8_t n=0;n<MOVE_CACHE_SIZE;++n) {
            queue_depth=n;
            PrintLine& m=q.writeSlot();
            m=PrintLine{};
            int32_t steps[4]={0,0,0,0};
            float requested=20+40*(n%3)+runtime_seed;
            switch(f) {
                case 0:steps[0]=4;steps[3]=n%3==0?1:0;break;
                case 1:steps[0]=6;steps[1]=n%2?5:-5;steps[3]=1;break;
                case 2:steps[0]=16;steps[3]=1;break;
                case 3:steps[n%2]=32;steps[3]=3;break;
                case 4:steps[0]=64;steps[1]=48;steps[3]=7;break;
                case 5:steps[0]=16000;requested=200;break;
                case 6:if(n%3==0)steps[3]=-280;else {steps[0]=16;steps[3]=n%3==1?1:0;}break;
                case 7:steps[0]=38400;steps[1]=32000;requested=100;break;
            }
            const float units[4]={1.0f/160,1.0f/160,1.0f/1280,1.0f/280};
            float distances[4];
            for(uint8_t a=0;a<4;++a) {
                m.setAxisSteps(a,steps[a]);
                distances[a]=float(steps[a]<0?-steps[a]:steps[a])*units[a];
            }
            m.setSegmentGeometry(distances);
            mark(1);MotionPlanner::prepareMove(m,distances,requested,q);mark(129);
            mark(2);MotionPlanner::updateLookahead(q);mark(130);
            q.publish(true);
            if(n==0) {
                auto* active=q.activateFront();
                active->fixStartAndEndSpeed();
                if(active->stepsRemaining>1)active->stepsRemaining/=2;
            }
            queue_depth=q.count();
            measure(q);
        }
    }
    mark(255);
    while(true){}
}
