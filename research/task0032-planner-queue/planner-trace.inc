// Included in the existing pause-sense-offset replay translation unit.
// Actual planner, queue, and nextStepSpeed; release and batch clocks modeled.
// No UART, foreground CPU, pause scheduling, or physical STEP pin simulation.
#include "estimator-candidate.h"
#include "duration-ledger.h"
namespace PlannerResearch {
using U64 = unsigned long long;
struct Meta { float requested=0, cap=0; unsigned id=0; };
struct Work { double x=0,y=0,z=0,e=0; float requested=0, cap=0; };
struct Trace {
    MoveQueue queue;
    Meta meta[MOVE_CACHE_SIZE];
    PlannerEstimateResearch::DurationLedger<MOVE_CACHE_SIZE> ledger;
    uint32_t pendingDuration=UINT32_MAX;
    unsigned durationBuilds=0, durationReplans=0;
    static Trace* owner;
    static float seconds(const PrintLine& m, uint32_t left) {
        const auto& p=m.stepProfile;
        return PlannerEstimateResearch::time<false>(m.delta[m.primaryAxis],left,p.accelSteps,p.decelSteps,m.lookahead.accelerationPrim,p.vStart,p.vEnd,p.vMax);
    }
    static void rebuilt(PrintLine& m) {
        auto& t=*owner;
        const uint8_t index=&m-&t.queue.slot(0);
        const uint32_t units=PlannerEstimateResearch::durationUnits(seconds(m,m.delta[m.primaryAxis]));
        ++t.durationBuilds;
        if(index==t.queue.writeIndex()) t.pendingDuration=units;
        else {t.ledger.replace(index,units);++t.durationReplans;}
    }
    const char* name;
    unsigned serial, group, capMode;
    bool advance;
    float chord, speed;
    U64 now=0, event=0, idle=0;
    uint32_t interval=10000;
    uint8_t batch=1;
    unsigned admitted=0, retired=0;
    StepExecutor::StepClock clock;
    bool finishing=false;
    int32_t previous[4]={};
    double xyz=0;
    Trace(const char* n, float c, float s, unsigned baud, unsigned g, unsigned cm, bool a)
      : name(n), serial(baud), group(g), capMode(cm), advance(a), chord(c), speed(s) {
        assert(!owner);owner=this;researchProfileHook=rebuilt;
    }
    ~Trace(){researchProfileHook=nullptr;owner=nullptr;}
    void prefix(const char* kind) const {
        std::printf("{\"event\":\"%s\",\"workload\":\"%s\",\"capacity\":%u,\"chord\":%.8g,\"requested\":%.8g,\"baud\":%u,\"group\":%u,\"cap_mode\":%u,\"advance\":%u,\"ticks\":%llu,\"admitted\":%u,\"retired\":%u",
                    kind,name,unsigned(MOVE_CACHE_SIZE),chord,speed,serial,group,capMode,unsigned(advance),now,admitted,retired);
    }
    void snapshot(const char* kind) {
        const auto ref=queue.remainingMotion();
        assert(interruptsEnabled && std::isfinite(ref.seconds));
        const auto ls=ledger.snapshot(queue.readIndex(),queue.count()!=0);
        const auto* active=queue.active();
        const uint32_t tail=active && !(active->flags&(FLAG_QUEUE_TASK|FLAG_WARMUP)) ?
            PlannerEstimateResearch::durationUnits(seconds(*active,active->stepsRemaining)):0;
        uint64_t units=0;
        assert(ledger.remaining(ls,active!=nullptr,tail,units));
        const double ledgerSeconds=double(units)/10000;
        // Exact analytic profile reference, not physical duration. Allow
        // per-slot rounding plus binary32 reference arithmetic.
        assert(std::abs(ledgerSeconds-ref.seconds)<=.0000501*(queue.count()+1)+std::abs(ref.seconds)*.000003+1e-7);
        if(!queue.count())assert(ls.full==0 && ls.unavailable==0);
        prefix(kind);
        std::printf(",\"ledger_s\":%.9g,\"ledger_full_units\":%llu,\"ledger_builds\":%u,\"ledger_replans\":%u",ledgerSeconds,(unsigned long long)ls.full,durationBuilds,durationReplans);
        std::printf(",\"read\":%u,\"write\":%u,\"count\":%u,\"active\":%u,\"reference_s\":%.9g,\"xyz_mm\":%.9g,\"filament_mm\":%.9g",
                    unsigned(queue.readIndex()),unsigned(queue.writeIndex()),unsigned(queue.count()),unsigned(queue.active()!=nullptr),ref.seconds,ref.xyzMm,ref.filamentMm);
        if (!std::strcmp(kind,"publish")) {
            std::printf(",\"planning\":{\"first\":%u,\"limit\":%u,\"reserve_ticks\":%u,\"prepare\":%u,\"profile\":%u,\"profile_rebuild\":%u,\"junction\":%u,\"decel\":%u,\"accel\":%u,\"update\":%u,\"back_visits\":%u,\"forward_visits\":%u,\"reserve_visits\":%u,\"fixed_search\":%u,\"no_window\":%u,\"extrusion_boundary\":%u,\"axis_limit\":%u,\"wide_divide\":%u,\"back_roots\":%u,\"forward_roots\":%u}",
                        pc.first,pc.limit,pc.reserveTicks,pc.prepare,pc.profile,pc.profileRebuild,pc.junction,pc.decel,pc.accel,pc.update,pc.backVisits,pc.forwardVisits,pc.reserveVisits,pc.fixedSearch,pc.noWindow,pc.extrusionBoundary,pc.axisLimit,pc.wideDivide,pc.backRoots,pc.forwardRoots);
        }
        std::printf(",\"slots\":[");
        uint8_t index=queue.readIndex();
        double lower=0, contiguous=0, nominal=0, paced=0, fast=0, fastContiguous=0;
        bool barrier=false;
        for(unsigned n=0;n<queue.count();++n,MoveQueue::nextIndex(index)) {
            PrintLine& m=queue.slot(index);
            if(n) std::printf(",");
            const bool special=(m.flags & (FLAG_WARMUP|FLAG_QUEUE_TASK))!=0;
            const bool active=n==0 && queue.active()!=nullptr;
            const int32_t total=special?0:m.delta[m.primaryAxis];
            const int32_t left=special?0:active?m.stepsRemaining:total;
            const bool ready=m.isStepProfileReady(), blocked=m.isBlocked();
            barrier=barrier||special||blocked||!ready;
            std::printf("{\"slot\":%u,\"id\":%u,\"active\":%u,\"special\":%u,\"flags\":%u,\"join_flags\":%u,\"fixed_exit\":%u,\"ready\":%u,\"blocked\":%u,\"total\":%ld,\"left\":%ld",
                        unsigned(index),meta[index].id,unsigned(active),unsigned(special),unsigned(m.flags),unsigned(m.joinFlags),unsigned(m.isEndSpeedFixed()),unsigned(ready),unsigned(blocked),long(total),long(left));
            if(!special) {
                const auto& l=m.lookahead;
                const auto& p=m.stepProfile;
                const double fraction=total>0?double(left)/total:0;
                const double floorS=left>0?double(left)/std::max(1.0,std::min(double(p.vMax),31250.0)):0;
                lower+=floorS;
                const float fastS=left>0&&total>0?PlannerEstimateResearch::time<true>(total,left,p.accelSteps,p.decelSteps,l.accelerationPrim,p.vStart,p.vEnd,p.vMax):0;
                fast+=fastS;
                if(!barrier)fastContiguous+=fastS;
                if(!barrier) contiguous+=floorS;
                if(meta[index].requested>0) nominal+=l.distance*fraction/meta[index].requested;
                paced+=double(m.timeInTicks)*fraction/F_CPU;
                std::printf(",\"distance\":%.9g,\"requested\":%.9g,\"cap\":%.9g,\"paced_ticks\":%ld,\"full\":%.9g,\"entry\":%.9g,\"exit\":%.9g,\"junction\":%.9g,\"min_speed\":%.9g,\"primary\":%u,\"acceleration\":%lu,\"accel_distance2\":%.9g,\"vStart\":%u,\"vEnd\":%u,\"vMax\":%u,\"accelSteps\":%lu,\"decelSteps\":%lu,\"advanceL\":%lu,\"steps\":[",
                            l.distance,meta[index].requested,meta[index].cap,long(m.timeInTicks),m.fullSpeed,l.startSpeed,l.endSpeed,l.maxJunctionSpeed,l.minSpeed,unsigned(m.primaryAxis),(unsigned long)l.accelerationPrim,l.accelerationDistance2,unsigned(p.vStart),unsigned(p.vEnd),unsigned(p.vMax),(unsigned long)p.accelSteps,(unsigned long)p.decelSteps,(unsigned long)p.advanceL);
                for(unsigned axis=0;axis<4;++axis) {
                    if(axis) std::printf(",");
                    std::printf("%ld",long(m.isNegativeMoveOfAxis(axis)?-m.delta[axis]:m.delta[axis]));
                }
                std::printf("],\"fast_s\":%.9g",fastS);
            }
            std::printf("}");
        }
        std::printf("],\"nominal_s\":%.9g,\"paced_proxy_s\":%.9g,\"lower_s\":%.9g,\"contiguous_lower_s\":%.9g,\"fast_s\":%.9g,\"fast_contiguous_s\":%.9g}\n",nominal,paced,lower,contiguous,fast,fastContiguous);
    }
    U64 arrival(unsigned i, unsigned prefill) const {
        if(i<prefill) return 0;
        const U64 k=i-prefill;
        const U64 wire=(U64(F_CPU)*40*10+serial-1)/serial;
        const U64 mean=serial==250000?F_CPU/500:wire;
        return k/group*group*mean+(k%group+1)*wire+(group>1&&k>=48?F_CPU*8ULL/100:0)+(group==32&&k>=32?F_CPU*2ULL:0);
    }
    void admit(const Work& w) {
        assert(queue.count()<MOVE_CACHE_SIZE);
        const uint8_t index=queue.writeIndex();
        PrintLine& m=queue.writeSlot();
        m=PrintLine{};
        const double target[4]={w.x,w.y,w.z,w.e};
        float distances[4];
        for(unsigned a=0;a<4;++a) {
            const int32_t t=std::lround(target[a]*Printer::stepsPerMm(a));
            const int32_t steps=t-previous[a];
            previous[a]=t;
            m.setAxisSteps(a,steps);
            distances[a]=std::abs(steps)*Printer::mmPerStep(a);
        }
        m.setSegmentGeometry(distances);
        assert(m.stepsRemaining>0);
        xyz+=std::sqrt(distances[0]*distances[0]+distances[1]*distances[1]+distances[2]*distances[2]);
        meta[index].id=admitted+1;
        meta[index].requested=w.requested;
        meta[index].cap=w.cap;
        pc=PathCounts{};
        pendingDuration=UINT32_MAX;
        MotionPlanner::prepareMove(m,distances,std::min(w.requested,w.cap),queue);
        MotionPlanner::updateLookahead(queue);
        assert(m.isStepProfileReady()&&!m.isBlocked());
        assert(pendingDuration!=UINT32_MAX);
        ledger.publish(index,pendingDuration);
        queue.publish(true);
        ++admitted;
        snapshot("publish");
    }
    void start() {
        PrintLine& m=*queue.activateFront();
        assert(m.isStepProfileReady()&&!m.isBlocked()&&m.stepsRemaining>0&&m.stepProfile.vStart);
        m.fixStartAndEndSpeed();
        clock.start(m.stepProfile.vStart);
        finishing=false;
        event=now+std::max(uint32_t(1),interval>>1);
        snapshot("activate");
    }
    void step() {
        PrintLine& m=*queue.active();
        if(finishing) {
            assert(m.stepsRemaining==0);
            prefix("retire");
            std::printf(",\"id\":%u}\n",meta[queue.readIndex()].id);
            ledger.retire(queue.readIndex());
            queue.retireFront();
            HAL::allowInterrupts();
            ++retired;
            return;
        }
        const uint8_t count=std::min<int32_t>(batch,m.stepsRemaining);
        m.stepsRemaining-=count;
        const speed_t v=StepExecutor::nextStepSpeed(m,clock,count,interval);
        assert(v);
        interval=std::max<uint32_t>(512,F_CPU/v);
        batch=(STEP_PACKING_MIN_INTERVAL+interval-1)/interval;
        const uint32_t packed=interval*batch;
        finishing=m.stepsRemaining==0;
        event=now+(finishing?std::max(uint32_t(1),packed>>1):packed);
    }
    void run(const Work* work,unsigned size,unsigned prefill) {
        Printer::extrusionOutput.setAdvanceEnabled(advance);
        Extruder::current->advanceL=advance?100.0f:0.0f;
        prefix("begin");
        std::printf(",\"step_per_mm\":[%.9g,%.9g,%.9g,%.9g],\"max_feed\":[%.9g,%.9g,%.9g,%.9g],\"xy_jerk\":%.9g,\"z_jerk\":%.9g,\"e_jerk\":%.9g}\n",
                    Printer::stepsPerMm(0),Printer::stepsPerMm(1),Printer::stepsPerMm(2),Printer::stepsPerMm(3),Printer::axisMaxFeedrate(0),Printer::axisMaxFeedrate(1),Printer::axisMaxFeedrate(2),Printer::axisMaxFeedrate(3),Printer::xyJerkLimit(),Printer::zJerkLimit(),Extruder::current->maxEJerk);
        for(unsigned i=0;i<prefill;++i) admit(work[i]);
        while(retired<size) {
            if(queue.active()&&event==now) step();
            while(admitted<size&&queue.count()<MOVE_CACHE_SIZE&&arrival(admitted,prefill)<=now) admit(work[admitted]);
            if(!queue.active()&&queue.count()) start();
            if(retired==size) break;
            const U64 input=admitted<size&&queue.count()<MOVE_CACHE_SIZE?arrival(admitted,prefill):~U64(0);
            const U64 next=std::min(input,queue.active()?event:~U64(0));
            assert(next>now&&next!=~U64(0));
            if(!queue.active()) idle+=next-now;
            now=next;
        }
        assert(admitted==size&&retired==size&&queue.count()==0&&interruptsEnabled);
        prefix("end");
        std::printf(",\"idle_ticks\":%llu,\"xyz_mm\":%.9g}\n",idle,xyz);
    }
};
Trace* Trace::owner=nullptr;

void circle(float chord,float speed,unsigned baud,unsigned group,bool advance,unsigned prefill=12) {
    constexpr unsigned size=96;
    Work moves[size];
    constexpr double r=1.35;
    const double angle=2*std::asin(chord/(2*r));
    for(unsigned i=0;i<size;++i) {
        moves[i].x=r*(std::cos((i+1)*angle)-1);
        moves[i].y=r*std::sin((i+1)*angle);
        moves[i].e=chord*(i+1)*.05;
        moves[i].requested=moves[i].cap=speed;
    }
    Trace t("circle",chord,speed,baud,group,0,advance);
    t.run(moves,size,prefill);
}
void transition(float chord,float speed,unsigned mode,unsigned group,bool advance) {
    constexpr unsigned straight=6, curve=96, size=straight+curve+3;
    Work moves[size];
    double x=0,y=0,e=0,heading=0;
    for(unsigned i=0;i<size;++i) {
        const bool dense=i>=straight&&i<straight+curve;
        const double len=dense?chord:3.0;
        if(dense) heading+=2*std::asin(chord/2.7);
        x+=len*std::cos(heading); y+=len*std::sin(heading); e+=len*.05;
        moves[i].x=x; moves[i].y=y; moves[i].e=e;
        moves[i].requested=speed;
        // External script, not a governor: compare cap on curve only vs last
        // two approach moves also capped. Never apply this to isolated travel.
        moves[i].cap=mode&&(dense||(mode==2&&i>=straight-2&&i<straight))?std::min(speed,chord*500):speed;
    }
    Trace t("transition",chord,speed,250000,group,mode,advance);
    t.run(moves,size,12);
}
void mixed(bool advance) {
    Work w[10];
    double x=0,y=0,z=0,e=0;
    for(unsigned i=0;i<10;++i) {
        if(i==2) e-=1;
        else if(i==3) e+=1;
        else if(i==4) z+=.2;
        else if(i==5) x+=50;
        else if(i==6) x+=.025; // deliberate quantized zero E
        else if(i==7) {x+=.025;e+=.0002;}
        else if(i==8) {x+=.025;e+=.005;}
        else {x+=1;y+=.2;e+=.05;}
        w[i].x=x; w[i].y=y; w[i].z=z; w[i].e=e;
        w[i].requested=w[i].cap=i==5?Printer::axisMaxFeedrate(X_AXIS):60;
    }
    Trace t("mixed",.025,60,250000,1,0,advance);
    t.run(w,10,6);
}
void isolatedTravel() {
    Work w[1]; w[0].x=100; w[0].requested=w[0].cap=Printer::axisMaxFeedrate(X_AXIS);
    Trace t("isolated_travel",100,w[0].requested,250000,1,0,false);
    t.run(w,1,1);
}
void barriers() {
    Trace t("barriers",.2,60,250000,1,0,false);
    Work w; w.x=1;w.requested=w.cap=60;
    t.admit(w);
    auto& first=*t.queue.activateFront();
    first.stepsRemaining/=2;
    first.fixStartAndEndSpeed();
    w.x=2;t.admit(w);
    t.queue.slot(1).block();
    t.snapshot("blocked_front_successor");
    t.queue.slot(1).unblock();
    t.queue.writeSlot()=PrintLine{};
    t.ledger.publish(t.queue.writeIndex(),0);
    t.queue.publishTask(QueueTask::EnableZCompensation);
    t.snapshot("task_barrier");
    t.queue.writeSlot()=PrintLine{};
    t.queue.writeSlot().flags=FLAG_WARMUP;
    t.ledger.publish(t.queue.writeIndex(),0);
    t.queue.publish(false);
    t.snapshot("warmup_barrier");
    t.ledger.clear();
    t.queue.clear();
    t.snapshot("cleared");
}
} // namespace PlannerResearch

void plannerResearch() {
    using namespace PlannerResearch;
    const char* env=std::getenv("PLANNER_SCENARIO");
    const bool smoke=env&&!std::strcmp(env,"smoke");
    if(smoke) circle(.1,60,250000,1,false);
    else if(env&&!std::strcmp(env,"stress")) {
        for(float c:{.5f,.2f,.1f,.05f,.025f}) {
            circle(c,100,250000,32,false,1);
            circle(c,100,250000,32,true,1);
        }
        for(float c:{.1f,.05f,.025f})for(unsigned mode:{0U,1U,2U})transition(c,100,mode,32,false);
    } else {
        for(float c:{.5f,.2f,.1f,.05f,.025f})
            for(float s:{20.f,60.f,100.f}) {
                circle(c,s,115200,1,false);
                circle(c,s,250000,1,false);
                circle(c,s,250000,16,false);
            }
        for(float c:{.1f,.05f,.025f})
            for(unsigned mode:{0U,1U,2U}) {
                transition(c,100,mode,1,false);
                transition(c,100,mode,16,true);
            }
    }
    mixed(false);mixed(true);isolatedTravel();barriers();
}
