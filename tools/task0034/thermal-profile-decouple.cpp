// A45: A42 plant replay with existing single-channel decouple state and latch.
// This mirrors the foreground algorithm; it is not an AVR execution test.
// Share the exact A38 calibration, plant, PID and three candidate rules.
// c++ -std=c++11 -O2 -Wall -Wextra -Werror tools/task0034/thermal-profile-holdout.cpp -o /tmp/holdout
// /tmp/holdout 4   (25ms integration); /tmp/holdout 8 (12.5ms)
#define main a38_original_main
#include "thermal-profile-validation.cpp"
#undef main

namespace {
enum class Fault { None, Gain, Leak, Detached };
struct Case {const char* name;Fault fault;double amount;int onset;bool external,loadRelease;};
// onset: 0 cold, 1 after 30s, 2 during hold (120s before off), 3 off.
const Case cases[]={
    {"nominal",Fault::None,0,0,false,false},
    {"load-release",Fault::None,0,0,false,true},
    {"cross-heat",Fault::None,0,3,true,false},
    {"gain-10-cold",Fault::Gain,1.1,0,false,false},
    {"gain-25-cold",Fault::Gain,1.25,0,false,false},
    {"gain-50-cold",Fault::Gain,1.5,0,false,false},
    {"gain-25-rise",Fault::Gain,1.25,1,false,false},
    {"gain-50-rise",Fault::Gain,1.5,1,false,false},
    {"gain-25-hold",Fault::Gain,1.25,2,false,false},
    {"gain-50-hold",Fault::Gain,1.5,2,false,false},
    {"gain-80-hold",Fault::Gain,1.8,2,false,false},
    {"leak-10-off",Fault::Leak,.1,3,false,false},
    {"leak-25-off",Fault::Leak,.25,3,false,false},
    {"leak-50-off",Fault::Leak,.5,3,false,false},
    {"leak-100-off",Fault::Leak,1,3,false,false},
    {"detached-hold",Fault::Detached,0,2,false,false},
};
constexpr unsigned caseCount=sizeof(cases)/sizeof(cases[0]);
// Mirror TemperatureControl.cpp and Extruder.h (period 12000ms, rise 1C,
// hold corridor 20C, hold entry <=19C, >10 failures). One valid sensor and
// one active heater; no other channel contributes to the shared counter.
// Reset on setpoint change, including off. Shadow rules never intervene.
struct ExistingGuard {
    enum Phase { Idle, Full, Hold };
    Phase phase=Idle;
    uint32_t last=0;
    float base=0;
    unsigned errors=0;
    int first=-1;
    int cause=0; // 1 no rise, 2 hold failure, 3 absolute measured cutoff
    void reset(){phase=Idle;}
    void observe(uint32_t now,float t,double target){
        if(first>=0)return;
        if(t>275&&t<=300){first=int(now);cause=3;return;}
        bool error=false;
        if(now-last>12000){
            if(phase==Full){
                error=t-base<1;
                if(!error){last=now;base=t;}
            }else if(phase==Hold){
                error=std::fabs(t-target)>20;
                if(!error)last=now-12000+1000;
            }
        }
        if(error){
            if(++errors>10){first=int(now);cause=phase==Full?1:2;}
        }else if(errors)--errors;
    }
    void decision(uint32_t now,float t,double target,uint8_t duty){
        if(first>=0||target<40){phase=Idle;return;}
        if(phase!=Hold&&target-t>20&&duty){
            if(phase!=Full){phase=Full;last=now;base=t;}
        }else if(phase!=Hold&&std::fabs(t-target)<=19){
            phase=Hold;last=now;base=float(target);
        }
    }
};
struct Result {
    std::array<int,3> hit,cause;
    int faultAt=-1,cutoffAt=-1,bodyLimitAt=-1;
    double bodyPeak=0;
    int existing=-1,existingCause=0;
    bool intervention=false;
};
Result replayHoldout(const Plant& p,const Calibration& c,int j,const Case& scenario,int substeps,bool intervene){
    // Use current firmware's absolute measured cutoff, not its bed setpoint
    // limit. The relay training setpoints remain unchanged.
    const double cutoff=275, target=p.bed?90:200, ambient=scenario.loadRelease?15:25;
    const uint32_t off=p.bed?660000:480000,end=off+(p.bed?180000:120000);
    const uint32_t onset=scenario.onset==0?0:scenario.onset==1?30000:scenario.onset==2?off-120000:off;
    Model m(ambient);Pid pid(c,j);Shadow shadow(c.profile,m.measured());
    uint8_t request=pid.update(m.measured(),target,p.cap,cutoff);
    shadow.decision(0,m.measured(),request);
    ExistingGuard existing;
    existing.decision(0,m.measured(),target,request);
    double detachedSensor=m.sensor;
    Result out;out.faultAt=scenario.fault==Fault::None?-1:int(onset);out.bodyPeak=m.body;
    for(uint32_t now=100;now<=end;now+=100){
        const uint32_t prev=now-100;
        const bool active=prev>=onset;
        double power=p.watts*dutyFraction(request);
        if(active&&scenario.fault==Fault::Gain)power*=scenario.amount;
        if(active&&scenario.fault==Fault::Leak)power=std::max(power,p.watts*scenario.amount);
        const double load=scenario.loadRelease&&prev>=off/3&&prev<off/2?(p.bed?50:5):0;
        const double external=scenario.external&&active&&prev<onset+60000?(p.bed?80:8):0;
        for(int i=0;i<substeps;++i){
            m.step(p,power,ambient,load,external,.1/substeps);
            if(active&&scenario.fault==Fault::Detached)detachedSensor+=(ambient-detachedSensor)*(.1/substeps)/5.;
            else detachedSensor=m.sensor;
            out.bodyPeak=std::max(out.bodyPeak,m.body);
            if(out.bodyLimitAt<0&&m.body>cutoff)out.bodyLimitAt=int(now);
        }
        float t=float(std::round(detachedSensor*10)/10);
        if(scenario.loadRelease||scenario.external)t+=float(int((now/100)%3)-1)*.1f;
        if(out.cutoffAt<0&&t>cutoff)out.cutoffAt=int(now);
        // setTargetTemperature(0) clears phase before the next check.
        if(now==off)existing.reset();
        const double demand=now<off?target:0;
        existing.observe(now,t,demand);
        shadow.observe(now,t);
        request=pid.update(t,demand,p.cap,cutoff);
        if(intervene&&existing.first>=0){request=0;out.intervention=true;}
        existing.decision(now,t,demand,request);
        shadow.decision(now,t,request);
    }
    out.hit=shadow.hit;out.cause=shadow.cause;out.existing=existing.first;out.existingCause=existing.cause;return out;
}
struct Aggregate {
    unsigned traces=0,measuredCutoffs=0,bodyLimits=0;
    unsigned alarms[3]{},prefault[3]{},after[3]{},beforeMeasured[3]{},beforeBody[3]{};
    std::vector<int> delays[3];
    unsigned existingHits=0,existingBeforeFault=0,existingBeforeBody=0,existingCauses[4]{},interventions=0;
    std::vector<int> existingDelay;
    void add(const Result& r){
        ++traces;measuredCutoffs+=r.cutoffAt>=0;bodyLimits+=r.bodyLimitAt>=0;
        interventions+=r.intervention;
        if(r.existing>=0){
            ++existingHits;++existingCauses[r.existingCause];
            if(r.faultAt>=0&&r.existing<r.faultAt)++existingBeforeFault;
            if(r.faultAt>=0&&r.existing>=r.faultAt)existingDelay.push_back(r.existing-r.faultAt);
            if(r.bodyLimitAt>=0&&r.existing<r.bodyLimitAt)++existingBeforeBody;
        }
        for(unsigned i=0;i<3;++i){
            if(r.hit[i]<0)continue;
            ++alarms[i];
            if(r.faultAt<0)continue;
            if(r.hit[i]<r.faultAt){++prefault[i];continue;}
            ++after[i];
            beforeMeasured[i]+=r.cutoffAt>=0&&r.hit[i]<r.cutoffAt;
            beforeBody[i]+=r.bodyLimitAt>=0&&r.hit[i]<r.bodyLimitAt;
            delays[i].push_back(r.hit[i]-r.faultAt);
        }
    }
};
}
int main(int argc,char** argv){
    const int substeps=argc>1?std::atoi(argv[1]):4;
    if(substeps!=4&&substeps!=8)return 2;
    Aggregate totals[2][caseCount],latched[2][caseCount];unsigned calibrated[2]{};
    for(bool bed:{false,true})for(double watts:bed?std::vector<double>{300,450,600}:std::vector<double>{30,60,100})
    for(double mass:bed?std::vector<double>{300,800}:std::vector<double>{2,6,18})
    for(double coupling:bed?std::vector<double>{8,20}:std::vector<double>{4,12})
    for(double loss:bed?std::vector<double>{2,4}:std::vector<double>{.04,.07})
    for(double tau:bed?std::vector<double>{1,5}:std::vector<double>{.15,.75,1.5})for(uint8_t cap:{uint8_t(128),uint8_t(255)}){
        const Plant p{bed,watts,mass,coupling,loss,tau,bed?40.:.75,cap};
        const Calibration c=train(p,substeps);
        if(c.outcome!=3)continue;
        ++calibrated[bed];
        for(int j=bed?2:0;j<(bed?4:2);++j)for(unsigned i=0;i<caseCount;++i){
            totals[bed][i].add(replayHoldout(p,c,j,cases[i],substeps,false));
            latched[bed][i].add(replayHoldout(p,c,j,cases[i],substeps,true));
        }
    }
    std::printf("steps,%d,calibrated,%u,%u,traces,%u\n",substeps,calibrated[0],calibrated[1],(calibrated[0]+calibrated[1])*2*caseCount);
    std::puts("kind,scenario,traces,legacy_hits,legacy_prefault,no_rise,hold,absolute,legacy_before_body,body_limit_shadow,body_limit_latched,tight_shadow_hits,median_delay_ms,max_delay_ms");
    for(int k=0;k<2;++k)for(unsigned s=0;s<caseCount;++s){
        auto& a=totals[k][s];auto& l=latched[k][s];auto& d=a.existingDelay;
        std::sort(d.begin(),d.end());
        std::printf("%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d\n",
            k?"bed":"hotend",cases[s].name,a.traces,a.existingHits,a.existingBeforeFault,
            a.existingCauses[1],a.existingCauses[2],a.existingCauses[3],a.existingBeforeBody,
            a.bodyLimits,l.bodyLimits,a.alarms[0],d.empty()?-1:d[d.size()/2],d.empty()?-1:d.back());
    }
    return 0;
}
