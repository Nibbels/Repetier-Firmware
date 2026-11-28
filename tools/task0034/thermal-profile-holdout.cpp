// A42: extended shadow-policy stress study, NOT runtime limits.
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
struct Result {
    std::array<int,3> hit,cause;
    int faultAt=-1,cutoffAt=-1,bodyLimitAt=-1;
    double bodyPeak=0;
};
Result replayHoldout(const Plant& p,const Calibration& c,int j,const Case& scenario,int substeps){
    // Use current firmware's absolute measured cutoff, not its bed setpoint
    // limit. The relay training setpoints remain unchanged.
    const double cutoff=275, target=p.bed?90:200, ambient=scenario.loadRelease?15:25;
    const uint32_t off=p.bed?660000:480000,end=off+(p.bed?180000:120000);
    const uint32_t onset=scenario.onset==0?0:scenario.onset==1?30000:scenario.onset==2?off-120000:off;
    Model m(ambient);Pid pid(c,j);Shadow shadow(c.profile,m.measured());
    uint8_t request=pid.update(m.measured(),target,p.cap,cutoff);
    shadow.decision(0,m.measured(),request);
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
        shadow.observe(now,t);
        request=pid.update(t,now<off?target:0,p.cap,cutoff);
        shadow.decision(now,t,request);
    }
    out.hit=shadow.hit;out.cause=shadow.cause;return out;
}
struct Aggregate {
    unsigned traces=0,measuredCutoffs=0,bodyLimits=0;
    unsigned alarms[3]{},prefault[3]{},after[3]{},beforeMeasured[3]{},beforeBody[3]{};
    std::vector<int> delays[3];
    void add(const Result& r){
        ++traces;measuredCutoffs+=r.cutoffAt>=0;bodyLimits+=r.bodyLimitAt>=0;
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
    Aggregate totals[2][caseCount];unsigned calibrated[2]{};
    for(bool bed:{false,true})for(double watts:bed?std::vector<double>{300,450,600}:std::vector<double>{30,60,100})
    for(double mass:bed?std::vector<double>{300,800}:std::vector<double>{2,6,18})
    for(double coupling:bed?std::vector<double>{8,20}:std::vector<double>{4,12})
    for(double loss:bed?std::vector<double>{2,4}:std::vector<double>{.04,.07})
    for(double tau:bed?std::vector<double>{1,5}:std::vector<double>{.15,.75,1.5})for(uint8_t cap:{uint8_t(128),uint8_t(255)}){
        const Plant p{bed,watts,mass,coupling,loss,tau,bed?40.:.75,cap};
        const Calibration c=train(p,substeps);
        if(c.outcome!=3)continue;
        ++calibrated[bed];
        for(int j=bed?2:0;j<(bed?4:2);++j)for(unsigned i=0;i<caseCount;++i)
            totals[bed][i].add(replayHoldout(p,c,j,cases[i],substeps));
    }
    std::printf("steps,%d,calibrated,%u,%u,traces,%u\n",substeps,calibrated[0],calibrated[1],(calibrated[0]+calibrated[1])*2*caseCount);
    std::puts("kind,scenario,candidate,traces,measured_cutoffs,body_limits,alarms,prefault,detected_after_fault,before_measured_cutoff,before_body_limit,delay_min_ms,delay_median_ms,delay_max_ms");
    for(int k=0;k<2;++k)for(unsigned s=0;s<caseCount;++s){auto& a=totals[k][s];for(unsigned i=0;i<3;++i){
        auto& d=a.delays[i];std::sort(d.begin(),d.end());
        std::printf("%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d\n",k?"bed":"hotend",cases[s].name,i,a.traces,a.measuredCutoffs,a.bodyLimits,a.alarms[i],a.prefault[i],a.after[i],a.beforeMeasured[i],a.beforeBody[i],d.empty()?-1:d.front(),d.empty()?-1:d[d.size()/2],d.empty()?-1:d.back());
    }}
}
