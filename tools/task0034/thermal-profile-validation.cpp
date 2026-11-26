// A38 shadow-policy experiment. No firmware thresholds, device measurements,
// actuator qualification, ADC identity or actual PID/AVR timing here.
// c++ -std=c++11 -O2 tools/task0034/thermal-profile-validation.cpp -o /tmp/profile-validation
// /tmp/profile-validation [8|4]  (integration steps per 100ms sample)
#include "../../Repetier/ThermalRelayProfile.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>
using ThermalRelayProfile::Recorder;
using ThermalRelayProfile::Summary;
struct Plant {
    bool bed;
    double watts, bodyC, coupling, loss, tau, heaterC;
    uint8_t cap;
};
struct Model {
    double heater,body,sensor;
    explicit Model(double t):heater(t),body(t),sensor(t){}
    float measured()const{return float(std::round(sensor*10)/10);}
    void step(const Plant& p,double power,double ambient,double load,double external,double dt){
        const double transfer=p.coupling*(heater-body);
        heater+=dt*(power-transfer-p.loss*.15*(heater-ambient))/p.heaterC;
        body+=dt*(transfer-p.loss*.85*(body-ambient)-load+external)/p.bodyC;
        sensor+=dt*(body-sensor)/p.tau;
        if(!std::isfinite(body)||!std::isfinite(heater)||!std::isfinite(sensor))std::abort();
    }
};
static double dutyFraction(uint8_t d){return d>=254?1.0:double(d&254)/256;}
static double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
struct Calibration {
    Summary profile;
    float ku=0,tu=0;
    int outcome=0; // 0 timeout, 1 cutoff, 2 no-rise, 3 convergence, 4 nonconvergence
};
static Calibration train(const Plant& p,int substeps){
    const float target=p.bed?90:200, cutoff=target+(p.bed?15:20);
    const uint32_t limit=p.bed?3600000:1200000, phaseLimit=p.bed?600000:180000;
    const uint32_t riseLimit=p.bed?60000:20000;
    Model m(25);Recorder r;r.begin(0,25,p.cap,p.cap);
    Calibration out;bool heat=true;uint32_t phase=0,start=0,rise=0,on=0;
    float low=25,high=25,riseTemp=25,kuSum=0,tuSum=0,prevKu=0,prevTu=0;
    unsigned cycles=0,stable=0;
    for(uint32_t now=100;now<=limit;now+=100){
        for(int i=0;i<substeps;++i)m.step(p,heat?p.watts*dutyFraction(p.cap):0,25,0,0,.1/substeps);
        const float t=m.measured();
        if(t>cutoff){out.outcome=1;return out;}
        if(now-phase>phaseLimit)return out;
        r.sample(now,t,heat?p.cap:0);
        low=std::min(low,t);high=std::max(high,t);
        if(heat&&t>=target+.375f){heat=false;on=now-start;phase=now;}
        else if(heat&&now-rise>=riseLimit){
            if(t-riseTemp<1){out.outcome=2;return out;}
            rise=now;riseTemp=t;
        }else if(!heat&&t<=target-.375f){
            r.completeOff(now);++cycles;
            const uint32_t period=now-start;
            const float fraction=float(on)/float(period),span=high-low;
            bool usable=false;
            if(cycles>2&&period>=1000&&span>=.75f&&fraction>=.05f&&fraction<=.95f){
                const float ku=4.f*p.cap*std::sin(3.14159265f*fraction)/(3.14159265f*span),tu=period*.001f;
                if(ku>0&&ku<=1000&&tu>0){
                    usable=true;
                    if(stable&&(std::fabs(ku-prevKu)>.15f*prevKu||std::fabs(tu-prevTu)>.15f*prevTu)){stable=0;kuSum=tuSum=0;}
                    if(!stable){prevKu=ku;prevTu=tu;}
                    ++stable;kuSum+=ku;tuSum+=tu;
                }
            }
            if(!usable){stable=0;kuSum=tuSum=0;}
            if(cycles==8){
                if(stable>=3&&r.completedProfile()){
                    out.outcome=3;out.profile=*r.completedProfile();out.ku=kuSum/stable;out.tu=tuSum/stable;
                }else out.outcome=4;
                return out;
            }
            heat=true;phase=start=rise=now;riseTemp=low=high=t;
        }
        r.afterDecision(now,t,heat?p.cap:0);
    }
    return out;
}
struct Pid {
    double kp,ki,kd,integral=0,slope=0,previous=0;
    bool valid=false;
    Pid(const Calibration& c,int j){
        const double rules[4][3]={{.45,1.6,.08},{1./2.2,2.2,1./6.3},{.65,1.6,.16},{.45,3,.08}};
        kp=rules[j][0]*c.ku;ki=kp/(rules[j][1]*c.tu);kd=kp*rules[j][2]*c.tu;
        if(!(kp>0&&kp<=1000&&ki>0&&ki<=1000&&kd>=0&&kd<=100000))std::abort();
    }
    uint8_t update(double t,double target,uint8_t cap,double cutoff){
        if(valid)slope+=.1/.6*((t-previous)/.1-slope);else slope=0;
        previous=t;
        if(target<40||t>cutoff){integral=0;valid=false;return 0;}
        const double error=target-t,pd=kp*error-kd*slope;
        const double candidate=clamp(integral+ki*error*(valid?.1:0),-20,cap);
        if((pd+candidate<=cap||error<0)&&(pd+candidate>=0||error>0))integral=candidate;
        valid=true;return uint8_t(clamp(pd+integral,0,cap));
    }
};
struct Candidate {double factor,rateMargin,reboundMargin;uint32_t graceMs;};
static const Candidate candidates[3]={{1.1,.1,.5,1000},{1.5,.3,2,3000},{2,.5,5,5000}};
// Test like-for-like nonoverlapping 1s/5s windows. For ordinary PID, start
// new windows on a zero/positive duty transition; partial duty still uses
// the full-power calibration maximum (no unvalidated linear PWM scaling).
// A long continuous zero-command phase can be flagged by excess rebound or
// a fresh 5s positive rise after the trained peak delay + grace. These are
// observational hypotheses, never a proof of a stuck electrical switch.
struct Shadow {
    struct Anchor {uint32_t at;float t;};
    const Summary& p;
    bool heating=false;
    Anchor phase,one,five;
    std::array<int,3> hit{{-1,-1,-1}},cause{{0,0,0}}; // 1 excessive, 2 off
    Shadow(const Summary& profile,float t):p(profile),phase{0,t},one{0,t},five{0,t}{}
    void decision(uint32_t now,float t,uint8_t request){
        const bool next=request!=0;
        if(next!=heating){heating=next;phase=one=five={now,t};}
    }
    void observe(uint32_t now,float t){
        const uint32_t d1=now-one.at,d5=now-five.at;
        const float slope1=d1>=1000?(t-one.t)*1000.f/d1:0;
        const float slope5=d5>=5000?(t-five.t)*1000.f/d5:0;
        for(unsigned i=0;i<3;++i){
            if(hit[i]>=0)continue;
            const auto& c=candidates[i];int why=0;
            if(heating){
                const bool have1=p.mask&(Summary::First1|Summary::Later1);
                const bool have5=p.mask&(Summary::First5|Summary::Later5);
                if((have1&&d1>=1000&&slope1>std::max(p.rates[0],p.rates[2])*c.factor+c.rateMargin)||
                   (have5&&d5>=5000&&slope5>std::max(p.rates[1],p.rates[3])*c.factor+c.rateMargin))why=1;
            }else if(now-phase.at>=1000&&t-phase.t>p.rebound*c.factor+c.reboundMargin){why=2;}
            else if(!heating&&d5>=5000&&five.at-phase.at>=p.peakDelayMs+c.graceMs&&t-five.t>c.reboundMargin){why=2;}
            if(why){hit[i]=int(now);cause[i]=why;}
        }
        if(d1>=1000)one={now,t};
        if(d5>=5000)five={now,t};
    }
};
// Fault-free disturbances are deliberately out-of-calibration operation,
// not an assumption that all such PID traces are safe. Check hard cutoff
// separately. Frozen sensor/stuck-off are negative controls for these two
// new rules; existing no-rise/hold/stale protections remain necessary.
enum Scenario { Cold, Warm, FanLoad, CrossHeat, StuckOn, ExcessPower, Frozen, StuckOff, Count };
static const char* names[]={"cold","warm","fan-load","cross-heat","stuck-on","excess-power","frozen","stuck-off"};
struct Replay {std::array<int,3> hit,cause;int cutoff=-1,faultAt=-1;bool settled=false;};
static Replay replay(const Plant& p,const Calibration& c,int j,Scenario sc,int substeps){
    const double target=p.bed?(sc==Warm?100:sc==FanLoad?70:90):(sc==Warm?220:sc==FanLoad?180:200);
    const double ambient=sc==FanLoad?15:sc==CrossHeat?35:25;
    const double cutoff=p.bed?180:280; // illustrative independent measured-temperature bound
    const uint32_t off=p.bed?660000:480000,end=off+(p.bed?180000:120000);
    const int faultAt=sc==StuckOn?int(off):sc==ExcessPower?0:(sc==Frozen||sc==StuckOff)?30000:-1;
    Model m(sc==Warm?target-15:ambient);Pid pid(c,j);Shadow shadow(c.profile,m.measured());
    uint8_t request=pid.update(m.measured(),target,p.cap,cutoff);
    shadow.decision(0,m.measured(),request);
    float frozen=m.measured();Replay out;out.faultAt=faultAt;
    double holdError=0;
    for(uint32_t now=100;now<=end;now+=100){
        const uint32_t prev=now-100;
        const bool failed=faultAt>=0&&int(prev)>=faultAt;
        double power=p.watts*dutyFraction(request);
        if(failed&&sc==StuckOn)power=p.watts;
        if(failed&&sc==ExcessPower)power*=1.8;
        if(failed&&sc==StuckOff)power=0;
        const double load=sc==FanLoad&&prev>=off/3&&prev<off/2?(p.bed?35:3):0;
        const double external=sc==CrossHeat&&prev>=off?(p.bed?80:8):0;
        for(int i=0;i<substeps;++i)m.step(p,power,ambient,load,external,.1/substeps);
        float t=m.measured();
        if(failed&&sc==Frozen)t=frozen;else frozen=t;
        // Bounded deterministic measurement perturbation, not stochastic fit.
        if(sc==FanLoad||sc==CrossHeat)t+=float(int((now/100)%3)-1)*.1f;
        if(out.cutoff<0&&t>cutoff)out.cutoff=int(now);
        if(now>=off-30000&&now<off)holdError=std::max(holdError,std::fabs(t-target));
        shadow.observe(now,t);
        request=pid.update(t,now<off?target:0,p.cap,cutoff);
        shadow.decision(now,t,request);
    }
    out.hit=shadow.hit;out.cause=shadow.cause;out.settled=holdError<=5;
    return out;
}
struct Totals {
    unsigned traces=0,settled=0,cutoff=0,safeAlarms[3]{},alarms[3]{},prefault[3]{},detected[3]{},wrongCause[3]{},beforeCutoff[3]{};
    std::vector<int> delays[3];
    void add(const Replay& r,Scenario sc){
        ++traces;settled+=r.settled;cutoff+=r.cutoff>=0;
        for(unsigned i=0;i<3;++i){
            if(r.hit[i]<0)continue;
            ++alarms[i];
            if(r.faultAt<0){safeAlarms[i]+=r.cutoff<0;continue;}
            if(r.hit[i]<r.faultAt){++prefault[i];continue;}
            ++detected[i];
            if((sc==StuckOn&&r.cause[i]!=2)||(sc==ExcessPower&&r.cause[i]!=1))++wrongCause[i];
            beforeCutoff[i]+=r.cutoff<0||r.hit[i]<r.cutoff;
            delays[i].push_back(r.hit[i]-r.faultAt);
        }
    }
};
int main(int argc,char** argv){
    const int substeps=argc>1?std::atoi(argv[1]):4;
    if(substeps!=4&&substeps!=8)return 2;
    Totals totals[2][Count];unsigned outcomes[2][5]{},masks[2][64]{};
    for(bool bed:{false,true})for(double watts:bed?std::vector<double>{300,450,600}:std::vector<double>{30,60,100})
    for(double mass:bed?std::vector<double>{300,800}:std::vector<double>{2,6,18})
    for(double coupling:bed?std::vector<double>{8,20}:std::vector<double>{4,12})
    for(double loss:bed?std::vector<double>{2,4}:std::vector<double>{.04,.07})
    for(double tau:bed?std::vector<double>{1,5}:std::vector<double>{.15,.75,1.5})for(uint8_t cap:{uint8_t(128),uint8_t(255)}){
        const Plant p{bed,watts,mass,coupling,loss,tau,bed?40.:.75,cap};
        const Calibration c=train(p,substeps);++outcomes[bed][c.outcome];
        if(c.outcome!=3)continue;
        ++masks[bed][c.profile.mask];
        for(int j=bed?2:0;j<(bed?4:2);++j)for(int s=0;s<Count;++s){
            const auto scenario=Scenario(s);
            totals[bed][s].add(replay(p,c,j,scenario,substeps),scenario);
        }
    }
    std::printf("steps_per_sample,%d\n",substeps);
    for(int k=0;k<2;++k){
        const char* kind=k?"bed":"hotend";
        std::printf("calibration,%s,%u,%u,%u,%u,%u\n",kind,outcomes[k][0],outcomes[k][1],outcomes[k][2],outcomes[k][3],outcomes[k][4]);
        for(int m=0;m<64;++m)if(masks[k][m])std::printf("mask,%s,%d,%u\n",kind,m,masks[k][m]);
    }
    std::puts("kind,scenario,candidate,traces,settled,cutoff,alarms,alarms_without_cutoff_fault_free,prefault,detected_after_fault,wrong_cause,earlier_than_cutoff_or_none,delay_min_ms,delay_median_ms,delay_max_ms");
    for(int k=0;k<2;++k)for(int s=0;s<Count;++s){auto& a=totals[k][s];for(unsigned i=0;i<3;++i){
        auto& d=a.delays[i];std::sort(d.begin(),d.end());
        std::printf("%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d\n",k?"bed":"hotend",names[s],i,a.traces,a.settled,a.cutoff,a.alarms[i],a.safeAlarms[i],a.prefault[i],a.detected[i],a.wrongCause[i],a.beforeCutoff[i],d.empty()?-1:d.front(),d.empty()?-1:d[d.size()/2],d.empty()?-1:d.back());
    }}
}
