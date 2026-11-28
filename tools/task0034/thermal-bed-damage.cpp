// A48: bed-only material threshold sensitivity; hypothetical A38/A45 plants.
// c++ -std=c++11 -O2 -Wall -Wextra -Werror tools/task0034/thermal-bed-damage.cpp -o /tmp/bed-damage
// /tmp/bed-damage 4 (25ms integration), or 8 (12.5ms).
#define main a38_original_main
#include "thermal-profile-validation.cpp"
#undef main
namespace {
enum Fault { None, Gain, Leak, Detached };
struct Case {const char* name;Fault fault;double amount;int onset;bool external,loadRelease;};
const Case cases[]={
    {"nominal",None,0,0,false,false}, {"load-release",None,0,0,false,true},
    {"cross-heat",None,0,3,true,false},
    {"gain-10-cold",Gain,1.1,0,false,false}, {"gain-25-cold",Gain,1.25,0,false,false},
    {"gain-50-cold",Gain,1.5,0,false,false}, {"gain-25-rise",Gain,1.25,1,false,false},
    {"gain-50-rise",Gain,1.5,1,false,false}, {"gain-25-hold",Gain,1.25,2,false,false},
    {"gain-50-hold",Gain,1.5,2,false,false}, {"gain-80-hold",Gain,1.8,2,false,false},
    {"leak-10-off",Leak,.1,3,false,false}, {"leak-25-off",Leak,.25,3,false,false},
    {"leak-50-off",Leak,.5,3,false,false}, {"leak-100-off",Leak,1,3,false,false},
    {"detached-hold",Detached,0,2,false,false}
};
// A45 single-channel guard, with configurable measured cutoff. No ADC or
// independent sensor modeled. A physical damage threshold is separate.
struct Guard {
    enum Phase { Idle, Full, Hold };Phase phase=Idle;
    uint32_t last=0;float base=0;unsigned errors=0;int first=-1;
    void observe(uint32_t now,float t,double target,double cutoff){
        if(first>=0)return;
        if(t>cutoff&&t<=300){first=int(now);return;}
        bool error=false;
        if(now-last>12000){
            if(phase==Full){error=t-base<1;if(!error){last=now;base=t;}}
            else if(phase==Hold){error=std::fabs(t-target)>20;if(!error)last=now-12000+1000;}
        }
        if(error){if(++errors>10)first=int(now);}else if(errors)--errors;
    }
    void decision(uint32_t now,float t,double target,uint8_t duty){
        if(first>=0||target<40){phase=Idle;return;}
        if(phase!=Hold&&target-t>20&&duty){if(phase!=Full){phase=Full;last=now;base=t;}}
        else if(phase!=Hold&&std::fabs(t-target)<=19){phase=Hold;last=now;base=float(target);}
    }
};
struct Result {double heater=25,body=25,sensor=25,postBody=0,postRise=0;int trip=-1;};
Result replay(const Plant& p,const Calibration& c,int j,const Case& sc,int substeps,double cutoff,bool independent){
    const uint32_t off=660000,end=840000;
    const uint32_t onset=sc.onset==0?0:sc.onset==1?30000:sc.onset==2?off-120000:off;
    const double ambient=sc.loadRelease?15:25,target=90;
    Model m(ambient);Pid pid(c,j);Guard guard;Result r;
    uint8_t duty=pid.update(m.measured(),target,p.cap,cutoff);
    guard.decision(0,m.measured(),target,duty);
    double detached=m.sensor,tripBody=0;
    for(uint32_t now=100;now<=end;now+=100){
        const uint32_t prev=now-100;const bool active=prev>=onset;
        double power=p.watts*dutyFraction(duty);
        if(active&&sc.fault==Gain)power*=sc.amount;
        if(active&&sc.fault==Leak)power=std::max(power,p.watts*sc.amount);
        if(independent&&guard.first>=0)power=0;
        const double load=sc.loadRelease&&prev>=off/3&&prev<off/2?50:0;
        const double external=sc.external&&active&&prev<onset+60000?80:0;
        for(int i=0;i<substeps;++i){
            m.step(p,power,ambient,load,external,.1/substeps);
            if(active&&sc.fault==Detached)detached+=(ambient-detached)*(.1/substeps)/5.;else detached=m.sensor;
            r.heater=std::max(r.heater,m.heater);r.body=std::max(r.body,m.body);
            if(guard.first>=0)r.postBody=std::max(r.postBody,m.body);
        }
        float t=float(std::round(detached*10)/10);
        if(sc.loadRelease||sc.external)t+=float(int((now/100)%3)-1)*.1f;
        r.sensor=std::max(r.sensor,double(t));
        if(now==off)guard.phase=Guard::Idle;
        const double demand=now<off?target:0;
        guard.observe(now,t,demand,cutoff);
        if(guard.first>=0&&r.trip<0){r.trip=guard.first;tripBody=m.body;r.postBody=m.body;}
        duty=pid.update(t,demand,p.cap,cutoff);
        if(guard.first>=0)duty=0;
        guard.decision(now,t,demand,duty);
    }
    if(r.trip>=0)r.postRise=r.postBody-tripBody;
    return r;
}
struct Aggregate {
    unsigned n=0,trips=0,h150=0,b150=0,s150=0;
    double hmax=0,bmax=0,smax=0,postRise=0;
    void add(const Result& r){++n;trips+=r.trip>=0;h150+=r.heater>=150;b150+=r.body>=150;s150+=r.sensor>=150;
        hmax=std::max(hmax,r.heater);bmax=std::max(bmax,r.body);smax=std::max(smax,r.sensor);postRise=std::max(postRise,r.postRise);}
};
}
int main(int argc,char** argv){
    const int steps=argc>1?std::atoi(argv[1]):4;if(steps!=4&&steps!=8)return 2;
    const double cutoffs[]={275,150,120};constexpr unsigned ncase=sizeof(cases)/sizeof(cases[0]);
    Aggregate totals[3][2][ncase];unsigned plants=0;
    for(double watts:{300.,450.,600.})for(double mass:{300.,800.})for(double coupling:{8.,20.})
    for(double loss:{2.,4.})for(double tau:{1.,5.})for(uint8_t cap:{uint8_t(128),uint8_t(255)}){
        const Plant p{true,watts,mass,coupling,loss,tau,40.,cap};const Calibration c=train(p,steps);
        if(c.outcome!=3)continue;
        ++plants;
        for(int j:{2,3})for(unsigned k=0;k<3;++k)for(unsigned g=0;g<2;++g)for(unsigned i=0;i<ncase;++i)
            totals[k][g][i].add(replay(p,c,j,cases[i],steps,cutoffs[k],g!=0));
    }
    std::printf("steps,%d,plants,%u,traces,%u\n",steps,plants,plants*2*3*2*ncase);
    std::puts("cutoff,independent,scenario,n,trips,heater150,body150,sensor150,heater_max,body_max,sensor_max,post_trip_body_rise_max");
    for(unsigned k=0;k<3;++k)for(unsigned g=0;g<2;++g)for(unsigned i=0;i<ncase;++i){const auto& a=totals[k][g][i];
        std::printf("%.0f,%u,%s,%u,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f\n",cutoffs[k],g,cases[i].name,a.n,a.trips,a.h150,a.b150,a.s150,a.hmax,a.bmax,a.smax,a.postRise);}
    return 0;
}
