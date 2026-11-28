// Actual source above. Only GPIO, timebase and HAL arithmetic are host stubs.
// This checks the reference at compare boundaries; not actual wall-time or
// mechanical acceleration with nested ISR lateness.
static unsigned long cases=0,stepsChecked=0;
static void periodCheck(uint32_t before,uint32_t after,uint32_t delay){
    const double denominator=sqrt(double(before))+sqrt(double(after));
    assert(denominator>0 && delay>=32000000.0/denominator);
}
unsigned trial(uint8_t axis,bool mixed,int sign,unsigned inject,bool stoppable,
               uint16_t maximum,uint16_t exit,uint32_t acceleration,int32_t distance,
               bool cancelPrepared=false){
    reset(axis,sign>0?100:5000);
    primaryAxis=axis;
    int32_t distances[3]={};distances[axis]=sign*distance;
    if(mixed){assert(axis==X_AXIS);distances[Y_AXIS]=-sign*(distance*371/600);distances[Z_AXIS]=sign*(distance*129/600);}
    for(uint8_t a=0;a<3;++a){
        physical[a]=distances[a]<0?5000:100;closed[a]=false;
        if(a<Z_AXIS){Printer::xyMotion.setMotorSteps(a,physical[a]);Printer::xyMotion.setNominalSteps(a,physical[a]);Printer::xyMotion.resetDirect(a);}
        else{Printer::zMotion.setMotorReference(physical[a]);Printer::zMotion.setNominalSteps(physical[a]);Printer::zMotion.resetDirectPosition();}
    }
    auto& m=PrintLine::direct;m.flags=FLAG_ABORT_AT_ENDSTOPS;m.dir=0;m.primaryAxis=axis;m.stepsRemaining=distance;m.stepProfile={};
    m.stepProfile.vStart=m.stepProfile.vEnd=exit;m.stepProfile.vMax=maximum;
    m.directProfile=PreparedDirectProfile::prepare(maximum,exit,acceleration);
    const auto lim=m.directProfile;
    assert(lim.valid);
    int32_t origins[3],nominals[3];
    m.delta[E_AXIS]=0;
    for(uint8_t a=0;a<3;++a){
        m.delta[a]=std::abs(distances[a]);if(distances[a])m.setMoveOfAxis(a);
        if(distances[a]>0)m.setPositiveDirectionForAxis(a);
        if(a==Z_AXIS)Printer::zMotion.requestDirectMove(distances[a]);else Printer::xyMotion.requestDirect(a,distances[a]);
        origins[a]=motor(a);nominals[a]=nominal(a);
    }
    m.task=stoppable?DIRECT_PREPARED_STOPPABLE:DIRECT_PREPARED;
    armed=false;requested=false;visits=0;injectAt=inject;
    StepExecutor::directionOwner=StepExecutor::DirectionOwner::None;
    StepExecutor::stepsPerTimerCall=5;StepExecutor::stepIntervalTicks=1997;
    if(cancelPrepared){
        InterruptProtectedBlock lock;PrintLine::stopDirectMove();
        assert(m.task==DIRECT_PREPARED_CANCELLED && m.stepsRemaining==distance);
    }
    armed=true;
    long period;
    {InterruptProtectedBlock lock;period=StepExecutor::performDirectMove();}
    auto& clock=StepExecutor::stepClocks[FOR_DIRECT];
    assert(!pulses);
    if(m.task!=DIRECT_IDLE){
        assert(!cancelPrepared && m.stepsRemaining==distance);
        assert(clock.steps==0 && clock.ticks<=PreparedDirectProfile::cap(lim,distance-1));
        periodCheck(uint32_t(exit)*exit,clock.ticks,period);
    }
    unsigned n=0;
    for(unsigned tick=0;PrintLine::directMoveActive();++tick){
        assert(tick<unsigned(distance)+2);
        const uint32_t before=clock.ticks;
        const unsigned oldCount=clock.steps;
        const bool wasRequested=requested;
        {InterruptProtectedBlock lock;period=StepExecutor::performDirectMove();}
        assert(!high[0]&&!high[1]&&!high[2]&&m.stepsRemaining>=0);
        n=unsigned(std::abs(motor(axis)-origins[axis]));
        assert(clock.steps==n && n<=unsigned(distance));
        assert(n==oldCount+1 || (oldCount==0 && requested && m.task==DIRECT_IDLE && n==0));
        for(uint8_t a=0;a<3;++a){
            const int32_t d=distances[a];
            const int32_t wanted=(n*std::abs(d)+unsigned((distance-1)/2))/distance*(d<0?-1:1);
            assert(motor(a)==physical[a] && nominal(a)==nominals[a]);
            assert(ledger(a).steps()==motor(a)-origins[a] && motor(a)-origins[a]==wanted);
        }
        if(m.task!=DIRECT_IDLE){
            assert(m.stepsRemaining==distance-int32_t(n));
            assert(clock.ticks<=PreparedDirectProfile::cap(lim,m.stepsRemaining-1));
            assert(uint64_t(clock.ticks>before?clock.ticks-before:before-clock.ticks)<=lim.drop);
            if(wasRequested || requested)assert(clock.ticks<=before);
            periodCheck(before,clock.ticks,period);
        }else if(n)assert(clock.ticks<=lim.exitEnergy);
        ++stepsChecked;
    }
    armed=false;
    for(uint8_t a=0;a<3;++a)assert(ledger(a).steps()==ledger(a).target());
    if(cancelPrepared)assert(!n);
    else if(requested){
        const auto after=unsigned(std::abs(physical[axis]-primaryAt));
        const uint32_t diff=energyAt>lim.exitEnergy?energyAt-lim.exitEnergy:0;
        const uint32_t bound=diff/lim.drop+uint32_t(diff%lim.drop!=0)+1;
        assert(after<=bound);maxTail=std::max(maxTail,after);
    }else assert(n==unsigned(distance));
    // Ordinary direct reference does not overwrite suspended queue cadence.
    assert(StepExecutor::stepsPerTimerCall==5 && StepExecutor::stepIntervalTicks==1997);
    ++cases;return visits;
}
int main(){
    for(uint8_t a=0;a<3;++a)for(int sign:{1,-1})for(bool stoppable:{false,true}){
        const bool z=a==Z_AXIS;const uint16_t vmax=z?11520:15238,exit=z?192:990;const uint32_t accel=z?128000:228570;
        trial(a,false,sign,0,stoppable,vmax,exit,accel,600,true);
        unsigned count=trial(a,false,sign,0,stoppable,vmax,exit,accel,600);
        for(unsigned at=1;at<=count;++at)trial(a,false,sign,at,stoppable,vmax,exit,accel,600);
        count=trial(a,false,sign,0,stoppable,vmax,exit,accel,2000);
        for(unsigned at=1;at<=count;at+=23)trial(a,false,sign,at,stoppable,vmax,exit,accel,2000);
    }
    for(int sign:{1,-1})for(bool stoppable:{false,true}){
        const unsigned count=trial(X_AXIS,true,sign,0,stoppable,15238,990,228570,600);
        for(unsigned at=1;at<=count;++at)trial(X_AXIS,true,sign,at,stoppable,15238,990,228570,600);
    }
    for(int32_t length:{1,2,31,32,33,506,519})for(uint16_t vmax:{uint16_t(192),uint16_t(11520),uint16_t(31250)}){
        unsigned count=trial(X_AXIS,false,1,0,true,vmax,192,128000,length);
        for(unsigned at=1;at<=count;at+=7)trial(X_AXIS,false,1,at,true,vmax,192,128000,length);
    }
    assert(maxTail>32);
    printf("Actual direct energy reference: %lu cases, %lu checked compare boundaries, max observed tail %u primary steps PASS\n",cases,stepsChecked,maxTail);
}
