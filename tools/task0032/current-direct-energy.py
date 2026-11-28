#!/usr/bin/env python3
"""Compose a one-pulse distance-domain ordinary-direct reference on #225.

No new per-slot or persistent state. The planner-only lookahead union member
is replaced by prepared limits at direct publication; the direct clock stores
the energy/root of the armed boundary. Queued and guarded profiles stay legacy.
The real timer deadline and CPU budget must be qualified before selection.
"""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent


def once(s,old,new):
    assert s.count(old)==1,(old,s.count(old))
    return s.replace(old,new)


def candidate(fw):
    h=(fw/'motion.h').read_text();m=(fw/'motion.cpp').read_text()
    e=(fw/'StepExecutor.cpp').read_text();eh=(fw/'StepExecutor.h').read_text()
    h=once(h,'#include "QueueTask.h"','#include "QueueTask.h"\n#include "PreparedDirectProfile.h"')
    h=once(h,'    DIRECT_RUNNING_STOPPABLE_STOPPING,\n};','    DIRECT_RUNNING_STOPPABLE_STOPPING,\n    DIRECT_PREPARED_CANCELLED,\n};')
    h=once(h,'''    void limitToStopTail(bool refitRamp);
    // Only Timer1, at an IRQ-closed complete logical step boundary.
    void applyRunningStopTail();''','''    static bool directStopping(char phase) {
        return phase==DIRECT_RUNNING_STOPPING || phase==DIRECT_RUNNING_STOPPABLE_STOPPING;
    }''')
    h=once(h,'    LookaheadData lookahead;','''    // Only ordinary direct moves replace planner scratch with immutable
    // execution limits, after all planner reads and before publication.
    // Queued and guarded moves retain the existing lookahead/profile ABI.
    static_assert(sizeof(PreparedDirectProfile::Limits)<=sizeof(LookaheadData), "Direct profile must not enlarge queue slots");
    union {
        LookaheadData lookahead;
        PreparedDirectProfile::Limits directProfile;
    };''')
    h=once(h,'phase == DIRECT_RUNNING_STOPPING) &&','phase == DIRECT_RUNNING_STOPPING || phase == DIRECT_PREPARED_CANCELLED) &&')
    m=once(m,'if (direct.task >= DIRECT_PREPARED && direct.stepsRemaining > 0)',
        'if (direct.task >= DIRECT_PREPARED && direct.task != DIRECT_PREPARED_CANCELLED && direct.stepsRemaining > 0)')
    m=once(m,'''    direct.setSegmentGeometry(axisDistanceMM);

    float feedrate''','''    // Reactivate planner scratch after a previous ordinary direct profile.
    direct.lookahead = {};
    direct.setSegmentGeometry(axisDistanceMM);

    float feedrate''')
    m=once(m,'''    MotionPlanner::prepareStepProfile(direct);

    InterruptProtectedBlock publish;''','''    MotionPlanner::prepareStepProfile(direct);
    if (guard == GuardedStop::Normal) {
        const uint16_t maximum = RMath::min(uint16_t(31250),direct.stepProfile.vMax);
        direct.stepProfile.vStart = RMath::min(maximum,direct.stepProfile.vStart);
        direct.stepProfile.vEnd = RMath::min(maximum,direct.stepProfile.vEnd);
        direct.stepProfile.vMax = maximum;
        // Compute exact stopping horizon before the publication mask.
        const auto limits=PreparedDirectProfile::prepare(maximum,direct.stepProfile.vEnd,
                                                         direct.lookahead.accelerationPrim);
        direct.directProfile=limits;
    }

    InterruptProtectedBlock publish;''')
    m=once(m,'''    if (direct.task == DIRECT_PREPARING_STOPPED)
        direct.limitToStopTail(true);
    direct.task = stoppable ? DIRECT_PREPARED_STOPPABLE : DIRECT_PREPARED;''','''    direct.task = direct.task == DIRECT_PREPARING_STOPPED ? DIRECT_PREPARED_CANCELLED :
        (stoppable ? DIRECT_PREPARED_STOPPABLE : DIRECT_PREPARED);''')
    a=m.index('    if(phase==DIRECT_PREPARED || phase==DIRECT_PREPARED_STOPPABLE)\n',m.index('void PrintLine::stopDirectMove()'))
    b=m.index('void PrintLine::fitRampToRemainingSteps()',a)
    m=m[:a]+'''    if(phase==DIRECT_PREPARED || phase==DIRECT_PREPARED_STOPPABLE)
        direct.task=DIRECT_PREPARED_CANCELLED;
}

'''+m[b:]
    eh=once(eh,'    static uint16_t nextStepSpeed(','''    // Ordinary direct moves reuse StepClock::ticks as armed energy and
    // peak as its floor root; queue/guarded use the original time clock.
    static uint32_t armDirectStep(PrintLine& move);
    static uint16_t nextStepSpeed(''')
    e=once(e,'''long StepExecutor::performDirectMove() {
    auto& direct = PrintLine::direct;''','''long StepExecutor::performDirectMove() {
    auto& direct = PrintLine::direct;
    if (direct.task==DIRECT_PREPARED_CANCELLED) {
        HAL::allowInterrupts();
        finishMove(direct,false);
        return 1000;
    }''')
    e=once(e,'''        if (!(direct.flags & FLAG_SCAN_PATH) &&''','''        if (!guardedStartup && !direct.directProfile.valid) {
            HAL::allowInterrupts();
            finishMove(direct,false);
            return 1000;
        }
        if (!(direct.flags & FLAG_SCAN_PATH) &&''')
    e=once(e,'''        updateAdvanceSteps(direct, direct.stepProfile.vStart);

        return (stepIntervalTicks >> 1);''','''        if (!guardedStartup) {
            auto& clock=stepClocks[FOR_DIRECT];
            clock.ticks=uint32_t(clock.peak)*clock.peak;
            HAL::allowInterrupts();
            const uint32_t delay=armDirectStep(direct);
            if (!delay) {
                finishMove(direct,false);
                return 1000;
            }
            // No Advance lead before a cancellable first nominal step.
            // The first completed boundary publishes its actual rate.
            return delay;
        }
        updateAdvanceSteps(direct, direct.stepProfile.vStart);

        return (stepIntervalTicks >> 1);''')
    e=once(e,'// Keep the ramp update in the specialized executor, as in the original AVR build.', '''// Called with I=1 and OCIE1A excluded. All mutable execution data is
// Timer1-owned. Only the stop-phase byte can change while calculating.
uint32_t StepExecutor::armDirectStep(PrintLine& move) {
    auto& clock=stepClocks[FOR_DIRECT];
    const auto& limits=move.directProfile;
    if (!limits.valid || move.stepsRemaining<=0) return 0;
    const auto root=[](uint32_t energy) -> uint16_t {
        // The prepared ceiling is at most 31250^2, inside signed HAL domain.
        if (!energy) return 0;
        const uint16_t r=HAL::integerSqrt(static_cast<int32_t>(energy));
        return uint32_t(r)*r>energy ? uint16_t(r-1) : r;
    };
    const auto plan=[&](bool stopping) -> StopTailEnergy::Group {
        if (stopping) {
            if (!clock.steps || clock.ticks<=limits.exitEnergy)
                return {clock.ticks,0,clock.peak,0};
            // No fixed distance cap. The pre-existing backward envelope
            // ensures sufficient admitted path to reach the exit energy.
            const StopTailEnergy::Plan brake{clock.ticks,limits.exitEnergy,limits.drop,1,true};
            return StopTailEnergy::next(brake,clock.peak,1,root);
        }
        return PreparedDirectProfile::next(limits,clock.ticks,clock.peak,
                                          uint32_t(move.stepsRemaining),1,root);
    };
    const bool stopping=PrintLine::directStopping(move.task);
    auto group=plan(stopping);
    // At most one monotone request transition. Recompute if observed during
    // preparation. A request after this sample can leave one armed pulse.
    if (!stopping && PrintLine::directStopping(move.task)) group=plan(true);
    if (!group.steps || !group.ticks || group.ticks>0xffffffUL) return 0;
    clock.ticks=group.energy;
    clock.peak=group.root;
    return group.ticks<minimumStepIntervalTicks ? minimumStepIntervalTicks : group.ticks;
}

// Keep the ramp update in the specialized executor, as in the original AVR build.''')
    e=once(e,'''    const bool guarded = guard != GuardedStop::Normal;
    const bool scan''','''    const bool guarded = guard != GuardedStop::Normal;
    const bool energyDirect = !forQueue && !guarded;
    const bool scan''')
    e=once(e,'''        if (cancelled) {
            finishMove(*move, false);''','''        if (cancelled || PrintLine::directStopping(move->task)) {
            finishMove(*move, false);''')
    e=once(e,'''    if(!forQueue)move->applyRunningStopTail();
    fast8_t max_loops = guarded ? 1 : stepsPerTimerCall;''','''    if (energyDirect && !stepClocks[FOR_DIRECT].steps && PrintLine::directStopping(move->task)) {
        HAL::allowInterrupts();
        finishMove(*move,false);
        return 1000;
    }
    fast8_t max_loops = (guarded || energyDirect) ? 1 : stepsPerTimerCall;''')
    e=once(e,'''        if(!forQueue) {
            // Every axis ledger and STEP fall belongs to the completed
            // logical transaction. No nested requester changes remaining.
            if(completed<max_loops && move->stepsRemaining>0) {
                HAL::allowInterrupts();
                asm volatile("nop" ::: "memory");
                HAL::forbidInterrupts();
            }
            move->applyRunningStopTail();
            if(move->stepsRemaining<=0)break;
        }
''','')
    e=once(e,'''    const speed_t v = nextStepSpeed(*move, stepClocks[forQueue], completed, stepIntervalTicks);''','''    if (energyDirect) {
        auto& clock=stepClocks[FOR_DIRECT];
        clock.steps+=completed;
        // This is the completed armed boundary, not the next planned one.
        updateAdvanceSteps(*move,clock.peak);
        const uint32_t delay=move->isNoMove() || move->stepsRemaining<=0 ? 0 : armDirectStep(*move);
        if (!delay) {
            finishMove(*move,false);
            return 1000;
        }
        return delay;
    }
    const speed_t v = nextStepSpeed(*move, stepClocks[forQueue], completed, stepIntervalTicks);''')
    return {'motion.h':h,'motion.cpp':m,'StepExecutor.cpp':e,'StepExecutor.h':eh,
        **{name:(HERE/name).read_text() for name in ('StopTailEnergy.h','PreparedDirectProfile.h')}}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True,help='Exact qualified #225 Repetier directory')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();fw=a.source.resolve();out=a.out.resolve()
    assert fw!=out and fw not in out.parents
    files=candidate(fw);out.mkdir(parents=True,exist_ok=True)
    for name,s in files.items():(out/name).write_text(s)
    (out/'provenance.json').write_text(json.dumps({
        'input_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(fw.iterdir()) if p.is_file() and p.suffix in ('.h','.cpp','.ino')},
        'candidate_sha256':{name:hashlib.sha256(s.encode()).hexdigest() for name,s in files.items()},
        'scope':'ordinary direct one-pulse energy reference; queued and guarded unchanged; no throughput or hardware claim'},indent=2)+'\n')

if __name__=='__main__':main()
