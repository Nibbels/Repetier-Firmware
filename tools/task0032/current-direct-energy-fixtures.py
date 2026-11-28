#!/usr/bin/env python3
"""Retain canonical planner, pause, guard and scan assertions on direct-energy candidate."""
import argparse
from pathlib import Path

def once(s,old,new):
    assert s.count(old)==1,(old,s.count(old))
    return s.replace(old,new)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    a=p.parse_args();t=a.source/'tests'
    path=t/'pause-sense-offset.py';s=path.read_text()
    s=once(s,'"    LookaheadData lookahead;")','"    // Only ordinary direct moves replace")')
    # Extraction of all other actual planner, pause and motion bodies stays.
    path.write_text(s)
    path=t/'pause-sense-offset.cpp';s=path.read_text()
    s=once(s,'#include "direct-phase-firmware.inc"','#include "direct-phase-firmware.inc"\n#include "PreparedDirectProfile.h"')
    s=once(s,'    LookaheadData lookahead{};','    LookaheadData lookahead{};\n    PreparedDirectProfile::Limits directProfile{};')
    s=once(s,'    void limitToStopTail(bool refitRamp);\n    void applyRunningStopTail();\n','')
    start=s.index('void directRamp() {');end=s.index('// Dispatch the replay',start)
    s=s[:start]+'''void directRamp() {
    PrintLine& move = PrintLine::direct;
    const int32_t steps[4] = {161, 0, 0, 0};
    const auto reserve = [&] {
        move = PrintLine{};
        InterruptProtectedBlock lock;
        PrintLine::reserveDirectMove(true);
    };

    reserve();
    PrintLine::prepareDirectMove(steps, true, false);
    assert(interruptsEnabled && move.task == DIRECT_PREPARED_STOPPABLE);
    assert(move.isStepProfileReady() && move.stepsRemaining == 161);
    assert(move.stepProfile.vStart == move.stepProfile.vEnd &&
        move.stepProfile.accelSteps == move.stepProfile.decelSteps && move.stepProfile.accelSteps > 16);
    const auto prepared=move.directProfile;
    assert(prepared.valid && prepared.drop==2*move.lookahead.accelerationPrim);
    assert(prepared.exitEnergy==uint32_t(move.stepProfile.vEnd)*move.stepProfile.vEnd);
    assert(prepared.ceiling==uint32_t(move.stepProfile.vMax)*move.stepProfile.vMax);

    // Keep generic planner's symmetric short-segment refit covered.
    // The ordinary direct executor no longer uses that as its stop tail.
    PrintLine legacyStop = move;
    legacyStop.stepsRemaining = 32;
    legacyStop.invalidateStepProfile();
    MotionPlanner::prepareStepProfile(legacyStop);
    assert(legacyStop.stepProfile.accelSteps==16 && legacyStop.stepProfile.decelSteps==16);
    const auto initialRamp=move.stepProfile;
    {
        InterruptProtectedBlock lock;
        PrintLine::stopDirectMove();
    }
    assert(move.task==DIRECT_PREPARED_CANCELLED && move.stepsRemaining==161);
    assert(move.stepProfile.accelSteps==initialRamp.accelSteps && move.stepProfile.decelSteps==initialRamp.decelSteps);

    reserve();
    interruptBeforeLock = 1;
    onInterrupt = [] {
        assert(interruptsEnabled && PrintLine::hasStoppableDirectMove());
        InterruptProtectedBlock lock;
        PrintLine::stopDirectMove();
        assert(PrintLine::direct.task == DIRECT_PREPARING_STOPPED);
    };
    PrintLine::prepareDirectMove(steps, true, false);
    assert(interruptBeforeLock == 0 && interruptsEnabled);
    assert(move.task==DIRECT_PREPARED_CANCELLED && move.stepsRemaining==161);
    assert(move.directProfile.drop==prepared.drop && move.directProfile.horizon==prepared.horizon);

    reserve();
    PrintLine::prepareDirectMove(steps, true, false);
    {
        InterruptProtectedBlock lock;
        move.task = DIRECT_RUNNING_STOPPABLE;
        PrintLine::stopDirectMove();
        assert(move.task==DIRECT_RUNNING_STOPPABLE_STOPPING && PrintLine::hasStoppableDirectMove());
        assert(move.stepsRemaining==161 && move.directProfile.drop==prepared.drop);
        PrintLine::stopDirectMove();
        assert(move.stepsRemaining==161);
    }

    reserve();
    const int32_t noSteps[4] = {};
    PrintLine::prepareDirectMove(noSteps, true, false);
    assert(!PrintLine::directMoveActive() && move.stepsRemaining == 0);
}

'''+s[end:]
    path.write_text(s)
    path=t/'task0034-guarded-drive.py';s=path.read_text()
    s=once(s,'motion.index("void PrintLine::applyRunningStopTail(")','motion.index("void PrintLine::stopDirectMove(")')
    path.write_text(s)
    path=t/'task0034-guarded-drive.cpp';s=path.read_text()
    s=once(s,'uint16_t ComputeV(uint32_t, uint32_t) { return 0; }',
        '''uint16_t ComputeV(uint32_t, uint32_t) { return 0; }
uint16_t integerSqrt(int32_t x) {
    assert(interruptsEnabled && x>0);
    return uint16_t(sqrt(double(x))+0.5);
}''')
    path.write_text(s)
    # Retire the obsolete Visual Micro check in this historical disposable
    # checkout too; the qualified P0 successor already removed it.
    for obsolete in ('tests/task0013-visual-micro.py','Repetier/Repetier.vcxproj','Repetier/Repetier.vcxproj.filters'):
        (a.source/obsolete).unlink()
    path=t/'task0034-stop-tail.py';s=path.read_text()
    s=once(s,'Compare the actual optimized direct-stop refit to the historical formula.',
        'Compare the retained generic ramp refit to its historical formula.')
    s=once(s,'void PrintLine::limitToStopTail(','void PrintLine::fitRampToRemainingSteps(')
    s=once(s,'    void limitToStopTail(bool);\n','')
    s=once(s,'    p.limitToStopTail(refit);',
        '    if(p.stepsRemaining>32){\n        p.stepsRemaining=32;\n        if(refit)p.fitRampToRemainingSteps();\n    }')
    path.write_text(s)
    path=t/'task0034-direct-enable.py';s=path.read_text()
    s=once(s,'DIRECT_RUNNING=4,DIRECT_RUNNING_STOPPABLE=5,FOR_DIRECT=0;',
        'DIRECT_RUNNING=4,DIRECT_RUNNING_STOPPABLE=5,DIRECT_PREPARED_CANCELLED=10,FOR_DIRECT=0;')
    s=once(s,'    float fullSpeed=60;','    struct { bool valid=true; } directProfile;\n    float fullSpeed=60;')
    s=once(s,'struct Clock { int peak=42;','struct Clock { uint32_t ticks=0; int peak=42;')
    s=once(s,'    static void finishMove(PrintLine&,bool){assert(false);}',
        '    static void finishMove(PrintLine& d,bool queued){assert(!queued);d.task=0;}\n'
        '    static uint32_t armDirectStep(PrintLine& d){assert(irq && timer1Masked && d.directProfile.valid);return 1234;}')
    s=once(s,'        if(task==DIRECT_PREPARED_STOPPABLE && ticks==5)\n            PrintLine::direct.stepsRemaining=32;','')
    s=once(s,'        assert(StepExecutor::performDirectMove()==5000);',
        '        assert(StepExecutor::performDirectMove()==1234);')
    s=once(s,'        assert(!irq && timer1Masked && enabled==28 && directions==1 && advance==1 && !pulses && ticks==27);',
        '        assert(irq && timer1Masked && enabled==28 && directions==1 && advance==0 && !pulses && ticks==27);')
    s=once(s,'        assert(PrintLine::direct.stepsRemaining==(task==DIRECT_PREPARED_STOPPABLE?32:120));',
        '        assert(PrintLine::direct.stepsRemaining==120);\n        irq=false;')
    s=once(s,'    for (uint8_t flags :',
        '''    // Release before readiness: no driver activation or pulse after cancellation.
    irq=false; enabled=directions=advance=pulses=ticks=0;
    PrintLine::direct=PrintLine(); PrintLine::direct.task=DIRECT_PREPARED_STOPPABLE;
    for(ticks=0;ticks<5;++ticks)assert(StepExecutor::performDirectMove()==2000);
    PrintLine::direct.task=DIRECT_PREPARED_CANCELLED;
    assert(StepExecutor::performDirectMove()==1000);
    assert(PrintLine::direct.task==0 && enabled==5 && !directions && !advance && !pulses);
    for (uint8_t flags :''')
    path.write_text(s)
    print("Canonical direct preparation and guarded/scan fixtures migrated; prior assertions outside changed contract retained.")

if __name__=='__main__':main()
