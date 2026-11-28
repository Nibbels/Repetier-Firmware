#!/usr/bin/env python3
"""Disposable direct-stop single-writer candidate, not an integration gate.

A running stop request changes only the existing byte-sized phase. Timer1
alone applies the 32-logical-step limit at a complete pulse transaction.
Prepared refit and immediate terminal cancellation remain unchanged.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess
import tempfile
import zipfile


def once(s, old, new):
    assert s.count(old)==1, (old, s.count(old))
    return s.replace(old,new)


def candidate(fw):
    h=(fw/'motion.h').read_text()
    m=(fw/'motion.cpp').read_text()
    e=(fw/'StepExecutor.cpp').read_text()
    h=once(h,'    DIRECT_RUNNING_STOPPABLE,\n};',
        '''    DIRECT_RUNNING_STOPPABLE,
    // Same physical owner/path, but the foreground/PWM has requested
    // a finite running tail. No 32-bit counter write by the requester.
    DIRECT_RUNNING_STOPPING,
    DIRECT_RUNNING_STOPPABLE_STOPPING,
};''')
    h=once(h,'    void limitToStopTail(bool refitRamp);',
        '''    void limitToStopTail(bool refitRamp);
    // Only Timer1, at an IRQ-closed complete logical step boundary.
    void applyRunningStopTail();
    static bool directRunningStoppable(char phase) {
        return phase==DIRECT_RUNNING_STOPPABLE || phase==DIRECT_RUNNING_STOPPABLE_STOPPING;
    }''')
    h=once(h,'(phase == DIRECT_PREPARED || phase == DIRECT_RUNNING) &&',
        '(phase == DIRECT_PREPARED || phase == DIRECT_RUNNING || phase == DIRECT_RUNNING_STOPPING) &&')
    h=once(h,'return task == DIRECT_PREPARING_STOPPABLE || task == DIRECT_PREPARED_STOPPABLE || task == DIRECT_RUNNING_STOPPABLE;',
        'return task == DIRECT_PREPARING_STOPPABLE || task == DIRECT_PREPARED_STOPPABLE || directRunningStoppable(task);')
    m=once(m,'    direct.limitToStopTail(phase == DIRECT_PREPARED_STOPPABLE || phase == DIRECT_PREPARED);',
        '''    if(phase==DIRECT_RUNNING || phase==DIRECT_RUNNING_STOPPABLE) {
        // Atomic byte publication. In-flight pulse/ledger transactions
        // finish before Timer1 applies the cap. A repeated release is a no-op.
        direct.task=phase==DIRECT_RUNNING ? DIRECT_RUNNING_STOPPING : DIRECT_RUNNING_STOPPABLE_STOPPING;
        return;
    }
    if(phase==DIRECT_PREPARED || phase==DIRECT_PREPARED_STOPPABLE)
        direct.limitToStopTail(true);''')
    m=once(m,'void PrintLine::limitToStopTail(bool refitRamp) {',
        '''void PrintLine::applyRunningStopTail() {
    const char phase=task;
    if(phase==DIRECT_RUNNING_STOPPING || phase==DIRECT_RUNNING_STOPPABLE_STOPPING)
        limitToStopTail(false);
}

void PrintLine::limitToStopTail(bool refitRamp) {''')
    e=once(e,'(guarded || move->task == DIRECT_RUNNING_STOPPABLE ? ZMotion::Path::StoppableDirect : ZMotion::Path::Direct)',
        '(guarded || PrintLine::directRunningStoppable(move->task) ? ZMotion::Path::StoppableDirect : ZMotion::Path::Direct)')
    e=once(e,'''    HAL::allowInterrupts();
    fast8_t max_loops = guarded ? 1 : stepsPerTimerCall;
    if (move->stepsRemaining < max_loops)
        max_loops = move->stepsRemaining;
    HAL::forbidInterrupts();

    for (fast8_t loop = 0; loop < max_loops; loop++) {''',
        '''    // Retain a real nested IRQ admission window, but observe the
    // counter only at the subsequent closed transaction boundary.
    HAL::allowInterrupts();
    asm volatile("nop" ::: "memory");
    HAL::forbidInterrupts();
    if(!forQueue)move->applyRunningStopTail();
    fast8_t max_loops = guarded ? 1 : stepsPerTimerCall;
    if (move->stepsRemaining < max_loops)
        max_loops = move->stepsRemaining;

    fast8_t completed = 0;
    for (; completed < max_loops;) {''')
    e=once(e,'''        Printer::endXYZSteps();
    }                       // for loop''',
        '''        Printer::endXYZSteps();
        ++completed;
        if(!forQueue) {
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
    }                       // for loop''')
    e=once(e,'nextStepSpeed(*move, stepClocks[forQueue], max_loops, stepIntervalTicks)',
        'nextStepSpeed(*move, stepClocks[forQueue], completed, stepIntervalTicks)')
    return {'motion.h':h,'motion.cpp':m,'StepExecutor.cpp':e}


HARNESS=r'''
#include <cassert>
#include <cstdint>
#include <algorithm>
#include <cstdio>
#include <initializer_list>
namespace RMath { using std::min; }
ENUM
struct PrintLine {
    static PrintLine direct;
    volatile char task=DIRECT_IDLE;
    int32_t stepsRemaining=0;
    struct {uint32_t accelSteps=0,decelSteps=0;} stepProfile;
    static bool directMovePreparing(char phase){return phase>=DIRECT_PREPARING && phase<=DIRECT_PREPARING_STOPPED;}
    static void stopDirectMove();
    void applyRunningStopTail();
    void limitToStopTail(bool);
    void fitRampToRemainingSteps();
    static bool directRunningStoppable(char phase);
};
PrintLine PrintLine::direct;
HELPER
BODY
int main(){
    auto& d=PrintLine::direct;
    unsigned long cases=0;
    for(int32_t initial:{0,1,31,32,33,255,256,257,65535,65536,65537}){
        for(char phase:{char(DIRECT_RUNNING),char(DIRECT_RUNNING_STOPPABLE)}){
            d.task=phase;d.stepsRemaining=initial;d.stepProfile.accelSteps=d.stepProfile.decelSteps=100;
            const bool stoppable=phase==DIRECT_RUNNING_STOPPABLE;
            PrintLine::stopDirectMove();
            assert(d.stepsRemaining==initial && d.stepProfile.accelSteps==100 && d.stepProfile.decelSteps==100);
            assert(d.task==(stoppable?DIRECT_RUNNING_STOPPABLE_STOPPING:DIRECT_RUNNING_STOPPING));
            assert(d.directRunningStoppable(d.task)==stoppable);
            for(unsigned n=0;n<10;++n)PrintLine::stopDirectMove();
            assert(d.stepsRemaining==initial);
            d.applyRunningStopTail();
            assert(d.stepsRemaining==std::min(initial,32));
            while(d.stepsRemaining>0){
                --d.stepsRemaining;
                PrintLine::stopDirectMove();d.applyRunningStopTail();
            }
            assert(d.stepsRemaining==0 && d.stepProfile.accelSteps==100 && d.stepProfile.decelSteps==100);
            ++cases;
        }
        for(char phase:{char(DIRECT_PREPARING),char(DIRECT_PREPARING_STOPPABLE),char(DIRECT_PREPARING_STOPPED)}){
            d.task=phase;d.stepsRemaining=initial;
            PrintLine::stopDirectMove();assert(d.task==DIRECT_PREPARING_STOPPED && d.stepsRemaining==initial);++cases;
        }
        for(char phase:{char(DIRECT_PREPARED),char(DIRECT_PREPARED_STOPPABLE)}){
            d.task=phase;d.stepsRemaining=initial;d.stepProfile.accelSteps=d.stepProfile.decelSteps=100;
            PrintLine::stopDirectMove();assert(d.task==phase && d.stepsRemaining==std::min(initial,32));
            assert(d.stepProfile.accelSteps==(initial>32?16u:100u));++cases;
        }
        d.task=DIRECT_IDLE;d.stepsRemaining=initial;
        PrintLine::stopDirectMove();assert(d.task==DIRECT_IDLE && d.stepsRemaining==initial);++cases;
    }
    std::printf("Actual running-stop byte publication / cap and prepared ownership: %lu cases PASS\n",cases);
}
'''


def replay(files,out):
    h=files['motion.h'];m=files['motion.cpp']
    enum=h[h.index('enum DirectMovePhase'):h.index('enum class GuardedStop')]
    helper=h[h.index('    static bool directRunningStoppable('):h.index('    static bool directMovePreparing(')]
    helper=helper.replace('static bool directRunningStoppable(','bool PrintLine::directRunningStoppable(').strip()
    body=m[m.index('void PrintLine::stopDirectMove()'):m.index('/** \\brief Check if move is new.')]
    test=HARNESS.replace('ENUM',enum).replace('HELPER',helper).replace('BODY',body)
    (out/'stop-tail-owner.cpp').write_text(test)
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'replay'
        subprocess.run(['c++','-std=c++11','-O1','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all',str(out/'stop-tail-owner.cpp'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)


def pulse_replay(fw,files,out):
    z=(fw/'ZMotion.cpp').read_text()
    e=files['StepExecutor.cpp'];m=files['motion.cpp']
    (out/'actual.inc').write_text(z.removeprefix('#include "Repetier.h"\n')+'\n'+
        e[e.index('template <int8_t limitDirection>'):]+'\n'+
        m[m.index('void PrintLine::stopDirectMove()'):m.index('/** \\brief Check if move is new.')])
    cpp=Path(__file__).with_name('current-stop-tail-replay.cpp')
    boards={"RF1000":["DEVICE_TYPE_RF1000"],"RF2000":["DEVICE_TYPE_RF2000"],
            "RF2000v2":["DEVICE_TYPE_RF2000v2"],
            "RF2000-X44":["DEVICE_TYPE_RF2000","NUM_EXTRUDER=1","EXT0_EXTRUDER_COOLER_PIN=X44_PIN"]}
    result={}
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'pulses'
        for board,defs in boards.items():
            subprocess.run(['c++','-std=c++11','-O1','-Wall','-Wextra','-Werror','-fsanitize=undefined',
                '-fno-sanitize-recover=all','-I',str(out),'-I',str(fw),'-DMOTHERBOARD='+defs[0],
                *['-D'+flag for flag in defs[1:]],str(cpp),'-o',str(exe)],check=True)
            log=subprocess.check_output([str(exe)],text=True,timeout=30)
            print(board,log,end='',flush=True)
            result[board]=log.splitlines()
    return result


def run(fw,out,identity):
    fw=fw.resolve();out=out.resolve()
    assert fw!=out and fw not in out.parents
    out.mkdir(parents=True,exist_ok=True)
    files=candidate(fw)
    for name,s in files.items():(out/name).write_text(s)
    replay(files,out)
    result=pulse_replay(fw,files,out)
    (out/'provenance.json').write_text(json.dumps({
        'input':identity,'input_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(fw.iterdir()) if p.is_file() and p.suffix in ('.h','.cpp','.ino')},
        'candidate_sha256':{name:hashlib.sha256(s.encode()).hexdigest() for name,s in files.items()},
        'replay':result,'scope':'host actual-source pulse/ledger fixture; GPIO, timer and ramp math stubs; no linked AVR, no physical braking or P0 release'},indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    src=p.add_mutually_exclusive_group(required=True)
    src.add_argument('--source',type=Path,help='Exact already-materialized disposable firmware directory')
    src.add_argument('--archive',type=Path,help='Exact previously built candidate evidence ZIP')
    p.add_argument('--archive-sha256')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.source:
        run(a.source,a.out,{'source':str(a.source.resolve())});return
    sha=hashlib.sha256(a.archive.read_bytes()).hexdigest()
    assert sha==a.archive_sha256
    with zipfile.ZipFile(a.archive) as z, tempfile.TemporaryDirectory() as d:
        assert z.testzip() is None
        summary=json.loads(z.read('candidate-evidence/summary.json'))
        fw=Path(d)/'Repetier';fw.mkdir()
        prefix='candidate-evidence/RF1000/build/sketch/'
        for name in z.namelist():
            if not name.startswith(prefix):continue
            rel=Path(name[len(prefix):])
            if rel.parent!=Path('.') or rel.suffix not in ('.h','.cpp','.ino'):continue
            data=re.sub(r'^#line 1 "[^\n]+"\n','',z.read(name).decode(),count=1)
            (fw/rel).write_text(data)
        run(fw,a.out,{'archive_sha256':sha,'candidate_sha':summary['sha'],'build_failures':summary['failures']})

if __name__=='__main__':main()
