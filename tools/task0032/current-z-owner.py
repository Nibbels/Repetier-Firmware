#!/usr/bin/env python3
"""One physical Z direction/STEP owner and coherent nominal XYZ snapshot.

Preserves existing limit/overtravel/correction ledgers and cadence. This is
an independent prerequisite to combined physical timing, not a new clock.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def once(s,old,new):
    assert s.count(old)==1,(old,s.count(old))
    return s.replace(old,new)


def candidate(fw):
    h=(fw/'ZMotion.h').read_text();z=(fw/'ZMotion.cpp').read_text()
    e=(fw/'StepExecutor.cpp').read_text();bed=(fw/'BedMatrix.cpp').read_text()
    h=once(h,'''    int32_t nominalSteps() const {
        InterruptProtectedBlock lock;
        return nominalSteps_;
    }''','''    int32_t nominalSteps() const {
        InterruptProtectedBlock lock;
        return nominalStepsLocked();
    }
    // Caller holds the short XYZ snapshot transaction, not interpolation.
    int32_t nominalStepsLocked() const { return nominalSteps_; }''')
    h=once(h,'    bool stepPath(Path path);','    bool stepPath(Path path, int8_t direction);')
    h=once(h,'''    INLINE void startPulse(int8_t direction) {
        WRITE(Z_STEP_PIN, HIGH);
        motorSteps_ += direction;
    }''','''    // Timer1 is the sole physical pulse owner. Caller has closed the IRQ
    // admission window; DIR, rise and applied motor ledger are one transaction.
    INLINE void startPulse(int8_t direction) {
        if (directionIsPositive() != (direction > 0)) {
            setDirection(direction > 0);
#ifdef __AVR__
            // Preserve the audited additional two cycles on a changed DIR.
            __builtin_avr_delay_cycles(2);
#endif
        }
        WRITE(Z_STEP_PIN, HIGH);
        motorSteps_ += direction;
    }''')
    z=once(z,'''    const bool positive = pending > 0;
    if (directionIsPositive() != positive)
        setDirection(positive);
    const int8_t direction = positive ? 1 : -1;''','''    const int8_t direction = pending > 0 ? 1 : -1;''')
    z=once(z,'''bool ZMotion::stepPath(Path path) {
    const int8_t direction = directionIsPositive() ? 1 : -1;''','''bool ZMotion::stepPath(Path path, int8_t direction) {''')
    z=once(z,'''        HAL::allowInterrupts();
        startPulse(direction);
        HAL::forbidInterrupts();
        scanSteps_ += direction;''','''        HAL::allowInterrupts();
        asm volatile("nop" ::: "memory");
        HAL::forbidInterrupts();
        startPulse(direction);
        scanSteps_ += direction;''')
    z=once(z,'''        // endstop check and pulse start, as they do for X and Y.''','''        // endstop check. Close the window before physical DIR/STEP/ledger.''')
    z=once(z,'''        if (result == Step::Pulse)
            startPulse(direction);
        HAL::forbidInterrupts();''','''        HAL::forbidInterrupts();
        if (result == Step::Pulse)
            startPulse(direction);''')
    e=once(e,'Printer::zMotion.stepPath(path)','Printer::zMotion.stepPath(path, move->isZPositiveMove() ? 1 : -1)')
    bed=once(bed,'''    return getZMatrixDepth(
        Printer::xyMotion.nominalSteps(X_AXIS),
        Printer::xyMotion.nominalSteps(Y_AXIS));''','''    int32_t x,y;
    {
        InterruptProtectedBlock lock;
        x=Printer::xyMotion.nominalSteps(X_AXIS);
        y=Printer::xyMotion.nominalSteps(Y_AXIS);
    }
    return getZMatrixDepth(x,y);''')
    bed=once(bed,'''        const auto& layers = Printer::layers;
        const BedCompensation::Geometry geometry{
            Printer::zMotion.nominalSteps(),''','''        int32_t x,y,z;
        {
            InterruptProtectedBlock lock;
            x=Printer::xyMotion.nominalSteps(X_AXIS);
            y=Printer::xyMotion.nominalSteps(Y_AXIS);
            z=Printer::zMotion.nominalStepsLocked();
        }
        const auto& layers = Printer::layers;
        const BedCompensation::Geometry geometry{
            z,''')
    bed=once(bed,'''BedCompensation::calculate(geometry, [] { return getZMatrixDepth_CurrentXY(); })''','''BedCompensation::calculate(geometry, [x,y] { return getZMatrixDepth(x,y); })''')
    return {'ZMotion.h':h,'ZMotion.cpp':z,'StepExecutor.cpp':e,'BedMatrix.cpp':bed}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();fw=a.source.resolve();out=a.out.resolve()
    assert fw!=out and fw not in out.parents
    files=candidate(fw);out.mkdir(parents=True,exist_ok=True)
    for name,s in files.items():(out/name).write_text(s)
    (out/'provenance.json').write_text(json.dumps({
        'input_sha256':{name:hashlib.sha256((fw/name).read_bytes()).hexdigest() for name in files},
        'candidate_sha256':{name:hashlib.sha256(s.encode()).hexdigest() for name,s in files.items()},
        'scope':'explicit logical direction, common physical edge owner, coherent XYZ sample; no common timing/admission claim'},indent=2)+'\n')
    (out/'actual-z-motion.inc').write_text(files['ZMotion.cpp'].removeprefix('#include "Repetier.h"\n'))
    fixture=Path(__file__).with_name('current-z-owner-replay.cpp')
    boards={'RF1000':['DEVICE_TYPE_RF1000'],'RF2000':['DEVICE_TYPE_RF2000'],
        'RF2000v2':['DEVICE_TYPE_RF2000v2'],
        'RF2000-X44':['DEVICE_TYPE_RF2000','NUM_EXTRUDER=1','EXT0_EXTRUDER_COOLER_PIN=X44_PIN']}
    with tempfile.TemporaryDirectory() as d:
        exe=Path(d)/'replay'
        for board,defs in boards.items():
            subprocess.run(['c++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-fsanitize=undefined',
                '-fno-sanitize-recover=all','-I',str(out),'-I',str(fw),'-DMOTHERBOARD='+defs[0],
                *['-D'+flag for flag in defs[1:]],str(fixture),'-o',str(exe)],check=True)
            log=subprocess.check_output([str(exe)],text=True,timeout=90)
            print(board,log,end='',flush=True);(out/(board+'.log')).write_text(log)

if __name__=='__main__':main()
