#!/usr/bin/env python3
"""Compare typed G1 and host G1 against the pre-conversion motion contract."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]/"Repetier"
source=(root/"Printer.cpp").read_text()
body=source[source.index("inline bool isExtrusionAllowed("):source.index("/** Move to machine coordinates")]
harness=r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <cstdint>
#include "MoveTarget.h"
#include "GCodeModes.h"
#define EXTRUDE_MAXLENGTH 100
#define PSTR(x) x
enum{X_AXIS,Y_AXIS,Z_AXIS,E_AXIS};
static bool calibrated,inhibited,publishes,xyzPublished;
static unsigned planned,admitted,diagnostics;
static float plannedFeed,flow,digit;
struct GCode {
    unsigned mask; float X,Y,Z,E,F; bool internalCommand;
    bool hasX(){return mask&1;}bool hasY(){return mask&2;}
    bool hasZ(){return mask&4;}bool hasE(){return mask&8;}
    bool hasF(){return mask&16;}bool hasNoXYZ(){return !(mask&7);}
};
namespace Printer {
    MoveTarget moveTarget;GCodeModes gcodeModes;float feedrate;int feedrateMultiply;
    float requestedFeedrate(){return feedrate;}
    float flowMultiplier(){return flow;}bool heatingInhibited(){return inhibited;}
    struct {float extrusionFactor(){return digit;}} digitCompensation;
    struct {void admitPrintMotion(){++admitted;}} printJob;
    void setFeedrate(float);bool queueGCodeCoordinates(GCode*);
    bool oldQueueGCodeCoordinates(GCode*);bool queueInternalAxisMove(uint8_t,float,float=0);
}
namespace Com {void printFLN(const char*){++diagnostics;}}
namespace Extruder {bool allHeatersCalibrated(){return calibrated;}}
namespace PrintLine {
    bool prepareQueueMove(bool a,bool b,float f,bool xyz){
        assert(!a&&b);++planned;plannedFeed=f;xyzPublished=xyz;return publishes;
    }
}
''' + body + r'''
// Independent reference: old host implementation, before factoring.
bool Printer::oldQueueGCodeCoordinates(GCode* com) {
    if (com->hasE() && !Extruder::allHeatersCalibrated()) {
        Com::printFLN(PSTR("Error: PID tune required before printing"));
        return false;
    }
    const bool isXYZMove = !com->hasNoXYZ();
    if (com->hasX())
        moveTarget.setFromGCodeMM(X_AXIS, com->X, gcodeModes.relativeXYZ());
    if (com->hasY())
        moveTarget.setFromGCodeMM(Y_AXIS, com->Y, gcodeModes.relativeXYZ());
    if (com->hasZ())
        moveTarget.setFromGCodeMM(Z_AXIS, com->Z, gcodeModes.relativeXYZ());
    float e = com->E;
    bool isEMove = com->hasE() && isExtrusionAllowed(e);
    if (isEMove) {
        if (gcodeModes.relativeE())
            moveTarget.addMM(E_AXIS, e);
        else
            moveTarget.setMachineMM(E_AXIS, e);
    }
    if (com->hasF())
        Printer::setFeedrate(com->F);
    if (!isXYZMove && !isEMove)
        return false;
    const bool published = PrintLine::prepareQueueMove(false, true, Printer::feedrate, isXYZMove);
    if (published && !com->internalCommand)
        printJob.admitPrintMotion();
    return published;
}
struct Result {float coords[4],rem[4],feed,planFeed;unsigned p,a,d;bool xyz,ret;};
static void reset(bool rel,bool eRel,int multiplier){
    for(uint8_t a=0;a<4;++a){
        Printer::moveTarget.setMachineMM(a,10.12345f+a,.00000003f);
        if(a<3)Printer::moveTarget.setWorkOffsetMM(a,1.25f*a);
    }
    Printer::gcodeModes.setRelativeXYZ(rel);
    Printer::gcodeModes.setRelativeEOverride(eRel);
    Printer::feedrate=11.5f;Printer::feedrateMultiply=multiplier;
    planned=admitted=diagnostics=0;plannedFeed=0;xyzPublished=false;
}
static Result capture(bool ret){
    Result r{};
    for(uint8_t a=0;a<4;++a){
        r.coords[a]=Printer::moveTarget.machineMM(a);
        r.rem[a]=Printer::moveTarget.remainderMM(a);
    }
    r.feed=Printer::feedrate;r.planFeed=plannedFeed;
    r.p=planned;r.a=admitted;r.d=diagnostics;r.xyz=xyzPublished;r.ret=ret;return r;
}
static void equal(Result a,Result b){
    for(unsigned i=0;i<4;++i){
        assert(a.coords[i]==b.coords[i]);assert(a.rem[i]==b.rem[i]);
    }
    assert(a.feed==b.feed&&a.planFeed==b.planFeed&&a.p==b.p&&a.a==b.a&&
           a.d==b.d&&a.xyz==b.xyz&&a.ret==b.ret);
}
int main(){
    unsigned hostCases=0,typedCases=0;
    for(bool rel:{false,true})for(bool eRel:{false,true})
    for(bool cal:{false,true})for(bool inhibit:{false,true})
    for(bool pub:{false,true})for(int mult:{30,100,170})
    for(float scale:{.5f,1.f,1.75f})
    for(float val:{-120.f,-90.f,-.5f,0.f,.008f,90.f,245.f})
    for(float f:{0.f,100.f,500.f,4800.f}){
        calibrated=cal;inhibited=inhibit;publishes=pub;flow=scale;digit=.9f;
        for(unsigned mask=0;mask<32;++mask)for(bool internal:{false,true}){
            GCode c{mask,val,val,val,val,f,internal};
            reset(rel,eRel,mult);auto old=capture(Printer::oldQueueGCodeCoordinates(&c));
            reset(rel,eRel,mult);equal(old,capture(Printer::queueGCodeCoordinates(&c)));
            ++hostCases;
        }
        for(uint8_t axis=0;axis<4;++axis){
            GCode c{(1u<<axis)|(f?16u:0u),val,val,val,val,f,true};
            reset(rel,eRel,mult);auto old=capture(Printer::oldQueueGCodeCoordinates(&c));
            reset(rel,eRel,mult);equal(old,capture(Printer::queueInternalAxisMove(axis,val,f)));
            ++typedCases;
        }
    }
    assert(hostCases==516096 && typedCases==32256);
    std::printf("%u host + %u typed motion comparisons PASS\n",hostCases,typedCases);
}
'''
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/"move.cpp";exe=Path(d)/"move"
    p.write_text(harness)
    subprocess.run(["g++","-std=c++11","-O2","-Wall","-Wextra","-Werror",
                    "-I",str(root),str(p),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
