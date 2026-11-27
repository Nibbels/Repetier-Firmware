#!/usr/bin/env python3
"""Independent linked explicit DIR and coherent XYZ sample counterfactual.
Keep whole-move Z reservation: combined physical acceleration/debt and
admission still unproven. No production source writes.
"""
import argparse,difflib,importlib.util,pathlib,tempfile
HERE=pathlib.Path(__file__).resolve().parent
def load(name,file):
    spec=importlib.util.spec_from_file_location(name,HERE/file)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
d=load("direct","direct-z-source.py")
h=load("hop","z-hop-reversibility.py")
q=d.q

def modify(dst,tag):
    assert tag in ("explicit-z","explicit-z-boundary")
    d.modify(dst,"direct-z")
    zh=dst/"ZMotion.h";zc=dst/"ZMotion.cpp"
    header,source=h.patch(zh.read_text(),zc.read_text())
    zh.write_text(header);zc.write_text(source)
    p=dst/"StepExecutor.cpp";s=p.read_text()
    s=q.replace(s,"Printer::zMotion.stepPath(path)",
        "Printer::zMotion.stepPath(path, move->isZPositiveMove() ? 1 : -1)")
    if tag=="explicit-z-boundary":
        # Direct/queue transaction is complete, including STEP fall and
        # shared remaining counter update. The nested stop writer can only
        # shorten remaining to 32, greater than the maximum 10 packed steps
        # (5000 / 512 rounded up); revalidate this condition in linked tests.
        # AVR SEI delays admission by one instruction. NOP ensures CLI
        # cannot immediately close the window before a pending IRQ enters.
        s=q.replace(s,"        Printer::endXYZSteps();\n    }                       // for loop",
            """        Printer::endXYZSteps();
        if (!forQueue) {
            HAL::allowInterrupts();
            asm volatile("nop" ::: "memory");
            HAL::forbidInterrupts();
        }
    }                       // for loop""")
    p.write_text(s)
    # Keep interpolation outside the lock. Other callers of CurrentXY
    # (e.g. scan) also get a coherent XY pair. The layer state is still
    # the existing foreground planned context, not an execution snapshot.
    p=dst/"BedMatrix.cpp";s=p.read_text()
    s=q.replace(s,"""    return getZMatrixDepth(
        Printer::xyMotion.nominalSteps(X_AXIS),
        Printer::xyMotion.nominalSteps(Y_AXIS));""",
        """    int32_t x,y;
    {
        InterruptProtectedBlock lock;
        x=Printer::xyMotion.nominalSteps(X_AXIS);
        y=Printer::xyMotion.nominalSteps(Y_AXIS);
    }
    return getZMatrixDepth(x,y);""")
    s=q.replace(s,"""        const auto& layers = Printer::layers;
        const BedCompensation::Geometry geometry{
            Printer::zMotion.nominalSteps(),""",
        """        int32_t x,y,z;
        {
            InterruptProtectedBlock lock;
            x=Printer::xyMotion.nominalSteps(X_AXIS);
            y=Printer::xyMotion.nominalSteps(Y_AXIS);
            z=Printer::zMotion.nominalSteps();
        }
        const auto& layers = Printer::layers;
        const BedCompensation::Geometry geometry{
            z,""")
    s=q.replace(s,"""BedCompensation::calculate(geometry, [] { return getZMatrixDepth_CurrentXY(); })""",
        """BedCompensation::calculate(geometry, [x,y] { return getZMatrixDepth(x,y); })""")
    p.write_text(s)
    diff=[]
    for p in sorted(dst.glob("*")):
        if not p.is_file():continue
        old=(q.FW/p.name).read_bytes() if (q.FW/p.name).is_file() else b""
        new=p.read_bytes()
        if old!=new:
            diff.extend(difflib.unified_diff(old.decode().splitlines(True),new.decode().splitlines(True),
                fromfile="a/Repetier/"+p.name,tofile="b/Repetier/"+p.name))
    return "".join(diff)
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--out",type=pathlib.Path,required=True)
    p.add_argument("--tools",type=pathlib.Path,required=True)
    p.add_argument("--boundary",action="store_true")
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    q.modify=modify
    with tempfile.TemporaryDirectory(prefix="explicit-z-") as tmp:
        q.build(pathlib.Path(tmp)/"Repetier",a.out,"explicit-z-boundary" if a.boundary else "explicit-z",a.tools)
if __name__=="__main__":main()
