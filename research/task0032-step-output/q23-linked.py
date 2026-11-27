#!/usr/bin/env python3
"""Compare actual linked Q23 counterfactual with canonical baseline.
Temporary source copy only, no production changes. Compile candidate only.
Compare ordered edges, logical state, same-invocation pulse widths and cycles.
No concurrent ISR dispatch, multi-call timing or physical qualification.
"""
import argparse, collections, difflib, hashlib, json, pathlib, shutil, struct, subprocess, tempfile, zipfile
import linked_timer1 as lt

HERE=pathlib.Path(__file__).resolve().parent
FW=lt.ROOT/"Repetier"
Q=1<<23
FLAGS={
    "RF1000":"-DMOTHERBOARD=DEVICE_TYPE_RF1000",
    "RF2000":"-DMOTHERBOARD=DEVICE_TYPE_RF2000",
    "RF2000v2":"-DMOTHERBOARD=DEVICE_TYPE_RF2000v2",
    "RF2000-X44":"-DMOTHERBOARD=DEVICE_TYPE_RF2000 -DNUM_EXTRUDER=1 -DEXT0_EXTRUDER_COOLER_PIN=X44_PIN",
}

def replace(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)

def modify(dst,padded):
    shutil.copytree(FW,dst)
    changes={}
    h=(HERE/"ecmp-q23.h").read_text().replace("ExtrusionCompensationQ23","ExtrusionCompensation")
    changes["ExtrusionCompensation.h"]=h
    p="BedMatrix.cpp"
    changes[p]=replace((FW/p).read_text(),"const float ratio = ExtrusionCompensation::prepareRatio","const auto ratio = ExtrusionCompensation::prepareRatio")
    p="StartLine.cpp"
    s=replace((FW/p).read_text(),"float ecmpRemainder;","ExtrusionCompensation::Units ecmpRemainder;")
    s=replace(s,"extrusionCompensation.remainder()","extrusionCompensation.remainderUnits()")
    changes[p]=replace(s,"extrusionCompensation.restoreRemainder(","extrusionCompensation.restoreRemainderUnits(")
    if padded:
        # Deliberately conservative comparator: explicit 31-cycle dwell for
        # physical E rising edge (base and extra) and the extra pulse low gap.
        # Not a selected final implementation. Preserve old edge order.
        p="StepExecutor.cpp";s=(FW/p).read_text()
        s=replace(s,"                    Extruder::step();\n                }",
                  "                    Extruder::step();\n                    __builtin_avr_delay_cycles(31);\n                }")
        s=replace(s,"                        Extruder::unstep();\n                        Printer::extrusionCompensation.consumeExtraStep(forward);\n                        Extruder::step();",
                  "                        Extruder::unstep();\n                        Printer::extrusionCompensation.consumeExtraStep(forward);\n                        __builtin_avr_delay_cycles(31);\n                        Extruder::step();\n                        __builtin_avr_delay_cycles(31);")
        changes[p]=s
    diff=[]
    for p,s in changes.items():
        diff.extend(difflib.unified_diff((FW/p).read_text().splitlines(True),s.splitlines(True),fromfile="a/Repetier/"+p,tofile="b/Repetier/"+p))
        (dst/p).write_text(s)
    return "".join(diff)

def build(dst,out,padded,tools):
    tag="guarded" if padded else "raw"
    root=out/tag;root.mkdir()
    (root/"candidate.diff").write_text(modify(dst,padded))
    archive=root/"candidate.zip"
    meta={"classification":"task_private_counterfactual_not_shipping","variant":tag,
          "source_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in dst.glob("*") if p.is_file()},
          "flags":FLAGS,"elf":{}}
    with zipfile.ZipFile(archive,"w",compression=zipfile.ZIP_DEFLATED) as z:
        for b,flags in FLAGS.items():
            d=root/b;d.mkdir()
            buildpath=d/"build";buildpath.mkdir()
            cmd=["arduino-cli","compile","--fqbn","arduino:avr:mega:cpu=atmega2560","--build-path",str(buildpath),
                 "--build-property","compiler.cpp.extra_flags="+flags,
                 "--build-property",f'compiler.c.elf.extra_flags=-Xlinker -Map -Xlinker "{buildpath}/firmware.map" -Wl,--cref',
                 str(dst),"--verbose"]
            with (d/"compile.log").open("w") as log:
                subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
            elf=buildpath/"Repetier.ino.elf"
            meta["elf"][b]=hashlib.sha256(elf.read_bytes()).hexdigest()
            for name,args in [("symbols.txt",["avr-nm","-C","-S","-n",str(elf)]),("disassembly.txt",["avr-objdump","-d","-C","-z",str(elf)]),("size.txt",["avr-size","-A",str(elf)])]:
                raw=subprocess.check_output([str(tools/args[0]),*args[1:]])
                z.writestr(b+"/"+name,raw)
                (d/name).write_bytes(raw)
            for name in ("Repetier.ino.hex","Repetier.ino.elf","firmware.map"):
                z.write(buildpath/name,b+"/build/"+name)
            print("built",tag,b,meta["elf"][b],flush=True)
        z.writestr("summary.json",json.dumps(meta))
    return archive

def normalize(s,q23):
    s=dict(s)
    raw=bytes.fromhex(s.pop("ecmp_bytes"))
    s["ecmp_units"]=list(struct.unpack("<ii",raw)) if q23 else [round(x*Q) for x in struct.unpack("<ff",raw)]
    return s

def compare(archive,baseline,out,tag):
    mod=lt.load_interpreter()
    result={}
    for b in lt.BOARDS:
        img=mod.Image(archive,b);img.q23=True
        img.step_pins.update({(0x31,1):"DIR_X",(0x31,7):"DIR_Y",(0x10b,1):"DIR_Z",(0x22,6):"DIR_E0",(0x28,3):"DIR_E1"})
        if b=="RF1000":img.step_pins.pop((0x28,3),None)
        rows=[json.loads(l) for l in (baseline/(b+".jsonl")).read_text().splitlines()]
        summary={"matched":0,"failed":0,"mismatch_examples":[],"width_min":{},"baseline_width_min":{},"short_pulses":0,"baseline_short_pulses":0,"short_examples":[],"max_mask":0,"max_baseline_mask":0,"worst_mask_examples":[],"configured_E_primary":[]}
        trace_file=out/tag/(b+".jsonl")
        with trace_file.open("w") as f:
            for a in rows:
                if a["classification"]!="isolated_linked_instruction_model":continue
                try:
                    c=lt.execute(mod,img,a["fixture"],a["mode"],a["case"])
                    assert c["classification"]==a["classification"]
                    differences=[]
                    for field in ("before","after"):
                        old,new=normalize(a[field],False),normalize(c[field],True)
                        differences.extend(field+"."+key for key in old if old[key]!=new[key])
                    for field in ("physical_rising_edges","return_interval"):
                        if a[field]!=c[field]:differences.append(field)
                    if [e[:2] for e in a["step_dir_edges"]]!=[e[:2] for e in c["step_dir_edges"]]:
                        differences.append("ordered_gpio_edges")
                    if differences:raise ValueError(differences)
                    summary["matched"]+=1
                    for src,prefix in ((a,"baseline_"),(c,"")):
                        for pin,edge,cycles in src["edge_phase_widths"]:
                            if pin.startswith("DIR"):continue
                            key=pin+":"+edge
                            d=summary[prefix+"width_min"]
                            d[key]=min(d.get(key,cycles),cycles)
                            if cycles<31:
                                summary[prefix+"short_pulses"]+=1
                                if not prefix and len(summary["short_examples"])<8:
                                    summary["short_examples"].append([src["case"],src["fixture"],pin,edge,cycles])
                    summary["max_baseline_mask"]=max(summary["max_baseline_mask"],a["max_masked_cycles"])
                    if c["max_masked_cycles"]>summary["max_mask"]:
                        summary["max_mask"]=c["max_masked_cycles"]
                        summary["worst_mask_examples"]=[c["case"],c["fixture"]]
                    if c["case"]=="Eprimary-config-max":
                        summary["configured_E_primary"].append({"fixture":c["fixture"],
                          "base":[a["exclusive_cycles"],a["max_masked_cycles"]],
                          "candidate":[c["exclusive_cycles"],c["max_masked_cycles"]]})
                    c["paired_baseline_cycles"]=a["exclusive_cycles"];c["paired_baseline_mask"]=a["max_masked_cycles"]
                    f.write(json.dumps(c,separators=(",",":"))+"\n")
                except Exception as e:
                    summary["failed"]+=1
                    if len(summary["mismatch_examples"])<8:summary["mismatch_examples"].append([a["case"],a["mode"],a["fixture"],repr(e)])
        result[b]=summary
        print(tag,b,json.dumps(summary,separators=(",",":")),flush=True)
    (out/tag/"comparison.json").write_text(json.dumps(result,indent=2)+"\n")
    return result

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--baseline",type=pathlib.Path,required=True)
    p.add_argument("--out",type=pathlib.Path,required=True)
    p.add_argument("--tools",type=pathlib.Path,required=True)
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    all_results={}
    for padded in (False,True):
        tag="guarded" if padded else "raw"
        with tempfile.TemporaryDirectory(prefix="q23-") as tmp:
            archive=build(pathlib.Path(tmp)/"Repetier",args.out,padded,args.tools)
            all_results[tag]=compare(archive,args.baseline,args.out,tag)
    # Raw variant is exploratory and may violate pulse timing. Guarded
    # candidate must preserve logical state and meet all sampled within-call
    # pulse minima before even considering it for integrated replay.
    for variant,rows in all_results.items():
        for board,s in rows.items():
            assert s["failed"]==0,(variant,board,s["mismatch_examples"])
            if variant=="guarded":assert s["short_pulses"]==0,(board,s["short_examples"])
if __name__=="__main__":main()
