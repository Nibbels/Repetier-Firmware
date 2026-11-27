#!/usr/bin/env python3
"""Measure coalescence sensitivity on one linked image per board.
The guard values are diagnostic inputs, not production timing budgets.
No nested interrupts, full planner workload or mechanical acceleration proof.
"""
import argparse,importlib.util,json,pathlib
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("unified_replay",HERE/"unified-z-linked-replay.py")
u=importlib.util.module_from_spec(spec);spec.loader.exec_module(u)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("archive",type=pathlib.Path);p.add_argument("out",type=pathlib.Path)
    a=p.parse_args();mod=u.lt.load_interpreter();rows=[]
    # Representative single/dual boards for the first policy sensitivity;
    # all four boards remain required for a selected fixed policy.
    for board in ("RF1000","RF2000"):
        img=mod.Image(a.archive,board)
        for kind,rate in (("idle",5333),("xy",152),("xy",9143),("xye",15238)):
            for gap in (512,1600,2048,3072):
                row=u.run_scenario(mod,img,kind,rate,"step",merge_guard=gap)
                assert row["physical_correction_rises"]==640 and row["end_applied"]==0
                assert row["correction_only"]+row["merged_correction"]==640
                assert row["min_gap"]>=1390-1 # nominal 9 mm/s cap, ceil(16MHz/11520)
                row["board"]=board;rows.append(row)
                print("merge-gap",board,json.dumps(row,separators=(",",":")),flush=True)
    a.out.write_text(json.dumps({"schema":1,"classification":"isolated linked instruction replay; guard sensitivity, not production timing", "rows":rows},indent=2)+"\n")

if __name__=="__main__":main()
