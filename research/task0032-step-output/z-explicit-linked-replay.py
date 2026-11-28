#!/usr/bin/env python3
"""Actual linked Z direction handoff counterfactual, including canceled pairs."""
import argparse,importlib.util,json,pathlib
HERE=pathlib.Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("unified_replay",HERE/"unified-z-linked-replay.py")
u=importlib.util.module_from_spec(spec);spec.loader.exec_module(u)

def trial(mod,img,owner,sign,opposite,wrong_pin,rate):
    c,move,errors,o=u.configure(mod,img,"z",rate,negative=sign<0,owner=owner)
    # Initial nominal owner is active. Deliberately inject the opposite
    # actual DIR pin as after an independent correction pulse.
    c.mem[0x10b]=(c.mem[0x10b]&~2)|(2 if (sign>0)!=wrong_pin else 0)
    if not owner:c.symput("Printer::zMotion",sign*100000,4,24)
    if opposite:u.target(c,-sign*100)
    start_motor=u.zfield(c,0);start_nominal=u.zfield(c,4)
    start_direct=u.zfield(c,20);start_applied=u.zfield(c,16)
    remaining=c.get(move+98,4,signed=True)
    physical_sign=1 if c.mem[0x10b]&2 else -1
    row=u.invoke(c,0)
    n=remaining-c.get(move+98,4,signed=True)
    assert n>0
    pulse=0;last_dir=None;last_rise=None;edges=row["edges"]
    for pin,state,t in edges:
        if pin=="DIR_Z":
            if last_rise is not None:assert t-last_rise>=4
            physical_sign=1 if state else -1;last_dir=t
        elif pin=="Z" and state:
            if last_dir is not None:assert t-last_dir>=4,(img.board,t,last_dir)
            pulse+=physical_sign;last_rise=t
    nominal=u.zfield(c,4)-start_nominal
    direct=u.zfield(c,20)-start_direct
    applied=u.zfield(c,16)-start_applied
    motor=u.zfield(c,0)-start_motor
    assert motor==pulse==nominal+direct+applied
    return dict(owner=owner,sign=sign,opposite=opposite,wrong_pin=wrong_pin,rate=rate,n=n,
        nominal=nominal,direct=direct,applied=applied,motor=motor,cycles=row["cycles"],
        mask=row["max_mask"],stack=row["stack"],dir_edges=sum(p=="DIR_Z" for p,s,t in edges))

def main():
    p=argparse.ArgumentParser()
    p.add_argument("baseline",type=pathlib.Path);p.add_argument("candidate",type=pathlib.Path)
    p.add_argument("out",type=pathlib.Path);a=p.parse_args()
    mod=u.lt.load_interpreter();result={}
    for board in u.lt.BOARDS:
        old=mod.Image(a.baseline,board);new=mod.Image(a.candidate,board)
        for img in (old,new):img.step_pins[(0x10b,1)]="DIR_Z"
        rows=[];hazards=0
        for owner in (0,1):
            for sign in (-1,1):
                for opposite in (False,True):
                    for wrong_pin in (False,True):
                        for rate in (152,7000,11520):
                            x=trial(mod,old,owner,sign,opposite,wrong_pin,rate)
                            y=trial(mod,new,owner,sign,opposite,wrong_pin,rate)
                            assert x["n"]==y["n"]
                            n=y["n"];expected=sign*n
                            assert y["nominal"]==(expected if owner else 0)
                            assert y["direct"]==(0 if owner else expected)
                            assert y["applied"]==(-expected if opposite else 0)
                            assert y["motor"]==(0 if opposite else expected)
                            if opposite:assert not y["dir_edges"]
                            if not wrong_pin:
                                for k in ("nominal","direct","applied","motor"):assert x[k]==y[k]
                            else:
                                assert x["nominal"]+x["direct"]==-expected
                                hazards+=1
                            rows.append(dict(old=x,new=y))
        result[board]=dict(cases=len(rows),constructed_old_direction_aliases=hazards,rows=rows)
        print("Z explicit linked",board,"cases",len(rows),"old aliases",hazards,
              "max new cycles",max(r["new"]["cycles"] for r in rows),
              "max new mask",max(r["new"]["mask"] for r in rows),flush=True)
    a.out.write_text(json.dumps(result,indent=2)+"\n")
if __name__=="__main__":main()
