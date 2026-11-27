#!/usr/bin/env python3
"""Read authoritative CI artifact; pair prepareMove + updateLookahead samples.

No new timing model. Pass the original Actions ZIP as argument.
"""
import json
import statistics
import sys
import zipfile

def main():
    report={}
    with zipfile.ZipFile(sys.argv[1]) as z:
        for capacity in (24,28,32):
            rows=json.loads(z.read(f"avr/{capacity}-outer-cycles.json"))
            pairs={}
            for r in rows:
                if r["stage"] not in (1,2):
                    continue
                key=(r["fixture"],r["depth"])
                stages=pairs.setdefault(key,{})
                assert r["stage"] not in stages
                stages[r["stage"]]=r["inclusive_cycles"]
            assert len(pairs)==capacity*8 and all(set(x)=={1,2} for x in pairs.values())
            totals=[(sum(v.values()),f,d) for (f,d),v in pairs.items()]
            vals=[x[0] for x in totals]
            maximum=max(totals)
            report[str(capacity)]={
                "n":len(vals),"min_cycles":min(vals),"median_cycles":statistics.median(vals),
                "mean_cycles":statistics.mean(vals),"max_cycles":maximum[0],
                "max_fixture":maximum[1],"max_depth":maximum[2],
                "over_32000_cycles":sum(v>32000 for v in vals),
                "fixture_medians":{str(f):statistics.median([v for v,ff,d in totals if ff==f]) for f in range(8)}
            }
    print(json.dumps({"schema":1,"qualification":"Paired isolated marker samples; no concurrent ISR or sustained load model","tested_artifact":sys.argv[1],"capacities":report},indent=2))
if __name__=="__main__":
    main()
