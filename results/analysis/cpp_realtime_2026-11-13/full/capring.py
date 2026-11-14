import csv, math, sys
for run in sys.argv[1:]:
    try: d=list(csv.DictReader(open(f'runs/{run}/driver.csv')))
    except FileNotFoundError: continue
    t0=float(d[0]['t'])
    i=next((k for k,r in enumerate(d) if r['captured']=='1'),None)
    sp=lambda a,b: math.hypot(float(b['ring_x'])-float(a['ring_x']),float(b['ring_y'])-float(a['ring_y']))/max(1e-3,float(b['t'])-float(a['t']))
    speeds=[(float(d[k]['t'])-t0, sp(d[k-10],d[k])) for k in range(10,len(d),5)]
    moving=[t for t,s in speeds if s>0.06]
    lastmove=max(moving) if moving else float('nan')
    if i is None: print(f'{run}: no capture; recorded ring last >0.06 m/s at +{lastmove:.1f} s'); continue
    print(f"{run}: capture at +{float(d[i]['t'])-t0:.1f} s, ring speed then {sp(d[max(0,i-10)],d[i]):.3f} m/s; recorded ring last >0.06 m/s at +{lastmove:.1f} s")
