import numpy as np
from common import *
for tag in ('sq5_2','sq2_5','k1_4'):
    d,s=load(tag)
    tip=d['tip']; fz=frozen_mask(d)
    sp=phase_spans(s)
    t0=[a for a,b,v in sp if v=='TAKEOFF'][0]
    a0=[a for a,b,v in sp if v in ('MATCH_VELOCITY','SETTLE_ABOVE_PICKUP')][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    # holds: runs of frozen
    runs=[]; i=0; n=len(fz)
    while i<n:
        if fz[i]:
            j=i
            while j<n and fz[j]: j+=1
            runs.append((tip[i-1,0], tip[j-1,0]-tip[i-1,0], j-i, tip[j,0] if j<n else np.nan, np.linalg.norm(tip[j,2:5]-tip[i-1,2:5]) if j<n else np.nan, tip[j-1,1]-tip[i-1,1]))
            i=j
        else: i+=1
    runs=np.array(runs)
    for lab,lo,hi in (('whole',tip[0,0],tip[-1,0]),('takeoff->taken_back',t0,a1_),('attach window',a0,a1_)):
        m=(runs[:,0]>=lo)&(runs[:,0]<hi)
        mm=(tip[:,0]>=lo)&(tip[:,0]<hi)
        r=runs[m]
        gaps=np.diff(r[:,0]) if len(r)>1 else np.array([np.nan])
        print(tag,lab,'dur %.1f s, frozen samples %.1f%%, holds %d (%.2f /s), hold median %.2f s p90 %.2f max %.2f, stamp advance in hold median %.2f s, onset-to-onset median %.2f s p10 %.2f'%(hi-lo,100*fz[mm].mean(),len(r),len(r)/(hi-lo),np.median(r[:,1]),np.percentile(r[:,1],90),r[:,1].max(),np.median(r[:,5]),np.nanmedian(gaps),np.nanpercentile(gaps,10)))
    # freeze-free windows: fraction of time t in attach window where [t, t+W] has no frozen sample
    tt=np.arange(a0,a1_-1.0,0.01)
    fr_t=tip[fz,0]
    for W in (0.3,0.5,1.0,1.3,2.0):
        ok=[]
        for x in tt:
            k=np.searchsorted(fr_t,x)
            ok.append(not (k<len(fr_t) and fr_t[k]<=x+W))
        print('   window %.1f s freeze-free from %.0f%% of start times'%(W,100*np.mean(ok)))
