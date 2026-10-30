import json, numpy as np
from common import *
for tag in ('sq5_2','sq2_5','a1','t6h_4'):
    d,s=load(tag); w=WELD[tag]
    t,rel,plate,R=plate_frame(d,w)
    fz=frozen_mask(d)
    sp=phase_spans(s)
    t0=[a for a,b,v in sp if v=='TAKEOFF'][0]
    xy=np.hypot(rel[:,0],rel[:,1]); n=rel[:,2]
    print('==',tag)
    # attach window: from first MATCH to TAKEN_BACK
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    m=(t>=a0)&(t<a1_)
    for lab,mm in (('attach window all',m),('attach window fresh',m&~fz)):
        x=xy[mm]
        print('  %s: n %d, xy median %.1f p10 %.1f p90 %.1f mm; <=24 %.0f%% <=35 %.0f%% <=45 %.0f%% <=60 %.0f%%'%(lab,len(x),1e3*np.median(x),1e3*np.percentile(x,10),1e3*np.percentile(x,90),100*np.mean(x<=.024),100*np.mean(x<=.035),100*np.mean(x<=.045),100*np.mean(x<=.06)))
    print('  frozen share in attach window %.1f%%'%(100*fz[m].mean()))
    for a,b,v in sp:
        if v in ('DESCEND_TO_ATTACHMENT','ATTACH_READY','LIFT_PROOF'):
            mm=(t>=a)&(t<b)
            if mm.sum()<2: continue
            k=np.argmin(xy[mm]); 
            print('  %-22s %6.2f+%4.2f s: xy min %.1f mm (normal there %+.1f), xy med %.1f, normal min %+.1f max %+.1f mm, in K24 %.0f%%, frozen %.0f%%'%(v,a-t0,b-a,1e3*xy[mm][k],1e3*n[mm][k],1e3*np.median(xy[mm]),1e3*n[mm].min(),1e3*n[mm].max(),100*np.mean(xy[mm]<=.024),100*fz[mm].mean()))
