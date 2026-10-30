import numpy as np
from common import *
for tag in ('sq5_2','sq2_5','k1_4'):
    d,s=load(tag); w=WELD[tag]
    t,rel,plate,R=plate_frame(d,w)
    fz=frozen_mask(d)
    sp=phase_spans(s)
    t0=[a for a,b,v in sp if v=='TAKEOFF'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    m=(t>=t0)&(t<a1_)
    xy=np.hypot(rel[:,0],rel[:,1])
    dist=np.linalg.norm(rel,axis=1)
    print(tag)
    for lo,hi in ((0,0.2),(0.2,0.35),(0.35,0.6),(0.6,1.0),(1.0,2.0),(2.0,5)):
        mm=m&(dist>=lo)&(dist<hi)
        if mm.sum()>20: print('   tip %.2f-%.2f m from plate 9: frozen %.0f%% of %d samples'%(lo,hi,100*fz[mm].mean(),mm.sum()))
    # per phase
    for a,b,v in sp:
        if a>=t0 and a<a1_ and b-a>0.5:
            mm=(t>=a)&(t<b)
            print('   %-24s %6.2f s: frozen %.0f%%'%(v,b-a,100*fz[mm].mean()))
