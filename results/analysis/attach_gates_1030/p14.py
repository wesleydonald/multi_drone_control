import numpy as np
from common import *
for tag in ('sq5_2','sq2_5'):
    d,s=load(tag); w=WELD[tag]
    t,rel,plate,R=plate_frame(d,w)
    fz=frozen_mask(d)
    sp=phase_spans(s)
    xy=np.hypot(rel[:,0],rel[:,1])
    for ph in (('DESCEND_TO_ATTACHMENT',),('MATCH_VELOCITY',),('DESCEND_TO_ATTACHMENT','MATCH_VELOCITY')):
        m=np.zeros(len(t),bool)
        for a,b,v in sp:
            if v in ph: m|=(t>=a)&(t<b)
        m&=~fz
        x=xy[m]
        if len(x)<10: continue
        fr={r:np.mean(x<=r) for r in (0.024,0.032,0.035,0.040,0.045,0.050,0.060)}
        print(tag,'+'.join(p[:7] for p in ph),'fresh n %d median %.1f mm; '%(len(x),1e3*np.median(x))+', '.join('<=%d %.0f%%'%(1e3*r,100*v) for r,v in fr.items()),
              '| share of touches inside r_slot that are inside K 24: r32 %.0f%%, r40 %.0f%%, r45 %.0f%%'%(100*fr[0.024]/fr[0.032],100*fr[0.024]/fr[0.040],100*fr[0.024]/fr[0.045]))
    # mean offset of the tip from the plate point in MATCH (bias)
    m=np.zeros(len(t),bool)
    for a,b,v in sp:
        if v in ('MATCH_VELOCITY','DESCEND_TO_ATTACHMENT'): m|=(t>=a)&(t<b)
    m&=~fz
    print(tag,'   tip minus plate point, plate frame, MATCH+DESCEND: mean (%.1f, %.1f) mm, sd (%.1f, %.1f) mm'%(1e3*rel[m,0].mean(),1e3*rel[m,1].mean(),1e3*rel[m,0].std(),1e3*rel[m,1].std()))
