import numpy as np
from common import *
for tag in ('sq5_2','sq2_5','k1_4','t6h_4'):
    d,s=load(tag); w=WELD[tag]
    t,rel,plate,R=plate_frame(d,w)
    fz=frozen_mask(d)
    body=d['body']; ib=hold_idx(body[:,0],t)
    btt=np.linalg.norm(d['tip'][:,2:5]-body[ib,2:5],axis=1)
    sp=phase_spans(s)
    t0=[a for a,b,v in sp if v=='TAKEOFF'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    m=(t>=t0+3)&(t<a1_)&(rel[:,2]>0.05)
    for lab,mm in (('all',m),('fresh',m&~fz),('frozen',m&fz)):
        b=btt[mm]
        if len(b): print(tag,'free-hang body-to-tip %-6s n %5d median %.4f p5 %.4f p95 %.4f sd %.4f'%(lab,len(b),np.median(b),np.percentile(b,5),np.percentile(b,95),b.std()))
    # rolling 2 s medians spread (what l_hang would read at different contact times)
    tt=t[m]; bb=btt[m]
    meds=[np.median(bb[(tt>x-2)&(tt<=x)]) for x in tt[::50] if ((tt>x-2)&(tt<=x)).sum()>50]
    print(tag,'  2 s rolling l_hang median %.4f, range %.4f-%.4f'%(np.median(meds),min(meds),max(meds)))
