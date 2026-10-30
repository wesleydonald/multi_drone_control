import numpy as np
from common import *
# ring motion stats in the attach window: plate point displacement over 0.15 s and 0.2 s, plate z speed
for tag in ('sq5_2','sq2_5'):
    d,s=load(tag); w=WELD[tag]
    sp=phase_spans(s)
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    ring=d['ring']
    m=(ring[:,0]>=a0)&(ring[:,0]<a1_)
    rr=ring[m]
    P=np.array([rr[k,2:5]+quat_rot(rr[k,5:9])@np.asarray(w) for k in range(len(rr))])
    t=rr[:,0]
    for H in (0.10,0.14,0.20):
        j=np.searchsorted(t,t+H); ok=j<len(t)
        dP=P[j[ok]]-P[ok]
        dxy=np.hypot(dP[:,0],dP[:,1]); dz=dP[:,2]; d3=np.linalg.norm(dP,axis=1)
        print(tag,'plate move over %.2f s: 3-D median %.1f p90 %.1f mm; >6 mm %.0f%%; z: |dz| median %.1f p90 %.1f, dz > +20 mm %.1f%%, dz < -20 %.1f%%'%(H,1e3*np.median(d3),1e3*np.percentile(d3,90),100*np.mean(d3>.006),1e3*np.median(abs(dz)),1e3*np.percentile(abs(dz),90),100*np.mean(dz>.02),100*np.mean(dz<-.02)))
    v=np.gradient(P,t,axis=0)
    print(tag,'plate speed median %.3f p90 %.3f m/s, vz |median| %.3f p90 %.3f'%(np.median(np.linalg.norm(v,axis=1)),np.percentile(np.linalg.norm(v,axis=1),90),np.median(abs(v[:,2])),np.percentile(abs(v[:,2]),90)))
    # ring frozen share
    same=np.all(rr[1:,2:9]==rr[:-1,2:9],axis=1); print(tag,'ring repeated in window %.1f%%'%(100*same.mean()))
    body=d['body']; mb=(body[:,0]>=a0)&(body[:,0]<a1_); bb=body[mb]
    sameb=np.all(bb[1:,2:5]==bb[:-1,2:5],axis=1); print(tag,'body repeated %.1f%%'%(100*sameb.mean()))
    # QUAD4 body xy speed
    vb=np.gradient(bb[:,2:5],bb[:,0],axis=0); sxy=np.hypot(vb[:,0],vb[:,1])
    print(tag,'QUAD4 body xy speed median %.3f p90 %.3f p99 %.3f max %.3f; >0.25 %.1f%% >0.35 %.1f%%'%(np.median(sxy),np.percentile(sxy,90),np.percentile(sxy,99),sxy.max(),100*np.mean(sxy>.25),100*np.mean(sxy>.35)))
