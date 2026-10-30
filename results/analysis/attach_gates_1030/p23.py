"""Stricter variant (Tejen: stale evidence earns no dwell): repeats skipped AND the proof's clock advances only over
fresh samples (each hold's time removed), with the 0.5 s speed window and d_ok 8 mm. Synthetic attach + noise and f4."""
import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import *
SD=np.array([0.0013,0.0012,0.0038])
def run_syn(tag, off=(0.01,0,0.002), step=0.25, horizon=20.0, fresh_clock=True, seed=1, d_ok=0.008, win=0.5, gap=0.1):
    rng=np.random.default_rng(seed)
    d,s=load(tag); w=np.asarray(WELD[tag]); tip=d['tip']; ring=d['ring']; fz=frozen_mask(d)
    sp=phase_spans(s); a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    ir=hold_idx(ring[:,0],tip[:,0]); Rk=np.array([quat_rot(q) for q in ring[ir,5:9]]); Pk=ring[ir,2:5]+np.einsum('kij,j->ki',Rk,w)
    wi=np.where((tip[:,0]>=a0)&(tip[:,0]<a1_))[0]
    cfg=ComotionConfig(contact_xy=0.024,lift_fallback_s=0.0,taut_fallback=1.041,gap_s=gap,speed_window_s=win,d_ok=d_ok)
    c=np.asarray(off); out=[]
    for tc in np.arange(tip[wi[0],0],tip[wi[-1],0],step):
        p=ComotionProof(cfg); p.magnet(True); meas=None; tcon=None; res=None; first=None; nf=0
        k=np.searchsorted(tip[:,0],tc); tt=tc; held=0.0; last_fresh=None
        while tt<tc+horizon:
            kk=wi[0]+((k-wi[0])%len(wi)); true=Pk[kk]+Rk[kk]@c
            fr=fz[kk] and meas is not None
            tm=meas if fr else true+rng.normal(0,1,3)*SD; meas=tm
            k+=1; kn=wi[0]+((k-wi[0])%len(wi)); dt=tip[kn,0]-tip[kk,0]
            if dt<=0 or dt>0.2: dt=0.0107
            if not fr:
                if last_fresh is not None and fresh_clock: held += max(0.0, tt-last_fresh-0.0107)
                last_fresh=tt
                v=tt-held
                ev=p.step(v,tm,Pk[kk],body=true+np.array([0,0,1.04]),fresh=True,rotation=Rk[kk],lift_done=(tcon is not None and tt>=tcon+2.0))
                if ev is None: ev=p.tick(v)
                if ev=='contact': tcon=tt
                if ev=='failed' and tcon is not None:
                    nf+=1; tcon=None; first=first or 'fail '+p.last_fail['reason']
                    if nf>=3: res='gave up'; break
                if ev=='proven': res='proven'; first=first or 'proven'; break
            tt+=dt
        out.append((res,first,tt-tc))
    n=len(out); return n, sum(o[1]=='proven' for o in out), sum(o[0]=='proven' for o in out), [o[2] for o in out if o[0]=='proven']
for tag in ('sq5_2','sq2_5'):
    n,f1,pr,tp=run_syn(tag)
    print('%s fresh-time clock, skip, window .5, d_ok 8 mm: first proof %.0f%%, within 3 tries %.0f%%, median %.1f s'%(tag,100*f1/n,100*pr/n,np.median(tp)))
