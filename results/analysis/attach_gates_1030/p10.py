import sys, math
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import *
from collections import Counter
tag='sq2_5'; d,s=load(tag); w=np.asarray(WELD[tag])
tip=d['tip']; ring=d['ring']
fz=frozen_mask(d)
sp=phase_spans(s); a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
ir=hold_idx(ring[:,0],tip[:,0]); Rk=np.array([quat_rot(q) for q in ring[ir,5:9]]); Pk=ring[ir,2:5]+np.einsum('kij,j->ki',Rk,w)
cfg=ComotionConfig(contact_xy=0.024,lift_fallback_s=0.0,taut_fallback=1.041,gap_s=0.35)
c=np.array([0.01,0,0.002])
tot=Counter(); 
for tc in np.arange(a0+0.5,a1_-8,0.5):
    p=ComotionProof(cfg); p.magnet(True)
    k=np.searchsorted(tip[:,0],tc); tcon=None; meas=None
    while tip[k,0]<tc+8:
        tk=tip[k,0]; true=Pk[k]+Rk[k]@c; fr=fz[k] and meas is not None
        tm=meas if fr else true; meas=tm
        if not fr:
            ev=p.step(tk,tm,Pk[k],body=true+np.array([0,0,1.04]),fresh=True,rotation=Rk[k],lift_done=(tcon is not None and tk>=tcon+2.0))
            if ev=='contact': tcon=tk
            if p.state=='prove' and p.t_lift is not None:
                cf=p.config
                ma=np.mean([x[1] for x in p._win],axis=0); normal=ma[2]
                btt=np.median([x[3] for x in p._win])
                conds={'drift':p.drift<=cf.d_ok,'speed':math.isfinite(p.rel_speed) and p.rel_speed<=cf.speed_ok,'normal':abs(normal)<=cf.normal_ok,'taut':btt>=p.l_hang-cf.taut_margin,'moved':p.comotion>=cf.comotion_range or p.rise>=cf.h_plate,'follows':p.drift<=cf.follow_ratio*max(p.comotion,abs(p.rise))}
                for kk,v in conds.items():
                    if not v: tot[kk]+=1
                tot['n']+=1
            if ev in ('proven','failed'): break
        else:
            p.tick(tk)
        k+=1
print(tot)
