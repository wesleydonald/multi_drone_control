import sys, math
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
import p16
from collections import Counter
for tag in ('f4',):
    P=p16.prep(tag); a=P['air']; t=P['t']; w=P['w0']
    for skip,gap,win in ((False,0.1,0.2),(True,0.35,0.5)):
        cfg=ComotionConfig(contact_xy=0.024,lift_fallback_s=0.0,taut_fallback=0.0,gap_s=gap,speed_window_s=win)
        tot=Counter(); vals={'drift':[],'speed':[],'normal':[]}
        for tc in np.arange(t[a[0]]+0.5,t[a[-1]]-8,1.0):
            p=ComotionProof(cfg); p.magnet(True); tcon=None
            k=np.searchsorted(t,tc)
            while k<a[-1] and t[k]<tc+6:
                if skip and P['fz'][k]: p.tick(t[k]); k+=1; continue
                plate=P['ring_p'][k]+P['R'][k]@w
                ev=p.step(t[k],P['tip'][k],plate,body=P['body'][k],fresh=True,rotation=P['R'][k],lift_done=(tcon is not None and t[k]>=tcon+2.0))
                if ev=='contact': tcon=t[k]
                if p.state=='prove' and p.t_lift is not None:
                    cf=p.config; ma=np.mean([x[1] for x in p._win],axis=0)
                    c={'drift':p.drift<=cf.d_ok,'speed':math.isfinite(p.rel_speed) and p.rel_speed<=cf.speed_ok,'normal':abs(ma[2])<=cf.normal_ok,'follows':p.drift<=cf.follow_ratio*max(p.comotion,abs(p.rise))}
                    for kk,v in c.items():
                        if not v: tot[kk]+=1
                    tot['n']+=1
                    vals['drift'].append(p.drift); vals['speed'].append(p.rel_speed); vals['normal'].append(ma[2])
                if ev in ('failed','proven'): break
                k+=1
        print(tag,'skip' if skip else 'as configured',dict(tot),'drift p50 %.1f p90 %.1f mm, speed p50 %.3f p90 %.3f, normal p50 %+.1f mm'%(1e3*np.median(vals['drift']),1e3*np.percentile(vals['drift'],90),np.nanmedian(vals['speed']),np.nanpercentile(vals['speed'],90),1e3*np.median(vals['normal'])))
