import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
import p16
from collections import Counter
for tag in ('f4','h4f_3'):
    P=p16.prep(tag); a=P['air']; t=P['t']; w=P['w0']
    cfg=ComotionConfig(contact_xy=0.024,lift_fallback_s=0.0,taut_fallback=0.0,speed_window_s=0.5,d_ok=0.008)
    out=[]
    hz=15 if tag=='f4' else 4.0
    for tc in np.arange(t[a[0]]+0.3,t[a[-1]]-(8 if tag=='f4' else 4),0.25 if tag=='f4' else 0.1):
        p=ComotionProof(cfg); p.magnet(True); tcon=None; nf=0; res=None; first=None; held=0.0; lf=None
        k=np.searchsorted(t,tc)
        while k<a[-1] and t[k]<tc+hz:
            if P['fz'][k]: k+=1; continue
            if lf is not None: held+=max(0.0,t[k]-lf-0.0107)
            lf=t[k]; v=t[k]-held
            plate=P['ring_p'][k]+P['R'][k]@w
            ev=p.step(v,P['tip'][k],plate,body=P['body'][k],fresh=True,rotation=P['R'][k],lift_done=(tcon is not None and t[k]>=tcon+2.0))
            if ev is None: ev=p.tick(v)
            if ev=='contact': tcon=t[k]
            if ev=='failed' and tcon is not None:
                nf+=1; tcon=None; first=first or 'fail '+p.last_fail['reason']
                if nf>=3: res='gave up'; break
            if ev=='proven': res='proven'; first=first or 'proven'; break
            k+=1
        out.append((res,first))
    print(tag,'fresh-time clock + skip + window .5 + d_ok 8 mm (taut off): first',dict(Counter(o[1] for o in out)),'within 3 tries',sum(o[0]=='proven' for o in out),'/',len(out))
