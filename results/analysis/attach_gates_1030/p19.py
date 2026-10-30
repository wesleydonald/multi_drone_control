import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
import drone_magnet.comotion_proof as cpm
import p16
from collections import Counter
base=cpm.ComotionConfig
EXTRA={}
def mk(**kw):
    kw['taut_fallback']=0.0
    kw.update(EXTRA)
    return base(**kw)
p16.ComotionConfig=mk
variants=[('as configured',{},{}),
          ('speed window 0.5 s',{'win':0.5},{}),
          ('speed window 0.5 s + gap 0.35',{'win':0.5,'gap':0.35},{}),
          ('skip repeats + gap .35 + window .5',{'skip':True,'gap':0.35,'win':0.5},{}),
          ('skip + gap .35 + window .5 + d_ok 8 mm',{'skip':True,'gap':0.35,'win':0.5},{'d_ok':0.008}),
          ('speed_ok 0.06 only',{},{'speed_ok':0.06})]
for tag in ('f4','h4f_3'):
    P=p16.prep(tag)
    # shorter horizon for h4f_3's 7.9 s in the air
    for lab,kw,ex in variants:
        EXTRA.clear(); EXTRA.update(ex)
        a=P['air']; t=P['t']
        n,c,pr,tf,tp=p16.proof(tag,P,step=0.25,**kw) if tag=='f4' else (None,)*5
        if tag=='h4f_3':
            # inline short-horizon version
            from drone_magnet.comotion_proof import ComotionProof
            cfg=mk(contact_xy=0.024,lift_fallback_s=0.0,gap_s=kw.get('gap',0.1),speed_window_s=kw.get('win',0.2))
            out=[]
            for tc in np.arange(t[a[0]]+0.3,t[a[-1]]-4.0,0.1):
                p=ComotionProof(cfg); p.magnet(True); tcon=None; r=None
                k=np.searchsorted(t,tc)
                while k<a[-1]:
                    if kw.get('skip') and P['fz'][k]: p.tick(t[k]); k+=1; continue
                    plate=P['ring_p'][k]+P['R'][k]@P['w0']
                    ev=p.step(t[k],P['tip'][k],plate,body=P['body'][k],fresh=True,rotation=P['R'][k],lift_done=(tcon is not None and t[k]>=tcon+2.0))
                    if ev is None: ev=p.tick(t[k])
                    if ev=='contact': tcon=t[k]
                    if ev in ('failed','proven'): r=ev if ev=='proven' else 'fail '+p.last_fail['reason']; break
                    k+=1
                out.append(r)
            print('%s %-42s n %d: first proof %s'%(tag,lab,len(out),dict(Counter(out))))
            continue
        print('%s %-42s n %d: contact %d/%d; first proof %s; proven within 3 tries %d/%d, median %.1f s'%(tag,lab,n,len(tf),n,dict(c),pr,n,np.median(tp) if tp else np.nan))
