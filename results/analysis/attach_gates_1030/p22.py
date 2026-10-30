"""Synthetic catch (real ring motion over plate 9 in sq5_2/sq2_5, their attach-window freeze pattern) plus body 24's
in-flight noise from f4 (white, sd 1.3/1.2/3.8 mm): as configured vs proposed."""
import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import *
SD=np.array([0.0013,0.0012,0.0038])
def run(tag, off, K=0.024, skip=False, gap=0.1, win=0.2, d_ok=0.006, lost=0.045, step=0.25, horizon=20.0, seed=1):
    rng=np.random.default_rng(seed)
    d,s=load(tag); w=np.asarray(WELD[tag])
    tip=d['tip']; ring=d['ring']; fz=frozen_mask(d)
    sp=phase_spans(s)
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    ir=hold_idx(ring[:,0],tip[:,0]); Rk=np.array([quat_rot(q) for q in ring[ir,5:9]]); Pk=ring[ir,2:5]+np.einsum('kij,j->ki',Rk,w)
    win_i=np.where((tip[:,0]>=a0)&(tip[:,0]<a1_))[0]
    cfg=ComotionConfig(contact_xy=K,lift_fallback_s=0.0,taut_fallback=1.041,gap_s=gap,speed_window_s=win,d_ok=d_ok,lost=lost,sep_xy=lost)
    c=np.asarray(off,float); out=[]
    for tc in np.arange(tip[win_i[0],0], tip[win_i[-1],0], step):
        p=ComotionProof(cfg); p.magnet(True); meas=None; tcon=None; res=None; first=None; nf=0
        k=np.searchsorted(tip[:,0],tc); tt=tc
        while tt<tc+horizon:
            kk=win_i[0]+((k-win_i[0])%len(win_i))
            true=Pk[kk]+Rk[kk]@c
            fr=fz[kk] and meas is not None
            tm=meas if fr else true+rng.normal(0,1,3)*SD
            meas=tm
            k+=1; kn=win_i[0]+((k-win_i[0])%len(win_i)); dt=tip[kn,0]-tip[kk,0]
            if dt<=0 or dt>0.2: dt=0.0107
            if skip and fr: ev=p.tick(tt)
            else:
                ev=p.step(tt,tm,Pk[kk],body=true+np.array([0,0,1.040]),fresh=True,rotation=Rk[kk],lift_done=(tcon is not None and tt>=tcon+2.0))
                if ev is None: ev=p.tick(tt)
            if ev=='contact': tcon=tt
            if ev=='failed' and tcon is not None:
                nf+=1; tcon=None; first=first or 'fail '+p.last_fail['reason']
                if nf>=3: res='gave up'; break
            if ev=='proven': res='proven'; first=first or 'proven'; break
            tt+=dt
        out.append((res,first,tt-tc))
    n=len(out); f1=sum(o[1]=='proven' for o in out); pr=sum(o[0]=='proven' for o in out)
    tp=[o[2] for o in out if o[0]=='proven']
    return n,f1,pr,tp
for tag in ('sq5_2','sq2_5'):
    for lab,kw in (('as configured',{}),('proposed (skip, gap .35, window .5, d_ok 8 mm)',dict(skip=True,gap=0.35,win=0.5,d_ok=0.008)),
                   ('proposed + K .045 lost .065, catch 4 cm off',dict(skip=True,gap=0.35,win=0.5,d_ok=0.008,K=0.045,lost=0.065,off=(0.04,0,0.002)))):
        off=kw.pop('off',(0.01,0,0.002))
        n,f1,pr,tp=run(tag,off,**kw)
        print('%s %-48s n %d: first proof %.0f%%, proven within 3 tries %.0f%%, time median %.1f s'%(tag,lab,n,100*f1/n,100*pr/n,np.median(tp) if tp else np.nan))
