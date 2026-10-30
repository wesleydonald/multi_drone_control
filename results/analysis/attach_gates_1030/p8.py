"""Variants: first-try outcome, time to contact, and a repeat-skipping manager (B1 gap 0.35 credited, B2 gap 0.1)."""
import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import *
LIFT_S=2.0
def run(tag, off, K=0.024, freeze=True, skip=False, gap=0.1, step=0.25, lost=None, horizon=20.0, max_fail=3):
    d,s=load(tag); w=np.asarray(WELD[tag])
    tip=d['tip']; ring=d['ring']; body=d['body']
    fz=frozen_mask(d) if freeze else np.zeros(len(tip),bool)
    sp=phase_spans(s)
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    ir=hold_idx(ring[:,0],tip[:,0]); ib=hold_idx(body[:,0],tip[:,0])
    Rk=np.array([quat_rot(q) for q in ring[ir,5:9]])
    Pk=ring[ir,2:5]+np.einsum('kij,j->ki',Rk,w)
    kw=dict(contact_xy=K, lift_fallback_s=0.0, taut_fallback=1.041, gap_s=gap)
    if lost: kw.update(lost=lost, sep_xy=lost)
    cfg=ComotionConfig(**kw)
    c=np.asarray(off,float)
    # loop the window so a 20 s horizon wraps (the ring motion repeats its statistics, not its phase)
    win=np.where((tip[:,0]>=a0)&(tip[:,0]<a1_))[0]
    T=tip[win[-1],0]-tip[win[0],0]
    out=[]
    for tc in np.arange(tip[win[0],0], tip[win[-1],0], step):
        p=ComotionProof(cfg); p.magnet(True)
        # free hang first: 1 s of real samples before tc (inside the window, wrapped)
        meas=None; t_contact=None; res=None; tries_out=[]; first=None; t_first_contact=None
        k=np.searchsorted(tip[:,0], tc); tt=tc; nfail=0
        while tt < tc+horizon:
            kk = win[0] + ((k-win[0]) % len(win))
            # synthetic clock: keep time monotone across the wrap
            tk = tt
            true=Pk[kk]+Rk[kk]@c
            btrue=true+np.array([0,0,1.040])
            frozen = fz[kk] and meas is not None
            tm = meas if frozen else true
            meas = tm
            k+=1
            nxt = win[0] + ((k-win[0]) % len(win))
            dtk = tip[nxt,0]-tip[kk,0]
            if dtk<=0 or dtk>0.2: dtk=0.0107
            tt += dtk
            if skip and frozen:
                ev=p.tick(tk)
            else:
                lift=(t_contact is not None and tk>=t_contact+LIFT_S)
                ev=p.step(tk, tm, Pk[kk], body=btrue, fresh=True, rotation=Rk[kk], lift_done=lift)
                if ev is None: ev=p.tick(tk)
            if ev=='contact':
                t_contact=tk
                if t_first_contact is None: t_first_contact=tk-tc
            if ev=='failed' and t_contact is not None:
                nfail+=1; tries_out.append(p.last_fail['reason']); t_contact=None
                if first is None: first='failed '+p.last_fail['reason']
                if nfail>=max_fail: res='gave up'; break
            if ev=='proven':
                res='proven'; 
                if first is None: first='proven'
                break
        out.append((res, first, t_first_contact, tt-tc, nfail))
    return out
def show(tag,lab,out):
    n=len(out)
    pr=sum(1 for o in out if o[0]=='proven'); f1=sum(1 for o in out if o[1]=='proven')
    tc=[o[2] for o in out if o[2] is not None]
    tp=[o[3] for o in out if o[0]=='proven']
    print('%s %-40s n %d: contact median %.2f s (p90 %.2f); first proof passes %.0f%%; proven within 3 tries %.0f%%, time median %.1f s p90 %.1f'%(tag,lab,n,np.median(tc) if tc else np.nan,np.percentile(tc,90) if tc else np.nan,100*f1/n,100*pr/n,np.median(tp) if tp else np.nan,np.percentile(tp,90) if tp else np.nan))
if __name__=='__main__':
    off=(0.01,0.0,0.002)
    for tag in ('sq5_2','sq2_5'):
        show(tag,'as flown (K .024, freezes)',run(tag,off))
        show(tag,'no freeze',run(tag,off,freeze=False))
        show(tag,'freezes, repeats skipped, gap 0.1',run(tag,off,skip=True,gap=0.1))
        show(tag,'freezes, repeats skipped, gap 0.35',run(tag,off,skip=True,gap=0.35))
        show(tag,'freezes, repeats skipped, gap 0.50',run(tag,off,skip=True,gap=0.50))
