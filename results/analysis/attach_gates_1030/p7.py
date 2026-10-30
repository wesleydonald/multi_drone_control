"""Synthetic catch on the real ring motion and the real tip-freeze pattern, through our ComotionProof (rig config)."""
import sys, math
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import *

LIFT_S = 2.0          # his lift: commanded -0.01 -> +0.03 at 0.02 m/s
def run(tag, offsets, K=0.024, freeze=True, lost=None, sep_xy=None, step=0.25, ring_shift=0.0, extra=None):
    d,s=load(tag); w=np.asarray(WELD[tag])
    tip=d['tip']; ring=d['ring']; body=d['body']
    fz=frozen_mask(d) if freeze else np.zeros(len(tip),bool)
    if isinstance(freeze, np.ndarray): fz=freeze
    sp=phase_spans(s)
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    ir=hold_idx(ring[:,0],tip[:,0]); ib=hold_idx(body[:,0],tip[:,0])
    Rk=np.array([quat_rot(q) for q in ring[ir,5:9]])
    Pk=ring[ir,2:5]+np.einsum('kij,j->ki',Rk,w)
    kw=dict(contact_xy=K, lift_fallback_s=0.0, taut_fallback=1.041)
    if lost is not None: kw['lost']=lost
    if sep_xy is not None: kw['sep_xy']=sep_xy
    if extra: kw.update(extra)
    cfg=ComotionConfig(**kw)
    res={}
    starts=np.arange(a0+3.0, a1_-6.0, step)
    for off in offsets:
        out=[]
        c=np.asarray(off,float)
        for tc in starts:
            p=ComotionProof(cfg)
            k0=np.searchsorted(tip[:,0], tc-3.0); k1=np.searchsorted(tip[:,0], tc+9.0)
            p.magnet(True)
            meas=None; t_contact=None; result=None; t_res=None; fails=[]
            for k in range(k0,k1):
                tk=tip[k,0]
                if tk<tc:
                    tm=tip[k,2:5]; bm=body[ib[k],2:5]
                else:
                    true=Pk[k]+Rk[k]@c
                    # body hangs taut above the true tip (QUAD4 above the plate on its cable)
                    btrue=true+np.array([0,0,1.040])
                    tm=true if not (fz[k] and meas is not None and tk>tc+1e-9) else meas
                    bm=btrue
                meas=tm
                lift=(t_contact is not None and tk>=t_contact+LIFT_S)
                ev=p.step(tk, tm, Pk[k], body=bm, fresh=True, rotation=Rk[k], lift_done=lift)
                if ev is None: ev=p.tick(tk)
                if ev=='contact' and tk>=tc: t_contact=tk
                elif ev=='contact': pass
                if ev=='failed' and t_contact is not None:
                    fails.append(p.last_fail['reason']); t_contact=None
                    if len(fails)>=3: result='failed x3'; t_res=tk; break
                if ev=='proven': result='proven'; t_res=tk; break
            out.append((tc, result, (t_res-tc) if t_res else np.nan, fails, p.tries))
        res[tuple(off)]=out
    return res

def summarise(tag, res, label):
    for off,out in res.items():
        n=len(out)
        proven=[o for o in out if o[1]=='proven']
        cont=[o for o in out if o[4]>0]
        reasons={}
        for o in out:
            for f in o[3]: reasons[f]=reasons.get(f,0)+1
        tp=[o[2] for o in proven]
        print('%s %-28s off %s: contact in %d/%d, proven %d/%d (%.0f%%), time to proven median %s s; fail reasons %s'%(tag,label,tuple(round(x,3) for x in off),len(cont),n,len(proven),n,100*len(proven)/n, ('%.1f'%np.median(tp)) if tp else '-', reasons))

if __name__=='__main__':
    offs=[(0.0,0.0,0.002),(0.02,0.0,0.002),(0.0,0.03,0.002),(0.04,0.0,0.002)]
    for tag in ('sq5_2','sq2_5'):
        summarise(tag, run(tag, offs, K=0.024, freeze=True), 'K 0.024, freezes as flown')
        summarise(tag, run(tag, offs, K=0.024, freeze=False), 'K 0.024, no freeze')
        summarise(tag, run(tag, offs, K=0.045, freeze=False, lost=0.065, sep_xy=0.065), 'K .045 lost .065, no freeze')
        summarise(tag, run(tag, offs, K=0.045, freeze=True, lost=0.065, sep_xy=0.065), 'K .045 lost .065, freezes')
