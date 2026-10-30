"""Real streams (tip, ring, body, coil, his phase and lift signal) through ComotionProof: rig config vs the proposed one.
No-catch flights must never prove; t6h_4's catch must."""
import sys, json
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import *
TAUT={'t6h_4':0.575,'t6f2_4':0.575,'t6h5_3':0.565,'t6g_2':0.575,'a1':0.575,'a3_2':0.575,'sq5_2':1.041,'sq2_5':1.041}
for t_ in ('t6f2_4','t6h5_3','t6g_2','a3_2'): WELD[t_]=(0.013,0.210,0.024)
CONTACT_PHASES=('MATCH_VELOCITY','DESCEND_TO_ATTACHMENT','ATTACH_READY','LIFT_PROOF','RING_ATTACHED')
def replay(tag, K=0.024, skip=False, gap=0.1, win=0.2, d_ok=0.006, lost=0.045):
    d,s=load(tag); w=np.asarray(WELD[tag])
    tip=d['tip']; ring=d['ring']; body=d['body']; fz=frozen_mask(d)
    ir=hold_idx(ring[:,0],tip[:,0]); ib=hold_idx(body[:,0],tip[:,0])
    coil=s.get('/drone_3/magnet_coil',[]); ph=s.get('/join_planner/phase',[]); lift=s.get('/join_planner/proof_lift_complete',[])
    ct=np.array([c[0] for c in coil]); pt=np.array([p[0] for p in ph]); lt=np.array([l[0] for l in lift]) if lift else None
    cfg=ComotionConfig(contact_xy=K,lift_fallback_s=0.0 if lift else 1.8,taut_fallback=TAUT[tag],gap_s=gap,speed_window_s=win,d_ok=d_ok,lost=lost,sep_xy=lost)
    p=ComotionProof(cfg); evs=[]; on=False
    for k in range(len(tip)):
        tk=tip[k,0]
        j=np.searchsorted(ct,tk,side='right')-1
        want = j>=0 and coil[j][1]=='ON'
        if want!=on:
            p.magnet(want); on=want
        jp=np.searchsorted(pt,tk,side='right')-1; phase=ph[jp][1] if jp>=0 else ''
        if p.state=='prove' and phase not in CONTACT_PHASES:
            if p.abort('aborted',tk): evs.append((tk,'failed','aborted'))
        ld=False
        if lt is not None:
            jl=np.searchsorted(lt,tk,side='right')-1; ld = jl>=0 and bool(lift[jl][1])
        if skip and fz[k]:
            ev=p.tick(tk)
        else:
            R=quat_rot(ring[ir[k],5:9]); P=ring[ir[k],2:5]+R@w
            ev=p.step(tk,tip[k,2:5],P,body=body[ib[k],2:5],fresh=True,rotation=R,lift_done=ld)
            if ev is None: ev=p.tick(tk)
        if ev: evs.append((tk,ev,p.last_fail['reason'] if ev=='failed' else ''))
    return evs
for tag in ('t6h_4','t6f2_4','t6h5_3','t6g_2','a1','a3_2','sq5_2','sq2_5'):
    for lab,kw in (('rig as typed',{}),('proposed: skip+gap.35+win.5+d_ok 8mm',dict(skip=True,gap=0.35,win=0.5,d_ok=0.008)),('proposed + K .045 lost .065',dict(skip=True,gap=0.35,win=0.5,d_ok=0.008,K=0.045,lost=0.065))):
        e=replay(tag,**kw)
        from collections import Counter
        c=Counter(x[1]+(':'+x[2] if x[2] else '') for x in e)
        print('%-7s %-40s %s'%(tag,lab,dict(c)))
