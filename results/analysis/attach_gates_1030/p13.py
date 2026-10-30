"""His bottom hold on a synthetic catch (real ring motion, real tip-freeze pattern): below / gross / quiet timers."""
import sys, math
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/tejen_mission')
from tejen_mission.m2_irl_attachment_policy import QuietMissTracker
from common import *
def run(tag, off, freeze=True, step=0.25, horizon=45.0, skip=False):
    d,s=load(tag); w=np.asarray(WELD[tag])
    tip=d['tip']; ring=d['ring']; body=d['body']
    fz=frozen_mask(d) if freeze else np.zeros(len(tip),bool)
    sp=phase_spans(s)
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    win=np.where((tip[:,0]>=a0)&(tip[:,0]<a1_))[0]
    ir=hold_idx(ring[:,0],tip[:,0]); Rk=np.array([quat_rot(q) for q in ring[ir,5:9]]); Pk=ring[ir,2:5]+np.einsum('kij,j->ki',Rk,w)
    c=np.asarray(off,float)
    out=[]
    for tc in np.arange(tip[win[0],0], tip[win[-1],0], step):
        k=np.searchsorted(tip[:,0],tc); tt=0.0; meas=None; mstamp=None
        below_t0=None; gross_t0=None; q=QuietMissTracker(wait_s=4.0,improvement_m=0.005); last=None; vel=np.zeros(3)
        nxt_tick=0.0; why=None; t_why=None
        while tt<horizon:
            kk=win[0]+((k-win[0])%len(win))
            true=Pk[kk]+Rk[kk]@c
            fr=fz[kk] and meas is not None
            if not fr: meas=true; mstamp=tt
            k+=1; kn=win[0]+((k-win[0])%len(win)); dt=tip[kn,0]-tip[kk,0]
            if dt<=0 or dt>0.2: dt=0.0107
            # his 30 Hz ticks in [tt, tt+dt)
            while nxt_tick < tt+dt:
                rel=Rk[kk].T@(meas-Pk[kk])
                xy=math.hypot(rel[0],rel[1])
                # his smoothed plate-frame tip speed: updated when the tip stamp advances (the frozen repeats advance it)
                stamp=tt
                if last is not None and stamp>last[0]+1e-6:
                    vel=.35*(rel-last[1])/(stamp-last[0])+.65*vel
                if last is None or stamp>last[0]+1e-6: last=(stamp,rel.copy())
                spd=math.hypot(vel[0],vel[1])
                now=nxt_tick
                below=rel[2] < -0.020
                below_t0=(now if below_t0 is None else below_t0) if below else None
                gross=xy>0.12
                gross_t0=(now if gross_t0 is None else gross_t0) if gross else None
                if below and now-below_t0>=0.12: why='below'; break
                if gross and now-gross_t0>=0.3: why='gross'; break
                quiet=spd<=0.03 and xy>0.035
                if q.step(now=now,xy_error_m=xy,eligible=quiet,key=0): why='quiet'; break
                nxt_tick+=1/30
            if why: t_why=nxt_tick; break
            tt+=dt
        out.append((why,t_why))
    return out
def show(tag,lab,out):
    n=len(out)
    from collections import Counter
    c=Counter(o[0] for o in out)
    for W in (0.5,1.0,4.5):
        pass
    early={W: sum(1 for o in out if o[0] and o[1]<=W)/n for W in (0.4,0.75,1.0,5.0)}
    ts=[o[1] for o in out if o[0]]
    print('%s %-34s n %d: left the hold %s; by 0.4 s %.0f%%, 0.75 s %.0f%%, 1 s %.0f%%, 5 s %.0f%%; median leave %.1f s'%(tag,lab,n,dict(c),100*early[0.4],100*early[0.75],100*early[1.0],100*early[5.0],np.median(ts) if ts else np.nan))
for tag in ('sq5_2','sq2_5'):
    for off in ((0.01,0,0.002),(0.03,0,0.002),(0.045,0,0.002)):
        show(tag,'catch %s, freezes'%(off[:2],),run(tag,off))
        show(tag,'catch %s, no freeze'%(off[:2],),run(tag,off,freeze=False))
