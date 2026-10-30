"""Who leads at 0.05-0.3 Hz (cross-spectral phase), the planner's restoring action, the integral's role, and the
0.55 m triangle (22 Oct h3)."""
import numpy as np, pandas as pd
from scipy.signal import csd, welch
import d1
WIN=d1.WIN+[('h3','2026-10-22','triangle 0.55 m (22 Oct), hover',(45.0,53.8),3)]
def phase(x,y,fs=10.0,band=(0.05,0.3)):
    """mean phase (deg) of y relative to x over the band, + = y lags x; coherence-weighted, per axis mean."""
    n=min(len(x),128)
    ph=[];co=[]
    for k in (0,1):
        f,pxy=csd(x[:,k],y[:,k],fs=fs,nperseg=n,detrend='linear')
        _,pxx=welch(x[:,k],fs=fs,nperseg=n,detrend='linear'); _,pyy=welch(y[:,k],fs=fs,nperseg=n,detrend='linear')
        m=(f>=band[0])&(f<=band[1])
        if not m.any(): m=(f>0)&(f<=0.4)
        coh=np.abs(pxy[m])**2/(pxx[m]*pyy[m])
        ph.append(np.degrees(np.angle(np.sum(pxy[m]*coh))))   # angle of y relative to x: + means y lags
        co.append(float(np.mean(coh)))
    return float(np.mean(ph)), float(np.mean(co))
rows=[]
for run,day,label,win,nc in WIN:
    pl,trs,ints=d1.load(run,day)
    t0=pl.sim_time.iloc[0]; a,b=t0+win[0],t0+win[1]; tg=np.arange(a,b,0.1)
    I=lambda c: np.interp(tg,pl.sim_time,pl[c])
    ring=np.column_stack([I('load_x'),I('load_y')])
    hov=np.array([pl.hover_x.iloc[-1],pl.hover_y.iloc[-1]]) if 'hover_x' in pl else ring.mean(0)
    pose=np.mean([np.column_stack([np.interp(tg,trs[i].sim_time,trs[i].pose_x),np.interp(tg,trs[i].sim_time,trs[i].pose_y)]) for i in range(nc)],axis=0)
    ref=np.mean([np.column_stack([np.interp(tg,trs[i].sim_time,trs[i].ref_x),np.interp(tg,trs[i].sim_time,trs[i].ref_y)]) for i in range(nc)],axis=0)
    fint=np.column_stack([I('int_fx'),I('int_fy')])
    p_pose,c_pose=phase(pose,ring)      # ring relative to the carriers: + = ring lags carriers
    p_ref,c_ref=phase(ring,ref)         # refs relative to the ring: + = refs lag the ring
    # restoring: (carriers' centroid - ring) against (ring - hover), low-passed; slope (m/m) and correlation
    off=d1.lp(ring-hov-(ring-hov).mean(0)); lean=d1.lp((pose-ring)-(pose-ring).mean(0))
    slope=float(np.sum(off*lean)/np.sum(off*off)); corr=float(np.mean([np.corrcoef(off[:,k],lean[:,k])[0,1] for k in (0,1)]))
    # integral force vs ring offset
    fi=d1.lp(fint-fint.mean(0))
    ci=float(np.mean([np.corrcoef(fi[:,k],off[:,k])[0,1] for k in (0,1)])) if np.std(fi)>1e-6 else float('nan')
    # ring speed (planner log has no xy speed: differentiate), share under the 0.05 m/s gate
    v=np.gradient(ring,0.1,axis=0); sp=np.hypot(v[:,0],v[:,1])
    rows.append((run,label,win[1]-win[0],p_pose,c_pose,p_ref,c_ref,slope,corr,ci,float(np.mean(sp<0.05)),float(np.median(sp))))
print('run, window, s | ring vs carriers: phase (+ ring lags), coherence | refs vs ring: phase (+ refs lag), coh | lean per offset (m/m), corr | corr(int F, offset) | ring speed < 0.05 m/s share, median')
for r in rows:
    print('%-6s %-40s %5.1f | %+6.1f deg %.2f | %+6.1f deg %.2f | %+.3f %+.2f | %+.2f | %.0f%% %.3f'%r)
