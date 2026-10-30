"""Ring drift in the triangle and the square (29 Oct; 22 Oct h3 on 0.55 m): planner and tracker logs.
Per window: the ring's xy below 0.3 Hz, the carriers' centroid, the ring relative to it, the hover target error,
the model integral, the carriers' references and tracking errors, the tilt coupling."""
import glob, os, re, sys, json
import numpy as np, pandas as pd
from scipy.signal import butter, filtfilt
REPO='/home/wesley/multi_drone_control'
WIN=[('h4f_3','2026-10-29','square, top',(65.5,70.5),4),
     ('ch1b_2','2026-10-29','square, hover',(71.0,86.0),4),
     ('sq2_5','2026-10-29','triangle, his climb',(71.0,94.5),3),
     ('sq2_5','2026-10-29','triangle, his transit (QUAD4 near)',(94.5,114.3),3),
     ('sq2_5','2026-10-29','triangle, MATCH (QUAD4 over)',(115.4,139.1),3),
     ('sq5_2','2026-10-29','triangle, his climb',(46.6,54.7),3),
     ('sq5_2','2026-10-29','triangle, MATCH+descents (QUAD4 over)',(61.5,86.5),3),
     ('sq3','2026-10-29','triangle, QUAD4 held far',(150.0,188.0),3),
     ('sq4_5','2026-10-29','triangle, QUAD4 held far',(112.0,145.4),3),
     ('k1_4','2026-10-29','triangle, QUAD4 at pickup',(112.2,138.4),3),
     ('h3','2026-10-22','triangle 0.55 m (22 Oct), hover',(45.0,53.8),3)]
def lp(x,fc=0.3,fs=10.0):
    b,a=butter(2,fc/(fs/2)); return filtfilt(b,a,x,axis=0)
def load(run,day):
    base=f'{REPO}/results/rig/{day}/{run}_logs/logs'
    pl=pd.read_csv(sorted(glob.glob(f'{base}/dissipative_planner/*/log.csv'))[0],low_memory=False)
    trs={}
    for f in sorted(glob.glob(f'{base}/tracker/drone*/log.csv')):
        i=int(re.search(r'drone(\d+)_',f).group(1)); trs[i]=pd.read_csv(f,low_memory=False)
    logf=f'{REPO}/results/rig/{day}/{run}.log'
    ints=[]
    for line in open(logf,errors='ignore'):
        m=re.search(r'\[(\d{10}\.\d+)\] \[dissipative_planner\]: \[planner int\] b=\(([-+\d.]+), ([-+\d.]+), ([-+\d.]+)\) m  F=\(([-+\d.]+), ([-+\d.]+), ([-+\d.]+)\) N  gate xy (\d) z (\d)',line)
        if m: ints.append([float(g) for g in m.groups()])
    return pl,trs,np.array(ints)
def stats(run,day,label,win,nc):
    pl,trs,ints=load(run,day)
    t0=pl.sim_time.iloc[0]; a,b=t0+win[0],t0+win[1]
    w=pl[(pl.sim_time>=a)&(pl.sim_time<=b)].copy()
    tg=np.arange(a,b,0.1)
    I=lambda col: np.interp(tg,pl.sim_time,pl[col])
    ring=np.column_stack([I('load_x'),I('load_y')])
    cen=np.mean([np.column_stack([I(f'd{i}_x'),I(f'd{i}_y')]) for i in range(nc)],axis=0)
    hov=np.array([pl.hover_x.iloc[-1],pl.hover_y.iloc[-1]]) if 'hover_x' in pl else ring.mean(0)
    rel=ring-cen
    def r(x): x=x-x.mean(axis=0); return 100*float(np.sqrt(np.mean(np.sum(lp(x)**2,axis=1))))
    def rr(x): x=x-x.mean(axis=0); return 100*float(np.sqrt(np.mean(np.sum(x**2,axis=1))))
    out=dict(run=run,label=label,dur=win[1]-win[0],ring_lp=r(ring),ring_all=rr(ring),cen_lp=r(cen),rel_lp=r(rel),
             err_mean=100*float(np.linalg.norm(ring.mean(axis=0)-hov)))
    rl=lp(ring-ring.mean(0)); cl=lp(cen-cen.mean(0)); ql=lp(rel-rel.mean(0))
    out['corr_ring_cen']=float(np.mean([np.corrcoef(rl[:,k],cl[:,k])[0,1] for k in (0,1)]))
    # integral
    if 'int_fx' in pl:
        fx=np.column_stack([I('int_fx'),I('int_fy')])
        out['int_f_mean']=float(np.linalg.norm(fx.mean(0))); out['int_f_lp']=float(np.sqrt(np.mean(np.sum(lp(fx-fx.mean(0))**2,axis=1))))
        out['int_b_mean_cm']=100*float(np.linalg.norm(np.column_stack([I('int_bx'),I('int_by')]).mean(0)))
    if len(ints):
        m=(ints[:,0]>=a)&(ints[:,0]<=b)
        out['gate_xy_on']=float(ints[m,7].mean()) if m.any() else np.nan
        out['gate_z_on']=float(ints[m,8].mean()) if m.any() else np.nan
    # planner's plan 5 nodes ahead relative to the ring (what it asks of the ring)
    p5=np.column_stack([I('plan5_x'),I('plan5_y')])
    out['plan5_minus_ring_lp']=r(p5-ring)
    # trackers
    refm=[];errm=[];refs=[];poses=[]
    for i in range(nc):
        d=trs[i]; Tq=lambda c: np.interp(tg,d.sim_time,d[c])
        ref=np.column_stack([Tq('ref_x'),Tq('ref_y')]); pose=np.column_stack([Tq('pose_x'),Tq('pose_y')])
        refs.append(ref); poses.append(pose)
        refm.append(r(ref)); errm.append(r(pose-ref))
    out['ref_lp']=float(np.mean(refm)); out['trk_err_lp']=float(np.mean(errm))
    refc=np.mean(refs,axis=0); posec=np.mean(poses,axis=0)
    out['refcen_lp']=r(refc); out['trk_err_cen_lp']=r(posec-refc)
    # lag: ring LP vs reference-centroid LP cross-correlation (positive = refs lead the ring)
    x=lp(refc-refc.mean(0)); y=rl
    best=None
    for lag in range(-30,31):
        if lag>=0: c=np.mean([np.corrcoef(x[:len(x)-lag,k],y[lag:,k])[0,1] for k in (0,1)])
        else: c=np.mean([np.corrcoef(x[-lag:,k],y[:len(y)+lag,k])[0,1] for k in (0,1)])
        if best is None or c>best[1]: best=(lag/10.0,c)
    out['ref_leads_ring_s'],out['ref_ring_corr']=best
    # tilt coupling: normal's horizontal components (tilt toward) vs ring minus centroid
    q=np.column_stack([I('qw'),I('qx'),I('qy'),I('qz')])
    qw,qx,qy,qz=q.T
    nx=2*(qx*qz+qw*qy); ny=2*(qy*qz-qw*qx)
    nl=lp(np.column_stack([nx,ny])-np.column_stack([nx,ny]).mean(0))
    out['tilt_lp_deg']=float(np.degrees(np.sqrt(np.mean(np.sum(nl**2,axis=1)))))
    out['corr_tilt_rel']=float(np.mean([np.corrcoef(nl[:,k],ql[:,k])[0,1] for k in (0,1)]))
    out['corr_tilt_ring']=float(np.mean([np.corrcoef(nl[:,k],rl[:,k])[0,1] for k in (0,1)]))
    # ring z and carriers' z low-pass
    z=I('load_z'); out['ring_z_lp_cm']=100*float(np.std(lp(z-z.mean())))
    # throttle trend
    thr=[np.interp(tg,trs[i].sim_time,trs[i].thr_out) for i in range(nc)]
    out['thr_mean']=float(np.mean(thr)); out['thr_spread']=float(np.ptp([np.mean(x) for x in thr]))
    return out
if __name__=='__main__':
    res=[stats(*w) for w in WIN]
    json.dump(res,open('d1.json','w'),indent=1)
    keys=['ring_lp','ring_all','cen_lp','rel_lp','corr_ring_cen','err_mean','int_f_mean','int_f_lp','int_b_mean_cm','gate_xy_on','gate_z_on','plan5_minus_ring_lp','ref_lp','refcen_lp','trk_err_lp','trk_err_cen_lp','ref_leads_ring_s','ref_ring_corr','tilt_lp_deg','corr_tilt_rel','corr_tilt_ring','ring_z_lp_cm','thr_mean','thr_spread']
    for o in res:
        print(o['run'],o['label'],'%.1f s'%o['dur'])
        print('   '+', '.join('%s %s'%(k,('%.2f'%o[k]) if isinstance(o.get(k),float) else o.get(k)) for k in keys))
