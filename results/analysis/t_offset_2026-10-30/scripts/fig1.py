from load import *
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT='/home/wesley/multi_drone_control/results/analysis/t_offset_2026-10-30'
A=load_all()
C={'ours':'#c0392b','d05':'#e67e22','d10':'#b7950b','d20':'#27ae60','nors':'#2c3e50','plan':'#8e44ad','nplan':'#7f8c8d'}
LAB={'ours':'ours (resize at detection)','d05':'late 0.5 s','d10':'late 1 s','d20':'late 2 s','nors':'no resize','plan':'planned pull','nplan':'no resize + planned pull'}
def ny(x):
    return x.df.t.values-x.te, np.degrees(np.arcsin(np.clip(x.df.ny.values,-1,1)))
g=np.arange(-1,20,0.01)
nors=np.mean([np.interp(g,*ny(A[r])) for r in RUNS['nors']],0)
fig,ax=plt.subplots(3,1,figsize=(11,11))
for arm in ['nors','ours','d05','d10','d20','plan']:
    for j,r in enumerate(RUNS[arm]):
        x=A[r]; t,v=ny(x)
        m=(t>-1)&(t<20)
        ax[0].plot(t[m],v[m],color=C[arm],lw=1.0,alpha=0.9,label=f'{LAB[arm]}' if j==0 else None)
        if x.tres is not None and arm!='plan':
            ax[0].axvline(x.tres-x.te,color=C[arm],ls=':',lw=0.8)
        vi=np.interp(g,t,v)
        if arm not in ('plan',):
            ax[1].plot(g,vi-nors,color=C[arm],lw=1.0,label=f'{LAB[arm]}' if j==0 else None)
            if x.tres is not None: ax[1].axvline(x.tres-x.te,color=C[arm],ls=':',lw=0.8)
        if x.tres is not None and arm in ('ours','d05','d10','d20'):
            ax[2].plot(g-(x.tres-x.te),vi-nors,color=C[arm],lw=1.0,label=f'{LAB[arm]}' if j==0 else None)
ax[0].set_ylim(-5,60); ax[0].set_xlim(-1,20)
ax[0].set_ylabel('ring tilt about the 0-6 axis (deg)\n+ = plate-9 side low')
ax[0].set_title('sqT: ring tilt after the unannounced release of plate 9 (t_e = 0); dotted = each run\'s resize')
ax[1].set_xlim(-1,20); ax[1].set_ylim(-6,14); ax[1].axhline(0,color='k',lw=0.5)
ax[1].set_ylabel('tilt minus the no-resize mean (deg)'); ax[1].set_xlabel('t - t_e (s)')
ax[2].set_xlim(-1,18); ax[2].set_ylim(-6,14); ax[2].axhline(0,color='k',lw=0.5); ax[2].axvline(0,color='k',lw=0.5)
ax[2].set_ylabel('tilt minus the no-resize mean (deg)'); ax[2].set_xlabel('t - t_resize (s)')
for a in ax: a.grid(alpha=0.3); a.legend(fontsize=8,loc='upper right',ncol=2)
fig.tight_layout(); fig.savefig(f'{OUT}/q1_tilt_aligned.png',dpi=110)
# zoom on first 4 s
fig,ax=plt.subplots(1,2,figsize=(13,4.5))
for arm in ['nors','ours','d05','d10','d20']:
    for j,r in enumerate(RUNS[arm]):
        x=A[r]; t,v=ny(x); vi=np.interp(g,t,v)
        ax[0].plot(g,vi,color=C[arm],lw=1,label=LAB[arm] if j==0 else None)
        ax[1].plot(g,vi-nors,color=C[arm],lw=1,label=LAB[arm] if j==0 else None)
        if x.tres is not None:
            for a in ax: a.axvline(x.tres-x.te,color=C[arm],ls=':',lw=0.8)
ax[0].set_xlim(0,6); ax[1].set_xlim(0,6); ax[1].set_ylim(-8,10)
ax[0].set_ylabel('tilt (deg)'); ax[1].set_ylabel('tilt - no-resize mean (deg)')
for a in ax: a.set_xlabel('t - t_e (s)'); a.grid(alpha=.3); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig(f'{OUT}/q1_tilt_zoom.png',dpi=110)
