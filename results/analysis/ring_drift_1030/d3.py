"""Is the drift a mode? Autocorrelation of the ring's detrended xy (planner log, 10 Hz): first zero, first trough,
the trough depth (an oscillation shows a negative trough), and the stiffness the carriers' lean implies."""
import numpy as np
import d1
from scipy.signal import detrend
G=9.81
for run,day,label,win,nc in d1.WIN:
    pl,trs,ints=d1.load(run,day)
    t0=pl.sim_time.iloc[0]; a,b=t0+win[0],t0+win[1]; tg=np.arange(a,b,0.1)
    ring=np.column_stack([np.interp(tg,pl.sim_time,pl.load_x),np.interp(tg,pl.sim_time,pl.load_y)])
    ac=[]
    for k in (0,1):
        x=detrend(ring[:,k]); n=len(x)
        c=np.correlate(x,x,'full')[n-1:]/np.arange(n,0,-1); c/=c[0]; ac.append(c[:min(n//2,150)])
    c=np.mean(ac,axis=0); lags=np.arange(len(c))*0.1
    z=np.flatnonzero(c<0); z0=lags[z[0]] if len(z) else np.nan
    if len(z):
        seg=c[z[0]:]; i=np.argmin(seg); tr=lags[z[0]+i]; depth=seg[i]
    else: tr=np.nan; depth=np.nan
    elev=np.radians(np.nanmedian([np.nanmedian(pl[f'elev{i}']) for i in range(nc)])) if 'elev0' in pl else np.radians(60)
    L=1.0 if day=='2026-10-29' else 0.55
    h=L*np.sin(elev) if np.degrees(elev)>5 else L*np.sin(np.radians(60))
    print('%-6s %-40s %5.1f s: acf zero %.1f s, trough %.1f s (%.2f), period from trough %.1f s (%.2f Hz); cable elev %.0f deg, h %.2f m'%(run,label,win[1]-win[0],z0,tr,depth,2*tr,1/(2*tr) if tr==tr else np.nan,np.degrees(elev),h))
