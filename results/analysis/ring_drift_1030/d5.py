"""The same 15 s window length everywhere (ch1b_2's square hover is 15 s): ring xy below 0.3 Hz, median and range."""
import numpy as np
import d1
out={}
for run,day,label,win,nc in d1.WIN:
    pl,trs,ints=d1.load(run,day)
    t0=pl.sim_time.iloc[0]
    vals=[]
    s=win[0]
    while s+15.0<=win[1]+1e-6:
        tg=np.arange(t0+s,t0+s+15.0,0.1)
        ring=np.column_stack([np.interp(tg,pl.sim_time,pl.load_x),np.interp(tg,pl.sim_time,pl.load_y)])
        x=d1.lp(ring-ring.mean(0)); vals.append(100*np.sqrt(np.mean(np.sum(x**2,axis=1))))
        s+=5.0
    if vals: print('%-6s %-40s %d windows of 15 s: below 0.3 Hz median %.2f cm (%.2f-%.2f)'%(run,label,len(vals),np.median(vals),min(vals),max(vals)))
    else: print('%-6s %-40s shorter than 15 s'%(run,label))
