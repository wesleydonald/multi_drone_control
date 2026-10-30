import numpy as np
from scipy.signal import welch
from common import *
for tag in ('sq5_2','sq2_5'):
    d,s=load(tag); w=WELD[tag]
    sp=phase_spans(s)
    a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1_=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    ring=d['ring']; m=(ring[:,0]>=a0)&(ring[:,0]<a1_); rr=ring[m]
    tg=np.arange(rr[0,0],rr[-1,0],0.01)
    ii=hold_idx(rr[:,0],tg)
    C=rr[ii,2:5]
    P=np.array([rr[k,2:5]+quat_rot(rr[k,5:9])@np.asarray(w) for k in ii])
    N=np.array([quat_rot(rr[k,5:9])[:,2] for k in ii])
    tilt=np.degrees(np.arccos(np.clip(N[:,2],-1,1)))
    for lab,x in (('centre z',C[:,2]),('centre x',C[:,0]),('plate z',P[:,2]),('plate x',P[:,0]),('plate y',P[:,1]),('normal x',N[:,0])):
        f,p=welch(x-x.mean(),fs=100,nperseg=512)
        df=f[1]-f[0]
        tot=p.sum()*df
        def band(a,b): mm=(f>=a)&(f<b); return np.sqrt(p[mm].sum()*df)
        k=np.argmax(p*(f>2))
        print(tag,'%-9s rms %.1f mm: <0.3 Hz %.1f, 0.3-2 %.1f, 2-4 %.1f, 4-7 %.1f, 7-20 %.1f; peak above 2 Hz at %.1f Hz'%(lab,1e3*np.sqrt(tot) if 'normal' not in lab else np.degrees(np.sqrt(tot))*1e3/1e3*1000/1000,1e3*band(0,.3),1e3*band(.3,2),1e3*band(2,4),1e3*band(4,7),1e3*band(7,20),f[k]))
