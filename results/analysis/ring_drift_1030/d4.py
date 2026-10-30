"""Body 8 (the ring) as a sensor: its low-frequency xy at rest on the floor, and in flight against the mean of the four
plate magnet markers (independent Motive bodies) in the square. Then the notch's response at 0.05-0.3 Hz."""
import numpy as np
from scipy.signal import butter, filtfilt
G='/tmp/claude-1000/-home-wesley-multi-drone-control/d021810e-9d5d-427c-be70-40b127760327/scratchpad/gates'
def lpf(x,fs,fc=0.3):
    b,a=butter(2,fc/(fs/2)); return filtfilt(b,a,x,axis=0)
def rest_noise(tag):
    d=dict(np.load(f'{G}/{tag}.npz')); r=d['ring']
    m=r[:,4]<0.03
    idx=np.flatnonzero(m)
    # longest run before the first lift-off
    lift=np.flatnonzero(r[:,4]>0.05)
    end=lift[0] if len(lift) else len(r)
    idx=idx[idx<end]
    t=r[idx,0]; tg=np.arange(t[0],t[-1],0.01)
    xy=np.column_stack([np.interp(tg,t,r[idx,2]),np.interp(tg,t,r[idx,3])])
    x=lpf(xy-xy.mean(0),100)
    return tg[-1]-tg[0], 1000*float(np.sqrt(np.mean(np.sum(x**2,axis=1)))), 1000*float(np.sqrt(np.mean(np.sum((xy-xy.mean(0))**2,axis=1))))
for tag in ('sq5_2','sq2_5','k1_4'):
    dur,lp,allr=rest_noise(tag)
    print('%s body 8 on the floor before lift-off, %.0f s: xy below 0.3 Hz %.2f mm rms, all %.2f mm'%(tag,dur,lp,allr))
for tag in ('h4f_3','f4','ch1b'):
    d=dict(np.load(f'{G}/{tag}_sq.npz')); ring=d['payload_motion_capture_state']
    air=ring[:,3]>0.8
    t=ring[air,0]; tg=np.arange(t[0],t[-1],0.01)
    R=np.column_stack([np.interp(tg,t,ring[air,1]),np.interp(tg,t,ring[air,2])])
    mk=[]
    for i in range(4):
        tp=d[f'drone_{i}_magnet_tip_state']
        mk.append(np.column_stack([np.interp(tg,tp[:,0],tp[:,1]),np.interp(tg,tp[:,0],tp[:,2])]))
    M=np.mean(mk,axis=0)
    diff=(R-M)-(R-M).mean(0)
    # f4 flies a figure-8: compare the low-passed motion of each
    rl=lpf(R-R.mean(0),100); dl=lpf(diff,100)
    print('%s in the air %.0f s: ring xy below 0.3 Hz %.1f cm rms, ring minus the 4 plate markers\' mean below 0.3 Hz %.2f cm rms'%(tag,tg[-1]-tg[0],100*np.sqrt(np.mean(np.sum(rl**2,axis=1))),100*np.sqrt(np.mean(np.sum(dl**2,axis=1)))))
# notch: H(s) = (s^2 + w0^2) / (s^2 + (w0/Q) s + w0^2), f0 5.3 Hz, Q 1
w0=2*np.pi*5.3; Q=1.0
for f in (0.05,0.1,0.2,0.3,1.0):
    s=1j*2*np.pi*f; H=(s*s+w0*w0)/(s*s+(w0/Q)*s+w0*w0)
    print('notch at %.2f Hz: gain %.4f, phase %.2f deg, delay %.1f ms'%(f,abs(H),np.degrees(np.angle(H)),-1000*np.angle(H)/(2*np.pi*f)))
