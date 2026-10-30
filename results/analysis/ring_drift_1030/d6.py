"""QUAD4 over the ring (sq5_2, sq2_5 MATCH windows): the ring's displacement toward plate 9 and its tilt against
QUAD4's height above the ring and its horizontal distance from plate 9 (bags, 100 Hz, low-passed at 0.3 Hz)."""
import sys, json
import numpy as np
from scipy.signal import butter, filtfilt
sys.path.insert(0,'/tmp/claude-1000/-home-wesley-multi-drone-control/d021810e-9d5d-427c-be70-40b127760327/scratchpad/gates')
from common import load, quat_rot, hold_idx, phase_spans
def lpf(x,fs=100,fc=0.3):
    b,a=butter(2,fc/(fs/2)); return filtfilt(b,a,x,axis=0)
for tag in ('sq5_2','sq2_5'):
    d,s=load(tag); ring=d['ring']; body=d['body']
    sp=phase_spans(s); a0=[a for a,b,v in sp if v=='MATCH_VELOCITY'][0]; a1=[a for a,b,v in sp if v=='TAKEN_BACK'][0]
    tg=np.arange(a0,a1,0.01)
    ir=hold_idx(ring[:,0],tg); ib=hold_idx(body[:,0],tg)
    R=np.array([quat_rot(q) for q in ring[ir,5:9]])
    c=ring[ir,2:5]; q4=body[ib,2:5]
    u9=R[:,:,1]                      # ring +y (toward plate 9), world
    disp=np.einsum('ki,ki->k',c[:,:2]-c[:,:2].mean(0),u9[:,:2]/np.linalg.norm(u9[:,:2],axis=1,keepdims=True))
    tilt9=np.degrees(np.arcsin(-R[:,2,1]))   # ring +y axis's z component: plate 9 side down = positive
    hz=q4[:,2]-c[:,2]
    p9=c+np.einsum('kij,j->ki',R,np.array([-0.005,0.197,0.028]))
    hd=np.linalg.norm(q4[:,:2]-p9[:,:2],axis=1)
    D,T,H=lpf(disp-disp.mean()),lpf(tilt9-tilt9.mean()),lpf(hz-hz.mean())
    print('%s MATCH..TAKEN_BACK %.0f s: QUAD4 %.2f-%.2f m above the ring (median %.2f), %.2f m (median) from plate 9 in xy; ring toward plate 9 LP sd %.1f cm, plate-9-down tilt LP sd %.2f deg (mean %+.1f); corr(height, displacement) %+.2f, corr(height, tilt) %+.2f, corr(tilt, displacement) %+.2f'%(
        tag,a1-a0,hz.min(),hz.max(),np.median(hz),np.median(hd),100*D.std(),T.std(),tilt9.mean(),np.corrcoef(H,D)[0,1],np.corrcoef(H,T)[0,1],np.corrcoef(T,D)[0,1]))
