"""Same real-catch replay on the carriers' own magnet markers (bodies 21-23, never frozen) in f4."""
import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
import drone_magnet.comotion_proof as cpm
import p16
from common import quat_rot, hold_idx
from collections import Counter
base=cpm.ComotionConfig
def mk(**kw):
    kw['taut_fallback']=0.0; return base(**kw)
p16.ComotionConfig=mk
def prep_i(tag,i):
    d=dict(np.load(f'{tag}_sq.npz'))
    ring=d['payload_motion_capture_state']; tp=d[f'drone_{i}_magnet_tip_state']; body=d[f'drone_{i}_motion_capture_state']
    fz=np.concatenate([[False],np.all(tp[1:,1:4]==tp[:-1,1:4],axis=1)])
    ir=hold_idx(ring[:,0],tp[:,0]); ib=hold_idx(body[:,0],tp[:,0])
    R=np.array([quat_rot(q) for q in ring[ir,4:8]])
    rel=np.einsum('kji,kj->ki',R,tp[:,1:4]-ring[ir,1:4]); rz=ring[ir,3]
    w0=np.median(rel[(rz<0.03)&~fz],axis=0); air=np.where(rz>0.8)[0]
    sd=rel[air][~fz[air]].std(axis=0)
    return dict(t=tp[:,0],tip=tp[:,1:4],ring_p=ring[ir,1:4],R=R,body=body[ib,1:4],fz=fz,w0=w0,air=air,rz=rz), sd
for i in range(4):
    P,sd=prep_i('f4',i)
    n,c,pr,tf,tp=p16.proof('f4',P,step=0.5)
    n2,c2,pr2,_,_=p16.proof('f4',P,step=0.5,skip=True,gap=0.35,win=0.5)
    print('f4 marker %d (body %d): airborne sd (%.1f, %.1f, %.1f) mm, frozen %.1f%%: as configured first proof %s, within 3 tries %d/%d | skip+gap.35+win.5: within 3 tries %d/%d'%(i,21+i,*(1e3*sd),100*P['fz'][P['air']].mean(),dict(c),pr,n,pr2,n2))
