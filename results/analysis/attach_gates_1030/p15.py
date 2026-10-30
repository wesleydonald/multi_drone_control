"""Square flights: each carrier's magnet marker in the ring frame, at rest vs airborne (ring z > 0.8 m)."""
import numpy as np
from common import quat_rot, hold_idx
for tag in ('h4f_3','f4','ch1b'):
    d=dict(np.load(f'{tag}_sq.npz'))
    ring=d['payload_motion_capture_state']
    print('==',tag)
    for i in range(4):
        k=f'drone_{i}_magnet_tip_state'
        tp=d[k]
        fz=np.concatenate([[False],np.all(tp[1:,1:4]==tp[:-1,1:4],axis=1)])
        ir=hold_idx(ring[:,0],tp[:,0])
        rel=np.array([quat_rot(ring[j,4:8]).T@(tp[n,1:4]-ring[j,1:4]) for n,j in enumerate(ir)])
        rz=ring[ir,3]
        rest=(rz<0.03)&~fz
        air=(rz>0.8)&~fz
        if rest.sum()<20 or air.sum()<20: 
            print(i,'insufficient',rest.sum(),air.sum()); continue
        r0=np.median(rel[rest],axis=0); r1=np.median(rel[air],axis=0)
        sd=rel[air].std(axis=0)
        print('  marker %d: rest (%.1f, %.1f, %+.1f) mm, airborne median (%.1f, %.1f, %+.1f), shift (%+.1f, %+.1f, %+.1f) mm, airborne sd (%.1f, %.1f, %.1f); frozen in air %.1f%%'%(i,*(1e3*r0),*(1e3*r1),*(1e3*(r1-r0)),*(1e3*sd),100*fz[rz>0.8].mean()))
