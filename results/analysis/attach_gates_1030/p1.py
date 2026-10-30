import json, numpy as np
from common import *
for tag in ('sq5_2','sq2_5','t6h_4','a1'):
    d,s=load(tag)
    tip=d['tip']; dt=np.diff(tip[:,0])
    print('==',tag,'tip rate %.0f Hz'%(1/np.median(dt)),'stamp rate %.0f'%(1/np.median(np.diff(tip[:,1]))), 'bag dur %.0f s'%(tip[-1,0]-tip[0,0]))
    sp=phase_spans(s)
    t0=None
    for a,b,v in sp:
        if v=='TAKEOFF' and t0 is None: t0=a
    for a,b,v in sp:
        if t0 and a>=t0-1: print('  %7.2f %6.2f %s'%(a-t0,b-a,v))
    # proof states
    pr=[(t,json.loads(v)) for t,v in s['/magnet/attach_proof']]
    st=[(t,p['state']) for t,p in pr]
    last=None
    for t,v in st:
        if v!=last: print('   proof %7.2f %s'%(t-t0,v)); last=v
    coil=s.get('/drone_3/magnet_coil',[])
    last=None
    for t,v in coil:
        if v!=last: print('   coil %7.2f %s'%(t-t0,v)); last=v
    c=s.get('/magnet/contact',[])
    print('   contact true msgs', sum(1 for t,v in c if v))
