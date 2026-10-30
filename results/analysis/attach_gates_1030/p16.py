"""A real caught magnet with real freezes: QUAD4's marker (body 24) on plate 9 in the square flights of 29 Oct, through
our ComotionProof as the rig configures it (K 0.024, P 1.038 -> taut fallback 1.041, lift-done 2.0 s after contact)."""
import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
from common import quat_rot, hold_idx
from collections import Counter
def prep(tag):
    d=dict(np.load(f'{tag}_sq.npz'))
    ring=d['payload_motion_capture_state']; tp=d['drone_3_magnet_tip_state']; body=d['drone_3_motion_capture_state']
    fz=np.concatenate([[False],np.all(tp[1:,1:4]==tp[:-1,1:4],axis=1)])
    ir=hold_idx(ring[:,0],tp[:,0]); ib=hold_idx(body[:,0],tp[:,0])
    R=np.array([quat_rot(q) for q in ring[ir,4:8]])
    rel=np.einsum('kji,kj->ki',R,tp[:,1:4]-ring[ir,1:4])
    rz=ring[ir,3]
    w0=np.median(rel[(rz<0.03)&~fz],axis=0)
    air=np.where((rz>0.8))[0]
    return dict(t=tp[:,0],tip=tp[:,1:4],ring_p=ring[ir,1:4],R=R,body=body[ib,1:4],fz=fz,w0=w0,air=air,rz=rz)
def freeze_stats(tag,P):
    a=P['air']; t=P['t'][a]; fz=P['fz'][a]
    runs=[];i=0
    while i<len(fz):
        if fz[i]:
            j=i
            while j<len(fz) and fz[j]: j+=1
            runs.append(t[j-1]-t[i-1]); i=j
        else: i+=1
    fr=t[fz]
    ok={}
    for W in (0.3,1.0):
        xs=np.arange(t[0],t[-1]-W,0.01); c=0
        for x in xs:
            k=np.searchsorted(fr,x)
            c+= not (k<len(fr) and fr[k]<=x+W)
        ok[W]=c/len(xs)
    btt=np.linalg.norm(P['tip'][a]-P['body'][a],axis=1)[~fz]
    print('%s airborne %.1f s: body 24 frozen %.1f%%, holds %d (%.2f/s) median %.2f s max %.2f; freeze-free 0.3 s %.0f%%, 1.0 s %.0f%%; body-to-tip median %.4f p5 %.4f'%(tag,t[-1]-t[0],100*fz.mean(),len(runs),len(runs)/(t[-1]-t[0]),np.median(runs) if runs else 0,max(runs) if runs else 0,100*ok[0.3],100*ok[1.0],np.median(btt),np.percentile(btt,5)))
def proof(tag,P,shift=(0,0,0),K=0.024,skip=False,gap=0.1,win=0.2,step=0.5,lost=None):
    a=P['air']; w=P['w0']+np.asarray(shift)
    kw=dict(contact_xy=K,lift_fallback_s=0.0,taut_fallback=1.041,gap_s=gap,speed_window_s=win)
    if lost: kw.update(lost=lost,sep_xy=lost)
    cfg=ComotionConfig(**kw)
    t=P['t']; out=[]
    for tc in np.arange(t[a[0]]+0.5,t[a[-1]]-8,step):
        p=ComotionProof(cfg); p.magnet(True); tcon=None; res=None; nf=0; first=None; tfirst=None
        k=np.searchsorted(t,tc)
        while k<a[-1] and t[k]<tc+15:
            if skip and P['fz'][k]: p.tick(t[k]); k+=1; continue
            plate=P['ring_p'][k]+P['R'][k]@w
            ev=p.step(t[k],P['tip'][k],plate,body=P['body'][k],fresh=True,rotation=P['R'][k],lift_done=(tcon is not None and t[k]>=tcon+2.0))
            if ev is None: ev=p.tick(t[k])
            if ev=='contact':
                tcon=t[k]
                if tfirst is None: tfirst=t[k]-tc
            if ev=='failed' and tcon is not None:
                nf+=1; tcon=None
                if first is None: first='fail '+p.last_fail['reason']
                if nf>=3: res='gave up'; break
            if ev=='proven':
                res='proven'; first=first or 'proven'; break
            k+=1
        out.append((res,first,tfirst,(t[k]-tc) if res=='proven' else None))
    n=len(out); c=Counter(o[1] for o in out)
    tp=[o[3] for o in out if o[3] is not None]; tf=[o[2] for o in out if o[2] is not None]
    return n,c,sum(o[0]=='proven' for o in out),tf,tp
if __name__=='__main__':
    for tag in ('h4f_3','f4','ch1b'):
        P=prep(tag); freeze_stats(tag,P)
        for lab,kw in (('as configured (K .024)',{}),
                       ('weld point 3 cm off, K .024',dict(shift=(0.03,0,0))),
                       ('weld point 3 cm off, K .045, lost .065',dict(shift=(0.03,0,0),K=0.045,lost=0.065)),
                       ('skip repeats, gap .35, speed window .5',dict(skip=True,gap=0.35,win=0.5)),
                       ('skip + K .045 lost .065, 3 cm off',dict(shift=(0.03,0,0),K=0.045,lost=0.065,skip=True,gap=0.35,win=0.5))):
            n,c,pr,tf,tp=proof(tag,P,**kw)
            print('   %-42s n %d: contact %d/%d (median %.2f s); first proof %s; proven within 3 tries %d/%d, median %.1f s'%(lab,n,len(tf),n,np.median(tf) if tf else np.nan,dict(c),pr,n,np.median(tp) if tp else np.nan))
