import numpy as np
import p16
from collections import Counter
orig=p16.ComotionConfig
def mk(**kw):
    kw['taut_fallback']=0.0   # the square's 60 deg cable shortens body-to-tip by ~5 mm: not the attach geometry
    return orig(**kw)
p16.ComotionConfig=mk
for tag in ('f4','ch1b'):
    P=p16.prep(tag)
    for lab,kw in (('as configured (K .024), taut off',{}),
                   ('3 cm off, K .045 lost .065, taut off',dict(shift=(0.03,0,0),K=0.045,lost=0.065)),
                   ('no freeze (repeats skipped) + gap .35 + window .5',dict(skip=True,gap=0.35,win=0.5)),
                   ('repeats skipped only (gap .1, window .2)',dict(skip=True))):
        n,c,pr,tf,tp=p16.proof(tag,P,step=0.25,**kw)
        print('%s %-50s n %d: contact %d/%d (median %.2f s); first proof %s; proven within 3 tries %d/%d, median %.1f s'%(tag,lab,n,len(tf),n,np.median(tf) if tf else np.nan,dict(c),pr,n,np.median(tp) if tp else np.nan))
