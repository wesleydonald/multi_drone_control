"""Row 1's fresh-time variant with K 0.045 / loss 0.065 and a catch 4 cm off (synthetic, as p23)."""
import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
import drone_magnet.comotion_proof as cpm
import p23
base=cpm.ComotionConfig
def mk(**kw):
    kw.update(contact_xy=0.045, lost=0.065, sep_xy=0.065); return base(**kw)
p23.ComotionConfig=mk
for tag in ('sq5_2','sq2_5'):
    n,f1,pr,tp=p23.run_syn(tag, off=(0.04,0,0.002))
    print('%s fresh-time, K .045 loss .065, catch 4 cm off: first %.0f%%, within 3 %.0f%%, median %.1f s'%(tag,100*f1/n,100*pr/n,np.median(tp)))
