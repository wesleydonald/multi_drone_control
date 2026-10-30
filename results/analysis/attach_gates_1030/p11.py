import sys
import numpy as np
sys.path.insert(0, '/home/wesley/multi_drone_control/src/drone_magnet')
import drone_magnet.comotion_proof as cp
import p8
orig=cp.ComotionConfig
def mk(**kw):
    kw.setdefault('speed_window_s',0.5)
    return orig(**kw)
p8.ComotionConfig=mk
for tag in ('sq5_2','sq2_5'):
    p8.show(tag,'skip repeats, gap .35, speed window .5',p8.run(tag,(0.01,0,0.002),skip=True,gap=0.35))
    p8.show(tag,'skip, gap .35, window .5, K .045 off 4 cm',p8.run(tag,(0.04,0,0.002),skip=True,gap=0.35,K=0.045,lost=0.065))
    p8.show(tag,'as flown, window .5 only',p8.run(tag,(0.01,0,0.002)))
