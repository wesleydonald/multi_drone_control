#!/usr/bin/env bash
# chain_tj_load.sh (13 Nov 07:05): Te-Jen's line (swing 3, spline 2, splice 0.20, refresh 0.2, cap 2.9) under 8 busy loops
cd "$(dirname "$0")"
BKL=_z29s20_sw3_sp20 LF=nro LO=66.0 LX=0.30 LY=1.70 LZ=2.55 LR=90.0 ./load_run.sh 8 _tj_sw3_sp20_load8
BKL=_z29s20_sw3_sp20 LF=fm9hc_4 LO=70.0 LX=0.2873 LY=1.7320 LZ=2.7691 LR=50.0 ./load_run.sh 8 _tj_sw3_sp20_load8
BKL=_z29s20_sw3_sp20 LF=fm9hc_5 LO=47.0 LX=0.5947 LY=1.6132 LZ=2.7690 LR=50.0 ./load_run.sh 8 _tj_sw3_sp20_load8
