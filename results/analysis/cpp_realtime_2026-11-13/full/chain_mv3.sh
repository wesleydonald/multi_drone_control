#!/usr/bin/env bash
# chain_mv3.sh (13 Nov 03:12): after chain_mv2.sh (pid 279224): spline 3, splice 0.20, refresh 0.2, cap 2.9 on the moving windows
# (nro x2, fm9hc_4/_2/_5), nro under the 8-loop stress, and the still case fm9_5 (spline 3 must not break it)
while kill -0 279224 2>/dev/null; do sleep 5; done
cd "$(dirname "$0")"
BK=_z29s20sp3 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z29s20sp3
BK=_z29s20sp3 ./run_flight.sh fm9hc_4 70.0 0.2873 1.7320 2.7691 50.0 _mv_z29s20sp3
BK=_z29s20sp3 ./run_flight.sh fm9hc_2 55.0 0.7875 1.7022 2.7650 50.0 _mv_z29s20sp3
BK=_z29s20sp3 ./run_flight.sh fm9hc_5 47.0 0.5947 1.6132 2.7690 50.0 _mv_z29s20sp3
BKL=_z29s20sp3 LF=nro LO=66.0 LX=0.30 LY=1.70 LZ=2.55 LR=90.0 ./load_run.sh 8 _mv_z29s20sp3_load8
BKL=_z29s20sp3 ./load_run.sh 8 _z29s20sp3_load8
BK=_z29s20sp3 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z29s20sp3_r2
