#!/usr/bin/env bash
# chain_mv.sh: the moving circle (nro, 90 s) at real time, cap 3.8, splice 0.20, refresh 0.2; spline 4 / 3 / 2 (13 Nov 03:10)
cd "$(dirname "$0")"
BK=_z38s20 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z38s20
BK=_z38s20sp3 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z38s20sp3
BK=_z38s20sp2 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z38s20sp2
