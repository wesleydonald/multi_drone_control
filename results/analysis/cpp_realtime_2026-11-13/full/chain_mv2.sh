#!/usr/bin/env bash
# chain_mv2.sh (13 Nov 03:10): after chain_mv.sh (pid 277149): moving windows of fm9hc_2/_4/_5 and a repeat of nro, cap 3.8 + splice 0.20; nro with cap 2.9 + splice 0.20
while kill -0 277149 2>/dev/null; do sleep 5; done
cd "$(dirname "$0")"
BK=_z38s20 ./run_flight.sh fm9hc_4 70.0 0.2873 1.7320 2.7691 50.0 _mv_z38s20
BK=_z38s20 ./run_flight.sh fm9hc_2 55.0 0.7875 1.7022 2.7650 50.0 _mv_z38s20
BK=_z38s20 ./run_flight.sh fm9hc_5 47.0 0.5947 1.6132 2.7690 50.0 _mv_z38s20
BK=_z38s20 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z38s20_r2
BK=_z29s20 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _mv_z29s20
