#!/usr/bin/env bash
# chain_tj.sh (13 Nov 06:52, Te-Jen's ask): swing 3 deg + spline 2.0 / 2.5, outer_octagon, tracking 0.06, refresh 0.2,
# splice 0.20, cap 2.9; the moving circle (nro 90 s, fm9hc_4/_2/_5 windows) and the still hover (fm9_5)
cd "$(dirname "$0")"
for t in 20 25; do BK=_z29s20_sw3_sp$t ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _tj_sw3_sp$t; done
for t in 20 25; do BK=_z29s20_sw3_sp$t ./run_flight.sh fm9hc_4 70.0 0.2873 1.7320 2.7691 50.0 _tj_sw3_sp$t; done
for t in 20 25; do BK=_z29s20_sw3_sp$t ./run_flight.sh fm9_5 135.0 -0.6209 1.6300 2.6934 50.0 _tj_sw3_sp$t; done
for t in 20 25; do BK=_z29s20_sw3_sp$t ./run_flight.sh fm9hc_2 55.0 0.7875 1.7022 2.7650 50.0 _tj_sw3_sp$t; done
for t in 20 25; do BK=_z29s20_sw3_sp$t ./run_flight.sh fm9hc_5 47.0 0.5947 1.6132 2.7690 50.0 _tj_sw3_sp$t; done
for t in 20 25; do BK=_z29s20_sw3_sp$t ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _tj_sw3_sp${t}_r2; done
