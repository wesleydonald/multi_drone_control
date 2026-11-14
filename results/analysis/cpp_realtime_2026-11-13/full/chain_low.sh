#!/usr/bin/env bash
# chain_low.sh (13 Nov 07:27, Wesley: lower the limits): A swing 2 + carrier tracking 0.04; B swing 1 + tracking 0.03; spline 2, splice 0.20, refresh 0.2, cap 2.9
cd "$(dirname "$0")"
for v in A B; do
  BK=_low$v ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 _low$v
  BK=_low$v ./run_flight.sh fm9hc_4 70.0 0.2873 1.7320 2.7691 50.0 _low$v
  BK=_low$v ./run_flight.sh fm9hc_5 47.0 0.5947 1.6132 2.7690 50.0 _low$v
done
for v in A B; do BK=_low$v ./run_flight.sh fm9hc_2 55.0 0.7875 1.7022 2.7650 50.0 _low$v; done
