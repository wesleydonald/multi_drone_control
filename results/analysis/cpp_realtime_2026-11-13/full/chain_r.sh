#!/usr/bin/env bash
# chain_r.sh <BK suffix> <tag>: the four 12 Nov flights in real time with backend<BK>.yaml
cd "$(dirname "$0")"
BK=$1 ./run_flight.sh fm9_5 135.0 -0.6209 1.6300 2.6934 50.0 $2
BK=$1 ./run_flight.sh fm9hc_2 55.0 0.7875 1.7022 2.7650 50.0 $2
BK=$1 ./run_flight.sh fm9hc_4 70.0 0.2873 1.7320 2.7691 50.0 $2
BK=$1 ./run_flight.sh fm9hc_5 47.0 0.5947 1.6132 2.7690 50.0 $2
BK=$1 ./run_flight.sh nro 66.0 0.30 1.70 2.55 90.0 $2
