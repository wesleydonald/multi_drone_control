# For Mitch, 16 Oct: what the rig data says about the discs, the cables and the basket (draft for Wesley)

## Why reattaching is unreliable (8 rig attach flights, 14-15 Oct)
- **The magnet is rarely over the plate at the bottom.** Within the catch radius 22 % of the time (24 mm today).
- **When it misses, it falls off the edge.** The bottom-hold "slides" were the magnet reaching plate height 3.3-4.3 cm
  along the ring from the plate centre, past the 60 mm plate's edge.
- **The ring bounces.** About 0.8 cm at 1.8 Hz under QUAD4. QUAD4 followed it a quarter of a second late, doubling the
  magnet's bounce against the plate. A software change that holds QUAD4's height still is being checked in simulation.
- **The approach can cross a carrier.** On one flight QUAD4's tether passed 10 cm from a carrier on the way in.

## The hardware, in numbers

| Change | Effect (from the rig data and the geometry) |
|---|---|
| 90 mm discs (45 mm catch radius) | time over the plate at the bottom 22 % -> 49 %; all three edge slips would have landed on the plate |
| 110 mm discs | 58 % |
| 1 m cables on the carriers | QUAD4's room from the nearest carrier 0.45 -> 0.70 m |
| 1 m tether on QUAD4 | the same room, and QUAD4 hovers 1.06 m above the ring instead of 0.64 (less downwash on it: on 14 Oct the ring tilted 5-8 deg with QUAD4 overhead, 1-3 without); slower swing (0.50 Hz vs 0.66) |
| Smaller basket (radius 0.225 -> 0.15 m) with 1 m cables | costs only 0.04 m of QUAD4's room, but the neighbouring rods come 0.23 -> 0.15 m from QUAD4's tether at the ring; twelve 90 mm discs need a radius of at least ~0.18 m (only plates 3, 7, 9 and 11 are used, so fewer plates would allow less) |

## Simulation, overnight
- **1 m cables:** the three carriers fly the paths tried (circle, figure-8, line, orbit at 0.125-0.2 m/s) as well as on
  today's 0.55 m rods (which fly every path up to 0.3 m/s), and the full
  attach (contact, proof, fold-in, landing) works with 1 m cables on all four drones.
- **Latency:** the carry tolerates 30 ms of extra command delay at 0.3 m/s; 45 ms tips a drone at takeoff.

## What would help most
1. Bigger discs first (the clearest effect on the catch).
2. 1 m tethers, at least on QUAD4.
3. The carriers' magnet hold measured once (one let go mid-flight on 15 Oct): a spring-scale pull per carrier, cold
   and after ~20 min switched ON, on a full and a low pack (the coils stay ON from each launch, and nothing records
   their current or temperature).
