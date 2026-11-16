# Cooperative transport of a cable-suspended ring with quadrotors that detach and re-attach

The code behind the experiments of the combined paper (Wesley Donald, Te-Jen Yu and supervisors): a team of
quadrotors carries a ring on cables; one quadrotor detaches, flies a task (picks up an object and drops it into the
moving ring), and re-attaches to the ring in flight. ROS 2 Humble; the real quadrotors fly through ELRS radios with
motion capture; the simulation is Gazebo Harmonic.

[EXPERIMENTS.md](EXPERIMENTS.md) lists every experiment of the paper: the command that ran it and the code version.

## Packages
| Package | What it does |
|---|---|
| `mpc_planner` | The proposed planner: one optimal control problem over the ring, its cables and every quadrotor; it plans each quadrotor's reference and cable tension. Team reconfiguration resizes it for one fewer or one more quadrotor. |
| `dissipative_planner` | The node that runs the planner in flight: lift-off, the path of the ring, detachment (unload, then release), attachment (the fold-in of a returning quadrotor), the hand-over to and from the collaborator's controller, LAND. |
| `tracker` | Each quadrotor's model predictive controller (acados) that follows the planner's reference. |
| `fleet_manager` | Arming, takeoff, LAND and the safety checks of the whole team. |
| `drone_magnet` | The magnets on the cable ends, the attachment check (contact, a small lift, then the magnet moving with the plate), the radio switch between our controller and the collaborator's. |
| `drone_communication` | The ELRS radio interface and the motion-capture stream. |
| `bringup` | Launch files (`real_control_launch.py`, `sim_control_launch.py`; `mode:=attach`) and the configuration profiles. |
| `simulation_communication`, `drone_visualisation` | The Gazebo interfaces and the airframe model. |
| `interfaces`, `utility_objects` | Messages and shared helpers. |
| `tejen_mpc`, `tejen_dynamic_planner`, `tejen_mission`, `tejen_utility_objects` | Te-Jen Yu's single-quadrotor controller, his path planner (C++) and his mission (pickup, drop-off, attachment), as integrated here. Their origin is in each `PROVENANCE.txt`; the changes made for this work are in `src/tejen_mission/CHANGES_FROM_TEJEN.md`. |

Also: `experiments/rig/` (the scripts that flew the real-world experiments), `configs/` (their parameter files and
the simulation experiments), `simulation_assets/` (the simulated worlds), `tools/` (the flight wrapper and console,
the simulation runner, the planner prebuild).

## Build
Needs ROS 2 Humble, [acados](https://github.com/acados/acados) with its Python interface (`acados_template`), CasADi,
NumPy, SciPy, PyYAML, Matplotlib, Eigen 3, GLPK and qpOASES; for the simulation Gazebo Harmonic and `ros_gz`.
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 tools/prebuild_planner.py --from-config configs/experiments/<experiment>.yaml   # the planner's solvers
```

## Run
- Simulation: `python3 tools/run_experiment.py configs/experiments/<experiment>.yaml` (headless; one run).
- Real world: `bash experiments/rig/<script>.sh`, with the console `python3 tools/console.py` in a second terminal.
  The scripts expect our lab: the motion-capture forwarder on UDP 1511 and the quadrotors' radios.
