"""
sim_control_launch.py -- the controllers of a Gazebo or SIL run, one mode per experiment family.

    ros2 launch bringup sim_io_launch.py num_drones:=3             # FIRST: clock, poses, mocap, RViz
    ros2 launch bringup sim_control_launch.py mode:=mpc num_drones:=3 load_traj:=circle

mode:=
    mpc          carry on the load planner (mpc_planner)                    [default]
    free_hover   free_hover at target_z instead of the planner (no ring)
    dissipative  carry + mid-flight detach (/fleet/detach k); --detachable world
    network      the dissipative network flies the whole flight after the lift
    attach       three carriers + a newcomer that welds on (M1); sil:=true for tools/sil_bench.py

Defaults: bringup/config/common.yaml, sim.yaml and its modes: entry (bringup/profiles.py).
sim.yaml is the rig twin (the *_rig worlds). legacy:=true flies the worlds in
simulation_assets/old_worlds/ on sim_legacy.yaml; network and attach exist only there.
Knobs that are not launch arguments go in params_file:=<yaml>. What each mode starts, and why:
bringup/graph_carry.py, graph_detach.py, graph_attach.py.
"""
import os

from bringup import graph_attach, graph_carry, graph_detach, profiles
from bringup.real_mode import record_typed_args
from launch import LaunchDescription
from launch.actions import OpaqueFunction

GRAPHS = {'mpc': graph_carry.sim_mpc, 'free_hover': graph_carry.sim_mpc,
          'dissipative': graph_detach.dissipative, 'network': graph_detach.sim_network,
          'attach': graph_attach.attach}


def launch_setup(context, *args, **kwargs):
    mode, vals = profiles.apply(context, 'sim')
    return GRAPHS[mode](context, os.path.dirname(os.path.abspath(__file__)), vals)


def generate_launch_description():
    # record_typed_args first: a typed value is told from a profile default by it
    return LaunchDescription([OpaqueFunction(function=record_typed_args)]
                             + profiles.declarations('sim')
                             + [OpaqueFunction(function=launch_setup)])
