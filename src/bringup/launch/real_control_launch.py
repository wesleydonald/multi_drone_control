"""
real_control_launch.py -- the controllers on the rig (wall clock, real radios and mocap).

    ros2 launch bringup real_io_launch.py num_drones:=4             # terminal 1, FIRST
    ros2 launch bringup real_control_launch.py mode:=mpc num_drones:=4 target_z:=1.0

mode:=
    mpc          carry on the load planner                                    [default]
    free_hover   no ring: each drone climbs to hover_z (kT of an airframe)
    dissipative  carry + detach: the OCP resizes and the magnet releases on /fleet/detach k
    attach       M1 on the rig: ONE launch, includes real_io_launch.py itself
    m2           M2 on the rig (partner_m2:=true): ONE launch, includes real_io_launch.py

mpc, free_hover and dissipative run after real_io_launch.py in another terminal; attach and m2
start the rig I/O themselves (real_io:=false when another stack owns mocap and radios).
Defaults: bringup/config/common.yaml, real.yaml and its modes: entry (bringup/profiles.py):
the 30 Sep 2026 lab values. Knobs that are not launch arguments go in params_file:=<yaml>.
What each mode starts, and why: bringup/graph_carry.py, graph_detach.py, graph_attach.py.
"""
import os

from bringup import graph_attach, graph_carry, graph_detach, profiles
from bringup.real_mode import record_typed_args
from launch import LaunchDescription
from launch.actions import OpaqueFunction

GRAPHS = {'mpc': graph_carry.real_mpc, 'free_hover': graph_carry.real_free_hover,
          'dissipative': graph_detach.real_dissipative, 'attach': graph_attach.attach,
          'm2': graph_detach.dissipative}


def launch_setup(context, *args, **kwargs):
    mode, vals = profiles.apply(context, 'real')
    return GRAPHS[mode](context, os.path.dirname(os.path.abspath(__file__)), vals)


def generate_launch_description():
    # record_typed_args first: a typed value is told from a profile default by it
    return LaunchDescription([OpaqueFunction(function=record_typed_args)]
                             + profiles.declarations('real')
                             + [OpaqueFunction(function=launch_setup)])
