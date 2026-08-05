"""Headless Gazebo experiment harness (THESIS_PLAN §9.1).

Split from tools/run_experiment.py the same way tools/sil/ is split from
tools/sil_bench.py: the CLI orchestrates processes, this package holds the parts that
can be imported and unit-tested without ROS or Gazebo running.
"""
