"""Out-of-loop thrust-ratio estimator: one node per drone.

WHY THIS IS A SEPARATE NODE
    The parameter UKF is 39 acados propagations per update. Benchmarked alone that
    is ~4.5 ms, but measured in a live 3-drone run -- three controller processes,
    Gazebo, the planner and RViz all competing for cores -- it took 25-150 ms
    against the tracker's 20 ms control period. Run inside controller_mpc's timer
    that STALLS the control loop, so the tracker degraded even in shadow mode where
    its output was not being used at all. An estimator must never be able to do
    that. Here it runs in its own process: if it falls behind, it simply produces
    estimates less often, and the 50 Hz control loop never waits for it.

WIRE FORMAT (Float64MultiArray, the same convention as the planner reference topic)
    in   /drone_N/kt_input     [t, pose(13), u_state(4), u_rate(4), a_cable(3), airborne]
                                = 26 doubles, published by the tracker every cycle
    out  /drone_N/kt_estimate  [kT, std, update_count, solve_ms, status_code]

    Consecutive kt_input samples bracket one propagation interval: predict from
    sample k's pose with sample k's applied control, correct against sample k+1's
    measured pose. So the node needs no clock sync with the tracker and tolerates
    dropped messages -- it just uses whatever interval it actually received.
"""

import os
import fcntl

# Own acados working directory. The chdir must happen before the solver is loaded
# (acados resolves its generated sources relative to cwd), but the compile LOCK is
# taken inside __init__ rather than here: an import-time exclusive lock on the same
# file controller_mpc locks means merely importing both modules into one process
# deadlocks, which is a trap for any test or tool that loads both.
# Resolved from the workspace root (or MDC_ACADOS_ROOT), not a literal path, so
# the repo is not pinned to one machine's home directory. Must match the
# directory controller_mpc uses -- they share the generated solver.
from utility_objects.run_context import acados_dir                 # noqa: E402

ACADOS_DIR = acados_dir('quad_load')
os.chdir(ACADOS_DIR)

import signal                                                    # noqa: E402
import sys                                                       # noqa: E402
import time                                                      # noqa: E402

import numpy as np                                               # noqa: E402
import rclpy                                                     # noqa: E402
from rclpy.node import Node                                      # noqa: E402
from std_msgs.msg import Float64MultiArray                       # noqa: E402

from .acados import generate_sim_integrator                      # noqa: E402
from .thrust_ratio_ukf import ThrustRatioUKF, N_POSE             # noqa: E402

IN_LEN = 26
STATUS_CODES = {
    'updated': 0.0, 'initialized': 1.0, 'pose_reseeded': 2.0,
    'bad_measurement': 3.0, 'propagation_failed': 4.0,
    'singular_innovation': 5.0, 'not_initialized': 6.0,
}


class ThrustRatioEstimatorNode(Node):
    def __init__(self):
        super().__init__('thrust_ratio_estimator')

        self.drone_id = int(self.declare_parameter('drone_id', 0).value)
        # Seed and band centre. Must match the tracker's kt_seed. This is the drone's
        # ACTUAL hover kT -- deliberately NOT the tracker's thrust_ratio, which is the
        # takeoff constant and is kept low on purpose to pop the drones off their
        # stands. kt_seed <= 0 falls back to thrust_ratio (the old coupled behaviour).
        _tr = float(self.declare_parameter('thrust_ratio', 24.0).value)
        _seed = float(self.declare_parameter('kt_seed', 0.0).value)
        self.seed = _seed if _seed > 0.0 else _tr
        dev = float(self.declare_parameter('kt_max_deviation', 0.15).value)
        kt_min = float(self.declare_parameter('kt_min', 12.0).value)
        kt_max = float(self.declare_parameter('kt_max', 60.0).value)
        if dev > 0.0:
            kt_min = max(kt_min, self.seed * (1.0 - dev))
            kt_max = min(kt_max, self.seed * (1.0 + dev))
        if kt_min >= kt_max:
            self.get_logger().warn('empty kT band; falling back to 12..60')
            kt_min, kt_max = 12.0, 60.0
        self.kt_min, self.kt_max = kt_min, kt_max

        q_kt = float(self.declare_parameter('ukf_q_kt', 0.001).value)
        meas_std = float(self.declare_parameter('ukf_meas_std', 0.05).value)
        # Minimum interval between updates. The node is free-running now, so this is
        # about CPU courtesy to the controllers sharing the machine, not deadlines.
        self.min_period_s = 1.0 / max(
            1.0, float(self.declare_parameter('ukf_rate_hz', 25.0).value))
        # Reject an interval that is not a plausible one-step propagation: the applied
        # control is assumed constant across it, which stops being true if the tracker
        # stalled or messages were dropped.
        self.max_dt_s = float(self.declare_parameter('ukf_max_dt_s', 0.20).value)
        self.print_period_s = float(
            self.declare_parameter('kt_print_period_s', 1.0).value)

        # Serialise code generation across the N estimator processes (and against the
        # trackers, which lock the same file) so they cannot race into the same build.
        sim_dt = 1.0 / 50.0
        lock = open(os.path.join(ACADOS_DIR, '.compile.lock'), 'w')
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            fresh = _sim_is_fresh()
            self.get_logger().info(
                f"[kT node {self.drone_id}] "
                f"{'loading cached' if fresh else 'compiling'} sim integrator...")
            self.sim = generate_sim_integrator(
                sim_dt, generate=not fresh, build=not fresh)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)
            lock.close()

        self.ukf = ThrustRatioUKF(
            initial_params=np.array([self.seed, 0.0, 0.12, 70.0, 670.0, 0.5]),
            measurement_std=meas_std,
            param_process_std=np.sqrt(np.array([q_kt, 1e-5, 1e-5, 1.0, 1.0, 0.1])))
        self.ukf.param_min[0] = self.kt_min
        self.ukf.param_max[0] = self.kt_max

        self.prev = None                 # previous kt_input sample
        self.last_update_t = None        # wall time of the last accepted update
        self.last_print_t = None
        self.solve_ms = 0.0
        self.q_ref = np.array([1.0, 0.0, 0.0, 0.0])

        self.pub = self.create_publisher(
            Float64MultiArray, f'/drone_{self.drone_id}/kt_estimate', 1)
        self.create_subscription(
            Float64MultiArray, f'/drone_{self.drone_id}/kt_input', self._on_input, 5)

        self.get_logger().info(
            f"[kT node {self.drone_id}] ready. seed={self.seed:.2f}, "
            f"band {self.kt_min:.2f}..{self.kt_max:.2f}, "
            f"q_kt={q_kt:g}, <={1.0/self.min_period_s:.0f} Hz.")

    # ─────────────────────────────────────────────────────────────────────

    def _on_input(self, msg: Float64MultiArray):
        d = np.asarray(msg.data, dtype=float)
        if d.size < IN_LEN:
            return
        sample = {
            't': float(d[0]),
            'pose': d[1:14].copy(),
            'u_state': d[14:18].copy(),
            'u_rate': d[18:22].copy(),
            'a_cable': d[22:25].copy(),
            'airborne': bool(d[25] > 0.5),
        }
        prev, self.prev = self.prev, sample

        if not sample['airborne']:
            # Held: keep the pose block fresh but KEEP the learned parameters --
            # re-seeding them here would mean a momentary dip below the tracker's
            # airborne gate throws away the whole flight's estimate.
            self.ukf.seed_pose(sample['pose'])
            self._publish('waiting_for_airborne')
            return
        if prev is None or not prev['airborne']:
            self.ukf.seed_pose(sample['pose'])
            self._publish('waiting_for_interval')
            return

        now = time.monotonic()
        if (self.last_update_t is not None
                and now - self.last_update_t < self.min_period_s):
            self._publish('rate_limited')
            return

        dt = sample['t'] - prev['t']
        if not (1e-4 < dt <= self.max_dt_s):
            # Stalled tracker or dropped messages: the constant-control assumption
            # across the interval no longer holds, so skip rather than corrupt kT.
            self.ukf.seed_pose(sample['pose'])
            self._publish('bad_interval')
            return
        self.last_update_t = now

        u_state, u_rate = prev['u_state'], prev['u_rate']
        a_cable, sim = prev['a_cable'], self.sim

        def propagate(pose, params):
            sim.set('T', dt)
            sim.set('x', np.concatenate([pose, u_state]))
            sim.set('u', u_rate)
            sim.set('p', np.concatenate([params, a_cable, self.q_ref]))
            if sim.solve() != 0:
                return None
            return sim.get('x')[:N_POSE]

        t0 = time.perf_counter()
        try:
            # Predict from the PREVIOUS pose over the interval its control produced,
            # and correct against the pose measured at the end of it.
            self.ukf.x[:N_POSE] = prev['pose']
            result = self.ukf.update(sample['pose'], propagate)
            status = str(result['status'])
        except Exception as exc:
            status = 'error'
            self.get_logger().warn(f'UKF update failed, kT unchanged: {exc}',
                                   throttle_duration_sec=2.0)
        self.solve_ms = (time.perf_counter() - t0) * 1000.0
        self._publish(status)
        self._report(status)

    def _publish(self, status):
        s = self.ukf.snapshot()
        m = Float64MultiArray()
        m.data = [float(s['thrust_ratio']), float(s['thrust_ratio_std']),
                  float(s['update_count']), float(self.solve_ms),
                  STATUS_CODES.get(status, -1.0)]
        self.pub.publish(m)

    def _report(self, status):
        if self.print_period_s <= 0.0:
            return
        now = time.monotonic()
        if self.last_print_t is not None and now - self.last_print_t < self.print_period_s:
            return
        self.last_print_t = now
        s = self.ukf.snapshot()
        p = self.ukf.params
        self.get_logger().info(
            f"[kT d{self.drone_id}] est={s['thrust_ratio']:6.2f} "
            f"+/-{s['thrust_ratio_std']:4.2f} "
            f"(band {self.kt_min:.1f}..{self.kt_max:.1f}, seed {self.seed:.1f}) | "
            f"drag={p[1]:5.3f} tau={p[2]:5.3f} | "
            f"inn={s['innovation_norm']:6.3f} {self.solve_ms:5.1f}ms | "
            f"n={s['update_count']} [{status}]")


def _sim_is_fresh():
    so = os.path.join(ACADOS_DIR, 'c_generated_code',
                      'libacados_sim_solver_quad_load_dynamics_sim.so')
    js = os.path.join(ACADOS_DIR, 'quad_load_dynamics_sim.json')
    if not (os.path.exists(so) and os.path.exists(js)):
        return False
    pkg = os.path.dirname(os.path.abspath(__file__))
    src = [os.path.join(pkg, 'acados.py'), os.path.join(pkg, 'dynamics.py')]
    m = os.path.getmtime(so)
    return all(os.path.exists(s) and os.path.getmtime(s) <= m for s in src)


def main(args=None):
    rclpy.init(args=args)
    node = ThrustRatioEstimatorNode()
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
