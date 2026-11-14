"""multi_drone_control 4 Nov: the throttle-state bound of his payload OCP, set at run time (kept out of acados.py,
dynamics.py and mpc_modes.py so the cached solver's fingerprint, and so the build, is unchanged)."""
import numpy as np

# his OCP's throttle-state upper bound as acados.py builds it (ubx[0] on idxbx [15, 13, 14, 16])
THROTTLE_STATE_MAX = 0.6
THROTTLE_STATE_INDEX = 15     # the throttle (control) state in his OCP: idxbx [15, 13, 14, 16] (acados.py)


def set_throttle_state_max(ocp_solver, value, max_rate=1.0):
    """multi_drone_control 4 Nov (Te-Jen's review of RIG-1104-k1m_3: carrying the object QUAD4 sat at the 0.60 bound
    for 17 s and sank 0.4 m under his reference): replace the throttle state's upper bound at run time on every
    stage that carries it (1..N-1; stage 0 is the measured state), keeping the attitude-rate bounds. No rebuild."""
    value = float(value)
    if not THROTTLE_STATE_MAX <= value <= 0.95:
        raise ValueError(f'throttle_state_max {value} outside [{THROTTLE_STATE_MAX}, 0.95]')
    ub = np.array([value, max_rate, max_rate, max_rate])
    # Te-Jen's review 11 Nov: keep the state bounds the solver was built with (a later profile may differ); only the
    # throttle state's entry changes
    cons = getattr(getattr(ocp_solver, 'acados_ocp', None), 'constraints', None)
    built = None if cons is None else getattr(cons, 'ubx', None)
    idx = None if cons is None else getattr(cons, 'idxbx', None)
    if built is not None and idx is not None and THROTTLE_STATE_INDEX in [int(i) for i in idx]:
        ub = np.array(built, dtype=float).copy()
        ub[[int(i) for i in idx].index(THROTTLE_STATE_INDEX)] = value
    n = int(ocp_solver.acados_ocp.solver_options.N_horizon)
    for i in range(1, n):
        ocp_solver.constraints_set(i, 'ubx', ub)
    return n - 1


def battery_throttle_offset(slope, v_ref, voltage, max_offset=0.10):
    """multi_drone_control 4 Nov: the rig's identified voltage term (30 Sep: throttle offset rises 0.022 per volt below
    23.5 V) as a radio-throttle addition, at most max_offset; 0 when off, with no voltage, or above v_ref."""
    if slope <= 0.0 or voltage is None:
        return 0.0
    return float(min(max_offset, max(0.0, slope * (v_ref - float(voltage)))))


def battery_compensated_channel(channel_2, offset):
    """ELRS throttle channel (-1..1) with the offset added in throttle (0..1), held to the channel's range."""
    if offset <= 0.0 or channel_2 <= -1.0:
        return channel_2                       # off, or a zero-throttle command (disarmed / not yet taken off)
    thr = min(1.0, (float(channel_2) + 1.0) / 2.0 + offset)
    return round(thr * 2.0 - 1.0, 3)
