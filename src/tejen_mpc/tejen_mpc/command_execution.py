"""Evaluate the linear MPC command state at the bounded execution interval."""
import numpy as np


def command_at_elapsed_time(initial_command, first_stage_command, elapsed_s, stage_duration_s):
    """Interpolate command states only; never retime the vehicle/reference horizon.

    Command dynamics are command_dot=input, constant within the first stage.
    Staying between its initial and terminal commands preserves stage bounds.
    """
    initial = np.asarray(initial_command, dtype=float)
    endpoint = np.asarray(first_stage_command, dtype=float)
    if initial.shape != (4,) or endpoint.shape != (4,):
        raise ValueError("Expected four initial and first-stage command states")
    if not np.isfinite(initial).all() or not np.isfinite(endpoint).all():
        raise ValueError("Command states must be finite")
    if not np.isfinite(elapsed_s) or not np.isfinite(stage_duration_s) or stage_duration_s <= 0:
        raise ValueError("Execution interval must be finite and stage duration positive")
    fraction = np.clip(float(elapsed_s) / float(stage_duration_s), 0., 1.)
    return initial + fraction * (endpoint - initial)
