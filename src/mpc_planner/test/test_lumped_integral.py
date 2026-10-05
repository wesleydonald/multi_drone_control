"""LumpedForceIntegral: per-axis bounded integral of the ring position error, applied as -k*b,
held per axis when its gate drops, zero with the ring down, reset on demand."""
import numpy as np

from mpc_planner.lumped_integral import LumpedForceIntegral

HZ = 10.0


def _run(li, err, seconds, gxy=True, gz=True):
    for _ in range(int(seconds * HZ)):
        li.update(err, gxy, gz)
    return li.b


def test_off_does_nothing():
    li = LumpedForceIntegral(0.0, 0.15, HZ, 5.4, 8.5)
    assert np.all(_run(li, [0.1, 0.1, 0.1], 20.0) == 0.0)


def test_sign_low_ring_plans_a_heavier_ring():
    li = LumpedForceIntegral(0.4, 0.15, HZ, 5.4, 8.5)
    _run(li, [0.0, 0.0, 0.05], 2.5)             # ring 5 cm low for 1 tau
    assert abs(li.b[2] - 0.05) < 1e-9
    assert abs(li.force()[2] - (-8.5 * 0.05)) < 1e-9   # downward: more planned tension


def test_sign_ring_off_minus_x_pushes_plus_x():
    li = LumpedForceIntegral(0.4, 0.15, HZ, 5.4, 8.5)
    _run(li, [0.05, 0.0, 0.0], 2.5)             # ring 5 cm toward -x of its reference
    assert li.force()[0] < 0.0                  # modelled as a -x force, so the plan leans +x


def test_bounded_in_metres_and_newtons():
    li = LumpedForceIntegral(0.4, 0.15, HZ, 5.4, 8.5)
    _run(li, [-1.0, 1.0, 1.0], 30.0)
    assert np.allclose(li.b, [-0.15, 0.15, 0.15]) and li.near_bound.all()
    assert np.allclose(np.abs(li.force()), li.force_bound)
    assert np.allclose(li.force_bound, [0.81, 0.81, 1.275])


def test_axes_gate_independently_and_hold():
    li = LumpedForceIntegral(0.4, 0.15, HZ, 5.4, 8.5)
    _run(li, [0.05, 0.05, 0.05], 1.0)
    held = li.b.copy()
    _run(li, [0.5, 0.5, 0.5], 1.0, gxy=False, gz=True)
    assert np.allclose(li.b[:2], held[:2]) and li.b[2] > held[2]


def test_zero_with_the_ring_down_but_state_kept():
    li = LumpedForceIntegral(0.4, 0.15, HZ, 5.4, 8.5)
    _run(li, [0.0, 0.0, 0.05], 2.0)
    assert np.all(li.force(down=True) == 0.0) and li.b[2] > 0.0


def test_reset():
    li = LumpedForceIntegral(0.4, 0.15, HZ, 5.4, 8.5)
    _run(li, [0.05, 0.05, 0.05], 2.0)
    li.reset()
    assert np.all(li.b == 0.0) and np.all(li.n_updates == 0)
