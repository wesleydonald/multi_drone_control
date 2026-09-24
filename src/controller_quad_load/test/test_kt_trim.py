"""kt_trim: the gain estimate recovers the true gain from a synthetic hover, holds when
gated, respects its bound, the sign of the cable term is the tracker model's, the floor
test blocks and leaks, the inputs are time-aligned, and a converged estimate freezes."""
import numpy as np

from controller_quad_load.kt_trim import KtTrim, hover_thrust_gain, GRAV

KT_TRUE, KT_TYPED, DT = 33.0, 36.75, 0.02
A_CABLE = -4.69          # a taut 45 deg cable pulling the drone down, m/s^2


def _fly(trim, seconds, kt_true=KT_TRUE, a_cable=A_CABLE, steady=True, noise=0.0, seed=0,
         a_cable_model=None, wiggle=0.0, contact_possible=True):
    """Point-mass vertical plant with a tracker that servos height (P on position, D on
    velocity) using the trim's CURRENT gain to convert acceleration to throttle.
    `wiggle`: amplitude (m/s^2) of a 2 Hz command perturbation, the tracker's own
    position corrections."""
    rng = np.random.default_rng(seed)
    a_model = a_cable if a_cable_model is None else a_cable_model
    z, vz, k = 1.0, 0.0, 0
    for k in range(int(seconds / DT)):
        a_cmd = (GRAV - a_model - 4.0 * (z - 1.0) - 3.0 * vz
                 + wiggle * np.sin(2 * np.pi * 2.0 * k * DT))
        u = float(np.clip(a_cmd / trim.kt_hat, 0.0, 1.0))
        a_true = kt_true * u - GRAV + a_cable
        vz += a_true * DT
        z += vz * DT
        trim.measure_accel(vz + rng.normal(0, noise))
        trim.filter_inputs(u, 1.0)
        trim.update(a_model, steady, contact_possible=contact_possible)
    return z, vz


def test_instantaneous_formula_is_the_model_inverted():
    # v_dot = kT u R33 - g + a_cable  ->  kT = (v_dot + g - a_cable)/(u R33)
    assert hover_thrust_gain(0.0, A_CABLE, u=0.44, r33=1.0) == (GRAV - A_CABLE) / 0.44


def test_recovers_the_true_gain_within_three_time_constants():
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT)
    _fly(trim, 6.0)
    assert abs(trim.kt_hat - KT_TRUE) < 0.01 * KT_TRUE
    assert not trim.railed


def test_noise_on_the_velocity_does_not_bias_it():
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT)
    _fly(trim, 12.0, noise=0.02)
    assert abs(trim.kt_hat - KT_TRUE) < 0.02 * KT_TRUE


def test_gate_holds_the_value():
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT)
    _fly(trim, 5.0, steady=False)
    assert trim.kt_hat == KT_TYPED and trim.n_updates == 0


def test_bound_holds_and_flags():
    trim = KtTrim(KT_TYPED, max_frac=0.10, tau_s=1.5, dt=DT)
    _fly(trim, 12.0, kt_true=0.78 * KT_TYPED)         # 22 % off: inside the contact band, beyond the bound
    assert trim.kt_hat >= trim.lo - 1e-9 and trim.railed


def test_ill_conditioned_ticks_are_skipped():
    assert hover_thrust_gain(0.0, A_CABLE, u=0.05, r33=1.0) is None
    assert hover_thrust_gain(0.0, A_CABLE, u=0.4, r33=0.3) is None


def test_mass_error_is_absorbed_as_an_effective_gain():
    """The cable model says -4.69 but the plant pulls -5.63 (20 % heavier load): the
    estimate settles where the throttle holds the height, i.e. a lower effective gain."""
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT)
    _fly(trim, 12.0, kt_true=KT_TYPED, a_cable=1.2 * A_CABLE, a_cable_model=A_CABLE)
    expect = KT_TYPED * (GRAV - A_CABLE) / (GRAV - 1.2 * A_CABLE)
    assert abs(trim.kt_hat - expect) < 0.02 * KT_TYPED


def test_floor_contact_blocks_the_update_and_leaks_to_typed():
    """On the floor at takeoff throttle the model predicts a big climb and measures none:
    no update, and a previously railed estimate decays back to the typed gain."""
    trim = KtTrim(40.4, tau_s=1.5, dt=DT)
    trim.kt_hat = trim.hi                                  # railed high, as in Gazebo
    for _ in range(int(15.0 / DT)):
        trim.measure_accel(0.0)
        trim.filter_inputs(0.28, 1.0)
        trim.update(A_CABLE, True)                         # typed model predicts -3.2, measures 0
    assert trim.n_updates == 0
    assert abs(trim.kt_hat - 40.4) < 0.02 * 40.4


def test_a_sagging_hover_is_not_contact():
    """A 25 % gain error at a real hover disagrees by ~3.6 m/s^2: still updated."""
    trim = KtTrim(KT_TYPED, max_frac=0.3, tau_s=1.5, dt=DT)
    kt_true = KT_TYPED / 1.25
    _fly(trim, 12.0, kt_true=kt_true)
    assert trim.n_updates > 0 and abs(trim.kt_hat - kt_true) < 0.02 * kt_true


def test_typed_low_hover_converges_once_the_load_is_up():
    """A gain typed 20 % LOW sits outside the floor band at hover (Gazebo 2026-09-24
    15:06 stuck at typed). With the load off its rest height the floor test is off and
    the estimate converges; with it on, the estimate stays at typed."""
    kt_true = KT_TYPED * 1.25
    stuck = KtTrim(KT_TYPED, max_frac=0.3, tau_s=1.5, dt=DT)
    _fly(stuck, 8.0, kt_true=kt_true, contact_possible=True)
    assert abs(stuck.kt_hat - KT_TYPED) < 0.02 * KT_TYPED         # a few transient ticks, no convergence
    free = KtTrim(KT_TYPED, max_frac=0.3, tau_s=1.5, dt=DT)
    _fly(free, 8.0, kt_true=kt_true, contact_possible=False)
    assert abs(free.kt_hat - kt_true) < 0.02 * kt_true


def test_tracking_corrections_do_not_bias_the_estimate():
    """The tracker's own 2 Hz throttle corrections (a 1.5 m/s^2 wiggle, 10 % of hover
    thrust) must not move the estimate: throttle goes through the acceleration's filter,
    so the ratio stays the plant's gain (Gazebo 2026-09-24 16:28 chattered on this)."""
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT, freeze_frac=0.0)
    _fly(trim, 12.0, kt_true=KT_TYPED, wiggle=1.5)
    assert abs(trim.kt_hat - KT_TYPED) < 0.005 * KT_TYPED


def test_converged_estimate_freezes_for_the_flight():
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT)
    _fly(trim, 12.0)
    assert trim.frozen and abs(trim.kt_hat - KT_TRUE) < 0.01 * KT_TRUE
    held = trim.kt_hat
    _fly(trim, 6.0, kt_true=1.2 * KT_TRUE)                 # the plant changes: no reaction
    assert trim.kt_hat == held
    trim.reset()
    assert not trim.frozen and trim.kt_hat == KT_TYPED


def test_freeze_needs_two_quiet_seconds():
    trim = KtTrim(KT_TYPED, tau_s=1.5, dt=DT)
    _fly(trim, 1.0)                                       # still moving
    assert not trim.frozen
