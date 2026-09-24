"""kt_trim: the gain estimate recovers the true gain from a synthetic hover, holds when
gated, respects its bound, and the sign of the cable term is the tracker model's."""
import numpy as np

from controller_quad_load.kt_trim import KtTrim, hover_thrust_gain, GRAV

KT_TRUE, KT_TYPED, DT = 33.0, 36.75, 0.02
A_CABLE = -4.69          # a taut 45 deg cable pulling the drone down, m/s^2


def _fly(trim, seconds, kt_true=KT_TRUE, a_cable=A_CABLE, steady=True, noise=0.0, seed=0,
         a_cable_model=None):
    """Point-mass vertical plant with a tracker that servos height (P on position, D on
    velocity) using the trim's CURRENT gain to convert acceleration to throttle."""
    rng = np.random.default_rng(seed)
    a_model = a_cable if a_cable_model is None else a_cable_model
    z, vz = 1.0, 0.0
    for _ in range(int(seconds / DT)):
        a_cmd = GRAV - a_model - 4.0 * (z - 1.0) - 3.0 * vz     # hold z = 1
        u = float(np.clip(a_cmd / trim.kt_hat, 0.0, 1.0))
        a_true = kt_true * u - GRAV + a_cable
        vz += a_true * DT
        z += vz * DT
        trim.measure_accel(vz + rng.normal(0, noise))
        trim.update(a_model, u, 1.0, steady)
    return z, vz


def test_instantaneous_formula_is_the_model_inverted():
    # v_dot = kT u R33 - g + a_cable  ->  kT = (v_dot + g - a_cable)/(u R33)
    assert hover_thrust_gain(0.0, A_CABLE, u=0.44, r33=1.0) == (GRAV - A_CABLE) / 0.44


def test_recovers_the_true_gain_within_three_time_constants():
    trim = KtTrim(KT_TYPED, tau_s=8.0, dt=DT)
    _fly(trim, 24.0)
    assert abs(trim.kt_hat - KT_TRUE) < 0.01 * KT_TRUE
    assert not trim.railed


def test_noise_on_the_velocity_does_not_bias_it():
    trim = KtTrim(KT_TYPED, tau_s=8.0, dt=DT)
    _fly(trim, 30.0, noise=0.02)
    assert abs(trim.kt_hat - KT_TRUE) < 0.02 * KT_TRUE


def test_gate_holds_the_value():
    trim = KtTrim(KT_TYPED, tau_s=8.0, dt=DT)
    _fly(trim, 5.0, steady=False)
    assert trim.kt_hat == KT_TYPED and trim.n_updates == 0


def test_bound_holds_and_flags():
    trim = KtTrim(KT_TYPED, max_frac=0.10, tau_s=8.0, dt=DT)
    _fly(trim, 30.0, kt_true=0.78 * KT_TYPED)         # 22 % off: inside the contact band, beyond the bound
    assert trim.kt_hat >= trim.lo - 1e-9 and trim.railed


def test_ill_conditioned_ticks_are_skipped():
    assert hover_thrust_gain(0.0, A_CABLE, u=0.05, r33=1.0) is None
    assert hover_thrust_gain(0.0, A_CABLE, u=0.4, r33=0.3) is None


def test_mass_error_is_absorbed_as_an_effective_gain():
    """The cable model says -4.69 but the plant pulls -5.63 (20 % heavier load): the
    estimate settles where the throttle holds the height, i.e. a lower effective gain."""
    trim = KtTrim(KT_TYPED, tau_s=8.0, dt=DT)
    _fly(trim, 30.0, kt_true=KT_TYPED, a_cable=1.2 * A_CABLE, a_cable_model=A_CABLE)
    # the plant's extra pull shows up as a smaller effective gain, and the servo holds
    # its height on that gain
    expect = KT_TYPED * (GRAV - A_CABLE) / (GRAV - 1.2 * A_CABLE)
    assert abs(trim.kt_hat - expect) < 0.02 * KT_TYPED


def test_floor_contact_blocks_the_update_and_leaks_to_typed():
    """On the floor at takeoff throttle the model predicts a big climb and measures none:
    no update, and a previously railed estimate decays back to the typed gain."""
    trim = KtTrim(40.4, tau_s=1.5, dt=DT)
    trim.kt_hat = trim.hi                                  # railed high, as in Gazebo
    for _ in range(int(15.0 / DT)):
        trim.measure_accel(0.0)
        trim.update(A_CABLE, u=0.28, r33=1.0, steady=True)   # typed model predicts -3.2, measures 0
    assert trim.n_updates == 0
    assert abs(trim.kt_hat - 40.4) < 0.02 * 40.4


def test_a_sagging_hover_is_not_contact():
    """A 25 % gain error at a real hover disagrees by ~3.6 m/s^2: still updated."""
    trim = KtTrim(KT_TYPED, max_frac=0.3, tau_s=1.5, dt=DT)
    kt_true = KT_TYPED / 1.25
    _fly(trim, 12.0, kt_true=kt_true)
    assert trim.n_updates > 0 and abs(trim.kt_hat - kt_true) < 0.02 * kt_true
