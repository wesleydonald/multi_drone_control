"""The figure-8's own size (Wesley 22 Oct): fig8_size sizes the figure-8 (x +-size, y +-size/2), traj_radius the circle,
spin and orbit; unset (0, '' or None: the legacy profile, a params.json from before 22 Oct) the figure-8 follows
traj_radius as it did. One resolver for the planner and every tool that rebuilds a flight's path."""
import numpy as np
import pytest

from mpc_planner.load_trajectory import LoadTrajectory, path_radius


def test_the_figure8_takes_fig8_size_and_nothing_else_does():
    assert path_radius('fig_8', 0.5, 0.9) == 0.9
    for kind in ('circle', 'spin', 'orbit', 'hover', 'line_x', 'line_spin'):
        assert path_radius(kind, 0.5, 0.9) == 0.5


@pytest.mark.parametrize('unset', [None, '', 0.0, '0.0', -1.0])
def test_unset_the_figure8_follows_traj_radius_as_before(unset):
    assert path_radius('fig_8', 1.0, unset) == 1.0


def test_from_params_rebuilds_old_and_new_flights():
    old = {'load_traj': 'fig_8', 'traj_speed': 0.2, 'traj_distance': 1.0, 'traj_radius': 1.0}     # before 22 Oct
    new = {'load_traj': 'fig_8', 'traj_speed': 0.2, 'traj_distance': 1.0, 'traj_radius': 0.5, 'fig8_size': 0.9}
    assert LoadTrajectory.from_params(old).radius == 1.0 and LoadTrajectory.from_params(new).radius == 0.9
    orbit = LoadTrajectory.from_params({'load_traj': 'orbit', 'traj_radius': 0.5, 'fig8_size': 0.9})
    assert (orbit.kind, orbit.radius, orbit.speed) == ('orbit', 0.5, 0.1)          # the node's defaults fill in
    spin = LoadTrajectory.from_params({'load_traj': 'line_spin', 'traj_distance': 0.0, 'traj_speed': ''},
                                      speed=0.125)
    assert (spin.distance, spin.speed) == (0.0, 0.125)                              # 0 is a value, '' is not
    assert LoadTrajectory.from_params({}, kind='orbit').kind == 'orbit'


@pytest.mark.parametrize('size', [0.5, 0.9, 1.0])
def test_the_size_is_the_x_half_extent_and_y_is_half_of_it(size):
    tr = LoadTrajectory('fig_8', 0.2, 0.0, path_radius('fig_8', 0.5, size))
    T = tr._fig8_theta(0.0)[3]
    xy = np.array([tr.offset_at(t)[:2] for t in np.linspace(0.0, T, 4001)])
    assert xy[:, 0].max() == pytest.approx(size, abs=1e-3) and xy[:, 0].min() == pytest.approx(-size, abs=1e-3)
    assert np.abs(xy[:, 1]).max() == pytest.approx(size / 2, abs=1e-3)
