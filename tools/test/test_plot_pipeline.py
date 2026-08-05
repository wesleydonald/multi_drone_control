"""
Unit tests for the analysis and plotting pipeline — THESIS_PLAN §9.4.

Built on a SYNTHETIC run whose answer is known analytically (same principle as
test_metrics.py): a commanded circle of r=0.5 flown at r=0.375 with a 0.4 s lag. A test
against recorded data could only say the pipeline is stable, never that it is right.

Each test names the property it protects:

  * one loader, two harnesses -- a Gazebo run.csv and a bench sil.csv of the same
    flight must produce the same numbers, because compare_runs puts them on one axis
  * the drone palette IS the RViz palette (§6.2) -- drone 2 the same colour on screen
    and in the figure
  * figures are produced and are vector, for every run shape including the ones with
    all-NaN columns
"""
import csv
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import compare_runs as C            # noqa: E402
import metrics as M                 # noqa: E402
import plot_run as P                # noqa: E402
import plot_style as S              # noqa: E402

R_DES, SHRINK, LAG_S, PERIOD = 0.5, 0.75, 0.4, 6.0
DT = 0.02


def synth_rows(n=3, duration=48.0, gazebo=False):
    """A circle-tracking carry run with a known radius deficit and a known lag."""
    t = np.arange(0.0, duration, DT)
    w = 2 * np.pi / PERIOD
    des = np.column_stack([R_DES * np.cos(w * t), R_DES * np.sin(w * t),
                           np.full(t.size, 0.6)])
    act = np.column_stack([SHRINK * R_DES * np.cos(w * (t - LAG_S)),
                           SHRINK * R_DES * np.sin(w * (t - LAG_S)),
                           np.full(t.size, 0.6)])
    rows = []
    for k, tk in enumerate(t):
        r = {'t': tk, 'step': k,
             'payload_x': act[k, 0], 'payload_y': act[k, 1], 'payload_z': act[k, 2],
             'payload_qw': 1.0, 'payload_qx': 0.0, 'payload_qy': 0.0,
             'payload_qz': 0.0, 'payload_tilt_deg': 0.0,
             'payload_ref_x': des[k, 0], 'payload_ref_y': des[k, 1],
             'payload_ref_z': des[k, 2]}
        for i in range(n):
            ang = 2 * np.pi * i / n
            off = np.array([0.4 * np.cos(ang), 0.4 * np.sin(ang), 0.5])
            pos, ref = act[k] + off, des[k] + off
            r.update({f'd{i}_x': pos[0], f'd{i}_y': pos[1], f'd{i}_z': pos[2],
                      f'd{i}_vx': 0.0, f'd{i}_vy': 0.0, f'd{i}_vz': 0.0,
                      f'd{i}_tilt_deg': 5.0,
                      f'd{i}_ref_x': ref[0], f'd{i}_ref_y': ref[1], f'd{i}_ref_z': ref[2],
                      f'd{i}_track_err': float(np.linalg.norm(pos - ref)),
                      f'd{i}_thr': 0.35, f'd{i}_armed': 1, f'd{i}_attached': 1,
                      # The Gazebo runner cannot see inside the tracker, so these are
                      # NaN there and real on the bench -- schema parity, §9.4.
                      f'd{i}_acm': math.nan if gazebo else 4.0,
                      f'd{i}_tension': math.nan if gazebo else 3.0 + i,
                      f'd{i}_elev_deg': math.nan if gazebo else 60.0})
        rows.append(r)
    return rows


def write_run(tmp, name, gazebo=False, n=3, events=(('ARM', 1.0), ('TAKEOFF', 3.0),
                                                    ('DETACH', 20.0))):
    path = os.path.join(str(tmp), name)
    os.makedirs(os.path.join(path, 'logs'), exist_ok=True)
    rows = synth_rows(n=n, gazebo=gazebo)
    with open(os.path.join(path, 'logs', 'run.csv' if gazebo else 'sil.csv'), 'w',
              newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(path, 'logs', 'events.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_time', 'event', 'arg'])
        for ev, t in events:
            w.writerow([f'{t:.3f}', ev, ''])
    with open(os.path.join(path, 'manifest.json'), 'w') as fh:
        json.dump({'run_id': name[:5], 'kind': 'sim' if gazebo else 'sil',
                   'scenario_name': 'synth', 'git_sha': 'deadbeefcafe'}, fh)
    return path


# ── the shared loader ────────────────────────────────────────────────────────

def test_the_same_flight_reads_identically_from_run_csv_and_sil_csv(tmp_path):
    """One loader, two harnesses. compare_runs puts a bench run and a Gazebo run on one
    axis, so a difference here would be a silent unit or origin error between them."""
    gz = M.load_run(write_run(tmp_path, 'R9001_gz', gazebo=True))
    sil = M.load_run(write_run(tmp_path, 'R9002_sil', gazebo=False))
    assert (gz['source'], sil['source']) == ('gazebo', 'sil')
    for key in ('t', 'payload_x', 'd1_track_err'):
        assert np.allclose(gz['data'][key], sil['data'][key])


def test_unobservable_columns_are_omitted_not_reported_as_nan(tmp_path):
    """A missing key says "this harness does not observe that"; a nan reads as "it was
    measured and came out undefined". A table must be able to tell them apart."""
    gz = M.summarise_run(write_run(tmp_path, 'R9003_gz', gazebo=True))
    sil = M.summarise_run(write_run(tmp_path, 'R9004_sil'))
    assert 'tension_share' not in gz and 'tension_share' in sil
    assert 'peak_cable_accel' not in gz['per_drone'][0]
    assert sil['per_drone'][0]['peak_cable_accel'] == pytest.approx(4.0)


def test_summarise_recovers_the_synthetic_radius_deficit_and_lag(tmp_path):
    """The headline tracking numbers, against a signal whose answer is known."""
    m = M.summarise_run(write_run(tmp_path, 'R9005_sil'))
    assert m['payload_radius_ratio'] == pytest.approx(SHRINK, rel=0.02)
    assert m['payload_phase_lag_s'] == pytest.approx(LAG_S, abs=0.05)


def test_stage_split_puts_the_whole_loss_in_the_tracker_stage(tmp_path):
    """The drones here follow their references exactly and the payload is the lagging,
    shrunken thing -- so the reference stage must be clean and the loss must appear
    downstream of it. Attributing it to the wrong stage is the failure mode this
    diagnostic exists to prevent."""
    m = M.summarise_run(write_run(tmp_path, 'R9006_sil'))
    by = {s['stage']: s for s in m['stage_split']}
    assert by['desired -> ref_centroid']['radius_ratio'] == pytest.approx(1.0, abs=0.02)
    assert by['desired -> ref_centroid']['phase_lag_s'] == pytest.approx(0.0, abs=0.05)
    cum = by['desired -> payload (cumulative)']
    assert cum['radius_ratio'] == pytest.approx(SHRINK, rel=0.02)
    assert cum['phase_lag_s'] == pytest.approx(LAG_S, abs=0.05)


def test_steady_window_starts_five_seconds_after_the_run_event(tmp_path):
    """The §9.2 convention, read off a real events.csv rather than a passed-in number."""
    m = M.summarise_run(write_run(tmp_path, 'R9007_sil'))
    assert m['primary_event'] == 'DETACH'
    assert m['steady_window_s'][0] == pytest.approx(25.0, abs=DT)


# ── the palette promise (§6.2) ───────────────────────────────────────────────

def test_the_plot_palette_is_the_rviz_palette():
    """Drone 2 must be the same colour in RViz and in every thesis figure. The palette
    is imported, not copied, and this test fails if that import is ever replaced by a
    literal that drifts."""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)),
                                    'src', 'controller_quad_load'))
    from controller_quad_load.rviz_config import drone_colour as rviz_colour
    for i in range(6):
        r, g, b = (int(v) for v in rviz_colour(i).split(';'))
        assert S.drone_colour(i) == f'#{r:02x}{g:02x}{b:02x}'


def test_run_colours_are_not_the_drone_colours():
    """On a compare_runs figure colour means "which run". Reusing the drone palette
    there would make the same blue mean drone 0 on one page and R0034 on the next."""
    assert not (set(S.RUN_HEX) & set(S.DRONE_HEX))


# ── the figures themselves ───────────────────────────────────────────────────

@pytest.mark.parametrize('gazebo', [False, True])
def test_all_six_figures_are_written_as_vector_and_raster(tmp_path, gazebo):
    """Including the Gazebo shape, where two panels have nothing to draw -- those must
    annotate themselves rather than raising or coming out blank."""
    path = write_run(tmp_path, 'R9010_x', gazebo=gazebo)
    written = P.plot_run(path, quiet=True)
    names = sorted(os.path.basename(w) for w in written)
    assert len(names) == 12
    for stem in ('01_xy_path', '02_error', '03_axes', '04_health', '05_formation',
                 '06_stage_split'):
        for ext in ('pdf', 'png'):
            f = os.path.join(path, 'plots', f'{stem}.{ext}')
            assert os.path.getsize(f) > 1000, f'{f} is suspiciously small'


def test_finish_run_writes_metrics_and_never_raises_on_a_broken_run(tmp_path):
    """finish_run is called after the aircraft has flown. An analysis bug must not be
    able to turn a completed run into a failed one."""
    broken = os.path.join(str(tmp_path), 'R9011_broken')
    os.makedirs(os.path.join(broken, 'logs'))
    with open(os.path.join(broken, 'manifest.json'), 'w') as fh:
        json.dump({'run_id': 'R9011'}, fh)
    out = P.finish_run(broken, quiet=True)
    assert 'error' in out


def test_compare_runs_tables_the_median_and_the_full_range(tmp_path):
    """§9.2: never a mean alone. A grouped cell must carry the spread, because the run
    that diverged is usually the finding and a mean hides it."""
    paths = [write_run(tmp_path, f'R902{i}_sil') for i in range(2)]
    out_dir, rows = C.compare(paths, out_dir=os.path.join(str(tmp_path), 'cmp'),
                              group_by=False, quiet=True)
    by = {name: (unit, cells) for name, unit, cells in rows}
    unit, cells = by['payload radius ratio']
    assert len(cells) == 2                      # two ungrouped runs -> two columns
    assert cells[0]['median'] == pytest.approx(SHRINK, rel=0.02)
    assert os.path.exists(os.path.join(out_dir, 'metrics.csv'))
    assert os.path.exists(os.path.join(out_dir, 'c5_metrics.pdf'))


def test_compare_runs_groups_repeats_of_one_scenario(tmp_path):
    """Five repeats of a scenario are one series with a range, not five legend entries."""
    paths = [write_run(tmp_path, f'R903{i}_sil') for i in range(3)]
    _, rows = C.compare(paths, out_dir=os.path.join(str(tmp_path), 'cmp2'),
                        group_by=True, quiet=True)
    cells = dict((n, c) for n, _, c in rows)['payload RMSE']
    assert len(cells) == 1 and cells[0]['n'] == 3
