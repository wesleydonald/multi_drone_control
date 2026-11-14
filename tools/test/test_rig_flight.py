"""tools/rig_flight.sh argument handling (dry run): run name last or first, auto-suffix (7 Oct)."""
import os
import subprocess

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SH = os.path.join(REPO, 'tools', 'rig_flight.sh')


def _run(args, tmp):
    env = dict(os.environ, RIG_FLIGHT_DRY_RUN='1', RIG_FLIGHT_DIR=str(tmp))
    return subprocess.run(['bash', SH, *args], env=env, capture_output=True, text=True)


def test_name_last(tmp_path):
    r = _run(['real_control_launch.py', 'mode:=mpc', 'num_drones:=4', 'r303'], tmp_path)
    assert r.returncode == 0 and 'F=r303 LAUNCH=real_control_launch.py N=4' in r.stdout
    assert 'ARGS=mode:=mpc num_drones:=4' in r.stdout and 'r303' not in r.stdout.split('ARGS=')[1]


def test_old_order_still_works(tmp_path):
    r = _run(['r303', 'real_control_launch.py', 'num_drones:=3'], tmp_path)
    assert r.returncode == 0 and 'F=r303 LAUNCH=real_control_launch.py N=3' in r.stdout


def test_existing_name_gets_a_suffix(tmp_path):
    (tmp_path / 'r303.log').write_text('')
    (tmp_path / 'r303_2_logs').mkdir()
    r = _run(['real_control_launch.py', 'r303'], tmp_path)
    assert r.returncode == 0 and 'F=r303_3 ' in r.stdout and 'this run is r303_3' in r.stdout


def test_missing_name_or_bad_name_is_refused(tmp_path):
    assert _run(['real_control_launch.py', 'num_drones:=4'], tmp_path).returncode == 2
    assert _run(['real_control_launch.py', 'r 3'], tmp_path).returncode == 2


def test_attach_bags_the_newcomer_radio_too(tmp_path):
    """mode:=attach counts only the tethered drones; the bag must cover the newcomer (drone 3)."""
    r = _run(['real_control_launch.py', 'mode:=attach', 'r400'], tmp_path)
    assert r.returncode == 0 and ' N=4 ' in r.stdout
    r = _run(['real_control_launch.py', 'mode:=attach', 'num_drones:=3', 'r401'], tmp_path)
    assert ' N=4 ' in r.stdout
    r = _run(['real_control_launch.py', 'num_drones:=4', 'r402'], tmp_path)
    assert ' N=4 ' in r.stdout


def test_refuses_without_a_local_dds_domain(tmp_path):
    env = {k: v for k, v in os.environ.items() if k not in ('ROS_LOCALHOST_ONLY', 'ROS_DOMAIN_ID')}
    env['RIG_FLIGHT_DIR'] = str(tmp_path)
    r = subprocess.run(['bash', SH, 'real_control_launch.py', 'r403'], env=env, capture_output=True, text=True)
    assert r.returncode == 1 and 'ROS_LOCALHOST_ONLY=1' in r.stderr
    assert not list(tmp_path.iterdir()), 'nothing may be written before the check'


def test_the_planner_cpus_are_echoed_when_set(tmp_path):
    env = {k: v for k, v in os.environ.items() if k != 'MDC_PLANNER_CPUS'}
    env.update(RIG_FLIGHT_DRY_RUN='1', RIG_FLIGHT_DIR=str(tmp_path))
    r = subprocess.run(['bash', SH, 'real_control_launch.py', 'r500'], env=env, capture_output=True, text=True)
    assert r.returncode == 0 and 'MDC_PLANNER_CPUS' not in r.stdout
    env['MDC_PLANNER_CPUS'] = '0-7'
    r = subprocess.run(['bash', SH, 'real_control_launch.py', 'r500'], env=env, capture_output=True, text=True)
    assert r.returncode == 0 and 'rig_flight: MDC_PLANNER_CPUS=0-7' in r.stdout


# ── partners_file (4 -> 2 -> 4): a radio per partner, every partner's and his instances' topics in the bag ──

PARTNERS_YAML = 'partners:\n  - drone: 3\n    plate: 9\n  - drone: 2\n    plate: 3\n'


def _added(stdout):
    lines = [ln for ln in stdout.splitlines() if 'the bag adds' in ln]
    assert len(lines) == 1, stdout
    return lines[0].split('the bag adds', 1)[1].split()


def test_partners_file_adds_a_radio_per_partner_and_their_topics(tmp_path):
    pf = tmp_path / 'partners.yaml'
    pf.write_text(PARTNERS_YAML)
    r = _run(['real_control_launch.py', 'mode:=attach', 'num_drones:=2', 'partner_contact:=true',
              f'partners_file:={pf}', 'r424'], tmp_path)
    assert r.returncode == 0 and ' N=4 ' in r.stdout and 'DRY RUN partners 3 2;' in r.stdout
    added = _added(r.stdout)
    for t in ('/fleet/detach_set', '/drone_2/partner/release', '/drone_2/magnet/object_attached',
              '/drone_2/magnet/contact', '/drone_2/magnet/attach_proof', '/drone_2/magnet_tip_pose',
              '/drone_2/take_back', '/drone_2/magnet_coil', '/drone_2/ELRSCommand_tejen', '/tejen_d2/join_planner/phase',
              '/tejen_d2/join_planner/give_up', '/tejen_d2/join_planner/land_now', '/tejen_d2/drone_command',
              '/tejen_d2/object_attached', '/tejen_d2/fake_attachment_point/pose', '/drone_2/magnet_operator',
              '/drone_3/magnet_operator'):
        assert t in added, t
    # QUAD4 keeps today's names (already in the bag list): nothing under /tejen_d3 or a second copy of its topics
    assert not [t for t in added if t.startswith('/tejen_d3') or (t.startswith('/drone_3/')
                                                                   and t != '/drone_3/magnet_operator')]
    assert len(added) == len(set(added))


def test_partners_file_without_partner_contact_adds_the_base_topics_only(tmp_path):
    pf = tmp_path / 'partners.yaml'
    pf.write_text(PARTNERS_YAML)
    r = _run(['real_control_launch.py', 'mode:=attach', 'num_drones:=2', f'partners_file:={pf}', 'r424'], tmp_path)
    added = _added(r.stdout)
    assert '/drone_2/partner/release' in added and '/tejen_d2/join_planner/phase' not in added


def test_without_partners_file_the_dry_run_is_todays(tmp_path):
    r = _run(['real_control_launch.py', 'mode:=attach', 'partner_contact:=true', 'r425'], tmp_path)
    assert r.returncode == 0 and 'DRY RUN partners' not in r.stdout and 'the bag adds' not in r.stdout
    assert 'rig_flight: DRY RUN F=r425 LAUNCH=real_control_launch.py N=4 ' in r.stdout


def test_a_partners_file_with_no_partner_is_refused(tmp_path):
    r = _run(['real_control_launch.py', 'mode:=attach', f'partners_file:={tmp_path / "none.yaml"}', 'r426'], tmp_path)
    assert r.returncode == 2 and 'no partner read' in r.stderr
    (tmp_path / 'empty.yaml').write_text('partners: []\n')
    r = _run(['real_control_launch.py', 'mode:=attach', f'partners_file:={tmp_path / "empty.yaml"}', 'r426'], tmp_path)
    assert r.returncode == 2 and 'no partner read' in r.stderr
