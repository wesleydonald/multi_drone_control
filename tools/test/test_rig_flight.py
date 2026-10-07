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
