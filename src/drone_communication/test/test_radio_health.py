"""radio_health: the once-a-second record of one ELRS radio the console reads (no ROS)."""
import math
import types

from drone_communication.elrs_interface import radio_health


def _node(**kw):
    n = types.SimpleNamespace(serial_port='/dev/QUAD1', ser=object(), reconnects=0, reconnect_t=None, cmd_times=[],
                              link={}, battery_voltage=0.0, battery_t=None, mode='UNKNOWN', mode_t=None,
                              attitude=None, attitude_t=None, fc_arm_state='disarmed')
    n.__dict__.update(kw)
    return n


def test_no_telemetry_yet_reads_as_unknown_not_zero_volts():
    h = radio_health(_node(), 100.0)
    assert h['voltage'] is None and h['voltage_age_s'] is None and h['fc_tilt_deg'] is None
    assert h['connected'] is True and h['cmd_rate_hz'] == 0 and h['cmd_max_gap_s'] is None


def test_rates_gaps_ages_and_tilt():
    times = [95.0 + 0.02 * k for k in range(250)] + [100.0 - 0.3]
    h = radio_health(_node(cmd_times=sorted(times), battery_voltage=24.96, battery_t=99.5, mode='AIR*', mode_t=99.0,
                           attitude=(math.radians(-12.0), 0.0), attitude_t=99.9,
                           link={'rssi_dbm': -40, 'lq': 100, 'downlink_lq': 100, 't': 99.8}, reconnects=2,
                           reconnect_t=40.0), 100.0)
    assert h['voltage'] == 24.96 and h['voltage_age_s'] == 0.5 and h['mode'] == 'AIR*' and h['mode_age_s'] == 1.0
    assert h['fc_tilt_deg'] == 12.0 and h['fc_roll_deg'] == -12.0
    assert h['reconnects'] == 2 and h['reconnect_age_s'] == 60.0 and h['lq'] == 100
    assert 0.0 < h['cmd_max_gap_s'] < 0.3 and h['cmd_rate_hz'] > 0


def test_old_command_stamps_are_dropped():
    n = _node(cmd_times=[1.0, 2.0, 99.0])
    radio_health(n, 100.0)
    assert n.cmd_times == [99.0]
