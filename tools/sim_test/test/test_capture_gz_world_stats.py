from pathlib import Path
import importlib.util


MODULE_PATH = Path(__file__).resolve().parents[1] / "capture_gz_world_stats.py"
spec = importlib.util.spec_from_file_location("capture_gz_world_stats", MODULE_PATH)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)


def test_parse_world_stats_extracts_rtf_and_correlated_times(monkeypatch):
    monkeypatch.setattr(module.time, "time", lambda: 1234.5)
    text = """
sim_time {
  sec: 12
  nsec: 250000000
}
real_time {
  sec: 17
  nsec: 500000000
}
paused: false
iterations: 12250
real_time_factor: 0.7
step_size {
  sec: 0
  nsec: 1000000
}
"""
    rows = list(module.parse_world_stats_lines(text.splitlines()))
    assert len(rows) == 1
    row = rows[0]
    assert row["wall_epoch_s"] == 1234.5
    assert row["sim_time_s"] == 12.25
    assert row["real_time_s"] == 17.5
    assert row["real_time_factor"] == 0.7
    assert row["iterations"] == 12250
    assert row["paused"] is False
    assert row["step_size_s"] == 0.001


def test_parse_world_stats_yields_multiple_samples():
    text = """
sim_time {
  sec: 1
}
real_time {
  sec: 2
}
paused: false
iterations: 1000
real_time_factor: 0.5
sim_time {
  sec: 2
}
real_time {
  sec: 3
}
paused: false
iterations: 2000
real_time_factor: 0.75
"""
    rows = list(module.parse_world_stats_lines(text.splitlines()))
    assert [row["real_time_factor"] for row in rows] == [0.5, 0.75]
    assert [row["iterations"] for row in rows] == [1000, 2000]
