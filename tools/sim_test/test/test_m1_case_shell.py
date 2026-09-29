from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_m1_case_shell_exists_and_delegates_to_supervised_runner():
    script = ROOT / "tools" / "sim_test" / "run_m1_case.sh"
    assert script.is_file()
    text = script.read_text()
    assert 'run.sh" cases' in text
    assert 'run.sh" run --scenario c1f6 --case "${CASE}" --gui --manual-control' in text


def test_legacy_c1f6_manual_runner_remains_baseline_compatible():
    script = ROOT / "tools" / "sim_test" / "run_c1f6_manual.sh"
    text = script.read_text()
    assert "--scenario c1f6" in text
    assert "--manual-control" in text
    assert "--case" not in text
