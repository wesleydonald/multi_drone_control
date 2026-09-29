from pathlib import Path
import importlib.util


MODULE_PATH = Path(__file__).resolve().parents[1] / "summarize_m2c_performance.py"
spec = importlib.util.spec_from_file_location("summarize_m2c_performance", MODULE_PATH)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)


def test_summary_reports_controller_duration_and_window_rtf(tmp_path):
    stages = tmp_path / "stages.tsv"
    stages.write_text(
        "wall_epoch_s\twall_iso\tstage\tdetail\n"
        "0\tx\trunner_ready\t\n"
        "10\tx\tcontroller_0_start\t\n"
        "50\tx\tcontroller_0_ready\t\n"
        "51\tx\trtf_window_after_controller_0_start\t\n"
        "55\tx\trtf_window_after_controller_0_end\t\n"
        "60\tx\tpass\t\n",
        encoding="utf-8",
    )
    rtf = tmp_path / "rtf.csv"
    rtf.write_text(
        "wall_epoch_s,wall_iso_utc,sim_time_s,real_time_s,real_time_factor,iterations,paused,step_size_s\n"
        "52,x,1,1,0.7,1,False,0.001\n"
        "53,x,2,2,0.8,2,False,0.001\n",
        encoding="utf-8",
    )
    summary = module.summarize(stages, rtf)
    assert "controller_0_startup: 40.000" in summary
    assert "after_controller_0: n=2 mean=0.7500 median=0.7500" in summary
    assert "total_to_pass: 60.000" in summary
