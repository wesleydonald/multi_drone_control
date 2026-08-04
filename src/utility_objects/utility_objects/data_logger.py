# Simplified DataLogger for a single CSV file
import csv
import json
import os
import shutil
from datetime import datetime


def run_log_dir(logging_name, run_name, base_dir=None):
    """Create and return <base>/logs/<logging_name>/<run_name>_<timestamp>/.
    Same layout DataLogger uses, for a node that wants the directory without a CSV.

    base_dir defaults to the CWD, preserving the original behaviour exactly for
    existing callers. Pass utility_objects.run_context.log_base_dir() to write
    into the results tree instead -- see the note in DataLogger below."""
    base = os.getcwd() if base_dir is None else base_dir
    log_dir = os.path.join(base, 'logs', logging_name,
                           run_name + '_' + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)
    return log_dir


def write_params(log_dir, params):
    """Dump `params` as params.json in log_dir, so a run identifies its own
    configuration. Without this, telling two runs apart means reverse-engineering the
    settings from the flown trajectory, which is unreliable (a lean-vs-no-lean A/B is
    not separable from the logs after the fact -- the speed-driven node lag has the
    same signature as the lean).

    Never raises: a logging failure must not take down a flying controller."""
    try:
        safe = {}
        for k, v in dict(params).items():
            safe[str(k)] = v if isinstance(
                v, (bool, int, float, str, type(None))) else str(v)
        path = os.path.join(log_dir, 'params.json')
        with open(path, 'w') as f:
            json.dump(safe, f, indent=2, sort_keys=True)
        return path
    except Exception:
        return None


def node_params(node, extra=None):
    """Every declared ROS parameter of `node` as a plain dict, plus `extra`. Generic so
    a new launch arg shows up in params.json without anyone remembering to add it."""
    out = {}
    try:
        for name, p in node.get_parameters_by_prefix('').items():
            out[name] = p.value
    except Exception:
        pass
    if extra:
        out.update(extra)
    return out


class DataLogger:
    def __init__(self, logging_name, trajectory_name, headers, base_dir=None):
        """base_dir defaults to the CWD, which is the original behaviour and is
        left untouched for every existing caller.

        Passing an explicit base_dir matters for any node that calls os.chdir()
        -- the acados-based trackers chdir into their generated-code directory at
        import time, so a CWD-relative log path put 237 MB of flight data inside a
        build directory that gets deleted on every forced solver rebuild. Those
        nodes now pass utility_objects.run_context.log_base_dir()."""
        # Ensure log directory exists under base_dir, with subdirectory for LOGGING_NAME and trajectory_name_datetime
        base_log_dir = os.path.join(os.getcwd() if base_dir is None else base_dir, 'logs')
        log_dir = os.path.join(base_log_dir, logging_name, trajectory_name + '_' + datetime.now().strftime("%Y%m%d_%H%M%S"))
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # Directory this run's files live in, so callers can drop params.json etc.
        # alongside the CSV (see write_params).
        self.log_dir = log_dir
        # Compose log file name: always 'log.csv' in the log_dir
        self.csv_path = os.path.join(log_dir, 'log.csv')
        self.headers = headers
        self.file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.headers)
        self.closed = False

    def append_row(self, row):
        if len(row) != len(self.headers):
            raise ValueError(f"Row length {len(row)} does not match header length {len(self.headers)}")
        self.writer.writerow(row)

    def close(self):
        if not self.closed:
            self.file.close()
            self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


    def copy_source_files_to_output(self, source_files_info):
        try:
            source_code_dir = os.path.join(self.output_folder, 'source_code')
            os.makedirs(source_code_dir, exist_ok=True)
            
            for filename, filepath in source_files_info.items():
                if os.path.exists(filepath):
                    shutil.copy2(filepath, os.path.join(source_code_dir, filename))
                    print(f"Copied {filename} to {source_code_dir}")
            
            print(f"Source code files copied to: {source_code_dir}")
            
        except Exception as e:
            print(f"Warning: Could not copy source files: {e}")