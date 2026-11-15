"""Resolved parameters of the trial line (launch_dump_trial_fm9h8.json) -> the replay's yaml files.

backend.yaml: his base config, the rig overrides, the launch's generated file and the inline values, in the launch's
order (later wins), with only the replay's plumbing changed: state_topic /virtual_q4/state (the virtual QUAD4),
use_sim_time true (the bag's clock), the CSV paths per run. ring_bridge.yaml: the dumped values, use_sim_time true.
planner_params.json: online_join_planner's resolved values (inline + its yaml files) for the mission driver."""
import json
import os
import sys

import yaml

D = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))   # output (and dump) folder
dump = json.load(open(os.path.join(D, 'launch_dump_trial_fm9h8.json')))


def untype(v):
    kind, val = v
    return [untype(x) for x in val] if kind == 'array' else val


def node(exe):
    return next(e for e in dump['entries'] if e.get('executable') == exe)


def resolved(exe, generated=None):
    e = node(exe)
    params = {k: untype(v) for k, v in e['parameters'].items()}
    files = params.pop('__params_files__', [])
    out = {}
    for f in files:
        if generated and f.startswith('/tmp/'):
            f = generated
        doc = yaml.safe_load(open(f)) or {}
        for key, body in doc.items():
            out.update((body or {}).get('ros__parameters', {}))
    out.update(params)
    return out, files


backend, bfiles = resolved('dynamic_planner_transfer_backend', os.path.join(D, 'backend_trial_generated.yaml'))
backend.update(state_topic='/virtual_q4/state', use_sim_time=True, csv_path='BACKEND_CSV', replan_csv_path='REPLAN_CSV')
yaml.safe_dump({'/**': {'ros__parameters': dict(sorted(backend.items()))}}, open(os.path.join(D, 'backend.yaml'), 'w'),
               default_flow_style=False)
rb, _ = resolved('ring_bridge')
rb.update(use_sim_time=True)
yaml.safe_dump({'/**': {'ros__parameters': dict(sorted(rb.items()))}}, open(os.path.join(D, 'ring_bridge.yaml'), 'w'),
               default_flow_style=False)
pl, pfiles = resolved('online_join_planner')
json.dump(dict(sorted(pl.items())), open(os.path.join(D, 'planner_params.json'), 'w'), indent=1, default=str)
print('backend files', bfiles)
print('planner files', pfiles)
