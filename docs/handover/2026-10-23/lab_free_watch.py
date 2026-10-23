"""Exit 0 once nobody else is on the lab PC (others' cores < 2.0 on two checks 5 min apart); exit 1 after 8 h."""
import sys, time
sys.path.insert(0, '/home/wesley/multi_drone_control/tools/remote')
import lab_batch
calm, t0 = 0, time.time()
while time.time() - t0 < 8 * 3600:
    try:
        load = lab_batch.lab_load()
        others, ours = load['others'], load['ours']
    except Exception as e:                      # an ssh hiccup is not "free"
        others, ours = float('nan'), float('nan')
        print(time.strftime('%H:%M'), 'probe failed:', e, flush=True)
    print(time.strftime('%H:%M'), f'others {others:.1f} cores, ours {ours:.1f}', flush=True)
    calm = calm + 1 if others < 2.0 else 0
    if calm >= 2:
        print('lab free', flush=True)
        sys.exit(0)
    time.sleep(300)
sys.exit(1)
