#!/bin/bash
# Round 2 (1-14 Oct): the 12 day archives are on OneDrive (sizes checked against MANIFEST.txt, 24 Oct). Delete the
# uncited sim run folders in ~/mdc_onedrive_upload/round2_delete_list.txt and the 12 local archives.
set -euo pipefail
L=~/mdc_onedrive_upload/round2_delete_list.txt
echo "deleting $(grep -c . "$L") run folders"
while read -r d; do
  case "$d" in /home/wesley/multi_drone_control/results/2026-10-0[1-9]/R*_sim_*|/home/wesley/multi_drone_control/results/2026-10-1[0-4]/R*_sim_*) rm -rf -- "$d" ;; *) echo "skip (unexpected path): $d" ;; esac
done < "$L"
rm -f ~/mdc_onedrive_upload/results_2026-10-*.tar.zst
df -h / | tail -1
