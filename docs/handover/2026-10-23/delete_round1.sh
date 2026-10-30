#!/bin/bash
# Round 1 (Aug-Sep): the 15 day archives are on OneDrive (sizes checked against MANIFEST.txt, 23 Oct). Delete the 408
# uncited sim run folders listed in ~/mdc_onedrive_upload/round1_delete_list.txt and the 15 local archives.
set -euo pipefail
L=~/mdc_onedrive_upload/round1_delete_list.txt
n=$(grep -c . "$L"); echo "deleting $n run folders"
while read -r d; do
  case "$d" in /home/wesley/multi_drone_control/results/2026-0[89]-*/R*_sim_*) rm -rf -- "$d" ;; *) echo "skip (unexpected path): $d" ;; esac
done < "$L"
rm -f ~/mdc_onedrive_upload/results_2026-0[89]-*.tar.zst
df -h / | tail -1
