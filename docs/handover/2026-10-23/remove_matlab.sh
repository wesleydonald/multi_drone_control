#!/bin/bash
# Remove MATLAB R2026a (a user-local install) and everything of it (Wesley 23 Oct: not used; nothing in the repo needs it).
set -u
cd /home/wesley
du -shc MATLAB .MathWorks .matlab Downloads/matlab_installer Downloads/matlab_R2026a_Linux.zip \
  Downloads/MATLAB_UR5e_RTDE-main "Downloads/MATLAB_UR5e_RTDE-main.zip" "Downloads/MATLAB_UR5e_RTDE-main(1).zip" 2>/dev/null | tail -1
rm -rf MATLAB .MathWorks .matlab Downloads/matlab_installer Downloads/matlab_R2026a_Linux.zip \
  Downloads/MATLAB_UR5e_RTDE-main "Downloads/MATLAB_UR5e_RTDE-main.zip" "Downloads/MATLAB_UR5e_RTDE-main(1).zip" \
  .local/share/applications/mw-matlab.desktop .local/share/applications/mw-matlabconnector.desktop
cp .bashrc .bashrc.bak_matlab && sed -i 's|^export PATH=/home/wesley/MATLAB/R2026a/bin:\$PATH|# &  (MATLAB removed 23 Oct)|' .bashrc
grep -n "MATLAB" .bashrc
df -h / | tail -1
