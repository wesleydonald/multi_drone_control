# Parity image of the laptop sim stack: mdc-humble:db1c3ca
# Build context is tools/remote/ (see lab_build.sh): needs debs/ and t_renderer from the laptop.
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG ACADOS_COMMIT=db1c3caa6571fbeadf06ac10081ed15965cd0926
# The lab PC is shared: keep the compile under the 16-core batch limit.
ARG JOBS=12

RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic at the laptop's exact versions (gz-sim 8.10.0, physics 7.6.0,
# ros_gzharmonic 0.244.12), repacked from the laptop by repack_gz_debs.sh.
COPY debs/ /tmp/debs/
RUN apt-get update && apt-get install -y --no-install-recommends --allow-downgrades \
      /tmp/debs/*.deb \
      build-essential cmake git rsync procps psmisc \
      libglpk-dev glpk-utils libeigen3-dev qtbase5-dev python3-pyqt5 \
      python3-pip python3-scipy python3-matplotlib python3-yaml python3-pytest \
      python3-future python3-opencv python3-colcon-common-extensions \
      ros-humble-tf-transformations ros-humble-xacro ros-humble-robot-state-publisher \
      ros-humble-cv-bridge ros-humble-rosbag2-py \
 && for d in /tmp/debs/*.deb; do apt-mark hold "$(dpkg-deb -f "$d" Package)" >/dev/null; done \
 && rm -rf /tmp/debs /var/lib/apt/lists/*

# The laptop's pip packages live in ~/.local; here they go to /usr/local, which also
# shadows the apt numpy 1.21 exactly as ~/.local does on the laptop.
RUN pip3 install --no-cache-dir \
      numpy==1.24.4 casadi==3.7.2 pandas==2.3.3 pyserial==3.5 transforms3d==0.4.2 \
      python-dateutil==2.9.0.post0 Deprecated==1.2.13 cython

# acados at the laptop's commit. BLASFEO_TARGET is pinned to what X64_AUTOMATIC chose
# on the laptop (HASWELL); left automatic, the lab CPU could select different kernels.
RUN git clone https://github.com/acados/acados.git /opt/acados \
 && cd /opt/acados && git checkout ${ACADOS_COMMIT} \
 && git submodule update --init --recursive \
 && mkdir build && cd build \
 && cmake .. -DACADOS_WITH_OPENMP=OFF -DBLASFEO_TARGET=X64_INTEL_HASWELL \
      -DCMAKE_BUILD_TYPE=Release \
 && make install -j"${JOBS}" \
 && pip3 install --no-cache-dir -e /opt/acados/interfaces/acados_template
COPY t_renderer /opt/acados/bin/t_renderer

RUN useradd -m -u 1000 -d /home/wesley -s /bin/bash wesley \
 && ln -s /opt/acados /home/wesley/acados \
 && chown -R wesley:wesley /opt/acados \
 && mkdir -p /home/wesley/multi_drone_control && chown wesley:wesley /home/wesley/multi_drone_control

# tejen_dynamic_planner links a static qpOASES 3.2.2 from ~/.local, as on the laptop.
RUN git clone --depth 1 -b releases/3.2.2 https://github.com/coin-or/qpOASES.git /tmp/qpOASES \
 && cmake -S /tmp/qpOASES -B /tmp/qpOASES/build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/home/wesley/.local -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DQPOASES_BUILD_EXAMPLES=OFF \
 && cmake --build /tmp/qpOASES/build -j"${JOBS}" --target install \
 && chown -R wesley:wesley /home/wesley/.local && rm -rf /tmp/qpOASES

# run_context finds the repo root by .git, which lab_sync.sh does not copy.
ENV MDC_REPO_ROOT=/home/wesley/multi_drone_control \
    ACADOS_SOURCE_DIR=/home/wesley/acados \
    LD_LIBRARY_PATH=/opt/acados/lib \
    PYTHONPATH=/opt/acados/interfaces/acados_template \
    GZ_SIM_RESOURCE_PATH=/home/wesley/multi_drone_control/simulation_assets:/home/wesley/multi_drone_control/simulation_assets/models

COPY entrypoint.sh /usr/local/bin/mdc-entrypoint.sh
USER wesley
WORKDIR /home/wesley/multi_drone_control
ENTRYPOINT ["/usr/local/bin/mdc-entrypoint.sh"]
CMD ["bash"]
