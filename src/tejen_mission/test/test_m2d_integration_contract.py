from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
DRONE = ROOT / "src" / "tejen_mission" / "tejen_mission"
LAUNCH = ROOT / "src" / "tejen_mission" / "launch"
CORE = ROOT / "src" / "tejen_dynamic_planner" / "standalone"
ROS_CPP = ROOT / "src" / "tejen_dynamic_planner" / "src" / "transfer_backend_node.cpp"
CONTROLLER = ROOT / "src" / "tejen_mpc" / "tejen_mpc" / "main.py"
RUNNER = ROOT / "tools" / "sim_test" / "run_m2d_sequential_attachment.sh"
RVIZ = ROOT / "src" / "drone_visualisation" / "rviz" / "m2d_four_drone.rviz"
M2_PANEL = ROOT / "src" / "drone_visualisation" / "src" / "m2_fleet_panel.cpp"
M2_PANEL_HEADER = ROOT / "src" / "drone_visualisation" / "include" / "drone_visualisation" / "m2_fleet_panel.hpp"
PLUGIN_DESCRIPTION = ROOT / "src" / "drone_visualisation" / "plugin_description.xml"
FLEET = DRONE / "m2_fleet_manager.py"
OBSERVER = ROOT / "tools" / "sim_test" / "m2d_commissioning_observer.py"
CLEANUP = ROOT / "tools" / "sim_test" / "cleanup_m2b_stale.sh"


def text(path: Path) -> str:
    return path.read_text()


def test_m2d_launch_is_four_vehicle_namespaced_and_static_obstacle_free():
    launch = text(LAUNCH / "m2d_four_drone_sequential.launch.py")
    assert "DRONE_IDS = (0, 1, 2, 3)" in launch
    assert '"m2d_enabled": True' in launch
    assert '"static_scene_enabled": False' in launch
    assert '"m2b_ring_pose_source": "pose_array"' in launch
    assert '"assigned_plate_topic": f"/{ns}/m2d/assigned_plate_id"' in launch
    assert '"state_topic": f"/{ns}/attachment/state"' in launch
    assert '"confirmed_topic": f"/{ns}/attachment/confirmed"' in launch
    assert '"cooperative_attached_tether_enabled": True' in launch
    assert 'f"/{peer}/m2d/attached_plate_id"' in launch
    assert '"moving_basket_committed_trajectory_topic": "/m2d/ring/committed_trajectory"' in launch
    assert '"moving_basket_collision_mode": "segmented_ring"' in launch
    assert '"moving_ring_segment_count": M2D_RING_SEGMENT_COUNT' in launch
    assert '"moving_ring_segment_padding_m": M2D_RING_SEGMENT_PADDING_M' in launch
    assert '"reuse_prepared_commit_on_authority": True' in launch
    assert '"octopus_max_runtime_s": 0.10' in launch
    # Namespaced M2D backends must inherit the complete commissioned C1F.6
    # parameter block by value.  Passing the YAML filename directly would key
    # it to the old unnamespaced node and silently fall back to C++ defaults.
    assert "def _load_commissioned_backend_params" in launch
    assert 'data["dynamic_planner_transfer_backend"]["ros__parameters"]' in launch
    assert "deepcopy(commissioned_backend_params)" in launch
    assert 'str(dynamic_share / "config" / "c1f6_moving_rendezvous.yaml")' not in launch
    package_xml = text(ROOT / "src" / "tejen_mission" / "package.xml")
    assert "<exec_depend>python3-yaml</exec_depend>" in package_xml


def test_m2d_segmented_ring_geometry_matches_gazebo_fixture_and_stays_hollow():
    launch = text(LAUNCH / "m2d_four_drone_sequential.launch.py")
    sdf_path = ROOT / "simulation_assets" / "tejen" / "m2a_ring_fixture.sdf"
    model = ET.parse(sdf_path).getroot().find("model")
    assert model is not None

    segments = [
        collision
        for collision in model.findall(".//collision")
        if collision.attrib["name"].startswith("ring_segment_")
    ]
    segments.sort(key=lambda collision: int(collision.attrib["name"].split("_")[2]))
    assert len(segments) == 24

    first_pose = [float(v) for v in segments[0].findtext("pose").split()]
    first_size = [float(v) for v in segments[0].findtext("geometry/box/size").split()]
    radius = math.hypot(first_pose[0], first_pose[1])
    assert radius == 0.25
    assert first_pose[2] == -0.02
    assert first_size == [0.068067840828, 0.06, 0.03]

    expected_literals = {
        "M2D_RING_SEGMENT_COUNT": "24",
        "M2D_RING_SEGMENT_CENTER_RADIUS_M": "0.25",
        "M2D_RING_SEGMENT_TANGENTIAL_LENGTH_M": "0.068067840828",
        "M2D_RING_SEGMENT_RADIAL_WIDTH_M": "0.060",
        "M2D_RING_SEGMENT_HEIGHT_M": "0.030",
        "M2D_RING_SEGMENT_CENTER_Z_OFFSET_M": "-0.020",
        "M2D_RING_SEGMENT_PADDING_M": "0.005",
    }
    for name, literal in expected_literals.items():
        assert re.search(rf"^{name}\s*=\s*{re.escape(literal)}$", launch, re.MULTILINE)

    # Physical padding is deliberately small: the fixture remains topologically
    # hollow before the separately preserved tracking/ego C-space inflation.
    padded_radial_half = 0.5 * first_size[1] + 0.005
    assert radius - padded_radial_half > 0.20



def test_m2d_backend_marker_time_is_declared_at_publish_markers_scope():
    backend = text(ROS_CPP)
    markers = backend.split("void publishMarkers() noexcept {", 1)[1].split(
        "void publishStatus()", 1
    )[0]
    # v16 accidentally declared now_s only inside cooperative_scene_enabled_, while
    # the segmented-ring RViz block immediately after it also consumes now_s.  Keep
    # the shared timestamp at publishMarkers() scope so both marker paths compile.
    assert "const rclcpp::Time now = get_clock()->now();\n            const double now_s = now.seconds();" in markers
    cooperative = markers.split("if (cooperative_scene_enabled_)", 1)[1]
    assert "const double now_s = now.seconds();" not in cooperative


def test_m2d_segmented_ring_backend_batches_scene_updates_and_preserves_predictor():
    backend = text(ROS_CPP)
    world_header = text(CORE / "include" / "dynamic_planner" / "world_snapshot.hpp")
    world_source = text(CORE / "src" / "world_snapshot.cpp")
    assert 'moving_basket_collision_mode_ == "segmented_ring"' in backend
    assert "SegmentedRingGeometry segmented_ring_geometry_" in backend
    assert "refreshMovingBasketCollisionObstacles" in backend
    assert "segmentWorldCenterOffset" in backend
    assert "segmentWorldHalfExtents" in backend
    assert "m2d_ring_segment_" in text(
        ROOT / "src" / "tejen_dynamic_planner" / "include" / "tejen_dynamic_planner" / "moving_basket_geometry.hpp"
    )
    assert "world_->upsertCooperativeTrajectories(std::move(segments))" in backend
    assert "void upsertCooperativeTrajectories" in world_header
    assert "++version_;" in world_source.split(
        "void VersionedWorld::upsertCooperativeTrajectories", 1
    )[1].split("bool VersionedWorld::removeCooperativeTrajectory", 1)[0]
    # Target prediction still reads the unmodified ring-centre commitment.
    predictor = backend.split('target_predictor_type_ == "committed_trajectory"', 1)[1]
    assert "moving_basket_trajectory_sample_.trajectory" in predictor


def test_m2d_authority_reuses_prepared_shape_only_after_fresh_collision_certificate():
    backend = text(ROS_CPP)
    authority = backend.split(
        "void authorityCallback(const std_msgs::msg::Bool::SharedPtr msg)", 1
    )[1].split("void disableBackend", 1)[0]
    assert '"reuse_prepared_commit_on_authority", false' in backend
    assert "shadow_commit_->shiftedToStartTime(now_s)" in authority
    assert "TrajectorySafetyChecker checker" in authority
    assert "checker.checkCommitted(" in authority
    assert "world_->snapshot(now_s)" in authority
    assert "world_->runIfVersionCurrent(" in authority
    assert 'latest_status_ = "AUTHORITY_GRANTED_REBASED_PREPARED"' in authority
    assert "authority_ack_pub_->publish(ack)" in authority
    # Unsafe/stale-time rebases must retain the commissioned fresh-solve fallback.
    assert "falling back to fresh solve" in authority
    assert "makeStationaryHoverTrajectory" in authority


def test_m2d_lost_unacknowledged_grant_resets_backend_epoch_for_clean_retry():
    planner = text(DRONE / "online_join_planner.py")
    prepare = planner.split(
        "if self.c1f2_prepare_settled and not grant_state.ready:", 1
    )[1].split("# Python deliberately keeps", 1)[0]
    assert "self.c1f2_authority_grant_pending = False" in prepare
    assert "self.c1f2_authority_acknowledged = False" in prepare
    assert "self.c1f2_authority_grant_time = None" in prepare
    assert "self.c1f2_authority_grant_ros_time_s = None" in prepare
    assert "self.c1f2_authority_ack_time = None" in prepare
    assert "self.c1f2_publish_authority(False)" in prepare
    assert "self.c1f1_publish_shadow_enable(False)" in prepare



def test_online_planner_keeps_manual_arm_gate_and_live_ring_target():
    planner = text(DRONE / "online_join_planner.py")
    assert 'self.declare_parameter("m2d_enabled", False)' in planner
    assert "def m2d_mission_permission_callback" in planner
    assert "def m2d_assigned_plate_callback" in planner
    assert "def m2b_ring_pose_world" in planner
    assert 'self.m2b_ring_pose_source == "pose_array"' in planner
    assert "not self.m2d_assignment_received or not self.m2d_mission_permission" in planner
    # Permission loss after launch must remove mission authority through the
    # existing M2 fault/hold path, but normal attached-hold progression is exempt.
    assert "MissionPhase.M2_ATTACHED_HOLD" in planner
    assert "M2D fleet permission revoked while vehicle mission was active" in planner
    assert "MissionPhase.M2_FAULT" in planner
    # Once software proof has promoted a vehicle to ATTACHED_HOLD, M2D loss is
    # fleet-terminal.  The legacy local same-plate retry path remains only for
    # pre-proof failures and non-M2D B1/B2 commissioning.
    assert "M2D proven attachment lost during attached hold" in planner
    attached_hold_block = planner.split(
        "if self.phase == MissionPhase.M2_ATTACHED_HOLD:", 1
    )[1].split("if self.phase == MissionPhase.M2_DETACHED_RETREAT:", 1)[0]
    assert "if self.m2d_enabled:" in attached_hold_block
    assert "MissionPhase.M2_FAULT" in attached_hold_block


def test_attachment_nodes_accept_frozen_dynamic_plate_and_live_ring():
    observer = text(DRONE / "m2_attachment_observer.py")
    capture = text(DRONE / "m2a_physical_capture_manager.py")
    for source in (observer, capture):
        assert 'declare_parameter("assigned_plate_topic"' in source
        assert 'declare_parameter("ring_pose_source"' in source
        assert '"pose_array"' in source
    assert "self.assignment_locked = True" in observer
    assert "Ignoring M2D plate reassignment after attach command" in capture
    assert "ring_pose_timeout_s" in capture


def test_supervisor_uses_software_proof_not_gazebo_truth_for_advancement():
    core = text(DRONE / "m2d_sequential_supervision.py")
    wrapper = text(DRONE / "m2d_fleet_supervisor.py")
    assert 'ATTACHED_PHASE = "M2_ATTACHED_HOLD"' in core
    assert "evidence.confirmed" in core
    assert "not evidence.lost" in core
    assert "joint_detached" not in core
    assert "joint_truth" not in core
    assert 'f"{ns}/attachment/diagnostics"' in wrapper
    assert 'f"{ns}/m2d/mission_permission"' in wrapper
    assert 'f"{ns}/m2d/attached_plate_id"' in wrapper


def test_attached_tether_geometry_is_endpoint_derived_not_swing_cone():
    header = text(CORE / "include" / "dynamic_planner" / "ego_collision_model.hpp")
    source = text(CORE / "src" / "ego_collision_model.cpp")
    backend = text(ROS_CPP)
    assert "struct AttachedTetherGeometry" in header
    assert "anchor_from_body" in header
    assert "plate_from_body" in header
    assert "#include <Eigen/Geometry>" in source
    assert 'ConvexComponent{"attached_tether"' in source
    assert "g.plate_from_body - g.anchor_from_body" in source
    assert "cooperativeAttachedTetherGeometry" in backend
    assert "cooperativePlateWorldPosition" in backend
    assert "cooperative_ring_pose_sample_.rotation * plate_ring" in backend
    assert "cooperative_attached_plate_topics" in backend
    coop_log = backend.split('Accepted cooperative shared trajectory', 1)[0][-200:]
    basket_log = backend.split('Accepted moving basket', 1)[0][-200:]
    assert 'RCLCPP_DEBUG(' in coop_log
    assert 'RCLCPP_DEBUG(' in basket_log
    # Existing SuspendedGeometry remains for the suspended-payload use case, but
    # the M2D attached tether has no swing-angle field of its own.
    tether_decl = header.split("struct AttachedTetherGeometry", 1)[1].split("};", 1)[0]
    assert "swing" not in tether_decl.lower()


def test_ring_commitment_is_measured_stationary_authority():
    source = text(DRONE / "m2d_ring_commitment.py")
    assert 'declare_parameter("ring_pose_topic", "/model/payload_model/pose")' in source
    assert 'declare_parameter("ring_pose_index", 1)' in source
    assert "msg.terminal_hold = True" in source
    assert "CommittedTrajectory" in source
    assert "PoseArray" in source


def test_controller_launch_preserves_m2c_default_and_allows_m2d_permission_gate():
    source = text(LAUNCH / "m2c_vehicle_controller.launch.py")
    controller = text(CONTROLLER)
    assert 'DeclareLaunchArgument("require_external_arm_permission", default_value="false")' in source
    assert '"require_external_arm_permission": require_external_arm_permission' in source
    assert '"external_arm_permission_topic": f"/{ns}/join_planner/arm_permission"' in source
    assert 'DeclareLaunchArgument("arming_state_feedback_period_s", default_value="0.0")' in source
    assert '"arming_state_feedback_period_s": arming_state_feedback_period_s' in source
    assert "self.declare_parameter('arming_state_feedback_period_s', 0.0)" in controller
    assert "self.cb.publish_arming_state" in controller
    assert "takeoff_state_feedback" in controller
    assert "publish_operator_state_feedback" in controller


def test_runner_is_manual_and_supports_ground_one_and_full_validation():
    runner = text(RUNNER)
    assert 'M2D_GOAL_ATTACHMENTS:-4' in runner
    assert '0|1|2|4)' in runner
    assert "manual_arm_takeoff=true" in runner
    assert "require_external_arm_permission:=true" in runner
    assert "arming_state_feedback_period_s:=0.5" in runner
    assert "ros2 service call /drone_${drone_id}/arming_service" not in runner
    assert "ros2 topic pub --once -w 2 /drone_${drone_id}/command" in runner
    assert "'{data: ARM}'" in runner
    assert "'{data: TAKEOFF}'" in runner
    assert "wait_for_armed_feedback" in runner
    assert "wait_for_takeoff_acknowledgement" in runner
    assert "M2_VERTICAL_TAKEOFF" in runner
    assert "CONTROLLER TRUE + PLANNER PHASE ADVANCED" in runner
    assert "manual_activate_vehicle" in runner
    assert "release_bootstrap_until_armable" in runner
    assert '"M2_WAIT_FOR_ARM"' in runner
    assert "bootstrap_never_reached_wait_for_arm" in runner
    assert "publish_bootstrap_release" not in runner
    # Commissioning uses one persistent mission-state observer rather than spawning
    # ROS CLI discovery/echo nodes. Graph discovery is diagnostic at most and must
    # never be an authoritative readiness gate.
    assert "m2d_commissioning_observer.py" in runner
    assert "load_observer_snapshot" in runner
    assert "commissioning_status.json" in runner
    assert "commissioning_status.env" in runner
    assert "ros2 node list" not in runner
    assert "ros2 service list" not in runner
    assert "ros2 topic info" not in runner
    assert "ros2 topic echo" not in runner
    for forbidden in (
        "_node_missing",
        "_service_missing",
        "reference_subscribers",
        "elrs_subscribers",
        "command_subscribers",
        "detach_subscribers",
        "bootstrap_release_subscribers",
    ):
        assert forbidden not in runner
    # The runner prints manual ARM/TAKEOFF commands. It must not execute them itself.
    command_block = runner.split("print_arm_command()", 1)[1].split("write_pass_result()", 1)[0]
    assert "cat <<EOT" in command_block
    assert "eval" not in command_block
    assert "bash -c" not in command_block
    assert "M2D_PASS" in runner
    assert "m2d_clock_policy=all_sim_ros" in runner


def test_m2d_commitments_are_single_source_and_terminal_hold_is_cubic_truthful():
    planner = text(DRONE / "online_join_planner.py")
    assert "if self.publish_committed_trajectory and not self.m2d_enabled" in planner
    assert "stationary_phases = {" in planner
    assert "MissionPhase.M2_ATTACHED_HOLD" in planner
    assert "terminal_velocity = 3.0 * (control[3] - control[2]) / duration" in planner
    assert "terminal_acceleration = (" in planner
    assert "np.max(np.abs(terminal_velocity)) <= 1e-6" in planner
    assert "np.max(np.abs(terminal_acceleration)) <= 1e-6" in planner
    assert "reference.accelerations[-1]" not in planner.split(
        "def publish_m2d_reference_commitment", 1
    )[1].split("def publish_reference", 1)[0]

    fleet = text(FLEET)
    assert "if self.pass_latched:" in fleet
    assert "M2C is a ground-only commissioning gate" in fleet


def test_octopus_failure_retains_witness_candidate_and_backend_visualizes_it():
    header = text(CORE / "include" / "dynamic_planner" / "octopus_search.hpp")
    search = text(CORE / "src" / "octopus_search.cpp")
    backend = text(ROS_CPP)
    test_source = text(CORE / "tests" / "test_octopus.cpp")
    assert "diagnostic_control_points" in header
    assert "result.diagnostic_control_points = completed" in search
    assert "finalCheckFailureWitness" in backend
    assert "C1F final-check witness: obstacle=%s interval=%d" in backend
    assert '"failed_candidate_path"' in backend
    assert '"failed_candidate_control_points"' in backend
    assert '"final_check_witness"' in backend
    assert '"m2d_ring_segment_collision"' in backend
    assert '"m2d_ring_segment_tracking"' in backend
    assert '"cooperative_attached_tether_physical"' in backend
    assert '"cooperative_attached_tether_collision"' in backend
    assert "testFinalCheckFailureRetainsDiagnosticWitnessAndCandidate" in test_source
    assert '"cooperative_drone_1::attached_tether_vs_ego_body"' in test_source


def test_m2d_runner_uses_namespaced_four_drone_rviz_config():
    runner = text(RUNNER)
    rviz = text(RVIZ)
    assert "m2d_four_drone.rviz" in runner
    assert "RViz: M2D four-drone planner/collision witness view LAUNCHED (non-authoritative)" in runner
    for drone_id in range(4):
        assert f"/drone_{drone_id}/join_planner/markers" in rviz
        assert f"/drone_{drone_id}/dynamic_planner/markers" in rviz
    assert "cooperative_attached_tether_physical" in rviz
    assert "cooperative_attached_tether_collision" in rviz
    assert "failed_candidate_path" in rviz
    assert "final_check_witness" in rviz



def test_commissioning_observer_tracks_functional_state_without_graph_authority():
    observer = text(OBSERVER)
    runner = text(RUNNER)
    assert 'super().__init__("m2d_commissioning_observer")' in observer
    assert 'Clock, "/clock"' in observer
    assert 'String, "/m2c/status"' in observer
    assert 'Bool, "/m2c/ready"' in observer
    assert 'String, "/m2c/assignment"' in observer
    assert 'String, "/m2d/status"' in observer
    assert 'f"{ns}/magnet/joint_detached_truth"' in observer
    assert 'f"{ns}/join_planner/phase"' in observer
    assert 'f"{ns}/dynamic_planner/transfer_status"' in observer
    assert 'f"{ns}/arming_state_feedback"' in observer
    assert 'f"{ns}/takeoff_state_feedback"' in observer
    assert "get_node_names_and_namespaces" not in observer
    assert "get_service_names_and_types" not in observer
    assert "count_subscribers" not in observer
    assert 'f"{ns}/tejen_mpc/c1d_status"' not in observer
    assert 'f"{ns}/telemetry"' not in observer
    assert 'f"{ns}/join_planner/reference"' not in observer
    assert "time.monotonic()" in observer
    assert "use_sim_time" not in observer
    assert "os.replace" in observer
    assert "observer_wall_unix_s" in observer
    assert runner.index("m2d_commissioning_observer.py") < runner.index("m2d_four_drone_sequential.launch.py")


def test_m2d_runner_uses_owner_level_functional_readiness_only():
    runner = text(RUNNER)
    # Thesis-specific fixture release remains end-to-end acknowledged. The raw
    # Gazebo state is transition-based, so prove the bridge transport listener is
    # present rather than requiring a nonexistent initial ROS `false` sample.
    assert "wait_for_gz_subscriber" in runner
    assert "prime_joint_truth_observation" not in runner
    assert "release_joint_until_detached" in runner
    assert 'wait_for_int_eq m2c_joint_detached_count 4 60' in runner
    # Backend runtime output and planner phase are authoritative component evidence.
    assert 'wait_for_bool_key "drone_${i}_backend_status_seen" true 60' in runner
    assert "release_bootstrap_until_armable" in runner
    assert '"M2_WAIT_FOR_ARM"' in runner
    # Controller construction is proven by the deliberately post-init DISARMED
    # heartbeat as consumed by M2C, not by service/node/diagnostic discovery.
    assert 'wait_for_int_eq m2c_disarmed_controller_count "$((i + 1))" 180' in runner
    assert "controller_ready" not in runner
    assert "reference_seen" not in runner
    assert "telemetry_seen" not in runner
    # Fleet authority then collapses to the existing aggregate contracts.
    assert 'wait_for_bool_key m2c_ready_latched' not in runner
    assert 'if wait_for_m2d_state ACTIVE_0 120' in runner
    assert "wait_for_assignment" not in runner
    assert 'm2c_candidate_count:-}" == "72"' not in runner


def test_m2d_cleanup_owns_m2d_process_groups_and_fastdds_shared_memory():
    cleanup = text(CLEANUP)
    assert ".m2d_active_pgid" in cleanup
    assert "m2d_four_drone_sequential.launch.py" in cleanup
    assert "m2d_fleet_supervisor" in cleanup
    assert "m2d_ring_commitment" in cleanup
    assert "m2d_commissioning_observer.py" in cleanup
    assert "fastrtps_*" in cleanup
    assert "sem.fastrtps_*" in cleanup
    daemon_stop = cleanup.index("ros2 daemon stop")
    dds_clear = cleanup.index("for DDS_PATH in /dev/shm/fastrtps_*")
    daemon_start = cleanup.index("ros2 daemon start")
    assert daemon_stop < dds_clear < daemon_start

def test_v9_rviz_is_late_and_non_authoritative():
    runner = text(RUNNER)
    active_gate = runner.index("if wait_for_m2d_state ACTIVE_0 120")
    rviz_launch = runner.index("start_managed ros2 run rviz2")
    assert active_gate < rviz_launch
    assert "RViz: M2D four-drone planner/collision witness view LAUNCHED (non-authoritative)" in runner
    assert "wait_for_bool_key node_rviz2" not in runner
    assert 'fail "rviz_never_started"' not in runner
    assert 'fail "m2d_rviz_config_missing"' not in runner


def test_startup_timeouts_are_hardened_without_weakening_mission_gates():
    runner = text(RUNNER)
    assert 'wait_for_int_eq m2c_disarmed_controller_count "$((i + 1))" 180' in runner
    assert 'wait_for_bool_key m2c_ready_latched' not in runner
    assert 'if wait_for_m2d_state ACTIVE_0 120' in runner
    assert 'wait_for_m2d_state "ACTIVE_${next_index}" 600' in runner
    assert "planner_phase_is_after_takeoff" in runner
    assert "m2d_aborted_before_first_permission" in runner


def test_v10_m2c_ready_is_latched_by_persistent_observer():
    observer = text(OBSERVER)
    runner = text(RUNNER)
    assert "self.m2c_ready_latched = False" in observer
    assert "if self.m2c_ready:" in observer
    assert "self.m2c_ready_latched = True" in observer
    assert '"m2c_ready_latched": self.m2c_ready_latched' in observer
    assert 'wait_for_bool_key m2c_ready_latched' not in runner
    assert 'wait_for_bool_key m2c_ready true' not in runner
    assert 'if wait_for_m2d_state ACTIVE_0 120' in runner


def test_v14_paused_detach_proves_gz_truth_listener_before_release():
    runner = text(RUNNER)
    assert "gz_topic_subscriber_line()" in runner
    assert "wait_for_gz_subscriber()" in runner
    assert "prime_joint_truth_observation" not in runner
    assert 'wait_for_gz_subscriber "/drone_${i}/magnet/joint_detached_truth" 15' in runner
    listener_call = runner.index('wait_for_gz_subscriber "/drone_${i}/magnet/joint_detached_truth" 15')
    release_call = runner.index('release_joint_until_detached "$i" 30')
    assert listener_call < release_call
    assert 'drone_${i}_raw_truth_listener_not_discovered' in runner
    assert 'drone_${i}_raw_truth_ros_path_not_observed' not in runner

def test_v15_action_loops_acknowledge_success_before_timeout_failure():
    runner = text(RUNNER)

    assert 'local drone_id="$1" timeout_s="$2" started="$(date +%s)" key=' not in runner
    assert 'local key="drone_${drone_id}_joint_detached"' in runner
    assert 'local drone_id="$1" timeout_s="$2" started="$(date +%s)" phase_key=' not in runner
    assert 'local phase_key="drone_${drone_id}_phase"' in runner

    detach_start = runner.index("release_joint_until_detached() {")
    detach_end = runner.index("\n}\n\nwait_for_m2d_state()", detach_start)
    detach = runner[detach_start:detach_end]
    assert "while true; do" in detach
    assert "while (( $(date +%s) - started < timeout_s )); do" not in detach
    first_ack = detach.index('if load_observer_snapshot && [[ "${!key:-false}" == "true" ]]; then')
    deadline = detach.index('if (( $(date +%s) - started >= timeout_s )); then')
    publish = detach.index('ros2 topic pub --once "$detach_topic"')
    post_action_ack = detach.rindex('if load_observer_snapshot && [[ "${!key:-false}" == "true" ]]; then')
    assert first_ack < deadline < publish < post_action_ack
    assert "commissioning_observer_died_during_drone_${i}_detach" in runner
    assert "drone_${i}_world_step_failed_during_detach" in runner
    assert "drone_${i}_detach_confirmation_timeout" in runner
    assert "drone_${i}_raw_detached_truth_missing" not in runner

    bootstrap_start = runner.index("release_bootstrap_until_armable() {")
    bootstrap_end = runner.index("\n}\n\nwait_for_armed_feedback()", bootstrap_start)
    bootstrap = runner[bootstrap_start:bootstrap_end]
    phase_ack = bootstrap.index('[[ "$phase" == "M2_WAIT_FOR_ARM" ]] && return 0')
    deadline = bootstrap.index('(( $(date +%s) - started < timeout_s )) || return 1')
    publish = bootstrap.index('ros2 topic pub --once "$release_topic"')
    assert phase_ack < deadline < publish
    assert "commissioning_observer_died_during_drone_${i}_bootstrap" in runner
    assert "commissioning_observer_died_during_drone_${drone_id}_arm" in runner
    assert "commissioning_observer_died_during_drone_${drone_id}_takeoff" in runner

    observer = text(OBSERVER)
    assert "snapshot_on_change: bool = False" in observer
    assert "if snapshot_on_change and changed:" in observer
    assert "snapshot_on_change=True" in observer


def test_v8_octopus_witness_tests_link_collision_geometry_helpers():
    standalone_cmake = text(CORE / "CMakeLists.txt")
    ros_cmake = text(ROOT / "src" / "tejen_dynamic_planner" / "CMakeLists.txt")
    assert (
        "target_link_libraries(test_octopus PRIVATE dynamic_planner_octopus "
        "dynamic_planner_collision)"
    ) in standalone_cmake
    assert (
        "target_link_libraries(test_c1f5p2_octopus_restart_contract "
        "dynamic_planner_octopus_ros dynamic_planner_collision_ros)"
    ) in ros_cmake


def test_backend_runtime_status_is_gated_only_after_unpause_and_clock_advance():
    runner = text(RUNNER)
    unpause = runner.index("--req 'pause: false'")
    clock_advance = runner.index('wait_for_int_gt_value clock_sim_ns "$CLOCK_BEFORE_UNPAUSE" 60')
    backend_gate = runner.index('wait_for_bool_key "drone_${i}_backend_status_seen" true 60')
    assert unpause < clock_advance < backend_gate
    pre_unpause = runner[:unpause]
    assert 'backend_status_seen" true' not in pre_unpause
    assert "backend_status_not_observed_after_unpause" in runner


def test_v11_observer_exports_numeric_sim_clock_for_post_unpause_progress_proof():
    observer = text(OBSERVER)
    runner = text(RUNNER)
    assert 'self.create_subscription(Clock, "/clock", self._clock_callback, 10)' in observer
    assert "self.clock_sim_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)" in observer
    assert '"clock_sim_ns": self.clock_sim_ns if self.clock_sim_ns is not None else ""' in observer
    assert 'CLOCK_BEFORE_UNPAUSE="${clock_sim_ns:-}"' in runner
    assert 'simulation_clock_not_advancing_after_unpause' in runner
    assert 'ROS simulation time: ADVANCING after unpause' in runner


def test_v11_clock_wording_distinguishes_bridge_presence_from_time_progress():
    runner = text(RUNNER)
    assert 'ROS simulation clock: OBSERVED while world is paused' in runner
    assert 'ROS clock:    SIMULATION (/clock observed persistently)' not in runner

def test_v12_clock_subscription_does_not_collide_with_rclpy_node_clock_attribute():
    observer = text(OBSERVER)
    assert 'self.create_subscription(Clock, "/clock", self._clock_callback, 10)' in observer
    assert 'def _clock_callback(self, msg: Clock) -> None:' in observer
    assert 'self.create_subscription(Clock, "/clock", self._clock, 10)' not in observer
    assert 'def _clock(self, msg: Clock) -> None:' not in observer




def test_m2d_operator_hold_and_release_before_land_contract_is_wired_end_to_end():
    launch = text(LAUNCH / "m2d_four_drone_sequential.launch.py")
    supervisor = text(DRONE / "m2d_fleet_supervisor.py")
    core = text(DRONE / "m2d_sequential_supervision.py")
    planner = text(DRONE / "online_join_planner.py")
    runner = text(RUNNER)

    assert 'DeclareLaunchArgument("target_attachment_count", default_value="4")' in launch
    assert 'DeclareLaunchArgument("operator_hold_after_goal", default_value="false")' in launch
    assert '"operator_land_request_topic": "/m2d/operator/land_request"' in launch
    assert 'f"{ns}/join_planner/land_now"' in supervisor
    assert '"M2D_SUCCESS_HOLD"' in core
    assert '"M2D_LANDING"' in core
    assert '"M2D_LANDED"' in core
    assert 'tuple(reversed(ordered_attached))' in core

    assert 'self.is_m2b and self.phase == MissionPhase.M2_ATTACHED_HOLD' in planner
    assert 'MissionPhase.M2_LANDING_STAGE' in planner
    assert 'release before landing' in planner
    assert 'self.m2b_landing_detach_hold_position' in planner
    assert 'Only after detached truth is fresh' in planner

    assert 'M2D_GOAL_ATTACHMENTS=2' in runner
    assert 'M2D_OPERATOR_HOLD' in runner
    assert 'wait_for_m2d_state M2D_SUCCESS_HOLD' in runner
    assert 'wait_for_m2d_state M2D_LANDED' in runner


def test_m2d_rviz_uses_dedicated_fleet_panel_without_replacing_single_drone_panel():
    rviz = text(RVIZ)
    panel = text(M2_PANEL)
    panel_header = text(M2_PANEL_HEADER)
    plugin = text(PLUGIN_DESCRIPTION)
    cmake = text(ROOT / "src" / "drone_visualisation" / "CMakeLists.txt")

    assert "drone_visualisation/M2FleetPanel" in rviz
    assert "drone_visualisation/M2FleetPanel" in plugin
    assert "src/m2_fleet_panel.cpp" in cmake
    assert "src/arm_panel.cpp" in cmake
    assert "class M2FleetPanel" in panel_header
    assert '"/m2d/status"' in panel
    assert '"/m2d/operator/land_request"' in panel
    for drone_id in range(4):
        assert f'"/drone_{drone_id}/command"' not in panel  # generated from vehicleId(), not duplicated literals
    assert '"/" + vehicle + "/command"' in panel
    assert '"/" + vehicle + "/arming_state_feedback"' in panel
    assert '"/" + vehicle + "/join_planner/phase"' in panel
    assert 'fleet_state_ != "M2D_SUCCESS_HOLD"' in panel
    assert 'publishCommand(i, "DISARM")' in panel
