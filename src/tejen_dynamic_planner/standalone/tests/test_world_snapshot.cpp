#include "dynamic_planner/world_snapshot.hpp"
#include "dynamic_planner/trajectory_geometry.hpp"
#include "dynamic_planner/separator.hpp"
#include "test_common.hpp"

#include <iostream>
#include <functional>
#include <Eigen/Geometry>

using namespace dynamic_planner;

namespace {

WorldSnapshot rotatedFullHeightPanelWorld() {
    WorldSnapshot world;
    world.ego_half_extents = Vec3(.105,.105,.1);
    world.ego_tracking_error_half_extents = Vec3(.1,.1,.05);
    world.ego_body_rotation = Eigen::Quaterniond(.9999944846969365,
        .0020711725112486025,.002567123047008457,-.0003882002045990777)
        .normalized().toRotationMatrix();
    auto& g=world.ego_suspended_geometry;
    g.enabled=g.payload_attached=true;
    g.swing_envelope=SwingEnvelope::OuterOctagon;
    g.cable_length_m=.49;
    g.magnet_center_below_cable_end_m=0.;
    g.payload_half_extents=Vec3(.043,.043,.011);
    g.payload_center_from_magnet_center=Vec3(0.,0.,-.036);
    world.workspace.enabled=true;
    world.workspace.lower=Vec3(-2.5,-2.5,0.);
    world.workspace.upper=Vec3(2.5,2.5,1.6);
    // Frozen S50/80 panel, not a dependency on ignored runtime artifacts.
    Eigen::MatrixXd panel(8,3);
    panel << -.8893993505113151,.7479779942740059,0.,
        -.8893993505113151,.7479779942740059,1.6,
        1.752022005725995,3.389399350511316,0.,
        1.752022005725995,3.389399350511316,1.6,
        -.8186886723926605,.6772673161553513,0.,
        -.8186886723926605,.6772673161553513,1.6,
        1.8227326838446496,3.3186886723926614,0.,
        1.8227326838446496,3.3186886723926614,1.6;
    world.physical_static_obstacles.push_back({"wall",panel});
    return world;
}

void testWorkspacePrismCompactionAndCollisionEquivalence() {
    const auto world=rotatedFullHeightPanelWorld();
    const auto timed=world.timeIndexedObstacles(4,0.,4.);
    const auto components=assemblyComponents(world.ego_half_extents,
        world.ego_suspended_geometry,world.ego_body_rotation);
    requireTrue(timed.size()==4 && components.size()==4,
                "prism compaction dropped an assembly collision relation");
    const auto bounds=assemblyCentreBounds(world.workspace,world.ego_half_extents,
        world.ego_suspended_geometry,world.ego_tracking_error_half_extents,world.ego_body_rotation);
    std::size_t inside=0,clear=0;
    Separator oracle;
    for(std::size_t i=0;i<components.size();++i){
        auto tracked=components[i];
        tracked.vertices=inflateVerticesByAabb(tracked.vertices,world.ego_tracking_error_half_extents);
        const auto raw=physicalObstacleToConfigurationSpace(
            world.physical_static_obstacles.front().vertices,tracked);
        const auto old=compactHorizontalLayers(raw);
        const auto& compact=timed[i].interval_vertices.front();
        requireTrue(compact.rows()<old.rows(),"eligible prism still retains redundant Z layers");
        if(i==0)requireTrue(old.rows()>=128 && compact.rows()<=32,
                           "rotated body did not recover bounded prismatic geometry");
        requireNear(compact.col(2).minCoeff(),raw.col(2).minCoeff(),0.,"prism changed lower Z extent");
        requireNear(compact.col(2).maxCoeff(),raw.col(2).maxCoeff(),0.,"prism changed upper Z extent");
        for(int angle=0;angle<180;++angle){
            const Vec3 horizontal(std::cos(angle*.07),std::sin(angle*.07),0.);
            requireNear((compact*horizontal).maxCoeff(),(raw*horizontal).maxCoeff(),1e-12,
                        "prism changed horizontal collision support");
            const Vec3 axis(horizontal.x(),horizontal.y(),std::sin(angle*.19));
            requireTrue((compact*axis).maxCoeff()+1e-12 >= (raw*axis).maxCoeff(),
                        "prism failed global containment of the original configuration hull");
        }
        // Independent LP membership and short convex trajectory hulls near the
        // diagonal face/corners, within the identical assembly-centre Z bounds.
        for(int ix=0;ix<8;++ix)for(int iy=0;iy<8;++iy)for(double z:{.7,1.,1.4}){
            Eigen::MatrixXd hull(4,3);
            const Vec3 centre(-1.05+.46*ix,.11+.52*iy,z);
            for(int row=0;row<4;++row)hull.row(row)=
                (centre+Vec3(.013*(row-1),-.017*(row%2),.006*(row%2))).transpose();
            requireTrue(hull.col(2).minCoeff()>=bounds.first.z() &&
                        hull.col(2).maxCoeff()<=bounds.second.z(),"oracle sample escaped legal Z");
            const bool before=oracle.solve(hull,raw).feasible;
            const bool after=oracle.solve(hull,compact).feasible;
            requireTrue(before==after,"prism changed collision decisions inside the legal workspace");
            before?++clear:++inside;
        }
    }
    requireTrue(inside>0 && clear>0,"prism oracle did not exercise both contact and clearance");
}

void testIneligibleAndLegacyPrismsRetainExactGeometry() {
    const auto base=rotatedFullHeightPanelWorld();
    for(int variant=0;variant<6;++variant){
        auto world=base;
        if(variant==0)world.ego_suspended_geometry.swing_envelope=SwingEnvelope::LegacySquare;
        if(variant==1)world.workspace.enabled=false;
        if(variant==2)world.physical_static_obstacles[0].vertices.col(2)*=.9; // too short
        if(variant==3){ // tilted prism, not merely a tall AABB
            const Eigen::Matrix3d tilt=Eigen::AngleAxisd(.02,Vec3::UnitX()).toRotationMatrix();
            world.physical_static_obstacles[0].vertices=
                (world.physical_static_obstacles[0].vertices*tilt.transpose()).eval();
        }
        if(variant==4)world.physical_static_obstacles[0].vertices(1,0)+=1e-12; // unpaired layer
        if(variant==5)world.physical_static_obstacles[0].vertices.col(1).setZero(); // zero XY area
        const auto timed=world.timeIndexedObstacles(1,0.,1.);
        const auto components=assemblyComponents(world.ego_half_extents,
            world.ego_suspended_geometry,world.ego_body_rotation);
        for(std::size_t i=0;i<components.size();++i){
            auto tracked=components[i];
            tracked.vertices=inflateVerticesByAabb(tracked.vertices,world.ego_tracking_error_half_extents);
            const auto expected=compactHorizontalLayers(physicalObstacleToConfigurationSpace(
                world.physical_static_obstacles[0].vertices,tracked));
            requireMatrixNear(timed[i].interval_vertices.front(),expected,0.,
                              "ineligible/legacy prism changed its original geometry");
        }
    }
}

CommittedTrajectory linearTrajectory(const Vec3& a, const Vec3& b,
                                     double t0, double t1) {
    TrajectoryPiece piece;
    piece.knots = openUniformKnots(t0, t1, 1);
    piece.control_points.resize(4, 3);
    piece.control_points.row(0) = a.transpose();
    piece.control_points.row(1) = (a + (b - a) / 3.0).transpose();
    piece.control_points.row(2) = (a + 2.0 * (b - a) / 3.0).transpose();
    piece.control_points.row(3) = b.transpose();
    piece.valid_from = t0;
    piece.valid_until = t1;
    CommittedTrajectory trajectory;
    requireTrue(trajectory.replaceSuffix(t0, piece).accepted,
                "failed to construct synthetic trajectory");
    return trajectory;
}

void testExactCollisionChanges() {
    WorldSnapshot base;
    base.ego_suspended_geometry.enabled = true;
    CooperativeObstacleTrajectory peer;
    peer.name = "peer";
    peer.trajectory = linearTrajectory(Vec3(0, 2, 1), Vec3(1, 2, 1), 0, 4);
    base.cooperative_obstacles.push_back(peer);
    auto latest = base;
    latest.version = 99;
    latest.captured_at_s = .1;
    auto delta = latest.collisionChangesSince(base);
    requireTrue(!delta.full_check && !delta.ego_body_changed &&
                delta.cooperative_names.empty(), "metadata triggered collision rechecking");
    latest.ego_body_rotation = Eigen::AngleAxisd(.01, Vec3::UnitY()).toRotationMatrix();
    delta = latest.collisionChangesSince(base);
    requireTrue(!delta.full_check && delta.ego_body_changed && delta.cooperative_names.empty(),
                "rotation did not select body/workspace only");
    latest.cooperative_obstacles[0].tracking_error_half_extents.x() += 1e-12;
    delta = latest.collisionChangesSince(base);
    requireTrue(!delta.full_check && delta.ego_body_changed &&
                delta.cooperative_names == std::vector<std::string>{"peer"},
                "combined changes lost a tiny peer geometry change");
    latest = base;
    latest.cooperative_obstacles[0].trajectory = linearTrajectory(Vec3(0, 2, 1), Vec3(1, 2, 1), .01, 4.01);
    requireTrue(latest.collisionChangesSince(base).cooperative_names.size() == 1,
                "trajectory timing change was ignored");
    for (int kind = 0; kind < 5; ++kind) {
        latest = base;
        if (kind == 0) latest.ego_suspended_geometry.payload_attached = true;
        if (kind == 1) latest.ego_tracking_error_half_extents.x() += 1e-12;
        if (kind == 2) latest.ego_half_extents.z() += 1e-12;
        if (kind == 3) latest.workspace.enabled = true;
        if (kind == 4) latest.physical_static_obstacles.push_back({"new", axisAlignedBoxVertices(Vec3::Zero(), Vec3::Ones())});
        requireTrue(latest.collisionChangesSince(base).full_check,
                    "general geometry change incorrectly reused prior checks");
    }
    latest = base;
    latest.cooperative_obstacles.clear();
    requireTrue(latest.collisionChangesSince(base).full_check, "peer removal delta was ambiguous");
}

void testConvexCooperativeSupportsAndTemporalChildren() {
    WorldSnapshot world;
    world.ego_half_extents = Vec3(.105, .105, .1);
    world.ego_tracking_error_half_extents = Vec3(.1, .08, .05);
    world.ego_body_rotation = (Eigen::AngleAxisd(.6, Vec3::UnitZ()) *
        Eigen::AngleAxisd(-.17, Vec3::UnitY()) *
        Eigen::AngleAxisd(.13, Vec3::UnitX())).toRotationMatrix();
    auto& g = world.ego_suspended_geometry;
    g.enabled = g.payload_attached = true;
    g.swing_envelope = SwingEnvelope::OuterOctagon;
    CooperativeObstacleTrajectory peer;
    peer.name = "peer";
    peer.physical_half_extents = Vec3(.105, .105, .1);
    peer.tracking_error_half_extents = Vec3(.05, .04, .03);
    peer.trajectory = linearTrajectory(Vec3(.5, .6, 1.), Vec3(.8, .4, 1.05), 0., 1.);
    world.cooperative_obstacles.push_back(peer);
    const auto components = assemblyComponents(world.ego_half_extents, g, world.ego_body_rotation);
    const auto timed = world.timeIndexedObstacles(4, 0., 1.);
    requireTrue(timed.size() == components.size(), "convex mode changed component activation");
    const auto other = axisAlignedBoxVertices(Vec3::Zero(), peer.physical_half_extents);
    const Vec3 tracking = world.ego_tracking_error_half_extents + peer.tracking_error_half_extents;
    for (std::size_t i = 0; i < components.size(); ++i) {
        requireTrue(timed[i].name == "peer::body_vs_ego_" + components[i].name,
                    "convex mode changed relation identity");
        auto check = [&](const Eigen::MatrixXd& actual, double start, double end) {
            const auto center = convexCurveHullVertices(peer.trajectory, start, end);
            for (int k = 0; k < 70; ++k) {
                const Vec3 direction(std::cos(k*.713), std::sin(k*.713), std::sin(k*.137));
                const double expected = (center*direction).maxCoeff() + (other*direction).maxCoeff() +
                    (components[i].vertices*(-direction)).maxCoeff() + tracking.dot(direction.cwiseAbs());
                requireNear((actual*direction).maxCoeff(), expected, 2e-12,
                            "cooperative convex support was replaced by component AABB or lost tracking");
            }
        };
        for (int interval = 0; interval < 4; ++interval)
            check(timed[i].interval_vertices[interval], interval*.25, (interval+1)*.25);
        requireTrue(static_cast<bool>(timed[i].temporal_hull_builder), "convex temporal source missing");
        check(timed[i].temporal_hull_builder(.10, .70), .10, .70);
    }
    auto legacy = world;
    legacy.ego_suspended_geometry.swing_envelope = SwingEnvelope::LegacySquare;
    const auto legacy_components = assemblyComponents(legacy.ego_half_extents,
        legacy.ego_suspended_geometry, legacy.ego_body_rotation);
    const auto legacy_timed = legacy.timeIndexedObstacles(4, 0., 1.);
    std::size_t legacy_vertices = 0, convex_vertices = 0;
    for (std::size_t i = 0; i < legacy_components.size(); ++i) {
        Vec3 center, half;
        componentAabb(legacy_components[i], &center, &half);
        for (int interval = 0; interval < 4; ++interval) {
            Eigen::MatrixXd centres = convexCurveHullVertices(peer.trajectory, interval*.25, (interval+1)*.25);
            centres.rowwise() -= center.transpose();
            const auto expected = inflateVerticesByAabb(centres, peer.physical_half_extents + half +
                legacy.ego_tracking_error_half_extents + peer.tracking_error_half_extents);
            requireMatrixNear(legacy_timed[i].interval_vertices[interval], expected, 0.,
                              "legacy cooperative C-space changed");
            legacy_vertices += legacy_timed[i].interval_vertices[interval].rows();
            convex_vertices += timed[i].interval_vertices[interval].rows();
        }
    }
    std::cout << "cooperative fixture vertices legacy/convex: " << legacy_vertices
              << "/" << convex_vertices << '\n';
}

}  // namespace

int main() {
    try {
        testWorkspacePrismCompactionAndCollisionEquivalence();
        testIneligibleAndLegacyPrismsRetainExactGeometry();
        testExactCollisionChanges();
        testConvexCooperativeSupportsAndTemporalChildren();
        // N3's rotated corner cuts use the unchanged legacy suspended geometry.
        // Redundant Minkowski vertices must not multiply every separator LP.
        {
            WorldSnapshot legacy;
            legacy.ego_half_extents = Vec3(.105, .105, .1);
            legacy.ego_tracking_error_half_extents = Vec3(.1, .1, .05);
            auto& geometry = legacy.ego_suspended_geometry;
            geometry.enabled = geometry.payload_attached = true;
            geometry.cable_length_m = .49;
            geometry.max_swing_angle_rad = 10. * std::acos(-1.) / 180.;
            const auto rotation = Eigen::AngleAxisd(
                std::acos(-1.) / 4., Vec3::UnitZ()).toRotationMatrix();
            const Eigen::MatrixXd panel = axisAlignedBoxVertices(
                Vec3::Zero(), Vec3(.05, 4., .8)) * rotation.transpose();
            legacy.physical_static_obstacles.push_back({"corner", panel});
            const auto components = assemblyComponents(legacy.ego_half_extents, geometry);
            const auto timed_legacy = legacy.timeIndexedObstacles(4, 0., 8.);
            requireTrue(timed_legacy.size() == components.size(), "static component count changed");
            for (std::size_t i = 0; i < components.size(); ++i) {
                auto tracked = components[i];
                tracked.vertices = inflateVerticesByAabb(tracked.vertices,
                    legacy.ego_tracking_error_half_extents);
                const auto raw = physicalObstacleToConfigurationSpace(panel, tracked);
                const auto& compact = timed_legacy[i].interval_vertices.front();
                requireTrue(compact.rows() < raw.rows() / 2,
                            "legacy static C-space retains redundant Minkowski vertices");
                for (int direction = 0; direction < 180; ++direction) {
                    const Vec3 d(std::cos(direction * .713), std::sin(direction * .713),
                                 std::sin(direction * .137));
                    requireNear((compact * d).maxCoeff(), (raw * d).maxCoeff(), 1e-12,
                                "static compaction changed convex support/inflation");
                }
                for (const auto& interval : timed_legacy[i].interval_vertices)
                    requireMatrixNear(interval, compact, 0., "static interval geometry changed");
            }
        }
        WorldSnapshot snapshot;
        {
            VersionedWorld rotated_world;
            const Eigen::Quaterniond q(Eigen::AngleAxisd(.3, Vec3::UnitY()));
            rotated_world.setEgoBodyOrientation(q);
            const auto version = rotated_world.currentVersion();
            rotated_world.setEgoBodyOrientation(Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z()));
            requireTrue(rotated_world.currentVersion() == version,
                        "q and -q must not create different worlds");
            const auto state = rotated_world.snapshot(0.);
            requireTrue((state.ego_body_rotation - q.toRotationMatrix()).norm() < 1e-12,
                        "snapshot must preserve full measured body rotation");
            rotated_world.setEgoBodyOrientation(Eigen::Quaterniond::Identity());
            requireTrue(!rotated_world.runIfVersionCurrent(version, [] {}),
                        "changed attitude must invalidate obsolete geometry checks");
            const auto before_invalid = rotated_world.currentVersion();
            bool invalid_rejected = false;
            try { rotated_world.setEgoBodyOrientation(Eigen::Quaterniond(0., 0., 0., 0.)); }
            catch (const std::invalid_argument&) { invalid_rejected = true; }
            requireTrue(invalid_rejected && rotated_world.currentVersion() == before_invalid,
                        "invalid quaternion must not become an identity update");
        }
        snapshot.captured_at_s = 0.0;
        snapshot.ego_half_extents = Vec3::Constant(0.10);

        CooperativeObstacleTrajectory cooperative;
        cooperative.name = "carrier";
        cooperative.trajectory = linearTrajectory(
            Vec3(0.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0), 0.0, 1.0);
        cooperative.physical_half_extents = Vec3::Constant(0.10);
        cooperative.tracking_error_half_extents = Vec3::Constant(0.05);
        snapshot.cooperative_obstacles.push_back(cooperative);

        const auto timed = snapshot.timeIndexedObstacles(1, 0.0, 1.0);
        requireTrue(timed.size() == 1U, "cooperative obstacle missing from timed world");
        const Eigen::MatrixXd& vertices = timed.front().interval_vertices.front();
        requireNear(vertices.col(0).minCoeff(), -0.25, 1e-10,
                    "cooperative hull x minimum does not include body+tracking+ego tube");
        requireNear(vertices.col(0).maxCoeff(), 1.25, 1e-10,
                    "cooperative hull x maximum does not include body+tracking+ego tube");
        requireNear(vertices.col(1).minCoeff(), -0.25, 1e-10,
                    "cooperative hull y minimum incorrect");
        requireNear(vertices.col(1).maxCoeff(), 0.25, 1e-10,
                    "cooperative hull y maximum incorrect");

        VersionedWorld world(snapshot);
        requireTrue(world.currentVersion() == 0U, "initial version should be zero");
        bool action_ran = false;
        requireTrue(world.runIfVersionCurrent(0U, [&]() { action_ran = true; }),
                    "atomic gate rejected current version");
        requireTrue(action_ran, "atomic gate did not run action");

        cooperative.tracking_error_half_extents = Vec3::Constant(0.06);
        world.upsertCooperativeTrajectory(cooperative);
        requireTrue(world.currentVersion() == 1U, "world update did not advance version");
        action_ran = false;
        requireTrue(!world.runIfVersionCurrent(0U, [&]() { action_ran = true; }),
                    "atomic gate accepted stale version");
        requireTrue(!action_ran, "stale atomic gate ran commit action");

        const WorldSnapshot updated = world.snapshot(0.5);
        requireTrue(updated.version == 1U, "snapshot version mismatch");
        requireNear(updated.cooperative_obstacles.front().tracking_error_half_extents.x(),
                    0.06, 1e-12, "updated tracking tube not preserved");

        // A segmented moving fixture must enter the world as one coherent scene
        // update. Twenty-four per-segment version bumps would make every planner
        // snapshot stale before certification can complete.
        CooperativeObstacleTrajectory segment_a = cooperative;
        segment_a.name = "m2d_ring_segment_00";
        CooperativeObstacleTrajectory segment_b = cooperative;
        segment_b.name = "m2d_ring_segment_01";
        const auto before_batch = world.currentVersion();
        world.upsertCooperativeTrajectories({segment_a, segment_b});
        requireTrue(world.currentVersion() == before_batch + 1U,
                    "cooperative batch update must advance world version exactly once");
        const WorldSnapshot batched = world.snapshot(0.5);
        bool found_a = false;
        bool found_b = false;
        for (const auto& obstacle : batched.cooperative_obstacles) {
            found_a = found_a || obstacle.name == "m2d_ring_segment_00";
            found_b = found_b || obstacle.name == "m2d_ring_segment_01";
        }
        requireTrue(found_a && found_b, "cooperative batch update lost segmented obstacles");

        world.upsertCooperativeTrajectory(cooperative);
        world.upsertCooperativeTrajectories({segment_b, segment_a});
        requireTrue(world.currentVersion() == before_batch + 1U,
                    "identical single/batch updates advanced world version");

        // Test each collision-relevant field independently, including tiny changes
        // and disabled-geometry fields (no approximate equality or hidden state).
        const std::vector<std::function<void(CooperativeObstacleTrajectory&)>> changes = {
            [](auto& o) { o.trajectory = o.trajectory.shiftedToStartTime(0.0001); },
            [](auto& o) { o.trajectory = linearTrajectory(Vec3(1e-12,0,1), Vec3(1,0,1), 0,1); },
            [](auto& o) { o.physical_half_extents.x() += 1e-12; },
            [](auto& o) { o.tracking_error_half_extents.y() += 1e-12; },
            [](auto& o) { o.suspended_geometry.enabled = true; },
            [](auto& o) { o.suspended_geometry.cable_length_m += .01; },
            [](auto& o) { o.suspended_geometry.cable_radius_m += .001; },
            [](auto& o) { o.suspended_geometry.max_swing_angle_rad += .01; },
            [](auto& o) { o.suspended_geometry.magnet_half_extents.z() += .001; },
            [](auto& o) { o.suspended_geometry.magnet_center_below_cable_end_m += .001; },
            [](auto& o) { o.suspended_geometry.payload_attached = true; },
            [](auto& o) { o.suspended_geometry.payload_half_extents.x() += .001; },
            [](auto& o) { o.suspended_geometry.payload_center_from_magnet_center.x() += .001; },
            [](auto& o) { o.attached_tether_geometry.enabled = true; },
            [](auto& o) { o.attached_tether_geometry.anchor_from_body.x() += .001; },
            [](auto& o) { o.attached_tether_geometry.plate_from_body.x() += .001; },
            [](auto& o) { o.attached_tether_geometry.radius_m += .001; },
        };
        for (const auto& change : changes) {
            VersionedWorld isolated;
            isolated.upsertCooperativeTrajectory(cooperative);
            auto modified = cooperative;
            change(modified);
            isolated.upsertCooperativeTrajectories({modified, segment_a});
            requireTrue(isolated.currentVersion() == 2U, "changed batch must advance exactly once");
            isolated.upsertCooperativeTrajectory(modified);
            isolated.upsertCooperativeTrajectories({segment_a, modified});
            requireTrue(isolated.currentVersion() == 2U, "repeated changed obstacle not idempotent");
            isolated.upsertCooperativeTrajectory(cooperative);
            requireTrue(isolated.currentVersion() == 3U, "single update lost a real field change");
        }

        // C1F.4: ego closed-loop tracking allowance is part of the planned
        // cooperative collision geometry, not merely a runtime warning.
        world.setEgoTrackingErrorHalfExtents(Vec3::Constant(0.02));
        requireTrue(world.currentVersion() == 3U,
                    "ego tracking tube update did not advance world version");
        const WorldSnapshot tracked = world.snapshot(0.5);
        requireNear(tracked.ego_tracking_error_half_extents.x(), 0.02, 1e-12,
                    "ego tracking tube not preserved in snapshot");
        const auto tracked_timed = tracked.timeIndexedObstacles(1, 0.0, 1.0);
        const Eigen::MatrixXd& tracked_vertices =
            tracked_timed.front().interval_vertices.front();
        requireNear(tracked_vertices.col(0).minCoeff(), -0.28, 1e-10,
                    "ego tracking tube missing from cooperative x inflation");
        requireNear(tracked_vertices.col(0).maxCoeff(), 1.28, 1e-10,
                    "ego tracking tube missing from cooperative x inflation");

        std::cout << "test_world_snapshot: PASS\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "test_world_snapshot: FAIL: " << exc.what() << "\n";
        return 1;
    }
}
