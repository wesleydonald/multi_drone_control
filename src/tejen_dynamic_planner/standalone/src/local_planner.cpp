#include "dynamic_planner/local_planner.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace dynamic_planner {

LocalPlanner::LocalPlanner(LocalPlannerConfig config) : config_(std::move(config)) {
    if (config_.num_segments != 4) {
        throw std::invalid_argument("R6.3B.1 intentionally keeps the validated four-segment local planner");
    }
}

LocalPlanResult LocalPlanner::plan(const LocalPlanRequest& request) const {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();

    LocalPlanResult output;
    output.requested_terminal_boundary = request.terminal_boundary;
    if (!request.initial_state.position.allFinite() ||
        !request.initial_state.velocity.allFinite() ||
        !request.initial_state.acceleration.allFinite() ||
        !request.local_goal.allFinite() ||
        !std::isfinite(request.t_start) ||
        !std::isfinite(request.duration_s) || !(request.duration_s > 0.0)) {
        output.status = "INVALID_REQUEST";
        output.message = "non-finite local-plan request or non-positive duration";
        return output;
    }

    output.knots = openUniformKnots(
        request.t_start, request.t_start + request.duration_s, config_.num_segments);
    const ControlPoints q012 = initialControlPointsFromState(request.initial_state, output.knots);

    try {
        const TerminalBoundary search_boundary = request.terminal_boundary;
        // C1F.8d: Octopus now carries the same terminal contract as refinement.
        // In Continuation mode it searches q_(N-1) explicitly and appends only
        // the fixed endpoint q_N, leaving terminal velocity genuinely free.
        const Vec3 search_goal = search_boundary.mode == TerminalMode::MovingRendezvous
            ? terminalSearchPrecursor(
                output.knots, config_.num_segments + kCubicDegree, search_boundary)
            : request.local_goal;
        OctopusSearch search(
            output.knots, q012, search_goal, request.obstacles,
            request.terminal_hold_obstacles, search_boundary, config_.octopus,
            request.octopus_stop_requested, request.octopus_stop_reason);
        output.search = search.search();
    } catch (const std::exception& exc) {
        output.status = "SEARCH_EXCEPTION";
        output.message = exc.what();
        output.total_solve_time_s = std::chrono::duration<double>(Clock::now() - started).count();
        return output;
    }

    if (!output.search.success || !output.search.control_points.has_value()) {
        output.status = output.search.status.empty() ? "SEARCH_FAILED" : output.search.status;
        output.message = "Octopus Search did not return a valid local trajectory";
        output.total_solve_time_s = std::chrono::duration<double>(Clock::now() - started).count();
        return output;
    }

    output.partial = output.search.status.find("PADDED_CLOSEST_PARTIAL") != std::string::npos;
    // A natural timeout can select the unextended q2 seed, including a moving
    // splice prefix. Optimizing its own endpoint rewards stopping indefinitely.
    // Use explicit search provenance, not a numerical stationarity threshold.
    // Originally non-stopped searched evasive histories and callback stops keep
    // their endpoint; ordinary stopped requests keep the nominal soft objective.
    const bool seed_timeout = output.partial &&
        output.search.termination_reason == "RUNTIME_REACHED" &&
        output.search.partial_fallback_selected_index == 2;
    const Vec3 selected_endpoint = output.search.control_points->bottomRows(1).transpose();
    output.effective_terminal_boundary = request.terminal_boundary;
    if (output.partial) {
        output.effective_terminal_boundary.mode = TerminalMode::Stopped;
        output.effective_terminal_boundary.position =
            output.search.control_points->row(output.search.control_points->rows() - 1).transpose();
        if (seed_timeout) {
            // Stopped fixes terminal derivatives, not terminal position.
            output.effective_terminal_boundary.position = request.local_goal;
        }
        output.effective_terminal_boundary.velocity.setZero();
    }

    std::string seed_goal_failure;
    try {
        // MADER/PB.71: an ordinary stopped partial is an initial guess and
        // corridor, not a replacement destination. Keep the nominal soft goal.
        // Originally non-stopped requests retain their evasive endpoint and
        // the existing unextended-timeout fallback semantics above/below.
        const Vec3 refinement_goal = output.partial &&
            request.terminal_boundary.mode != TerminalMode::Stopped
            ? output.effective_terminal_boundary.position
            : request.local_goal;
        try {
            output.refinement = refineOctopusTrajectory(
                output.search, output.knots, refinement_goal, request.obstacles,
                config_.refinement, output.effective_terminal_boundary);
        } catch (const std::exception& exc) {
            if (!seed_timeout) throw;
            seed_goal_failure = std::string("EXCEPTION: ") + exc.what();
        }
        if (seed_timeout && (!output.refinement || !output.refinement->success ||
                             !output.refinement->control_points)) {
            if (seed_goal_failure.empty()) seed_goal_failure = output.refinement
                ? output.refinement->status : "NO_REFINEMENT_RESULT";
            // One bounded retry of the already-certified stopped endpoint. It
            // still goes through all refinement constraints and outer checking;
            // never publish unchecked raw search points after QP failure.
            output.effective_terminal_boundary.position = selected_endpoint;
            output.refinement = refineOctopusTrajectory(
                output.search, output.knots, selected_endpoint, request.obstacles,
                config_.refinement, output.effective_terminal_boundary);
        }
    } catch (const std::exception& exc) {
        output.status = "REFINEMENT_EXCEPTION";
        output.message = exc.what();
        output.total_solve_time_s = std::chrono::duration<double>(Clock::now() - started).count();
        return output;
    }

    if (!output.refinement->success || !output.refinement->control_points.has_value()) {
        output.status = output.refinement->status.empty() ? "REFINEMENT_FAILED" : output.refinement->status;
        output.message = output.refinement->message;
        output.total_solve_time_s = std::chrono::duration<double>(Clock::now() - started).count();
        return output;
    }

    output.success = true;
    output.status = "SUCCESS";
    output.message = output.refinement->message;
    if (seed_timeout) {
        output.message += seed_goal_failure.empty()
            ? "; unextended_seed_soft_goal=true"
            : "; unextended_seed_endpoint_retry=" + seed_goal_failure;
    }
    output.control_points = output.refinement->control_points;
    output.total_solve_time_s = std::chrono::duration<double>(Clock::now() - started).count();
    return output;
}

}  // namespace dynamic_planner
