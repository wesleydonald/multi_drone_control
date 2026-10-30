#include "tejen_dynamic_planner/planner_completion_waitable.hpp"
#include "dynamic_planner/execution_authority.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
using tejen_dynamic_planner::PlannerCompletionWaitable;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class AdmissionWorld final : public dynamic_planner::WorldSnapshotSource {
public:
    std::uint64_t version = 17;
    dynamic_planner::WorldSnapshot snapshot(double time) const override {
        dynamic_planner::WorldSnapshot value;
        value.version = version;
        value.captured_at_s = time;
        return value;
    }
    std::uint64_t currentVersion() const override { return version; }
    bool runIfVersionCurrent(std::uint64_t expected,
                             const std::function<void()>& action) const override {
        if (expected != version) return false;
        action();
        return true;
    }
};
}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<rclcpp::Node>("planner_completion_regression");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        AdmissionWorld world;
        dynamic_planner::ReplanResult candidate;
        candidate.accepted = true;
        candidate.certified_world_version = 17;
        candidate.splice_time_s = .100;
        std::uint64_t epoch = 7;
        double now = .079;
        int installed = 0, processed = 0;
        std::string decision;
        std::future<int> ready;
        const auto executor_thread = std::this_thread::get_id();
        auto consume = [&] {
            require(std::this_thread::get_id() == executor_thread, "installed on worker thread");
            if (!ready.valid()) return;
            require(ready.wait_for(0s) == std::future_status::ready,
                    "notification preceded result availability");
            require(ready.get() == 42, "worker result changed");
            ++processed;
            decision = dynamic_planner::installCertifiedReplacement(
                world, candidate, 7, epoch, .078, now, 0., true,
                [&] { ++installed; });
        };
        auto wake = std::make_shared<PlannerCompletionWaitable>(
            node->get_node_base_interface()->get_context(), consume);
        node->get_node_waitables_interface()->add_waitable(wake, nullptr);
        auto submit = [&](bool delay = false) {
            auto task = std::make_shared<std::packaged_task<int()>>([] { return 42; });
            ready = task->get_future();
            return std::async(std::launch::async, [task, wake, delay] {
                if (delay) std::this_thread::sleep_for(10ms);
                (*task)();  // Publish the future BEFORE triggering the executor.
                require(wake->notify(), "completion notification failed");
            });
        };
        auto finish = [&] {
            auto worker = submit();
            worker.get();
        };
        auto collect = [&](int expected) {
            const auto deadline = std::chrono::steady_clock::now()+1s;
            while (processed < expected && std::chrono::steady_clock::now() < deadline)
                executor.spin_once(5ms);
            require(processed == expected, "completion lost without a reference timer tick");
        };

        // No reference timer exists: a result ready at78ms installs before100ms.
        executor.spin_some();  // Drain the executor's initial graph wake-up.
        auto first_worker = submit(true);
        const auto waiting = std::chrono::steady_clock::now();
        // Graph discovery can wake spin_once without executing a callback. Do
        // not confuse that unrelated wake with a missing worker notification.
        // A missing guard trigger still incurs the full1s wait and fails below.
        while (processed == 0 && std::chrono::steady_clock::now()-waiting < 500ms)
            executor.spin_once(1s);
        first_worker.get();
        require(processed == 1 && std::chrono::steady_clock::now()-waiting < 500ms,
                "worker failed to wake blocked executor promptly");
        collect(1);
        require(installed == 1 && decision == "install", "fresh completion not installed");
        require(wake->notify(), "duplicate notification failed");
        executor.spin_once(5ms);
        require(processed == 1 && installed == 1, "duplicate installation");

        // Model the queued attitude callback draining completion first, then
        // applying (not dropping/freezing) its new geometry. Pending wake is a no-op.
        finish();
        consume();
        ++world.version;
        executor.spin_once(5ms);
        require(processed == 2 && installed == 2 && world.version == 18,
                "ready drain lost result or suppressed measured geometry");

        finish();
        collect(3);
        require(decision == "world_version_changed" && installed == 2,
                "genuinely obsolete geometry was installed");
        world.version = 17;
        finish();
        epoch = 8;  // LAND/disable/fallback invalidation before callback delivery.
        collect(4);
        require(decision == "obsolete_authority_epoch" && installed == 2,
                "cancelled worker regained authority");
        epoch = 7;
        now = .100;
        finish();
        collect(5);
        require(decision == "obsolete_splice_discarded" && installed == 2,
                "deadline was relaxed to admit late completion");

        // A spurious event cannot install an already consumed result. Shutdown
        // stops notification; the worker/wake lifetime remains explicit.
        std::promise<void> release;
        auto released = release.get_future();
        auto last_task = std::make_shared<std::packaged_task<int()>>(
            [&] { released.wait(); return 42; });
        ready = last_task->get_future();
        auto last_worker = std::async(std::launch::async, [last_task, wake] {
            (*last_task)();
            require(!wake->notify(), "stopped completion channel accepted notification");
        });
        wake->stop();
        release.set_value();
        last_worker.get();  // Join the in-flight worker before releasing node/wake.
        require(ready.wait_for(0s) == std::future_status::ready && ready.get() == 42,
                "shutdown failed to retain/join worker outcome");
        executor.spin_once(5ms);
        require(processed == 5 && installed == 2, "shutdown executed completion");
        node->get_node_waitables_interface()->remove_waitable(wake, nullptr);
        executor.remove_node(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
