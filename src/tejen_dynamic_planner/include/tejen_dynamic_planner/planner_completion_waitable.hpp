#pragma once

#include <rclcpp/exceptions.hpp>
#include <rclcpp/guard_condition.hpp>
#include <rclcpp/waitable.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <utility>

namespace tejen_dynamic_planner {

// Notification only: the worker publishes its future before notify(). The
// callback runs in the node's normal mutually-exclusive executor group; it
// never installs authority on the worker. Existing timer drains remain valid.
class PlannerCompletionWaitable final : public rclcpp::Waitable {
public:
    PlannerCompletionWaitable(rclcpp::Context::SharedPtr context,
                              std::function<void()> callback)
        : guard_(std::move(context)), callback_(std::move(callback)),
          token_(std::make_shared<int>(0)) {}

    bool notify() noexcept {
        if (stopped_.load()) return false;
        pending_.store(true);
        try {
            guard_.trigger();
            return true;
        } catch (const rclcpp::exceptions::RCLError&) {
            // Context shutdown can race worker completion. The future is still
            // ready for a normal drain/join; notification never owns the result.
            return false;
        }
    }

    void stop() noexcept {
        stopped_.store(true);
        pending_.store(false);
    }

    std::size_t get_number_of_ready_guard_conditions() override { return 1; }
    void add_to_wait_set(rcl_wait_set_t* wait_set) override {
        guard_.add_to_wait_set(wait_set);
    }
    bool is_ready(rcl_wait_set_t*) override {
        return !stopped_.load() && pending_.load();
    }
    std::shared_ptr<void> take_data() override {
        return pending_.exchange(false) ? token_ : nullptr;
    }
    void execute(std::shared_ptr<void>& data) override {
        if (data && !stopped_.load()) callback_();
    }

private:
    rclcpp::GuardCondition guard_;
    std::function<void()> callback_;
    const std::shared_ptr<void> token_;
    std::atomic<bool> pending_{false};
    std::atomic<bool> stopped_{false};
};

}  // namespace tejen_dynamic_planner
