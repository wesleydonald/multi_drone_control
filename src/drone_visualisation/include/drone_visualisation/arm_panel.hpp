#ifndef DRONE_VISUALISATION__ARM_PANEL_HPP_
#define DRONE_VISUALISATION__ARM_PANEL_HPP_

#include <QVBoxLayout>
#include <QPushButton>
#include <QShortcut>
#include <QLabel>
#include <QSpinBox>
#include <QTimer>
#include <QWidget>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <interfaces/msg/telemetry.hpp>
#include <interfaces/msg/elrs_command.hpp>
#include <chrono>
#include <vector>

namespace drone_visualisation
{

class ArmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ArmPanel(QWidget* parent = nullptr);

  void onInitialize() override;
  // Persist/restore the ShowDetach/ShowAttach flags so a launch can hide those
  // rows (rviz config keys ShowDetach/ShowAttach, written by the launch files),
  // plus NumDrones, which sizes the per-drone status rows. Note load() and
  // onInitialize() can run in either order, so both call rebuild().
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onButtonPressed();
  void onSpacePressed();
  void onTakeoffPressed();
  void onLandPressed();
  void onDetachPressed();
  void onAttachPressed();
  void onDisarmRepeat();
  void refreshDroneLabels();

private:
  void updateButtonState();
  void updateStatusLabel();
  void applyShowDetach();
  void applyShowAttach();
  void applyShowMagnets();
  void onMagnetToggled(int i, bool on);
  void updateMagnetButton(int i);
  // (Re)build the per-drone rows and their subscriptions for num_drones_.
  // Safe to call repeatedly and before node_ exists.
  void rebuild();
  void updateDroneLabel(int i);
  void armingStateCallback(const std_msgs::msg::Bool::SharedPtr msg, int i);
  void telemetryCallback(const interfaces::msg::Telemetry::SharedPtr msg, int i);
  void elrsCallback(const interfaces::msg::ELRSCommand::SharedPtr msg, int i);
  void sendArm();
  void sendDisarm();
  bool managerRunning() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detach_pub_;
  // ATTACH: arms the approach drone's electromagnet (/magnet/command "ON"); it then welds
  // to the payload on contact and folds into the fleet.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr magnet_cmd_pub_;
  // One arming + telemetry + command subscription per drone, index-aligned with
  // drone_labels_ / drone_armed_ / drone_voltage_ / drone_thr_.
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> arming_state_subs_;
  std::vector<rclcpp::Subscription<interfaces::msg::Telemetry>::SharedPtr> telemetry_subs_;
  std::vector<rclcpp::Subscription<interfaces::msg::ELRSCommand>::SharedPtr> elrs_subs_;
  // The fleet manager's decisions and the planner's phase, both latched (the panel may
  // start after them).
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manager_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;

  QPushButton* arm_button_;
  QPushButton* takeoff_button_;
  QPushButton* land_button_;
  QPushButton* detach_button_;
  QSpinBox* detach_id_spin_;
  QWidget* detach_row_;
  QPushButton* attach_button_;
  QWidget* attach_row_;
  QShortcut* space_shortcut_;
  QLabel* status_label_;
  QLabel* phase_label_;
  // DISARM goes out DISARM_REPEATS times, DISARM_PERIOD_MS apart: one message can be lost.
  QTimer* disarm_timer_;
  int disarm_left_;
  // Throttle arrives at 50 Hz per drone; the rows are redrawn from this timer instead.
  QTimer* refresh_timer_;
  // Per-drone "D0  ARMED  24.10 V  thr 0.54" rows live here, one QLabel each.
  QVBoxLayout* drone_rows_layout_;
  std::vector<QLabel*> drone_labels_;
  // Per-drone tether magnet toggles (MAGNET Di ON/OFF), one checkable button each,
  // publishing String ON|OFF to /drone_<i>/magnet (elrs_interface latches the aux
  // channel). Shown when the launch sets ShowMagnets; locked while any drone is armed.
  QVBoxLayout* magnet_rows_layout_;
  std::vector<QPushButton*> magnet_buttons_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> magnet_pubs_;

  bool is_armed_;          // fleet-level: true if ANY drone reports armed
  int armed_count_;
  bool detach_used_;       // one DETACH per flight; cleared when the fleet disarms
  bool show_detach_;
  bool show_attach_;
  bool show_magnets_;
  bool magnet_initial_on_;
  int num_drones_;
  std::vector<bool> drone_armed_;
  std::vector<float> drone_voltage_;
  std::vector<float> drone_thr_;
  std::vector<bool> drone_thr_seen_;
  // wall time since which the drone has sat at the throttle cap (-1 = not at the cap)
  std::vector<double> drone_cap_since_;
  // Tracked separately: telemetry comes from elrs_interface (terminal 1) while
  // arming feedback comes from the controllers (terminal 2), so battery must be
  // able to display before any arming feedback exists.
  std::vector<bool> drone_seen_;        // first arming_state_feedback seen
  std::vector<bool> drone_volt_seen_;   // first telemetry seen
  std::string status_message_;
  bool status_alarm_;      // red status line (a refusal or a fault from the manager)
  std::string phase_message_;
};

}  // namespace drone_visualisation

#endif  // DRONE_VISUALISATION__ARM_PANEL_HPP_
