#ifndef DRONE_VISUALISATION__ARM_PANEL_HPP_
#define DRONE_VISUALISATION__ARM_PANEL_HPP_

#include <QVBoxLayout>
#include <QPushButton>
#include <QShortcut>
#include <QLabel>
#include <QSpinBox>
#include <QWidget>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <interfaces/srv/set_arming.hpp>
#include <interfaces/msg/telemetry.hpp>
#include <interfaces/srv/set_arming.hpp>
#include <chrono>

namespace drone_visualisation
{

class ArmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ArmPanel(QWidget* parent = nullptr);

  void onInitialize() override;
  // Persist/restore the ShowDetach flag so the launch can hide the detach row
  // (rviz config key ShowDetach, written by rviz_quad_load_launch.py detach:=).
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onButtonPressed();
  void onSpacePressed();
  void onTakeoffPressed();
  void onLandPressed();
  void onDetachPressed();

private:
  void updateButtonState();
  void updateStatusLabel();
  void applyShowDetach();
  void armingStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void telemetryCallback(const interfaces::msg::Telemetry::SharedPtr msg);
  void callArmingService(bool arm);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detach_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arming_state_sub_;
  rclcpp::Subscription<interfaces::msg::Telemetry>::SharedPtr telemetry_sub_;
  rclcpp::Client<interfaces::srv::SetArming>::SharedPtr arming_client_;

  QPushButton* arm_button_;
  QPushButton* takeoff_button_;
  QPushButton* land_button_;
  QPushButton* detach_button_;
  QSpinBox* detach_id_spin_;
  QWidget* detach_row_;
  QShortcut* space_shortcut_;
  QLabel* status_label_;
  QLabel* battery_label_;

  bool is_armed_;
  bool show_detach_;
  float battery_voltage_;
  std::string status_message_;
};

}  // namespace drone_visualisation

#endif  // DRONE_VISUALISATION__ARM_PANEL_HPP_
