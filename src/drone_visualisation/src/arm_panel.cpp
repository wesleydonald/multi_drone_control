#include "drone_visualisation/arm_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <QHBoxLayout>

namespace drone_visualisation
{

ArmPanel::ArmPanel(QWidget* parent)
: rviz_common::Panel(parent), is_armed_(false), show_detach_(true), show_attach_(true), battery_voltage_(0.0f), status_message_("Waiting for controller...")
{
  auto layout = new QVBoxLayout;
  
  // Status label
  status_label_ = new QLabel("Status: Waiting for controller...");
  status_label_->setStyleSheet("font-size: 12px; padding: 5px; background-color: #f0f0f0; border-radius: 3px;");
  status_label_->setFixedHeight(30);  // or setMinimumHeight(40);
  status_label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(status_label_);
  
  // Battery voltage label
  battery_label_ = new QLabel("Battery: -- V");
  battery_label_->setStyleSheet("font-size: 12px; padding: 5px; background-color: #f0f0f0; border-radius: 3px;");
  battery_label_->setFixedHeight(30);  // or setMinimumHeight(40);
  battery_label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(battery_label_);
  
  // ARM/DISARM button
  arm_button_ = new QPushButton("ARM");
  arm_button_->setStyleSheet("background-color: #51cf66; color: white; font-weight: bold;");
  arm_button_->setFixedHeight(80);      // or setMinimumHeight(40);
  layout->addWidget(arm_button_);
  
  // TAKEOFF button
  takeoff_button_ = new QPushButton("TAKEOFF");
  takeoff_button_->setStyleSheet("background-color: #4dabf7; color: white; font-weight: bold;");
  takeoff_button_->setEnabled(false);  // Disabled by default
  takeoff_button_->setFixedHeight(80);  // or setMinimumHeight(40);
  layout->addWidget(takeoff_button_);

  // LAND button. The planner descends the payload to touchdown and then
  // announces /fleet/landed, at which point the fleet manager disarms — so this
  // is the graceful counterpart to DISARM (which cuts thrust immediately).
  land_button_ = new QPushButton("LAND");
  land_button_->setStyleSheet("background-color: #f59f00; color: white; font-weight: bold;");
  land_button_->setEnabled(false);  // only meaningful once flying
  land_button_->setFixedHeight(80);
  layout->addWidget(land_button_);

  // DETACH row: pick a drone id and release it mid-flight. Publishes the physical
  // drone id to /fleet/detach, which the dissipative controller hands to the
  // spring-damper network (the remaining fleet re-settles with the load suspended).
  detach_row_ = new QWidget;
  auto detach_layout = new QHBoxLayout(detach_row_);
  detach_layout->setContentsMargins(0, 0, 0, 0);
  auto detach_label = new QLabel("drone");
  detach_id_spin_ = new QSpinBox;
  detach_id_spin_->setRange(0, 9);
  detach_id_spin_->setValue(1);
  detach_id_spin_->setFixedHeight(80);
  detach_button_ = new QPushButton("DETACH");
  detach_button_->setStyleSheet("background-color: #cc5de8; color: white; font-weight: bold;");
  detach_button_->setEnabled(false);  // only meaningful once flying
  detach_button_->setFixedHeight(80);
  detach_layout->addWidget(detach_label);
  detach_layout->addWidget(detach_id_spin_);
  detach_layout->addWidget(detach_button_, 1);
  layout->addWidget(detach_row_);

  // ATTACH row: arm the approach drone's electromagnet. Publishes "ON" to /magnet/command;
  // the magnet then welds to the payload on contact and the drone folds into the dissipative
  // network (via /magnet/object_attached). Shown when the launch sets attach:=true.
  attach_row_ = new QWidget;
  auto attach_layout = new QHBoxLayout(attach_row_);
  attach_layout->setContentsMargins(0, 0, 0, 0);
  attach_button_ = new QPushButton("ATTACH");
  attach_button_->setStyleSheet("background-color: #20c997; color: white; font-weight: bold;");
  attach_button_->setEnabled(false);  // only meaningful once flying
  attach_button_->setFixedHeight(80);
  attach_layout->addWidget(attach_button_, 1);
  layout->addWidget(attach_row_);

  setLayout(layout);

  // Create spacebar shortcut for DISARM
  space_shortcut_ = new QShortcut(QKeySequence(Qt::Key_Space), this);
  space_shortcut_->setContext(Qt::ApplicationShortcut);

  connect(space_shortcut_, &QShortcut::activated, this, &ArmPanel::onSpacePressed);
  connect(arm_button_, &QPushButton::clicked, this, &ArmPanel::onButtonPressed);
  connect(takeoff_button_, &QPushButton::clicked, this, &ArmPanel::onTakeoffPressed);
  connect(land_button_, &QPushButton::clicked, this, &ArmPanel::onLandPressed);
  connect(detach_button_, &QPushButton::clicked, this, &ArmPanel::onDetachPressed);
  connect(attach_button_, &QPushButton::clicked, this, &ArmPanel::onAttachPressed);
}

void ArmPanel::onInitialize()
{
  auto ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  node_ = ros_node->get_raw_node();
  // FLEET-DRIVEN: the buttons publish ARM/DISARM/TAKEOFF strings to the central
  // fleet manager (/fleet/command), which arms/disarms ALL drones together — the
  // same path as `ros2 topic pub /fleet/command ...`. (The old per-drone
  // `drone_arming_service` doesn't exist in the multi-drone stack.)
  command_pub_ = node_->create_publisher<std_msgs::msg::String>("/fleet/command", 10);
  // DETACH goes to the dissipative controller (physical drone id).
  detach_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/fleet/detach", 10);
  // ATTACH arms the approach drone's magnet (the magnet manager welds on contact).
  magnet_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/magnet/command", 10);

  // Reflect the fleet armed state from drone 0's feedback (drones arm together).
  arming_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/drone_0/arming_state_feedback", 10,
    std::bind(&ArmPanel::armingStateCallback, this, std::placeholders::_1));

  // Battery voltage from drone 0's telemetry (elrs_interface publishes per-drone).
  telemetry_sub_ = node_->create_subscription<interfaces::msg::Telemetry>(
    "/drone_0/telemetry", 10,
    std::bind(&ArmPanel::telemetryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "ArmPanel initialized (fleet mode -> /fleet/command)");
}

void ArmPanel::onButtonPressed()
{
  // Toggle arming state using service
  callArmingService(!is_armed_);
}

void ArmPanel::onSpacePressed()
{
  // Spacebar always disarms using service
  callArmingService(false);
}

void ArmPanel::onTakeoffPressed()
{
  if (command_pub_ && is_armed_) {
    std_msgs::msg::String msg;
    msg.data = "TAKEOFF";
    command_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "TAKEOFF command sent");
  } else if (!is_armed_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot takeoff - drone is not armed");
  }
}

void ArmPanel::onLandPressed()
{
  if (command_pub_ && is_armed_) {
    std_msgs::msg::String msg;
    msg.data = "LAND";
    command_pub_->publish(msg);
    status_message_ = "Sent LAND to fleet";
    updateStatusLabel();
    RCLCPP_INFO(node_->get_logger(), "LAND command sent");
  } else if (!is_armed_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot land - drone is not armed");
  }
}

void ArmPanel::onDetachPressed()
{
  if (!is_armed_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot detach - fleet is not armed/flying");
    return;
  }
  if (!detach_pub_ || detach_pub_->get_subscription_count() == 0) {
    status_message_ = "No /fleet/detach subscriber (dissipative not running)";
    updateStatusLabel();
    RCLCPP_WARN(node_->get_logger(),
                "No subscriber on /fleet/detach — the dissipative controller may not be running");
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = detach_id_spin_->value();
  detach_pub_->publish(msg);
  status_message_ = "Detached drone " + std::to_string(msg.data);
  updateStatusLabel();
  RCLCPP_INFO(node_->get_logger(), "DETACH command sent for drone %d", msg.data);
  // advance to the next drone id for a quick 4->3->2 sequence.
  if (detach_id_spin_->value() < detach_id_spin_->maximum()) {
    detach_id_spin_->setValue(detach_id_spin_->value() + 1);
  }
}

void ArmPanel::onAttachPressed()
{
  if (!is_armed_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot attach - fleet is not armed/flying");
    return;
  }
  if (!magnet_cmd_pub_ || magnet_cmd_pub_->get_subscription_count() == 0) {
    status_message_ = "No /magnet/command subscriber (magnet manager not running)";
    updateStatusLabel();
    RCLCPP_WARN(node_->get_logger(),
                "No subscriber on /magnet/command — the magnet manager may not be running");
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "ON";
  magnet_cmd_pub_->publish(msg);
  status_message_ = "Magnet ON — approaching & attaching to payload";
  updateStatusLabel();
  RCLCPP_INFO(node_->get_logger(), "ATTACH: magnet armed (/magnet/command ON)");
}

void ArmPanel::applyShowDetach()
{
  if (detach_row_) {
    detach_row_->setVisible(show_detach_);
  }
}

void ArmPanel::applyShowAttach()
{
  if (attach_row_) {
    attach_row_->setVisible(show_attach_);
  }
}

void ArmPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  bool show = true;
  if (config.mapGetBool("ShowDetach", &show)) {
    show_detach_ = show;
  }
  applyShowDetach();
  bool show_a = true;
  if (config.mapGetBool("ShowAttach", &show_a)) {
    show_attach_ = show_a;
  }
  applyShowAttach();
}

void ArmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ShowDetach", show_detach_);
  config.mapSetValue("ShowAttach", show_attach_);
}

void ArmPanel::callArmingService(bool arm)
{
  // FLEET COMMAND: publish ARM/DISARM to /fleet/command (the fleet manager arms
  // every drone). No subscriber => the fleet manager (terminal 2) isn't running.
  if (!command_pub_ || command_pub_->get_subscription_count() == 0) {
    status_message_ = "Fleet manager not running";
    updateStatusLabel();
    RCLCPP_WARN(node_->get_logger(),
                "No subscriber on /fleet/command — fleet manager may not be running");
    return;
  }

  std_msgs::msg::String msg;
  msg.data = arm ? "ARM" : "DISARM";
  command_pub_->publish(msg);

  status_message_ = arm ? "Sent ARM to fleet" : "Sent DISARM to fleet";
  updateStatusLabel();
  RCLCPP_INFO(node_->get_logger(), "Fleet command sent: %s", msg.data.c_str());
}

void ArmPanel::updateButtonState()
{
  if (is_armed_) {
    arm_button_->setText("DISARM");
    arm_button_->setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold;");
    takeoff_button_->setEnabled(true);
    land_button_->setEnabled(true);
    detach_button_->setEnabled(true);
    attach_button_->setEnabled(true);
  } else {
    arm_button_->setText("ARM");
    arm_button_->setStyleSheet("background-color: #51cf66; color: white; font-weight: bold;");
    takeoff_button_->setEnabled(false);
    land_button_->setEnabled(false);
    detach_button_->setEnabled(false);
    attach_button_->setEnabled(false);
  }
  updateStatusLabel();
}

void ArmPanel::updateStatusLabel()
{
  QString display_text = QString::fromStdString(status_message_);
  
  if (is_armed_) {
    status_label_->setText(display_text);
    status_label_->setStyleSheet("font-size: 12px; font-weight: bold; padding: 5px; background-color: #ffe066; border-radius: 3px; color: #c92a2a;");
  } else {
    status_label_->setText(display_text);
    status_label_->setStyleSheet("font-size: 12px; font-weight: bold; padding: 5px; background-color: #f0f0f0; border-radius: 3px; color: #495057;");
  }
}

void ArmPanel::armingStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Update internal state based on feedback from controller
  bool previous_state = is_armed_;
  is_armed_ = msg->data;
  
  if (previous_state != is_armed_) {
    if (is_armed_) {
      status_message_ = "Armed - Ready for takeoff";
      RCLCPP_INFO(node_->get_logger(), "Drone armed by controller");
    } else {
      status_message_ = "Disarmed";
      RCLCPP_INFO(node_->get_logger(), "Drone disarmed by controller");
    }
    updateButtonState();
  }
}

void ArmPanel::telemetryCallback(const interfaces::msg::Telemetry::SharedPtr msg)
{
  battery_voltage_ = msg->battery_voltage;
  
  // Update battery label with voltage
  QString battery_text = QString("Battery: %1 V").arg(battery_voltage_, 0, 'f', 2);
  
  // Color code based on voltage (typical LiPo: 4.2V max, 3.0V min per cell)
  // Assuming 4S battery: 16.8V full, 14.8V nominal, 12.0V empty
  QString style;
  if (battery_voltage_ >= 15.6) {
    // Green - Good
    style = "font-size: 12px; padding: 5px; background-color: #51cf66; color: white; border-radius: 3px; font-weight: bold;";
  } else if (battery_voltage_ >= 14.4) {
    // Yellow - Medium
    style = "font-size: 12px; padding: 5px; background-color: #ffd43b; color: #495057; border-radius: 3px; font-weight: bold;";
  } else if (battery_voltage_ >= 13.2) {
    // Orange - Low
    style = "font-size: 12px; padding: 5px; background-color: #ff922b; color: white; border-radius: 3px; font-weight: bold;";
  } else {
    // Red - Critical
    style = "font-size: 12px; padding: 5px; background-color: #ff6b6b; color: white; border-radius: 3px; font-weight: bold;";
  }
  
  battery_label_->setText(battery_text);
  battery_label_->setStyleSheet(style);
}

}  // namespace drone_visualisation

PLUGINLIB_EXPORT_CLASS(drone_visualisation::ArmPanel, rviz_common::Panel)
