#include "drone_visualisation/arm_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace drone_visualisation
{

ArmPanel::ArmPanel(QWidget* parent)
: rviz_common::Panel(parent), is_armed_(false), battery_voltage_(0.0f), status_message_("Waiting for controller...")
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
  
  setLayout(layout);

  // Create spacebar shortcut for DISARM
  space_shortcut_ = new QShortcut(QKeySequence(Qt::Key_Space), this);
  space_shortcut_->setContext(Qt::ApplicationShortcut);
  
  connect(space_shortcut_, &QShortcut::activated, this, &ArmPanel::onSpacePressed);
  connect(arm_button_, &QPushButton::clicked, this, &ArmPanel::onButtonPressed);
  connect(takeoff_button_, &QPushButton::clicked, this, &ArmPanel::onTakeoffPressed);
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
  } else {
    arm_button_->setText("ARM");
    arm_button_->setStyleSheet("background-color: #51cf66; color: white; font-weight: bold;");
    takeoff_button_->setEnabled(false);
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
