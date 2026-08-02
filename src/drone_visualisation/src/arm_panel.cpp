#include "drone_visualisation/arm_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <QHBoxLayout>

namespace drone_visualisation
{

ArmPanel::ArmPanel(QWidget* parent)
: rviz_common::Panel(parent), is_armed_(false), show_detach_(true), show_attach_(true), num_drones_(1), status_message_("Waiting for controller...")
{
  auto layout = new QVBoxLayout;

  // Fleet status label (the ARM/TAKEOFF/LAND command feedback line).
  status_label_ = new QLabel("Status: Waiting for controller...");
  status_label_->setStyleSheet("font-size: 12px; padding: 5px; background-color: #f0f0f0; border-radius: 3px;");
  status_label_->setFixedHeight(30);  // or setMinimumHeight(40);
  status_label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(status_label_);

  // Per-drone rows (armed state + battery), one per drone. Populated by
  // rebuild() once NumDrones is known from the rviz config -- the fleet arms
  // together, but a drone that fails to arm or drops arm mid-flight is only
  // visible if every drone is shown separately.
  drone_rows_layout_ = new QVBoxLayout;
  drone_rows_layout_->setContentsMargins(0, 0, 0, 0);
  layout->addLayout(drone_rows_layout_);

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

  // Subscriptions are per-drone and depend on NumDrones, which arrives via
  // load(). rviz may call load() before or after onInitialize(), so both call
  // rebuild() and it does the work once node_ is available.
  rebuild();

  RCLCPP_INFO(node_->get_logger(), "ArmPanel initialized (fleet mode -> /fleet/command)");
}

void ArmPanel::rebuild()
{
  // ── Qt rows ───────────────────────────────────────────────────────────────
  for (auto* label : drone_labels_) {
    drone_rows_layout_->removeWidget(label);
    delete label;
  }
  drone_labels_.clear();

  drone_armed_.assign(num_drones_, false);
  drone_voltage_.assign(num_drones_, 0.0f);
  drone_seen_.assign(num_drones_, false);
  drone_volt_seen_.assign(num_drones_, false);

  for (int i = 0; i < num_drones_; ++i) {
    auto* label = new QLabel;
    label->setFixedHeight(26);
    label->setAlignment(Qt::AlignCenter);
    drone_labels_.push_back(label);
    drone_rows_layout_->addWidget(label);
    updateDroneLabel(i);
  }

  // ── ROS subscriptions ─────────────────────────────────────────────────────
  // Dropping the old handles unsubscribes; recreate for the new fleet size.
  arming_state_subs_.clear();
  telemetry_subs_.clear();
  if (!node_) {
    return;   // onInitialize() will call us again once node_ exists
  }
  for (int i = 0; i < num_drones_; ++i) {
    const std::string ns = "/drone_" + std::to_string(i);
    arming_state_subs_.push_back(node_->create_subscription<std_msgs::msg::Bool>(
      ns + "/arming_state_feedback", 10,
      [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
        this->armingStateCallback(msg, i);
      }));
    // Battery voltage comes from that drone's elrs_interface telemetry.
    telemetry_subs_.push_back(node_->create_subscription<interfaces::msg::Telemetry>(
      ns + "/telemetry", 10,
      [this, i](const interfaces::msg::Telemetry::SharedPtr msg) {
        this->telemetryCallback(msg, i);
      }));
  }
}

void ArmPanel::updateDroneLabel(int i)
{
  if (i < 0 || i >= static_cast<int>(drone_labels_.size())) {
    return;
  }

  // Arming state and battery arrive from different launches, so each shows "--"
  // until its own source is up: arming from the controllers (terminal 2),
  // battery from elrs_interface (terminal 1).
  const QString armed_str = drone_seen_[i]
                              ? (drone_armed_[i] ? "ARMED" : "DISARMED")
                              : "--";
  const float v = drone_voltage_[i];
  const QString volt_str = drone_volt_seen_[i]
                             ? QString("%1 V").arg(v, 0, 'f', 2)
                             : QString("-- V");
  QString text = QString("D%1   %2   %3").arg(i).arg(armed_str).arg(volt_str);

  // Nothing heard from this drone at all -- neutral grey, not a battery colour.
  if (!drone_volt_seen_[i]) {
    drone_labels_[i]->setText(text);
    drone_labels_[i]->setStyleSheet(
      "font-size: 12px; padding: 3px; background-color: #f0f0f0; "
      "color: #868e96; border-radius: 3px;");
    return;
  }

  // Colour by battery (4S LiPo: 16.8 V full, 14.8 V nominal, 12.0 V empty), so
  // the weakest pack in the fleet is obvious at a glance.
  QString bg, fg = "white";
  if (v >= 15.6f)      { bg = "#51cf66"; }
  else if (v >= 14.4f) { bg = "#ffd43b"; fg = "#495057"; }
  else if (v >= 13.2f) { bg = "#ff922b"; }
  else                 { bg = "#ff6b6b"; }

  drone_labels_[i]->setText(text);
  drone_labels_[i]->setStyleSheet(
    QString("font-size: 12px; padding: 3px; background-color: %1; color: %2; "
            "border-radius: 3px; font-weight: bold;").arg(bg).arg(fg));
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

  // Fleet size for the per-drone rows. The launch files write this; absent or
  // nonsensical values fall back to a single drone.
  int n = 0;
  if (config.mapGetInt("NumDrones", &n) && n > 0) {
    num_drones_ = n;
  }
  rebuild();
}

void ArmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ShowDetach", show_detach_);
  config.mapSetValue("ShowAttach", show_attach_);
  config.mapSetValue("NumDrones", num_drones_);
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

void ArmPanel::armingStateCallback(const std_msgs::msg::Bool::SharedPtr msg, int i)
{
  if (i < 0 || i >= static_cast<int>(drone_armed_.size())) {
    return;
  }
  const bool was_seen = drone_seen_[i];
  const bool previous_drone_state = drone_armed_[i];
  drone_armed_[i] = msg->data;
  drone_seen_[i] = true;
  updateDroneLabel(i);

  if (was_seen && previous_drone_state == drone_armed_[i]) {
    return;
  }

  // Fleet-level state drives the buttons. ANY drone armed counts as armed, so
  // DISARM and LAND stay reachable when only part of the fleet is live.
  const bool previous_fleet = is_armed_;
  is_armed_ = false;
  int armed_count = 0;
  for (int k = 0; k < static_cast<int>(drone_armed_.size()); ++k) {
    if (drone_armed_[k]) { is_armed_ = true; ++armed_count; }
  }

  const int n = static_cast<int>(drone_armed_.size());
  if (armed_count == 0) {
    status_message_ = "Disarmed";
  } else if (armed_count == n) {
    status_message_ = "Armed - Ready for takeoff";
  } else {
    // Partial arm is the case the old drone-0-only panel could not show.
    status_message_ = "PARTIAL: " + std::to_string(armed_count) + "/" +
                      std::to_string(n) + " armed";
  }

  RCLCPP_INFO(node_->get_logger(), "drone %d %s by controller (%d/%d armed)",
              i, drone_armed_[i] ? "armed" : "disarmed", armed_count, n);

  if (previous_fleet != is_armed_) {
    updateButtonState();
  } else {
    updateStatusLabel();
  }
}

void ArmPanel::telemetryCallback(const interfaces::msg::Telemetry::SharedPtr msg, int i)
{
  if (i < 0 || i >= static_cast<int>(drone_voltage_.size())) {
    return;
  }
  drone_voltage_[i] = msg->battery_voltage;
  drone_volt_seen_[i] = true;
  updateDroneLabel(i);
}

}  // namespace drone_visualisation

PLUGINLIB_EXPORT_CLASS(drone_visualisation::ArmPanel, rviz_common::Panel)
