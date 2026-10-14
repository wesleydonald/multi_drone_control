#include "drone_visualisation/arm_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_visualisation
{

namespace
{
// DISARM is the rig kill switch: send it several times, a single message can be lost.
constexpr int DISARM_REPEATS = 5;
constexpr int DISARM_PERIOD_MS = 100;
// A drone pinned at its tracker's throttle cap cannot hold its share of the load. The cap
// comes from each tracker (/drone_i/throttle_max, latched); 0.6 until one is heard.
constexpr float THR_CAP_DEFAULT = 0.6f;
constexpr float THR_CAP_MARGIN = 0.01f;
constexpr double THR_CAP_HOLD_S = 2.0;
// "In the air": above the lowest height seen; a pose older than the rig's pose timeout is not used.
constexpr double AIRBORNE_M = 0.05;
constexpr double POSE_STALE_S = 0.25;
const char* const HIS_MPC = "/tejen/tejen_mpc";
const char* const HIS_PLANNER = "/online_join_planner";

double wallNow()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Manager messages that mean something went wrong get the red status line.
bool isAlarm(const std::string& s)
{
  for (const char* key : {"REFUSED", "FAILED", "EMERGENCY", "disarmed before", "disarmed in flight",
                          "Cannot"}) {
    if (s.find(key) != std::string::npos) {
      return true;
    }
  }
  return false;
}

// Manager messages after which the fleet is no longer armed for TAKEOFF.
bool endsArm(const std::string& s)
{
  for (const char* key : {"DISARM", "EMERGENCY STOP", "ARM FAILED", "ARM REFUSED", "TAKEOFF REFUSED",
                          "Disarm complete"}) {
    if (s.find(key) != std::string::npos) {
      return true;
    }
  }
  return false;
}

QPushButton* flightButton(const char* text, const char* colour)
{
  auto b = new QPushButton(text);
  b->setStyleSheet(QString("QPushButton { background-color: %1; color: white; font-weight: bold; }"
                           "QPushButton:disabled { background-color: #e9ecef; color: #868e96; }").arg(colour));
  b->setFixedHeight(60);
  b->setEnabled(false);
  b->setFocusPolicy(Qt::NoFocus);   // SPACE stays DISARM whatever was clicked last
  return b;
}
}  // namespace

ArmPanel::ArmPanel(QWidget* parent)
: rviz_common::Panel(parent), disarm_left_(0), is_armed_(false), armed_count_(0), detach_used_(false),
  show_detach_(true), show_attach_(true), show_magnets_(false), magnet_initial_on_(true), num_drones_(1),
  status_message_("Waiting for the fleet manager..."), status_alarm_(false), arm_complete_(false),
  partner_present_(false), his_armed_(false), his_link_ok_(false), give_up_subs_(0)
{
  auto layout = new QVBoxLayout;

  // Fleet status: the fleet manager's latest decision (ARM complete, refusals, faults) or the
  // panel's own command feedback, red when something went wrong.
  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  status_label_->setMinimumHeight(30);
  status_label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(status_label_);

  // Planner phase (waiting / creep / settle / lifting / holding / landing).
  phase_label_ = new QLabel("Phase: --");
  phase_label_->setAlignment(Qt::AlignCenter);
  phase_label_->setStyleSheet("font-size: 12px; padding: 3px; color: #495057;");
  layout->addWidget(phase_label_);

  air_label_ = new QLabel("In the air: --");
  air_label_->setAlignment(Qt::AlignCenter);
  air_label_->setStyleSheet("font-size: 12px; padding: 3px; color: #495057;");
  layout->addWidget(air_label_);

  // Per-drone rows (armed state, battery, throttle), one per drone. Populated by
  // rebuild() once NumDrones is known from the rviz config -- the fleet arms
  // together, but a drone that fails to arm or drops arm mid-flight is only
  // visible if every drone is shown separately.
  drone_rows_layout_ = new QVBoxLayout;
  drone_rows_layout_->setContentsMargins(0, 0, 0, 0);
  layout->addLayout(drone_rows_layout_);

  // Per-drone tether magnet toggles, stacked. Populated by rebuild(); hidden unless
  // the launch sets ShowMagnets (both launches; rig: elrs_interface, sim: sim_magnet).
  magnet_rows_layout_ = new QVBoxLayout;
  magnet_rows_layout_->setContentsMargins(0, 0, 0, 0);
  layout->addLayout(magnet_rows_layout_);

  // ARM/DISARM button
  arm_button_ = new QPushButton("ARM");
  arm_button_->setStyleSheet("background-color: #51cf66; color: white; font-weight: bold;");
  arm_button_->setFixedHeight(80);
  layout->addWidget(arm_button_);

  // TAKEOFF button: enabled only when every drone reports armed.
  takeoff_button_ = new QPushButton("TAKEOFF");
  takeoff_button_->setStyleSheet("background-color: #4dabf7; color: white; font-weight: bold;");
  takeoff_button_->setEnabled(false);
  takeoff_button_->setFixedHeight(80);
  layout->addWidget(takeoff_button_);

  // LAND button. The planner descends the payload to touchdown and then
  // announces /fleet/landed, at which point the fleet manager disarms — so this
  // is the graceful counterpart to DISARM (which cuts thrust immediately).
  land_button_ = new QPushButton("LAND");
  land_button_->setStyleSheet("background-color: #f59f00; color: white; font-weight: bold;");
  land_button_->setEnabled(false);
  land_button_->setFixedHeight(80);
  layout->addWidget(land_button_);

  // DETACH row: pick a drone id and release it mid-flight (/fleet/detach). One DETACH per
  // flight: the button locks after it until the fleet disarms.
  detach_row_ = new QWidget;
  auto detach_layout = new QHBoxLayout(detach_row_);
  detach_layout->setContentsMargins(0, 0, 0, 0);
  auto detach_label = new QLabel("drone");
  detach_id_spin_ = new QSpinBox;
  detach_id_spin_->setRange(0, 9);
  detach_id_spin_->setValue(3);
  detach_id_spin_->setFixedHeight(80);
  detach_id_spin_->setFocusPolicy(Qt::ClickFocus);   // keeps SPACE for DISARM
  detach_button_ = new QPushButton("DETACH");
  detach_button_->setStyleSheet("background-color: #cc5de8; color: white; font-weight: bold;");
  detach_button_->setEnabled(false);
  detach_button_->setFixedHeight(80);
  detach_layout->addWidget(detach_label);
  detach_layout->addWidget(detach_id_spin_);
  detach_layout->addWidget(detach_button_, 1);
  layout->addWidget(detach_row_);

  // ATTACH row: arm the approach drone's electromagnet. Publishes "ON" to /magnet/command;
  // the magnet then welds to the payload on contact and the drone folds into the fleet
  // (via /magnet/object_attached). Shown when the launch sets attach:=true.
  attach_row_ = new QWidget;
  auto attach_layout = new QHBoxLayout(attach_row_);
  attach_layout->setContentsMargins(0, 0, 0, 0);
  attach_button_ = new QPushButton("ATTACH");
  attach_button_->setStyleSheet("background-color: #20c997; color: white; font-weight: bold;");
  attach_button_->setEnabled(false);
  attach_button_->setFixedHeight(80);
  attach_layout->addWidget(attach_button_, 1);
  layout->addWidget(attach_row_);

  // Tejen's stack, shown while his MPC is in the graph. Flight order: his ARM, ARM, TAKEOFF, his TAKEOFF.
  partner_row_ = new QWidget;
  auto partner_layout = new QHBoxLayout(partner_row_);
  partner_layout->setContentsMargins(0, 0, 0, 0);
  his_arm_button_ = flightButton("his ARM", "#7048e8");
  his_takeoff_button_ = flightButton("his TAKEOFF", "#7048e8");
  give_up_button_ = flightButton("GIVE UP", "#495057");
  his_disarm_button_ = flightButton("his DISARM", "#7048e8");
  for (auto* b : {his_arm_button_, his_takeoff_button_, give_up_button_, his_disarm_button_}) {
    partner_layout->addWidget(b, 1);
  }
  partner_row_->setVisible(false);
  layout->addWidget(partner_row_);
  for (auto* b : {arm_button_, takeoff_button_, land_button_, detach_button_, attach_button_}) {
    b->setFocusPolicy(Qt::NoFocus);
  }

  setLayout(layout);

  // Spacebar = DISARM, from anywhere in RViz.
  space_shortcut_ = new QShortcut(QKeySequence(Qt::Key_Space), this);
  space_shortcut_->setContext(Qt::ApplicationShortcut);

  disarm_timer_ = new QTimer(this);
  disarm_timer_->setInterval(DISARM_PERIOD_MS);
  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(250);
  refresh_timer_->start();
  graph_timer_ = new QTimer(this);
  graph_timer_->setInterval(1000);
  graph_timer_->start();

  connect(space_shortcut_, &QShortcut::activated, this, &ArmPanel::onSpacePressed);
  connect(arm_button_, &QPushButton::clicked, this, &ArmPanel::onButtonPressed);
  connect(takeoff_button_, &QPushButton::clicked, this, &ArmPanel::onTakeoffPressed);
  connect(land_button_, &QPushButton::clicked, this, &ArmPanel::onLandPressed);
  connect(detach_button_, &QPushButton::clicked, this, &ArmPanel::onDetachPressed);
  connect(attach_button_, &QPushButton::clicked, this, &ArmPanel::onAttachPressed);
  connect(disarm_timer_, &QTimer::timeout, this, &ArmPanel::onDisarmRepeat);
  connect(refresh_timer_, &QTimer::timeout, this, &ArmPanel::refreshDroneLabels);
  connect(graph_timer_, &QTimer::timeout, this, &ArmPanel::refreshGraph);
  connect(his_arm_button_, &QPushButton::clicked, this, &ArmPanel::onHisArmPressed);
  connect(his_takeoff_button_, &QPushButton::clicked, this, &ArmPanel::onHisTakeoffPressed);
  connect(give_up_button_, &QPushButton::clicked, this, &ArmPanel::onGiveUpPressed);
  connect(his_disarm_button_, &QPushButton::clicked, this, &ArmPanel::onHisDisarmPressed);
  updateStatusLabel();
}

void ArmPanel::onInitialize()
{
  auto ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  node_ = ros_node->get_raw_node();
  // The buttons publish ARM/DISARM/TAKEOFF/LAND to the fleet manager (/fleet/command),
  // the same path as `ros2 topic pub /fleet/command ...`.
  command_pub_ = node_->create_publisher<std_msgs::msg::String>("/fleet/command", 10);
  detach_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/fleet/detach", 10);
  magnet_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/magnet/command", 10);

  // Latched: the manager and planner may have spoken before RViz started.
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  manager_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/fleet/manager_status", latched,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      status_message_ = msg->data;
      status_alarm_ = isAlarm(msg->data);
      if (msg->data.find("ARM sequence complete") != std::string::npos) {
        arm_complete_ = true;
      } else if (endsArm(msg->data)) {
        arm_complete_ = false;
      }
      updateButtonState();
    });
  phase_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/fleet/phase", latched,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      phase_message_ = msg->data;
      phase_label_->setText(QString("Phase: %1").arg(QString::fromStdString(msg->data)));
    });
  his_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/tejen/drone_command", 10);
  give_up_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/join_planner/give_up", 10);
  his_armed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/tejen/drone_arming_state_feedback", 5,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (his_armed_ != msg->data) {
        his_armed_ = msg->data;
        updateButtonState();
      }
    });
  summary_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/fleet/mocap_summary", 2,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto bodies = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data))
                            .object().value("bodies").toObject();
      for (int i = 0; i < static_cast<int>(drone_z_.size()); ++i) {
        const auto b = bodies.value(QString("drone_%1").arg(i)).toObject();
        const auto z = b.value("z");
        const auto age = b.value("age_s");
        if (!z.isDouble() || !age.isDouble() || age.toDouble() > POSE_STALE_S) {
          drone_z_[i] = std::numeric_limits<double>::quiet_NaN();
          continue;
        }
        drone_z_[i] = z.toDouble();
        if (std::isnan(drone_rest_z_[i]) || drone_z_[i] < drone_rest_z_[i]) {
          drone_rest_z_[i] = drone_z_[i];
        }
      }
    });

  // Subscriptions are per-drone and depend on NumDrones, which arrives via
  // load(). rviz may call load() before or after onInitialize(), so both call
  // rebuild() and it does the work once node_ is available.
  rebuild();
}

void ArmPanel::rebuild()
{
  // ── Qt rows ───────────────────────────────────────────────────────────────
  for (auto* label : drone_labels_) {
    drone_rows_layout_->removeWidget(label);
    delete label;
  }
  drone_labels_.clear();
  for (auto* btn : magnet_buttons_) {
    magnet_rows_layout_->removeWidget(btn);
    delete btn;
  }
  magnet_buttons_.clear();

  drone_armed_.assign(num_drones_, false);
  drone_voltage_.assign(num_drones_, 0.0f);
  drone_thr_.assign(num_drones_, 0.0f);
  drone_thr_seen_.assign(num_drones_, false);
  drone_cap_since_.assign(num_drones_, -1.0);
  drone_cap_.assign(num_drones_, THR_CAP_DEFAULT);
  drone_seen_.assign(num_drones_, false);
  drone_volt_seen_.assign(num_drones_, false);
  drone_z_.assign(num_drones_, std::numeric_limits<double>::quiet_NaN());
  drone_rest_z_.assign(num_drones_, std::numeric_limits<double>::quiet_NaN());

  for (int i = 0; i < num_drones_; ++i) {
    auto* label = new QLabel;
    label->setFixedHeight(26);
    label->setAlignment(Qt::AlignCenter);
    drone_labels_.push_back(label);
    drone_rows_layout_->addWidget(label);
    updateDroneLabel(i);
  }
  for (int i = 0; i < num_drones_; ++i) {
    auto* btn = new QPushButton;
    btn->setCheckable(true);
    btn->setChecked(magnet_initial_on_);
    btn->setFixedHeight(36);
    btn->setVisible(show_magnets_ && !magnetHidden(i));
    btn->setEnabled(!is_armed_);
    connect(btn, &QPushButton::toggled, this, [this, i](bool on) { this->onMagnetToggled(i, on); });
    magnet_buttons_.push_back(btn);
    magnet_rows_layout_->addWidget(btn);
    updateMagnetButton(i);
  }

  // ── ROS subscriptions ─────────────────────────────────────────────────────
  // Dropping the old handles unsubscribes; recreate for the new fleet size.
  arming_state_subs_.clear();
  telemetry_subs_.clear();
  elrs_subs_.clear();
  cap_subs_.clear();
  magnet_pubs_.clear();
  if (!node_) {
    return;   // onInitialize() will call us again once node_ exists
  }
  for (int i = 0; i < num_drones_; ++i) {
    const std::string ns = "/drone_" + std::to_string(i);
    magnet_pubs_.push_back(magnetHidden(i) ? nullptr
                           : node_->create_publisher<std_msgs::msg::String>(ns + "/magnet", 10));
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
    // The command the radio actually sends (the mux output where there is one).
    elrs_subs_.push_back(node_->create_subscription<interfaces::msg::ELRSCommand>(
      ns + "/ELRSCommand", rclcpp::QoS(1).best_effort(),
      [this, i](const interfaces::msg::ELRSCommand::SharedPtr msg) {
        this->elrsCallback(msg, i);
      }));
    cap_subs_.push_back(node_->create_subscription<std_msgs::msg::Float64>(
      ns + "/throttle_max", rclcpp::QoS(1).reliable().transient_local(),
      [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
        if (i < static_cast<int>(drone_cap_.size())) {
          drone_cap_[i] = static_cast<float>(msg->data);
        }
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
  const QString thr_str = (drone_thr_seen_[i] && drone_armed_[i])
                            ? QString("thr %1").arg(drone_thr_[i], 0, 'f', 2)
                            : QString("thr --");
  const bool at_cap = drone_cap_since_[i] >= 0.0 &&
                      wallNow() - drone_cap_since_[i] >= THR_CAP_HOLD_S;
  QString text = QString("D%1   %2   %3   %4%5").arg(i + 1).arg(armed_str).arg(volt_str).arg(thr_str)
                   .arg(at_cap ? "  AT CAP" : "");

  // Nothing heard from this drone at all -- neutral grey, not a battery colour.
  if (!drone_volt_seen_[i] && !at_cap) {
    drone_labels_[i]->setText(text);
    drone_labels_[i]->setStyleSheet(
      "font-size: 12px; padding: 3px; background-color: #f0f0f0; "
      "color: #868e96; border-radius: 3px;");
    return;
  }

  // Colour by battery (6S: 25.2 V full; the pre-flight rule is >= 24.0 V at rest), so the
  // weakest pack is obvious at a glance. A drone pinned at the throttle cap turns red.
  QString bg, fg = "white";
  if (at_cap)          { bg = "#e03131"; }
  else if (v >= 24.0f) { bg = "#51cf66"; }
  else if (v >= 22.8f) { bg = "#ffd43b"; fg = "#495057"; }
  else if (v >= 21.0f) { bg = "#ff922b"; }
  else                 { bg = "#ff6b6b"; }

  drone_labels_[i]->setText(text);
  drone_labels_[i]->setStyleSheet(
    QString("font-size: 12px; padding: 3px; background-color: %1; color: %2; "
            "border-radius: 3px; font-weight: bold;").arg(bg).arg(fg));
}

void ArmPanel::refreshDroneLabels()
{
  for (int i = 0; i < static_cast<int>(drone_labels_.size()); ++i) {
    updateDroneLabel(i);
  }
  updateAirLabel();
}

void ArmPanel::updateAirLabel()
{
  QStringList up;
  for (int i = 0; i < static_cast<int>(drone_z_.size()); ++i) {
    if (!std::isnan(drone_z_[i]) && !std::isnan(drone_rest_z_[i]) && drone_z_[i] - drone_rest_z_[i] > AIRBORNE_M) {
      up << QString("QUAD%1 %2 m").arg(i + 1).arg(drone_z_[i] - drone_rest_z_[i], 0, 'f', 2);
    }
  }
  air_label_->setText(up.isEmpty() ? QString("Nobody in the air") : "In the air: " + up.join(", "));
  air_label_->setStyleSheet(up.isEmpty() ? "font-size: 12px; padding: 3px; color: #495057;"
                                         : "font-size: 12px; padding: 3px; font-weight: bold; "
                                           "background-color: #ffe066; color: #495057; border-radius: 3px;");
}

void ArmPanel::refreshGraph()
{
  if (!node_) {
    return;
  }
  const auto names = node_->get_node_names();
  const bool present = std::find(names.begin(), names.end(), HIS_MPC) != names.end();
  bool mpc = false;
  bool planner = false;
  const auto infos = node_->get_subscriptions_info_by_topic("/tejen/drone_command");
  for (const auto& e : infos) {
    const std::string ns = e.node_namespace() == "/" ? "" : e.node_namespace();
    mpc = mpc || ns + "/" + e.node_name() == HIS_MPC;
    planner = planner || ns + "/" + e.node_name() == HIS_PLANNER;
  }
  const size_t matched = his_cmd_pub_ ? his_cmd_pub_->get_subscription_count() : 0;
  if (!mpc || !planner) {
    his_link_ok_ = false;
    his_link_why_ = std::string("waiting for ") + (!mpc ? "tejen_mpc" : "online_join_planner");
  } else if (matched < infos.size()) {
    his_link_ok_ = false;
    his_link_why_ = "matched " + std::to_string(matched) + " of " + std::to_string(infos.size());
  } else {
    his_link_ok_ = true;
    his_link_why_.clear();
  }
  give_up_subs_ = node_->count_subscribers("/join_planner/give_up");
  if (present != partner_present_) {
    partner_present_ = present;
    partner_row_->setVisible(present);
  }
  updateButtonState();
}

void ArmPanel::updatePartnerButtons()
{
  // each greyed-out button says why, as in tools/console.py
  auto set = [](QPushButton* b, const char* label, bool ok, const std::string& why) {
    b->setEnabled(ok);
    b->setText(ok || why.empty() ? QString(label)
                                 : QString("%1\n(%2)").arg(label).arg(QString::fromStdString(why)));
  };
  set(his_arm_button_, "his ARM", his_link_ok_ && !his_armed_ && !is_armed_,
      !his_link_ok_ ? his_link_why_ : his_armed_ ? "QUAD4 armed" : "fleet armed");
  set(his_takeoff_button_, "his TAKEOFF", arm_complete_ && his_armed_ && his_link_ok_,
      !arm_complete_ ? "after the fleet ARM" : !his_armed_ ? "QUAD4 not armed" : his_link_why_);
  set(give_up_button_, "GIVE UP", give_up_subs_ > 0 && is_armed_,
      give_up_subs_ == 0 ? "our planner not listening" : "nothing armed");
  set(his_disarm_button_, "his DISARM", his_link_ok_ && !is_armed_,
      !his_link_ok_ ? his_link_why_ : "fleet still armed: LAND or DISARM first");
}

void ArmPanel::sendHis(const std::string& cmd)
{
  std_msgs::msg::String msg;
  msg.data = cmd;
  his_cmd_pub_->publish(msg);
  status_message_ = "Sent his " + cmd;
  status_alarm_ = false;
  updateStatusLabel();
}

void ArmPanel::onHisArmPressed()
{
  if (his_cmd_pub_ && his_link_ok_ && !his_armed_ && !is_armed_) {
    sendHis("ARM");
  }
}

void ArmPanel::onHisTakeoffPressed()
{
  if (his_cmd_pub_ && arm_complete_ && his_armed_ && his_link_ok_) {
    sendHis("TAKEOFF");
  }
}

void ArmPanel::onHisDisarmPressed()
{
  if (his_cmd_pub_ && his_link_ok_ && !is_armed_) {
    sendHis("DISARM");
  }
}

void ArmPanel::onGiveUpPressed()
{
  if (!give_up_pub_ || give_up_subs_ == 0 || !is_armed_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = true;
  give_up_pub_->publish(msg);
  status_message_ = "Sent GIVE UP (QUAD4 taken back, the ring stays up)";
  status_alarm_ = false;
  updateStatusLabel();
}

void ArmPanel::onButtonPressed()
{
  if (is_armed_) {
    sendDisarm();
  } else {
    sendArm();
  }
}

void ArmPanel::onSpacePressed()
{
  sendDisarm();
}

bool ArmPanel::managerRunning() const
{
  // By name: other nodes (fleet_viz, the planner) also subscribe to /fleet/command, so a
  // subscriber count says nothing about the manager.
  if (!node_) {
    return false;
  }
  const auto names = node_->get_node_names();
  return std::any_of(names.begin(), names.end(), [](const std::string& n) {
    return n == "/fleet_manager" || n == "fleet_manager";
  });
}

void ArmPanel::sendArm()
{
  if (!command_pub_ || !managerRunning()) {
    status_message_ = "Fleet manager not running (start T2)";
    status_alarm_ = true;
    updateStatusLabel();
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "ARM";
  command_pub_->publish(msg);
  status_message_ = "Sent ARM";
  status_alarm_ = false;
  updateStatusLabel();
}

void ArmPanel::sendDisarm()
{
  // Never blocked: even with no manager in sight the message goes out.
  if (!command_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "DISARM";
  command_pub_->publish(msg);
  disarm_left_ = DISARM_REPEATS - 1;
  disarm_timer_->start();
  status_message_ = managerRunning() ? "Sent DISARM" : "Sent DISARM (fleet manager not seen!)";
  status_alarm_ = !managerRunning();
  updateStatusLabel();
}

void ArmPanel::onDisarmRepeat()
{
  if (disarm_left_ <= 0 || !command_pub_) {
    disarm_timer_->stop();
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "DISARM";
  command_pub_->publish(msg);
  --disarm_left_;
}

void ArmPanel::onTakeoffPressed()
{
  if (!command_pub_ || armed_count_ < num_drones_ || !arm_complete_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "TAKEOFF";
  command_pub_->publish(msg);
  status_message_ = "Sent TAKEOFF";
  status_alarm_ = false;
  updateStatusLabel();
}

void ArmPanel::onLandPressed()
{
  if (command_pub_ && is_armed_) {
    std_msgs::msg::String msg;
    msg.data = "LAND";
    command_pub_->publish(msg);
    status_message_ = "Sent LAND";
    status_alarm_ = false;
    updateStatusLabel();
  }
}

void ArmPanel::onDetachPressed()
{
  if (!is_armed_ || detach_used_) {
    return;
  }
  if (!detach_pub_ || detach_pub_->get_subscription_count() == 0) {
    status_message_ = "DETACH not sent: nothing listens on /fleet/detach (detach launch not running)";
    status_alarm_ = true;
    updateStatusLabel();
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = detach_id_spin_->value();
  detach_pub_->publish(msg);
  detach_used_ = true;          // one DETACH per flight
  detach_button_->setEnabled(false);
  status_message_ = "Sent DETACH drone " + std::to_string(msg.data);
  status_alarm_ = false;
  updateStatusLabel();
}

void ArmPanel::onAttachPressed()
{
  if (!is_armed_) {
    return;
  }
  if (!magnet_cmd_pub_ || magnet_cmd_pub_->get_subscription_count() == 0) {
    status_message_ = "ATTACH not sent: nothing listens on /magnet/command (magnet manager not running)";
    status_alarm_ = true;
    updateStatusLabel();
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "ON";
  magnet_cmd_pub_->publish(msg);
  status_message_ = "Sent ATTACH (magnet ON, approaching)";
  status_alarm_ = false;
  updateStatusLabel();
}

void ArmPanel::updateMagnetButton(int i)
{
  if (i < 0 || i >= static_cast<int>(magnet_buttons_.size())) {
    return;
  }
  const bool on = magnet_buttons_[i]->isChecked();
  magnet_buttons_[i]->setText(QString("MAGNET D%1   %2%3").arg(i + 1).arg(on ? "ON" : "OFF")
                                .arg(is_armed_ ? "   (locked while armed)" : ""));
  magnet_buttons_[i]->setStyleSheet(on
    ? "background-color: #20c997; color: white; font-weight: bold;"
    : "background-color: #adb5bd; color: white; font-weight: bold;");
}

void ArmPanel::onMagnetToggled(int i, bool on)
{
  updateMagnetButton(i);
  if (!node_ || i >= static_cast<int>(magnet_pubs_.size()) || !magnet_pubs_[i]) {
    return;
  }
  if (magnet_pubs_[i]->get_subscription_count() == 0) {
    status_message_ = "No /drone_" + std::to_string(i) + "/magnet subscriber (radio not running)";
    status_alarm_ = true;
    updateStatusLabel();
  }
  std_msgs::msg::String msg;
  msg.data = on ? "ON" : "OFF";
  magnet_pubs_[i]->publish(msg);
}

void ArmPanel::applyShowMagnets()
{
  for (size_t i = 0; i < magnet_buttons_.size(); ++i) {
    magnet_buttons_[i]->setVisible(show_magnets_ && !magnetHidden(static_cast<int>(i)));
  }
}

bool ArmPanel::magnetHidden(int i) const
{
  return std::find(hidden_magnets_.begin(), hidden_magnets_.end(), i) != hidden_magnets_.end();
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
  bool show_m = false;
  if (config.mapGetBool("ShowMagnets", &show_m)) {
    show_magnets_ = show_m;
  }
  bool m_on = true;
  if (config.mapGetBool("MagnetInitialOn", &m_on)) {
    magnet_initial_on_ = m_on;
  }
  QString hidden;
  hidden_magnets_.clear();
  if (config.mapGetString("HiddenMagnets", &hidden)) {
    for (const auto& tok : hidden.split(',', Qt::SkipEmptyParts)) {
      bool ok = false;
      const int d = tok.trimmed().toInt(&ok);
      if (ok) {
        hidden_magnets_.push_back(d);
      }
    }
  }

  // Fleet size for the per-drone rows. The launch files write this; absent or
  // nonsensical values fall back to a single drone.
  int n = 0;
  if (config.mapGetInt("NumDrones", &n) && n > 0) {
    num_drones_ = n;
  }
  rebuild();
  applyShowMagnets();
}

void ArmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ShowDetach", show_detach_);
  config.mapSetValue("ShowAttach", show_attach_);
  config.mapSetValue("ShowMagnets", show_magnets_);
  config.mapSetValue("MagnetInitialOn", magnet_initial_on_);
  if (!hidden_magnets_.empty()) {
    QStringList ids;
    for (int d : hidden_magnets_) {
      ids << QString::number(d);
    }
    config.mapSetValue("HiddenMagnets", ids.join(","));
  }
  config.mapSetValue("NumDrones", num_drones_);
}

void ArmPanel::updateButtonState()
{
  if (is_armed_) {
    arm_button_->setText("DISARM  (space)");
    arm_button_->setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold;");
  } else {
    // with his stack up, his ARM comes first (the manager refuses the fleet ARM until QUAD4's FC is armed)
    const bool his_first = partner_present_ && !his_armed_;
    arm_button_->setText(his_first ? "ARM\n(his ARM first)" : "ARM");
    arm_button_->setStyleSheet(his_first ? "background-color: #e9ecef; color: #868e96;"
                                         : "background-color: #51cf66; color: white; font-weight: bold;");
    detach_used_ = false;        // a new flight may detach again
  }
  arm_button_->setEnabled(is_armed_ || !partner_present_ || his_armed_);
  // TAKEOFF only after the manager's "ARM sequence complete" with the whole fleet armed.
  takeoff_button_->setEnabled(arm_complete_ && armed_count_ == num_drones_ && num_drones_ > 0);
  land_button_->setEnabled(is_armed_);
  detach_button_->setEnabled(is_armed_ && !detach_used_);
  attach_button_->setEnabled(is_armed_);
  // Tether magnets stay as they are while anything is armed: a mis-click would drop one.
  for (int i = 0; i < static_cast<int>(magnet_buttons_.size()); ++i) {
    magnet_buttons_[i]->setEnabled(!is_armed_);
    updateMagnetButton(i);
  }
  updatePartnerButtons();
  updateStatusLabel();
}

void ArmPanel::updateStatusLabel()
{
  status_label_->setText(QString::fromStdString(status_message_));
  if (status_alarm_) {
    status_label_->setStyleSheet("font-size: 12px; font-weight: bold; padding: 5px; "
                                 "background-color: #e03131; border-radius: 3px; color: white;");
  } else if (is_armed_) {
    status_label_->setStyleSheet("font-size: 12px; font-weight: bold; padding: 5px; "
                                 "background-color: #ffe066; border-radius: 3px; color: #c92a2a;");
  } else {
    status_label_->setStyleSheet("font-size: 12px; font-weight: bold; padding: 5px; "
                                 "background-color: #f0f0f0; border-radius: 3px; color: #495057;");
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
  if (!msg->data) {
    arm_complete_ = false;
  }
  updateDroneLabel(i);

  if (was_seen && previous_drone_state == drone_armed_[i]) {
    return;
  }

  // Fleet-level state: ANY drone armed counts as armed, so DISARM and LAND stay
  // reachable when only part of the fleet is live.
  is_armed_ = false;
  armed_count_ = 0;
  for (int k = 0; k < static_cast<int>(drone_armed_.size()); ++k) {
    if (drone_armed_[k]) { is_armed_ = true; ++armed_count_; }
  }
  updateButtonState();
}

void ArmPanel::telemetryCallback(const interfaces::msg::Telemetry::SharedPtr msg, int i)
{
  if (i < 0 || i >= static_cast<int>(drone_voltage_.size())) {
    return;
  }
  drone_voltage_[i] = msg->battery_voltage;
  drone_volt_seen_[i] = true;
}

void ArmPanel::elrsCallback(const interfaces::msg::ELRSCommand::SharedPtr msg, int i)
{
  if (i < 0 || i >= static_cast<int>(drone_thr_.size())) {
    return;
  }
  // channel_2 = 2 u - 1 (the tracker's throttle u in [0, 1])
  const float thr = msg->armed ? 0.5f * (msg->channel_2 + 1.0f) : 0.0f;
  drone_thr_[i] = thr;
  drone_thr_seen_[i] = true;
  if (thr >= drone_cap_[i] - THR_CAP_MARGIN) {
    if (drone_cap_since_[i] < 0.0) {
      drone_cap_since_[i] = wallNow();
    }
  } else {
    drone_cap_since_[i] = -1.0;
  }
}

}  // namespace drone_visualisation

PLUGINLIB_EXPORT_CLASS(drone_visualisation::ArmPanel, rviz_common::Panel)
