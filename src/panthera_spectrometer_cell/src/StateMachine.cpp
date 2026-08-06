#include "panthera_spectrometer_cell/StateMachine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace panthera_spectrometer_cell
{
namespace
{

class TickGuard
{
public:
  explicit TickGuard(std::atomic_bool & flag)
  : flag_(flag)
  {
    owns_ = !flag_.exchange(true);
  }

  ~TickGuard()
  {
    if (owns_) {
      flag_.store(false);
    }
  }

  bool owns() const
  {
    return owns_;
  }

private:
  std::atomic_bool & flag_;
  bool owns_{false};
};

std::string jsonEscape(const std::string & text)
{
  std::ostringstream out;
  for (const char c : text) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

bool isOutletDischargeEvent(EventType type)
{
  return type == EventType::OUTLET_1_DISCHARGE_DONE ||
         type == EventType::OUTLET_2_DISCHARGE_DONE;
}

OutletId outletFromEvent(EventType type)
{
  if (type == EventType::OUTLET_1_DISCHARGE_DONE) {
    return OutletId::OUTLET_1;
  }
  if (type == EventType::OUTLET_2_DISCHARGE_DONE) {
    return OutletId::OUTLET_2;
  }
  return OutletId::NONE;
}

Event makeEvent(
  EventType type,
  const std::string & source,
  const std::string & name,
  const std::string & message,
  std::uint64_t timestamp_ms)
{
  Event event;
  event.type = type;
  event.source = source;
  event.name = name;
  event.message = message;
  event.timestampMs = timestamp_ms;
  return event;
}

}  // namespace

StateMachine::StateMachine(
  rclcpp::Node::SharedPtr node,
  WorkcellConfig config,
  std::shared_ptr<RobotActions> robot,
  std::shared_ptr<Sensors> sensors,
  std::shared_ptr<Spectrometer> spectrometer)
: node_(std::move(node)),
  logger_(node_->get_logger().get_child("state_machine")),
  config_(std::move(config)),
  robot_(std::move(robot)),
  sensors_(std::move(sensors)),
  spectrometer_(std::move(spectrometer)),
  boot_time_(std::chrono::steady_clock::now())
{
  initializeRuntimeState();

  state_pub_ = node_->create_publisher<std_msgs::msg::String>("/spectrometer_cell/state", 10);
  error_pub_ = node_->create_publisher<std_msgs::msg::String>("/spectrometer_cell/error", 10);
  context_pub_ = node_->create_publisher<std_msgs::msg::String>("/spectrometer_cell/context", 10);
  setupManualControlServices();

  publishState();
  publishContext();
}

void StateMachine::initializeRuntimeState()
{
  const auto now = nowMs();
  outlet_status_[OutletId::OUTLET_1] = OutletStatus::WAITING_FILL;
  outlet_status_[OutletId::OUTLET_2] = OutletStatus::WAITING_FILL;
  pending_outlets_.clear();
  context_.clearTask(0, State::INIT, now);
  context_.currentState = State::INIT;
  context_.previousState = State::INIT;
  context_.clearError();
  state_ = State::INIT;
  last_wait_notice_time_ = std::chrono::steady_clock::now();
}

std::uint64_t StateMachine::nowMs() const
{
  const auto elapsed = std::chrono::steady_clock::now() - boot_time_;
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
}

void StateMachine::tick()
{
  TickGuard guard(tick_running_);
  if (!guard.owns()) {
    return;
  }

  auto events = collectEvents();
  Event event = popHighestPriorityEvent(events);
  logEvent(event);

  if (handleGlobalEvent(event)) {
    checkInvariant();
    publishContext();
    return;
  }

  switch (state_) {
    case State::INIT:
      updateInit(event);
      break;
    case State::IDLE:
      updateIdle(event);
      break;
    case State::WAIT_DISCHARGE:
      updateWaitDischarge(event);
      break;
    case State::SELECT_TASK:
      updateSelectTask(event);
      break;
    case State::PICK_FROM_OUTLET:
      updatePickFromOutlet(event);
      break;
    case State::MEASURE_SPECTROMETER_BEFORE_PLACE:
      updateMeasureBeforePlace(event);
      break;
    case State::PLACE_TO_SPECTROMETER:
      updatePlaceToSpectrometer(event);
      break;
    case State::START_DETECTION:
      updateStartDetection(event);
      break;
    case State::WAIT_DETECTION_DONE:
      updateWaitDetectionDone(event);
      break;
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      updateMeasureBeforePick(event);
      break;
    case State::PICK_FROM_SPECTROMETER:
      updatePickFromSpectrometer(event);
      break;
    case State::CLEAN_CUP:
      updateCleanCup(event);
      break;
    case State::RETURN_CUP:
      updateReturnCup(event);
      break;
    case State::COMPLETE_CYCLE:
      updateCompleteCycle(event);
      break;
    case State::PAUSED:
      updatePaused(event);
      break;
    case State::ERROR:
      updateError(event);
      break;
    case State::ESTOP:
      updateEstop(event);
      break;
    case State::RESET:
      updateReset(event);
      break;
  }

  checkTimeouts();
  checkInvariant();
  publishContext();
}

State StateMachine::currentState() const
{
  return state_;
}

ActionResult StateMachine::recoverGripper()
{
  TickGuard guard(tick_running_);
  if (!guard.owns()) {
    return ActionResult::fail("gripper recovery rejected: state machine is busy");
  }
  if (context_.hasActiveTask || context_.cupInGripper || context_.spectrometerOccupied) {
    return ActionResult::fail(
      "gripper recovery rejected: active task or cup ownership is not empty");
  }
  const bool allowed_state =
    state_ == State::INIT || state_ == State::IDLE ||
    state_ == State::WAIT_DISCHARGE || state_ == State::PAUSED ||
    state_ == State::ERROR;
  if (!allowed_state) {
    return ActionResult::fail(
      "gripper recovery rejected in state " + toString(state_));
  }
  const auto recovery = robot_->ensureGripperReady(true);
  if (!recovery.success) {
    return recovery;
  }
  const auto motion_test = robot_->verifyGripperMotion();
  if (!motion_test.success) {
    return ActionResult::fail(
      "gripper reset succeeded but physical motion self-test failed: " + motion_test.message);
  }
  return motion_test;
}

ActionResult StateMachine::reloadConfig(const WorkcellConfig & config)
{
  TickGuard guard(tick_running_);
  if (!guard.owns()) {
    return ActionResult::fail("reload config rejected: state machine tick is running");
  }

  if (context_.activeCommandId != 0 || !context_.activeActionName.empty()) {
    return ActionResult::fail("reload config rejected: robot action is active");
  }

  const bool safe_state =
    state_ == State::IDLE ||
    state_ == State::WAIT_DISCHARGE ||
    state_ == State::PAUSED ||
    state_ == State::ERROR ||
    state_ == State::ESTOP;
  if (!safe_state) {
    return ActionResult::fail(
      "reload config rejected: current state is " + toString(state_) +
      ", switch to IDLE/WAIT_DISCHARGE/PAUSED first");
  }

  config_ = config;
  if (robot_) {
    robot_->updateConfig(config_);
  }
  if (sensors_) {
    sensors_->updateConfig(config_);
  }
  if (spectrometer_) {
    spectrometer_->updateConfig(config_);
  }
  publishContext();
  RCLCPP_WARN(logger_, "workcell config reloaded without restarting node");
  return ActionResult::ok("workcell config reloaded without restart");
}

std::vector<Event> StateMachine::collectEvents()
{
  std::vector<Event> events;
  const auto stamp = nowMs();

  if (sensors_->isEmergencyStopActive() && state_ != State::ESTOP) {
    events.push_back(
      makeEvent(
        EventType::ESTOP_PRESSED,
        "sensors",
        "emergency_stop",
        "emergency stop active",
        stamp));
  } else if (!sensors_->isEmergencyStopActive() && estop_pressed_ && !estop_released_) {
    events.push_back(
      makeEvent(
        EventType::ESTOP_RELEASED,
        "sensors",
        "emergency_stop",
        "emergency stop released",
        stamp));
  }

  if (sensors_->consumeManualResetRequest()) {
    events.push_back(
      makeEvent(
        EventType::RESET_SYSTEM,
        "operator",
        "request_reset",
        "manual reset requested",
        stamp));
  }

  if (manual_mode_enabled_.load() && !context_.pauseRequested &&
    state_ != State::PAUSED && state_ != State::ERROR &&
    state_ != State::ESTOP && state_ != State::RESET)
  {
    events.push_back(
      makeEvent(
        EventType::PAUSE_AUTO,
        "operator",
        "manual_mode",
        "manual pause requested",
        stamp));
  }

  if (manual_step_requested_.exchange(false)) {
    events.push_back(
      makeEvent(
        EventType::MANUAL_NEXT_STEP,
        "operator",
        "step_once",
        "manual next step requested",
        stamp));
  }

  const auto discharge = sensors_->readDischargeDone();
  if (discharge.ready) {
    const EventType type = discharge.outletId == OutletId::OUTLET_1 ?
      EventType::OUTLET_1_DISCHARGE_DONE :
      EventType::OUTLET_2_DISCHARGE_DONE;
    events.push_back(
      makeEvent(
        type, "sensors", toString(discharge.outletId), discharge.message,
        stamp));
  }

  if (events.empty()) {
    events.push_back(makeEvent(EventType::NONE, "state_machine", "", "", stamp));
  }
  return events;
}

Event StateMachine::popHighestPriorityEvent(std::vector<Event> & events) const
{
  auto best = std::min_element(
    events.begin(), events.end(),
    [this](const Event & lhs, const Event & rhs) {
      return eventPriority(lhs.type) < eventPriority(rhs.type);
    });
  if (best == events.end()) {
    return makeEvent(EventType::NONE, "state_machine", "", "", nowMs());
  }
  return *best;
}

int StateMachine::eventPriority(EventType type) const
{
  switch (type) {
    case EventType::ESTOP_PRESSED:
      return 1;
    case EventType::ACTION_FAILED:
    case EventType::SPECTROMETER_ERROR:
      return 3;
    case EventType::TIMEOUT:
      return 4;
    case EventType::PAUSE_AUTO:
      return 5;
    case EventType::RESET_SYSTEM:
      return 6;
    case EventType::MANUAL_STATE_REQUEST:
      return 7;
    case EventType::MANUAL_NEXT_STEP:
      return 8;
    case EventType::ACTION_SUCCESS:
      return 9;
    case EventType::SENSOR_VALID:
    case EventType::SENSOR_INVALID:
      return 10;
    case EventType::SPECTROMETER_STARTED:
    case EventType::SPECTROMETER_BUSY:
    case EventType::SPECTROMETER_DETECTION_DONE:
      return 11;
    case EventType::OUTLET_1_DISCHARGE_DONE:
    case EventType::OUTLET_2_DISCHARGE_DONE:
      return 12;
    case EventType::START_AUTO:
    case EventType::RESUME_AUTO:
    case EventType::CLEAR_ERROR:
    case EventType::CLEAR_ESTOP:
    case EventType::ESTOP_RELEASED:
      return 13;
    case EventType::NONE:
    case EventType::CLEAR_DISCHARGE_SIGNAL:
      return 14;
  }
  return 100;
}

void StateMachine::logEvent(const Event & event) const
{
  if (event.type == EventType::NONE) {
    return;
  }
  RCLCPP_INFO(
    logger_,
    "EVENT_RECEIVED source=%s type=%s name=%s message=%s timestamp_ms=%lu",
    event.source.c_str(),
    toString(event.type).c_str(),
    event.name.c_str(),
    event.message.c_str(),
    event.timestampMs);
}

bool StateMachine::handleGlobalEvent(const Event & event)
{
  if (event.type == EventType::ESTOP_PRESSED) {
    if (state_ == State::ESTOP) {
      return true;
    }
    estop_pressed_ = true;
    estop_released_ = false;
    auto_mode_enabled_ = false;
    manual_mode_enabled_.store(false);
    manual_step_requested_.store(false);
    context_.setError("emergency stop active");
    clearActiveAction();
    const auto stop_result = robot_->stop();
    if (!stop_result.success) {
      RCLCPP_ERROR(logger_, "ESTOP stop failed: %s", stop_result.message.c_str());
    }
    RCLCPP_ERROR(logger_, "ESTOP pressed source=%s", event.source.c_str());
    transitionTo(State::ESTOP, "ESTOP_PRESSED");
    publishError();
    return true;
  }

  if (event.type == EventType::ESTOP_RELEASED) {
    estop_released_ = true;
    RCLCPP_WARN(logger_, "ESTOP_RELEASED: hardware stop released, reset is still required");
    if (state_ == State::ESTOP) {
      publishContext();
      return true;
    }
  }

  if (isOutletDischargeEvent(event.type)) {
    handleDischargeEvent(event);
    return true;
  }

  if (event.type == EventType::PAUSE_AUTO && state_ != State::PAUSED) {
    pauseCurrentStateOrDefer();
    return true;
  }

  if (event.type == EventType::RESET_SYSTEM && state_ != State::RESET) {
    manual_mode_enabled_.store(false);
    manual_step_requested_.store(false);
    auto_mode_enabled_ = false;
    transitionTo(State::RESET, "RESET_SYSTEM");
    return true;
  }

  if (event.type == EventType::MANUAL_STATE_REQUEST) {
    return handleManualStateRequest(event);
  }

  return false;
}

void StateMachine::handleDischargeEvent(const Event & event)
{
  const OutletId outlet = outletFromEvent(event.type);
  enqueueOutletIfAllowed(outlet, event.source);
  if (state_ == State::WAIT_DISCHARGE && !pending_outlets_.empty()) {
    transitionTo(State::SELECT_TASK, "pending outlet available");
  }
}

void StateMachine::enqueueOutletIfAllowed(OutletId outlet, const std::string & source)
{
  if (outlet == OutletId::NONE) {
    RCLCPP_WARN(logger_, "INVALID_DISCHARGE source=%s outlet=NONE", source.c_str());
    return;
  }

  const auto current = outlet_status_[outlet];
  if (current != OutletStatus::WAITING_FILL) {
    RCLCPP_WARN(
      logger_,
      "DUPLICATE_OR_ILLEGAL_DISCHARGE source=%s outlet=%s status=%s ignored",
      source.c_str(),
      toString(outlet).c_str(),
      toString(current).c_str());
    return;
  }

  if (pendingContains(outlet)) {
    RCLCPP_WARN(logger_, "DUPLICATE_DISCHARGE_QUEUE outlet=%s ignored", toString(outlet).c_str());
    return;
  }

  outlet_status_[outlet] = OutletStatus::FILLED;
  pending_outlets_.push_back(outlet);
  RCLCPP_INFO(logger_, "QUEUE_PUSH outlet=%s", toString(outlet).c_str());
}

bool StateMachine::pendingContains(OutletId outlet) const
{
  return std::find(
    pending_outlets_.begin(), pending_outlets_.end(),
    outlet) != pending_outlets_.end();
}

void StateMachine::updateInit(const Event &)
{
  pending_outlets_.clear();
  outlet_status_[OutletId::OUTLET_1] = OutletStatus::WAITING_FILL;
  outlet_status_[OutletId::OUTLET_2] = OutletStatus::WAITING_FILL;
  context_.clearTask(0, State::INIT, nowMs());

  // Bring up non-motion I/O first so reset, discharge and detection services
  // remain available when the robot's Home guard deliberately rejects INIT.
  auto result = sensors_->initialize();
  if (!result.success) {
    fail("INIT sensors failed: " + result.message);
    return;
  }

  result = spectrometer_->initialize();
  if (!result.success) {
    fail("INIT spectrometer failed: " + result.message);
    return;
  }

  result = robot_->initialize();
  if (!result.success) {
    fail("INIT robot failed: " + result.message);
    return;
  }

  transitionTo(State::IDLE, "initialization complete");
}

void StateMachine::updateIdle(const Event & event)
{
  context_.clearTask(context_.cycleId, State::IDLE, nowMs());
  clearActiveAction();

  if (event.type == EventType::START_AUTO || event.type == EventType::RESUME_AUTO ||
    auto_mode_enabled_)
  {
    auto_mode_enabled_ = true;
    transitionTo(State::WAIT_DISCHARGE, "auto mode enabled");
    return;
  }
}

void StateMachine::updateWaitDischarge(const Event & event)
{
  if (event.type == EventType::MANUAL_NEXT_STEP) {
    if (pending_outlets_.empty()) {
      RCLCPP_WARN(logger_, "MANUAL_NEXT_STEP rejected in WAIT_DISCHARGE: no pending outlet");
      return;
    }
    transitionTo(State::SELECT_TASK, "manual next step with pending outlet");
    return;
  }

  if (!pending_outlets_.empty()) {
    transitionTo(State::SELECT_TASK, "pending outlet available");
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed_since_notice =
    std::chrono::duration<double>(now - last_wait_notice_time_).count();
  if (elapsed_since_notice >= config_.loop.waitDischargeSoftTimeoutSec) {
    RCLCPP_INFO(
      logger_,
      "WAIT_DISCHARGE soft timeout: no outlet ready for %.1fs, continuing",
      elapsed_since_notice);
    last_wait_notice_time_ = now;
  }
}

void StateMachine::updateSelectTask(const Event &)
{
  if (pending_outlets_.empty()) {
    transitionTo(State::WAIT_DISCHARGE, "no pending outlet");
    return;
  }

  const auto gripper_ready = robot_->ensureGripperReady();
  if (!gripper_ready.success) {
    RCLCPP_WARN(
      logger_, "SELECT_TASK waiting for gripper recovery: %s",
      gripper_ready.message.c_str());
    return;
  }

  const OutletId outlet = pending_outlets_.front();
  pending_outlets_.pop_front();

  if (outlet_status_[outlet] != OutletStatus::FILLED || context_.hasActiveTask ||
    context_.cupInGripper || context_.spectrometerOccupied)
  {
    std::ostringstream out;
    out << "SELECT_TASK invalid context outlet=" << toString(outlet)
        << " outlet_status=" << toString(outlet_status_[outlet])
        << " hasActiveTask=" << context_.hasActiveTask
        << " cupInGripper=" << context_.cupInGripper
        << " spectrometerOccupied=" << context_.spectrometerOccupied;
    fail(out.str());
    return;
  }

  context_.beginTask(next_cycle_id_++, outlet, state_, nowMs());
  context_.currentState = state_;
  context_.clearError();
  RCLCPP_INFO(
    logger_, "TASK_CREATED cycle=%lu outlet=%s", context_.cycleId, toString(
      outlet).c_str());
  transitionTo(State::PICK_FROM_OUTLET, "task selected");
}

void StateMachine::updatePickFromOutlet(const Event &)
{
  const bool ok = runActionWithFeedback(
    "pick_from_outlet",
    [this]() {return robot_->pickFromOutlet(context_.outletId);});
  if (!ok) {
    return;
  }

  context_.cupPickedFromOutlet = true;
  context_.cupInGripper = true;
  outlet_status_[context_.outletId] = OutletStatus::CUP_TAKEN;
  clearActiveAction();

  if (context_.pauseRequested) {
    context_.pausedFromState = State::MEASURE_SPECTROMETER_BEFORE_PLACE;
    transitionTo(State::PAUSED, "pause requested after pick_from_outlet");
  } else {
    transitionTo(State::MEASURE_SPECTROMETER_BEFORE_PLACE, "cup picked from outlet");
  }
}

void StateMachine::updateMeasureBeforePlace(const Event &)
{
  double value = 0.0;
  std::string error;
  bool waiting = false;
  if (!measureSpectrometerPosition("measure_before_place", value, error, waiting)) {
    if (waiting) {
      RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 2000, "%s", error.c_str());
      return;
    }
    fail(error);
    return;
  }

  context_.spectrometerPositionBeforePlace = value;
  context_.spectrometerPositionBeforePlaceValid = true;
  RCLCPP_INFO(logger_, "SENSOR_VALID name=measure_before_place value=%.3fmm", value);
  transitionTo(State::PLACE_TO_SPECTROMETER, "valid spectrometer position before place");
}

void StateMachine::updatePlaceToSpectrometer(const Event &)
{
  const bool ok = runActionWithFeedback(
    "place_to_spectrometer",
    [this]() {return robot_->placeToSpectrometer(context_.spectrometerPositionBeforePlace);});
  if (!ok) {
    return;
  }

  context_.cupInGripper = false;
  context_.cupPlacedToSpectrometer = true;
  context_.spectrometerOccupied = true;
  clearActiveAction();

  if (context_.pauseRequested) {
    context_.pausedFromState = State::START_DETECTION;
    transitionTo(State::PAUSED, "pause requested after place_to_spectrometer");
  } else {
    transitionTo(State::START_DETECTION, "cup placed to spectrometer");
  }
}

void StateMachine::updateStartDetection(const Event &)
{
  if (!startDetectionWithFeedback()) {
    return;
  }

  context_.detectionStarted = true;
  RCLCPP_INFO(logger_, "SPECTROMETER_STARTED");
  transitionTo(State::WAIT_DETECTION_DONE, "spectrometer detection started");
}

void StateMachine::updateWaitDetectionDone(const Event & event)
{
  if (event.type == EventType::MANUAL_NEXT_STEP) {
    if (!context_.detectionDone) {
      RCLCPP_WARN(logger_, "MANUAL_NEXT_STEP rejected: detection is not done");
      return;
    }
    transitionTo(State::MEASURE_SPECTROMETER_BEFORE_PICK, "manual next after detection done");
    return;
  }

  bool done = false;
  const auto result = spectrometer_->pollDetectionDone(done);
  if (!result.success) {
    fail("SPECTROMETER_ERROR: " + result.message);
    return;
  }

  if (!done && !config_.simulation.enabled) {
    const auto scan = sensors_->pollScanTracking();
    done = scan.done;
    RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 5000,
      "LASER_SCAN phase=%s farthest=%.3fmm remaining=%.1fs",
      toString(scan.phase), scan.farthestMm, scan.remainingSec);
  }

  if (done) {
    context_.detectionDone = true;
    RCLCPP_INFO(logger_, "SPECTROMETER_DETECTION_DONE message=%s", result.message.c_str());
    transitionTo(State::MEASURE_SPECTROMETER_BEFORE_PICK, "spectrometer detection done");
  }
}

void StateMachine::updateMeasureBeforePick(const Event &)
{
  double value = 0.0;
  std::string error;
  bool waiting = false;
  if (!measureSpectrometerPosition("measure_before_pick", value, error, waiting)) {
    if (waiting) {
      RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 2000, "%s", error.c_str());
      return;
    }
    fail(error);
    return;
  }

  context_.spectrometerPositionBeforePick = value;
  context_.spectrometerPositionBeforePickValid = true;
  RCLCPP_INFO(logger_, "SENSOR_VALID name=measure_before_pick value=%.3fmm", value);
  transitionTo(State::PICK_FROM_SPECTROMETER, "valid spectrometer position before pick");
}

void StateMachine::updatePickFromSpectrometer(const Event &)
{
  const bool ok = runActionWithFeedback(
    "pick_from_spectrometer",
    [this]() {return robot_->pickFromSpectrometer(context_.spectrometerPositionBeforePick);});
  if (!ok) {
    return;
  }

  context_.cupPickedFromSpectrometer = true;
  context_.cupInGripper = true;
  context_.spectrometerOccupied = false;
  clearActiveAction();

  if (context_.pauseRequested) {
    context_.pausedFromState = State::CLEAN_CUP;
    transitionTo(State::PAUSED, "pause requested after pick_from_spectrometer");
  } else {
    transitionTo(State::CLEAN_CUP, "cup picked from spectrometer");
  }
}

void StateMachine::updateCleanCup(const Event &)
{
  const bool ok = runActionWithFeedback(
    "clean_cup",
    [this]() {return robot_->cleanCup();});
  if (!ok) {
    return;
  }

  context_.cupCleaned = true;
  context_.cupInGripper = true;
  outlet_status_[context_.outletId] = OutletStatus::WAITING_RETURN;
  clearActiveAction();

  if (context_.pauseRequested) {
    context_.pausedFromState = State::RETURN_CUP;
    transitionTo(State::PAUSED, "pause requested after clean_cup");
  } else {
    transitionTo(State::RETURN_CUP, "cup cleaned");
  }
}

void StateMachine::updateReturnCup(const Event &)
{
  const OutletId return_outlet = context_.outletId;
  const bool ok = runActionWithFeedback(
    "return_cup",
    [this, return_outlet]() {return robot_->returnCupToOutlet(return_outlet);});
  if (!ok) {
    return;
  }

  context_.cupReturned = true;
  context_.cupInGripper = false;
  outlet_status_[context_.outletId] = OutletStatus::WAITING_FILL;
  clearActiveAction();

  if (context_.pauseRequested) {
    context_.pausedFromState = State::COMPLETE_CYCLE;
    transitionTo(State::PAUSED, "pause requested after return_cup");
  } else {
    transitionTo(State::COMPLETE_CYCLE, "cup returned to original outlet");
  }
}

void StateMachine::updateCompleteCycle(const Event &)
{
  const double cycle_time =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() -
    context_.cycleStartTime).count();
  RCLCPP_INFO(
    logger_,
    "COMPLETE_CYCLE cycle=%lu outlet=%s place_laser=%.3fmm pick_laser=%.3fmm cycle_time=%.2fs",
    context_.cycleId,
    toString(context_.outletId).c_str(),
    context_.spectrometerPositionBeforePlace,
    context_.spectrometerPositionBeforePick,
    cycle_time);

  const auto keep_cycle_id = context_.cycleId;
  context_.clearTask(keep_cycle_id, State::COMPLETE_CYCLE, nowMs());
  context_.clearError();

  if (!pending_outlets_.empty()) {
    transitionTo(State::SELECT_TASK, "cycle complete and pending outlet exists");
  } else {
    transitionTo(State::WAIT_DISCHARGE, "cycle complete and queue empty");
  }
}

void StateMachine::updatePaused(const Event & event)
{
  if (event.type == EventType::RESUME_AUTO || event.type == EventType::START_AUTO) {
    resumeFromPaused();
    return;
  }

  if (!manual_mode_enabled_.load() && auto_mode_enabled_) {
    resumeFromPaused();
  }
}

void StateMachine::updateError(const Event & event)
{
  auto_mode_enabled_ = false;
  if (!error_stop_sent_) {
    const auto stop_result = robot_->stop();
    if (!stop_result.success) {
      RCLCPP_ERROR(logger_, "ERROR robot.stop failed: %s", stop_result.message.c_str());
    }
    error_stop_sent_ = true;
    publishError();
  }

  if (event.type == EventType::CLEAR_ERROR) {
    context_.clearError();
    publishError();
    RCLCPP_WARN(logger_, "CLEAR_ERROR: staying in ERROR until RESET_SYSTEM");
  }
}

void StateMachine::updateEstop(const Event & event)
{
  auto_mode_enabled_ = false;
  clearActiveAction();
  if (event.type == EventType::CLEAR_ESTOP || event.type == EventType::RESET_SYSTEM) {
    if (!sensors_->isEmergencyStopActive()) {
      estop_released_ = true;
      transitionTo(State::RESET, "ESTOP cleared and reset requested");
    } else {
      RCLCPP_WARN(logger_, "ESTOP reset rejected: hardware emergency stop still active");
    }
  }
}

void StateMachine::updateReset(const Event &)
{
  if (sensors_->isEmergencyStopActive()) {
    context_.setError("RESET rejected: emergency stop still active");
    transitionTo(State::ESTOP, context_.lastError);
    publishError();
    return;
  }

  const auto result = robot_->reset();
  if (!result.success) {
    fail("RESET failed: " + result.message);
    return;
  }

  error_stop_sent_ = false;
  estop_pressed_ = false;
  estop_released_ = true;
  auto_mode_enabled_ = false;
  manual_mode_enabled_.store(false);
  manual_step_requested_.store(false);
  clearActiveAction();

  if (context_.hasActiveTask && context_.recoveryMode == RecoveryMode::UNKNOWN) {
    RCLCPP_WARN(
      logger_,
      "RESET with unfinished task and UNKNOWN recovery; operator must confirm before auto resumes");
  }

  pending_outlets_.clear();
  outlet_status_[OutletId::OUTLET_1] = OutletStatus::WAITING_FILL;
  outlet_status_[OutletId::OUTLET_2] = OutletStatus::WAITING_FILL;
  context_.clearTask(context_.cycleId, State::RESET, nowMs());
  context_.clearError();
  publishError();
  RCLCPP_INFO(logger_, "RESET_RESULT success");
  transitionTo(State::IDLE, "reset complete");
}

bool StateMachine::transitionTo(State next, const std::string & reason)
{
  if (state_ == next) {
    return true;
  }

  if (!isTransitionAllowed(state_, next, context_)) {
    RCLCPP_WARN(
      logger_,
      "STATE_TRANSITION_REJECTED current=%s target=%s reason=%s",
      toString(state_).c_str(),
      toString(next).c_str(),
      reason.c_str());
    return false;
  }

  std::string precondition_reason;
  if (!canEnterStateUnlocked(next, context_, &precondition_reason)) {
    const std::string error = "Cannot enter " + toString(next) + ": " + precondition_reason;
    RCLCPP_ERROR(logger_, "STATE_PRECONDITION_FAILED %s", error.c_str());
    if (next != State::ERROR && state_ != State::ERROR) {
      fail(error);
    }
    return false;
  }

  onExit(state_);
  const State previous = state_;
  context_.previousState = previous;
  state_ = next;
  context_.currentState = next;
  context_.stateEnterTimeMs = nowMs();
  context_.stateEnterTime = std::chrono::steady_clock::now();
  context_.lastTransitionReason = reason;

  if (state_ == State::WAIT_DISCHARGE) {
    last_wait_notice_time_ = std::chrono::steady_clock::now();
  }

  RCLCPP_INFO(
    logger_,
    "STATE_CHANGE %s -> %s reason=%s",
    toString(previous).c_str(),
    toString(next).c_str(),
    reason.c_str());
  publishState();
  onEnter(state_);
  return true;
}

bool StateMachine::isTransitionAllowed(State from, State to, const CycleContext & ctx) const
{
  if (to == State::ESTOP || to == State::RESET) {
    return true;
  }
  switch (from) {
    case State::INIT:
      return to == State::IDLE || to == State::ERROR || to == State::ESTOP;
    case State::IDLE:
      return to == State::WAIT_DISCHARGE || to == State::PAUSED || to == State::RESET ||
             to == State::ERROR || to == State::ESTOP;
    case State::WAIT_DISCHARGE:
      return to == State::SELECT_TASK || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::SELECT_TASK:
      return to == State::PICK_FROM_OUTLET || to == State::WAIT_DISCHARGE ||
             to == State::ERROR || to == State::ESTOP;
    case State::PICK_FROM_OUTLET:
      return to == State::MEASURE_SPECTROMETER_BEFORE_PLACE ||
             to == State::WAIT_DISCHARGE || to == State::PAUSED ||
             to == State::ERROR || to == State::ESTOP;
    case State::MEASURE_SPECTROMETER_BEFORE_PLACE:
      return to == State::PLACE_TO_SPECTROMETER || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::PLACE_TO_SPECTROMETER:
      return to == State::START_DETECTION || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::START_DETECTION:
      return to == State::WAIT_DETECTION_DONE || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::WAIT_DETECTION_DONE:
      return to == State::MEASURE_SPECTROMETER_BEFORE_PICK || to == State::PAUSED ||
             to == State::ERROR || to == State::ESTOP;
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      return to == State::PICK_FROM_SPECTROMETER || to == State::PAUSED ||
             to == State::ERROR || to == State::ESTOP;
    case State::PICK_FROM_SPECTROMETER:
      return to == State::CLEAN_CUP || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::CLEAN_CUP:
      return to == State::RETURN_CUP || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::RETURN_CUP:
      return to == State::COMPLETE_CYCLE || to == State::PAUSED || to == State::ERROR ||
             to == State::ESTOP;
    case State::COMPLETE_CYCLE:
      return to == State::SELECT_TASK || to == State::WAIT_DISCHARGE || to == State::PAUSED ||
             to == State::ERROR || to == State::ESTOP;
    case State::PAUSED:
      return to == ctx.pausedFromState || to == State::IDLE || to == State::RESET ||
             to == State::ERROR || to == State::ESTOP;
    case State::ERROR:
      return to == State::RESET || to == State::ESTOP;
    case State::ESTOP:
      return to == State::RESET;
    case State::RESET:
      return to == State::IDLE || to == State::ERROR || to == State::ESTOP;
  }
  return false;
}

bool StateMachine::canEnterState(State target, const CycleContext & ctx) const
{
  return canEnterStateUnlocked(target, ctx, nullptr);
}

bool StateMachine::canEnterStateUnlocked(
  State target,
  const CycleContext & ctx,
  std::string * reason) const
{
  auto reject = [reason](const std::string & text) {
      if (reason) {
        *reason = text;
      }
      return false;
    };

  switch (target) {
    case State::INIT:
    case State::IDLE:
    case State::PAUSED:
    case State::ERROR:
      return true;
    case State::ESTOP:
      return true;
    case State::RESET:
      if (sensors_ && sensors_->isEmergencyStopActive()) {
        return reject("emergency stop still active");
      }
      return true;
    case State::WAIT_DISCHARGE:
      if (!auto_mode_enabled_) {
        return reject("autoMode is false");
      }
      if (ctx.hasActiveTask) {
        return reject("active task exists");
      }
      if (ctx.activeCommandId != 0) {
        return reject("automatic action still active");
      }
      return true;
    case State::SELECT_TASK:
      if (ctx.hasActiveTask) {
        return reject("active task exists");
      }
      if (pending_outlets_.empty()) {
        return reject("pendingOutlets is empty");
      }
      if (ctx.cupInGripper) {
        return reject("cup is in gripper");
      }
      if (ctx.spectrometerOccupied) {
        return reject("spectrometer is occupied");
      }
      return true;
    case State::PICK_FROM_OUTLET:
      if (!ctx.hasActiveTask) {
        return reject("no active task");
      }
      if (ctx.outletId == OutletId::NONE) {
        return reject("outletId is NONE");
      }
      if (outlet_status_.at(ctx.outletId) != OutletStatus::FILLED) {
        return reject("outlet is not FILLED");
      }
      if (ctx.cupInGripper || ctx.cupPickedFromOutlet) {
        return reject("cup already picked or in gripper");
      }
      return true;
    case State::MEASURE_SPECTROMETER_BEFORE_PLACE:
      return ctx.cupPickedFromOutlet && ctx.cupInGripper ?
             true : reject("cup has not been picked from outlet");
    case State::PLACE_TO_SPECTROMETER:
      if (!ctx.cupInGripper) {
        return reject("cup is not in gripper");
      }
      if (!ctx.spectrometerPositionBeforePlaceValid) {
        return reject("before-place spectrometer position is invalid");
      }
      if (ctx.spectrometerOccupied) {
        return reject("spectrometer already occupied");
      }
      return true;
    case State::START_DETECTION:
      if (!ctx.cupPlacedToSpectrometer || !ctx.spectrometerOccupied) {
        return reject("cup is not placed on spectrometer");
      }
      if (ctx.detectionStarted) {
        return reject("detection already started");
      }
      return true;
    case State::WAIT_DETECTION_DONE:
      if (!ctx.detectionStarted || !ctx.cupPlacedToSpectrometer || !ctx.spectrometerOccupied) {
        return reject("detection has not started with cup on spectrometer");
      }
      return true;
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      return ctx.detectionDone && ctx.spectrometerOccupied ?
             true : reject("detection is not done or spectrometer is not occupied");
    case State::PICK_FROM_SPECTROMETER:
      if (!ctx.detectionDone || !ctx.spectrometerOccupied) {
        return reject("detection not done or spectrometer empty");
      }
      if (!ctx.spectrometerPositionBeforePickValid) {
        return reject("before-pick spectrometer position is invalid");
      }
      if (ctx.cupInGripper) {
        return reject("cup already in gripper");
      }
      return true;
    case State::CLEAN_CUP:
      return ctx.cupPickedFromSpectrometer && ctx.cupInGripper ?
             true : reject("cup has not been picked from spectrometer");
    case State::RETURN_CUP:
      if (!ctx.cupCleaned || !ctx.cupInGripper) {
        return reject("cup is not cleaned in gripper");
      }
      if (ctx.outletId == OutletId::NONE) {
        return reject("outletId is NONE");
      }
      if (outlet_status_.at(ctx.outletId) != OutletStatus::WAITING_RETURN) {
        return reject("source outlet is not WAITING_RETURN");
      }
      return true;
    case State::COMPLETE_CYCLE:
      return ctx.cupReturned && !ctx.cupInGripper ?
             true : reject("cup has not been returned");
  }
  return reject("unknown state");
}

void StateMachine::onExit(State)
{
}

void StateMachine::onEnter(State state)
{
  if (state == State::MEASURE_SPECTROMETER_BEFORE_PLACE ||
    state == State::MEASURE_SPECTROMETER_BEFORE_PICK)
  {
    sensors_->beginSpectrometerMeasurement();
  }
  if (state == State::WAIT_DETECTION_DONE) {
    sensors_->startScanTracking();
  }
  if (state == State::WAIT_DISCHARGE) {
    const auto result = robot_->moveToOutletWait();
    if (!result.success) {
      fail("WAIT_DISCHARGE wait pose failed: " + result.message);
      return;
    }
  }
  if (state == State::ERROR) {
    auto_mode_enabled_ = false;
  }
  if (state == State::PAUSED) {
    auto_mode_enabled_ = false;
  }
}

void StateMachine::fail(const std::string & message)
{
  std::string final_message = message;
  const bool estop_active = estop_pressed_ || state_ == State::ESTOP ||
    (sensors_ && sensors_->isEmergencyStopActive());
  const bool cup_may_be_held = context_.cupInGripper ||
    message.rfind("CUP_HELD:", 0) == 0 ||
    message.find("message=CUP_HELD:") != std::string::npos;
  // INIT may start from an unknown physical pose. Never invent a collision-free
  // recovery path before the encoders have confirmed the commissioned Home.
  if (!estop_active && robot_ && state_ != State::INIT && !cup_may_be_held) {
    const auto recovery = robot_->recoverHomeAfterError();
    if (recovery.success) {
      RCLCPP_ERROR(logger_, "SAFETY_RECOVERY_OK %s", recovery.message.c_str());
      final_message += "; safety recovery: Home encoder-confirmed";
    } else {
      RCLCPP_FATAL(logger_, "SAFETY_RECOVERY_FAILED %s", recovery.message.c_str());
      final_message += "; SAFETY RECOVERY FAILED, KEEP ENABLED: " + recovery.message;
    }
  } else if (cup_may_be_held) {
    RCLCPP_ERROR(
      logger_, "CUP_HELD_RECOVERY_DEFERRED keep enabled at measured pose; no blind Home move");
    final_message += "; cup may be held: keep enabled, blind Home recovery suppressed";
  }
  context_.setError(final_message);
  RCLCPP_ERROR(
    logger_, "ERROR state=%s reason=%s", toString(state_).c_str(), final_message.c_str());
  publishError();
  transitionTo(State::ERROR, final_message);
}

void StateMachine::startAction(const std::string & action_name)
{
  context_.activeCommandId = next_command_id_++;
  context_.activeActionName = action_name;
  context_.activeCommandStartTimeMs = nowMs();
  RCLCPP_INFO(
    logger_,
    "ACTION_START name=%s commandId=%lu",
    action_name.c_str(),
    context_.activeCommandId);
}

void StateMachine::clearActiveAction()
{
  context_.activeCommandId = 0;
  context_.activeActionName.clear();
  context_.activeCommandStartTimeMs = 0;
}

bool StateMachine::runActionWithFeedback(const std::string & action_name, const ActionFn & action)
{
  startAction(action_name);
  const auto result = action();
  Event feedback;
  feedback.type = result.success ? EventType::ACTION_SUCCESS : EventType::ACTION_FAILED;
  feedback.source = "robot_actions";
  feedback.name = action_name;
  feedback.message = result.message;
  feedback.timestampMs = nowMs();
  feedback.commandId = context_.activeCommandId;
  feedback.actionName = action_name;

  if (!isCurrentActionFeedback(feedback)) {
    RCLCPP_WARN(
      logger_,
      "STALE_ACTION_EVENT name=%s commandId=%lu ignored activeName=%s activeCommandId=%lu",
      feedback.actionName.c_str(),
      feedback.commandId,
      context_.activeActionName.c_str(),
      context_.activeCommandId);
    return false;
  }

  if (!result.success) {
    handleActionFailure(feedback, result.message);
    return false;
  }

  RCLCPP_INFO(
    logger_,
    "ACTION_SUCCESS name=%s commandId=%lu message=%s",
    action_name.c_str(),
    feedback.commandId,
    result.message.c_str());
  return true;
}

bool StateMachine::isCurrentActionFeedback(const Event & event) const
{
  return event.commandId == context_.activeCommandId &&
         event.actionName == context_.activeActionName &&
         context_.activeCommandId != 0;
}

void StateMachine::handleActionFailure(const Event & event, const std::string & message)
{
  if (!isCurrentActionFeedback(event)) {
    RCLCPP_WARN(
      logger_,
      "STALE_ACTION_EVENT name=%s commandId=%lu ignored",
      event.actionName.c_str(),
      event.commandId);
    return;
  }
  if (event.actionName == "pick_from_outlet" &&
    message.rfind("NO_CUP_AT_OUTLET:", 0) == 0)
  {
    const auto outlet = context_.outletId;
    const auto cycle_id = context_.cycleId;
    outlet_status_[outlet] = OutletStatus::WAITING_FILL;
    context_.clearTask(cycle_id, State::PICK_FROM_OUTLET, nowMs());
    RCLCPP_WARN(
      logger_,
      "TASK_ABORTED_NO_CUP cycle=%lu outlet=%s message=%s",
      cycle_id,
      toString(outlet).c_str(),
      message.c_str());
    transitionTo(State::WAIT_DISCHARGE, message);
    return;
  }
  if (message.rfind("CUP_RELEASED:", 0) == 0) {
    context_.cupInGripper = false;
    if (event.actionName == "return_cup") {
      context_.cupReturned = true;
      outlet_status_[context_.outletId] = OutletStatus::WAITING_FILL;
    } else if (event.actionName == "place_to_spectrometer") {
      context_.cupPlacedToSpectrometer = true;
      context_.spectrometerOccupied = true;
    }
    RCLCPP_WARN(
      logger_, "CUP_RELEASE_CONFIRMED action=%s; Home recovery is permitted",
      event.actionName.c_str());
  }
  std::ostringstream out;
  out << "ACTION_FAILED name=" << event.actionName
      << " commandId=" << event.commandId
      << " message=" << message;
  fail(out.str());
}

bool StateMachine::completeCurrentAction(
  const Event & event,
  const std::string & expected_action_name)
{
  if (event.actionName != expected_action_name || !isCurrentActionFeedback(event)) {
    RCLCPP_WARN(
      logger_,
      "STALE_ACTION_EVENT expected=%s actual=%s commandId=%lu ignored",
      expected_action_name.c_str(),
      event.actionName.c_str(),
      event.commandId);
    return false;
  }
  clearActiveAction();
  return true;
}

bool StateMachine::measureSpectrometerPosition(
  const std::string & request_name,
  double & out_value,
  std::string & out_error,
  bool & out_waiting)
{
  RCLCPP_INFO(logger_, "SENSOR_REQUEST name=%s", request_name.c_str());
  const auto result = sensors_->readSpectrometerPosition();
  if (!result.success) {
    out_error = request_name + " position measurement invalid: " + result.message;
    return false;
  }
  if (!result.ready) {
    out_waiting = true;
    out_error = request_name + " " + result.message;
    return false;
  }
  if (!validateSpectrometerMeasurement(result.value)) {
    std::ostringstream out;
    out << request_name << " position value out of range: " << result.value << "mm";
    out_error = out.str();
    return false;
  }
  out_value = result.value;
  return true;
}

bool StateMachine::validateSpectrometerMeasurement(double value) const
{
  return value >= config_.spectrometerAxis.laserMinMm &&
         value <= config_.spectrometerAxis.laserMaxMm;
}

bool StateMachine::startDetectionWithFeedback()
{
  RCLCPP_INFO(logger_, "SPECTROMETER_START_REQUEST");
  const auto result = spectrometer_->startDetection();
  if (!result.success) {
    fail("SPECTROMETER_ERROR start failed: " + result.message);
    return false;
  }
  return true;
}

bool StateMachine::pauseCurrentStateOrDefer()
{
  switch (state_) {
    case State::IDLE:
    case State::WAIT_DISCHARGE:
    case State::MEASURE_SPECTROMETER_BEFORE_PLACE:
    case State::WAIT_DETECTION_DONE:
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      context_.pausedFromState = state_;
      context_.pauseRequested = false;
      transitionTo(State::PAUSED, "PAUSE_AUTO");
      return true;
    case State::PICK_FROM_OUTLET:
    case State::PLACE_TO_SPECTROMETER:
    case State::PICK_FROM_SPECTROMETER:
    case State::CLEAN_CUP:
    case State::RETURN_CUP:
      context_.pauseRequested = true;
      RCLCPP_WARN(logger_, "PAUSE_DEFERRED current=%s", toString(state_).c_str());
      return true;
    default:
      RCLCPP_WARN(logger_, "PAUSE_REJECTED current=%s", toString(state_).c_str());
      return false;
  }
}

bool StateMachine::resumeFromPaused()
{
  if (state_ != State::PAUSED) {
    return false;
  }
  const State target = context_.pausedFromState;
  auto_mode_enabled_ = true;
  manual_mode_enabled_.store(false);
  context_.pauseRequested = false;
  if (target == State::INIT || target == State::PAUSED || target == State::ERROR ||
    target == State::ESTOP || !canEnterState(target, context_))
  {
    fail("pause resume failed: invalid pausedFromState " + toString(target));
    return false;
  }
  return transitionTo(target, "RESUME_AUTO");
}

bool StateMachine::handleManualStateRequest(const Event & event)
{
  RCLCPP_WARN(
    logger_,
    "MANUAL_STATE_REQUEST current=%s target=%s force=%s reason=%s",
    toString(state_).c_str(),
    toString(event.targetState).c_str(),
    event.force ? "true" : "false",
    event.message.c_str());

  if (state_ == State::ESTOP && event.targetState != State::RESET) {
    RCLCPP_WARN(
      logger_, "STATE_REQUEST_REJECTED current=ESTOP target=%s",
      toString(event.targetState).c_str());
    return true;
  }
  if (state_ == State::ERROR && event.targetState != State::RESET &&
    event.targetState != State::ESTOP)
  {
    RCLCPP_WARN(
      logger_, "STATE_REQUEST_REJECTED current=ERROR target=%s",
      toString(event.targetState).c_str());
    return true;
  }
  if (event.targetState == State::PICK_FROM_SPECTROMETER &&
    !context_.spectrometerPositionBeforePickValid)
  {
    RCLCPP_WARN(
      logger_,
      "STATE_REQUEST_REJECTED target=PICK_FROM_SPECTROMETER reason=missing before-pick measurement");
    return true;
  }
  if (event.targetState == State::RETURN_CUP &&
    (context_.outletId == OutletId::NONE || !context_.cupCleaned))
  {
    RCLCPP_WARN(
      logger_,
      "STATE_REQUEST_REJECTED target=RETURN_CUP reason=missing outlet or cup not cleaned");
    return true;
  }

  if (!event.force && !isTransitionAllowed(state_, event.targetState, context_)) {
    RCLCPP_WARN(
      logger_,
      "STATE_REQUEST_REJECTED current=%s target=%s reason=not legal successor",
      toString(state_).c_str(),
      toString(event.targetState).c_str());
    return true;
  }

  if (!canEnterState(event.targetState, context_)) {
    RCLCPP_WARN(
      logger_,
      "STATE_REQUEST_REJECTED current=%s target=%s reason=precondition failed",
      toString(state_).c_str(),
      toString(event.targetState).c_str());
    return true;
  }
  transitionTo(event.targetState, "manual state request: " + event.message);
  return true;
}

std::optional<State> StateMachine::getNextExpectedState(
  State current,
  const CycleContext & context) const
{
  switch (current) {
    case State::WAIT_DISCHARGE:
      if (!pending_outlets_.empty()) {
        return State::SELECT_TASK;
      }
      return std::nullopt;
    case State::WAIT_DETECTION_DONE:
      if (context.detectionDone) {
        return State::MEASURE_SPECTROMETER_BEFORE_PICK;
      }
      return std::nullopt;
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      if (context.spectrometerPositionBeforePickValid) {
        return State::PICK_FROM_SPECTROMETER;
      }
      return std::nullopt;
    default:
      return std::nullopt;
  }
}

void StateMachine::checkTimeouts()
{
  if (state_ == State::WAIT_DISCHARGE || state_ == State::IDLE || state_ == State::PAUSED ||
    state_ == State::ERROR || state_ == State::ESTOP || state_ == State::RESET)
  {
    return;
  }

  const double elapsed = secondsInState();
  double timeout_sec = 0.0;
  std::string message;
  switch (state_) {
    case State::PICK_FROM_OUTLET:
      timeout_sec = config_.loop.actionTimeoutSec;
      message = "pick from outlet timeout";
      break;
    case State::MEASURE_SPECTROMETER_BEFORE_PLACE:
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 10000,
        "LASER_WAIT before-place elapsed=%.1fs; holding safely for stable standard position",
        elapsed);
      return;
    case State::PLACE_TO_SPECTROMETER:
      timeout_sec = config_.loop.actionTimeoutSec;
      message = "place to spectrometer timeout";
      break;
    case State::START_DETECTION:
      timeout_sec = config_.loop.spectrometerStartTimeoutSec;
      message = "start detection timeout";
      break;
    case State::WAIT_DETECTION_DONE:
      timeout_sec = config_.loop.detectionTimeoutSec;
      message = "detection timeout";
      break;
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 10000,
        "LASER_WAIT before-pick elapsed=%.1fs; holding safely for stable target position",
        elapsed);
      return;
    case State::PICK_FROM_SPECTROMETER:
      timeout_sec = config_.loop.actionTimeoutSec;
      message = "pick from spectrometer timeout";
      break;
    case State::CLEAN_CUP:
      timeout_sec = config_.loop.actionTimeoutSec;
      message = "clean cup timeout";
      break;
    case State::RETURN_CUP:
      timeout_sec = config_.loop.actionTimeoutSec;
      message = "return cup timeout";
      break;
    default:
      return;
  }

  if (timeout_sec > 0.0 && elapsed > timeout_sec) {
    if (state_ == State::WAIT_DETECTION_DONE) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *node_->get_clock(),
        30000,
        "DETECTION_WAIT elapsed=%.1fs exceeds %.1fs; holding safely until completion signal",
        elapsed,
        timeout_sec);
      return;
    }
    fail("TIMEOUT " + message);
  }
}

bool StateMachine::checkInvariant()
{
  auto invariantFail = [this](const std::string & reason) {
      if (state_ == State::ERROR || state_ == State::ESTOP || state_ == State::RESET) {
        return false;
      }
      context_.setError("Invariant violation: " + reason);
      RCLCPP_ERROR(logger_, "INVARIANT_FAILED %s", reason.c_str());
      publishError();
      transitionTo(State::ERROR, context_.lastError);
      return false;
    };

  if (context_.hasActiveTask && context_.outletId == OutletId::NONE) {
    return invariantFail("active task without outletId");
  }
  if (state_ == State::RETURN_CUP && context_.outletId == OutletId::NONE) {
    return invariantFail("RETURN_CUP without outletId");
  }
  if (state_ == State::PICK_FROM_SPECTROMETER &&
    !context_.spectrometerPositionBeforePickValid)
  {
    return invariantFail("PICK_FROM_SPECTROMETER without before-pick measurement");
  }
  if (context_.cupReturned && context_.cupInGripper) {
    return invariantFail("cup returned but still in gripper");
  }
  if (context_.spectrometerOccupied && !context_.cupPlacedToSpectrometer) {
    return invariantFail("spectrometer occupied but cup not placed");
  }
  if (context_.cupPickedFromSpectrometer && context_.spectrometerOccupied) {
    return invariantFail("cup picked from spectrometer but spectrometer still occupied");
  }
  if (context_.detectionDone && !context_.detectionStarted) {
    return invariantFail("detection done before detection started");
  }
  if (context_.cupCleaned && !context_.cupPickedFromSpectrometer) {
    return invariantFail("cup cleaned before picking from spectrometer");
  }
  if (context_.activeCommandId != 0 && context_.activeActionName.empty()) {
    return invariantFail("activeCommandId set without activeActionName");
  }
  if (state_ == State::ESTOP && context_.activeCommandId != 0) {
    return invariantFail("ESTOP with active command");
  }
  if (state_ == State::ERROR && auto_mode_enabled_) {
    return invariantFail("ERROR with autoMode true");
  }
  if (state_ == State::IDLE && context_.cupInGripper &&
    context_.recoveryMode == RecoveryMode::UNKNOWN)
  {
    return invariantFail("IDLE with cup in gripper and no recovery confirmation");
  }
  if (state_ == State::COMPLETE_CYCLE && !context_.cupReturned) {
    return invariantFail("COMPLETE_CYCLE without cupReturned");
  }
  return true;
}

double StateMachine::secondsInState() const
{
  return secondsSince(context_.stateEnterTimeMs);
}

double StateMachine::secondsSince(std::uint64_t start_time_ms) const
{
  if (start_time_ms == 0) {
    return 0.0;
  }
  const auto now = nowMs();
  return now > start_time_ms ? static_cast<double>(now - start_time_ms) / 1000.0 : 0.0;
}

std::string StateMachine::outletStatusJson() const
{
  std::ostringstream out;
  out << "{"
      << "\"OUTLET_1\":\"" << toString(outlet_status_.at(OutletId::OUTLET_1)) << "\","
      << "\"OUTLET_2\":\"" << toString(outlet_status_.at(OutletId::OUTLET_2)) << "\""
      << "}";
  return out.str();
}

std::string StateMachine::pendingOutletsJson() const
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < pending_outlets_.size(); ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "\"" << toString(pending_outlets_[i]) << "\"";
  }
  out << "]";
  return out.str();
}

void StateMachine::publishState()
{
  std_msgs::msg::String msg;
  msg.data = toString(state_);
  state_pub_->publish(msg);
}

void StateMachine::publishError()
{
  std_msgs::msg::String msg;
  msg.data = context_.lastError;
  error_pub_->publish(msg);
}

void StateMachine::publishContext()
{
  const auto laser = sensors_->laserSnapshot();
  const auto scan = sensors_->pollScanTracking();
  std::ostringstream out;
  out << "{"
      << "\"state\":\"" << toString(state_) << "\","
      << "\"previous_state\":\"" << toString(context_.previousState) << "\","
      << "\"cycle_id\":" << context_.cycleId << ","
      << "\"has_active_task\":" << (context_.hasActiveTask ? "true" : "false") << ","
      << "\"outlet\":\"" << toString(context_.outletId) << "\","
      << "\"cup_in_gripper\":" << (context_.cupInGripper ? "true" : "false") << ","
      << "\"cup_picked_from_outlet\":" << (context_.cupPickedFromOutlet ? "true" : "false") << ","
      << "\"cup_placed_to_spectrometer\":" <<
    (context_.cupPlacedToSpectrometer ? "true" : "false") << ","
      << "\"spectrometer_occupied\":" << (context_.spectrometerOccupied ? "true" : "false") << ","
      << "\"detection_started\":" << (context_.detectionStarted ? "true" : "false") << ","
      << "\"detection_done\":" << (context_.detectionDone ? "true" : "false") << ","
      << "\"cup_picked_from_spectrometer\":" <<
    (context_.cupPickedFromSpectrometer ? "true" : "false") << ","
      << "\"cup_cleaned\":" << (context_.cupCleaned ? "true" : "false") << ","
      << "\"cup_returned\":" << (context_.cupReturned ? "true" : "false") << ","
      << "\"spectrometer_position_before_place_valid\":"
      << (context_.spectrometerPositionBeforePlaceValid ? "true" : "false") << ","
      << "\"spectrometer_position_before_pick_valid\":"
      << (context_.spectrometerPositionBeforePickValid ? "true" : "false") << ","
      << "\"spectrometer_position_before_place_mm\":"
      << context_.spectrometerPositionBeforePlace << ","
      << "\"spectrometer_position_before_pick_mm\":"
      << context_.spectrometerPositionBeforePick << ","
      << "\"laser_filtered_valid\":" << (laser.filteredValid ? "true" : "false") << ","
      << "\"laser_filtered_mm\":" << laser.filteredMm << ","
      << "\"laser_stable\":" << (laser.stable ? "true" : "false") << ","
      << "\"laser_stable_mm\":" << laser.stableMm << ","
      << "\"laser_span_mm\":" << laser.spanMm << ","
      << "\"laser_scan_phase\":\"" << toString(scan.phase) << "\","
      << "\"laser_scan_farthest_mm\":" << scan.farthestMm << ","
      << "\"laser_scan_remaining_sec\":" << scan.remainingSec << ","
      << "\"active_command_id\":" << context_.activeCommandId << ","
      << "\"active_action_name\":\"" << jsonEscape(context_.activeActionName) << "\","
      << "\"pending_outlets\":" << pendingOutletsJson() << ","
      << "\"outlet_status\":" << outletStatusJson() << ","
      << "\"auto_mode\":" << (auto_mode_enabled_ ? "true" : "false") << ","
      << "\"manual_mode\":" << (manual_mode_enabled_.load() ? "true" : "false") << ","
      << "\"pause_requested\":" << (context_.pauseRequested ? "true" : "false") << ","
      << "\"paused_from_state\":\"" << toString(context_.pausedFromState) << "\","
      << "\"recovery_mode\":\"" << toString(context_.recoveryMode) << "\","
      << "\"last_transition_reason\":\"" << jsonEscape(context_.lastTransitionReason) << "\","
      << "\"error\":\"" << jsonEscape(context_.lastError) << "\""
      << "}";

  std_msgs::msg::String msg;
  msg.data = out.str();
  context_pub_->publish(msg);
}

void StateMachine::setupManualControlServices()
{
  manual_mode_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "/spectrometer_cell/manual_mode",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      manual_mode_enabled_.store(true);
      manual_step_requested_.store(false);
      response->success = true;
      response->message = "manual state-machine pause requested";
      RCLCPP_WARN(logger_, "%s", response->message.c_str());
      publishContext();
    });

  auto_mode_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "/spectrometer_cell/auto_mode",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      manual_mode_enabled_.store(false);
      manual_step_requested_.store(false);
      auto_mode_enabled_ = true;
      if (state_ == State::PAUSED) {
        resumeFromPaused();
      } else if (state_ == State::IDLE) {
        transitionTo(State::WAIT_DISCHARGE, "auto_mode service");
      }
      response->success = true;
      response->message = "automatic state-machine stepping enabled";
      RCLCPP_WARN(logger_, "%s", response->message.c_str());
      publishContext();
    });

  step_once_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "/spectrometer_cell/step_once",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      manual_mode_enabled_.store(true);
      manual_step_requested_.store(true);
      response->success = true;
      response->message = "queued one state-machine step";
      RCLCPP_INFO(logger_, "%s", response->message.c_str());
      publishContext();
    });
}

}  // namespace panthera_spectrometer_cell
