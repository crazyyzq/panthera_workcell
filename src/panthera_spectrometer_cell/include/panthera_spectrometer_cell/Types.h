#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace panthera_spectrometer_cell
{

enum class State
{
  INIT,
  IDLE,
  WAIT_DISCHARGE,
  SELECT_TASK,
  PICK_FROM_OUTLET,
  MEASURE_SPECTROMETER_BEFORE_PLACE,
  PLACE_TO_SPECTROMETER,
  START_DETECTION,
  WAIT_DETECTION_DONE,
  MEASURE_SPECTROMETER_BEFORE_PICK,
  PICK_FROM_SPECTROMETER,
  CLEAN_CUP,
  RETURN_CUP,
  COMPLETE_CYCLE,
  PAUSED,
  ERROR,
  ESTOP,
  RESET
};

enum class OutletId
{
  NONE,
  OUTLET_1,
  OUTLET_2
};

enum class OutletStatus
{
  WAITING_FILL,
  FILLED,
  CUP_TAKEN,
  WAITING_RETURN,
  ERROR
};

enum class EventType
{
  NONE,
  START_AUTO,
  PAUSE_AUTO,
  RESUME_AUTO,
  RESET_SYSTEM,
  CLEAR_ERROR,
  ESTOP_PRESSED,
  ESTOP_RELEASED,
  CLEAR_ESTOP,
  OUTLET_1_DISCHARGE_DONE,
  OUTLET_2_DISCHARGE_DONE,
  CLEAR_DISCHARGE_SIGNAL,
  SPECTROMETER_STARTED,
  SPECTROMETER_BUSY,
  SPECTROMETER_DETECTION_DONE,
  SPECTROMETER_ERROR,
  MANUAL_NEXT_STEP,
  MANUAL_STATE_REQUEST,
  ACTION_SUCCESS,
  ACTION_FAILED,
  SENSOR_VALID,
  SENSOR_INVALID,
  TIMEOUT
};

enum class RecoveryMode
{
  UNKNOWN,
  CUP_IN_GRIPPER,
  CUP_ON_SPECTROMETER,
  CUP_AT_OUTLET,
  CUP_REMOVED_BY_OPERATOR
};

struct Event
{
  EventType type{EventType::NONE};
  std::string source{"state_machine"};
  std::string name;
  std::string message;
  std::uint64_t timestampMs{0};
  std::uint64_t commandId{0};
  std::string actionName;
  State targetState{State::IDLE};
  bool force{false};
  double sensorValue{0.0};
  bool sensorValid{false};
};

struct ActionResult
{
  bool success{false};
  std::string message;

  static ActionResult ok(const std::string & text = "ok")
  {
    return ActionResult{true, text};
  }

  static ActionResult fail(const std::string & text)
  {
    return ActionResult{false, text};
  }
};

struct MeasurementResult
{
  bool success{false};
  bool ready{false};
  double value{0.0};
  double span{0.0};
  std::string message;

  static MeasurementResult ok(double measured_value, const std::string & text = "ok")
  {
    return MeasurementResult{true, true, measured_value, 0.0, text};
  }

  static MeasurementResult waiting(const std::string & text)
  {
    return MeasurementResult{true, false, 0.0, 0.0, text};
  }

  static MeasurementResult fail(const std::string & text)
  {
    return MeasurementResult{false, false, 0.0, 0.0, text};
  }
};

struct DischargeResult
{
  bool ready{false};
  OutletId outletId{OutletId::NONE};
  std::string message;
};

struct CycleContext
{
  std::uint64_t cycleId{0};

  bool hasActiveTask{false};
  OutletId outletId{OutletId::NONE};

  bool cupInGripper{false};
  bool cupPickedFromOutlet{false};
  bool cupPlacedToSpectrometer{false};
  bool spectrometerOccupied{false};
  bool detectionStarted{false};
  bool detectionDone{false};
  bool cupPickedFromSpectrometer{false};
  bool cupCleaned{false};
  bool cupReturned{false};

  bool spectrometerPositionBeforePlaceValid{false};
  bool spectrometerPositionBeforePickValid{false};
  double spectrometerPositionBeforePlace{0.0};
  double spectrometerPositionBeforePick{0.0};

  std::uint64_t activeCommandId{0};
  std::string activeActionName;
  std::uint64_t activeCommandStartTimeMs{0};

  State previousState{State::INIT};
  State currentState{State::INIT};
  State pausedFromState{State::INIT};

  bool pauseRequested{false};
  bool manualOverride{false};

  std::uint64_t stateEnterTimeMs{0};
  std::uint64_t cycleStartTimeMs{0};
  std::chrono::steady_clock::time_point cycleStartTime;
  std::chrono::steady_clock::time_point stateEnterTime;

  std::string lastError;
  std::string errorMessage;
  std::string lastTransitionReason;
  RecoveryMode recoveryMode{RecoveryMode::UNKNOWN};

  void clearTask(std::uint64_t keep_cycle_id, State state, std::uint64_t now_ms)
  {
    cycleId = keep_cycle_id;
    hasActiveTask = false;
    outletId = OutletId::NONE;
    cupInGripper = false;
    cupPickedFromOutlet = false;
    cupPlacedToSpectrometer = false;
    spectrometerOccupied = false;
    detectionStarted = false;
    detectionDone = false;
    cupPickedFromSpectrometer = false;
    cupCleaned = false;
    cupReturned = false;
    spectrometerPositionBeforePlaceValid = false;
    spectrometerPositionBeforePickValid = false;
    spectrometerPositionBeforePlace = 0.0;
    spectrometerPositionBeforePick = 0.0;
    activeCommandId = 0;
    activeActionName.clear();
    activeCommandStartTimeMs = 0;
    pauseRequested = false;
    manualOverride = false;
    recoveryMode = RecoveryMode::UNKNOWN;
    currentState = state;
    stateEnterTimeMs = now_ms;
    stateEnterTime = std::chrono::steady_clock::now();
    cycleStartTimeMs = now_ms;
    cycleStartTime = stateEnterTime;
  }

  void beginTask(std::uint64_t new_cycle_id, OutletId outlet, State state, std::uint64_t now_ms)
  {
    clearTask(new_cycle_id, state, now_ms);
    hasActiveTask = true;
    outletId = outlet;
    cycleStartTimeMs = now_ms;
    cycleStartTime = std::chrono::steady_clock::now();
  }

  void clearError()
  {
    lastError.clear();
    errorMessage.clear();
  }

  void setError(const std::string & message)
  {
    lastError = message;
    errorMessage = message;
  }

  void resetForCycle(std::uint64_t next_cycle_id)
  {
    clearTask(next_cycle_id, State::IDLE, 0);
    clearError();
  }
};

using Vec3 = std::array<double, 3>;

inline std::string toString(State state)
{
  switch (state) {
    case State::INIT:
      return "INIT";
    case State::IDLE:
      return "IDLE";
    case State::WAIT_DISCHARGE:
      return "WAIT_DISCHARGE";
    case State::SELECT_TASK:
      return "SELECT_TASK";
    case State::PICK_FROM_OUTLET:
      return "PICK_FROM_OUTLET";
    case State::MEASURE_SPECTROMETER_BEFORE_PLACE:
      return "MEASURE_SPECTROMETER_BEFORE_PLACE";
    case State::PLACE_TO_SPECTROMETER:
      return "PLACE_TO_SPECTROMETER";
    case State::START_DETECTION:
      return "START_DETECTION";
    case State::WAIT_DETECTION_DONE:
      return "WAIT_DETECTION_DONE";
    case State::MEASURE_SPECTROMETER_BEFORE_PICK:
      return "MEASURE_SPECTROMETER_BEFORE_PICK";
    case State::PICK_FROM_SPECTROMETER:
      return "PICK_FROM_SPECTROMETER";
    case State::CLEAN_CUP:
      return "CLEAN_CUP";
    case State::RETURN_CUP:
      return "RETURN_CUP";
    case State::COMPLETE_CYCLE:
      return "COMPLETE_CYCLE";
    case State::PAUSED:
      return "PAUSED";
    case State::ERROR:
      return "ERROR";
    case State::ESTOP:
      return "ESTOP";
    case State::RESET:
      return "RESET";
  }
  return "UNKNOWN";
}

inline std::string toString(OutletId outlet)
{
  switch (outlet) {
    case OutletId::NONE:
      return "NONE";
    case OutletId::OUTLET_1:
      return "OUTLET_1";
    case OutletId::OUTLET_2:
      return "OUTLET_2";
  }
  return "UNKNOWN";
}

inline std::string toString(OutletStatus status)
{
  switch (status) {
    case OutletStatus::WAITING_FILL:
      return "WAITING_FILL";
    case OutletStatus::FILLED:
      return "FILLED";
    case OutletStatus::CUP_TAKEN:
      return "CUP_TAKEN";
    case OutletStatus::WAITING_RETURN:
      return "WAITING_RETURN";
    case OutletStatus::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

inline std::string toString(EventType type)
{
  switch (type) {
    case EventType::NONE:
      return "NONE";
    case EventType::START_AUTO:
      return "START_AUTO";
    case EventType::PAUSE_AUTO:
      return "PAUSE_AUTO";
    case EventType::RESUME_AUTO:
      return "RESUME_AUTO";
    case EventType::RESET_SYSTEM:
      return "RESET_SYSTEM";
    case EventType::CLEAR_ERROR:
      return "CLEAR_ERROR";
    case EventType::ESTOP_PRESSED:
      return "ESTOP_PRESSED";
    case EventType::ESTOP_RELEASED:
      return "ESTOP_RELEASED";
    case EventType::CLEAR_ESTOP:
      return "CLEAR_ESTOP";
    case EventType::OUTLET_1_DISCHARGE_DONE:
      return "OUTLET_1_DISCHARGE_DONE";
    case EventType::OUTLET_2_DISCHARGE_DONE:
      return "OUTLET_2_DISCHARGE_DONE";
    case EventType::CLEAR_DISCHARGE_SIGNAL:
      return "CLEAR_DISCHARGE_SIGNAL";
    case EventType::SPECTROMETER_STARTED:
      return "SPECTROMETER_STARTED";
    case EventType::SPECTROMETER_BUSY:
      return "SPECTROMETER_BUSY";
    case EventType::SPECTROMETER_DETECTION_DONE:
      return "SPECTROMETER_DETECTION_DONE";
    case EventType::SPECTROMETER_ERROR:
      return "SPECTROMETER_ERROR";
    case EventType::MANUAL_NEXT_STEP:
      return "MANUAL_NEXT_STEP";
    case EventType::MANUAL_STATE_REQUEST:
      return "MANUAL_STATE_REQUEST";
    case EventType::ACTION_SUCCESS:
      return "ACTION_SUCCESS";
    case EventType::ACTION_FAILED:
      return "ACTION_FAILED";
    case EventType::SENSOR_VALID:
      return "SENSOR_VALID";
    case EventType::SENSOR_INVALID:
      return "SENSOR_INVALID";
    case EventType::TIMEOUT:
      return "TIMEOUT";
  }
  return "UNKNOWN";
}

inline std::string toString(RecoveryMode mode)
{
  switch (mode) {
    case RecoveryMode::UNKNOWN:
      return "UNKNOWN";
    case RecoveryMode::CUP_IN_GRIPPER:
      return "CUP_IN_GRIPPER";
    case RecoveryMode::CUP_ON_SPECTROMETER:
      return "CUP_ON_SPECTROMETER";
    case RecoveryMode::CUP_AT_OUTLET:
      return "CUP_AT_OUTLET";
    case RecoveryMode::CUP_REMOVED_BY_OPERATOR:
      return "CUP_REMOVED_BY_OPERATOR";
  }
  return "UNKNOWN";
}

inline OutletId outletFromString(const std::string & text)
{
  if (text == "OUTLET_1" || text == "outlet_1" || text == "1") {
    return OutletId::OUTLET_1;
  }
  if (text == "OUTLET_2" || text == "outlet_2" || text == "2") {
    return OutletId::OUTLET_2;
  }
  return OutletId::NONE;
}

}  // namespace panthera_spectrometer_cell
