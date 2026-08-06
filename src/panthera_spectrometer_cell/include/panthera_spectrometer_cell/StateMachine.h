#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "panthera_spectrometer_cell/Config.h"
#include "panthera_spectrometer_cell/RobotActions.h"
#include "panthera_spectrometer_cell/Sensors.h"
#include "panthera_spectrometer_cell/Spectrometer.h"
#include "panthera_spectrometer_cell/Types.h"

namespace panthera_spectrometer_cell
{

class StateMachine
{
public:
  StateMachine(
    rclcpp::Node::SharedPtr node,
    WorkcellConfig config,
    std::shared_ptr<RobotActions> robot,
    std::shared_ptr<Sensors> sensors,
    std::shared_ptr<Spectrometer> spectrometer);

  void tick();
  State currentState() const;
  ActionResult recoverGripper();
  ActionResult reloadConfig(const WorkcellConfig & config);

private:
  using ActionFn = std::function<ActionResult()>;

  std::uint64_t nowMs() const;
  void initializeRuntimeState();

  std::vector<Event> collectEvents();
  Event popHighestPriorityEvent(std::vector<Event> & events) const;
  int eventPriority(EventType type) const;
  void logEvent(const Event & event) const;

  bool handleGlobalEvent(const Event & event);
  void handleDischargeEvent(const Event & event);
  void enqueueOutletIfAllowed(OutletId outlet, const std::string & source);
  bool pendingContains(OutletId outlet) const;

  void updateInit(const Event & event);
  void updateIdle(const Event & event);
  void updateWaitDischarge(const Event & event);
  void updateSelectTask(const Event & event);
  void updatePickFromOutlet(const Event & event);
  void updateMeasureBeforePlace(const Event & event);
  void updatePlaceToSpectrometer(const Event & event);
  void updateStartDetection(const Event & event);
  void updateWaitDetectionDone(const Event & event);
  void updateMeasureBeforePick(const Event & event);
  void updatePickFromSpectrometer(const Event & event);
  void updateCleanCup(const Event & event);
  void updateReturnCup(const Event & event);
  void updateCompleteCycle(const Event & event);
  void updatePaused(const Event & event);
  void updateError(const Event & event);
  void updateEstop(const Event & event);
  void updateReset(const Event & event);

  bool transitionTo(State next, const std::string & reason);
  bool isTransitionAllowed(State from, State to, const CycleContext & ctx) const;
  bool canEnterState(State target, const CycleContext & ctx) const;
  bool canEnterStateUnlocked(State target, const CycleContext & ctx, std::string * reason) const;
  void onExit(State state);
  void onEnter(State state);
  void fail(const std::string & message);

  bool runActionWithFeedback(const std::string & action_name, const ActionFn & action);
  bool isCurrentActionFeedback(const Event & event) const;
  void startAction(const std::string & action_name);
  void clearActiveAction();
  void handleActionFailure(const Event & event, const std::string & message);
  bool completeCurrentAction(const Event & event, const std::string & expected_action_name);

  bool measureSpectrometerPosition(
    const std::string & request_name,
    double & out_value,
    std::string & out_error,
    bool & out_waiting);
  bool validateSpectrometerMeasurement(double value) const;
  bool startDetectionWithFeedback();

  bool pauseCurrentStateOrDefer();
  bool resumeFromPaused();
  bool handleManualStateRequest(const Event & event);
  std::optional<State> getNextExpectedState(State current, const CycleContext & context) const;

  void checkTimeouts();
  bool checkInvariant();
  double secondsInState() const;
  double secondsSince(std::uint64_t start_time_ms) const;

  std::string outletStatusJson() const;
  std::string pendingOutletsJson() const;
  void publishState();
  void publishError();
  void publishContext();
  void setupManualControlServices();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  WorkcellConfig config_;
  std::shared_ptr<RobotActions> robot_;
  std::shared_ptr<Sensors> sensors_;
  std::shared_ptr<Spectrometer> spectrometer_;

  State state_{State::INIT};
  CycleContext context_;
  std::uint64_t next_cycle_id_{1};
  std::uint64_t next_command_id_{1000};
  bool error_stop_sent_{false};
  bool estop_pressed_{false};
  bool estop_released_{true};
  bool auto_mode_enabled_{true};
  std::atomic_bool tick_running_{false};
  std::atomic_bool manual_mode_enabled_{false};
  std::atomic_bool manual_step_requested_{false};
  std::chrono::steady_clock::time_point last_wait_notice_time_;
  std::chrono::steady_clock::time_point boot_time_;

  std::deque<OutletId> pending_outlets_;
  std::map<OutletId, OutletStatus> outlet_status_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr context_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_once_srv_;
};

}  // namespace panthera_spectrometer_cell
