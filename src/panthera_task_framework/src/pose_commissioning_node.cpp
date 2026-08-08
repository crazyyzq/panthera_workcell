#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

struct PlanningSettings
{
  double planning_time_sec{10.0};
  int planning_attempts{10};
  double velocity_scale{0.08};
  double acceleration_scale{0.08};
};

struct OrientationCandidate
{
  std::string name;
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
};

struct TargetCandidate
{
  std::string name;
  std::string station;
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  std::vector<std::string> orientations;
};

struct ProbeConfig
{
  std::string arm_group{"arm"};
  std::string base_frame{"base_link"};
  std::string hand_frame{"gripper_center"};
  PlanningSettings planning;
  std::map<std::string, std::vector<double>> safe_joint_poses;
  std::map<std::string, OrientationCandidate> orientations;
  std::vector<TargetCandidate> targets;
};

double readDouble(const YAML::Node & node, const std::string & key, double default_value)
{
  return node && node[key] ? node[key].as<double>() : default_value;
}

int readInt(const YAML::Node & node, const std::string & key, int default_value)
{
  return node && node[key] ? node[key].as<int>() : default_value;
}

std::string readString(
  const YAML::Node & node,
  const std::string & key,
  const std::string & default_value)
{
  return node && node[key] ? node[key].as<std::string>() : default_value;
}

std::array<double, 3> readVec3(const YAML::Node & node, const std::string & label)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(label + " must be a 3 element array");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

std::vector<double> readDoubleVector(const YAML::Node & node, const std::string & label)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error(label + " must be a numeric array");
  }
  std::vector<double> values;
  values.reserve(node.size());
  for (const auto & value : node) {
    values.push_back(value.as<double>());
  }
  return values;
}

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

std::string defaultConfigFile()
{
  return ament_index_cpp::get_package_share_directory("panthera_task_framework") +
         "/config/station_pose_commissioning.yaml";
}

ProbeConfig loadProbeConfig(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);

  ProbeConfig config;
  config.arm_group = readString(root, "arm_group", config.arm_group);
  config.base_frame = readString(root, "base_frame", config.base_frame);
  config.hand_frame = readString(root, "hand_frame", config.hand_frame);

  const auto planning = root["planning"];
  config.planning.planning_time_sec =
    readDouble(planning, "planning_time_sec", config.planning.planning_time_sec);
  config.planning.planning_attempts =
    readInt(planning, "planning_attempts", config.planning.planning_attempts);
  config.planning.velocity_scale =
    readDouble(planning, "velocity_scale", config.planning.velocity_scale);
  config.planning.acceleration_scale =
    readDouble(planning, "acceleration_scale", config.planning.acceleration_scale);

  const auto safe_poses = root["safe_joint_poses"];
  if (!safe_poses || !safe_poses.IsMap()) {
    throw std::runtime_error("safe_joint_poses must be a map");
  }
  for (const auto & item : safe_poses) {
    const std::string name = item.first.as<std::string>();
    config.safe_joint_poses[name] = readDoubleVector(item.second, "safe_joint_poses." + name);
  }

  const auto orientations = root["orientations"];
  if (!orientations || !orientations.IsMap()) {
    throw std::runtime_error("orientations must be a map");
  }
  for (const auto & item : orientations) {
    OrientationCandidate orientation;
    orientation.name = item.first.as<std::string>();
    orientation.rpy = readVec3(item.second["rpy"], "orientations." + orientation.name + ".rpy");
    config.orientations[orientation.name] = orientation;
  }

  const auto targets = root["targets"];
  if (!targets || !targets.IsSequence()) {
    throw std::runtime_error("targets must be a sequence");
  }
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const auto item = targets[i];
    TargetCandidate target;
    target.name = readString(item, "name", "");
    target.station = readString(item, "station", "");
    target.xyz = readVec3(item["xyz"], "targets[" + std::to_string(i) + "].xyz");
    if (target.name.empty()) {
      throw std::runtime_error("targets[" + std::to_string(i) + "].name is required");
    }
    if (!item["orientations"] || !item["orientations"].IsSequence()) {
      throw std::runtime_error("targets[" + target.name + "].orientations must be a sequence");
    }
    for (const auto & orientation : item["orientations"]) {
      target.orientations.push_back(orientation.as<std::string>());
    }
    config.targets.push_back(target);
  }

  return config;
}

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

std::string vectorToJson(const std::vector<double> & values)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << values[i];
  }
  out << "]";
  return out.str();
}

template<typename T>
T getOrDeclareParameter(
  rclcpp::Node & node,
  const std::string & name,
  const T & default_value)
{
  if (node.has_parameter(name)) {
    rclcpp::Parameter parameter;
    if (node.get_parameter(name, parameter)) {
      return parameter.get_value<T>();
    }
  }
  return node.declare_parameter<T>(name, default_value);
}

}  // namespace

class PoseCommissioningNode : public rclcpp::Node
{
public:
  PoseCommissioningNode()
  : Node("panthera_pose_commissioning",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    config_file_ = getOrDeclareParameter<std::string>(*this, "config_file", defaultConfigFile());
    result_file_ = getOrDeclareParameter<std::string>(*this, "result_file", "");
    stop_after_run_ = getOrDeclareParameter<bool>(*this, "stop_after_run", true);
  }

  bool run()
  {
    try {
      config_ = loadProbeConfig(config_file_);
    } catch (const std::exception & exc) {
      RCLCPP_FATAL(
        get_logger(), "failed to load probe config '%s': %s",
        config_file_.c_str(), exc.what());
      return false;
    }

    try {
      arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        config_.arm_group);
      arm_->setEndEffectorLink(config_.hand_frame);
      arm_->setPoseReferenceFrame(config_.base_frame);
      arm_->setPlanningTime(config_.planning.planning_time_sec);
      arm_->setNumPlanningAttempts(config_.planning.planning_attempts);
      arm_->setMaxVelocityScalingFactor(config_.planning.velocity_scale);
      arm_->setMaxAccelerationScalingFactor(config_.planning.acceleration_scale);
    } catch (const std::exception & exc) {
      RCLCPP_FATAL(get_logger(), "failed to initialize MoveGroupInterface: %s", exc.what());
      return false;
    }

    std::ofstream result_stream;
    if (!result_file_.empty()) {
      result_stream.open(result_file_, std::ios::out | std::ios::trunc);
      if (!result_stream.is_open()) {
        RCLCPP_FATAL(get_logger(), "failed to open result_file: %s", result_file_.c_str());
        return false;
      }
    }

    const auto joint_names = arm_->getJointNames();
    std::size_t success_count = 0;
    std::size_t fail_count = 0;

    RCLCPP_INFO(
      get_logger(),
      "pose commissioning start config=%s starts=%zu targets=%zu hand_frame=%s",
      config_file_.c_str(),
      config_.safe_joint_poses.size(),
      config_.targets.size(),
      config_.hand_frame.c_str());

    for (const auto & start_item : config_.safe_joint_poses) {
      const auto & start_name = start_item.first;
      const auto & start_positions = start_item.second;
      if (start_positions.size() != joint_names.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "skip start '%s': joint count %zu does not match group joint count %zu",
          start_name.c_str(),
          start_positions.size(),
          joint_names.size());
        continue;
      }

      for (const auto & target : config_.targets) {
        for (const auto & orientation_name : target.orientations) {
          const auto orientation_it = config_.orientations.find(orientation_name);
          if (orientation_it == config_.orientations.end()) {
            RCLCPP_ERROR(
              get_logger(),
              "skip target '%s': unknown orientation '%s'",
              target.name.c_str(),
              orientation_name.c_str());
            continue;
          }

          const bool ok =
            planFromStart(start_name, start_positions, target, orientation_it->second);
          if (ok) {
            ++success_count;
          } else {
            ++fail_count;
          }
          writeResult(
            result_stream, start_name, start_positions, target, orientation_it->second,
            ok);
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "pose commissioning finished success=%zu failed=%zu result_file=%s",
      success_count,
      fail_count,
      result_file_.empty() ? "<none>" : result_file_.c_str());
    return success_count > 0;
  }

  bool stopAfterRun() const
  {
    return stop_after_run_;
  }

private:
  bool planFromStart(
    const std::string & start_name,
    const std::vector<double> & start_positions,
    const TargetCandidate & target,
    const OrientationCandidate & orientation)
  {
    moveit_msgs::msg::RobotState start_state;
    start_state.joint_state.name = arm_->getJointNames();
    start_state.joint_state.position = start_positions;
    arm_->setStartState(start_state);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = config_.base_frame;
    pose.header.stamp = now();
    pose.pose.position.x = target.xyz[0];
    pose.pose.position.y = target.xyz[1];
    pose.pose.position.z = target.xyz[2];
    pose.pose.orientation = quaternionFromRpy(
      orientation.rpy[0],
      orientation.rpy[1],
      orientation.rpy[2]);

    arm_->setPoseTarget(pose, config_.hand_frame);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = arm_->plan(plan);
    arm_->clearPoseTargets();

    const bool ok = result == moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(
      get_logger(),
      "[PROBE] %s start=%s station=%s target=%s orientation=%s xyz=[%.5f, %.5f, %.5f]",
      ok ? "OK" : "FAIL",
      start_name.c_str(),
      target.station.c_str(),
      target.name.c_str(),
      orientation.name.c_str(),
      target.xyz[0],
      target.xyz[1],
      target.xyz[2]);
    return ok;
  }

  void writeResult(
    std::ofstream & stream,
    const std::string & start_name,
    const std::vector<double> & start_positions,
    const TargetCandidate & target,
    const OrientationCandidate & orientation,
    bool success)
  {
    if (!stream.is_open()) {
      return;
    }
    stream << std::fixed << std::setprecision(6)
           << "{"
           << "\"success\":" << (success ? "true" : "false") << ","
           << "\"start\":\"" << jsonEscape(start_name) << "\","
           << "\"start_joints\":" << vectorToJson(start_positions) << ","
           << "\"station\":\"" << jsonEscape(target.station) << "\","
           << "\"target\":\"" << jsonEscape(target.name) << "\","
           << "\"orientation\":\"" << jsonEscape(orientation.name) << "\","
           << "\"xyz\":[" << target.xyz[0] << "," << target.xyz[1] << "," << target.xyz[2] << "],"
           << "\"rpy\":[" << orientation.rpy[0] << "," << orientation.rpy[1] << "," <<
      orientation.rpy[2] << "]"
           << "}\n";
  }

  std::string config_file_;
  std::string result_file_;
  bool stop_after_run_{true};
  ProbeConfig config_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PoseCommissioningNode>();
  const bool ok = node->run();

  if (!node->stopAfterRun()) {
    rclcpp::spin(node);
  }

  node.reset();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
