#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <set>

#include "panthera_motion/Catalog.hpp"
#include "panthera_motion/Compiler.hpp"

namespace
{

panthera_motion::MotionCatalog validCatalog()
{
  panthera_motion::MotionCatalog catalog;
  panthera_motion::PointDefinition start;
  start.name = "start";
  start.joints = {0.0, 0.01, 0.01, 0.0, 0.0, 0.0};
  catalog.points.emplace(start.name, start);

  panthera_motion::PointDefinition end;
  end.name = "end";
  end.joints = {0.0, 0.18, 0.18, 0.0, 0.0, 0.0};
  catalog.points.emplace(end.name, end);

  panthera_motion::RouteDefinition route;
  route.name = "route";
  route.start = "start";
  panthera_motion::SegmentDefinition segment;
  segment.name = "to_end";
  segment.to = "end";
  route.segments.push_back(segment);
  catalog.routes.emplace(route.name, route);
  return catalog;
}

TEST(MotionCatalog, ValidCatalogPasses)
{
  const auto result = validCatalog().validate();
  EXPECT_TRUE(result.success) << result.message;
}

TEST(MotionCatalog, MissingPointReferenceFails)
{
  auto catalog = validCatalog();
  catalog.routes.at("route").segments.front().to = "missing";
  const auto result = catalog.validate();
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("unknown point"), std::string::npos);
}

TEST(MotionCatalog, IkSeedCycleFails)
{
  auto catalog = validCatalog();
  catalog.points.at("start").ik_seed = "end";
  catalog.points.at("end").ik_seed = "start";
  const auto result = catalog.validate();
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("cycle"), std::string::npos);
}

TEST(MotionCatalog, LinearVerticalConstraintNeedsPoseTarget)
{
  auto catalog = validCatalog();
  auto & segment = catalog.routes.at("route").segments.front();
  segment.type = panthera_motion::SegmentType::LINEAR;
  segment.constraints.vertical_axis = "z";
  const auto result = catalog.validate();
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("needs a pose"), std::string::npos);
}

TEST(MotionCatalog, InvalidSegmentSpeedFails)
{
  auto catalog = validCatalog();
  catalog.routes.at("route").segments.front().velocity_scale = 1.1;
  const auto result = catalog.validate();
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("velocity/acceleration"), std::string::npos);
}

TEST(MotionCatalog, InvalidJerkLimitFails)
{
  auto catalog = validCatalog();
  catalog.defaults.max_jerk_rad_sec3 = 0.0;
  const auto result = catalog.validate();
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("invalid limits"), std::string::npos);

  catalog = validCatalog();
  catalog.defaults.cartesian_max_jerk_rad_sec3 = 0.0;
  EXPECT_FALSE(catalog.validate().success);
}

TEST(TrajectoryScaling, InvalidScaleIsRejected)
{
  // Compiler.hpp deliberately keeps speed scaling strict: production trajectories
  // may be slowed but never accelerated beyond their compiled base limits.
  EXPECT_THROW(
    panthera_motion::scaleTrajectory(trajectory_msgs::msg::JointTrajectory{}, 1.1),
    std::invalid_argument);
}

TEST(TrajectoryScaling, SlowsTimeVelocityAndAccelerationConsistently)
{
  trajectory_msgs::msg::JointTrajectory source;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start.sec = 2;
  point.velocities = {2.0};
  point.accelerations = {4.0};
  source.points.push_back(point);

  const auto scaled = panthera_motion::scaleTrajectory(source, 0.5);
  ASSERT_EQ(scaled.points.size(), 1u);
  EXPECT_EQ(scaled.points.front().time_from_start.sec, 4);
  EXPECT_EQ(scaled.points.front().time_from_start.nanosec, 0u);
  ASSERT_EQ(scaled.points.front().velocities.size(), 1u);
  ASSERT_EQ(scaled.points.front().accelerations.size(), 1u);
  EXPECT_DOUBLE_EQ(scaled.points.front().velocities.front(), 1.0);
  EXPECT_DOUBLE_EQ(scaled.points.front().accelerations.front(), 1.0);
  EXPECT_DOUBLE_EQ(panthera_motion::trajectoryDurationSec(scaled), 4.0);
}

TEST(TrajectoryStart, UsesMeasuredStationaryStateWithoutChangingLaterPoints)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {1.0, 2.0};
  start.velocities = {0.1, 0.2};
  start.accelerations = {0.3, 0.4};
  trajectory.points.push_back(start);
  trajectory_msgs::msg::JointTrajectoryPoint end;
  end.positions = {3.0, 4.0};
  end.time_from_start.nanosec = 50'000'000;
  trajectory.points.push_back(end);

  const double correction = panthera_motion::alignTrajectoryStart(
    trajectory, {1.04, 1.98});

  EXPECT_NEAR(correction, 0.04, 1e-12);
  EXPECT_EQ(trajectory.points.front().positions, (std::vector<double>{1.04, 1.98}));
  EXPECT_EQ(trajectory.points.front().velocities, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(trajectory.points.front().accelerations, (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(trajectory.points.back().positions, (std::vector<double>{3.0, 4.0}));
  EXPECT_EQ(trajectory.points.back().time_from_start.nanosec, 250'000'000u);
}

TEST(TrajectoryStart, RejectsDimensionMismatch)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1"};
  trajectory.points.emplace_back();
  trajectory.points.front().positions = {1.0};
  EXPECT_THROW(
    panthera_motion::alignTrajectoryStart(trajectory, {1.0, 2.0}),
    std::invalid_argument);
}

TEST(TrajectoryStart, PreservesCloseMitHoldingCommandForCartesianJog)
{
  std::vector<double> output;
  const auto result = panthera_motion::selectHoldingCommandStart(
    {1.000, 2.000}, {1.004, 1.997}, 0.05, output);

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(output, (std::vector<double>{1.004, 1.997}));
}

TEST(TrajectoryStart, RejectsMissingOrDivergentHoldingCommand)
{
  std::vector<double> output;
  EXPECT_FALSE(
    panthera_motion::selectHoldingCommandStart(
      {1.0, 2.0}, {}, 0.05, output).success);
  EXPECT_FALSE(
    panthera_motion::selectHoldingCommandStart(
      {1.0, 2.0}, {1.0, 2.2}, 0.05, output).success);
}

TEST(TrajectoryResume, StartsSettledAtNearestRemainingPoint)
{
  trajectory_msgs::msg::JointTrajectory source;
  source.joint_names = {"joint1"};
  for (int index = 0; index < 4; ++index) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {static_cast<double>(index)};
    point.velocities = {1.0};
    point.accelerations = {0.0};
    point.time_from_start.sec = index;
    source.points.push_back(point);
  }

  const auto resumed = panthera_motion::makeResumeTrajectory(source, {1.1}, 0.2);

  ASSERT_EQ(resumed.points.size(), 3u);
  EXPECT_EQ(resumed.points.front().positions, (std::vector<double>{1.1}));
  EXPECT_EQ(resumed.points.front().velocities, (std::vector<double>{0.0}));
  EXPECT_DOUBLE_EQ(panthera_motion::trajectoryDurationSec(resumed), 2.0);
  EXPECT_EQ(resumed.points.back().positions, (std::vector<double>{3.0}));
}

TEST(MotionState, PositionSlopeIgnoresStationaryEncoderJitter)
{
  const std::vector<double> times{0.00, 0.01, 0.02, 0.03, 0.04};
  const std::vector<std::vector<double>> positions{
    {1.0000, 2.0},
    {1.0006, 2.0},
    {0.9994, 2.0},
    {1.0006, 2.0},
    {1.0000, 2.0},
  };
  EXPECT_NEAR(panthera_motion::maxAbsPositionSlope(times, positions), 0.0, 1e-12);
}

TEST(MotionState, PositionSlopeKeepsSustainedMotion)
{
  const std::vector<double> times{0.00, 0.01, 0.02, 0.03, 0.04};
  const std::vector<std::vector<double>> positions{
    {1.0000, 2.0},
    {1.0008, 2.0},
    {1.0016, 2.0},
    {1.0024, 2.0},
    {1.0032, 2.0},
  };
  EXPECT_NEAR(panthera_motion::maxAbsPositionSlope(times, positions), 0.08, 1e-12);
}

TEST(TrajectoryDynamicsLimit, StretchesQuinticDynamicsConsistently)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1"};
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {0.0};
  start.velocities = {0.0};
  start.accelerations = {0.0};
  trajectory.points.push_back(start);
  auto end = start;
  end.positions = {1.0};
  end.time_from_start.sec = 1;
  trajectory.points.push_back(end);

  const auto result = panthera_motion::enforceTrajectoryDynamicsLimits(
    trajectory, {100.0}, {100.0}, 10.0);
  ASSERT_TRUE(result.success) << result.message;
  const double stretched_duration = panthera_motion::trajectoryDurationSec(trajectory);
  EXPECT_GT(stretched_duration, 1.0);
  EXPECT_DOUBLE_EQ(trajectory.points.front().velocities.front(), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.points.back().accelerations.front(), 0.0);
}

TEST(TrajectoryDynamicsLimit, BoundsVelocityBetweenWaypoints)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1"};
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {0.0};
  start.velocities = {0.0};
  start.accelerations = {0.0};
  trajectory.points.push_back(start);
  auto end = start;
  end.positions = {1.0};
  end.time_from_start.sec = 1;
  trajectory.points.push_back(end);

  const auto result = panthera_motion::enforceTrajectoryDynamicsLimits(
    trajectory, {1.0}, {100.0}, 1000.0);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_GT(panthera_motion::trajectoryDurationSec(trajectory), 1.8);
}

TEST(TrajectoryDynamicsLimit, RejectsIncompleteDynamics)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1"};
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {0.0};
  start.time_from_start.sec = 0;
  trajectory.points.push_back(start);
  auto end = start;
  end.positions = {1.0};
  end.time_from_start.sec = 1;
  trajectory.points.push_back(end);

  const auto result = panthera_motion::enforceTrajectoryDynamicsLimits(
    trajectory, {1.0}, {1.0}, 10.0);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("velocity"), std::string::npos);
}

TEST(ProductionCatalog, KeepsMinimalOperatorFacingProfile)
{
  const auto catalog =
    panthera_motion::MotionCatalog::loadFromFile(PANTHERA_TEST_CATALOG_PATH);

  const std::set<std::string> required_points{
    "outlet_1_grasp",
    "outlet_2_grasp",
    "spectrometer_place",
    "spectrometer_pick",
    "spectrometer_wait",
    "clean_dump",
    "brush_center",
  };
  std::size_t tunable_count = 0;
  for (const auto & item : catalog.points) {
    const auto & tags = item.second.tags;
    if (std::find(tags.begin(), tags.end(), "tunable") != tags.end()) {
      ++tunable_count;
    }
  }
  EXPECT_EQ(tunable_count, required_points.size());
  for (const auto & name : required_points) {
    const auto * point = catalog.findPoint(name);
    ASSERT_NE(point, nullptr) << name;
    EXPECT_TRUE(point->pose.has_value()) << name;
    EXPECT_NE(std::find(point->tags.begin(), point->tags.end(), "tunable"), point->tags.end());

    const auto * entry = catalog.findRoute("debug_safe_to_" + name);
    const auto * exit = catalog.findRoute("debug_" + name + "_to_safe");
    ASSERT_NE(entry, nullptr) << name;
    ASSERT_NE(exit, nullptr) << name;
    ASSERT_FALSE(entry->segments.empty()) << name;
    ASSERT_FALSE(exit->segments.empty()) << name;
    EXPECT_EQ(entry->start, "safe_joint_center") << name;
    EXPECT_EQ(entry->segments.back().to, name) << name;
    EXPECT_EQ(exit->start, name) << name;
    EXPECT_EQ(exit->segments.back().to, "safe_joint_center") << name;
  }

  EXPECT_NE(catalog.findRoute("home_to_safe_center"), nullptr);
  EXPECT_NE(catalog.findRoute("safe_center_to_home"), nullptr);
  const auto * home = catalog.findPoint("home_near");
  ASSERT_NE(home, nullptr);
  EXPECT_EQ(home->joints, std::vector<double>(6, 0.0));

  const std::set<std::string> required_routes{
    "home_to_outlet_1_grasp_smooth",
    "home_to_outlet_2_grasp_smooth",
    "outlet_wait_to_home_continuous",
    "outlet_wait_to_outlet_1_grasp",
    "outlet_wait_to_outlet_2_grasp",
    "outlet_1_grasp_to_spectrometer_place",
    "outlet_2_grasp_to_spectrometer_place",
    "outlet_1_grasp_to_spectrometer_hover",
    "outlet_2_grasp_to_spectrometer_hover",
    "spectrometer_place_to_wait",
    "spectrometer_sensor_place_to_wait",
    "spectrometer_wait_to_pick",
    "home_to_spectrometer_pick_hover_recovery",
    "spectrometer_pick_to_clean_dump",
    "spectrometer_pick_to_brush_entry_smooth",
    "spectrometer_pick_hover_to_brush_entry_smooth",
    "debug_spectrometer_pick_hover_to_safe",
    "spectrometer_pick_hover_to_clean_hover_recovery",
    "clean_dump_to_pour",
    "clean_dump_pour_shake_once",
    "clean_dump_pour_to_clean_hover",
    "clean_dump_pour_to_brush_entry",
    "brush_entry_to_center",
    "brush_center_to_entry",
    "brush_entry_to_clean_hover",
    "brush_center_to_clean_hover_smooth",
    "brush_entry_to_outlet_1_return_smooth",
  };
  for (const auto & name : required_routes) {
    EXPECT_NE(catalog.findRoute(name), nullptr) << name;
  }

  const auto * outlet_route = catalog.findRoute("outlet_wait_to_outlet_1_grasp");
  ASSERT_NE(outlet_route, nullptr);
  ASSERT_FALSE(outlet_route->segments.empty());
  const auto & final_segment = outlet_route->segments.back();
  EXPECT_EQ(final_segment.type, panthera_motion::SegmentType::LINEAR);
  EXPECT_EQ(final_segment.constraints.vertical_axis, "z");
  EXPECT_TRUE(final_segment.constraints.keep_orientation);
  ASSERT_TRUE(final_segment.velocity_scale.has_value());
  EXPECT_DOUBLE_EQ(*final_segment.velocity_scale, 1.0);
  EXPECT_DOUBLE_EQ(catalog.defaults.max_jerk_rad_sec3, 300.0);

  const auto & transfer_segment = outlet_route->segments.front();
  ASSERT_TRUE(transfer_segment.velocity_scale.has_value());
  EXPECT_DOUBLE_EQ(*transfer_segment.velocity_scale, 1.0);

  const auto * home_to_outlet_2 =
    catalog.findRoute("home_to_outlet_2_grasp_smooth");
  ASSERT_NE(home_to_outlet_2, nullptr);
  EXPECT_EQ(home_to_outlet_2->start, "home_near");
  ASSERT_FALSE(home_to_outlet_2->segments.empty());
  const auto & outlet_2_final_segment = home_to_outlet_2->segments.back();
  EXPECT_EQ(outlet_2_final_segment.to, "outlet_2_grasp");
  EXPECT_EQ(outlet_2_final_segment.type, panthera_motion::SegmentType::LINEAR);
  EXPECT_EQ(outlet_2_final_segment.constraints.vertical_axis, "z");
  EXPECT_TRUE(outlet_2_final_segment.constraints.keep_orientation);

  const auto * outlet_wait_to_home =
    catalog.findRoute("outlet_wait_to_home_continuous");
  ASSERT_NE(outlet_wait_to_home, nullptr);
  EXPECT_EQ(outlet_wait_to_home->start, "outlet_wait");
  ASSERT_FALSE(outlet_wait_to_home->segments.empty());
  EXPECT_EQ(outlet_wait_to_home->segments.back().to, "home_near");

  const auto * direct_return =
    catalog.findRoute("brush_entry_to_outlet_1_return_smooth");
  ASSERT_NE(direct_return, nullptr);
  ASSERT_GE(direct_return->segments.size(), 4u);
  EXPECT_EQ(direct_return->segments[0].to, "brush_entry_clear_high");
  EXPECT_EQ(direct_return->segments[0].type, panthera_motion::SegmentType::LINEAR);
  EXPECT_EQ(direct_return->segments[0].constraints.vertical_axis, "z");
  EXPECT_TRUE(direct_return->segments[0].constraints.keep_orientation);
  EXPECT_EQ(direct_return->segments[1].to, "brush_clear_high");
  EXPECT_EQ(direct_return->segments[2].to, "outlet_1_hover");
  EXPECT_EQ(direct_return->segments.back().to, "outlet_1_return");
  const auto * outlet_1_grasp = catalog.findPoint("outlet_1_grasp");
  const auto * outlet_1_return = catalog.findPoint("outlet_1_return");
  const auto * outlet_2_grasp = catalog.findPoint("outlet_2_grasp");
  const auto * outlet_2_return = catalog.findPoint("outlet_2_return");
  ASSERT_NE(outlet_1_grasp, nullptr);
  ASSERT_NE(outlet_1_return, nullptr);
  ASSERT_NE(outlet_2_grasp, nullptr);
  ASSERT_NE(outlet_2_return, nullptr);
  ASSERT_TRUE(outlet_1_grasp->pose.has_value());
  ASSERT_TRUE(outlet_1_return->pose.has_value());
  ASSERT_TRUE(outlet_2_grasp->pose.has_value());
  ASSERT_TRUE(outlet_2_return->pose.has_value());
  EXPECT_NEAR(
    outlet_1_return->pose->xyz[2] - outlet_1_grasp->pose->xyz[2], 0.010, 1e-9);
  EXPECT_NEAR(
    outlet_2_return->pose->xyz[2] - outlet_2_grasp->pose->xyz[2], 0.010, 1e-9);
  for (const auto & segment : direct_return->segments) {
    EXPECT_NE(segment.to, "clean_dump_pour");
    EXPECT_NE(segment.to, "clean_dump");
  }

  const auto * clean_route =
    catalog.findRoute("spectrometer_pick_to_brush_entry_smooth");
  ASSERT_NE(clean_route, nullptr);
  const auto dump_entry = std::find_if(
    clean_route->segments.begin(), clean_route->segments.end(),
    [](const auto & segment) {return segment.to == "clean_dump";});
  ASSERT_NE(dump_entry, clean_route->segments.end());
  EXPECT_TRUE(dump_entry->stop_at_end);
  EXPECT_EQ(clean_route->segments.back().type, panthera_motion::SegmentType::LINEAR);
  EXPECT_EQ(clean_route->segments.back().to, "brush_entry");
  const auto pour_segment = std::find_if(
    clean_route->segments.begin(), clean_route->segments.end(),
    [](const auto & segment) {return segment.name == "rotate_wrist_to_pour";});
  ASSERT_NE(pour_segment, clean_route->segments.end());
  ASSERT_TRUE(pour_segment->velocity_scale.has_value());
  ASSERT_TRUE(pour_segment->acceleration_scale.has_value());
  EXPECT_DOUBLE_EQ(*pour_segment->velocity_scale, 1.0);
  EXPECT_DOUBLE_EQ(*pour_segment->acceleration_scale, 0.50);

  const auto * dump = catalog.findPoint("clean_dump");
  const auto * brush_entry = catalog.findPoint("brush_entry");
  ASSERT_NE(dump, nullptr);
  ASSERT_NE(brush_entry, nullptr);
  ASSERT_TRUE(dump->pose.has_value());
  ASSERT_TRUE(brush_entry->pose.has_value());
  EXPECT_NEAR(dump->pose->xyz[0], -0.05874, 1e-9);
  EXPECT_NEAR(brush_entry->pose->xyz[0] - dump->pose->xyz[0], 0.100, 1e-9);
}

TEST(ProductionCatalog, UsesFullVelocityScaleOutsideSafetyRoutes)
{
  const auto catalog =
    panthera_motion::MotionCatalog::loadFromFile(PANTHERA_TEST_CATALOG_PATH);
  EXPECT_DOUBLE_EQ(catalog.defaults.velocity_scale, 1.0);

  const std::set<std::string> safety_routes{
    "home_to_safe_center",
    "safe_center_to_home",
    "safe_center_to_outlet_wait",
    "home_to_outlet_wait_continuous",
    "outlet_wait_to_safe_center",
    "outlet_wait_to_home_continuous",
  };
  for (const auto & item : catalog.routes) {
    const auto & name = item.first;
    const auto & route = item.second;
    const bool exempt =
      name.rfind("debug_", 0) == 0 ||
      name.find("_recovery") != std::string::npos ||
      name.find("_commission") != std::string::npos ||
      safety_routes.count(name) != 0;
    if (exempt) {
      continue;
    }
    EXPECT_DOUBLE_EQ(route.velocity_scale, 1.0) << name;
    for (const auto & segment : route.segments) {
      if (segment.velocity_scale) {
        EXPECT_DOUBLE_EQ(*segment.velocity_scale, 1.0) << name << "/" << segment.name;
      }
    }
  }
}

}  // namespace
