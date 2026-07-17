#include <gtest/gtest.h>

#include <stdexcept>

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

}  // namespace
