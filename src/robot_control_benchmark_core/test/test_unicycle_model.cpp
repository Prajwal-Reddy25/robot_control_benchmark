#include <cmath>
#include <gtest/gtest.h>
#include "robot_control_benchmark_core/unicycle_model.hpp"

using robot_control_benchmark_core::Command;
using robot_control_benchmark_core::State;

TEST(UnicycleModel, StraightMotion)
{
  const auto result = robot_control_benchmark_core::integrate_exact(State{}, Command{1.0, 0.0}, 0.5);
  EXPECT_NEAR(result.x, 0.5, 1e-12);
  EXPECT_NEAR(result.y, 0.0, 1e-12);
  EXPECT_NEAR(result.yaw, 0.0, 1e-12);
}

TEST(UnicycleModel, CircularMotion)
{
  const auto result = robot_control_benchmark_core::integrate_exact(State{}, Command{1.0, 1.0}, M_PI_2);
  EXPECT_NEAR(result.x, 1.0, 1e-12);
  EXPECT_NEAR(result.y, 1.0, 1e-12);
  EXPECT_NEAR(result.yaw, M_PI_2, 1e-12);
}

TEST(UnicycleModel, RejectsInvalidStep)
{
  EXPECT_THROW(robot_control_benchmark_core::integrate_exact(State{}, Command{}, 0.0), std::invalid_argument);
}

TEST(UnicycleModel, EulerJacobians)
{
  const auto [a, b] = robot_control_benchmark_core::linearize_euler(
    State{0.0, 0.0, M_PI_2}, Command{2.0, 0.0}, 0.1);
  EXPECT_NEAR(a[2], -0.2, 1e-12);
  EXPECT_NEAR(a[5], 0.0, 1e-12);
  EXPECT_NEAR(b[2], 0.1, 1e-12);
  EXPECT_NEAR(b[5], 0.1, 1e-12);
}
