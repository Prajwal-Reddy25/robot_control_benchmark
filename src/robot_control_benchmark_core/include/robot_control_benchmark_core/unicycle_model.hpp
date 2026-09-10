#pragma once

#include <array>
#include <stdexcept>
#include <utility>

namespace robot_control_benchmark_core
{

struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command
{
  double linear{0.0};
  double angular{0.0};
};

/// Wrap an angle to [-pi, pi).
double wrap_angle(double angle) noexcept;

/// Zero-order-hold exact integration for constant v and omega over dt.
State integrate_exact(const State & state, const Command & command, double dt);

/// Jacobians of forward Euler dynamics, returned row-major.
std::pair<std::array<double, 9>, std::array<double, 6>>
linearize_euler(const State & state, const Command & command, double dt);

}  // namespace robot_control_benchmark_core

