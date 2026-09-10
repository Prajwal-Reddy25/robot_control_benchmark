#include "robot_control_benchmark_core/unicycle_model.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control_benchmark_core
{

double wrap_angle(double angle) noexcept
{
  constexpr double pi = 3.141592653589793238462643383279502884;
  constexpr double two_pi = 2.0 * pi;
  angle = std::fmod(angle + pi, two_pi);
  if (angle < 0.0) {
    angle += two_pi;
  }
  return angle - pi;
}

State integrate_exact(const State & state, const Command & command, double dt)
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("dt must be finite and positive");
  }
  State next = state;
  if (std::abs(command.angular) < 1e-9) {
    next.x += command.linear * dt * std::cos(state.yaw);
    next.y += command.linear * dt * std::sin(state.yaw);
  } else {
    const double next_yaw = state.yaw + command.angular * dt;
    next.x += command.linear / command.angular * (std::sin(next_yaw) - std::sin(state.yaw));
    next.y -= command.linear / command.angular * (std::cos(next_yaw) - std::cos(state.yaw));
  }
  next.yaw = wrap_angle(state.yaw + command.angular * dt);
  return next;
}

std::pair<std::array<double, 9>, std::array<double, 6>>
linearize_euler(const State & state, const Command & command, double dt)
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("dt must be finite and positive");
  }
  const double s = std::sin(state.yaw);
  const double c = std::cos(state.yaw);
  const std::array<double, 9> a{
    1.0, 0.0, -dt * command.linear * s,
    0.0, 1.0,  dt * command.linear * c,
    0.0, 0.0, 1.0};
  const std::array<double, 6> b{
    dt * c, 0.0,
    dt * s, 0.0,
    0.0, dt};
  return {a, b};
}

}  // namespace robot_control_benchmark_core

