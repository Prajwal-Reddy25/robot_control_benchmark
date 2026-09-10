#pragma once

#include <string>
#include "robot_control_benchmark_core/unicycle_model.hpp"

namespace robot_control_benchmark_core
{

struct Reference
{
  State pose{};
  Command feedforward{};
};

/// Minimal interface for native controllers or adapters.
class Controller
{
public:
  virtual ~Controller() = default;
  virtual std::string name() const = 0;
  virtual void reset() = 0;
  virtual Command compute(const State & measured, const Reference & reference, double dt) = 0;
};

}  // namespace robot_control_benchmark_core

