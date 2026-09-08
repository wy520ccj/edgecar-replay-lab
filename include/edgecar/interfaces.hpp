#pragma once

#include "edgecar/types.hpp"

#include <cstddef>
#include <memory>
#include <optional>

namespace edgecar {

class IFrameSource {
 public:
  virtual ~IFrameSource() = default;
  virtual std::optional<Frame> next() = 0;
};

class IPerceptionBackend {
 public:
  virtual ~IPerceptionBackend() = default;
  virtual PerceptionResult infer(const Frame& frame) = 0;
};

class IPlanner {
 public:
  virtual ~IPlanner() = default;
  virtual ControlCommand plan(const VehicleState& state,
                              const PerceptionResult& perception) = 0;
};

class ISafetySupervisor {
 public:
  virtual ~ISafetySupervisor() = default;
  virtual SafetyDecision filter(const Frame& frame,
                                const PerceptionResult& perception,
                                const ControlCommand& requested,
                                std::int64_t cycle_time_ms) = 0;
  virtual void reset() = 0;
};

class IActuatorSink {
 public:
  virtual ~IActuatorSink() = default;
  virtual void send(const ControlCommand& command) = 0;
};

}  // namespace edgecar
