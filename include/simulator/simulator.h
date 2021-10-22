#pragma once

#include <memory>
#include "utils/utils.h"
#include "model/vehicle.h"
#include "control/control.h"
// #include "control/pid.h"
// // #include "simulator/task_manager.h"

namespace simulator {
  class Simulator {
    private:
      utils::config::Config config_;
      
    public:
      Simulator(const utils::config::Config& config);
      utils::types::Path run(vehicle::Model* model, control::Controller* controller, const std::string& mission);
  };
}