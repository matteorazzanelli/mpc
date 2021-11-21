
#include "simulator/simulator.h"
#include <limits>
#include <glog/logging.h>

namespace simulator {
  Simulator::Simulator(const utils::config::Config& config) : config_(config) {}

  utils::types::Path Simulator::run(vehicle::Model* model, control::Controller* controller, const std::string& mission) {
    
    utils::types::Pose goal;
    if (mission == "Posture") {
      goal = utils::types::Pose {
      .position = utils::types::Position {
        .x = config_.posture.x,
        .y = config_.posture.y
      },
      .heading = config_.posture.heading,
      .steering_angle = 0.0
      };
    }
    else {
      LOG(ERROR) << "Mission type not supported.";
    }

    utils::types::Path result(config_.simulation.iterations);
    for (size_t i = 0; i < config_.simulation.iterations; i ++) {

      controller->updateControlError(model, goal);
      
      utils::types::Control control = controller->updateControl();
      
      utils::types::Pose robot_pose = model->updateState(control);
      
      result[i] = utils::types::PathPoint{
        .pose = robot_pose,
        .s = 0.0,
        .k = 0.0
      };
    }

    return result;
  }
}