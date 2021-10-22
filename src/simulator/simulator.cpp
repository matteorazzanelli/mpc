
#include "simulator/simulator.h"
#include <limits>

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
    utils::types::Pose robot_pose;
    double eulerDist_w = std::numeric_limits<double>::max();
    utils::types::Path result;
    
    for (size_t i = 0; i < config_.simulation.iterations; i ++) {
      robot_pose = model->getState();
      controller->updateControlError(robot_pose, goal);
      utils::types::Control control = controller->updateControl();
      robot_pose = model->updateState(control);
      if (utils::math::eulerDistance(robot_pose.position,goal.position) < config_.posture.distance_tolerance)
        break;
      result.push_back(utils::types::PathPoint{
        .pose = robot_pose,
        .s = 0.0,
        .k = 0.0,
        .u = control
      });
    }

    return result;
  }
}