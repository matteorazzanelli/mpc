#include "model/vehicle.h"
#include <math.h>

namespace vehicle {
  Model::Model(const utils::config::Config& config) {
    input_.v = 0.0;
    input_.w = 0.0;
    state_.position.x = config.vehicle.x;
    state_.position.y = config.vehicle.y;
    state_.heading = config.vehicle.heading;
    state_.steering_angle = config.vehicle.steering;
    dt_ = config.simulation.dt;
  }

  ::utils::types::Pose Model::getState() {return state_;}
  ::utils::types::Control Model::getInput() {return input_;}

  Unicycle::Unicycle(const utils::config::Config& config) : Model(config) {}

  Bicycle::Bicycle(const utils::config::Config& config) : Model(config), length_(config.vehicle.length) {}

  ::utils::types::Pose Unicycle::updateState(const ::utils::types::Control& input) {
    // unicycle integration
    state_.position.x = state_.position.x + input.v * cos(state_.heading) * dt_;
    state_.position.y = state_.position.y + input.v * sin(state_.heading) * dt_;
    state_.heading = state_.heading + input.w * dt_;
    // update input
    input_ = input;
    return state_;
  }

  ::utils::types::Pose Bicycle::updateState(const ::utils::types::Control& input) {
    // bicycle integration
    state_.position.x = state_.position.x + input.v * cos(state_.heading) * dt_;
    state_.position.y = state_.position.y + input.v * sin(state_.heading) * dt_;
    state_.heading = state_.heading + input.v * tan(state_.steering_angle) / length_ * dt_;
    state_.steering_angle = state_.steering_angle + input_.w * dt_;
    // update input
    input_ = input;
    return state_;
  }
}