#include "model/vehicle.h"
#include <math.h>
namespace vehicle {
  Model::Model(const double& dt) : dt_(dt) {
    input_.v = 0.0;
    input_.w = 0.0;
    state_.position.x = 0.0;
    state_.position.y = 0.0;
    state_.heading = 0.0;
    state_.steering_angle = 0.0;
    input_.v = 0.0;
    input_.w = 0.0;
  }

  Model::Model(const double& dt, const Pose& state) : 
    dt_(dt), state_(state) {
    input_.v = 0.0;
    input_.w = 0.0;
  }

  ::utils::types::Pose Model::getState() {return state_;}
  ::utils::types::Control Model::getInput() {return input_;}

  Unicycle::Unicycle(const double &dt) : Model(dt){}
  Unicycle::Unicycle(const double &dt, const Pose& state) : Model(dt, state) {}

  Bicycle::Bicycle(const double &dt) : Model(dt){}
  Bicycle::Bicycle(const double &dt, const Pose& state) : Model(dt, state) {}

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