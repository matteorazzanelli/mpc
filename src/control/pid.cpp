
#include "control/pid.h"
#include <glog/logging.h>

namespace control {
  Pid::Pid(const double& kp, const double& kd, const double& ki, 
          const double& max_integral_error, const double& max_control, const double & dt, 
          utils::types::PIDType pid_type, utils::types::ControlType control_type) : Controller(dt),
    kp_(kp), kd_(kd), ki_(ki), max_integral_error_(max_integral_error), max_control_(max_control), error_(0.0), last_error_(0.0), integral_error_(0.0), 
    pid_type_(pid_type), control_type_(control_type) {}

  void Pid::updateControlError(vehicle::Model* model, const utils::types::Pose& target) {
    if (pid_type_ == PIDType::ANGULAR) {
      double losAngle = utils::math::losAngle(model->getState().position, target.position);
      double angDist = utils::math::shortestAngularDistance(model->getState().heading, losAngle);
      error_ = angDist;
    }
    if (pid_type_ == PIDType::LATERAL) {
      double latDist = utils::math::lateralDistance(model->getState(), target);
      error_ = latDist;
    }
  }

  utils::types::Control Pid::updateControl() {
    double err_d = (error_ - last_error_) / dt_;
    last_error_ = error_;
    integral_error_ += error_ * dt_;
    integral_error_ = fabs(integral_error_) > max_integral_error_ ? utils::math::sign(integral_error_) * max_integral_error_ : integral_error_;
    double control = (kp_*error_ + kd_*err_d + ki_*integral_error_);
    control = fabs(control) > max_control_ ? utils::math::sign(control) * max_control_ : control;
    if (control_type_ == ControlType::ANGULAR) {
      return utils::types::Control{
        .v = 0.01,
        .w = control
      };
    }
    if (control_type_ == ControlType::LINEAR) {
      return utils::types::Control{
        .v = control,
        .w = 0.01
      };
    }
    // LOG(ERROR) << "Control type not set correctly.";
    CHECK(false) << "Control type not supported.";
  }
}