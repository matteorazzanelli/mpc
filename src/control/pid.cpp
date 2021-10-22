
#include "control/pid.h"
#include <glog/logging.h>

namespace control {
  Pid::Pid(const double& kp, const double& kd, const double& ki, 
          const double& max_err, const double & dt, utils::types::PIDType pid_type, utils::types::ControlType control_type) : Controller(dt),
    kp_(kp), kd_(kd), ki_(ki), max_error_(max_err), error_(0.0), last_error_(0.0), integral_error_(0.0), 
    pid_type_(pid_type), control_type_(control_type) {}

  bool Pid::resetErrors() {
    error_ = 0.0;
    last_error_ = 0.0;
    integral_error_ = 0.0;

    return true;
  }

  bool Pid::setTuningParameters(const utils::config::PIDControl& parameters) {
    kp_ = parameters.kp;
    kd_ = parameters.kd;
    ki_ = parameters.ki;
    max_error_ = parameters.max_err;

    return true;
  }

  void Pid::updateControlError(const utils::types::Pose& pose, const utils::types::Pose& target) {
    if (pid_type_ == PIDType::ANGULAR) {
      double losAngle = utils::math::losAngle(pose.position, target.position);
      double angDist = utils::math::shortestAngularDistance(pose.heading, losAngle);
      error_ = angDist;
    }
    if (pid_type_ == PIDType::LATERAL) {
      double latDist = utils::math::lateralDistance(pose, target);
      error_ = latDist;
    }
  }

  utils::types::Control Pid::updateControl() {
    double err_d = (error_ - last_error_) / dt_;
    last_error_ = error_;
    integral_error_ += error_ * dt_;
    integral_error_ = integral_error_ > max_error_ ? max_error_ : integral_error_;
    integral_error_ = integral_error_ < -max_error_ ? -max_error_ : integral_error_;
    if (control_type_ == ControlType::ANGULAR) {
      return utils::types::Control{
        .v = 0.01,
        .w = (kp_*error_ + kd_*err_d + ki_*integral_error_)
      };
    }
    if (control_type_ == ControlType::LINEAR) {
      return utils::types::Control{
        .v = (kp_*error_ + kd_*err_d + ki_*integral_error_),
        .w = 0.01
      };
    }
    // LOG(ERROR) << "Control type not set correctly.";
    CHECK(false) << "Control type not supported.";
  }
}