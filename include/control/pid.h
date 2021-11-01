#pragma once
#include "control/control.h"

namespace control {
  class Pid : public Controller {
    private:
      double kp_;
      double kd_;
      double ki_;
      double error_;
      double integral_error_;
      double max_integral_error_;
      double max_control_;
      double last_error_;
      ControlType control_type_;
      PIDType pid_type_;
    public:
      Pid(const double& kp, const double& kd, const double& ki, const double& max_err, const double & dt, utils::types::PIDType pid_type, utils::types::ControlType control_type  = ControlType::ANGULAR);
      bool resetErrors();
      bool setTuningParameters(const utils::config::PIDControl& parameters);
      Control updateControl();
      void updateControlError(const utils::types::Pose& pose, const utils::types::Pose& target);
      double getControlError();
  };
}