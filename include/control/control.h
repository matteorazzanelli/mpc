#pragma once
#include "utils/utils.h"

using namespace utils::types;

namespace control {
  class Controller {
    public:
      Controller(const double& dt) : dt_(dt) {}
      virtual Control updateControl() = 0;
      virtual void updateControlError(const utils::types::Pose& pose, const utils::types::Pose& target) = 0;
    protected:
      double dt_ = 0.0;
      Control c_; 
  };
}