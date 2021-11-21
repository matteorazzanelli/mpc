#pragma once
#include "utils/utils.h"
#include "model/vehicle.h"
using namespace utils::types;

namespace control {
  class Controller {
    public:
      Controller(const double& dt) : dt_(dt) {}
      virtual Control updateControl() = 0;
      virtual void updateControlError(vehicle::Model* model, const utils::types::Pose& target) = 0;
    protected:
      double dt_ = 0.0;
      Control c_; 
  };
}