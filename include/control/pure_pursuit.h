#pragma once

#include "control/control.h"
#include "utils/utils.h"

namespace control {
  class PurePursuit : public Controller {
    private:
      double ld_;
      double k_;
      size_t closest_index_ = 0;
      double curvature_ = 0.0;
      double dt_ = 0.1;
      double last_maneuver_ = 0.0;
      utils::types::Path path_;
      utils::types::PathPoint last_lokahead_point_;
    public:
      PurePursuit(const double& ld, const double& k, const double & dt);
      void init(const utils::types::Path& path);
      Control updateControl();
      void updateControlError(vehicle::Model* model, const utils::types::Pose& target);
  };
}