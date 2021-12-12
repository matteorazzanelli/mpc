#pragma once

#include "control/control.h"
#include "utils/utils.h"

namespace control {
  class PurePursuit : public Controller {
    private:
      double ld_;
      double k_;
      size_t closest_index_;
      double curvature_;
      double last_maneuver_;
      double max_control_;
      double length_;
      const utils::types::Path* ppath_;
      utils::types::PathPoint last_lokahead_point_;
    public:
      PurePursuit(const double& ld, const double& k, const double& max_control, const double& l, const double & dt);
      void init(const utils::types::Path& path);
      Control updateControl();
      void updateControlError(vehicle::Model* model, const utils::types::Pose& target);
  };
}