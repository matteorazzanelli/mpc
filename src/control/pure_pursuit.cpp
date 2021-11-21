#include "control/pure_pursuit.h"
#include <glog/logging.h>

namespace control {
  PurePursuit::PurePursuit(const double& ld, const double& k, const double & dt) : 
    Controller(dt), ld_(ld), k_(k) {}

  void PurePursuit::init(const utils::types::Path& path) {
    path_ = path;
    last_lokahead_point_ = path.front();
    closest_index_ = 0;
    curvature_ = 0.0;
  }
  
  Control PurePursuit::updateControl() {
    // We control the (steering) angular rate: omega = v * curvature
    return utils::types::Control {
      .v = 0.01,
      .w = 0.01 * curvature_
    };
  }

  void PurePursuit::updateControlError(vehicle::Model* model, const utils::types::Pose& target) {
    // Update lookahead distance wrt velocity
    double robot_velocity = model->getInput().v;
    double adaptive_ld = k_ * robot_velocity + ld_;
    // vinesmsuic.github.io/2020/09/29/robotics-purepersuit
    utils::types::Pose robot_pose = model->getState();
    // Find the path point closest to the vehicle
    const utils::types::PathPoint* start = utils::path::closestPoint(closest_index_, robot_pose.position, path_);
    // Find the lookahead point
    for(size_t i = closest_index_; i < path_.size(); i ++){
      const utils::types::PathPoint* goal = utils::path::lookaheadPoint(path_.at(i), path_.at(i+1), robot_pose.position, path_, adaptive_ld, last_lokahead_point_);
      if(goal) {
        last_lokahead_point_ = *goal;
        break;
      }
    }
    // Cross track distance
    double a = -std::tan(robot_pose.heading);
    double b = -1;
    double c = std::tan(robot_pose.heading)*robot_pose.position.x - robot_pose.position.y;
    double x = utils::math::pointLineDistance(a,b,c,last_lokahead_point_.pose.position.x,last_lokahead_point_.pose.position.y);
    // Check the side wrt the path
    double side = std::sin(robot_pose.heading) * (last_lokahead_point_.pose.position.x - robot_pose.position.x) - 
                  std::cos(robot_pose.heading) * (last_lokahead_point_.pose.position.y - robot_pose.position.y);
    double sign = static_cast<double>(utils::math::sign(side));
    
    // Calculate the curvature of the arc to the lookahead point
    curvature_ = sign * (2.0 * x / std::pow(adaptive_ld,2)); // = 1/R

    // Alternative
    // double desired_yaw = atan2((last_lokahead_point_.pose.position.y - robot_pose.position.y),(last_lokahead_point_.pose.position.x - robot_pose.position.x));
    // double diff_orientation = utils::math::shortestAngularDistance(robot_pose.heading,desired_yaw);
    // curvature_ = 2.0 * std::sin(diff_orientation) / adaptive_ld;

    // R    = ld/(2*sin(alpha))
    // 1/R  = (2*sin(alpha))/ld = (2*x)/(ld**2)
    //delta = atan2(L/R) = atan2((L*2*x)/(ld*ld)) = atan2(ld*curvature_)

    // If control the delta:
    // delta = std::min( std::atan2(2 * x * L / ld*ld, 1.0), delta_max_ );
  }
}