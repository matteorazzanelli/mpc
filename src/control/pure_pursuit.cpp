#include "control/pure_pursuit.h"
#include <glog/logging.h>

namespace control {
  PurePursuit::PurePursuit(const double& ld, const double& k, const double& max_control, const double& l, const double & dt) : 
    Controller(dt), ld_(ld), k_(k), max_control_(max_control), length_(l), closest_index_(0), last_maneuver_(0.0),
    curvature_(0.0) {}

  void PurePursuit::init(const utils::types::Path& path) {
    ppath_ = &path;
    last_lokahead_point_ = path.front();
    closest_index_ = 0;
    curvature_ = 0.0;
    last_maneuver_ = 0.0;
  }
  
  Control PurePursuit::updateControl() {
    // compute steering angle: for unicycle length_ = 0
    double steering_angle = std::atan2(length_*curvature_,1.0);
    double omega = (steering_angle - last_maneuver_) / dt_;
    omega = fabs(omega) > max_control_ ? utils::math::sign(omega)*max_control_ : omega;
    // We control the (steering) angular rate: omega = v * curvature
    auto control = utils::types::Control {
      .v = 0.01,
      .w = 0.01 * curvature_ + omega // FIXME: omega shuould be of the previous step not current
    };
    last_maneuver_ = steering_angle;
    return control;
  }

  void PurePursuit::updateControlError(vehicle::Model* model, const utils::types::Pose& target) {
    // Update lookahead distance wrt velocity
    double robot_velocity = model->getInput().v;
    double adaptive_ld = k_ * robot_velocity + ld_;
    // vinesmsuic.github.io/2020/09/29/robotics-purepersuit
    utils::types::Pose robot_pose = model->getState();
    // Find the path point closest to the vehicle
    const utils::types::PathPoint* start = utils::path::closestPoint(closest_index_, robot_pose.position, *ppath_);
    // Find the lookahead point
    for(size_t i = closest_index_; i < ppath_->size()-1; i ++){
      const utils::types::PathPoint* goal = utils::path::lookaheadPoint(ppath_->at(i), ppath_->at(i+1), robot_pose.position, *ppath_, adaptive_ld, last_lokahead_point_);
      if(goal) {
        last_lokahead_point_ = *goal;
        break;
      }
    }
    // Cross track distance
    double a = -std::tan(robot_pose.heading);
    double b = 1;
    double c = std::tan(robot_pose.heading)*robot_pose.position.x - robot_pose.position.y;
    double x = utils::math::pointLineDistance(a,b,c,last_lokahead_point_.pose.position.x,last_lokahead_point_.pose.position.y);
    // Find the side wrt path with the cross product of (Xl-Xr, Yl-Yr) and n (n is the normal to the line)
    // If we define dx = Xr and dy = Yr, then the normals are (-dy, dx) and (dy, -dx) = (-sin, cos) and (sin, -cos).
    double side = - std::sin(robot_pose.heading) * (last_lokahead_point_.pose.position.x - robot_pose.position.x)
                  + std::cos(robot_pose.heading) * (last_lokahead_point_.pose.position.y - robot_pose.position.y);
    double sign = static_cast<double>(utils::math::sign(side));
    
    // Calculate the curvature of the arc to the lookahead point
    curvature_ = sign * (2.0 * x / std::pow(adaptive_ld,2)); // = 1/R
  }
}