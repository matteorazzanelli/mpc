#include "utils/utils.h"
#include <glog/logging.h>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>

using namespace utils::types;

namespace utils {
  namespace types {
    Position Position::operator+(const Position &p) const {
      return {.x = x + p.x, .y = y + p.y};
    }
    Position Position::operator-(const Position &p) const {
      return {.x = x - p.x, .y = y - p.y};
    }
    Position Position::operator*(const Position &p) const {
      return {.x = x * p.x, .y = y * p.y};
    }
    Position Position::operator/(const Position &p) const {
      return {.x = x / p.x, .y = y / p.y};
    }
  }
  
  namespace math {
    double interpolateLinear(double x, double x1, double x2, double y1, double y2) {
      CHECK_GT(x2, x1) << "Provided out of order interpolation boundaries: " << x1 << " >= " << x2;
      return (y1 + (y2 - y1)/(x2 - x1)*(x - x1));
    }

    double interpolateAnglesLinear(double x, double x1, double x2, double y1, double y2) {
      CHECK_GT(x2, x1) << "Provided out of order interpolation boundaries: " << x1 << " >= " << x2;

      if (std::fabs(x2 - x) < EPSILON_TOLERANCE) {
        return y2;
      } else if (std::fabs(x - x1) < EPSILON_TOLERANCE) {
        return y1;
      }
      
      CHECK_GE(x2, x) << "Cannot extrapolate state in the future : " << x << " > " << x2;
      CHECK_LE(x1, x) << "Cannot extrapolate state in the past : " << x << " < " << x1;

      // interpolation factor [(y-y1)/(y2-y1)=(x-x1)/(x2-x1)]
      double m = (x-x1)/(x2-x1);

      return slerp(m, y1, y2);
    }

    double slerp(double m, double alpha1, double alpha2) {
      double alpha = angleDifference(alpha1, alpha2);

      if (std::fabs(alpha) < EPSILON_TOLERANCE) {
        return alpha1;
      }

      // Coordinates of the interpolated point on the unitary circumference
      double x = std::sin((1-m)*alpha)/std::sin(alpha)*std::cos(alpha1) +  std::sin(m*alpha)/std::sin(alpha)*std::cos(alpha2);
      double y = std::sin((1-m)*alpha)/std::sin(alpha)*std::sin(alpha1) +  std::sin(m*alpha)/std::sin(alpha)*std::sin(alpha2);

      return std::atan2(y, x);
    }

    double normalizeAngle(double angle) {
      while (angle > M_PI) {
        angle -= 2.0 * M_PI;
      }
      while (angle < -M_PI) {
        angle += 2.0 * M_PI;
      }
      return angle;
    }
  }
  
  namespace path {
    Path toPath(const std::vector<Position>& positions) {
      Path p(positions.size());
      if (positions.empty())
        throw std::invalid_argument("The given vector is empty!");
      if (positions.size() < 2)
        LOG(ERROR) << "Path must be comosed of at least 2 components.";
      
      for (size_t i = 0; i < positions.size(); i ++) {
        p[i] = utils::types::PathPoint {
          .pose = utils::types::Pose {
            .position = utils::types::Position {
              .x = positions[i].x,
              .y = positions[i].y
            },
            .heading = 0.0,
            .steering_angle = 0.0
          },
          .s = 0.0,
          .k = 0.0
        };
      }
      addCurvilinearDistance(p);
      addCurvature(p);
      addOrientation(p);
      return p;
    }

    void addOrientation(Path& path) {
      if (path.empty())
        throw std::invalid_argument("The given track is empty, can't compute orientation!");
      for (size_t i = 0; i < path.size() - 1; i++)
        path[i].pose.heading = utils::math::losAngle(path[i].pose.position, path[i+1].pose.position);
        path.back().pose.heading = path[path.size() - 2].pose.heading;
      }

    void addCurvilinearDistance(Path& path) {
      CHECK_GE(path.size(),1) << "Path must contain at least 1 element.";
      double distance = 0.0;
      path[0].s = distance;
      for (size_t i = 1; i < path.size(); i ++) {
        path[i].s = distance + utils::math::eulerDistance(path[i-1].pose.position, path[i].pose.position);
      }
    }

    void addCurvature(Path& path) {
      CHECK_GE(path.size(),2) << "Path must contain at least 2 elements.";
      path.front().k = 0.0;
      path.back().k = 0.0;
      double Ax, Ay, Bx, By, Cx, Cy, AB, BC, CA, area;
      for(size_t i = 1; i < path.size()-1; i ++) {
        // find area of the triangle
        Ax = path[i-1].pose.position.x;
        Ay = path[i-1].pose.position.y;
        Bx = path[i].pose.position.x;
        By = path[i].pose.position.y;
        Cx = path[i+1].pose.position.x;
        Cy = path[i+1].pose.position.y;
        AB = utils::math::eulerDistance(path[i-1].pose.position, path[i].pose.position);
        BC = utils::math::eulerDistance(path[i].pose.position, path[i+1].pose.position);
        CA = utils::math::eulerDistance(path[i+1].pose.position, path[i-1].pose.position);
        // compute area from determinant
        area = 0.5 * std::fabs(Ax*(By-Cy) + Bx*(Cy - Ay) + Cx*(Ay - By));
        if(area < utils::math::EPSILON_TOLERANCE)
          path[i].k = 0.0; // points are colinear
        else
          path[i].k = 4 * area / (AB * BC * CA);
      }
    }

    const PathPoint* closestPoint(size_t &index, const Position& point, const Path& path) {
      CHECK_GE(index, 0) << "Negative index not alloews.";
      double closest_distance = std::numeric_limits<double>::max();
      const PathPoint* pp; 
      for (size_t i = index; i < path.size(); i++) {
        double wx = path.at(i).pose.position.x;
        double wy = path.at(i).pose.position.y;
        double current_closest_distance = std::pow((point.x - wx),2) + pow((point.y - wy),2);
        if (current_closest_distance < closest_distance) {
          closest_distance = current_closest_distance;
          index = i;
          pp = &path.at(i);
        }
      }
      return pp;
    }

    const PathPoint* lookaheadPoint(const PathPoint& start, const PathPoint& end, const Position& robot_position, const Path& path, double ld, const PathPoint& last_lp) {
      // Check intersection segment circumference
      double start_x, start_y, end_x, end_y, c_x, c_y;
      start_x = start.pose.position.x;
      start_y = start.pose.position.y;
      end_x = end.pose.position.x;
      end_y = end.pose.position.y;
      c_x = robot_position.x;
      c_y = robot_position.y;
      double a = std::pow(end_x - start_x,2) + std::pow(end_y - start_y,2);
      double b = start_y * ( (end_y - start_y) + c_y) - end_y * c_y + start_x * ( (end_y - start_x) + c_x) - end_x * c_x;
      double delta = std::pow(end_x-start_x,2) * (ld*ld - c_y*c_y) + 
                    std::pow(end_y-start_y,2) * (ld*ld - c_y*c_x) + 
                    2*start_x*start_y*c_x*c_y + 2*end_x*end_y*c_x*c_y - 
                    std::pow(start_x*start_x*end_y*end_y - end_x*end_x*start_y*start_y,2) + 
                    2*start_x*end_y*(-c_y*(end_x-start_x+c_x) + c_x*(end_y-start_y)) + 
                    2*start_y*end_x*(-c_x*(end_y-start_y+c_y) + c_y*(end_x-start_x));
      // CHECK_GE(delta, 0) << "No intersection.";
      delta = std::sqrt(delta);
      double t1 = (-b + delta) / (2 * a);
      double t2 = (-b - delta) / (2 * a);
      const PathPoint* pp;
      if (t1 >= 0 && t1 <= 1) {
        pp = &end; // FIXME should be an interpolation between start and end
      }
      else if (t2 >= 0 && t2 <= 1){
        pp = &start; // FIXME should be an interpolation between end and start
      }
      else {
        pp = &last_lp;
      }
      return pp;
    }
  }
  
  namespace config {
    Config parseJson(const std::string &config_file) {
      // Read file
      CHECK(std::filesystem::exists(config_file)) << "Config file " << config_file.c_str() << " does not exist.";
      std::ifstream json_stream(config_file);
      Json::Value json;
      json_stream >> json;

      // Parse config
      Config config;
      config.simulation.iterations = json["Simulation"]["iterations"].asUInt();
      config.simulation.dt = json["Simulation"]["dt"].asDouble();

      config.vehicle.length = json["Vehicle"]["length"].asDouble();
      config.vehicle.x = json["Vehicle"]["x"].asDouble();
      config.vehicle.y = json["Vehicle"]["y"].asDouble();
      config.vehicle.heading = json["Vehicle"]["heading"].asDouble();
      config.vehicle.steering_angle = json["Vehicle"]["steering_angle"].asDouble();
      config.vehicle.max_control = json["Vehicle"]["max_control"].asDouble();
      
      config.pid_ang_uni.kp = json["Control"]["PID_angular_unicycle"]["kp"].asDouble();
      config.pid_ang_uni.kd = json["Control"]["PID_angular_unicycle"]["kd"].asDouble();
      config.pid_ang_uni.ki = json["Control"]["PID_angular_unicycle"]["ki"].asDouble();
      config.pid_ang_uni.max_integral_error = json["Control"]["PID_angular_unicycle"]["max_integral_error"].asDouble();

      config.pid_lat_uni.kp = json["Control"]["PID_lateral_unicycle"]["kp"].asDouble();
      config.pid_lat_uni.kd = json["Control"]["PID_lateral_unicycle"]["kd"].asDouble();
      config.pid_lat_uni.ki = json["Control"]["PID_lateral_unicycle"]["ki"].asDouble();
      config.pid_lat_uni.max_integral_error = json["Control"]["PID_lateral_unicycle"]["max_integral_error"].asDouble();

      config.pid_ang_bi.kp = json["Control"]["PID_angular_bicycle"]["kp"].asDouble();
      config.pid_ang_bi.kd = json["Control"]["PID_angular_bicycle"]["kd"].asDouble();
      config.pid_ang_bi.ki = json["Control"]["PID_angular_bicycle"]["ki"].asDouble();
      config.pid_ang_bi.max_integral_error = json["Control"]["PID_angular_bicycle"]["max_integral_error"].asDouble();

      config.pid_lat_bi.kp = json["Control"]["PID_lateral_bicycle"]["kp"].asDouble();
      config.pid_lat_bi.kd = json["Control"]["PID_lateral_bicycle"]["kd"].asDouble();
      config.pid_lat_bi.ki = json["Control"]["PID_lateral_bicycle"]["ki"].asDouble();
      config.pid_lat_bi.max_integral_error = json["Control"]["PID_lateral_bicycle"]["max_integral_error"].asDouble();

      config.pure_pursuit.k = json["Control"]["PurePursuit"]["k"].asDouble();
      config.pure_pursuit.ld = json["Control"]["PurePursuit"]["ld"].asDouble();

      config.posture.x = json["Mission"]["Posture"]["x"].asDouble();
      config.posture.y = json["Mission"]["Posture"]["y"].asDouble();
      config.posture.heading = json["Mission"]["Posture"]["heading"].asDouble();
      config.posture.distance_tolerance = json["Mission"]["Posture"]["distance_tolerance"].asDouble();
      config.posture.angular_tolerance = json["Mission"]["Posture"]["angular_tolerance"].asDouble();

      return config;
    }

  }
}