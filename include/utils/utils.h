#pragma once

#include <vector>
#include <math.h>
#include <string>
#include <variant>

namespace utils {
  namespace types {
    struct Position {
      double x = 0.0;
      double y = 0.0;
    };
    struct Pose {
      Position position;
      double heading = 0.0;
      double steering_angle = 0.0;
    };
    struct Control {
      double v = 0.0;
      double w = 0.0;
    };
    enum class ControlType : unsigned int {
      ANGULAR = 0,
      LINEAR = 1
    };
    enum class PIDType : unsigned int {
      ANGULAR = 0,
      LATERAL = 1
    };
    struct PathPoint {
      Pose pose;
      double s; // distance from the beginning, (i.e. curvilinear abscissa)
      double k; // curvature
      Control u;
    };
    struct TrajPoint {
      PathPoint path_point;
      double time;
    };
    using Path = std::vector<PathPoint>;
    using Traj = std::vector<TrajPoint>;
    using SimResult = std::variant<Path, Traj>;
  }

  namespace math {
    constexpr double EPSILON_TOLERANCE = 1E-9;
    static inline double shortestAngularDistance(const double &from, const double &to) {
      double a = fmod(fmod(to - from, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
      if (a > M_PI)
        a -= 2.0 * M_PI;
      return a;
    }
    static inline double angleDifference(double alpha1, double alpha2) {
      return std::atan2(std::sin(alpha2 - alpha1), std::cos(alpha2 - alpha1));
    }
    static inline double eulerDistance(const utils::types::Position& from, const utils::types::Position& to) {
      return sqrt( pow(to.x - from.x, 2) + pow(to.y - from.y, 2) );
    }
    static inline double modPi(const double& angle) {
      return fmod(angle + M_PI, M_PI * 2.0) - M_PI;
    }
    static inline double losAngle(const utils::types::Position& from, const utils::types::Position& to) {
      return atan2(to.y - from.y, to.x - from.x);
    }
    static inline double lateralDistance(const utils::types::Pose& from, const utils::types::Pose& to) {
      return (from.position.x - to.position.x) * sin(from.heading) - (from.position.y - to.position.y) * cos(from.heading);
    }
    double interpolateLinear(double x, double x1, double x2, double y1, double y2);

    double interpolateAnglesLinear(double x, double x1, double x2, double y1, double y2);

    double slerp(double m, double alpha1, double alpha2);

    template <typename T> int sign(T val) {
      return (T(0) < val) - (val < T(0));
    }

    double normalizeAngle(double angle);
  }

  namespace config {
    struct Simulation {
      unsigned int iterations;
      double dt;
    };
    struct Vehicle {
      double length;
      double x;
      double y;
      double heading;
      double steering;
      double max_control;
    };
    struct PIDControl {
      double kp;
      double kd;
      double ki;
      double max_integral_error;
    };
    struct PostureMission {
      double x;
      double y;
      double heading;
      double distance_tolerance;
      double angular_tolerance;
    };
    struct Config {
      Simulation simulation;
      Vehicle vehicle;
      PIDControl pid_ang;
      PIDControl pid_lat;
      PostureMission posture;
    };
    Config parseJson(const std::string &config_file);
  }
}
