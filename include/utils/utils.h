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
      Position operator+(const Position &p) const;
      Position operator-(const Position &p) const;
      Position operator*(const Position &p) const;
      Position operator/(const Position &p) const;
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
    };
    struct TrajPoint {
      PathPoint path_point;
      double time;
    };
    using Path = std::vector<PathPoint>;
    using Traj = std::vector<TrajPoint>;

    // the first version is used for posture mission
    Path toPath(const std::vector<std::vector<double>>& vec);
    // second one for path and traj mission
    Path toPath(const std::vector<Position>& vec);
  }

  namespace math {
    constexpr double EPSILON_TOLERANCE = 1E-6;
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

    static inline double pointLineDistance(double a, double b, double c, double lx, double ly) {
      return std::fabs(a*lx+b*ly+c)/std::sqrt(a*a+b*b);
    }
  }

  namespace path {
    using namespace utils::types;
    void addCurvilinearDistance(Path& path);
    void addCurvature(Path& path);
    void addOrientation(Path& path);
    Path toPath(const std::vector<Position>& positions);
    const PathPoint* closestPoint(size_t &index, const Position& point, const Path& path);
    const PathPoint* lookaheadPoint(const PathPoint& start, const PathPoint& end, const Position& robot_position, const Path& path, double ld, const PathPoint& last_lp);
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
      double steering_angle;
      double max_control;
    };
    struct PIDControl {
      double kp;
      double kd;
      double ki;
      double max_integral_error;
    };
    struct PPControl {
      double ld;
      double k;
    };
    struct PostureMission {
      double x;
      double y;
      double heading;
      double distance_tolerance;
      double angular_tolerance;
    };
    struct PathMission {

    };
    struct TrajMission {

    };
    struct Config {
      Simulation simulation;
      Vehicle vehicle;
      PIDControl pid_ang_uni;
      PIDControl pid_lat_uni;
      PIDControl pid_ang_bi;
      PIDControl pid_lat_bi;
      PPControl pure_pursuit;
      PostureMission posture;
      PathMission path;
      TrajMission traj;
    };

    Config parseJson(const std::string &config_file);
    utils::config::TrajMission toTraj(const utils::config::PostureMission& mission);
  }
}
