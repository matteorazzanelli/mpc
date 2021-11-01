#include "utils/utils.h"
#include <glog/logging.h>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>

namespace utils {
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
      config.vehicle.steering = json["Vehicle"]["steering"].asDouble();
      config.vehicle.max_control = json["Vehicle"]["max_control"].asDouble();
      
      config.pid_ang.kp = json["Control"]["PID_angular"]["kp"].asDouble();
      config.pid_ang.kd = json["Control"]["PID_angular"]["kd"].asDouble();
      config.pid_ang.ki = json["Control"]["PID_angular"]["ki"].asDouble();
      config.pid_ang.max_integral_error = json["Control"]["PID_angular"]["max_integral_error"].asDouble();

      config.pid_lat.kp = json["Control"]["PID_lateral"]["kp"].asDouble();
      config.pid_lat.kd = json["Control"]["PID_lateral"]["kd"].asDouble();
      config.pid_lat.ki = json["Control"]["PID_lateral"]["ki"].asDouble();
      config.pid_lat.max_integral_error = json["Control"]["PID_lateral"]["max_integral_error"].asDouble();

      config.posture.x = json["Mission"]["Posture"]["x"].asDouble();
      config.posture.y = json["Mission"]["Posture"]["y"].asDouble();
      config.posture.heading = json["Mission"]["Posture"]["heading"].asDouble();
      config.posture.distance_tolerance = json["Mission"]["Posture"]["distance_tolerance"].asDouble();
      config.posture.angular_tolerance = json["Mission"]["Posture"]["angular_tolerance"].asDouble();

      return config;
    }

  }
}