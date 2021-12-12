
#include <iostream>
#include "utils/utils.h"
#include "simulator/simulator.h"
#include "control/pid.h"
#include "control/pure_pursuit.h"

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {

  // Retrieve config
  utils::config::Config config = utils::config::parseJson("/home/matteo/autonomous_guidance/config/config.json");

  // Define the simulator object
  simulator::Simulator simulation(config);

  // Define all possible controller
  control::Pid *pid_ang_uni = new control::Pid(config.pid_ang_uni.kp, config.pid_ang_uni.kd, config.pid_ang_uni.ki, config.pid_ang_uni.max_integral_error, config.vehicle.max_control, config.simulation.dt, PIDType::ANGULAR);
  control::Pid *pid_lat_uni = new control::Pid(config.pid_lat_uni.kp, config.pid_lat_uni.kd, config.pid_lat_uni.ki, config.pid_lat_uni.max_integral_error, config.vehicle.max_control, config.simulation.dt, PIDType::LATERAL);
  control::Pid *pid_ang_bi = new control::Pid(config.pid_ang_bi.kp, config.pid_ang_bi.kd, config.pid_ang_bi.ki, config.pid_ang_bi.max_integral_error, config.vehicle.max_control, config.simulation.dt, PIDType::ANGULAR);
  control::Pid *pid_lat_bi = new control::Pid(config.pid_lat_bi.kp, config.pid_lat_bi.kd, config.pid_lat_bi.ki, config.pid_lat_bi.max_integral_error, config.vehicle.max_control, config.simulation.dt, PIDType::LATERAL);
  control::PurePursuit *pp_uni = new control::PurePursuit(config.pure_pursuit.ld, config.pure_pursuit.k, config.vehicle.max_control, 0.0, config.simulation.dt);
  control::PurePursuit *pp_bi = new control::PurePursuit(config.pure_pursuit.ld, config.pure_pursuit.k, config.vehicle.max_control, config.vehicle.length, config.simulation.dt);

  // Define all possible vehicles
  vehicle::Unicycle *unicycle_pid_ang = new vehicle::Unicycle(config);
  vehicle::Unicycle *unicycle_pid_lat = new vehicle::Unicycle(config);
  vehicle::Unicycle *unicycle_pp = new vehicle::Unicycle(config);
  vehicle::Bicycle *bicycle_pid_ang = new vehicle::Bicycle(config);
  vehicle::Bicycle *bicycle_pid_lat = new vehicle::Bicycle(config);
  vehicle::Bicycle *bicycle_pp = new vehicle::Bicycle(config);

  // first point is always robot initial pose
  utils::types::Path path = utils::path::toPath({
    utils::types::Position{config.vehicle.x, config.vehicle.y},
    utils::types::Position{config.posture.x, config.posture.y}
  });
  pp_uni->init(path);
  pp_bi->init(path);
  
  // Run a specific simulation
  Path uni_pidang = simulation.run(unicycle_pid_ang, pid_ang_uni, "Posture");
  Path uni_pidlat = simulation.run(unicycle_pid_lat, pid_lat_uni, "Posture");
  Path bi_pidang = simulation.run(bicycle_pid_ang, pid_ang_bi, "Posture");
  Path bi_pidlat = simulation.run(bicycle_pid_lat, pid_lat_bi, "Posture");
  Path uni_pp = simulation.run(unicycle_pp, pp_uni, "Posture");
  Path bi_pp = simulation.run(bicycle_pp, pp_bi, "Posture");

  // Store
  std::vector<double> uni_pidang_x(config.simulation.iterations);
  std::vector<double> uni_pidang_y(config.simulation.iterations);
  std::vector<double> uni_pidlat_x(config.simulation.iterations);
  std::vector<double> uni_pidlat_y(config.simulation.iterations);
  std::vector<double> bi_pidang_x(config.simulation.iterations);
  std::vector<double> bi_pidang_y(config.simulation.iterations);
  std::vector<double> bi_pidlat_x(config.simulation.iterations);
  std::vector<double> bi_pidlat_y(config.simulation.iterations);
  std::vector<double> uni_pp_x(config.simulation.iterations);
  std::vector<double> uni_pp_y(config.simulation.iterations);
  std::vector<double> bi_pp_x(config.simulation.iterations);
  std::vector<double> bi_pp_y(config.simulation.iterations);
  size_t i = 0;
  for (size_t i = 0; i < config.simulation.iterations; i ++) {
    uni_pidang_x[i] = uni_pidang[i].pose.position.x;
    uni_pidang_y[i] = uni_pidang[i].pose.position.y;
    uni_pidlat_x[i] = uni_pidlat[i].pose.position.x;
    uni_pidlat_y[i] = uni_pidlat[i].pose.position.y;
    bi_pidang_x[i] = bi_pidang[i].pose.position.x;
    bi_pidang_y[i] = bi_pidang[i].pose.position.y;
    bi_pidlat_x[i] = bi_pidlat[i].pose.position.x;
    bi_pidlat_y[i] = bi_pidlat[i].pose.position.y;
    uni_pp_x[i] = uni_pp[i].pose.position.x;
    uni_pp_y[i] = uni_pp[i].pose.position.y;
    bi_pp_x[i] = bi_pp[i].pose.position.x;
    bi_pp_y[i] = bi_pp[i].pose.position.y;
  }
  std::vector<double> goal_x(1,config.posture.x), goal_y(1,config.posture.y);
  std::vector<double> start_x(1,config.vehicle.x), start_y(1,config.vehicle.y);

  // Plot
  plt::named_plot("Unicycle Angular PID",uni_pidang_x, uni_pidang_y,"r--");
  plt::named_plot("Unicycle Lateral PID",uni_pidlat_x, uni_pidlat_y,"g--");
  plt::named_plot("Bicycle Angular PID",bi_pidang_x, bi_pidang_y,"b--");
  plt::named_plot("Bicycle Lateral PID",bi_pidlat_x, bi_pidlat_y,"y--");
  plt::named_plot("Unicycle Pure Pursuit",uni_pp_x, uni_pp_y,"m--");
  plt::named_plot("Bicycle Pure Pursuit",bi_pp_x, bi_pp_y,"c--");
  plt::named_plot("Goal",goal_x,goal_y,"ro");
  plt::named_plot("Start",start_x,start_y,"bo");
  plt::title("AGV position");
  plt::legend();
  plt::show();

  return 0;
}