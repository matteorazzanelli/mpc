
#include <iostream>
#include "utils/utils.h"
#include "simulator/simulator.h"
#include "control/pid.h"

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {

  // Retrieve config
  utils::config::Config config = utils::config::parseJson("/home/matteo/autonomous_guidance/config/config.json");

  // Define the simulator object
  simulator::Simulator simulation(config);

  // Define all possible vehicles and controller
  control::Pid *pid_ang = new control::Pid(config.pid_ang.kp, config.pid_ang.kd, config.pid_ang.ki, 
                                          config.pid_ang.max_integral_error, config.vehicle.max_control, 
                                          config.simulation.dt, PIDType::ANGULAR);
  control::Pid *pid_lat = new control::Pid(config.pid_lat.kp, config.pid_lat.kd, config.pid_lat.ki, 
                                          config.pid_lat.max_integral_error, config.vehicle.max_control, 
                                          config.simulation.dt, PIDType::LATERAL);
  vehicle::Unicycle *unicycle_pid_ang = new vehicle::Unicycle(config);
  vehicle::Unicycle *unicycle_pid_lat = new vehicle::Unicycle(config);
  vehicle::Bicycle *bicycle_pid_ang = new vehicle::Bicycle(config);
  vehicle::Bicycle *bicycle_pid_lat = new vehicle::Bicycle(config);
  
  // Run a specific simulation
  Path uni_pidang = simulation.run(unicycle_pid_ang, pid_ang, "Posture");
  Path uni_pidlat = simulation.run(unicycle_pid_lat, pid_lat, "Posture");
  Path bi_pidang = simulation.run(bicycle_pid_ang, pid_ang, "Posture");
  Path bi_pidlat = simulation.run(bicycle_pid_lat, pid_lat, "Posture");

  // Store
  std::vector<double> uni_pidang_x(config.simulation.iterations);
  std::vector<double> uni_pidang_y(config.simulation.iterations);
  std::vector<double> uni_pidlat_x(config.simulation.iterations);
  std::vector<double> uni_pidlat_y(config.simulation.iterations);
  std::vector<double> bi_pidang_x(config.simulation.iterations);
  std::vector<double> bi_pidang_y(config.simulation.iterations);
  std::vector<double> bi_pidlat_x(config.simulation.iterations);
  std::vector<double> bi_pidlat_y(config.simulation.iterations);
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
  }
  std::vector<double> goal_x(1,config.posture.x), goal_y(1,config.posture.y);
  std::vector<double> start_x(1,config.vehicle.x), start_y(1,config.vehicle.y);

  // Plot
  plt::named_plot("Unicycle Angular PID",uni_pidang_x, uni_pidang_y,"r--");
  plt::named_plot("Unicycle Lateral PID",uni_pidlat_x, uni_pidlat_y,"b--");
  plt::named_plot("Bicycle Angular PID",bi_pidang_x, bi_pidang_y,"g--");
  plt::named_plot("Bicycle Lateral PID",bi_pidlat_x, bi_pidlat_y,"m--");
  plt::named_plot("Goal",goal_x,goal_y,"ro");
  plt::named_plot("Start",start_x,start_y,"bo");
  plt::title("AGV position");
  plt::legend();
  plt::show();

  return 0;
}