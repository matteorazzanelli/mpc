
#include <iostream>
#include "utils/utils.h"
#include "simulator/simulator.h"
#include "control/pid.h"
#include "control/control.h"
// #include "model/vehicle.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char *argv[]) {

  // Retrieve config
  utils::config::Config config = utils::config::parseJson("/home/matteo/autonomous_guidance/config/config.json");

  // Define the simulator object
  simulator::Simulator simulation(config);

  // Define all possible vehicles and controllers
  vehicle::Unicycle *unicycle = new vehicle::Unicycle(config.simulation.dt);
  vehicle::Bicycle *bicycle = new vehicle::Bicycle(config.simulation.dt);
  control::Pid *pid_ang = new control::Pid(config.pid_ang.kp, config.pid_ang.kd, config.pid_ang.ki, config.pid_ang.max_err, config.simulation.dt, PIDType::ANGULAR);
  control::Pid *pid_lat = new control::Pid(config.pid_lat.kp, config.pid_lat.kd, config.pid_lat.ki, config.pid_lat.max_err, config.simulation.dt, PIDType::LATERAL);
  // Run a specific simulation
  Path simulation_result = simulation.run(unicycle, pid_ang, "Posture");
  Path simulation_result2 = simulation.run(unicycle, pid_lat, "Posture");

  // Store
  std::vector<double> robot_x(simulation_result.size());
  std::vector<double> robot_y(simulation_result.size());
  size_t i = 0;
  for (const auto& simpoint : simulation_result) {
    robot_x[i] = simulation_result[i].pose.position.x;
    robot_y[i] = simulation_result[i].pose.position.y;
    i ++;
  }

  // plot
  plt::named_plot("Angular_PID",robot_x, robot_y,"r--");
  // plt::named_plot("goal",goal_x,goal_y,"o");
  plt::title("AGV position");
  plt::legend();
  plt::show();

  return 0;
}