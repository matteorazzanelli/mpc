
#include "utils/utils.h"

using namespace utils::types;

namespace vehicle {

  class Model {
    public:
      Model(const double& dt);
      Model(const double& dt, const Pose& state);
      virtual Pose updateState(const Control& input) = 0;
      Pose getState();
      Control getInput();
    protected:
      double dt_ = 0.0;
      Control input_;
      Pose state_;
  };

  class Unicycle : public Model {
    public:
      Unicycle(const double &dt);
      Unicycle(const double &dt, const Pose& state);
      Pose updateState(const Control& input);
  };

  class Bicycle : public Model {
    private:
      double length_;
    public:
      Bicycle(const double &dt);
      Bicycle(const double &dt, const Pose& state);
      Pose updateState(const Control& input);
  };
}

