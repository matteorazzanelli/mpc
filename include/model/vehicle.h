
#include "utils/utils.h"

using namespace utils::types;

namespace vehicle {

  class Model {
    public:
      Model(const utils::config::Config& config);
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
      Unicycle(const utils::config::Config& config);
      Pose updateState(const Control& input);
  };

  class Bicycle : public Model {
    private:
      double length_;
    public:
      Bicycle(const utils::config::Config& config);
      Pose updateState(const Control& input);
  };
}

