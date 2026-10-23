// Offline MuJoCo probe of the production waist FK, without ROS dependencies.
#include "control_module/dance_state_adapter.h"

extern "C" int dance_waist_fk(const double* input, double* orientation) {
  using namespace xyber_x1_infer::rl_control_module;
  DanceAnchorState anchor;
  if (!ComputeX1DanceAnchor({0, 0, 0}, {input[0], input[1], input[2], input[3]},
                           {input[4], input[5], input[6]}, anchor)) return 0;
  for (size_t i = 0; i < 4; ++i) orientation[i] = anchor.orientation[i];
  return 1;
}
