#pragma once

#include <eigen3/Eigen/Core>
#include <qpOASES.hpp>

using array_t = Eigen::Array<double, Eigen::Dynamic, 1>;

namespace xyber_x1_infer::joy_stick_module {

class JoyVelLimiter {
 public:
  JoyVelLimiter(int32_t dim, double dt, double vel_limit);
  JoyVelLimiter(int32_t dim, double dt, array_t lb, array_t ub);

  void reset();
  array_t update(array_t target_pos);

 private:
  size_t dim_;
  double dt_;
  array_t state_;
  array_t lb_;
  array_t ub_;
  Eigen::MatrixXd H_;
  qpOASES::QProblemB solver_;
  qpOASES::Options options_;
};
}  // namespace xyber_x1_infer::joy_stick_module