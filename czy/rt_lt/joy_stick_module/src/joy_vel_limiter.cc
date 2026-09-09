#include "joy_stick_module/joy_vel_limiter.h"

namespace xyber_x1_infer::joy_stick_module {

JoyVelLimiter::JoyVelLimiter(int32_t dim, double dt, double vel_limit) : dim_(dim), dt_(dt) {
  lb_.resize(dim_);
  lb_.setConstant(-vel_limit);
  ub_.resize(dim_);
  ub_.setConstant(vel_limit);
  state_.resize(dim_);
  state_.setZero();
  H_ = dt_ * dt_ * Eigen::MatrixXd::Identity(dim_, dim_);

  options_.printLevel = qpOASES::PL_NONE;
  options_.initialStatusBounds = qpOASES::ST_INACTIVE;
  options_.numRefinementSteps = 1;
  options_.enableCholeskyRefactorisation = 1;
  solver_ = qpOASES::QProblemB(dim_);
  solver_.setOptions(options_);
}

JoyVelLimiter::JoyVelLimiter(int32_t dim, double dt, array_t lb, array_t ub) : dim_(dim), dt_(dt) {
  if (lb.size() != dim || ub.size() != dim) {
    throw std::runtime_error("Dimension mismatch: sizes of lb and ub must equal dim");
  }
  lb_ = lb;
  ub_ = ub;
  state_.resize(dim_);
  state_.setZero();
  H_ = dt_ * dt_ * Eigen::MatrixXd::Identity(dim_, dim_);

  options_.printLevel = qpOASES::PL_NONE;
  options_.initialStatusBounds = qpOASES::ST_INACTIVE;
  options_.numRefinementSteps = 1;
  options_.enableCholeskyRefactorisation = 1;
  solver_ = qpOASES::QProblemB(dim_);
  solver_.setOptions(options_);
}

void JoyVelLimiter::reset() { state_.setZero(); }

array_t JoyVelLimiter::update(array_t target_pos) {
  array_t g(dim_);
  g = (state_ - target_pos) * dt_;
  int32_t nWSR = 10;
  solver_.init(H_.data(), g.data(), lb_.data(), ub_.data(), nWSR, 0);

  array_t xOpt(dim_);
  solver_.getPrimalSolution(xOpt.data());
  state_ += xOpt * dt_;
  return state_;
}
}  // namespace xyber_x1_infer::joy_stick_module