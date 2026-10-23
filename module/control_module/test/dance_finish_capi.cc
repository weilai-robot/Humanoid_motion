// Offline FFI for the exact production finishing state/command core.
#include "control_module/dance_finish_core.h"

using namespace xyber_x1_infer::rl_control_module;
extern "C" {
void* dance_finish_create(const double* parameters, const double* joints, const int* legs) {
  try {
    DanceFinishCore::Config c;
    double* fields[]{&c.blend_s, &c.upper_delay_s, &c.upper_blend_s, &c.landing_min_s,
      &c.landing_hold_s, &c.stand_blend_s, &c.verify_s, &c.max_tilt_deg, &c.quiet_tilt_deg,
      &c.quiet_gyro, &c.quiet_joint_speed, &c.quiet_base_speed, &c.contact_on, &c.contact_off,
      &c.max_foot_height_delta, &c.max_foot_velocity_delta, &c.recovery_tilt_deg, &c.recovery_gyro,
      &c.recovery_speed, &c.recovery_delay_s, &c.upper_target_speed, &c.force_filter_s, &c.max_stand_pose_error};
    for (size_t i = 0; i < 23; ++i) *fields[i] = parameters[i];
    size_t offset = 0;
    for (auto* values : {&c.lower, &c.upper, &c.nominal, &c.kp, &c.kd, &c.stand_kp, &c.stand_kd, &c.stand_pose}) {
      std::copy_n(joints+offset, 29, values->begin()); offset += 29;
    }
    for (size_t i = 0; i < 12; ++i) c.legs[i] = legs[i];
    return new DanceFinishCore(c);
  } catch (...) { return nullptr; }
}
void dance_finish_destroy(void* core) { delete static_cast<DanceFinishCore*>(core); }
int dance_finish_begin(void* core, const double* targets, int64_t stamp) {
  try {
    DanceFinishCore::Joints q; std::copy_n(targets, 29, q.begin());
    static_cast<DanceFinishCore*>(core)->Begin(q, stamp); return 1;
  } catch (...) { return 0; }
}
const char* dance_finish_reason(void* core) { return static_cast<DanceFinishCore*>(core)->Reason(); }
void dance_finish_update(void* pointer, const double* values, const double* proposed,
                         int64_t clock_ns, int64_t source_ns, int contact_valid, double* result) {
  auto& core = *static_cast<DanceFinishCore*>(pointer);
  DanceFinishCore::Input in;
  std::copy_n(values, 4, in.orientation.begin()); std::copy_n(values+4, 3, in.gyro.begin());
  std::copy_n(values+7, 29, in.q.begin()); std::copy_n(values+36, 29, in.qd.begin());
  std::copy_n(values+65, 29, in.effort.begin());
  in.source_ns = source_ns; in.contact_input_valid = contact_valid;
  DanceFinishCore::Joints target; std::copy_n(proposed, 29, target.begin());
  core.Update(in, target, clock_ns);
  size_t offset = 0;
  for (const auto* a : {&core.Targets(), &core.Kp(), &core.Kd()}) {
    std::copy(a->begin(), a->end(), result+offset); offset += 29;
  }
  for (double v : core.Forces()) result[offset++] = v;
  for (bool v : core.Contacts()) result[offset++] = v;
  for (double v : core.Velocity()) result[offset++] = v;
  result[offset++] = core.QuietSeconds(); result[offset++] = core.TiltDegrees();
  result[offset++] = static_cast<int>(core.GetPhase());
  result[offset++] = core.StandingConfirmed(); result[offset++] = core.Recoveries();
}
}
