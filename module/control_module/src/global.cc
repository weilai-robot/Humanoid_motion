#include "control_module/global.h"  // IWYU pragma: keep

namespace xyber_x1_infer::rl_control_module {

aimrt::logger::LoggerRef global_logger;
void SetLogger(aimrt::logger::LoggerRef logger) { global_logger = logger; }
aimrt::logger::LoggerRef GetLogger() { return global_logger ? global_logger : aimrt::logger::GetSimpleLoggerRef(); }

}  // namespace xyber_x1_infer::rl_control_module
