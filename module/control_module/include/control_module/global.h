#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace xyber_x1_infer::rl_control_module {

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();

}  // namespace xyber_x1_infer::rl_control_module
