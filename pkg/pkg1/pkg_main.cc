#include <cstring>

#include "aimrt_pkg_c_interface/pkg_macro.h"
#include "ankle_identifier_module/ankle_identifier_module.h"
#include "dcu_driver_module/dcu_driver_module.h"
#include "joy_stick_module/joy_stick_module.h"
#include "control_module/control_module.h"
#include "sim_module/sim_module.h"

static std::tuple<std::string_view, std::function<aimrt::ModuleBase*()>>
    aimrt_module_register_array[]{
        {"DcuDriverModule",
         []() -> aimrt::ModuleBase* {
           return new xyber_x1_infer::dcu_driver_module::DcuDriverModule();
         }},
        {"AnkleIdentifierModule",
         []() -> aimrt::ModuleBase* {
           return new xyber_x1_infer::ankle_identifier_module::AnkleIdentifierModule();
         }},
        {"JoyStickModule",
         []() -> aimrt::ModuleBase* {
           return new xyber_x1_infer::joy_stick_module::JoyStickModule();
         }},
        {"ControlModule",
         []() -> aimrt::ModuleBase* {
           return new xyber_x1_infer::rl_control_module::ControlModule();
         }},
        {"SimModule",
         []() -> aimrt::ModuleBase* {
           return new xyber_x1_infer::sim_module::SimModule();
         }},
    };

AIMRT_PKG_MAIN(aimrt_module_register_array)
