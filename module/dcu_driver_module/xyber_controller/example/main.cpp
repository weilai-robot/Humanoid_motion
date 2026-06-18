/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:16:21
 */

#include <cstdint>
#include <iostream>
#include <thread>

#include "xyber_controller.h"

using namespace xyber;
using namespace std::chrono_literals;

int main() {
  // Step 1. Create an instance of XyberController as singleton
  XyberControllerPtr controller(XyberController::GetInstance());

  // Step 2. Setup the dcu and actuator network according to your robot
  // communication topology.

  // Create a DCU, and name it as "body". We set id = 1, cause this is the very
  // first node in EtherCAT bus.
  uint8_t ethercat_id = 1;
  std::string dcu_name = "body";
  controller->CreateDcu(dcu_name, ethercat_id);

  // Create an PowerFlow-R52 type Actuator, name it by the type and attach it on
  // the "body" DCU. Assuming its can_id is 1, and connected to the dcu channel1
  uint8_t actuator_can_id = 1;
  std::string actuator_name = "PowerFlowR52";
  controller->AttachActuator(dcu_name, CtrlChannel::CTRL_CH1, ActuatorType::POWER_FLOW_R52,
                             actuator_name, actuator_can_id);

  // Step 3. Start the controller

  // Setup EtherCAT realtime thread, 90 for the priority, bind the cpu core 1
  controller->SetRealtime(90, 1);

  // Set the EtherCAT frequency to 1k(1000000ns) and enable DC sync.
  bool ret = controller->Start("enp2s0", 1000000, true);
  if (!ret) {
    std::cout << "Start Failed" << std::endl;
    return 0;
  }

  // Step 4. Enable All actuator
  ret = controller->EnableAllActuator();
  if (ret) {
    std::cout << "Enable Actuator Success" << std::endl;
  } else {
    std::cout << "Enable Actuator Failed" << std::endl;
    return 0;
  }
  // enable imu
  controller->ApplyDcuImuOffset(dcu_name);

  // Step 5. Control the actuator
  float dt = 0;
  float pos_begin = controller->GetPosition(actuator_name);
  for (size_t i = 0; i < 100 * 30; i++) {
    // read imu
    DcuImu imu = controller->GetDcuImuData(dcu_name);
    std::cout << "imu: " << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << ", "
              << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << std::endl;

    // Set target position using MIT mode
    double pos_cmd = pos_begin + 2 * sin(dt);
    controller->SetMitCmd(actuator_name, pos_cmd, 0, 0, 0.9, 0.2);

    // read current position
    float pos_now = controller->GetPosition(actuator_name);
    std::cout << "Position: Cmd " << pos_cmd << " Now " << pos_now << std::endl;

    // phase control
    dt += 0.01;
    if (dt >= 6.28) {
      dt = 0.0;
    }
    std::this_thread::sleep_for(10ms);
  }

  // Step 6. Disable the actuator and stop the controller
  controller->DisableAllActuator();
  controller->Stop();

  return 0;
}
