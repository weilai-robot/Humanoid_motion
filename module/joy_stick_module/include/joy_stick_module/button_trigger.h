#pragma once

namespace xyber_x1_infer::joy_stick_module {

class ButtonTrigger {
 public:
  bool Update(bool pressed, bool edge_triggered) {
    const bool publish = pressed && (!edge_triggered || !previous_pressed_);
    previous_pressed_ = pressed;
    return publish;
  }

 private:
  bool previous_pressed_ = false;
};

}  // namespace xyber_x1_infer::joy_stick_module
