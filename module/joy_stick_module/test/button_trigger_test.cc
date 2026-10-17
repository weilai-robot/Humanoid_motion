#include "joy_stick_module/button_trigger.h"

#include <gtest/gtest.h>

namespace xyber_x1_infer::joy_stick_module {

TEST(ButtonTrigger, EarlyGuideRequiresReleaseAndANewPress) {
  ButtonTrigger guide;
  EXPECT_FALSE(guide.Update(false, true));
  EXPECT_TRUE(guide.Update(true, true));  // Controller rejects before stand is reached.
  for (int tick = 0; tick < 200; ++tick) EXPECT_FALSE(guide.Update(true, true));
  EXPECT_FALSE(guide.Update(false, true));
  EXPECT_TRUE(guide.Update(true, true));  // A new Guide can authorize preparation -> playback.
  EXPECT_FALSE(guide.Update(true, true));
}

TEST(ButtonTrigger, ExistingLevelTriggeredBindingsKeepPublishing) {
  ButtonTrigger button;
  EXPECT_FALSE(button.Update(false, false));
  for (int tick = 0; tick < 10; ++tick) EXPECT_TRUE(button.Update(true, false));
  EXPECT_FALSE(button.Update(false, false));
}

}  // namespace xyber_x1_infer::joy_stick_module
