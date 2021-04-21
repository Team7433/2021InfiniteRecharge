#pragma once
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/button/Button.h>
#include <string.h>


namespace frc2 {
/**
 * A class used to bind command scheduling to joystick POV presses.  Can be
 * composed with other buttons with the operators in Trigger.
 *
 * @see Trigger
 */
class NetworkButton : public Button {
 public:
  /**
   * Creates a POVButton that commands can be bound to.
   *
   * @param name The address of the network
   */
  NetworkButton(wpi::StringRef name)
      : Button([name] {
        return frc::SmartDashboard::GetBoolean(name, false);

        }) {}
};
}  // namespace frc2