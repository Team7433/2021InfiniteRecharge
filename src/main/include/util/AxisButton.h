#pragma once
#include <frc/GenericHID.h>
#include <frc/XboxController.h>

#include <frc2/command/button/Button.h>


namespace frc2 {
/**
 * A class used to bind command scheduling to joystick POV presses.  Can be
 * composed with other buttons with the operators in Trigger.
 *
 * @see Trigger
 */
class AxisButton : public Button {
 public:
  /**
   * Creates a POVButton that commands can be bound to.
   *
   * @param joystick The joystick on which the button is located.
   * @param rightSide either left side or right side to activate button.
   * @param hand The number of the thumbstick on the controller.
   */
  AxisButton(frc::GenericHID* joystick, bool rightSide, frc::XboxController::JoystickHand hand)
      : Button([joystick, rightSide, hand] {
        if (rightSide) {  return (joystick->GetX(hand) > 0.2);  } else { return (joystick->GetX(hand) < -0.2); }

        }) {}
};
}  // namespace frc2
