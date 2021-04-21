// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "subsystems/Arm.h"
#include "subsystems/Climber.h"
#include "subsystems/DriveTrain.h"

#include <frc2/Timer.h>
#include <frc/Joystick.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimbMode
    : public frc2::CommandHelper<frc2::CommandBase, ClimbMode> {
 public:
  ClimbMode(Arm* arm, Climber* climber, DriveTrain* m_driveTrain, frc::Joystick* joystick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


 private:
  Arm* m_arm;
  Climber* m_climber;
  DriveTrain* m_driveTrain;
  frc::Joystick* m_joystick;
  frc2::Timer m_timer;

  bool m_armLocked = false;
  bool m_sliderSaftey = true;
  bool m_climbMode = false;
  bool m_matchTriggered = false;

};
