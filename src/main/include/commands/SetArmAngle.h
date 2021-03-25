/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Arm.h"

#include <units/angle.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetArmAngle
    : public frc2::CommandHelper<frc2::CommandBase, SetArmAngle> {
 public:
  SetArmAngle(Arm* arm, double angle);

  SetArmAngle(Arm* arm, std::function<double()> angle);

  SetArmAngle(Arm* arm, std::function<units::degree_t()> angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  Arm* m_arm;
  std::function<units::degree_t()> m_angle;
};
