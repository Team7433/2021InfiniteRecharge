/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

#include "Constants.h"

#include <iostream>


using namespace ShooterConstants;


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunShooter
    : public frc2::CommandHelper<frc2::CommandBase, RunShooter> {
 public:
  RunShooter(Shooter* shooter, double Velocity);
  RunShooter(Shooter* shooter, std::function<double()> Velocity);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Shooter* m_shooter;
  std::function<double()>  m_velocity;
  double m_targetVelocity;
  bool m_done = false;
};
