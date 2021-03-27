/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Gyro.h"
#include "subsystems/DriveTrain.h"

#include "Constants.h"

#include <units/angle.h>

using namespace DriveTrainConstants;

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class GoToAngle
    : public frc2::CommandHelper<frc2::CommandBase, GoToAngle> {
 public:
  GoToAngle(DriveTrain* drivetrain, Gyro* gyro, frc::Joystick* joystick, double target);
  GoToAngle(DriveTrain* drivetrain, Gyro* gyro, frc::Joystick* joystick, units::degree_t target);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:

  DriveTrain* m_driveTrain;
  Gyro* m_gyro;
  frc::Joystick* m_joystick;
  units::degree_t m_target;


  
  bool m_done = false;

  double m_difference;
  double m_output;

  double m_kp;                                                                  
};
