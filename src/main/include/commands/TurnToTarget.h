/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/Timer.h>

#include "subsystems/Vision.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Gyro.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurnToTarget
    : public frc2::CommandHelper<frc2::CommandBase, TurnToTarget> {
 public:
  TurnToTarget(Vision* vision, Gyro* gyro, DriveTrain* drivetrain);
  TurnToTarget(Gyro* , DriveTrain*, units::degree_t);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  Vision* m_vision;
  DriveTrain* m_driveTrain;
  Gyro* m_gyro;
  frc2::Timer m_timer;

  units::degree_t m_gyroTarget;

  bool m_done = false;

  units::degree_t m_overideAngle;
  bool m_overide = false;

  double m_kp = 0.01;
  double m_ki = 0.001;
  units::degree_t m_izone = 6.0_deg;
  double m_accumulator = 0.0;
  double m_ks;

  units::degree_t m_error;
  units::degree_t m_startError;
  units::degree_t m_lastGyroAngle = 0_deg;

  double m_Iaccumulator;

};
