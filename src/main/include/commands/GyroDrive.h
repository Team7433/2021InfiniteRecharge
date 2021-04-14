// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gyro.h"
#include "subsystems/DriveTrain.h"



/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class GyroDrive
    : public frc2::CommandHelper<frc2::CommandBase, GyroDrive> {
 public:
  GyroDrive(Gyro* Gyro, DriveTrain* driveTrain, std::function<units::degree_t()> headingAngle, std::function<double()> forwardPower);
  GyroDrive(Gyro* Gyro, DriveTrain* driveTrain, units::degree_t headingAngle, double forwardPower);
  GyroDrive(Gyro* Gyro, DriveTrain* driveTrain, units::degree_t headingAngle, std::function<double()> forwardPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Gyro* m_gyro;
  DriveTrain* m_driveTrain;

  units::degree_t m_error;

  double m_kp = 0.05;


  units::degree_t m_startError;

  
  std::function<units::degree_t()> m_headingAngle;
  std::function<double()> m_forwardPower;
};
