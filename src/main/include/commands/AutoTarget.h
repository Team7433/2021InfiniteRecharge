// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/InstantCommand.h>

#include "units/length.h"

#include "subsystems/Arm.h"
#include "subsystems/Shooter.h"
#include "subsystems/Vision.h"
#include "subsystems/Gyro.h"
#include "subsystems/DriveTrain.h"

#include "commands/SetArmAngle.h"
#include "commands/RunShooter.h"
#include "commands/TurnToTarget.h"
#include "commands/GyroDrive.h"


class AutoTarget
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoTarget> {
 public:
  //overides for arm and shooter without turn to target or gyro drive
  AutoTarget(std::function<units::meter_t()> DistanceM, Arm* arm, Shooter* shooter, bool update = false);
  AutoTarget(units::meter_t DistanceM, Arm* arm, Shooter* shooter);
  AutoTarget(Vision* vision, Arm* arm, Shooter* shooter, bool update = false);
  //overide for turn to target

  AutoTarget(std::function<units::meter_t()> distanceM, std::function<units::degree_t()> gyroTarget, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain);
  AutoTarget(Vision* vision, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain);

  //overide for gyrodrive
  // AutoTarget(std::function<units::meter_t()> distanceM, std::function<units::degree_t()> gyroTarget, std::function<double()> forwardOutput, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain);
  // AutoTarget(std::function<double()> forwardOutput, Vision* vision, Arm* arm, Shooter* shooter, Gyro* gyro, DriveTrain* drivetrain);
  


 private:
  std::function<units::degree_t()> m_angle;
  std::function<double()> m_velocity;
  units::degree_t m_targetGyroAngle;
};
