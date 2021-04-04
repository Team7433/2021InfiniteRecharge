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

#include "commands/SetArmAngle.h"
#include "commands/RunShooter.h"


class AutoTarget
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoTarget> {
 public:
  AutoTarget(std::function<units::meter_t()> DistanceM, Arm* arm, Shooter* shooter);
  AutoTarget(units::meter_t DistanceM, Arm* arm, Shooter* shooter);
  AutoTarget(Vision* vision, Arm* arm, Shooter* shooter);


 private:
  std::function<units::degree_t()> m_angle;
  std::function<double()> m_velocity;
};
