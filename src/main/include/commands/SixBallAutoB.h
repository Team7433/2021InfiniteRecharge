// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/ExampleSubsystem.h"
#include "subsystems/FloorIntake.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/BallHolder.h"
#include "subsystems/Feeder.h"
#include "subsystems/Gyro.h"
#include "subsystems/Vision.h"
#include "subsystems/Arm.h"

class SixBallAutoB
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 SixBallAutoB> {
 public:
  SixBallAutoB(FloorIntake*, DriveTrain*, Shooter*, BallHolder*, Feeder*, Gyro*, Vision*, Arm*);
};
