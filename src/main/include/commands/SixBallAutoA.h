// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/Timer.h>

#include "subsystems/FloorIntake.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/BallHolder.h"
#include "subsystems/Feeder.h"
#include "subsystems/Gyro.h"
#include "subsystems/Vision.h"
#include "subsystems/Arm.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/DriveWithJoystick.h"
#include "commands/SetFloorIntake.h"
#include "commands/RunShooter.h"
#include "commands/SetBallHolder.h"
#include "commands/SetFeeder.h"
#include "commands/SetBallManipulation.h"
#include "commands/ChangeCamMode.h"
#include "commands/TurnToTarget.h"
#include "commands/SetArmAngle.h"
#include "commands/ManualArmControl.h"
#include "commands/DriveRunProfile.h"
#include "commands/UnloadMagazine.h"
#include "commands/AutoTarget.h"
#include "commands/DriveMotionControl.h"

class SixBallAutoA
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 SixBallAutoA> {
 public:
  SixBallAutoA(FloorIntake*, DriveTrain*, Shooter*, BallHolder*, Feeder*, Gyro*, Vision*, Arm*);
};
