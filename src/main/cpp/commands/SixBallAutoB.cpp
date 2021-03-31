// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SixBallAutoB.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/ExampleCommand.h"
#include "commands/DriveWithJoystick.h"
#include "commands/SetFloorIntake.h"
#include "commands/RunShooter.h"
#include "commands/SetBallHolder.h"
#include "commands/SetFeeder.h"
#include "commands/SetBallManipulation.h"
#include "commands/ChangeCamMode.h"
#include "commands/GoToAngle.h"
#include "commands/TurnToTarget.h"
#include "commands/SetArmAngle.h"
#include "commands/ManualArmControl.h"
#include "commands/DistanceSet.h"
#include "commands/DriveRunProfile.h"
#include "commands/UnloadMagazine.h"

SixBallAutoB::SixBallAutoB(FloorIntake* m_floorIntake, DriveTrain* m_driveTrain, Shooter* m_shooter, BallHolder* m_ballHolder, Feeder* m_feeder, Gyro* m_gyro, Vision* m_vision, Arm* m_arm) {

  // 6B
  AddCommands(
    frc2::ParallelCommandGroup(
      SetArmAngle(m_arm, [m_vision] {
          double distance = 3.5;
          return 15.5504 + (130.439 / (distance + 2.46224));
        }), // SetArmAngle
        RunShooter(m_shooter, [m_vision] {
          double distance = 3.5;
          return 10648.9 + 1447.44 * distance;
        }) // RunShooter
    ),

    UnloadMagazine(m_ballHolder, m_feeder),

    TurnToTarget(m_gyro, m_driveTrain, 90.0),

    frc2::ParallelDeadlineGroup(
      DriveRunProfile(m_driveTrain, "6BCollectBalls"),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.3, 0.3, 0, /* Storing */ true),
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 7)
    ), //Parallel Deadline Group


    frc2::ParallelDeadlineGroup(
      DriveRunProfile(m_driveTrain, "6BToShoot"),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.3, 0.3, 0, /* Storing */ true)),      

    SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0, 0, 0, 0, false),


    frc2::ParallelCommandGroup(
      TurnToTarget(m_vision, m_gyro, m_driveTrain),
      SetArmAngle(m_arm, [m_vision] {
          double distance = m_vision->getPortDistance() / 1000;
          return 15.5504 + (130.439 / (distance + 2.46224));
        }), // SetArmAngle
        RunShooter(m_shooter, [m_vision] {
          double distance = m_vision->getPortDistance() / 1000;
          return 10648.9 + 1447.44 * distance;
        }) // RunShooter
    ), // Parallel Command Group

    UnloadMagazine(m_ballHolder, m_feeder),

    frc2::ParallelCommandGroup(
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 6),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0, 0, 0, 0, false)
    ) // Parallel Deadline Group

  ); // Add Commands

}
