// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SixBallAutoB.h"

SixBallAutoB::SixBallAutoB(FloorIntake* m_floorIntake, DriveTrain* m_driveTrain, Shooter* m_shooter, BallHolder* m_ballHolder, Feeder* m_feeder, Gyro* m_gyro, Vision* m_vision, Arm* m_arm) {

  // 6B
  AddCommands(
    // //start timer
    frc2::InstantCommand([this] {std::cout << "Auto Started\n"; }),

    AutoTarget(4.4_m, m_arm, m_shooter),

    UnloadMagazine(m_ballHolder, m_feeder, m_floorIntake),

    TurnToTarget(m_gyro, m_driveTrain, 90.0_deg),

    frc2::ParallelDeadlineGroup(
      DriveRunProfile(m_driveTrain, "6BCollectBalls"),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true),
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 6_deg)
    ), //Parallel Deadline Group

    TurnToTarget(m_gyro, m_driveTrain, 90.0_deg),
    frc2::ParallelDeadlineGroup(

      DriveRunProfile(m_driveTrain, "6BToShoot"),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true)),      

    SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0, 0, 0, 0, false),

  
    AutoTarget(m_vision, m_arm, m_shooter, m_gyro, m_driveTrain),
  

    UnloadMagazine(m_ballHolder, m_feeder, m_floorIntake, true), //unload all balls
    //reset arm position, rampdown shooter, stop floor intake
    frc2::ParallelDeadlineGroup(
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 6_deg),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0, 0, 0, 0, false)
    ) // Parallel Deadline Group
    //end timer and print out autonomous length
    // frc2::InstantCommand([this] { m_timer.Stop(); std::cout << units::time::to_string(m_timer.Get()) << std::endl; m_timer.Reset(); })
  ); // Add Commands

}
