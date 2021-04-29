// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SixBallAutoA.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SixBallAutoA::SixBallAutoA(FloorIntake* m_floorIntake, DriveTrain* m_driveTrain, Shooter* m_shooter, BallHolder* m_ballHolder, Feeder* m_feeder, Gyro* m_gyro, Vision* m_vision, Arm* m_arm) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

   AddCommands(
    // //start timer
    frc2::InstantCommand([this] {std::cout << "Auto Started\n"; }),
    DriveMotionControl(m_driveTrain, m_gyro, -0.5_m, 0_mps, 0_mps, -1_mps, -1_mps_sq, [m_gyro] {return m_gyro->GetYaw();}),
    DriveMotionControl(m_driveTrain, m_gyro, 0.5_m, 0_mps, 0_mps, 1_mps, 1_mps_sq, [m_gyro] {return m_gyro->GetYaw();}),
    SetArmAngle(m_arm, 30_deg),
    AutoTarget(m_vision, m_arm, m_shooter, m_gyro, m_driveTrain),
    UnloadMagazine(m_ballHolder, m_feeder, m_floorIntake),
    TurnToTarget(m_gyro, m_driveTrain, 90.0_deg),

    frc2::ParallelDeadlineGroup(
      // DriveRunProfile(m_driveTrain, "6BCollectBalls"),
      DriveMotionControl(m_driveTrain, m_gyro, 4_m, 0_mps, 0_mps, 3_mps, 3_mps_sq, 90_deg),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true),
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 6_deg)
    ), //Parallel Deadline Group
    frc2::InstantCommand([] {std::cout << "WORKING\n";}),
    frc2::ParallelDeadlineGroup(
      // DriveRunProfile(m_driveTrain, "6BToShoot"),
      DriveMotionControl(m_driveTrain, m_gyro, -4_m, 0_mps, 0_mps, -3_mps, -3_mps_sq, 88_deg),
          frc2::InstantCommand([] {std::cout << "WORKING2\n";}),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true)

      ),      
    frc2::InstantCommand([] {std::cout << "WORKING3\n";}),
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
