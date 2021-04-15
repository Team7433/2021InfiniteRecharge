// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ThreeBallAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ThreeBallAuto::ThreeBallAuto(FloorIntake* floorIntake, DriveTrain* driveTrain, Shooter* shooter, BallHolder* ballHolder, Feeder* feeder, Gyro* gyro, Vision* vision, Arm* arm, AutoVaribles* autoVaribles) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

  AddCommands(

    // AutoTarget(vision, arm, shooter, gyro, driveTrain),
    AutoTarget(3_m, arm, shooter),
    UnloadMagazine(ballHolder, feeder, floorIntake, true),
    frc2::ParallelDeadlineGroup(
      DriveMotionControl(driveTrain, gyro, -2.1_m, 0_mps, 0_mps, -1_mps, 0.75_mps_sq, [gyro] {return gyro->GetYaw();}),
      SetBallManipulation(feeder, ballHolder, floorIntake, 0, 0, 0, 0, false),
      RunShooter(shooter, 0.0)
    ),
    DriveMotionControl(driveTrain, gyro, 0.5_m, 0_mps, 0_mps, 1_mps, 1_mps_sq, [gyro] {return gyro->GetYaw();}),
    SetArmAngle(arm, 6_deg)

  ); //Add Commands

}
