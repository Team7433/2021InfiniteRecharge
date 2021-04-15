// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SixBallAutoC.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SixBallAutoC::SixBallAutoC(FloorIntake* floorIntake, DriveTrain* driveTrain, Shooter* shooter, BallHolder* ballHolder, Feeder* feeder, Gyro* gyro, Vision* vision, Arm* arm, AutoVaribles* autoVaribles) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  AddCommands(    
      frc2::InstantCommand([this, gyro, driveTrain, vision, autoVaribles] {
      autoVaribles->a_leftStartPos = driveTrain->getLeftDistance();
      autoVaribles->a_rightStartPos = driveTrain->getRightDistance();
      autoVaribles->a_targetAngle = gyro->GetYaw(); //Sets target Gyro angle

    }), // instant command


    //Drive Backward while unloading
    frc2::ParallelDeadlineGroup( 
      DriveMotionControl(driveTrain, gyro, 2.3_m, 0_mps, 0_mps, 1_mps, 1_mps_sq, [autoVaribles] { return autoVaribles->a_targetAngle; } ),
      AutoTarget([this, driveTrain, autoVaribles] {
          return (3_m + ((driveTrain->getRightDistance() - autoVaribles->a_rightStartPos) + (driveTrain->getLeftDistance() - autoVaribles->a_leftStartPos)) / 2);
          }, arm, shooter, true, true, driveTrain
      ), // AutoTarget
      
      frc2::SequentialCommandGroup(
        frc2::WaitUntilCommand([this, arm, driveTrain, autoVaribles] {
          return units::math::fabs(arm->GetArmAngleMotorUnits() - arm->CalculateAngleFromDistance((3_m + ((driveTrain->getRightDistance() - autoVaribles->a_rightStartPos) + (driveTrain->getLeftDistance() - autoVaribles->a_leftStartPos)) / 2))) < 1_deg;
        }), // wait until command group
        UnloadMagazine(ballHolder, feeder, floorIntake, true)
      ) // sequential command group
    ), // parallel deadline group

    frc2::ParallelDeadlineGroup(
      DriveMotionControl(driveTrain, gyro, 1.8_m, 0_mps, 0_mps, 2_mps, 1_mps_sq, 90_deg),
      SetBallManipulation(feeder, ballHolder, floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true),      
      RunShooter(shooter, 0.0),
      SetArmAngle(arm, 6_deg)
    ), // Parallel Command Group

    AutoTarget(vision, arm, shooter, gyro, driveTrain),
    UnloadMagazine(ballHolder, feeder, floorIntake, true),
    TurnToTarget(gyro, driveTrain, -168_deg),
    frc2::ParallelDeadlineGroup(
      frc2::SequentialCommandGroup(
        DriveMotionControl(driveTrain, gyro, 1.9_m, 0_mps, 1_mps, 2_mps, 3_mps_sq, [gyro] { return gyro->GetYaw(); }),
        DriveMotionControl(driveTrain, gyro, 1.0_m, 1_mps, 0_mps, 1_mps, 1_mps_sq, [gyro] { return gyro->GetYaw(); })
      ),
      SetBallManipulation(feeder, ballHolder, floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true),
      RunShooter(shooter, 0.0),
      SetArmAngle(arm, 6_deg)
    )

); // Add commands

}
