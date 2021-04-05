// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/EightBallAutoA.h"

#include <frc2/command/WaitUntilCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
EightBallAutoA::EightBallAutoA(FloorIntake* floorIntake, DriveTrain* driveTrain, Shooter* shooter, BallHolder* ballHolder, Feeder* feeder, Gyro* gyro, Vision* vision, Arm* arm) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());
  

  AddCommands(

    frc2::InstantCommand([this, gyro, driveTrain, vision] {
      m_timer.Start();
      m_startingRightEncoder = driveTrain->getRightEncoder(); //Reads starting Right drivetrain encoder count
      m_startingLeftEncoder = driveTrain->getLeftEncoder(); //Reads starting left drivetrain encoder count
      m_targetAngle = gyro->GetYaw() + vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / vision->getPortDistance()); //Sets target Gyro angle
      
       }), // instant command

    frc2::ParallelDeadlineGroup( 
      DriveMotionControl(driveTrain, gyro, -3.5_m, 0_mps, -2_mps, -2_mps, -3_mps_sq, m_targetAngle),
      AutoTarget([this, driveTrain] {
          return units::meter_t(m_startingDistance + DriveTrainConstants::kMetersPerUnit * ((driveTrain->getRightEncoder() - m_startingRightEncoder) + (driveTrain->getLeftEncoder() - m_startingLeftEncoder)) / 2);
          }, arm, shooter, true
      ), // AutoTarget
      
      frc2::SequentialCommandGroup(
        frc2::WaitUntilCommand([this, arm, driveTrain] {
          return units::math::fabs(arm->GetArmAngleMotorUnits() - arm->CalculateAngleFromDistance(units::meter_t(m_startingDistance + DriveTrainConstants::kMetersPerUnit * ((driveTrain->getRightEncoder() - m_startingRightEncoder) + (driveTrain->getLeftEncoder() - m_startingLeftEncoder)) / 2))) < 1_deg;
        }), // wait until command group
        UnloadMagazine(ballHolder, feeder)
      ) // sequential command group
    ), // parallel deadline group

    frc2::ParallelCommandGroup(
      DriveMotionControl(driveTrain, gyro, -2_m, -2_mps, -2_mps, -2_mps, -3_mps_sq, 90_deg),
      SetBallManipulation(feeder, ballHolder, floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true),      
      RunShooter(shooter, 0.0),
      SetArmAngle(arm, 6_deg)
    ), // Parallel Command Group

    DriveMotionControl(driveTrain, gyro, -1_m, -2_mps, 0_mps, -2_mps, 3_mps_sq, 90_deg),
    DriveMotionControl(driveTrain, gyro, 1_m, 0_mps, 0_mps, 2_mps, 3_mps_sq, 90_deg),
    AutoTarget(vision, arm, shooter, gyro, driveTrain),
    frc2::WaitUntilCommand([arm] { return units::math::fabs(arm->GetArmAngleMotorUnits() - arm->GetTargetPositionUnits()) < 1_deg; }),
    UnloadMagazine(ballHolder, feeder)



  ); //Add Commands


}
