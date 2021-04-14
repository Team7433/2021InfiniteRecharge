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
  std::function<void()> func = [&, this, driveTrain, gyro] {
      // m_timer.Start();
    std::cout << "Init all variables Before \n";
    // setStartedLeftPos(driveTrain->getLeftDistance());
    // this->SetStartedRightPos(driveTrain->getRightDistance());
    // m_rightStartPos = driveTrain->getRightDistance(); //Reads starting Right drivetrain encoder count
    // m_leftStartPos = driveTrain->getLeftDistance(); //Reads starting left drivetrain encoder count
    // *m_targetAngle_p = gyro->GetYaw(); // gyro->GetYaw() + vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / vision->getPortDistance()); //Sets target Gyro angle
    std::cout << "Starting Right Encoder " << m_rightStartPos.to<double>() << ", Starting left encoder " << m_leftStartPos.to<double>() << ", target angle " << units::angle::to_string(gyro->GetYaw()) << "Starting Right Encoder function: "<< units::length::to_string(GetStartRightPos()) <<std::endl;
  };

  AddCommands(
    //start timer and setup for AutoTarget
    frc2::InstantCommand(func), // instant command
    frc2::InstantCommand([this] {
      std::cout << "starting distance" << units::length::to_string(this->GetStartDistance()) << "GetRightDistance " << units::length::to_string(GetStartRightPos()) <<std::endl; } )
    //Drive Backward while unloading
    // frc2::ParallelDeadlineGroup( 
    //   DriveMotionControl(driveTrain, gyro, 2_m, 0_mps, 0_mps, 1_mps, 1_mps_sq, [gyro] { std::cout << units::angle::to_string(gyro->GetYaw()) <<std::endl; return gyro->GetYaw(); }),
      
      // AutoTarget([this, driveTrain] {
      //     std::cout << "Starting Right Encoder" << GetStartRightPos().to<double>() << ", Starting left encoder " << GetStartLeftPos().to<double>() << "starting Distance " << GetStartDistance() << std::endl; 
      //     return units::meter_t(m_startingDistance + ((driveTrain->getRightDistance() - m_rightStartPos) + (driveTrain->getLeftDistance() - m_leftStartPos)) / 2);
      //     }, arm, shooter, true
      // ), // AutoTarget
      
      // frc2::SequentialCommandGroup(
      //   frc2::WaitUntilCommand([this, arm, driveTrain] {
      //     return units::math::fabs(arm->GetArmAngleMotorUnits() - arm->CalculateAngleFromDistance(units::meter_t(m_startingDistance + ((driveTrain->getRightDistance() - m_rightStartPos) + (driveTrain->getLeftDistance() - m_leftStartPos)) / 2))) < 1_deg;
      //   }), // wait until command group
      //   UnloadMagazine(ballHolder, feeder, floorIntake, true)
      // ) // sequential command group
    // ) // parallel deadline group

    // frc2::ParallelCommandGroup(
    //   frc2::SequentialCommandGroup(
    //     TurnToTarget(gyro, driveTrain, 90_deg),
    //     DriveMotionControl(driveTrain, gyro, 2_m, 0_mps, 0_mps, 1_mps, 1_mps_sq, 90_deg)
    //   ),
    //   SetBallManipulation(feeder, ballHolder, floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true),      
    //   RunShooter(shooter, 0.0),
    //   SetArmAngle(arm, 6_deg)
    // ) // Parallel Command Group
    
    // DriveMotionControl(driveTrain, gyro, 1_m, 2_mps, 0_mps, 2_mps, 3_mps_sq, 90_deg) //Bring robot to a stop
    // //Setup for autoTarget
    // frc2::InstantCommand([this, driveTrain, gyro, vision] {
    //   m_startingRightEncoder = driveTrain->getRightEncoder(); //Reads starting Right drivetrain encoder count
    //   m_startingLeftEncoder = driveTrain->getLeftEncoder(); //Reads starting left drivetrain encoder count
    //   m_targetAngle = gyro->GetYaw() + vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / vision->getPortDistance()); //Sets target Gyro angle

    // }), // Instand Command
    // frc2::ParallelCommandGroup(
    //   DriveMotionControl(driveTrain, gyro, 1_m, 0_mps, 0_mps, 2_mps, 3_mps_sq, m_targetAngle),
    //   AutoTarget([this, driveTrain] {
    //     return units::meter_t(m_startingDistance + DriveTrainConstants::kMetersPerUnit * ((driveTrain->getRightEncoder() - m_startingRightEncoder) + (driveTrain->getLeftEncoder() - m_startingLeftEncoder)) / 2);
    //     }, arm, shooter, true
    //   ), // AutoTarget

    //   frc2::SequentialCommandGroup(
    //     frc2::WaitUntilCommand([this, arm, driveTrain] { return units::math::fabs(arm->GetArmAngleMotorUnits() -  arm->CalculateAngleFromDistance(units::meter_t(m_startingDistance + DriveTrainConstants::kMetersPerUnit * ((driveTrain->getRightEncoder() - m_startingRightEncoder) + (driveTrain->getLeftEncoder() - m_startingLeftEncoder)) / 2))) < 1_deg; }),
    //     UnloadMagazine(ballHolder, feeder, floorIntake, true)
    //   ) // sequential command group

    // ), // Parallel Command Group

    // DriveMotionControl(driveTrain, gyro, 2_m, 2_mps, 0_mps, 2_mps, -2_mps_sq, m_targetAngle), 

    // frc2::InstantCommand([this] { m_timer.Stop(); std::cout << "Autonomous Routine Run time: " << units::time::to_string(m_timer.Get()) << std::endl; m_timer.Reset();}) // Prints out autonomous runtime



  ); //Add Commands


}
