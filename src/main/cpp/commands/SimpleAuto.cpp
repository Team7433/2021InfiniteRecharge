// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SimpleAuto.h"


// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SimpleAuto::SimpleAuto(Feeder * m_feeder, BallHolder * m_ballholder, FloorIntake * m_floorIntake, DriveTrain * m_driveTrain, Arm * m_arm, Vision * m_vision, Gyro * m_gyro, Shooter * m_shooter) {

  //Auto to run 6A
  AddCommands(
    frc2::ParallelCommandGroup(
      SetArmAngle(m_arm, [] {
        double distance = 4.4;
        return units::degree_t(15.5504 + (130.439 / (distance + 2.46224)));
      }), // SetArmAngle
      RunShooter(m_shooter, [] {
        double distance = 4.4;
        return 10648.9 + 1447.44 * distance;
      }) // RunShooter
    ), // ParallelCommandGroup - Get into shooting mode
    UnloadMagazine(m_ballholder, m_feeder, m_floorIntake),
    frc2::ParallelDeadlineGroup(
      DriveRunProfile(m_driveTrain, "6ACollectBalls"),
      SetBallManipulation(m_feeder, m_ballholder, m_floorIntake, 0.45, 0.3, 0.3, 0, /* Storing */ true),
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 7_deg)
    ),
    frc2::ConditionalCommand(
      frc2::ParallelCommandGroup( // ### Future Todo: These can probably be hard coded and therefore can get to position earlier
        SetArmAngle(m_arm, [m_vision] {
          double distance = m_vision->getPortDistance().to<double>();
          return units::degree_t(15.5504 + (130.439 / (distance + 2.46224)));
        }), // SetArmAngle
        RunShooter(m_shooter, [m_vision] {
          double distance = m_vision->getPortDistance().to<double>();
          return 10648.9 + 1447.44 * distance;
        }), // RunShooter
        TurnToTarget(m_vision, m_gyro, m_driveTrain)
      ), // ParallelCommandGroup
      frc2::InstantCommand([this] { // Not Detected
        std::cout << "@@@ Auto - Did not detect target to target towards @@@\n"; 
        Cancel(); // Do not run the remainder of auto if cannot find target
      }), 
      [m_vision] {
        bool detected = m_vision->getPowerPortDetected();
        std::cout << "Detected: " << detected << "\n";
        frc::SmartDashboard::PutBoolean("Testing/IsDetected", detected);
        return detected;
      } // Conditional Condition
    ), // ConditionalCommand TargetDetected
    UnloadMagazine(m_ballholder, m_feeder, m_floorIntake),
    frc2::ParallelDeadlineGroup(
    RunShooter(m_shooter, 0.0),
    SetArmAngle(m_arm, 6_deg),
    SetBallManipulation(m_feeder, m_ballholder, m_floorIntake, 0, 0, 0, 0, false)
    )
  ); // AddCommands



}
