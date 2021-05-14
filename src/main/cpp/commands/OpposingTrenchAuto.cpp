// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/OpposingTrenchAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OpposingTrenchAuto::OpposingTrenchAuto(FloorIntake* m_floorIntake, DriveTrain* m_driveTrain, Shooter* m_shooter, BallHolder* m_ballHolder, Feeder* m_feeder, Gyro* m_gyro, Vision* m_vision, Arm* m_arm) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand(), BarCommand());

  AddCommands(
    frc2::ParallelDeadlineGroup(
      DriveMotionControl(m_driveTrain, m_gyro, 2.1_m, 0_mps, 0_mps, 2_mps, 2.5_mps_sq, [m_gyro] {return m_gyro->GetYaw(); }),
      SetArmAngle(m_arm, 5_deg),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true)
    ),
    frc2::ParallelDeadlineGroup(
      TurnToTarget(m_gyro, m_driveTrain, [m_gyro] {return m_gyro->GetYaw() + 44_deg; } ),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true)
    ),
    frc2::ParallelDeadlineGroup(
      DriveMotionControl(m_driveTrain, m_gyro, 0.2_m, 0_mps, 0_mps, 0.5_mps, 0.5_mps_sq, [m_gyro] {return m_gyro->GetYaw(); }),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0.45, 0.5, 0.3, 0, /* Storing */ true)
    ),
    DriveMotionControl(m_driveTrain, m_gyro, -1_m, 0_mps, -2_mps, -2_mps, -2_mps_sq, [m_gyro] {return m_gyro->GetYaw(); }),
    frc2::ParallelDeadlineGroup(
      DriveMotionControl(m_driveTrain, m_gyro, -2_m, -2_mps, 0_mps, -2_mps, -2_mps_sq, [m_gyro] {return m_gyro->GetYaw() + 10_deg; }),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0, 0, 0, 0, false)
    ),
    TurnToTarget(m_gyro, m_driveTrain, [m_gyro] {return m_gyro->GetYaw() -10_deg;}),
    AutoTarget(m_vision, m_arm, m_shooter, m_gyro, m_driveTrain),
    UnloadMagazine(m_ballHolder, m_feeder, m_floorIntake, true), //unload all balls

    frc2::ParallelDeadlineGroup(
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 6_deg),
      SetBallManipulation(m_feeder, m_ballHolder, m_floorIntake, 0, 0, 0, 0, false)
    )

  );

}
