// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SimpleAuto.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

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

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SimpleAuto::SimpleAuto(Feeder * m_feeder, BallHolder * m_ballHolder, FloorIntake * m_floorIntake, DriveTrain * m_driveTrain, Arm * m_arm, Vision * m_vision, Gyro * m_gyro, Shooter * m_shooter) {

  AddCommands(
    frc2::ParallelCommandGroup(
      SetArmAngle(m_arm, [] {
        double distance = 4.4;

        // double Angle = 0.00262689 * (std::pow(distance, 6)) + -0.0721799 * (std::pow(distance, 5)) + 0.767968 * (std::pow(distance, 4)) + -4.01135 * (std::pow(distance, 3)) + 11.3266 * (std::pow(distance, 2)) + -21.1942 * (std::pow(distance, 1)) + 56.4509;
        double Angle = 15.5504 + (130.439 / (distance + 2.46224));
        frc::SmartDashboard::PutNumber("AutoArmAngle", Angle);
        return Angle;

      }),
      RunShooter(m_shooter, [] {
        double distance = 4.4;

        // double Speed = -0.67217 * (std::pow(distance, 6)) + 17.5164 * (std::pow(distance, 5)) - 168.137 * (std::pow(distance, 4)) + 707.557 * (std::pow(distance, 3)) - 1182.52 * (std::pow(distance, 2)) + 1721.41 * (std::pow(distance, 1)) + 9963.13;
        double Speed = 10648.9 + 1447.44 * distance;
        frc::SmartDashboard::PutNumber("TargetSpeed", Speed);

        return Speed;
      })
    )
  
  
  )



}
