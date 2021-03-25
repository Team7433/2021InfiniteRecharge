// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SimpleAuto.h"

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

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SimpleAuto::SimpleAuto(Feeder * m_feeder, BallHolder * m_ballholder, FloorIntake * m_floorIntake, DriveTrain * m_driveTrain, Arm * m_arm, Vision * m_vision, Gyro * m_gyro, Shooter * m_shooter) {

  //Auto to run 6A
  AddCommands(
    frc2::ParallelCommandGroup(
      SetArmAngle(m_arm, [] {
        double distance = 4.4;
        return 15.5504 + (130.439 / (distance + 2.46224));
      }), // SetArmAngle
      RunShooter(m_shooter, [] {
        double distance = 4.4;
        return 10648.9 + 1447.44 * distance;
      }) // RunShooter
    ), // ParallelCommandGroup - Get into shooting mode
    UnloadMagazine(m_ballholder, m_feeder),
    frc2::ParallelCommandGroup(
      DriveRunProfile(m_driveTrain, "6ACollectBalls"),
      SetBallManipulation(m_feeder, m_ballholder, m_floorIntake, 0.5, 0.3, 0.3, 0, /* Storing */ true),
      RunShooter(m_shooter, 0.0),
      SetArmAngle(m_arm, 6)
    ),
    frc2::ConditionalCommand(
      frc2::ParallelCommandGroup( // ### Future Todo: These can probably be hard coded and therefore can get to position earlier
        SetArmAngle(m_arm, [m_vision] {
          double distance = m_vision->getPortDistance() / 1000;
          return 15.5504 + (130.439 / (distance + 2.46224));
        }), // SetArmAngle
        RunShooter(m_shooter, [m_vision] {
          double distance = m_vision->getPortDistance() / 1000;
          return 10648.9 + 1447.44 * distance;
        }), // RunShooter
        TurnToTarget(m_vision, m_gyro, m_driveTrain)
      ), // ParallelCommandGroup
      frc2::InstantCommand([this] { // Not Detected
        std::cout << "@@@ Auto - Did not detect target to target towards @@@\n"; 
        Cancel(); // Do not run the remainder of auto if cannot find target
      }), 
      [m_vision] {
        return m_vision->getPowerPortDetected();
      } // Conditional Condition
    ), // ConditionalCommand TargetDetected
    UnloadMagazine(m_ballholder, m_feeder),
    RunShooter(m_shooter, 0.0),
    SetArmAngle(m_arm, 6)
  ); // AddCommands



}
