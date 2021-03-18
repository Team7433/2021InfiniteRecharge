/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/ConditionalCommand.h>
#include <POVButton.h>

#include "commands/ExampleCommand.h"
#include "commands/DriveWithJoystick.h"
#include "commands/SetFloorIntake.h"
#include "commands/RunShooter.h"
#include "commands/SetBallHolder.h"
#include "commands/SetFeeder.h"
#include "commands/SetBallManipulation.h"
#include "commands/ChangeCamMode.h"
#include"commands/GoToAngle.h"
#include "commands/TurnToTarget.h"
#include "commands/SetArmAngle.h"
#include "commands/ManualArmControl.h"
#include "commands/DistanceSet.h"
#include "commands/DriveRunProfile.h"

#include "subsystems/ExampleSubsystem.h"
#include "subsystems/FloorIntake.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/BallHolder.h"
#include "subsystems/Feeder.h"
#include "subsystems/Gyro.h"
#include "subsystems/Vision.h"
#include "subsystems/Arm.h"

#include "Constants.h"

using namespace DriverControls;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void zeroOutputDisabled();
  void ResetStartOfTeleop();
 private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  FloorIntake m_floorIntake;
  DriveTrain m_driveTrain;
  BallHolder m_ballholder;
  Feeder m_feeder;
  Shooter m_shooter;
  Gyro m_gyro;
  Vision m_vision;
  Arm m_arm;

  ExampleCommand m_autonomousCommand;


  //Joysticks
  frc::Joystick m_driverStick{kMainDriverStickId};
  frc::XboxController m_operatorController{kOperatorControllerId};
  frc::Joystick m_buttonBox{kButtonBoxId};

  std::string m_pathName = "JustForward";
  std::string m_pathName2 = "JustForwardSlow";

  // frc::Joystick m_buttonBOX{kButtonBoxId}; //Used for testing in pits and at home

  void ConfigureButtonBindings();
};
