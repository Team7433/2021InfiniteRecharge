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
#include <frc/smartdashboard/SendableChooser.h>
#include <POVButton.h>
#include "util/AxisButton.h"
#include "util/TriggerButton.h"

#include <units/angle.h>
#include <units/length.h>

#include "commands/DriveWithJoystick.h"
#include "commands/SetFloorIntake.h"
#include "commands/RunShooter.h"
#include "commands/SetBallHolder.h"
#include "commands/SetFeeder.h"
#include "commands/SetBallManipulation.h"
#include "commands/ChangeCamMode.h"
#include "commands/TurnToTarget.h"
#include "commands/SetArmAngle.h"
#include "commands/ManualArmControl.h"
#include "commands/DriveRunProfile.h"
#include "commands/AutoTarget.h"
#include "commands/UnloadMagazine.h"
#include "commands/GyroDrive.h"
#include "commands/AutoTarget.h"
#include "commands/DriveMotionControl.h"
#include "commands/StatusLight.h"
#include "commands/ReverseMagazine.h"

#include "commands/EightBallAutoA.h"
#include "commands/SimpleAuto.h"
#include "commands/SixBallAutoB.h"
#include "commands/SixBallAutoC.h"
#include "commands/ThreeBallAuto.h"

#include "subsystems/FloorIntake.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/BallHolder.h"
#include "subsystems/Feeder.h"
#include "subsystems/Gyro.h"
#include "subsystems/Vision.h"
#include "subsystems/Arm.h"
#include "subsystems/AutoVaribles.h"
#include "subsystems/RGBStrip.h"
#include "subsystems/Climber.h"

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
  frc2::ParallelCommandGroup& GetIntakeCommand();

  void CoastMode() { m_driveTrain.SetCoastMode(); } // set drivetrain to coast mode
  void BrakeMode() { m_driveTrain.SetBrakeMode(); } // set drivetrain to brake mode
  void RainbowMode() {m_strip.Rainbow();}
  void ControlLight(double R, double G, double B) {m_strip.SetRGBStrip(R, G, B);}
  void SetLimelightLED(VisionConstants::LEDState state) {m_vision.SetLED(state); }
  units::degree_t GetArmAngle() {return m_arm.GetArmAngleUnits(); }
  Vision GetVisionSubsystem() { return m_vision; }
  units::degree_t GetTargetError() {return units::degree_t(m_gyro.GetYaw() + m_vision.getPowerPortHorizontalAngle() - units::math::atan(160_mm / m_vision.getPortDistance())) - m_gyro.GetYaw();}
  void zeroOutputDisabled();
  void ResetStartOfTeleop();
 private:
  // The robot's subsystems and commands are defined here...
  FloorIntake m_floorIntake;
  DriveTrain m_driveTrain;
  BallHolder m_ballholder;
  Feeder m_feeder;
  Shooter m_shooter;
  Gyro m_gyro;
  Vision m_vision;
  Arm m_arm;
  RGBStrip m_strip;
  AutoVaribles m_autoVaribles;
  Climber m_climber;

  frc::SendableChooser<int> m_autoChooser;



  // ExampleCommand m_autonomousCommand;


  //Joysticks
  frc::Joystick m_driverStick{kMainDriverStickId};
  frc::XboxController m_operatorController{kOperatorControllerId};

#ifdef ButtonBox  
  frc::Joystick m_buttonBox{kButtonBoxId};
#endif //ButtonBox


  frc2::ParallelCommandGroup m_storing{SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0.5, 0.3, 0.3, 0, /* Storing */ true), frc2::InstantCommand{[this] {m_intakeState = RobotContainerConstants::storing;}}};
  frc2::ParallelCommandGroup m_stop{
    SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0, 0, 0, 0, false),
    RunShooter(&m_shooter, 0.0), 
    frc2::InstantCommand{[this] {m_intakeState = RobotContainerConstants::stop;}}
  }; //Stopy
  frc2::ParallelCommandGroup m_shooting{SetBallManipulation{&m_feeder, &m_ballholder, &m_floorIntake, 0.5, 0.3, 0.3, 0.5, false}, frc2::InstantCommand{[this] {m_intakeState = RobotContainerConstants::shooting;}}};




  RobotContainerConstants::IntakeState m_intakeState;

  units::meter_t m_startingDistance;
  double m_startingRightEncoder;
  double m_startingLeftEncoder;
  double m_metersPerEncoder;
  units::degree_t m_targetAngle;

  // frc::Joystick m_buttonBOX{kButtonBoxId}; //Used for testing in pits and at home

  void ConfigureButtonBindings();
  void ConfigureButtonBox();
};
