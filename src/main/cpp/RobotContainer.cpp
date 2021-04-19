/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <units/length.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitUntilCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() : m_vision(&m_arm)
{
  // Initialize all of your commands and subsystems here
  m_driveTrain.SetDefaultCommand(DriveWithJoystick(&m_driverStick, &m_driveTrain));
  m_strip.SetDefaultCommand(StatusLight(&m_strip, [this] { return m_arm.GetArmAngleMotorUnits(); }, [this] { return m_arm.GetTargetPositionUnits(); }, [this] { return m_vision.getPowerPortDetected();}, [this] { return m_shooter.GetVelocityLoopTarget(); } ));

  m_autoChooser.SetDefaultOption("SixBallAutoC", 0);
  m_autoChooser.AddOption("SixBallAutoB", 1);
  m_autoChooser.AddOption("ThreeBallAuto", 2);
  //m_arm.SetDefaultCommand(ManualArmControl(&m_arm, &m_operatorController));
  // Configure the button bindings
  ConfigureButtonBindings();

  frc::SmartDashboard::PutData(&m_autoChooser);
  frc::SmartDashboard::PutNumber("Custom/Speed", 16000);
  frc::SmartDashboard::PutNumber("Custom/Angle", 29);
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here
  // frc2::JoystickButton(&m_operatorController, 1).WhenPressed(frc2::ParallelCommandGroup(SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0.5, 0.3, 0.3, 0, /* Storing */ true); , frc2::InstantCommand([this] {m_intakeState = RobotContainerConstants::storing;}))); //Story
  frc2::JoystickButton(&m_operatorController, 1).WhenPressed(&m_storing); //Story
  frc2::JoystickButton(&m_operatorController, 2).WhenPressed(&m_stop); //stoppy
  frc2::JoystickButton(&m_operatorController, 3).WhileHeld(frc2::InstantCommand( [this] {if (units::math::fabs(m_arm.GetArmAngleUnits() - m_arm.GetTargetPositionUnits())) {m_shooting.Schedule(false);} } ) ); //shooty
  // frc2::JoystickButton(&m_operatorController, 4).WhenPressed(SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, -0.5, -0.3, -0.3, -0.3, false)); //Reversy
  // frc2::JoystickButton(&m_operatorController, 4).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; } )); //Reversy
  // frc2::JoystickButton(&m_operatorController, 4).WhenReleased(frc2::InstantCommand([this] { GetIntakeCommand().Schedule();}));
  // frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; }), false);
  // frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; }), false);
  
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_intakeState; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; } ), false);
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_intakeState; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; } ), false);
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhenReleased(frc2::InstantCommand([this] { GetIntakeCommand().Schedule();}));
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhenReleased(frc2::InstantCommand([this] { GetIntakeCommand().Schedule();}));

  frc2::JoystickButton(&m_driverStick, 11).WhenPressed(SixBallAutoC(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm, &m_autoVaribles));

  frc2::JoystickButton(&m_driverStick, 12).WhenPressed(frc2::SequentialCommandGroup(
    DriveMotionControl(&m_driveTrain, &m_gyro, 2_m, 0_mps, 1_mps, 2_mps, 3_mps_sq, [this] { return m_gyro.GetYaw(); }),
    DriveMotionControl(&m_driveTrain, &m_gyro, 0.8_m, 1_mps, 0_mps, 1_mps, 1_mps_sq, [this] { return m_gyro.GetYaw(); })
  )); 

  frc2::JoystickButton(&m_driverStick, 7).WhenPressed(RunShooter(&m_shooter, 17000.00));
  // frc2::JoystickButton(&m_driverStick, 9).WhenPressed(RunShooter(&m_shooter, [] { return frc::SmartDashboard::GetNumber("Shooter/Custom Speed", 0); }));
  frc2::JoystickButton(&m_driverStick, 9).WhenPressed(frc2::InstantCommand([this] {m_gyro.Reset();}));
  frc2::JoystickButton(&m_driverStick, 8).WhenPressed(RunShooter(&m_shooter, 0.0));

  frc2::POVButton(&m_operatorController, 0).WhenPressed(SetArmAngle(&m_arm, 29_deg));
  frc2::POVButton(&m_operatorController, 90).WhenPressed(SetArmAngle(&m_arm, 24_deg));
  frc2::POVButton(&m_operatorController, 180).WhenPressed(frc2::ParallelCommandGroup(SetArmAngle(&m_arm, 6_deg), RunShooter(&m_shooter, 0.0)));
  frc2::POVButton(&m_operatorController, 270).WhenPressed(SetArmAngle(&m_arm, 40_deg));

  
  frc2::AxisButton(&m_operatorController, true, frc::XboxController::JoystickHand::kLeftHand).WhenPressed(frc2::InstantCommand( [this] { m_feeder.RunRevolutions(1, 5, 5); } ));
  frc2::AxisButton(&m_operatorController, false, frc::XboxController::JoystickHand::kLeftHand).WhenPressed(frc2::InstantCommand( [this] { m_feeder.RunRevolutions(-1, 5, 5); } ));

  // frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhenPressed(frc2::InstantCommand( [this] {m_feeder.SetFeeder(-0.3); m_ballholder.SetMagazine(-0.3); m_ballholder.SetIndexer(-0.3);} ));
  // frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhenReleased(frc2::InstantCommand( [this] {m_feeder.SetFeeder(0.0); m_ballholder.SetMagazine(0.0); m_ballholder.SetIndexer(0.0);} ));
  // frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhenPressed(frc2::InstantCommand([this] {m_floorIntake.Set(FloorIntakeConstants::Out, -0.65);} ));
  // frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhenReleased(frc2::InstantCommand([this] {m_floorIntake.Set(FloorIntakeConstants::Out, 0.0);} ));
 


  // frc2::POVButton(&m_driverStick, 0).WhenPressed(SetArmAngle(&m_arm, [] {
  //   double newAngle = frc::SmartDashboard::GetNumber("Custom/Angle", 0) + 1;
  //   frc::SmartDashboard::PutNumber("Custom/Angle", newAngle);
  //   return newAngle;
  // }));

  // frc2::POVButton(&m_driverStick, 180).WhenPressed(SetArmAngle(&m_arm, [] {
  //   double newAngle = frc::SmartDashboard::GetNumber("Custom/Angle", 0) - 1;
  //   frc::SmartDashboard::PutNumber("Custom/Angle", newAngle);
  //   return newAngle;
  // }));

  // frc2::POVButton(&m_driverStick, 270).WhenPressed(RunShooter(&m_shooter, [] {
  //   double newSpeed = frc::SmartDashboard::GetNumber("Custom/Speed", 0) - 100.0;
  //   frc::SmartDashboard::PutNumber("Custom/Speed", newSpeed);
  //   return newSpeed;
  // }));

  // frc2::POVButton(&m_driverStick, 90).WhenPressed(RunShooter(&m_shooter, [] {
  //   double newSpeed = frc::SmartDashboard::GetNumber("Custom/Speed", 0) + 100.0;
  //   frc::SmartDashboard::PutNumber("Custom/Speed", newSpeed);
  //   return newSpeed;
  // }));

  // auto set angle and speed of arm and shooter
  frc2::JoystickButton(&m_driverStick, 5).WhenPressed(frc2::InstantCommand([this] {m_feeder.RunRevolutions(32.6, 5, 40);}));
  
  frc2::JoystickButton(&m_driverStick, 2).WhenPressed(frc2::ConditionalCommand(AutoTarget(&m_vision, &m_arm, &m_shooter, &m_gyro, &m_driveTrain), frc2::InstantCommand([] {std::cout << "no target detected\n";}), [this] { return m_vision.getPowerPortDetected(); }));

  frc2::JoystickButton(&m_driverStick, 4).WhenPressed(SetArmAngle(&m_arm, 10_deg));


  frc2::JoystickButton(&m_driverStick, 3).WhileHeld(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] { 
      m_startingDistance = m_vision.getPortDistance(); //Reads starting distance using limelight
      m_startingRightEncoder = m_driveTrain.getRightEncoder(); //Reads starting Right drivetrain encoder count
      m_startingLeftEncoder = m_driveTrain.getLeftEncoder(); //Reads starting left drivetrain encoder count
      m_targetAngle = units::degree_t(m_gyro.GetYaw() + m_vision.getPowerPortHorizontalAngle() - units::math::atan(160_mm / m_vision.getPortDistance())); //Sets target Gyro angle
      frc::SmartDashboard::PutNumber("ShootOnTheRun/targetAngle", m_targetAngle.to<double>());
    }),
    // TurnToTarget(&m_gyro, &m_driveTrain, [this] { return m_targetAngle; }),
    frc2::ConditionalCommand(
      frc2::ParallelCommandGroup(
        
        GyroDrive(&m_gyro, &m_driveTrain, [this] {return m_targetAngle; }, [this] { return -m_driverStick.GetY(); }),
        AutoTarget([this] {
          return units::meter_t(m_startingDistance.to<double>() + DriveTrainConstants::kMetersPerUnit * ((m_driveTrain.getRightEncoder() - m_startingRightEncoder) + (m_driveTrain.getLeftEncoder() - m_startingLeftEncoder)) / 2);
        }, &m_arm, &m_shooter, true, true, &m_driveTrain)

      ),
      frc2::InstantCommand([] {std::cout << "No Target Detected \n";}),
      [this] {return m_vision.getPowerPortDetected();}
    ) // conditional command

  ));


  // frc2::JoystickButton(&m_buttonBOX, 4).WhenPressed(RunShooter(&m_shooter, [] {
  //   double newSpeed = frc::SmartDashboard::GetNumber("ShooterSpeed", 0) - 100.0;
  //   frc::SmartDashboard::PutNumber("ShooterSpeed", newSpeed);
  //   return newSpeed;
  // }));
 ConfigureButtonBox();
} // ConfigureButtonBindings

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  switch (m_autoChooser.GetSelected())
  {
  case 0 : //SixBallAutoC
    return new SixBallAutoC(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm, &m_autoVaribles);
    break;
  case 1: //SixBallAutoB
    return new SixBallAutoB(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm);
    break;
  case 2:  //three ball auto
    return new ThreeBallAuto(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm, &m_autoVaribles);
    break;
  }
  return nullptr;
}


frc2::ParallelCommandGroup& RobotContainer::GetIntakeCommand() {

  switch (m_intakeState)
  {
  case RobotContainerConstants::IntakeState::shooting:
    return m_shooting;
    break;
  
  case RobotContainerConstants::IntakeState::stop:
    return m_stop;
    break;

  case RobotContainerConstants::IntakeState::storing:
    return m_storing;
    break;
  }

}

void RobotContainer::zeroOutputDisabled()
{
  m_shooter.ManualControl(0);
}

void RobotContainer::ResetStartOfTeleop()
{

  m_arm.SetLock(ArmConstants::Unlock);
}

void RobotContainer::ConfigureButtonBox() {
  #ifdef ButtonBox

    frc2::JoystickButton(&m_buttonBox, 1).WhenPressed([this] { 
      m_gyro.Reset(); 
      // m_climber.RunRevolutions(5.0, 1, 2);
    });
    frc2::JoystickButton(&m_buttonBox, 2).WhileHeld(frc2::InstantCommand([this] {m_climber.RunDynamicRevoltions([this] {return 2.5*(-(m_buttonBox.GetRawAxis(3)+2));});}));

    // frc2::JoystickButton(&m_driverStick, 5).WhenPressed(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock); m_climber.SetOutput(0.0);} ));
    // frc2::JoystickButton(&m_driverStick, 4).WhenPressed(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock); m_climber.SetOutput(0.6);} ));

    frc2::JoystickButton(&m_buttonBox , 5).WhenPressed(frc2::InstantCommand([this] {m_arm.SetLock(ArmConstants::Lock_Position::Lock);}));
    frc2::JoystickButton(&m_buttonBox, 6).WhenPressed(frc2::InstantCommand([this] {m_arm.SetLock(ArmConstants::Lock_Position::Unlock);}));
    frc2::JoystickButton(&m_buttonBox, 3).WhileHeld(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock); m_climber.SetOutput([this] {return m_buttonBox.GetY();}); }));
    frc2::JoystickButton(&m_buttonBox, 4).WhenPressed(frc2::InstantCommand([this] {m_climber.SetOutput([] {return 0.0;});} ) );
    // frc2::JoystickButton(&m_buttonBox, 1).WhenPressed(frc2::InstantCommand([this] {m_driveTrain.playSong(); });
    // frc2::JoystickButton(&m_buttonBOX, 4).WhenPressed(RunShooter(&m_shooter, [] {
    //   double newSpeed = frc::SmartDashboard::GetNumber("ShooterSpeed", 0) - 100.0;
    //   frc::SmartDashboard::PutNumber("ShooterSpeed", newSpeed);
    //   return newSpeed;
    // }));
    
  #endif //ButtonBox
}