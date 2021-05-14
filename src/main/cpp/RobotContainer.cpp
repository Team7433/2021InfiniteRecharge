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
  m_strip.SetDefaultCommand(StatusLight(&m_strip, &m_arm, &m_shooter, &m_driveTrain, &m_vision));

  // Sets up autonomous chooser
  m_autoChooser.SetDefaultOption("SixBall AutoA", 0);
  m_autoChooser.AddOption("SixBall AutoB", 1);
  m_autoChooser.AddOption("SixBall AutoC", 2);
  m_autoChooser.AddOption("ThreeBall Auto", 3);
  m_autoChooser.AddOption("No Auto", 4);
  m_autoChooser.AddOption("Opposing Trench Auto", 5);

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
  frc2::JoystickButton(&m_operatorController, 4).WhenPressed(&m_shooting); // shooting
  frc2::JoystickButton(&m_operatorController, 4).WhenReleased(&m_storing);

  frc2::JoystickButton(&m_operatorController, 6).WhenReleased(
    frc2::ParallelCommandGroup(
      frc2::InstantCommand([this] {m_storing.Schedule(); m_driveTrain.SstVelocityRamping(0.0);}),
      SetArmAngle(&m_arm, 5_deg),
      RunShooter(&m_shooter, 0.0)
    ) // Parallel Command
  ); // When released 

  frc2::JoystickButton(&m_operatorController, 5).WhenPressed(
    frc2::ParallelCommandGroup(
      RunShooter(&m_shooter, 0.0),
      SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0, 0, 0, 0, false)
    ) // Parallel Command
  ); // WhenPressed

  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_intakeState; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; } ), false);
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhileHeld(ReverseMagazine(&m_ballholder, &m_floorIntake, &m_feeder, [this] { return m_intakeState; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::L_trig) > 0.6; }, [this] { return m_operatorController.GetRawAxis(frc::XboxTriggers::R_trig) > 0.6; } ), false);
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::R_trig).WhenReleased(frc2::InstantCommand([this] { GetIntakeCommand().Schedule();}));
  frc2::TriggerButton(&m_operatorController, frc::XboxTriggers::L_trig).WhenReleased(frc2::InstantCommand([this] { GetIntakeCommand().Schedule();}));


  frc2::POVButton(&m_operatorController, 0).WhenPressed(frc2::ParallelCommandGroup(SetArmAngle(&m_arm, 80_deg), frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(1.0);}) ));
  frc2::POVButton(&m_operatorController, 90).WhenPressed(frc2::ParallelCommandGroup(SetArmAngle(&m_arm, 24_deg), frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(0.5);}) ));
  frc2::POVButton(&m_operatorController, 270).WhenPressed(frc2::ParallelCommandGroup(SetArmAngle(&m_arm, 31_deg), frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(1.0);})));
  frc2::POVButton(&m_operatorController, 180).WhenPressed(frc2::ParallelCommandGroup(SetArmAngle(&m_arm, 5_deg), RunShooter(&m_shooter, 0.0), frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(0.0);}) ));
  frc2::JoystickButton(&m_driverStick, 4).WhenPressed(frc2::ParallelCommandGroup(SetArmAngle(&m_arm, 31_deg), frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(1.0);}) ));


  frc2::NetworkButton("Arm/LockButton").WhenPressed(frc2::InstantCommand([this] {m_arm.SetLock(ArmConstants::Lock_Position::Lock); } ));
  frc2::NetworkButton("Arm/LockButton").WhenReleased(frc2::InstantCommand([this] {m_arm.SetLock(ArmConstants::Lock_Position::Unlock); } ));

  frc2::AxisButton(&m_operatorController, true, frc::XboxController::JoystickHand::kLeftHand).WhenPressed(frc2::InstantCommand( [this] { m_feeder.RunRevolutions(1, 5, 5); } ));
  frc2::AxisButton(&m_operatorController, false, frc::XboxController::JoystickHand::kLeftHand).WhenPressed(frc2::InstantCommand( [this] { m_feeder.RunRevolutions(0.5, 5, 5); } ));

  frc2::JoystickButton(&m_driverStick, 7).WhenPressed(RunShooter(&m_shooter, 17000.00));
  frc2::JoystickButton(&m_driverStick, 8).WhenPressed(RunShooter(&m_shooter, 0.0));

  // auto set angle and speed of arm and shooter
  frc2::JoystickButton(&m_driverStick, 5).WhenPressed(frc2::InstantCommand([this] {m_feeder.RunRevolutions(32.6, 5, 40);}));
  
  frc2::JoystickButton(&m_driverStick, 2).WhileHeld(frc2::ConditionalCommand(
            frc2::SequentialCommandGroup(
              frc2::InstantCommand([this] { 
                m_startingDistance = m_vision.getPortDistance(); //Reads starting distance using limelight
                m_startingRightEncoder = m_driveTrain.getRightEncoder(); //Reads starting Right drivetrain encoder count
                m_startingLeftEncoder = m_driveTrain.getLeftEncoder(); //Reads starting left drivetrain encoder count
                m_targetAngle = units::degree_t(m_gyro.GetYaw() + m_vision.getPowerPortHorizontalAngle() - units::math::atan(160_mm / m_vision.getPortDistance())); //Sets target Gyro angle
                frc::SmartDashboard::PutNumber("ShootOnTheRun/targetAngle", m_targetAngle.to<double>());
              }),

              TurnToTarget(&m_gyro, &m_driveTrain, [this] { return m_targetAngle; }),

              frc2::ParallelCommandGroup(
        
              GyroDrive(&m_gyro, &m_driveTrain, [this] {return m_targetAngle; }, [this] { return -m_driverStick.GetY(); }),
              AutoTarget([this] {
                return units::meter_t(m_startingDistance.to<double>() + DriveTrainConstants::kMetersPerUnit * ((m_driveTrain.getRightEncoder() - m_startingRightEncoder) + (m_driveTrain.getLeftEncoder() - m_startingLeftEncoder)) / 2);
              }, &m_arm, &m_shooter, true, true, &m_driveTrain)

            )

            )
            , frc2::InstantCommand([] {std::cout << "no target detected\n";}), [this] { return m_vision.getPowerPortDetected(); 
  }));

  frc2::JoystickButton(&m_driverStick, 6).WhenPressed(frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(0.0);} ));

  // frc2::JoystickButton(&m_driverStick, 4).WhenPressed(SetArmAngle(&m_arm, 10_deg));

  frc2::JoystickButton(&m_driverStick, 3).WhenPressed(frc2::ParallelCommandGroup(
    frc2::InstantCommand([this] {m_driveTrain.SstVelocityRamping(0.5);}),
    SetArmAngle(&m_arm, 66_deg),
    RunShooter(&m_shooter, 9600.0)

  ));                   
  frc2::JoystickButton(&m_operatorController, 8).WhenPressed(ClimbMode(&m_arm, &m_climber, &m_driveTrain, &m_driverStick, &m_operatorController));
  frc2::JoystickButton(&m_driverStick, 12).WhenPressed(RunShooter(&m_shooter, 6000));
  frc2::JoystickButton(&m_driverStick, 11).WhenPressed(RunShooter(&m_shooter, 8000));
  // frc2::JoystickButton(&m_driverStick, 12).WhenPressed(OpposingTrenchAuto(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm));
 ConfigureButtonBox();
} // ConfigureButtonBindings

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  switch (m_autoChooser.GetSelected())
  {
  case 0 : //SixBallAutoA
    return new SixBallAutoA(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm);
    break;
  case 1: //SixBallAutoB
    return new SixBallAutoB(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm);
    break;
  case 2:  //SixballautoC
    return new SixBallAutoC(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm, &m_autoVaribles);
    break;
  case 3: //Three ball auto
    return new ThreeBallAuto(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm, &m_autoVaribles);
    break;
  case 4: //no auto
    return nullptr;
    break;
  case 5: //opposing Trench Auto
    return new OpposingTrenchAuto(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm);
    break;
  }
}


frc2::Command& RobotContainer::GetIntakeCommand() {

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


void RobotContainer::ResetStartOfTeleop()
{
  m_arm.SetLock(ArmConstants::Unlock);
}

void RobotContainer::ConfigureButtonBox() {
  #ifdef ButtonBox

    // frc2::JoystickButton(&m_buttonBox, 1).WhenPressed([this] { 
    //   // m_gyro.Reset(); 
    //   m_arm.SetAngle(75_deg);
    //   // m_climber.RunRevolutions(5.0, 1, 2);
    // });

    frc2::JoystickButton(&m_buttonBox, 2).WhileHeld(frc2::InstantCommand([this] {std::cout << (4.53/2)*((m_buttonBox.GetRawAxis(3))+1)-1.75 << std::endl;  m_climber.RunDynamicRevoltions([this] {return (4.53/2)*(-(m_buttonBox.GetRawAxis(3))+1)-1.75;});}));

    // frc2::JoystickButton(&m_driverStick, 5).WhenPressed(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock); m_climber.SetOutput(0.0);} ));
    // frc2::JoystickButton(&m_driverStick, 4).WhenPressed(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock); m_climber.SetOutput(0.6);} ));
    frc2::JoystickButton(&m_buttonBox, 7).WhenPressed(frc2::InstantCommand( [this] {m_climber.RunDynamicRevoltions([] {return 0.0;});}));
    frc2::JoystickButton(&m_buttonBox, 8).WhenPressed(TurnToTarget(&m_gyro, &m_driveTrain, 0_deg, false));
    frc2::JoystickButton(&m_buttonBox, 4).WhenPressed(frc2::InstantCommand([this] {m_climber.SetOutput([] {return 0.0;});} ) );
    frc2::JoystickButton(&m_buttonBox , 5).WhenPressed(frc2::InstantCommand([this] {m_arm.SetLock(ArmConstants::Lock_Position::Lock);}));
    frc2::JoystickButton(&m_buttonBox, 6).WhenPressed(frc2::InstantCommand([this] {m_arm.SetLock(ArmConstants::Lock_Position::Unlock);}));
    frc2::JoystickButton(&m_buttonBox, 11).WhenPressed(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Lock);} ));
    frc2::JoystickButton(&m_buttonBox, 12).WhenPressed(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock);} ));
    frc2::JoystickButton(&m_buttonBox, 3).WhileHeld(frc2::InstantCommand([this] {m_climber.SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock); m_climber.SetOutput([this] {return m_buttonBox.GetY();}); }));
   
    // frc2::JoystickButton(&m_buttonBox, 1).WhenPressed(frc2::InstantCommand([this] {m_driveTrain.playSong(); });
    // frc2::JoystickButton(&m_buttonBOX, 4).WhenPressed(RunShooter(&m_shooter, [] {
    //   double newSpeed = frc::SmartDashboard::GetNumber("ShooterSpeed", 0) - 100.0;
    //   frc::SmartDashboard::PutNumber("ShooterSpeed", newSpeed);
    //   return newSpeed;
    // }));
    
  #endif //ButtonBox
}