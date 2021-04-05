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

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() : m_vision(&m_arm)
{
  // Initialize all of your commands and subsystems here
  m_driveTrain.SetDefaultCommand(DriveWithJoystick(&m_driverStick, &m_driveTrain));
  //m_arm.SetDefaultCommand(ManualArmControl(&m_arm, &m_operatorController));
  // Configure the button bindings
  ConfigureButtonBindings();

  frc::SmartDashboard::PutNumber("Custom/Speed", 16000);
  frc::SmartDashboard::PutNumber("Custom/Angle", 29);
}

void RobotContainer::ConfigureButtonBindings()
{
  // Configure your button bindings here
  frc2::JoystickButton(&m_operatorController, 1).WhenPressed(SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0.5, 0.3, 0.3, 0, /* Storing */ true)); //Story
  frc2::JoystickButton(&m_operatorController, 2).WhenPressed(frc2::ParallelCommandGroup(
    SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0, 0, 0, 0, false),
    RunShooter(&m_shooter, 0.0)
  )); //Stopy
  frc2::JoystickButton(&m_operatorController, 3).WhenPressed(SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, 0.3, 0.3, 0.3, 0.5, false)); //Shooty
  frc2::JoystickButton(&m_operatorController, 4).WhenPressed(SetBallManipulation(&m_feeder, &m_ballholder, &m_floorIntake, -0.5, -0.3, -0.3, -0.3, false)); //Reversy
  frc2::JoystickButton(&m_driverStick, 11).WhenPressed(DriveRunProfile(&m_driveTrain, m_pathName2));
  // frc2::JoystickButton(&m_driverStick, 12).WhenPressed(SimpleAuto(&m_feeder, &m_ballholder, &m_floorIntake, &m_driveTrain, &m_arm, &m_vision, &m_gyro, &m_shooter));
  frc2::JoystickButton(&m_driverStick, 12).WhenPressed(SixBallAutoB(&m_floorIntake, &m_driveTrain, &m_shooter, &m_ballholder, &m_feeder, &m_gyro, &m_vision, &m_arm));
  // frc2::JoystickButton(&m_driverStick, 12).WhenPressed(TurnToTarget(&m_gyro, &m_driveTrain, 90));

  //frc2::JoystickButton(&m_driverStick, 1).WhileHeld(GoToAngle(&m_driveTrain, &m_gyro, &m_driverStick, 0.0));
  // frc2::JoystickButton(&m_driverStick, 2).WhileHeld(TurnToTarget(&m_vision, &m_gyro, &m_driveTrain));
  // frc2::JoystickButton(&m_driverStick, 2).WhenPressed(DistanceSet(&m_vision, &m_arm, &m_shooter));

  frc2::JoystickButton(&m_driverStick, 7).WhenPressed(RunShooter(&m_shooter, 17000.00));
  frc2::JoystickButton(&m_driverStick, 9).WhenPressed(RunShooter(&m_shooter, [] { return frc::SmartDashboard::GetNumber("Custom/Speed", 0); }));
  frc2::JoystickButton(&m_driverStick, 8).WhenPressed(RunShooter(&m_shooter, 0.0));
  // frc2::JoystickButton(&m_driverStick, 12).WhenPressed(SetArmAngle(&m_arm, 56));

  frc2::POVButton(&m_operatorController, 0).WhenPressed(SetArmAngle(&m_arm, 29));
  frc2::POVButton(&m_operatorController, 90).WhenPressed(SetArmAngle(&m_arm, 35));
  frc2::POVButton(&m_operatorController, 180).WhenPressed(SetArmAngle(&m_arm, 6));
  frc2::POVButton(&m_operatorController, 270).WhenPressed(SetArmAngle(&m_arm, 40));

  frc2::POVButton(&m_driverStick, 0).WhenPressed(SetArmAngle(&m_arm, [] {
    double newAngle = frc::SmartDashboard::GetNumber("Custom/Angle", 0) + 1;
    frc::SmartDashboard::PutNumber("Custom/Angle", newAngle);
    return newAngle;
  }));

  frc2::POVButton(&m_driverStick, 180).WhenPressed(SetArmAngle(&m_arm, [] {
    double newAngle = frc::SmartDashboard::GetNumber("Custom/Angle", 0) - 1;
    frc::SmartDashboard::PutNumber("Custom/Angle", newAngle);
    return newAngle;
  }));

  frc2::POVButton(&m_driverStick, 270).WhenPressed(RunShooter(&m_shooter, [] {
    double newSpeed = frc::SmartDashboard::GetNumber("Custom/Speed", 0) - 100.0;
    frc::SmartDashboard::PutNumber("Custom/Speed", newSpeed);
    return newSpeed;
  }));

  frc2::POVButton(&m_driverStick, 90).WhenPressed(RunShooter(&m_shooter, [] {
    double newSpeed = frc::SmartDashboard::GetNumber("Custom/Speed", 0) + 100.0;
    frc::SmartDashboard::PutNumber("Custom/Speed", newSpeed);
    return newSpeed;
  }));



  //frc2::POVButton(&m_operatorController, 270).WhenPressed(SetArmAngle(&m_arm, [] { return frc::SmartDashboard::GetNumber("ArmAngle", 0); }));

  // auto set angle and speed of arm and shooter
  frc2::JoystickButton(&m_driverStick, 2).WhenPressed(frc2::ConditionalCommand(
    frc2::ParallelCommandGroup(
    SetArmAngle(&m_arm, [this] {
      units::meter_t distance = m_vision.getPortDistance();

      return m_arm.CalculateAngleFromDistance(distance);;
    }),
    RunShooter(&m_shooter, [this] {
      double distance = ( m_vision.getPortDistance().convert<units::length::millimeter>() ).to<double>();
      
      return 10648.9 + 1447.44 * distance;
    }),
    TurnToTarget(&m_vision, &m_gyro, &m_driveTrain)
  ) , frc2::InstantCommand([this] {  }), 
  [this] {
    return m_vision.getPowerPortDetected();
  }));

 
    // frc2::JoystickButton(&m_driverStick, 2).WhileHeld(GyroDrive(&m_gyro, &m_driveTrain, 90.0, [this] {return -m_driverStick.GetY(); } ));


  frc2::JoystickButton(&m_driverStick, 3).WhileHeld(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] {
      m_startingDistance = m_vision.getPortDistance() / 1000; //Reads starting distance using limelight
      m_startingRightEncoder = m_driveTrain.getRightEncoder();
      m_startingLeftEncoder = m_driveTrain.getLeftEncoder();
      m_targetAngle = (m_gyro.GetYaw() + m_vision.getPowerPortHorizontalAngle()) - atan(160 / m_vision.getPortDistance()) * (180/kPi); //reads target angle and converting to gyro angle
    }),
    frc2::ParallelCommandGroup(
      GyroDrive(&m_gyro, &m_driveTrain, [this] { return m_targetAngle; }, [this] {return -m_driverStick.GetY(); } ),
      SetArmAngle(&m_arm, [this] { 
        double currentDistance = m_startingDistance + DriveTrainConstants::kMetersPerUnit * ((m_driveTrain.getRightEncoder() - m_startingRightEncoder) + (m_driveTrain.getLeftEncoder() - m_startingLeftEncoder)) / 2;
        // double currentDistance = m_startingDistance + m_metersPerEncoder*((m_driveTrain.getLeftEncoder() + m_driveTrain.getRightEncoder()) / 2); 
        double Angle = 15.5504 + (130.439 / (currentDistance + 2.46224));
        frc::SmartDashboard::PutNumber("AutoArmAngle", Angle);
        return Angle;
      }, true),
      RunShooter(&m_shooter, [this] {

        // double currentDistance = m_startingDistance + m_metersPerEncoder*((m_driveTrain.getLeftEncoder() + m_driveTrain.getRightEncoder()) / 2);
        double currentDistance = m_startingDistance + DriveTrainConstants::kMetersPerUnit * ((m_driveTrain.getRightEncoder() - m_startingRightEncoder) + (m_driveTrain.getLeftEncoder() - m_startingLeftEncoder)) / 2;
       
        double Speed = 10648.9 + 1447.44 * currentDistance;

        frc::SmartDashboard::PutNumber("TargetSpeed", Speed);

        return Speed;

      })

    )

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
  // An example command will be run in autonomous
  return nullptr;
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
    });
    // frc2::JoystickButton(&m_buttonBOX, 4).WhenPressed(RunShooter(&m_shooter, [] {
    //   double newSpeed = frc::SmartDashboard::GetNumber("ShooterSpeed", 0) - 100.0;
    //   frc::SmartDashboard::PutNumber("ShooterSpeed", newSpeed);
    //   return newSpeed;
    // }));
    
  #endif //ButtonBox
}