/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/GoToAngle.h"

GoToAngle::GoToAngle(DriveTrain* drivetrain, Gyro* gyro, frc::Joystick* joystick, double target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain, gyro});

  m_gyro = gyro;
  m_driveTrain = drivetrain;
  m_target = units::degree_t( target );
  m_joystick = joystick;

}

GoToAngle::GoToAngle(DriveTrain* drivetrain, Gyro* gyro, frc::Joystick* joystick, units::degree_t target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain, gyro});

  m_gyro = gyro;
  m_driveTrain = drivetrain;
  m_target = target;
  m_joystick = joystick;

}

// Called when the command is initially scheduled.
void GoToAngle::Initialize() {

  frc::SmartDashboard::PutNumber("Drive Kp", 0.0);
  frc::SmartDashboard::PutNumber("Target angle", 0.0);
  frc::SmartDashboard::PutNumber("Drive Turn Output", 0.0);
  frc::SmartDashboard::PutNumber("Drive Angle Error", 0.0);

}

// Called repeatedly when this Command is scheduled to run
void GoToAngle::Execute() {
  
  m_kp = frc::SmartDashboard::GetNumber("Drive Kp", 0.0);

  m_target = units::degree_t( frc::SmartDashboard::GetNumber("Target angle", 0.0) );



  m_difference = ( m_target - m_gyro->GetYaw() ).to<double>();

  m_output = m_difference * m_kp;

  frc::SmartDashboard::PutNumber("Drive Turn Output", m_output);
  frc::SmartDashboard::PutNumber("Drive Angle Error", m_difference);

  m_driveTrain->CurvatureDrive(-m_joystick->GetY(), m_output, false);

}

// Called once the command ends or is interrupted.
void GoToAngle::End(bool interrupted) {

}

// Returns true when the command should end.
bool GoToAngle::IsFinished() { return false; }
