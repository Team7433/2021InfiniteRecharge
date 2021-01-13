/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveWithJoystick.h"


DriveWithJoystick::DriveWithJoystick(frc::Joystick* joystick, DriveTrain* drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain});

  m_joystick = joystick;
  m_driveTrain = drivetrain;

}

// Called when the command is initially scheduled.
void DriveWithJoystick::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute() {
  if (m_joystick->GetRawButton(1)) {
    m_driveTrain->CurvatureDrive(-m_joystick->GetY() , m_joystick->GetZ() * 0.6, m_joystick->GetRawButton(1));
  } else {
    m_driveTrain->CurvatureDrive(-m_joystick->GetY() , m_joystick->GetZ(), m_joystick->GetRawButton(1));
  }

  //m_driveTrain->ArcadeDrive(-m_joystick->GetY() * 0.75 , m_joystick->GetZ() * 0.6, true);

  
}

// Called once the command ends or is interrupted.
void DriveWithJoystick::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveWithJoystick::IsFinished() { return false; }
