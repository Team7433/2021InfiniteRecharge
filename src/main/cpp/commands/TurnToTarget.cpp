/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnToTarget.h"
#include <cmath>
#include <units/math.h>

TurnToTarget::TurnToTarget(Vision* vision, Gyro* gyro, DriveTrain* drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({vision, drivetrain, gyro});
  
  m_vision = vision;
  m_driveTrain = drivetrain;
  m_gyro = gyro;

  frc::SmartDashboard::PutNumber("TTT/Drive limelight Kp", 0.0);
}

// Called when the command is initially scheduled.
void TurnToTarget::Initialize() {
  //Checks if there is a target if not ends command

  if (m_vision->getPowerPortDetected() == false) {
    m_done = true;
  }
  //using vision to set a gyro target

  m_gyroTarget = m_gyro->GetYaw() + m_vision->getPowerPortHorizontalAngle() - units::math::atan(160_mm / m_vision->getPortDistance());
  frc::SmartDashboard::PutNumber("TTT/Gyro target limelight", m_gyroTarget.to<double>());


}

// Called repeatedly when this Command is scheduled to run
void TurnToTarget::Execute() { 
  

  //Sets the error
  m_error = ( m_gyroTarget - m_gyro->GetYaw() ).to<double>();
  frc::SmartDashboard::PutNumber("TTT/gyro error", m_error);
 

  //Sets the output to arcade drive.(error x kp)
  frc::SmartDashboard::PutNumber("TTT/output power: ", (m_error * m_kp + 0.1));
  
  if (m_error > 0) {
    m_ks = 0.091;
  } else {
    m_ks = -0.091;
  }

  m_driveTrain->ArcadeDrive(0, (m_error * m_kp + m_ks), false);



}

// Called once the command ends or is interrupted.
void TurnToTarget::End(bool interrupted) {
  m_driveTrain->ArcadeDrive(0, 0, false);
  m_done = false;

  
                                                   


}

// Returns true when the command should end.
bool TurnToTarget::IsFinished() { 
  if(m_vision->getPowerPortDetected() == false) {
    return true;
  }
  return false;
  }
