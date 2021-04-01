// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GyroDrive.h"

GyroDrive::GyroDrive(Gyro* Gyro, DriveTrain* driveTrain, std::function<double()> headingAngle, std::function<double()> forwardPower) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({Gyro, driveTrain});

  m_gyro = Gyro;
  m_driveTrain = driveTrain;
  m_headingAngle = headingAngle;
  m_forwardPower = forwardPower;

}
GyroDrive::GyroDrive(Gyro* Gyro, DriveTrain* driveTrain, double headingAngle, double forwardPower) : GyroDrive(Gyro, driveTrain, [headingAngle] {return headingAngle; }, [forwardPower] { return forwardPower; }) {}
GyroDrive::GyroDrive(Gyro* Gyro, DriveTrain* driveTrain, double headingAngle, std::function<double()> forwardPower) : GyroDrive(Gyro, driveTrain, [headingAngle] { return headingAngle; }, forwardPower) {}



// Called when the command is initially scheduled.
void GyroDrive::Initialize() {

  m_error = m_headingAngle() - m_gyro->GetYaw();
  m_startError = m_error;

}

// Called repeatedly when this Command is scheduled to run
void GyroDrive::Execute() {
  

  double outputPower = (m_error*m_kp);

  m_driveTrain->CurvatureDrive(m_forwardPower(), outputPower, false);
  
  m_error = m_headingAngle() - m_gyro->GetYaw();

  
}

// Called once the command ends or is interrupted.
void GyroDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool GyroDrive::IsFinished() {
  return false;
}
