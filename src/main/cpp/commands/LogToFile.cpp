// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LogToFile.h"
#include <fstream>

LogToFile::LogToFile(DriveTrain * drivetrain, Gyro * gyro) {
  m_driveTrain = drivetrain;
  m_gyro = gyro;
}

// Called when the command is initially scheduled.
void LogToFile::Initialize() {
  m_logText = "";
}

// Called repeatedly when this Command is scheduled to run
void LogToFile::Execute() {
  m_logText = m_logText + std::to_string(m_driveTrain->getLeftDistance().to<double>()) + ","
                        + std::to_string(m_driveTrain->getRightDistance().to<double>()) + ","
                        + std::to_string(m_gyro->GetYaw().to<double>()) + ","
                        + std::to_string(m_driveTrain->getUltrasonicDistanceA().to<double>()) + ","
                        + std::to_string(m_driveTrain->getUltrasonicDistanceB().to<double>()) + ","
                        + std::to_string(m_driveTrain->getUltrasonicDistanceC().to<double>()) + ","
                        + std::to_string(m_driveTrain->getUltrasonicDistanceD().to<double>()) + "\n";
}

// Called once the command ends or is interrupted.
void LogToFile::End(bool interrupted) {

  std::string time = units::time::to_string(frc2::Timer::GetFPGATimestamp());
  std::ofstream myfile;
  myfile.open ("home/lvuser/logs/" + time + ".csv");
  std::cout << m_logText;
  myfile << m_logText;
  myfile.close();
}

// Returns true when the command should end.
bool LogToFile::IsFinished() {
  return false;
}
