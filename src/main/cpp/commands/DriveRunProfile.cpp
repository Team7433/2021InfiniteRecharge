// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveRunProfile.h"

DriveRunProfile::DriveRunProfile(DriveTrain * drive, std::string profile) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  m_drive = drive;
  m_profile = profile;
}

// Called when the command is initially scheduled.
void DriveRunProfile::Initialize() {
  m_drive->SetSlot(0);
  std::cout << "Start Profile " << m_profile << "\n";
  m_drive->MPLoad(m_profile);
  m_drive->MPStart();
  m_profileTimer.Start();

}

// Called repeatedly when this Command is scheduled to run
void DriveRunProfile::Execute() {

  m_drive->MPUpdate();

}

// Called once the command ends or is interrupted.
void DriveRunProfile::End(bool interrupted) {

  double time = m_profileTimer.Get().to<double>();
  std::cout << "Stop Profile " << m_profile << ". Finished in a time of " << time << "\n";
  m_drive->MPStop();
  m_profileTimer.Stop();

}

// Returns true when the command should end.
bool DriveRunProfile::IsFinished() {
  return m_drive->MPIsFinished(); 
}
