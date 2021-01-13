/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ChangeCamMode.h"

ChangeCamMode::ChangeCamMode(Vision* vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_vision = vision;
}

// Called when the command is initially scheduled.
void ChangeCamMode::Initialize() {

  if (m_vision->getCamMode() == 1) {
    m_vision->changeCamMode(0);
  } else {
    m_vision->changeCamMode(1);
  }


}

// Called repeatedly when this Command is scheduled to run
void ChangeCamMode::Execute() {}

// Called once the command ends or is interrupted.
void ChangeCamMode::End(bool interrupted) {}

// Returns true when the command should end.
bool ChangeCamMode::IsFinished() { return true; }
