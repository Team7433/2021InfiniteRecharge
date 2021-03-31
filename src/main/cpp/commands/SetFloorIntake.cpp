/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetFloorIntake.h"
#include "frc/Timer.h"
#include <iostream>

SetFloorIntake::SetFloorIntake(FloorIntake * intake, FloorIntakeConstants::Position position, double rollerSpeed) {
  AddRequirements({intake});
  
  //make subsystem and targets avalible to initialize
  m_intake = intake;
  m_position = position;
  m_roller = rollerSpeed;
}

// Called when the command is initially scheduled.
void SetFloorIntake::Initialize() {
  m_intake->Set(m_position, m_roller);

}

// Called repeatedly when this Command is scheduled to run
void SetFloorIntake::Execute() {


}

// Called once the command ends or is interrupted.
void SetFloorIntake::End(bool interrupted) {
  m_intake->Set(m_position, m_roller);
}

// Returns true when the command should end.
bool SetFloorIntake::IsFinished() { return false; }
