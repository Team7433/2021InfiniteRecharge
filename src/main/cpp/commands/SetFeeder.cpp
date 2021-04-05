/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetFeeder.h"

SetFeeder::SetFeeder(Feeder * feeder, FeederConstants::BeltPosition positon, double output) {
  AddRequirements(feeder);
  m_feeder= feeder;
  m_beltPositon = positon;
  m_output = output;
}

// Called when the command is initially scheduled.
void SetFeeder::Initialize() {
  m_feeder->SetFeeder(m_output);
  m_feeder->SetPosition(m_beltPositon);
}

// Called repeatedly when this Command is scheduled to run
void SetFeeder::Execute() {}

// Called once the command ends or is interrupted.
void SetFeeder::End(bool interrupted) {}

// Returns true when the command should end.
bool SetFeeder::IsFinished() { return true; }
