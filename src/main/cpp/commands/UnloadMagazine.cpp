// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/UnloadMagazine.h"
#include <units/time.h>

#include <iostream>

UnloadMagazine::UnloadMagazine(BallHolder* ballHolder, Feeder* feeder, FloorIntake* intake, bool intakePosition) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ballHolder, feeder, intake});
  m_ballHolder = ballHolder;
  m_feeder = feeder;
  m_intake = intake;
  m_intakePos = intakePosition;
}

// Called when the command is initially scheduled.
void UnloadMagazine::Initialize() {

  m_ballHolder->SetMagazine(0.35);
  m_feeder->SetFeeder(0.55);
  m_ballHolder->SetIndexer(0.3);
  if(m_intakePos) { m_intake->Set(FloorIntakeConstants::Position::Out, 0.4); }
  m_timer.Reset();
  m_timer.Start();

}

// Called repeatedly when this Command is scheduled to run
void UnloadMagazine::Execute() {

  if (m_timer.Get() > AutonmousConstants::kUnloadMagazineTimeout) {

    m_done = true;

  }

  if (m_ballHolder->GetSensorBeltFull() == true || m_ballHolder->GetSensorBeltIn() || m_ballHolder->GetSensorBeltMiddle()) {
    m_timer.Stop();
    m_timer.Reset();
    m_timer.Start();
  }



}


// Called once the command ends or is interrupted.
void UnloadMagazine::End(bool interrupted) {

  m_ballHolder->SetMagazine(0.0);
  m_feeder->SetFeeder(0.0);
  m_ballHolder->SetIndexer(0.0);
  if (m_intakePos) {
    m_intake->Set(FloorIntakeConstants::Position::Out, 0.0);
  } else {
    m_intake->Set(FloorIntakeConstants::Position::In, 0.0);
  }
  m_timer.Stop();
  m_timer.Reset();
  m_done = false;

}

// Returns true when the command should end.
bool UnloadMagazine::IsFinished() {
  return m_done;
}
