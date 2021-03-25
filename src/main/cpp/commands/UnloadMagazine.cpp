// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/UnloadMagazine.h"
#include <units/time.h>

#include <iostream>

UnloadMagazine::UnloadMagazine(BallHolder* ballHolder, Feeder* feeder) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ballHolder, feeder});
  m_ballHolder = ballHolder;
  m_feeder = feeder;
}

// Called when the command is initially scheduled.
void UnloadMagazine::Initialize() {

  m_ballHolder->SetMagazine(0.3);
  m_feeder->SetFeeder(0.5);
  m_ballHolder->SetIndexer(0.3);
  m_timer.Reset();
  m_timer.Start();

}

// Called repeatedly when this Command is scheduled to run
void UnloadMagazine::Execute() {

  std::cout << m_timer.Get() << std::endl;


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
  m_timer.Stop();
  m_timer.Reset();
  m_done = false;

}

// Returns true when the command should end.
bool UnloadMagazine::IsFinished() {
  return m_done;
}
