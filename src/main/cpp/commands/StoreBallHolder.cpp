/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/StoreBallHolder.h"
#include "Constants.h"

StoreBallHolder::StoreBallHolder(BallHolder * ballholder, FloorIntake * floorIntake, double indexSpeed, double magazineSpeed, double intakeSpeed) {
  AddRequirements({ballholder, floorIntake});
  m_ballholder = ballholder;
  m_intake = floorIntake;
  m_intakeSpeed = intakeSpeed;
  m_indexRunSpeed = indexSpeed;
  m_magazineRunSpeed = magazineSpeed;
}

// Called when the command is initially scheduled.
void StoreBallHolder::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void StoreBallHolder::Execute() {

  

  if (m_ballholder->GetSensorBeltFull()) {

    m_ballholder->SetMagazine(0);

    if (m_ballholder->GetSensorIndexerOut()) {

      m_ballholder->SetIndexer(0);

      if (m_ballholder->GetSensorIndexerIn()) {
        m_intake->Set(Position::Out, 0);
      } else {
        m_intake->Set(Position::Out, m_intakeSpeed);
      }

    } else {

      m_ballholder->SetIndexer(m_indexRunSpeed);
      m_intake->Set(Position::Out, m_intakeSpeed);

    }

  } else {
    m_intake->Set(Position::Out, m_intakeSpeed);
    m_ballholder->SetIndexer(m_indexRunSpeed);

    if (m_timer.Get() < BallHolderConstants::kTimerMagazineDelay) {
      m_ballholder->SetMagazine(m_magazineRunSpeed);
      m_timer.Reset();
      m_timer.Stop();
    }

    if (m_ballholder->GetSensorBeltIn() && m_lastSensorBeltIn == false) {
      m_timer.Reset();
      m_timer.Start();
    } 

    if (!m_ballholder->GetSensorBeltIn()) {
      m_ballholder->SetMagazine(0);
      m_timer.Reset();
      m_timer.Stop();
    }
  }

    
}

  

// Called once the command ends or is interrupted.
void StoreBallHolder::End(bool interrupted) {}

// Returns true when the command should end.
bool StoreBallHolder::IsFinished() { return false; }
