/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/StoreBallHolder.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

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
    frc::SmartDashboard::PutNumber("magazine/Timer", m_timer.Get().to<double>());
    frc::SmartDashboard::PutBoolean("magazine/Last", m_lastSensorBeltIn);

    if (m_timer.Get() > BallHolderConstants::kTimerMagazineDelay) {
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

  m_lastSensorBeltIn = m_ballholder->GetSensorBeltIn();


  double current = m_intake->GetCurrent();

  

  if (m_intake->GetPercentageOutput() > 0) {
    
    m_currentAverage = m_currentAverage - (m_currentAverage/5) + (abs(current)/5);
    frc::SmartDashboard::PutNumber("overcurrent/CurrentAverageFloorIntake", m_currentAverage);
    frc::SmartDashboard::PutNumber("overcurrent/currentFloorIntake", current);
    frc::SmartDashboard::PutBoolean("overcurrent/StalldetectedFloorIntake", m_overloaded);

    if (m_currentAverage > 17.0) {

      if (m_overloaded == false) {
        m_overloaded = true;
        m_timer2.Reset();
        m_timer2.Start();
      }

    }


  }

  if (m_overloaded == true) {
    m_intake->Set(Position::Out, -0.65);
    if (m_timer2.Get() > BallHolderConstants::kOverloadReverseLength) {
          
          m_intake->Set(Position::Out, m_intakeSpeed);
          m_overloaded = false;
          m_currentAverage = 0.0;
          m_timer2.Stop();
          }
  }

    
}

  

// Called once the command ends or is interrupted.
void StoreBallHolder::End(bool interrupted) {}

// Returns true when the command should end.
bool StoreBallHolder::IsFinished() { return false; }
