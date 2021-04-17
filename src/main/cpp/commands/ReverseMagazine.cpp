// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ReverseMagazine.h"

ReverseMagazine::ReverseMagazine(BallHolder* ballHolder, FloorIntake* floorIntake, Feeder* feeder, std::function<bool()> LeftTrig, std::function<bool()> RightTrig) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({floorIntake, ballHolder, feeder});

  m_ballHolder = ballHolder;
  m_floorIntake = floorIntake;
  m_feeder = feeder;
  m_rightTrig = RightTrig;
  m_leftTrig = LeftTrig; 

}

// Called when the command is initially scheduled.
void ReverseMagazine::Initialize() {

  m_ballHolder->SetIndexer(0.0);
  m_ballHolder->SetMagazine(0.0);
  m_feeder->SetFeeder(0.0);
  m_floorIntake->Set(Position::Out, 0.0);

}

// Called repeatedly when this Command is scheduled to run
void ReverseMagazine::Execute() {

  if (m_leftTrig()) {
    m_ballHolder->SetMagazine(-0.3);
    m_ballHolder->SetIndexer(-0.3);
    m_feeder->SetFeeder(-0.3);
  } else {

    m_ballHolder->SetMagazine(0.0);
    m_ballHolder->SetIndexer(0.0);
    m_feeder->SetFeeder(0.0);

  }

  if (m_rightTrig()) {

    m_floorIntake->Set(Position::Out, -0.5);

  } else {

    m_floorIntake->Set(Position::Out, 0.0);

  }

}

// Called once the command ends or is interrupted.
void ReverseMagazine::End(bool interrupted) { }

// Returns true when the command should end.
bool ReverseMagazine::IsFinished() {
  return false;
}
