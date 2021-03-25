/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RunShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>



RunShooter::RunShooter(Shooter* shooter, double  velocity) : RunShooter(shooter, [velocity] { return velocity; }) {

}

RunShooter::RunShooter(Shooter* shooter, std::function<double()>  velocity) {

  AddRequirements({shooter});

  m_shooter = shooter;
  m_velocity = velocity;

}

// Called when the command is initially scheduled.
void RunShooter::Initialize() {
  m_actualVelocity = m_velocity();
}

// Called repeatedly when this Command is scheduled to run
void RunShooter::Execute() {
  if (m_actualVelocity != 0) {
    m_shooter->configKF((474.592 + (384574 / (m_actualVelocity - 1917.6))) / 10000);
  }
  double difference = m_actualVelocity - m_shooter->GetVelocityLoopTarget();

  double setVelocity = 0;

  if (difference > kshooterRampSpeed) {
    setVelocity = m_shooter->GetVelocityLoopTarget() + kshooterRampSpeed;
  } else if (difference < -kshooterRampSpeed) {
    setVelocity = m_shooter->GetVelocityLoopTarget() - kshooterRampSpeed;
  } else {
    setVelocity = m_actualVelocity;
  }

  m_shooter->SetVelocity(setVelocity);
  frc::SmartDashboard::PutNumber("CurrentVelocity", m_shooter->GetVelocity());
}

// Called once the command ends or is interrupted.
void RunShooter::End(bool interrupted) {
  m_done = false;

}

// Returns true when the command should end.
bool RunShooter::IsFinished() { 
  frc::SmartDashboard::PutBoolean("Shooter/OnTarget", m_shooter->GetVelocityLoopTarget() == m_actualVelocity);
  return (m_shooter->GetVelocityLoopTarget() == m_actualVelocity); 
}
