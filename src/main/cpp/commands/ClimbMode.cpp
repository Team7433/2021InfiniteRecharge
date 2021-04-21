// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbMode.h"

ClimbMode::ClimbMode(Arm* arm, Climber* climber, DriveTrain* driveTrain, frc::Joystick* joystick) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements( {arm, climber} );

  m_arm = arm;
  m_climber = climber;
  m_driveTrain = driveTrain;
  m_joystick = joystick;

}

// Called when the command is initially scheduled.
void ClimbMode::Initialize() {
  m_arm->SetAngle(75_deg);  //sets arm to climb angle
}

// Called repeatedly when this Command is scheduled to run
void ClimbMode::Execute() {

  if (m_climbMode) { 
    if(m_arm->GetArrived() && !m_armLocked) {

      m_arm->SetLock(ArmConstants::Lock_Position::Lock);
      m_armLocked = true;

    }
    if ((m_joystick->GetRawAxis(3)+1) < 0.2) {
      m_sliderSaftey = false;
    }

    if (m_armLocked && !m_sliderSaftey) {
      m_climber->SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock);
      m_driveTrain->SetCoastMode();
      m_climber->RunDynamicRevoltions([this] {return (4.53/2.0)*(m_joystick->GetRawAxis(3)+1.0) -1.75; });
    }
  } else { // if climbDode

    m_climber->SetLockPosition(ClimberConstants::ClimberLock_Position::Lock); // locks climb

  }

  //toggle climbMode
  if (m_joystick->GetRawButtonPressed(1)) {

    m_climbMode = !m_climbMode;

  }
  //toggles to lock mode when match time reaches less than 2
  // if (m_timer.GetMatchTime() < 2_s && !m_matchTriggered) {
    
  //   m_climbMode = false;
  //   m_matchTriggered = true;

  // }



}

// Called once the command ends or is interrupted.
void ClimbMode::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbMode::IsFinished() {
  return false;
}
