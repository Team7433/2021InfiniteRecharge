// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbMode.h"

ClimbMode::ClimbMode(Arm* arm, Climber* climber, DriveTrain* driveTrain, frc::Joystick* Driverjoystick, frc::XboxController* Operatorjoystick) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements( {arm, climber, driveTrain} );

  m_arm = arm;
  m_climber = climber;
  m_driveTrain = driveTrain;
  m_driverJoystick = Driverjoystick;
  m_operatorJoystick = Operatorjoystick;

}

// Called when the command is initially scheduled.
void ClimbMode::Initialize() {
  m_arm->SetAngle(75_deg);  //sets arm to climb angle
  m_driveTrain->SetCoastMode();
}

// Called repeatedly when this Command is scheduled to run
void ClimbMode::Execute() {

    if (m_driverJoystick->GetRawButton(1)) {
      m_driveTrain->CurvatureDrive(-m_driverJoystick->GetY()*0.5, m_driverJoystick->GetZ() * 0.3, m_driverJoystick->GetRawButton(1));
    } else {
      m_driveTrain->CurvatureDrive(-m_driverJoystick->GetY()*0.5, m_driverJoystick->GetZ(), m_driverJoystick->GetRawButton(1));
    }

  if (m_climbMode) { 
    //lock arm position
    if(m_arm->GetArrived() && !m_armLocked) {
      m_arm->SetLock(ArmConstants::Lock_Position::Lock);
      m_armLocked = true;
    }

    // slider saftey
    if ((m_driverJoystick->GetRawAxis(3)+1) < 0.2) {
      m_sliderSaftey = false;
    }

    // if slider saftey and arm is locked is clear then enable climb
    if (m_armLocked && !m_sliderSaftey) {
      m_climber->SetLockPosition(ClimberConstants::ClimberLock_Position::Unlock);
      m_driveTrain->SetCoastMode();
      m_climber->RunDynamicRevoltions([this] {return (((m_driverJoystick->GetRawAxis(3)+1.0)/2)*4.5) -1.90; });
    }
  } else { 
    // locks climber
    m_climber->SetLockPosition(ClimberConstants::ClimberLock_Position::Lock); // locks climb

  }

  //toggle climbMode
  if (m_operatorJoystick->GetRawButtonPressed(6)) {
    m_climbMode = !m_climbMode;
  }
  //toggles to lock mode when match time reaches less than 2
  if (m_timer.GetMatchTime() < 2_s && !m_matchTriggered) {
    
    m_climbMode = false;
    m_matchTriggered = true;

  }



}

// Called once the command ends or is interrupted.
void ClimbMode::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbMode::IsFinished() {
  return false;
}
