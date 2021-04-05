/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetArmAngle.h"

SetArmAngle::SetArmAngle(Arm* arm, units::degree_t angle) : SetArmAngle(arm, [angle] { return angle; } ) {}


SetArmAngle::SetArmAngle(Arm* arm, std::function<units::degree_t()> angle, bool update) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
  m_angle = angle;
  m_arm = arm;
  m_update = update;
}

// Called when the command is initially scheduled.
void SetArmAngle::Initialize() {
  if (m_update == false) {
    m_setAngle = m_angle();
    m_arm->SetAngle(m_setAngle);
  }

}

// Called repeatedly when this Command is scheduled to run
void SetArmAngle::Execute() {

  if (m_update) {
    m_setAngle = m_angle();
    m_arm->SetAngle(m_setAngle);

  }

}

// Called once the command ends or is interrupted.
void SetArmAngle::End(bool interrupted) {
}

// Returns true when the command should end.
bool SetArmAngle::IsFinished() { 

  frc::SmartDashboard::PutNumber("Arm/SetFinishDifference", (m_arm->GetArmAngleMotorUnits() - m_setAngle).to<double>());
  return m_arm->GetArmAngleMotorUnits() > m_setAngle - 1.5_deg && m_arm->GetArmAngleMotorUnits() < m_setAngle + 1.5_deg && m_update == false;

}
