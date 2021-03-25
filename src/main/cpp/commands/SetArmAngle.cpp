/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetArmAngle.h"

SetArmAngle::SetArmAngle(Arm* arm, double angle) : SetArmAngle(arm, [angle] { return angle; } ) {}

SetArmAngle::SetArmAngle(Arm* arm, std::function<double()> angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
  m_angle = [angle] { return units::degree_t(angle()); };
  m_arm = arm;
}

SetArmAngle::SetArmAngle(Arm* arm, std::function<units::degree_t()> angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});
  m_angle = angle;
  m_arm = arm;
}

// Called when the command is initially scheduled.
void SetArmAngle::Initialize() {

  m_arm->SetAngle(m_angle().to<double>());

}

// Called repeatedly when this Command is scheduled to run
void SetArmAngle::Execute() {}

// Called once the command ends or is interrupted.
void SetArmAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool SetArmAngle::IsFinished() { return true; }
