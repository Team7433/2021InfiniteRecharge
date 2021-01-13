/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ManualArmControl.h"

ManualArmControl::ManualArmControl(Arm* arm, frc::XboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm});

  m_arm = arm;
  m_controller = controller;
}

// Called when the command is initially scheduled.
void ManualArmControl::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ManualArmControl::Execute() {
  m_arm->ManualControl(m_controller->GetY(frc::XboxController::JoystickHand::kLeftHand));
}

// Called once the command ends or is interrupted.
void ManualArmControl::End(bool interrupted) {}

// Returns true when the command should end.
bool ManualArmControl::IsFinished() { return false; }
