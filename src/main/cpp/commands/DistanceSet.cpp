/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DistanceSet.h"
#include <math.h>

DistanceSet::DistanceSet(Vision* vision, Arm* arm, Shooter* shooter) {
  // Use addRequirements() here to declare subsystem dependencies.

  m_vision = vision;
}

// Called when the command is initially scheduled.
void DistanceSet::Initialize() {

//grabs the distance from port using vision subsystem
  if(m_vision->getPowerPortDetected()) {
    // double distance = m_vision->getPortDistance()/ 1000;

    // //sets the speed using line of best fit equation
    // double Speed = -0.67217*(std::pow(distance, 6)) + 17.5164*(std::pow(distance, 5)) + 168.137*(std::pow(distance, 4)) + 707.557*(std::pow(distance, 3)) + 1182.52*(std::pow(distance, 2)) + 1721.41*(std::pow(distance, 1)) + 9963.13;
    // double Angle = 0.00262689*(std::pow(distance, 6)) + -0.0721799*(std::pow(distance, 5)) + 0.767968*(std::pow(distance, 4)) + -4.01135*(std::pow(distance, 3)) + 11.3266*(std::pow(distance, 2)) + -21.1942*(std::pow(distance, 1)) + 56.4509;

    // frc::SmartDashboard::PutNumber("TargetSpeed", Speed);
    // frc::SmartDashboard::PutNumber("TargetAngle", Angle);

    // m_arm->SetAngle(Angle);
    // m_shooter->SetVelocity(Speed);

  }
}

// Called repeatedly when this Command is scheduled to run
void DistanceSet::Execute() {}

// Called once the command ends or is interrupted.
void DistanceSet::End(bool interrupted) {}

// Returns true when the command should end.
bool DistanceSet::IsFinished() { return false; }
