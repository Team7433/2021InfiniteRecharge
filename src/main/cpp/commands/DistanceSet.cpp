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
  m_arm = arm;
  m_shooter = shooter;
}

DistanceSet::DistanceSet(units::meter_t distanceM, Arm* arm, Shooter* shooter) {

  m_distanceM = distanceM;
  m_arm = arm;
  m_shooter = shooter;
  m_distanceOveride = true;

}


// Called when the command is initially scheduled.
void DistanceSet::Initialize() {

//grabs the distance from port using vision subsystem otherwise overrided use overrided distance
  if(m_vision->getPowerPortDetected()) {
    units::meter_t distance = m_distanceM;
    if (m_distanceOveride == false) {
      distance = m_vision->getPortDistance();
    }
  
    //sets the speed using line of best fit equation
    units::degree_t Angle = 15.5504_deg + (units::degree_t(130.439 / (2.46224 + atof(units::length::to_string(distance).c_str()))));
    double Speed = 10648.9 + 1447.44 * atof(units::length::to_string(distance).c_str());
    //Displays target speed and angle on smartDashboard
     frc::SmartDashboard::PutNumber("TargetSpeed", Speed);
     frc::SmartDashboard::PutString("TargetAngle", units::angle::to_string(Angle));

    m_arm->SetAngle(Angle);
    m_shooter->SetVelocity(Speed);

  }
}

// Called repeatedly when this Command is scheduled to run
void DistanceSet::Execute() {}

// Called once the command ends or is interrupted.
void DistanceSet::End(bool interrupted) {}

// Returns true when the command should end.
bool DistanceSet::IsFinished() { return false; }
